-- @widget  Wind
-- @version 1.0
-- Wind Estimator: Windgeschwindigkeit, Windrichtung und True Airspeed (TAS)
-- ausschliesslich aus GPS Speed + GPS Course.
--
-- Methode: gleitender algebraischer Kreisfit (Kasa-Methode) ueber ein
-- rollierendes Fenster der letzten N GPS-Vektoren. Die Methode setzt
-- voraus, dass die Fluggeschwindigkeit (TAS) innerhalb des Fensters
-- ungefaehr konstant ist (Gas/Anstellwinkel nicht stark veraendern) und
-- dass sich der GPS-Kurs innerhalb des Fensters ausreichend aendert
-- (normale Kurven/Wenden reichen, kein dediziertes Kreisfliegen noetig).
--
-- Hat das Modell keine eigene GPS-Course-Quelle (manche GPS-Sensoren liefern
-- nur Position + Zeit, keinen fertigen Kurs), wird der Kurs stattdessen aus
-- zwei aufeinanderfolgenden GPS-Fixes berechnet (Vorwaerts-Azimut). Ein
-- "neuer Fix" wird ueber eine Aenderung der GPS-Zeit-Quelle erkannt, damit
-- nicht mehrfach derselbe Fix zwischen zwei GPS-Updates vermessen wird.
--
-- Registriert drei Sources (WindSpd, WindDir, TAS), die sich Sampling und
-- Fit teilen -- pro Modell braucht es nur einen Durchlauf, nicht drei.
--
-- GPS-Speed-/Course-Quelle (bzw. GPS-Positions-/Zeit-Quelle als Fallback)
-- und Fit-Parameter je Modell aus params.txt lesen (liegt neben main.lua;
-- dort auch das Parameter-Schema fuer die Widget-Konfiguration-App, siehe
-- Ethos-Scripts/widget-config).
-- Zeilenformat: Modellname;speedSource=...;courseSource=...;gpsSource=...;
--               gpsTimeSource=...;windowSize=...;sampleIntervalTenths=...;
--               minSpeed=...

local DEG2RAD = math.pi / 180
local RAD2DEG = 180 / math.pi

local translations = {
  WindSpd = {en = "Wind Speed", de = "Windgeschw."},
  WindDir = {en = "Wind Direction", de = "Windrichtung"},
  TAS     = {en = "TAS", de = "TAS"},
}

local function makeName(key)
  return function()
    local locale = system.getLocale()
    local t = translations[key]
    return t[locale] or t.en
  end
end

-- ---------- Default-Werte, falls params.txt fehlt oder fuer das Modell
-- keine Zeile hat (ueber die Widget-Konfiguration-App aenderbar) ----------
local DEFAULT_SPEED_SOURCE  = "GPS Speed"
local DEFAULT_COURSE_SOURCE = "GPS Course"
local DEFAULT_GPS_SOURCE      = "GPS"        -- Fallback: Positions-Quelle (liefert Lat/Lon)
local DEFAULT_GPS_TIME_SOURCE = "GPS clock"  -- Fallback: erkennt neue Fixes an ihrer Aenderung
local DEFAULT_WINDOW_SIZE   = 40   -- max. Anzahl Samples im Fenster
local DEFAULT_SAMPLE_TENTHS = 7    -- Sample-Intervall in 0.1s-Schritten (0.7s)
local DEFAULT_MIN_SPEED     = 2    -- Einheiten wie GPS-Speed-Quelle; darunter kein Sample

-- Intern fest (nicht ueber die Config-App gesetzt, wie im Original):
local MIN_SAMPLES   = 10    -- Mindestanzahl Samples fuer einen Fit
local FIT_INTERVAL  = 1.0   -- s, wie oft der Fit neu gerechnet wird
local MAX_SPREAD_R  = 0.85  -- Circular-Variance-Schwelle (kleiner = mehr Kursstreuung noetig)

-- Plausibilitaets-Schranken gegen entartete Kreisfits: bei fast geradem Flug
-- liegen die Bodenvektoren nahezu auf einer Linie. Der Kasa-Fit "gelingt" dann
-- zwar noch, legt aber einen riesigen Kreis durch die Punkte -- Mittelpunkt
-- (= Wind) und Radius (= TAS) werden beide absurd gross und nahezu gleich.
-- Ein einmal akzeptierter Ausreisser bleibt haengen, weil Wind nur bei einem
-- neuen akzeptierten Fit ueberschrieben wird. Darum wird jeder Fit gegen die
-- tatsaechlich beobachteten Bodengeschwindigkeiten geprueft.
local MAX_TAS_FACTOR  = 2.0   -- TAS darf hoechstens 2x die schnellste beobachtete Ground Speed sein
local MAX_WIND_FACTOR = 1.0   -- Wind darf die schnellste beobachtete Ground Speed nicht uebersteigen
local MAX_RESIDUAL_FRAC = 0.25 -- max. RMS-Abweichung der Punkte vom Kreis, relativ zum Radius

local function resolveParams(modelName)
  local params = {
    speedSource = DEFAULT_SPEED_SOURCE,
    courseSource = DEFAULT_COURSE_SOURCE,
    gpsSource = DEFAULT_GPS_SOURCE,
    gpsTimeSource = DEFAULT_GPS_TIME_SOURCE,
    windowSize = DEFAULT_WINDOW_SIZE,
    sampleIntervalTenths = DEFAULT_SAMPLE_TENTHS,
    minSpeed = DEFAULT_MIN_SPEED,
  }
  local file = io.open("params.txt", "r")
  if not file then
    return params
  end

  local line = file:read("*l")
  while line do
    if line:match("%S") and not line:match("^%s*#") then
      local lineModel, rest = line:match("^%s*(.-)%s*;%s*(.-)%s*$")
      if lineModel and rest and lineModel == modelName then
        for pair in (rest .. ";"):gmatch("(.-);") do
          local key, value = pair:match("^%s*(.-)%s*=%s*(.-)%s*$")
          if key and value and value ~= "" then
            params[key] = value
          end
        end
      end
    end
    line = file:read("*l")
  end
  file:close()

  return params
end

-- ---------------- geteilter Zustand (Sampling + Fit) ----------------
local currentModelName = nil
local speedSourceName, courseSourceName = DEFAULT_SPEED_SOURCE, DEFAULT_COURSE_SOURCE
local gpsSourceName, gpsTimeSourceName = DEFAULT_GPS_SOURCE, DEFAULT_GPS_TIME_SOURCE
local windowSize = DEFAULT_WINDOW_SIZE
local sampleInterval = DEFAULT_SAMPLE_TENTHS / 10
local minSpeed = DEFAULT_MIN_SPEED

local lastParamsCheck = 0
local PARAMS_CHECK_INTERVAL = 1  -- s, wie oft params.txt neu geprueft wird

local bufX, bufY, bufN, bufHead = {}, {}, 0, 1
local lastSampleTime, lastFitTime = 0, 0

local windSpeed, windDirFrom, tas = 0, 0, 0
local haveFit = false

-- Zustand fuer die Kurs-aus-Position-Fallback-Berechnung (siehe
-- derivedCourseFromPosition weiter unten).
local lastFixLat, lastFixLon = nil, nil
local lastFixTimeKey = nil
local lastCourse = nil

-- TAS = Bodenvektor (aus GPS Speed/Course) minus Windvektor (aus windSpeed/
-- windDirFrom). Ist noch kein Wind gefittet, sind windSpeed/windDirFrom 0/0
-- (Default), TAS ist dann schlicht die Ground Speed. Laeuft unabhaengig vom
-- Kreisfit bei jedem Aufruf, damit TAS jeder GPS-Speed-/Course-Aenderung
-- sofort folgt -- nur windSpeed/windDirFrom selbst brauchen fuer ein Update
-- weiterhin einen neuen Kreisfit.
local function estimateTAS(groundSpeedValue, groundCourseValue, currentWindSpeed, currentWindDirFrom)
  local groundRad = groundCourseValue * DEG2RAD
  local groundVx = groundSpeedValue * math.sin(groundRad)
  local groundVy = groundSpeedValue * math.cos(groundRad)

  -- currentWindDirFrom ist die Richtung, AUS der der Wind kommt; seine
  -- Geschwindigkeits-Vektorrichtung (wohin die Luftmasse zieht) ist um 180°
  -- gedreht.
  local windToRad = (currentWindDirFrom + 180) * DEG2RAD
  local windVx = currentWindSpeed * math.sin(windToRad)
  local windVy = currentWindSpeed * math.cos(windToRad)

  local tasVx = groundVx - windVx
  local tasVy = groundVy - windVy
  return math.sqrt(tasVx * tasVx + tasVy * tasVy)
end

local function applyParams(modelName)
  local p = resolveParams(modelName)
  speedSourceName = p.speedSource
  courseSourceName = p.courseSource
  gpsSourceName = p.gpsSource
  gpsTimeSourceName = p.gpsTimeSource
  windowSize = math.max(MIN_SAMPLES, tonumber(p.windowSize) or DEFAULT_WINDOW_SIZE)
  sampleInterval = (tonumber(p.sampleIntervalTenths) or DEFAULT_SAMPLE_TENTHS) / 10
  minSpeed = tonumber(p.minSpeed) or DEFAULT_MIN_SPEED

  -- Ringpuffer bei verkleinerter Fenstergroesse zuruecksetzen (einfacher als
  -- Umindizieren; der naechste Fit sammelt einfach neu).
  if bufN > windowSize then
    bufX, bufY, bufN, bufHead = {}, {}, 0, 1
  end

  -- Positions-Fallback-Zustand zuruecksetzen, damit nach einem Modell- oder
  -- Quellenwechsel kein Kurs aus zwei nicht zusammengehoerigen Fixes berechnet wird.
  lastFixLat, lastFixLon = nil, nil
  lastFixTimeKey = nil
end

-- Vorwaerts-Azimut (Kurs) von Punkt 1 nach Punkt 2 in Grad, 0..360.
local function bearingDeg(lat1, lon1, lat2, lon2)
  local phi1, phi2 = lat1 * DEG2RAD, lat2 * DEG2RAD
  local dLambda = (lon2 - lon1) * DEG2RAD
  local y = math.sin(dLambda) * math.cos(phi2)
  local x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dLambda)
  local brng = math.atan(y, x) * RAD2DEG
  if brng < 0 then
    brng = brng + 360
  end
  return brng
end

-- Liest Lat/Lon aus dem Rueckgabewert einer GPS-Positions-Source. Ethos
-- liefert dafuer eine Tabelle; die genaue Feldbenennung ist nicht in jeder
-- Firmware-Version dieselbe, daher werden mehrere gaengige Varianten
-- probiert. Im Zweifel im Sim mit print(gpsSource:value()) gegenpruefen.
local function extractLatLon(value)
  if type(value) ~= "table" then
    return nil, nil
  end
  local lat = value.lat or value.latitude or value.Lat or value.Latitude
  local lon = value.lon or value.lng or value.longitude or value.Lon or value.Longitude
  if type(lat) == "number" and type(lon) == "number" then
    return lat, lon
  end
  return nil, nil
end

-- Fallback, wenn keine eigene GPS-Course-Quelle konfiguriert/vorhanden ist:
-- Kurs aus zwei aufeinanderfolgenden GPS-Fixes ableiten. Ein neuer Fix wird
-- daran erkannt, dass sich der Rueckgabewert der GPS-Zeit-Quelle veraendert
-- hat (tostring-Vergleich, damit sowohl Zahlen/Strings als auch Tabellen
-- funktionieren) -- so wird nicht wiederholt derselbe Fix vermessen, nur weil
-- wakeup() haeufiger laeuft als der GPS-Sensor neue Positionen liefert.
-- Liefert nil, solange noch kein zweiter, unterschiedlicher Fix vorliegt.
local function derivedCourseFromPosition()
  local gpsSrc = system.getSource(gpsSourceName)
  local timeSrc = system.getSource(gpsTimeSourceName)
  if not gpsSrc or not timeSrc then
    return nil
  end

  local timeKey = tostring(timeSrc:value())
  if timeKey == lastFixTimeKey then
    return lastCourse
  end
  lastFixTimeKey = timeKey

  local lat, lon = extractLatLon(gpsSrc:value())
  if not lat or not lon then
    return lastCourse
  end

  if lastFixLat and lastFixLon and (lat ~= lastFixLat or lon ~= lastFixLon) then
    lastCourse = bearingDeg(lastFixLat, lastFixLon, lat, lon)
  end

  lastFixLat, lastFixLon = lat, lon
  return lastCourse
end

-- ============================================================
-- Kasa-Kreisfit (algebraische Loesung, rein linear, kein Iterieren)
-- Input: Arrays x[], y[] gleicher Laenge n
-- Output: cx, cy (Kreismittelpunkt = Windvektor), r (Radius = TAS), ok
-- ============================================================
local function kasaFit(x, y, n)
  local sx, sy = 0, 0
  for i = 1, n do
    sx = sx + x[i]
    sy = sy + y[i]
  end
  local mx, my = sx / n, sy / n

  local Suu, Svv, Suv = 0, 0, 0
  local Suuu, Svvv, Suvv, Svuu = 0, 0, 0, 0

  for i = 1, n do
    local u = x[i] - mx
    local v = y[i] - my
    local uu = u * u
    local vv = v * v
    Suu = Suu + uu
    Svv = Svv + vv
    Suv = Suv + u * v
    Suuu = Suuu + uu * u
    Svvv = Svvv + vv * v
    Suvv = Suvv + u * vv
    Svuu = Svuu + v * uu
  end

  -- Lineares 2x2 Gleichungssystem:
  -- [Suu Suv] [uc]   [ (Suuu+Suvv)/2 ]
  -- [Suv Svv] [vc] = [ (Svvv+Svuu)/2 ]
  -- Konditionierung relativ zur Groessenordnung der Streuung pruefen: eine
  -- absolute Schwelle (frueher 1e-9) sagt nichts aus, weil detA mit der vierten
  -- Potenz der Geschwindigkeiten skaliert. Bei fast kollinearen Punkten ist detA
  -- winzig gegenueber (Suu+Svv)^2 -- genau der Fall, der sonst einen riesigen,
  -- unsinnigen Kreis liefert.
  local detA = Suu * Svv - Suv * Suv
  local scale = Suu + Svv
  if scale <= 0 or math.abs(detA) < 1e-6 * scale * scale then
    return nil, nil, nil, false -- Punkte kollinear / kein Fit moeglich
  end

  local rhs1 = (Suuu + Suvv) / 2
  local rhs2 = (Svvv + Svuu) / 2

  local uc = (rhs1 * Svv - rhs2 * Suv) / detA
  local vc = (Suu * rhs2 - Suv * rhs1) / detA

  local cx = uc + mx
  local cy = vc + my
  local r2 = uc * uc + vc * vc + (Suu + Svv) / n
  if r2 < 0 then
    return nil, nil, nil, false
  end

  return cx, cy, math.sqrt(r2), true
end

-- Circular Variance der Kurse: 0 = perfekt gleichverteilt (guter Fit
-- moeglich), 1 = alle Kurse identisch (kein Fit moeglich).
local function courseSpreadOk(x, y, n, maxSpreadR)
  local sc, ss = 0, 0
  for i = 1, n do
    local mag = math.sqrt(x[i] * x[i] + y[i] * y[i])
    if mag > 1e-6 then
      sc = sc + x[i] / mag
      ss = ss + y[i] / mag
    end
  end
  local R = math.sqrt(sc * sc + ss * ss) / n
  return R < maxSpreadR
end

-- Prueft einen Fit gegen die tatsaechlich beobachteten Bodenvektoren. Faengt
-- entartete Kreise ab (fast gerader Flug -> riesiger Kreis, siehe
-- MAX_TAS_FACTOR & Co.), die sonst dauerhaft als Wind/TAS haengen bleiben.
local function fitIsPlausible(cx, cy, r, x, y, n)
  if not r or r <= 1e-3 then
    return false
  end

  local maxGround = 0
  for i = 1, n do
    local mag = math.sqrt(x[i] * x[i] + y[i] * y[i])
    if mag > maxGround then
      maxGround = mag
    end
  end
  if maxGround <= 1e-3 then
    return false
  end

  -- TAS und Wind muessen in derselben Groessenordnung wie die gemessenen
  -- Bodengeschwindigkeiten liegen.
  if r > MAX_TAS_FACTOR * maxGround then
    return false
  end
  local candidateWind = math.sqrt(cx * cx + cy * cy)
  if candidateWind > MAX_WIND_FACTOR * maxGround then
    return false
  end

  -- Passen die Punkte ueberhaupt auf den gefundenen Kreis? Bei einem entarteten
  -- Fit ist die Abweichung relativ zum Radius zwar klein, bei einem schlechten
  -- Fit auf echten Daten aber gross -- beides soll verworfen werden.
  local sumSq = 0
  for i = 1, n do
    local dx = x[i] - cx
    local dy = y[i] - cy
    local d = math.sqrt(dx * dx + dy * dy) - r
    sumSq = sumSq + d * d
  end
  local rms = math.sqrt(sumSq / n)
  if rms > MAX_RESIDUAL_FRAC * r then
    return false
  end

  return true
end

-- ============================================================
-- Gemeinsame Aktualisierung: Modell-/Parameterwechsel, Sampling, Fit.
-- Wird von allen drei Source-wakeup()s aufgerufen; die Sample-/Fit-
-- Throttles sorgen dafuer, dass die eigentliche Arbeit trotzdem nur
-- einmal pro Zyklus passiert.
-- ============================================================
local function updateWind()
  local modelName = model.name()
  local now = os.clock()

  if modelName ~= currentModelName then
    currentModelName = modelName
    lastParamsCheck = now
    applyParams(modelName)
  elseif now - lastParamsCheck >= PARAMS_CHECK_INTERVAL then
    lastParamsCheck = now
    applyParams(modelName)
  end

  local speedSrc = system.getSource(speedSourceName)
  if not speedSrc then
    return
  end

  -- Kurs bevorzugt von der konfigurierten GPS-Course-Quelle; ist keine
  -- vorhanden (Sensor liefert nur Position + Zeit), aus zwei
  -- aufeinanderfolgenden GPS-Fixes ableiten.
  local speed = speedSrc:value()
  local courseSrc = system.getSource(courseSourceName)
  local course
  if courseSrc then
    course = courseSrc:value()
  else
    course = derivedCourseFromPosition()
  end

  if now - lastSampleTime >= sampleInterval then
    if speed and course and speed >= minSpeed then
      local rad = course * DEG2RAD
      local vx = speed * math.sin(rad)  -- Ost-Komponente
      local vy = speed * math.cos(rad)  -- Nord-Komponente

      bufX[bufHead] = vx
      bufY[bufHead] = vy
      bufHead = bufHead + 1
      if bufHead > windowSize then
        bufHead = 1
      end
      if bufN < windowSize then
        bufN = bufN + 1
      end

      lastSampleTime = now
    end
  end

  if now - lastFitTime >= FIT_INTERVAL and bufN >= MIN_SAMPLES then
    lastFitTime = now

    if courseSpreadOk(bufX, bufY, bufN, MAX_SPREAD_R) then
      local cx, cy, r, fitOk = kasaFit(bufX, bufY, bufN)
      if fitOk and fitIsPlausible(cx, cy, r, bufX, bufY, bufN) then
        windSpeed = math.sqrt(cx * cx + cy * cy)
        -- cx,cy zeigt in die Richtung, in die der Wind weht (Ost/Nord).
        -- Meteorologische Konvention: Richtung, AUS der der Wind kommt.
        local toDir = math.atan(cx, cy) * RAD2DEG
        if toDir < 0 then toDir = toDir + 360 end
        local fromDir = toDir + 180
        if fromDir >= 360 then fromDir = fromDir - 360 end

        windDirFrom = fromDir
        haveFit = true
      end
    end
  end

  if speed and course then
    tas = estimateTAS(speed, course, windSpeed, windDirFrom)
  end
end

-- ---------------- Sources ----------------

local function speedLikeInit(source)
  source:value(0.0)
  source:decimals(1)
end

local function windDirInit(source)
  source:value(0)
  source:decimals(0)
  source:unit(UNIT_DEGREE)
end

local function windSpdWakeup(source)
  updateWind()
  source:value(haveFit and windSpeed or 0)
end

local function windDirWakeup(source)
  updateWind()
  source:value(haveFit and windDirFrom or 0)
end

local function tasWakeup(source)
  updateWind()
  source:value(tas)
end

local function init()
  system.registerSource({key = "WindSpd", name = makeName("WindSpd"), init = speedLikeInit, wakeup = windSpdWakeup})
  system.registerSource({key = "WindDir", name = makeName("WindDir"), init = windDirInit, wakeup = windDirWakeup})
  system.registerSource({key = "TAS", name = makeName("TAS"), init = speedLikeInit, wakeup = tasWakeup})
end

return {init = init}
