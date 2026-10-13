-- @widget  GPS Course
-- @version 1.0
-- GPS Course: berechnet den Kurs ueber Grund (0..360 Grad) aus zwei
-- aufeinanderfolgenden GPS-Positionen, fuer Empfaenger, die zwar Position
-- und Zeit liefern, aber keinen fertigen GPS-Course-Sensor.
--
-- Methode: Vorwaerts-Azimut (Grosskreis-Bearing) zwischen dem vorherigen und
-- dem aktuellen Fix. Ein "neuer Fix" wird an einer Aenderung der GPS-Zeit-
-- Quelle erkannt -- Ethos ruft wakeup() deutlich haeufiger auf, als der
-- GPS-Empfaenger neue Positionen liefert (typ. 1-5 Hz), sonst wuerde immer
-- wieder derselbe unveraenderte Fix vermessen.
--
-- Zwischen zwei Fixes wird der zuletzt berechnete Kurs gehalten; solange noch
-- kein zweiter, ausreichend weit entfernter Fix vorliegt, liefert der Source 0.
--
-- Positions- und Zeit-Quelle je Modell aus params.txt (liegt neben main.lua;
-- dort auch das Parameter-Schema fuer die Widget-Konfiguration-App, siehe
-- Ethos-Scripts/widget-config).
-- Zeilenformat: Modellname;gpsSource=...;gpsTimeSource=...

local DEG2RAD = math.pi / 180
local RAD2DEG = 180 / math.pi

local translations = {
  GPSCrs = {en = "GPS Course (calc)", de = "GPS Kurs (ber.)"},
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
local DEFAULT_GPS_SOURCE      = "GPS"        -- Positions-Quelle (liefert Lat/Lon)
local DEFAULT_GPS_TIME_SOURCE = "GPS clock"  -- erkennt neue Fixes an ihrer Aenderung

-- Intern fest: Mindestdistanz zwischen zwei Fixes in Metern. Darunter wird
-- kein neuer Kurs berechnet, sondern der letzte gehalten -- im Stand (und bei
-- Schrittgeschwindigkeit) besteht die Positionsaenderung sonst fast nur aus
-- GPS-Rauschen und der Kurs springt wild.
local MIN_FIX_DISTANCE = 3.0
local EARTH_RADIUS = 6371000  -- m

local PARAMS_CHECK_INTERVAL = 1  -- s, wie oft params.txt neu geprueft wird

local function resolveParams(modelName)
  local params = {
    gpsSource = DEFAULT_GPS_SOURCE,
    gpsTimeSource = DEFAULT_GPS_TIME_SOURCE,
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

-- ---------------- Zustand ----------------
local currentModelName = nil
local gpsSourceName, gpsTimeSourceName = DEFAULT_GPS_SOURCE, DEFAULT_GPS_TIME_SOURCE
local lastParamsCheck = 0

local lastFixLat, lastFixLon = nil, nil
local lastFixTimeKey = nil
local course = nil

local function resetFixState()
  lastFixLat, lastFixLon = nil, nil
  lastFixTimeKey = nil
  course = nil
end

local function applyParams(modelName)
  local p = resolveParams(modelName)
  if p.gpsSource ~= gpsSourceName or p.gpsTimeSource ~= gpsTimeSourceName then
    gpsSourceName = p.gpsSource
    gpsTimeSourceName = p.gpsTimeSource
    -- Quellenwechsel: kein Kurs aus zwei nicht zusammengehoerigen Fixes.
    resetFixState()
  end
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

-- Distanz zweier Fixes in Metern, equirektangulare Naeherung. Fuer die
-- wenigen Meter zwischen zwei GPS-Updates mehr als genau genug.
local function distanceM(lat1, lon1, lat2, lon2)
  local dLat = (lat2 - lat1) * DEG2RAD
  local dLon = (lon2 - lon1) * DEG2RAD * math.cos((lat1 + lat2) / 2 * DEG2RAD)
  return math.sqrt(dLat * dLat + dLon * dLon) * EARTH_RADIUS
end

-- Liest Lat/Lon aus dem Rueckgabewert einer GPS-Positions-Source. Ethos
-- liefert dafuer eine Tabelle; die genaue Feldbenennung ist nicht in jeder
-- Firmware-Version dieselbe, daher werden mehrere gaengige Varianten
-- probiert. Im Zweifel im Sim mit print(gpsSource:value()) gegenpruefen.
local function extractLatLon(value)
  if type(value) ~= "table" then
    return nil, nil
  end
  local lat = value.lat or value.latitude or value.Lat or value.Latitude or value[1]
  local lon = value.lon or value.lng or value.longitude or value.Lon or value.Longitude or value[2]
  if type(lat) == "number" and type(lon) == "number" then
    return lat, lon
  end
  return nil, nil
end

-- Kernlogik: nur bei einem neuen GPS-Fix (erkannt an geaenderter GPS-Zeit)
-- die Position lesen und den Kurs zum vorherigen Fix berechnen.
-- tostring() auf den Rohwert der Zeit-Quelle macht den Vergleich robust
-- gegen deren Rueckgabetyp (Zahl, String oder Tabelle); im schlimmsten Fall
-- wird bei jedem wakeup() gesampelt, was durch MIN_FIX_DISTANCE abgefangen
-- wird.
local function updateCourse()
  local modelName = model.name()
  local now = os.clock()

  if modelName ~= currentModelName then
    currentModelName = modelName
    lastParamsCheck = now
    resetFixState()
    applyParams(modelName)
  elseif now - lastParamsCheck >= PARAMS_CHECK_INTERVAL then
    lastParamsCheck = now
    applyParams(modelName)
  end

  local gpsSrc = system.getSource(gpsSourceName)
  local timeSrc = system.getSource(gpsTimeSourceName)
  if not gpsSrc or not timeSrc then
    return
  end

  local timeKey = tostring(timeSrc:value())
  if timeKey == lastFixTimeKey then
    return  -- kein neuer Fix, alten Kurs halten
  end
  lastFixTimeKey = timeKey

  local lat, lon = extractLatLon(gpsSrc:value())
  if not lat or not lon then
    return
  end

  if lastFixLat and lastFixLon then
    if distanceM(lastFixLat, lastFixLon, lat, lon) >= MIN_FIX_DISTANCE then
      course = bearingDeg(lastFixLat, lastFixLon, lat, lon)
      lastFixLat, lastFixLon = lat, lon
    end
    -- Unter MIN_FIX_DISTANCE bleibt der alte Referenzpunkt stehen, damit sich
    -- langsame Bewegung ueber mehrere Fixes zu einer auswertbaren Strecke
    -- aufsummiert, statt komplett verworfen zu werden.
  else
    lastFixLat, lastFixLon = lat, lon
  end
end

-- ---------------- Source ----------------

local function courseInit(source)
  source:value(0)
  source:decimals(0)
  source:unit(UNIT_DEGREE)
end

local function courseWakeup(source)
  updateCourse()
  source:value(course or 0)
end

local function init()
  system.registerSource({key = "GPSCrs", name = makeName("GPSCrs"), init = courseInit, wakeup = courseWakeup})
end

return {init = init}
