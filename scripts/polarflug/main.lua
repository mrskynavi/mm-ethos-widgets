-- @widget  Polarflug
-- @version 1.1
-- Polarflug fuer Ethos -- Vollbild-Widget, hilft beim Erfliegen der Polare.
--
-- Anzeige 1 (Balken): ein Balken pro Geschwindigkeitsbereich (Standard 20-30, 30-40 ... km/h).
--   Im Balken steht die Zahl der Messpunkte, die fuer die Polare nutzbar sind. Farbe:
--     rot    = zu wenig, die Polare ist in diesem Bereich nicht gestuetzt (Standard unter 20)
--     orange = die Polare laesst sich rechnen, ist aber noch ungenau (20 bis 59)
--     gruen  = Bereich ist ok, es braucht keine weiteren Messungen (ab 60)
--   Der Bereich der aktuellen Fluggeschwindigkeit (TAS) ist weiss umrahmt. Wechselt die TAS im
--   Geradeausflug in einen anderen Bereich, wird er angesagt (nach 2 s, gegen Phygoide).
-- Anzeige 2 (Polare): Knopf oben rechts. Physikalische Polare Sinken = A*v^3 + B/v wie im Tab
--   Polare des MM Flight Analyzers, mit allen Messpunkten, Klassen-Medianen und der aktuellen TAS.
--   Violett: die Polare aus scripts/maccready/polaren.lua. Knopf "-> polaren.lua" (zweimal tippen)
--   aktualisiert dort den Eintrag dieses Modells (alte Datei: polaren_alt.lua): im erflogenen
--   Bereich gilt die Messung, ausserhalb bleibt die alte Polare ("Bereich ergaenzen").
--
-- Messpunkt (gleiche Filter wie der Tab Polare der App): eine Sekunde Gleitflug,
--   Motor aus und 15 s danach, geradeaus (Kurs aendert unter 8 Grad/s), Fahrt aendert unter
--   0.6 m/s^2, gerechnet ueber +-2.5 s; die ersten 30 s zaehlen nicht.
--   Sinken = -(Vario + (v1^2 - v0^2) / (2 g dt))   (barometrisches Vario wie in der App;
--   bei kompensiertem TE-Vario entfaellt der zweite Teil). Fahrt = TAS.
--   Zusaetzlich zur App: gezaehlt wird erst, wenn der Wind gemessen ist (einmal kreisen),
--   sonst waere die TAS nur die Grundgeschwindigkeit.
-- Die Messpunkte bleiben pro Modell gespeichert (punkte_<Modell>.txt neben diesem Skript) und
--   sammeln sich ueber mehrere Fluege. Loeschen in der Konfiguration.
--
-- Wind und TAS: unveraendert aus dem Sollfahrt-Widget uebernommen (GPS-Kurs ab 3 m, Bodenvektor
--   mit GPS-Geschw., Kasa-Kreisfit ueber ~28 s, Plausibilitaet wie im Analyzer, erster Fit nur
--   mit vollem Fenster, Windtabelle in 100-m-Schichten nach GPS-Hoehe).
--
-- Alle Zahlenfelder sind ganzzahlig.

local KMH = 3.6
local G = 9.81
local STORE_VER = 1

-- ---------------------------------------------------------------- Einstellungen

local SOURCE_KEYS = {"gps", "gpsSpd", "gpsAlt", "vario", "motor"}

-- Reihenfolge nur am Ende erweitern: Ethos-Storage arbeitet nach Aufrufreihenfolge.
local NUM_DEFS = {
  {key = "varioType",  def = 2},    -- 1 = kompensiert (TE), 2 = barometrisch (wie die App)
  {key = "teFactor",   def = 100},  -- barometrisch: Energiekompensation in %
  {key = "motorThr",   def = 3},    -- Motor-Sensor ueber diesem Wert = Motor an (z. B. 3 A, 300 U/min)
  {key = "vMin",       def = 20},   -- erster Bereich ab (km/h)
  {key = "vMax",       def = 100},  -- letzter Bereich bis (km/h)
  {key = "width",      def = 10},   -- Breite eines Bereichs (km/h)
  {key = "redBelow",   def = 20},   -- rot unter so vielen Messpunkten
  {key = "greenFrom",  def = 60},   -- gruen ab so vielen Messpunkten
  {key = "announceMode", def = 2},  -- 1 = aus, 2 = Bereich (z. B. 40), 3 = Fahrt gerundet
  {key = "annDelay",   def = 20},   -- Ansage erst nach so langer Zeit im neuen Bereich (0.1 s)
  {key = "minSpeed",   def = 15},   -- darunter keine Ansage (km/h)
}

-- ---------------------------------------------------------------- Wind und TAS (aus Sollfahrt)

local function clamp(x, lo, hi)
  if x < lo then return lo end
  if x > hi then return hi end
  return x
end

-- Exponentielles Tiefpassfilter mit Zeitschritt dt und Zeitkonstante tau (s).
local function lpf(prev, x, dt, tau)
  if prev == nil then return x end
  if tau <= 0.05 then return x end
  return prev + (1 - math.exp(-dt / tau)) * (x - prev)
end

-- ---------------------------------------------------------------- GPS und Wind

local M_PER_DEG = 111320
-- Lua 5.3+ hat kein math.atan2: math.atan mit zwei Argumenten
local atan2 = math.atan2 or function(y, x) return math.atan(y, x) end
local SAMPLE_DT = 0.7         -- Abstand der Geschwindigkeits-Stuetzwerte (s)
local WINDOW = 40             -- Stuetzwerte im Fit-Fenster (~28 s)
local MIN_SAMPLES = 10
local FIT_INTERVAL = 1.0
local MAX_SPREAD_R = 0.85     -- Richtungsstreuung: 1 = alle gleich, 0 = voller Kreis
local MAX_TAS_FACTOR = 2.0    -- Radius darf max. so viel groesser als die mittlere Grundgeschw. sein
local MAX_WIND_FACTOR = 1.0   -- Wind darf nicht groesser als die Fluggeschwindigkeit sein
local MAX_RESIDUAL_FRAC = 0.25
local WIND_HOLD = 900         -- s, nach so langer Zeit ohne Messung gilt eine Schicht als veraltet (Anzeige)

-- Windtabelle: Schichten nach Hoehe. Der erste gute Fit fuellt alle Schichten (Startwert),
-- jeder weitere Fit ueberschreibt/ergaenzt nur die Schichten seines Hoehenbereichs.
local BINS, BIN_H, BIN_OFF = 120, 100, 500      -- 120 Schichten a 100 m (-500 m ... 11500 m)
local BIN_ALPHA = 0.3                           -- Gewicht eines neuen Fits in einer Schicht
local function binOf(alt) return clamp(math.floor((alt + BIN_OFF) / BIN_H) + 1, 1, BINS) end
local function binCenter(i) return (i - 1) * BIN_H - BIN_OFF + BIN_H / 2 end

-- Der erste Fit gilt fuer ALLE Hoehen, deshalb braucht er ein volles Fenster (~28 s). Mit wenigen
-- Stuetzwerten passt beim Start (Beschleunigen, erste Kurve) oft ein falscher Kreis.
local function minSamples(w) return w.wt.filled and MIN_SAMPLES or WINDOW end

local function windApply(w, cx, cy, lo, hi, now)
  local t = w.wt
  if not t.filled then
    for i = 1, BINS do t[i] = {cx = cx, cy = cy, t = now, meas = false} end
    t.filled = true
  end
  for i = binOf(lo), binOf(hi) do
    local b = t[i]
    if b.meas then
      b.cx, b.cy = b.cx + BIN_ALPHA * (cx - b.cx), b.cy + BIN_ALPHA * (cy - b.cy)
    else
      b.cx, b.cy = cx, cy
    end
    b.t, b.meas = now, true
  end
end

-- Wind (m/s Ost/Nord) in der Hoehe alt, linear zwischen den Schichtmitten. Dritter Wert: Schicht.
local function windAt(w, alt)
  local t = w.wt
  if not t.filled then return nil end
  local i = binOf(alt)
  local c = binCenter(i)
  local j = clamp((alt >= c) and i + 1 or i - 1, 1, BINS)
  local f = (j == i) and 0 or clamp(math.abs(alt - c) / BIN_H, 0, 0.5)
  local a, b = t[i], t[j]
  return a.cx * (1 - f) + b.cx * f, a.cy * (1 - f) + b.cy * f, a
end

-- Kreisfit nach Kasa. pts = {{x=, y=}, ...} (Ost/Nord-Geschwindigkeit in m/s).
-- Liefert cx, cy (= Wind), r (= TAS), relativer Rest.
local function kasaFit(pts)
  local n = #pts
  if n < 3 then return nil end
  local mx, my = 0, 0
  for _, p in ipairs(pts) do mx, my = mx + p.x, my + p.y end
  mx, my = mx / n, my / n
  local Suu, Svv, Suv, Suuu, Svvv, Suvv, Svuu = 0, 0, 0, 0, 0, 0, 0
  for _, p in ipairs(pts) do
    local u, v = p.x - mx, p.y - my
    Suu, Svv, Suv = Suu + u * u, Svv + v * v, Suv + u * v
    Suuu, Svvv = Suuu + u * u * u, Svvv + v * v * v
    Suvv, Svuu = Suvv + u * v * v, Svuu + v * u * u
  end
  local det = Suu * Svv - Suv * Suv
  if math.abs(det) < 1e-9 then return nil end
  local b1, b2 = 0.5 * (Suuu + Suvv), 0.5 * (Svvv + Svuu)
  local uc = (b1 * Svv - b2 * Suv) / det
  local vc = (b2 * Suu - b1 * Suv) / det
  local r2 = uc * uc + vc * vc + (Suu + Svv) / n
  if r2 <= 0 then return nil end
  local r = math.sqrt(r2)
  local cx, cy = mx + uc, my + vc
  local res = 0
  for _, p in ipairs(pts) do
    local d = math.sqrt((p.x - cx) ^ 2 + (p.y - cy) ^ 2)
    res = res + (d - r) ^ 2
  end
  return cx, cy, r, math.sqrt(res / n) / r
end

-- Genug Richtungsaenderung im Fenster (Kreisen/Kurven)? 1 = alle Richtungen gleich.
local function courseSpread(pts)
  local sx, sy, n = 0, 0, 0
  for _, p in ipairs(pts) do
    local m = math.sqrt(p.x * p.x + p.y * p.y)
    if m > 0.1 then sx, sy, n = sx + p.x / m, sy + p.y / m, n + 1 end
  end
  if n == 0 then return 1 end
  return math.sqrt(sx * sx + sy * sy) / n
end

local function fitIsPlausible(cx, cy, r, resFrac, pts)
  if not r or r < 1 then return false end
  local mean = 0
  for _, p in ipairs(pts) do mean = mean + math.sqrt(p.x * p.x + p.y * p.y) end
  mean = mean / #pts
  if r > MAX_TAS_FACTOR * mean then return false end
  if math.sqrt(cx * cx + cy * cy) > MAX_WIND_FACTOR * r then return false end
  if resFrac > MAX_RESIDUAL_FRAC then return false end
  return true
end

-- Breiten-/Laengengrad aus dem, was value() der GPS-Quelle liefert.
local function extractLatLon(v)
  if type(v) ~= "table" then return nil end
  local lat = v.lat or v.latitude or v.Lat or v.Latitude or v[1]
  local lon = v.lon or v.lng or v.long or v.longitude or v.Lon or v.Longitude or v[2]
  if type(lat) == "number" and type(lon) == "number" then return lat, lon end
  return nil
end

-- Position der GPS-Quelle. Liefert lat, lon oder nil und eine Fehlermeldung.
local validSource   -- (unten definiert)
local function readPosition(src)
  if not validSource(src) then return nil, nil, "GPS-Quelle waehlen" end
  local ok, v = pcall(src.value, src)
  if ok then
    local la, lo = extractLatLon(v)
    if la then return la, lo end
  end
  if OPTION_LATITUDE and OPTION_LONGITUDE then
    local ok1, la = pcall(src.value, src, {options = OPTION_LATITUDE})
    local ok2, lo = pcall(src.value, src, {options = OPTION_LONGITUDE})
    if ok1 and ok2 and type(la) == "number" and type(lo) == "number" then return la, lo end
  end
  return nil, nil, "GPS: Position nicht lesbar"
end

-- Neue GPS-Position verarbeiten: Geschwindigkeitsvektor, Wind-Fit, TAS, Gegenwind.
-- Stuetzwerte entstehen nur, wenn sich die Position wirklich geaendert hat (frischer Fix),
-- damit der Zeitstempel zur Position passt.
local function gpsUpdate(w, lat, lon, now)
  w.gpsOk = w.gSeen ~= nil and now - w.gSeen <= 3
  if lat == nil then return end
  local fresh = (lat ~= w.lastLat or lon ~= w.lastLon)
  w.lastLat, w.lastLon = lat, lon
  if fresh then w.gSeen = now end
  w.gpsOk = w.gSeen ~= nil and now - w.gSeen <= 3
  if w.bLat == nil or now - w.bT > 3 then          -- Basis (neu) setzen
    w.bLat, w.bLon, w.bT = lat, lon, now
    return
  end
  local dt = now - w.bT
  if fresh and dt >= SAMPLE_DT then
    local vn = (lat - w.bLat) * M_PER_DEG / dt
    local ve = (lon - w.bLon) * M_PER_DEG * math.cos(math.rad(lat)) / dt
    w.bLat, w.bLon, w.bT = lat, lon, now
    w.gve, w.gvn = ve, vn            -- letzter Stuetzwert (Glaettung folgt in process)
    local win = w.win
    -- grosser Hoehensprung: alte Stuetzwerte gehoeren zu einer anderen Windschicht
    if #win > 0 and math.abs((w.alt or 0) - win[#win].alt) > 150 then for i = #win, 1, -1 do win[i] = nil end end
    if math.sqrt(ve * ve + vn * vn) >= 2 then
      win[#win + 1] = {x = ve, y = vn, alt = w.alt or 0}
      if #win > WINDOW then table.remove(win, 1) end
    end
    -- Kreisfit
    if #win >= minSamples(w) and (w.fitT == nil or now - w.fitT >= FIT_INTERVAL) then
      w.fitT = now
      local cx, cy, r, res = kasaFit(win)
      if cx and courseSpread(win) <= MAX_SPREAD_R and fitIsPlausible(cx, cy, r, res, win) then
        w.fit = {cx = cx, cy = cy, r = r, t = now}      -- letzter roher Fit
        local lo, hi = win[1].alt, win[1].alt
        for _, p in ipairs(win) do lo, hi = math.min(lo, p.alt), math.max(hi, p.alt) end
        windApply(w, cx, cy, lo, hi, now)
      end
    end
  end
end

-- Aus Geschwindigkeitsvektor und Wind: Grundgeschwindigkeit, Kurs, TAS, Gegenwind (m/s), Wind.
local function airData(w, now)
  if not w.gpsOk or w.gve == nil then return nil end
  local gs = math.sqrt(w.gve ^ 2 + w.gvn ^ 2)
  local course = atan2(w.gve, w.gvn)
  local alt = w.alt or 0
  local cx, cy, bin = windAt(w, alt)
  local tas, head, wind = gs, nil, nil
  if cx then
    -- Fluggeschwindigkeit mit dem Wind der aktuellen Hoehe (Windtabelle)
    tas = math.sqrt((w.gve - cx) ^ 2 + (w.gvn - cy) ^ 2)
    head = -(cx * math.sin(course) + cy * math.cos(course))
    local from = math.deg(atan2(cx, cy)) + 180
    wind = {spd = math.sqrt(cx ^ 2 + cy ^ 2), from = from % 360, age = now - bin.t, meas = bin.meas, alt = alt}
  end
  return {gs = gs, course = course, tas = tas, head = head, wind = wind}
end

validSource = function(source)
  if source and type(source) ~= "boolean" then return source end
  return nil
end

local function readNumber(source)
  if not validSource(source) then return nil end
  local v = source:value()
  if type(v) == "number" then return v end
  return nil
end

-- ---- Mit GPS-Geschw.: Bodenvektor wie im MM Flight Analyzer
-- Kurs = Azimut vom letzten Ankerpunkt zur neuen Position, erst ab 3 m Abstand.
local MIN_FIX_DISTANCE = 3.0
local R_EARTH = 6371000
local function distM(la1, lo1, la2, lo2)
  local dLat = math.rad(la2 - la1)
  local dLon = math.rad(lo2 - lo1) * math.cos(math.rad((la1 + la2) / 2))
  return math.sqrt(dLat * dLat + dLon * dLon) * R_EARTH
end
local function bearingDeg(la1, lo1, la2, lo2)
  local p1, p2, dl = math.rad(la1), math.rad(la2), math.rad(lo2 - lo1)
  local y = math.sin(dl) * math.cos(p2)
  local x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
  return math.deg(atan2(y, x)) % 360
end

local function courseUpdate(w, lat, lon, now)
  if lat ~= nil and (lat ~= w.lastLat or lon ~= w.lastLon) then
    w.lastLat, w.lastLon, w.gSeen = lat, lon, now
    if w.aLat == nil then
      w.aLat, w.aLon = lat, lon
    elseif distM(w.aLat, w.aLon, lat, lon) >= MIN_FIX_DISTANCE then
      w.crs = bearingDeg(w.aLat, w.aLon, lat, lon)
      w.aLat, w.aLon = lat, lon
    end
  end
  w.gpsOk = w.gSeen ~= nil and now - w.gSeen <= 3
end

-- Geschwindigkeit des Sensors in m/s (Ethos liefert eine Einheiten-Konstante, der Simulator den Text).
local function speedMs(src)
  local v = readNumber(src)
  if v == nil then return nil end
  local ok, u = pcall(src.unit, src)
  if not ok then u = nil end
  if u == "m/s" or (UNIT_METER_PER_SECOND and u == UNIT_METER_PER_SECOND) then return v end
  if u == "kn" or (UNIT_KNOT and u == UNIT_KNOT) then return v * 0.514444 end
  if u == "mph" or (UNIT_MPH and u == UNIT_MPH) then return v * 0.44704 end
  return v / KMH                                   -- Standard km/h
end

-- Plausibel wie im Analyzer: Radius hoechstens 2x, Wind hoechstens 1x die groesste
-- Grundgeschwindigkeit im Fenster, mittlere Abweichung vom Kreis hoechstens 25 % des Radius.
local function fitOkMax(cx, cy, r, resFrac, pts)
  if not r or r <= 1e-3 then return false end
  local maxG = 0
  for _, p in ipairs(pts) do maxG = math.max(maxG, math.sqrt(p.x * p.x + p.y * p.y)) end
  if maxG <= 1e-3 or r > MAX_TAS_FACTOR * maxG then return false end
  if math.sqrt(cx * cx + cy * cy) > MAX_WIND_FACTOR * maxG then return false end
  return resFrac <= MAX_RESIDUAL_FRAC
end

-- Ein Schritt mit Sensor-Geschwindigkeit: Bodenvektor, Stuetzwert alle 0.7 s, Fit jede Sekunde,
-- gute Fits gehen wie bisher in die Windtabelle (Hoehenbereich des Fensters).
local function sensorUpdate(w, lat, lon, spd, now)
  courseUpdate(w, lat, lon, now)
  if spd ~= nil and w.crs ~= nil then
    local c = math.rad(w.crs)
    w.gve, w.gvn = spd * math.sin(c), spd * math.cos(c)
  else
    w.gve, w.gvn = nil, nil
  end
  local win = w.win
  if w.gve and spd >= 2 / KMH and (w.sT == nil or now - w.sT >= SAMPLE_DT) then
    w.sT = now
    -- grosser Hoehensprung: alte Stuetzwerte gehoeren zu einer anderen Windschicht
    if #win > 0 and math.abs((w.alt or 0) - win[#win].alt) > 150 then for i = #win, 1, -1 do win[i] = nil end end
    win[#win + 1] = {x = w.gve, y = w.gvn, alt = w.alt or 0}
    if #win > WINDOW then table.remove(win, 1) end
  end
  if #win >= minSamples(w) and (w.fitT == nil or now - w.fitT >= FIT_INTERVAL) then
    w.fitT = now
    if courseSpread(win) < MAX_SPREAD_R then
      local cx, cy, r, res = kasaFit(win)
      if cx and fitOkMax(cx, cy, r, res, win) then
        w.fit = {cx = cx, cy = cy, r = r, t = now}
        local lo, hi = win[1].alt, win[1].alt
        for _, p in ipairs(win) do lo, hi = math.min(lo, p.alt), math.max(hi, p.alt) end
        windApply(w, cx, cy, lo, hi, now)
      end
    end
  end
end

-- Ist das Widget gerade auf dem Bildschirm? (lcd.isVisible ab Ethos 1.5)
-- Fehlt die Funktion (aelteres Ethos), gilt das Widget als sichtbar.
-- Achtung: in Modell-/Systemmenues kann Ethos faelschlich "sichtbar" melden.
local function widgetVisible()
  if not (lcd and lcd.isVisible) then return true end
  local ok, v = pcall(lcd.isVisible)
  return not ok or v ~= false
end

-- Spielt einen Ton. Ethos erwartet ganze Zahlen (Hz, ms), deshalb wird gerundet.
-- Ist das Widget nicht sichtbar, bleibt es still (Berechnung laeuft trotzdem weiter).
local function playToneSafe(freq, len)
  if not widgetVisible() then return true end
  if not (system and system.playTone) then return false, "system.playTone fehlt" end
  local ok, err = pcall(system.playTone, math.floor(freq + 0.5), math.floor(len + 0.5), 0)
  if not ok then return false, tostring(err) end
  return true
end

local function currentModelName()
  if model and model.name then
    local ok, name = pcall(model.name)
    if ok and type(name) == "string" then return name end
  end
  return nil
end

-- ---------------------------------------------------------------- Messpunkte (wie Tab Polare der App)

local HALF = 2.5          -- Fenster +-2.5 s
local AFTER_MOTOR = 15    -- s nach dem Motor zaehlt nichts
local MAX_TURN = 8        -- Grad/s
local MAX_ACC = 0.6       -- m/s^2
local MIN_V = 5           -- m/s
local SKIP = 30           -- die ersten 30 s zaehlen nicht
local SAMPLE_STEP = 0.2   -- Stuetzwerte im Fenster (s)
local CLASS_W, CLASS_MIN = 4, 12   -- Klassen der physikalischen Kurve (km/h, mind. Punkte), wie in der App
local MAX_POINTS = 4000

local function nBrackets(w) return clamp(math.ceil((w.vMax - w.vMin) / math.max(w.width, 1)), 1, 20) end
local function bracketOf(w, kmh) return math.floor((kmh - w.vMin) / math.max(w.width, 1)) + 1 end
local function bracketLo(w, b) return w.vMin + (b - 1) * w.width end

-- Ethos bricht einen Aufruf ab, wenn er zu viele Lua-Befehle braucht ("Max instructions count
-- reached"). Alles, was ueber alle Messpunkte laeuft, wird deshalb in Stuecken zu CHUNK Punkten
-- pro wakeup erledigt (Laden, Zaehlen, Klassen, Speichern, Punkte fuers Diagramm).
local CHUNK = 250

local function recount(w)
  local n = nBrackets(w)
  local c = {}
  for i = 1, n do c[i] = 0 end
  for i = 1, #w.pv do
    local b = bracketOf(w, w.pv[i])
    if c[b] then c[b] = c[b] + 1 end
  end
  w.cnt = c
  w.cntSig = table.concat({w.vMin, w.vMax, w.width}, ",")
end

-- Zaehlt in Stuecken neu. Liefert true, solange noch Arbeit offen ist.
local function recountStep(w)
  local sig = table.concat({w.vMin, w.vMax, w.width}, ",")
  local j = w.cntJob
  if not j or j.sig ~= sig then
    local c = {}
    for i = 1, nBrackets(w) do c[i] = 0 end
    j = {sig = sig, c = c, i = 1, n = #w.pv}
    w.cntJob = j
  end
  local pv, c = w.pv, j.c
  local e = math.min(j.i + CHUNK - 1, j.n)
  for i = j.i, e do
    local b = bracketOf(w, pv[i])
    if c[b] then c[b] = c[b] + 1 end
  end
  j.i = e + 1
  if j.i <= j.n then return true end
  -- Punkte, die waehrend des Zaehlens dazukamen
  for i = j.n + 1, #pv do
    local b = bracketOf(w, pv[i])
    if c[b] then c[b] = c[b] + 1 end
  end
  w.cnt, w.cntSig, w.cntJob = c, sig, nil
  return false
end

local function addPoint(w, kmh, sink, now)
  local pv, ps = w.pv, w.ps
  pv[#pv + 1], ps[#ps + 1] = kmh, sink
  local b = bracketOf(w, kmh)
  if w.cnt[b] then w.cnt[b] = w.cnt[b] + 1 end
  if #pv > MAX_POINTS then
    -- die aeltesten auf einmal entfernen (table.move ist C), dann neu zaehlen und neu schreiben
    local drop = MAX_POINTS // 10
    local n = #pv
    table.move(pv, drop + 1, n, 1); table.move(ps, drop + 1, n, 1)
    for i = n, n - drop + 1, -1 do pv[i], ps[i] = nil, nil end
    w.cntSig, w.rewrite = nil, true
  end
  w.dirty, w.fitDirty, w.lastOk = true, true, now
end

local function angDiff(a, b) return math.abs(((b - a + 540) % 360) - 180) end

-- Stuetzwert in den Ringpuffer. Fehlt etwas, wird der Puffer geleert: kein Fenster ueber eine Luecke.
local function pushSample(w, now, tas, vario, crs)
  local r = w.ring
  if tas == nil or vario == nil or crs == nil then
    for i = #r, 1, -1 do r[i] = nil end
    return
  end
  if #r > 0 and now - r[#r].t < SAMPLE_STEP then return end
  r[#r + 1] = {t = now, v = tas, va = vario, c = crs}
  while #r > 0 and now - r[1].t > 2 * HALF + 1 do table.remove(r, 1) end
end

-- Prueft die Sekunde um now - 2.5 s. Setzt w.reason (warum nicht gezaehlt) oder zaehlt einen Punkt.
local function evalPoint(w, now)
  local tc = now - HALF
  local sec = math.floor(tc)
  if sec == w.lastSec then return end
  local r = w.ring
  if #r < 3 then w.reason = "keine Daten"; return end
  if w.loadIt then w.reason = "laedt Punkte"; return end
  if tc - (w.t0 or now) < SKIP then w.reason = "Start"; return end
  if not w.wt.filled then w.reason = "Wind fehlt: kreisen"; return end
  if w.lastMotor and tc - w.lastMotor < AFTER_MOTOR then w.reason = "Motor"; return end
  local j = 1
  while j < #r and r[j].t < tc - HALF do j = j + 1 end
  local k = #r
  local a, b = r[j], r[k]
  local dt = b.t - a.t
  if dt < 3 then w.reason = "keine Daten"; return end
  if angDiff(a.c, b.c) / dt > MAX_TURN then w.reason = "Kurve"; return end
  if math.abs(b.v - a.v) / dt > MAX_ACC then w.reason = "Fahrt aendert sich"; return end
  local sv, sw = 0, 0
  for m = j, k do sv, sw = sv + r[m].v, sw + r[m].va end
  local n = k - j + 1
  local vm = sv / n
  if vm <= MIN_V then w.reason = "zu langsam"; return end
  local kin = 0
  if w.varioType == 2 then kin = w.teFactor / 100 * (b.v * b.v - a.v * a.v) / (2 * G * dt) end
  local sink = -(sw / n + kin)
  if not (sink > -3 and sink < 5) then w.reason = "Ausreisser"; return end
  w.lastSec = sec
  w.reason = nil
  addPoint(w, vm * KMH, sink, now)
end

-- ---------------------------------------------------------------- Polare (wie die App, "Physikalisch")

local function median(a)
  table.sort(a)
  local n = #a
  if n % 2 == 1 then return a[(n + 1) / 2] end
  return (a[n / 2] + a[n / 2 + 1]) / 2
end

-- Klassen zu 4 km/h mit mind. 12 Punkten, Median des Sinkens.
local function polarClasses(w)
  local bins = {}
  for i = 1, #w.pv do
    local k = math.floor(w.pv[i] / CLASS_W)
    local t = bins[k]
    if not t then t = {}; bins[k] = t end
    t[#t + 1] = w.ps[i]
  end
  local out = {}
  for k, a in pairs(bins) do
    if #a >= CLASS_MIN then out[#out + 1] = {v = k * CLASS_W + CLASS_W / 2, m = median(a), n = #a} end
  end
  table.sort(out, function(x, y) return x.v < y.v end)
  return out
end

-- Kleinste Quadrate fuer Sinken = A*v^3 + B/v (v in m/s), jede Klasse mit ihrer Punktzahl gewichtet.
local function polarFit(bs)
  if #bs < 3 then return nil end
  local s11, s12, s22, t1, t2 = 0, 0, 0, 0, 0
  for _, b in ipairs(bs) do
    local v = b.v / KMH
    local f1, f2 = v ^ 3, 1 / v
    s11, s12, s22 = s11 + b.n * f1 * f1, s12 + b.n * f1 * f2, s22 + b.n * f2 * f2
    t1, t2 = t1 + b.n * f1 * b.m, t2 + b.n * f2 * b.m
  end
  local det = s11 * s22 - s12 * s12
  if det == 0 then return nil end
  local A = (t1 * s22 - t2 * s12) / det
  local B = (s11 * t2 - s12 * t1) / det
  if A <= 0 or B <= 0 then return nil end
  local function wf(kmh) local v = kmh / KMH; return A * v ^ 3 + B / v end
  local vms = (B / (3 * A)) ^ 0.25 * KMH
  local vbg = (B / A) ^ 0.25 * KMH
  return {A = A, B = B, w = wf, vms = vms, wms = wf(vms), vbg = vbg, ld = vbg / KMH / wf(vbg)}
end

local function refreshFit(w)
  w.classes = polarClasses(w)
  w.polarFit = polarFit(w.classes)
  w.fitDirty = false
end

-- Klassen und Fit in Stuecken. Liefert true, solange noch Arbeit offen ist.
local function fitStep(w)
  local j = w.fitJob
  if not j then
    j = {bins = {}, i = 1, n = #w.pv}
    w.fitJob = j
    w.fitDirty = false
  end
  local pv, ps, bins = w.pv, w.ps, j.bins
  local e = math.min(j.i + CHUNK - 1, j.n, #pv)
  for i = j.i, e do
    local k = math.floor(pv[i] / CLASS_W)
    local t = bins[k]
    if not t then t = {}; bins[k] = t end
    t[#t + 1] = ps[i]
  end
  j.i = e + 1
  if j.i <= math.min(j.n, #pv) then return true end
  local out = {}
  for k, a in pairs(bins) do
    if #a >= CLASS_MIN then out[#out + 1] = {v = k * CLASS_W + CLASS_W / 2, m = median(a), n = #a} end
  end
  table.sort(out, function(x, y) return x.v < y.v end)
  w.classes = out
  w.polarFit = polarFit(out)
  w.fitJob = nil
  w.newFor = false     -- neue Parabel rechnen (naechster Schritt)
  return false
end

-- ---------------------------------------------------------------- Speichern pro Modell

local function fileName(w)
  local n = tostring(w.modelName or "Modell"):gsub("[^%w%-_]", "_")
  return "punkte_" .. n .. ".txt"
end

local function loadPoints(w)
  w.pv, w.ps = {}, {}
  w.loadIt, w.saveJob, w.fitJob, w.cntJob, w.rewrite, w.savedN = nil, nil, nil, nil, false, 0
  w.dots, w.classes, w.polarFit, w.newPol, w.newFor = nil, nil, nil, nil, false
  if io and io.open then
    local ok, f = pcall(io.open, fileName(w), "r")
    if ok and f then
      local okR, text = pcall(f.read, f, "*a")
      pcall(f.close, f)
      if okR and type(text) == "string" then
        w.loadIt = text:gmatch("(%-?[%d%.]+);(%-?[%d%.]+)")
      end
    end
  end
  w.cntSig, w.fitDirty, w.dirty = nil, true, false
end

-- Zerlegt die geladene Datei in Stuecken. Liefert true, solange noch Arbeit offen ist.
local function loadStep(w)
  local it = w.loadIt
  local pv, ps = w.pv, w.ps
  for _ = 1, CHUNK do
    local a, b = it()
    if not a then
      w.loadIt, w.cntSig, w.fitDirty, w.savedN = nil, nil, true, #pv
      return false
    end
    a, b = tonumber(a), tonumber(b)
    if a and b then pv[#pv + 1], ps[#ps + 1] = a, b end
  end
  return true
end

local FILE_HEAD = "# Polarflug Messpunkte: Fahrt km/h;Sinken m/s\n"

local function writeFile(w, mode, text)
  if not (io and io.open) then w.saveErr = "io.open fehlt"; return false end
  local ok, f = pcall(io.open, fileName(w), mode)
  if not (ok and f) then return false end
  local okW, e = pcall(f.write, f, text)
  pcall(f.close, f)
  w.saveErr = (not okW) and tostring(e) or nil
  return okW
end

-- Neue Punkte ans Ende der Datei haengen (wenige Punkte, kurzer Aufruf). Klappt das nicht, oder
-- wurden alte Punkte entfernt, wird die Datei in Stuecken neu geschrieben (saveStep).
local function savePoints(w, now)
  w.saveT = now
  local n = #w.pv
  if w.rewrite or w.loadIt or (w.savedN or 0) > n then w.rewrite = true; return end
  if (w.savedN or 0) == n then w.dirty = false; return end
  local parts = {}
  for i = (w.savedN or 0) + 1, n do parts[#parts + 1] = string.format("%.1f;%.2f\n", w.pv[i], w.ps[i]) end
  if writeFile(w, "a", table.concat(parts)) then
    w.savedN, w.dirty = n, false
  else
    w.rewrite = true
  end
end

-- Datei in Stuecken neu schreiben. Liefert true, solange noch Arbeit offen ist.
local function saveStep(w, now)
  local j = w.saveJob
  if not j then
    j = {parts = {FILE_HEAD}, i = 1}
    w.saveJob = j
    w.rewrite = false
  end
  local pv, ps, parts = w.pv, w.ps, j.parts
  local e = math.min(j.i + CHUNK - 1, #pv)
  for i = j.i, e do parts[#parts + 1] = string.format("%.1f;%.2f\n", pv[i], ps[i]) end
  j.i = e + 1
  if j.i <= #pv then return true end
  w.saveJob, w.saveT = nil, now
  if writeFile(w, "w", table.concat(parts)) then
    w.savedN, w.dirty = #pv, false
  elseif not w.saveErr then
    w.saveErr = "Speichern nicht moeglich"
  end
  return false
end

local function clearPoints(w)
  w.pv, w.ps = {}, {}
  w.loadIt, w.saveJob, w.fitJob, w.cntJob, w.rewrite = nil, nil, nil, nil, false
  w.cntSig, w.fitDirty, w.dots = nil, true, nil
  w.saveT = os.clock()
  if writeFile(w, "w", FILE_HEAD) then w.savedN, w.dirty = 0, false else w.saveErr = "Speichern nicht moeglich" end
end

-- ---------------------------------------------------------------- Ansage

-- Einheit km/h fuer system.playNumber. Der Name der Konstante ist je nach Ethos-Version anders
-- (Ethos 1.6.6 kennt UNIT_KPH nicht). Sonst einmal in _G suchen; ohne Treffer nur die Zahl.
local kmhUnit
local function findKmhUnit()
  if kmhUnit ~= nil then return kmhUnit end
  kmhUnit = UNIT_KPH or UNIT_KMH or UNIT_KM_PER_HOUR or UNIT_KILOMETER_PER_HOUR or UNIT_KMPH
  if kmhUnit == nil then
    for k, v in pairs(_G) do
      if type(k) == "string" and type(v) == "number" and k:sub(1, 5) == "UNIT_"
        and (k:find("KPH") or k:find("KMH") or k:find("KILOMETER") or k:find("KM_")) then
        kmhUnit = v
        break
      end
    end
  end
  if kmhUnit == nil then kmhUnit = false end
  return kmhUnit
end

local function speak(w, value, now)
  if not widgetVisible() then return end   -- unsichtbar: keine Ansage (wie Sollfahrt)
  w.annText, w.annT = string.format("%d km/h angesagt", value), now
  w.annCount = (w.annCount or 0) + 1
  local err
  if not (system and system.playNumber) then err = "system.playNumber fehlt"
  else
    local u = findKmhUnit()
    local ok, e
    if u then ok, e = pcall(system.playNumber, value, u, 0) end
    if not ok then ok, e = pcall(system.playNumber, value, UNIT_RAW or 0, 0) end   -- nur die Zahl
    if not ok then err = tostring(e) end
  end
  w.annErr = err
  if err then playToneSafe(1000, 90); playToneSafe(1300, 90) end
end

-- Neuer Bereich: erst wenn die TAS ein Stueck (1/4 Bereich, max. 2.5 km/h) ueber der Grenze liegt
-- und so lange im Bereich bleibt, wie in der Konfiguration eingestellt (Standard 2 s).
-- Angesagt wird nur im Geradeausflug (Kurs aendert unter 12 Grad/s), sonst redet das Widget
-- beim Kreisen in der Thermik dauernd.
local ANN_TURN = 12
local function straight(w)
  local r = w.ring
  if #r < 3 then return false end
  local dt = r[#r].t - r[1].t
  return dt >= 2 and angDiff(r[1].c, r[#r].c) / dt <= ANN_TURN
end

local function announceStep(w, now, kmh)
  if w.announceMode == 1 or kmh == nil or kmh < w.minSpeed or not straight(w) then w.candBr = nil; return end
  local b = bracketOf(w, kmh)
  if b == w.annBr then w.candBr = nil; return end
  if b ~= w.candBr then
    local lo = bracketLo(w, b)
    local m = math.min(2.5, w.width / 4)
    if w.annBr ~= nil and (kmh < lo + m or kmh > lo + w.width - m) then return end
    w.candBr, w.candT = b, now
  end
  if now - w.candT >= w.annDelay / 10 then
    w.annBr, w.candBr = b, nil
    local value = (w.announceMode == 3) and math.floor(kmh + 0.5) or bracketLo(w, b)
    speak(w, value, now)
  end
end

-- ---------------------------------------------------------------- Polare in polaren.lua uebernehmen
-- Knopf in der Polaren-Ansicht: die gemessene Kurve ersetzt den Eintrag dieses Modells in
-- scripts/maccready/polaren.lua (dieselbe Datei, die der MM Flight Analyzer schreibt und das
-- Sollfahrt-Widget liest). Format wie in der App: Vario = a*v^2 + b*v + c (m/s, Sinken negativ),
-- gueltig vMin..vMax (m/s). Die Parabel wird wie in der App (mcParabola) auf die Kurve
-- A*v^3 + B/v im gemessenen Fahrtbereich gelegt. Die alte Datei bleibt als polaren_alt.lua daneben.

local POLAR_PATHS = {
  "/scripts/maccready/polaren.lua",
  "scripts/maccready/polaren.lua",
  "../maccready/polaren.lua",
}

local function readText(path)
  if not (io and io.open) then return nil end
  local ok, f = pcall(io.open, path, "r")
  if not (ok and f) then return nil end
  local okR, text = pcall(f.read, f, "*a")
  pcall(f.close, f)
  if okR and type(text) == "string" then return text end
  return nil
end

local function writeText(path, text)
  local ok, f = pcall(io.open, path, "w")
  if not (ok and f) then return "kann " .. path .. " nicht schreiben" end
  local okW, e = pcall(f.write, f, text)
  pcall(f.close, f)
  if not okW then return tostring(e) end
  return nil
end

-- Liest polaren.lua. Liefert Tabelle, Pfad, Text (Tabelle leer, wenn die Datei fehlt).
local function loadPolarFile()
  for _, path in ipairs(POLAR_PATHS) do
    local text = readText(path)
    if text and load then
      local okL, chunk = pcall(load, text, "polaren", "t", {})
      if okL and type(chunk) == "function" then
        local ok, t = pcall(chunk)
        if ok and type(t) == "table" then return t, path, text end
      end
    end
  end
  return {}, nil, nil
end

-- Eintrag fuer ein Modell: exakter Name, sonst ohne Gross-/Kleinschreibung und Leerraum.
local function findKey(all, name)
  if type(name) ~= "string" then return nil end
  if type(all[name]) == "table" then return name end
  local function norm(x) return (x:lower():gsub("^%s+", ""):gsub("%s+$", "")) end
  for key, e in pairs(all) do
    if type(key) == "string" and type(e) == "table" and norm(key) == norm(name) then return key end
  end
  return nil
end

-- Gespeicherte Polare des aktiven Modells fuer die Anzeige (Sinken positiv, v in m/s).
local function refreshStored(w, now)
  if w.storedT and now - w.storedT < 30 and w.storedModel == w.modelName then return end
  w.storedT, w.storedModel = now, w.modelName
  local all = loadPolarFile()
  local key = findKey(all, w.modelName)
  local e = key and all[key]
  w.stored = nil
  if e and type(e.a) == "number" and type(e.b) == "number" and type(e.c) == "number" then
    local a, b, c = e.a, e.b, e.c
    if c < 0 then a, b, c = -a, -b, -c end
    w.stored = {a = a, b = b, c = c, vMin = e.vMin, vMax = e.vMax}
  end
end

-- 3x3-Gleichungssystem (Gauss mit Pivot)
local function solve3(M, r)
  local A = {{M[1][1], M[1][2], M[1][3], r[1]}, {M[2][1], M[2][2], M[2][3], r[2]}, {M[3][1], M[3][2], M[3][3], r[3]}}
  for i = 1, 3 do
    local p = i
    for k = i + 1, 3 do if math.abs(A[k][i]) > math.abs(A[p][i]) then p = k end end
    A[i], A[p] = A[p], A[i]
    if math.abs(A[i][i]) < 1e-12 then return nil end
    for k = i + 1, 3 do
      local f = A[k][i] / A[i][i]
      for j = i, 4 do A[k][j] = A[k][j] - f * A[i][j] end
    end
  end
  local x = {}
  for i = 3, 1, -1 do
    local sum = A[i][4]
    for j = i + 1, 3 do sum = sum - A[i][j] * x[j] end
    x[i] = sum / A[i][i]
  end
  return x
end

-- Parabel a*v^2 + b*v + c (Sinken positiv, m/s), kleinste Quadrate ueber die Zielkurve target(v)
-- zwischen lo und hi (m/s), Stuetzstellen alle 0.1 m/s.
local function fitParabola(target, lo, hi)
  local S, R = {0, 0, 0, 0, 0}, {0, 0, 0}
  local v = lo
  while v <= hi + 1e-9 do
    local wv = target(v)
    local p = 1
    for i = 1, 5 do S[i] = S[i] + p; p = p * v end
    R[1], R[2], R[3] = R[1] + wv, R[2] + wv * v, R[3] + wv * v * v
    v = v + 0.1
  end
  local x = solve3({{S[1], S[2], S[3]}, {S[2], S[3], S[4]}, {S[3], S[4], S[5]}}, R)
  if not x then return nil end
  local c, b, a = x[1], x[2], x[3]
  if not (a > 0 and c > 0) then return nil end
  return {a = a, b = b, c = c, lo = lo, hi = hi}
end

-- Parabel durch die Kurve A*v^3 + B/v zwischen lo und hi (m/s), wie mcParabola der App.
local function parabola(fit, lo, hi)
  return fitParabola(function(v) return fit.A * v ^ 3 + fit.B / v end, lo, hi)
end

-- Neue Polare "Bereich ergaenzen": im erflogenen Bereich (Klassen mit genug Punkten) gilt die
-- Messung, ausserhalb bleibt die alte Polare aus polaren.lua (old: Sinken positiv, vMin/vMax m/s).
-- Daraus wird eine Parabel ueber den gemeinsamen Bereich gelegt. Ohne alte Polare: nur die Messung.
-- Liefert die Parabel und den erflogenen Bereich (m/s).
local function newPolar(fit, bs, old)
  if not fit or #bs < 3 then return nil end
  local lo = math.max(bs[1].v - CLASS_W / 2, 1) / KMH
  local hi = (bs[#bs].v + CLASS_W / 2) / KMH
  local L, H = lo, hi
  if old and type(old.vMin) == "number" and type(old.vMax) == "number" and old.vMax > old.vMin then
    L, H = math.min(lo, old.vMin), math.max(hi, old.vMax)
  else
    old = nil
  end
  local p = fitParabola(function(v)
    if v >= lo and v <= hi or not old then return fit.A * v ^ 3 + fit.B / v end
    return old.a * v * v + old.b * v + old.c
  end, L, H)
  if p then p.mlo, p.mhi, p.merged = lo, hi, old ~= nil and (L < lo or H > hi) end
  return p
end

-- Alte Polare eines Eintrags aus polaren.lua (Sinken positiv) oder nil.
local function oldFromEntry(e)
  if not (e and type(e.a) == "number" and type(e.b) == "number" and type(e.c) == "number") then return nil end
  local a, b, c = e.a, e.b, e.c
  if c < 0 then a, b, c = -a, -b, -c end
  if a <= 0 or c <= 0 then return nil end
  return {a = a, b = b, c = c, vMin = tonumber(e.vMin), vMax = tonumber(e.vMax)}
end

local function g5(x) return (string.format("%.5g", x):gsub("^%-0$", "0")) end

-- Schreibt die gemessene Polare. Liefert Meldung (Erfolg) oder nil, Fehler.
local function writeMeasuredPolar(w)
  local f, bs = w.polarFit, w.classes or {}
  if not f or #bs < 3 then return nil, "keine Kurve" end
  local name = w.modelName
  if type(name) ~= "string" or name == "" then return nil, "Modellname fehlt" end
  local all, path, oldText = loadPolarFile()
  path = path or POLAR_PATHS[1]
  local key = findKey(all, name) or name
  local p = newPolar(f, bs, oldFromEntry(all[key]))
  if not p then return nil, "Parabel nicht moeglich" end
  all[key] = {a = -p.a, b = -p.b, c = -p.c, vMin = p.lo, vMax = p.hi}
  if oldText then
    local backup = path:gsub("polaren%.lua$", "polaren_alt.lua")
    writeText(backup, oldText)
  end
  local keys = {}
  for k, e in pairs(all) do if type(k) == "string" and type(e) == "table" then keys[#keys + 1] = k end end
  table.sort(keys)
  local out = {
    "-- MM Flight Analyzer: Polaren fuer das MacCready-Widget",
    "-- Vario (m/s, Sinken negativ) = a*v^2 + b*v + c, v in m/s; vMin/vMax = gueltiger Bereich in m/s",
    string.format("-- %s: Widget Polarflug, %d Messpunkte fuer %.0f-%.0f km/h%s", key, #w.pv, p.mlo * KMH, p.mhi * KMH,
      p.merged and ", ausserhalb alte Polare" or ""),
    "-- Wird von der App ueberschrieben, wenn sie die Polaren auf den Sender schreibt.",
    "return {",
  }
  for _, k in ipairs(keys) do
    local e = all[k]
    out[#out + 1] = string.format("  [%q] = { a = %s, b = %s, c = %s, vMin = %.1f, vMax = %.1f },",
      k, g5(e.a or 0), g5(e.b or 0), g5(e.c or 0), tonumber(e.vMin) or 0, tonumber(e.vMax) or 0)
  end
  out[#out + 1] = "}"
  local err = writeText(path, table.concat(out, "\n") .. "\n")
  if err then return nil, err end
  w.storedT = nil
  return string.format("polaren.lua: %s neu (%.0f-%.0f km/h%s)", key, p.lo * KMH, p.hi * KMH, p.merged and ", ergaenzt" or "")
end

-- ---------------------------------------------------------------- Ablauf

local function create()
  local w = {lastInvalidate = 0, win = {}, wt = {}, ring = {}, pv = {}, ps = {}, cnt = {}, view = 1}
  for _, d in ipairs(NUM_DEFS) do w[d.key] = d.def end
  return w
end

-- Punkte fuers Diagramm (Pixel, ohne doppelte) in Stuecken; Massstab kommt aus paintPolar (w.geo).
local MAX_DOTS = 1000
local function dotsStep(w, now)
  local g = w.geo
  local j = w.dotJob
  if not j then
    local n = #w.pv
    j = {key = g.key, n = n, i = math.max(1, n - MAX_DOTS + 1), d = {}, seen = {}}
    w.dotJob = j
  end
  if j.key ~= g.key then w.dotJob = nil; return true end
  local pv, ps, d, seen = w.pv, w.ps, j.d, j.seen
  local vlo, ymax, px0, py0 = g.vlo, g.ymax, g.px0, g.py0
  local sx, sy = (g.px1 - px0) / (g.vhi - vlo), (g.py1 - py0) / ymax
  local e = math.min(j.i + CHUNK - 1, j.n, #pv)
  for i = j.i, e do
    local v, sk = pv[i], ps[i]
    if v >= vlo and v <= g.vhi and sk >= 0 and sk <= ymax then
      local x, y = math.floor(px0 + (v - vlo) * sx + 0.5), math.floor(py0 + sk * sy + 0.5)
      local k = (x // 2) * 1024 + y // 2
      if not seen[k] then seen[k] = true; d[#d + 1] = x; d[#d + 1] = y end
    end
  end
  j.i = e + 1
  if j.i <= math.min(j.n, #pv) then return true end
  w.dots, w.dotsKey, w.dotsN, w.dotsT, w.dotJob = d, j.key, j.n, now, nil
  return false
end

-- Hintergrundarbeit: pro wakeup hoechstens ein Stueck, damit kein Aufruf zu lang wird.
local function work(w, now)
  if w.loadIt then loadStep(w); return end
  if w.cntJob or w.cntSig ~= table.concat({w.vMin, w.vMax, w.width}, ",") then recountStep(w); return end
  if w.saveJob or (w.rewrite and now - (w.saveT or -99) >= 5) then saveStep(w, now); return end
  if w.fitJob or (w.fitDirty and (w.polT == nil or now - w.polT >= 3)) then
    if not w.fitJob then w.polT = now end
    fitStep(w)
    return
  end
  if not w.storedT or now - w.storedT >= 30 or w.storedModel ~= w.modelName then
    local old = w.stored
    refreshStored(w, now)
    if (old == nil) ~= (w.stored == nil) or (old and w.stored and (old.a ~= w.stored.a or old.c ~= w.stored.c
      or old.vMin ~= w.stored.vMin or old.vMax ~= w.stored.vMax)) then w.newFor = false end
    return
  end
  if not w.newFor then
    w.newFor = true
    w.newPol = newPolar(w.polarFit, w.classes or {}, w.stored)
    return
  end
  local g = w.geo
  if w.view == 2 and g and (w.dotJob or g.key ~= w.dotsKey or (#w.pv ~= w.dotsN and now - (w.dotsT or 0) >= 2)) then
    dotsStep(w, now)
  end
end

local function wakeup(w)
  local now = os.clock()
  local dt = w.lastT and (now - w.lastT) or 0
  w.lastT = now
  if dt > 0.5 then dt = 0.5 end
  w.t0 = w.t0 or now

  -- Messpunkte des aktiven Modells laden (auch nach Modellwechsel)
  local name = currentModelName()
  if not w.loaded or name ~= w.modelName then
    if w.loaded and w.dirty and not w.loadIt then savePoints(w, now) end
    if w.loaded and (w.rewrite or w.saveJob) and not w.loadIt then
      saveStep(w, now)      -- erst fertig schreiben, dann wechseln (naechste wakeups)
    else
      w.modelName, w.loaded = name, true
      loadPoints(w)
    end
  end

  -- Hoehe: GPS-Hoehe, sonst aus dem Vario aufintegriert (wie Sollfahrt)
  local vario = readNumber(w.vario)
  local alt = readNumber(w.gpsAlt)
  if alt == nil then
    w.altInt = (w.altInt or 0) + (vario or 0) * dt
    alt = w.altInt
  end
  w.alt = alt
  local lat, lon, perr = readPosition(w.gps)
  if validSource(w.gpsSpd) then
    sensorUpdate(w, lat, lon, speedMs(w.gpsSpd), now)
  else
    gpsUpdate(w, lat, lon, now)
  end
  local air = airData(w, now)
  w.air = air
  local tas = air and air.tas or nil
  w.tasS = tas and lpf(w.tasS, tas, dt, 1.0) or nil

  local mv = readNumber(w.motor)
  if mv ~= nil and math.abs(mv) > w.motorThr then w.lastMotor = now end

  pushSample(w, now, tas, vario, air and (math.deg(air.course) % 360) or nil)
  evalPoint(w, now)

  local note
  if not validSource(w.gps) then note = "GPS-Quelle waehlen"
  elseif perr then note = perr
  elseif not w.gpsOk then note = "GPS: kein Fix / keine Bewegung"
  elseif vario == nil then note = "Vario-Quelle waehlen" end
  w.dataNote = note

  w.tasA = tas and lpf(w.tasA, tas, dt, 2.0) or nil    -- fuer die Ansage etwas ruhiger
  announceStep(w, now, w.tasA and w.tasA * KMH or nil)

  if w.dirty and (w.saveT == nil or now - w.saveT >= 30) then savePoints(w, now) end
  work(w, now)

  if now - w.lastInvalidate >= 0.2 then
    w.lastInvalidate = now
    lcd.invalidate()
  end
end

-- ---------------------------------------------------------------- Zeichnen

local function rgb(r, g, b) return lcd.RGB(r, g, b) end
local C = {}
local function colors()
  C.BG, C.DIM, C.BOX = rgb(0, 0, 0), rgb(150, 150, 150), rgb(40, 40, 40)
  C.GREEN, C.ORANGE, C.RED, C.BLUE = rgb(0, 200, 80), rgb(255, 170, 0), rgb(255, 60, 50), rgb(0, 170, 255)
  C.GRID, C.PT = rgb(70, 70, 70), rgb(120, 120, 120)
  C.RED_D, C.ORANGE_D, C.GREEN_D, C.BOX_D = rgb(95, 28, 25), rgb(100, 68, 0), rgb(0, 75, 32), rgb(24, 24, 24)
  C.OLD, C.NEW = rgb(170, 120, 255), rgb(0, 220, 90)
end

local function drawT(x, y, boxW, str, font, color, align)
  lcd.font(font)
  lcd.color(color)
  local tw, th = lcd.getTextSize(str)
  local px = x
  if align == "c" then px = x + math.floor((boxW - tw) / 2)
  elseif align == "r" then px = x + boxW - tw end
  lcd.drawText(px, y, str)
  return th, tw
end

local function textHeight(font)
  lcd.font(font)
  local _, th = lcd.getTextSize("0")
  return th
end

local function statusColor(w, n, dim)
  if dim then
    if n < w.redBelow then return C.RED_D end
    if n < w.greenFrom then return C.ORANGE_D end
    return C.GREEN_D
  end
  if n < w.redBelow then return C.RED end
  if n < w.greenFrom then return C.ORANGE end
  return C.GREEN
end

-- Kopfzeile: TAS, Zaehl-Status, Umschaltknopf. Liefert die Hoehe.
local function paintHead(w, W, now)
  local m = 6
  local headH = textHeight(FONT_L) + 6
  local tasStr = w.tasS and string.format("%.0f", w.tasS * KMH) or "--"
  local _, tw = drawT(m, 3, 120, tasStr, FONT_L, WHITE, "l")
  drawT(m + tw + 4, 3 + textHeight(FONT_L) - textHeight(FONT_XS) - 3, 60, "km/h TAS", FONT_XS, C.DIM, "l")
  local btnW = math.min(110, math.floor(W * 0.28))
  local b = {x = W - m - btnW, y = 3, w = btnW, h = headH - 6}
  w.button = b
  lcd.color(rgb(60, 60, 60))
  lcd.drawFilledRectangle(b.x, b.y, b.w, b.h)
  drawT(b.x, b.y + math.floor((b.h - textHeight(FONT_S)) / 2), b.w, w.view == 1 and "POLARE" or "BALKEN", FONT_S, WHITE, "c")
  local right = b.x
  w.saveBtn = nil
  if w.view == 2 then
    -- Knopf "polaren.lua": erster Druck fragt nach, zweiter (innert 4 s) schreibt
    local sb = {x = b.x - 8 - btnW, y = 3, w = btnW, h = headH - 6}
    w.saveBtn = sb
    right = sb.x
    local ready = w.polarFit ~= nil
    local ask = w.saveAsk and now - w.saveAsk < 4
    lcd.color(ask and C.ORANGE or rgb(60, 60, 60))
    lcd.drawFilledRectangle(sb.x, sb.y, sb.w, sb.h)
    drawT(sb.x, sb.y + math.floor((sb.h - textHeight(FONT_XS)) / 2), sb.w, ask and "SICHER?" or "-> polaren.lua",
      FONT_XS, ask and C.BG or (ready and WHITE or C.DIM), "c")
  end
  local st, sc = (w.reason or "keine Daten"), C.DIM
  if w.lastOk and now - w.lastOk < 1.6 then st, sc = "ZAEHLT", C.GREEN
  elseif w.reason == "Wind fehlt: kreisen" then sc = C.ORANGE end
  local cx = m + 120
  drawT(cx, 3 + math.floor((headH - 6 - textHeight(FONT_S)) / 2), right - cx - 4, st, FONT_S, sc, "c")
  return headH
end

-- Fusszeile: Wind und Summe der Punkte (oder Fehler).
local function paintFoot(w, W, H, now)
  local m = 6
  local xsH = textHeight(FONT_XS)
  local y = H - xsH - 3
  local wind = w.air and w.air.wind
  local wt, wc = "Wind: kreisen = messen", C.ORANGE
  if wind then
    wt = string.format("%sWind %.0f km/h aus %03d", wind.meas and "" or "~", wind.spd * KMH, math.floor(wind.from + 0.5) % 360)
    wc = (wind.meas and wind.age <= WIND_HOLD) and WHITE or C.ORANGE
  end
  drawT(m, y, W / 2, wt, FONT_XS, wc, "l")
  local rt, rc = string.format("%d Punkte", #w.pv), C.DIM
  if w.saveMsg and now - (w.saveMsgT or 0) < 8 then rt, rc = w.saveMsg, w.saveMsgErr and C.RED or C.GREEN
  elseif w.annErr then rt, rc = "ANSAGE: " .. string.sub(w.annErr, 1, 40), C.RED
  elseif w.saveErr then rt, rc = "SPEICHERN: " .. string.sub(w.saveErr, 1, 40), C.RED
  elseif w.annText and now - (w.annT or 0) < 4 then rt = w.annText .. "  |  " .. rt end
  drawT(W / 2, y, W / 2 - m, rt, FONT_XS, rc, "r")
  return xsH + 6
end

local function paintBars(w, W, top, bot)
  local m = 6
  local n = nBrackets(w)
  local xsH = textHeight(FONT_XS)
  local labY = bot - xsH - 2
  local y0, y1 = top + 4, labY - 8          -- Balkenflaeche
  local colW = math.floor((W - 2 * m) / n)
  local gap = math.max(2, math.floor(colW * 0.12))
  local bh = y1 - y0
  local cur = w.tasS and bracketOf(w, w.tasS * KMH) or nil
  if cur and (cur < 1 or cur > n) then cur = nil end
  for i = 1, n do
    local x = m + (i - 1) * colW + math.floor(gap / 2)
    local bw = colW - gap
    local cnt = w.cnt[i] or 0
    local active = (i == cur)
    local dim = cur ~= nil and not active      -- andere Bereiche abgedunkelt, solange einer aktiv ist
    lcd.color(dim and C.BOX_D or C.BOX)
    lcd.drawFilledRectangle(x, y0, bw, bh)
    local fh = math.floor(bh * math.min(cnt / math.max(w.greenFrom, 1), 1) + 0.5)
    if cnt > 0 and fh < 3 then fh = 3 end
    if fh > 0 then
      lcd.color(statusColor(w, cnt, dim))
      lcd.drawFilledRectangle(x, y1 - fh, bw, fh)
    end
    -- Schwelle rot/orange als feine Linie
    local ry = y1 - math.floor(bh * w.redBelow / math.max(w.greenFrom, 1) + 0.5)
    lcd.color(C.GRID)
    lcd.drawLine(x, ry, x + bw - 1, ry)
    -- Zahl oben im Balken (aktiver Bereich gross)
    drawT(x, y0 + 4, bw, tostring(cnt), active and FONT_L or FONT_S, dim and C.PT or WHITE, "c")
    -- Beschriftung
    local lo = bracketLo(w, i)
    local lab = (colW >= 46) and string.format("%d-%d", lo, lo + w.width) or tostring(lo)
    drawT(x - 4, labY, bw + 8, lab, FONT_XS, active and WHITE or (dim and C.GRID or C.DIM), "c")
    if active then
      lcd.color(WHITE)
      lcd.drawRectangle(x - 3, y0 - 3, bw + 6, bh + 6, 3)
    end
  end
  -- TAS-Marke unter den Balken
  if w.tasS then
    local v = w.tasS * KMH
    local x = m + (v - w.vMin) / w.width * colW
    if x >= m and x <= m + n * colW then
      x = math.floor(x)
      lcd.color(C.BLUE)
      lcd.drawFilledTriangle(x, y1 + 1, x - 5, y1 + 7, x + 5, y1 + 7)
    end
  end
end

local function paintPolar(w, W, top, bot)
  local m = 6
  local xsH = textHeight(FONT_XS)
  local infoY = bot - xsH - 2
  local px0, px1 = m + 34, W - m - 4
  local py0, py1 = top + 6, infoY - xsH - 8
  local vlo, vhi = w.vMin, w.vMin + nBrackets(w) * w.width
  local bs = w.classes or {}
  local ymax = 1.5
  for _, b in ipairs(bs) do ymax = math.max(ymax, b.m * 1.3) end
  ymax = math.min(math.ceil(ymax * 2) / 2, 5)
  local function X(v) return math.floor(px0 + (v - vlo) / (vhi - vlo) * (px1 - px0) + 0.5) end
  local function Y(s) return math.floor(py0 + s / ymax * (py1 - py0) + 0.5) end

  -- Raster: Bereiche (unten farbig nach Punktzahl), Sinken alle 0.5 m/s (negativ = Vario)
  for i = 1, nBrackets(w) do
    local x0, x1 = X(bracketLo(w, i)), X(bracketLo(w, i) + w.width)
    lcd.color(statusColor(w, w.cnt[i] or 0))
    lcd.drawFilledRectangle(x0 + 1, py1 + 1, x1 - x0 - 1, 4)
    lcd.color(C.GRID)
    lcd.drawLine(x0, py0, x0, py1)
    if i % 2 == 1 or nBrackets(w) <= 8 then
      drawT(x0 - 20, py1 + 6, 40, tostring(bracketLo(w, i)), FONT_XS, C.DIM, "c")
    end
  end
  lcd.color(C.GRID)
  lcd.drawLine(X(vhi), py0, X(vhi), py1)
  local s = 0
  while s <= ymax + 1e-6 do
    lcd.color(C.GRID)
    lcd.drawLine(px0, Y(s), px1, Y(s))
    drawT(m, Y(s) - math.floor(xsH / 2), px0 - m - 4, s == 0 and "0" or string.format("-%.1f", s), FONT_XS, C.DIM, "r")
    s = s + 0.5
  end

  -- Messpunkte (die letzten 1000, als Pixel in wakeup vorbereitet, siehe dotsStep)
  local key = string.format("%d,%d,%d,%d,%d,%d,%.2f", vlo, vhi, px0, px1, py0, py1, ymax)
  if not w.geo or w.geo.key ~= key then
    w.geo = {key = key, vlo = vlo, vhi = vhi, px0 = px0, px1 = px1, py0 = py0, py1 = py1, ymax = ymax}
  end
  local d = w.dots
  if d and w.dotsKey == key then
    lcd.color(C.PT)
    for i = 1, #d, 2 do lcd.drawFilledRectangle(d[i] - 1, d[i + 1] - 1, 2, 2) end
  end

  -- Kurven: alt = polaren.lua (violett), neu = Parabel, die der Knopf schreiben wuerde (gruen)
  local function curve(fn, a, e, color)
    lcd.color(color)
    local lx, ly
    local v = math.max(vlo, a)
    while v <= math.min(vhi, e) + 1e-6 do
      local x, y = X(v), Y(clamp(fn(v / KMH), 0, ymax))
      if lx then lcd.drawLine(lx, ly, x, y); lcd.drawLine(lx, ly + 1, x, y + 1) end
      lx, ly = x, y
      v = v + 2
    end
  end
  local sp = w.stored
  if sp then
    curve(function(ms) return sp.a * ms * ms + sp.b * ms + sp.c end,
      (sp.vMin or vlo / KMH) * KMH, (sp.vMax or vhi / KMH) * KMH, C.OLD)
  end
  local f = w.polarFit
  local np = f and w.newPol
  if np then
    curve(function(ms) return np.a * ms * ms + np.b * ms + np.c end, np.lo * KMH, np.hi * KMH, C.NEW)
    -- erflogener Bereich als Klammer unten an der gruenen Kurve
    lcd.color(C.NEW)
    local bx0, bx1 = X(math.max(vlo, np.mlo * KMH)), X(math.min(vhi, np.mhi * KMH))
    lcd.drawLine(bx0, py1 - 3, bx1, py1 - 3)
    lcd.drawLine(bx0, py1 - 8, bx0, py1 - 3)
    lcd.drawLine(bx1, py1 - 8, bx1, py1 - 3)
  elseif #bs >= 2 then
    lcd.color(C.ORANGE)
    for i = 2, #bs do lcd.drawLine(X(bs[i - 1].v), Y(bs[i - 1].m), X(bs[i].v), Y(bs[i].m)) end
  end
  -- Legende
  local ly = py0 + 2
  if sp then drawT(px0 + 4, ly, 200, "alt: polaren.lua", FONT_XS, C.OLD, "l"); ly = ly + xsH end
  if np then drawT(px0 + 4, ly, 300, np.merged and "neu: Messung + alte ausserhalb" or "neu: wuerde geschrieben", FONT_XS, C.NEW, "l") end
  -- Klassen-Mediane
  lcd.color(WHITE)
  for _, b in ipairs(bs) do
    if b.v >= vlo and b.v <= vhi and b.m >= 0 and b.m <= ymax then lcd.drawFilledCircle(X(b.v), Y(b.m), 3) end
  end
  -- Aktuelle TAS
  if w.tasS then
    local v = w.tasS * KMH
    if v >= vlo and v <= vhi then
      lcd.color(C.BLUE)
      lcd.drawLine(X(v), py0, X(v), py1)
      lcd.drawLine(X(v) + 1, py0, X(v) + 1, py1)
    end
  end
  drawT(px0, py0 + 2, px1 - px0 - 4, "Vario m/s  |  km/h", FONT_XS, C.DIM, "r")

  -- Kennzahlen der Parabel: bestes Gleiten bei sqrt(c/a), auf den gueltigen Bereich begrenzt
  local function glide(p, lo, hi)
    local v = clamp(math.sqrt(p.c / p.a), lo, hi)
    return v / (p.a * v * v + p.b * v + p.c), v * KMH
  end
  local info, ic = "", C.DIM
  if np then
    local ld, vb = glide(np, np.lo, np.hi)
    info, ic = string.format("neu: GZ %.1f bei %.0f km/h", ld, vb), WHITE
    if sp then
      local ld2, vb2 = glide(sp, sp.vMin or 1, sp.vMax or 99)
      info = info .. string.format("   alt: GZ %.1f bei %.0f km/h", ld2, vb2)
    end
    -- Bestes Gleiten oder geringstes Sinken ausserhalb der gemessenen Klassen: unsicher
    local lo, hi = bs[1].v - CLASS_W / 2, bs[#bs].v + CLASS_W / 2
    if f.vbg < lo or f.vbg > hi or f.vms < lo then
      info, ic = info .. "  (unsicher)", C.ORANGE
    end
  elseif #bs >= 3 then
    info = "Mediane passen noch nicht zu A*v^3+B/v: schneller und langsamer fliegen"
    ic = C.ORANGE
  else
    info = string.format("Kurve ab 3 Klassen zu %d km/h mit je %d Punkten (jetzt %d)", CLASS_W, CLASS_MIN, #bs)
  end
  drawT(m, infoY, W - 2 * m, info, FONT_XS, ic, "c")
end

local function paint(w)
  local W, H = lcd.getWindowSize()
  local now = os.clock()
  colors()
  lcd.color(C.BG)
  lcd.drawFilledRectangle(0, 0, W, H)
  local top = paintHead(w, W, now)
  local footH = paintFoot(w, W, H, now)
  if w.view == 2 then paintPolar(w, W, top, H - footH) else paintBars(w, W, top, H - footH) end

  if w.dataNote then
    local nh = textHeight(FONT_STD) + 10
    local ny = math.floor(H / 2) - math.floor(nh / 2)
    lcd.color(C.BG)
    lcd.drawFilledRectangle(0, ny, W, nh)
    drawT(0, ny + 5, W, w.dataNote, FONT_STD, C.ORANGE, "c")
  end
end

-- ---------------------------------------------------------------- Touch / Tasten

local function toggleView(w)
  w.view = (w.view == 1) and 2 or 1
  lcd.invalidate()
  return true
end

local function event(w, category, value, x, y)
  if EVT_TOUCH and category == EVT_TOUCH and (TOUCH_START == nil or value == TOUCH_START) then
    local b = w.button
    if b and x >= b.x and x <= b.x + b.w and y >= b.y and y <= b.y + b.h then return toggleView(w) end
    local sb = w.saveBtn
    if sb and x >= sb.x and x <= sb.x + sb.w and y >= sb.y and y <= sb.y + sb.h then
      local now = os.clock()
      if not w.polarFit then
        w.saveMsg, w.saveMsgErr, w.saveMsgT = "Noch keine Kurve zum Speichern", true, now
      elseif w.saveAsk and now - w.saveAsk < 4 then
        w.saveAsk = nil
        local msg, err = writeMeasuredPolar(w)
        w.saveMsg, w.saveMsgErr, w.saveMsgT = msg or ("polaren.lua: " .. tostring(err)), msg == nil, now
      else
        w.saveAsk = now
      end
      lcd.invalidate()
      return true
    end
  end
  if EVT_KEY and category == EVT_KEY and KEY_ENTER_BREAK and value == KEY_ENTER_BREAK then return toggleView(w) end
  return false
end

-- ---------------------------------------------------------------- Konfiguration

local function addSource(w, label, prop)
  local line = form.addLine(label)
  return form.addSourceField(line, nil, function() return w[prop] end, function(v) w[prop] = v end)
end

local function addNumber(w, label, prop, lo, hi)
  local line = form.addLine(label)
  return form.addNumberField(line, nil, lo, hi, function() return w[prop] end, function(v) w[prop] = v end)
end

local function configure(w)
  addSource(w, "GPS", "gps")
  addSource(w, "GPS-Geschw. (empfohlen)", "gpsSpd")
  addSource(w, "GPS Hoehe (Windschichten)", "gpsAlt")
  addSource(w, "Vario", "vario")
  local line = form.addLine("Vario-Typ")
  local tef
  form.addChoiceField(line, nil, {{"Kompensiert (TE)", 1}, {"Barometrisch", 2}},
    function() return w.varioType end,
    function(v) w.varioType = v; if tef and tef.enable then pcall(tef.enable, tef, v == 2) end end)
  tef = addNumber(w, "Barometrisch: Kompensation (%)", "teFactor", 0, 150)
  if tef and tef.enable then pcall(tef.enable, tef, w.varioType == 2) end
  addSource(w, "Motor (Strom oder Drehzahl)", "motor")
  addNumber(w, "Motor an ueber", "motorThr", 0, 20000)

  addNumber(w, "Bereiche ab (km/h)", "vMin", 0, 200)
  addNumber(w, "Bereiche bis (km/h)", "vMax", 10, 300)
  addNumber(w, "Bereichsbreite (km/h)", "width", 2, 50)
  addNumber(w, "Rot unter (Punkte)", "redBelow", 1, 1000)
  addNumber(w, "Gruen ab (Punkte)", "greenFrom", 2, 2000)

  line = form.addLine("Ansage beim Wechsel")
  form.addChoiceField(line, nil, {{"Aus", 1}, {"Bereich (z. B. 40)", 2}, {"Fahrt gerundet", 3}},
    function() return w.announceMode end, function(v) w.announceMode = v end)
  addNumber(w, "Ansage-Verzoegerung (0.1 s)", "annDelay", 0, 100)
  addNumber(w, "Mindestfahrt Ansage (km/h)", "minSpeed", 0, 100)

  line = form.addLine("Messpunkte")
  form.addStaticText(line, form.getFieldSlots(line)[0], string.format("%d (%s)", #w.pv, fileName(w)))
  line = form.addLine("Messpunkte loeschen")
  if form.addTextButton then
    form.addTextButton(line, nil, "Loeschen", function() clearPoints(w); return true end)
  end
end

local function setDefaults(w)
  for _, k in ipairs(SOURCE_KEYS) do w[k] = nil end
  for _, d in ipairs(NUM_DEFS) do w[d.key] = d.def end
end

local function read(w)
  if storage.read("ver") ~= STORE_VER then
    setDefaults(w)
  else
    for _, k in ipairs(SOURCE_KEYS) do w[k] = validSource(storage.read(k)) end
    for _, d in ipairs(NUM_DEFS) do
      local v = storage.read(d.key)
      w[d.key] = (type(v) == "number") and v or d.def
    end
  end
  w.cntSig = nil
end

local function write(w)
  storage.write("ver", STORE_VER)
  for _, k in ipairs(SOURCE_KEYS) do storage.write(k, w[k]) end
  for _, d in ipairs(NUM_DEFS) do storage.write(d.key, w[d.key]) end
end

local function init()
  -- Ethos akzeptiert Widget-Schluessel nur mit weniger als 8 Zeichen.
  system.registerWidget({
    key = "polfl01",
    name = "Polarflug",
    create = create,
    paint = paint,
    wakeup = wakeup,
    event = event,
    configure = configure,
    read = read,
    write = write,
  })
end

-- Fuer Tests ausserhalb von Ethos
local M = {init = init}
M._test = {
  create = create, wakeup = wakeup, evalPoint = evalPoint, pushSample = pushSample, polarClasses = polarClasses,
  polarFit = polarFit, recount = recount, bracketOf = bracketOf, announceStep = announceStep,
  loadPoints = loadPoints, savePoints = savePoints, clearPoints = clearPoints,
  parabola = parabola, newPolar = newPolar, writeMeasuredPolar = writeMeasuredPolar, loadPolarFile = loadPolarFile,
}
return M
