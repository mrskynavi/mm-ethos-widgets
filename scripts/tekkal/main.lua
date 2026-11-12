-- @widget  TEK-Kalibrierung
-- @version 1.2
-- TEK-Kalibrierung fuer Ethos -- Vollbild-Widget fuer den SM GPS-Logger 3 (Airspeed ueber die TEK-Duese).
-- Bestimmt in einem Flug:
--   1. den Airspeed-Faktor des Loggers (Einstellung 80..120 %) gegen das GPS, der Wind wird dabei
--      mitgeschaetzt (Kreise mit gleichbleibender Fahrt, dazu Geraden in verschiedene Richtungen),
--   2. wie gut die TEK-Duese kompensiert und welche elektronische Korrektur das TEK-Vario braucht
--      (Fahrtwechsel geradeaus: hochziehen, andruecken).
-- Am Flugende stehen beide Werte auf dem Schirm und in ergebnis_<Modell>.txt neben diesem Skript.
--
-- Physik (eine Duese misst beides):
--   Die TEK-Duese senkt den Druck um c * q (q = Staudruck, ideal c = 1). Der Logger rechnet daraus die
--   Fahrt v_mess = sqrt(c) * v und das TEK-Vario = Steigen + c * d(v^2/2g)/dt.
--   Fahrt:  k = v_wahr / v_mess. Neuer Logger-Faktor = alter Faktor * k.
--   TEK:    TE_wahr = TE_mess + x * d(v_mess^2/2g)/dt mit x = (1 - c)/c. Ist die Fahrt richtig gemessen
--           (statischer Druck im Rumpf stimmt), ist x = k^2 - 1. Das Widget misst x zusaetzlich direkt.
--   Elektronische Korrektur pro Modell statt Duese oder Logger aendern (1.2): dieses Skript registriert
--   zwei Quellen. "Fahrt korr." = Logger-Fahrt * airK (IAS in m/s, als "Fahrt Staudruck" in SkyNavigator,
--   Sollfahrt, Polarflug waehlen), "TEK korr." = TEK-Vario + x * d(v^2/2g)/dt aus der Logger-Fahrt (als
--   Vario mit Vario-Art TE oder im Ethos-Vario). Beide Werte stellt man in der Konfiguration ein oder
--   uebernimmt die Messung mit UEBERNEHMEN. Gemessen wird immer gegen die rohe Logger-Fahrt, eine schon
--   eingestellte Korrektur verfaelscht die neue Messung also nicht. Der Logger-Faktor muss dann so bleiben,
--   wie er beim Messen war (Vorgabe 100 %).
--   Die Korrekturen, die Fahrt-Einstellungen und die Sensornamen liegen in korrektur_<Modell>.txt. Damit
--   rechnen die Quellen auch, wenn das Widget nicht auf einem Bildschirm des Modells liegt (es muss nur
--   einmal konfiguriert worden sein).
--
-- Fahrt-Faktor (Kreise): pro voller Kreis (360 Grad, Drehrate ueber 6 Grad/s) Fit
--   |Bodenvektor - Wind| = k * Fahrt_horizontal ueber Wind Ost/Nord und k (Kasa-Startwert, dann
--   Gauss-Newton, pro wakeup ein Schritt). Fahrt_horizontal = sqrt(TAS^2 - Vario^2), TAS aus der
--   Logger-Fahrt (IAS, Normatmosphaere wie SkyNavigator 1.27). Ein Kreis zaehlt, wenn die Fahrt um
--   weniger als 10 % schwankt, die Punkte auf dem Kreis liegen (Rest unter 1.2 m/s und 8 %) und der
--   Wind kleiner als die Fahrt ist. Ergebnis = Median der Kreise.
-- Fahrt-Faktor (Geraden): nach einem Kreis gilt dessen Wind 3 min (+-150 m). Geradeaus (Drehrate unter
--   4 Grad/s seit 3 s) jede Sekunde k = |Boden - Wind| / Fahrt, getrennt nach Fahrtbereich (10 km/h)
--   und Himmelsrichtung (4 Viertel). Ein Bereich gilt, wenn zwei gegenueberliegende Viertel Werte haben
--   (Windfehler hebt sich auf). Zeigt, ob der Faktor von der Fahrt abhaengt.
-- TEK (geradeaus, Drehrate unter 15 Grad/s): Raster 0.1 s, Energie E = v^2/2g, beide Reihen mit 0.25 s
--   geglaettet und hochpass-gefiltert (4 s), dann TE_hp gegen dE_hp regressiert, fuer Verzoegerungen
--   -0.5..+1.0 s; es gilt die Verzoegerung mit dem kleinsten Rest. x = -Sxy/Sxx.
--
-- Alle Zahlenfelder der Konfiguration sind ganzzahlig.

local KMH = 3.6
local G = 9.81
local STORE_VER = 1
local atan2 = math.atan2 or function(y, x) return math.atan(y, x) end

-- ---------------------------------------------------------------- Einstellungen

local SOURCE_KEYS = {"air", "vario", "gpsSpd", "gpsCrs", "gps", "alt"}

-- Reihenfolge nur am Ende erweitern: Ethos-Storage arbeitet nach Aufrufreihenfolge.
local NUM_DEFS = {
  {key = "airUnit",   def = 1},    -- 1 automatisch (ohne Einheit: Knoten), 2 km/h, 3 Knoten, 4 m/s
  {key = "airKind",   def = 1},    -- 1 Logger liefert IAS (Normdichte), 2 TAS
  {key = "altKind",   def = 1},    -- 1 Hoehe ue. M. (GPS), 2 Hoehe ueber Start + Platzhoehe
  {key = "fieldAlt",  def = 400},  -- Platzhoehe ue. M. (m)
  {key = "curFactor", def = 100},  -- Airspeed-Faktor, der jetzt im Logger steht (%)
  {key = "tekX",      def = 0},    -- elektronische TEK-Korrektur fuer die Quelle "TEK korr." (0.1 %)
  {key = "tekLag",    def = 0},    -- Verzoegerung der Korrektur (0.1 s), mit UEBERNEHMEN gesetzt
  {key = "tones",     def = 1},    -- Toene bei Kreis gezaehlt / verworfen
  {key = "circles",   def = 3},    -- so viele Kreise braucht der Fahrt-Faktor
  {key = "manSec",    def = 30},   -- so viele Sekunden Fahrtwechsel braucht die TEK-Messung
  {key = "airK",      def = 1000}, -- Fahrt-Korrektur fuer die Quelle "Fahrt korr." (0.1 %, auf die Logger-Fahrt)
}

local function clamp(x, lo, hi)
  if x < lo then return lo end
  if x > hi then return hi end
  return x
end

local function lpf(prev, x, dt, tau)
  if prev == nil then return x end
  if tau <= 0.05 then return x end
  return prev + (1 - math.exp(-dt / tau)) * (x - prev)
end

local function validSource(source)
  if source and type(source) ~= "boolean" then return source end
  return nil
end

local function readNumber(source)
  if not validSource(source) then return nil end
  local ok, v = pcall(source.value, source)
  if ok and type(v) == "number" then return v end
  return nil
end

local function unitIs(src, name, txt)
  local ok, u = pcall(src.unit, src)
  if not ok then return false end
  local c = rawget(_G, name)
  return (c ~= nil and u == c) or u == txt
end

-- Geschwindigkeit eines Sensors in m/s. unitSel: 1 automatisch (fallback), 2 km/h, 3 kn, 4 m/s
local function speedMs(src, unitSel, fallback)
  local v = readNumber(src)
  if v == nil then return nil end
  local u = unitSel or 1
  if u == 1 then
    if unitIs(src, "UNIT_METER_PER_SECOND", "m/s") then u = 4
    elseif unitIs(src, "UNIT_KMH", "km/h") or unitIs(src, "UNIT_KILOMETER_PER_HOUR", "km/h") then u = 2
    elseif unitIs(src, "UNIT_KNOT", "kn") or unitIs(src, "UNIT_KNOTS", "kn") then u = 3
    elseif unitIs(src, "UNIT_MPH", "mph") then u = 5
    else u = fallback end
  end
  if u == 2 then return v / KMH end
  if u == 3 then return v * 0.514444 end
  if u == 5 then return v * 0.44704 end
  return v
end

local function extractLatLon(v)
  if type(v) ~= "table" then return nil end
  local lat = v.lat or v.latitude or v.Lat or v.Latitude or v[1]
  local lon = v.lon or v.lng or v.long or v.longitude or v.Lon or v.Longitude or v[2]
  if type(lat) == "number" and type(lon) == "number" then return lat, lon end
  return nil
end

local function readPosition(src)
  if not validSource(src) then return nil end
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
  return nil
end

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

-- vorzeichenbehaftete Kursaenderung a -> b in Grad (-180..180)
local function angDiff(a, b) return ((b - a + 540) % 360) - 180 end

local function playToneSafe(freq, len)
  if not (system and system.playTone) then return end
  pcall(system.playTone, math.floor(freq + 0.5), math.floor(len + 0.5), 0)
end

local function currentModelName()
  if model and model.name then
    local ok, name = pcall(model.name)
    if ok and type(name) == "string" then return name end
  end
  return nil
end

-- ---------------------------------------------------------------- Zustand (einmal pro Skript)
-- Widget und Quellen ("TEK korr.", "Fahrt korr.") teilen sich Messung und Einstellungen. Die Einstellungen
-- kommen vom Widget; liegt es nicht auf einem Bildschirm des aktiven Modells, aus korrektur_<Modell>.txt.

local CFG = nil          -- Einstellungen des (letzten) Widgets, CFG.model = sein Modell
local S = nil            -- Messzustand
local FC = nil           -- Einstellungen aus der Datei (ohne Widget), {model, t, cfg}

local CIRCLE_DT = 0.3    -- Abstand der Kreis-Stuetzwerte (s)
local CIRCLE_MAX = 200   -- hoechstens so viele Stuetzwerte pro Kreis (60 s)
local GRID = 0.1         -- Raster der TEK-Messung (s)
local LAG_MIN, LAG_MAX = -5, 10   -- Verzoegerungen in Rasterschritten
local RING = 16

-- ---------------------------------------------------------------- Korrekturen pro Modell
-- korrektur_<Modell>.txt neben diesem Skript, Zeilen key=wert: Korrekturen (tekX, tekLag, airK), Einstellungen der
-- Fahrt und die Namen der Sensoren (air, vario, alt). Damit rechnen die Quellen auch ohne das Widget auf dem
-- Bildschirm. UEBERNEHMEN kann den Widget-Speicher nicht schreiben (das macht Ethos nur beim Schliessen der
-- Konfiguration), deshalb gilt die Datei: read() liest sie, UEBERNEHMEN und write() schreiben sie.
local KORR_NUM = {"tekX", "tekLag", "airK", "airUnit", "airKind", "altKind", "fieldAlt"}
local KORR_SRC = {"air", "vario", "alt"}

local function korrName(mname)
  return "korrektur_" .. tostring(mname or currentModelName() or "Modell"):gsub("[^%w%-_]", "_") .. ".txt"
end

local function srcName(src)
  if not validSource(src) then return "" end
  local ok, n = pcall(src.name, src)
  if ok and type(n) == "string" then return (n:gsub("[\r\n]", " ")) end
  return ""
end

local function readKorr(mname)
  if not (io and io.open) then return nil end
  local ok, f = pcall(io.open, korrName(mname), "r")
  if not (ok and f) then return nil end
  local okR, text = pcall(f.read, f, "*a")
  pcall(f.close, f)
  text = okR and tostring(text or "") or ""
  local r = {}
  local a, b = text:match("^(%-?%d+);(%d+)")                  -- Format 1.1: tekX;tekLag
  if a then r.tekX, r.tekLag = a, b end
  for k, v in text:gmatch("([%w]+)=([^\r\n]*)") do r[k] = v end
  return r
end

local function saveKorr(w)
  if not (io and io.open) then return end
  local t = {}
  for _, k in ipairs(KORR_NUM) do t[#t + 1] = string.format("%s=%d", k, math.floor((w[k] or 0) + 0.5)) end
  local old = readKorr()
  for _, k in ipairs(KORR_SRC) do
    local n = srcName(w[k])
    if n == "" and old and old[k] then n = old[k] end      -- Sensor (noch) nicht gewaehlt: alten Namen behalten
    t[#t + 1] = k .. "=" .. n
  end
  local ok, f = pcall(io.open, korrName(), "w")
  if ok and f then
    pcall(f.write, f, table.concat(t, "\n") .. "\n")
    pcall(f.close, f)
  end
  FC = nil
end

local function applyKorrNums(w, r)
  if not r then return end
  local v = tonumber(r.tekX); if v then w.tekX = clamp(v, -500, 500) end
  v = tonumber(r.tekLag); if v then w.tekLag = clamp(v, 0, RING - 1) end
  v = tonumber(r.airK); if v then w.airK = clamp(v, 500, 2000) end
end

local function loadKorr(w) applyKorrNums(w, readKorr()) end

-- Einstellungen aus der Datei, wenn kein Widget des aktiven Modells laeuft (alle 10 s neu gelesen)
local function fileCfg(now, mname)
  if FC and FC.model == mname and now - FC.t < 10 then return FC.cfg end
  local r = readKorr(mname)
  local cfg = nil
  if r then
    cfg = {tones = 2, model = mname, fromFile = true}
    for _, d in ipairs(NUM_DEFS) do cfg[d.key] = d.def end
    for _, k in ipairs(KORR_NUM) do local v = tonumber(r[k]); if v then cfg[k] = v end end
    applyKorrNums(cfg, r)
    for _, k in ipairs(KORR_SRC) do
      local n = r[k]
      if n and n ~= "" and system and system.getSource then
        local ok, src = pcall(system.getSource, n)
        if ok then cfg[k] = validSource(src) end
      end
    end
  end
  FC = {model = mname, t = now, cfg = cfg}
  return cfg
end

local function newState()
  local s = {
    t0 = nil, lastT = nil, flyT = 0,
    seg = {}, segTurn = 0, segSign = 0, segT = nil, segCrs = nil, calmT = nil,
    circles = {}, rejects = 0, fit = nil, lastCircle = nil, lastReject = nil,
    straightT = 0, stLastT = nil, cls = {},
    gridAcc = 0, gridN = 0, okRun = 0,
    rDe = {}, rTe = {}, rDeS = {}, ri = 0,
    sxy = {}, sxx = {}, syy = {}, sn = 0, manSec = 0,
    model = nil, savedSig = nil, saveT = nil,
  }
  for L = LAG_MIN, LAG_MAX do s.sxy[L], s.sxx[L], s.syy[L] = 0, 0, 0 end
  return s
end

local function resetAll()
  S = newState()
end

-- ---------------------------------------------------------------- Kreis-Fit

-- Kasa-Kreis durch die Bodenvektoren: Startwert fuer den Wind.
local function kasa(seg)
  local n = #seg
  local mx, my = 0, 0
  for i = 1, n do local p = seg[i]; mx, my = mx + p[1], my + p[2] end
  mx, my = mx / n, my / n
  local Suu, Svv, Suv, Suuu, Svvv, Suvv, Svuu = 0, 0, 0, 0, 0, 0, 0
  for i = 1, n do
    local p = seg[i]
    local u, v = p[1] - mx, p[2] - my
    local uu, vv = u * u, v * v
    Suu, Svv, Suv = Suu + uu, Svv + vv, Suv + u * v
    Suuu, Svvv, Suvv, Svuu = Suuu + uu * u, Svvv + vv * v, Suvv + u * vv, Svuu + v * uu
  end
  local det = Suu * Svv - Suv * Suv
  local sc = Suu + Svv
  if sc <= 0 or math.abs(det) < 1e-6 * sc * sc then return nil end
  local b1, b2 = 0.5 * (Suuu + Suvv), 0.5 * (Svvv + Svuu)
  return mx + (b1 * Svv - b2 * Suv) / det, my + (b2 * Suu - b1 * Suv) / det
end

-- Ein Gauss-Newton-Schritt fuer (Wx, Wy, k): Rest r_i = |g_i - W| - k * a_i
local function gnStep(seg, f)
  local Wx, Wy, k = f.wx, f.wy, f.k
  local a11, a12, a13, a22, a23, a33, b1, b2, b3 = 0, 0, 0, 0, 0, 0, 0, 0, 0
  for i = 1, #seg do
    local p = seg[i]
    local dx, dy = p[1] - Wx, p[2] - Wy
    local d = math.sqrt(dx * dx + dy * dy)
    if d > 0.1 then
      local j1, j2, j3 = -dx / d, -dy / d, -p[3]
      local r = d - k * p[3]
      a11, a12, a13 = a11 + j1 * j1, a12 + j1 * j2, a13 + j1 * j3
      a22, a23, a33 = a22 + j2 * j2, a23 + j2 * j3, a33 + j3 * j3
      b1, b2, b3 = b1 - j1 * r, b2 - j2 * r, b3 - j3 * r
    end
  end
  -- 3x3 mit Cramer
  local det = a11 * (a22 * a33 - a23 * a23) - a12 * (a12 * a33 - a23 * a13) + a13 * (a12 * a23 - a22 * a13)
  if math.abs(det) < 1e-9 then return false end
  local d1 = b1 * (a22 * a33 - a23 * a23) - a12 * (b2 * a33 - a23 * b3) + a13 * (b2 * a23 - a22 * b3)
  local d2 = a11 * (b2 * a33 - a23 * b3) - b1 * (a12 * a33 - a23 * a13) + a13 * (a12 * b3 - b2 * a13)
  local d3 = a11 * (a22 * b3 - b2 * a23) - a12 * (a12 * b3 - b2 * a13) + b1 * (a12 * a23 - a22 * a13)
  f.wx, f.wy, f.k = Wx + d1 / det, Wy + d2 / det, k + d3 / det
  return true
end

-- Fit eines vollen Kreises in Stuecken (pro wakeup ein Schritt, Ethos bricht lange Aufrufe ab).
local function fitStep(s, now)
  local f = s.fit
  local seg = f.seg
  if f.step == 0 then
    local wx, wy = kasa(seg)
    if not wx then s.fit = nil; s.rejects = s.rejects + 1; s.lastReject = {t = now, why = "Kreis entartet"}; return end
    local sda, saa, sa, sa2 = 0, 0, 0, 0
    for i = 1, #seg do
      local p = seg[i]
      local d = math.sqrt((p[1] - wx) ^ 2 + (p[2] - wy) ^ 2)
      sda, saa, sa, sa2 = sda + d * p[3], saa + p[3] * p[3], sa + p[3], sa2 + p[3] * p[3]
    end
    f.wx, f.wy, f.k = wx, wy, sda / saa
    local n = #seg
    f.am = sa / n
    f.asd = math.sqrt(math.max(sa2 / n - f.am * f.am, 0))
    f.step = 1
    return
  end
  if f.step <= 4 then
    if not gnStep(seg, f) then f.step = 9 else f.step = f.step + 1 end
    return
  end
  -- bewerten
  local res, n = 0, #seg
  for i = 1, n do
    local p = seg[i]
    local d = math.sqrt((p[1] - f.wx) ^ 2 + (p[2] - f.wy) ^ 2) - f.k * p[3]
    res = res + d * d
  end
  local rms = math.sqrt(res / n)
  local wind = math.sqrt(f.wx * f.wx + f.wy * f.wy)
  local tas = f.k * f.am
  local why
  if f.step == 9 then why = "Fit unloesbar"
  elseif f.asd > 0.10 * f.am then why = string.format("Fahrt schwankt %.0f %%", 100 * f.asd / f.am)
  elseif rms > 1.2 or rms > 0.08 * tas then why = string.format("Kreis unruhig (%.1f m/s)", rms)
  elseif wind > 0.8 * tas then why = "Wind zu stark"
  elseif f.k < 0.6 or f.k > 1.6 then why = string.format("Faktor %.2f unplausibel", f.k) end
  s.fit = nil
  if why then
    s.rejects = s.rejects + 1
    s.lastReject = {t = now, why = why}
    if s.tones == 1 then playToneSafe(400, 300) end
    return
  end
  local c = {k = f.k, wx = f.wx, wy = f.wy, wind = wind, from = (math.deg(atan2(f.wx, f.wy)) + 180) % 360,
    v = f.am * f.k, ias = f.ias, alt = f.alt, t = now, rms = rms}
  local cs = s.circles
  cs[#cs + 1] = c
  if #cs > 30 then table.remove(cs, 1) end
  s.lastCircle = c
  if s.tones == 1 then
    playToneSafe(1200, 80)
    playToneSafe(1600, 120)
  end
end

local function median(t)
  local a = {}
  for i = 1, #t do a[i] = t[i] end
  table.sort(a)
  local n = #a
  if n == 0 then return nil end
  if n % 2 == 1 then return a[(n + 1) // 2] end
  return 0.5 * (a[n // 2] + a[n // 2 + 1])
end

-- Ergebnis Fahrt: Median der Kreise, Streuung (halbe Spannweite der mittleren Haelfte)
local function airResult(s)
  local ks = {}
  for i, c in ipairs(s.circles) do ks[i] = c.k end
  local k = median(ks)
  if not k then return nil end
  table.sort(ks)
  local n = #ks
  local spread = 0
  if n >= 2 then
    local lo, hi = ks[math.max(1, math.floor(n * 0.25 + 0.5))], ks[math.min(n, math.ceil(n * 0.75))]
    spread = math.max((hi - lo) / 2, 0)
  end
  return k, spread, n
end

-- Fahrtbereich der Geraden: Mittel der Viertel; gilt, wenn zwei gegenueberliegende Viertel Werte haben
local function classResult(c)
  local sum, n, have = 0, 0, {}
  for q = 0, 3 do
    local e = c[q]
    if e and e.n >= 3 then sum, n, have[q] = sum + e.s / e.n, n + 1, true end
  end
  if n == 0 then return nil end
  local opp = (have[0] and have[2]) or (have[1] and have[3])
  return sum / n, opp, n
end

-- Ergebnis TEK: beste Verzoegerung, x, Bestimmtheit, ungefaehre Unsicherheit
local function tekResult(s)
  if s.sn < 50 then return nil end
  local best, bestV = nil, -1
  for L = LAG_MIN, LAG_MAX do
    local sxx = s.sxx[L]
    if sxx > 1e-6 then
      local v = s.sxy[L] ^ 2 / sxx
      if v > bestV then best, bestV = L, v end
    end
  end
  if not best then return nil end
  local sxx, sxy, syy = s.sxx[best], s.sxy[best], s.syy[best]
  local x = -sxy / sxx
  local r2 = (syy > 1e-9) and (sxy * sxy / (sxx * syy)) or 0
  local nEff = math.max(s.sn / 10, 3)                      -- Werte im 0.1-s-Raster sind ueber ~1 s verbunden
  local se = math.sqrt(math.max(syy - sxy * sxy / sxx, 0) / (nEff - 2) / (sxx / s.sn * nEff))
  return x, r2, best * GRID, se
end

-- ---------------------------------------------------------------- Messung

-- Ein Schritt der Messung. Laeuft aus dem Widget und aus der Quelle "TEK korr.", rechnet aber nur einmal
-- pro Zeitpunkt.
local function measure(now)
  local mname = currentModelName()
  local w = CFG
  if not w or w.model ~= mname then w = fileCfg(now, mname) end
  if not w then return end
  if not S then resetAll() end
  local s = S
  if s.lastT == now then return end
  s.tones = w.tones
  local dt = s.lastT and (now - s.lastT) or 0
  s.lastT = now
  if dt > 0.5 then dt = 0.5 end

  -- Modellwechsel: neu beginnen
  if s.model == nil then s.model = mname
  elseif mname ~= s.model then resetAll(); S.model = mname; return end

  -- Hoehe (fuer die Luftdichte und den Wind der Geraden)
  local alt = readNumber(w.alt)
  local msl = alt and ((w.altKind == 2) and (alt + w.fieldAlt) or alt) or w.fieldAlt
  s.msl = msl

  -- Fahrt
  local ias = speedMs(w.air, w.airUnit, 3)
  local tas = nil
  if ias then
    if w.airKind == 2 then tas = ias
    else
      local sigma = (1 - 2.2558e-5 * clamp(msl, -500, 6000)) ^ 4.2559
      tas = ias / math.sqrt(sigma)
    end
  end
  s.ias, s.tas = ias, tas
  s.airKorr = ias and ias * (w.airK or 1000) / 1000     -- Quelle "Fahrt korr." (IAS wie der Logger, in m/s)
  local vario = readNumber(w.vario)
  s.vario = vario
  local gs = speedMs(w.gpsSpd, 1, 2)                 -- GPS-Geschw.: ohne Einheit km/h
  s.gs = gs

  -- Kurs: GPS-Kurs-Quelle ab 3 m/s, sonst aus Positionen (ab 3 m Abstand)
  local crs = nil
  if validSource(w.gpsCrs) then
    local c = readNumber(w.gpsCrs)
    -- SM GPS-Logger 3: "Flugrichtung" (DIY 0522) in Grad; kommt sie roh in 0.1 Grad, ist sie bis 3600
    if c and c > 360.5 and c <= 3600.5 then c = c / 10 end
    if c and gs and gs >= 3 then crs = c % 360 end
  else
    local lat, lon = readPosition(w.gps)
    if lat and (lat ~= s.pLat or lon ~= s.pLon) then
      s.pLat, s.pLon = lat, lon
      if s.aLat == nil then s.aLat, s.aLon = lat, lon
      elseif distM(s.aLat, s.aLon, lat, lon) >= 3 then
        s.crsP = bearingDeg(s.aLat, s.aLon, lat, lon)
        s.aLat, s.aLon = lat, lon
      end
    end
    crs = s.crsP
  end
  -- Drehrate aus Kursaenderungen (Grad/s, geglaettet)
  if crs and s.crsPrev and dt > 0 then
    local r = angDiff(s.crsPrev, crs) / dt
    if math.abs(r) < 120 then s.rate = lpf(s.rate, r, dt, 1.0) end
  end
  s.crsPrev = crs
  s.crs = crs

  local flying = tas and gs and tas > 6 and gs > 3
  if flying then s.flyT = s.flyT + dt end
  s.flying = flying
  local rate = s.rate or 0

  -- ---- Kreise
  if s.fit then fitStep(s, now) end
  if flying and crs and math.abs(rate) > 6 then
    local sign = rate > 0 and 1 or -1
    if sign ~= s.segSign then s.seg, s.segTurn, s.segSign, s.segT, s.segCrs = {}, 0, sign, nil, crs end
    s.calmT = nil
    if s.segT == nil or now - s.segT >= CIRCLE_DT then
      s.segT = now
      s.segTurn = s.segTurn + math.abs(angDiff(s.segCrs or crs, crs))
      s.segCrs = crs
      local vz = vario or 0
      local ah = math.sqrt(math.max(tas * tas - vz * vz, 1))
      local c = math.rad(crs)
      local seg = s.seg
      seg[#seg + 1] = {gs * math.sin(c), gs * math.cos(c), ah, ias}
      if #seg > CIRCLE_MAX then s.seg, s.segTurn, s.segSign = {}, 0, 0 end
      if s.segTurn >= 360 and #s.seg >= 12 and not s.fit then
        local si, n = 0, #seg
        for i = 1, n do si = si + seg[i][4] end
        s.fit = {seg = seg, step = 0, ias = si / n, alt = msl}
        s.seg, s.segTurn, s.segCrs = {}, 0, crs           -- naechster Kreis beginnt hier
      end
    end
  else
    s.calmT = s.calmT or now
    if now - s.calmT > 2 then s.seg, s.segTurn, s.segSign = {}, 0, 0 end
  end

  -- ---- Geraden mit dem Wind des letzten Kreises
  if flying and crs and math.abs(rate) < 4 then s.straightT = s.straightT + dt else s.straightT = 0 end
  local lc = s.lastCircle
  if lc and s.straightT >= 3 and now - lc.t <= 180 and math.abs(msl - lc.alt) <= 150
    and (s.stLastT == nil or now - s.stLastT >= 1) then
    s.stLastT = now
    local c = math.rad(crs)
    local ax, ay = gs * math.sin(c) - lc.wx, gs * math.cos(c) - lc.wy
    local vz = vario or 0
    local ah = math.sqrt(math.max(tas * tas - vz * vz, 1))
    local k = math.sqrt(ax * ax + ay * ay) / ah
    if k > 0.6 and k < 1.6 then
      local ci = math.floor(ias * lc.k * KMH / 10)            -- Bereich nach wahrer IAS (10 km/h)
      local cl = s.cls[ci]
      if not cl then cl = {}; s.cls[ci] = cl end
      local q = math.floor(((crs + 45) % 360) / 90)          -- 0 N, 1 O, 2 S, 3 W
      local e = cl[q]
      if not e then e = {s = 0, n = 0}; cl[q] = e end
      e.s, e.n = e.s + k, e.n + 1
    end
  end

  -- ---- TEK im 0.1-s-Raster
  s.gridAcc = s.gridAcc + dt
  if s.gridAcc >= GRID then
    s.gridAcc = s.gridAcc - GRID
    if s.gridAcc > GRID then s.gridAcc = 0 end
    if tas and vario then
      local E = tas * tas / (2 * G)
      local Es0 = s.Es
      s.Es = lpf(s.Es, E, GRID, 0.25)
      local dE = Es0 and (s.Es - Es0) / GRID or 0
      s.dEs = lpf(s.dEs, dE, GRID, 0.25)
      s.TEs = lpf(s.TEs, vario, GRID, 0.25)
      s.dEl = lpf(s.dEl, s.dEs, GRID, 4)
      s.TEl = lpf(s.TEl, s.TEs, GRID, 4)
      local dEh, TEh = s.dEs - s.dEl, s.TEs - s.TEl
      s.ri = s.ri % RING + 1
      -- rDeS: dE/dt einmal geglaettet (Quelle "TEK korr."). Die Regression vergleicht lpf(TEK) mit lpf(dE), die
      -- gefundene Verzoegerung gilt also zwischen dem rohen TEK und diesem dE.
      s.rDe[s.ri], s.rTe[s.ri], s.rDeS[s.ri] = dEh, TEh, dE
      local ok = flying and math.abs(rate) < 15 and s.flyT > 20
      s.okRun = ok and s.okRun + 1 or 0
      if s.okRun > 40 then                                  -- 4 s ruhig: Hochpass eingeschwungen
        s.sn = s.sn + 1
        if math.abs(dEh) > 0.4 then s.manSec = s.manSec + GRID end
        local rDe, rTe, ri = s.rDe, s.rTe, s.ri
        for L = LAG_MIN, LAG_MAX do
          local x = rDe[(ri - (L > 0 and L or 0) - 1) % RING + 1]
          local y = rTe[(ri - (L < 0 and -L or 0) - 1) % RING + 1]
          s.sxy[L] = s.sxy[L] + x * y
          s.sxx[L] = s.sxx[L] + x * x
          s.syy[L] = s.syy[L] + y * y
        end
      end
    end
  end

  -- Korrigiertes TEK fuer die Quelle "TEK korr." (Korrektur um die eingestellte Verzoegerung versetzt)
  if vario then
    local lag = clamp(w.tekLag or 0, 0, RING - 1)
    local d = (s.ri > 0) and s.rDeS[(s.ri - lag - 1) % RING + 1] or nil
    s.teKorr = vario + (w.tekX or 0) / 1000 * (d or 0)
  else
    s.teKorr = nil
  end
end

-- ---------------------------------------------------------------- Ergebnisdatei

local function fileName()
  local n = tostring(S and S.model or "Modell"):gsub("[^%w%-_]", "_")
  return "ergebnis_" .. n .. ".txt"
end

local function resultText(w)
  local s = S
  local lines = {"TEK-Kalibrierung " .. tostring(s.model or "")}
  if os and os.date then lines[#lines + 1] = os.date("%Y-%m-%d %H:%M") end
  local k, sp, n = airResult(s)
  if k then
    lines[#lines + 1] = string.format("Fahrt: Korrektur %.1f %% +-%.1f aus %d Kreisen (Quelle Fahrt korr.: %.1f %%), oder Logger-Faktor %d %% -> %d %%",
      100 * k, 100 * sp, n, (w.airK or 1000) / 10, w.curFactor, math.floor(w.curFactor * k + 0.5))
  else
    lines[#lines + 1] = "Fahrt: noch kein Kreis"
  end
  for _, c in ipairs(s.circles) do
    lines[#lines + 1] = string.format("  Kreis: IAS %.0f km/h, k %.3f, Wind %.0f km/h aus %03d, Rest %.2f m/s, %.0f m",
      c.ias * KMH, c.k, c.wind * KMH, math.floor(c.from + 0.5) % 360, c.rms, c.alt)
  end
  local keys = {}
  for ci in pairs(s.cls) do keys[#keys + 1] = ci end
  table.sort(keys)
  for _, ci in ipairs(keys) do
    local kc, opp, nq = classResult(s.cls[ci])
    if kc then
      lines[#lines + 1] = string.format("  Gerade %d-%d km/h: k %.3f (%d Richtungen%s)", ci * 10, ci * 10 + 10, kc, nq,
        opp and "" or ", Gegenrichtung fehlt")
    end
  end
  local x, r2, lag, se = tekResult(s)
  if x then
    lines[#lines + 1] = string.format("TEK: Korrektur %+.1f %% +-%.1f (r2 %.2f, Verzoegerung %.1f s, %.0f s Fahrtwechsel)",
      100 * x, 100 * se, r2, lag, s.manSec)
  else
    lines[#lines + 1] = "TEK: noch keine Messung"
  end
  if k then lines[#lines + 1] = string.format("TEK aus Fahrt-Faktor erwartet: %+.1f %%", 100 * (k * k - 1)) end
  return table.concat(lines, "\n") .. "\n"
end

local function saveResult(w, now)
  local s = S
  local sig = string.format("%d/%d/%d", #s.circles, s.rejects, math.floor(s.sn / 600))
  if sig == s.savedSig or (s.saveT and now - s.saveT < 30) then return end
  if #s.circles == 0 and s.sn < 600 then return end
  s.savedSig, s.saveT = sig, now
  if not (io and io.open) then return end
  local ok, f = pcall(io.open, fileName(), "w")
  if ok and f then
    pcall(f.write, f, resultText(w))
    pcall(f.close, f)
  end
end

-- ---------------------------------------------------------------- Widget

local function create()
  local w = {lastInvalidate = 0, model = currentModelName()}
  for _, d in ipairs(NUM_DEFS) do w[d.key] = d.def end
  CFG = w
  return w
end

local function wakeup(w)
  w.model = currentModelName()
  CFG = w
  local now = os.clock()
  measure(now)
  saveResult(w, now)
  if now - w.lastInvalidate >= 0.3 then
    w.lastInvalidate = now
    lcd.invalidate()
  end
end

-- ---------------------------------------------------------------- Zeichnen

local function rgb(r, g, b) return lcd.RGB(r, g, b) end
local C = {}
local function colors()
  C.BG, C.DIM, C.BOX, C.TXT = rgb(0, 0, 0), rgb(150, 150, 150), rgb(40, 40, 40), rgb(255, 255, 255)
  C.GREEN, C.ORANGE, C.RED, C.BLUE = rgb(0, 200, 80), rgb(255, 170, 0), rgb(255, 60, 50), rgb(0, 170, 255)
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

local function button(x, y, bw, bh, label, col, txtCol)
  lcd.color(col)
  lcd.drawFilledRectangle(x, y, bw, bh)
  drawT(x, y + math.floor((bh - textHeight(FONT_XS)) / 2), bw, label, FONT_XS, txtCol or C.TXT, "c")
  return {x = x, y = y, w = bw, h = bh}
end

-- Was jetzt fliegen?
local function advice(w, s)
  if not s.tas then return "Fahrt-Quelle waehlen (Airspeed des Loggers)", C.ORANGE end
  if not s.vario then return "Vario-Quelle waehlen (TEK-Vario)", C.ORANGE end
  if not s.gs then return "GPS-Geschw. waehlen", C.ORANGE end
  if not s.crs and s.flying then return "GPS-Kurs (oder GPS-Position) waehlen", C.ORANGE end
  if not s.flying then return "Bereit - am Boden", C.DIM end
  local n = #s.circles
  if s.fit then return "Kreis wird gerechnet ...", C.BLUE end
  if n < w.circles then
    return string.format("Kreisen, Fahrt konstant halten (%d von %d Kreisen)", n, w.circles), C.TXT
  end
  if s.manSec < w.manSec then
    return string.format("Geradeaus: Fahrt wechseln, hochziehen/andruecken (%.0f von %d s)", s.manSec, w.manSec), C.TXT
  end
  return "Fertig. Gerade schnell/langsam in Gegenrichtungen prueft die Fahrt", C.GREEN
end

local function paint(w)
  local W, H = lcd.getWindowSize()
  local now = os.clock()
  colors()
  lcd.color(C.BG)
  lcd.drawFilledRectangle(0, 0, W, H)
  if not S then resetAll() end
  local s = S
  local m = 6
  local sH, xsH, lH = textHeight(FONT_S), textHeight(FONT_XS), textHeight(FONT_L)

  -- Kopf: Live-Werte
  local live = string.format("Fahrt %s  GPS %s  TAS %s km/h   TEK %s  korr %s m/s",
    s.ias and string.format("%.0f", s.ias * KMH) or "--",
    s.gs and string.format("%.0f", s.gs * KMH) or "--",
    s.tas and string.format("%.0f", s.tas * KMH) or "--",
    s.vario and string.format("%+.1f", s.vario) or "--",
    s.teKorr and string.format("%+.1f", s.teKorr) or "--")
  drawT(m, 3, W - 2 * m - 130, live, FONT_S, C.TXT, "l")
  -- Knopf NEU (zweimal tippen)
  local ask = w.resetAsk and now - w.resetAsk < 4
  w.btnReset = button(W - m - 60, 2, 60, sH + 4, ask and "SICHER?" or "NEU", ask and C.ORANGE or C.BOX, ask and C.BG)
  local top = sH + 10

  -- zwei Spalten
  local colW = math.floor((W - 3 * m) / 2)
  local xL, xR = m, 2 * m + colW
  local foot = H - xsH - 6
  local y

  -- ---- links: Fahrt
  y = top
  drawT(xL, y, colW, "FAHRT-KORREKTUR", FONT_XS, C.DIM, "l")
  y = y + xsH + 2
  local k, sp, n = airResult(s)
  w.btnTakeAir = nil
  if k then
    local col = (n >= w.circles and sp < 0.02) and C.GREEN or C.ORANGE
    if k < 0.8 or k > 1.25 then col = C.RED end
    local _, tw = drawT(xL, y, colW, string.format("%.1f %%", 100 * k), FONT_XL, col, "l")
    drawT(xL + tw + 8, y + 2, colW - tw - 8, string.format("+-%.1f %%, %d Kreise", 100 * sp, n), FONT_XS, C.TXT, "l")
    drawT(xL + tw + 8, y + 2 + xsH, colW - tw - 8, string.format("oder im Logger %d %%", math.floor(w.curFactor * k + 0.5)),
      FONT_XS, C.DIM, "l")
    y = y + textHeight(FONT_XL) + 2
    local aAsk = w.takeAirAsk and now - w.takeAirAsk < 4
    w.btnTakeAir = button(xL, y, math.min(colW, 200), sH + 6, aAsk and "SICHER?" or "UEBERNEHMEN",
      aAsk and C.ORANGE or C.BOX, aAsk and C.BG)
    y = y + sH + 10
  else
    drawT(xL, y, colW, "--", FONT_XL, C.DIM, "l")
    y = y + textHeight(FONT_XL) + 2
  end
  drawT(xL, y, colW, string.format("Quelle Fahrt korr.: %.1f %%", (w.airK or 1000) / 10), FONT_XS, C.DIM, "l")
  y = y + xsH
  -- letzte Kreise
  local cs = s.circles
  for i = #cs, math.max(1, #cs - 2), -1 do
    local c = cs[i]
    drawT(xL, y, colW, string.format("%.0f km/h  k %.3f  Wind %.0f/%03d", c.ias * KMH, c.k, c.wind * KMH,
      math.floor(c.from + 0.5) % 360), FONT_XS, C.TXT, "l")
    y = y + xsH
  end
  if s.lastReject and now - s.lastReject.t < 15 then
    drawT(xL, y, colW, "verworfen: " .. s.lastReject.why, FONT_XS, C.ORANGE, "l")
    y = y + xsH
  end
  -- Geraden nach Fahrtbereich
  local keys = {}
  for ci in pairs(s.cls) do keys[#keys + 1] = ci end
  table.sort(keys)
  for _, ci in ipairs(keys) do
    if y + xsH > foot then break end
    local kc, opp, nq = classResult(s.cls[ci])
    if kc then
      drawT(xL, y, colW, string.format("Gerade %d-%d: k %.3f %s", ci * 10, ci * 10 + 10, kc,
        opp and "" or "(Gegenrichtung fehlt)"), FONT_XS, opp and C.TXT or C.DIM, "l")
      y = y + xsH
    end
  end

  -- ---- rechts: TEK
  y = top
  drawT(xR, y, colW, "TEK-KORREKTUR", FONT_XS, C.DIM, "l")
  y = y + xsH + 2
  local x, r2, lag, se = tekResult(s)
  if x and s.manSec >= 5 then
    local col = (s.manSec >= w.manSec and se < 0.04) and C.GREEN or C.ORANGE
    local _, tw = drawT(xR, y, colW, string.format("%+.0f %%", 100 * x), FONT_XL, col, "l")
    drawT(xR + tw + 8, y + 2, colW - tw - 8, string.format("+-%.0f %%, r2 %.2f", 100 * se, r2), FONT_XS, C.TXT, "l")
    drawT(xR + tw + 8, y + 2 + xsH, colW - tw - 8, string.format("Verz. %.1f s, %.0f s Wechsel", lag, s.manSec), FONT_XS, C.TXT, "l")
    y = y + textHeight(FONT_XL) + 2
    local txt = x > 0.02 and "Duese kompensiert zu wenig" or x < -0.02 and "Duese kompensiert zu stark" or "Duese passt"
    drawT(xR, y, colW, txt, FONT_XS, C.TXT, "l")
    y = y + xsH
  else
    drawT(xR, y, colW, "--", FONT_XL, C.DIM, "l")
    y = y + textHeight(FONT_XL) + 2
    drawT(xR, y, colW, string.format("Fahrtwechsel %.0f s", s.manSec), FONT_XS, C.DIM, "l")
    y = y + xsH
  end
  if k then
    drawT(xR, y, colW, string.format("aus Airspeed-Faktor erwartet %+.0f %%", 100 * (k * k - 1)), FONT_XS, C.DIM, "l")
    y = y + xsH
  end
  drawT(xR, y, colW, string.format("Quelle TEK korr.: %+.1f %%, %.1f s", (w.tekX or 0) / 10, (w.tekLag or 0) * GRID),
    FONT_XS, C.DIM, "l")
  y = y + xsH + 4
  local tAsk = w.takeAsk and now - w.takeAsk < 4
  w.btnTake = nil
  if x and s.manSec >= 5 then
    w.btnTake = button(xR, y, math.min(colW, 200), sH + 6, tAsk and "SICHER?" or "UEBERNEHMEN",
      tAsk and C.ORANGE or C.BOX, tAsk and C.BG)
  end

  -- Fuss: was fliegen
  local adv, acol = advice(w, s)
  drawT(m, foot, W - 2 * m, adv, FONT_XS, acol, "l")
end

-- ---------------------------------------------------------------- Touch

local function inBtn(b, x, y) return b and x >= b.x and x <= b.x + b.w and y >= b.y and y <= b.y + b.h end

local function event(w, category, value, x, y)
  if EVT_TOUCH and category == EVT_TOUCH and (TOUCH_START == nil or value == TOUCH_START) then
    local now = os.clock()
    if inBtn(w.btnReset, x, y) then
      if w.resetAsk and now - w.resetAsk < 4 then w.resetAsk = nil; local mn = S and S.model; resetAll(); S.model = mn
      else w.resetAsk = now end
      lcd.invalidate()
      return true
    end
    if inBtn(w.btnTakeAir, x, y) then
      if w.takeAirAsk and now - w.takeAirAsk < 4 then
        w.takeAirAsk = nil
        local k = airResult(S)
        if k then
          w.airK = clamp(math.floor(1000 * k + 0.5), 500, 2000)
          saveKorr(w)
        end
      else w.takeAirAsk = now end
      lcd.invalidate()
      return true
    end
    if inBtn(w.btnTake, x, y) then
      if w.takeAsk and now - w.takeAsk < 4 then
        w.takeAsk = nil
        local xv, _, lag = tekResult(S)
        if xv then
          -- neue Korrektur = bisherige + gemessener Rest (gemessen wurde am Logger-TEK, nicht an TEK korr.)
          w.tekX = clamp(math.floor(1000 * xv + 0.5), -500, 500)
          w.tekLag = clamp(math.floor(lag / GRID + 0.5), 0, RING - 1)
          saveKorr(w)
        end
      else w.takeAsk = now end
      lcd.invalidate()
      return true
    end
  end
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

local function addChoice(w, label, prop, list)
  local line = form.addLine(label)
  return form.addChoiceField(line, nil, list, function() return w[prop] end, function(v) w[prop] = v end)
end

local function configure(w)
  addSource(w, "Fahrt (Airspeed des Loggers, nicht Fahrt korr.)", "air")
  addChoice(w, "Fahrt: Einheit", "airUnit", {{"Automatisch (ohne: Knoten)", 1}, {"km/h", 2}, {"Knoten", 3}, {"m/s", 4}})
  addChoice(w, "Fahrt: Logger liefert", "airKind", {{"IAS (Normdichte)", 1}, {"TAS", 2}})
  addNumber(w, "Airspeed-Faktor im Logger (%)", "curFactor", 50, 200)
  addSource(w, "TEK-Vario des Loggers (nicht TEK korr.)", "vario")
  addSource(w, "GPS-Geschwindigkeit", "gpsSpd")
  addSource(w, "Flugrichtung / GPS-Kurs (empfohlen)", "gpsCrs")
  addSource(w, "GPS-Position (ohne GPS-Kurs)", "gps")
  addSource(w, "Hoehe (Luftdichte)", "alt")
  addChoice(w, "Hoehe ist", "altKind", {{"ueber Meer (GPS)", 1}, {"ueber Start + Platzhoehe", 2}})
  addNumber(w, "Platzhoehe ue. M. (m)", "fieldAlt", -400, 5000)
  addNumber(w, "Quelle Fahrt korr.: Faktor (0.1 %)", "airK", 500, 2000)
  addNumber(w, "Quelle TEK korr.: Korrektur (0.1 %)", "tekX", -500, 500)
  addNumber(w, "Quelle TEK korr.: Verzoegerung (0.1 s)", "tekLag", 0, RING - 1)
  addChoice(w, "Toene bei Kreis", "tones", {{"Ein", 1}, {"Aus", 2}})
  addNumber(w, "Kreise fuer den Faktor", "circles", 1, 20)
  addNumber(w, "Sekunden Fahrtwechsel fuer TEK", "manSec", 5, 300)
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
  loadKorr(w)
  w.model = currentModelName()
  CFG = w
  saveKorr(w)                 -- Sensornamen fuer die Quellen ohne Widget
end

local function write(w)
  storage.write("ver", STORE_VER)
  for _, k in ipairs(SOURCE_KEYS) do storage.write(k, w[k]) end
  for _, d in ipairs(NUM_DEFS) do storage.write(d.key, w[d.key]) end
  saveKorr(w)
end

-- ---------------------------------------------------------------- Quelle "TEK korr."

local function srcInit(source)
  source:value(0)
  source:decimals(2)
  if UNIT_METER_PER_SECOND then pcall(source.unit, source, UNIT_METER_PER_SECOND) end
end

local function srcWakeup(source)
  measure(os.clock())
  local v = S and S.teKorr
  source:value(v or 0)
end

local function airInit(source)
  source:value(0)
  source:decimals(1)
  if UNIT_METER_PER_SECOND then pcall(source.unit, source, UNIT_METER_PER_SECOND) end
end

local function airWakeup(source)
  measure(os.clock())
  local v = S and S.airKorr
  source:value(v or 0)
end

local function init()
  -- Ethos akzeptiert Schluessel nur mit weniger als 8 Zeichen.
  system.registerWidget({
    key = "tekkal1",
    name = "TEK-Kalibrierung",
    create = create,
    paint = paint,
    wakeup = wakeup,
    event = event,
    configure = configure,
    read = read,
    write = write,
  })
  if system.registerSource then
    system.registerSource({key = "TEKkorr", name = "TEK korr.", init = srcInit, wakeup = srcWakeup})
    system.registerSource({key = "AIRkorr", name = "Fahrt korr.", init = airInit, wakeup = airWakeup})
  end
end

local M = {init = init}
M._test = {
  measure = measure, state = function() return S end, setCfg = function(w) CFG = w end, reset = resetAll,
  airResult = airResult, tekResult = tekResult, classResult = classResult, resultText = resultText,
  create = create, wakeup = wakeup, paint = paint, event = event, read = read, write = write,
}
return M
