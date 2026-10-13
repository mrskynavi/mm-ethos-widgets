-- @widget  Sollfahrt
-- @version 1.0
-- Sollfahrt (MacCready Speed-to-Fly) fuer Ethos -- Vollbild-Widget mit Tonausgabe.
--
-- Das Widget arbeitet nur zusammen mit dem MM Flight Analyzer: Die App schreibt die Polaren
-- auf den Sender (scripts/maccready/polaren.lua). Polarendaten werden hier NICHT eingegeben.
-- Fehlt die Polare fuer das aktive Modell, zeigt die Konfiguration nur diesen Status (erste
-- Zeile) und das Widget nur die Meldung "Polare fehlt".
--
-- Sensoren
--   GPS        Position -> Kurs (aus den Positionsaenderungen, ab 3 m Abstand)
--   GPS-Geschw. Grundgeschwindigkeit vom GPS-Empfaenger (empfohlen, wie im MM Flight Analyzer).
--              Ohne diesen Sensor rechnet das Widget die Geschwindigkeit aus den Positionen;
--              die sind im Log auf ~1 m gerundet und kommen etwa jede Sekunde, das rauscht stark.
--   GPS Hoehe  Hoehe in m, bestimmt die Windschicht
--   Vario      kompensiert (TE) oder rein barometrisch. Barometrisch: das Widget addiert die
--              Energiekompensation selbst:  TE = Vario + Faktor * (v/g) * dv/dt
--              (Faktor in %, Vario-Zeitkonstante gleicht die Traegheit des Varios an)
--
-- Wind (automatisch): Windtabelle nach Hoehe
--   Kreisfit (Kasa) ueber die GPS-Geschwindigkeitsvektoren der letzten ~28 s: Kreismittelpunkt =
--   Windvektor, Radius = Fluggeschwindigkeit. Mit GPS-Geschw. wie im MM Flight Analyzer: alle 0.7 s
--   ein Bodenvektor (Sensor-Geschwindigkeit in Richtung GPS-Kurs), Fit jede Sekunde, gleiche
--   Plausibilitaetsgrenzen. Vor dem ersten Fit: Wind 0, TAS = Grundgeschwindigkeit. Der erste Fit
--   braucht ein volles Fenster (40 Stuetzwerte), damit der Start keinen falschen Wind liefert.
--   Der erste gute Fit gilt als Startwert fuer ALLE Hoehen ("~" in der Anzeige). Jeder weitere Fit ergaenzt/ueberschreibt nur die 100-m-Schichten
--   seines Hoehenbereichs (gleitend, 30 % pro Fit). Die Fluggeschwindigkeit fuer Polare und Netto
--   ist |Grundgeschwindigkeit - Wind der aktuellen Hoehe| (linear zwischen den Schichten), auch
--   im Geradeausflug und sofort nach einem Hoehenwechsel. Die Hoehe kommt vom Sensor "GPS Hoehe"
--   (ohne Sensor: aus dem Vario aufintegriert). Der GPS-Kurs gilt als Richtung zum Ziel.
--
-- Rechnung
--   Polare:  Sinken w(v) = A*v^2 + B*v + C
--   Netto:   Luftmassenbewegung = TE + Sinken(TAS)   (positiv = Luft steigt), geglaettet
--   Sollfahrt:  v = h + sqrt( h^2 + (C + MC - Netto + B*h) / A )      (h = Gegenwind m/s)
--   Abweichung = geglaettete Ist-Fahrt - Sollfahrt (km/h), positiv = zu schnell.
--
-- Glaettung (gegen die Phygoide): Fahrt, Netto, Vario; Totzone; Ton-Verzoegerung.
-- Ton (system.playTone): wie beim Vario zeigt die Tonhoehe die Richtung, die Taktfolge die
-- Groesse der Abweichung. Modus: Schalter (an = Gleiten) oder Automatik.
-- MC: Poti/Schieber (hat Vorrang), MC+/MC- Schalter oder +/- auf dem Touchscreen. Eine Aenderung
-- wird nach 1.2 s Ruhe per Sprachausgabe angesagt (Fehler werden im Widget angezeigt).
--
-- Alle Zahlenfelder sind ganzzahlig: MC in 0.1 m/s, Zeiten in s bzw. 0.1 s.

local KMH = 3.6
local G = 9.81
local STORE_VER = 5   -- gpsSpd wird am Ende gelesen/geschrieben, deshalb unveraendert

-- ---------------------------------------------------------------- Einstellungen

local SOURCE_KEYS = {"gps", "gpsAlt", "vario", "mcSrc", "mcUp", "mcDn", "modeSw", "soundSw"}

-- Reihenfolge nur am Ende erweitern: Ethos-Storage arbeitet nach Aufrufreihenfolge.
local NUM_DEFS = {
  {key = "varioType",  def = 1},    -- 1 = kompensiert (TE), 2 = barometrisch
  {key = "teFactor",   def = 100},  -- barometrisch: Kompensation in %
  {key = "varioTau",   def = 10},   -- barometrisch: Vario-Zeitkonstante in 0.1 s
  {key = "ballast",    def = 100},  -- Fluggewicht in % des Polaren-Gewichts
  {key = "bugs",       def = 0},    -- Verschmutzung: Zuschlag aufs Sinken in %
  {key = "mc",         def = 10},   -- aktueller MC in 0.1 m/s
  {key = "mcMax",      def = 30},   -- Poti-Bereich 0..mcMax in 0.1 m/s
  {key = "mcStep",     def = 5},    -- Schrittweite Schalter/Touch in 0.1 m/s
  {key = "smoothSpd",  def = 8},    -- Fahrt-Glaettung s
  {key = "smoothNet",  def = 8},    -- Netto-Glaettung s
  {key = "smoothVar",  def = 2},    -- Vario-Glaettung s
  {key = "dead",       def = 3},    -- Totzone +-km/h
  {key = "delay",      def = 20},   -- Ton-Verzoegerung in 0.1 s
  {key = "climbThr",   def = 5},    -- Steigen-Schwelle Automatik in 0.1 m/s
  {key = "minSpeed",   def = 15},   -- darunter keine Auswertung (km/h)
  {key = "announceMode", def = 2}, -- 1 = Sprache, 2 = Sprache + Quittungston, 3 = nur Quittungston
  {key = "toneLogic",  def = 2},    -- 1 = schneller fliegen = hoeher, 2 = bremsen = hoeher
}
local BOOL_DEFS = {
  {key = "demo",     def = false},
  {key = "tone",     def = true},
  {key = "announce", def = true},   -- MC per Sprachausgabe ansagen
}

-- Moegliche Orte der Polaren-Datei (erster Treffer gewinnt).
local POLAR_PATHS = {
  "/scripts/maccready/polaren.lua",
  "scripts/maccready/polaren.lua",
  "../maccready/polaren.lua",
  "polaren.lua",
}

-- ---------------------------------------------------------------- Rechenkern

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

-- Polare aus Koeffizienten: Sinken (m/s, positiv) = A*v^2 + B*v + C, v in m/s.
-- vMin/vMax = gueltiger Fahrtbereich (m/s); die Sollfahrt wird darauf begrenzt.
local function makePolar(A, B, C, vMin, vMax)
  if type(A) ~= "number" or type(B) ~= "number" or type(C) ~= "number" then return nil end
  if A <= 1e-9 or C <= 0 then return nil end
  vMin = (type(vMin) == "number" and vMin > 0) and vMin or 1
  vMax = (type(vMax) == "number" and vMax > vMin) and vMax or (vMin * 3)
  local vms = -B / (2 * A)                       -- Fahrt des geringsten Sinkens
  local lo = math.min(math.max(vms, vMin), vMax)  -- niedrigste Sollfahrt
  return {A = A, B = B, C = C, vms = (vms > 0) and vms or vMin, lo = lo, vmin = vMin, vmax = vMax}
end

-- Quadratische Polare durch 3 Punkte (v in m/s, w = Sinken positiv in m/s).
local function buildPolar(vs, ws, k, bugs)
  local s = math.sqrt(math.max(k, 0.1))
  local v1, v2, v3 = vs[1] * s, vs[2] * s, vs[3] * s
  local w1, w2, w3 = ws[1] * s * (1 + bugs), ws[2] * s * (1 + bugs), ws[3] * s * (1 + bugs)
  if v1 >= v2 or v2 >= v3 then return nil end
  local f12 = (w2 - w1) / (v2 - v1)
  local f23 = (w3 - w2) / (v3 - v2)
  local A = (f23 - f12) / (v3 - v1)
  if A <= 1e-6 then return nil end
  local B = f12 - A * (v1 + v2)
  local C = w1 - f12 * v1 + A * v1 * v2
  return makePolar(A, B, C, v1, v3)
end

-- Polare aus a,b,c der App (Fluggewicht wie dort: s = sqrt(m_neu/m_alt) -> a/s, b, c*s).
local function polarFromAbc(a, b, c, vMin, vMax, mass, bugs)
  -- Die App kann die Polare auch als Vertikalgeschwindigkeit schreiben (Sinken = negativ,
  -- also c < 0): dann Vorzeichen umdrehen.
  if type(c) == "number" and c < 0 and type(a) == "number" and type(b) == "number" then
    a, b, c = -a, -b, -c
  end
  local s = math.sqrt(math.max(mass, 0.1))
  local f = 1 + bugs
  return makePolar(a / s * f, b * f, c * s * f, (vMin or 0) * s, (vMax or 0) * s)
end

local function polarSink(p, v)
  return p.A * v * v + p.B * v + p.C
end

-- Sollfahrt (m/s) fuer MC (m/s), Netto-Luftbewegung (m/s, positiv = steigend) und
-- Gegenwindkomponente h (m/s, positiv = Gegenwind, negativ = Rueckenwind; Standard 0).
--   v = h + sqrt( h^2 + (c + MC - netto + b*h) / a )      (h = 0: sqrt((c + MC - netto) / a))
-- Zweiter Rueckgabewert: true, wenn auf vMax begrenzt wurde.
local function speedToFly(p, mc, netto, headwind)
  local h = headwind or 0
  local x = p.C + mc - netto + p.B * h          -- darf bei starkem Gegenwind negativ sein
  local rad = h * h + x / p.A
  local v = h + math.sqrt(math.max(rad, 0))
  return clamp(v, p.lo, p.vmax), v > p.vmax + 1e-9
end

-- Liest die Polaren-Datei der App. Liefert Tabelle {Modellname = {a,b,c,vMin,vMax}} oder nil.
local polarPaths = POLAR_PATHS
local function loadPolarFile()
  for _, path in ipairs(polarPaths) do
    local chunk
    if loadfile then
      local ok, c = pcall(loadfile, path, "t", {})
      if ok and type(c) == "function" then chunk = c end
    end
    if not chunk and io and io.open and load then
      local okOpen, f = pcall(io.open, path, "r")
      if okOpen and f then
        local okRead, text = pcall(f.read, f, "*a")
        pcall(f.close, f)
        if okRead and type(text) == "string" then
          local okLoad, c = pcall(load, text, "polaren", "t", {})
          if okLoad and type(c) == "function" then chunk = c end
        end
      end
    end
    if chunk then
      local ok, t = pcall(chunk)
      if ok and type(t) == "table" then return t, path end
    end
  end
  return nil
end

-- Eintrag fuer ein Modell: exakter Name, sonst ohne Gross-/Kleinschreibung und Leerraum.
local function findPolarEntry(all, name)
  if type(all) ~= "table" or type(name) ~= "string" then return nil end
  if type(all[name]) == "table" then return all[name], name end
  local function norm(s) return (s:lower():gsub("^%s+", ""):gsub("%s+$", "")) end
  local wanted = norm(name)
  for key, entry in pairs(all) do
    if type(key) == "string" and type(entry) == "table" and norm(key) == wanted then return entry, key end
  end
  return nil
end

-- Ein Rechenschritt. teRaw in m/s, vRaw = Fluggeschwindigkeit in m/s (nil = keine Daten).
-- hRaw: Gegenwindkomponente in m/s (nil = kein Wind bekannt).
local function process(w, teRaw, vRaw, dt, now, hRaw)
  if not w.polar then w.valid = false; return end
  if teRaw == nil or vRaw == nil or vRaw * KMH < w.minSpeed then
    w.valid = false
    w.vSlow, w.vFast, w.teFast, w.teSlow, w.netto = nil, nil, nil, nil, nil
    w.vstf, w.dv, w.stfLimited = nil, nil, false
    w.tooFastSince, w.tooSlowSince = nil, nil
    return
  end
  w.valid = true

  local tauVar = w.smoothVar
  w.vFast  = lpf(w.vFast,  vRaw,  dt, tauVar)
  w.teFast = lpf(w.teFast, teRaw, dt, tauVar)
  w.vSlow  = lpf(w.vSlow,  vRaw,  dt, w.smoothSpd)
  w.teSlow = lpf(w.teSlow, teRaw, dt, 4)

  -- Netto: Steigen der Luftmasse = TE + Sinken der Polare bei der Fluggeschwindigkeit
  local nettoInst = w.teFast + polarSink(w.polar, w.vFast)
  w.netto = lpf(w.netto, nettoInst, dt, w.smoothNet)

  -- Gegenwindkomponente geglaettet; ohne Winddaten nach 3 s zurueck auf 0
  if hRaw ~= nil then
    w.headwind = lpf(w.headwind, hRaw, dt, w.smoothNet)
    w.windStale = nil
  else
    w.windStale = w.windStale or now
    if now - w.windStale > 3 then w.headwind = nil end
  end

  w.vstf, w.stfLimited = speedToFly(w.polar, w.mcValue, clamp(w.netto, -5, 5), w.headwind)
  w.dv = (w.vSlow - w.vstf) * KMH                       -- km/h, positiv = zu schnell

  -- Trend der geglaetteten Fahrt (km/h pro s), stark geglaettet
  if w.vSlowPrev ~= nil and dt > 0 then
    w.trend = lpf(w.trend, (w.vSlow - w.vSlowPrev) * KMH / dt, dt, 2)
  end
  w.vSlowPrev = w.vSlow

  -- Modus: Schalter oder Automatik (langsam und steigend = Kreisen)
  if w.modeFixed ~= nil then
    w.cruise = w.modeFixed
    w.climbSince, w.cruiseSince = nil, nil
  else
    local circling = w.teSlow > w.climbThr / 10 and w.vSlow < 1.3 * w.polar.vms
    if circling then
      w.cruiseSince = nil
      w.climbSince = w.climbSince or now
      if now - w.climbSince >= 3 then w.cruise = false end
    else
      w.climbSince = nil
      w.cruiseSince = w.cruiseSince or now
      if now - w.cruiseSince >= 3 then w.cruise = true end
    end
  end

  -- Ton-Bedingung: Abweichung ausserhalb Totzone, anhaltend
  if w.cruise and math.abs(w.dv) > w.dead then
    if w.dv > 0 then
      w.tooFastSince, w.tooSlowSince = w.tooFastSince or now, nil
    else
      w.tooSlowSince, w.tooFastSince = w.tooSlowSince or now, nil
    end
  else
    w.tooFastSince, w.tooSlowSince = nil, nil
  end
end

-- Liefert Frequenz (Hz), Laenge (ms) und Pause bis zum naechsten Piep (s), oder nil.
local function toneFor(w, now)
  local since = w.tooFastSince or w.tooSlowSince
  if not since or now - since < w.delay / 10 then return nil end
  if not w.dv or math.abs(w.dv) <= w.dead then return nil end
  -- Tonhoehe zeigt die Richtung, die Taktfolge die Groesse der Abweichung. Stille in der Totzone.
  --   Logik 1 (Standard): schneller fliegen noetig = hoeher, langsamer fliegen noetig = tiefer
  --   Logik 2 (Vario-Analogie "Knueppel"): bremsen/ziehen = hoeher, schneller/druecken = tiefer
  local excess = clamp(math.abs(w.dv) - w.dead, 0, 25)
  local needFaster = w.dv < 0
  local high = needFaster
  if w.toneLogic == 2 then high = not needFaster end
  if high then
    -- hoher Ton: Piepen, je groesser die Abweichung, desto schneller
    local period = math.max(0.14, 0.55 - excess * 0.016)
    local len = clamp(period * 550, 70, 140)
    return 850 + excess * 20, len, period
  end
  -- tiefer Ton: laengere, tiefere Piepser (Vario im Sinken); je groesser die Abweichung, desto tiefer
  local period = math.max(0.30, 0.50 - excess * 0.01)
  return math.max(220, 400 - excess * 8), clamp(period * 600, 180, 300), period
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

-- ---------------------------------------------------------------- Ethos-Anbindung

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

-- Warum ist der Ton gerade (nicht) zu hoeren? Kurztext fuer die Kopfzeile.
local function toneReason(w, now, soundOn)
  if w.toneErr then return "FEHLER", true end
  if not soundOn then return "AUS", false end
  if not w.valid then return "keine Daten", false end
  if not w.cruise then return "Kreisen", false end
  if not w.dv or math.abs(w.dv) <= w.dead then return "Totzone", false end
  local since = w.tooFastSince or w.tooSlowSince
  if not since or now - since < w.delay / 10 then return "Verzoegerung", false end
  return "spielt", false
end

-- MC ansagen. Speichert Erfolg oder Fehler in w.announceInfo / w.announceErr.
-- Faellt die Sprachausgabe aus, kommt ein Bestaetigungs-Doppelton.
local function sayMC(w, mc, now)
  local mode = w.announceMode or 2
  w.announceT = now
  w.announceCount = (w.announceCount or 0) + 1
  w.announceInfo = string.format("MC %.1f angesagt", mc)
  w.speakMC = mc
  -- Quittungston zuerst (Tonhoehe zeigt den MC), die Sprache folgt 0.6 s spaeter,
  -- damit der Ton nicht von der Sprachausgabe verschluckt wird.
  if mode >= 2 then playToneSafe(500 + clamp(mc, 0, 5) * 200, 150) end
  w.speakAt = (mode ~= 3) and (now + 0.6) or nil
  w.nextBeep = math.max(w.nextBeep or 0, now + 3.5)
end

-- Sprachausgabe des gemerkten MC-Werts (zeitversetzt nach sayMC).
local function speakPending(w, now)
  if not w.speakAt or now < w.speakAt then return end
  local mc = w.speakMC
  w.speakAt, w.speakMC = nil, nil
  if not widgetVisible() then return end   -- unsichtbar: keine Ansage
  local err
  local unit = UNIT_METER_PER_SECOND
  if not (system and system.playNumber) then
    err = "system.playNumber fehlt"
  elseif unit == nil then
    err = "UNIT_METER_PER_SECOND fehlt"
  else
    local ok, e = pcall(system.playNumber, mc, unit, 1)
    if not ok then err = tostring(e) end
  end
  if err then
    w.announceErr, w.announceInfo = err, nil
    playToneSafe(1000, 90)
    playToneSafe(1300, 90)
  else
    w.announceErr = nil
  end
  w.nextBeep = math.max(w.nextBeep or 0, now + 3.5)
end

local function currentModelName()
  if model and model.name then
    local ok, name = pcall(model.name)
    if ok and type(name) == "string" then return name end
  end
  return nil
end

-- Datei neu einlesen (bei Modellwechsel, erzwungen oder alle 30 s).
local function refreshPolarFile(w, now, force)
  local name = currentModelName()
  w.modelName = name
  if not force and name == w.fileModel and w.fileChecked and now - w.fileChecked < 30 then return end
  w.fileModel, w.fileChecked = name, now
  local all = loadPolarFile()
  w.fileFound = all ~= nil
  w.fileEntry, w.fileKey = nil, nil
  if all and name then
    w.fileEntry, w.fileKey = findPolarEntry(all, name)
  end
  local sig = "-"
  local e = w.fileEntry
  if e then sig = string.format("%s,%s,%s,%s,%s", tostring(e.a), tostring(e.b), tostring(e.c), tostring(e.vMin), tostring(e.vMax)) end
  w.fileSig = sig
end

-- Polare bauen: nur aus der Datei der App. Ohne gueltigen Eintrag gibt es keine Polare.
local function rebuildPolar(w)
  w.polar, w.polarNote = nil, nil
  local e = w.fileEntry
  if e then
    local p = polarFromAbc(e.a, e.b, e.c, e.vMin, e.vMax, w.ballast / 100, w.bugs / 100)
    if p then w.polar = p else w.polarNote = "Eintrag ungueltig" end
  elseif w.fileFound then
    w.polarNote = "kein Eintrag fuer dieses Modell"
  else
    w.polarNote = "keine Polaren-Datei auf dem Sender"
  end
end

-- Statuszeile fuer Konfiguration und Widget. Zweiter Wert: true = Polare vorhanden.
local function polarStatus(w)
  local name = w.modelName or "?"
  if w.polar then return "vorhanden: " .. tostring(w.fileKey or name), true end
  return "FEHLT fuer '" .. name .. "' (" .. tostring(w.polarNote or "?") .. ")", false
end

local function create()
  local w = {
    lastT = nil, lastInvalidate = 0, nextBeep = 0, lastMcChange = nil, mcAnnounced = nil,
    mcUpOn = false, mcDnOn = false, cruise = true, buttons = nil,
    polarSig = nil, win = {}, wt = {},
  }
  for _, d in ipairs(NUM_DEFS) do w[d.key] = d.def end
  for _, d in ipairs(BOOL_DEFS) do w[d.key] = d.def end
  w.mcValue = w.mc / 10
  return w
end

local function demoInputs(w, now)
  -- Simulierte Phygoide + wechselnde Luftmasse, damit Anzeige und Ton im Simulator testbar sind.
  local vTrue = (62 + 7 * math.sin(2 * math.pi * now / 7.0) + 8 * math.sin(2 * math.pi * now / 55)) / KMH
  local air = 1.3 * math.sin(2 * math.pi * now / 40)
  local sink = w.polar and polarSink(w.polar, vTrue) or 0.7
  return -sink + air, vTrue
end

-- Kinetischer Anteil (v/g)*dv/dt der TAS (m/s), ueber 1 s Basis (weniger Rauschen).
local function kineticStep(w, tas, dt, now)
  if tas == nil then w.tasF, w.kin, w.kT0 = nil, nil, nil; return end
  w.tasF = lpf(w.tasF, tas, dt, 1.0)
  if w.kT0 == nil then w.kT0, w.kTas0 = now, w.tasF; return end
  local span = now - w.kT0
  if span >= 1.0 then
    local raw = w.tasF / G * (w.tasF - w.kTas0) / span
    local tau = (w.varioType == 2) and (w.varioTau / 10) or 4
    w.kin = lpf(w.kin, raw, span, tau)
    w.kT0, w.kTas0 = now, w.tasF
  end
end

-- Alle Messwerte -> TE (m/s), TAS (m/s), Gegenwind (m/s) und ein Hinweistext, falls Daten fehlen.
local function inputsStep(w, now, dt)
  -- Hoehe: GPS-Hoehe, sonst aus dem Vario aufintegriert (relativ, nur Notbehelf)
  local vario = readNumber(w.vario)
  local alt = readNumber(w.gpsAlt)
  if alt == nil then
    w.altInt = (w.altInt or 0) + (vario or 0) * dt
    alt = w.altInt
  end
  w.alt = alt
  local lat, lon, perr = readPosition(w.gps)
  if validSource(w.gpsSpd) then
    sensorUpdate(w, lat, lon, speedMs(w.gpsSpd), now)   -- wie im Analyzer (empfohlen)
  else
    gpsUpdate(w, lat, lon, now)                         -- nur Positionen
  end
  local air = airData(w, now)
  local tas = air and air.tas or nil
  w.air = air
  kineticStep(w, tas, dt, now)
  local kin = w.kin or 0

  -- Vario / TE (barometrisch: Energiekompensation selbst addieren)
  local te
  if vario ~= nil then
    te = (w.varioType == 2) and (vario + w.teFactor / 100 * kin) or vario
  end

  local note
  if not validSource(w.gps) then note = "GPS-Quelle waehlen"
  elseif perr then note = perr
  elseif not w.gpsOk then note = "GPS: kein Fix / keine Bewegung"
  elseif te == nil then note = "Vario-Quelle waehlen"
  elseif tas ~= nil and tas * KMH < w.minSpeed then note = "Fahrt unter Mindestfahrt" end
  return te, tas, air and air.head or nil, note
end

local function wakeup(w)
  local now = os.clock()
  local dt = w.lastT and (now - w.lastT) or 0
  w.lastT = now
  if dt > 0.5 then dt = 0.5 end

  -- Polare nur neu bauen, wenn sich etwas geaendert hat
  refreshPolarFile(w, now, false)
  local sig = table.concat({w.ballast, w.bugs, tostring(w.fileSig), tostring(w.fileFound)}, ",")
  if sig ~= w.polarSig then
    w.polarSig = sig
    rebuildPolar(w)
  end
  if not w.polar then
    w.valid = false
    w.buttons = nil
    if now - w.lastInvalidate >= 0.5 then
      w.lastInvalidate = now
      lcd.invalidate()
    end
    return
  end

  -- MC: Poti hat Vorrang, sonst Schalter-Flanken
  local mcPot = readNumber(w.mcSrc)
  if mcPot ~= nil then
    local lo, hi = -1024, 1024
    local src = w.mcSrc
    if src.minimum and src.maximum then
      local okMin, mn = pcall(src.minimum, src)
      local okMax, mx = pcall(src.maximum, src)
      if okMin and okMax and type(mn) == "number" and type(mx) == "number" and mx > mn then
        lo, hi = mn, mx
      end
    end
    local frac = clamp((mcPot - lo) / (hi - lo), 0, 1)
    local newMc = math.floor(frac * w.mcMax + 0.5)
    newMc = math.floor(newMc / math.max(w.mcStep, 1) + 0.5) * math.max(w.mcStep, 1)
    if newMc ~= w.mc then w.mc = newMc; w.lastMcChange = now end
  else
    local up = (readNumber(w.mcUp) or 0) > 0
    local dn = (readNumber(w.mcDn) or 0) > 0
    if up and not w.mcUpOn then w.mc = clamp(w.mc + w.mcStep, 0, w.mcMax); w.lastMcChange = now end
    if dn and not w.mcDnOn then w.mc = clamp(w.mc - w.mcStep, 0, w.mcMax); w.lastMcChange = now end
    w.mcUpOn, w.mcDnOn = up, dn
  end
  w.mcValue = w.mc / 10

  -- MC ansagen, sobald er 1.2 s stabil ist (auch Touch/Drehgeber setzen lastMcChange)
  if w.mcAnnounced == nil then w.mcAnnounced = w.mc end
  if w.announce and w.lastMcChange and now - w.lastMcChange > 0.8 and w.mcAnnounced ~= w.mc then
    w.mcAnnounced = w.mc
    sayMC(w, w.mcValue, now)
  end
  speakPending(w, now)

  -- Modus-Schalter
  local sw = readNumber(w.modeSw)
  if validSource(w.modeSw) and sw ~= nil then w.modeFixed = sw > 0 else w.modeFixed = nil end

  -- Eingaben
  local te, tas, hRaw, note
  if w.demo then
    te, tas = demoInputs(w, now)
    hRaw = 4 * math.sin(2 * math.pi * now / 50)
    w.air = {gs = tas, course = 0, tas = tas, head = hRaw, wind = {spd = 14 / KMH, from = 270, age = 0}}
  else
    te, tas, hRaw, note = inputsStep(w, now, dt)
  end
  w.dataNote = note

  local wasCruise = w.cruise
  process(w, te, tas, dt, now, hRaw)
  if w.valid and wasCruise ~= w.cruise then
    playToneSafe(w.cruise and 700 or 500, 120)
  end

  -- Ton
  local soundOn = w.tone
  local ss = validSource(w.soundSw)
  if ss then soundOn = soundOn and (readNumber(ss) or 0) > 0 end
  local announcePending = w.announce and w.mcAnnounced ~= w.mc   -- Ansage steht noch aus: Ton still
  if soundOn and w.valid and now >= w.nextBeep and not announcePending then
    local f, len, gap = toneFor(w, now)
    if f then
      local ok, err = playToneSafe(f, len)
      w.toneErr = (not ok) and err or nil
      w.nextBeep = now + gap
    end
  end
  w.toneState = toneReason(w, now, soundOn)

  if now - w.lastInvalidate >= 0.1 then
    w.lastInvalidate = now
    lcd.invalidate()
  end
end

-- ---------------------------------------------------------------- Zeichnen

local function rgb(r, g, b) return lcd.RGB(r, g, b) end

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

local function paintNoPolar(w, W, H)
  local DIM, RED = rgb(150, 150, 150), rgb(255, 60, 50)
  local y = math.floor(H / 2) - 70
  local th = drawT(0, y, W, "POLARE FEHLT", FONT_XL, RED, "c")
  y = y + th + 8
  th = drawT(0, y, W, "Modell: " .. tostring(w.modelName or "?"), FONT_STD, WHITE, "c")
  y = y + th + 12
  th = drawT(0, y, W, "Polare im MM Flight Analyzer erstellen", FONT_S, DIM, "c")
  y = y + th + 2
  drawT(0, y, W, "und auf den Sender schreiben.", FONT_S, DIM, "c")
  if w.polarNote then
    drawT(0, H - textHeight(FONT_XS) - 4, W, tostring(w.polarNote), FONT_XS, DIM, "c")
  end
end

local function paint(w)
  local W, H = lcd.getWindowSize()
  local BG, DIM = rgb(0, 0, 0), rgb(150, 150, 150)
  local GREEN, ORANGE, RED = rgb(0, 200, 80), rgb(255, 170, 0), rgb(255, 60, 50)
  local BLUE = rgb(0, 170, 255)

  lcd.color(BG)
  lcd.drawFilledRectangle(0, 0, W, H)

  if not w.polar then
    w.buttons = nil
    paintNoPolar(w, W, H)
    return
  end

  local margin = 6
  local headH = textHeight(FONT_S) + 6
  local leftW = math.floor(W * 0.27)
  local rightW = math.floor(W * 0.31)
  local centerW = W - leftW - rightW

  -- Kopfzeile: Modus, Ton, Demo
  local modeText, modeColor = "GLEITEN", GREEN
  if w.valid and not w.cruise then modeText, modeColor = "KREISEN", BLUE end
  if not w.valid then modeText, modeColor = "KEINE DATEN", DIM end
  drawT(0, 3, W, modeText, FONT_S, modeColor, "c")
  local soundOn = w.tone
  local ss = validSource(w.soundSw)
  if ss then soundOn = soundOn and (readNumber(ss) or 0) > 0 end
  local stateText = w.toneState or (soundOn and "" or "AUS")
  local stateColor = DIM
  if stateText == "spielt" then stateColor = GREEN
  elseif stateText == "FEHLER" then stateColor = RED
  elseif stateText == "Verzoegerung" then stateColor = ORANGE end
  drawT(margin, 3, 140, "TON: " .. stateText, FONT_S, stateColor, "l")
  local errText = w.toneErr or (w.announceErr and ("ANSAGE: " .. w.announceErr))
  if errText then
    drawT(margin, H - textHeight(FONT_XS) - 2, W - margin, string.sub(errText, 1, 70), FONT_XS, RED, "l")
  end
  if w.demo then drawT(0, 3, W - margin, "DEMO", FONT_S, ORANGE, "r") end

  -- Linke Spalte: Netto + TE-Vario
  local y = headH + 4
  local th = drawT(margin, y, leftW - margin, "NETTO", FONT_S, DIM, "l")
  y = y + th
  local nettoStr, nettoColor = "--", DIM
  if w.valid and w.netto then
    nettoStr = string.format("%+.1f", w.netto)
    nettoColor = w.netto >= 0.1 and GREEN or (w.netto <= -0.1 and RED or WHITE)
  end
  th = drawT(margin, y, leftW - margin, nettoStr, FONT_XL, nettoColor, "l")
  y = y + th
  drawT(margin, y, leftW - margin, "m/s", FONT_S, DIM, "l")
  y = y + textHeight(FONT_S) + 10
  drawT(margin, y, leftW - margin, "VARIO TE", FONT_S, DIM, "l")
  y = y + textHeight(FONT_S)
  local teStr = (w.valid and w.teFast) and string.format("%+.1f", w.teFast) or "--"
  drawT(margin, y, leftW - margin, teStr, FONT_L, WHITE, "l")

  -- Status klein unten links: Polare, Glaettung, Wind, Gegenwind
  local xsH = textHeight(FONT_XS)
  local sy = H - margin - (xsH + 2) * 4 - 2
  local pname = tostring(w.fileKey or w.modelName or "?")
  drawT(margin, sy, leftW - margin, string.format("%s %d%%", string.sub(pname, 1, 10), w.ballast), FONT_XS, GREEN, "l")
  drawT(margin, sy + xsH + 2, leftW - margin, string.format("Glaett. %ds  Totz. +-%d", w.smoothSpd, w.dead), FONT_XS, DIM, "l")
  local wind = w.air and w.air.wind
  local windText, windColor = "Wind: kreisen = messen", DIM
  if wind then
    windText = string.format("%sWind %.0f km/h aus %03d", wind.meas and "" or "~", wind.spd * KMH, math.floor(wind.from + 0.5) % 360)
    windColor = (wind.meas and wind.age <= WIND_HOLD) and WHITE or ORANGE
  end
  drawT(margin, sy + (xsH + 2) * 2, leftW - margin, windText, FONT_XS, windColor, "l")
  local headText, headColor = "", DIM
  if w.valid and w.headwind then
    local kmh = w.headwind * KMH
    if math.abs(kmh) < 0.5 then
      headText = "Wind quer / 0"
    elseif kmh > 0 then
      headText, headColor = string.format("Gegenwind %.0f km/h", kmh), ORANGE
    else
      headText, headColor = string.format("Rueckenwind %.0f km/h", -kmh), GREEN
    end
  end
  drawT(margin, sy + (xsH + 2) * 3, leftW - margin, headText, FONT_XS, headColor, "l")

  -- Mitte: Balkeninstrument
  local barW = math.min(math.floor(centerW * 0.42), 70)
  local barX = leftW + math.floor((centerW - barW) / 2)
  local barTop = headH + 6
  local barBot = H - margin - 4
  local barH = barBot - barTop
  local midY = barTop + math.floor(barH / 2)
  local range = 15
  if w.valid and w.vstf then range = math.max(15, math.floor(w.vstf * KMH * 0.25 / 5 + 0.5) * 5) end
  local pxPerKmh = (barH / 2 - 14) / range

  lcd.color(rgb(40, 40, 40))
  lcd.drawFilledRectangle(barX, barTop, barW, barH)
  -- Totzone
  local dzPx = math.floor(w.dead * pxPerKmh)
  lcd.color(rgb(0, 70, 30))
  lcd.drawFilledRectangle(barX, midY - dzPx, barW, math.max(dzPx * 2, 1))
  -- Skalenstriche
  lcd.color(rgb(90, 90, 90))
  local tickStep = range >= 30 and 10 or 5
  local t = tickStep
  while t <= range do
    local off = math.floor(t * pxPerKmh)
    lcd.drawLine(barX, midY - off, barX + barW - 1, midY - off)
    lcd.drawLine(barX, midY + off, barX + barW - 1, midY + off)
    t = t + tickStep
  end
  -- Abweichungsbalken
  if w.valid and w.dv then
    local dvc = clamp(w.dv, -range, range)
    local px = math.floor(math.abs(dvc) * pxPerKmh)
    local excess = math.abs(w.dv) - w.dead
    local color = GREEN
    if excess > 0 then color = (excess > 8) and RED or ORANGE end
    lcd.color(color)
    if dvc >= 0 then
      lcd.drawFilledRectangle(barX + 4, midY - px, barW - 8, math.max(px, 1))
    else
      lcd.drawFilledRectangle(barX + 4, midY, barW - 8, math.max(px, 1))
    end
  end
  -- Mittellinie (Sollfahrt)
  lcd.color(WHITE)
  lcd.drawLine(barX - 6, midY, barX + barW + 5, midY)
  lcd.drawLine(barX - 6, midY + 1, barX + barW + 5, midY + 1)
  drawT(barX - 20, barTop + 1, barW + 40, "BREMSEN", FONT_XS, DIM, "c")
  drawT(barX - 20, barBot - textHeight(FONT_XS) - 1, barW + 40, "SCHNELLER", FONT_XS, DIM, "c")
  if w.valid and w.dv then
    drawT(barX + barW + 8, midY - textHeight(FONT_S) - 4, centerW, string.format("%+.0f", w.dv), FONT_S, WHITE, "l")
  end
  -- Trend-Dreieck links neben der Mittellinie (Fahrt nimmt zu / ab)
  if w.valid and w.trend and math.abs(w.trend) > 0.5 then
    local tx, ty, s = barX - 14, midY, 7
    lcd.color(BLUE)
    if w.trend > 0 then
      lcd.drawFilledTriangle(tx, ty - 14 - s, tx - s, ty - 14, tx + s, ty - 14)
    else
      lcd.drawFilledTriangle(tx, ty + 14 + s, tx - s, ty + 14, tx + s, ty + 14)
    end
  end

  -- Rechte Spalte: MC, Soll, Ist
  local rx = W - rightW + margin
  local rw = rightW - margin * 2
  y = headH + 4
  local potMode = validSource(w.mcSrc) ~= nil
  th = drawT(rx, y, rw, potMode and "MC (Poti)" or "MC", FONT_S, DIM, "c")
  y = y + th
  th = drawT(rx, y, rw, string.format("%.1f", w.mcValue), FONT_XL, WHITE, "c")
  y = y + th
  drawT(rx, y, rw, "m/s", FONT_S, DIM, "c")
  y = y + textHeight(FONT_S) + 4

  -- Touch-Buttons - / +
  local btnH = math.min(48, math.floor(H * 0.16))
  local btnW = math.floor((rw - 8) / 2)
  w.buttons = {
    minus = {x = rx, y = y, w = btnW, h = btnH},
    plus = {x = rx + btnW + 8, y = y, w = btnW, h = btnH},
  }
  local btnColor = potMode and rgb(35, 35, 35) or rgb(60, 60, 60)
  for name, b in pairs(w.buttons) do
    lcd.color(btnColor)
    lcd.drawFilledRectangle(b.x, b.y, b.w, b.h)
    local label = (name == "plus") and "+" or "-"
    local lh = textHeight(FONT_L)
    drawT(b.x, b.y + math.floor((b.h - lh) / 2), b.w, label, FONT_L, potMode and DIM or WHITE, "c")
  end
  y = y + btnH + 8

  local limited = w.valid and w.stfLimited
  drawT(rx, y, rw, limited and "SOLL (MAX)" or "SOLL km/h", FONT_S, limited and ORANGE or DIM, "c")
  y = y + textHeight(FONT_S)
  local sollStr = (w.valid and w.vstf) and string.format("%.0f", w.vstf * KMH) or "--"
  th = drawT(rx, y, rw, sollStr, FONT_L, limited and ORANGE or WHITE, "c")
  y = y + th + 2
  drawT(rx, y, rw, "IST km/h", FONT_S, DIM, "c")
  y = y + textHeight(FONT_S)
  local istStr, istColor = "--", DIM
  if w.valid and w.vSlow then
    istStr = string.format("%.0f", w.vSlow * KMH)
    local excess = math.abs(w.dv) - w.dead
    istColor = excess <= 0 and GREEN or ((excess > 8) and RED or ORANGE)
  end
  th = drawT(rx, y, rw, istStr, FONT_L, istColor, "c")
  y = y + th
  local annText = string.format("Ansagen: %d", w.announceCount or 0)
  if w.announce and w.mcAnnounced ~= nil and w.mcAnnounced ~= w.mc then annText = "Ansage folgt ..."
  elseif w.announceInfo and os.clock() - (w.announceT or 0) < 6 then annText = w.announceInfo end
  drawT(rx, y, rw, annText, FONT_XS, DIM, "c")

  -- Hinweis, warum keine Daten da sind (ganz zuletzt, ueber allem, auf dunklem Grund)
  if not w.valid and w.dataNote then
    local nh = textHeight(FONT_STD) + 10
    local ny = math.floor(H / 2) - math.floor(nh / 2)
    lcd.color(rgb(0, 0, 0))
    lcd.drawFilledRectangle(0, ny, W, nh)
    drawT(0, ny + 5, W, w.dataNote, FONT_STD, ORANGE, "c")
  end
end

-- ---------------------------------------------------------------- Touch / Tasten

local function inside(b, x, y)
  return b and x >= b.x and x <= b.x + b.w and y >= b.y and y <= b.y + b.h
end

local function changeMc(w, delta)
  if validSource(w.mcSrc) then return end
  w.mc = clamp(w.mc + delta, 0, w.mcMax)
  w.mcValue = w.mc / 10
  w.lastMcChange = os.clock()
end

local function event(w, category, value, x, y)
  if EVT_TOUCH and category == EVT_TOUCH and (TOUCH_START == nil or value == TOUCH_START) and w.buttons then
    if inside(w.buttons.plus, x, y) then changeMc(w, w.mcStep); lcd.invalidate(); return true end
    if inside(w.buttons.minus, x, y) then changeMc(w, -w.mcStep); lcd.invalidate(); return true end
  end
  if EVT_KEY and category == EVT_KEY and w.polar then
    if KEY_ROTARY_RIGHT and value == KEY_ROTARY_RIGHT then changeMc(w, w.mcStep); lcd.invalidate(); return true end
    if KEY_ROTARY_LEFT and value == KEY_ROTARY_LEFT then changeMc(w, -w.mcStep); lcd.invalidate(); return true end
  end
  return false
end

-- ---------------------------------------------------------------- Konfiguration

local function addSource(w, label, prop)
  local line = form.addLine(label)
  return form.addSourceField(line, nil, function() return w[prop] end, function(v) w[prop] = v end)
end

local function addSwitch(w, label, prop)
  local line = form.addLine(label)
  return form.addSwitchField(line, nil, function() return w[prop] end, function(v) w[prop] = v end)
end

local function addNumber(w, label, prop, lo, hi)
  local line = form.addLine(label)
  return form.addNumberField(line, nil, lo, hi, function() return w[prop] end, function(v) w[prop] = v end)
end

local function addBool(w, label, prop)
  local line = form.addLine(label)
  return form.addBooleanField(line, form.getFieldSlots(line)[0], function() return w[prop] end, function(v) w[prop] = v end)
end

-- Felder fuer das barometrische Vario sind nur bei diesem Typ bedienbar.
local function updateBaroFields(w)
  for _, f in ipairs(w.baroFields or {}) do
    if f and f.enable then pcall(f.enable, f, w.varioType == 2) end
  end
end

local function configure(w)
  w.baroFields = {}
  refreshPolarFile(w, os.clock(), true)
  rebuildPolar(w)

  -- 1. Zeile: Polare vorhanden? Ohne Polare keine weiteren Eingaben.
  local text, ok = polarStatus(w)
  local line = form.addLine("Polare")
  form.addStaticText(line, form.getFieldSlots(line)[0], text)
  if not ok then
    line = form.addLine("Hinweis")
    form.addStaticText(line, form.getFieldSlots(line)[0], "MM Flight Analyzer: Polare erstellen und auf den Sender schreiben")
    return
  end

  -- Sensoren
  addSource(w, "GPS", "gps")
  addSource(w, "GPS-Geschw. (empfohlen)", "gpsSpd")
  addSource(w, "GPS Hoehe (Windschichten)", "gpsAlt")
  addSource(w, "Vario", "vario")
  line = form.addLine("Vario-Typ")
  form.addChoiceField(line, nil, {{"Kompensiert (TE)", 1}, {"Barometrisch", 2}},
    function() return w.varioType end,
    function(v) w.varioType = v; updateBaroFields(w) end)
  w.baroFields[#w.baroFields + 1] = addNumber(w, "Barometrisch: Kompensation (%)", "teFactor", 0, 150)
  w.baroFields[#w.baroFields + 1] = addNumber(w, "Barometrisch: Vario-Zeitkonst. (0.1 s)", "varioTau", 0, 100)
  updateBaroFields(w)

  -- MC
  addSource(w, "MC Poti/Schieber (optional)", "mcSrc")
  addSwitch(w, "MC+ Schalter (optional)", "mcUp")
  addSwitch(w, "MC- Schalter (optional)", "mcDn")
  addNumber(w, "MC Maximum (0.1 m/s)", "mcMax", 10, 100)
  addNumber(w, "MC Schritt (0.1 m/s)", "mcStep", 1, 20)
  addBool(w, "MC ansagen", "announce")
  line = form.addLine("MC-Ansage")
  form.addChoiceField(line, nil, {{"Sprache", 1}, {"Sprache + Ton", 2}, {"Nur Ton", 3}},
    function() return w.announceMode end, function(v) w.announceMode = v end)

  -- Modus und Ton
  addSwitch(w, "Modus-Schalter (an = Gleiten)", "modeSw")
  addNumber(w, "Steigen-Schwelle Auto (0.1 m/s)", "climbThr", 1, 30)
  addBool(w, "Ton", "tone")
  addSwitch(w, "Ton-Schalter (optional)", "soundSw")
  line = form.addLine("Ton-Logik")
  form.addChoiceField(line, nil,
    {{"Schneller fliegen = hoeher", 1}, {"Bremsen = hoeher", 2}},
    function() return w.toneLogic end, function(v) w.toneLogic = v end)

  -- Glaettung
  addNumber(w, "Glaettung Fahrt (s)", "smoothSpd", 1, 30)
  addNumber(w, "Glaettung Netto (s)", "smoothNet", 1, 30)
  addNumber(w, "Glaettung Vario (s)", "smoothVar", 0, 10)
  addNumber(w, "Totzone (+- km/h)", "dead", 0, 20)
  addNumber(w, "Ton-Verzoegerung (0.1 s)", "delay", 0, 100)
  addNumber(w, "Mindestfahrt (km/h)", "minSpeed", 0, 60)

  -- Modell
  addNumber(w, "Fluggewicht (% der Polare)", "ballast", 50, 300)
  addNumber(w, "Verschmutzung (%)", "bugs", 0, 30)
  addBool(w, "Demo-Modus (Simulator)", "demo")
end

local function setDefaults(w)
  for _, k in ipairs(SOURCE_KEYS) do w[k] = nil end
  for _, d in ipairs(NUM_DEFS) do w[d.key] = d.def end
  for _, d in ipairs(BOOL_DEFS) do w[d.key] = d.def end
end

local function read(w)
  -- Versionsmarke zuerst: aeltere Speicherstaende (anderes Layout) werden verworfen.
  local same = storage.read("ver") == STORE_VER
  if not same then
    setDefaults(w)
  else
    for _, k in ipairs(SOURCE_KEYS) do
      w[k] = validSource(storage.read(k))
    end
    for _, d in ipairs(NUM_DEFS) do
      local v = storage.read(d.key)
      if type(v) == "number" then w[d.key] = v else w[d.key] = d.def end
    end
    for _, d in ipairs(BOOL_DEFS) do
      local v = storage.read(d.key)
      if type(v) == "boolean" then w[d.key] = v else w[d.key] = d.def end
    end
  end
  -- spaeter dazugekommen, deshalb am Ende (Ethos-Storage arbeitet nach Aufrufreihenfolge)
  w.gpsSpd = same and validSource(storage.read("gpsSpd")) or nil
  w.mcValue = w.mc / 10
  w.polarSig = nil
  w.fileChecked = nil
end

local function write(w)
  storage.write("ver", STORE_VER)
  for _, k in ipairs(SOURCE_KEYS) do storage.write(k, w[k]) end
  for _, d in ipairs(NUM_DEFS) do storage.write(d.key, w[d.key]) end
  for _, d in ipairs(BOOL_DEFS) do storage.write(d.key, w[d.key]) end
  storage.write("gpsSpd", w.gpsSpd)
end

local function init()
  -- Ethos akzeptiert Widget-Schluessel nur mit weniger als 8 Zeichen.
  system.registerWidget({
    key = "sollf01",
    name = "Sollfahrt",
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
  buildPolar = buildPolar, polarSink = polarSink, speedToFly = speedToFly,
  process = process, toneFor = toneFor, lpf = lpf, create = create, rebuildPolar = rebuildPolar,
  makePolar = makePolar, polarFromAbc = polarFromAbc, loadPolarFile = loadPolarFile,
  findPolarEntry = findPolarEntry, refreshPolarFile = refreshPolarFile,
  setPolarPaths = function(list) polarPaths = list end,
  kasaFit = kasaFit, courseSpread = courseSpread, fitIsPlausible = fitIsPlausible,
  extractLatLon = extractLatLon, readPosition = readPosition, gpsUpdate = gpsUpdate,
  sensorUpdate = sensorUpdate, courseUpdate = courseUpdate, fitOkMax = fitOkMax,
  airData = airData, windAt = windAt, binOf = binOf, inputsStep = inputsStep, sayMC = sayMC, polarStatus = polarStatus,
}
return M
