-- @widget  SkyNavigator
-- @version 1.43
-- SkyNavigator: GPS-Triangle-Navigation fuer Ethos -- Vollbild-Widget mit Tonausgabe.
--
-- Die Aufgaben (Dreiecke) kommen aus dem MM Flight Analyzer, Tab "SkyNavigator": Die App schreibt
-- sie beim Anschliessen des Senders nach scripts/skynav/aufgaben.lua. In der Konfiguration waehlst
-- Du nur noch die Aufgabe. Die Polare fuer den Sollfahrt-Teil kommt wie beim Widget Sollfahrt aus
-- scripts/maccready/polaren.lua (ohne Polare laeuft die Navigation trotzdem).
-- Ballast: Am Sender stehen eine Ballast-Quelle (0..100 %, optional: Poti, Schieber, Schalter, Kanal) und Ballast maximal in kg (ohne Poti gilt er
-- fix). Das Leergewicht steht pro Modell als mass (kg) in polaren.lua; die Polare wird auf Leergewicht plus
-- Ballast umgerechnet, die Kopfzeile zeigt Ballast und Fluggewicht.
--
-- Regeln nach "GPS Triangle Definition V1.7b" und den Reglementen Light/Sport/Scale:
--   Dreieck rechtwinklig-gleichschenklig, Ursprung O = Mitte der Hypotenuse, dTP = Abstand O-Ecke,
--   Richtung alpha = Peilung O -> TP1, TP3 gegenueber, TP2 im rechten Winkel bei alpha - 90.
--   Start-/Ziellinie durch O senkrecht zur Hypotenuse (je 2 km lang), Startsektor = Seite TP3.
--   Wendepunkt erreicht = ein GPS-Fix im Viertelkreis (r 1.5 km) mit Spitze in der Ecke,
--   Winkelhalbierende = Winkelhalbierende der Schenkel (nach aussen).
--   Start gilt nur "scharf" (ARM-Schalter oder Tippen) und im Startfenster; Einflugwerte aus dem
--   letzten Fix im Startsektor. Strafe = 50 + 2*(Hoehe-Limit) + 2*(Speed-Limit), 50 nur einmal.
--   Ziel: erster Fix im Zielsektor, 3D-Speed >= Mindestspeed, innerhalb der Arbeitszeit.
--   Schnitt = Anzahl Dreiecke * Umfang / (letztes Ziel - Start), Umfang = dTP * (2 + 2*sqrt(2)).
--
-- Seiten (je Seite ein Schalter mit Stellung in der Konfiguration; Tippen auf die Kopfzeile geht
-- auch): START, GLEITEN, THERMIK, AUSWERTUNG. Laufende Toene (Sollfahrt, Start, Vario) kommen nur fuer die angezeigte
-- Seite. Varioton (wie XCSoar): Seite Thermik, beim Kreisen auf allen Seiten, auf Gleiten statt Sollfahrt bei Netto > MC. Wendeton, Start, Sektor und Ziel kommen immer, auch wenn am Sender ein anderer Bildschirm offen ist.
-- Beim Start sagt das Widget "Einflug OK" bzw. "<n> Strafpunkte" (audio/einflug_ok.wav, strafpunkte.wav, Zahl vom Sender).
--
-- Optimaler Weg (Seite GLEITEN): Das Widget rechnet im Luftmassen-System. Dort sind Kurven echte
-- Kreise mit dem aktuellen Wenderadius, die Wendepunkte wandern mit -Wind. Fuer die naechsten drei
-- Ecken liegt ein Kreis (Radius R) so, dass er den Sektor um "Reserve" Meter innen beruehrt. Der
-- Weg = Tangente zum ersten Kreis, Bogen, Aussentangente zum naechsten Kreis ... Die Ankunftszeiten
-- (und damit der Versatz der Ecken) werden dreimal nachgerechnet. Daraus kommen der Soll-Steuerkurs
-- (Ansage "xx links/rechts" vor allem bei wachsender Ablage, Sprachdateien in scripts/skynav/audio; Kurs-Ticker) und die
-- Zeit bis zum Einleiten der Kurve (Wendeton: Countdown-Piepse, dann hoher Ton "jetzt wenden").
--
-- Restzeit-Sollfahrt (Zeit-MC): Aus Restzeit, Netto pro Schenkel, Wind, Hoehe und Steigen rechnet das Widget, wie
-- viele Dreiecke noch passen, und zeigt statt des festen MC den MC, der sie am schnellsten schafft (Tippen auf das
-- MC-Feld schaltet zwischen festem MC und MC ZEIT; mit MC-Poti: Anschlag oben = MC ZEIT). Im Bart: ab welcher Hoehe der Endanflug ohne Bart reicht.
--
-- IGC: Das Widget schreibt ab dem Scharfschalten bzw. Abheben (mit 90 s Vorlauf) eine IGC-Datei nach
-- scripts/skynav/fluege/ (1 Hz, I-Record SUS, LTSK, C-Records, E-Records PEV/STA/TPC/FIN/STP,
-- G-Record). Die App holt sie beim naechsten Sync ins Archiv und wertet sie aus.

local VERSION = "1.43"
local KMH = 3.6
local G = 9.81
local STORE_VER = 1
local M_PER_DEG = 111320          -- WGS84: re * pi / 180
local SECTOR_R = 1500
local LINE_HALF = 2000
local atan2 = math.atan2 or function(y, x) return math.atan(y, x) end
local PI, TWO_PI = math.pi, 2 * math.pi

local P_OVER, P_START, P_GLIDE, P_THERM, P_EVAL = 1, 2, 3, 4, 5
local PAGE_NAMES = {"ÜBERSICHT", "START", "GLEITEN", "THERMIK", "WERTUNG"}
local PAGE_SHORT = {"ÜBER", "START", "GLEIT", "THERM", "WERT"}
local S_IDLE, S_ARMED, S_STARTED, S_STOPPED = 0, 1, 2, 3

local TASK_PATHS = {"/scripts/skynav/aufgaben.lua", "scripts/skynav/aufgaben.lua", "aufgaben.lua"}
-- Vom Systemtool "SkyNav Aufgabe" (scripts/skynavtool): Aufgaben aus dem Code, aktive Aufgabe, Startfenster
-- (eine Tabelle statt einzelner locals: der Hauptteil ist nahe an der Grenze von 200 locals)
local FELD = {paths = {"/scripts/skynav/feld.lua", "scripts/skynav/feld.lua", "feld.lua"}}
local POLAR_PATHS = {"/scripts/maccready/polaren.lua", "scripts/maccready/polaren.lua", "../maccready/polaren.lua"}
local RADIUS_PATHS = {"/scripts/skynav/radien.lua", "scripts/skynav/radien.lua", "radien.lua"}
local PRED_S = 5      -- s, Vorhersage in der Kursgrafik (Seite Gleiten)
local R_DEFAULT = 30   -- m, bis das Widget den ersten Wenderadius des Modells gelernt hat
local FLIGHT_DIRS = {"/scripts/skynav/fluege/", "fluege/", "/scripts/skynav/", ""}
local AUDIO_DIRS = {"/scripts/skynav/audio/", "audio/"}

-- Klassen (Reglement), falls die Aufgabe keine Werte mitbringt
local CLASSES = {
  Light = {dist = 200, maxH = 250, maxV = 70, minF = 10, work = 20, maxAlt = 350},
  Sport = {dist = 350, maxH = 400, maxV = 120, minF = 20, work = 30, maxAlt = 750},
  Scale = {dist = 500, maxH = 500, maxV = 120, minF = 20, work = 30, maxAlt = 750},
}

-- ---------------------------------------------------------------- Hilfsfunktionen

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

-- Winkel in Grad auf -180..180
local function angDiff(a, b)
  local d = (a - b) % 360
  if d > 180 then d = d - 360 end
  return d
end

-- Peilung (Grad, 0 = Nord, im Uhrzeigersinn) eines Vektors (x Ost, y Nord)
local function bearingOf(x, y) return math.deg(atan2(x, y)) % 360 end
local function dirVec(deg) local r = math.rad(deg) return math.sin(r), math.cos(r) end

-- Bogen von Winkel a nach b (Bogenmass, Mathe-Konvention) im Drehsinn s (+1 = links/gegen Uhrzeiger)
local function arcAngle(a, b, s)
  local d = (s > 0) and (b - a) or (a - b)
  d = d % TWO_PI
  return d
end

local function fmtMS(sec)
  if sec == nil then return "--:--" end
  local neg = sec < 0
  sec = math.floor(math.abs(sec) + 0.5)
  return string.format("%s%02d:%02d", neg and "-" or "", math.floor(sec / 60), sec % 60)
end

-- ---------------------------------------------------------------- Polare und Sollfahrt
-- (gleich wie im Widget Sollfahrt: Sinken w(v) = A*v^2 + B*v + C, v in m/s)

local function makePolar(A, B, C, vMin, vMax)
  if type(A) ~= "number" or type(B) ~= "number" or type(C) ~= "number" then return nil end
  if A <= 1e-9 or C <= 0 then return nil end
  vMin = (type(vMin) == "number" and vMin > 0) and vMin or 1
  vMax = (type(vMax) == "number" and vMax > vMin) and vMax or (vMin * 3)
  local vms = -B / (2 * A)
  local lo = math.min(math.max(vms, vMin), vMax)
  return {A = A, B = B, C = C, vms = (vms > 0) and vms or vMin, lo = lo, vmin = vMin, vmax = vMax}
end

local function polarFromAbc(a, b, c, vMin, vMax, mass, bugs)
  if type(c) == "number" and c < 0 and type(a) == "number" and type(b) == "number" then
    a, b, c = -a, -b, -c
  end
  local s = math.sqrt(math.max(mass, 0.1))
  local f = 1 + bugs
  return makePolar(a / s * f, b * f, c * s * f, (vMin or 0) * s, (vMax or 0) * s)
end

local function polarSink(p, v) return p.A * v * v + p.B * v + p.C end

local function speedToFly(p, mc, netto, headwind)
  local h = headwind or 0
  local x = p.C + mc - netto + p.B * h
  local rad = h * h + x / p.A
  local v = h + math.sqrt(math.max(rad, 0))
  return clamp(v, p.lo, p.vmax), v > p.vmax + 1e-9
end

-- Lua-Datei lesen (loadfile, sonst io.open + load), leere Umgebung. Liefert Tabelle, Pfad.
local function loadLuaTable(paths)
  for _, path in ipairs(paths) do
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
          local okLoad, c = pcall(load, text, "=" .. path, "t", {})
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

local function findByName(all, name)
  if type(all) ~= "table" or type(name) ~= "string" then return nil end
  if type(all[name]) == "table" then return all[name], name end
  local function norm(s) return (s:lower():gsub("^%s+", ""):gsub("%s+$", "")) end
  local wanted = norm(name)
  for key, entry in pairs(all) do
    if type(key) == "string" and type(entry) == "table" and norm(key) == wanted then return entry, key end
  end
  return nil
end

-- ---------------------------------------------------------------- Wind (Kreisfit wie Sollfahrt)

local SAMPLE_DT, WINDOW = 0.7, 40       -- Fit ab 10 Stuetzwerten (erster Fit: volles Fenster), jede Sekunde
local MAX_SPREAD_R, MAX_TAS_FACTOR, MAX_WIND_FACTOR, MAX_RESIDUAL_FRAC = 0.3, 2.0, 1.0, 0.25

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

local function courseSpread(pts)
  local sx, sy, n = 0, 0, 0
  for _, p in ipairs(pts) do
    local m = math.sqrt(p.x * p.x + p.y * p.y)
    if m > 0.1 then sx, sy, n = sx + p.x / m, sy + p.y / m, n + 1 end
  end
  if n == 0 then return 1 end
  return math.sqrt(sx * sx + sy * sy) / n
end

local function fitOk(cx, cy, r, resFrac, pts)
  if not r or r <= 1e-3 then return false end
  local maxG = 0
  for _, p in ipairs(pts) do maxG = math.max(maxG, math.sqrt(p.x * p.x + p.y * p.y)) end
  if maxG <= 1e-3 or r > MAX_TAS_FACTOR * maxG then return false end
  if math.sqrt(cx * cx + cy * cy) > MAX_WIND_FACTOR * maxG then return false end
  return resFrac <= MAX_RESIDUAL_FRAC
end

-- Mit Staudruck-Fahrt (1.42): Fit mit bekannter Fahrt statt Kasa. Gesucht Wind W und Faktor k mit
-- |Boden_i - W| = k * TAS_i (Gauss-Newton, 3 Unbekannte, k faengt einen Kalibrierfehler der Fahrt auf).
-- Der Kasa-Fit nimmt konstante Fahrt an; Fahrtwechsel im Fenster (Ziehen/Druecken, halbe Kreise, Motorsteigflug
-- mit 3D-Geschwindigkeit des SM-Loggers) verschoben den Kreismittelpunkt, und der Wind kam zeitweise aus der
-- Gegenrichtung (Log 13.11.: Ostwind 15-20 km/h, Fits bis 13 km/h aus Sued/West). Mit Fahrt: alle Fits NO.
-- Grenzen mit Fahrt: Kurs-Streuung unter 0.5 (etwa ein halber Kreis genuegt), Rest unter 12 % der Fahrt,
-- k 0.7..1.4. Ohne Fahrt wie bisher Kasa mit Streuung unter 0.3 (rund 265 Grad). Replay 13.11.: Wind aus
-- 100..320 Grad vorher 326 s, mit Fahrt 0 s, ohne Fahrt (nur Motor ausgeblendet) 69 s.
-- Funktionen lokal in windFit (Ethos: max. 200 lokale Namen im Hauptteil).

-- Stuetzwert (Bodenvektor, dazu TAS falls Staudruck gilt) ins Fenster, Fit jede Sekunde.
-- Wind = Vektor, wohin die Luft zieht (m/s).
local function windStep(w, ve, vn, now, tas)
  local win = w.win
  if ve and math.sqrt(ve * ve + vn * vn) >= 2 / KMH and (w.sT == nil or now - w.sT >= SAMPLE_DT) then
    w.sT = now
    win[#win + 1] = {x = ve, y = vn, a = tas}
    if #win > WINDOW then table.remove(win, 1) end
  end
end

-- Kreisfit (jede Sekunde), laeuft in einem Aufruf ohne neuen GPS-Fix
local function windFit(w, now)
  local win = w.win
  local need = w.windOk and 10 or WINDOW
  if #win >= need and (w.fitT == nil or now - w.fitT >= 1) then
    w.fitT = now
    -- (Funktionen erst hier erzeugt: nur einmal pro Sekunde, nicht in jedem wakeup)
    local AS_SPREAD, AS_RES_FRAC, AS_K_MIN, AS_K_MAX, KASA_SPREAD = 0.5, 0.12, 0.7, 1.4, 0.3
    -- 3x3 loesen (Cramer), nil wenn singulaer
    local function solve3(A, b)
      local function det(m)
        return m[1][1] * (m[2][2] * m[3][3] - m[2][3] * m[3][2]) - m[1][2] * (m[2][1] * m[3][3] - m[2][3] * m[3][1])
          + m[1][3] * (m[2][1] * m[3][2] - m[2][2] * m[3][1])
      end
      local D = det(A)
      if math.abs(D) < 1e-9 then return nil end
      local r = {}
      for c = 1, 3 do
        local M = {{A[1][1], A[1][2], A[1][3]}, {A[2][1], A[2][2], A[2][3]}, {A[3][1], A[3][2], A[3][3]}}
        for i = 1, 3 do M[i][c] = b[i] end
        r[c] = det(M) / D
      end
      return r
    end

    -- Wind mit bekannter Fahrt: Start beim bisherigen Wind (oder 0), 8 Schritte. Gibt wx, wy, k, Rest (m/s) zurueck.
    local function airFit(pts, wx, wy)
      local k = 1
      for _ = 1, 8 do
        local A = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}
        local b = {0, 0, 0}
        for _, p in ipairs(pts) do
          local dx, dy = p.x - wx, p.y - wy
          local m = math.sqrt(dx * dx + dy * dy)
          if m > 1e-3 then
            local r, j1, j2, j3 = m - k * p.a, -dx / m, -dy / m, -p.a   -- Ableitungen nach wx, wy, k
            b[1], b[2], b[3] = b[1] - j1 * r, b[2] - j2 * r, b[3] - j3 * r
            A[1][1], A[1][2], A[1][3] = A[1][1] + j1 * j1, A[1][2] + j1 * j2, A[1][3] + j1 * j3
            A[2][2], A[2][3], A[3][3] = A[2][2] + j2 * j2, A[2][3] + j2 * j3, A[3][3] + j3 * j3
          end
        end
        A[2][1], A[3][1], A[3][2] = A[1][2], A[1][3], A[2][3]
        local s = solve3(A, b)
        if not s then return nil end
        wx, wy, k = wx + s[1], wy + s[2], k + s[3]
      end
      local res = 0
      for _, p in ipairs(pts) do res = res + (math.sqrt((p.x - wx) ^ 2 + (p.y - wy) ^ 2) - k * p.a) ^ 2 end
      return wx, wy, k, math.sqrt(res / #pts)
    end
    local sp = courseSpread(win)
    -- Staudruck: alle Stuetzwerte im Fenster mit Fahrt
    local nA, sumA, maxG = 0, 0, 0
    for _, p in ipairs(win) do
      if p.a and p.a > 3 then nA, sumA = nA + 1, sumA + p.a end
      maxG = math.max(maxG, math.sqrt(p.x * p.x + p.y * p.y))
    end
    local cx, cy, good
    if nA == #win then
      if sp < AS_SPREAD then
        local k, res
        cx, cy, k, res = airFit(win, w.windOk and w.wx or 0, w.windOk and w.wy or 0)
        good = cx ~= nil and k >= AS_K_MIN and k <= AS_K_MAX and res <= AS_RES_FRAC * k * sumA / nA
          and math.sqrt(cx * cx + cy * cy) <= MAX_WIND_FACTOR * maxG
        if good then w.windK = k end
      end
    elseif sp < KASA_SPREAD then
      local r, res
      cx, cy, r, res = kasaFit(win)
      good = cx ~= nil and fitOk(cx, cy, r, res, win)
    end
    if good then
      if w.windOk then
        w.wx, w.wy = w.wx + 0.3 * (cx - w.wx), w.wy + 0.3 * (cy - w.wy)
      else
        w.wx, w.wy, w.windOk = cx, cy, true
      end
      w.windT = now
    end
  end
end

-- ---------------------------------------------------------------- Aufgabe und Geometrie

local function num(v, def)
  v = tonumber(v)
  if v == nil then return def end
  return v
end

-- Aufgabe aus aufgaben.lua vervollstaendigen und die Geometrie im lokalen System (m, x Ost, y Nord,
-- Ursprung O) rechnen.
local function buildTask(t)
  if type(t) ~= "table" then return nil end
  local cl = CLASSES[t.cls or ""] or CLASSES.Sport
  local T = {
    id = tostring(t.id or t.name or "?"), name = tostring(t.name or "Aufgabe"), cls = tostring(t.cls or ""),
    lat = num(t.lat), lon = num(t.lon), elev = num(t.elev, 0), dir = num(t.dir, 0) % 360,
    dist = num(t.dist, cl.dist), maxH = num(t.maxH, cl.maxH), maxV = num(t.maxV, cl.maxV),
    minF = num(t.minF, cl.minF), work = num(t.work, cl.work), maxAlt = num(t.maxAlt, cl.maxAlt),
    slot = type(t.slot) == "table" and t.slot or nil, rct = type(t.rct) == "string" and t.rct or nil,
    zones = {},
  }
  if T.lat == nil or T.lon == nil or T.dist <= 0 then return nil end
  T.cosLat = math.cos(math.rad(T.lat))
  local ux, uy = dirVec(T.dir)
  local px, py = dirVec(T.dir - 90)
  local d = T.dist
  T.ux, T.uy, T.px, T.py = ux, uy, px, py
  local V = {{x = d * ux, y = d * uy}, {x = d * px, y = d * py}, {x = -d * ux, y = -d * uy}}
  local outDeg = {T.dir + 22.5, T.dir - 90, T.dir + 157.5}
  T.tp = {}
  for i = 1, 3 do
    local bx, by = dirVec(outDeg[i])
    local prev, nxt = V[(i + 1) % 3 + 1], V[i % 3 + 1]
    local ix, iy = V[i].x - prev.x, V[i].y - prev.y
    local ox, oy = nxt.x - V[i].x, nxt.y - V[i].y
    T.tp[i] = {x = V[i].x, y = V[i].y, bx = bx, by = by, s = (ix * oy - iy * ox) > 0 and 1 or -1, n = "TP" .. i}
  end
  T.perim = d * (2 + 2 * math.sqrt(2))
  for _, z in ipairs(type(t.zones) == "table" and t.zones or {}) do
    if type(z) == "table" and z.lat and z.lon then
      local zx = (num(z.lon, 0) - T.lon) * M_PER_DEG * T.cosLat
      local zy = (num(z.lat, 0) - T.lat) * M_PER_DEG
      T.zones[#T.zones + 1] = {
        t = (z.t == "R") and "R" or "C", name = tostring(z.name or "Zone"), x = zx, y = zy, lat = num(z.lat, 0), lon = num(z.lon, 0),
        r = num(z.r, 100), dir = num(z.dir, 0), len = num(z.len, 100), wid = num(z.wid, 50),
        lo = num(z.lo, 0), hi = num(z.hi, 9999),
      }
    end
  end
  return T
end

local function toLocal(T, lat, lon)
  return (lon - T.lon) * M_PER_DEG * T.cosLat, (lat - T.lat) * M_PER_DEG
end

local function inSector(T, i, x, y)
  local c = T.tp[i]
  local dx, dy = x - c.x, y - c.y
  local n = math.sqrt(dx * dx + dy * dy)
  if n > SECTOR_R then return false end
  if n < 1 then return true end
  return (dx * c.bx + dy * c.by) / n >= math.cos(math.rad(45)) - 1e-9
end

local function inZone(z, x, y, agl)
  if agl < z.lo or agl > z.hi then return false end
  if z.t == "C" then return (x - z.x) ^ 2 + (y - z.y) ^ 2 <= z.r * z.r end
  local ux, uy = dirVec(z.dir)
  local dx, dy = x - z.x, y - z.y
  return math.abs(dx * ux + dy * uy) <= z.len / 2 and math.abs(dx * uy - dy * ux) <= z.wid / 2
end

local function penaltyOf(T, agl, kmh)
  local a, v = math.floor(agl + 0.5), math.floor(kmh + 0.5)
  local ha, hv = a > T.maxH, v > T.maxV
  if not ha and not hv then return 0 end
  return 50 + 2 * math.max(0, a - T.maxH) + 2 * math.max(0, v - T.maxV)
end

-- Startfenster (UTC-Sekunden seit Tagesbeginn ist zu wenig: Datum zaehlt mit). slot = {y,mo,d,h,mi,len}
-- Liefert Beginn und Ende als "UTC-Epoche" im Sinn von os.time() der Senderuhr (siehe utcNow).
local function slotEpoch(slot)
  if type(slot) ~= "table" or not slot.h then return nil end
  local y, mo, d = num(slot.y, 1970), num(slot.mo, 1), num(slot.d, 1)
  -- Tage seit 1970 (proleptisch gregorianisch), unabhaengig von der Zeitzone der Senderuhr
  local function days(yy, mm, dd)
    if mm <= 2 then yy, mm = yy - 1, mm + 12 end
    local era = math.floor(yy / 400)
    local yoe = yy - era * 400
    local doy = math.floor((153 * (mm - 3) + 2) / 5) + dd - 1
    local doe = yoe * 365 + math.floor(yoe / 4) - math.floor(yoe / 100) + doy
    return era * 146097 + doe - 719468
  end
  local t0 = days(y, mo, d) * 86400 + num(slot.h, 0) * 3600 + num(slot.mi, 0) * 60
  return t0, t0 + num(slot.len, 10) * 60
end

-- ---------------------------------------------------------------- Optimaler Weg

-- Ecken ab Index i (1..3), n Stueck, zyklisch. Kreis: Mittelpunkt = Ecke + (Reserve - R) * Halbierende
local function cornersFrom(T, i, n, R, margin)
  local out = {}
  for k = 0, n - 1 do
    local j = (i - 1 + k) % 3 + 1
    local c = T.tp[j]
    local ax, ay = c.x + margin * c.bx, c.y + margin * c.by
    local r = (type(R) == "function") and R(j) or R                 -- Radius je Ecke (gelernt pro TP)
    out[#out + 1] = {i = j, ax = ax, ay = ay, zx = ax - r * c.bx, zy = ay - r * c.by,
      thA = atan2(c.by, c.bx), s = c.s, R = r, tx = c.x, ty = c.y, bx = c.bx, by = c.by, margin = margin}
  end
  return out
end

-- Plant den Weg ab Position (px,py) im Luftmassen-System. W = Wind (wx,wy), V = Fahrt, R = Radius.
-- Liefert: hdg (Soll-Steuerkurs Grad), dT (Luft-Strecke bis Kurvenbeginn m), tT (s), inTurn,
-- t1 (Ankunft an der ersten Ecke s), pts (Boden-Punkte fuer die Zeichnung), entry/exit (Boden).
-- hdgNow (optional, Grad, Steuerkurs in der Luft): liegt die Tangente zum ersten Kreis hinter dem Modell
-- (Kurvenbeginn verpasst), fliegt der Plan erst geradeaus weiter und wendet spaeter durch denselben Zielpunkt.
local lapRoute   -- weiter unten, gebraucht im wakeup

local function planPath(px, py, wx, wy, V, R, C, hdgNow, noPts)
  V = math.max(V, 3)
  local n = #C
  local ts = {}
  local acc = 0
  local lx, ly = px, py
  for k = 1, n do
    -- Startwert mit Grundgeschwindigkeit, damit die Iteration auch bei starkem Wind konvergiert
    local sx, sy = C[k].ax - lx, C[k].ay - ly
    local sl = math.sqrt(sx * sx + sy * sy)
    local gs = V
    if sl > 1 then
      local wu = (wx * sx + wy * sy) / sl
      local disc = wu * wu - (wx * wx + wy * wy) + V * V
      if disc > 0 then gs = math.max(wu + math.sqrt(disc), 3) end
    end
    acc = acc + sl / gs
    ts[k] = acc
    lx, ly = C[k].ax, C[k].ay
  end
  -- Iteration der Ankunftszeiten (Kreise wandern mit dem Wind). Zeichenpunkte erst im letzten Durchgang,
  -- sobald die Zeit steht (oder nach 6 Durchgaengen); spart viele Instruktionen.
  local res
  local it, final, draw, prevAll = 0, false, false, nil
  local lateT
  while true do
    it = it + 1
    final = final or it >= 6
    draw = final and not noPts
    res = {pts = {}}
    local L = 0
    local Z = {}
    for k = 1, n do Z[k] = {x = C[k].zx - wx * ts[k], y = C[k].zy - wy * ts[k]} end
    local s1 = C[1].s
    local R1 = C[1].R or R
    local dx, dy = px - Z[1].x, py - Z[1].y
    local d = math.sqrt(dx * dx + dy * dy)
    local thIn
    local thA = {}
    for k = 1, n do thA[k] = C[k].thA end
    local function addPt(ax, ay, t) if draw then res.pts[#res.pts + 1] = {x = ax + wx * t, y = ay + wy * t} end end
    addPt(px, py, 0)
    -- Kurvenbeginn verpasst? Dann Kreis, der in Flugrichtung beruehrt und durch den Zielpunkt geht
    local forced = false
    if hdgNow and d < 200 then                                        -- nur nahe an der Ecke (nicht beim Ausflug)
      local hx, hy = dirVec(hdgNow)
      local late = d <= R1 * 1.02                                     -- schon im Kreis: Kurvenbeginn verpasst
      if not late then
        local phi = atan2(dy, dx)
        local th0 = phi + s1 * math.acos(clamp(R1 / d, -1, 1))
        local tx, ty = Z[1].x + R1 * math.cos(th0) - px, Z[1].y + R1 * math.sin(th0) - py
        local tl = math.sqrt(tx * tx + ty * ty)
        -- mehr als ~30 Grad abdrehen noetig, und zwar kurz vor dem Kreis (Tangentenpunkt hoechstens 3 s bzw.
        -- 3 Radien entfernt). Weiter weg ist es nur ein Kursfehler, kein verpasster Kurvenbeginn: sonst schaltete
        -- ein Pilot, der 150 m vor der Ecke 30 Grad neben dem Plan fliegt, auf "Ecke verpasst" (kein Wendeton,
        -- keine Kurs-Ansagen mehr bis zum Sektor).
        late = tl > 1 and (tx * hx + ty * hy) / tl < 0.87 and tl < math.max(3 * R1, 3 * V)
      end
      if late then
        -- geradeaus weiter (ueber Grund mit Wind), bis der Punkt margin tief im Sektor liegt, dann wenden
        local c1 = C[1]
        local gvx, gvy = V * hx + wx, V * hy + wy
        local gv = math.sqrt(gvx * gvx + gvy * gvy)
        if gv > 1 then
          -- Suche des Punkts im Sektor haengt nicht von der Iteration ab: nur einmal rechnen
          if lateT == nil then
            lateT = false
            local function inS(Lg)
              local t = Lg / gv
              local rx, ry = px + gvx * t - c1.tx, py + gvy * t - c1.ty
              local rl = math.sqrt(rx * rx + ry * ry)
              local dep = rx * c1.bx + ry * c1.by
              return rl > 0.1 and dep >= c1.margin and dep / rl >= 0.7071
            end
            -- grob in 10 m, dann fein in 2 m
            for Lg = 0, 300, 10 do
              if inS(Lg) then
                local L0 = Lg
                for Lf = math.max(Lg - 8, 0), Lg, 2 do if inS(Lf) then L0 = Lf; break end end
                lateT = L0 / gv
                break
              end
            end
          end
          if lateT then
            local t = lateT
            local gx, gy = px + gvx * t, py + gvy * t
            do
              forced = true
              local sx, sy = px + V * hx * t, py + V * hy * t       -- Luftpunkt des Kurvenbeginns
              local nxm, nym = -hy * s1, hx * s1                      -- Kreismitte auf der Kurvenseite
              Z[1] = {x = sx + R1 * nxm, y = sy + R1 * nym}
              thIn = atan2(sy - Z[1].y, sx - Z[1].x)
              thA[1] = thIn
              L = V * t
              res.dT, res.tT = L, t
              res.hdg = hdgNow % 360
              addPt(sx, sy, t)
              res.entry = {x = gx, y = gy}
            end
          end
        end
      end
    end
    if forced then
      res.late = true
    elseif d <= R1 * 1.02 then
      res.inTurn = true
      thIn = atan2(dy, dx)
      res.dT, res.tT = 0, 0
      local tx, ty = -math.sin(thIn) * s1, math.cos(thIn) * s1      -- Tangente im Drehsinn
      res.hdg = bearingOf(tx, ty)
    else
      local phi = atan2(dy, dx)
      local beta = math.acos(clamp(R1 / d, -1, 1))
      thIn = phi + s1 * beta
      local tx, ty = Z[1].x + R1 * math.cos(thIn), Z[1].y + R1 * math.sin(thIn)
      L = math.sqrt(math.max(d * d - R1 * R1, 0))
      res.dT, res.tT = L, L / V
      res.hdg = bearingOf(tx - px, ty - py)
      addPt(tx, ty, L / V)
      res.entry = {x = tx + wx * L / V, y = ty + wy * L / V}
    end
    for k = 1, n do
      local s = C[k].s
      local R = C[k].R or R
      local arcA = arcAngle(thIn, thA[k], s)
      if arcA > 1.75 * PI then arcA = 0 end                         -- Zielpunkt eben ueberflogen
      -- Bogen bis zum Zielpunkt (fuer die Zeichnung in Schritten von 8 m, hoechstens alle 15 Grad: 3.82 = 12 / pi)
      local steps = draw and math.max(1, math.min(math.floor(arcA * R / 8), math.ceil(arcA * 3.82))) or 0
      for j = 1, steps do
        local th = thIn + s * arcA * j / steps
        addPt(Z[k].x + R * math.cos(th), Z[k].y + R * math.sin(th), (L + arcA * R * j / steps) / V)
      end
      L = L + arcA * R
      ts[k] = L / V
      if k == 1 then res.t1 = ts[1] elseif k == 2 then res.t2 = ts[2] end
      if k == n then break end
      -- Aussentangente zum naechsten Kreis (gleicher Drehsinn)
      local ex, ey = Z[k + 1].x - Z[k].x, Z[k + 1].y - Z[k].y
      local el = math.sqrt(ex * ex + ey * ey)
      if el < 1e-6 then el = 1e-6 end
      ex, ey = ex / el, ey / el
      local rx, ry = ey * s, -ex * s                               -- Radius-Richtung: e um -s*90 Grad
      -- verschiedene Radien: Aussentangente kippt um asin((R1 - R2) / Abstand)
      local R2 = C[k + 1].R or R
      local cg = clamp((R - R2) / el, -0.95, 0.95)
      local sg = math.sqrt(1 - cg * cg)
      rx, ry = rx * sg + ex * cg, ry * sg + ey * cg
      local thOut = atan2(ry, rx)
      local arcB = arcAngle(thA[k], thOut, s)
      local stepsB = draw and math.max(1, math.min(math.floor(arcB * R / 8), math.ceil(arcB * 3.82))) or 0
      for j = 1, stepsB do
        local th = thA[k] + s * arcB * j / stepsB
        addPt(Z[k].x + R * math.cos(th), Z[k].y + R * math.sin(th), (L + arcB * R * j / stepsB) / V)
      end
      L = L + arcB * R
      if k == 1 then
        local qx, qy = Z[1].x + R * math.cos(thOut), Z[1].y + R * math.sin(thOut)
        res.exit = {x = qx + wx * L / V, y = qy + wy * L / V}
      end
      L = L + el * sg
      thIn = thOut
      addPt(Z[k + 1].x + R2 * rx, Z[k + 1].y + R2 * ry, L / V)
    end
    res.L, res.tAll = L, L / V
    if final then break end
    if prevAll and math.abs(res.tAll - prevAll) < 0.01 then final = true end
    prevAll = res.tAll
  end
  return res
end

-- Signierte Querablage (m) von (x,y) zur Linie a->b, positiv = rechts der Linie
local function crossTrack(ax, ay, bx, by, x, y)
  local dx, dy = bx - ax, by - ay
  local l = math.sqrt(dx * dx + dy * dy)
  if l < 1 then return 0 end
  return ((x - ax) * dy - (y - ay) * dx) / l
end

-- ---------------------------------------------------------------- Ethos-Anbindung

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

local function speedMs(src)
  local v = readNumber(src)
  if v == nil then return nil end
  local ok, u = pcall(src.unit, src)
  if not ok then u = nil end
  if u == "m/s" or (UNIT_METER_PER_SECOND and u == UNIT_METER_PER_SECOND) then return v end
  if u == "kn" or (UNIT_KNOT and u == UNIT_KNOT) then return v * 0.514444 end
  if u == "mph" or (UNIT_MPH and u == UNIT_MPH) then return v * 0.44704 end
  return v / KMH
end

local function widgetVisible()
  if not (lcd and lcd.isVisible) then return true end
  local ok, v = pcall(lcd.isVisible)
  return not ok or v ~= false
end

local function currentModelName()
  if model and model.name then
    local ok, name = pcall(model.name)
    if ok and type(name) == "string" then return name end
  end
  return nil
end

-- ---------------------------------------------------------------- Ton

-- page = Seite, zu der der Ton gehoert: kommt nur, wenn diese Seite angezeigt wird und das Widget auf dem
-- Bildschirm ist. nil = Ereignis (Wendeton, Sektor, Start, Ziel, Zone): kommt auf jeder Seite und auch,
-- wenn am Sender ein anderer Bildschirm offen ist (die Wertung laeuft ja weiter).
local function audible(w, page)
  if not w.sound then return false end
  if validSource(w.soundSw) and (readNumber(w.soundSw) or 0) <= 0 then return false end
  -- Wendevorhersage hat Vorrang vor allem: rund um die Wende (w.prioUntil, siehe turnTone) schweigen alle anderen
  -- Toene und Ansagen, nur die Wendetoene selbst (w.prioPlay) kommen durch
  if w.prioUntil and not w.prioPlay and os.clock() < w.prioUntil then return false end
  if page == nil then return true end            -- Ereignis: Aufrufer hat die Seite schon geprueft
  if not widgetVisible() then return false end
  return w.page == page
end

local function tone(w, page, freq, len)
  if not audible(w, page) then return end
  if not (system and system.playTone) then return end
  pcall(system.playTone, math.floor(freq + 0.5), math.floor(len + 0.5), 0)
end

local function haptic(w, page, ms)
  if not audible(w, page) then return end
  if not (system and system.playHaptic) then return end
  if not pcall(system.playHaptic, math.floor(ms), 0) then pcall(system.playHaptic, "-") end
end

-- Zahlen ansagen. Die Zahlen liegen als Systemdateien des Senders unter /audio/<sprache>/<stimme>/system/<n>.wav,
-- nicht im audio-Ordner des Widgets. Diese Dateien spielt das Widget direkt mit system.playFile (wie links.wav),
-- weil system.playNumber hier stumm blieb: UNIT_RAW gibt es in Ethos nicht (nil), dann wurde gar nichts gesagt.
-- Nur ganze Zahlen 0..999; findet das Widget den Systemordner nicht, bleibt playNumber mit UNIT_RAW/UNIT_NONE.
local sayNumber
do -- eigener Block (das Hauptkapitel ist am Limit von 200 lokalen Variablen)
local numDir
local function exists(path)
  if not (io and io.open) then return false end
  local ok, f = pcall(io.open, path, "r")
  if ok and f then pcall(f.close, f); return true end
  return false
end
local function findNumDir()
  if numDir ~= nil then return numDir or nil end
  numDir = false
  local langs, seen = {}, {}
  local function addLang(l)
    if type(l) == "string" and l ~= "" then
      l = l:lower():sub(1, 2)
      if not seen[l] then seen[l] = true; langs[#langs + 1] = l end
    end
  end
  if system and system.getLocale then local ok, l = pcall(system.getLocale); if ok then addLang(l) end end
  addLang("de"); addLang("en")
  for _, l in ipairs(langs) do
    for _, d in ipairs({"/audio/" .. l .. "/default/system/", "/audio/" .. l .. "/system/",
                        "/audio/" .. l .. "/us/system/", "/audio/" .. l .. "/gb/system/"}) do
      if exists(d .. "5.wav") then numDir = d; return d end
    end
  end
  return nil
end
local function playNum(d, n)
  -- 0..100 als eigene Datei, darueber Hunderter + Rest (Dateien 200.wav .. 900.wav, 100.wav)
  if n > 100 then
    local h, r = math.floor(n / 100) * 100, n % 100
    pcall(system.playFile, d .. string.format("%d.wav", h))
    if r > 0 then pcall(system.playFile, d .. string.format("%d.wav", r)) end
  else
    pcall(system.playFile, d .. string.format("%d.wav", n))
  end
end
function sayNumber(w, page, v, unit, dec)
  if not audible(w, page) then return end
  if not system then return end
  local n = math.floor((tonumber(v) or 0) + 0.5)
  local d = (dec or 0) == 0 and n >= 0 and n <= 999 and system.playFile and findNumDir()
  if d then playNum(d, n); return end
  if system.playNumber then
    unit = unit or UNIT_RAW or UNIT_NONE or 0
    if not pcall(system.playNumber, v, unit, dec or 0) then pcall(system.playNumber, v) end
  end
end
end

-- Optionale Sprachdateien (links.wav, rechts.wav, ...) im Ordner audio des Widgets
local audioFile
do -- Cache im eigenen Block (das Hauptkapitel ist am Limit von 200 lokalen Variablen)
local audioCache = {}
function audioFile(name)
  if audioCache[name] ~= nil then return audioCache[name] or nil end
  audioCache[name] = false
  if io and io.open then
    for _, d in ipairs(AUDIO_DIRS) do
      local ok, f = pcall(io.open, d .. name, "r")
      if ok and f then pcall(f.close, f); audioCache[name] = d .. name; break end
    end
  end
  return audioCache[name] or nil
end
end

local function sayWord(w, page, name)
  if not audible(w, page) then return false end
  local p = audioFile(name)
  if p and system and system.playFile then pcall(system.playFile, p); return true end
  return false
end

-- ---------------------------------------------------------------- IGC

-- Pruefsumme fuer den G-Record: in Gleitkomma gerechnet, damit sie in Ethos (Lua 5.4) und im
-- Simulator der App (fengari, 32-Bit-Ganzzahlen) gleich herauskommt
local function hashStr(h, s, k, mod)
  for i = 1, #s do
    h = (h * k + s:byte(i)) % mod
  end
  return h
end

local function igcLat(v)
  local a = math.abs(v)
  local d = math.floor(a)
  local m = math.floor((a - d) * 60000 + 0.5)
  if m >= 60000 then d, m = d + 1, m - 60000 end
  return string.format("%02d%05d%s", d, m, v >= 0 and "N" or "S")
end

local function igcLon(v)
  local a = math.abs(v)
  local d = math.floor(a)
  local m = math.floor((a - d) * 60000 + 0.5)
  if m >= 60000 then d, m = d + 1, m - 60000 end
  return string.format("%03d%05d%s", d, m, v >= 0 and "E" or "W")
end

-- Uhrzeit vom GPS: Quelle "GPS-Zeit" (Sekunden des Tages in UTC, Unix-Zeit oder Text hh:mm:ss). Ohne eigene
-- Quelle versucht das Widget eine Zeit-Option der GPS-Quelle. Gemerkt wird der Versatz zur Senderuhr, damit die
-- Zeit zwischen den Telemetrie-Werten weiterlaeuft.
local function gpsSecs(w)
  local src = validSource(w.gpsTime)
  if src then
    local v = readNumber(src)
    if v then return v end
    local ok, s = pcall(src.stringValue, src)
    if ok and type(s) == "string" then
      local h, mi, se = s:match("(%d+):(%d+):?(%d*)")
      if h then return tonumber(h) * 3600 + tonumber(mi) * 60 + (tonumber(se) or 0) end
    end
    return nil
  end
  if validSource(w.gps) then
    for _, name in ipairs({"OPTION_TIME", "OPTION_GPS_TIME", "OPTION_DATE_TIME"}) do
      local opt = rawget(_G, name)
      if opt then
        local ok, r = pcall(w.gps.value, w.gps, {options = opt})
        if ok and type(r) == "number" and r > 0 then return r end
      end
    end
  end
  return nil
end

-- Senderuhr (os.time). Ohne gueltige Uhr wirft os.time() auf dem Sender "time result cannot be represented":
-- dann weiter ab dem letzten guten Wert (oder ab 0) mit os.clock(). Die GPS-Zeit korrigiert das ohnehin.
local radioTime
do
  local base
  function radioTime()
    local ok, t = pcall(os.time)
    if ok and type(t) == "number" then base = t - os.clock(); return t, true end
    return math.floor((base or 0) + os.clock()), false
  end
end

local function updateGpsClock(w, now)
  local v = gpsSecs(w)
  if not v or v <= 0 then return end
  local radio = radioTime()
  local guess = radio - (w.utcOffset or 0) * 60
  local utc
  w.gpsFull = v > 1e8                                          -- GPS liefert auch das Datum
  if v > 1e8 then utc = v
  elseif v < 86400 then
    utc = guess - guess % 86400 + v
    if utc - guess > 43200 then utc = utc - 86400 elseif guess - utc > 43200 then utc = utc + 86400 end
  else return end
  w.gpsDelta, w.gpsClockT = utc - radio, now
end

local function gpsClock(w)
  return w.gpsDelta ~= nil and w.gpsClockT ~= nil and os.clock() - w.gpsClockT < 120
end

-- UTC-Zeit: vom GPS, wenn es eine Zeit liefert; sonst os.time() der Senderuhr minus Versatz (Minuten). Die App
-- schreibt den Versatz ihres Rechners in aufgaben.lua; in der Konfiguration laesst er sich ueberschreiben.
local function utcNow(w)
  local t = radioTime()
  if gpsClock(w) then return t + w.gpsDelta end
  return t - (w.utcOffset or 0) * 60
end

-- Ortszeit (fuer Anzeige und das von Hand eingestellte Startfenster)
local function localNow(w)
  return utcNow(w) + (w.utcOffset or 0) * 60
end

local function hms(utc)
  local s = math.floor(utc) % 86400
  return string.format("%02d%02d%02d", math.floor(s / 3600), math.floor(s / 60) % 60, s % 60)
end

local function ymd(utc)
  if os and os.date then
    local ok, t = pcall(os.date, "!*t", math.floor(utc))
    if ok and type(t) == "table" and t.year then return t.year, t.month, t.day end
  end
  return 1970, 1, 1
end

-- Zeilen kommen sofort in den Schreibpuffer; die Pruefsumme rechnet igcStep in kleinen Portionen nach,
-- damit kein Aufruf die Instruktionsgrenze von Ethos (20000) erreicht (der Kopf mit Vorlauf ist lang)
local function igcAdd(w, line)
  local L = w.igc
  if not L then return end
  line = line .. "\r\n"
  L.buf[#L.buf + 1] = line
  L.pend[#L.pend + 1] = line
end

-- Pruefsumme fuer hoechstens maxChars Zeichen weiterrechnen; true = alles gerechnet
local function igcHash(L, maxChars)
  while L.pi <= #L.pend and maxChars > 0 do
    local line = L.pend[L.pi]
    L.h1 = hashStr(L.h1, line, 31.0, 2147483647.0)
    L.h2 = hashStr(L.h2, line, 131.0, 2147483629.0)
    L.pend[L.pi] = nil
    L.pi = L.pi + 1
    maxChars = maxChars - #line
  end
  if L.pi > #L.pend then L.pend, L.pi = {}, 1; return true end
  return false
end

local function bRecord(w, f)
  local alt = math.floor((f.msl or 0) + 0.5)
  local pAlt = math.floor((f.pmsl or f.msl or 0) + 0.5)
  local sus = math.floor((f.clk % 1) * 100) % 100
  return string.format("B%s%s%s%s%05d%05d%02d", hms(f.utc), igcLat(f.lat), igcLon(f.lon), f.valid and "A" or "V", pAlt, alt, sus)
end

local function writeFile(path, mode, text)
  if not (io and io.open) then return false end
  local ok, f = pcall(io.open, path, mode)
  if not (ok and f) then return false end
  local okW = pcall(f.write, f, text)
  pcall(f.close, f)
  return okW
end

local function fileExists(path)
  if not (io and io.open) then return false end
  local ok, f = pcall(io.open, path, "r")
  if ok and f then pcall(f.close, f); return true end
  return false
end

local function igcFlush(w, L)
  L = L or w.igc
  if not L or #L.buf == 0 then return end
  local text = table.concat(L.buf)
  if writeFile(L.path, L.opened and "a" or "w", text) then
    L.opened = true
    L.buf = {}
    L.err = nil
  else
    L.err = "IGC: Schreiben fehlgeschlagen"
  end
end

local function rctLines(T)
  if T.rct then
    local out = {}
    for line in T.rct:gmatch("[^\r\n]+") do out[#out + 1] = line end
    return out
  end
  -- Aufgabe vom Systemtool: Richtung mit einer Nachkommastelle wie im QR-Code, Zonen als C/R-Zeilen
  local r = function(v) return math.floor(v + 0.5) end
  local function deg(v)
    local d10 = r(v * 10)
    if d10 % 10 == 0 then return string.format("%d", d10 // 10) end
    return string.format("%.1f", d10 / 10)
  end
  local out = {"V:02.0", string.format("T:%s,%.6f,%.6f,%d,%s,%d,%d,%d,%d,%d", (T.name:gsub(",", " ")), T.lat, T.lon,
    r(T.elev), deg(T.dir), r(T.dist), r(T.maxH), r(T.maxV), r(T.minF), r(T.work))}
  for i, z in ipairs(T.zones or {}) do
    local nm = "Zone " .. i
    if z.t == "R" then
      out[#out + 1] = string.format("R:%s,%.6f,%.6f,%s,%d,%d,%d,%d", nm, z.lat, z.lon, deg(z.dir), r(z.len), r(z.wid), r(z.lo), r(z.hi))
    else
      out[#out + 1] = string.format("C:%s,%.6f,%.6f,%d,%d,%d", nm, z.lat, z.lon, r(z.r), r(z.lo), r(z.hi))
    end
  end
  return out
end

local function igcOpen(w, f)
  local T = w.task
  if w.igc or not T then return end
  local y, mo, d = ymd(f.utc)
  local date = string.format("%04d-%02d-%02d", y, mo, d)
  -- erster Ordner, in den sich schreiben laesst; Nummer = erster freier Flug des Tages
  local path, nr = nil, 1
  for _, dd in ipairs(FLIGHT_DIRS) do
    for k = 1, 99 do
      local p = string.format("%s%s-XSN-SKYNAV-%02d.IGC", dd, date, k)
      if not fileExists(p) then
        if writeFile(p, "w", "") then path, nr = p, k end
        break
      end
    end
    if path then break end
  end
  path = path or string.format("%s-XSN-SKYNAV-99.IGC", date)
  w.igc = {path = path, buf = {}, pend = {}, pi = 1, h1 = 7.0, h2 = 13.0, opened = false, lastFlush = f.clk}
  local dd2 = string.format("%02d%02d%02d", d, mo, y % 100)
  local ver = (system and system.getVersion) and system.getVersion() or {}
  igcAdd(w, "AXSNSKYNAV")
  igcAdd(w, string.format("HFDTEDATE:%s,%02d", dd2, nr))
  igcAdd(w, "HFPLTPILOT:" .. (w.pilot or ""))
  igcAdd(w, "HFGTYGLIDERTYPE:" .. string.sub(w.modelName or "", 1, 30))
  igcAdd(w, "HFGIDGLIDERID:" .. (w.gliderId or ""))
  igcAdd(w, "HFDTMGPSDATUM:WGS84")
  igcAdd(w, "HFRFWFIRMWAREVERSION:SkyNavigator " .. VERSION)
  igcAdd(w, "HFRHWHARDWAREVERSION:Ethos " .. tostring(ver.board or "") .. " " .. tostring(ver.version or ""))
  igcAdd(w, "HFFTYFRTYPE:Maechler Modelle,SkyNavigator Ethos-Widget")
  igcAdd(w, "HFGPSRECEIVER:" .. (w.gpsName or "Telemetrie-GPS") .. ",,max10000m")
  igcAdd(w, "HFCIDCOMPETITIONID:" .. (w.compId or ""))
  igcAdd(w, "HFCCLCOMPETITIONCLASS:" .. string.upper(T.cls or ""))
  igcAdd(w, "I013637SUS")
  igcAdd(w, "LPilotID:" .. (w.pilotId or ""))
  igcAdd(w, "LProtocolVersion02.0")
  for _, l in ipairs(rctLines(T)) do igcAdd(w, "LTSK:" .. l) end
  local function cRec(x, y, name)
    local lat = T.lat + y / M_PER_DEG
    local lon = T.lon + x / (M_PER_DEG * T.cosLat)
    igcAdd(w, "C" .. igcLat(lat) .. igcLon(lon) .. name)
  end
  igcAdd(w, string.format("C%s000000%s000003RC GPS Triangle Racing", dd2, dd2))
  local nm = string.upper(T.name)
  cRec(0, 0, nm)
  cRec(0, 0, "START")
  for i = 1, 3 do cRec(T.tp[i].x, T.tp[i].y, "TURN POINT " .. i) end
  cRec(0, 0, "FINISH")
  cRec(0, 0, nm)
  -- Vorlauf (Ringpuffer der letzten 90 s)
  for _, b in ipairs(w.ring) do igcAdd(w, b) end
  w.ring = {}
  igcFlush(w)
  w.igcSkip = true                                                 -- Pruefsumme erst ab dem naechsten Aufruf
end

local function igcEvent(w, code, f)
  if not w.igc then return end
  igcAdd(w, "E" .. hms(f.utc) .. code)
  igcAdd(w, bRecord(w, f))
end

-- Schliessen: Datei kommt in die Warteschlange, igcStep schreibt den G-Record, sobald die Pruefsumme steht
local function igcClose(w)
  local L = w.igc
  if not L then return end
  w.igcQ = w.igcQ or {}
  w.igcQ[#w.igcQ + 1] = L
  w.igc = nil
end

-- pro wakeup: Pruefsummen in Portionen, fertig gerechnete Dateien abschliessen
local function igcStep(w)
  if w.igcSkip then w.igcSkip = nil; return end
  local budget = 400
  local Q = w.igcQ
  if Q and Q[1] then
    local L = Q[1]
    if igcHash(L, budget) then
      L.buf[#L.buf + 1] = string.format("G%08X%08X", math.floor(L.h1), math.floor(L.h2)) .. "\r\n"
      igcFlush(w, L)
      w.lastIgc = L.path
      table.remove(Q, 1)
    end
    return
  end
  if w.igc then igcHash(w.igc, budget) end
end

-- ---------------------------------------------------------------- Zustand

local SOURCE_KEYS = {"gps", "gpsSpd", "gpsAlt", "relAlt", "vario", "swOver", "swStart", "swGlide", "swTherm", "swEval", "armSw", "soundSw", "mcSrc", "gpsTime"}
local NUM_DEFS = {
  {key = "taskIdx",   def = 0},     -- 0 = in der App als aktiv markierte Aufgabe
  {key = "altMode",   def = 1},     -- 1 = GPS minus Platzhoehe, 2 = relative Hoehe (Sensor)
  {key = "varioType", def = 1},     -- 1 = TE vom Sensor, 2 = barometrisch (Widget kompensiert mit der Fahrt, BAL.TE)
  {key = "radMode",   def = 1},     -- ungenutzt (Wenderadius wird pro Modell gelernt), Platz fuer die Storage-Reihenfolge
  {key = "radFix",    def = 30},    -- ungenutzt
  {key = "margin",    def = 5},     -- m Reserve im Sektor (Logger 12 Hz: 5 m reichen fuer sichere Treffer)
  {key = "warnSec",   def = 2},     -- Wendeton: Countdown (s) vor "Jetzt wenden" am Kurvenbeginn
  {key = "lead",      def = 5},     -- Vorhalt Kurveneinleitung 0.1 s
  {key = "cmdMin",    def = 10},    -- Kommando ab x Grad
  {key = "cmdGap",    def = 8},     -- Kommando höchstens alle x s
  {key = "zoomSec",   def = 10},    -- Eckansicht x s vor dem Wendepunkt
  {key = "winLo",     def = 10},    -- Einflugfenster: Limit minus x m
  {key = "speedMargin", def = 3},   -- Vibration ab Limit minus x km/h
  {key = "mc",        def = 10},    -- 0.1 m/s
  {key = "mcMax",     def = 30},
  {key = "ballast",   def = 100},
  {key = "bugs",      def = 0},
  {key = "dead",      def = 3},
  {key = "utcOver",   def = -999},  -- Versatz Senderuhr - UTC in min, -999 = aus der App
  {key = "slotH",     def = -1},    -- Startfenster von Hand: Stunde (Senderuhr), -1 = aus der Aufgabe
  {key = "slotM",     def = 0},
  {key = "slotLen",   def = 10},
  {key = "startToneLogic", def = 1}, -- 1 = zu tief gibt hohe Toene, 2 = zu hoch gibt hohe Toene
  {key = "startSpan", def = 400},   -- m, Start-Seite: Karte von der Linie bis zum linken Rand (fester Massstab)
  {key = "trackMin",  def = 3},     -- Uebersicht: Flugweg der letzten x Minuten (0 = keiner)
  {key = "glideF1",   def = 1},     -- Seite Gleiten, Felder unten (App): 0 leer, 1 TP, 2 Wenden in, 3 Hoehe,
  {key = "glideF2",   def = 2},     -- 4 Hoehe Ziellinie, 5 Rest, 6 Runde, 7 letzte Runde, 8 Grund, 9 Gleitzahl,
  {key = "glideF3",   def = 4},     -- 10 Vario 30 s, 11 Gegenwind, 12 Seitenwind, 13 Ablage, 14 Dreiecke, 15 Radius,
                                    -- 16 Zeit-MC, 17 Endanflug ab
  {key = "thermF1",   def = 1},     -- Seite Thermik, vier Werte (App): 0 leer, 1 Vario, 2 Vario 30 s, 3 Thermik,
  {key = "thermF2",   def = 3},     -- 4 Gewinn, 5 Hoehe, 6 Netto Thermik, 7 Reserve Maximalhoehe, 8 Rest,
  {key = "thermF3",   def = 4},     -- 9 Dreiecke, 10 Zeit-MC, 11 Radius, 12 Wind
  {key = "thermF4",   def = 5},
  {key = "rzRes",     def = 3},     -- Restzeit-Sollfahrt: Zeitreserve % der Restzeit (mind. 15 s)
  {key = "rzPilot",   def = 3},     -- Pilotfaktor % (laengerer Weg auf den letzten zwei Runden)
  {key = "rzHmin",    def = 60},    -- m, tiefste Hoehe, auf der man noch gleiten will
}
local BOOL_DEFS = {
  {key = "sound", def = true},
  {key = "eventsAlways", def = false},
  {key = "stfTone", def = true},
  {key = "navVoice", def = true},
  {key = "varioTone", def = true},    -- Varioton des Widgets (dann Vario-Ton des Senders aus)
  {key = "startTone", def = true},
  {key = "autoPage", def = true},
  {key = "rzOn", def = true},        -- Restzeit-Sollfahrt (Zeit-MC) statt festem MC
}
local TEXT_DEFS = {
  {key = "compId", def = ""},
  {key = "gliderId", def = ""},
}
-- Ballast (am Sender): Poti 0..100 % und Ballast maximal in 0.1 kg. Steht im Speicher nach den Texten,
-- damit aeltere Konfigurationen ihre Reihenfolge behalten.
local BAL = {}

-- Querlage (Rollwinkel vom Lagesensor, z. B. Stabi-Empfaenger): rechts haengend positiv. Daraus die Drehrate
-- omega = g * tan(phi) / v; das Widget rechnet vor, wie weit sich der Kurs in tau s (qlTau, App) noch dreht, und
-- zeigt nur die Restkorrektur. Ohne Sensor gilt die Drehrate aus dem GPS-Kurs (traeger, tau hoechstens 1 s).
BAL.QL = {}

-- Durchgehender tiefer Ton (Sinkton, Sollfahrt zu schnell, Einflug zu hoch): Stuecke von 250 ms, das naechste
-- wird 80 ms vor dem Ende des laufenden bestellt. endKey = Feld in w mit dem Ende des zuletzt bestellten Stuecks.
-- Liefert die Zeit fuer die naechste Bestellung.
function BAL.hold(w, page, f, now, endKey)
  local e = math.max(now, w[endKey] or 0) + 0.25
  tone(w, page, f, 250)
  w[endKey] = e
  return e - 0.08
end

-- Kompensiertes Vario (Vario-Art "barometrisch"): TE = Vario + d(TAS^2 / 2g)/dt. Die Fahrt kommt pro GPS-Fix
-- (GPS plus Wind, ohne Wind die Fahrt ueber Grund); die Aenderung der Energiehoehe ueber rund 1 s, auf +-4 m/s
-- begrenzt und 0.5 s geglaettet (wie die App, aber nur rueckwaerts gerechnet). Am Boden (unter 5 m/s) keine.
-- Mit Staudruck-Fahrt (BAL.AS) laeuft es in jedem wakeup mit Basis 0.5 s und Glaettung 0.3 s statt pro GPS-Fix.
function BAL.TE(w, now, tas, base)
  base = base or 1.0
  local h = w.teHist
  if not h or (#h > 0 and now - h[#h].t > 3) then h = {}; w.teHist, w.teDE, w.teT = h, nil, nil end
  if not tas or tas < 5 then w.teDE = 0; return end
  h[#h + 1] = {t = now, e = tas * tas / (2 * G)}
  while #h > 2 and now - h[2].t >= base do table.remove(h, 1) end
  local a, b = h[1], h[#h]
  if b.t - a.t >= base / 2 then
    local de = clamp((b.e - a.e) / (b.t - a.t), -4, 4)
    w.teDE = lpf(w.teDE, de, now - (w.teT or now), base < 1 and 0.3 or 0.5)
    w.teT = now
  end
end                                                         -- in BAL, weil das Hauptkapitel am Limit von 200 lokalen Variablen ist

-- Einstellungen aus der App (aufgaben.lua, Tabelle settings): Schluessel, Bereich. Am Sender stehen nur noch
-- Aufgabe, Sensoren, Schalter und das Startfenster von Hand.
local APP_SETTINGS = {
  sound = true, eventsAlways = true, startTone = true, stfTone = true, navVoice = true, varioTone = true, autoPage = true,
  winLo = {2, 50}, speedMargin = {0, 20}, cmdMin = {3, 45}, cmdGap = {3, 60}, startToneLogic = {1, 2}, startSpan = {100, 2000}, glideF1 = {0, 17}, glideF2 = {0, 17}, glideF3 = {0, 17},
  thermF1 = {0, 12}, thermF2 = {0, 12}, thermF3 = {0, 12}, thermF4 = {0, 12},
  margin = {0, 100}, warnSec = {0, 10}, lead = {0, 30}, zoomSec = {3, 60}, zoomAfter = {0, 30}, mc = {0, 50}, mcMax = {10, 100},
  ballast = {50, 300}, bugs = {0, 30}, dead = {0, 20}, compId = "", gliderId = "",
  rzOn = true, rzRes = {0, 20}, rzPilot = {0, 20}, rzHmin = {0, 300},
  qlTau = {0, 30}, qlTrim = true, qaTau = {0, 60}, qaRate = {5, 300}, vzLo = {0, 20}, vzHi = {0, 20}, vDamp = {0, 20}, vtRange = {3, 20}, vtGlide = {0, 30},
  thBest = true,
}

local function applyAppSettings(w, st)
  if type(st) ~= "table" then return end
  local changed = false
  for k, def in pairs(APP_SETTINGS) do
    local v = st[k]
    if type(def) == "table" and type(v) == "number" then v = math.floor(clamp(v, def[1], def[2]) + 0.5)
    elseif def == true and type(v) ~= "boolean" then v = nil
    elseif type(def) == "string" then v = type(v) == "string" and v:sub(1, 20) or nil
    elseif type(def) == "table" then v = nil end
    if v ~= nil and w[k] ~= v then w[k] = v; changed = true end
  end
  if changed then w.polarT = nil; if not w.mcSrc then w.mcValue = w.mc / 10 end end
end

local function resetFlight(w)
  if w.igc then igcClose(w) end
  w.state = S_IDLE
  w.flying, w.landedT, w.slowSince = false, nil, nil
  w.tp, w.laps, w.startT, w.startUtc, w.lapT = 0, {}, nil, nil, nil
  w.cornerHold, w.badEntry, w.cornerAfter = nil, nil, nil
  w.entry, w.penalty, w.zoneHit, w.altHit = nil, 0, false, false
  w.lastFix, w.lastInStart, w.stopReason = nil, nil, nil
  w.trail, w.ring = {}, {}
  w.track, w.marks = {}, {}
  w.ref, w.tpT, w.tpPrev = nil, nil, nil
  w.therm = nil
  w.rz, w.rzPh = nil, nil -- w.rzTap (MC ZEIT gewaehlt) bleibt: die Wahl des Piloten gilt weiter
  w.evalScroll = 0
  w.msg, w.msgT = nil, nil
end

local function create()
  local w = {win = {}, lastInvalidate = 0, page = P_OVER, mcValue = 1.0, qlTau = 15, qlTrim = true, qaTau = 25, qaRate = 30, zoomAfter = 5, vzLo = 3, vzHi = 1, vtStyle = 1, vtDev = 1, vDamp = 4, vtRange = 10, vtGlide = 5}
  for _, d in ipairs(NUM_DEFS) do w[d.key] = d.def end
  for _, d in ipairs(BOOL_DEFS) do w[d.key] = d.def end
  for _, d in ipairs(TEXT_DEFS) do w[d.key] = d.def end
  resetFlight(w)
  return w
end

local function note(w, text, now)
  w.msg, w.msgT = text, now
end

-- ---------------------------------------------------------------- Aufgaben-Datei

-- feld.lua (Systemtool) alle 3 s als Text vergleichen: nach dem Schliessen des Tools soll die Aufgabe gleich da sein
function FELD.changed(w, now)
  if w.feldT and now - w.feldT < (w.feldText == "" and 10 or 3) and now >= w.feldT then return false end
  w.feldT = now
  local text = ""
  for _, path in ipairs(FELD.paths) do
    local okOpen, f = pcall(io.open, path, "r")
    if okOpen and f then
      local okRead, s = pcall(f.read, f, "*a")
      pcall(f.close, f)
      if okRead and type(s) == "string" then text = s; break end
    end
  end
  if text == w.feldText then return false end
  w.feldText = text
  return true
end

function FELD.table(w)
  if not w.feldText or w.feldText == "" or not load then return nil end
  local okLoad, c = pcall(load, w.feldText, "=feld.lua", "t", {})
  if not okLoad or type(c) ~= "function" then return nil end
  local ok, t = pcall(c)
  return ok and type(t) == "table" and t or nil
end

local function refreshTasks(w, now, force)
  local feldNew = FELD.changed(w, now)
  if not force and not feldNew and w.tasksT and now - w.tasksT < 20 and now >= w.tasksT then return end
  w.tasksT = now
  local t = loadLuaTable(TASK_PATHS)
  local list = {}
  w.tasksFound = t ~= nil
  if t then
    for _, e in ipairs(type(t.tasks) == "table" and t.tasks or {}) do
      local T = buildTask(e)
      if T then list[#list + 1] = T end
    end
    w.utcFile = tonumber(t.utcOffsetMin)
    w.pilot = type(t.pilot) == "string" and t.pilot or w.pilot
    w.pilotId = type(t.pilotId) == "string" and t.pilotId or w.pilotId
    w.activeId = t.active and tostring(t.active) or nil
    applyAppSettings(w, t.settings)
  end
  -- Aufgaben vom Systemtool; dessen aktive Aufgabe gilt, wenn sie neuer ist als die Datei der App
  local fd = FELD.table(w)
  w.feldSlot = nil
  if fd then
    for _, e in ipairs(type(fd.tasks) == "table" and fd.tasks or {}) do
      local T = buildTask(e)
      if T then T.feld = true; list[#list + 1] = T end
    end
    local off = (w.utcOver ~= -999) and w.utcOver or (w.utcFile or 0)
    local fw, aw = tonumber(fd.written), t and tonumber(t.written)
    if fd.active and fd.active ~= "" and (not aw or not fw or fw - off * 60 >= aw) then w.activeId = tostring(fd.active) end
    if type(fd.slot) == "table" and tonumber(fd.slot.h) and tonumber(fd.slot.y) then w.feldSlot = fd.slot end
  end
  w.tasks = list
  local idx = w.taskIdx or 0
  if idx == 0 then
    idx = 1
    for i, T in ipairs(list) do if T.id == w.activeId then idx = i end end
  end
  idx = clamp(idx, 1, math.max(#list, 1))
  local T = list[idx]
  local sig = T and (T.id .. T.lat .. T.lon .. T.dir .. T.dist .. T.elev .. T.maxH .. T.maxV .. T.minF .. T.work .. #T.zones
    .. tostring(T.slot and T.slot.h)) or "-"
  if sig ~= w.taskSig then
    w.taskSig = sig
    -- Aufgabe gewechselt: nur ohne laufenden Flug uebernehmen
    if w.state ~= S_STARTED then w.task = T end
  end
  w.utcOffset = (w.utcOver ~= -999) and w.utcOver or (w.utcFile or 0)
end

-- Wenderadius pro Modell: aus den Wenden ausserhalb der Thermik gelernt und in radien.lua gespeichert
local function loadRadii(w)
  local t, path = loadLuaTable(RADIUS_PATHS)
  w.radii, w.radiiPath = {}, path
  if type(t) == "table" then
    for k, v in pairs(t) do
      if type(k) == "string" and type(v) == "table" then
        local m = {}
        for _, key in ipairs({"all", "tp1", "tp2", "tp3"}) do
          local r = v[key]
          if type(r) == "number" and r >= 5 and r <= 200 then m[key] = r end
        end
        if type(v.ql) == "number" and math.abs(v.ql) <= 45 then m.ql = v.ql end   -- Querlage-Nullpunkt (Grad)
        if type(v.qk) == "number" and math.abs(v.qk) >= 5 and math.abs(v.qk) <= 300 then m.qk = v.qk end   -- Rollrate Querruder (Grad/s voll)
        if type(v.tv) == "number" and v.tv >= 0 and v.tv <= 3 then m.tv = v.tv end   -- Vorhalt fuer "Jetzt wenden" (s)
        w.radii[k] = m
      end
    end
  end
end

local function saveRadii(w)
  local keys = {}
  for k in pairs(w.radii) do keys[#keys + 1] = k end
  table.sort(keys)
  local out = {"-- Wenderadien pro Modell (m). Das Widget lernt sie selbst aus den Wenden an den Ecken.\n-- ql = Nullpunkt der Querlage (Grad), gesetzt mit \"Querlage nullen\".\n-- qk = Rollrate bei vollem Querruder (Grad/s), gelernt mit Lagesensor, fuer Fluege ohne.\n-- tv = Vorhalt (s) fuer \"Jetzt wenden\", gelernt aus der Sektortiefe nach jeder Wende.\nreturn {\n"}
  for _, k in ipairs(keys) do
    local m, parts = w.radii[k], {}
    for _, key in ipairs({"all", "tp1", "tp2", "tp3", "ql", "qk", "tv"}) do
      if m[key] then parts[#parts + 1] = string.format(key == "tv" and "%s = %.2f" or "%s = %.1f", key, m[key]) end
    end
    out[#out + 1] = string.format("  [%q] = {%s},\n", k, table.concat(parts, ", "))
  end
  out[#out + 1] = "}\n"
  local text = table.concat(out)
  local paths = w.radiiPath and {w.radiiPath} or RADIUS_PATHS
  for _, path in ipairs(paths) do
    if writeFile(path, "w", text) then w.radiiPath = path; return true end
  end
  return false
end

-- Neue Wende mit Radius r bei (x, y): fliesst in den Modellwert und, nahe an einem TP, in dessen Wert
local function learnRadius(w, r, x, y)
  if not w.radii then return end
  local name = w.modelName or "?"
  local m = w.radii[name] or {}
  w.radii[name] = m
  local function mix(key)
    local old = m[key]
    m[key] = math.floor(((old and (0.7 * old + 0.3 * r)) or r) * 10 + 0.5) / 10
  end
  -- nur Wenden an einem TP zaehlen (nicht Kurven beim Anflug oder vor dem Start irgendwo)
  local T = w.task
  if not (T and x) then return end
  local best, bj = 150, nil
  for j = 1, 3 do
    local d = math.sqrt((x - T.tp[j].x) ^ 2 + (y - T.tp[j].y) ^ 2)
    if d < best then best, bj = d, j end
  end
  if not bj then return end
  mix("all")
  mix("tp" .. bj)
  w.radiiDirty = true
end

-- Ballast in kg: Poti-Anteil mal Ballast maximal, ohne Poti fix Ballast maximal. Liefert kg und Anteil (oder nil)
function BAL.now(w)
  local mx = (w.balMax or 0) / 10
  if mx <= 0 then return 0, nil end
  local v = readNumber(w.balSrc)
  if not v then return mx, nil end
  local fr = clamp((v + 1024) / 2048, 0, 1)
  return fr * mx, fr
end

-- Polare fuer das aktuelle Gewicht: Leergewicht der Polare (mass, kg, aus der App) plus Ballast; Fahrt und
-- Sinken mal sqrt(Gewicht / Leergewicht). Ohne mass gilt wie bisher das Fluggewicht in % aus der App.
function BAL.polar(w)
  local e = w.polarE
  local m0 = e and type(e.mass) == "number" and e.mass > 0 and e.mass or nil
  w.polarMass = m0
  local fac = clamp(m0 and (m0 + (w.balKg or 0)) / m0 or w.ballast / 100, 0.5, 4)   -- 1.41: Gewichtsfaktor plausibel
  w.polar = e and polarFromAbc(e.a, e.b, e.c, e.vMin, e.vMax, fac, w.bugs / 100) or nil
end

local function refreshPolar(w, now, force)
  local name = currentModelName()
  if name ~= w.modelName or not w.radii then w.modelName = name; loadRadii(w) end
  if w.radiiDirty and (not w.radiiSaveT or now - w.radiiSaveT > 20) then
    w.radiiDirty, w.radiiSaveT = false, now
    saveRadii(w)
  end
  if not force and name == w.polarModel and w.polarT and now - w.polarT < 30 then return end
  w.polarModel, w.polarT = name, now
  local all = loadLuaTable(POLAR_PATHS)
  local e = all and findByName(all, name or "")
  w.polarE = e
  BAL.polar(w)
end

-- Startfenster als UTC-Epoche: vom Systemtool (Ortszeit, gilt bis 6 h nach dem Ende), von Hand (Konfiguration) oder
-- aus der Aufgabe (UTC). Reglement: die letzte Minute zaehlt ganz, also Ende = Beginn + Laenge + 59 s.
local function slotWindow(w)
  local fs = w.feldSlot
  if fs then
    local a = slotEpoch(fs)
    if a then
      a = a - (w.utcOffset or 0) * 60
      local b = a + num(fs.len, 10) * 60 + 59
      if utcNow(w) < b + 6 * 3600 then return a, b, "sender" end
    end
  end
  if w.slotH >= 0 then
    local now = localNow(w)
    local t0 = now - now % 86400 + w.slotH * 3600 + w.slotM * 60 - (w.utcOffset or 0) * 60
    return t0, t0 + w.slotLen * 60 + 59, "hand"
  end
  local T = w.task
  if not T or not T.slot then return nil end
  local a, b = slotEpoch(T.slot)
  if not a then return nil end
  return a, b + 59, "aufgabe"
end

-- ---------------------------------------------------------------- Messwerte

-- Fahrt vom Staudruck-Sensor (Quelle "Fahrt Staudruck", z.B. SM GPS-Logger 3 mit Prandtl-Rohr, Wert 0A0 Airspeed
-- in Knoten). Gemessen wird die angezeigte Fahrt (IAS), die wahre ist um die Luftdichte groesser:
-- TAS = IAS / sqrt(sigma), sigma = (1 - 2.2558e-5 * h)^4.2559 (Normatmosphaere, h ue. M.); auf 500 m +2.5 %,
-- auf 1500 m +7.5 %. Gilt (w.asOk), solange der Wert sich innert 5 s aendert und nicht unter 5 m/s liegt, waehrend
-- das GPS ueber 12 m/s zeigt (Rohr verstopft, Schlauch ab). Dann ersetzt sie die Fahrt aus GPS und Wind ueberall
-- (TAS, Sollfahrt IST, Netto, Querlage aus der Drehrate) und gibt die TE-Kompensation (Vario-Art barometrisch)
-- in jedem wakeup. Plausibilitaet: im Flug ist das Mittel der Fahrt etwa das Mittel der GPS-Geschwindigkeit
-- (Wind mittelt sich in Kreisen und Schenkeln weg); nach 2 min ausserhalb 0.6..1.7 kommt "Fahrt-Sensor pruefen".
BAL.AS = {}
-- km/h-Einheit erkennen, auch wenn die Konstante je nach Ethos-Version anders heisst (wie Polarflug findKmhUnit):
-- vorher fiel eine Fahrt in km/h (SM-Logger, Ethos zeigt km/h) ohne passende Konstante auf Knoten (x1.85 zu hoch)
function BAL.isKmh(un)
  if un == nil or un == "km/h" then return un == "km/h" end
  if BAL.kmhU == nil then
    BAL.kmhU = false
    for k, v in pairs(_G) do
      if type(k) == "string" and k:sub(1, 5) == "UNIT_" and (k:find("KPH") or k:find("KMH") or k:find("KM_PER") or k:find("KILOMETER_PER") or k:find("KMPH")) then
        BAL.kmhU = v; break
      end
    end
  end
  return BAL.kmhU ~= false and un == BAL.kmhU
end
function BAL.AS.read(w, now, dt)
  local src = validSource(w.airSrc)
  local v = src and readNumber(src)
  if not v then w.asOk, w.asTas = false, nil; return end
  local u = w.airUnit or 1
  if u == 1 then
    local ok, un = pcall(src.unit, src)
    if not ok then un = nil end
    local function is(name, txt) local c = rawget(_G, name); return (c ~= nil and un == c) or un == txt end
    if is("UNIT_METER_PER_SECOND", "m/s") then u = 4
    elseif is("UNIT_KPH", "km/h") or is("UNIT_KMH", "km/h") or is("UNIT_KILOMETER_PER_HOUR", "km/h") or BAL.isKmh(un) then u = 2
    elseif is("UNIT_MPH", "mph") then u = 5
    else u = 3 end                                            -- Knoten oder ohne Einheit (DIY-Sensor): Knoten
  end
  local ias = v * (u == 2 and 1 / KMH or u == 3 and 0.514444 or u == 5 and 0.44704 or 1)
  if v ~= w.asRaw then w.asRaw, w.asChg = v, now end
  local f = w.cur
  local h = f and f.msl or 0
  local sigma = (1 - 2.2558e-5 * clamp(h, -500, 6000)) ^ 4.2559
  local tas = ias / math.sqrt(sigma)
  w.asTas = lpf(w.asTas, tas, dt, 0.2)
  local gs = f and f.gs or 0
  w.asOk = now - (w.asChg or now) < 5 and not (tas < 5 and gs > 12)
  if w.asOk and w.flying and gs > 8 then
    w.asSa, w.asSg = (w.asSa or 0) + tas * dt, (w.asSg or 0) + gs * dt
    w.asN = (w.asN or 0) + dt
    if w.asN > 120 and not w.asWarned then
      local r = w.asSa / math.max(w.asSg, 1)
      if r < 0.6 or r > 1.7 then w.asWarned = true; note(w, string.format("Fahrt-Sensor pruefen (x%.1f GPS)", r), now) end
    end
  end
  if w.asOk then BAL.TE(w, now, w.asTas, 0.5) end
end

-- GPS-Satelliten (Quelle optional): Warnung in der Kopfzeile unter 6
function BAL.satText(w)
  local n = readNumber(w.satSrc)
  if n and w.gpsOk and n < 6 then return string.format("  GPS %d SAT", math.floor(n + 0.5)) end
  return nil
end

-- ---------------------------------------------------------------- Sensoren automatisch (1.30)
-- Statt jede Quelle einzeln zu waehlen, gibt man in der Konfiguration die angeschlossenen Geraete an:
-- devMain 0 = keines (alle Quellen von Hand), 1 = SM GPS-Logger 3, 2 = FrSky Vario (dazu ein FrSky-GPS, falls
-- vorhanden); devStabi = FrSky-Stabi-Empfaenger zusaetzlich (Querlage, Nickwinkel, Beschleunigung). Das Widget
-- sucht die Telemetrie-Quellen selbst: zuerst ueber die S.Port-AppID mit Instanz 0..15 (sprachunabhaengig),
-- dann ueber die Ethos-Namen (deutsch/englisch). Mit Logger UND Stabi muss der Logger laut SM eine andere
-- Instanz haben (z.B. 1), darum wird dann bei Vario/Hoehe die hoechste Instanz genommen.
-- Eine Quelle "von Hand" (Konfiguration, Einzelne Quellen) geht immer vor. Gibt es fuer das Modell
-- scripts/tekkal/korrektur_<Modell>.txt (TEK-Kalibrierung), gelten mit dem Logger "TEK korr." als Vario und
-- "Fahrt korr." als Fahrt. Vario-Art "Automatisch": Logger (TEK-Duese) und TEK korr. = TE, FrSky Vario =
-- barometrisch. Gesucht wird beim Lesen und Oeffnen der Konfiguration sowie im Flug eine Rolle pro wakeup,
-- alle 10 s (fehlt etwas) bzw. 30 s.
BAL.SN = {
  KEYS = {"gps", "gpsSpd", "gpsAlt", "gpsTime", "satSrc", "crsSrc", "relAlt", "vario", "airSrc", "bankSrc", "pitchSrc", "accZSrc", "accXSrc"},
  ROLE = {
    gps      = {dev = "gps",   lbl = "GPS",      ids = {0x0800}, names = {"GPS"}},
    gpsSpd   = {dev = "gps",   lbl = "Geschw.",  ids = {0x0830}, names = {"GPS-Geschw.", "GSpd", "GPS Speed"}},
    gpsAlt   = {dev = "gps",   lbl = "GPS-Höhe", ids = {0x0820}, names = {"GAlt", "GPS-Höhe", "GPS Alt"}},
    gpsTime  = {dev = "gps",   lbl = "Zeit",     ids = {0x0850}, names = {"GPS-Zeit", "GPS Clock", "GPS Time"}},
    satSrc   = {dev = "gps",   lbl = "Sats",     ids = {0x0860}, names = {"Satelliten", "Sats", "GPS Sats"}},
    crsSrc   = {dev = "frgps", lbl = "Kurs",     ids = {0x0840}, names = {"GPS-Kurs", "GPS Course", "Hdg"}},  -- 1.40: nicht beim SM-Logger
    relAlt   = {dev = "baro",  lbl = "Höhe",     ids = {0x0100}, names = {"Altitude", "Flughöhe", "Höhe"}},
    vario    = {dev = "baro",  lbl = "Vario",    ids = {0x0110}, names = {"Steigr. Vario", "VSpeed", "Vario"}, korr = "TEK korr."},
    airSrc   = {dev = "sm",    lbl = "Fahrt",    ids = {0x0A00}, names = {"Airspeed", "ASpd"}, korr = "Fahrt korr."},
    bankSrc  = {dev = "stabi", lbl = "Querlage", names = {"R.Winkel", "R.Angle", "Roll", "Rollwinkel"}},
    pitchSrc = {dev = "stabi", lbl = "Nick",     names = {"P.Winkel", "P.Angle", "Pitch", "Nickwinkel"}},
    accZSrc  = {dev = "acc",   lbl = "AccZ",     ids = {0x0720}, names = {"AccZ", "Acc Z"}},
    accXSrc  = {dev = "acc",   lbl = "AccX",     ids = {0x0700}, names = {"AccX", "Acc X"}},
  },
  KORR_DIRS = {"/scripts/tekkal/", "scripts/tekkal/", "../tekkal/"},
}

-- Methode einer Ethos-Quelle sicher aufrufen (nil, wenn es sie nicht gibt)
function BAL.SN.call(s, m)
  local okF, f = pcall(function() return s[m] end)
  if not (okF and type(f) == "function") then return nil end
  local ok, v = pcall(f, s)
  if ok then return v end
  return nil
end

-- echte Quelle? (Ethos liefert fuer ein leeres Feld je nach Version nil, false oder eine Quelle "---")
function BAL.SN.isSet(s)
  if not validSource(s) then return false end
  local none = rawget(_G, "CATEGORY_NONE")
  if none ~= nil and BAL.SN.call(s, "category") == none then return false end
  local n = BAL.SN.call(s, "name")
  if n == "" or n == "---" then return false end
  return true
end

function BAL.SN.on(w) return (w.devMain or 0) >= 1 or w.devStabi == true end

-- Quelle ueber AppID (Zahl) oder Namen suchen; tele = nur Telemetrie-Sensoren annehmen
function BAL.SN.get(spec, tele)
  if not (system and system.getSource) then return nil end
  local cat = rawget(_G, "CATEGORY_TELEMETRY_SENSOR") or rawget(_G, "CATEGORY_TELEMETRY")
  local ok, s
  if type(spec) == "number" then
    if not cat then return nil end
    ok, s = pcall(system.getSource, {category = cat, appId = spec})
  else
    if tele and cat then ok, s = pcall(system.getSource, {category = cat, name = spec}) end
    if not (ok and BAL.SN.isSet(s)) then ok, s = pcall(system.getSource, spec) end
  end
  if not (ok and BAL.SN.isSet(s)) then return nil end
  if tele and cat then
    local c = BAL.SN.call(s, "category")
    local c2 = rawget(_G, "CATEGORY_TELEMETRY")
    if c ~= nil and c ~= cat and c ~= c2 then return nil end   -- z.B. ein Kanal, der zufaellig so heisst
  end
  return s
end

-- Gibt es eine Korrektur der TEK-Kalibrierung fuer dieses Modell? (gleicher Dateiname wie in scripts/tekkal)
function BAL.SN.korrFile()
  if not (io and io.open) then return false end
  local name = "korrektur_" .. tostring(currentModelName() or "Modell"):gsub("[^%w%-_]", "_") .. ".txt"
  for _, d in ipairs(BAL.SN.KORR_DIRS) do
    local ok, f = pcall(io.open, d .. name, "r")
    if ok and f then pcall(f.close, f); return true end
  end
  return false
end

function BAL.SN.wanted(w, R)
  local dm, st = w.devMain or 0, w.devStabi == true
  if R.dev == "gps" or R.dev == "baro" then return dm >= 1 end
  if R.dev == "sm" then return dm == 1 end
  if R.dev == "frgps" then return dm == 2 end   -- GPS-Kurs nur mit FrSky-GPS; beim SM-Logger Kurs aus den Positionen
  if R.dev == "stabi" then return st end
  if R.dev == "acc" then return st or dm == 1 end
  return false
end

-- wirksame Quelle einer Rolle setzen (w[k] liest der Rest des Widgets wie bisher)
function BAL.SN.applyKey(w, k)
  local man = w.man
  if not man then return end
  local m = man[k]
  local f = BAL.SN.on(w) and w.snFound and w.snFound[k] or nil
  w.snKorrK = w.snKorrK or {}
  if BAL.SN.isSet(m) or not f then w[k] = m else w[k] = f end
  if k == "vario" then
    local vt = man.varioType or 0
    if vt ~= 1 and vt ~= 2 then vt = (w.devMain == 2 and not (w[k] == f and w.snKorrK.vario)) and 2 or 1 end
    w.varioType = vt
  elseif k == "airSrc" then
    w.airUnit = (w[k] == m) and (man.airUnit or 1) or 1   -- automatisch: Einheit der Quelle (ohne Einheit = Knoten)
  end
end

function BAL.SN.apply(w)
  for _, k in ipairs(BAL.SN.KEYS) do BAL.SN.applyKey(w, k) end
end

function BAL.SN.resolve(w, k)
  local R = BAL.SN.ROLE[k]
  local found, korr = nil, false
  if BAL.SN.wanted(w, R) then
    if R.korr and w.devMain == 1 and w.snKorr then found = BAL.SN.get(R.korr, false); korr = found ~= nil end
    local desc = R.dev == "baro" and w.devMain == 1 and w.devStabi == true
    for _, id in ipairs(R.ids or {}) do
      if found then break end
      for j = 0, 15 do
        found = BAL.SN.get(id + (desc and 15 - j or j))
        if found then break end
      end
    end
    for _, n in ipairs(R.names) do
      if found then break end
      found = BAL.SN.get(n, true)
    end
  end
  w.snFound[k] = found
  w.snKorrK[k] = korr
  BAL.SN.applyKey(w, k)
end

function BAL.SN.resolveAll(w)
  if not w.man then return end
  w.snFound, w.snKorrK = w.snFound or {}, w.snKorrK or {}
  w.snKorr = (w.devMain == 1) and BAL.SN.korrFile() or false
  for _, k in ipairs(BAL.SN.KEYS) do BAL.SN.resolve(w, k) end
end

-- im Flug: eine Rolle pro wakeup, dann Pause (neu gefundene Sensoren, Modellwechsel, neue Korrektur-Datei)
function BAL.SN.step(w, now)
  if not w.man or not BAL.SN.on(w) then return end
  if w.snNext and now < w.snNext then return end
  local i = (w.snI or 0) + 1
  local k = BAL.SN.KEYS[i]
  if not k then
    w.snI = 0
    local miss = false
    for _, kk in ipairs(BAL.SN.KEYS) do
      if BAL.SN.wanted(w, BAL.SN.ROLE[kk]) and not BAL.SN.isSet(w[kk]) then miss = true end
    end
    w.snNext = now + (miss and 10 or 30)
    return
  end
  w.snFound, w.snKorrK = w.snFound or {}, w.snKorrK or {}
  if i == 1 then w.snKorr = (w.devMain == 1) and BAL.SN.korrFile() or false end
  BAL.SN.resolve(w, k)
  w.snI = i
end

-- von Hand gewaehlte Werte merken (read) und fuer den Speicher zurueckgeben (write)
function BAL.SN.load(w)
  local man = {varioType = w.varioType, airUnit = w.airUnit}
  for _, k in ipairs(BAL.SN.KEYS) do man[k] = w[k] end
  w.man, w.snFound, w.snKorrK, w.snI, w.snNext = man, {}, {}, 0, nil
  BAL.SN.resolveAll(w)
end
function BAL.SN.stored(w, k)
  if w.man and (BAL.SN.ROLE[k] or k == "varioType" or k == "airUnit") then return w.man[k] end
  return w[k]
end

-- Zusammenfassung fuer die Konfiguration: Zeilen mit je bis 4 Eintraegen
function BAL.SN.summary(w)
  local found, miss = {}, {}
  for _, k in ipairs(BAL.SN.KEYS) do
    local R = BAL.SN.ROLE[k]
    local hand = w.man and BAL.SN.isSet(w.man[k])
    if hand then found[#found + 1] = R.lbl .. " (Hand)"
    elseif BAL.SN.wanted(w, R) then
      local s = w.snFound and w.snFound[k]
      if s then
        local txt = R.lbl
        if w.snKorrK and w.snKorrK[k] then txt = R.lbl .. " (" .. R.korr .. ")" end
        found[#found + 1] = txt
      elseif k ~= "satSrc" and k ~= "crsSrc" and k ~= "gpsSpd" then miss[#miss + 1] = R.lbl end
    end
  end
  local function rows(t)
    local r = {}
    for i = 1, #t, 4 do r[#r + 1] = table.concat(t, ", ", i, math.min(i + 3, #t)) end
    return r
  end
  return rows(found), rows(miss)
end

-- Liefert eine neue Messung (fix) oder nil, wenn kein frischer GPS-Fix kam.
local function readInputs(w, now, dt)
  local lat, lon = readPosition(w.gps)
  local vario = readNumber(w.vario)
  if vario ~= nil then w.vSlow2 = lpf(w.vSlow2, vario, dt, 2) end   -- Sensor wie er kommt (Vertikalgeschwindigkeit, 3D-Speed)
  w.vSens = vario                                                     -- fuer die Fusion mit AccZ (BAL.VF)
  -- Vario-Art barometrisch: das Widget kompensiert selbst (Aenderung der Bewegungsenergie, siehe BAL.TE)
  if vario ~= nil and w.varioType == 2 then vario = vario + (w.teDE or 0) end
  w.varioRaw = vario
  if vario ~= nil then w.vFast = lpf(w.vFast, vario, dt, 0.6) end
  if lat == nil then
    w.gpsOk = w.gSeen ~= nil and now - w.gSeen <= 3
    return nil
  end
  -- Ethos liefert Breite und Laenge als zwei Telemetriewerte, die oft nicht gleichzeitig neu sind.
  -- Ein halber Fix (neue Breite, alte Laenge) liegt neben der Spur und verdreht den Kurs um bis zu 45 Grad.
  -- Darum gilt ein Fix erst, wenn beide Werte neu sind, oder HALF_FIX_WAIT s nach der ersten Aenderung
  -- (z.B. genau Nord/Sued, dann aendert sich die Laenge nicht). Unveraendert seit 1 s = Stillstand am Boden.
  local HALF_FIX_WAIT = 0.8
  if lat ~= w.rawLat then w.newLat = true end
  if lon ~= w.rawLon then w.newLon = true end
  if (w.newLat or w.newLon) and w.pendT == nil then w.pendT = now end
  w.rawLat, w.rawLon = lat, lon
  local fresh
  if w.pendT then fresh = (w.newLat and w.newLon) or now - w.pendT >= HALF_FIX_WAIT
  else fresh = w.gSeen ~= nil and now - w.gSeen >= 1.0 end
  if not fresh then
    w.gpsOk = w.gSeen ~= nil and now - w.gSeen <= 3
    return nil
  end
  w.newLat, w.newLon, w.pendT = false, false, nil
  if lat ~= w.lastLat or lon ~= w.lastLon then w.posMoveT = now end   -- fuer die Warnung "GPS steht"
  local dtFix = w.gSeen and (now - w.gSeen) or 1
  w.lastLat, w.lastLon, w.gSeen, w.gpsOk = lat, lon, now, true
  local T = w.task
  local msl = readNumber(w.gpsAlt)
  local rel = readNumber(w.relAlt)
  local agl
  if w.altMode == 2 and rel ~= nil then agl = rel
  elseif msl ~= nil then agl = msl - (T and T.elev or 0)
  else agl = rel or 0 end
  -- Kurs aus Positionen (ab 3 m), Geschwindigkeit vom Sensor oder aus den Positionen
  local x, y = 0, 0
  if T then x, y = toLocal(T, lat, lon) else
    x, y = (lon - (w.lon0 or lon)) * M_PER_DEG * math.cos(math.rad(lat)), (lat - (w.lat0 or lat)) * M_PER_DEG
    w.lat0, w.lon0 = w.lat0 or lat, w.lon0 or lon
  end
  -- Kurs ueber eine Basis von 2 s: vom juengsten Fix, der mindestens 2 s alt ist, zum neuen Fix. Von Fix zu Fix
  -- (0,5 bis 1 s, wenige Meter) macht die Aufloesung der Position (etwa 1 m) den Kurs unruhig, Kurszeiger,
  -- 5-s-Vektor und links/rechts pendelten. 2 s kosten etwa 1 s Verzoegerung. Nach einer Luecke beginnt sie neu.
  local h = w.crsHist
  if not h or (#h > 0 and now - h[#h].t > 5) then h = {}; w.crsHist = h end
  h[#h + 1] = {x = x, y = y, t = now}
  while #h > 2 and now - h[2].t >= 2.0 do table.remove(h, 1) end
  local b = h[1]
  local ddx, ddy = x - b.x, y - b.y
  local dd = math.sqrt(ddx * ddx + ddy * ddy)
  local gsPos
  if dd >= 3 then
    w.crs = bearingOf(ddx, ddy)
    gsPos = dd / math.max(now - b.t, 0.05)
  end
  local gs = speedMs(w.gpsSpd)
  if gs == nil then
    if gsPos then w.gsPos = lpf(w.gsPos, gsPos, dtFix, 1.5) end
    gs = w.gsPos or 0
  end
  -- Kurs vom GPS-Empfaenger (Quelle "GPS-Kurs", aus der Doppler-Geschwindigkeit, ohne die rund 1 s Verzug der
  -- 2-s-Basis); erst ab 3 m/s, darunter ist er unruhig
  -- (SM GPS-Logger 3: DIY-Wert 0522 "Flugrichtung"; kommt er roh in 0.1 Grad, wird er umgerechnet)
  -- 1.42: nur, solange er zum Kurs aus den Positionen passt (Mittel der Abweichung unter 45 Grad). Eine von Hand
  -- gewaehlte "Flugrichtung" des SM-Loggers ist kein Kurs und verdrehte sonst den Wind.
  local cs = readNumber(w.crsSrc)
  if cs and cs > 360 and cs <= 3600 then cs = cs / 10 end
  if cs and gs >= 3 and cs >= 0 and cs <= 360 then
    if dd >= 3 then w.csDev = lpf(w.csDev, math.abs(angDiff(cs, w.crs)), dtFix, 10) end
    if w.csDev and w.csDev < 45 then w.crs = cs % 360 end
  end
  local ve, vn = nil, nil
  if w.crs then local sx, sy = dirVec(w.crs); ve, vn = gs * sx, gs * sy end
  -- 1.42: keine Stuetzwerte im Motorsteigflug (Hoehe aendert sich ueber 6 m/s): der SM-Logger meldet die
  -- 3D-Geschwindigkeit, die Bodenvektoren waren dort zu lang und zogen den Kreisfit weg
  if w.lastAgl then w.climbW = lpf(w.climbW, (agl - w.lastAgl) / math.max(dtFix, 0.05), dtFix, 1) end
  if ve and math.abs(w.climbW or 0) <= 6 then windStep(w, ve, vn, now, w.asOk and w.asTas or nil) end
  -- Vertikalgeschwindigkeit: Vario, sonst Hoehenaenderung
  local vz = w.vSlow2
  if vz == nil and w.lastAgl then
    w.vzAlt = lpf(w.vzAlt, (agl - w.lastAgl) / math.max(dtFix, 0.05), dtFix, 2)
    vz = w.vzAlt
  end
  w.lastAgl = agl
  vz = vz or 0
  local v3d = math.sqrt(gs * gs + vz * vz) * KMH
  -- Luftvektor (TAS) und Steuerkurs
  local tas, hdg = gs, w.crs
  if ve and w.windOk then
    local ax, ay = ve - w.wx, vn - w.wy
    tas = math.sqrt(ax * ax + ay * ay)
    hdg = bearingOf(ax, ay)
  end
  if w.asOk and w.asTas then tas = w.asTas else BAL.TE(w, now, tas) end   -- Staudruck: Fahrt und TE aus BAL.AS
  -- Drehrate -> Wenderadius
  if hdg and w.hdgPrev and dtFix > 0.05 then
    local dA = angDiff(hdg, w.hdgPrev)
    w.turnSum = (w.turnSum or 0) + dA                              -- Drehung mit Vorzeichen, fuer die Kreiserkennung
    local omega = math.rad(math.abs(dA)) / dtFix
    w.omega = lpf(w.omega, omega, dtFix, 1.5)
    w.omS = lpf(w.omS, math.rad(dA) / dtFix, dtFix, 0.7)            -- mit Vorzeichen (rechts +), fuer die Querlage
    -- Wenderadius nur aus Kurven ausserhalb der Thermik (Wenden an den Ecken), bei voller Drehrate
    if w.omega > math.rad(25) and omega > 0.8 * w.omega and tas > 3 and not w.therm then
      local r = clamp(tas / omega, 8, 200)
      w.rMeas = lpf(w.rMeas, r, dtFix, 3)
      -- Radius der Wende = geflogene Luftstrecke / gedrehter Winkel (weniger anfaellig fuer Ausreisser)
      local a = w.turnAcc or {len = 0, core = 0, n = 0, ang = 0, sx = 0, sy = 0}
      a.len, a.core = a.len + tas * dtFix, a.core + omega * dtFix
      a.n, a.sx, a.sy = a.n + 1, a.sx + x, a.sy + y
      w.turnAcc = a
    end
    if w.turnAcc then
      w.turnAcc.ang = w.turnAcc.ang + omega * dtFix
      -- Wende fertig: mindestens 60 Grad gedreht, dann gilt der mittlere Radius dieser Wende
      if w.omega < math.rad(12) or w.therm then
        local a = w.turnAcc
        if not w.therm and a.ang >= math.rad(60) and a.n >= 2 then
          learnRadius(w, clamp(a.len / a.core, 8, 200), a.sx / a.n, a.sy / a.n)
        end
        w.turnAcc = nil
      end
    end
  end
  w.hdgPrev = hdg
  w.tasF = lpf(w.tasF, tas, dtFix, 2)
  w.gsF = lpf(w.gsF, gs, dtFix, 1.5)
  if w.v3dPrev then w.acc = lpf(w.acc, (v3d - w.v3dPrev) / KMH / math.max(dtFix, 0.05), dtFix, 2) end
  w.v3dPrev = v3d
  local utc = utcNow(w)
  local pmsl = msl
  if rel ~= nil and T then pmsl = rel + T.elev end
  return {clk = now, utc = utc, lat = lat, lon = lon, x = x, y = y, agl = agl, msl = msl or agl + (T and T.elev or 0),
    pmsl = pmsl, gs = gs, ve = ve, vn = vn, vz = vz, v3d = v3d, tas = tas, hdg = hdg, crs = w.crs, valid = true}
end

-- ---------------------------------------------------------------- Wertung

-- Wenderadius fuer TP j (nil = allgemein): gelernt pro Modell und TP, sonst Modell, sonst Messung
local function radius(w, j)
  local m = w.radii and w.radii[w.modelName or "?"]
  return (m and ((j and m["tp" .. j]) or m.all)) or w.rMeas or R_DEFAULT
end
local function radiusFn(w) return function(j) return radius(w, j) end end

-- Referenzlinie fuer die Querablage: vom aktuellen Punkt zur Tangente an die naechste Ecke
local function setRef(w, f)
  local T = w.task
  if not T then return end
  local C = cornersFrom(T, w.tp + 1, 1, radiusFn(w), w.margin)
  local p = planPath(f.x, f.y, w.windOk and w.wx or 0, w.windOk and w.wy or 0, w.tasF or 20, radius(w), C, nil, true)
  local e = p.entry or {x = C[1].ax, y = C[1].ay}
  w.ref = {ax = f.x, ay = f.y, bx = e.x, by = e.y}
end

local function event(w, code, f, now)
  w.lastEvent, w.lastEventT = code, now
  -- Marken fuer die Uebersicht (Start, Wendepunkte, Ziel)
  if f and f.x and (code == "STA" or code == "TPC" or code == "FIN") then
    w.marks = w.marks or {}
    w.marks[#w.marks + 1] = {x = f.x, y = f.y, k = code}
  end
  if code == "STA" or code == "TPC" or code == "FIN" then w.cornerHold = nil end   -- Sektor erreicht
  -- Nach der Wende bleibt die Eckansicht der eben erreichten Ecke noch zoomAfter s stehen (App, Vorgabe 5 s)
  if code == "TPC" and f and f.clk then w.cornerAfter = {k = w.tp, clk = f.clk} end
  if code == "STA" or code == "FIN" then w.cornerAfter = nil end
  if code == "STP" then w.endClk = f and f.clk end
  if code == "PEV" or code == "STA" or code == "TPC" or code == "FIN" or code == "STP" then igcEvent(w, code, f) end
end

local function assess(w, f, now)
  local T = w.task
  -- Ringpuffer / laufende IGC-Datei (1 Hz)
  if not w.lastB or f.clk - w.lastB >= 0.95 then
    w.lastB = f.clk
    local b = bRecord(w, f)
    if w.igc then igcAdd(w, b) else
      w.ring[#w.ring + 1] = b
      if #w.ring > 90 then table.remove(w.ring, 1) end
    end
  end
  -- Flug erkennen
  if not w.flying and f.v3d > 20 and f.agl > 20 then
    w.flying, w.takeoffT = true, now
    if T then igcOpen(w, f) end
  end
  if w.flying and f.agl < 20 and f.v3d < 5 then
    w.slowSince = w.slowSince or now
    if now - w.slowSince > 5 then
      w.flying, w.landedT = false, now
      if w.state == S_STARTED or w.state == S_ARMED then
        w.state, w.stopReason = S_STOPPED, "Landung"
        event(w, "STP", f, now)
      end
      w.closeAt = now + 5
      if w.autoPage then w.autoTarget = P_EVAL end
    end
  else
    w.slowSince = nil
  end
  -- Spur der letzten 70 s, hoechstens 4 Punkte pro Sekunde (haelt die Zeichenschleifen kurz)
  local lp = w.trail[#w.trail]
  if not lp or now - lp.t >= 0.25 then w.trail[#w.trail + 1] = {x = f.x, y = f.y, v = w.vFast or f.vz, t = now} end
  if #w.trail > 0 and now - w.trail[1].t > 75 then
    -- vorne abschneiden (nur die ~20 alten Punkte anfassen, der Rest wird in C verschoben)
    local tr, i, n = w.trail, 1, #w.trail
    while i < n and now - tr[i].t > 70 do i = i + 1 end
    table.move(tr, i, n, 1)
    for k = n - i + 2, n do tr[k] = nil end
  end
  -- ganzer Flugweg fuer die Uebersicht (ausgeduennt)
  w.track = w.track or {}
  local lt = w.track[#w.track]
  if not lt or now - lt.t > 2 or (f.x - lt.x) ^ 2 + (f.y - lt.y) ^ 2 > 400 then
    w.track[#w.track + 1] = {x = f.x, y = f.y, t = now, on = w.state == S_STARTED}
    if #w.track > 3000 then
      local k = {}
      for i = 1, #w.track, 2 do k[#k + 1] = w.track[i] end
      w.track = k
    end
  end
  if not T then w.lastFix = f; return end
  local s = f.x * T.ux + f.y * T.uy
  if w.badEntry and (w.state ~= S_STARTED or w.tp > 0 or s > T.dist * 0.66) then
    w.badEntry = nil
    if w.autoPage and w.state == S_STARTED then w.autoTarget = P_GLIDE end
  end
  local lat = f.x * T.px + f.y * T.py
  f.s, f.lat_ = s, lat
  local prev = w.lastFix
  -- geflogene Strecke ueber Grund in der laufenden Runde (fuer den Index)
  if w.state == S_STARTED and prev then w.lapDist = (w.lapDist or 0) + math.sqrt((f.x - prev.x) ^ 2 + (f.y - prev.y) ^ 2) end
  -- Linie ueberquert (Startsektor -> Zielsektor)
  if prev and prev.s and prev.s < 0 and s >= 0 and math.abs(lat) <= LINE_HALF and math.abs(prev.lat_ or 0) <= LINE_HALF then
    if w.state == S_ARMED then
      local a, b = slotWindow(w)
      local nowClock = utcNow(w)
      -- ohne gueltige Uhr (Senderuhr kaputt, GPS ohne Zeit) laesst sich das Fenster nicht pruefen: Start gilt
      -- Ohne gueltige Senderuhr ist das Datum unbekannt (GPS meist nur Tageszeit): dann nur die Tageszeit pruefen
      local _, clockOk = radioTime()
      if a and not clockOk and not (gpsClock(w) and w.gpsFull) then
        local d = a % 86400
        a, b, nowClock = d, d + (b - a), nowClock % 86400
        if nowClock < a - 43200 then nowClock = nowClock + 86400 end
      end
      if a and (nowClock < a or nowClock > b) then
        note(w, nowClock < a and "Start ungültig: Fenster noch zu" or "Start ungültig: Fenster vorbei", now)
        w.badStartT = now
      else
        local e = w.lastInStart or prev
        w.state = S_STARTED
        w.startT, w.startUtc, w.lapT = e.clk, e.utc, e.clk
        w.lapDist = 0
        w.entry = {agl = e.agl, v3d = e.v3d}
        w.penalty = penaltyOf(T, e.agl, e.v3d)
        w.tp = 0
        w.startedAt = now
        event(w, "STA", e, now)
        setRef(w, f)
        -- Einflug mit Strafe (zu hoch/zu schnell): Start-Seite bleibt, damit man sieht, was falsch war,
        -- und gleich neu scharf stellen kann. Erst nach 2/3 des Wegs zu TP1 geht es zur Seite Gleiten.
        w.badEntry = w.penalty > 0 or nil
        if w.badEntry then
          note(w, string.format("EINFLUG %s, Strafe %d", e.agl > T.maxH and "ZU HOCH" or "ZU SCHNELL", w.penalty), now)
          if w.autoPage then w.autoTarget = P_START end
        elseif w.autoPage then w.autoTarget = P_GLIDE end
      end
    elseif w.state == S_STARTED and w.tp == 3 then
      if f.v3d >= T.minF and f.clk - w.startT <= T.work * 60 then
        local lt = f.clk - w.lapT
        w.laps[#w.laps + 1] = {n = #w.laps + 1, time = lt, speed = T.perim / math.max(lt, 1) * KMH,
          agl = f.agl, dAlt = f.agl - (w.lapAgl or (w.entry and w.entry.agl) or f.agl), at = f.clk - w.startT, v3d = f.v3d,
          dist = w.lapDist, idx = w.lapDist and (w.lapDist / T.perim * 100) or nil}
        w.lapDist = 0
        w.lapAgl = f.agl
        w.lapT, w.tp = f.clk, 0
        w.lapDoneAt = now
        event(w, "FIN", f, now)
        setRef(w, f)
      else
        note(w, "Ziel ungültig (Speed/Arbeitszeit)", now)
      end
    end
  end
  if s < 0 then w.lastInStart = f end
  -- Wendepunkte
  if w.state == S_STARTED and w.tp < 3 and inSector(T, w.tp + 1, f.x, f.y) then
    w.tp = w.tp + 1
    w.tpT, w.tpPrev = now, w.tp
    event(w, "TPC", f, now)
    setRef(w, f)
  end
  -- Sicherheitszonen und Maximalhoehe
  local inZ = false
  for _, z in ipairs(T.zones) do if inZone(z, f.x, f.y, f.agl) then inZ = true; w.zoneName = z.name end end
  if inZ and not w.inZoneNow then w.zoneEnterT = now; if w.state == S_STARTED then w.zoneHit = true end end
  w.inZoneNow = inZ
  if f.agl > T.maxAlt then if not w.overMax then w.overMaxT = now end; w.overMax = true; if w.state == S_STARTED then w.altHit = true end
  else w.overMax = false end
  -- Arbeitszeit
  if w.state == S_STARTED and f.clk - w.startT > T.work * 60 then
    w.state, w.stopReason = S_STOPPED, "Arbeitszeit"
    event(w, "STP", f, now)
    if w.autoPage then w.autoTarget = P_EVAL end
  end
  w.lastFix = f
end

-- Kennzahlen der Wertung
local function scoreOf(w)
  local T = w.task
  local n = #w.laps
  local res = {n = n, penalty = w.penalty or 0}
  if n > 0 and w.startT then
    local last = w.laps[n]
    res.time = last.at
    res.speed = n * T.perim / math.max(res.time, 1) * KMH
  end
  return res
end

-- ---------------------------------------------------------------- Seite START: Einflug

local function startCalc(w, f)
  local T = w.task
  if not T or not f or not f.s then return nil end
  local c = {}
  c.dist = -f.s                                   -- Abstand vor der Linie (positiv = davor)
  local gn = 0
  if f.ve then gn = f.ve * T.ux + f.vn * T.uy end   -- Annaeherung an die Linie (m/s)
  local gl = f.ve and (f.ve * T.px + f.vn * T.py) or 0
  c.gn = gn
  c.lat = f.lat_
  c.target = T.maxH - w.winLo / 2
  c.lo, c.hi = T.maxH - w.winLo, T.maxH
  if c.dist > 0 and gn > 1 then
    c.tLine = c.dist / gn
    c.predAgl = f.agl + (f.vz or 0) * c.tLine
    c.predV = f.v3d + clamp(w.acc or 0, -3, 3) * KMH * math.min(c.tLine, 5)
    c.predLat = f.lat_ + gl * c.tLine
    c.err = c.predAgl - c.target
    c.reqSlope = (f.agl - c.target) / c.dist        -- noetiger Gleitwinkel (Hoehe pro Meter)
    c.slope = -(f.vz or 0) / gn                     -- aktueller Gleitwinkel
  end
  c.penaltyNow = penaltyOf(T, f.agl, f.v3d)
  return c
end

local function startTones(w, c, now)
  if not c or not w.startTone or w.state ~= S_ARMED then w.startBeepT = nil; return end
  if not c.tLine or c.dist > 2500 or c.tLine > 90 then return end
  -- Zu schnell: Vibration
  local T = w.task
  if (c.predV or 0) > T.maxV - w.speedMargin or (w.lastFix and w.lastFix.v3d > T.maxV - w.speedMargin) then
    if not w.hapT or now - w.hapT > 1.0 then w.hapT = now; haptic(w, P_START, 250) end
  end
  -- Gleitwinkel-Ton: Standard hoch = zu tief (Hoehe sparen), tief = zu hoch (mehr sinken); startToneLogic 2
  -- dreht das um. Im Fenster ein leiser Takt.
  if w.startBeepT and now < w.startBeepT then return end
  local e = c.err
  local half = w.winLo / 2
  if math.abs(e) <= half then
    tone(w, P_START, 700, 40)
    w.startBeepT = now + 1.0
  elseif (e < 0) == (w.startToneLogic ~= 2) then
    local x = clamp(math.abs(e) - half, 0, 60)
    tone(w, P_START, 900 + x * 15, 70)
    w.startBeepT = now + math.max(0.12, 0.5 - x * 0.007)
  else
    -- tiefer Ton durchgehend (wie der Sinkton des Varios)
    local x = clamp(math.abs(e) - half, 0, 60)
    w.startBeepT = BAL.hold(w, P_START, math.max(250, 450 - x * 3), now, "startLowEnd")
  end
end

-- ---------------------------------------------------------------- Seite GLEITEN: Sollfahrt + Weg

-- Ecke geschnitten: schnellste Wende zurueck in den Sektor. Rechnet Links- und Rechtskurve (Radius R in der
-- Luft, mit Wind versetzt) und je den geraden Weg zum Zielpunkt im Sektor; dazu die Kurve, die am Zielpunkt
-- noch bis zum Kurs auf die naechste Ecke fehlt. Liefert einen Plan wie planPath (pts, hdg, inTurn, turnDir).
local function missPlan(w, f, T, j, wx, wy, V, R)
  local c = T.tp[j]
  local ax, ay = c.x + w.margin * c.bx, c.y + w.margin * c.by
  local nxt = T.tp[j % 3 + 1]
  local om = math.deg(V / R)                                       -- Grad pro Sekunde
  local STEP = 15                                                  -- 10 Grad sprengte im Testflug die Instruktionsgrenze
  local dt = STEP / om
  -- Restzeit ab einem Punkt im Sektor mit Steuerkurs h: Kurve (kuerzere Seite) bis auf Kurs zur naechsten
  -- Ecke, dann gerade bis dorthin. Kurve geschlossen gerechnet (Kreis in der Luft, Wind als Versatz),
  -- zwei Durchgaenge fuer die Peilung vom Kurvenende. pts nur auf Wunsch (fuer die Zeichnung).
  local function exitCost(x, y, h, wantPts)
    local phi, d = 0, 1
    local ex, ey, tt = x, y, 0
    for _ = 1, 2 do
      local dA = angDiff(bearingOf(nxt.x - ex, nxt.y - ey), h)
      d = dA > 0 and -1 or 1                                       -- -1 = rechts
      phi = math.abs(dA)
      tt = phi / om
      local cxv, cyv = dirVec(h - d * 90)
      local qx, qy = dirVec(h - d * phi + d * 90)
      ex = x + R * (cxv + qx) + wx * tt
      ey = y + R * (cyv + qy) + wy * tt
    end
    local h2 = h - d * phi
    local hx, hy = dirVec(h2)
    local gs = math.max(math.sqrt((V * hx + wx) ^ 2 + (V * hy + wy) ^ 2), 3)
    local pts
    if wantPts then
      pts = {}
      local cxv, cyv = dirVec(h - d * 90)
      local n = math.max(1, math.ceil(phi / 15))
      for k = 1, n do
        local p = phi * k / n
        local qx, qy = dirVec(h - d * p + d * 90)
        local tk = p / om
        pts[#pts + 1] = {x = x + R * (cxv + qx) + wx * tk, y = y + R * (cyv + qy) + wy * tk}
      end
    end
    return tt + math.sqrt((nxt.x - ex) ^ 2 + (nxt.y - ey) ^ 2) / gs, pts
  end
  local best
  local function offer(t0, dir, pts, n, ex, ey, h, tTurn)
    local sx, sy = ex or pts[n].x, ey or pts[n].y
    local tt = t0 + exitCost(sx, sy, h, false)
    if best and tt >= best.t then return end
    best = {t = tt, dir = dir, pts = pts, n = n, sx = sx, sy = sy, h = h, tTurn = tTurn, gx = ex, gy = ey}
  end
  -- Kurs als Einheitsvektor, pro Schritt um STEP gedreht (Drehmatrix statt sin/cos: spart Instruktionen)
  local cS, sS = math.cos(math.rad(STEP)), math.sin(math.rad(STEP))
  local cosTol = math.cos(math.rad(STEP / 2 + 0.01))
  local cx, cy, cbx, cby, mg = c.x, c.y, c.bx, c.by, w.margin
  local W2 = wx * wx + wy * wy
  for _, dir in ipairs({1, -1}) do                                 -- 1 = links, -1 = rechts
    local x, y, h, t = f.x, f.y, f.hdg, 0
    local hx, hy = dirVec(h)                                       -- Kurs jetzt
    local mx, my = dirVec(h - dir * STEP / 2)                      -- mittlerer Kurs im naechsten Schritt
    local sd = -dir * sS
    local pts = {{x = x, y = y}}
    for _ = 0, 360 / STEP do
      -- weiter drehen macht nur langsamer: abbrechen, sobald die Zeit schon ueber dem besten Weg liegt
      if best and t >= best.t then break end
      -- a) schon in der Kurve im Sektor: Wende hier fertig
      if t > 0 then
        local rx, ry = x - cx, y - cy
        local rl = math.sqrt(rx * rx + ry * ry)
        local dep = rx * cbx + ry * cby
        if rl > 0.1 and dep >= mg and dep / rl >= 0.7071 then
          offer(t, dir, pts, #pts, nil, nil, h, t)
          break
        end
      end
      -- b) gerade zum Zielpunkt: Grundgeschwindigkeit s entlang u mit |s*u - W| = V
      local gx, gy = ax - x, ay - y
      local D = math.sqrt(gx * gx + gy * gy)
      if D > 1 and gx * hx + gy * hy > 0 then                      -- Zielpunkt hinter dem Kurs: kein Kandidat
        local ux, uy = gx / D, gy / D
        local wu = wx * ux + wy * uy
        local disc = wu * wu - W2 + V * V
        if disc > 0 then
          local sg = wu + math.sqrt(disc)
          local qx, qy = sg * ux - wx, sg * uy - wy                -- noetiger Steuerkurs in der Luft (Laenge V)
          if sg > 1 and (qx * hx + qy * hy) >= cosTol * V then
            offer(t + D / sg, dir, pts, #pts, ax, ay, bearingOf(qx, qy), t)
          end
        end
      end
      -- STEP Grad weiter drehen (in der Luft), Bodenpunkt mit Wind
      x, y, t, h = x + (V * mx + wx) * dt, y + (V * my + wy) * dt, t + dt, h - dir * STEP
      hx, hy = hx * cS + hy * sd, hy * cS - hx * sd
      mx, my = mx * cS + my * sd, my * cS - mx * sd
      pts[#pts + 1] = {x = x, y = y}
    end
  end
  if best then
    -- Zeichenpunkte nur fuer den besten Weg
    local P = {}
    for k = 1, best.n do P[k] = best.pts[k] end
    if best.gx then P[#P + 1] = {x = best.gx, y = best.gy} end
    local _, epts = exitCost(best.sx, best.sy, best.h, true)
    for k = 1, #epts do P[#P + 1] = epts[k] end
    local last = P[#P]
    best.pts, best.ex, best.ey = P, last.x, last.y
  end
  if not best then return nil end
  -- weiter zur naechsten Ecke wie sonst geplant
  local rest = planPath(best.ex, best.ey, wx, wy, V, R, cornersFrom(T, j % 3 + 1, 2, radiusFn(w), w.margin))
  for k = 2, #(rest.pts or {}) do best.pts[#best.pts + 1] = rest.pts[k] end
  return {pts = best.pts, inTurn = best.tTurn > 0, turnDir = best.dir, hdg = (f.hdg - best.dir * 90) % 360,
    tT = 0, dT = 0, t1 = best.t, tAll = best.t + (rest.tAll or 0), missed = true}
end

local function nettoCalc(w, f, dt)
  -- Sollfahrt wie Widget Sollfahrt (gleiche Glaettung wie dessen Standard: Vario und Fahrt 2 s,
  -- Netto 8 s), damit beide Widgets bei gleichem MC und Wind dieselbe Sollfahrt zeigen
  -- 1.41: nur plausible Eingaben (Vario +-30 m/s, Fahrt bis 100 m/s), Netto auf +-20 m/s begrenzt: am 13.11.
  -- zeigte das Feld NETTO zu Beginn des Flugs fuenfstellige Werte
  if w.polar and f and f.tas and f.tas * KMH > 15 and f.tas < 100 and w.varioRaw ~= nil and math.abs(w.varioRaw) < 30 then
    local p = w.polar
    w.teN = lpf(w.teN, w.varioRaw, dt, 2)
    w.tasN = lpf(w.tasN, f.tas, dt, 2)
    w.netto = clamp(lpf(w.netto, w.teN + polarSink(p, w.tasN), dt, 8), -20, 20)
    local head = nil
    if w.windOk and f.hdg then local hx, hy = dirVec(f.hdg); head = -(w.wx * hx + w.wy * hy) end
    -- Zeit-MC, solange er gilt (siehe w.rzMan in wakeup: Poti oder Tippen auf das MC-Feld). Dann zaehlt
    -- das gelernte Netto-Mittel des Schenkels (wie im Modell der Restzeit), nicht das Netto im Moment.
    local rz = not w.rzMan and w.rz
    local net = w.netto
    if rz and rz.mcz and rz.net then net = rz.net[math.min(w.tp, 3) + 1] end
    w.vstf = speedToFly(p, rz and rz.mcz or w.mcValue, clamp(net, -5, 5), head)
    w.stfDv = ((w.tasF or f.tas) - w.vstf) * KMH
  elseif not w.polar then
    w.vstf, w.stfDv = nil, nil
  end
  -- mittleres Sinken im Geradeausflug (Ersatz, wenn keine Polare da ist)
  if f and f.vz and not w.therm and (w.omega or 0) < math.rad(8) then w.sinkAvg = lpf(w.sinkAvg, -f.vz, dt, 20) end
end

-- Weg, Querablage, Ankunftshoehe. Laeuft nur bei einem neuen GPS-Fix. full = eine sichtbare Seite zeigt den
-- Weg (Gleiten, Uebersicht, Start nach dem Start); sonst nur Distanz und Zeit zur Ecke fuer die Eckansicht.
-- pro wakeup: Querlage lesen, Nullpunkt (radien.lua, pro Modell) und Nachtrimmen im geraden Flug abziehen
function BAL.QL.read(w, now, dt)
  local v = readNumber(w.bankSrc)
  if v == nil or math.abs(v) > 180 then w.qlPhi, w.qlRaw = nil, nil; return end
  if w.bankInv then v = -v end
  w.qlRaw, w.qlEst = v, nil
  local m = w.radii and w.radii[w.modelName or "?"]
  local phi = v - (m and m.ql or 0) - (w.qlTrimV or 0)
  w.qlPhi = phi
  -- gerade nach GPS (10 s, nicht im Bart, in Fahrt): dort muss die Querlage 0 sein
  local f = w.cur
  if f and w.flying and not w.therm and (f.gs or 0) > 5 and math.abs(w.omS or 1) < math.rad(3) then
    w.qlStraight = (w.qlStraight or 0) + dt
    if w.qlStraight >= 10 then
      -- nur kleine Reste nachtrimmen; grosse Querlage heisst meist "Kurve beginnt" (der Sensor ist dem GPS voraus)
      if w.qlTrim and math.abs(phi) < 8 then w.qlTrimV = clamp((w.qlTrimV or 0) + phi * dt / 20, -5, 5) end
      -- bleibt sie 5 s am Stueck gross, passt der Sensor nicht (falsche Quelle, Achse): Drehrate dann aus GPS
      if math.abs(phi) > 10 then w.qlBadT = (w.qlBadT or 0) + dt else w.qlBadT = math.max((w.qlBadT or 0) - dt, 0) end
      if w.qlBadT >= 5 then w.qlBad = true elseif w.qlBadT == 0 then w.qlBad = nil end
    end
  else
    w.qlStraight = 0
    if math.abs(phi) <= 10 then w.qlBadT = math.max((w.qlBadT or 0) - dt, 0) end
  end
end

-- Nullen: jetziger Rohwert wird der Nullpunkt dieses Modells (Fluegel waagrecht halten)
function BAL.QL.zero(w)
  if not w.qlRaw or not w.radii then return false end
  local name = w.modelName or "?"
  local m = w.radii[name] or {}
  w.radii[name] = m
  m.ql = math.floor(w.qlRaw * 10 + 0.5) / 10
  w.qlTrimV, w.qlBad = 0, nil
  saveRadii(w)
  return true
end

-- Drehrate (rad/s, rechts +) aus der Querlage; ohne brauchbaren Sensor aus dem GPS-Kurs, sonst 0
function BAL.QL.omega(w, f)
  if w.qlPhi and not w.qlBad then
    local phi = math.abs(w.qlPhi) < 3 and 0 or clamp(w.qlPhi, -60, 60)
    return 9.81 * math.tan(math.rad(phi)) / math.max(w.tasF or f.tas or 15, 8)
  end
  return w.omS or 0
end

-- Querlage ohne Lagesensor (Wettbewerb: keine Kreisel, Reglement 2.7). Der Querruder-Knueppel gibt die Rollrate
-- (k Grad/s bei Vollausschlag), aufsummiert ist das die Querlage sofort beim Einleiten. Die Querlage aus der
-- GPS-Drehrate (phi = atan(omega * v / g), traege und unruhig) zieht die Summe in qaTau s nach.
-- k lernt das Widget, solange ein Lagesensor dran ist (Trainingsflug mit Stabi-Empfaenger): Aenderung der Querlage
-- in 1.6 s gegen die Knueppel-Summe 0.4 s frueher, Vorzeichen inklusive; pro Modell in radien.lua (qk). Ohne
-- gelernten Wert gilt qaRate aus der App. Log 7.11.: Verzug zum Stabi-Sensor 0.1-0.3 s statt 1.6 s mit GPS allein.
BAL.QA = {}
function BAL.QA.step(w, now, dt)
  local f = w.cur
  local q = w.ailSrc and readNumber(w.ailSrc)
  if not q or not w.omS or not f or (w.qaTau or 25) <= 0 then w.qaB, w.qaH = nil, nil; return end
  q = clamp(q / 1024, -1, 1)
  local bg = math.deg(math.atan(w.omS * math.max(w.tasF or f.tas or 15, 8) / G))
  local fly = w.flying and (f.gs or 0) > 8
  local sens = w.qlPhi and not w.qlBad
  -- Knueppel-Mitte (Trimmung): nach 5 s geradeaus nachgefuehrt
  if fly and math.abs(w.omS) < math.rad(3) then w.qaStr = (w.qaStr or 0) + dt else w.qaStr = 0 end
  if w.qaStr > 5 then w.qaZero = lpf(w.qaZero, q, dt, 10) end
  local qq = q - (w.qaZero or 0)
  if math.abs(qq) < 0.015 then qq = 0 end
  w.qaI = (w.qaI or 0) + qq * dt
  local m = w.radii and w.radii[w.modelName or "?"]
  if not w.qaT or now - w.qaT >= 0.2 then
    w.qaT = now
    local h = w.qaH or {}
    w.qaH = h
    h[#h + 1] = {t = now, i = w.qaI, p = sens and w.qlPhi or nil}
    while #h > 2 and now - h[2].t >= 2.2 do table.remove(h, 1) end
    if fly and sens and h[1].t <= now - 1.95 then
      local a, c, e
      for _, x in ipairs(h) do
        if not c and x.t >= now - 2.0 then c = x end
        if not a and x.t >= now - 1.6 then a = x end
        if not e and x.t >= now - 0.4 then e = x end
      end
      if a.p then
        local x, y = e.i - c.i, w.qlPhi - a.p
        w.qaSxy = (w.qaSxy or 0) * 0.9983 + x * y      -- vergisst in rund 2 min
        w.qaSxx = (w.qaSxx or 0) * 0.9983 + x * x
        if w.qaSxx > 2 and m == nil and w.radii then m = {}; w.radii[w.modelName or "?"] = m end
        if w.qaSxx > 2 and m then
          local kn = w.qaSxy / w.qaSxx
          m.qk = math.floor((kn < 0 and -1 or 1) * clamp(math.abs(kn), 5, 300) + 0.5)
          if math.abs(m.qk - (w.qkSaved or 0)) >= 2 and now - (w.qkSaveT or 0) >= 60 then
            w.qkSaved, w.qkSaveT = m.qk, now
            saveRadii(w)
          end
        end
      end
    end
  end
  local k = (m and m.qk) or w.qaRate or 30
  if not fly then w.qaB = bg
  else w.qaB = (w.qaB or bg) + dt * k * qq + dt / ((w.qaTau or 25) / 10) * (bg - w.qaB) end
  if not sens then w.qlPhi, w.qlEst = clamp(w.qaB, -80, 80), true end
end

-- Restkorrektur und gebogene Ablage-Vorhersage. g.rest = Kursfehler nach der Drehung in tau s,
-- g.qlState = "hold" (Querlage reicht), "out" (zu viel: ausleiten) oder nil
function BAL.QL.apply(w, g, f)
  g.rest, g.qlState = nil, nil
  local tau = (w.qlTau or 15) / 10
  if not g.err or tau <= 0 then return end
  local om
  if w.qlPhi and not w.qlBad then
    local phi = math.abs(w.qlPhi) < 3 and 0 or clamp(w.qlPhi, -60, 60)
    om = 9.81 * math.tan(math.rad(phi)) / math.max(w.tasF or f.tas or 15, 8)
  elseif w.omS then
    om, tau = w.omS, math.min(tau, 1)
  else
    return
  end
  local turn = math.deg(om * tau)
  local rest = g.err - turn
  g.rest, g.turn = rest, turn
  if math.abs(turn) >= 3 then
    if math.abs(rest) < 5 then g.qlState = "hold"
    elseif rest * turn < 0 and (g.err * turn > 0 or math.abs(g.err) < 5) then g.qlState = "out" end
  end
  -- Ablage in PRED_S s: tau s auf dem Bogen (Sehne), danach geradeaus mit dem neuen Kurs
  if g.xt and f.ve and f.vn and w.ref then
    local dx, dy = w.ref.bx - w.ref.ax, w.ref.by - w.ref.ay
    local l = math.sqrt(dx * dx + dy * dy)
    if l >= 1 then
      local t1 = math.min(tau, PRED_S)
      local c1, s1, c2, s2 = math.cos(om * t1 / 2), math.sin(om * t1 / 2), math.cos(om * t1), math.sin(om * t1)
      local se = (f.ve * c1 + f.vn * s1) * t1 + (f.ve * c2 + f.vn * s2) * (PRED_S - t1)
      local sn = (-f.ve * s1 + f.vn * c1) * t1 + (-f.ve * s2 + f.vn * c2) * (PRED_S - t1)
      g.xtPred = g.xt + (se * dy - sn * dx) / l
    end
  end
end

local function glideCalc(w, f, full)
  local T = w.task
  local g = {}
  if not T or not f or w.state ~= S_STARTED then return g end
  if not full then
    g.next = w.tp % 3 + 1
    local nxt = T.tp[g.next]
    g.dV = math.sqrt((f.x - nxt.x) ^ 2 + (f.y - nxt.y) ^ 2)
    g.tCorner = g.dV / math.max(f.gs or w.tasF or f.tas or 20, 5)
    if g.tCorner < w.zoomSec then g.corner = g.next; w.cornerHold = g.next end
    if w.cornerHold == g.next and g.dV < 400 then g.corner = g.next end
    if not g.corner and w.cornerAfter and f.clk and f.clk - w.cornerAfter.clk < (w.zoomAfter or 5) then g.corner = w.cornerAfter.k end
    return g
  end
  local R = radius(w, w.tp + 1)
  local C = cornersFrom(T, w.tp + 1, 3, radiusFn(w), w.margin)
  local wx, wy = w.windOk and w.wx or 0, w.windOk and w.wy or 0
  -- Ecke geschnitten (Eckansicht lief, Zielpunkt liegt jetzt hinter dem Modell): schnellste Wende rechnen,
  -- sonst normal planen (nicht beides, das sprengt die Instruktionsgrenze von Ethos)
  local plan, missed
  local j = w.tp + 1
  if w.tp < 3 and w.cornerHold == j and f.hdg and f.ve then
    local c = T.tp[j]
    local ax, ay = c.x + w.margin * c.bx, c.y + w.margin * c.by
    local dd = math.sqrt((ax - f.x) ^ 2 + (ay - f.y) ^ 2)
    if dd < 400 and (ax - f.x) * f.ve + (ay - f.y) * f.vn < 0 then
      plan = missPlan(w, f, T, j, wx, wy, w.tasF or f.tas or 20, radius(w, j))
      missed = plan ~= nil
    end
  end
  -- Kurvenbeginn verpasst (planPath: geradeaus bis in den Sektor, dann wenden): im naechsten Aufruf die
  -- Wende zur anderen Seite (missPlan) rechnen und die schnellere nehmen (verglichen bis zur naechsten Ecke). Nie beide im selben Aufruf
  -- (Instruktionsgrenze); ist die andere Seite schneller, bleibt es bis zum Sektor bei ihr.
  local lm, heavy = w.lateMiss, missed
  if lm and lm.j ~= j then lm = nil; w.lateMiss = nil end
  if not plan and lm and f.hdg then
    local now = os.clock()
    if lm.use or lm.pending then
      local mp = missPlan(w, f, T, j, wx, wy, w.tasF or f.tas or 20, radius(w, j))
      heavy = true
      if mp and (lm.use or mp.t1 < lm.t2 - (now - lm.t)) then
        plan, missed, lm.use = mp, true, true
      elseif lm.pending then
        plan, lm.pending, lm.t = lm.plan, false, now
      else
        w.lateMiss = nil
      end
    end
  end
  plan = plan or planPath(f.x, f.y, wx, wy, w.tasF or f.tas or 20, R, C, f.hdg)
  if plan.late and not missed and w.tp < 3 then
    local now = os.clock()
    if not lm or now - lm.t >= 1 then w.lateMiss = {j = j, t = now, t2 = plan.t2 or 1e9, plan = plan, pending = true} end
  end
  g.plan, g.R, g.C = plan, R, C
  if f.hdg then g.err = angDiff(plan.hdg, f.hdg) end   -- positiv = rechts drehen
  if w.ref then
    g.xt = crossTrack(w.ref.ax, w.ref.ay, w.ref.bx, w.ref.by, f.x, f.y)
    -- Vorhersage: Querablage in PRED_S s mit dem jetzigen Kurs und Tempo ueber Grund
    if f.ve and f.vn then
      local dx, dy = w.ref.bx - w.ref.ax, w.ref.by - w.ref.ay
      local l = math.sqrt(dx * dx + dy * dy)
      if l >= 1 then g.xtPred = g.xt + (f.ve * dy - f.vn * dx) / l * PRED_S end
    end
  end
  -- Kurs-Kommando (Anzeige "x° LINKS/RECHTS" und Ansage) passend zum Ablage-Balken: zurueck zur Soll-Linie, mit
  -- einem Anschnitt, der mit der Ablage waechst (0.5 Grad pro Meter, hoechstens 30 Grad). Vorher zeigte das
  -- Kommando direkt zum Wendekreis: links der Linie und schon kraeftig zur Linie unterwegs hiess es dann
  -- "LINKS", obwohl der Balken "rechts" sagte. Kurz vor der Ecke (Kurve in weniger als 6 s) gilt der Plan.
  if g.xt and f.crs and w.ref and not plan.inTurn and not missed and (plan.tT or 0) > 6 then
    local lb = bearingOf(w.ref.bx - w.ref.ax, w.ref.by - w.ref.ay)
    g.err = angDiff(lb - clamp(g.xt * 0.5, -30, 30), f.crs)
  end
  BAL.QL.apply(w, g, f)
  g.next = w.tp % 3 + 1
  local nxt = T.tp[g.next]
  g.dV = math.sqrt((f.x - nxt.x) ^ 2 + (f.y - nxt.y) ^ 2)
  g.brgTP = bearingOf(nxt.x - f.x, nxt.y - f.y)
  -- Wind zerlegt auf den Sollkurs (Linie zur naechsten Ecke): Gegenwind positiv, Seitenwind positiv = blaest nach rechts
  local crsRef = w.ref and bearingOf(w.ref.bx - w.ref.ax, w.ref.by - w.ref.ay) or plan.hdg
  local hx, hy = dirVec(crsRef)
  if w.windOk then g.head, g.cross = -(wx * hx + wy * hy) * KMH, (wx * hy - wy * hx) * KMH end
  -- Hoehe an der Ziellinie: restliche Ecken dieser Runde, dann von TP3 zur Linie (durch den Ursprung)
  local V = w.vstf or w.tasF or f.tas or 20
  local nRem = 3 - w.tp
  local tRem
  local gsU = math.max(V + wx * T.ux + wy * T.uy, 3)                -- Grundgeschwindigkeit Richtung TP1
  if nRem > 0 then
    -- Restzeit der Runde: zweite Planung nur jede Sekunde und nie im selben Aufruf wie die Fehlwende
    local now = os.clock()
    local hc = w.hFinC
    if not hc or hc.tp ~= w.tp or (not heavy and now - hc.t >= 1) then
      local pr = planPath(f.x, f.y, wx, wy, V, R, cornersFrom(T, w.tp + 1, nRem, radiusFn(w), w.margin), f.hdg, true)
      hc = {tp = w.tp, t = now, tAll = pr.tAll or 0, x = f.x, y = f.y}
      w.hFinC = hc
    end
    tRem = math.max(hc.tAll - (now - hc.t), 0) + T.dist / gsU
  else
    tRem = math.max(-(f.x * T.ux + f.y * T.uy), 0) / gsU
  end
  local sink = w.polar and polarSink(w.polar, V) or w.sinkAvg
  if sink then g.hFin, g.tFin = f.agl - sink * tRem, tRem end
  -- Eckansicht: zoomSec Sekunden vor der naechsten Ecke (Ankunft nach Plan). Sobald der Sektor erreicht ist,
  -- zaehlt die naechste Ecke, die Eckansicht der erreichten Ecke bleibt noch zoomAfter s, dann wieder Gleiten.
  local gs = math.max(f.gs or V, 5)
  g.tCorner = plan.t1 or (g.dV / gs)
  if g.tCorner < w.zoomSec then g.corner = g.next; w.cornerHold = g.next end
  -- Ecke geschnitten (Sektor verpasst): Eckansicht bleibt, bis der Sektor erreicht ist (dann wechselt g.next)
  -- oder man sich mehr als 400 m vom TP entfernt hat
  if w.cornerHold == g.next and g.dV < 400 then g.corner = g.next end
  -- Nach der Wende: Eckansicht der erreichten Ecke noch zoomAfter s (siehe event)
  if not g.corner and w.cornerAfter and f.clk and f.clk - w.cornerAfter.clk < (w.zoomAfter or 5) then g.corner = w.cornerAfter.k end
  return g
end

-- Wendeton: Einzaehlen "drei, zwei, eins" (mindestens 3 s, sonst warnSec) im Sekundentakt und ein deutliches
-- "Jetzt wenden" (hoher langer Ton mit Vibration) um den gelernten Vorhalt tv VOR dem fruehesten sicheren
-- Kurvenbeginn (BAL.WV, 1.43). Der Kurvenbeginn wird bei jedem Aufruf aus dem
-- aktuellen Plan genommen (nicht im Voraus festgelegt): fliegt der Pilot neben dem Plan, verschiebt sich "Jetzt"
-- (im Grenzfall "geradeaus bis in den Sektor, dann wenden"). Kommt auf jeder Seite und auch, wenn das Widget nicht auf
-- dem Bildschirm ist (Ereignis). Genau einmal pro Ecke und Runde; kein Ton, wenn der Pilot schon kraeftig dreht.
-- Liefert true, solange der Countdown laeuft. Vorrang: ab Zaehlbeginn - 1.5 s bis 1 s nach
-- "Jetzt" (und 2.5 s ab jeder Kurs-Ansage vor der Ecke) schweigen alle anderen Toene und Ansagen auf allen Seiten
-- (w.prioUntil, siehe audible). So ist auch die Warteschlange des Senders leer, wenn der Countdown beginnt;
-- Ereignistoene aus dieser Zeit kommen danach.
local turnTone
do -- eigener Block: das Hauptkapitel ist am Limit von 200 lokalen Variablen
-- Zeit (s) bis zum fruehesten Kurvenbeginn, bei dem eine Kurve vom JETZIGEN Steuerkurs aus (Radius R in der Luft,
-- Drehsinn der Ecke, Wind als Versatz) sicher in den Sektor fuehrt: mindestens Reserve + 5 m tief. Frueher eingeleitet
-- verfehlt sie den Sektor. 0 = jetzt; nil = auch 12 s geradeaus und dann wenden reicht nicht (Kurs korrigieren).
-- Bogen in 15-Grad-Schritten bis 180 Grad, Kurvenbeginn per Halbierung (die Kurve wird mit spaeterem Beginn tiefer).
-- Einmal pro GPS-Fix gerechnet.
local function headingTurnT(w, g, f)
  local c = g.C and g.C[1]
  if not (c and c.tx and f and f.hdg) then return nil end
  if w.htT and w.htT.clk == f.clk and w.htT.tp == w.tp then return w.htT.v end
  local V = math.max(w.tasF or f.tas or 15, 5)
  local R = c.R or R_DEFAULT
  local s = c.s
  local wx, wy = w.windOk and w.wx or 0, w.windOk and w.wy or 0
  local h0 = math.rad(f.hdg)
  local dep = (c.margin or w.margin) + 10
  local om = V / R
  local function ok(t0)
    -- Kurvenbeginn nach t0 s geradeaus (Boden: Luft + Wind)
    local sx, sy = f.x + (V * math.sin(h0) + wx) * t0, f.y + (V * math.cos(h0) + wy) * t0
    for k = 1, 12 do
      local a = k * math.rad(15)                                   -- gedrehter Winkel
      local t = a / om
      local h = h0 - s * a                                         -- Peilung: links = kleiner
      -- Luftkreis: Versatz vom Kurvenbeginn = R * (Sehne), dazu Wind
      local x = sx + s * R * (math.cos(h) - math.cos(h0)) + wx * t
      local y = sy - s * R * (math.sin(h) - math.sin(h0)) + wy * t
      local rx, ry = x - c.tx, y - c.ty
      local d = rx * c.bx + ry * c.by
      if d >= dep and d * d >= 0.5 * (rx * rx + ry * ry) then return true end
    end
    return false
  end
  local v
  if ok(0) then v = 0
  elseif ok(12) then
    local lo, hi = 0, 12
    for _ = 1, 7 do
      local mid = (lo + hi) / 2
      if ok(mid) then hi = mid else lo = mid end
    end
    v = hi
  end
  w.htT = {clk = f.clk, tp = w.tp, v = v}
  return v
end

-- Vorrang mindestens bis now + s halten
local function prioHold(w, now, s)
  if not w.prioUntil or w.prioUntil < now + s then w.prioUntil = now + s end
end

-- eigene Toene der Wendevorhersage: kommen trotz Vorrang durch
local function prioTone(w, f, len, hap)
  w.prioPlay = true
  tone(w, nil, f, len)
  if hap then haptic(w, nil, hap) end
  w.prioPlay = nil
end

-- ---------------------------------------------------------------- Wende: Vorhalt, zu frueh, verpasst (1.43)
-- tv  = Vorhalt (s) fuer "Jetzt", pro Modell in radien.lua (tv), gelernt aus der Sektortiefe nach jeder Wende.
-- early = Warnung, wenn das Modell vor dem Sektor schon so eindreht, dass die Kurve (volle Drehrate V/R) den
--         Sektor nicht erreicht: tiefer Doppelton + Vibration ("kurz geradeaus").
-- miss  = Alarm, wenn der naechste TP ohne Sektor hinter dem Modell liegt (nahe vorbei, Abstand waechst 3 s im
--         Geradeausflug) oder das Modell im Sektor des uebernaechsten TP ist: Toene, Vibration, "Wendepunkt n
--         verpasst" (audio/wendepunkt.wav, verpasst.wav), alle 6 s, Balken "TPn FEHLT" bis der Sektor erreicht ist.
BAL.WV = {TV0 = 1.0, LO = 0.3, HI = 1.5}

function BAL.WV.tv(w)
  local m = w.radii and w.radii[w.modelName or "?"]
  return (m and m.tv) or BAL.WV.TV0
end

-- Tiefe im Sektor (m entlang der Halbierenden) und Abstand zum TP
function BAL.WV.depth(c, f)
  local rx, ry = f.x - c.x, f.y - c.y
  return rx * c.bx + ry * c.by, math.sqrt(rx * rx + ry * ry)
end

-- "Jetzt" gespielt: merken, mit welcher Geschwindigkeit das Modell in Richtung Sektor flog
function BAL.WV.jetzt(w, g)
  local f, T = w.cur, w.task
  local c = T and g.next and T.tp[g.next]
  if not (f and c and f.ve) then return end
  w.wvJ = {j = g.next, vb = f.ve * c.bx + f.vn * c.by, lap = #w.laps}
end

function BAL.WV.alarm(w, now, j)
  prioHold(w, now, 3.5)
  w.prioPlay = true
  tone(w, nil, 900, 150); tone(w, nil, 700, 150); tone(w, nil, 500, 300)
  haptic(w, nil, 500)
  if audioFile("verpasst.wav") then
    sayWord(w, nil, "wendepunkt.wav"); sayNumber(w, nil, j, UNIT_RAW, 0); sayWord(w, nil, "verpasst.wav")
  end
  w.prioPlay = nil
end

function BAL.WV.learn(w, M)
  local J = M.J
  local m = w.radii and w.radii[w.modelName or "?"]
  if not (J and J.vb > 3 and w.radii) or M.dmax > 100 then return end   -- >100 m: im Sektor gekreist
  if not m then m = {}; w.radii[w.modelName or "?"] = m end
  local tv = m.tv or BAL.WV.TV0
  local need = (w.margin or 5) + 10
  if M.dmax < (w.margin or 5) + 5 then tv = tv - 0.3                      -- knapp: sofort zurueck
  else tv = tv + clamp(0.3 * (M.dmax - need) / J.vb, -0.3, 0.3) end   -- hoechstens 0.3 s pro Wende
  m.tv = clamp(tv, BAL.WV.LO, BAL.WV.HI)
  w.radiiDirty = true
  w.wvLast = {j = M.j, dmax = M.dmax, ex = (M.dmax - need) / J.vb, tv = m.tv}
end

function BAL.WV.step(w, f, now, dt)
  if dt > 0 then w.wkDt = (w.wkDt or dt) * 0.9 + dt * 0.1 end
  local T = w.task
  if w.state ~= S_STARTED or not T then w.missTP, w.wvX, w.wvM, w.wvTp = nil, nil, nil, w.tp; return end
  -- Sektor erreicht: Messung der Tiefe beginnt, Alarm aus
  if w.wvTp ~= w.tp then
    if w.tp >= 1 and w.tp <= 3 then
      local J = w.wvJ
      if J and (J.j ~= w.tp or J.lap ~= #w.laps) then J = nil end
      w.wvM = {j = w.tp, dmax = -1e9, t = now, J = J}
    end
    w.wvTp, w.missTP, w.wvX, w.wvJ, w.wvE = w.tp, nil, nil, nil, nil
  end
  -- Alarm wiederholen (auch ohne neuen Fix)
  local X = w.wvX
  if w.missTP and X and X.nextT and now >= X.nextT then
    X.nextT = now + 6
    if not X.toward then BAL.WV.alarm(w, now, w.missTP) end
  end
  if not (f and f.ve) or f.clk == w.wvClk then return end
  w.wvClk = f.clk
  -- Tiefe nach dem Sektor-Eintritt: bis sie wieder abnimmt (Kurve fertig), hoechstens 12 s, nicht im Bart
  local M = w.wvM
  if M then
    local c = T.tp[M.j]
    local d = BAL.WV.depth(c, f)
    if d > M.dmax then M.dmax = d end
    if w.therm or now - M.t > 12 then w.wvM = nil
    elseif d < M.dmax - 5 then w.wvM = nil; BAL.WV.learn(w, M) end
  end
  local j = w.tp % 3 + 1
  local c = T.tp[j]
  local dep, dist = BAL.WV.depth(c, f)
  local gs = math.sqrt(f.ve * f.ve + f.vn * f.vn)
  -- zu frueh eingedreht: dreht in Wenderichtung (> 12 Grad/s), TP naeher als 150 m, noch nicht im Sektor;
  -- Kurve mit voller Drehrate V/R ab jetzt (Bogen in 15-Grad-Schritten bis 270 Grad) erreicht den Sektor nicht
  local s = c.s
  if dist < 150 and f.hdg and (w.omS or 0) * -s > math.rad(12) and not inSector(T, j, f.x, f.y) then
    local V = math.max(w.tasF or f.tas or 15, 5)
    local R = radius(w, j)
    local wx, wy = w.windOk and w.wx or 0, w.windOk and w.wy or 0
    local h0, om, ok = math.rad(f.hdg), V / R, false
    for k = 1, 18 do
      local a = k * math.rad(15)
      local t, h = a / om, h0 - s * a
      local x = f.x + s * R * (math.cos(h) - math.cos(h0)) + wx * t
      local y = f.y - s * R * (math.sin(h) - math.sin(h0)) + wy * t
      local rx, ry = x - c.x, y - c.y
      local dd = rx * c.bx + ry * c.by
      if dd >= (w.margin or 5) and dd * dd >= 0.5 * (rx * rx + ry * ry) then ok = true; break end
    end
    if not ok and (not w.wvE or now - w.wvE >= 2.5) then
      w.wvE = now
      prioHold(w, now, 1)
      prioTone(w, 600, 150, 250); prioTone(w, 450, 250)
    end
  end
  -- verpasst (nur TP1..3, nicht die Ziellinie; nicht im Landeanflug unter 50 m)
  if w.tp >= 3 or (f.agl or 999) < 50 then return end
  if not X or X.j ~= j then X = {j = j, dmin = dist, dprev = dist}; w.wvX = X end
  if dist < X.dmin then X.dmin = dist end
  local cosA = gs > 3 and dist > 1 and ((c.x - f.x) * f.ve + (c.y - f.y) * f.vn) / (dist * gs) or 0
  X.toward = cosA > 0.5
  local away = cosA < -0.17 and dist > X.dprev and (w.omega or 0) < math.rad(15)
  X.dprev = dist
  if away then X.awayT = X.awayT or now else X.awayT = nil end
  if not w.missTP then
    local passed = X.dmin < 250 and dist > X.dmin + 40 and X.awayT and now - X.awayT >= 3
    local nextSec = j < 3 and inSector(T, j + 1, f.x, f.y)
    if passed or nextSec then
      w.missTP, X.nextT = j, now + 6
      BAL.WV.alarm(w, now, j)
    end
  end
end

function turnTone(w, g, now)
  local p = g.plan
  if not p or w.warnSec <= 0 or w.state ~= S_STARTED then return false end
  if g.next ~= w.tp % 3 + 1 then return false end              -- Plan noch fuer die eben erreichte Ecke
  local key = #w.laps * 4 + w.tp
  local wt = w.warn
  if wt and wt.key ~= key then wt = nil; w.warn = nil end
  if wt and wt.done then
    -- deutlich weg von der Kurve (abgebrochen, z. B. Bart): fuer diese Ecke wieder frei
    if not (p.inTurn or p.missed or p.late) and (p.tT or 0) - w.lead / 10 > w.warnSec + 4 then w.warn = nil end
    return false
  end
  -- Zeit bis zum Kurvenbeginn. Massgebend ist der jetzige Kurs: "Jetzt" kommt erst, wenn eine Kurve von hier aus
  -- den Sektor sicher erreicht. Zeigt der Kurs so, dass keine Kurve den Zielpunkt erreicht, zaehlt der Plan (die
  -- Kurs-Ansage sagt, wohin); bei "Ecke geschnitten" ohne erreichbaren Zielpunkt sofort.
  local tT = p.tT or 99
  local th = w.cur and headingTurnT(w, g, w.cur)
  if th then tT = th
  elseif p.missed then tT = (g.dV or 0) < 2 * ((g.C and g.C[1] and g.C[1].R) or R_DEFAULT) and 0 or 99
  else
    -- Mit diesem Kurs fuehrt keine Kurve in den Sektor: kein "Jetzt", dafuer kurz vor der Ecke die Kurs-Ansage
    -- nach dem Plan ("15 rechts"), alle 3 s, auf jeder Seite
    if (p.tT or 99) - w.lead / 10 <= w.warnSec + 3 and g.err and math.abs(g.err) >= 5 and (not w.crsT or now - w.crsT >= 3) then
      w.crsT = now
      local right = g.err > 0
      local fn = right and "rechts.wav" or "links.wav"
      prioHold(w, now, 2.5)
      w.prioPlay = true
      if audioFile(fn) then
        sayNumber(w, nil, math.max(5, math.floor(math.abs(g.err) / 5 + 0.5) * 5), UNIT_RAW, 0); sayWord(w, nil, fn)
      else
        tone(w, nil, right and 2200 or 1500, 120)
      end
      w.prioPlay = nil
    end
    tT = 99
  end
  -- Vorhalt (1.43): "Jetzt" kommt tv s VOR dem fruehesten sicheren Kurvenbeginn. Bis die Kurve wirklich dreht,
  -- vergehen Reaktion, Einrollen und Aufbau der Drehrate (ASW-17 am 13.11.: Knueppel bis Drehung 1.2 s, Drehung im
  -- Mittel 1.7 s nach "Jetzt", Sektortiefe 24..70 m statt 15 m). tv lernt das Widget pro Modell aus der erreichten
  -- Sektortiefe (BAL.WV.learn), Start 1.0 s, 0.3..1.5 s. Gegen zu fruehes Eindrehen warnt BAL.WV.early.
  -- Ausgeloest wird schon, wenn der naechste wakeup zu spaet waere (halber gemessener Takt).
  tT = tT - (w.cur and (now - w.cur.clk) or 0)                 -- Fix ist bis zu einem GPS-Takt alt
  tT = tT - BAL.WV.tv(w) - 0.5 * (w.wkDt or 0.1)
  local turning = (w.omega or 0) > math.rad(15)
  -- Einzaehlen: mindestens 3 s ("drei, zwei, eins", dann der lange hohe Ton)
  local nCnt = math.max(w.warnSec, 3)
  -- Vorrang schon 1.5 s vor dem Einzaehlen (Warteschlange leeren) und waehrend es laeuft
  if (wt or tT <= nCnt + 1.5) and not turning then prioHold(w, now, 0.4) end
  if not wt then
    if tT > nCnt + 0.3 then return false end
    wt = {key = key}
    w.warn = wt
    if turning then wt.done = true; return false end           -- dreht schon: kein Ton mehr
  end
  if tT <= 0.15 then
    -- Jetzt wenden
    wt.done = true
    if not turning then
      prioHold(w, now, 1.6)                                    -- 0.6 s Ton, danach noch 1 s still
      prioTone(w, 2400, 600, 400)
      w.quietUntil = now + 1.5
      BAL.WV.jetzt(w, g)
    end
    return false
  end
  if tT > nCnt + 1 then return false end                       -- Kurs passt gerade nicht: warten, Kurs-Ansage fuehrt
  -- Einzaehlen: Zahl k, sobald die Restzeit unter k faellt (gleicher Abstand wie "Jetzt", also im Sekundentakt). Springt die Restzeit
  -- (Kurswechsel), faellt eine Zahl weg statt falsch zu zaehlen.
  local k = math.floor(tT - 0.15) + 1                          -- naechste faellige Zahl
  if k >= 1 and k <= nCnt and (not wt.last or k < wt.last) and tT <= k + 0.15 and tT > k - 0.5 then
    wt.last = k
    w.prioPlay = true
    sayNumber(w, nil, k, UNIT_RAW, 0)
    w.prioPlay = nil
  end
  w.quietUntil = now + 1
  return true
end
end

local function glideTones(w, g, now, warn)
  if not g.plan then
    -- vor dem Start: nur Sollfahrt
  else
    if warn then return end
    -- Kurs-Ansage "xx links" / "xx rechts" (Zahl ohne "Grad", dann links.wav/rechts.wav aus scripts/skynav/audio;
    -- fehlen die Dateien, ein Zwitscher: steigend = rechts, fallend = links). Vor allem, wenn man immer mehr
    -- vom Kurs wegdriftet:
    -- Ausloeser 1 (Trend): die Ablage in PRED_S s ist >= 15 m und waechst, 0.6 s lang -> sofort, danach alle 4 s,
    -- solange sie weiter waechst. Ausloeser 2 (Kurs): Kursfehler >= cmdMin fuer 2 s, hoechstens alle cmdGap s,
    -- aber nicht, solange die Ablage schon kleiner wird (der Pilot korrigiert bereits).
    -- keine Kommandos in der Kurve und kurz nach einem Wendepunkt (dort zeigt die Eckansicht den Weg)
    local turning = (w.omega or 0) > math.rad(8) or (w.tpT and now - w.tpT < 6)
    -- Mit Querlage: Kursfehler nach der vorausgerechneten Drehung (g.rest); HALTEN und AUSLEITEN ohne Kurs-Ansage
    local ge = g.rest or g.err
    if w.navVoice and ge and not turning and not g.plan.inTurn and (g.plan.tT or 0) > 6 and (not w.quietUntil or now > w.quietUntil) then
      local a, right = math.abs(ge), ge > 0
      local xt, xp = g.xt, g.xtPred
      local drift = xt and xp and math.abs(xt) <= 300 and math.abs(xp) >= 15 and math.abs(xp) > math.abs(xt) + 2
      local closing = xt and xp and math.abs(xp) < math.abs(xt) - 2
      if drift then w.driftSince = w.driftSince or now else w.driftSince = nil end
      local fire, deg = false, a
      if drift and now - w.driftSince >= 0.6 and (not w.cmdT or now - w.cmdT >= 4) then
        fire, right = true, xp < 0
        if (ge > 0) ~= right or a < 5 then
          -- Kursfehler zeigt nicht zur Linie zurueck: Winkel, mit dem man von der Linie wegdriftet
          local gs = math.max(w.gsF or 10, 5)
          deg = math.deg(math.asin(clamp((math.abs(xp) - math.abs(xt)) / (gs * PRED_S), 0, 1)))
        end
      end
      if a >= w.cmdMin and not closing and not g.qlState then
        w.errSince = w.errSince or now
        local changed = not w.cmdErr or math.abs(ge - w.cmdErr) >= 10
        if not fire and now - w.errSince >= 2 and (not w.cmdT or now - w.cmdT >= w.cmdGap or (changed and now - w.cmdT >= 4)) then fire = true end
      else
        w.errSince = nil
      end
      if fire then
        w.cmdT, w.cmdErr = now, ge
        deg = math.max(5, math.floor(deg / 5 + 0.5) * 5)
        if audioFile(right and "rechts.wav" or "links.wav") then
          sayNumber(w, P_GLIDE, deg, UNIT_RAW, 0)
          sayWord(w, P_GLIDE, right and "rechts.wav" or "links.wav")
          w.quietUntil = now + 2
        else
          w.cmdTones = {t = now, f = right and {1500, 2200} or {2200, 1500}}
          w.quietUntil = now + 0.6
        end
        return
      end
    end
  end
  if w.cmdTones and now >= w.cmdTones.t then
    local ft = w.cmdTones.f
    if ft[1] then tone(w, P_GLIDE, table.remove(ft, 1), 90); w.cmdTones.t = now + 0.12 end
    if #ft == 0 then w.cmdTones = nil end
    return
  end
  -- Sollfahrt-Ton (Logik "Bremsen = hoeher" wie im Sollfahrt-Widget)
  if w.stfTone and w.stfDv and (not w.quietUntil or now > w.quietUntil) and (not w.stfBeepT or now >= w.stfBeepT) then
    local dv = w.stfDv
    if math.abs(dv) > w.dead then
      local excess = clamp(math.abs(dv) - w.dead, 0, 25)
      if dv > 0 then
        local period = math.max(0.14, 0.55 - excess * 0.016)
        local len = clamp(period * 550, 70, 140)
        tone(w, P_GLIDE, 850 + excess * 20, len)
        w.stfBeepT, w.stfEnd = now + period, now + len / 1000
      else
        -- zu schnell: tiefer Ton durchgehend (wie der Sinkton des Varios)
        w.stfBeepT = BAL.hold(w, P_GLIDE, math.max(220, 400 - excess * 8), now, "stfLowEnd")
        w.stfEnd = w.stfLowEnd
      end
      return
    end
  end
  -- Kurs-Ticker (abschaltbar in der Widget-Konfiguration): kurze hohe Klicks, nur solange die Ablage waechst.
  -- Je schneller sie waechst, desto dichter (1 m/s: 1 s, 5 m/s: 0.4 s, ab 10 m/s: 0.2 s). Laeuft der Flieger
  -- zur Linie zurueck, ist es still. Nur in den Pausen der Sollfahrt, die hat Vorrang.
  if w.ticker and g.plan and g.xt and g.xtPred and not g.plan.inTurn and (w.omega or 0) <= math.rad(8)
      and not (w.tpT and now - w.tpT < 6) and (not w.quietUntil or now > w.quietUntil) and math.abs(g.xt) <= 300 then
    local rate = (math.abs(g.xtPred) - math.abs(g.xt)) / PRED_S
    if rate >= 1 and math.abs(g.xtPred) >= 8 and (not w.tickT or now >= w.tickT)
        and (not w.stfEnd or now >= w.stfEnd + 0.04) and (not w.stfBeepT or w.stfBeepT - now >= 0.08 or now >= w.stfBeepT + 0.3) then
      tone(w, P_GLIDE, 3000, 25)
      w.tickT = now + clamp(2 / rate, 0.2, 1.0)
    end
  end
end

-- ---------------------------------------------------------------- Seite THERMIK

local function thermCalc(w, f, now, dt)
  local v = w.vFast
  if v ~= nil then
    w.avg30 = w.avg30 or {}
    if not w.avgT or now - w.avgT >= 1 then
      w.avgT = now
      w.avg30[#w.avg30 + 1] = v
      if #w.avg30 > 30 then table.remove(w.avg30, 1) end
    end
  end
  -- Kreisen erkennen: Drehrate > 12 Grad/s ueber 8 s ohne Unterbruch (hoechstens 2 s geradeaus) und mindestens
  -- 270 Grad in eine Richtung. Wenden an den Ecken (rund 120 Grad) und S-Kurven sind kein Bart.
  local circ = (w.omega or 0) > math.rad(12)
  if circ then
    if not w.circSince then w.circSince, w.turnAt = now, w.turnSum or 0 end
    w.straightSince = nil
  else
    w.straightSince = w.straightSince or now
    if w.circSince and not w.therm and now - w.straightSince > 2 then w.circSince = nil end
  end
  if w.circSince and now - w.circSince > 8 and not w.therm and f and math.abs((w.turnSum or 0) - (w.turnAt or 0)) >= 270 then
    w.therm = {t0 = now, h0 = f.agl, hMax = f.agl}
    -- keine automatische Umschaltung auf Thermik: nur ueber den Schalter oder den Reiter
  end
  if w.therm and w.straightSince and now - w.straightSince > 8 then
    w.lastTherm = w.therm
    w.therm.tEnd = now - 8
    w.therm, w.circSince = nil, nil
    -- abgetrieben: Querablage ab hier neu messen (Linie vom Ausstieg zur naechsten Ecke)
    if w.state == S_STARTED and f then setRef(w, f) end
  end
  if w.therm and f then
    w.therm.hMax = math.max(w.therm.hMax, f.agl)
    w.therm.h = f.agl
    -- Netto-Steigen der Luft ueber die ganze Thermik: Vario plus Eigensinken, im Kreis mit Lastvielfachem
    -- n = 1/cos(Querlage) korrigiert (Sinken ~ n^1.5), tan(Querlage) = omega * v / g
    if w.polar and w.teN ~= nil and w.tasN and w.tasN * KMH > 15 then
      local tb = (w.omega or 0) * w.tasN / 9.81
      local n15 = (1 + tb * tb) ^ 0.75
      local tm = w.therm
      tm.nSum = (tm.nSum or 0) + (w.teN + polarSink(w.polar, w.tasN) * n15) * dt
      tm.nT = (tm.nT or 0) + dt
    end
  end
end

-- Varioton des Widgets, Klang waehlbar (Widget-Konfiguration vtStyle, ab 1.28 nicht mehr aus der App: XCSoar, OpenVario, XCVario, Ethos/EdgeTX, siehe BAL.VT.climb). Standard XCSoar (VarioSynthesiser, Standardwerte): TE-Vario v auf +-5 m/s begrenzt.
-- Steigen: 500 Hz bei 0 bis 1500 Hz bei +5 m/s, Periode 600 ms bei 0 bis 150 ms bei +5 m/s, davon 2/3 Ton und
-- 1/3 Pause. Sinken: durchgehender Ton, 500 Hz bei 0 bis 200 Hz bei -5 m/s. Totzone (still) von -vzLo bis
-- +vzHi (App, 0.1 m/s; Vorgabe wie XCSoar -0.3 bis +0.1).
-- Wann: auf der Seite Thermik immer; beim Kreisen auf allen Seiten (auch Start); auf der Seite Gleiten (mit
-- Eckansicht) ersetzt er den Sollfahrt-Ton, solange das Netto-Steigen das Minimalsinken plus vtGlide uebersteigt
-- (siehe BAL.VT.mode; nicht mit MC ZEIT, das ist der Endanflug). Der Sollfahrt-Ton selbst bleibt unveraendert. Laeuft ueber tone(), darum gilt
-- der Vorrang der Wendevorhersage. In BAL, weil das Hauptkapitel am Limit von 200 lokalen Variablen ist.
BAL.VT = {}

-- Wert fuer den Varioton, ruhig und trotzdem schnell. Daempfung vDamp (App, 0.1 s, Vorgabe 4).
-- Mit AccZ (und Vario-Art barometrisch): Fusion wie bei guten Varios. Die senkrechte Beschleunigung
-- (AccZ * cos Roll * cos Nick - 1 g) liefert die schnelle Aenderung, das Vario korrigiert langsam (Komplementaerfilter,
-- Verstaerkung 1.5/s gegen den Wert von vor 0.3 s, weil der Vario-Sensor nachhinkt; Nullpunkt der Beschleunigung
-- wird mitgeschaetzt). Im Log vom 7.11.2026 ist AccZ dem Vario rund 0.7 s voraus. Die Kompensation kommt mit AccX
-- schnell: TAS * (-AccX + sin Nick) ~ TAS * dv/dt / g, hochpass 1.5 s, dazu der langsame Teil aus der GPS-Fahrt
-- (teDE). Danach vDamp als Glaettung. Ohne AccZ: adaptive Daempfung, kleine Abweichungen (unter 0.5 m/s) mit
-- 0.3 + 3 * vDamp s, grosse fast sofort (0.3 s).
function BAL.VF(w, now, dt)
  local x = w.varioRaw
  if x == nil then return w.vtV end
  local damp = (w.vDamp or 4) / 10
  local az = w.varioType == 2 and w.vSens ~= nil and readNumber(w.accZSrc)
  if az then
    if math.abs(az) > 4 then az = az / G end                 -- Quelle in m/s2 statt g
    local r, p = math.rad(w.qlPhi or 0), math.rad(readNumber(w.pitchSrc) or 0)
    w.fuA = lpf(w.fuA, (az * math.cos(r) * math.cos(p) - 1) * G, dt, 0.3)
    local y = (w.fuY or w.vSens) + (w.fuA - (w.fuB or 0)) * dt
    local q = w.fuQ or {}
    w.fuQ = q
    q[#q + 1] = {t = now, y = y}
    while #q > 1 and now - q[2].t >= 0.3 do table.remove(q, 1) end
    local e = w.vSens - q[1].y
    y = y + 1.5 * dt * e
    w.fuB = clamp((w.fuB or 0) - 0.02 * dt * e, -2, 2)
    w.fuY = y
    local c = w.teDE or 0
    local ax, tas = readNumber(w.accXSrc), w.tasF
    if ax and tas and tas > 5 and not w.asOk then             -- mit Staudruck ist teDE schon schnell (BAL.AS)
      if math.abs(ax) > 4 then ax = ax / G end
      local cf = tas * (-ax + math.sin(p))
      w.fuCs = lpf(w.fuCs, cf, dt, 1.5)
      c = cf - w.fuCs + c
    end
    w.vtFu = true
    return lpf(w.vtV, y + c, dt, damp)
  end
  w.fuY, w.fuQ, w.vtFu = nil, nil, nil
  local f = lpf(w.vtFast, x, dt, 0.3)
  w.vtFast = f
  local v0 = w.vtV or f
  local ts = 0.3 + damp * 3
  return lpf(v0, f, dt, ts + (0.3 - ts) * math.min(1, ((f - v0) / 0.5) ^ 2))
end

function BAL.VT.mode(w, now, dt)
  w.vtV = BAL.VF(w, now, dt)
  -- Klang 4 (Ethos/EdgeTX) wie am Sender: Sensorwert ungefiltert (Ethos filtert nicht, die Glaettung von BAL.VF
  -- kostet rund 0.5 s). TE-Kompensation (Vario-Art barometrisch) bleibt. Christoph am Sender 9.11.2026.
  if w.vtStyle == 4 and w.varioRaw ~= nil then w.vtV = w.varioRaw end
  -- Kreisen: Drehrate > 12 Grad/s und mehr als 200 Grad gedreht (mehr als eine Wende an der Ecke);
  -- vorbei nach 2 s geradeaus
  if (w.omega or 0) > math.rad(12) then
    w.vtStr = nil
    if not w.vtC0 then w.vtC0, w.vtTurn0 = now, w.turnSum or 0 end
    if math.abs((w.turnSum or 0) - w.vtTurn0) >= 200 then w.vtCirc = true end
  else
    w.vtStr = w.vtStr or now
    if now - w.vtStr > 2 then w.vtC0, w.vtCirc = nil, false end
  end
  -- Netto fuer die Umschaltung: TE plus Eigensinken aus der Polare, 1 s geglaettet (das Netto der Sollfahrt mit
  -- 8 s ist fuer einen Bart, den man in 2 bis 3 s durchfliegt, zu traege). Ab 1.24 (Christoph 9.11.2026): das Netto
  -- muss zuerst das eigene Minimalsinken (Polare) ausgleichen. Rein bei Netto > Minimalsinken + vtGlide (App,
  -- 0.1 m/s, Vorgabe 0.5) fuer 0.5 s, raus bei Netto < Minimalsinken fuer 2 s. Mit MC ZEIT nie.
  local p = w.polar
  if p and w.varioRaw ~= nil and w.tasN and w.tasN * KMH > 15 then
    w.vtNet = lpf(w.vtNet, w.varioRaw + polarSink(p, w.tasN), dt, 1)
  else
    w.vtNet = nil
  end
  local sMin = p and polarSink(p, p.lo) or 0
  if w.vtNet and (w.rzMan or w.rzOn == false) then
    if w.vtTh then
      if w.vtNet < sMin then
        w.vtOut = w.vtOut or now
        if now - w.vtOut >= 2 then w.vtTh, w.vtOut = false, nil end
      else w.vtOut = nil end
    elseif w.vtNet > sMin + (w.vtGlide or 5) / 10 then
      w.vtIn = w.vtIn or now
      if now - w.vtIn >= 0.5 then w.vtTh, w.vtIn = true, nil end
    else w.vtIn = nil end
  else
    w.vtTh, w.vtIn, w.vtOut = false, nil, nil
  end
  local pg = w.page
  w.vtOn = w.varioTone and w.vtV ~= nil and (pg == P_THERM or w.vtCirc or (pg == P_GLIDE and w.vtTh)) or false
end

-- Klang (Widget-Konfiguration, vtStyle): Frequenz f (Hz), Periode und Tondauer (ms) beim Steigen v (m/s, 0..5), Frequenz beim Sinken.
-- 1 XCSoar (VarioSynthesiser): 500..1500 Hz linear, Periode 600..150 ms, 2/3 Ton.
-- 2 OpenVario (variod audiovario.c): 350 + 120 Hz pro m/s; Periode 1.088 s / v (bis 0.5 m/s: 2.18 s), gut 51 % Ton;
--   Sinken 350 / (1 + 0.85 |v|) Hz.
-- 3 XCVario (ESPAudio.cpp, Standard: 500 Hz Mitte, 2 Oktaven, Bereich 5 m/s, Exponent 1, ein Ton mit Pausen):
--   f = 500 + 500 * (1 + v/5) * v/5 (bis 1500 Hz), Sinken 500 - 125 * (1 + |v|/5) * |v|/5 (bis 250 Hz);
--   Takt 1 + 9 v/5 pro s, Ton 90 % der Periode minus 40 ms, Pause 10 % plus 40 ms (Periode hier mind. 100 ms).
-- 4 Ethos/EdgeTX (EdgeTX vario.cpp, Standardwerte Pitch zero 700 Hz, Pitch max 1700 Hz, Repeat 500 ms; Ethos ist
--   nicht offen, hat aber dieselben Einstellungen): f = 700 + 1000 v/5, Periode 80 + 420 (1 - v/5)^2 ms (im
--   50-ms-Takt von wakeup real mind. ~100 ms), nur 20 % Ton; Sinken 700 - 350 |v|/5 (bis 350 Hz).
function BAL.VT.climb(st, v)
  if st == 4 then
    local x = v / 5
    local per = 80 + 420 * (1 - x) ^ 2
    return 700 + 1000 * x, per, math.max(20, per / 5)
  elseif st == 2 then
    local per = v > 0.5 and 1088 / v or 2177
    return 350 + 120 * v, per, per * 0.51
  elseif st == 3 then
    local x = v / 5
    local per = math.max(100, 1000 / (1 + 9 * x))
    return 500 + 500 * (1 + x) * x, per, math.max(40, per * 0.9 - 40)
  end
  local per = 150 + (5 - v) * 90
  return 500 + 200 * v, per, per - math.floor(per / 3)
end
function BAL.VT.sink(st, v)
  local a = -v
  if st == 2 then return 350 / (1 + 0.85 * a) end
  if st == 3 then local x = a / 5; return 500 - 125 * (1 + x) * x end
  if st == 4 then return 700 - 70 * a end
  return 500 - 60 * a
end

-- Ausgabe ueber das Ethos-Vario (Konfiguration "Varioton-Ausgabe" = Ethos, vtDev 2), ab 1.29: Das Widget spielt
-- dann keinen eigenen Varioton, sondern liefert den Wert in die Lua-Quelle "SkyNav Vario" (m/s). Die
-- Sonderfunktion "Vario abspielen" mit dieser Quelle (Mitte: Still) macht daraus den Ton des Senders: durchgehender
-- Sinkton und gleichmaessiger Takt, weil Ethos den Ton selbst im Audio-Takt erzeugt. Lua kann das nicht: playTone
-- startet einen Ton nur in einem wakeup (Abstaende schwanken), darum Luecken oder Ueberlappungen im Sinkton und
-- unruhiger Takt im Steigen (Christoph 12.11.2026). Wenn das Widget nicht spielen soll (Sollfahrt, Wendeton
-- hat Vorrang, Ereignistoene), liefert die Quelle 0, das liegt in der stillen Mitte. Wert ungefiltert wie Klang 4.
BAL.VO = {v = 0, t = -1}
-- Ab 1.31 zusaetzlich als Telemetrie-Sensor "SkyNav Vario" (DIY, AppID 0x5FF0, m/s): "Vario abspielen" bietet
-- nur Telemetrie-Sensoren mit m/s an, die Lua-Quelle erschien dort nicht (Christoph 13.11.2026). Das Widget legt
-- den Sensor beim ersten Mal selbst an (sucht zuerst nach der AppID, damit keine Doppel entstehen; Fehlversuch
-- hoechstens alle 10 s neu) und schreibt den Wert in jedem wakeup.
BAL.VO.APPID = 0x5FF0
function BAL.VO.sensor(w, now)
  local s = w.voSens
  if s then return s end
  if w.voTry and now - w.voTry < 10 then return nil end
  w.voTry = now
  local cat = rawget(_G, "CATEGORY_TELEMETRY_SENSOR") or rawget(_G, "CATEGORY_TELEMETRY")
  local ok
  if cat and system and system.getSource then
    -- nach AppID (nur wenn der gefundene Sensor sie wirklich hat), sonst nach Namen
    ok, s = pcall(system.getSource, {category = cat, appId = BAL.VO.APPID})
    if not ok or not BAL.SN.isSet(s) then s = nil end
    if s then local id = BAL.SN.call(s, "appId"); if id ~= nil and id ~= BAL.VO.APPID then s = nil end end
    if not s then
      ok, s = pcall(system.getSource, {category = cat, name = "SkyNav Vario"})
      if not ok or not BAL.SN.isSet(s) then s = nil end
    end
  end
  local new = false
  if not s and model and model.createSensor and rawget(_G, "SENSOR_TYPE_DIY") then
    ok, s = pcall(model.createSensor, {type = SENSOR_TYPE_DIY})
    if ok and s then
      new = true
      pcall(s.name, s, "SkyNav Vario")
      pcall(s.appId, s, BAL.VO.APPID)
      pcall(s.physId, s, 0)
    else s = nil end
  end
  -- 1.34: HF-Modul des Sensors = Modul eines echten Telemetrie-Sensors (Vario, GPS, Hoehe), auch bei einem schon
  -- angelegten Sensor. Ohne gueltiges Modul gilt der Sensor fuer Ethos als verloren (rot) und nimmt keinen Wert an
  -- (Christoph 13.11.2026; dashx und rfsuite setzen das Modul ebenfalls).
  if s then
    local m
    for _, k in ipairs({"vario", "relAlt", "gps", "gpsAlt", "gpsSpd"}) do
      local src = w[k]
      if m == nil and src and src ~= s then
        local x = BAL.SN.call(src, "module")
        if type(x) == "number" then m = x end
      end
    end
    w.voMod = m
    if m ~= nil and BAL.SN.call(s, "module") ~= m then pcall(s.module, s, m) end
  end
  -- 1.32: Ein DIY-Sensor rechnet intern ganzzahlig (protocolDecimals 0): 0.3 m/s kam als 0 an, der Sensor zeigte
  -- immer 0 (Christoph 13.11.2026). Darum intern 2 Dezimalen, auch bei einem schon angelegten Sensor; Einheit m/s.
  if s then
    if UNIT_METER_PER_SECOND then pcall(s.unit, s, UNIT_METER_PER_SECOND); pcall(s.protocolUnit, s, UNIT_METER_PER_SECOND) end
    pcall(s.protocolDecimals, s, 2)
    pcall(s.decimals, s, 2)
    if new then pcall(s.minimum, s, -20); pcall(s.maximum, s, 20) end
  end
  w.voSens, w.voMode, w.voBad, w.voGood, w.voLock, w.voSw, w.voLast = s, nil, 0, 0, nil, 0, nil
  return s
end
-- 1.33: Der Sensor zeigte am Sender weiter 0 (Christoph 13.11.2026). Wie Ethos den Wert eines Lua-Sensors annimmt,
-- ist je nach Version verschieden und schlecht dokumentiert. Darum probiert das Widget die Schreibwege selbst
-- durch und liest den Sensor zurueck: 1 = value(m/s), 2 = rawValue(0.01 m/s, ganzzahlig), 3 = rawValue(m/s).
-- Stimmt der zurueckgelesene Wert 5-mal, bleibt es dabei; stimmt er 1 s lang nicht, kommt der naechste Weg.
-- Passt nach zwei Runden keiner (oder laesst sich nichts zuruecklesen), gilt wieder der erste Weg.
-- Die Konfiguration zeigt den Stand (BAL.VO.info).
-- 1.35: vierter Weg value(0.01 m/s, ganzzahlig); pro Weg merkt sich das Widget gesendet / gelesen / Alter (voT)
-- fuer die Konfiguration, weil am Sender die langen Zeilen abgeschnitten waren.
function BAL.VO.write(s, mode, v)
  if mode == 2 then return pcall(s.rawValue, s, math.floor(v * 100 + 0.5)) end
  if mode == 3 then return pcall(s.rawValue, s, v) end
  if mode == 4 then return pcall(s.value, s, math.floor(v * 100 + 0.5)) end
  return pcall(s.value, s, v)
end
function BAL.VO.set(w, now, on)
  local v = on and w.varioRaw or 0
  v = clamp(v, -20, 20)
  BAL.VO.v, BAL.VO.t = v, now
  local s = BAL.VO.sensor(w, now)
  if not s then return end
  if not w.voMode then
    local ok, ver = pcall(system.getVersion)
    local mj = ok and type(ver) == "table" and tonumber(ver.major) or 0
    -- 1.36: Am Sender (Ethos 1.6.8, Christoph 13.11.2026) geht nur Weg 4: value() nimmt den Rohwert ganzzahlig in
    -- 0.01 m/s (Weg 1 kam als 0.00 an, rawValue wurde gar nicht uebernommen). Darum dort zuerst Weg 4, ab Ethos 26
    -- zuerst rawValue ganzzahlig (Weg 2); die Probe mit Zuruecklesen bleibt als Absicherung.
    w.voFirst = (mj >= 26 and type(s.rawValue) == "function") and 2 or 4
    w.voMode = w.voFirst
  end
  local last = w.voLast
  if not w.voLock and last and math.abs(last) >= 0.05 then
    local ok, rb = pcall(s.value, s)
    w.voRb = ok and type(rb) == "number" and rb or nil
    w.voT = w.voT or {}
    w.voT[w.voMode] = {s = last, g = w.voRb, a = BAL.SN.call(s, "age")}
    if w.voRb and math.abs(w.voRb - last) <= 0.03 then
      w.voGood, w.voBad = (w.voGood or 0) + 1, 0
      if w.voGood >= 5 then w.voLock = "ok" end
    else
      w.voBad, w.voGood = (w.voBad or 0) + 1, 0
      if w.voBad >= 20 then
        w.voBad, w.voSw = 0, (w.voSw or 0) + 1
        if w.voSw >= 8 then w.voMode, w.voLock = w.voFirst, "blind"
        else w.voMode = w.voMode % 4 + 1 end
      end
    end
  end
  if (w.voMode == 2 or w.voMode == 3) and type(s.rawValue) ~= "function" then w.voMode = w.voMode == 2 and 4 or 1 end
  if BAL.VO.write(s, w.voMode, v) then w.voLast, w.voErr = v, 0
  else
    w.voLast, w.voErr = nil, (w.voErr or 0) + 1
    if not w.voLock then w.voMode = w.voMode % 4 + 1 end
    if w.voErr >= 6 then w.voSens = nil end                    -- Sensor geloescht? In 10 s neu suchen/anlegen
  end
end
-- Stand fuer die Konfiguration: Sensor, Wert, ob das Vario gerade dran ist
function BAL.VO.info(w)
  local t = {}
  local s = w.voSens
  if not s then t[1] = "Sensor fehlt"
  elseif w.voLock == "ok" then t[1] = "Sensor ok, Weg " .. tostring(w.voMode)
  elseif w.voLock == "blind" then t[1] = "kein Weg passt"
  else t[1] = "Probe Weg " .. tostring(w.voMode or "-") end
  if not w.varioTone then t[2] = "still: Ton in App aus"
  elseif w.varioRaw == nil then t[2] = "still: kein Vario"
  elseif w.vtOn then t[2] = string.format("aktiv %+.2f", BAL.VO.v or 0)
  else t[2] = "still: Seite/Kreisen" end
  local ok, ver = pcall(system.getVersion)
  t[3] = "Ethos " .. tostring(ok and type(ver) == "table" and ver.version or "?")
  t[4] = "Modul " .. tostring(s and BAL.SN.call(s, "module")) .. "/" .. tostring(w.voMod) .. " W" .. tostring(w.voSw or 0)
  -- pro Schreibweg: gesendet > gelesen, Alter in ms
  for m = 1, 4 do
    local r = w.voT and w.voT[m]
    if r then
      t[#t + 1] = string.format("%d: %+.2f>%s %sms", m, r.s, r.g and string.format("%+.2f", r.g) or "nil", tostring(r.a))
    else t[#t + 1] = m .. ": nicht probiert" end
  end
  return t
end
-- 1.37 Rechenlast messen: Der Sinkton des Ethos-Varios klickert, sobald ein Widget rechnet oder zeichnet
-- (Christoph 13.11.2026): Ethos schiebt den Dauerton in kurzen Stuecken nach, und das stockt, wenn Lua oder der
-- Bildaufbau den Takt aufhalten. Gemessen werden pro 10 s: Dauer von wakeup und paint (Mittel/Maximum) und der
-- Abstand zwischen zwei wakeup (Takt; enthaelt auch das Zeichnen von Ethos selbst). Anzeige in der Konfiguration.
-- Dazu "Bildaufbau" (drawIv): wie oft das Widget neu zeichnen laesst.
BAL.TM = {CV = {1, 2, 4}, IV = {0.15, 0.3, 0.6, 1.0}, c = {}, r = nil, t0 = nil, last = nil}
function BAL.TM.add(k, d)
  local c = BAL.TM.c
  local e = c[k]
  if not e then e = {n = 0, s = 0, m = 0}; c[k] = e end
  e.n, e.s = e.n + 1, e.s + d
  if d > e.m then e.m = d end
  if not e.lo or d < e.lo then e.lo = d end
  if d > 0.08 then e.hi = (e.hi or 0) + 1 end
end
function BAL.TM.wake(f, w)
  local T = BAL.TM
  local t = os.clock()
  if T.last then T.add("g", t - T.last) end
  T.last = t
  -- 1.39 Rechentakt (calcIv): Der Sinkton des Ethos-Varios klickert auch auf dem Hauptbildschirm, also ohne
  -- Zeichnen (Christoph 13.11.2026); der Takt lag bei rund 100 ms, davon 20 bis 27 ms Rechnen. Zum Eingrenzen
  -- rechnet das Widget wahlweise nur in jedem 2. oder 4. Aufruf voll; dazwischen schreibt es nur den Vario-Wert.
  T.wn = (T.wn or 0) + 1
  local iv = BAL.TM.CV[w.calcIv or 1] or 1
  if iv > 1 and T.wn % iv ~= 0 then
    local s = w.voSens
    if s and w.vtDev == 2 and w.voMode then BAL.VO.write(s, w.voMode, BAL.VO.v or 0) end
    T.add("l", os.clock() - t)
    return
  end
  f(w)
  local t2 = os.clock()
  T.add("w", t2 - t)
  if not T.t0 then T.t0 = t2 elseif t2 - T.t0 >= 10 then T.r, T.c, T.t0 = T.c, {}, t2 end
end
-- 1.39: Am Sender (X20, Ethos 1.6.8) braucht ein Bild rund 125 ms, das Rechnen nur 20 bis 27 ms (Christoph
-- 13.11.2026). Darum misst jedes 4. Bild zusaetzlich jeden Zeichenbefehl einzeln (lcd ueber einen Zaehler
-- geleitet): Zeit und Anzahl pro Befehl, Anzeige in der Konfiguration, die teuersten zuerst.
function BAL.TM.proxy()
  local T = BAL.TM
  if T.px then return T.px end
  local real = lcd
  T.real, T.L = real, {}
  T.px = setmetatable({}, {__index = function(t, k)
    local f = real[k]
    if type(f) ~= "function" then return f end
    local g = function(...)
      local t0 = os.clock()
      local a, b, c = f(...)
      local e = T.L[k]
      if not e then e = {n = 0, s = 0}; T.L[k] = e end
      e.n, e.s = e.n + 1, e.s + (os.clock() - t0)
      return a, b, c
    end
    rawset(t, k, g)
    return g
  end})
  return T.px
end
function BAL.TM.paint(f, w)
  local T = BAL.TM
  T.pn = (T.pn or 0) + 1
  local probe = T.pn % 4 == 0
  if probe then local px = T.proxy(); lcd = px; T.Ln = (T.Ln or 0) + 1 end
  local t = os.clock()
  local ok, err = pcall(f, w)
  local d = os.clock() - t
  if probe then lcd = T.real; T.add("q", d) else T.add("p", d) end
  if not ok then error(err, 0) end
end
function BAL.TM.info()
  local r, t = BAL.TM.r, {}
  if not r then return {"Messung laeuft (10 s)"} end
  for _, d in ipairs({{"w", "Rechnen"}, {"l", "Leer"}, {"p", "Zeichnen"}, {"g", "Takt"}}) do
    local e = r[d[1]]
    if e and e.n > 0 then t[#t + 1] = string.format("%s %.0f/%.0f ms n%d", d[2], e.s / e.n * 1000, e.m * 1000, e.n) end
  end
  local g = r.g
  if g and g.n > 0 then t[#t + 1] = string.format("Takt min %.0f, >80ms %d%%", (g.lo or 0) * 1000, math.floor((g.hi or 0) / g.n * 100 + 0.5)) end
  -- Zeichenbefehle: Mittel pro Bild (ms) und Anzahl pro Bild, seit dem Einschalten
  local L, n = BAL.TM.L, BAL.TM.Ln or 0
  if L and n > 0 then
    local ks = {}
    for k in pairs(L) do ks[#ks + 1] = k end
    table.sort(ks, function(a, b) return L[a].s > L[b].s end)
    for i = 1, math.min(#ks, 7) do
      local e = L[ks[i]]
      t[#t + 1] = string.format("%s %.0fms x%.0f", ks[i]:gsub("^draw", ""), e.s / n * 1000, e.n / n)
    end
  end
  return t
end

function BAL.VO.init(source)
  source:value(0)
  source:decimals(2)
  if UNIT_METER_PER_SECOND then pcall(source.unit, source, UNIT_METER_PER_SECOND) end
end
function BAL.VO.wakeup(source)
  local o = BAL.VO
  -- Widget laeuft nicht (andere Seite der Bildschirme, Modell gewechselt): still
  source:value(os.clock() - o.t < 1 and o.v or 0)
end

function BAL.VT.play(w, now)
  local v = w.vtV
  if not w.vtOn or v == nil then w.vtNext, w.vtEnd = nil, nil; return end
  if w.quietUntil and now < w.quietUntil then return end      -- kurz nach Ereignistoenen (Sektor, Start ...)
  local rg = w.vtRange or 10                                  -- Tonbereich (App vtRange, m/s): bei rg klingt der Ton wie
  local iv = clamp(v * 100, -rg * 100, rg * 100)              -- die Kurven bei 5 m/s (Steigen und Sinken gestreckt)
  if iv >= -(w.vzLo or 3) * 10 and iv <= (w.vzHi or 1) * 10 then w.vtNext, w.vtEnd = nil, nil; return end
  local busy = w.vtEnd or 0                                   -- bis dann klingt der zuletzt bestellte Ton
  local st = w.vtStyle or 1
  if iv > 0 then
    if now < busy or (w.vtNext and now < w.vtNext) then return end
    local f, per, on = BAL.VT.climb(st, iv / 20 / rg)
    tone(w, w.page, f, on)
    -- Takt aus der geplanten Zeit (nicht aus "jetzt"), damit der Rhythmus im 50-ms-Raster von wakeup nicht driftet
    local base = (w.vtNext and now - w.vtNext < 0.1) and w.vtNext or now
    w.vtNext, w.vtEnd = base + per / 1000, now + on / 1000
  else
    -- durchgehender Ton: Stuecke von 250 ms (Klang 4: 150 ms, damit der Wechsel ins Steigen schneller kommt),
    -- das naechste kurz vor dem Ende des laufenden bestellt
    if now < busy - 0.08 then return end
    local len = st == 4 and 150 or 250
    tone(w, w.page, BAL.VT.sink(st, iv / 20 / rg), len)
    w.vtNext, w.vtEnd = nil, math.max(now, busy) + len / 1000
  end
end

-- ---------------------------------------------------------------- Restzeit-Sollfahrt (Zeit-MC)
-- Wie viele Dreiecke passen noch in die Arbeitszeit (Netto pro Schenkel, Wind, Steigen, Hoehe), und welcher
-- MC schafft sie in der kuerzesten Zeit (Zeit-MC)? Pro Schenkel i und MC m: Sollfahrt v (Polare, Gegenwind,
-- Netto des Schenkels), Zeit pro Meter 1/vg, Hoehenverlust pro Meter (Sinken - Netto)/vg. N Dreiecke (das
-- laufende zaehlt als erstes): Gleitzeit = Index k * (Rest dieser Runde + (N-1) Runden), die Hoehe, die ueber
-- hmin fehlt, wird mit Vc gestiegen. Pilotfaktor z auf den letzten zwei Runden, Lernfaktor f = echte durch
-- modellierte Rundenzeit. Zeit-MC = m mit der kuerzesten Zeit fuer N_max (bei Gleichstand zaehlt die Zeit; bei zu
-- viel Hoehe ergibt das von selbst mehr Fahrt). Zwei Portionen (Tabelle, Suche) in Aufrufen ohne GPS-Fix,
-- hoechstens einmal pro Sekunde (Instruktionsgrenze).
local RZ = {M = {}}
for i = 0, 24 do RZ.M[i + 1] = i * 0.25 end

-- Steigen aus den Baerten {t = Mitte, v = Steigen, d = Dauer}: Mittel nach Dauer, jeder aeltere Bart zaehlt halb
-- so viel wie der naechste. Ab 3 Baerten ueber mind. 2 min der Trend (Gerade nach Dauer gewichtet): nimmt das
-- Steigen ab, gilt fuer die Restzeit der Wert der Geraden in der Mitte der Restzeit (nicht weiter voraus als die
-- halbe Zeitspanne der Baerte), hoechstens auf die Haelfte des Mittels herunter. Zunehmendes Steigen wird nicht
-- eingeplant.
function RZ.climb(bs, now, rest)
  local n = #bs
  if n == 0 then return nil end
  local sw, sv, q = 0, 0, 1
  for i = n, 1, -1 do
    local g = bs[i].d * q
    sw, sv, q = sw + g, sv + g * bs[i].v, q / 2
  end
  local vc = sv / sw
  if n < 3 or bs[n].t - bs[1].t < 120 then return vc, vc end
  local st, sx, sy, sxx, sxy = 0, 0, 0, 0, 0
  for i = 1, n do
    local g, x = bs[i].d, bs[i].t - now
    st, sx, sy, sxx, sxy = st + g, sx + g * x, sy + g * bs[i].v, sxx + g * x * x, sxy + g * x * bs[i].v
  end
  local den = st * sxx - sx * sx
  if den <= 0 then return vc, vc end
  local b = (st * sxy - sx * sy) / den
  if b >= 0 then return vc, vc end
  local a = (sy - b * sx) / st
  return vc, clamp(a + b * math.min(math.max(rest, 0), bs[n].t - bs[1].t) / 2, vc / 2, vc)
end

-- Lernen und Tabelle pro MC-Wert: Rest dieser Runde (TP, HP) und ganze Runde (TL, HL), ohne Index k
function RZ.table(w, now)
  local T, f, p, r = w.task, w.cur, w.polar, w.rz
  if not (T and f and p and w.rzOn and w.state == S_STARTED and w.startT) then w.rz = nil; return end
  if not r or r.startT ~= w.startT then
    r = {startT = w.startT, net = {0, 0, 0, 0}, ks = {}, nl = #w.laps, mcS = 0, mcN = 0, dist = 0, tL = now, tS = now, th = w.lastTherm}
    w.rz = r
  end
  local dt = math.min(now - r.tL, 3)
  r.tL = now
  local leg = math.min(w.tp, 3) + 1
  -- Gleitstrecke ohne Baerte (fuer den Index), Netto nur geradeaus. Strecke im Kreisen erst merken: war es
  -- nur eine Wende (Bart unter 20 s), zaehlt sie doch, nach einem Bart die 8 s Geradeausflug am Ende.
  local ld = w.lapDist or 0
  local dd = ld >= (r.ld or 0) and ld - (r.ld or 0) or ld
  r.ld = ld
  if w.therm then r.thd = (r.thd or 0) + dd else
    r.dist = r.dist + dd
    -- Motor: Netto ueber 4 m/s (wie bei den Baerten), dazu 10 s danach (Netto ist 8 s geglaettet). Starkes
    -- Steigen im Delphinflug zaehlt mit.
    if (w.netto or 0) > 4 or (w.teN or 0) > 5 then r.motT = now end
    if w.netto and (w.omega or 0) < math.rad(8) and not (r.motT and now - r.motT < 10) then r.net[leg] = r.net[leg] + (clamp(w.netto, -3, 4) - r.net[leg]) * dt / 60 end
  end
  -- Steigen: nur aus ausgekurbelten Baerten seit dem Start (mind. 20 s und 15 m Gewinn, 0.2 bis 4 m/s, mehr
  -- ist Motor), die letzten 6 fuer RZ.climb. Vorher rechnet die Restzeit ohne Steigen (nur Abgleiten).
  local lt = w.lastTherm
  if lt and lt ~= r.th then
    r.th = lt
    local dur = (lt.tEnd or now) - lt.t0
    r.dist, r.thd = r.dist + (r.thd or 0) * (dur < 20 and 1 or 8 / (dur + 8)), 0
    local gain = lt.hMax - lt.h0
    if lt.t0 >= r.tS and dur >= 20 and gain >= 15 and gain / dur >= 0.2 and gain / dur <= 4 then
      r.bs = r.bs or {}
      r.bs[#r.bs + 1] = {t = lt.t0 + dur / 2, v = gain / dur, d = dur}
      if #r.bs > 6 then table.remove(r.bs, 1) end
    end
  end
  -- Runde fertig: Index und Lernfaktor f (Hoehengewinn/-verlust der Runde mit Vc umgerechnet)
  if #w.laps > r.nl then
    r.nl = #w.laps
    local L = w.laps[r.nl]
    r.ks[#r.ks + 1] = clamp(r.dist / T.perim, 1, 1.6)
    if #r.ks > 3 then table.remove(r.ks, 1) end
    r.dist = 0
    if r.TL and r.mcN > 0 then
      local mi = clamp(math.floor(r.mcS / r.mcN * 4 + 1.5), 1, r.nm)
      local tg, dh = r.k * r.TL[mi], r.k * r.HL[mi]
      local q = clamp(L.time / (r.vc and math.max(tg * 0.8, tg + (dh + (L.dAlt or 0)) / r.vc) or tg), 0.7, 2.5)
      r.f = r.f and (r.f + q) / 2 or q
    end
    r.mcS, r.mcN = 0, 0
  end
  local k = 0
  for _, v in ipairs(r.ks) do k = k + v end
  r.k = #r.ks > 0 and k / #r.ks or 1.05
  -- Steigen fuer die Dreiecke: nur ausgekurbelte Baerte (ohne: fast 0, also kein Steigen eingeplant), mit
  -- abnehmendem Trend fuer die Restzeit. Fuer die Abflughoehe im Bart zaehlt der laufende Bart ab 20 s, sonst
  -- die gelernten.
  local tm = w.therm
  local vt = tm and tm.h and now - tm.t0 >= 20 and (tm.h - tm.h0) / (now - tm.t0) or nil
  local vce
  if r.bs then r.vc, vce = RZ.climb(r.bs, now, T.work * 60 - (f.clk - w.startT)) end
  r.vcE = vce
  r.vcU = vce or 0.001
  r.vcG = math.max(0.2, vt or vce or 0.2)
  if r.mcz then r.mcS, r.mcN = r.mcS + (w.rzMan and w.mcValue or r.mcz), r.mcN + 1 end
  -- Schenkel O-TP1-TP2-TP3-O: Laenge, Wind laengs (positiv = Rueckenwind) und quer
  local wx, wy = w.windOk and w.wx or 0, w.windOk and w.wy or 0
  local len, wa, wq = {}, {}, {}
  local ax, ay = 0, 0
  for i = 1, 4 do
    local b = T.tp[i] or {x = 0, y = 0}
    local dx, dy = b.x - ax, b.y - ay
    local l = math.sqrt(dx * dx + dy * dy)
    len[i], wa[i], wq[i] = l, (wx * dx + wy * dy) / l, (wx * dy - wy * dx) / l
    ax, ay = b.x, b.y
  end
  local d0 = leg < 4 and math.sqrt((T.tp[leg].x - f.x) ^ 2 + (T.tp[leg].y - f.y) ^ 2) or math.max(-(f.x * T.ux + f.y * T.uy), 0)
  local TP, HP, TL, HL = {}, {}, {}, {}
  local nm = #RZ.M
  for mi = 1, nm do
    local tl, hl, tp, hp, over = 0, 0, 0, 0, true
    for i = 1, 4 do
      local n = r.net[i]
      local v, o = speedToFly(p, RZ.M[mi], n, -wa[i])
      over = over and o
      local vg = math.max(math.sqrt(math.max(v * v - wq[i] * wq[i], 1)) + wa[i], 1)
      local a, b = 1 / vg, (polarSink(p, v) - n) / vg
      tl, hl = tl + len[i] * a, hl + len[i] * b
      if i == leg then tp, hp = d0 * a, d0 * b elseif i > leg then tp, hp = tp + len[i] * a, hp + len[i] * b end
    end
    TP[mi], HP[mi], TL[mi], HL[mi] = tp, hp, tl, hl
    -- ab hier ueberall Vmax: groessere MC-Werte aendern nichts mehr
    if over then nm = mi; break end
  end
  r.TP, r.HP, r.TL, r.HL, r.nm = TP, HP, TL, HL, nm
end

-- Suche: N_max (Bisektion), Zeit-MC, Zeitreserve, was fuer ein Dreieck mehr fehlt, Abflughoehe aus dem Bart
function RZ.search(w, now)
  local r, T, f = w.rz, w.task, w.cur
  if not (r and r.TL and f and T) then return end
  local rest = T.work * 60 - (f.clk - w.startT)
  local avail = rest - math.max(15, rest * w.rzRes / 100)
  local F = r.f or 1
  local availM = avail / F
  local k, z, Vc, nm = r.k, w.rzPilot / 100, r.vcU, r.nm
  local TP, HP, TL, HL = r.TP, r.HP, r.TL, r.HL
  local hmax, hmin = T.maxAlt - 30, w.rzHmin
  local dh0 = math.min(f.agl, hmax) - hmin
  -- Zeit fuer N Dreiecke mit MC-Index mi (Pilotfaktor auf den letzten zwei Runden)
  local function ev(N, mi, h0)
    local cp, cl = k * (N <= 2 and 1 + z or 1), k * (N - 1 + z * math.min(2, N - 1))
    local tg, dh = cp * TP[mi] + cl * TL[mi], cp * HP[mi] + cl * HL[mi]
    return tg + math.max(0, dh - h0) / Vc, tg, dh
  end
  -- kleinster MC-Index, mit dem N Dreiecke passen
  local function fits(N) for mi = 1, nm do if ev(N, mi, dh0) <= availM then return mi end end end
  local lo, hi = 0, 1
  while hi < 64 and fits(hi) do lo, hi = hi, hi * 2 end
  while hi - lo > 1 do
    local mid = math.floor((lo + hi) / 2)
    if fits(mid) then lo = mid else hi = mid end
  end
  local N, mi = lo, lo > 0 and fits(lo) or nil
  -- Bei gleich vielen Dreiecken zaehlt die schnellere Zeit: Zeit-MC = MC mit der kuerzesten Zeit fuer die N
  -- Dreiecke (mit Kreisen bei fehlender Hoehe; uebrige Hoehe bis hmin wird in Fahrt umgesetzt)
  if mi then
    local tb = ev(N, mi, dh0)
    for j = mi + 1, nm do
      local t = ev(N, j, dh0)
      if t < tb then mi, tb = j, t end
    end
  end
  -- Fest pro Runde (sonst springt der Wert mit jeder kleinen Aenderung, die Zeit ist um das Optimum flach):
  -- neu gerechnet bei neuer Runde, beim Verlassen eines Barts und wenn der feste Wert die Dreiecke nicht mehr
  -- schafft. Gilt auch fuer die Anzeige MC ZEIT, wenn mit normalem MC geflogen wird.
  if mi then
    local L = r.lock
    if L and L.lap == #w.laps and L.th == w.lastTherm and L.mi <= nm and ev(N, L.mi, dh0) <= availM then
      mi = L.mi
    else
      r.lock = {mi = mi, lap = #w.laps, th = w.lastTherm}
    end
  end
  r.mcz = mi and RZ.M[mi]
  -- Mit normalem MC zaehlen die Dreiecke mit dem eingestellten MC (hoechstens so viele wie mit dem Zeit-MC)
  local mm = w.rzMan and clamp(math.floor(w.mcValue * 4 + 1.5), 1, nm)
  if mm then
    while N > 0 and ev(N, mm, dh0) > availM do N = N - 1 end
    mi = N > 0 and mm or nil
  end
  local best = 1e9
  for j = mm or 1, mm or nm do best = math.min(best, (ev(N + 1, j, dh0))) end
  r.N, r.behind = N, best * F - avail
  local _, _, dhN = ev(N, mi or 1, dh0)
  r.slack = mi and rest - ev(N, mi, dh0) * F or nil
  -- Hoehe reicht ohne Kreisen und der Zeit-MC liegt ueber dem Steigen: Hoehe wird gezielt in Fahrt umgesetzt
  -- (nur mit MC ZEIT: mit normalem MC bestimmt der Pilot die Fahrt). Nur wenn der Plan wirklich mindestens die
  -- halbe verfuegbare Hoehe (und 20 m) verbraucht; im Delphinflug mit Netto haelt man die Hoehe.
  r.down = not w.rzMan and mi ~= nil and dhN <= dh0 and dhN >= math.max(20, dh0 / 2) and RZ.M[mi] > Vc
  -- kein Dreieck mehr moeglich: bestes Gleiten (MC 0), ab jetzt zaehlt nur noch sicheres Landen
  if not r.mcz then r.mcz = 0 end
  -- Abflughoehe fuer den Endanflug ohne weiteren Bart: die mit der kuerzesten Gesamtzeit (Steigen hier plus
  -- Gleiten) fuer die N Dreiecke, unter dem Hoehenlimit; geht keine darunter, die tiefste
  r.hgo = nil
  if mi then
    local tb, nb, lowN
    for j = mm or 1, mm or nm do
      local _, tg, dh = ev(N, j, 1e9)
      local need = hmin + dh
      local t = math.max(0, need - f.agl) / r.vcG + tg
      if t <= availM then
        if need <= hmax then
          if not tb or t < tb then tb, nb = t, need end
        elseif not lowN or need < lowN then lowN = need end
      end
    end
    r.hgo = nb or lowN
  end
  r.over = r.hgo and r.hgo > hmax
  -- Anzeige: erst ab der zweiten Runde (Netto pro Schenkel und Rundenzeit bekannt), neue Zahl erst, wenn sie
  -- 10 s gleich bleibt
  if N ~= r.Nn then r.Nn, r.Nt = N, now end
  if not r.Ns or now - r.Nt >= 10 then r.Ns = N end
  -- im Bart die Abflughoehe erreicht: einmal pro Bart melden
  local th = w.therm
  if th and r.hgo and not r.over and f.agl >= r.hgo and r.goTh ~= th then
    r.goTh = th
    tone(w, P_THERM, 1200, 150); tone(w, P_THERM, 1600, 150); tone(w, P_THERM, 2000, 400); haptic(w, P_THERM, 300)
    sayWord(w, P_THERM, "endanflug.wav")
  end
end

-- ---------------------------------------------------------------- Seitenwahl

-- Ein Schalter (mit Stellung) je Seite. Die Seite, deren Schalter zuletzt eingeschaltet wurde, gilt, solange er
-- eingeschaltet bleibt. Ist keiner ein, waehlt die Automatik bzw. das Tippen auf die Kopfzeile.
local PAGE_SW = {"swOver", "swStart", "swGlide", "swTherm", "swEval"}

local function pageFromSwitch(w, now)
  w.swState = w.swState or {}
  local any = false
  for p, key in ipairs(PAGE_SW) do
    if validSource(w[key]) then
      any = true
      local on = (readNumber(w[key]) or -1024) > 0
      if on and not w.swState[p] then
        if not (w.swPage and w.swState[w.swPage]) then
          w.swRestore = {page = w.page, manual = w.manual, manualFrom = w.manualFrom}
        end
        w.swPage, w.manual = p, nil
      end
      w.swState[p] = on
    else
      w.swState[p] = nil
    end
  end
  if any then
    if not (w.swPage and w.swState[w.swPage]) then
      w.swPage = nil
      for p = 1, #PAGE_SW do if w.swState[p] then w.swPage = p; break end end
    end
    if w.swPage then return w.manual or w.swPage end
    -- Alle Seitenschalter aus: die Ansicht von vor dem Einschalten zurueckholen. Sie bleibt, bis die Automatik
    -- ein neues Ziel hat oder der Pilot tippt.
    local r = w.swRestore
    if r then
      w.swRestore = nil
      w.manual, w.manualFrom = r.manual, r.manualFrom
      if not w.manual and r.page and r.page ~= (w.autoPage and w.autoTarget or r.page) then
        w.manual, w.manualFrom = r.page, w.autoTarget
      end
      if not w.manual and r.page then return r.page end
    end
  end
  if w.manual then
    if w.autoTarget and w.autoTarget ~= w.manualFrom then w.manual = nil else return w.manual end
  end
  if w.autoPage and w.autoTarget then return w.autoTarget end
  return w.page
end

-- ---------------------------------------------------------------- wakeup

-- Scharf stellen oder, nach einem misslungenen Start (gestartet, noch keine Runde fertig), neu starten
-- viaSw = "up" (Schalter eingeschaltet) oder "down" (ausgeschaltet); nil = Tippen.
local function armOrRestart(w, now, viaSw)
  if not w.task then return end
  -- Ausschalten zaehlt nur als zweite Betaetigung beim Bestaetigen eines neuen Wertungsflugs
  if viaSw == "down" and not (w.state == S_STARTED and #w.laps > 0) then return end
  if w.state == S_IDLE then
    w.state, w.armedAt, w.armPending = S_ARMED, now, true
  elseif w.state == S_STARTED and #w.laps == 0 then
    w.state, w.armedAt, w.armPending = S_ARMED, now, true
    w.startT, w.startUtc, w.lapT, w.entry, w.penalty, w.tp, w.lapAgl = nil, nil, nil, nil, 0, 0, nil
    w.zoneHit, w.altHit, w.badEntry = nil, nil, nil
    w.restarts = (w.restarts or 0) + 1
    note(w, "Neustart: wieder scharf", now)
  elseif w.state == S_STOPPED or w.state == S_STARTED then
    -- Neuer Wertungsflug in der Luft (z. B. nach Motorsteigflug): bei fertigen Runden erst nach
    -- zweiter Betaetigung (Tippen innert 4 s, Schalter: zwei Umschaltungen in beliebiger Richtung innert 3 s,
    -- also ein-aus oder aus-ein). Das IGC des alten Versuchs wird geschlossen, ein neues beginnt.
    if w.state == S_STARTED and not (w.rsArm and now - w.rsArm < (w.rsWin or 4)) then
      w.rsArm, w.rsWin = now, viaSw and 3 or 4
      note(w, "Nochmal = neuer Wertungsflug", now)
      return
    end
    local fl, tk = w.flying, w.takeoffT
    resetFlight(w)
    w.flying, w.takeoffT, w.rsArm, w.rsWin = fl, tk, nil, nil
    w.state, w.armedAt, w.armPending = S_ARMED, now, true
    w.restarts = (w.restarts or 0) + 1
    note(w, "Neuer Wertungsflug: scharf", now)
  end
end

local function wakeup(w)
  local now = os.clock()
  local dt = w.lastT and (now - w.lastT) or 0
  w.lastT = now
  if dt > 0.5 then dt = 0.5 end
  refreshTasks(w, now, false)
  refreshPolar(w, now, false)
  -- Ballast: in 0.1 kg, mit Hysterese gegen Poti-Rauschen; eine Aenderung faerbt die Kopfzeile 3 s orange
  local bk, bfr = BAL.now(w)
  w.balFr = bfr
  if not w.balKg or math.abs(bk - w.balKg) > 0.07 then
    if w.balKg then w.balChgT = now end
    w.balKg = math.floor(bk * 10 + 0.5) / 10
    BAL.polar(w)
  end
  -- MC ZEIT oder fester MC: mit MC-Poti gilt der Poti-Wert, am Anschlag oben (ab 99 %) MC ZEIT; ohne Poti
  -- der feste MC, Tippen auf das MC-Feld schaltet auf MC ZEIT und zurueck
  if not validSource(w.mcSrc) then w.mcValue = w.mc / 10; w.rzMan = not w.rzTap else
    local v = readNumber(w.mcSrc)
    if v then w.mc = math.floor(clamp((v + 1024) / 2048, 0, 1) * w.mcMax + 0.5); w.mcValue = w.mc / 10 end
    -- Anschlag oben = MC ZEIT. Ab 97 % (v >= 962) ein, unter 94 % wieder aus: viele Potis erreichen nach der
    -- Kalibrierung die 1024 nicht ganz (bis 1.21 brauchte es 99 %, darum liess sich MC ZEIT oft nicht waehlen).
    if v then
      if w.rzMan == false then w.rzMan = v < 901 else w.rzMan = v < 962 end
    else w.rzMan = true end
  end
  -- ARM
  -- Jede Betaetigung (Einschalten) stellt scharf, nach einem misslungenen Start startet sie neu. Taster und
  -- Kippschalter gehen beide. Nach fertigen Runden braucht ein neuer Wertungsflug zwei Umschaltungen; dort
  -- zaehlt auch das Ausschalten, damit "2x schalten" mit einem Kippschalter aus jeder Stellung klappt.
  local arm = (readNumber(w.armSw) or -1) > 0
  if w.armInit and arm ~= w.armOn then armOrRestart(w, now, arm and "up" or "down") end
  w.armOn, w.armInit = arm, true
  if not w.gpsClkPoll or now - w.gpsClkPoll >= 1 then w.gpsClkPoll = now; updateGpsClock(w, now) end

  BAL.SN.step(w, now)
  BAL.AS.read(w, now, dt)
  BAL.QL.read(w, now, dt)
  BAL.QA.step(w, now, dt)
  if w.qlPress and now - w.qlPress >= 2 then
    w.qlPress = nil
    if BAL.QL.zero(w) then tone(w, nil, 2000, 150); note(w, "Querlage genullt", now) end
  end
  local f = readInputs(w, now, dt)
  if f then
    if w.armPending then
      w.armPending = nil
      if not w.igc and w.task then igcOpen(w, f) end
      event(w, "PEV", f, now)
    end
    assess(w, f, now)
    w.fix = f
  end
  local cur = w.fix
  if cur and now - cur.clk > 3 then cur = nil end
  w.cur = cur
  if w.igc and (now - (w.igc.lastFlush or 0) >= 5) then w.igc.lastFlush = now; igcFlush(w) end
  if w.closeAt and now >= w.closeAt then w.closeAt = nil; igcClose(w) end
  igcStep(w)

  w.page = pageFromSwitch(w, now)
  -- Schwere Rechnung nur bei neuem GPS-Fix (oder Seitenwechsel) und nur fuer die sichtbare Seite.
  -- Wertung, IGC, Ereignistoene, Kreiserkennung und Netto laufen immer.
  local pg = w.page
  -- Ein neuer Fix wird im naechsten Aufruf gerechnet (Ethos bricht nach 20 000 Instruktionen pro Aufruf ab,
  -- Einlesen/Wertung und Planung zusammen waeren zu viel); spaetestens im dritten.
  if f then w.calcDue = (w.calcDue or 0) + 1 end
  local redo = (w.calcDue ~= nil and (f == nil or w.calcDue >= 3)) or not cur or pg ~= w.calcPage
  if redo then w.calcDue = nil end
  -- Aufrufe ohne Fix: Windfit, Restzeit-Sollfahrt (Tabelle, im naechsten Aufruf die Suche), nie zusammen
  if not redo and f == nil then
    if w.rzPh then w.rzPh = nil; RZ.search(w, now)
    elseif not w.rzT or now - w.rzT >= 1 then w.rzT = now; RZ.table(w, now); w.rzPh = w.rz ~= nil
    else windFit(w, now) end
  end
  w.calcPage = pg
  if pg == P_START then
    if redo then w.sc = startCalc(w, cur) end
  else
    w.sc = nil
  end
  nettoCalc(w, cur, dt)
  -- ideale Runde (blau, vor dem Start) hier rechnen statt beim Zeichnen: eigene Instruktions-Portion
  if w.task and (pg == P_OVER or pg == P_START) and w.state ~= S_STARTED and w.state ~= S_STOPPED then lapRoute(w, w.task) end
  if redo then
    -- Den ganzen Weg auch auf den anderen Seiten (und bei verstecktem Widget) rechnen, sobald eine Ecke naht:
    -- der Wendeton kommt auf jeder Seite
    local gc = w.gc
    local near = w.state == S_STARTED and gc and gc.tCorner and gc.tCorner < w.zoomSec + w.warnSec + 8
    w.gc = glideCalc(w, cur, pg == P_GLIDE or pg == P_OVER or pg == P_START or near)
  end
  thermCalc(w, cur, now, dt)

  -- Ereignistoene. Start, Sektor und Ziel (Linie bzw. Sektor erreicht) immer, auf jeder Seite und auch bei
  -- verstecktem Widget; Scharf (PEV) und Stopp nur auf der passenden Seite, ausser "Ereignisse immer melden".
  -- In der Vorrangzeit der Wendevorhersage warten sie und kommen danach.
  local prio = w.prioUntil and now < w.prioUntil
  local e = w.lastEvent and w.lastEventT == now and w.lastEvent or nil
  if e and prio then w.evDefer, e = e, nil
  elseif not e and not prio and w.evDefer then e = w.evDefer end
  if e then
    w.evDefer = nil
    local pg = (e == "STA" or e == "PEV") and P_START or P_GLIDE
    if w.eventsAlways or w.page == pg or e == "STA" or e == "TPC" or e == "FIN" then
      if e == "PEV" then tone(w, nil, 800, 120); tone(w, nil, 1000, 120)
      elseif e == "STA" then tone(w, nil, 1000, 150); tone(w, nil, 1400, 250)
        -- Ansage "Einflug OK" bzw. "<n> Strafpunkte" (audio/einflug_ok.wav, strafpunkte.wav, Zahl vom Sender)
        if w.penalty > 0 then sayNumber(w, nil, w.penalty, UNIT_RAW, 0); sayWord(w, nil, "strafpunkte.wav")
        else sayWord(w, nil, "einflug_ok.wav") end
      elseif e == "TPC" then
        -- Sektor erreicht: deutlicher Dreiklang aufwaerts und Vibration
        tone(w, nil, 1000, 180); tone(w, nil, 1400, 180); tone(w, nil, 2000, 500); haptic(w, nil, 300)
      elseif e == "FIN" then tone(w, nil, 1200, 150); tone(w, nil, 1600, 300); sayNumber(w, nil, #w.laps, UNIT_RAW, 0)
      elseif e == "STP" then tone(w, nil, 600, 400) end
      w.quietUntil = now + (e == "STA" and 3 or 1.5)          -- nach dem Start Platz fuer die Einflug-Ansage
    end
  end
  if w.zoneEnterT == now then w.zoneDefer = true end
  if w.zoneDefer and not prio then w.zoneDefer = nil; tone(w, nil, 500, 600) end

  BAL.VT.mode(w, now, dt)
  local warn = cur and turnTone(w, w.gc or {}, now)           -- Wendeton auf jeder Seite
  BAL.WV.step(w, cur, now, dt)                                 -- Vorhalt lernen, zu frueh, verpasst
  warn = warn or (w.prioUntil and now < w.prioUntil)          -- Vorrang: keine Seitentoene
  if w.vtDev == 2 then BAL.VO.set(w, now, w.vtOn and not warn and not (w.quietUntil and now < w.quietUntil)) end
  if w.vtOn then
    -- Varioton ersetzt Sollfahrt-, Start- und Kurs-Toene (Thermik-Seite, Kreisen, Gleiten mit Netto > MC)
    w.cmdTones = nil
    if not warn and w.vtDev ~= 2 then BAL.VT.play(w, now) end
  elseif cur then
    if w.page == P_START then if not warn then startTones(w, w.sc, now) end
    elseif w.page == P_GLIDE then glideTones(w, w.gc, now, warn) end
  end

  if now - w.lastInvalidate >= (BAL.TM.IV[w.drawIv or 1] or 0.15) then
    w.lastInvalidate = now
    lcd.invalidate()
  end
end

-- ---------------------------------------------------------------- Zeichnen

local function rgb(r, g, b) return lcd.RGB(r, g, b) end
local F_L, F_XL = FONT_L, FONT_XL      -- grosse Werte, auf kleinen Bildschirmen eine Stufe kleiner
local SMALL = false                    -- kleiner Bildschirm (X18 und kleiner): kurze Beschriftungen
local C_BG, C_DIM, C_WHITE, C_GREEN, C_ORANGE, C_RED, C_BLUE, C_YELLOW, C_GRID
local function colors()
  C_BG, C_DIM, C_WHITE = rgb(0, 0, 0), rgb(150, 150, 150), rgb(255, 255, 255)
  C_GREEN, C_ORANGE, C_RED = rgb(0, 200, 80), rgb(255, 170, 0), rgb(255, 60, 50)
  C_BLUE, C_YELLOW, C_GRID = rgb(0, 170, 255), rgb(255, 230, 0), rgb(55, 55, 55)
end

local function fl(v) return math.floor(v + 0.5) end
-- Linien in Kartenansichten werden auf den Kasten CLIP beschnitten (Liang-Barsky)
local CLIP = nil
local floor = math.floor
local function line(x1, y1, x2, y2)
  local C = CLIP
  if C then
    local c1, c2, c3, c4 = C[1], C[2], C[3], C[4]
    if x1 < c1 or x1 > c3 or y1 < c2 or y1 > c4 or x2 < c1 or x2 > c3 or y2 < c2 or y2 > c4 then
      -- beide Enden auf derselben Aussenseite: nichts zu zeichnen
      if (x1 < c1 and x2 < c1) or (x1 > c3 and x2 > c3) or (y1 < c2 and y2 < c2) or (y1 > c4 and y2 > c4) then return end
      -- Liang-Barsky
      local dx, dy = x2 - x1, y2 - y1
      local t0, t1 = 0, 1
      local p, q, r
      for e = 1, 4 do
        if e == 1 then p, q = -dx, x1 - c1 elseif e == 2 then p, q = dx, c3 - x1
        elseif e == 3 then p, q = -dy, y1 - c2 else p, q = dy, c4 - y1 end
        if p == 0 then
          if q < 0 then return end
        else
          r = q / p
          if p < 0 then if r > t1 then return end; if r > t0 then t0 = r end
          else if r < t0 then return end; if r < t1 then t1 = r end end
        end
      end
      x1, y1, x2, y2 = x1 + t0 * dx, y1 + t0 * dy, x1 + t1 * dx, y1 + t1 * dy
    end
  end
  lcd.drawLine(floor(x1 + 0.5), floor(y1 + 0.5), floor(x2 + 0.5), floor(y2 + 0.5))
end
local function inClip(x, y) return not CLIP or (x >= CLIP[1] and x <= CLIP[3] and y >= CLIP[2] and y <= CLIP[4]) end
local function rect(x, y, w, h) lcd.drawFilledRectangle(fl(x), fl(y), math.max(fl(w), 1), math.max(fl(h), 1)) end
local function disc(x, y, r) lcd.drawFilledCircle(fl(x), fl(y), fl(r)) end
local function ring(x, y, r) lcd.drawCircle(fl(x), fl(y), fl(r)) end
local function tri(x1, y1, x2, y2, x3, y3) lcd.drawFilledTriangle(fl(x1), fl(y1), fl(x2), fl(y2), fl(x3), fl(y3)) end
-- Weg {x,y}... mit affiner Abbildung X = ox + x*ax + y*ay, Y = oy + x*bx + y*by zeichnen, Punkte naeher als
-- 3 px am letzten gezeichneten uebersprungen (spart Instruktionen, sieht gleich aus)
local function polyAff(pts, ox, ax, ay, oy, bx, by)
  local n = #pts
  if n < 2 then return end
  local p = pts[1]
  local lx, ly = ox + p.x * ax + p.y * ay, oy + p.x * bx + p.y * by
  for k = 2, n do
    p = pts[k]
    local x, y = ox + p.x * ax + p.y * ay, oy + p.x * bx + p.y * by
    local dx, dy = x - lx, y - ly
    if k == n or dx * dx + dy * dy >= 9 then line(lx, ly, x, y); lx, ly = x, y end
  end
end

local function drawT(x, y, boxW, str, font, color, align)
  lcd.font(font)
  lcd.color(color)
  local tw, th = lcd.getTextSize(str)
  local px = x
  if align == "c" then px = x + (boxW - tw) / 2
  elseif align == "r" then px = x + boxW - tw end
  lcd.drawText(fl(px), fl(y), str)
  return th, tw
end

local function textH(font)
  lcd.font(font)
  local _, th = lcd.getTextSize("0")
  return th
end


-- Wert mit Beschriftung in einer Box: label klein oben, Wert gross
local function valBox(x, y, bw, label, value, color, font)
  local h1 = drawT(x, y, bw, label, FONT_XS, C_DIM, "l")
  local h2 = drawT(x, y + h1, bw, value, font or F_L, color or C_WHITE, "l")
  return h1 + h2
end

-- gefuellter Pfeil von (x,y) in Richtung (ux,uy) (Einheitsvektor, Bildschirm), Laenge L, Schaftbreite sw
local function arrow(x, y, ux, uy, L, sw, hw)
  local hl = math.min(L * 0.45, hw * 1.6)
  local bx, by = x + ux * (L - hl), y + uy * (L - hl)
  local nx, ny = -uy, ux
  local h = sw / 2
  tri(x + nx * h, y + ny * h, x - nx * h, y - ny * h, bx + nx * h, by + ny * h)
  tri(x - nx * h, y - ny * h, bx - nx * h, by - ny * h, bx + nx * h, by + ny * h)
  tri(x + ux * L, y + uy * L, bx + nx * hw, by + ny * hw, bx - nx * hw, by - ny * hw)
end

local function planeSymbol(cx, cy, size, color)
  lcd.color(color)
  tri(math.floor(cx), math.floor(cy - size), math.floor(cx - size * 0.7), math.floor(cy + size * 0.7),
    math.floor(cx + size * 0.7), math.floor(cy + size * 0.7))
end

-- Einheitliches Kartenbild auf allen Seiten, das Dreieck dreht nie: wie im Reglement liegt die Hypotenuse
-- waagrecht unten, rechts = Richtung O -> TP1, oben = Richtung O -> TP2. Liefert Bild-Versatz (y nach unten).
local function flatXY(T, x, y) return x * T.ux + y * T.uy, -(x * T.px + y * T.py) end

-- Flugzeug an (px,py), Spitze in Bildrichtung des Kurses crs
local function planeFlat(T, px, py, crs, s, color)
  local hx, hy = flatXY(T, dirVec(crs or T.dir))
  local nx, ny = -hy, hx
  lcd.color(color)
  tri(px + hx * s, py + hy * s, px - hx * s * 0.7 + nx * s * 0.7, py - hy * s * 0.7 + ny * s * 0.7,
    px - hx * s * 0.7 - nx * s * 0.7, py - hy * s * 0.7 - ny * s * 0.7)
end

-- Vektor vor dem Flugzeug: wo es mit Kurs und Tempo ueber Grund in PRED_S s ist (Punkt jede Sekunde)
local function predFlat(P, f, color, w)
  if not (f and f.ve and f.vn) then return end
  -- Weg in PRED_S s mit der jetzigen Querlage (Drehrate konstant): Luftvektor drehen, Wind dazu.
  -- Ohne Querlage geradeaus wie bisher.
  local om = w and BAL.QL.omega(w, f) or 0
  local wx, wy = 0, 0
  if w and w.windOk then wx, wy = w.wx, w.wy end
  local ae, an = f.ve - wx, f.vn - wy
  local x, y = f.x, f.y
  local ax, ay = P(x, y)
  lcd.color(color)
  local N, dt = PRED_S * 2, 0.5
  for k = 1, N do
    local a = om * (k - 0.5) * dt                                   -- Kurs in der Mitte des Schritts
    local c, sn = math.cos(a), math.sin(a)
    x, y = x + ((ae * c + an * sn) + wx) * dt, y + ((-ae * sn + an * c) + wy) * dt
    local bx, by = P(x, y)
    line(ax, ay, bx, by)
    line(ax + 1, ay, bx + 1, by)
    if k % 2 == 0 and k < N then rect(math.floor(bx) - 1, math.floor(by) - 1, 3, 3) end
    ax, ay = bx, by
  end
  disc(ax, ay, 5)
end

-- Windpfeil (wohin er blaest) mit Staerke links daneben, im Kartenbild
local function windFlat(w, T, wx0, wy0, x0)
  if not w.windOk then return end
  local sx, sy = flatXY(T, w.wx, w.wy)
  local n = math.sqrt(sx * sx + sy * sy)
  lcd.color(C_BLUE)
  if n > 0.3 then arrow(wx0 - sx / n * 16, wy0 - sy / n * 16, sx / n, sy / n, 32, 4, 8) end
  drawT(x0, wy0 - math.floor(textH(FONT_S) / 2), wx0 - 22 - x0, string.format("%d km/h", math.floor(n * KMH + 0.5)), FONT_S, C_BLUE, "r")
end

-- Restzeit der Aufgabe (s): vor dem Start die volle Arbeitszeit, danach ab dem Start
local function restTime(w)
  local T = w.task
  if not T then return nil end
  if w.state == S_STOPPED and w.startT and w.endClk then return T.work * 60 - (w.endClk - w.startT), false end
  if w.state == S_STARTED and w.startT and w.cur and w.cur.clk then
    return T.work * 60 - (w.cur.clk - w.startT), true
  end
  return T.work * 60, false
end

-- Text fuer die Kopfzeile (lang, kurz, knapp), Farbe, Hintergrund. Ohne Ballast und ohne Leergewicht: nichts.
function BAL.label(w)
  local m0, kg = w.polarMass, w.balKg or 0
  local mx = (w.balMax or 0) / 10
  if mx <= 0 then
    if not m0 then return nil end
    local g = string.format("%.1f kg", m0)
    return {"GEWICHT " .. g, g}, C_DIM
  end
  local b = string.format("%.1f kg", kg)
  if not m0 then return {"BALLAST " .. b .. "  LEERGEWICHT?", "LEERGEWICHT?", "LEER?"}, C_ORANGE end
  local g = string.format("%.1f kg", m0 + kg)
  if w.balChgT and os.clock() - w.balChgT < 3 then
    local pre = w.balFr and string.format("BALLAST %d %% = ", math.floor(w.balFr * 100 + 0.5)) or "BALLAST "
    return {pre .. b .. "  GEWICHT " .. g, "BALLAST " .. b .. "  " .. g, b .. " " .. g, g}, C_BG, C_ORANGE
  end
  if kg <= 0 then return {"OHNE BALLAST  GEWICHT " .. g, "OHNE BALLAST  " .. g, g}, C_DIM end
  return {(w.balFr and "BALLAST " or "BALLAST FIX ") .. b .. "  GEWICHT " .. g, "BALLAST " .. b .. "  " .. g, b .. " " .. g, g}, C_BLUE
end

local function header(w, W)
  local th = math.max(textH(FONT_S), textH(FONT_STD))
  lcd.color(rgb(25, 25, 25))
  rect(0, 0, W, th + 4)
  -- Status rechts: Zustand, Uhr
  local st, col = "BEREIT", C_DIM
  if not w.task then st, col = "KEINE AUFGABE", C_RED
  elseif w.state == S_ARMED then st, col = ">> SCHARF <<", C_ORANGE
  elseif w.state == S_STARTED then st, col = string.format("RUNDE %d  %s", #w.laps + 1, w.tp == 3 and "ZIEL" or ("TP" .. (w.tp + 1))), C_GREEN
  elseif w.state == S_STOPPED then st, col = "BEENDET", C_BLUE end
  local ls = math.floor(localNow(w)) % 86400
  local clock
  if W < 600 then
    clock = string.format("%02d:%02d", math.floor(ls / 3600), math.floor(ls / 60) % 60)
    if w.state == S_STARTED then st = string.format("R%d %s", #w.laps + 1, w.tp == 3 and "ZIEL" or ("TP" .. (w.tp + 1))) end
    if w.state == S_ARMED then st = "SCHARF" end
    if w.state == S_STOPPED then st = "ENDE" end
  else
    clock = string.format("%02d:%02d:%02d", math.floor(ls / 3600), math.floor(ls / 60) % 60, ls % 60)
  end
  if w.missTP and w.state == S_STARTED then st, col = string.format("TP%d FEHLT", w.missTP), C_RED end
  if not w.gpsOk then st, col = st .. "  GPS?", C_RED end
  -- scharf oder in der Aufgabe, aber die Position aendert sich seit 8 s nicht: Sensor ohne Fix sendet oft
  -- die letzte Position weiter
  if w.gpsOk and (w.state == S_ARMED or w.state == S_STARTED) and w.posMoveT and os.clock() - w.posMoveT > 8 then
    st, col = st .. "  GPS STEHT", C_RED
  end
  local sat = BAL.satText(w)
  if sat then st, col = st .. sat, col == C_RED and C_RED or C_ORANGE end
  clock = clock .. (gpsClock(w) and "" or "*")
  local _, stW = drawT(W * 0.4, 2, W * 0.6 - 6, st .. "  " .. clock, FONT_S, col, "r")
  -- Restzeit prominent links
  local rest, running = restTime(w)
  local restW = 0
  if rest then
    local rc = not running and C_DIM or (rest < 120 and C_RED or (rest < 300 and C_ORANGE or C_WHITE))
    if rest < 0 then rest = 0 end
    _, restW = drawT(6, 1, W * 0.5, "REST " .. fmtMS(rest), FONT_STD, rc, "l")
  end
  -- Ballast und Fluggewicht in der Mitte
  local bt, bc, bg = BAL.label(w)
  if bt then
    local gap = SMALL and 8 or 16
    local x0, x1 = 6 + restW + gap, W - 6 - stW - gap
    lcd.font(FONT_S)
    local barW = fl(th * 1.4)
    local txt
    for _, t in ipairs(bt) do
      local tw = lcd.getTextSize(t)
      if tw + (w.balFr and barW + 6 or 0) <= x1 - x0 - (bg and 12 or 0) then txt = t; break end
    end
    if txt then
      local tw = lcd.getTextSize(txt)
      local bw = tw + (w.balFr and barW + 6 or 0)
      local bx = x0 + (x1 - x0 - bw) / 2
      if bg then lcd.color(bg); rect(bx - 6, 0, bw + 12, th + 4) end
      if w.balFr then
        lcd.color(bc)
        local by, bh = 2 + th * 0.25, th * 0.5
        rect(bx, by, barW, 1); rect(bx, by + bh - 1, barW, 1); rect(bx, by, 1, bh); rect(bx + barW - 1, by, 1, bh)
        if w.balFr > 0 then rect(bx + 2, by + 2, (barW - 4) * w.balFr, bh - 4) end
        bx = bx + barW + 6
      end
      drawT(bx, 2, tw, txt, FONT_S, bc, "l")
    end
  end
  return th + 6
end

-- Seiten-Reiter am unteren Rand (wie im Jeti-Entwurf). Liefert die nutzbare Hoehe darueber.
local function tabBar(w, W, H)
  local tb = textH(FONT_S) + 8
  local y0 = H - tb
  lcd.color(rgb(25, 25, 25))
  rect(0, y0, W, tb)
  local NP = #PAGE_NAMES
  local tabW = math.floor(W / NP)
  for i = 1, NP do
    local on = (i == w.page)
    if on then lcd.color(rgb(0, 90, 160)); rect((i - 1) * tabW, y0, tabW - 2, tb) end
    drawT((i - 1) * tabW, y0 + 4, tabW - 2, (W < 600 and PAGE_SHORT or PAGE_NAMES)[i], FONT_S, on and C_WHITE or C_DIM, "c")
  end
  w.hdrBox = {x = 0, y = y0, w = tabW * NP, h = tb}
  return y0 - 2
end

local function footerMsg(w, W, H, now)
  local msg = w.msg and now - (w.msgT or 0) < 6 and w.msg or nil
  if w.inZoneNow then msg = "SICHERHEITSZONE " .. tostring(w.zoneName or "") end
  if w.overMax then msg = "MAXIMALHÖHE " .. (w.task and w.task.maxAlt or "") .. " m" end
  if w.igc and w.igc.err then msg = w.igc.err end
  if w.missTP and w.state == S_STARTED then msg = string.format("TP%d VERPASST - ZURÜCK IN DEN SEKTOR", w.missTP) end
  if msg then
    local h = textH(FONT_S) + 4
    lcd.color(rgb(120, 0, 0))
    rect(0, H - h, W, h)
    drawT(0, H - h + 2, W, msg, FONT_S, C_WHITE, "c")
  end
end

local fillHP
-- Sektor eines TP fuellen (Kanten +-45 Grad zur Halbierenden), im Kasten x0,y0,bw,bh
local function fillSector(P, c, bd, x0, y0, bw, bh)
  local vx0, vy0 = P(c.x, c.y)
  local HP = {}
  local bxs, bys = P(c.x + c.bx * 100, c.y + c.by * 100)
  for _, off in ipairs({-45, 45}) do
    local ex, ey = dirVec(bd + off)
    local qx, qy = P(c.x + ex * 100, c.y + ey * 100)
    local dx, dy = qx - vx0, qy - vy0
    -- Halbebene a*X + b*Y + k >= 0 auf der Seite der Halbierenden
    local a, b = -dy, dx
    local k = -(a * vx0 + b * vy0)
    if a * bxs + b * bys + k < 0 then a, b, k = -a, -b, -k end
    HP[#HP + 1] = {a, b, k}
  end
  lcd.color(rgb(0, 55, 25))
  fillHP(HP, x0, y0, bw, bh)
end

-- Konvexes Vieleck (Liste {x,y,...} flach) an der Halbebene a*X + b*Y + k >= 0 abschneiden
local function clipHalf(P, a, b, k)
  local out = {}
  local n = #P
  if n < 6 then return out end
  local px, py = P[n - 1], P[n]
  local pv = a * px + b * py + k
  for i = 1, n, 2 do
    local qx, qy = P[i], P[i + 1]
    local qv = a * qx + b * qy + k
    if (pv >= 0) ~= (qv >= 0) then
      local t = pv / (pv - qv)
      out[#out + 1] = px + (qx - px) * t
      out[#out + 1] = py + (qy - py) * t
    end
    if qv >= 0 then out[#out + 1] = qx; out[#out + 1] = qy end
    px, py, pv = qx, qy, qv
  end
  return out
end

-- Konvexes Vieleck im Kasten fuellen (als Dreiecksfaecher, wenige Zeichenbefehle), Farbe vorher setzen
local function fillPoly(P, x0, y0, bw, bh)
  P = clipHalf(P, 1, 0, -x0)
  P = clipHalf(P, -1, 0, x0 + bw - 1)
  P = clipHalf(P, 0, 1, -y0)
  P = clipHalf(P, 0, -1, y0 + bh - 1)
  for i = 3, #P - 3, 2 do tri(P[1], P[2], P[i], P[i + 1], P[i + 2], P[i + 3]) end
end

-- Schnittmenge von Halbebenen a*X + b*Y + k >= 0 (Bildschirm) im Kasten fuellen, Farbe vorher setzen
function fillHP(HP, x0, y0, bw, bh)
  local P = {x0, y0, x0 + bw - 1, y0, x0 + bw - 1, y0 + bh - 1, x0, y0 + bh - 1}
  for _, h in ipairs(HP) do P = clipHalf(P, h[1], h[2], h[3]) end
  for i = 3, #P - 3, 2 do tri(P[1], P[2], P[i], P[i + 1], P[i + 2], P[i + 3]) end
end

-- Sicherheitszonen: Rahmen, gefuellt wenn das Modell mit dem jetzigen Steigen/Sinken beim Erreichen der Zone
-- (gerade, mit Grundgeschwindigkeit) in ihrem Hoehenband ist. P = Kartenprojektion, Kasten fuer die Fuellung.
local function drawZones(w, T, P, x0, y0, bw, bh)
  if not T or #T.zones == 0 then return end
  local f = w.cur
  for _, z in ipairs(T.zones) do
    local hot = false
    if f then
      local d
      if z.t == "C" then d = math.max(math.sqrt((f.x - z.x) ^ 2 + (f.y - z.y) ^ 2) - z.r, 0)
      else
        local ux, uy = dirVec(z.dir)
        local dx, dy = f.x - z.x, f.y - z.y
        local a = math.max(math.abs(dx * ux + dy * uy) - z.len / 2, 0)
        local b = math.max(math.abs(dx * uy - dy * ux) - z.wid / 2, 0)
        d = math.sqrt(a * a + b * b)
      end
      local t = math.min(d / math.max(f.gs or 10, 5), 120)
      local h = f.agl + (f.vz or 0) * t
      hot = h >= z.lo and h <= z.hi
    end
    if z.t == "C" then
      local zx, zy = P(z.x, z.y)
      local ex, ey = P(z.x + z.r, z.y)
      local r = math.sqrt((ex - zx) ^ 2 + (ey - zy) ^ 2)
      local n = 20
      local poly = {}
      for k = 0, n - 1 do
        local a = 2 * math.pi * k / n
        poly[#poly + 1] = zx + r * math.cos(a)
        poly[#poly + 1] = zy + r * math.sin(a)
      end
      if hot then lcd.color(rgb(120, 20, 20)); fillPoly(poly, x0, y0, bw, bh) end
      lcd.color(rgb(230, 60, 60))
      for k = 1, n do
        local j = k % n + 1
        line(poly[2 * k - 1], poly[2 * k], poly[2 * j - 1], poly[2 * j])
      end
    else
      local ux, uy = dirVec(z.dir)
      local vx, vy = uy, -ux
      local L, Wd = z.len / 2, z.wid / 2
      local c = {}
      for _, sgn in ipairs({{1, 1}, {1, -1}, {-1, -1}, {-1, 1}}) do
        local qx, qy = P(z.x + ux * L * sgn[1] + vx * Wd * sgn[2], z.y + uy * L * sgn[1] + vy * Wd * sgn[2])
        c[#c + 1] = {qx, qy}
      end
      if hot then
        local mxs, mys = 0, 0
        for _, q in ipairs(c) do mxs, mys = mxs + q[1] / 4, mys + q[2] / 4 end
        local HP = {}
        for k = 1, 4 do
          local a1, a2 = c[k], c[k % 4 + 1]
          local a, b = -(a2[2] - a1[2]), a2[1] - a1[1]
          local kk = -(a * a1[1] + b * a1[2])
          if a * mxs + b * mys + kk < 0 then a, b, kk = -a, -b, -kk end
          HP[#HP + 1] = {a, b, kk}
        end
        lcd.color(rgb(120, 20, 20))
        fillHP(HP, x0, y0, bw, bh)
      end
      lcd.color(rgb(230, 60, 60))
      for k = 1, 4 do local a1, a2 = c[k], c[k % 4 + 1]; line(a1[1], a1[2], a2[1], a2[2]) end
    end
  end
end

-- Seite UEBERSICHT: ganzes Dreieck mit Flugweg, flach wie im Reglement (Hypotenuse unten, TP3 links, TP1 rechts,
-- TP2 oben, Linie senkrecht durch den Ursprung). Vor dem Start ist die
-- TP1-Seite der Linie rot, danach ist der Sektor des naechsten TP gefuellt wie in der Eckansicht.
-- Optimale Runde vor dem Start (Ursprung -> TP1 -> TP2 -> TP3 -> Ursprung), alle 2 s neu gerechnet
-- Ideale Runde mit Wind, unabhaengig von der Position des Modells: geplant ab dem Ursprung ueber
-- TP1, TP2, TP3 und nochmals TP1; gezeichnet ab dem Kurvenbeginn an TP1, also ohne den Anflug vom Ursprung.
-- Das Stueck von TP3 zu TP1 zeigt, wo man ideal durch die Startlinie fliegt.
function lapRoute(w, T)
  -- nur neu rechnen, wenn sich Wind (1 m/s), Fahrt (2 m/s), Radien oder Aufgabe merklich aendern
  local wx, wy = w.windOk and w.wx or 0, w.windOk and w.wy or 0
  local V = w.tasF or 20
  local key = string.format("%d %d %d %.0f %.0f %.0f %s %d", floor(wx + 0.5), floor(wy + 0.5), floor(V / 2 + 0.5),
    radius(w, 1), radius(w, 2), radius(w, 3), tostring(T), w.margin)
  if not w.ovPlan or w.ovPlanKey ~= key then
    local pr = planPath(0, 0, floor(wx + 0.5), floor(wy + 0.5), floor(V / 2 + 0.5) * 2, radius(w, 1), cornersFrom(T, 1, 4, radiusFn(w), w.margin))
    local pts = {}
    for k = 2, #(pr.pts or {}) do pts[#pts + 1] = pr.pts[k] end
    w.ovPlan, w.ovPlanKey = pts, key
  end
  return w.ovPlan
end

local function paintOver(w, W, H, top)
  local T, f = w.task, w.cur
  local m = 6
  local y = top
  local mw = SMALL and (W - 2 * m) or math.floor(W * 0.64)
  local mh = H - y - m
  local mx0 = m
  lcd.color(rgb(15, 15, 15))
  rect(mx0, y, mw, mh)
  if not T then
    drawT(mx0, y + mh / 2 - 10, mw, "Keine Aufgabe", FONT_L, C_RED, "c")
    return
  end
  -- Bild: rechts = Richtung O -> TP1, oben = Richtung O -> TP2
  local function R(x, yv) return flatXY(T, x, yv) end
  -- Ausschnitt: nur das Dreieck (mit Sektorkanten), damit es bei jeder Schenkellaenge gleich gross ist.
  -- Ein Modell ausserhalb steht am Rand, mit seiner Distanz zum naechsten Ziel.
  local pts = {{0, 0}, {T.tp[1].x, T.tp[1].y}, {T.tp[2].x, T.tp[2].y}, {T.tp[3].x, T.tp[3].y}}
  for i = 1, 3 do local c = T.tp[i]; pts[#pts + 1] = {c.x + c.bx * T.dist * 0.25, c.y + c.by * T.dist * 0.25} end
  local x1, y1, x2, y2 = math.huge, math.huge, -math.huge, -math.huge
  for _, q in ipairs(pts) do
    local rx, ry = R(q[1], q[2])
    x1, y1, x2, y2 = math.min(x1, rx), math.min(y1, ry), math.max(x2, rx), math.max(y2, ry)
  end
  local pad = 18
  local sc = math.min((mw - 2 * pad) / math.max(x2 - x1, 0.1), (mh - 2 * pad) / math.max(y2 - y1, 0.1))
  local ccx, ccy = (x1 + x2) / 2, (y1 + y2) / 2
  local cx, cy = mx0 + mw / 2, y + mh / 2
  local function P(x, yv)
    local rx, ry = R(x, yv)
    return cx + (rx - ccx) * sc, cy + (ry - ccy) * sc
  end
  CLIP = {mx0, y, mx0 + mw - 1, y + mh - 1}
  local before = w.state ~= S_STARTED and w.state ~= S_STOPPED
  local ni = w.tp + 1
  -- Vor dem Start: TP1-Seite der Linie rot (dort darf man nicht herkommen)
  if before then
    local ax, ay = P(T.px * 100, T.py * 100)
    local bx, by = P(-T.px * 100, -T.py * 100)
    local tx, ty = P(T.ux * 100, T.uy * 100)
    local a, b = -(by - ay), bx - ax
    local k = -(a * ax + b * ay)
    if a * tx + b * ty + k < 0 then a, b, k = -a, -b, -k end
    lcd.color(rgb(60, 14, 14))
    fillHP({{a, b, k}}, mx0, y, mw, mh)
  elseif w.state == S_STARTED and ni <= 3 then
    fillSector(P, T.tp[ni], bearingOf(T.tp[ni].bx, T.tp[ni].by), mx0, y, mw, mh)
  end
  drawZones(w, T, P, mx0, y, mw, mh)
  -- Sektorkanten aller TP, Schenkel
  for i = 1, 3 do
    local c = T.tp[i]
    local bd = bearingOf(c.bx, c.by)
    lcd.color((w.state == S_STARTED and i == ni) and rgb(0, 140, 60) or rgb(0, 70, 35))
    for _, off in ipairs({-45, 45}) do
      local ex, ey = dirVec(bd + off)
      local ax, ay = P(c.x, c.y)
      local bx, by = P(c.x + ex * T.dist * 0.4, c.y + ey * T.dist * 0.4)
      line(ax, ay, bx, by)
    end
  end
  lcd.color(rgb(90, 90, 90))
  for i = 1, 3 do
    local a, b = T.tp[i], T.tp[i % 3 + 1]
    local ax, ay = P(a.x, a.y)
    local bx, by = P(b.x, b.y)
    line(ax, ay, bx, by)
  end
  -- Start-/Ziellinie; ist sie das naechste Ziel, doppelt und hell
  local lineNext = before or (w.state == S_STARTED and ni > 3)
  local l1x, l1y = P(T.px * LINE_HALF, T.py * LINE_HALF)
  local l2x, l2y = P(-T.px * LINE_HALF, -T.py * LINE_HALF)
  lcd.color(lineNext and C_YELLOW or rgb(140, 125, 0))
  line(l1x, l1y, l2x, l2y)
  if lineNext then line(l1x, l1y + 1, l2x, l2y + 1) end
  -- Optimale Route (blau) wie in der Eckansicht: in der Aufgabe vom Modell ueber die naechsten Ecken,
  -- vorher eine ganze Runde ab dem Ursprung (alle 2 s neu gerechnet)
  local route
  if w.state == S_STARTED and w.gc and w.gc.plan then route = w.gc.plan.pts
  elseif before then
    route = lapRoute(w, T)
  end
  if route then
    lcd.color(C_BLUE)
    polyAff(route, cx - ccx * sc, T.ux * sc, T.uy * sc, cy - ccy * sc, -T.px * sc, -T.py * sc)
  end
  -- Flugweg: vor dem Start grau, in der Aufgabe weiss. Die Bildpunkte werden zwischengespeichert (der Massstab
  -- haengt nur am Dreieck), neue Punkte ab 8 px Abstand, nur die letzten trackMin Minuten (Konfiguration).
  local tr = w.track or {}
  local lim = (w.trackMin or 3) * 60
  local key = string.format("%.4f %.1f %.1f %.1f %.1f %s", sc, ccx, ccy, cx, cy, tostring(T))
  local oc = w.ovTrack
  if not oc or oc.key ~= key or oc.n > #tr then oc = {key = key, n = 0, xs = {}, ys = {}, on = {}, ts = {}, gap = 64}; w.ovTrack = oc end
  local xs, ys, on, ts = oc.xs, oc.ys, oc.on, oc.ts
  for k = oc.n + 1, #tr do
    local p = tr[k]
    local ax, ay = P(p.x, p.y)
    local m0 = #xs
    if m0 == 0 or (ax - xs[m0]) ^ 2 + (ay - ys[m0]) ^ 2 >= oc.gap or p.on ~= on[m0] then
      xs[m0 + 1], ys[m0 + 1], on[m0 + 1], ts[m0 + 1] = ax, ay, p.on, p.t or 0
    end
  end
  oc.n = #tr
  -- vorne alles aelter als die eingestellte Spurlaenge abschneiden
  local nowT, n = os.clock(), #xs
  local i0 = 1
  while i0 <= n and nowT - ts[i0] > lim do i0 = i0 + 1 end
  if i0 > 1 then
    for _, a in ipairs({xs, ys, on, ts}) do
      table.move(a, i0, n, 1)
      for k = n - i0 + 2, n do a[k] = nil end
    end
    n = #xs
  end
  -- zu viele Stuecke (lange Spur, schneller Flug): jeden zweiten Punkt weglassen und den Abstand verdoppeln,
  -- so bleibt die ganze Zeitspanne sichtbar und das Zeichnen unter der Instruktionsgrenze
  if n > 160 then
    local j = 0
    for k = 1, n do
      if k % 2 == 1 or k == n or on[k] ~= on[k - 1] then
        j = j + 1
        xs[j], ys[j], on[j], ts[j] = xs[k], ys[k], on[k], ts[k]
      end
    end
    for k = j + 1, n do xs[k], ys[k], on[k], ts[k] = nil, nil, nil, nil end
    oc.gap, n = oc.gap * 4, j
  end
  if lim > 0 then
    local lastOn
    for k = 2, n do
      if on[k] ~= lastOn then lastOn = on[k]; lcd.color(on[k] and C_WHITE or rgb(100, 100, 100)) end
      line(xs[k - 1], ys[k - 1], xs[k], ys[k])
    end
    -- das juengste Stueck bis zur aktuellen Position
    if #tr > 0 and n > 0 then
      local ax, ay = P(tr[#tr].x, tr[#tr].y)
      line(xs[n], ys[n], ax, ay)
    end
  end
  -- Marken: Start gelb, Wendepunkt gruen, Runde fertig blau
  for _, mk in ipairs(w.marks or {}) do
    local ax, ay = P(mk.x, mk.y)
    if inClip(ax, ay) then
      lcd.color(mk.k == "STA" and C_YELLOW or (mk.k == "TPC" and C_GREEN or C_BLUE))
      disc(ax, ay, 3)
    end
  end
  -- Wendepunkte mit Nummer: naechster gruen, in dieser Runde erledigt dunkelgruen
  for i = 1, 3 do
    local tx, ty = P(T.tp[i].x, T.tp[i].y)
    if inClip(tx, ty) then
      local nextOne = w.state == S_STARTED and i == ni
      local done = w.state == S_STARTED and i < ni
      local col = nextOne and C_GREEN or (done and rgb(0, 120, 50) or C_DIM)
      lcd.color(col)
      disc(tx, ty, nextOne and 5 or 3)
      drawT(tx + 7, ty - 7, 60, "TP" .. i, FONT_XS, col, "l")
    end
  end
  -- Modell in Flugrichtung
  if f then
    local px, py = P(f.x, f.y)
    px, py = clamp(px, mx0 + 6, mx0 + mw - 6), clamp(py, y + 6, y + mh - 6)
    local fx, fy = P(f.x, f.y)
    predFlat(function(x, yv) local qx, qy = P(x, yv); return qx + px - fx, qy + py - fy end, f, C_ORANGE, w)
    planeFlat(T, px, py, f.crs, 8, C_WHITE)
    if px ~= fx or py ~= fy then
      local gx, gy = 0, 0
      if w.state == S_STARTED and ni <= 3 then gx, gy = T.tp[ni].x, T.tp[ni].y end
      local d = math.floor(math.sqrt((f.x - gx) ^ 2 + (f.y - gy) ^ 2) + 0.5)
      local lx = clamp(px - 60, mx0 + 2, mx0 + mw - 122)
      local ly = py < y + mh / 2 and py + 12 or py - 12 - textH(FONT_S)
      drawT(lx, ly, 120, string.format("%s %d m", (w.state == S_STARTED and ni <= 3) and ("TP" .. ni) or "Linie", d), FONT_S, C_WHITE, "c")
      -- weit weg vom Dreieck: meist ist eine andere Aufgabe aktiv als die, an deren Ort man steht
      local dO = math.sqrt(f.x * f.x + f.y * f.y)
      if dO > math.max(200, 3 * T.dist) then
        local txt = dO >= 1000 and string.format("%.1f km", dO / 1000) or string.format("%d m", math.floor(dO + 0.5))
        drawT(mx0, y + mh / 2 - textH(FONT_STD), mw, "Modell " .. txt .. " vom Dreieck", FONT_STD, C_ORANGE, "c")
        drawT(mx0, y + mh / 2 + 2, mw, "Aufgabe: " .. T.name, FONT_S, C_ORANGE, "c")
      end
    end
  end
  CLIP = nil
  -- Wind (wohin er blaest), im Bild der Übersicht
  windFlat(w, T, mx0 + mw - 26, y + 26, mx0)
  -- Rechts: was als naechstes kommt und wie der Start war
  local nxtTxt, nxtCol
  if before then nxtTxt, nxtCol = (w.state == S_ARMED and "Start über Linie" or "Scharf stellen"), C_ORANGE
  elseif w.state == S_STOPPED then nxtTxt, nxtCol = "Beendet", C_BLUE
  elseif ni > 3 then nxtTxt, nxtCol = "Ziellinie", C_GREEN
  else
    local c = T.tp[ni]
    nxtTxt, nxtCol = string.format("TP%d  %d m", ni, f and math.floor(math.sqrt((c.x - f.x) ^ 2 + (c.y - f.y) ^ 2) + 0.5) or 0), C_GREEN
  end
  if SMALL then
    drawT(mx0 + 4, y + mh - textH(FONT_S) - 2, mw - 8, nxtTxt .. string.format("   Runden %d", #w.laps), FONT_S, nxtCol, "l")
    return
  end
  local x0 = mx0 + mw + m
  local cw = W - x0 - m
  local yy = y
  yy = yy + valBox(x0, yy, cw, "NÄCHSTES ZIEL", nxtTxt, nxtCol, F_L) + 4
  local stTxt, stCol = "noch nicht", C_DIM
  if w.startT then
    local su = w.startUtc and (math.floor(w.startUtc + (w.utcOffset or 0) * 60) % 86400) or nil
    stTxt = su and string.format("%02d:%02d:%02d", math.floor(su / 3600), math.floor(su / 60) % 60, su % 60) or "ok"
    stCol = C_GREEN
  elseif w.badStartT then stTxt, stCol = "ungültig", C_RED end
  yy = yy + valBox(x0, yy, cw, "START", stTxt, stCol, FONT_STD) + 4
  if w.entry then
    yy = yy + valBox(x0, yy, cw, "EINFLUG", string.format("%d m  %d km/h", math.floor(w.entry.agl + 0.5), math.floor(w.entry.v3d + 0.5)), (w.penalty or 0) > 0 and C_RED or C_WHITE, FONT_STD) + 4
  end
  if w.state == S_STARTED or w.state == S_STOPPED then
    yy = yy + valBox(x0, yy, cw, "STRAFE", tostring(w.penalty or 0), (w.penalty or 0) > 0 and C_RED or C_GREEN, FONT_STD) + 4
  end
  local last = w.laps[#w.laps]
  yy = yy + valBox(x0, yy, cw, "RUNDEN", tostring(#w.laps) .. (last and last.idx and string.format("   Index %.1f %%", last.idx) or ""), C_WHITE, FONT_STD) + 4
  if w.state == S_STARTED then
    local tps = {}
    for i = 1, 3 do tps[#tps + 1] = (i < ni) and ("TP" .. i .. " ok") or ("TP" .. i) end
    valBox(x0, yy, cw, "DIESE RUNDE", table.concat(tps, "  "), C_WHITE, FONT_S)
  end
end

-- Seite START
local function paintStart(w, W, H, top)
  local T = w.task
  local c = w.sc
  local f = w.cur
  local m = 6
  -- Zeile: Startfenster und Arbeitszeit
  local a, b = slotWindow(w)
  local nowClock = utcNow(w)
  local slotTxt, slotCol = "kein Startfenster", C_DIM
  if a then
    if nowClock < a then slotTxt, slotCol = "Fenster öffnet in " .. fmtMS(a - nowClock), C_ORANGE
    elseif nowClock <= b then slotTxt, slotCol = "FENSTER OFFEN noch " .. fmtMS(b - nowClock), C_GREEN
    else slotTxt, slotCol = "Fenster geschlossen", C_RED end
  end
  local y = top
  local th = drawT(m, y, W - 2 * m, slotTxt, FONT_STD, slotCol, "l")
  drawT(m, y, W - 2 * m, string.format("Arbeitszeit %d min", math.floor(T and T.work or 0)), FONT_S, C_DIM, "r")
  y = y + th + 2
  -- Links: Draufsicht auf Linie und Dreieck
  local mapW = math.floor(W * 0.42)
  local mapH = H - y - m
  -- Draufsicht im festen Kartenbild (Hypotenuse waagrecht): Linie senkrecht, Startsektor (TP3-Seite) links,
  -- TP1-Seite rechts, TP2 oben
  local cx, cy = m + mapW * 0.6, y + mapH * 0.7
  lcd.color(rgb(20, 20, 20))
  rect(m, y, mapW, mapH)
  -- Vor dem Start: TP1-Seite der Linie ist die falsche Seite (rot)
  local before = w.state ~= S_STARTED and w.state ~= S_STOPPED
  if before then
    lcd.color(rgb(60, 14, 14))
    rect(math.floor(cx), y, math.max(1, math.floor(m + mapW - cx)), mapH)
  end
  -- fester Massstab aus der App: startSpan Meter von der Linie bis zum linken Rand (Modell weiter weg bleibt am Rand)
  local scale = (cx - m - 4) / w.startSpan
  CLIP = {m, y, m + mapW - 1, y + mapH - 1}
  if T then
    drawZones(w, T, function(x, yv) local a, b = flatXY(T, x, yv); return cx + a * scale, cy + b * scale end, m, y, mapW, mapH)
    local d = T.dist * scale
    lcd.color(C_GRID)
    line(cx - d, cy, cx + d, cy)
    line(cx - d, cy, cx, cy - d)
    line(cx, cy - d, cx + d, cy)
    -- optimale Runde (blau): zeigt vor dem Start, wo man von TP3 her durch die Linie zu TP1 fliegt
    local route = before and lapRoute(w, T) or (w.gc and w.gc.plan and w.gc.plan.pts)
    if route then
      lcd.color(C_BLUE)
      polyAff(route, cx, T.ux * scale, T.uy * scale, cy, -T.px * scale, -T.py * scale)
    end
  end
  -- Linie +-2 km
  lcd.color(C_YELLOW)
  line(cx, cy - LINE_HALF * scale, cx, cy + LINE_HALF * scale)
  line(cx + 1, cy - LINE_HALF * scale, cx + 1, cy + LINE_HALF * scale)
  CLIP = nil
  lcd.color(C_WHITE)
  disc(math.floor(cx), math.floor(cy), 3)
  drawT(math.floor(cx) + 4, y + 2, mapW, "Linie", FONT_XS, C_YELLOW, "l")
  drawT(m + 2, y + 2, mapW, "Start >", FONT_XS, C_DIM, "l")
  if before then drawT(math.floor(cx) + 4, y + mapH - textH(FONT_XS) * 2 - 6, m + mapW - cx - 6, "TP1-Seite", FONT_XS, rgb(200, 80, 80), "l") end
  -- Wind: Pfeil in Windrichtung (wohin er blaest)
  if T then
    if w.windOk then windFlat(w, T, m + mapW - 26, y + 26, m)
    else drawT(m, y + 20, mapW - 4, "Wind ?", FONT_S, C_DIM, "r") end
  end
  -- Flugweg der letzten 90 s (zeigt den Anflug, auch nach einem misslungenen Einflug)
  if T and w.track then
    CLIP = {m, y, m + mapW - 1, y + mapH - 1}
    local tNow, tr = os.clock(), w.track
    local ux, uy, qx, qy = T.ux * scale, T.uy * scale, T.px * scale, T.py * scale
    local x0, y0, x1, y1 = m, y, m + mapW - 1, y + mapH - 1
    -- von hinten, bis die Punkte aelter als 90 s sind (der ganze Flugweg waere zu teuer)
    for k = #tr, 1, -1 do
      local p = tr[k]
      if tNow - (p.t or 0) >= 90 then break end
      local ax, ay = cx + p.x * ux + p.y * uy, cy - p.x * qx - p.y * qy
      if ax >= x0 and ax <= x1 and ay >= y0 and ay <= y1 then
        lcd.color(p.on and C_WHITE or C_DIM); rect(math.floor(ax) - 1, math.floor(ay) - 1, 2, 2)
      end
    end
    CLIP = nil
  end
  if c and f and c.lat then
    local px, py = cx - c.dist * scale, cy - c.lat * scale
    px, py = clamp(px, m + 4, m + mapW - 4), clamp(py, y + 4, y + mapH - 4)
    -- Projektion zur Linie
    if c.predLat then
      local qy = clamp(cy - c.predLat * scale, y + 2, y + mapH - 2)
      lcd.color(math.abs(c.predLat) <= LINE_HALF and C_GREEN or C_RED)
      line(px, py, cx, qy)
      disc(math.floor(cx), math.floor(qy), 4)
    end
    planeFlat(T, px, py, f.crs, 8, C_WHITE)
    -- Zeit bis zur Linie mit der aktuellen Geschwindigkeit (beim Anflug die Annaeherung an die Linie)
    local secs = (c.dist >= 0 and c.tLine) or math.abs(c.dist) / math.max(f.gs or 0, 1)
    local dTxt = c.dist >= 0 and (c.tLine and string.format("%d s bis Linie", math.floor(secs + 0.5)) or "fliegt weg")
      or string.format("%d s hinter Linie", math.floor(secs + 0.5))
    local dF = SMALL and FONT_S or FONT_L
    drawT(m + 2, y + mapH - textH(dF) - 2, mapW - 4, dTxt, dF, C_WHITE, "l")
    if before and c.dist < 0 then
      drawT(m, y + textH(FONT_XS) + 6, mapW, "FALSCHE SEITE", SMALL and FONT_STD or FONT_L, C_RED, "c")
    end
  end
  -- Mitte: Gleitpfad wie beim ILS. Mitte = Fenstermitte an der Linie, Skala +-50 m.
  -- Zu tief: Pfeil von unten nach oben (steigen/weniger sinken), zu hoch: Pfeil von oben nach unten.
  local px0 = m * 2 + mapW
  local profW = math.floor(W * 0.24)
  lcd.color(rgb(20, 20, 20))
  rect(px0, y, profW, mapH)
  if T then
    local FS = 50
    local hdrH = textH(FONT_XS) + 2
    drawT(px0, y + 2, profW, "GLEITPFAD", FONT_XS, C_DIM, "c")
    local gy0, gh = y + hdrH + 2, mapH - hdrH - 4
    local gcx, gcy = math.floor(px0 + profW / 2), math.floor(gy0 + gh / 2)
    local k = (gh / 2 - 2) / FS
    local half = w.winLo / 2
    -- Fensterband und Mittellinie
    lcd.color(rgb(0, 70, 30))
    local bh = math.max(2, math.floor(half * k))
    rect(px0 + 4, gcy - bh, profW - 8, 2 * bh)
    lcd.color(C_GREEN)
    line(px0 + 4, gcy, px0 + profW - 4, gcy)
    -- Teilstriche alle 10 m
    lcd.color(C_GRID)
    for d = -FS, FS, 10 do
      if d ~= 0 then
        local ty = math.floor(gcy - d * k)
        local tl = (d % 50 == 0) and 10 or 5
        line(px0 + 4, ty, px0 + 4 + tl, ty)
        line(px0 + profW - 4 - tl, ty, px0 + profW - 4, ty)
      end
    end
    drawT(px0 + 3, math.floor(gcy - FS * k), profW, "+" .. FS, FONT_XS, C_DIM, "l")
    drawT(px0 + 3, math.floor(gcy + FS * k) - textH(FONT_XS), profW, "-" .. FS, FONT_XS, C_DIM, "l")
    if c and c.err then
      local e = c.err
      local aw = math.floor(profW * 0.28)
      local fV = SMALL and FONT_STD or FONT_L
      if math.abs(e) <= half then
        lcd.color(C_GREEN)
        disc(gcx, math.floor(gcy - clamp(e, -FS, FS) * k), math.max(5, math.floor(profW / 10)))
        drawT(px0, gy0 + gh - textH(FONT_S) - 2, profW, "IM FENSTER", FONT_S, C_GREEN, "c")
      else
        -- Pfeilspitze zeigt auf die Mitte, Laenge = Abweichung (begrenzt auf die Skala)
        local over = math.abs(e) - half
        local dir = e < 0 and 1 or -1                -- Bildschirm: zu tief Pfeil unten (zeigt hoch), zu hoch oben
        local col = e < 0 and C_BLUE or C_ORANGE
        local tipY = math.floor(gcy + dir * bh)
        local tailY = math.floor(gcy + dir * (bh + clamp(over, 5, FS) * k))
        local sw = math.max(2, math.floor(aw / 4))
        local hh = math.max(6, math.floor(aw * 0.7))
        lcd.color(col)
        if math.abs(tailY - tipY) > hh then
          local sy = tipY + dir * hh
          local ty0, ty1 = math.min(sy, tailY), math.max(sy, tailY)
          rect(gcx - sw, ty0, 2 * sw, math.max(1, ty1 - ty0))
        end
        tri(gcx, tipY, gcx - aw, tipY + dir * hh, gcx + aw, tipY + dir * hh)
        -- Text auf der freien Seite, zwischen Band und Skalenende
        local tH = textH(fV) + textH(FONT_XS)
        local ty = e < 0 and (gcy - bh - tH - 4) or (gcy + bh + 4)
        drawT(px0, ty, profW, string.format("%d m", math.floor(over + 0.5)), fV, col, "c")
        drawT(px0, ty + textH(fV), profW, e < 0 and "ZU TIEF" or "ZU HOCH", FONT_XS, col, "c")
      end
    else
      drawT(px0, gcy - textH(FONT_S) - 4, profW, "kein Anflug", FONT_S, C_DIM, "c")
    end
  end
  -- Rechts: Zahlen
  local rx = px0 + profW + m
  local rw = W - rx - m
  local yy = y
  local function row(label, val, col)
    yy = yy + valBox(rx, yy, rw, label, val, col, F_L) + 2
  end
  if not T then
    drawT(rx, yy, rw, "Aufgabe im MM Flight", FONT_S, C_DIM, "l")
    drawT(rx, yy + textH(FONT_S), rw, "Analyzer anlegen", FONT_S, C_DIM, "l")
    return
  end
  local hNow = f and string.format("%d m", math.floor(f.agl + 0.5)) or "--"
  local vNow = f and string.format("%d km/h", math.floor(f.v3d + 0.5)) or "--"
  if c and c.predAgl then
    local e = c.err
    local col = math.abs(e) <= w.winLo / 2 and C_GREEN or (c.predAgl > T.maxH and C_RED or C_ORANGE)
    row(SMALL and "HÖHE AN LINIE" or ("HÖHE AN LINIE (" .. (T.maxH - w.winLo) .. "-" .. T.maxH .. ")"), string.format("%d m  %+d", math.floor(c.predAgl + 0.5), math.floor(c.predAgl - T.maxH + 0.5)), col)
    local vc = (c.predV or 0) > T.maxV and C_RED or ((c.predV or 0) > T.maxV - w.speedMargin and C_ORANGE or C_GREEN)
    row(SMALL and string.format("SPEED (MAX %d)", math.floor(T.maxV)) or string.format("SPEED AN LINIE (max %d, jetzt %d)", math.floor(T.maxV), math.floor(f.v3d + 0.5)), string.format("%d km/h", math.floor((c.predV or 0) + 0.5)), vc)
    row("ZEIT BIS LINIE", fmtMS(c.tLine), C_WHITE)
  else
    row("HÖHE", hNow, C_WHITE)
    row("SPEED 3D", vNow, C_WHITE)
  end
  -- Strafe nur nach dem Ueberqueren der Linie, und nur wenn es eine gibt
  local started = w.state == S_STARTED and w.entry
  if started and (w.penalty or 0) > 0 then row("STRAFE", tostring(w.penalty), C_RED) end
  if started and not w.badEntry then
    local sTxt = string.format("Start: %d m %d km/h", math.floor(w.entry.agl + 0.5), math.floor(w.entry.v3d + 0.5))
    if (w.penalty or 0) > 0 then sTxt = sTxt .. string.format("  Strafe %d", w.penalty) end
    drawT(rx, yy, rw, sTxt, FONT_XS, (w.penalty or 0) > 0 and C_RED or C_GREEN, "l")
    yy = yy + textH(FONT_XS) + 2
  end
  if started and w.badEntry then
    local over = w.entry.agl - T.maxH
    local vOver = w.entry.v3d - T.maxV
    local why = over > 0 and string.format("%d m, ZU HOCH +%d", math.floor(w.entry.agl + 0.5), math.floor(over + 0.5)) or ""
    if vOver > 0 then why = why .. (why ~= "" and "  " or "") .. string.format("ZU SCHNELL +%d", math.floor(vOver + 0.5)) end
    drawT(rx, yy, rw, why, FONT_STD, C_RED, "l")
    yy = yy + textH(FONT_STD) + 4
  end
  local canRestart = w.state == S_STARTED
  if w.state == S_IDLE or w.state == S_STOPPED or canRestart then
    local bf = (canRestart and w.badEntry) and F_L or FONT_STD
    local txt = canRestart and "NEUSTART (tippen)" or "SCHARF (tippen)"
    if canRestart and #w.laps > 0 then
      txt = (w.rsArm and os.clock() - w.rsArm < (w.rsWin or 4)) and "NOCHMAL = NEUER FLUG" or "NEUSTART (2x tippen/schalten)"
    end
    w.armBox = {x = rx, y = yy, w = rw, h = textH(bf) + 8}
    lcd.color((canRestart and w.badEntry) and rgb(110, 70, 0) or rgb(70, 50, 0))
    rect(rx, yy, rw, w.armBox.h)
    drawT(rx, yy + 4, rw, txt, bf, C_ORANGE, "c")
    yy = yy + w.armBox.h + 6
  end
  -- Querlage nullen: nur am Boden (steht), Fluegel waagrecht halten, 2 s auf das Feld druecken
  if w.qlRaw and not w.flying and (not f or (f.gs or 0) < 2) and w.state ~= S_STARTED then
    local h = textH(FONT_S) + 8
    w.qlBox = {x = rx, y = yy, w = rw, h = h}
    lcd.color(w.qlPress and rgb(0, 70, 30) or rgb(40, 40, 40))
    rect(rx, yy, rw, h)
    drawT(rx, yy + 4, rw, string.format("QL %+.0f°  (2 s drücken = nullen)", w.qlPhi or 0), FONT_S, C_WHITE, "c")
  end
end

-- Eckansicht (Kurs oben), zentriert zwischen Ecke und Flugzeug
local function paintCorner(w, g, x0, y0, bw, bh)
  local T, f = w.task, w.cur
  local i = g.corner
  local c = T.tp[i]
  lcd.color(rgb(15, 15, 15))
  rect(x0, y0, bw, bh)
  local d = math.sqrt((f.x - c.x) ^ 2 + (f.y - c.y) ^ 2)
  local span = clamp(d * 1.3, 80, 400)
  local sc = math.min(bw, bh) / 2 / span
  local mx, my = (f.x + c.x) / 2, (f.y + c.y) / 2
  local cx, cy = x0 + bw / 2, y0 + bh / 2
  CLIP = {x0, y0, x0 + bw - 1, y0 + bh - 1}
  local function P(x, y)
    local rx, ry = flatXY(T, x - mx, y - my)
    return cx + rx * sc, cy + ry * sc
  end
  -- Sektor (Kanten +-45 Grad zur Halbierenden); noch nicht erreicht = gefuellt
  local bd = bearingOf(c.bx, c.by)
  local done = w.tp > 0 and i <= w.tp
  if not done then fillSector(P, c, bd, x0, y0, bw, bh) end
  drawZones(w, T, P, x0, y0, bw, bh)
  lcd.color(rgb(0, 110, 50))
  for _, off in ipairs({-45, 45}) do
    local ex, ey = dirVec(bd + off)
    local ax, ay = P(c.x, c.y)
    local bx, by = P(c.x + ex * span * 3, c.y + ey * span * 3)
    line(ax, ay, bx, by)
  end
  -- Schenkel des Dreiecks
  lcd.color(C_DIM)
  for _, j in ipairs({(i + 1) % 3 + 1, i % 3 + 1}) do
    local o = T.tp[j]
    local ax, ay = P(c.x, c.y)
    local bx, by = P(o.x, o.y)
    line(ax, ay, bx, by)
  end
  -- geplanter Weg
  if g.plan and i == g.next then
    lcd.color(C_BLUE)
    polyAff(g.plan.pts, cx - (mx * T.ux + my * T.uy) * sc, T.ux * sc, T.uy * sc, cy + (mx * T.px + my * T.py) * sc, -T.px * sc, -T.py * sc)
  end
  -- Spur der letzten 40 s (gefaerbt nach Vario), Projektion ausgeschrieben: diese Schleife ist die teuerste
  do
    local kux, kuy, kpx, kpy = T.ux * sc, T.uy * sc, T.px * sc, T.py * sc
    local c1, c2, c3, c4 = x0, y0, x0 + bw - 1, y0 + bh - 1
    local tr, nowc, lastCol = w.trail, os.clock(), nil
    for i = #tr, 1, -1 do
      local p = tr[i]
      if nowc - p.t > 40 then break end
      local dx, dy = p.x - mx, p.y - my
      local ax, ay = cx + dx * kux + dy * kuy, cy - (dx * kpx + dy * kpy)
      if ax >= c1 and ax <= c3 and ay >= c2 and ay <= c4 then
        local col = (p.v or 0) > 0.5 and C_YELLOW or C_WHITE
        if col ~= lastCol then lcd.color(col); lastCol = col end
        lcd.drawFilledRectangle(floor(ax) - 1, floor(ay) - 1, 2, 2)
      end
    end
  end
  local vx, vy = P(c.x, c.y)
  lcd.color(C_YELLOW)
  if inClip(vx, vy) then disc(vx, vy, 4) end
  local ax, ay = P(f.x, f.y)
  if inClip(ax, ay) then predFlat(P, f, C_ORANGE, w); planeFlat(T, ax, ay, f.crs, 8, C_WHITE)
  else planeFlat(T, clamp(ax, x0 + 8, x0 + bw - 8), clamp(ay, y0 + 8, y0 + bh - 8), f.crs, 8, C_WHITE) end
  CLIP = nil
  -- Wind: Pfeil (wohin er blaest), Staerke daneben
  windFlat(w, T, x0 + bw - 24, y0 + 24, x0)
  drawT(x0 + 4, y0 + 2, bw, "TP" .. i .. (done and "  INNEN" or string.format("  %d m", math.floor(d + 0.5))), FONT_STD, done and C_GREEN or C_WHITE, "l")
  if g.plan and g.plan.turnDir then
    -- Ecke geschnitten: schnellste Wende zurueck in den Sektor
    drawT(x0 + 4, y0 + 2 + textH(FONT_STD), bw, g.plan.turnDir > 0 and "VERPASST: < LINKS WENDEN" or "VERPASST: RECHTS WENDEN >", FONT_STD, C_ORANGE, "l")
  end
  drawT(x0, y0 + bh - textH(FONT_XS) - 2, bw - 4, string.format("%d m Raster", math.floor(span)), FONT_XS, C_DIM, "r")
  -- Querlage unten links, bei jeder Ecke gleich (Christoph 2026-11-08), auf dunklem Kasten, ohne Gradzahl
  if w.qlPhi then
    local k = SMALL and 0.6 or 1
    local bx, by = x0 + 6, y0 + bh - 6 - 64 * k
    lcd.color(rgb(10, 10, 10))
    rect(bx, by, 96 * k, 64 * k)
    lcd.color(C_WHITE)
    BAL.QL.draw(w, bx + 48 * k, by + 36 * k, k, 0)
  end
end

-- Sollfahrt-Balken (Mitte = Soll), wie im Widget Sollfahrt
local function stfBar(w, x0, y0, bw, bh)
  lcd.color(rgb(40, 40, 40))
  rect(x0, y0, bw, bh)
  local mid = y0 + math.floor(bh / 2)
  local range = 20
  local px = (bh / 2 - 4) / range
  lcd.color(rgb(0, 70, 30))
  local dz = math.floor(w.dead * px)
  rect(x0, mid - dz, bw, math.max(dz * 2, 1))
  if w.stfDv then
    local dv = clamp(w.stfDv, -range, range)
    local h = math.floor(math.abs(dv) * px)
    local ex = math.abs(w.stfDv) - w.dead
    lcd.color(ex <= 0 and C_GREEN or (ex > 8 and C_RED or C_ORANGE))
    if dv >= 0 then rect(x0 + 3, mid - h, bw - 6, math.max(h, 1))
    else rect(x0 + 3, mid, bw - 6, math.max(h, 1)) end
  end
  lcd.color(C_WHITE)
  line(x0 - 3, mid, x0 + bw + 2, mid)
end

-- Flugzeug von hinten, um die Querlage gekippt, daneben die Gradzahl (grau, wenn der Sensor nicht passt)
function BAL.QL.draw(w, px, py, k, side)
  local phi = clamp(w.qlPhi, -80, 80)
  if w.qlBad then lcd.color(C_DIM) end
  local c, s = math.cos(math.rad(phi)), math.sin(math.rad(phi))
  local L, t = 36 * k, 2.5 * k                                  -- halbe Spannweite, halbe Strichdicke
  local ax, ay, bx, by = px - L * c, py - L * s, px + L * c, py + L * s
  local nx, ny = -s * t, c * t
  tri(ax + nx, ay + ny, bx + nx, by + ny, bx - nx, by - ny)
  tri(ax + nx, ay + ny, bx - nx, by - ny, ax - nx, ay - ny)
  lcd.drawFilledCircle(math.floor(px), math.floor(py), math.max(2, math.floor(7.5 * k)))
  local fx, fy = px + 17 * k * s, py - 17 * k * c                -- Seitenleitwerk steht senkrecht zum Fluegel
  local tx, ty = c * 2 * k, s * 2 * k
  tri(px + tx, py + ty, px - tx, py - ty, fx, fy)
  tri(px + tx, py + ty, fx, fy, fx + tx, fy + ty)
  if side == 0 then return end                                  -- ohne Gradzahl (Eckansicht)
  local a = math.floor(math.abs(w.qlPhi) + 0.5)
  local txt = a < 1 and "0°" or string.format("%d° %s", a, w.qlPhi > 0 and "R" or "L")
  local tw = 120
  if side > 0 then drawT(px + 44 * k, py + 8 * k, tw, txt, FONT_S, w.qlBad and C_DIM or C_WHITE, "l")
  else drawT(px - 44 * k - tw, py + 8 * k, tw, txt, FONT_S, w.qlBad and C_DIM or C_WHITE, "r") end
end

local function paintGlide(w, W, H, top)
  local T, f, g = w.task, w.cur, w.gc or {}
  local m = 6
  -- Links: Sollfahrt. Schrift wie die drei Felder unten (gemeinsame Groesse, siehe fitFont)
  local lw = math.floor(W * 0.24)
  local y = top
  local barW = SMALL and 22 or 48
  local tx, tw = m + barW + 8, lw - barW - 8
  local rz = w.rz
  local rzM = not w.rzMan and rz and rz.mcz
  -- MC ZEIT gewaehlt (Poti oben oder Tippen), aber noch nicht gerechnet (vor dem Start, erste Sekunden):
  -- trotzdem MC ZEIT zeigen, damit man die Wahl sieht
  local rzSel = not w.rzMan and w.rzOn ~= false
  local leftVals = w.polar and {
    {"SOLL", w.vstf and string.format("%d", math.floor(w.vstf * KMH + 0.5)) or "--", C_WHITE},
    {"IST", w.tasF and string.format("%d", math.floor(w.tasF * KMH + 0.5)) or "--", (w.stfDv and math.abs(w.stfDv) <= w.dead) and C_GREEN or C_ORANGE},
    rzSel and {"MC ZEIT", rzM and string.format("%.1f", rzM) or "--", C_BLUE} or {"MC", string.format("%.1f", w.mcValue), C_WHITE},
    {"NETTO", w.netto and string.format("%+.1f", w.netto) or "--", (w.netto or 0) >= 0.1 and C_GREEN or ((w.netto or 0) <= -0.1 and C_RED or C_WHITE)},
  } or nil
  -- groesste Schrift, in der alle Werte (links und unten) in ihre Felder passen; alle gleich gross
  local function fitFont(fields, fw, fh)
    local hL = textH(FONT_XS)
    for _, fo in ipairs({FONT_XXL, FONT_XL, F_L, FONT_L, FONT_STD}) do
      lcd.font(fo)
      local _, th = lcd.getTextSize("0")
      local ok = true
      if leftVals then
        if 4 * (hL + th) > H - y - m then ok = false end
        for _, v in ipairs(leftVals) do if ok and lcd.getTextSize(v[2]) > tw then ok = false end end
      end
      if ok and fields then
        if textH(FONT_S) + th + 4 > fh then ok = false end
        lcd.font(fo)
        for _, v in ipairs(fields) do if ok and v[2] and lcd.getTextSize(v[2]) > fw - 14 then ok = false end end
      end
      if ok then return fo end
    end
    return FONT_STD
  end
  local function drawLeft(vf)
    if leftVals then
      stfBar(w, m, y, barW, H - y - m)
      local step = math.floor((H - y - m) / 4)
      for i, v in ipairs(leftVals) do
        valBox(tx, y + (i - 1) * step, tw, v[1], v[2], v[3], vf)
        if i == 3 then
          w.mcBox = {x = tx, y = y + 2 * step, w = tw, h = step}
          -- MC ZEIT mit Pfeil nach unten: Hoehe wird abgebaut
          if rzM and rz.down then
            lcd.font(FONT_XS)
            local lw2, lh2 = lcd.getTextSize(v[1])
            local ax, ay = tx + lw2 + 4 + lh2 * 0.3, y + 2 * step + lh2 * 0.5
            lcd.color(C_BLUE)
            tri(ax, ay + lh2 * 0.3, ax - lh2 * 0.3, ay - lh2 * 0.1, ax + lh2 * 0.3, ay - lh2 * 0.1)
          end
        end
      end
    else
      drawT(m, y, lw, "Polare fehlt", FONT_S, C_ORANGE, "l")
      drawT(m, y + textH(FONT_S), lw, "(Sollfahrt aus)", FONT_XS, C_DIM, "l")
      valBox(m, y + 2 * textH(FONT_S) + 4, lw, "FAHRT", w.tasF and string.format("%d", math.floor(w.tasF * KMH + 0.5)) or "--", C_WHITE, vf)
    end
  end
  -- Rechts: Navigation
  local nx = lw + m * 2
  local nw = W - nx - m
  if not T then drawLeft(fitFont()); drawT(nx, y, nw, "Keine Aufgabe", F_L, C_RED, "c"); return end
  if w.state ~= S_STARTED or not g.plan or not f then
    drawLeft(fitFont())
    drawT(nx, y + 10, nw, w.state == S_STOPPED and "Aufgabe beendet" or "Noch nicht gestartet", F_L, C_DIM, "c")
    if f then
      local dx, dy = -f.x, -f.y
      drawT(nx, y + 10 + textH(F_L) + 6, nw, string.format("Ursprung %d m, %03d Grad", math.floor(math.sqrt(dx * dx + dy * dy) + 0.5), math.floor(bearingOf(dx, dy) + 0.5)), FONT_STD, C_WHITE, "c")
    end
    return
  end
  if g.corner then
    drawLeft(fitFont())
    local infoH = textH(FONT_S) + 2
    paintCorner(w, g, nx, y, nw, H - y - m - infoH)
    local sc = scoreOf(w)
    local wind = w.windOk and string.format("Wind %d aus %03d", math.floor(math.sqrt(w.wx ^ 2 + w.wy ^ 2) * KMH + 0.5), math.floor(bearingOf(-w.wx, -w.wy) + 0.5)) or "Wind: kreisen"
    drawT(nx, H - m - infoH + 2, nw, string.format("Dreiecke %d  %s", sc.n, wind), FONT_S, C_DIM, "c")
    return
  end
  -- Gegen-/Rueckenwind links (Feld), daneben das Ablagefenster bis zum rechten Rand
  local hb = SMALL and 26 or 46
  local cx0, cw = nx + hb + 6, nw - hb - 6
  local ccx = cx0 + cw / 2
  -- Kommando gross; Rechts steht links der Mitte, Links rechts davon: > und < zeigen auf die Mitte
  local err = g.rest or g.err or 0
  local a = math.abs(err)
  local cmd, col, side
  if g.plan.turnDir then cmd, col, side = g.plan.turnDir > 0 and "< KURVE LINKS" or "KURVE RECHTS >", C_ORANGE, g.plan.turnDir > 0 and 1 or -1
  elseif g.plan.inTurn then cmd, col, side = "KURVE", C_BLUE, 0
  elseif g.qlState == "hold" then cmd, col, side = "QUERLAGE HALTEN", C_GREEN, 0
  elseif g.qlState == "out" then cmd, col, side = "AUSLEITEN", C_BLUE, 0
  elseif a < 5 then cmd, col, side = "KURS OK", C_GREEN, 0
  elseif err > 0 then cmd, col, side = string.format("%d° RECHTS >", math.floor(a + 0.5)), a >= w.cmdMin and C_ORANGE or C_WHITE, -1
  else cmd, col, side = string.format("< %d° LINKS", math.floor(a + 0.5)), a >= w.cmdMin and C_ORANGE or C_WHITE, 1 end
  local th
  if side < 0 then th = drawT(nx, y, ccx - 12 - nx, cmd, F_XL, col, "r")
  elseif side > 0 then th = drawT(ccx + 12, y, nx + nw - ccx - 12, cmd, F_XL, col, "l")
  else th = drawT(ccx - nw, y, 2 * nw, cmd, F_XL, col, "c") end
  local yy = y + th + 4
  local cdiH = SMALL and 54 or 100
  local k = cdiH / 100
  lcd.color(rgb(25, 25, 25))
  rect(nx, yy, hb, cdiH)
  rect(cx0, yy, cw, cdiH)
  -- Gegenwind: Pfeil vom oberen Rand nach unten (orange), Rueckenwind vom unteren Rand nach oben (gruen)
  local hw = g.head
  if hw and math.abs(hw) >= 1 then
    local hcx = nx + hb / 2
    local L = clamp(math.abs(hw) * 14 * k, 4, cdiH - 44 * k)
    local hcol = hw > 0 and C_ORANGE or C_GREEN
    lcd.color(hcol)
    local sw, hh, hl = 6 * k, 17 * k, 18 * k
    local num = string.format("%d", math.floor(math.abs(hw) + 0.5))
    local nh = textH(FONT_S)
    if hw > 0 then
      rect(hcx - sw, yy + 2, 2 * sw, L)
      tri(hcx, yy + 2 + L + hl, hcx - hh, yy + 2 + L, hcx + hh, yy + 2 + L)
      drawT(nx, math.min(yy + 4 + L + hl, yy + cdiH - nh), hb, num, FONT_S, hcol, "c")
    else
      rect(hcx - sw, yy + cdiH - 2 - L, 2 * sw, L)
      tri(hcx, yy + cdiH - 2 - L - hl, hcx - hh, yy + cdiH - 2 - L, hcx + hh, yy + cdiH - 2 - L)
      drawT(nx, math.max(yy + cdiH - 4 - L - hl - nh, yy), hb, num, FONT_S, hcol, "c")
    end
  end
  local xt = g.xt or 0
  local far = math.abs(xt) > 300
  if far then
    -- weit daneben (z. B. abgetrieben): kein Massstab mehr, Peilung direkt zur naechsten Ecke
    drawT(cx0, yy + 2, cw, string.format("WEIT DANEBEN  %d m %s", math.floor(math.abs(xt) + 0.5), xt > 0 and "rechts" or "links"), FONT_S, C_ORANGE, "c")
    drawT(cx0, yy + 2 + textH(FONT_S), cw, string.format("TP%d direkt %03d°  %.1f km", g.next, math.floor(g.brgTP + 0.5), g.dV / 1000), SMALL and FONT_STD or FONT_L, C_WHITE, "c")
  else
    local xp = g.xtPred or xt
    local rng = (math.max(math.abs(xt), math.abs(xp)) > 100) and 300 or 100
    local half = cw / 2 - 4
    lcd.color(C_GRID)
    for j = -3, 3 do if j ~= 0 then local tx = ccx + j * half / 4; line(tx, yy, tx, yy + cdiH) end end
    -- Balken oben: erwartete Ablage in PRED_S s, von der Soll-Linie bis dorthin
    local qx = ccx + clamp(xp / rng, -1, 1) * half
    lcd.color(math.abs(xp) < 20 and C_GREEN or C_ORANGE)
    rect(math.min(ccx, qx), yy + 4 * k, math.max(math.abs(qx - ccx), 2), 34 * k)
    lcd.color(C_BLUE)
    line(ccx, yy, ccx, yy + cdiH)
    line(ccx + 1, yy, ccx + 1, yy + cdiH)
    drawT(ccx + 4, yy + 40 * k, cw / 2, string.format("±%d m", rng), FONT_XS, C_DIM, "l")
    -- Flugzeug (Dreieck) bei der aktuellen Ablage, zwischen Balken und Seitenwindpfeil
    local ppx = ccx + clamp(xt / rng, -1, 1) * half
    lcd.color(math.abs(xt) < 20 and C_GREEN or C_WHITE)
    if w.qlPhi then BAL.QL.draw(w, ppx, yy + 62 * k, k, ppx > ccx and -1 or 1)
    else tri(ppx, yy + 43 * k, ppx - 14 * k, yy + 76 * k, ppx + 14 * k, yy + 76 * k) end
  end
  -- Seitenwind unten: Pfeil vom Rand her, von links nach rechts bei Wind von links und umgekehrt
  local cr = g.cross
  if cr and math.abs(cr) >= 1 then
    local L = clamp(math.abs(cr) * 14 * k, 4, cw / 2 - 40)
    local ay = yy + cdiH - 11 * k
    local num = string.format("%d", math.floor(math.abs(cr) + 0.5))
    lcd.color(C_BLUE)
    if cr > 0 then
      local x0 = cx0 + 2
      rect(x0, ay - 3 * k, L, 6 * k)
      tri(x0 + L + 14 * k, ay, x0 + L, ay - 10 * k, x0 + L, ay + 10 * k)
      drawT(x0 + L + 18 * k, ay - textH(FONT_S) / 2, 60, num, FONT_S, C_BLUE, "l")
    else
      local x1 = cx0 + cw - 2
      rect(x1 - L, ay - 3 * k, L, 6 * k)
      tri(x1 - L - 14 * k, ay, x1 - L, ay - 10 * k, x1 - L, ay + 10 * k)
      drawT(x1 - L - 18 * k - 60, ay - textH(FONT_S) / 2, 60, num, FONT_S, C_BLUE, "r")
    end
  end
  yy = yy + cdiH + 8
  -- Unten drei Felder, in der App waehlbar (glideF1..3)
  local hf = g.hFin
  local function field(code)
    if code == 1 then return "TP" .. g.next, string.format("%d m", math.floor(g.dV + 0.5)), C_WHITE
    elseif code == 2 then return "WENDEN IN", fmtMS(g.plan.tT), (g.plan.tT or 99) < 10 and C_ORANGE or C_WHITE
    elseif code == 3 then return "HÖHE", string.format("%d m", math.floor(f.agl + 0.5)), C_WHITE
    elseif code == 4 then
      return "HÖHE ZIELLINIE", hf and string.format("%+d m", math.floor(hf + 0.5)) or "--",
        hf and (hf < 30 and C_RED or (hf < 80 and C_ORANGE or C_GREEN)) or C_DIM
    elseif code == 5 then local r = restTime(w); return "REST", r and fmtMS(math.max(r, 0)) or "--", C_WHITE
    elseif code == 6 then return "RUNDE " .. (#w.laps + 1), fmtMS((f.clk or 0) - (w.lapT or f.clk)), C_WHITE
    elseif code == 7 then local l = w.laps[#w.laps]; return "LETZTE RUNDE", l and string.format("%.1f", l.speed) or "--", C_WHITE
    elseif code == 8 then return "GRUND km/h", string.format("%d", math.floor((w.gsF or f.gs or 0) * KMH + 0.5)), C_WHITE
    elseif code == 9 then
      local vz = f.vz or 0
      return "GLEITZAHL", vz < -0.05 and string.format("%d", math.floor((w.gsF or f.gs or 0) / -vz + 0.5)) or "--", C_WHITE
    elseif code == 10 then
      local s, n = 0, w.avg30 and #w.avg30 or 0
      for j = 1, n do s = s + w.avg30[j] end
      local v = n > 0 and s / n or 0
      return "VARIO Ø 30 s", string.format("%+.1f", v), v >= 0 and C_GREEN or C_RED
    elseif code == 11 then
      return hw and (hw >= 0 and "GEGENWIND" or "RÜCKENWIND") or "WIND", hw and string.format("%d km/h", math.floor(math.abs(hw) + 0.5)) or "--",
        hw and (hw >= 0 and C_ORANGE or C_GREEN) or C_DIM
    elseif code == 12 then
      return "SEITENWIND", cr and string.format("%d %s", math.floor(math.abs(cr) + 0.5), cr > 0 and "v. li" or "v. re") or "--", C_WHITE
    elseif code == 13 then return "ABLAGE", string.format("%d m %s", math.floor(math.abs(xt) + 0.5), xt > 0 and "re" or "li"), C_WHITE
    elseif code == 14 then
      -- Restzeit-Sollfahrt: Dreiecke, die noch in die Arbeitszeit passen (das laufende zaehlt mit), darunter
      -- Reserve, was fuer eines mehr fehlt, Hoehenabbau oder "kein weiteres". Ohne Rechnung: geflogene Dreiecke.
      if not (rz and rz.N) then return "DREIECKE", string.format("%d", #w.laps), C_WHITE end
      if #w.laps < 1 then return "DREIECKE", "--", C_DIM, "ab Runde 2", nil, C_DIM end
      local N, b = rz.Ns or rz.N, rz.behind
      if N ~= rz.N then return "DREIECKE", string.format("%d", N), C_WHITE, "", nil, C_DIM end
      if N == 0 then return "DREIECKE", "0", C_ORANGE, "kein weiteres", "", C_ORANGE end
      if b and b > 0 and b < 30 then return "DREIECKE", string.format("%d", N), C_WHITE, string.format("%d", N + 1), string.format("fehlen %d s", math.floor(b + 0.5)), C_ORANGE end
      if rz.down then return "DREIECKE", string.format("%d", N), C_WHITE, "Höhe wird abgebaut", nil, C_BLUE end
      return "DREIECKE", string.format("%d", N), C_WHITE, rz.slack and (rz.slack < 100 and string.format("Reserve %+d s", math.floor(rz.slack + 0.5)) or ("Reserve " .. fmtMS(rz.slack))), nil, C_GREEN
    elseif code == 15 then return "RADIUS", string.format("%d m", math.floor((g.R or 0) + 0.5)), C_WHITE
    elseif code == 16 then return "MC ZEIT", rz and rz.mcz and string.format("%.1f", rz.mcz) or "--", C_BLUE
    elseif code == 17 then
      return "ENDANFLUG AB", rz and rz.hgo and string.format("%d m", math.floor(rz.hgo + 0.5)) or "--",
        rz and rz.hgo and (f.agl >= rz.hgo and C_GREEN or (rz.over and C_ORANGE or C_WHITE)) or C_DIM end
    return nil
  end
  local fh = H - m - yy
  local fw = nw / 3
  local vals = {}
  for i, code in ipairs({w.glideF1, w.glideF2, w.glideF3}) do vals[i] = {field(code)} end
  -- eine Schrift fuer alle sieben Werte (vier links, drei unten)
  local vf = fitFont(fh > 20 and vals or nil, fw, fh)
  drawLeft(vf)
  if fh > 20 then
    local hL = textH(FONT_S)
    -- Werte aller drei Felder auf gleicher Hoehe: Wert und Zeile darunter zusammen mittig unter der Beschriftung
    lcd.font(vf)
    local _, thv = lcd.getTextSize("0")
    local vy = yy + 2 + hL + math.max(math.floor((fh - 2 - hL - thv - hL) / 2), 0)
    for i = 1, 3 do
      local x = nx + (i - 1) * fw
      lcd.color(rgb(17, 17, 17))
      rect(x + 2, yy, fw - 4, fh)
      local v = vals[i]
      if v[1] then
        drawT(x + 8, yy + 2, fw - 12, v[1], FONT_S, C_DIM, "l")
        local th, tw = drawT(x + 8, vy, fw - 12, v[2], vf, v[3], "l")
        if v[6] then
          -- Feld der Restzeit-Sollfahrt: Dreieck hinter der Zahl, Zeile darunter
          local function triO(tx0, ty0, sz, col)
            lcd.color(col)
            line(tx0, ty0 + sz, tx0 + sz * 1.1, ty0 + sz); line(tx0, ty0 + sz, tx0 + sz * 0.55, ty0); line(tx0 + sz * 1.1, ty0 + sz, tx0 + sz * 0.55, ty0)
          end
          -- Ziffern stehen etwa von 4 % bis 64 % der Zeilenhoehe: Dreieck mit gleicher Unterkante
          triO(x + 8 + tw + th * 0.15, vy + th * 0.14, th * 0.5, v[3])
          if v[4] and vy + th + hL <= yy + fh then
            local sy = vy + th
            local _, w1 = drawT(x + 8, sy, fw - 12, v[4], FONT_S, v[6], "l")
            if v[5] then
              triO(x + 12 + w1, sy + hL * 0.14, hL * 0.5, v[6])
              if v[5] ~= "" then drawT(x + 16 + w1 + hL * 0.6, sy, fw - 12, v[5], FONT_S, v[6], "l") end
            end
          end
        end
      end
    end
  end
end

local function paintTherm(w, W, H, top)
  local T, f = w.task, w.cur
  local m = 6
  local y = top
  local lw = math.floor(W * 0.36)
  local half = math.floor(lw / 2)
  local tm = w.therm
  local rz = w.rz
  local thAvg, gain = nil, nil
  if tm and f then
    local dtm = os.clock() - tm.t0
    gain = f.agl - tm.h0
    if dtm > 5 then thAvg = gain / dtm end
  end
  -- Links oben vier Werte (2 x 2), in der App waehlbar (thermF1..4): 0 leer, 1 Vario, 2 Vario 30 s, 3 Thermik,
  -- 4 Gewinn, 5 Hoehe, 6 Netto Thermik, 7 Reserve zur Maximalhoehe, 8 Rest, 9 Dreiecke, 10 Zeit-MC, 11 Radius, 12 Wind
  local function tf(code)
    if code == 1 then local v = w.vFast; return "VARIO", v and string.format("%+.1f", v) or "--", (v or 0) >= 0 and C_GREEN or C_RED
    elseif code == 2 then
      local a = 0
      if w.avg30 and #w.avg30 > 0 then for _, x in ipairs(w.avg30) do a = a + x end; a = a / #w.avg30 end
      return "Ø 30 s", string.format("%+.1f", a), a >= 0 and C_GREEN or C_RED
    elseif code == 3 then return "Ø THERMIK", thAvg and string.format("%+.1f", thAvg) or "--", thAvg and (thAvg >= 0 and C_GREEN or C_RED) or C_DIM
    elseif code == 4 then return "GEWINN", gain and string.format("%+d m", math.floor(gain + 0.5)) or "--", gain and C_WHITE or C_DIM
    elseif code == 5 then return "HÖHE", f and string.format("%d m", math.floor(f.agl + 0.5)) or "--", C_WHITE
    elseif code == 6 then
      local tmN = tm or w.lastTherm
      local nAvg = tmN and tmN.nT and tmN.nT > 5 and tmN.nSum / tmN.nT or nil
      return (tm or SMALL) and "NETTO Ø THERMIK" or "NETTO Ø LETZTE", nAvg and string.format("%+.1f", nAvg) or "--",
        nAvg and (nAvg >= 0.1 and C_GREEN or (nAvg <= -0.1 and C_RED or C_WHITE)) or C_DIM
    elseif code == 7 then
      if not (T and f) then return "RESERVE", "--", C_DIM end
      local r = T.maxAlt - f.agl
      return SMALL and "RESERVE" or ("RESERVE ZU " .. T.maxAlt), string.format("%d m", math.floor(r + 0.5)), r < 30 and C_RED or (r < 80 and C_ORANGE or C_WHITE)
    elseif code == 8 then local r = restTime(w); return "REST", r and fmtMS(math.max(r, 0)) or "--", C_WHITE
    elseif code == 9 then
      local N = rz and #w.laps > 0 and (rz.Ns or rz.N)
      return "DREIECKE", N and tostring(N) or "--", N and C_WHITE or C_DIM
    elseif code == 10 then return "MC ZEIT", rz and rz.mcz and string.format("%.1f", rz.mcz) or "--", C_BLUE
    elseif code == 11 then return "RADIUS", string.format("%d m", math.floor(radius(w) + 0.5)), C_WHITE
    elseif code == 12 then
      return "WIND", w.windOk and string.format("%d km/h", math.floor(math.sqrt(w.wx ^ 2 + w.wy ^ 2) * KMH + 0.5)) or "--", w.windOk and C_WHITE or C_DIM
    end
  end
  -- Unten links der Kasten Endanflug: Hoehe bis zur Abflughoehe (rz.hgo), aus der man ohne weiteren Bart die
  -- N Dreiecke in der Restzeit schafft. Zeilen: Beschriftung, Wert, Zeile darunter, Balken.
  local hX, hS = textH(FONT_XS), textH(FONT_S)
  local bF = SMALL and FONT_L or FONT_XL
  local bH = hX + textH(bF) + hS + 14
  local N = rz and (rz.Ns or rz.N)
  local lab, val, col, sub, prog, arrowUp = "ENDANFLUG", "--", C_DIM, nil, nil, false
  if not (T and f and w.state == S_STARTED) then sub = "nach dem Start"
  elseif not rz then sub = w.rzOn and "rechnet ..." or "Restzeit-Rechnung aus"
  elseif #w.laps < 1 then sub = "ab Runde 2"
  elseif not rz.hgo then val, col, sub = "0", C_ORANGE, "kein Dreieck mehr möglich"
  else
    local h0 = tm and tm.h0 or f.agl
    local need = rz.hgo - f.agl
    if rz.over then
      local hmax = T.maxAlt - 30
      lab, val, col = SMALL and "ENDANFLUG NOCH NICHT" or "ENDANFLUG NOCH NICHT MÖGLICH", "1 Bart mehr", C_ORANGE
      sub = {string.format("bis %d m steigen · %d", hmax, N), true}
      prog = (f.agl - h0) / math.max(hmax - h0, 1)
    elseif not tm then
      -- im Gleitflug: nur die Abflughoehe, und ob die Hoehe reicht
      lab, val, col = "ENDANFLUG AB", string.format("%d m", math.floor(rz.hgo + 0.5)), need <= 0 and C_GREEN or C_WHITE
      sub = need <= 0 and {string.format("Höhe reicht für %d", N), true} or {string.format("fehlen %d m für %d", math.floor(need + 0.5), N), true}
    elseif need <= 0 then
      val, col = "RAUS", C_GREEN
      sub = {string.format("%d m erreicht · %d", math.floor(rz.hgo + 0.5), N), true}
      prog = 1
    else
      local t = need / rz.vcG
      lab, val, col, arrowUp = SMALL and "NOCH TANKEN" or "ENDANFLUG: NOCH TANKEN", string.format("+%d m", math.floor(need + 0.5)), C_BLUE, true
      sub = {string.format("ab %d m · %d", math.floor(rz.hgo + 0.5), N), true, t < 100 and string.format(" · ca. %d s", math.floor(t + 0.5)) or (" · ca. " .. fmtMS(t))}
      prog = tm and (f.agl - h0) / math.max(rz.hgo - h0, 1) or nil
    end
  end
  -- Raster der vier Werte: groesste Schrift, in der alle in ihre Zelle passen
  local gH = H - m - y - bH - 4
  local rh = math.floor(gH / 2)
  local vals = {}
  for i, code in ipairs({w.thermF1, w.thermF2, w.thermF3, w.thermF4}) do vals[i] = {tf(code)} end
  local vf = FONT_STD
  for _, fo in ipairs({FONT_XXL, FONT_XL, F_L, FONT_L}) do
    local ok = hX + textH(fo) <= rh
    if ok then
      lcd.font(fo)
      for i = 1, 4 do if ok and vals[i][2] and lcd.getTextSize(vals[i][2]) > half - 6 then ok = false end end
    end
    if ok then vf = fo; break end
  end
  local vy = math.max(math.floor((rh - hX - textH(vf)) / 2), 0)
  for i = 1, 4 do
    local v = vals[i]
    if v[1] then valBox(m + ((i - 1) % 2) * half, y + math.floor((i - 1) / 2) * rh + vy, half - 4, v[1], v[2], v[3], vf) end
  end
  -- Kasten Endanflug
  local by = H - m - bH
  lcd.color(rgb(17, 17, 17))
  rect(m, by, lw, bH)
  local bx = m + 6
  drawT(bx, by + 3, lw - 12, lab, FONT_XS, C_DIM, "l")
  local vy2 = by + 3 + hX
  local th = textH(bF)
  if arrowUp then
    -- Pfeil nach oben vor dem Wert, Hoehe wie die Ziffern
    local ax, ay, s = bx + th * 0.3, vy2 + th * 0.06, th * 0.6
    lcd.color(col)
    tri(ax, ay, ax - s * 0.4, ay + s * 0.45, ax + s * 0.4, ay + s * 0.45)
    rect(ax - s * 0.1, ay + s * 0.4, s * 0.2, s * 0.6)
    bx = bx + th * 0.75
  end
  drawT(bx, vy2, lw - 12, val, bF, col, "l")
  bx = m + 6
  local sy = vy2 + th
  if type(sub) == "table" then
    -- Zeile mit Dreieck-Zeichen (true) zwischen den Texten
    local x = bx
    for _, s in ipairs(sub) do
      if s == true then
        local sz = hS * 0.5
        lcd.color(col)
        local tx0, ty0 = x + 3, sy + hS * 0.14
        line(tx0, ty0 + sz, tx0 + sz * 1.1, ty0 + sz); line(tx0, ty0 + sz, tx0 + sz * 0.55, ty0); line(tx0 + sz * 1.1, ty0 + sz, tx0 + sz * 0.55, ty0)
        x = x + sz * 1.1 + 4
      else
        local _, tw = drawT(x, sy, lw - 12, s, FONT_S, col, "l")
        x = x + tw
      end
    end
  elseif sub then drawT(bx, sy, lw - 12, sub, FONT_S, C_DIM, "l") end
  if prog then
    local bw = lw - 12
    lcd.color(rgb(50, 50, 50))
    rect(bx, by + bH - 7, bw, 4)
    lcd.color(col)
    rect(bx, by + bH - 7, math.floor(bw * clamp(prog, 0, 1)), 4)
  end
  -- Abflughoehe im Bart erreicht: Banner ueber der Karte (am Ende gezeichnet)
  local banner = tm and val == "RAUS"
  -- Rechts: Kreisbild im festen Kartenbild (Hypotenuse unten), mit Steigen eingefaerbt, Pfeil zum besten Steigen
  local mx0 = lw + m * 2
  local mw = W - mx0 - m
  local mh = H - y - m
  lcd.color(rgb(15, 15, 15))
  rect(mx0, y, mw, mh)
  if f then
    local R = radius(w)
    local function Rf(x, yv) if T then return flatXY(T, x, yv) end return x, -yv end
    local cx, cy = mx0 + mw / 2, y + mh / 2
    -- Ausschnitt: fest um den Kreis gezoomt (Massstab nach Wenderadius), Mitte = Schwerpunkt der Kreise der
    -- letzten 20 s im Luftmassen-System. Das Dreieck darf aus dem Bild fallen, dann zeigt ein Pfeil am Rand
    -- zum naechsten Ziel.
    local wx, wy = w.windOk and w.wx or 0, w.windOk and w.wy or 0
    local nowc = os.clock()
    local half = clamp(R * 3.5, 35, 150)
    -- nur die letzten 20 s: von hinten durch die Spur (bis 80 Punkte), beim ersten aelteren Punkt aufhoeren
    local tr = w.trail
    local sx, sy, sn = 0, 0, 0
    for i = #tr, 1, -1 do
      local p = tr[i]
      local age = nowc - p.t
      if age >= 20 then break end
      sx, sy, sn = sx + p.x + wx * age, sy + p.y + wy * age, sn + 1
    end
    local ox, oy = f.x, f.y
    if sn >= 10 and (w.therm or w.circSince) then
      ox, oy = sx / sn, sy / sn
      -- Kreise fuellen das Bild: Massstab nach dem gemessenen Kreis statt nach dem gelernten Radius
      local r2 = 0
      for i = #tr, #tr - sn + 1, -1 do
        local p = tr[i]
        local age = nowc - p.t
        local dx, dy = p.x + wx * age - ox, p.y + wy * age - oy
        local d2 = dx * dx + dy * dy
        if d2 > r2 then r2 = d2 end
      end
      half = clamp(math.sqrt(r2) * 1.5, 25, 150)
      -- ruhiges Bild: Mitte (im Luftmassen-System) und Massstab geglaettet. Die 20 s sind selten ganze Kreise,
      -- roh pendeln Mitte und groesster Abstand mit jedem neuen Punkt. Groesser werden in ~1 s, kleiner in ~6 s,
      -- Aenderungen unter 10 % gar nicht.
      local v = w.thView
      if not v or nowc - v.t > 3 then v = {x = ox, y = oy, h = half, t = nowc, t0 = nowc}; w.thView = v end
      local mx, my = ox - wx * (nowc - v.t0), oy - wy * (nowc - v.t0)
      if v.t == nowc then
        v.x, v.y = mx, my
      else
        local dt = nowc - v.t
        local k = clamp(dt / 2, 0, 1)
        v.x, v.y = v.x + (mx - v.x) * k, v.y + (my - v.y) * k
        if half > v.h * 1.1 then v.h = v.h + (half - v.h) * clamp(dt, 0, 1)
        elseif half < v.h * 0.9 then v.h = v.h + (half - v.h) * clamp(dt / 6, 0, 1) end
        v.t = nowc
      end
      ox, oy, half = v.x + wx * (nowc - v.t0), v.y + wy * (nowc - v.t0), v.h
    else
      w.thView = nil
    end
    local sc = math.min(mw, mh) / 2 / half
    do
      local pad = 16
      local fx, fy = Rf(f.x - ox, f.y - oy)
      fx, fy = fx * sc, fy * sc
      cx = cx - (fx - clamp(fx, -mw / 2 + pad, mw / 2 - pad))
      cy = cy - (fy - clamp(fy, -mh / 2 + pad, mh / 2 - pad))
    end
    CLIP = {mx0, y, mx0 + mw - 1, y + mh - 1}
    local function P(x, yv)
      local rx, ry = Rf(x - ox, yv - oy)
      return cx + rx * sc, cy + ry * sc
    end
    local ni = w.tp % 3 + 1
    if T then
      -- Schenkel, Startlinie, Sektorkanten des naechsten TP, Wendepunkte (erledigt grau, naechster gruen)
      -- aktiver Sektor gefuellt wie in der Eckansicht
      fillSector(P, T.tp[ni], bearingOf(T.tp[ni].bx, T.tp[ni].by), mx0, y, mw, mh)
      drawZones(w, T, P, mx0, y, mw, mh)
      lcd.color(rgb(90, 90, 90))
      for i = 1, 3 do
        local a, b = T.tp[i], T.tp[i % 3 + 1]
        local ax, ay = P(a.x, a.y)
        local bx, by = P(b.x, b.y)
        line(ax, ay, bx, by)
      end
      lcd.color(C_YELLOW)
      local lh = math.min(LINE_HALF, T.dist * 0.4)
      local l1x, l1y = P(T.px * lh, T.py * lh)
      local l2x, l2y = P(-T.px * lh, -T.py * lh)
      line(l1x, l1y, l2x, l2y)
      local c = T.tp[ni]
      local bd = bearingOf(c.bx, c.by)
      lcd.color(rgb(0, 110, 50))
      for _, off in ipairs({-45, 45}) do
        local ex, ey = dirVec(bd + off)
        local ax, ay = P(c.x, c.y)
        local bx, by = P(c.x + ex * T.dist * 0.3, c.y + ey * T.dist * 0.3)
        line(ax, ay, bx, by)
      end
      for i = 1, 3 do
        local tx, ty = P(T.tp[i].x, T.tp[i].y)
        if inClip(tx, ty) then
          local nextOne = (w.state == S_STARTED or w.state == S_ARMED) and i == ni
          lcd.color(nextOne and C_GREEN or C_DIM)
          disc(tx, ty, nextOne and 5 or 3)
          if inClip(tx + 30, ty + 14) then drawT(tx + 6, ty - 6, 40, tostring(i), FONT_XS, nextOne and C_GREEN or C_DIM, "l") end
        end
      end
    end
    -- Spur im Luftmassen-System (um den Wind zurueckversetzt), dann liegen die Kreise uebereinander.
    -- Gezeichnet als 3 px breite, glatte Kurve: die Spurpunkte (hoechstens 40, bei mehr jeder zweite, dritte ...) sind
    -- Kontrollpunkte einer quadratischen B-Spline, jedes Stueck geht von Mitte zu Mitte zweier Punkte und wird
    -- ab 14 px in 2, ab 28 px in 3 Teilstuecke zerlegt (wenige GPS-Punkte pro Kreis bleiben rund). Farbe nach dem
    -- Steigen am Kontrollpunkt, die neueste Spur liegt oben.
    local best, bv, bx0, by0 = nil, -99, 0, 0
    local showBest = w.thBest
    local lastCol = nil
    local c1, c2, c3, c4 = mx0 + 2, y + 2, mx0 + mw - 3, y + mh - 3
    -- P() ausgeschrieben (Drehung + Massstab), spart je Punkt zwei Funktionsaufrufe
    local kxx, kxy, kyx, kyy, ex, ey
    if T then kxx, kxy, kyx, kyy = T.ux * sc, T.uy * sc, -T.px * sc, -T.py * sc else kxx, kxy, kyx, kyy = sc, 0, 0, -sc end
    ex, ey = cx - ox * kxx - oy * kxy + 0.5, cy - ox * kyx - oy * kyy + 0.5
    -- Teilstueck 3 px breit (ganze Pixel): drei Linien quer zur Hauptrichtung versetzt
    local DL = lcd.drawLine
    local function seg(x1, y1, x2, y2)
      local dx, dy = x2 - x1, y2 - y1
      if (dx < 0 and -dx or dx) >= (dy < 0 and -dy or dy) then
        DL(x1, y1 - 1, x2, y2 - 1); DL(x1, y1, x2, y2); DL(x1, y1 + 1, x2, y2 + 1)
      else
        DL(x1 - 1, y1, x2 - 1, y2); DL(x1, y1, x2, y2); DL(x1 + 1, y1, x2 + 1, y2)
      end
    end
    -- erster Punkt juenger als 35 s (Zeiten steigen: Binaersuche), Schrittweite fuer hoechstens 40 Punkte
    local n = #tr
    local lo, hi, tmin = 1, n, nowc - 35
    while lo < hi do
      local mid = (lo + hi) // 2
      if tr[mid].t <= tmin then lo = mid + 1 else hi = mid end
    end
    local step = (n - lo) // 40 + 1
    lo = n - (n - lo) // step * step
    local px, py, pv0, mx, my
    for i = lo, n, step do
      local p = tr[i]
      local age = nowc - p.t
      local qx, qy = p.x + wx * age, p.y + wy * age
      local vx, vy = ex + qx * kxx + qy * kxy, ey + qx * kyx + qy * kyy
      local pv = p.v or 0
      if showBest and pv > bv then bv, best, bx0, by0 = pv, p, qx, qy end
      if px then
        local nx, ny = (px + vx) * 0.5, (py + vy) * 0.5
        if mx >= c1 and mx <= c3 and my >= c2 and my <= c4 and px >= c1 and px <= c3 and py >= c2 and py <= c4
          and nx >= c1 and nx <= c3 and ny >= c2 and ny <= c4 then
          local col = pv0 > 1.5 and C_RED or (pv0 > 0.5 and C_YELLOW or (pv0 > 0 and C_GREEN or C_BLUE))
          if col ~= lastCol then lcd.color(col); lastCol = col end
          local d, e = nx - mx, ny - my
          d = (d < 0 and -d or d) + (e < 0 and -e or e)
          local x0, y0 = floor(mx), floor(my)
          if d > 14 then
            -- B(t) = m + 2t(c - m) + t^2 (m - 2c + n), in k gleichen Schritten
            local k = d > 28 and 3 or 2
            local ax, ay = 2 * (px - mx), 2 * (py - my)
            local bx, by = mx - px - px + nx, my - py - py + ny
            for j = 1, k - 1 do
              local t = j / k
              local x1, y1 = floor(mx + t * (ax + t * bx)), floor(my + t * (ay + t * by))
              seg(x0, y0, x1, y1)
              x0, y0 = x1, y1
            end
          end
          seg(x0, y0, floor(nx), floor(ny))
        end
        px, py, pv0, mx, my = vx, vy, pv, nx, ny
      else
        px, py, pv0, mx, my = vx, vy, pv, vx, vy
      end
    end
    -- letztes halbes Stueck bis zum neuesten Punkt
    if px and mx >= c1 and mx <= c3 and my >= c2 and my <= c4 and px >= c1 and px <= c3 and py >= c2 and py <= c4 then
      local col = pv0 > 1.5 and C_RED or (pv0 > 0.5 and C_YELLOW or (pv0 > 0 and C_GREEN or C_BLUE))
      if col ~= lastCol then lcd.color(col) end
      seg(floor(mx), floor(my), floor(px), floor(py))
    end
    local ax0, ay0 = P(f.x, f.y)
    -- rote Linie zum besten Steigen der Spur (App thBest, Vorgabe aus)
    if best and bv > 0.3 then
      local bx, by = P(bx0, by0)
      lcd.color(C_RED)
      if inClip(bx, by) then ring(bx, by, 7) end
      line(ax0, ay0, bx, by)
    end
    CLIP = nil
    if T then planeFlat(T, ax0, ay0, f.crs, 8, C_WHITE) else planeSymbol(ax0, ay0, 8, C_WHITE) end
    if T then
      -- Ziel: naechster TP, vor dem Start die Startlinie
      local started = w.state == S_STARTED
      local gx, gy = 0, 0
      if started then gx, gy = T.tp[ni].x, T.tp[ni].y end
      local name = started and ("TP" .. ni) or "Start"
      local dist = math.floor(math.sqrt((gx - f.x) ^ 2 + (gy - f.y) ^ 2) + 0.5)
      local tx, ty = P(gx, gy)
      if not (tx >= mx0 + 6 and tx <= mx0 + mw - 7 and ty >= y + 6 and ty <= y + mh - 7) then
        -- Ziel ausserhalb: Pfeil am Rand in seiner Richtung, mit Distanz
        local ccx, ccy = mx0 + mw / 2, y + mh / 2
        local ex, ey = tx - ccx, ty - ccy
        local n = math.max(math.sqrt(ex * ex + ey * ey), 1e-3)
        ex, ey = ex / n, ey / n
        -- unten bleibt die Windzeile frei
        local k = math.min((mw / 2 - 6) / math.max(math.abs(ex), 1e-3), (mh / 2 - (ey > 0 and 26 or 6)) / math.max(math.abs(ey), 1e-3))
        local L = 34
        local hx, hy = ccx + ex * k, ccy + ey * k
        lcd.color(C_GREEN)
        arrow(hx - ex * L, hy - ey * L, ex, ey, L, 6, 9)
        -- Beschriftung hinter dem Pfeilende, bei waagrechtem Pfeil darueber
        local lx, ly = hx - ex * (L + 70), hy - ey * (L + 20)
        if math.abs(ey) < 0.5 then lx, ly = hx - ex * (L / 2 + 60), hy - 30 end
        lx = clamp(lx, mx0 + 64, mx0 + mw - 64)
        ly = clamp(ly, y + 4, y + mh - 24)
        drawT(lx - 60, ly, 120, string.format("%s %d m", name, dist), FONT_STD, C_GREEN, "c")
      else
        drawT(mx0 + 4, y + 2, mw, string.format("%s %d m", name, dist), FONT_S, C_GREEN, "l")
      end
    end
    if T then windFlat(w, T, mx0 + mw - 26, y + 24, mx0 + math.floor(mw / 2)) end
    local wind = w.windOk and string.format("Wind %d km/h aus %03d", math.floor(math.sqrt(w.wx ^ 2 + w.wy ^ 2) * KMH + 0.5), math.floor(bearingOf(-w.wx, -w.wy) + 0.5)) or "Wind: noch kein Fit"
    drawT(mx0, y + mh - textH(FONT_XS) - 2, mw - 4, wind .. string.format("  R %d m", math.floor(R + 0.5)), FONT_XS, C_DIM, "r")
  end
  if banner then
    local txt = "BART VERLASSEN · ENDANFLUG"
    local bf2 = F_L
    lcd.font(bf2)
    if lcd.getTextSize(txt) > mw - 12 then bf2 = FONT_STD; lcd.font(bf2); if lcd.getTextSize(txt) > mw - 12 then txt = "ENDANFLUG" end end
    local hb = textH(bf2) + 8
    lcd.color(C_GREEN)
    rect(mx0, y, mw, hb)
    drawT(mx0, y + 4, mw, txt, bf2, C_BG, "c")
  end
end

local function paintEval(w, W, H, top)
  local T = w.task
  local m = 6
  local y = top
  local sc = scoreOf(w)
  local col4 = math.floor((W - 2 * m) / 4)
  valBox(m, y, col4, "DREIECKE", tostring(sc.n), C_WHITE, F_XL)
  valBox(m + col4, y, col4, "SCHNITT", sc.speed and string.format("%.1f km/h", sc.speed) or "--", C_WHITE, F_L)
  valBox(m + 2 * col4, y, col4, "FLUGZEIT", sc.time and fmtMS(sc.time) or "--", C_WHITE, F_L)
  local hh = valBox(m + 3 * col4, y, col4, "STRAFE", tostring(sc.penalty), sc.penalty > 0 and C_RED or C_GREEN, F_L)
  y = y + hh + 2
  local info = {}
  if w.entry then info[#info + 1] = string.format("Einflug %d m / %d km/h", math.floor(w.entry.agl + 0.5), math.floor(w.entry.v3d + 0.5)) end
  if w.zoneHit then info[#info + 1] = "SICHERHEITSZONE verletzt (Wertung 0)" end
  if w.altHit then info[#info + 1] = "Maximalhöhe überschritten" end
  if w.stopReason then info[#info + 1] = "Ende: " .. w.stopReason end
  if w.lastIgc or (w.igc and w.igc.path) then info[#info + 1] = "IGC: " .. (tostring(w.igc and w.igc.path or w.lastIgc):match("([^/]+)$") or "") end
  drawT(m, y, W - 2 * m, table.concat(info, "   "), FONT_XS, (w.zoneHit or w.altHit) and C_RED or C_DIM, "l")
  y = y + textH(FONT_XS) + 4
  -- Rundentabelle
  local cols = {"Runde", "Zeit", "km/h", "Index", "Höhe", "dH"}
  local cw = math.floor((W - 2 * m) / #cols)
  lcd.color(rgb(40, 40, 40))
  local rh = textH(FONT_S) + 2
  rect(m, y, W - 2 * m, rh)
  for k, c in ipairs(cols) do drawT(m + (k - 1) * cw, y + 1, cw - 6, c, FONT_S, C_DIM, "r") end
  y = y + rh
  local rows = math.max(1, math.floor((H - y - m) / rh))
  local n = #w.laps
  w.evalScroll = clamp(w.evalScroll or 0, 0, math.max(0, n - rows))
  local best = nil
  for _, l in ipairs(w.laps) do if not best or l.time < best then best = l.time end end
  for k = 1 + w.evalScroll, math.min(n, w.evalScroll + rows) do
    local l = w.laps[k]
    local cells = {tostring(l.n), fmtMS(l.time), string.format("%.1f", l.speed), l.idx and string.format("%.1f%%", l.idx) or "--", string.format("%d", math.floor(l.agl + 0.5)), string.format("%+d", math.floor(l.dAlt + 0.5))}
    for c, s in ipairs(cells) do drawT(m + (c - 1) * cw, y, cw - 6, s, FONT_S, (l.time == best) and C_GREEN or C_WHITE, "r") end
    y = y + rh
  end
  if n == 0 then drawT(m, y + 4, W - 2 * m, "Noch keine Runde", FONT_S, C_DIM, "c") end
  -- Neuer Flug (2x tippen)
  local bw = math.floor(W * 0.22)
  w.resetBox = {x = W - m - bw, y = H - m - rh - 4, w = bw, h = rh + 4}
  local armed = w.resetArmed and os.clock() - w.resetArmed < 4
  lcd.color(armed and rgb(120, 30, 0) or rgb(50, 50, 50))
  rect(w.resetBox.x, w.resetBox.y, bw, rh + 4)
  drawT(w.resetBox.x, w.resetBox.y + 2, bw, armed and "NOCHMAL = NEU" or "NEUER FLUG", FONT_S, C_WHITE, "c")
end

local function paint(w)
  colors()
  local W, H = lcd.getWindowSize()
  local small = W < 600 or H < 260
  F_L, F_XL = small and FONT_STD or FONT_L, small and FONT_L or FONT_XL
  SMALL = small
  lcd.color(C_BG)
  rect(0, 0, W, H)
  local top = header(w, W)
  H = tabBar(w, W, H)
  w.armBox, w.resetBox, w.mcBox, w.qlBox = nil, nil, nil, nil
  if w.page == P_OVER then paintOver(w, W, H, top)
  elseif w.page == P_START then paintStart(w, W, H, top)
  elseif w.page == P_GLIDE then paintGlide(w, W, H, top)
  elseif w.page == P_THERM then paintTherm(w, W, H, top)
  else paintEval(w, W, H, top) end
  footerMsg(w, W, H, os.clock())
end

-- ---------------------------------------------------------------- Touch / Tasten

local function inside(b, x, y)
  return b and x >= b.x and x <= b.x + b.w and y >= b.y and y <= b.y + b.h
end

local function eventHandler(w, category, value, x, y)
  if w.qlPress and EVT_TOUCH and category == EVT_TOUCH and TOUCH_END and value == TOUCH_END then w.qlPress = nil; lcd.invalidate() end
  if EVT_TOUCH and category == EVT_TOUCH and (TOUCH_START == nil or value == TOUCH_START) then
    if inside(w.hdrBox, x, y) then
      local tabW = w.hdrBox.w / #PAGE_NAMES
      local p = clamp(math.floor(x / tabW) + 1, 1, #PAGE_NAMES)
      w.manual, w.manualFrom, w.page = p, w.autoTarget, p
      lcd.invalidate(); return true
    end
    if inside(w.qlBox, x, y) then
      if TOUCH_END == nil then
        -- ohne Loslassen-Ereignis: zweimal tippen innert 3 s
        local now = os.clock()
        if w.qlTap and now - w.qlTap < 3 then w.qlTap = nil; w.qlPress = now - 2 else w.qlTap = now end
      else
        w.qlPress = os.clock()
      end
      lcd.invalidate(); return true
    end
    if inside(w.armBox, x, y) and w.task then
      armOrRestart(w, os.clock())
      lcd.invalidate(); return true
    end
    if inside(w.mcBox, x, y) and not validSource(w.mcSrc) then
      w.rzTap = not w.rzTap
      lcd.invalidate(); return true
    end
    if inside(w.resetBox, x, y) then
      local now = os.clock()
      if w.resetArmed and now - w.resetArmed < 4 then resetFlight(w); w.resetArmed = nil else w.resetArmed = now end
      lcd.invalidate(); return true
    end
  end
  if EVT_KEY and category == EVT_KEY then
    local up = KEY_ROTARY_RIGHT and value == KEY_ROTARY_RIGHT
    local dn = KEY_ROTARY_LEFT and value == KEY_ROTARY_LEFT
    if (up or dn) and w.page == P_EVAL then w.evalScroll = (w.evalScroll or 0) + (up and 1 or -1); lcd.invalidate(); return true end
    if (up or dn) and w.page == P_GLIDE and not validSource(w.mcSrc) then
      -- Drehknopf: ueber das MC-Maximum hinaus nach rechts = MC ZEIT, von dort nach links zurueck auf das Maximum
      if w.rzTap then
        if dn then w.rzTap = false end
      elseif up and w.mc >= w.mcMax and w.rzOn ~= false then w.rzTap = true
      else w.mc = clamp(w.mc + (up and 1 or -1), 0, w.mcMax) end
      w.mcValue = w.mc / 10; lcd.invalidate(); return true
    end
  end
  return false
end

-- ---------------------------------------------------------------- Konfiguration

local function addSource(w, label, prop)
  local l = form.addLine(label)
  return form.addSourceField(l, nil, function() return w[prop] end, function(v) w[prop] = v end)
end
local function addSwitch(w, label, prop)
  local l = form.addLine(label)
  return form.addSwitchField(l, nil, function() return w[prop] end, function(v) w[prop] = v end)
end
local function addNumber(w, label, prop, lo, hi)
  local l = form.addLine(label)
  return form.addNumberField(l, nil, lo, hi, function() return w[prop] end, function(v) w[prop] = v end)
end
local function addBool(w, label, prop)
  local l = form.addLine(label)
  return form.addBooleanField(l, form.getFieldSlots(l)[0], function() return w[prop] end, function(v) w[prop] = v end)
end
local function addChoice(w, label, prop, choices, after)
  local l = form.addLine(label)
  return form.addChoiceField(l, nil, choices, function() return w[prop] end, function(v) w[prop] = v; if after then after(v) end end)
end
local function addText(w, label, prop)
  local l = form.addLine(label)
  if form.addTextField then
    return form.addTextField(l, nil, function() return w[prop] end, function(v) w[prop] = v end)
  end
end


local function configure(w)
  refreshTasks(w, os.clock(), true)
  local l = form.addLine("Aufgabe")
  if #w.tasks == 0 then
    form.addStaticText(l, form.getFieldSlots(l)[0], w.tasksFound and "keine Aufgabe in aufgaben.lua" or "aufgaben.lua fehlt")
    l = form.addLine("Hinweis")
    form.addStaticText(l, form.getFieldSlots(l)[0], "MM Flight Analyzer, Tab SkyNavigator: Aufgabe anlegen, Sender anschliessen")
  else
    local ch = {{"Aktive Aufgabe der App", 0}}
    for i, T in ipairs(w.tasks) do ch[#ch + 1] = {string.format("%s (%s %dm)%s", T.name, T.cls, math.floor(T.dist + 0.5), T.feld and " Sender" or ""), i} end
    form.addChoiceField(l, nil, ch, function() return clamp(w.taskIdx, 0, #w.tasks) end, function(v)
      w.taskIdx = v; w.taskSig = nil; refreshTasks(w, os.clock(), true) end)
  end
  -- Sensoren: Geraete waehlen, das Widget sucht die Quellen selbst (BAL.SN)
  if not w.man then BAL.SN.load(w) else BAL.SN.resolveAll(w) end
  local function rebuild()
    BAL.SN.resolveAll(w)
    if form.clear then pcall(form.clear); configure(w) end
  end
  addChoice(w, "Sensor-Gerät", "devMain", {{"SM GPS-Logger 3", 1}, {"FrSky Vario (+ FrSky-GPS)", 2}, {"keines (Quellen von Hand)", 0}}, rebuild)
  l = form.addLine("Zusätzlich Stabi-Empfänger (Querlage, Lage)")
  form.addBooleanField(l, form.getFieldSlots(l)[0], function() return w.devStabi == true end, function(v) w.devStabi = v; rebuild() end)
  if BAL.SN.on(w) then
    local fr, mi = BAL.SN.summary(w)
    for i, t in ipairs(fr) do l = form.addLine(i == 1 and "  gefunden" or ""); form.addStaticText(l, form.getFieldSlots(l)[0], t) end
    for i, t in ipairs(mi) do l = form.addLine(i == 1 and "  fehlt" or ""); form.addStaticText(l, form.getFieldSlots(l)[0], t) end
  end
  if w.devStabi or BAL.SN.isSet(w.man.bankSrc) then addBool(w, "Querlage: Richtung umkehren", "bankInv") end
  addChoice(w, "Höhe für Wertung", "altMode", {{"GPS minus Platzhöhe", 1}, {"Relative Höhe", 2}})
  addChoice(w, "Varioton-Ausgabe", "vtDev", {{"Widget-Toene", 1}, {"Ethos-Vario (Sensor SkyNav Vario)", 2}})
  if w.vtDev == 2 then
    for i, t in ipairs(BAL.VO.info(w)) do l = form.addLine(i == 1 and "  Stand" or ""); form.addStaticText(l, form.getFieldSlots(l)[0], t) end
  end
  addChoice(w, "Rechnen (Test)", "calcIv", {{"jeder Aufruf (normal)", 1}, {"jeder 2. Aufruf", 2}, {"jeder 4. Aufruf", 3}})
  addChoice(w, "Bildaufbau", "drawIv", {{"7 pro s (normal)", 1}, {"3 pro s", 2}, {"1.7 pro s", 3}, {"1 pro s", 4}})
  for i, t in ipairs(BAL.TM.info()) do l = form.addLine(i == 1 and "  Last (Mittel/Max)" or ""); form.addStaticText(l, form.getFieldSlots(l)[0], t) end
  addChoice(w, "Varioton-Klang (Widget-Toene)", "vtStyle", {{"XCSoar", 1}, {"OpenVario", 2}, {"XCVario", 3}, {"Ethos/EdgeTX", 4}})
  -- Seiten
  addSwitch(w, "Schalter Seite Übersicht", "swOver")
  addSwitch(w, "Schalter Seite Start", "swStart")
  addSwitch(w, "Schalter Seite Gleiten", "swGlide")
  addSwitch(w, "Schalter Seite Thermik", "swTherm")
  addSwitch(w, "Schalter Seite Auswertung", "swEval")
  addSwitch(w, "Scharf-Schalter (ARM)", "armSw")
  addSwitch(w, "Ton-Schalter (optional)", "soundSw")
  addBool(w, "Gleiten: Kurs-Ticker (Klicks bei wachsender Ablage)", "ticker")
  addSource(w, "MC-Quelle (optional)", "mcSrc")
  addSource(w, "Ballast-Quelle (optional, 0 bis 100 %)", "balSrc")
  addSource(w, "Querruder-Knüppel (Querlage ohne Lagesensor)", "ailSrc")
  l = form.addLine("Querlage nullen (Flügel waagrecht)")
  if form.addTextButton then
    form.addTextButton(l, nil, "Jetzt nullen", function()
      if BAL.QL.zero(w) then note(w, "Querlage genullt", os.clock()) end
      return true
    end)
  end
  local bf = addNumber(w, "Ballast maximal (ohne Quelle: fix)", "balMax", 0, 200)
  if bf then pcall(bf.decimals, bf, 1); pcall(bf.suffix, bf, " kg") end
  addNumber(w, "Übersicht: Spur der letzten x min (0 = aus)", "trackMin", 0, 5)
  -- Zeit
  addNumber(w, "Startfenster von Hand: Stunde (-1 = Aufgabe)", "slotH", -1, 23)
  addNumber(w, "Startfenster von Hand: Minute", "slotM", 0, 59)
  addNumber(w, "Startfenster von Hand: Länge (min)", "slotLen", 1, 120)
  -- Einzelne Quellen von Hand: gehen der Automatik vor (leer = automatisch)
  local hand = not BAL.SN.on(w)
  if not hand and form.clear then
    l = form.addLine("Einzelne Quellen von Hand")
    form.addBooleanField(l, form.getFieldSlots(l)[0], function() return w.snShow == true end, function(v)
      w.snShow = v; pcall(form.clear); configure(w) end)
  end
  if hand or w.snShow or not form.clear then
    local function man(label, k)
      local ln = form.addLine(label)
      return form.addSourceField(ln, nil, function() return w.man[k] end, function(v) w.man[k] = v; BAL.SN.applyKey(w, k) end)
    end
    local function manCh(label, k, ch)
      local ln = form.addLine(label)
      return form.addChoiceField(ln, nil, ch, function() return w.man[k] end, function(v) w.man[k] = v; BAL.SN.apply(w) end)
    end
    if not hand then l = form.addLine("  (leer = automatisch)") end
    man("GPS", "gps")
    man("GPS-Geschw. (empfohlen)", "gpsSpd")
    man("GPS-Höhe (ü. M.)", "gpsAlt")
    man("GPS-Zeit (sonst Senderuhr, Uhr mit *)", "gpsTime")
    man("Relative Höhe (optional)", "relAlt")
    man("Vario", "vario")
    manCh("Vario-Art", "varioType", {{"Automatisch (nach Gerät)", 0}, {"TE (Sensor kompensiert)", 1}, {"Barometrisch (Widget kompensiert)", 2}})
    man("Fahrt Staudruck (optional, z.B. Airspeed)", "airSrc")
    manCh("Fahrt-Sensor: Einheit", "airUnit", {{"Automatisch (ohne: Knoten)", 1}, {"km/h", 2}, {"Knoten", 3}, {"m/s", 4}})
    man("Flugrichtung / GPS-Kurs (optional, Grad)", "crsSrc")
    man("GPS-Satelliten (optional, Warnung unter 6)", "satSrc")
    man("Querlage (optional, Rollwinkel Grad)", "bankSrc")
    if not (w.devStabi or BAL.SN.isSet(w.man.bankSrc)) then addBool(w, "Querlage: Richtung umkehren", "bankInv") end
    man("Nickwinkel (optional, mit AccZ)", "pitchSrc")
    man("AccZ (optional, Varioton schneller)", "accZSrc")
    man("AccX (optional, mit AccZ)", "accXSrc")
  end
  l = form.addLine("Weitere Einstellungen")
  form.addStaticText(l, form.getFieldSlots(l)[0], "in der App (Tab SkyNavigator)")
  l = form.addLine("Version")
  form.addStaticText(l, form.getFieldSlots(l)[0], "SkyNavigator " .. VERSION)
end

local function setDefaults(w)
  for _, k in ipairs(SOURCE_KEYS) do w[k] = nil end
  for _, d in ipairs(NUM_DEFS) do w[d.key] = d.def end
  for _, d in ipairs(BOOL_DEFS) do w[d.key] = d.def end
  for _, d in ipairs(TEXT_DEFS) do w[d.key] = d.def end
  w.balSrc, w.balMax = nil, 0
  w.ticker = true
  w.bankSrc, w.bankInv = nil, false
  w.accZSrc, w.accXSrc, w.pitchSrc = nil, nil, nil
  w.ailSrc = nil
  w.airSrc, w.airUnit, w.crsSrc, w.satSrc = nil, 1, nil, nil
  w.vtStyle, w.vtDev = 1, 1
  w.drawIv, w.calcIv = 1, 1
  w.devMain, w.devStabi, w.varioType = 1, false, 0   -- 1.30: neue Widgets suchen die Sensoren des SM GPS-Logger 3
end

local function read(w)
  if storage.read("ver") ~= STORE_VER then setDefaults(w)
  else
    for _, k in ipairs(SOURCE_KEYS) do w[k] = validSource(storage.read(k)) end
    for _, d in ipairs(NUM_DEFS) do local v = storage.read(d.key); w[d.key] = type(v) == "number" and v or d.def end
    for _, d in ipairs(BOOL_DEFS) do local v = storage.read(d.key); if type(v) == "boolean" then w[d.key] = v else w[d.key] = d.def end end
    for _, d in ipairs(TEXT_DEFS) do local v = storage.read(d.key); w[d.key] = type(v) == "string" and v or d.def end
    w.balSrc = validSource(storage.read("balSrc"))
    local v = storage.read("balMax"); w.balMax = type(v) == "number" and v or 0
    v = storage.read("ticker"); w.ticker = v ~= false   -- steht zuletzt: aeltere Konfigurationen behalten ihre Reihenfolge
    w.bankSrc = validSource(storage.read("bankSrc"))   -- Querlage: nach dem Ticker angehaengt
    w.bankInv = storage.read("bankInv") == true
    w.accZSrc = validSource(storage.read("accZSrc"))   -- Fusion Varioton: nach der Querlage angehaengt
    w.accXSrc = validSource(storage.read("accXSrc"))
    w.pitchSrc = validSource(storage.read("pitchSrc"))
    w.ailSrc = validSource(storage.read("ailSrc"))     -- Querruder-Knueppel: nach der Fusion angehaengt
    w.airSrc = validSource(storage.read("airSrc"))     -- GPS-Logger 3 (1.27): Staudruck, Einheit, Kurs, Satelliten
    v = storage.read("airUnit"); w.airUnit = type(v) == "number" and v or 1
    w.crsSrc = validSource(storage.read("crsSrc"))
    w.satSrc = validSource(storage.read("satSrc"))
    v = storage.read("vtStyle"); w.vtStyle = (type(v) == "number" and v >= 1 and v <= 4) and v or 1  -- 1.28: Klang am Sender
    v = storage.read("vtDev"); w.vtDev = v == 2 and 2 or 1                                              -- 1.29
    v = storage.read("devMain"); w.devMain = (v == 1 or v == 2) and v or 0                             -- 1.30: Geraete, aeltere
    w.devStabi = storage.read("devStabi") == true                                                       -- Konfigurationen: von Hand
    v = storage.read("drawIv"); w.drawIv = (type(v) == "number" and v >= 1 and v <= 4) and v or 1      -- 1.37
    -- 1.38: Am Sender gingen nach jedem Neustart genau die zuletzt angehaengten Werte verloren (Sensor-Geraet,
    -- Varioton-Ausgabe; Christoph 13.11.2026), der Rest blieb: der Speicher eines Widgets fasst offenbar nicht
    -- beliebig viele Eintraege. Darum stehen diese Werte zusaetzlich gepackt im frueher ungenutzten Platz radFix
    -- (weit vorne): 100000 + Klang + 10 * Ausgabe + 100 * Geraet + 1000 * Stabi + 10000 * Bildaufbau.
    local p = w.radFix
    w.calcIv = 1
    if type(p) == "number" and p >= 1000000 and p < 2000000 then    -- 1.39: dazu Rechentakt * 100000
      p = math.floor(p) - 1000000
      local c6 = math.floor(p / 100000) % 10
      w.calcIv = (c6 >= 1 and c6 <= 3) and c6 or 1
      p = p % 100000 + 100000
    end
    if type(p) == "number" and p >= 100000 and p < 200000 then
      p = math.floor(p) - 100000
      local a, b, c, d, e = p % 10, math.floor(p / 10) % 10, math.floor(p / 100) % 10, math.floor(p / 1000) % 10, math.floor(p / 10000) % 10
      w.vtStyle = (a >= 1 and a <= 4) and a or 1
      w.vtDev = b == 2 and 2 or 1
      w.devMain = (c == 1 or c == 2) and c or 0
      w.devStabi = d == 1
      w.drawIv = (e >= 1 and e <= 4) and e or 1
    end
  end
  BAL.SN.load(w)
  w.mcValue = w.mc / 10
  w.tasksT, w.taskSig, w.polarT = nil, nil, nil
end

local function write(w)
  w.radFix = 1000000 + 100000 * (w.calcIv or 1) + (w.vtStyle or 1) + 10 * (w.vtDev or 1) + 100 * (w.devMain or 0) + 1000 * (w.devStabi and 1 or 0) + 10000 * (w.drawIv or 1)
  storage.write("ver", STORE_VER)
  for _, k in ipairs(SOURCE_KEYS) do storage.write(k, BAL.SN.stored(w, k)) end
  for _, d in ipairs(NUM_DEFS) do storage.write(d.key, BAL.SN.stored(w, d.key)) end
  for _, d in ipairs(BOOL_DEFS) do storage.write(d.key, w[d.key]) end
  for _, d in ipairs(TEXT_DEFS) do storage.write(d.key, w[d.key] or "") end
  storage.write("balSrc", w.balSrc)
  storage.write("balMax", w.balMax)
  storage.write("ticker", w.ticker ~= false)
  storage.write("bankSrc", BAL.SN.stored(w, "bankSrc"))
  storage.write("bankInv", w.bankInv == true)
  storage.write("accZSrc", BAL.SN.stored(w, "accZSrc"))
  storage.write("accXSrc", BAL.SN.stored(w, "accXSrc"))
  storage.write("pitchSrc", BAL.SN.stored(w, "pitchSrc"))
  storage.write("ailSrc", w.ailSrc)
  storage.write("airSrc", BAL.SN.stored(w, "airSrc"))
  storage.write("airUnit", BAL.SN.stored(w, "airUnit") or 1)
  storage.write("crsSrc", BAL.SN.stored(w, "crsSrc"))
  storage.write("satSrc", BAL.SN.stored(w, "satSrc"))
  storage.write("vtStyle", w.vtStyle or 1)
  storage.write("vtDev", w.vtDev or 1)
  storage.write("devMain", w.devMain or 0)
  storage.write("devStabi", w.devStabi == true)
  storage.write("drawIv", w.drawIv or 1)
end

local function init()
  -- Ethos akzeptiert Widget-Schluessel nur mit weniger als 8 Zeichen.
  system.registerWidget({
    key = "skynav1", name = "SkyNavigator",
    create = create, paint = function(w) BAL.TM.paint(paint, w) end, wakeup = function(w) BAL.TM.wake(wakeup, w) end, event = eventHandler,
    configure = configure, read = read, write = write,
  })
  if system.registerSource then
    system.registerSource({key = "SNVario", name = "SkyNav Vario", init = BAL.VO.init, wakeup = BAL.VO.wakeup})
  end
end

local M = {init = init}
M._test = {
  buildTask = buildTask, inSector = inSector, penaltyOf = penaltyOf, planPath = planPath, cornersFrom = cornersFrom,
  crossTrack = crossTrack, slotEpoch = slotEpoch, igcLat = igcLat, igcLon = igcLon, create = create, assess = assess,
  scoreOf = scoreOf, startCalc = startCalc, rz = RZ, angDiff = angDiff, bearingOf = bearingOf, toLocal = toLocal,
  slotWindow = slotWindow, refreshTasks = refreshTasks, rctLines = rctLines,
  voInfo = BAL.VO.info, tmInfo = BAL.TM.info,
}
return M
