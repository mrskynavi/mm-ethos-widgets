-- @widget  SkyNavigator
-- @version 1.0
-- SkyNavigator: GPS-Triangle-Navigation fuer Ethos -- Vollbild-Widget mit Tonausgabe.
--
-- Die Aufgaben (Dreiecke) kommen aus dem MM Flight Analyzer, Tab "SkyNavigator": Die App schreibt
-- sie beim Anschliessen des Senders nach scripts/skynav/aufgaben.lua. In der Konfiguration waehlst
-- Du nur noch die Aufgabe. Die Polare fuer den Sollfahrt-Teil kommt wie beim Widget Sollfahrt aus
-- scripts/maccready/polaren.lua (ohne Polare laeuft die Navigation trotzdem).
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
-- auch): START, GLEITEN, THERMIK, AUSWERTUNG. Toene kommen nur fuer die angezeigte Seite.
--
-- Optimaler Weg (Seite GLEITEN): Das Widget rechnet im Luftmassen-System. Dort sind Kurven echte
-- Kreise mit dem aktuellen Wenderadius, die Wendepunkte wandern mit -Wind. Fuer die naechsten drei
-- Ecken liegt ein Kreis (Radius R) so, dass er den Sektor um "Reserve" Meter innen beruehrt. Der
-- Weg = Tangente zum ersten Kreis, Bogen, Aussentangente zum naechsten Kreis ... Die Ankunftszeiten
-- (und damit der Versatz der Ecken) werden dreimal nachgerechnet. Daraus kommen der Soll-Steuerkurs
-- (Kommando "x Grad links/rechts") und die Zeit bis zum Einleiten der Kurve (Piepse davor).
--
-- IGC: Das Widget schreibt ab dem Scharfschalten bzw. Abheben (mit 90 s Vorlauf) eine IGC-Datei nach
-- scripts/skynav/fluege/ (1 Hz, I-Record SUS, LTSK, C-Records, E-Records PEV/STA/TPC/FIN/STP,
-- G-Record). Die App holt sie beim naechsten Sync ins Archiv und wertet sie aus.

local VERSION = "1.0"
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

local SAMPLE_DT, WINDOW, MIN_SAMPLES, FIT_INTERVAL = 0.7, 40, 10, 1.0
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

-- Stuetzwert (Bodenvektor) ins Fenster, Fit jede Sekunde. Wind = Vektor, wohin die Luft zieht (m/s).
local function windStep(w, ve, vn, now)
  local win = w.win
  if ve and math.sqrt(ve * ve + vn * vn) >= 2 / KMH and (w.sT == nil or now - w.sT >= SAMPLE_DT) then
    w.sT = now
    win[#win + 1] = {x = ve, y = vn}
    if #win > WINDOW then table.remove(win, 1) end
  end
  local need = w.windOk and MIN_SAMPLES or WINDOW
  if #win >= need and (w.fitT == nil or now - w.fitT >= FIT_INTERVAL) then
    w.fitT = now
    if courseSpread(win) < MAX_SPREAD_R then
      local cx, cy, r, res = kasaFit(win)
      if cx and fitOk(cx, cy, r, res, win) then
        if w.windOk then
          w.wx, w.wy = w.wx + 0.3 * (cx - w.wx), w.wy + 0.3 * (cy - w.wy)
        else
          w.wx, w.wy, w.windOk = cx, cy, true
        end
        w.windT = now
      end
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
        t = (z.t == "R") and "R" or "C", name = tostring(z.name or "Zone"), x = zx, y = zy,
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
local function planPath(px, py, wx, wy, V, R, C, hdgNow)
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
  local res
  for _ = 1, 6 do
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
    local function addPt(ax, ay, t) res.pts[#res.pts + 1] = {x = ax + wx * t, y = ay + wy * t} end
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
        late = tl > 1 and (tx * hx + ty * hy) / tl < 0.87             -- mehr als ~30 Grad abdrehen noetig
      end
      if late then
        -- geradeaus weiter (ueber Grund mit Wind), bis der Punkt margin tief im Sektor liegt, dann wenden
        local c1 = C[1]
        local gvx, gvy = V * hx + wx, V * hy + wy
        local gv = math.sqrt(gvx * gvx + gvy * gvy)
        if gv > 1 then
          for Lg = 0, 300, 2 do
            local t = Lg / gv
            local gx, gy = px + gvx * t, py + gvy * t
            local rx, ry = gx - c1.tx, gy - c1.ty
            local rl = math.sqrt(rx * rx + ry * ry)
            local dep = rx * c1.bx + ry * c1.by
            if rl > 0.1 and dep >= c1.margin and dep / rl >= 0.7071 then
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
              break
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
      -- Bogen bis zum Zielpunkt (fuer die Zeichnung in Schritten)
      local steps = math.max(1, math.floor(arcA * R / 8))
      for j = 1, steps do
        local th = thIn + s * arcA * j / steps
        addPt(Z[k].x + R * math.cos(th), Z[k].y + R * math.sin(th), (L + arcA * R * j / steps) / V)
      end
      L = L + arcA * R
      ts[k] = L / V
      if k == 1 then res.t1 = ts[1] end
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
      local stepsB = math.max(1, math.floor(arcB * R / 8))
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

-- ---------------------------------------------------------------- Ton (nur fuer die angezeigte Seite)

-- page = Seite, zu der der Ton gehoert (nil = Ereignis, siehe w.eventsAlways)
local function audible(w, page)
  if not w.sound then return false end
  if validSource(w.soundSw) and (readNumber(w.soundSw) or 0) <= 0 then return false end
  if not widgetVisible() then return false end
  if page == nil then return true end            -- Ereignis: Aufrufer hat die Seite schon geprueft
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

local function sayNumber(w, page, v, unit, dec)
  if not audible(w, page) then return end
  if system and system.playNumber and unit ~= nil then pcall(system.playNumber, v, unit, dec or 0) end
end

-- Optionale Sprachdateien (links.wav, rechts.wav, ...) im Ordner audio des Widgets
local audioCache = {}
local function audioFile(name)
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

local function updateGpsClock(w, now)
  local v = gpsSecs(w)
  if not v or v <= 0 then return end
  local radio = (os and os.time) and os.time() or 0
  local guess = radio - (w.utcOffset or 0) * 60
  local utc
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
  local t = (os and os.time) and os.time() or 0
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

local function igcAdd(w, line)
  local L = w.igc
  if not L then return end
  line = line .. "\r\n"
  L.buf[#L.buf + 1] = line
  L.h1 = hashStr(L.h1, line, 31.0, 2147483647.0)
  L.h2 = hashStr(L.h2, line, 131.0, 2147483629.0)
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

local function igcFlush(w)
  local L = w.igc
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
  local r = function(v) return math.floor(v + 0.5) end
  return {"V:02.0", string.format("T:%s,%.6f,%.6f,%d,%d,%d,%d,%d,%d,%d", (T.name:gsub(",", " ")), T.lat, T.lon,
    r(T.elev), r(T.dir), r(T.dist), r(T.maxH), r(T.maxV), r(T.minF), r(T.work))}
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
  w.igc = {path = path, buf = {}, h1 = 7.0, h2 = 13.0, opened = false, lastFlush = f.clk}
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
end

local function igcEvent(w, code, f)
  if not w.igc then return end
  igcAdd(w, "E" .. hms(f.utc) .. code)
  igcAdd(w, bRecord(w, f))
end

local function igcClose(w)
  local L = w.igc
  if not L then return end
  igcAdd(w, string.format("G%08X%08X", math.floor(L.h1), math.floor(L.h2)))
  igcFlush(w)
  w.lastIgc = L.path
  w.igc = nil
end

-- ---------------------------------------------------------------- Zustand

local SOURCE_KEYS = {"gps", "gpsSpd", "gpsAlt", "relAlt", "vario", "swOver", "swStart", "swGlide", "swTherm", "swEval", "armSw", "soundSw", "mcSrc", "gpsTime"}
local NUM_DEFS = {
  {key = "taskIdx",   def = 0},     -- 0 = in der App als aktiv markierte Aufgabe
  {key = "altMode",   def = 1},     -- 1 = GPS minus Platzhoehe, 2 = relative Hoehe (Sensor)
  {key = "varioType", def = 1},     -- 1 = TE, 2 = barometrisch
  {key = "radMode",   def = 1},     -- ungenutzt (Wenderadius wird pro Modell gelernt), Platz fuer die Storage-Reihenfolge
  {key = "radFix",    def = 30},    -- ungenutzt
  {key = "margin",    def = 5},     -- m Reserve im Sektor (Logger 12 Hz: 5 m reichen fuer sichere Treffer)
  {key = "warnSec",   def = 3},     -- Piepse vor dem Wenden (s)
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
}
local BOOL_DEFS = {
  {key = "sound", def = true},
  {key = "eventsAlways", def = false},
  {key = "stfTone", def = true},
  {key = "navVoice", def = true},
  {key = "varioTone", def = false},   -- Vario-Ton macht normalerweise der Sender selbst
  {key = "startTone", def = true},
  {key = "autoPage", def = true},
}
local TEXT_DEFS = {
  {key = "compId", def = ""},
  {key = "gliderId", def = ""},
}

-- Einstellungen aus der App (aufgaben.lua, Tabelle settings): Schluessel, Bereich. Am Sender stehen nur noch
-- Aufgabe, Sensoren, Schalter und das Startfenster von Hand.
local APP_SETTINGS = {
  sound = true, eventsAlways = true, startTone = true, stfTone = true, navVoice = true, varioTone = true, autoPage = true,
  winLo = {2, 50}, speedMargin = {0, 20}, cmdMin = {3, 45}, cmdGap = {3, 60}, startToneLogic = {1, 2}, startSpan = {100, 2000},
  margin = {0, 100}, warnSec = {0, 10}, lead = {0, 30}, zoomSec = {3, 60}, mc = {0, 50}, mcMax = {10, 100},
  ballast = {50, 300}, bugs = {0, 30}, dead = {0, 20}, compId = "", gliderId = "",
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
  w.entry, w.penalty, w.zoneHit, w.altHit = nil, 0, false, false
  w.lastFix, w.lastInStart, w.stopReason = nil, nil, nil
  w.trail, w.ring = {}, {}
  w.track, w.marks = {}, {}
  w.ref, w.tpT, w.tpPrev = nil, nil, nil
  w.therm = nil
  w.evalScroll = 0
  w.msg, w.msgT = nil, nil
end

local function create()
  local w = {win = {}, lastInvalidate = 0, page = P_OVER, mcValue = 1.0}
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

local function refreshTasks(w, now, force)
  if not force and w.tasksT and now - w.tasksT < 20 then return end
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
  w.tasks = list
  local idx = w.taskIdx or 0
  if idx == 0 then
    idx = 1
    for i, T in ipairs(list) do if T.id == w.activeId then idx = i end end
  end
  idx = clamp(idx, 1, math.max(#list, 1))
  local T = list[idx]
  local sig = T and (T.id .. T.lat .. T.lon .. T.dir .. T.dist .. tostring(T.slot and T.slot.h)) or "-"
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
        w.radii[k] = m
      end
    end
  end
end

local function saveRadii(w)
  local keys = {}
  for k in pairs(w.radii) do keys[#keys + 1] = k end
  table.sort(keys)
  local out = {"-- Wenderadien pro Modell (m). Das Widget lernt sie selbst aus den Wenden an den Ecken.\nreturn {\n"}
  for _, k in ipairs(keys) do
    local m, parts = w.radii[k], {}
    for _, key in ipairs({"all", "tp1", "tp2", "tp3"}) do
      if m[key] then parts[#parts + 1] = string.format("%s = %.1f", key, m[key]) end
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
  w.polar = e and polarFromAbc(e.a, e.b, e.c, e.vMin, e.vMax, w.ballast / 100, w.bugs / 100) or nil
end

-- Startfenster in Senderuhr-Zeit: von Hand (Konfiguration) oder aus der Aufgabe (UTC)
local function slotWindow(w)
  if w.slotH >= 0 then
    local now = localNow(w)
    local t0 = now - now % 86400 + w.slotH * 3600 + w.slotM * 60 - (w.utcOffset or 0) * 60
    return t0, t0 + w.slotLen * 60, "hand"
  end
  local T = w.task
  if not T or not T.slot then return nil end
  local a, b = slotEpoch(T.slot)
  if not a then return nil end
  return a, b, "aufgabe"
end

-- ---------------------------------------------------------------- Messwerte

-- Liefert eine neue Messung (fix) oder nil, wenn kein frischer GPS-Fix kam.
local function readInputs(w, now, dt)
  local lat, lon = readPosition(w.gps)
  local vario = readNumber(w.vario)
  w.varioRaw = vario
  if vario ~= nil then w.vFast = lpf(w.vFast, vario, dt, 0.6); w.vSlow2 = lpf(w.vSlow2, vario, dt, 2) end
  if lat == nil then
    w.gpsOk = w.gSeen ~= nil and now - w.gSeen <= 3
    return nil
  end
  -- neuer Fix: Position geaendert, oder unveraendert seit 1 s (Stillstand am Boden)
  local fresh = (lat ~= w.lastLat or lon ~= w.lastLon) or (w.gSeen ~= nil and now - w.gSeen >= 1.0)
  if not fresh then
    w.gpsOk = w.gSeen ~= nil and now - w.gSeen <= 3
    return nil
  end
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
  if w.ax == nil then w.ax, w.ay, w.aT = x, y, now end
  local ddx, ddy = x - w.ax, y - w.ay
  local dd = math.sqrt(ddx * ddx + ddy * ddy)
  local gsPos
  if dd >= 3 then
    w.crs = bearingOf(ddx, ddy)
    gsPos = dd / math.max(now - w.aT, 0.05)
    w.ax, w.ay, w.aT = x, y, now
  end
  local gs = speedMs(w.gpsSpd)
  if gs == nil then
    if gsPos then w.gsPos = lpf(w.gsPos, gsPos, dtFix, 1.5) end
    gs = w.gsPos or 0
  end
  local ve, vn = nil, nil
  if w.crs then local sx, sy = dirVec(w.crs); ve, vn = gs * sx, gs * sy end
  if ve then windStep(w, ve, vn, now) end
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
  -- Drehrate -> Wenderadius
  if hdg and w.hdgPrev and dtFix > 0.05 then
    local omega = math.rad(math.abs(angDiff(hdg, w.hdgPrev))) / dtFix
    w.omega = lpf(w.omega, omega, dtFix, 1.5)
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
  local p = planPath(f.x, f.y, w.windOk and w.wx or 0, w.windOk and w.wy or 0, w.tasF or 20, radius(w), C)
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
  w.trail[#w.trail + 1] = {x = f.x, y = f.y, v = w.vFast or f.vz, t = now}
  while #w.trail > 0 and now - w.trail[1].t > 70 do table.remove(w.trail, 1) end
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
        if w.autoPage then w.autoTarget = P_GLIDE end
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
    local x = clamp(math.abs(e) - half, 0, 60)
    tone(w, P_START, math.max(250, 450 - x * 3), 220)
    w.startBeepT = now + math.max(0.3, 0.6 - x * 0.004)
  end
end

-- ---------------------------------------------------------------- Seite GLEITEN: Sollfahrt + Weg

local function glideCalc(w, f, now, dt)
  local T = w.task
  local g = {}
  -- Sollfahrt wie Widget Sollfahrt (gleiche Glaettung wie dessen Standard: Vario und Fahrt 2 s,
  -- Netto 8 s), damit beide Widgets bei gleichem MC und Wind dieselbe Sollfahrt zeigen
  if w.polar and f and f.tas and f.tas * KMH > 15 and w.varioRaw ~= nil then
    local p = w.polar
    w.teN = lpf(w.teN, w.varioRaw, dt, 2)
    w.tasN = lpf(w.tasN, f.tas, dt, 2)
    w.netto = lpf(w.netto, w.teN + polarSink(p, w.tasN), dt, 8)
    local head = nil
    if w.windOk and f.hdg then local hx, hy = dirVec(f.hdg); head = -(w.wx * hx + w.wy * hy) end
    w.vstf = speedToFly(p, w.mcValue, clamp(w.netto, -5, 5), head)
    w.stfDv = ((w.tasF or f.tas) - w.vstf) * KMH
  elseif not w.polar then
    w.vstf, w.stfDv = nil, nil
  end
  -- mittleres Sinken im Geradeausflug (Ersatz, wenn keine Polare da ist)
  if f and f.vz and not w.therm and (w.omega or 0) < math.rad(8) then w.sinkAvg = lpf(w.sinkAvg, -f.vz, dt, 20) end
  if not T or not f or w.state ~= S_STARTED then return g end
  local R = radius(w, w.tp + 1)
  local C = cornersFrom(T, w.tp + 1, 3, radiusFn(w), w.margin)
  local wx, wy = w.windOk and w.wx or 0, w.windOk and w.wy or 0
  local plan = planPath(f.x, f.y, wx, wy, w.tasF or f.tas or 20, R, C, f.hdg)
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
    local pr = planPath(f.x, f.y, wx, wy, V, R, cornersFrom(T, w.tp + 1, nRem, radiusFn(w), w.margin), f.hdg)
    tRem = (pr.tAll or 0) + T.dist / gsU
  else
    tRem = math.max(-(f.x * T.ux + f.y * T.uy), 0) / gsU
  end
  local sink = w.polar and polarSink(w.polar, V) or w.sinkAvg
  if sink then g.hFin, g.tFin = f.agl - sink * tRem, tRem end
  -- Eckansicht: zoomSec Sekunden vor der naechsten Ecke (Ankunft nach Plan). Sobald der Sektor erreicht ist,
  -- zaehlt die naechste Ecke, die Seite ist sofort wieder Gleiten.
  local gs = math.max(f.gs or V, 5)
  g.tCorner = plan.t1 or (g.dV / gs)
  if g.tCorner < w.zoomSec then g.corner = g.next end
  return g
end

local function glideTones(w, g, now)
  if not g.plan then
    -- vor dem Start: nur Sollfahrt
  else
    -- Vorwarnung vor dem Wenden: Piepse bei warnSec..1 s, langer Ton bei 0 (mit Vorhalt)
    local tT = (g.plan.tT or 99) - w.lead / 10
    local slot = math.ceil(tT)
    if not g.plan.inTurn and tT <= w.warnSec + 0.05 and tT > -0.5 then
      if slot ~= w.warnSlot then
        w.warnSlot = slot
        if slot <= 0 then tone(w, P_GLIDE, 1600, 450) else tone(w, P_GLIDE, 1250, 80) end
      end
      w.quietUntil = now + 4
      return
    end
    if tT > w.warnSec + 1 then w.warnSlot = nil end
    -- Kurs-Kommandos: "x Grad links/rechts"
    -- keine Kommandos in der Kurve und kurz nach einem Wendepunkt (dort zeigt die Eckansicht den Weg)
    local turning = (w.omega or 0) > math.rad(8) or (w.tpT and now - w.tpT < 6)
    if w.navVoice and g.err and not turning and not g.plan.inTurn and (g.plan.tT or 0) > 6 and (not w.quietUntil or now > w.quietUntil) then
      local a = math.abs(g.err)
      if a >= w.cmdMin then
        w.errSince = w.errSince or now
        local changed = not w.cmdErr or math.abs(g.err - w.cmdErr) >= 10
        if now - w.errSince >= 2 and (not w.cmdT or now - w.cmdT >= w.cmdGap or (changed and now - w.cmdT >= 3)) then
          w.cmdT, w.cmdErr = now, g.err
          local deg = math.floor(a / 5 + 0.5) * 5
          sayNumber(w, P_GLIDE, deg, UNIT_DEGREE, 0)
          local right = g.err > 0
          if not sayWord(w, P_GLIDE, right and "rechts.wav" or "links.wav") then
            -- Ersatz ohne Sprachdatei: zwei Toene, steigend = rechts, fallend = links
            w.cmdTones = {t = now + 0.9, f = right and {650, 950} or {950, 650}}
          end
          w.quietUntil = now + 2.5
          return
        end
      else
        w.errSince = nil
      end
    end
  end
  if w.cmdTones and now >= w.cmdTones.t then
    local ft = w.cmdTones.f
    if ft[1] then tone(w, P_GLIDE, table.remove(ft, 1), 110); w.cmdTones.t = now + 0.16 end
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
        tone(w, P_GLIDE, 850 + excess * 20, clamp(period * 550, 70, 140))
        w.stfBeepT = now + period
      else
        local period = math.max(0.30, 0.50 - excess * 0.01)
        tone(w, P_GLIDE, math.max(220, 400 - excess * 8), clamp(period * 600, 180, 300))
        w.stfBeepT = now + period
      end
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
  -- Kreisen erkennen: Drehrate > 12 Grad/s ueber 8 s
  local circ = (w.omega or 0) > math.rad(12)
  if circ then w.circSince = w.circSince or now; w.straightSince = nil
  else w.straightSince = w.straightSince or now end
  if w.circSince and now - w.circSince > 8 and not w.therm and f then
    w.therm = {t0 = now, h0 = f.agl, hMax = f.agl}
    if w.autoPage and w.state == S_STARTED then w.autoTarget = P_THERM end
  end
  if w.therm and w.straightSince and now - w.straightSince > 8 then
    w.lastTherm = w.therm
    w.therm, w.circSince = nil, nil
    -- abgetrieben: Querablage ab hier neu messen (Linie vom Ausstieg zur naechsten Ecke)
    if w.state == S_STARTED and f then setRef(w, f) end
    if w.autoPage and w.state == S_STARTED then w.autoTarget = P_GLIDE end
  end
  if w.therm and f then
    w.therm.hMax = math.max(w.therm.hMax, f.agl)
    w.therm.h = f.agl
  end
end

local function thermTones(w, now)
  if not w.varioTone or w.vFast == nil then return end
  if w.varTT and now < w.varTT then return end
  local v = w.vFast
  if v > 0.2 then
    local f = 600 + clamp(v, 0, 6) * 160
    local period = clamp(0.55 - v * 0.07, 0.12, 0.55)
    tone(w, P_THERM, f, period * 450)
    w.varTT = now + period
  elseif v < -1.5 then
    tone(w, P_THERM, 300, 300)
    w.varTT = now + 0.9
  else
    w.varTT = now + 0.2
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
      if on and not w.swState[p] then w.swPage, w.manual = p, nil end
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
  end
  if w.manual then
    if w.autoTarget and w.autoTarget ~= w.manualFrom then w.manual = nil else return w.manual end
  end
  if w.autoPage and w.autoTarget then return w.autoTarget end
  return w.page
end

-- ---------------------------------------------------------------- wakeup

-- Scharf stellen oder, nach einem misslungenen Start (gestartet, noch keine Runde fertig), neu starten
local function armOrRestart(w, now)
  if not w.task then return end
  if w.state == S_IDLE then
    w.state, w.armedAt, w.armPending = S_ARMED, now, true
  elseif w.state == S_STARTED and #w.laps == 0 then
    w.state, w.armedAt, w.armPending = S_ARMED, now, true
    w.startT, w.startUtc, w.lapT, w.entry, w.penalty, w.tp, w.lapAgl = nil, nil, nil, nil, 0, 0, nil
    w.zoneHit, w.altHit = nil, nil
    w.restarts = (w.restarts or 0) + 1
    note(w, "Neustart: wieder scharf", now)
  elseif w.state == S_STARTED then
    note(w, "Neustart nur vor der ersten Runde", now)
  end
end

local function wakeup(w)
  local now = os.clock()
  local dt = w.lastT and (now - w.lastT) or 0
  w.lastT = now
  if dt > 0.5 then dt = 0.5 end
  refreshTasks(w, now, false)
  refreshPolar(w, now, false)
  if not validSource(w.mcSrc) then w.mcValue = w.mc / 10 else
    local v = readNumber(w.mcSrc)
    if v then w.mc = math.floor(clamp((v + 1024) / 2048, 0, 1) * w.mcMax + 0.5); w.mcValue = w.mc / 10 end
  end
  -- ARM
  -- Jede Betaetigung (Flanke) stellt scharf, nach einem misslungenen Start startet sie neu. Taster und
  -- Kippschalter gehen beide; zurueckschalten tut nichts.
  local arm = (readNumber(w.armSw) or -1) > 0
  if arm and not w.armOn and w.armInit then armOrRestart(w, now) end
  w.armOn, w.armInit = arm, true
  if not w.gpsClkPoll or now - w.gpsClkPoll >= 1 then w.gpsClkPoll = now; updateGpsClock(w, now) end

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

  w.page = pageFromSwitch(w, now)
  w.sc = startCalc(w, cur)
  w.gc = glideCalc(w, cur, now, dt)
  thermCalc(w, cur, now, dt)

  -- Ereignistoene (nur auf der passenden Seite, ausser "Ereignisse immer melden")
  if w.lastEvent and w.lastEventT == now then
    local e = w.lastEvent
    local pg = (e == "STA" or e == "PEV") and P_START or P_GLIDE
    if w.eventsAlways or w.page == pg or (e == "STA" and w.page == P_GLIDE) then
      if e == "PEV" then tone(w, nil, 800, 120); tone(w, nil, 1000, 120)
      elseif e == "STA" then tone(w, nil, 1000, 150); tone(w, nil, 1400, 250)
        if w.penalty > 0 then sayNumber(w, nil, w.penalty, UNIT_RAW, 0) end
      elseif e == "TPC" then tone(w, nil, 1500, 90); tone(w, nil, 1500, 90); tone(w, nil, 1800, 150)
      elseif e == "FIN" then tone(w, nil, 1200, 150); tone(w, nil, 1600, 300); sayNumber(w, nil, #w.laps, UNIT_RAW, 0)
      elseif e == "STP" then tone(w, nil, 600, 400) end
      w.quietUntil = now + 1.5
    end
  end
  if w.zoneEnterT == now then tone(w, nil, 500, 600) end

  if cur then
    if w.page == P_START then startTones(w, w.sc, now)
    elseif w.page == P_GLIDE then glideTones(w, w.gc, now)
    elseif w.page == P_THERM then thermTones(w, now) end
  end

  if now - w.lastInvalidate >= 0.15 then
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
local function line(x1, y1, x2, y2)
  if CLIP then
    local dx, dy = x2 - x1, y2 - y1
    local t0, t1 = 0, 1
    local p = {-dx, dx, -dy, dy}
    local q = {x1 - CLIP[1], CLIP[3] - x1, y1 - CLIP[2], CLIP[4] - y1}
    for k = 1, 4 do
      if p[k] == 0 then
        if q[k] < 0 then return end
      else
        local r = q[k] / p[k]
        if p[k] < 0 then if r > t1 then return end; if r > t0 then t0 = r end
        else if r < t0 then return end; if r < t1 then t1 = r end end
      end
    end
    x1, y1, x2, y2 = x1 + t0 * dx, y1 + t0 * dy, x1 + t1 * dx, y1 + t1 * dy
  end
  lcd.drawLine(fl(x1), fl(y1), fl(x2), fl(y2))
end
local function inClip(x, y) return not CLIP or (x >= CLIP[1] and x <= CLIP[3] and y >= CLIP[2] and y <= CLIP[4]) end
local function rect(x, y, w, h) lcd.drawFilledRectangle(fl(x), fl(y), math.max(fl(w), 1), math.max(fl(h), 1)) end
local function disc(x, y, r) lcd.drawFilledCircle(fl(x), fl(y), fl(r)) end
local function ring(x, y, r) lcd.drawCircle(fl(x), fl(y), fl(r)) end
local function tri(x1, y1, x2, y2, x3, y3) lcd.drawFilledTriangle(fl(x1), fl(y1), fl(x2), fl(y2), fl(x3), fl(y3)) end

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
local function predFlat(P, f, color)
  if not (f and f.ve and f.vn) then return end
  local ax, ay = P(f.x, f.y)
  local bx, by = P(f.x + f.ve * PRED_S, f.y + f.vn * PRED_S)
  lcd.color(color)
  line(ax, ay, bx, by)
  line(ax + 1, ay, bx + 1, by)
  for k = 1, PRED_S - 1 do
    local kx, ky = ax + (bx - ax) * k / PRED_S, ay + (by - ay) * k / PRED_S
    rect(math.floor(kx) - 1, math.floor(ky) - 1, 3, 3)
  end
  disc(bx, by, 5)
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
  if not w.gpsOk then st, col = st .. "  GPS?", C_RED end
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
  clock = clock .. (gpsClock(w) and "" or "*")
  drawT(W * 0.4, 2, W * 0.6 - 6, st .. "  " .. clock, FONT_S, col, "r")
  -- Restzeit prominent links
  local rest, running = restTime(w)
  if rest then
    local rc = not running and C_DIM or (rest < 120 and C_RED or (rest < 300 and C_ORANGE or C_WHITE))
    if rest < 0 then rest = 0 end
    drawT(6, 1, W * 0.5, "REST " .. fmtMS(rest), FONT_STD, rc, "l")
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

-- Schnittmenge von Halbebenen a*X + b*Y + k >= 0 (Bildschirm) im Kasten fuellen, Farbe vorher setzen
function fillHP(HP, x0, y0, bw, bh)
  for yr = math.floor(y0), math.floor(y0 + bh - 1), 2 do
    local lo, hi = x0, x0 + bw - 1
    for _, h in ipairs(HP) do
      local rhs = -(h[2] * yr + h[3])
      if math.abs(h[1]) < 1e-9 then
        if rhs > 0 then lo = hi + 1 end
      elseif h[1] > 0 then lo = math.max(lo, rhs / h[1])
      else hi = math.min(hi, rhs / h[1]) end
    end
    if hi > lo then rect(math.floor(lo), yr, math.floor(hi - lo) + 1, 2) end
  end
end

-- Seite UEBERSICHT: ganzes Dreieck mit Flugweg, flach wie im Reglement (Hypotenuse unten, TP3 links, TP1 rechts,
-- TP2 oben, Linie senkrecht durch den Ursprung). Vor dem Start ist die
-- TP1-Seite der Linie rot, danach ist der Sektor des naechsten TP gefuellt wie in der Eckansicht.
-- Optimale Runde vor dem Start (Ursprung -> TP1 -> TP2 -> TP3 -> Ursprung), alle 2 s neu gerechnet
-- Ideale Runde mit Wind, unabhaengig von der Position des Modells: geplant ab dem Ursprung ueber
-- TP1, TP2, TP3 und nochmals TP1; gezeichnet ab dem Kurvenbeginn an TP1, also ohne den Anflug vom Ursprung.
-- Das Stueck von TP3 zu TP1 zeigt, wo man ideal durch die Startlinie fliegt.
local function lapRoute(w, T)
  local nowc = os.clock()
  if not w.ovPlan or nowc - (w.ovPlanT or 0) > 2 then
    local pr = planPath(0, 0, w.windOk and w.wx or 0, w.windOk and w.wy or 0, w.tasF or 20, radius(w, 1), cornersFrom(T, 1, 4, radiusFn(w), w.margin))
    local pts = {}
    for k = 2, #(pr.pts or {}) do pts[#pts + 1] = pr.pts[k] end
    w.ovPlan, w.ovPlanT = pts, nowc
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
  -- Ausschnitt: Dreieck, Linie im Bereich des Dreiecks und das Modell
  local pts = {{0, 0}, {T.tp[1].x, T.tp[1].y}, {T.tp[2].x, T.tp[2].y}, {T.tp[3].x, T.tp[3].y}}
  for i = 1, 3 do local c = T.tp[i]; pts[#pts + 1] = {c.x + c.bx * T.dist * 0.25, c.y + c.by * T.dist * 0.25} end
  if f then pts[#pts + 1] = {f.x, f.y} end
  local x1, y1, x2, y2 = math.huge, math.huge, -math.huge, -math.huge
  for _, q in ipairs(pts) do
    local rx, ry = R(q[1], q[2])
    x1, y1, x2, y2 = math.min(x1, rx), math.min(y1, ry), math.max(x2, rx), math.max(y2, ry)
  end
  local pad = 18
  local sc = math.min((mw - 2 * pad) / math.max(x2 - x1, 50), (mh - 2 * pad) / math.max(y2 - y1, 50))
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
    for k = 2, #route do
      local ax, ay = P(route[k - 1].x, route[k - 1].y)
      local bx, by = P(route[k].x, route[k].y)
      line(ax, ay, bx, by)
    end
  end
  -- Flugweg: vor dem Start grau, in der Aufgabe weiss
  local tr = w.track or {}
  for k = 2, #tr do
    local a, b = tr[k - 1], tr[k]
    lcd.color(b.on and C_WHITE or rgb(100, 100, 100))
    local ax, ay = P(a.x, a.y)
    local bx, by = P(b.x, b.y)
    line(ax, ay, bx, by)
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
    predFlat(function(x, yv) local qx, qy = P(x, yv); return qx + px - fx, qy + py - fy end, f, C_ORANGE)
    planeFlat(T, px, py, f.crs, 8, C_WHITE)
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
    local d = T.dist * scale
    lcd.color(C_GRID)
    line(cx - d, cy, cx + d, cy)
    line(cx - d, cy, cx, cy - d)
    line(cx, cy - d, cx + d, cy)
    -- optimale Runde (blau): zeigt vor dem Start, wo man von TP3 her durch die Linie zu TP1 fliegt
    local route = before and lapRoute(w, T) or (w.gc and w.gc.plan and w.gc.plan.pts)
    if route then
      lcd.color(C_BLUE)
      for k = 2, #route do
        local ax, ay = flatXY(T, route[k - 1].x, route[k - 1].y)
        local bx, by = flatXY(T, route[k].x, route[k].y)
        line(cx + ax * scale, cy + ay * scale, cx + bx * scale, cy + by * scale)
        line(cx + ax * scale, cy + ay * scale + 1, cx + bx * scale, cy + by * scale + 1)
      end
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
  if started then
    local sTxt = string.format("Start: %d m %d km/h", math.floor(w.entry.agl + 0.5), math.floor(w.entry.v3d + 0.5))
    if (w.penalty or 0) > 0 then sTxt = sTxt .. string.format("  Strafe %d", w.penalty) end
    drawT(rx, yy, rw, sTxt, FONT_XS, (w.penalty or 0) > 0 and C_RED or C_GREEN, "l")
    yy = yy + textH(FONT_XS) + 2
  end
  local canRestart = w.state == S_STARTED and #w.laps == 0
  if w.state == S_IDLE or w.state == S_STOPPED or canRestart then
    w.armBox = {x = rx, y = yy, w = rw, h = textH(FONT_STD) + 8}
    lcd.color(rgb(70, 50, 0))
    rect(rx, yy, rw, w.armBox.h)
    drawT(rx, yy + 4, rw, canRestart and "NEUSTART (tippen)" or "SCHARF (tippen)", FONT_STD, C_ORANGE, "c")
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
    local pts = g.plan.pts
    for k = 2, #pts do
      local ax, ay = P(pts[k - 1].x, pts[k - 1].y)
      local bx, by = P(pts[k].x, pts[k].y)
      line(ax, ay, bx, by)
    end
  end
  -- Spur (gefaerbt nach Vario)
  for _, p in ipairs(w.trail) do
    local ax, ay = P(p.x, p.y)
    if inClip(ax, ay) then
      lcd.color((p.v or 0) > 0.5 and C_YELLOW or C_WHITE)
      rect(math.floor(ax) - 1, math.floor(ay) - 1, 2, 2)
    end
  end
  local vx, vy = P(c.x, c.y)
  lcd.color(C_YELLOW)
  if inClip(vx, vy) then disc(vx, vy, 4) end
  local ax, ay = P(f.x, f.y)
  if inClip(ax, ay) then predFlat(P, f, C_ORANGE); planeFlat(T, ax, ay, f.crs, 8, C_WHITE)
  else planeFlat(T, clamp(ax, x0 + 8, x0 + bw - 8), clamp(ay, y0 + 8, y0 + bh - 8), f.crs, 8, C_WHITE) end
  CLIP = nil
  -- Wind: Pfeil (wohin er blaest), Staerke daneben
  windFlat(w, T, x0 + bw - 24, y0 + 24, x0)
  drawT(x0 + 4, y0 + 2, bw, "TP" .. i .. (done and "  INNEN" or string.format("  %d m", math.floor(d + 0.5))), FONT_STD, done and C_GREEN or C_WHITE, "l")
  drawT(x0, y0 + bh - textH(FONT_XS) - 2, bw - 4, string.format("%d m Raster", math.floor(span)), FONT_XS, C_DIM, "r")
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

local function paintGlide(w, W, H, top)
  local T, f, g = w.task, w.cur, w.gc or {}
  local m = 6
  -- Links: Sollfahrt
  local lw = math.floor(W * 0.24)
  local y = top
  if w.polar then
    local barW = 26
    stfBar(w, m, y, barW, H - y - m)
    local tx = m + barW + 6
    local tw = lw - barW - 6
    local yy = y
    yy = yy + valBox(tx, yy, tw, "SOLL", w.vstf and string.format("%d", math.floor(w.vstf * KMH + 0.5)) or "--", C_WHITE, F_L)
    yy = yy + valBox(tx, yy, tw, "IST", w.tasF and string.format("%d", math.floor(w.tasF * KMH + 0.5)) or "--",
      (w.stfDv and math.abs(w.stfDv) <= w.dead) and C_GREEN or C_ORANGE, F_L)
    yy = yy + valBox(tx, yy, tw, "MC", string.format("%.1f", w.mcValue), C_WHITE, F_L)
    w.mcBox = {x = tx, y = yy - textH(F_L), w = tw, h = textH(F_L)}
    valBox(tx, yy, tw, "NETTO", w.netto and string.format("%+.1f", w.netto) or "--", (w.netto or 0) >= 0.1 and C_GREEN or ((w.netto or 0) <= -0.1 and C_RED or C_WHITE), FONT_STD)
  else
    drawT(m, y, lw, "Polare fehlt", FONT_S, C_ORANGE, "l")
    drawT(m, y + textH(FONT_S), lw, "(Sollfahrt aus)", FONT_XS, C_DIM, "l")
    valBox(m, y + 2 * textH(FONT_S) + 4, lw, "FAHRT", w.tasF and string.format("%d", math.floor(w.tasF * KMH + 0.5)) or "--", C_WHITE, F_L)
  end
  -- Rechts: Navigation
  local nx = lw + m * 2
  local nw = W - nx - m
  if not T then drawT(nx, y, nw, "Keine Aufgabe", F_L, C_RED, "c"); return end
  if w.state ~= S_STARTED or not g.plan or not f then
    drawT(nx, y + 10, nw, w.state == S_STOPPED and "Aufgabe beendet" or "Noch nicht gestartet", F_L, C_DIM, "c")
    if f then
      local dx, dy = -f.x, -f.y
      drawT(nx, y + 10 + textH(F_L) + 6, nw, string.format("Ursprung %d m, %03d Grad", math.floor(math.sqrt(dx * dx + dy * dy) + 0.5), math.floor(bearingOf(dx, dy) + 0.5)), FONT_STD, C_WHITE, "c")
    end
    return
  end
  local infoH = textH(FONT_S) + 2
  if g.corner then
    paintCorner(w, g, nx, y, nw, H - y - m - infoH)
  else
    -- Kommando gross
    local err = g.err or 0
    local a = math.abs(err)
    local cmd, col
    if g.plan.inTurn then cmd, col = "KURVE", C_BLUE
    elseif a < 5 then cmd, col = "KURS OK", C_GREEN
    elseif err > 0 then cmd, col = string.format("%d° RECHTS >", math.floor(a + 0.5)), a >= w.cmdMin and C_ORANGE or C_WHITE
    else cmd, col = string.format("< %d° LINKS", math.floor(a + 0.5)), a >= w.cmdMin and C_ORANGE or C_WHITE end
    local th = drawT(nx, y, nw, cmd, F_XL, col, "c")
    local yy = y + th + 4
    -- Kursgrafik: links Steuerpfeil, rechts Querablage zur Soll-Linie (blau) mit Flugzeug und oben einem
    -- Balken fuer die erwartete Ablage in PRED_S Sekunden (jetziger Kurs ueber Grund).
    local cdiH = SMALL and 38 or 78
    local sq = cdiH
    lcd.color(rgb(25, 25, 25))
    rect(nx, yy, sq, cdiH)
    do
      -- Steuerpfeil: Soll-Kurs relativ zur Nase, oben = geradeaus
      local acx, acy = nx + sq / 2, yy + cdiH / 2
      local ang = math.rad(err)
      local L = sq / 2 - 5
      local ux, uy = math.sin(ang), -math.cos(ang)
      lcd.color(a < 5 and C_GREEN or (a >= w.cmdMin and C_ORANGE or C_WHITE))
      arrow(acx - ux * L * 0.75, acy - uy * L * 0.75, ux, uy, L * 1.75, math.max(4, sq / 9), math.max(7, sq / 5))
    end
    local cx0, cw = nx + sq + 4, nw - sq - 4
    lcd.color(rgb(25, 25, 25))
    rect(cx0, yy, cw, cdiH)
    local xt = g.xt or 0
    local far = math.abs(xt) > 300
    local ccx = cx0 + cw / 2
    if far then
      -- weit daneben (z. B. abgetrieben): kein Massstab mehr, Peilung direkt zur naechsten Ecke
      drawT(cx0, yy + 2, cw, string.format("WEIT DANEBEN  %d m %s", math.floor(math.abs(xt) + 0.5), xt > 0 and "rechts" or "links"), FONT_S, C_ORANGE, "c")
      drawT(cx0, yy + 2 + textH(FONT_S), cw, string.format("TP%d direkt %03d°  %.1f km", g.next, math.floor(g.brgTP + 0.5), g.dV / 1000), SMALL and FONT_STD or FONT_L, C_WHITE, "c")
    else
      local xp = g.xtPred or xt
      local rng = (math.max(math.abs(xt), math.abs(xp)) > 100) and 300 or 100
      local half = cw / 2 - 14
      lcd.color(C_GRID)
      for k = -2, 2 do local tx = ccx + k * half / 2; line(tx, yy + 4, tx, yy + cdiH - 4) end
      lcd.color(C_BLUE)
      line(ccx, yy, ccx, yy + cdiH)
      line(ccx + 1, yy, ccx + 1, yy + cdiH)
      local ppx = ccx + clamp(xt / rng, -1, 1) * half
      local qx = ccx + clamp(xp / rng, -1, 1) * half
      local pcol = math.abs(xp) < 20 and C_GREEN or C_ORANGE
      -- Balken oben: erwartete Ablage in PRED_S s, von der Soll-Linie bis dorthin
      local bh = SMALL and 6 or 10
      local by = yy + 4
      lcd.color(pcol)
      rect(math.min(ccx, qx), by, math.max(math.abs(qx - ccx), 2), bh)
      -- Flugzeug wie bisher bei der aktuellen Ablage
      planeSymbol(ppx, yy + cdiH * 0.62, SMALL and 9 or 13, math.abs(xt) < 20 and C_GREEN or C_WHITE)
      local pTxt = string.format("in %d s: %d m %s", PRED_S, math.floor(math.abs(xp) + 0.5), math.abs(xp) < 3 and "auf Linie" or (xp > 0 and "rechts" or "links"))
      local lx = (qx > ccx) and (cx0 + 4) or (ccx + 8)
      drawT(lx, by + bh + 2, cw / 2 - 12, pTxt, FONT_S, pcol, "l")
      drawT(cx0 + 4, yy + cdiH - textH(FONT_XS) - 1, cw, string.format("+-%d m", rng), FONT_XS, C_DIM, "l")
    end
    -- Seitenwind nur klein unten rechts
    if g.cross and math.abs(g.cross) >= 2 then
      drawT(cx0, yy + cdiH - textH(FONT_XS) - 1, cw - 4, string.format("Seitenwind %d von %s", math.floor(math.abs(g.cross) + 0.5), g.cross > 0 and "links" or "rechts"), FONT_XS, C_DIM, "r")
    end
    yy = yy + cdiH + 2
    if not far and not SMALL then
      drawT(nx, yy, nw, string.format("Ablage %d m %s", math.floor(math.abs(xt) + 0.5), xt > 0 and "rechts" or "links"), FONT_XS, C_DIM, "c")
      yy = yy + textH(FONT_XS) + 4
    else yy = yy + 2 end
    local col3 = math.floor(nw / 3)
    valBox(nx, yy, col3, "TP" .. g.next, string.format("%d m", math.floor(g.dV + 0.5)), C_WHITE, F_L)
    valBox(nx + col3, yy, col3, "WENDEN IN", fmtMS(g.plan.tT), (g.plan.tT or 99) < 10 and C_ORANGE or C_WHITE, F_L)
    local hf = g.hFin
    local hfCol = hf and (hf < 30 and C_RED or (hf < 80 and C_ORANGE or C_WHITE)) or C_DIM
    if SMALL then
      -- kleiner Bildschirm: Zielhoehe statt Radius, Gegen-/Rueckenwind in der Infozeile
      yy = yy + valBox(nx + 2 * col3, yy, col3, "ZIEL-HÖHE", hf and string.format("%d m", math.floor(hf + 0.5)) or "--", hfCol, F_L) + 4
    else
      local hw = g.head
      local hwTxt = hw and string.format("%d km/h", math.floor(math.abs(hw) + 0.5)) or "--"
      yy = yy + valBox(nx + 2 * col3, yy, col3, hw and (hw >= 0 and "GEGENWIND" or "RÜCKENWIND") or "WIND", hwTxt,
        hw and (hw >= 0 and C_ORANGE or C_GREEN) or C_DIM, F_L) + 4
    end
    local lapNow = (f.clk or 0) - (w.lapT or f.clk)
    if not SMALL and yy + textH(FONT_XS) + textH(FONT_STD) < H - m - infoH then
      valBox(nx, yy, col3, "HÖHE AM ZIEL", hf and string.format("%d m", math.floor(hf + 0.5)) or "--", hfCol, FONT_STD)
      valBox(nx + col3, yy, col3, "RUNDE " .. (#w.laps + 1), fmtMS(lapNow), C_WHITE, FONT_STD)
      valBox(nx + 2 * col3, yy, col3, "RADIUS", string.format("%d m", math.floor(g.R + 0.5)), (w.radii and w.radii[w.modelName or "?"]) and C_WHITE or C_DIM, FONT_STD)
    end
  end
  -- Infozeile unten
  local sc = scoreOf(w)
  local wind = w.windOk and string.format("Wind %d aus %03d", math.floor(math.sqrt(w.wx ^ 2 + w.wy ^ 2) * KMH + 0.5), math.floor(bearingOf(-w.wx, -w.wy) + 0.5)) or "Wind: kreisen"
  local last = w.laps[#w.laps]
  local lastTxt = last and string.format("  Letzte %.1f km/h%s", last.speed, last.idx and string.format(" %.1f%%", last.idx) or "") or ""
  if SMALL and g.head then
    local hw = g.head
    drawT(nx, H - m - infoH + 2, nw, string.format("%s %d", hw >= 0 and "GEGENWIND" or "RÜCKENWIND", math.floor(math.abs(hw) + 0.5)), FONT_STD, hw >= 0 and C_ORANGE or C_GREEN, "l")
    drawT(nx, H - m - infoH + 2, nw, string.format("Dreiecke %d", sc.n), FONT_S, C_DIM, "r")
    return
  end
  drawT(nx, H - m - infoH + 2, nw, string.format("Dreiecke %d%s  %s", sc.n, lastTxt, wind), FONT_S, C_DIM, "c")
end

local function paintTherm(w, W, H, top)
  local T, f = w.task, w.cur
  local m = 6
  local y = top
  local lw = math.floor(W * 0.36)
  local v = w.vFast
  local th = drawT(m, y, lw, "VARIO", FONT_XS, C_DIM, "l")
  th = th + drawT(m, y + th, lw, v and string.format("%+.1f", v) or "--", F_XL, (v or 0) >= 0 and C_GREEN or C_RED, "l")
  local yy = y + th + 4
  local avg = 0
  if w.avg30 and #w.avg30 > 0 then for _, a in ipairs(w.avg30) do avg = avg + a end; avg = avg / #w.avg30 end
  local half = math.floor(lw / 2)
  valBox(m, yy, half, "Ø 30 s", string.format("%+.1f", avg), avg >= 0 and C_GREEN or C_RED, F_L)
  local tm = w.therm
  local thAvg, gain = nil, nil
  if tm and f then
    local dtm = os.clock() - tm.t0
    gain = f.agl - tm.h0
    if dtm > 5 then thAvg = gain / dtm end
  end
  yy = yy + valBox(m + half, yy, half, "Ø THERMIK", thAvg and string.format("%+.1f", thAvg) or "--", C_WHITE, F_L) + 4
  valBox(m, yy, half, "GEWINN", gain and string.format("%+d m", math.floor(gain + 0.5)) or "--", C_WHITE, F_L)
  yy = yy + valBox(m + half, yy, half, "HÖHE", f and string.format("%d m", math.floor(f.agl + 0.5)) or "--", C_WHITE, F_L) + 4
  if T and f then
    local rest = T.maxAlt - f.agl
    valBox(m, yy, half, SMALL and "RESERVE" or ("RESERVE ZU " .. T.maxAlt), string.format("%d m", math.floor(rest + 0.5)), rest < 30 and C_RED or (rest < 80 and C_ORANGE or C_WHITE), FONT_STD)
    local left = w.startT and (T.work * 60 - (f.clk - w.startT)) or nil
    valBox(m + half, yy, half, "RESTZEIT", left and fmtMS(left) or "--", C_WHITE, FONT_STD)
  end
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
    -- Ausschnitt: ganzes Dreieck (Massstab bleibt beim Kreisen gleich). Liegt das Modell
    -- ausserhalb, rutscht das Dreieck ueber den Rand, das Modell bleibt im Bild.
    local ox, oy, sc = f.x, f.y, math.min(mw, mh) / 2 / clamp(R * 2.2, 50, 300)
    if T then
      ox = (T.tp[1].x + T.tp[2].x + T.tp[3].x) / 3
      oy = (T.tp[1].y + T.tp[2].y + T.tp[3].y) / 3
      local span = 0
      for i = 1, 3 do span = math.max(span, math.sqrt((T.tp[i].x - ox) ^ 2 + (T.tp[i].y - oy) ^ 2)) end
      sc = math.min(mw, mh) / 2 / (span * 1.1 + 20)
      -- weit weg: so weit herauszoomen, dass vom Dreieck noch ein Stueck im Bild bleibt
      local dNear = math.huge
      for i = 1, 3 do
        local a, b = T.tp[i], T.tp[i % 3 + 1]
        local vx, vy = b.x - a.x, b.y - a.y
        local u = clamp(((f.x - a.x) * vx + (f.y - a.y) * vy) / (vx * vx + vy * vy), 0, 1)
        dNear = math.min(dNear, math.sqrt((a.x + u * vx - f.x) ^ 2 + (a.y + u * vy - f.y) ^ 2))
      end
      if dNear > 1 then sc = math.min(sc, 0.8 * math.min(mw, mh) / dNear) end
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
    -- Spur im Luftmassen-System (um den Wind zurueckversetzt), dann liegen die Kreise uebereinander
    local best, bv, bx0, by0 = nil, -99, 0, 0
    local nowc = os.clock()
    local wx, wy = w.windOk and w.wx or 0, w.windOk and w.wy or 0
    for _, p in ipairs(w.trail) do
      if nowc - p.t < 35 then
        local age = nowc - p.t
        local qx, qy = p.x + wx * age, p.y + wy * age
        local ax, ay = P(qx, qy)
        local pv = p.v or 0
        if pv > bv then bv, best, bx0, by0 = pv, p, qx, qy end
        local col = pv > 1.5 and C_RED or (pv > 0.5 and C_YELLOW or (pv > 0 and C_GREEN or C_BLUE))
        lcd.color(col)
        if inClip(ax, ay) then rect(ax - 2, ay - 2, 4, 4) end
      end
    end
    local ax0, ay0 = P(f.x, f.y)
    if best and bv > 0.3 then
      local bx, by = P(bx0, by0)
      lcd.color(C_RED)
      if inClip(bx, by) then ring(bx, by, 7) end
      line(ax0, ay0, bx, by)
    end
    CLIP = nil
    if T then planeFlat(T, ax0, ay0, f.crs, 8, C_WHITE) else planeSymbol(ax0, ay0, 8, C_WHITE) end
    if T and w.state == S_STARTED then
      local nxt = T.tp[ni]
      local tx, ty = P(nxt.x, nxt.y)
      if not (tx >= mx0 and tx <= mx0 + mw - 1 and ty >= y and ty <= y + mh - 1) then
        -- naechster TP ausserhalb: gruener Punkt am Rand in seiner Richtung
        local ex, ey = tx - (mx0 + mw / 2), ty - (y + mh / 2)
        local n = math.max(math.sqrt(ex * ex + ey * ey), 1e-3)
        ex, ey = ex / n, ey / n
        local k = math.min((mw / 2 - 8) / math.max(math.abs(ex), 1e-3), (mh / 2 - 8) / math.max(math.abs(ey), 1e-3))
        lcd.color(C_GREEN)
        disc(math.floor(mx0 + mw / 2 + ex * k), math.floor(y + mh / 2 + ey * k), 5)
      end
      drawT(mx0 + 4, y + 2, mw, string.format("TP%d %d m", ni, math.floor(math.sqrt((nxt.x - f.x) ^ 2 + (nxt.y - f.y) ^ 2) + 0.5)), FONT_S, C_GREEN, "l")
    end
    local wind = w.windOk and string.format("Wind %d km/h aus %03d", math.floor(math.sqrt(w.wx ^ 2 + w.wy ^ 2) * KMH + 0.5), math.floor(bearingOf(-w.wx, -w.wy) + 0.5)) or "Wind: noch kein Fit"
    drawT(mx0, y + mh - textH(FONT_XS) - 2, mw - 4, wind .. string.format("  R %d m", math.floor(R + 0.5)), FONT_XS, C_DIM, "r")
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
  w.armBox, w.resetBox, w.mcBox = nil, nil, nil
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
  if EVT_TOUCH and category == EVT_TOUCH and (TOUCH_START == nil or value == TOUCH_START) then
    if inside(w.hdrBox, x, y) then
      local tabW = w.hdrBox.w / #PAGE_NAMES
      local p = clamp(math.floor(x / tabW) + 1, 1, #PAGE_NAMES)
      w.manual, w.manualFrom, w.page = p, w.autoTarget, p
      lcd.invalidate(); return true
    end
    if inside(w.armBox, x, y) and w.task then
      if w.state == S_STOPPED then resetFlight(w) end
      armOrRestart(w, os.clock())
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
      w.mc = clamp(w.mc + (up and 1 or -1), 0, w.mcMax); w.mcValue = w.mc / 10; lcd.invalidate(); return true
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
    for i, T in ipairs(w.tasks) do ch[#ch + 1] = {string.format("%s (%s %dm)", T.name, T.cls, math.floor(T.dist + 0.5)), i} end
    form.addChoiceField(l, nil, ch, function() return clamp(w.taskIdx, 0, #w.tasks) end, function(v)
      w.taskIdx = v; w.taskSig = nil; refreshTasks(w, os.clock(), true) end)
  end
  -- Sensoren
  addSource(w, "GPS", "gps")
  addSource(w, "GPS-Geschw. (empfohlen)", "gpsSpd")
  addSource(w, "GPS-Höhe (ü. M.)", "gpsAlt")
  addSource(w, "GPS-Zeit (sonst Senderuhr, Uhr mit *)", "gpsTime")
  addSource(w, "Relative Höhe (optional)", "relAlt")
  addChoice(w, "Höhe für Wertung", "altMode", {{"GPS minus Platzhöhe", 1}, {"Relative Höhe", 2}})
  addSource(w, "Vario", "vario")
  -- Seiten
  addSwitch(w, "Schalter Seite Übersicht", "swOver")
  addSwitch(w, "Schalter Seite Start", "swStart")
  addSwitch(w, "Schalter Seite Gleiten", "swGlide")
  addSwitch(w, "Schalter Seite Thermik", "swTherm")
  addSwitch(w, "Schalter Seite Auswertung", "swEval")
  addSwitch(w, "Scharf-Schalter (ARM)", "armSw")
  addSwitch(w, "Ton-Schalter (optional)", "soundSw")
  addSource(w, "MC Poti (optional)", "mcSrc")
  -- Zeit
  addNumber(w, "Startfenster von Hand: Stunde (-1 = Aufgabe)", "slotH", -1, 23)
  addNumber(w, "Startfenster von Hand: Minute", "slotM", 0, 59)
  addNumber(w, "Startfenster von Hand: Länge (min)", "slotLen", 1, 120)
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
end

local function read(w)
  if storage.read("ver") ~= STORE_VER then setDefaults(w)
  else
    for _, k in ipairs(SOURCE_KEYS) do w[k] = validSource(storage.read(k)) end
    for _, d in ipairs(NUM_DEFS) do local v = storage.read(d.key); w[d.key] = type(v) == "number" and v or d.def end
    for _, d in ipairs(BOOL_DEFS) do local v = storage.read(d.key); if type(v) == "boolean" then w[d.key] = v else w[d.key] = d.def end end
    for _, d in ipairs(TEXT_DEFS) do local v = storage.read(d.key); w[d.key] = type(v) == "string" and v or d.def end
  end
  w.mcValue = w.mc / 10
  w.tasksT, w.taskSig, w.polarT = nil, nil, nil
end

local function write(w)
  storage.write("ver", STORE_VER)
  for _, k in ipairs(SOURCE_KEYS) do storage.write(k, w[k]) end
  for _, d in ipairs(NUM_DEFS) do storage.write(d.key, w[d.key]) end
  for _, d in ipairs(BOOL_DEFS) do storage.write(d.key, w[d.key]) end
  for _, d in ipairs(TEXT_DEFS) do storage.write(d.key, w[d.key] or "") end
end

local function init()
  -- Ethos akzeptiert Widget-Schluessel nur mit weniger als 8 Zeichen.
  system.registerWidget({
    key = "skynav1", name = "SkyNavigator",
    create = create, paint = paint, wakeup = wakeup, event = eventHandler,
    configure = configure, read = read, write = write,
  })
end

local M = {init = init}
M._test = {
  buildTask = buildTask, inSector = inSector, penaltyOf = penaltyOf, planPath = planPath, cornersFrom = cornersFrom,
  crossTrack = crossTrack, slotEpoch = slotEpoch, igcLat = igcLat, igcLon = igcLon, create = create, assess = assess,
  scoreOf = scoreOf, startCalc = startCalc, angDiff = angDiff, bearingOf = bearingOf, toLocal = toLocal,
}
return M
