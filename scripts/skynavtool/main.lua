-- @widget  SkyNav Aufgabe
-- @version 1.1
-- Systemtool "SkyNav Aufgabe" (SYS -> Werkzeuge): Aufgaben fuer das Widget SkyNavigator am Sender erfassen,
-- ohne PC. Der MM Flight Analyzer (Tab SkyNavigator, auch die Web-App) liest den QR-Code der Aufgabe und zeigt sie als
-- Buchstaben-Code; hier wird er abgetippt. Vor jedem Durchgang stellst Du hier das Startfenster ein.
--
-- Code: 23 Buchstaben ohne I, O, Q. Jede Zeile = bis 15 Datenzeichen + 3 Pruefzeichen (Reed-Solomon ueber GF(23)):
-- ein falscher Buchstabe pro Zeile wird korrigiert, zwei werden erkannt. Werte exakt wie im .rct (Koordinaten auf
-- 1e-6 Grad, Richtung auf 0.1 Grad, sonst ganzzahlig). Gleiche Rechnung in der App: index.html, <script id="skynavCode">.
--   Aufgabe:  lat+90e6 (28 Bit), lon+180e6 (29), Platzhoehe+1000 (14), Richtung*10 (12), dTP (10), Einflughoehe (10),
--             Einfluggeschw. (10), Mindestspeed (10), Arbeitszeit (7), Anzahl Zonen (3)            -> 36 Buchstaben
--   Kreis:    Nr (3), 0 (1), dLat+131072 (18), dLon+131072 (18), Radius (14), unten (14), oben (14) -> 25 Buchstaben
--   Rechteck: Nr, 1, dLat, dLon, Richtung*10 (12), Laenge, Breite, unten, oben (je 14)            -> 30 Buchstaben
--   Aenderung: Pruefsumme der Ausgangsaufgabe % 4096 (12), Maske (5: Ort, Hoehe, Richtung, dTP, Limits), Werte
-- Pruefwort: CRC-32 der Werte (ganzzahlig, siehe canonStr) als 4 Buchstaben. Gleich in App und Sender = gleiche Aufgabe.
--
-- Dateien: liest scripts/skynav/aufgaben.lua (von der App) und schreibt scripts/skynav/feld.lua (Aufgaben vom Sender,
-- aktive Aufgabe, Startfenster in Senderzeit). Das Widget SkyNavigator liest beide.

local VERSION = "1.1"
local FELD_PATHS = {"/scripts/skynav/feld.lua", "SCRIPTS:/skynav/feld.lua"}
local TASK_PATHS = {"/scripts/skynav/aufgaben.lua", "SCRIPTS:/skynav/aufgaben.lua"}
local CLASSES = {
  Light = {dist = 200, maxH = 250, maxV = 70, minF = 10, work = 20, lenMin = 4},
  Sport = {dist = 350, maxH = 400, maxV = 120, minF = 20, work = 30, lenMin = 5},
  Scale = {dist = 500, maxH = 500, maxV = 120, minF = 20, work = 30, lenMin = 11},
}

-- ---------------------------------------------------------------- Code (Reed-Solomon, Basis 23)

local ALPHA_S = "ABCDEFGHJKLMNPRSTUVWXYZ"
local P, ALPHA, NC, MAXD = 23, 5, 3, 15
local IDX = {}
for i = 1, #ALPHA_S do IDX[ALPHA_S:sub(i, i)] = i - 1 end

local function pw(a, e)
  local r = 1
  a = a % P
  while e > 0 do
    if e % 2 == 1 then r = r * a % P end
    a = a * a % P
    e = e // 2
  end
  return r
end
local function inv(a) return pw(a, P - 2) end

local GEN = {1}
for i = 1, NC do
  local r = (P - pw(ALPHA, i)) % P
  local n = {}
  for j = 1, #GEN + 1 do n[j] = 0 end
  for j = 1, #GEN do
    n[j] = (n[j] + GEN[j]) % P
    n[j + 1] = (n[j + 1] + GEN[j] * r) % P
  end
  GEN = n
end

local function rsEnc(msg)
  local k = #msg
  local r = {}
  for i = 1, k do r[i] = msg[i] end
  for i = k + 1, k + NC do r[i] = 0 end
  for i = 1, k do
    local c = r[i]
    if c ~= 0 then
      for j = 2, #GEN do r[i + j - 1] = (r[i + j - 1] - c * GEN[j]) % P end
    end
  end
  local out = {}
  for i = 1, k do out[i] = msg[i] end
  for i = k + 1, k + NC do out[i] = (P - r[i]) % P end
  return out
end

local function evalAt(cw, x)
  local v = 0
  for i = 1, #cw do v = (v * x + cw[i]) % P end
  return v
end

-- -> data, fixedPos (0-basiert, -1 = keiner), korrigierter Wert; nil bei zwei oder mehr Fehlern
local function rsDec(cw)
  local n = #cw
  local s1, s2, s3 = evalAt(cw, pw(ALPHA, 1)), evalAt(cw, pw(ALPHA, 2)), evalAt(cw, pw(ALPHA, 3))
  local data = {}
  if s1 == 0 and s2 == 0 and s3 == 0 then
    for i = 1, n - NC do data[i] = cw[i] end
    return data, -1
  end
  if s1 == 0 or s2 == 0 then return nil end
  local X = s2 * inv(s1) % P
  local e = s1 * inv(X) % P
  if s3 ~= e * pw(X, 3) % P then return nil end
  for pos = 0, n - 1 do
    if pw(ALPHA, n - 1 - pos) == X then
      local c = {}
      for i = 1, n do c[i] = cw[i] end
      c[pos + 1] = (c[pos + 1] - e) % P
      for i = 1, n - NC do data[i] = c[i] end
      return data, pos, c[pos + 1]
    end
  end
  return nil
end

-- Ziffern zur Basis 23 (hoechste zuerst) <-> Bitfelder. Ethos-Lua rechnet mit 32-Bit-Ganzzahlen: darum jedes Feld
-- in Stuecke von hoechstens 8 Bit zerlegen, so bleiben alle Zwischenwerte klein (unter 23 * 256).
local function chunks(w)
  local out, first = {}, w % 8
  if first == 0 then first = 8 end
  out[1] = math.min(first, w)
  local rest = w - out[1]
  while rest > 0 do out[#out + 1] = 8; rest = rest - 8 end
  return out
end

local function pack(fields)
  local d = {0}
  for _, f in ipairs(fields) do
    local v, w = f[1], f[2]
    local left = w
    for _, c in ipairs(chunks(w)) do
      left = left - c
      local carry, mul = (v >> left) & ((1 << c) - 1), 1 << c
      for i = #d, 1, -1 do
        local x = d[i] * mul + carry
        d[i] = x % P
        carry = x // P
      end
      while carry > 0 do
        table.insert(d, 1, carry % P)
        carry = carry // P
      end
    end
  end
  return d
end

local function unpack(d0, widths)
  local d = {}
  for i = 1, #d0 do d[i] = d0[i] end
  local cw = {}
  for _, w in ipairs(widths) do for _, c in ipairs(chunks(w)) do cw[#cw + 1] = c end end
  local parts = {}
  for k = #cw, 1, -1 do
    local div = 1 << cw[k]
    local rem = 0
    for i = 1, #d do
      local x = rem * P + d[i]
      d[i] = x // div
      rem = x % div
    end
    parts[k] = rem
  end
  for i = 1, #d do if d[i] ~= 0 then return nil end end
  local out, at = {}, 1
  for n, w in ipairs(widths) do
    local v = 0
    for _, c in ipairs(chunks(w)) do v = (v << c) | parts[at]; at = at + 1 end
    out[n] = v
  end
  return out
end

local LOG2P = math.log(P) / math.log(2)
local function bitsToSyms(b) return math.ceil(b / LOG2P - 1e-9) end
local function sum(t) local s = 0; for _, v in ipairs(t) do s = s + v end; return s end
local function codeLen(widths)
  local k = bitsToSyms(sum(widths))
  return k + NC * math.ceil(k / MAXD), k
end

local function encode(fields)
  local w = {}
  for i, f in ipairs(fields) do w[i] = f[2] end
  local _, k = codeLen(w)
  local d = pack(fields)
  while #d < k do table.insert(d, 1, 0) end
  local b = math.ceil(k / MAXD)
  local lines, at = {}, 0
  for i = 0, b - 1 do
    local n = math.ceil((k - at) / (b - i))
    local msg = {}
    for j = 1, n do msg[j] = d[at + j] end
    local cw = rsEnc(msg)
    local s = {}
    for j = 1, #cw do s[j] = ALPHA_S:sub(cw[j] + 1, cw[j] + 1) end
    lines[#lines + 1] = table.concat(s)
    at = at + n
  end
  return lines
end

local function clean(s) return (tostring(s or ""):upper():gsub("[^A-Z]", "")) end

-- -> vals, fixed ({line, pos, was, now}) oder nil, Fehlertext, Zeile
local function decode(text, widths)
  local s = clean(text)
  for i = 1, #s do
    if not IDX[s:sub(i, i)] then return nil, "Buchstabe " .. s:sub(i, i) .. " kommt im Code nicht vor" end
  end
  local L, k = codeLen(widths)
  if #s < L then return nil, string.format("zu kurz: %d von %d", #s, L) end
  if #s > L then return nil, string.format("zu lang: %d statt %d", #s, L) end
  local b = math.ceil(k / MAXD)
  local data, fixed, at, dat = {}, {}, 0, 0
  for i = 0, b - 1 do
    local n = math.ceil((k - dat) / (b - i))
    local cw = {}
    for j = 1, n + NC do cw[j] = IDX[s:sub(at + j, at + j)] end
    local dd, pos, val = rsDec(cw)
    if not dd then return nil, string.format("Tippfehler bei Zeichen %d-%d", at + 1, at + n + NC), i + 1 end
    if pos >= 0 then
      fixed[#fixed + 1] = {line = i + 1, pos = at + pos + 1, was = s:sub(at + pos + 1, at + pos + 1), now = ALPHA_S:sub(val + 1, val + 1)}
    end
    for j = 1, #dd do data[#data + 1] = dd[j] end
    at, dat = at + n + NC, dat + n
  end
  local vals = unpack(data, widths)
  if not vals then return nil, "Code passt nicht (falsche Art von Code?)" end
  return vals, fixed
end

local T_W = {28, 29, 14, 12, 10, 10, 10, 10, 7, 3}
local ZC_W = {3, 1, 18, 18, 14, 14, 14}
local ZR_W = {3, 1, 18, 18, 12, 14, 14, 14, 14}
local OFF = 131072
local CH_PARTS = {{28, 29}, {14}, {12}, {10}, {10, 10, 10, 7}}

local function rnd(v) return math.floor(v + 0.5) end
local function i6(v) return rnd((tonumber(v) or 0) * 1e6) end
-- Koordinate als ganze Millionstel: aus dem Code (lat6/lon6) exakt, sonst aus der Kommazahl
local function c6(t, k) return math.tointeger(t[k .. "6"]) or i6(t[k]) end
local function i10(v) return rnd(((tonumber(v) or 0) % 360) * 10) end

-- ganzzahlige Werte wie im Code; gleiche Reihenfolge wie canon() in der App
local function canon(t)
  local T = {c6(t, "lat"), c6(t, "lon"), rnd(t.elev or 0), i10(t.dir), rnd(t.dist or 0), rnd(t.maxH or 0), rnd(t.maxV or 0), rnd(t.minF or 0), rnd(t.work or 0)}
  local Z = {}
  for i, z in ipairs(t.zones or {}) do
    if i > 5 then break end
    if z.t == "R" then
      Z[i] = {"R", c6(z, "lat"), c6(z, "lon"), i10(z.dir), rnd(z.len or 0), rnd(z.wid or 0), rnd(z.lo or 0), rnd(z.hi or 0)}
    else
      Z[i] = {"C", c6(z, "lat"), c6(z, "lon"), rnd(z.r or 0), rnd(z.lo or 0), rnd(z.hi or 0)}
    end
  end
  return T, Z
end

local function joinInts(t, from)
  local s = {}
  for i = from or 1, #t do s[#s + 1] = type(t[i]) == "string" and t[i] or string.format("%d", t[i]) end
  return table.concat(s, ",")
end

local function canonStr(t)
  local T, Z = canon(t)
  local s = joinInts(T)
  for _, z in ipairs(Z) do s = s .. ";" .. joinInts(z) end
  return s
end

local CRC_T
local function crc32(str)
  if not CRC_T then
    CRC_T = {}
    for i = 0, 255 do
      local c = i
      for _ = 1, 8 do
        if c & 1 == 1 then c = (c >> 1) ~ 0xEDB88320 else c = c >> 1 end
      end
      CRC_T[i] = c
    end
  end
  local c = 0xFFFFFFFF
  for i = 1, #str do c = (c >> 8) ~ CRC_T[(c ~ str:byte(i)) & 0xFF] end
  return (~c) & 0xFFFFFFFF
end

local function crcOf(t) return crc32(canonStr(t)) end
-- CRC als vorzeichenlose 32-Bit-Zahl modulo m, auch wenn Lua nur 32-Bit-Ganzzahlen mit Vorzeichen hat
local function umod(n, m)
  local r = 0
  for b = 31, 0, -1 do r = (r * 2 + ((n >> b) & 1)) % m end
  return r
end
local function word(t)
  local n = umod(crcOf(t), 279841)
  local s = ""
  for _ = 1, 4 do s = ALPHA_S:sub(n % P + 1, n % P + 1) .. s; n = n // P end
  return s
end

local function decodeTask(text)
  local v, fx = decode(text, T_W)
  if not v then return nil, fx end
  return {lat6 = v[1] - 90000000, lon6 = v[2] - 180000000, lat = (v[1] - 90000000) / 1e6, lon = (v[2] - 180000000) / 1e6, elev = v[3] - 1000, dir = v[4] / 10,
    dist = v[5], maxH = v[6], maxV = v[7], minF = v[8], work = v[9]}, fx, v[10]
end

local function decodeZone(text, task)
  local s = clean(text)
  local isR = #s == 30
  if #s ~= 25 and #s ~= 30 then return nil, string.format("%d Buchstaben: Kreis 25, Rechteck 30", #s) end
  local v, fx = decode(s, isR and ZR_W or ZC_W)
  if not v then return nil, fx end
  if v[2] ~= (isR and 1 or 0) then return nil, "Art der Zone passt nicht" end
  local lat6 = c6(task, "lat") + v[3] - OFF
  local lon6 = c6(task, "lon") + v[4] - OFF
  local lat, lon = lat6 / 1e6, lon6 / 1e6
  local z
  if isR then z = {t = "R", lat6 = lat6, lon6 = lon6, lat = lat, lon = lon, dir = v[5] / 10, len = v[6], wid = v[7], lo = v[8], hi = v[9]}
  else z = {t = "C", lat6 = lat6, lon6 = lon6, lat = lat, lon = lon, r = v[5], lo = v[6], hi = v[7]} end
  return z, fx, v[1]
end

local function copyTask(t)
  local c = {}
  for k, v in pairs(t) do c[k] = v end
  return c
end

-- Aenderungs-Code bezogen auf base (die Aufgabe vom Morgen) -> neue Aufgabe, Maske, fixed
local function decodeChange(text, base)
  local s = clean(text)
  local tooShort = true
  for mask = 1, 31 do
    local w = {12, 5}
    for bit = 0, 4 do
      if mask & (1 << bit) ~= 0 then for _, x in ipairs(CH_PARTS[bit + 1]) do w[#w + 1] = x end end
    end
    local L = codeLen(w)
    if L >= #s then tooShort = tooShort and L > #s end
    if L == #s then
      local v, fx = decode(s, w)
      if not v and fx and fx:find("Tippfehler") then return nil, fx end
      if v and v[2] == mask then
        if v[1] ~= crcOf(base) & 4095 then return nil, "Code gehoert zu einer anderen Aufgabe" end
        local t = copyTask(base)
        local at = 3
        if mask & 1 ~= 0 then
          t.lat6, t.lon6 = v[at] - 90000000, v[at + 1] - 180000000
          t.lat, t.lon = t.lat6 / 1e6, t.lon6 / 1e6
          at = at + 2
        end
        if mask & 2 ~= 0 then t.elev = v[at] - 1000; at = at + 1 end
        if mask & 4 ~= 0 then t.dir = v[at] / 10; at = at + 1 end
        if mask & 8 ~= 0 then t.dist = v[at]; at = at + 1 end
        if mask & 16 ~= 0 then t.maxH, t.maxV, t.minF, t.work = v[at], v[at + 1], v[at + 2], v[at + 3] end
        return t, fx, mask
      end
    end
  end
  return nil, nil   -- noch keine passende Laenge
end

-- ---------------------------------------------------------------- Dateien

local function loadTable(paths)
  for _, path in ipairs(paths) do
    local chunk
    if io and io.open and load then
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

local function ser(v, ind)
  local tv = type(v)
  if tv == "number" then
    if math.type and math.type(v) == "integer" then return string.format("%d", v) end
    if v == math.floor(v) and math.abs(v) < 1e15 then return string.format("%d", math.floor(v)) end
    return string.format("%.7f", v):gsub("0+$", ""):gsub("%.$", "")
  elseif tv == "string" then return string.format("%q", v)
  elseif tv == "boolean" then return tostring(v)
  elseif tv == "table" then
    local out, keys = {}, {}
    for k in pairs(v) do if type(k) == "string" then keys[#keys + 1] = k end end
    table.sort(keys)
    for _, k in ipairs(keys) do out[#out + 1] = k .. " = " .. ser(v[k], ind .. "  ") end
    for _, x in ipairs(v) do out[#out + 1] = ser(x, ind .. "  ") end
    if #out == 0 then return "{}" end
    return "{\n" .. ind .. "  " .. table.concat(out, ",\n" .. ind .. "  ") .. "\n" .. ind .. "}"
  end
  return "nil"
end

local function writeFeld(feld)
  local text = "-- SkyNavigator: Aufgaben und Startfenster vom Systemtool SkyNav Aufgabe (Sender)\nreturn " .. ser(feld, "") .. "\n"
  for _, path in ipairs(FELD_PATHS) do
    local ok, f = pcall(io.open, path, "w")
    if ok and f then
      local okW = pcall(f.write, f, text)
      pcall(f.close, f)
      if okW then return true end
    end
  end
  return false
end

-- ---------------------------------------------------------------- Zustand

local function clsOf(t)
  for name, c in pairs(CLASSES) do if rnd(t.dist or 0) == c.dist then return name end end
  return tostring(t.cls or "")
end

local function fmtDir(d)
  d = tonumber(d) or 0
  if math.abs(d - rnd(d)) < 1e-6 then return string.format("%d°", rnd(d)) end
  return string.format("%.1f°", d)
end

local function taskFromFile(e, src)
  if type(e) ~= "table" or tonumber(e.lat) == nil or tonumber(e.lon) == nil then return nil end
  local t = {}
  for _, k in ipairs({"lat", "lon", "elev", "dir", "dist", "maxH", "maxV", "minF", "work", "maxAlt"}) do t[k] = tonumber(e[k]) end
  t.lat6, t.lon6 = math.tointeger(e.lat6), math.tointeger(e.lon6)
  t.id, t.name, t.cls, t.src = tostring(e.id or e.name or "?"), tostring(e.name or "Aufgabe"), tostring(e.cls or ""), src
  t.zones = {}
  for _, z in ipairs(type(e.zones) == "table" and e.zones or {}) do
    if type(z) == "table" then t.zones[#t.zones + 1] = copyTask(z) end
  end
  if type(e.base) == "table" then t.base = taskFromFile(e.base, "base") end
  t.word = word(t)
  return t
end

-- Senderuhr: os.time() zaehlt die Ortszeit so, als waere sie UTC (wie im Widget), also ohne os.time(Tabelle)
local function localNow() return math.floor(os.time()) end
local function wall(t)
  local ok, d = pcall(os.date, "!*t", math.floor(t))
  if ok and type(d) == "table" then return d end
  return {year = 1970, month = 1, day = 1, hour = 0, min = 0, sec = 0}
end
local function days(yy, mm, dd)
  if mm <= 2 then yy, mm = yy - 1, mm + 12 end
  local era = yy // 400
  local yoe = yy - era * 400
  local doy = (153 * (mm - 3) + 2) // 5 + dd - 1
  return era * 146097 + yoe * 365 + yoe // 4 - yoe // 100 + doy - 719468
end
local function slotEpoch(sl)
  return days(tonumber(sl.y) or 1970, tonumber(sl.mo) or 1, tonumber(sl.d) or 1) * 86400 + (tonumber(sl.h) or 0) * 3600 + (tonumber(sl.mi) or 0) * 60
end

local function loadState(s)
  s.app = loadTable(TASK_PATHS) or {}
  s.feld = loadTable(FELD_PATHS) or {}
  s.tasks = {}
  for _, e in ipairs(type(s.app.tasks) == "table" and s.app.tasks or {}) do
    local t = taskFromFile(e, "app")
    if t then s.tasks[#s.tasks + 1] = t end
  end
  for _, e in ipairs(type(s.feld.tasks) == "table" and s.feld.tasks or {}) do
    local t = taskFromFile(e, "feld")
    if t then s.tasks[#s.tasks + 1] = t end
  end
  local active = s.feld.active or s.app.active
  s.sel = 1
  for i, t in ipairs(s.tasks) do if t.id == tostring(active or "") then s.sel = i end end
  s.lens = type(s.feld.lens) == "table" and s.feld.lens or {}
  -- Beginn: gespeichertes Fenster, solange es nicht vorbei ist, sonst jetzt + 1 Minute
  local now = localNow()
  local sl = s.feld.slot
  local t0
  if type(sl) == "table" and sl.y then
    local e = slotEpoch(sl)
    if e + ((tonumber(sl.len) or 0) + 1) * 60 > now then t0 = e end
  end
  if not t0 then t0 = now - now % 60 + 60 end
  local d = wall(t0)
  s.slotH, s.slotM = d.hour, d.min
  local t = s.tasks[s.sel]
  s.slotLen = (type(sl) == "table" and sl.len) or (t and s.lens[clsOf(t)]) or (t and CLASSES[clsOf(t)] and CLASSES[clsOf(t)].lenMin) or 5
end

-- Beginn als Senderzeit (Sekunden); liegt er mehr als 12 h zurueck, ist der naechste Tag gemeint (z. B. 00:10 um 23:55)
local function slotStart(s)
  local now = localNow()
  local t0 = now - now % 86400 + s.slotH * 3600 + s.slotM * 60
  if t0 < now - 43200 then t0 = t0 + 86400 end
  return t0
end

-- withSlot: auch das Startfenster schreiben (Knopf Uebernehmen); sonst nur Aufgabenliste und aktive Aufgabe
local function save(s, withSlot)
  local t = s.tasks[s.sel]
  local now = localNow()
  local feld = s.feld
  feld.version = 1
  feld.active = t and t.id or nil
  if withSlot then
    local d = wall(slotStart(s) or now)
    feld.slot = {y = d.year, mo = d.month, d = d.day, h = s.slotH, mi = s.slotM, len = s.slotLen}
    if t then s.lens[clsOf(t)] = s.slotLen end
  end
  feld.written = now
  feld.lens = s.lens
  local list = {}
  for _, x in ipairs(s.tasks) do
    if x.src == "feld" then
      local e = {}
      for _, k in ipairs({"id", "name", "cls", "lat", "lon", "lat6", "lon6", "elev", "dir", "dist", "maxH", "maxV", "minF", "work", "maxAlt"}) do e[k] = x[k] end
      e.zones = x.zones
      if x.base then
        local b = {}
        for _, k in ipairs({"lat", "lon", "lat6", "lon6", "elev", "dir", "dist", "maxH", "maxV", "minF", "work"}) do b[k] = x.base[k] end
        b.zones = x.base.zones
        e.base = b
      end
      list[#list + 1] = e
    end
  end
  feld.tasks = list
  return writeFeld(feld)
end
local function saveSlot(s) return save(s, true) end
local function saveTasks(s) return save(s, false) end

-- ---------------------------------------------------------------- Zeichnen

local W, H = 800, 480
local C = {}
local function colors()
  C.bg, C.panel, C.key, C.dim, C.white = lcd.RGB(0, 0, 0), lcd.RGB(30, 30, 30), lcd.RGB(44, 44, 46), lcd.RGB(150, 150, 150), lcd.RGB(255, 255, 255)
  C.green, C.orange, C.red, C.blue, C.head, C.pri = lcd.RGB(52, 199, 89), lcd.RGB(255, 159, 10), lcd.RGB(255, 69, 58), lcd.RGB(90, 200, 250), lcd.RGB(10, 61, 107), lcd.RGB(10, 95, 168)
end

local function fl(v) return math.floor(v + 0.5) end
local function box(x, y, w, h, col)
  lcd.color(col)
  lcd.drawFilledRectangle(fl(x), fl(y), math.max(1, fl(w)), math.max(1, fl(h)))
end
local function frame(x, y, w, h, col)
  lcd.color(col)
  lcd.drawRectangle(fl(x), fl(y), fl(w), fl(h))
  lcd.drawRectangle(fl(x) + 1, fl(y) + 1, fl(w) - 2, fl(h) - 2)
end
local function txtSize(str, font) lcd.font(font); return lcd.getTextSize(str) end
local function text(x, y, w, str, font, col, align)
  lcd.font(font)
  lcd.color(col)
  local tw = lcd.getTextSize(str)
  local px = x
  if align == "c" then px = x + (w - tw) / 2 elseif align == "r" then px = x + w - tw end
  lcd.drawText(fl(px), fl(y), str)
  return tw
end
local function th(font) local _, h = txtSize("0", font); return h end

-- passende Schrift: groesste aus der Liste, in der str in w passt
local function fit(str, w, fonts)
  for _, f in ipairs(fonts) do if txtSize(str, f) <= w then return f end end
  return fonts[#fonts]
end

local function grouped(s)
  local out = {}
  for i = 1, #s, 4 do out[#out + 1] = s:sub(i, i + 3) end
  return table.concat(out, " ")
end

-- Knoepfe: {x, y, w, h, label, fn, style}
local function button(s, x, y, w, h, label, fn, style, fonts)
  local col = style == "pri" and C.pri or (style == "off" and C.panel or C.key)
  box(x, y, w, h, col)
  local f = fit(label, w - 8, fonts or {FONT_STD, FONT_S, FONT_XS})
  if fonts and th(f) > h - 6 then f = fit(label, w - 8, {FONT_STD, FONT_S, FONT_XS}) end
  text(x, y + (h - th(f)) / 2, w, label, f, style == "off" and C.dim or C.white, "c")
  if style ~= "off" then s.hits[#s.hits + 1] = {x = x, y = y, w = w, h = h, fn = fn} end
end

local function hit(s, x, y, w, h, fn) s.hits[#s.hits + 1] = {x = x, y = y, w = w, h = h, fn = fn} end

local function header(s, title, right)
  local hb = th(FONT_STD) + 12
  box(0, 0, W, hb, C.head)
  text(10, 6, W * 0.7, title, FONT_STD, C.white, "l")
  local cw = txtSize("Schliessen", FONT_S) + 20
  button(s, W - cw - 4, 4, cw, hb - 8, right or "Schliessen", function() if s.screen == "main" then system.exit() else s.screen = "main" end end)
  return hb
end

local function hm(h, m) return string.format("%02d:%02d", h, m) end

-- ---------------------------------------------------------------- Seite: Aufgaben + Startfenster

local function slotInfo(s)
  local now = localNow()
  local t0 = slotStart(s)
  if not t0 then return "" end
  local e = t0 + s.slotLen * 60 + 59
  local off = tonumber(s.app.utcOffsetMin) or 0
  local u = wall(t0 - off * 60)
  local pre
  if now < t0 then pre = string.format("öffnet in %d:%02d", (t0 - now) // 60, (t0 - now) % 60)
  elseif now <= e then pre = "offen"
  else pre = "vorbei" end
  return string.format("Fenster %s, %s UTC", pre, hm(u.hour, u.min)), e
end

local function paintMain(s)
  local hb = header(s, "SkyNav Aufgabe", "Schliessen")
  local pad = math.max(6, fl(W * 0.012))
  local lx, lw = pad, fl(W * 0.4)
  local rx = lx + lw + pad
  local rw = W - rx - pad
  local y = hb + pad
  text(lx, y, lw, "AUFGABEN", FONT_XS, C.dim, "l")
  local ly0 = y + th(FONT_XS) + 4
  local rowH = th(FONT_STD) + th(FONT_XS) + 12
  local btnH = math.max(34, th(FONT_STD) + 14)
  local listBottom = H - pad - btnH - pad
  local nVis = math.max(1, (listBottom - ly0) // (rowH + 4))
  s.top = math.max(1, math.min(s.top or 1, math.max(1, #s.tasks - nVis + 1)))
  if s.sel < s.top then s.top = s.sel elseif s.sel >= s.top + nVis then s.top = s.sel - nVis + 1 end
  if #s.tasks == 0 then
    text(lx, ly0 + 6, lw, "Noch keine Aufgabe.", FONT_S, C.dim, "l")
    text(lx, ly0 + 6 + th(FONT_S) + 2, lw, "Mit + Neu den Code eintippen.", FONT_S, C.dim, "l")
  end
  for i = s.top, math.min(#s.tasks, s.top + nVis - 1) do
    local t = s.tasks[i]
    local ry = ly0 + (i - s.top) * (rowH + 4)
    box(lx, ry, lw, rowH, C.panel)
    if i == s.sel then frame(lx, ry, lw, rowH, C.blue) end
    local nm = t.name
    local active = t.id == tostring(s.feld.active or s.app.active or "")
    local aw = active and (txtSize("aktiv", FONT_XS) + 10) or 0
    text(lx + 10, ry + 4, lw - 20 - aw, nm, fit(nm, lw - 20 - aw, {FONT_STD, FONT_S, FONT_XS}), C.white, "l")
    if active then text(lx + lw - aw - 6, ry + 4 + (th(FONT_STD) - th(FONT_XS)) / 2, aw, "aktiv", FONT_XS, C.green, "r") end
    local sub = string.format("%s, %s, %d Zonen, %s%s", clsOf(t), fmtDir(t.dir), #t.zones, t.word, t.src == "feld" and ", Sender" or "")
    text(lx + 10, ry + 6 + th(FONT_STD), lw - 20, sub, fit(sub, lw - 20, {FONT_XS}), C.dim, "l")
    hit(s, lx, ry, lw, rowH, function()
      s.sel = i
      s.slotLen = s.lens[clsOf(t)] or s.slotLen
    end)
  end
  if #s.tasks > nVis then
    local bw = 40
    button(s, lx + lw - 2 * bw - 4, listBottom - btnH + 4, bw, btnH - 4, "Auf", function() s.top = math.max(1, s.top - 1); s.sel = math.max(1, s.sel - 1) end)
    button(s, lx + lw - bw, listBottom - btnH + 4, bw, btnH - 4, "Ab", function() s.sel = math.min(#s.tasks, s.sel + 1) end)
  end
  local by = H - pad - btnH
  local half = (lw - pad) / 2
  button(s, lx, by, half, btnH, "+ Neu", function() s.openNew() end)
  local t = s.tasks[s.sel]
  if t and t.src == "feld" then
    button(s, lx + half + pad, by, half, btnH, s.delAsk and "Wirklich löschen?" or "Löschen", function()
      if s.delAsk then table.remove(s.tasks, s.sel); s.sel = math.max(1, s.sel - 1); s.delAsk = false; saveTasks(s)
      else s.delAsk = true end
    end)
  else
    button(s, lx + half + pad, by, half, btnH, "Löschen", nil, "off")
  end

  -- rechts: Startfenster
  text(rx, y, rw, "STARTFENSTER (DURCHGANG)", FONT_XS, C.dim, "l")
  local py = y + th(FONT_XS) + 4
  local bigF = H >= 400 and FONT_XXL or FONT_XL
  local ph = th(FONT_XS) + th(bigF) + th(FONT_XS) * 2 + 24
  box(rx, py, rw, ph, C.panel)
  local cx = rx + 10
  text(cx, py + 6, 200, "BEGINN", FONT_XS, C.dim, "l")
  local beg = s.edit == "begin" and (s.digits .. string.rep("-", 4 - #s.digits)) or hm(s.slotH, s.slotM)
  local bw = txtSize("00:00", bigF) + 16
  if s.edit == "begin" then frame(cx - 4, py + 6 + th(FONT_XS), bw, th(bigF) + 4, C.blue) end
  text(cx, py + 8 + th(FONT_XS), bw - 8, beg, bigF, C.white, "l")
  hit(s, cx - 4, py + 6, bw, th(bigF) + th(FONT_XS) + 6, function() s.edit, s.digits = "begin", "" end)
  local lx2 = cx + bw + 14
  text(lx2, py + 6, 120, "LÄNGE", FONT_XS, C.dim, "l")
  local lenTxt = s.edit == "len" and (s.digits ~= "" and s.digits or "-") or tostring(s.slotLen)
  local lwid = txtSize("00", bigF) + 16
  if s.edit == "len" then frame(lx2 - 4, py + 6 + th(FONT_XS), lwid, th(bigF) + 4, C.blue) end
  local lt = text(lx2, py + 8 + th(FONT_XS), lwid, lenTxt, bigF, C.white, "l")
  text(lx2 + lt + 4, py + 8 + th(FONT_XS) + th(bigF) - th(FONT_S) - 4, 60, "min", FONT_S, C.white, "l")
  hit(s, lx2 - 4, py + 6, lwid + 40, th(bigF) + th(FONT_XS) + 6, function() s.edit, s.digits = "len", "" end)
  local info, e = slotInfo(s)
  local ex = lx2 + lwid + 50
  if ex + 80 < rx + rw then
    text(ex, py + 6, rx + rw - ex, "ENDE", FONT_XS, C.dim, "l")
    if e then local d = wall(e); text(ex, py + 8 + th(FONT_XS) + th(bigF) - th(FONT_L) - 6, rx + rw - ex, string.format("%02d:%02d:%02d", d.hour, d.min, d.sec), FONT_L, C.green, "l") end
  end
  local iy = py + 14 + th(FONT_XS) + th(bigF)
  text(cx, iy, rw - 20, info, FONT_S, C.dim, "l")
  local cl = t and CLASSES[clsOf(t)]
  if cl then
    local hint = ({Light = "Light 4-8 min", Sport = "Sport 5-10 min", Scale = "Scale 11-20 min, Selbststart 10"})[clsOf(t)] .. " (+59 s)"
    text(cx, iy + th(FONT_S) + 2, rw - 20, hint, FONT_XS, C.dim, "l")
  end

  local ky = py + ph + pad
  if s.edit then
    -- Ziffernblock
    local cols, rows = 6, 2
    local gap = 6
    local kw = (rw - gap * (cols - 1)) / cols
    local kh = math.max(34, (H - pad - ky - btnH - pad - gap) / rows)
    local keys = {"1", "2", "3", "4", "5", "6", "7", "8", "9", "0", "DEL", "OK"}
    for i, k in ipairs(keys) do
      local r, c = (i - 1) // cols, (i - 1) % cols
      button(s, rx + c * (kw + gap), ky + r * (kh + gap), kw, kh, k, function() s.digit(k) end, k == "OK" and "pri" or nil)
    end
    button(s, rx, H - pad - btnH, rw, btnH, "Abbrechen", function() s.edit = nil end)
    return
  end
  local gap = pad
  local third = (rw - 2 * gap) / 3
  button(s, rx, ky, third, btnH, "-1 min", function() s.addMin(-1) end)
  button(s, rx + third + gap, ky, third, btnH, "+1 min", function() s.addMin(1) end)
  button(s, rx + 2 * (third + gap), ky, third, btnH, "+5 min", function() s.addMin(5) end)
  ky = ky + btnH + gap
  button(s, rx, ky, third, btnH, "Länge -", function() s.slotLen = math.max(1, s.slotLen - 1) end)
  button(s, rx + third + gap, ky, third, btnH, "Länge +", function() s.slotLen = math.min(59, s.slotLen + 1) end)
  button(s, rx + 2 * (third + gap), ky, third, btnH, "Jetzt", function()
    local now = localNow(); local d = wall(now - now % 60 + 60); s.slotH, s.slotM = d.hour, d.min end)
  local half2 = (rw - gap) / 2
  local yb = H - pad - btnH
  button(s, rx, yb, half2, btnH, "Änderungs-Code", t and function() s.openChange() end or nil, (not t) and "off" or nil)
  button(s, rx + half2 + gap, yb, half2, btnH, "Übernehmen", t and function()
    if saveSlot(s) then system.exit() else s.msg = "Schreiben fehlgeschlagen (scripts/skynav/feld.lua)" end
  end or nil, t and "pri" or "off")
  if s.msg then text(rx, yb - th(FONT_S) - 4, rw, s.msg, FONT_S, C.red, "l") end
end

-- ---------------------------------------------------------------- Tastatur

local KB_CODE = {}
for i = 1, #ALPHA_S do KB_CODE[i] = ALPHA_S:sub(i, i) end
KB_CODE[#KB_CODE + 1] = "DEL"
local KB_NAME = {}
for c in ("ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789 -"):gmatch(".") do KB_NAME[#KB_NAME + 1] = c end
KB_NAME[#KB_NAME + 1] = "DEL"

-- Tastatur ueber die ganze Breite; die letzte Taste (DEL) fuellt den Rest der letzten Zeile
local function keyboard(s, x, y, w, h, keys, cols, fn)
  local rows = math.ceil(#keys / cols)
  local gap = math.max(4, fl(W * 0.008))
  local kw, kh = (w - gap * (cols - 1)) / cols, (h - gap * (rows - 1)) / rows
  local fonts = {FONT_XL, FONT_L, FONT_STD}
  for i, k in ipairs(keys) do
    local r, c = (i - 1) // cols, (i - 1) % cols
    local label = k == " " and "Leer" or k
    local bw = kw
    if i == #keys then bw = w - c * (kw + gap) end
    button(s, x + c * (kw + gap), y + r * (kh + gap), bw, kh, label, function() fn(k) end, k == "DEL" and "pri" or nil,
      (k == "DEL" or k == " ") and {FONT_STD, FONT_S} or fonts)
  end
end

-- Eingabezeile: getippter Code gross, in Vierergruppen; zu lang -> vorne Gruppen weglassen, das Ende bleibt sichtbar
local function codeLine(x, y, w, h, str, isCode, col)
  local fonts = H >= 400 and {FONT_XXL, FONT_XL, FONT_L} or {FONT_XL, FONT_L, FONT_STD}
  local disp = isCode and grouped(str) or str
  local font = fonts[#fonts]
  for _, f in ipairs(fonts) do if th(f) <= h - 6 and txtSize(disp .. "_", f) <= w then font = f; break end end
  if isCode then
    local g, full = 0, disp
    while #disp > 6 and txtSize(disp .. "_", font) > w do g = g + 1; disp = ".." .. full:sub(5 * g + 1) end
  end
  local tw = text(x, y + (h - th(font)) / 2, w, disp, font, col or C.white, "l")
  box(x + tw + 3, y + 6, 3, h - 12, C.blue)
end

-- Text auf hoechstens zwei Zeilen umbrechen
local function wrap2(str, w, font)
  if txtSize(str, font) <= w then return {str} end
  local cut = #str
  while cut > 1 and (str:sub(cut, cut) ~= " " or txtSize(str:sub(1, cut - 1), font) > w) do cut = cut - 1 end
  if cut <= 1 then return {str} end
  return {str:sub(1, cut - 1), str:sub(cut + 1)}
end

-- ---------------------------------------------------------------- Seite: neue Aufgabe

local function checkNew(s)
  local n = s.new
  n.task, n.err, n.fix, n.nz = nil, nil, nil, 0
  n.zones, n.zerr = {}, {}
  local code = clean(n.fields[2].text)
  if #code > 0 then
    local t, fx, nz = decodeTask(code)
    if t then n.task, n.fix, n.nz = t, fx, nz else if #code >= 36 then n.err = fx end end
  end
  -- Zonenfelder an die Anzahl anpassen
  local want = 2 + (n.nz or 0)
  while #n.fields < want do n.fields[#n.fields + 1] = {label = "Zone " .. (#n.fields - 1), text = "", kind = "code"} end
  while #n.fields > want and #n.fields > 2 and n.fields[#n.fields].text == "" do table.remove(n.fields) end
  if n.task then
    for i = 3, #n.fields do
      local zt = clean(n.fields[i].text)
      if #zt >= 25 then
        local z, fx, idx = decodeZone(zt, n.task)
        if z and idx ~= i - 3 then z, fx = nil, "Code ist Zone " .. (idx + 1) .. ", nicht Zone " .. (i - 2) end
        if z then n.zones[i - 2] = z; n.fields[i].fix = fx
        elseif #zt == 30 or (#zt == 25 and not (fx or ""):find("Tippfehler") ) then n.zerr[i - 2] = fx
        elseif #zt == 25 then n.fields[i].pending = fx end
      end
    end
  end
  n.complete = n.task ~= nil and #n.zones == n.nz
  for i = 1, n.nz do if not n.zones[i] then n.complete = false end end
  if n.complete then
    local t = copyTask(n.task)
    t.zones = n.zones
    n.word = word(t)
  else n.word = nil end
end

local function fieldDone(s, i)
  local n = s.new
  local f = n.fields[i]
  if i == 1 then return false end
  if i == 2 then return n.task ~= nil end
  return n.zones[i - 2] ~= nil
end

local function typeKey(s, k)
  local n = s.new
  local f = n.fields[n.focus]
  if k == "DEL" then f.text = f.text:sub(1, -2)
  else
    local maxLen = n.focus == 1 and 32 or (n.focus == 2 and 36 or 30)
    if #clean(f.text) < maxLen or n.focus == 1 then f.text = f.text .. k end
  end
  checkNew(s)
  -- fertig -> naechstes Feld
  if n.focus >= 2 and fieldDone(s, n.focus) and n.focus < #n.fields then n.focus = n.focus + 1 end
end

-- Aufbau beider Eingabe-Seiten: Kopf, Zeile mit Reitern, grosse Eingabezeile, Status mit Knopf, Tastatur unten
local function entryLayout()
  local pad = math.max(5, fl(W * 0.01))
  local bigF = H >= 400 and FONT_XXL or FONT_XL
  return pad, th(FONT_S) + 14, math.max(th(bigF) + 12, fl(H * 0.13)), th(FONT_S) * 2 + 8
end

local function paintNew(s)
  local n = s.new
  local hb = header(s, "Neue Aufgabe", "Zurück")
  local pad, tabH, inH, stH = entryLayout()
  local y = hb + pad
  -- Reiter: Name, Aufgabe, Zonen; Farbe = Zustand
  local tx = pad
  for i, f in ipairs(n.fields) do
    local done = (i == 2 and n.task) or (i >= 3 and n.zones[i - 2])
    local bad = (i == 2 and n.err) or (i >= 3 and n.zerr[i - 2])
    local fixed = (i == 2 and n.fix and #n.fix > 0) or (i >= 3 and f.fix and #f.fix > 0)
    local label = f.label .. (done and " OK" or "")
    local tw = txtSize(label, FONT_S) + 20
    box(tx, y, tw, tabH, i == n.focus and C.head or C.panel)
    if i == n.focus then frame(tx, y, tw, tabH, C.blue) end
    text(tx, y + (tabH - th(FONT_S)) / 2, tw, label, FONT_S, bad and C.red or (fixed and C.orange or (done and C.green or C.white)), "c")
    hit(s, tx, y, tw, tabH, function() n.focus = i end)
    tx = tx + tw + 4
  end
  y = y + tabH + pad
  -- Eingabezeile
  local f = n.fields[n.focus]
  local need = n.focus == 2 and 36 or (#clean(f.text) > 25 and 30 or 25)
  local rw = math.max(txtSize("PRÜFWORT", FONT_XS), txtSize("WWWW", FONT_L)) + 20
  box(pad, y, W - 2 * pad, inH, C.panel)
  frame(pad, y, W - 2 * pad - rw - pad, inH, C.blue)
  if n.focus == 1 then codeLine(pad + 10, y, W - 3 * pad - rw - 24, inH, f.text, false)
  else codeLine(pad + 10, y, W - 3 * pad - rw - 24, inH, clean(f.text), true) end
  local rx = W - pad - rw
  if n.word then
    text(rx, y + 4, rw, "PRÜFWORT", FONT_XS, C.dim, "c")
    text(rx, y + 6 + th(FONT_XS), rw, n.word, FONT_L, C.green, "c")
  elseif n.focus >= 2 then
    text(rx, y + (inH - th(FONT_L)) / 2, rw, string.format("%d/%d", #clean(f.text), need), FONT_L, C.dim, "c")
  end
  y = y + inH + pad
  -- Status: Fehler, Korrektur oder Zusammenfassung; rechts Speichern
  local bw = txtSize("Speichern", FONT_STD) + 30
  local msg, mcol = nil, C.dim
  local fx = n.focus == 2 and n.fix or (n.focus >= 3 and f.fix)
  if n.focus == 2 and n.err then msg, mcol = n.err, C.red
  elseif n.focus >= 3 and n.zerr[n.focus - 2] then msg, mcol = n.zerr[n.focus - 2], C.red
  elseif fx and #fx > 0 then msg, mcol = string.format("Tippfehler korrigiert: Zeichen %d %s -> %s", fx[1].pos, fx[1].was, fx[1].now), C.orange
  elseif n.task then
    local t = n.task
    local cl = clsOf(t)
    msg = string.format("%s, %s, dTP %d, Platzhöhe %d m, %d Zonen", cl ~= "" and cl or "eigene Werte", fmtDir(t.dir), t.dist, t.elev, n.nz)
    if t.elev == 0 then msg, mcol = msg .. ". Platzhöhe 0 m: stimmt das?", C.orange end
    if not n.complete then
      for i = 1, n.nz do if not n.zones[i] then msg = msg .. string.format(". Jetzt Zone %d eintippen.", i); break end end
    else msg = msg .. ". Prüfwort mit der App vergleichen." end
  else msg = "Code der Aufgabe aus der App eintippen (36 Buchstaben)." end
  local lines = wrap2(msg, W - 3 * pad - bw, FONT_S)
  for k, l in ipairs(lines) do text(pad, y + (k - 1) * (th(FONT_S) + 2), W - 3 * pad - bw, l, fit(l, W - 3 * pad - bw, {FONT_S, FONT_XS}), mcol, "l") end
  button(s, W - pad - bw, y, bw, stH, "Speichern", n.word and function()
    local t = copyTask(n.task)
    t.zones = n.zones
    local name = n.fields[1].text:gsub("^%s+", ""):gsub("%s+$", "")
    t.name = name ~= "" and name or ("Aufgabe " .. n.word)
    t.cls = clsOf(t)
    t.id = "F-" .. n.word
    t.src = "feld"
    t.word = n.word
    for i = #s.tasks, 1, -1 do if s.tasks[i].id == t.id then table.remove(s.tasks, i) end end
    s.tasks[#s.tasks + 1] = t
    s.sel = #s.tasks
    s.feld.active = t.id
    s.slotLen = s.lens[t.cls] or (CLASSES[t.cls] and CLASSES[t.cls].lenMin) or s.slotLen
    if not saveTasks(s) then s.msg = "Schreiben fehlgeschlagen (scripts/skynav/feld.lua)" end
    s.screen = "main"
  end or nil, n.word and "pri" or "off")
  y = y + stH + pad
  -- Tastatur ueber die ganze Breite
  if n.focus == 1 then keyboard(s, pad, y, W - 2 * pad, H - pad - y, KB_NAME, 10, function(k) typeKey(s, k) end)
  else keyboard(s, pad, y, W - 2 * pad, H - pad - y, KB_CODE, 8, function(k) typeKey(s, k) end) end
end

-- ---------------------------------------------------------------- Seite: Aenderung

local function checkChange(s)
  local c = s.chg
  c.task, c.err, c.fix, c.mask = nil, nil, nil, nil
  local code = clean(c.text)
  if #code < 8 then return end
  local t, fx, mask = decodeChange(code, c.base)
  if t then c.task, c.fix, c.mask = t, fx, mask; c.word = word(t)
  elseif fx then c.err = fx end
end

local function paintChange(s)
  local c = s.chg
  local hb = header(s, "Änderung", "Zurück")
  local pad, tabH, inH, stH = entryLayout()
  local y = hb + pad
  local info = string.format("%s, Grundlage %s", c.orig.name, word(c.base))
  text(pad, y + (tabH - th(FONT_S)) / 2, W - 2 * pad, info, fit(info, W - 2 * pad, {FONT_S, FONT_XS}), C.dim, "l")
  y = y + tabH + pad
  local t = c.task
  local rw = math.max(txtSize("PRÜFWORT", FONT_XS), txtSize("WWWW", FONT_L)) + 20
  box(pad, y, W - 2 * pad, inH, C.panel)
  frame(pad, y, W - 2 * pad - rw - pad, inH, C.blue)
  codeLine(pad + 10, y, W - 3 * pad - rw - 24, inH, clean(c.text), true)
  local rx = W - pad - rw
  if t then
    text(rx, y + 4, rw, "PRÜFWORT", FONT_XS, C.dim, "c")
    text(rx, y + 6 + th(FONT_XS), rw, c.word, FONT_L, C.green, "c")
  else text(rx, y + (inH - th(FONT_L)) / 2, rw, tostring(#clean(c.text)), FONT_L, C.dim, "c") end
  y = y + inH + pad
  local parts = {}
  if t then
    if c.mask & 1 ~= 0 then parts[#parts + 1] = string.format("Ursprung %.6f / %.6f", t.lat, t.lon) end
    if c.mask & 2 ~= 0 then parts[#parts + 1] = string.format("Platzhöhe %d -> %d m", rnd(c.orig.elev or 0), t.elev) end
    if c.mask & 4 ~= 0 then parts[#parts + 1] = string.format("Richtung %s -> %s", fmtDir(c.orig.dir), fmtDir(t.dir)) end
    if c.mask & 8 ~= 0 then parts[#parts + 1] = string.format("dTP %d -> %d m", rnd(c.orig.dist or 0), t.dist) end
    if c.mask & 16 ~= 0 then parts[#parts + 1] = string.format("Limits %d m, %d km/h, Ziel %d km/h, %d min", t.maxH, t.maxV, t.minF, t.work) end
    if c.fix and #c.fix > 0 then parts[#parts + 1] = string.format("korrigiert: Zeichen %d %s -> %s", c.fix[1].pos, c.fix[1].was, c.fix[1].now) end
  end
  local msg = t and table.concat(parts, ", ") or (c.err or "Änderungs-Code aus der App eintippen.")
  local mcol = t and C.blue or (c.err and C.red or C.dim)
  local bw = txtSize("Übernehmen", FONT_STD) + 30
  local lines = wrap2(msg, W - 3 * pad - bw, FONT_S)
  for k, l in ipairs(lines) do text(pad, y + (k - 1) * (th(FONT_S) + 2), W - 3 * pad - bw, l, fit(l, W - 3 * pad - bw, {FONT_S, FONT_XS}), mcol, "l") end
  button(s, W - pad - bw, y, bw, stH, "Übernehmen", t and function()
    local o = c.orig
    local nt = copyTask(t)
    nt.name, nt.cls, nt.src = o.name, clsOf(t), "feld"
    nt.base = o.base or copyTask(c.base)
    nt.word = c.word
    nt.id = o.src == "feld" and o.id or ("F-" .. c.word)
    if o.src == "feld" then s.tasks[s.sel] = nt else s.tasks[#s.tasks + 1] = nt; s.sel = #s.tasks end
    s.feld.active = nt.id
    if not saveTasks(s) then s.msg = "Schreiben fehlgeschlagen (scripts/skynav/feld.lua)" end
    s.screen = "main"
  end or nil, t and "pri" or "off")
  y = y + stH + pad
  keyboard(s, pad, y, W - 2 * pad, H - pad - y, KB_CODE, 8, function(k)
    if k == "DEL" then c.text = c.text:sub(1, -2) elseif #clean(c.text) < 40 then c.text = c.text .. k end
    checkChange(s)
  end)
end

-- ---------------------------------------------------------------- Ethos

local function create()
  colors()
  local s = {screen = "main", hits = {}}
  loadState(s)
  s.openNew = function()
    s.new = {fields = {{label = "Name", text = "", kind = "name"}, {label = "Aufgabe", text = "", kind = "code"}}, focus = 2, zones = {}, zerr = {}}
    s.screen = "new"
  end
  s.openChange = function()
    local o = s.tasks[s.sel]
    if not o then return end
    s.chg = {orig = o, base = o.base or o, text = ""}
    s.screen = "chg"
  end
  s.addMin = function(d)
    local m = (s.slotH * 60 + s.slotM + d) % 1440
    s.slotH, s.slotM = m // 60, m % 60
  end
  s.digit = function(k)
    if k == "DEL" then s.digits = s.digits:sub(1, -2)
    elseif k == "OK" then
      if s.edit == "begin" and #s.digits >= 3 then
        local v = tonumber(s.digits)
        local h, m = v // 100, v % 100
        if h < 24 and m < 60 then s.slotH, s.slotM = h, m end
      elseif s.edit == "len" and #s.digits > 0 then
        s.slotLen = math.max(1, math.min(59, tonumber(s.digits)))
      end
      s.edit = nil
    elseif (s.edit == "begin" and #s.digits < 4) or (s.edit == "len" and #s.digits < 2) then
      s.digits = s.digits .. k
      if s.edit == "begin" and #s.digits == 4 then s.digit("OK") end
    end
  end
  return s
end

local function paint(s)
  W, H = lcd.getWindowSize()
  s.hits = {}
  box(0, 0, W, H, C.bg)
  if s.screen == "new" then paintNew(s)
  elseif s.screen == "chg" then paintChange(s)
  else paintMain(s) end
end

local function event(s, category, value, x, y)
  if EVT_TOUCH and category == EVT_TOUCH then
    if value == (TOUCH_END or value) then
      for i = #s.hits, 1, -1 do
        local h = s.hits[i]
        if x >= h.x and x <= h.x + h.w and y >= h.y and y <= h.y + h.h then
          if h.fn then
            local ask = s.delAsk
            h.fn()
            if ask and s.delAsk then s.delAsk = false end   -- anderer Knopf: Loeschen-Frage verwerfen
          end
          lcd.invalidate()
          return true
        end
      end
      s.delAsk = false
      lcd.invalidate()
    end
    return true
  end
  if EVT_KEY and category == EVT_KEY then
    if (KEY_RTN_BREAK and value == KEY_RTN_BREAK) or (KEY_EXIT_BREAK and value == KEY_EXIT_BREAK) then
      if s.screen ~= "main" then s.screen = "main"; lcd.invalidate(); return true end
      if s.edit then s.edit = nil; lcd.invalidate(); return true end
    end
  end
  return false
end

local lastSec
local function wakeup(s)
  local t = os.time()
  if t ~= lastSec then lastSec = t; lcd.invalidate() end
end

local ICON
local function init()
  if lcd and lcd.loadMask then
    local ok, m = pcall(lcd.loadMask, "icon.png")
    if ok then ICON = m end
  end
  system.registerSystemTool({name = "SkyNav Aufgabe", icon = ICON, create = create, wakeup = wakeup, paint = paint, event = event})
end

return {init = init, _test = {encode = encode, decode = decode, decodeTask = decodeTask, decodeZone = decodeZone,
  decodeChange = decodeChange, word = word, crc32 = crc32, canonStr = canonStr, create = create, paint = paint, event = event,
  saveSlot = saveSlot, saveTasks = saveTasks, loadState = loadState, VERSION = VERSION}}
