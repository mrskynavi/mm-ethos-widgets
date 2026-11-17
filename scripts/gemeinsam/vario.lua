-- @version 1.0
-- Gemeinsames Modul fuer alle Widgets: Vario-Eichfaktor lernen und anwenden.
-- Die Rechnung steht hier einmal (wie im MM Flight Analyzer, web/core/energie.js: varioLernen), damit jedes Widget
-- dieselbe Zahl nimmt. Laden: local VM = loadScript("/scripts/gemeinsam/vario.lua")()  (nil, wenn die Datei fehlt)
--
-- Vario und Hoehenaenderung messen dieselbe Bewegung, der Motor spielt keine Rolle. Alle WIN s wird die Hoehenrate
-- gegen das mittlere Vario gesetzt (f * Vario ~ Rate, kleinste Quadrate durch 0). Nur Fenster mit |Vario| >= MINABS zaehlen.
-- Ab MINN Fenstern und plausiblem Faktor (LO .. HI) gilt er, sonst 1.
-- Der gelernte Faktor wird pro Modell gespeichert (Widget-Speicher), damit er nach dem Einschalten sofort gilt.
local M = {}
local WIN, MINN, MINABS, LO, HI = 10, 20, 0.4, 0.6, 1.6

-- st: Zustand des Widgets (Tabelle). hA = Hoehe (m), vario = Vario (m/s), now = Zeit (s)
function M.step(st, now, hA, vario)
  if hA == nil or vario == nil then return st.varioF or 1 end
  if st.lT0 == nil then st.lT0, st.lH0, st.lVs, st.lVn = now, hA, 0, 0 end
  st.lVs, st.lVn = st.lVs + vario, st.lVn + 1
  if now - st.lT0 < WIN then return st.varioF or 1 end
  local vm = st.lVs / st.lVn
  if math.abs(vm) >= MINABS then
    local rate = (hA - st.lH0) / (now - st.lT0)
    st.lNum, st.lDen, st.lN = (st.lNum or 0) + rate * vm, (st.lDen or 0) + vm * vm, (st.lN or 0) + 1
    local f = st.lNum / st.lDen
    st.varioF = (st.lN >= MINN and f >= LO and f <= HI) and f or 1
  end
  st.lT0, st.lH0, st.lVs, st.lVn = now, hA, 0, 0
  return st.varioF or 1
end

-- Faktor auf das Vario anwenden
function M.apply(st, vario)
  if vario == nil then return nil end
  return vario * (st.varioF or 1)
end

-- Gespeicherten Faktor laden und speichern. read(key) / write(key, value) kommen vom Widget (storage.read/write).
-- Nur die Zahl, nicht die Zwischenwerte. Schreiben nur, wenn sich der Faktor um mehr als 0.005 aendert.
function M.load(st, read)
  local v = read and read("varioF")
  if type(v) == "number" and v >= LO and v <= HI then st.varioF = v; st.varioSaved = v end
end
function M.save(st, write)
  if not write or st.varioF == nil then return end
  if st.varioSaved == nil or math.abs(st.varioF - st.varioSaved) > 0.005 then
    st.varioSaved = st.varioF; pcall(write, "varioF", st.varioF)
  end
end

return M
