# MM Ethos Widgets

Ethos-Lua-Widgets von Mächler Modelle. Der MM Flight Analyzer holt sie von hier und kopiert sie beim Anschliessen des Senders nach `scripts/<Name>/` auf den Sender.

- Aufbau wie am Sender: `scripts/<Name>/main.lua` (weitere Dateien des Widgets im selben Ordner).
- Version als Kommentar ganz oben in `main.lua`:

```lua
-- @widget  Sollfahrt
-- @version 1.0
```

`scripts/maccready/polaren.lua` gehört nicht hierher, die schreibt die App selbst.

`scripts/skynav/aufgaben.lua` schreibt ebenfalls die App (Tab SkyNavigator), die Flüge legt das Widget in `scripts/skynav/fluege/` ab, die gelernten Wenderadien in `scripts/skynav/radien.lua`.

`scripts/skynav/audio/links.wav` und `rechts.wav` sind die Sprachdateien für die Kurs-Ansage „xx links/rechts“ (16 kHz, 16 Bit, mono).

`scripts/skynavtool/` ist kein Widget, sondern das Systemtool „SkyNav Aufgabe“ (am Sender unter System, Werkzeuge). Damit tippt man den Buchstaben-Code einer Aufgabe ab, den die App nach dem Scan des QR-Codes zeigt, und stellt vor jedem Durchgang das Startfenster ein. Es schreibt `scripts/skynav/feld.lua`; das Widget SkyNavigator liest diese Datei zusätzlich zu `aufgaben.lua`.

`scripts/tekkal/` ist das Widget „TEK-Kalibrierung“ für den SM GPS-Logger 3 (Airspeed über die TEK-Düse). Es bestimmt im Flug den Airspeed-Faktor des Loggers (aus Kreisen, Wind wird mitgeschätzt) und wie gut die TEK-Düse kompensiert (aus Fahrtwechseln geradeaus). Es registriert die Quelle „TEK korr.“ = TEK-Vario + Korrektur × d(v²/2g)/dt, damit lässt sich die Düse elektronisch nachkompensieren. Ergebnisse schreibt es nach `scripts/tekkal/ergebnis_<Modell>.txt`, die übernommene Korrektur nach `korrektur_<Modell>.txt`.
