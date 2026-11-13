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

SkyNavigator ab 1.30: In der Konfiguration wählt man das angeschlossene Sensor-Gerät (SM GPS-Logger 3 oder FrSky Vario mit FrSky-GPS) und ob zusätzlich ein Stabi-Empfänger dran ist. Das Widget sucht GPS, Höhe, Vario, Fahrt, Kurs, Satelliten, Querlage, Nickwinkel und Beschleunigung selbst (S.Port-AppID, sonst Ethos-Name). Gibt es `scripts/tekkal/korrektur_<Modell>.txt`, nimmt es mit dem Logger „TEK korr.“ und „Fahrt korr.“. Einzelne Quellen von Hand gehen vor (Schalter „Einzelne Quellen von Hand“).

`scripts/skynav/audio/links.wav` und `rechts.wav` sind die Sprachdateien für die Kurs-Ansage „xx links/rechts“ (16 kHz, 16 Bit, mono).

`scripts/skynavtool/` ist kein Widget, sondern das Systemtool „SkyNav Aufgabe“ (am Sender unter System, Werkzeuge). Damit tippt man den Buchstaben-Code einer Aufgabe ab, den die App nach dem Scan des QR-Codes zeigt, und stellt vor jedem Durchgang das Startfenster ein. Es schreibt `scripts/skynav/feld.lua`; das Widget SkyNavigator liest diese Datei zusätzlich zu `aufgaben.lua`.

`scripts/tekkal/` ist das Widget „TEK-Kalibrierung“ für den SM GPS-Logger 3 (Airspeed über die TEK-Düse). Es bestimmt im Flug die Fahrt-Korrektur (aus Kreisen, Wind wird mitgeschätzt) und wie gut die TEK-Düse kompensiert (aus Fahrtwechseln geradeaus). Es registriert zwei Quellen, die pro Modell korrigieren, ohne Logger oder Düse zu ändern: „Fahrt korr.“ (Logger-Fahrt × Faktor, in SkyNavigator, Sollfahrt und Polarflug als „Fahrt Staudruck“ wählen) und „TEK korr.“ (TEK-Vario + Korrektur × d(v²/2g)/dt, als Vario mit Vario-Art TE). Korrekturen und Sensornamen liegen in `scripts/tekkal/korrektur_<Modell>.txt`, damit laufen die Quellen auch ohne das Widget auf dem Bildschirm. Ergebnisse in `ergebnis_<Modell>.txt`. Ab 1.3: Kurs aus der GPS-Position (die „Flugrichtung“ des SM-Loggers ist kein Kurs), Kreise beliebig langsam und mit wechselnder Fahrt, Motorsteigflug wird ausgeschlossen (Motor-Quelle und/oder Vario ab 8 m/s, bis 15 s danach), Übernehmen und Neu beginnen über Schalter (ein Tippen ins Widget öffnet bei Ethos das Widget-Menü), Anweisung groß oben.
