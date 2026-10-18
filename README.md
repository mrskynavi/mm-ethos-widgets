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
