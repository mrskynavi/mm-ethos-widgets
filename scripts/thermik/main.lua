-- @widget  Thermik
-- @version 1.0
-- Thermik-Auswertung für das Ethos-Dashboard.
-- Jeder Steigflug (Thermik-Schalter EIN -> AUS) wird als Balken auf einer
-- Zeitachse dargestellt. Die Balkenhöhe zeigt je nach Konfiguration die
-- Steiggeschwindigkeit oder den Höhengewinn. Unter dem Diagramm stehen
-- Ø Steigen, Anzahl der Steigflüge und die total gewonnene Höhe.

local MODE_RATE = 1
local MODE_HEIGHT = 2
local MODE_DURATION = 3

local MAX_BAR_WIDTH = 6
local BAR_GAP = 1
local MIN_DURATION = 5  -- s, kürzere Schaltphasen zählen nicht als Steigflug

local function create()
  return {
    switchSource = nil,
    heightSource = nil,
    resetSource = nil,
    viewSource = nil,
    activeMode = nil,
    viewActive = false,
    viewOffset = 0,
    configuredMode = nil,
    demoMode = true,
    largeLayout = false,
    displayMode = MODE_RATE,
    climbActive = false,
    resetActive = false,
    startTime = nil,
    startHeight = nil,
    highestHeight = nil,
    liveGain = 0,
    climbs = {},
    totalHeight = 0,
    totalDuration = 0,
    -- Demo-Zustand
    demoRate = 0,
    lastInvalidate = 0
  }
end

local function resetClimbs(widget)
  widget.climbActive = false
  widget.startTime = nil
  widget.startHeight = nil
  widget.highestHeight = nil
  widget.liveGain = 0
  widget.climbs = {}
  widget.totalHeight = 0
  widget.totalDuration = 0
end

local function addClimb(widget, duration, gain)
  table.insert(widget.climbs, {
    height = gain,
    duration = duration,
    rate = gain / duration
  })
  widget.totalHeight = widget.totalHeight + gain
  widget.totalDuration = widget.totalDuration + duration
end

local function currentMode(widget)
  return widget.activeMode or widget.displayMode
end

local function climbValue(widget, height, rate, duration)
  local mode = currentMode(widget)
  if mode == MODE_HEIGHT then
    return height
  elseif mode == MODE_DURATION then
    return duration
  end
  return rate
end

-- Schöne Rasterschrittweite (1/2/5 * 10^n) für etwa vier Linien.
local function niceStep(maximum)
  local raw = math.max(maximum, 0.1) / 4
  local magnitude = 10 ^ math.floor(math.log(raw, 10))
  local fraction = raw / magnitude
  local step
  if fraction <= 1 then
    step = 1
  elseif fraction <= 2 then
    step = 2
  elseif fraction <= 5 then
    step = 5
  else
    step = 10
  end
  return step * magnitude
end

local function startClimb(widget, now)
  widget.climbActive = true
  widget.startTime = now
  widget.liveGain = 0
  if widget.demoMode then
    -- Demo: simulierter Höhengewinn mit zufälliger Steigrate.
    widget.demoRate = 0.5 + math.random() * 4.5
  elseif widget.heightSource then
    widget.startHeight = widget.heightSource:value()
    widget.highestHeight = widget.startHeight
  end
end

local function endClimb(widget, now)
  local duration = now - widget.startTime
  local gain = nil
  widget.climbActive = false

  if widget.demoMode then
    gain = widget.liveGain
  elseif widget.startHeight ~= nil and widget.highestHeight ~= nil then
    gain = math.max(0, widget.highestHeight - widget.startHeight)
  end

  widget.startTime = nil
  widget.startHeight = nil
  widget.highestHeight = nil
  widget.liveGain = 0

  if duration >= MIN_DURATION and gain ~= nil then
    addClimb(widget, duration, gain)
  end
end

local function paintLabelValue(x, y, label, value)
  lcd.font(FONT_STD)
  lcd.color(lcd.RGB(170, 170, 170))
  lcd.drawText(x, y, label)
  local _, labelHeight = lcd.getTextSize(label)
  lcd.font(FONT_BOLD)
  lcd.color(WHITE)
  lcd.drawText(x, y + labelHeight, value)
end

local function paint(widget)
  local width, height = lcd.getWindowSize()
  local largeDisplay = widget.largeLayout or width > 900 or height > 300

  -- Kennzahlenblock unter dem Diagramm: Höhe aus der Schrifthöhe ableiten.
  lcd.font(FONT_STD)
  local _, labelHeight = lcd.getTextSize("Ø")
  lcd.font(FONT_BOLD)
  local _, valueHeight = lcd.getTextSize("0")
  local statsHeight = labelHeight + valueHeight + 2

  local count = #widget.climbs
  local average = 0
  if widget.totalDuration > 0 then
    average = widget.totalHeight / widget.totalDuration
  end

  local totalSeconds = math.floor(widget.totalDuration + 0.5)
  local columnWidth = math.floor(width / 4)
  local statsTop = height - statsHeight - 2
  paintLabelValue(4, statsTop, "Ø Steigen", string.format("%.1f m/s", average))
  paintLabelValue(4 + columnWidth, statsTop, "Steigflüge", string.format("%d", count))
  paintLabelValue(4 + columnWidth * 2, statsTop, "Gesamthöhe", string.format("%.0f m", widget.totalHeight))
  paintLabelValue(4 + columnWidth * 3, statsTop, "Gesamtdauer",
    string.format("%d:%02d", math.floor(totalSeconds / 60), totalSeconds % 60))

  -- Titelzeile: gewählte Ansicht mit Einheit.
  local mode = currentMode(widget)
  local title = "Steiggeschwindigkeit (m/s)"
  if mode == MODE_HEIGHT then
    title = "Erreichte Höhe (m)"
  elseif mode == MODE_DURATION then
    title = "Steigdauer (s)"
  end
  lcd.font(largeDisplay and FONT_STD or FONT_S)
  local titleWidth, titleHeight = lcd.getTextSize(title)
  lcd.color(WHITE)
  lcd.drawText(math.floor((width - titleWidth) / 2), 2, title)

  local graphTop = titleHeight + 8
  local graphBottom = statsTop - 6
  local graphHeight = graphBottom - graphTop
  if graphHeight <= 20 then
    return
  end

  -- Werte aller Balken (inkl. laufendem Steigflug) bestimmen.
  local values = {}
  local maximum = 0
  for index, climb in ipairs(widget.climbs) do
    values[index] = climbValue(widget, climb.height, climb.rate, climb.duration)
    maximum = math.max(maximum, values[index])
  end
  local liveValue = nil
  if widget.climbActive and widget.startTime and os.clock() - widget.startTime >= MIN_DURATION then
    local elapsed = os.clock() - widget.startTime
    local rate = 0
    if elapsed > 0 then
      rate = widget.liveGain / elapsed
    end
    liveValue = climbValue(widget, widget.liveGain, rate, elapsed)
    maximum = math.max(maximum, liveValue)
  end

  -- Die Achse folgt dynamisch dem aktuellen Maximum.
  local step = niceStep(maximum)
  local axisMax = math.max(step, math.ceil(maximum / step - 1e-9) * step)
  local labelFormat = step < 1 and "%.1f" or "%d"

  local scaleWidth = largeDisplay and 44 or 34
  local plotLeft = scaleWidth
  local plotRight = width - 5
  local plotWidth = plotRight - plotLeft
  if plotWidth <= 10 then
    return
  end

  lcd.font(largeDisplay and FONT_STD or FONT_S)
  lcd.pen(DOTTED)
  lcd.color(lcd.RGB(120, 120, 120))
  local lines = math.floor(axisMax / step + 0.5)
  for i = 0, lines do
    local y = graphBottom - math.floor((i * step / axisMax) * graphHeight)
    if i > 0 then
      lcd.drawLine(plotLeft, y, plotRight, y)
    end
    local text = string.format(labelFormat, i * step)
    local textWidth, textHeight = lcd.getTextSize(text)
    lcd.color(WHITE)
    lcd.drawText(plotLeft - textWidth - 4, y - textHeight / 2, text)
    lcd.color(lcd.RGB(120, 120, 120))
  end
  lcd.pen(SOLID)
  lcd.color(WHITE)
  lcd.drawLine(plotLeft, graphBottom, plotRight, graphBottom)
  lcd.drawLine(plotLeft, graphTop, plotLeft, graphBottom)

  -- Balkenbreite: 6 px, solange alles passt, sonst bis auf 1 px verkleinern.
  local total = count + (liveValue and 1 or 0)
  local barWidth = 1
  for w = MAX_BAR_WIDTH, 1, -1 do
    if total * (w + BAR_GAP) - BAR_GAP <= plotWidth then
      barWidth = w
      break
    end
  end
  local pitch = barWidth + BAR_GAP
  local capacity = math.floor((plotWidth + BAR_GAP) / pitch)
  local first = math.max(1, total - capacity + 1)

  local x = plotLeft + 2
  local function drawBar(value, color)
    local barHeight = math.floor((value / axisMax) * graphHeight)
    if value > 0 and barHeight < 1 then
      barHeight = 1
    end
    lcd.color(color)
    lcd.drawFilledRectangle(x, graphBottom - barHeight, barWidth, barHeight)
    x = x + pitch
  end

  for index = first, total do
    if index <= count then
      drawBar(values[index], lcd.RGB(0, 170, 255))
    else
      drawBar(liveValue, lcd.RGB(0, 255, 0))
    end
  end

  if total == 0 then
    lcd.font(FONT_STD)
    lcd.color(lcd.RGB(170, 170, 170))
    lcd.drawText(plotLeft + 8, graphTop + 4, "Noch kein Steigflug")
  end
end

-- storage.read() liefert für nicht gesetzte oder inkompatible Einträge
-- unter Umständen ein boolean; nur echte Source-Objekte werden verwendet.
local function validSource(source)
  if source and type(source) ~= "boolean" then
    return source
  end
  return nil
end

local function readSwitch(source)
  if validSource(source) then
    local value = source:value()
    return type(value) == "number" and value > 0
  end
  return false
end

local function wakeup(widget)
  local now = os.clock()

  local resetOn = readSwitch(widget.resetSource)
  if resetOn and not widget.resetActive then
    resetClimbs(widget)
    lcd.invalidate()
  end
  widget.resetActive = resetOn

  -- Der Thermik-Schalter gilt auch im Demo-Modus; nur die Höhe wird simuliert.
  -- Ansicht-Wechsel: jede steigende Flanke schaltet zur nächsten Ansicht
  -- weiter, ausgehend von der konfigurierten. Eine geänderte Konfiguration
  -- setzt wieder auf die konfigurierte Ansicht zurück.
  if widget.configuredMode ~= widget.displayMode then
    widget.configuredMode = widget.displayMode
    widget.viewOffset = 0
  end
  local viewOn = readSwitch(widget.viewSource)
  if viewOn and not widget.viewActive then
    widget.viewOffset = (widget.viewOffset + 1) % 3
  end
  widget.viewActive = viewOn
  local mode = (widget.displayMode - 1 + widget.viewOffset) % 3 + 1
  if mode ~= widget.activeMode then
    widget.activeMode = mode
    lcd.invalidate()
  end

  local switchOn = readSwitch(widget.switchSource)

  if switchOn and not widget.climbActive then
    startClimb(widget, now)
  elseif not switchOn and widget.climbActive then
    endClimb(widget, now)
    lcd.invalidate()
    return
  end

  if widget.climbActive then
    if widget.demoMode then
      widget.liveGain = (now - widget.startTime) * widget.demoRate
    elseif widget.heightSource then
      local h = widget.heightSource:value()
      if h ~= nil then
        widget.highestHeight = math.max(widget.highestHeight or h, h)
        if widget.startHeight == nil then
          widget.startHeight = h
        end
        widget.liveGain = math.max(0, widget.highestHeight - widget.startHeight)
      end
    end
    -- Laufenden Balken ca. 4x pro Sekunde neu zeichnen.
    if now - widget.lastInvalidate >= 0.25 then
      widget.lastInvalidate = now
      lcd.invalidate()
    end
  end
end

local function configure(widget)
  local line = form.addLine("HD Bildschirm")
  form.addBooleanField(line, form.getFieldSlots(line)[0],
    function() return widget.largeLayout end,
    function(value) widget.largeLayout = value end)

  line = form.addLine("Demo-Modus")
  form.addBooleanField(line, form.getFieldSlots(line)[0],
    function() return widget.demoMode end,
    function(value)
      widget.demoMode = value
      resetClimbs(widget)
      if widget.heightField then
        widget.heightField:enable(not value)
      end
    end)

  line = form.addLine("Thermik-Schalter")
  form.addSwitchField(line, nil,
    function() return widget.switchSource end,
    function(value) widget.switchSource = value end)

  line = form.addLine("Höhenquelle")
  widget.heightField = form.addSourceField(line, nil,
    function() return widget.heightSource end,
    function(value) widget.heightSource = value end)
  widget.heightField:enable(not widget.demoMode)

  line = form.addLine("Balkenhöhe")
  form.addChoiceField(line, nil,
    {{"Steiggeschwindigkeit", MODE_RATE}, {"Erreichte Höhe", MODE_HEIGHT}, {"Steigdauer", MODE_DURATION}},
    function() return widget.displayMode end,
    function(value) widget.displayMode = value end)

  line = form.addLine("Ansicht-Wechsel")
  form.addSwitchField(line, nil,
    function() return widget.viewSource end,
    function(value) widget.viewSource = value end)

  line = form.addLine("Reset-Schalter")
  form.addSwitchField(line, nil,
    function() return widget.resetSource end,
    function(value) widget.resetSource = value end)
end

local function read(widget)
  widget.switchSource = validSource(storage.read("switchSource"))
  widget.heightSource = validSource(storage.read("heightSource"))
  widget.resetSource = validSource(storage.read("resetSource"))
  widget.viewSource = validSource(storage.read("viewSource"))
  local demoMode = storage.read("demoMode")
  if demoMode ~= nil then
    widget.demoMode = demoMode
  end
  local largeLayout = storage.read("largeLayout")
  if largeLayout ~= nil then
    widget.largeLayout = largeLayout
  end
  local displayMode = storage.read("displayMode")
  if displayMode ~= nil then
    widget.displayMode = displayMode
  end
end

local function write(widget)
  storage.write("switchSource", widget.switchSource)
  storage.write("heightSource", widget.heightSource)
  storage.write("resetSource", widget.resetSource)
  storage.write("viewSource", widget.viewSource)
  storage.write("demoMode", widget.demoMode)
  storage.write("largeLayout", widget.largeLayout)
  storage.write("displayMode", widget.displayMode)
end

local function init()
  -- Der key muss in Ethos kürzer als 8 Zeichen sein.
  system.registerWidget({
    key = "therm01",
    name = "Thermik",
    create = create,
    paint = paint,
    wakeup = wakeup,
    configure = configure,
    read = read,
    write = write
  })
end

return {init = init}
