-- @widget  Steigflüge
-- @version 1.0
-- Steigflug-Auswertung für das Ethos-Dashboard.

local function create()
  return {
    switchSource = nil,
    heightSource = nil,
    resetSource = nil,
    demoMode = true,
    largeLayout = false,
    climbActive = false,
    resetActive = false,
    startTime = nil,
    startHeight = nil,
    highestHeight = nil,
    climbs = {},
    totalHeight = 0,
    totalDuration = 0
  }
end

local function resetClimbs(widget)
  widget.climbActive = false
  widget.startTime = nil
  widget.startHeight = nil
  widget.highestHeight = nil
  widget.climbs = {}
  widget.totalHeight = 0
  widget.totalDuration = 0
end

local function addClimb(widget, duration, height)
  local averageSpeed = height / duration

  table.insert(widget.climbs, {
    height = height,
    averageSpeed = averageSpeed
  })
  widget.totalHeight = widget.totalHeight + height
  widget.totalDuration = widget.totalDuration + duration
end

local function paintLabelValue(x, y, label, value, valueOffset)
  lcd.font(FONT_STD)
  lcd.color(WHITE)
  lcd.drawText(x, y, label)
  lcd.font(FONT_BOLD)
  lcd.drawText(x, y + valueOffset, value)
end

local function paint(widget)
  local width, height = lcd.getWindowSize()
  local largeDisplay = widget.largeLayout or width > 900 or height > 300
  local leftWidth = math.min(210, math.floor(width * 0.27))
  local graphLeft = leftWidth + 8
  local graphRight = width - 5
  local graphTop = largeDisplay and 32 or 24
  local graphBottom = height - 10
  local graphHeight = graphBottom - graphTop
  local labelSpacing = largeDisplay and 70 or 46
  local valueOffset = largeDisplay and 28 or 18
  local globalAverage = 0

  if widget.totalDuration > 0 then
    globalAverage = widget.totalHeight / widget.totalDuration
  end

  -- Der Kennzahlenblock beginnt auf Höhe der obersten Rasterlinie.
  paintLabelValue(4, graphTop, "Steigflüge", string.format("%d", #widget.climbs), valueOffset)
  paintLabelValue(4, graphTop + labelSpacing, "Ø Steigen", string.format("%.1f m/s", globalAverage), valueOffset)
  paintLabelValue(4, graphTop + labelSpacing * 2, "Gesamthöhe", string.format("%.0f m", widget.totalHeight), valueOffset)

  if widget.climbActive then
    lcd.font(FONT_STD)
    lcd.color(lcd.RGB(0, 255, 0))
    lcd.drawText(4, graphTop + labelSpacing * 3, "Steigflug läuft")
  end

  if graphHeight <= 20 or graphRight <= graphLeft then
    return
  end

  if #widget.climbs == 0 then
    lcd.font(FONT_STD)
    lcd.color(lcd.RGB(170, 170, 170))
    lcd.drawText(graphLeft + 4, graphTop + 4, "Noch kein Steigflug")
    return
  end

  -- Der Diagrammbereich beginnt und endet an den auf 50 m gerundeten Werten.
  local minimumHeight = widget.climbs[1].height
  local maximumHeight = minimumHeight
  for _, climb in ipairs(widget.climbs) do
    minimumHeight = math.min(minimumHeight, climb.height)
    maximumHeight = math.max(maximumHeight, climb.height)
  end
  local minimumAxis = math.floor(minimumHeight / 50) * 50
  local maximumAxis = math.ceil(maximumHeight / 50) * 50
  if maximumAxis == minimumAxis then
    maximumAxis = maximumAxis + 50
  end
  local axisRange = maximumAxis - minimumAxis
  -- Die Skalenwerte stehen ohne Einheit vor der Linie; 6 px bleiben als Abstand.
  local scaleWidth = 30
  local plotLeft = graphLeft + scaleWidth
  local plotWidth = graphRight - plotLeft
  local scaleTextX = largeDisplay and graphLeft - 16 or graphLeft

  lcd.color(lcd.RGB(120, 120, 120))
  lcd.pen(DOTTED)
  for metres = minimumAxis, maximumAxis, 50 do
    local y = graphBottom - ((metres - minimumAxis) / axisRange) * graphHeight
    lcd.drawLine(plotLeft, y, graphRight, y)
    lcd.font(FONT_STD)
    lcd.drawText(scaleTextX, y - 8, string.format("%d", metres))
  end
  lcd.pen(SOLID)
  lcd.color(WHITE)
  lcd.drawLine(plotLeft, graphBottom, graphRight, graphBottom)

  -- Feste Balkenbreite wie bei der ersten Widget-Version; die letzten zwölf zählen.
  local barWidth = largeDisplay and 30 or 20
  local gap = 5
  local first = math.max(1, #widget.climbs - 12 + 1)
  local shown = #widget.climbs - first + 1
  local usedWidth = shown * barWidth + math.max(0, shown - 1) * gap
  local x = graphRight - usedWidth

  for index = first, #widget.climbs do
    local climb = widget.climbs[index]
    local barHeight = math.floor(((climb.height - minimumAxis) / axisRange) * graphHeight)
    local barTop = graphBottom - barHeight

    lcd.color(lcd.RGB(0, 170, 255))
    lcd.drawFilledRectangle(x, barTop, barWidth, barHeight)

    lcd.font(largeDisplay and FONT_S or FONT_XS)
    lcd.color(WHITE)
    local text = string.format("%.1f", climb.averageSpeed)
    local textWidth, textHeight = lcd.getTextSize(text)
    lcd.drawText(x + (barWidth - textWidth) / 2, math.max(0, barTop - textHeight - 2), text)
    x = x + barWidth + gap
  end

end

local function wakeup(widget)
  local resetOn = false
  if widget.resetSource then
    local value = widget.resetSource:value()
    resetOn = value ~= nil and value > 0
  end

  if resetOn and not widget.resetActive then
    resetClimbs(widget)
    lcd.invalidate()
  end
  widget.resetActive = resetOn

  local switchOn = false
  if widget.switchSource then
    local value = widget.switchSource:value()
    switchOn = value ~= nil and value > 0
  end

  if switchOn and not widget.climbActive then
    widget.climbActive = true
    widget.startTime = os.clock()
    if not widget.demoMode and widget.heightSource then
      widget.startHeight = widget.heightSource:value()
      widget.highestHeight = widget.startHeight
    end
    lcd.invalidate()
  elseif switchOn and widget.climbActive and not widget.demoMode and widget.heightSource then
    local height = widget.heightSource:value()
    if height ~= nil then
      widget.highestHeight = math.max(widget.highestHeight or height, height)
    end
  elseif not switchOn and widget.climbActive then
    local duration = os.clock() - widget.startTime
    local climbedHeight = nil
    widget.climbActive = false
    widget.startTime = nil

    if widget.demoMode then
      -- Testwert: später kann jederzeit wieder in den Demo-Modus gewechselt werden.
      climbedHeight = math.random(100, 300)
    elseif widget.startHeight ~= nil and widget.highestHeight ~= nil then
      climbedHeight = math.max(0, widget.highestHeight - widget.startHeight)
    end
    widget.startHeight = nil
    widget.highestHeight = nil

    -- Kurze Schaltimpulse oder ungültige Höhenwerte werden nicht gezählt.
    if duration >= 1 then
      if climbedHeight ~= nil then
        addClimb(widget, duration, climbedHeight)
      end
    end
    lcd.invalidate()
  end
end

local function configure(widget)
  local line = form.addLine("Steigflug-Schalter")
  form.addSourceField(line, nil,
    function() return widget.switchSource end,
    function(value) widget.switchSource = value end)

  line = form.addLine("Demo-Modus")
  local demoField = form.addBooleanField(line, form.getFieldSlots(line)[0],
    function() return widget.demoMode end,
    function(value)
      widget.demoMode = value
      if widget.heightField then
        widget.heightField:enable(not value)
      end
    end)

  line = form.addLine("Höhenquelle")
  widget.heightField = form.addSourceField(line, nil,
    function() return widget.heightSource end,
    function(value) widget.heightSource = value end)
  widget.heightField:enable(not widget.demoMode)

  line = form.addLine("HD Bildschirm")
  form.addBooleanField(line, form.getFieldSlots(line)[0],
    function() return widget.largeLayout end,
    function(value) widget.largeLayout = value end)

  line = form.addLine("Reset-Schalter")
  form.addSourceField(line, nil,
    function() return widget.resetSource end,
    function(value) widget.resetSource = value end)
end

local function read(widget)
  widget.switchSource = storage.read("switchSource")
  widget.heightSource = storage.read("heightSource")
  widget.resetSource = storage.read("resetSource")
  local demoMode = storage.read("demoMode")
  if demoMode ~= nil then
    widget.demoMode = demoMode
  end
  local largeLayout = storage.read("largeLayout")
  if largeLayout ~= nil then
    widget.largeLayout = largeLayout
  end
end

local function write(widget)
  storage.write("switchSource", widget.switchSource)
  storage.write("heightSource", widget.heightSource)
  storage.write("resetSource", widget.resetSource)
  storage.write("demoMode", widget.demoMode)
  storage.write("largeLayout", widget.largeLayout)
end

local function init()
  -- Der key muss in Ethos kürzer als 8 Zeichen sein.
  system.registerWidget({
    key = "climb01",
    name = "Steigfluege",
    create = create,
    paint = paint,
    wakeup = wakeup,
    configure = configure,
    read = read,
    write = write
  })
end

return {init = init}
