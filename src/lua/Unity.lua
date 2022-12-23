local version = 'v0.0'
local refresh = 0
local lcdChange = true
local updateValues = false
local readIdState = 0
local sendIdState = 0
local timestamp = 0
local bindmode = 0

local gotFirstResp = false

local binding = false

-- local AirRate = {
--     selected = 1,
--     list = {'------', 'AUTO', '1 kHz', '500 Hz', '250 Hz', '200 Hz', '150 Hz', '100 Hz', '50 Hz', '25 Hz', '4 Hz'},
--     dataId = {0xFF, 0xFE, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 8},
--     elements = 11
-- }
-- local TLMinterval = {
--     selected = 9,
--     list = {
--         'Off', '1:128', '1:64', '1:32', '1:16', '1:8', '1:4', '1:2', '------'
--     },
--     dataId = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0xFF},
--     elements = 9
-- }
local MaxPower915 = {
    selected = 1,
    list = {
        '------', '10 mW', '25 mW', '50 mW', '100 mW', '250 mW'
    },
    dataId = {0xFF, 0x00, 0x01, 0x02, 0x03, 0x04},
    elements = 6
}
local MaxPower2G4 = {
    selected = 1,
    list = {
        '------', '10 mW', '25 mW', '50 mW', '100 mW', '250 mW', '500 mW'
    },
    dataId = {0xFF, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05},
    elements = 7
}
-- local RFfreq = {
--     selected = 4,
--     list = {'915 MHz', '868 MHz', '433 MHz', '------'},
--     dataId = {0x00, 0x01, 0x02, 0xFF},
--     elements = 4
-- }

local selection = {
    selected = 1,
    state = false,
    -- list = {'AirRate', 'TLMinterval', 'MaxPower', 'RFfreq', "Bind"},
    -- elements = 5
    list = {'Power915', 'Power2G4'},
    elements = 2
}

-- returns flags to pass to lcd.drawText for inverted and flashing text
local function getFlags(element)
    if selection.selected ~= element then return 0 end
    if selection.selected == element and selection.state == false then
        return 0 + INVERS
    end
    -- this element is currently selected
    return 0 + INVERS + BLINK
end

local function increase(data)
    if data.selected < data.elements then
        data.selected = data.selected + 1
        --playTone(2000, 50, 0)
    end
    -- if data.selected > data.elements then data.selected = 1 end
end

local function decrease(data)
    if data.selected > 1 then
        data.selected = data.selected - 1
        --playTone(2000, 50, 0)
    end
    -- if data.selected < 1 then data.selected = data.elements end
end

-- local function readId()
--     -- stop sensors
--     if readIdState >= 1 and readIdState <= 15 and getTime() - timestamp > 11 then
--         sportTelemetryPush(sensorIdTx, 0x21, 0xFFFF, 0x80)
--         timestamp = getTime()
--         readIdState = readIdState + 1
--     end
--     -- request/read id
--     if readIdState >= 16 and readIdState <= 30 then
--         if getTime() - timestamp > 11 then
--             sportTelemetryPush(sensorIdTx, 0x30,
--                                sensor.sensorType.dataId[sensor.sensorType
--                                    .selected], 0x01)
--             timestamp = getTime()
--             readIdState = readIdState + 1
--         else
--             local physicalId, primId, dataId, value = sportTelemetryPop() -- frsky/lua: phys_id/sensor id, type/frame_id, sensor_id/data_id
--             if primId == 0x32 and dataId ==
--                 sensor.sensorType.dataId[sensor.sensorType.selected] then
--                 if bit32.band(value, 0xFF) == 1 then
--                     sensor.sensorId.selected = ((value - 1) / 256) + 1
--                     readIdState = 0
--                     lcdChange = true
--                 end
--             end
--         end
--     end
--     if readIdState == 31 then
--         readIdState = 0
--         lcdChange = true
--     end
-- end


--[[

It's unclear how the telemetry push/pop system works. We don't always seem to get
a response to a single push event. Can multiple responses be stacked up? Do they timeout?

If there are multiple responses we typically want the newest one, so this method
will keep reading until it gets a nil response, discarding the older data. A maximum number
of reads is used to defend against the possibility of this function running for an extended
period.

]]--

local function processResp()
    local tries=0
    local MAX_TRIES=5

    while tries<MAX_TRIES
    do
        local command, data = crossfireTelemetryPop()
        if (data == nil) then
            return
        else
            if (command == 0x2D) and (data[1] == 0xEA) and (data[2] == 0xEE) then

                -- Need to do unsigned to signed conversion
                if (data[3] > 127) then
                    MaxPower915.selected = data[3] - 256
                else
                    MaxPower915.selected = data[3]
                end

                if (data[4] > 127) then
                    MaxPower2G4.selected = data[4] - 256
                else
                    MaxPower2G4.selected = data[4]
                end

                gotFirstResp = true -- detect when first contact is made with TX module 

            end
        end -- data == nill
	    tries = tries+1
    end -- while loop
end


local function refreshHorus()
    lcd.clear()
    lcd.drawText(1, 1, 'Unity CFG v0.0', INVERS)    -- use the version string
    -- lcd.drawText(1, 25, 'Pkt. Rate', 0)
    -- lcd.drawText(1, 45, 'TLM Ratio', 0)
    lcd.drawText(1, 65, 'Max Power 915', 0)
    lcd.drawText(1, 85, 'Max Power 2G4', 0)
    -- lcd.drawText(1, 85, 'RF Freq', 0)

    -- lcd.drawText(100, 25, AirRate.list[AirRate.selected], getFlags(1))
    -- lcd.drawText(100, 45, TLMinterval.list[TLMinterval.selected], getFlags(2))
    lcd.drawText(150, 65, MaxPower915.selected, getFlags(1))
    lcd.drawText(150, 85, MaxPower2G4.selected, getFlags(2))
    -- lcd.drawText(100, 85, RFfreq.list[RFfreq.selected], getFlags(4))

    -- lcd.drawText(20, 110, '[Bind]', getFlags(5) + SMLSIZE)
    -- lcd.drawText(60, 110, '[Web Server]', getFlags(6) + SMLSIZE)

    -- if selection.selected == 5 then
    --     if selection.state == true then
    --         lcd.drawText(30, 110, 'Press [ENTER] to stop', MEDSIZE)
	-- 		if (bindmode == 0) then
				-- crossfireTelemetryPush(0x2D, {0xEE, 0xEA, 0xFF, 0x01})
	-- 		end
    --     end
    -- end

    lcdChange = false

end

-- local function refreshTaranis()
--     lcd.clear()
--     lcd.drawScreenTitle('ExpressLRS CFG ' .. version, 1, 1)
--     lcd.drawText(1, 11, 'Pkt. Rate', 0)
--     lcd.drawText(1, 21, 'TLM Ratio', 0)
--     lcd.drawText(1, 31, 'Set Power', 0)
--     lcd.drawText(1, 41, 'RF Freq', 0)

--     lcd.drawText(60, 11, AirRate.list[AirRate.selected], getFlags(1))
--     lcd.drawText(60, 21, TLMinterval.list[TLMinterval.selected], getFlags(2))
--     lcd.drawText(60, 31, MaxPower.list[MaxPower.selected], getFlags(3))
--     lcd.drawText(60, 41, RFfreq.list[RFfreq.selected], getFlags(4))

--     lcd.drawText(18, 54, '[Bind]', getFlags(5) + SMLSIZE)
--     lcd.drawText(55, 54, '[Web Server]', getFlags(6) + SMLSIZE)

--     if selection.selected == 5 then
--         if selection.state == true then
--             lcd.drawText(7, 53, 'Press [ENTER] to stop', MEDSIZE)
-- 			if (bindmode == 0) then
-- 				crossfireTelemetryPush(0x2D, {0xEE, 0xEA, 0xFF, 0x01})
-- 			end
--         end
-- 		if (selection.state == false) and (bindmode == 1) then
-- 			crossfireTelemetryPush(0x2D, {0xEE, 0xEA, 0xFF, 0x01})
-- 			bindmode = 0
-- 		end
--     end

--     lcdChange = false

-- end

-- redraw the screen
local function refreshLCD()

    -- if LCD_W == 480 then
        refreshHorus()
    -- else
        -- refreshTaranis()
    -- end

end

local function init_func()
    -- first push so that we get the current values. Didn't seem to work.
    crossfireTelemetryPush(0x2D, {0xEE, 0xEA, 0x00, 0x00})
	--crossfireTelemetryPush(0x2D, {0xEE, 0xEA, 0x00, 0x00})
	--crossfireTelemetryPush(0x2D, {0xEE, 0xEA, 0x00, 0x00})
    processResp()
	--if LCD_W == 480 then
    --    refreshHorus()
   -- else
   --     refreshTaranis()
  --  end
end

local function bg_func(event)
    --if refresh < 25 then 
        --refresh = refresh + 1 
    --end
end

--[[
  Called at (unspecified) intervals when the script is running and the screen is visible

  Handles key presses and sends state changes to the tx module.

  Basic strategy:
    read any outstanding telemetry data
    process the event, sending a telemetryPush if necessary
    if there was no push due to events, send the void push to ensure current values are sent for next iteration
    redraw the display

]]--
local function run_func(event)

    local pushed = false
	
	processResp() -- first check if we have data from the module
	
	if gotFirstResp == false then
		crossfireTelemetryPush(0x2D, {0xEE, 0xEA, 0x00, 0x00}) -- ping until we get a resp
	end

    -- now process key events
    if event == EVT_ROT_LEFT or 
       event == EVT_MINUS_BREAK or 
       event == EVT_DOWN_BREAK then
        if selection.state == false then
            decrease(selection)
			crossfireTelemetryPush(0x2D, {0xEE, 0xEA, 0x00, 0x00})
        else
            if selection.selected == 1 then
                -- 915 power
                crossfireTelemetryPush(0x2D, {0xEE, 0xEA, 0x01, 0x00})
                pushed = true
            elseif selection.selected == 2 then
                -- 2G4 power
                crossfireTelemetryPush(0x2D, {0xEE, 0xEA, 0x02, 0x00})
		        pushed = true
            -- elseif selection.selected == 3 then
            --     -- MaxPower
            --     crossfireTelemetryPush(0x2D, {0xEE, 0xEA, 0x03, 0x00})
		    --     pushed = true
            -- elseif selection.selected == 4 then
            --     -- RFFreq
            --     crossfireTelemetryPush(0x2D, {0xEE, 0xEA, 0x04, 0x00})
		    --     pushed = true
            end
	    end -- selection.state

    elseif event == EVT_ROT_RIGHT or 
           event == EVT_PLUS_BREAK or 
	       event == EVT_UP_BREAK then
        if selection.state == false then
            increase(selection)
			crossfireTelemetryPush(0x2D, {0xEE, 0xEA, 0x00, 0x00})
        else
            if selection.selected == 1 then
                -- 915 power
                crossfireTelemetryPush(0x2D, {0xEE, 0xEA, 0x01, 0x01})
                pushed = true
            elseif selection.selected == 2 then
                -- 2G4 power
                crossfireTelemetryPush(0x2D, {0xEE, 0xEA, 0x02, 0x01})
                pushed = true
            -- elseif selection.selected == 3 then
            --     -- MaxPower
            --     crossfireTelemetryPush(0x2D, {0xEE, 0xEA, 0x03, 0x01})
            --     pushed = true
            -- elseif selection.selected == 4 then
            --     -- RFFreq
            --     crossfireTelemetryPush(0x2D, {0xEE, 0xEA, 0x04, 0x01})
		    --     pushed = true
            end -- selection.selected
	    end -- selection.state

    elseif event == EVT_ENTER_BREAK then
        selection.state = not selection.state

    elseif event == EVT_EXIT_BREAK and selection.state then
        -- I was hoping to find the T16 RTN button as an alternate way of deselecting
	    -- a field, but no luck so far
        selection.state = false
    end

    if not pushed then
        -- ensure we get up to date values from the module for next time
        --crossfireTelemetryPush(0x2D, {0xEE, 0xEA, 0x00, 0x00})
    end

    refreshLCD()

    return 0
end

return {run = run_func, background = bg_func, init = init_func}
