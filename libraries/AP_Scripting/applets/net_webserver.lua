--[[
   example script to test lua webserver API
--]]

local web = require("web")

web.printf(web.MAV_SEVERITY.INFO, "Starting web server")
web.add_board_status("Arm Status", 'arming:is_armed() and "ARMED" or "DISARMED"')
web.add_board_status("AHRS Location", 'location_string(ahrs:get_location())')
web.add_board_status("GPS Location", 'location_string(ahrs:get_location())')

local function update()
    web.update()
    return update, 5
end

return update, 100
