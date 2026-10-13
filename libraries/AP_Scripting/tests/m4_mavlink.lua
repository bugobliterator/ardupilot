-- MAVLink, logger and singleton manual bindings of SITL M4 scripts (ScriptingM4MAVLink,
-- arducopter.py). M4 only: its cap, refusal and quota stages check bounds in-process has not.
-- Each stage says what it would see without the code it targets.
-- Needs OSD_TYPE 3 (MSP: an OSD backend without SFML), RNGFND1_TYPE 36 (Lua) and LOG_DISARMED 1.
-- The quota stages wait for SCR_USER4 to reach their number: the autotest reads M4ST.QHit around
-- each, so each class's refusals are counted alone. Each prints its flood size and the time it
-- took, from which the autotest works out the least hits its quota must give

local NAMED_VALUE_FLOAT_ID, HEARTBEAT_ID = 251, 0
local MAV_CMD_DO_SET_HOME, MAV_CMD_SCRIPTING = 179, 42701
local MAV_RESULT_ACCEPTED, MAV_RESULT_DENIED = 0, 2
local LABELS15 = "A,B,C,D,E,F,G,H,I,J,K,L,M,N,O"
local FORMAT15, UNITS15 = string.rep("f", 15), string.rep("-", 15)
local FILE_CONTENT_QUEUE_MAX = 40   -- AP_ScriptingM4_Remote.h
local BOUND_FILES = FILE_CONTENT_QUEUE_MAX + 5
local HEAD_CUSHION_KB, HEAD_KB_PER_FILE = 48, 8

local failures = 0
local function check(ok, name)
  if not ok then
    failures = failures + 1
    gcs:send_text(3, "M4 mavlink FAIL " .. name)
  end
end

-- the error text of a call, nil if it succeeded
local function error_of(f, ...)
  local ok, err = pcall(f, ...)
  if ok then
    return nil
  end
  return tostring(err)
end

local function has(text, part)
  return text ~= nil and string.find(text, part, 1, true) ~= nil
end

-- the scripts directory as the M7 names it: the package.path entry that is not ROMFS
local function scripts_dir()
  for template in string.gmatch(package.path, "[^;]+") do
    local dir = string.match(template, "^(.-)/modules/%?%.lua$")
    if dir and string.sub(dir, 1, 1) ~= "@" then
      return dir
    end
  end
  return nil
end

local function write_file(path, content, mode)
  local f = io.open(path, mode or "w")
  check(f ~= nil, "open " .. path)
  if f then
    f:write(content)
    f:close()
  end
end

-- a NAMED_VALUE_FLOAT payload: time_boot_ms, value, name[10]
local function named_value(name, value)
  return string.pack("<I4fc10", millis():toint(), value, name)
end

local gate = Parameter("SCR_USER4")
local tries = 0   -- updates a waiting stage has waited
local interval = 1000

local function wide_calls()
  -- 15 fields with units and multipliers: 21 values, as a method and as a dot call. With
  -- MAX_VALUES 16 both are refused on the M4 with "too many arguments"
  check(error_of(logger.write, logger, "M4LW", LABELS15, FORMAT15, UNITS15, UNITS15,
                 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15) == nil, "15 fields with units")
  check(error_of(logger.write, "M4LW", LABELS15, FORMAT15, UNITS15, UNITS15,
                 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15) == nil, "15 fields by dot")
  -- 22 values: refused on the M4 with the in-process argument text
  local err = error_of(logger.write, "M4LW", LABELS15, FORMAT15, UNITS15, UNITS15,
                       1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16)
  check(has(err, "#22 ") and has(err, "too many arguments"), "22 values '" .. tostring(err) .. "'")
end

local function logger_formats()
  -- a format character logger:write does not write is named before the logger takes a message
  -- type for the name; without the check SITL's Write_calc_msg_len() panics the vehicle on 'x'
  check(has(error_of(logger.write, logger, "M4LX", "A", "x", 1), "x unsupported format"), "format x")
  check(has(error_of(logger.write, logger, "M4LD", "A", "d", 1), "d unsupported format"), "format d")
  check(has(error_of(logger.write, logger, "M4LZ", "A", "Z", string.rep("z", 65)), "too long for Z format"),
        "a 65-character Z value")
  check(has(error_of(logger.write, logger, "M4LB", string.rep("a", 58), "f", 1), "labels must be less than 58"),
        "58-character labels")
  check(has(error_of(logger.write, logger, "M4LF", LABELS15 .. ",P", string.rep("f", 16),
                     1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16),
            "format must be less than 15"), "16 fields")
  -- a 267-byte row: refused before the logger allocates for the name, 1000 times. Without the
  -- length check each call leaks the logger's copy of the strings ("could not map message type")
  local refused = 0
  for _ = 1, 1000 do
    if has(error_of(logger.write, logger, "M4LC", "a,b,c,d", "ZZZZ", "a", "b", "c", "d"), "unknown format") then
      refused = refused + 1
    end
  end
  check(refused == 1000, "ZZZZ refused " .. refused .. " of 1000")
end

local function send_lengths()
  -- the packet is the call's copy: a 300-byte one is cut to the message's 18 bytes, a 1-byte one
  -- padded with zeros, as in-process. The autotest receives "m4long" = 1.5, and from the 1-byte
  -- packet time_boot_ms 77 with value 0 and an empty name
  local long = named_value("m4long", 1.5) .. string.rep("\255", 282)
  check(#long == 300, "a 300-byte packet")
  check(mavlink.send_chan(0, NAMED_VALUE_FLOAT_ID, long) == true, "send a 300-byte packet")
  check(mavlink.send_chan(0, NAMED_VALUE_FLOAT_ID, "\77") == true, "send a 1-byte packet")
  check(has(error_of(mavlink.send_chan, 0, 99999, "x"), "Unknown MAVLink message ID (99999)"), "unknown id")
  check(has(error_of(mavlink.send_chan, 0, NAMED_VALUE_FLOAT_ID, named_value("t", 1), 7),
            "Message has no target system field"), "a target on a message without one")
  check(has(error_of(mavlink.send_chan, 99, NAMED_VALUE_FLOAT_ID, named_value("t", 1)), "out of range"),
        "channel 99")
end

local function receive_setup()
  -- dot calls: without ROW_DOT_ACCESS the M7 gets nil for 10 and the call fails the range check
  check(error_of(mavlink.init, 10, 2) == nil, "mavlink.init(10, 2)")
  check(mavlink.register_rx_msgid(HEARTBEAT_ID) == true, "register HEARTBEAT")
  check(mavlink.register_rx_msgid(HEARTBEAT_ID) == false, "register HEARTBEAT again")
end

local function receive()
  -- the GCS's heartbeats reach the M7's queue; the M4 polls it
  local msg, chan, timestamp = mavlink.receive_chan()
  if msg == nil then
    tries = tries + 1
    check(tries < 20, "no HEARTBEAT in 20 s")
    return tries >= 20
  end
  check(#msg >= 291, "a message of " .. #msg .. " bytes")
  check(math.type(chan) == "integer", "the channel")
  check(timestamp:toint() > 0, "the timestamp")
  return true
end

local function block_cap()
  -- MAV_CMD_SCRIPTING is never blocked, asked first so that the exemption, not the cap, refuses
  -- it: without the exemption it is true. Then 32 blocks a session; the 33rd is refused, and a
  -- repeat is no new block. Without the cap the 33rd is true
  check(mavlink.block_command(MAV_CMD_SCRIPTING) == false, "block MAV_CMD_SCRIPTING")
  for id = 31000, 31031 do
    check(mavlink.block_command(id) == true, "block " .. id)
  end
  check(mavlink.block_command(31032) == false, "the 33rd block")
  check(mavlink.block_command(31000) == true, "a repeated block")
end

local function singletons()
  if not ahrs:home_is_set() then
    tries = tries + 1
    check(tries < 120, "no home in 120 s")
    return tries >= 120
  end
  check(error_of(gps.inject_data, gps, "abc") == nil, "inject_data")
  check(vehicle:set_target_velocity_NED(Vector3f()) == false, "set_target_velocity_NED outside GUIDED")
  check(type(SRV_Channels:get_safety_state()) == "boolean", "get_safety_state")
  check(DroneCAN_get_FlexDebug(0, 125, 1, 0) == nil, "FlexDebug without a node")
  local backend = rangefinder:get_backend(0)
  check(backend ~= nil, "the Lua rangefinder backend (RNGFND1_TYPE 36)")
  if backend then
    check(backend:handle_script_msg(4.5) == true, "handle_script_msg(distance)")
    local state = RangeFinder_State()
    state:status(4)   -- RangeFinder::Status::Good
    state:distance(3.0)
    state:last_reading(millis():toint())
    check(backend:handle_script_msg(state) == true, "handle_script_msg(state)")
    check(has(error_of(backend.handle_script_msg, backend, "x"), "number expected"), "handle_script_msg('x')")
  end
  -- the table's fields cross as values: home moves to x, y. Without the wrapper the table cannot
  -- cross ("M4: call arguments too large" or an argument error)
  local home = ahrs:get_home()
  local lat, lng = home:lat() + 1000, home:lng() + 1000
  local result = gcs:run_command_int(MAV_CMD_DO_SET_HOME,
                                     {p1 = 0, x = lat, y = lng, z = home:alt() / 100.0, frame = 0})
  check(result == MAV_RESULT_ACCEPTED, "DO_SET_HOME result " .. tostring(result))
  local moved = ahrs:get_home()
  check(moved:lat() == lat and moved:lng() == lng, "home moved to x, y")
  check(gcs:run_command_int(MAV_CMD_DO_SET_HOME, 5) == nil, "no table: no result")
  check(has(error_of(gcs.run_command_int, gcs, MAV_CMD_DO_SET_HOME), "too few arguments"), "one argument")
  return true
end

-- gcs:run_command_int refusals: each command would hold the M7 host thread for seconds or crash,
-- erase or reflash the vehicle. Each is refused at once with MAV_RESULT_DENIED; without the
-- refusal the command runs (a gyro calibration outlives the M4's 1 s call timeout, the
-- failure-creation triple with p4 0 gives UNSUPPORTED)
local function refusal(command, fields)
  return function()
    local start = millis()
    local result = gcs:run_command_int(command, fields)
    local elapsed = (millis() - start):toint()
    check(result == MAV_RESULT_DENIED, "command " .. command .. " result " .. tostring(result))
    check(elapsed < 900, "command " .. command .. " took " .. elapsed .. " ms")
  end
end

local function file_content()
  -- a file this script wrote, queued for FILE logging through the M7 replacement: the autotest
  -- finds its FILE rows
  local dir = scripts_dir()
  check(dir ~= nil, "the scripts directory")
  local path = dir .. "/m4_fc.txt"
  write_file(path, "m4 file content\n")
  logger:log_file_content(path)
end

local function file_paths()
  -- two files the M7 opens but the replacement refuses: the live log's directory by its alias
  -- (LASTLOG.TXT, which SITL writes there) and this script's own file through a path that
  -- climbs above the directory it starts from. The autotest finds no FILE rows of either.
  -- Without path_in_log_directory() the first is logged, without check_script_path() the second
  local dir = scripts_dir()
  local f = io.open(dir .. "/m4_cwd.txt", "r")   -- the SITL working directory's name, from the autotest
  check(f ~= nil, "m4_cwd.txt")
  if not f then
    return
  end
  local cwd = f:read("l")
  f:close()
  write_file(dir .. "/m4_pc.txt", "m4 path check\n")
  local climbing
  if string.sub(dir, 1, 1) == "/" then
    climbing = "/.." .. dir .. "/m4_pc.txt"
  else
    climbing = "../" .. cwd .. "/" .. string.gsub(dir, "^%./", "") .. "/m4_pc.txt"
  end
  check(io.open(climbing, "r") == nil, "io.open('" .. climbing .. "') opened")
  logger:log_file_content("@MAV_LOG/LASTLOG.TXT")
  logger:log_file_content(climbing)
end

-- the file queue bound: a head file that outlasts the stage (written ahead of the logger, which
-- logs 64 bytes per 10 ms in SITL), then one file a second (the FileLog quota) up to 45. The queue
-- takes the first 39 beside the head; without the bound all 45 are logged
local bound = { written = 0, queued = 0 }
local function file_bound()
  local dir = scripts_dir()
  local head = dir .. "/m4_fh.txt"
  local kilobyte = string.rep("h", 1023) .. "\n"
  if bound.written < HEAD_CUSHION_KB then
    -- 16 KB a second: 32 FileIo writes of the 50 a second
    write_file(head, string.rep(kilobyte, 16), bound.written == 0 and "w" or "a")
    bound.written = bound.written + 16
    interval = 1000
    return false
  end
  if bound.queued == 0 then
    logger:log_file_content(head)
  end
  if bound.queued < BOUND_FILES then
    write_file(head, string.rep(kilobyte, HEAD_KB_PER_FILE), "a")
    bound.queued = bound.queued + 1
    local name = string.format("%s/m4_q%02d.txt", dir, bound.queued)
    write_file(name, "q\n")
    logger:log_file_content(name)
    interval = 1100   -- under the FileLog refill of one a second
    return false
  end
  interval = 1000
  gcs:send_text(6, "M4 mavlink file bound queued " .. bound.queued)
  return true
end

-- the quota stages: each waits for its gate, then floods its class in one run and prints its
-- size and how long it took
local function gated(number, flood)
  return function()
    if gate:get() < number then
      return false
    end
    flood()
    return true
  end
end

-- the run's time on the M4 and on the vehicle, which refills the quotas: the time of the GPS
-- driver's last message, a vehicle clock this script reads to within a message period
local function timed(label, count, call)
  local start, vehicle_start = millis(), gps:last_message_time_ms(0)
  local extra = call(count)
  local elapsed, vehicle_elapsed = (millis() - start):toint(), (gps:last_message_time_ms(0) - vehicle_start):toint()
  gcs:send_text(6, string.format("M4 mavlink %s n %d ms %d vms %d%s", label, count, elapsed, vehicle_elapsed,
                                 extra or ""))
end

local function logger_quota()
  -- 3000 rows in one run: 500 pass at once, then 500 a second; the autotest counts the M4LQ rows.
  -- Without the quota all 3000 are written
  timed("log", 3000, function(n)
    for i = 1, n do
      logger:write("M4LQ", "N", "i", i)
    end
  end)
end

local function osd_write()
  -- 1500 writes in one run: 300 pass at once, then 300 a second. Without the Osd class none is refused
  check(osd ~= nil, "an OSD backend (OSD_TYPE 3)")
  timed("osdw", 1500, function(n)
    for _ = 1, n do
      osd:write(1, 1, "x")
    end
  end)
end

local function osd_frame()
  -- 300 whole-screen draws in one run: 40 pass at once, then 40 a second. Without the OsdFrame
  -- class none is refused
  timed("osdf", 300, function(n)
    for _ = 1, n do
      osd:draw_screen()
    end
  end)
end

local function send_quota()
  -- 1000 sends in one run: 200 go out at once, then 200 a second; the rest return false.
  -- Without the MavTx class only a full link refuses
  timed("tx", 1000, function(n)
    local sent = 0
    for i = 1, n do
      if mavlink.send_chan(0, NAMED_VALUE_FLOAT_ID, named_value("m4q", i)) then
        sent = sent + 1
      end
    end
    return " sent " .. sent
  end)
end

local function wind_estimate()
  -- last: its once-per-boot warning is a "Lua:" text and the prearm failure, as in-process
  local wind = ahrs:wind_estimate()
  check(wind ~= nil and type(wind:x()) == "number", "wind_estimate")
end

local stages = {
  { "wide_calls", wide_calls }, { "logger_formats", logger_formats }, { "send_lengths", send_lengths },
  { "receive_setup", receive_setup }, { "receive", receive }, { "block_cap", block_cap },
  { "singletons", singletons },
  { "refuse_calibration", refusal(241, {p1 = 1}) },                         -- PREFLIGHT_CALIBRATION
  { "refuse_storage_erase", refusal(245, {p1 = 2}) },                       -- PREFLIGHT_STORAGE
  { "refuse_storage_format", refusal(526, {p1 = 1, p2 = 1}) },              -- STORAGE_FORMAT
  { "refuse_crash_trigger", refusal(246, {p1 = 42, p2 = 24, p3 = 71, p4 = 0}) },   -- REBOOT_SHUTDOWN
  { "refuse_flash_bootloader", refusal(42650, {x = 290876}) },              -- FLASH_BOOTLOADER
  { "refuse_debug_trap", refusal(42700, {p1 = 32451}) },                    -- DEBUG_TRAP
  { "file_content", file_content }, { "file_paths", file_paths }, { "file_bound", file_bound },
  { "logger_quota", gated(1, logger_quota) }, { "osd_write", gated(2, osd_write) },
  { "osd_frame", gated(3, osd_frame) }, { "send_quota", gated(4, send_quota) },
  { "wind_estimate", wind_estimate },
}
local stage = 1
local announced = 0

local function update()
  if stage <= #stages then
    if announced < stage then
      -- ScriptingM4MAVLink waits for each: a stage that stalls the M7 names itself by the text missing
      gcs:send_text(6, "M4 mavlink stage " .. stages[stage][1])
      announced = stage
    end
    local ok, done = pcall(stages[stage][2])
    if not ok then
      gcs:send_text(3, string.sub("M4 mavlink FAILED: " .. tostring(done), 1, 50))
      return
    end
    if done ~= false then
      stage = stage + 1
      tries = 0
    end
    return update, interval
  end
  if failures == 0 then
    gcs:send_text(6, "M4 mavlink passed")
  else
    gcs:send_text(3, "M4 mavlink failed " .. failures)
  end
end

return update, 1000
