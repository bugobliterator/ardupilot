-- Generated bindings over the M4's RPC, one kind of binding per step. Steps run
-- two seconds apart, so one statistics window holds few methods. Every remote
-- call is counted here, the refused ones and the report's texts included; the
-- tests check that no SRPC row logs more than these counts and that all binding
-- rows, m4:other included, add up to the total. Vector2f, Vector3f, Quaternion and
-- Location run on the M4 itself (fields, operators, constructors, math): those calls
-- are not counted, so one that went remote would break that total. Location's five
-- state rows (change_alt_frame, get_alt_m, get_vector_from_origin_NEU*) stay remote
-- and are counted. The first step waits until the load-time calls have their own
-- window. Runs unchanged in-process (no M4 needed for the checks).

local counts = {}
local order = {}

local function count(name, n)
  if not counts[name] then
    counts[name] = 0
    order[#order + 1] = name
  end
  counts[name] = counts[name] + (n or 1)
end

local function check(condition, text)
  if not condition then
    error("M4 bindings: " .. text, 2)
  end
end

local function expect_error(fn, tail)
  local ok, err = pcall(fn)
  check(not ok, "no error for " .. tail)
  check(string.find(tostring(err), tail, 1, true) ~= nil, "got '" .. tostring(err) .. "'")
end

local steps = {
  function()   -- constructors, fields and methods of a local type (Location), on the M4
    local a = Location()
    local b = Location()
    a:lat(-353632621)
    a:lng(1491652374)
    a:alt(58409)
    b:lat(-353632621)
    b:lng(1491652374)
    b:offset(100, 0)
    local d = a:get_distance(b)
    check(math.abs(d - 100) < 0.5, "distance " .. tostring(d))
    check(b:lat() > a:lat(), "offset north")
    local c = a   -- a reference: a write through c is seen through a
    c:alt(100)
    check(a:alt() == 100, "reference semantics")
  end,
  function()   -- operators and copies of a local type (Vector3f), on the M4
    local v = Vector3f()
    local w = Vector3f()
    v:x(3)
    v:y(4)
    w:z(12)
    local s = v + w
    check(math.abs(s:length() - 13) < 1e-4, "length")
    local u = s:copy()
    u:normalize()
  end,
  function()   -- a nullable return and a parameter write (AP_Param)
    check(param:get("SCR_ENABLE") == 1, "SCR_ENABLE")
    check(param:get("NO_SUCH_PARAM") == nil, "missing parameter")
    count("param:get", 2)
    check(param:set("SCR_USER2", 5), "param:set")
    count("param:set", 1)
    check(param:get("SCR_USER2") == 5, "SCR_USER2")
    count("param:get", 1)
  end,
  function()   -- scheduler-semaphore methods; a mode change runs mode code on the M7's host thread
    local mode = vehicle:get_mode()
    count("vehicle:get_mode", 1)
    check(arming:is_armed() == false, "armed")
    count("arming:is_armed", 1)
    check(vehicle:set_mode(2), "set_mode ALT_HOLD")
    check(vehicle:set_mode(mode), "set_mode back")
    count("vehicle:set_mode", 2)
  end,
  function()   -- enums (no call) and handles (RC_Channel)
    check(gps.GPS_OK_FIX_3D == 3, "gps.GPS_OK_FIX_3D")
    local ch = rc:get_channel(1)
    count("rc:get_channel", 1)
    check(ch ~= nil, "rc:get_channel(1)")
    local norm = ch:norm_input()
    count("RC_Channel:norm_input", 1)
    check(norm >= -1 and norm <= 1, "norm_input " .. tostring(norm))
  end,
  function()   -- local boxed numerics and the M7's clock
    local t0 = millis()
    local u0 = micros()
    count("global:millis", 1)
    count("global:micros", 1)
    check((millis() - t0):toint() >= 0, "millis")
    count("global:millis", 1)
    check(tostring(uint32_t(7) + 3) == "10", "uint32_t")
    check(u0:tofloat() >= 0, "micros")
    -- uint64_t, whose text the M4 formats by hand
    local epoch = gps:time_epoch_usec(0)
    count("gps:time_epoch_usec", 1)
    check(epoch + 1 > epoch, "time_epoch_usec")
    check(tostring(uint64_t(1, 0)) == "4294967296", "uint64_t text")
    check(tostring(uint64_t(0)) == "0", "uint64_t zero")
    local max32 = uint32_t(0) - 1
    check(tostring(uint64_t(max32, max32)) == "18446744073709551615", "uint64_t max")
    local hi, lo = uint64_t(1, 2):split()
    check(hi:toint() == 1 and lo:toint() == 2, "uint64_t split")
    check(uint64_t(1, 0):toint() == nil, "uint64_t toint")
  end,
  function()   -- argument errors keep their in-process texts; other vehicles' bindings are nil
    expect_error(function() gcs:send_text("x", "y") end, "bad argument #1 to 'send_text' (number expected, got string)")
    expect_error(function() gcs:send_text(9, "y") end, "bad argument #1 to 'send_text' (out of range)")
    expect_error(function() gcs:send_text(6) end, "bad argument #1 to 'send_text' (too few arguments)")
    count("gcs:send_text", 3)
    -- more values than a call frame holds: refused on the M4, with the in-process text
    expect_error(function() gcs:send_text(6, "a", 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15) end,
                 "bad argument #17 to 'send_text' (too many arguments)")
    expect_error(function() ahrs:set_home(Vector3f()) end, "bad argument #1 to 'set_home' (Location expected, got Vector3f)")
    count("ahrs:set_home", 1)
    check(quadplane == nil and sub == nil, "bindings of other vehicles")
  end,
  function()   -- values the M7 made come back intact; a remote row writes into a local object
    local home = ahrs:get_home()   -- now a Location on the M4
    count("ahrs:get_home", 1)
    check(home ~= nil, "ahrs:get_home")
    if not ahrs:home_is_set() then
      -- no GPS fix yet (SITL) or none at all (Renode): a home of the test's own
      home:lat(-353632621)
      home:lng(1491652374)
      home:alt(58409)
    end
    count("ahrs:home_is_set", 1)
    -- decoded on the M7 with every field, loiter_ccw included (no 'M4: invalid Location'),
    -- and read back unchanged
    check(ahrs:set_home(home), "ahrs:set_home")
    count("ahrs:set_home", 1)
    local again = ahrs:get_home()
    count("ahrs:get_home", 1)
    check(again:lat() == home:lat() and again:lng() == home:lng() and again:alt() == home:alt() and
          again:relative_alt() == 0, "home round trip")
    -- a mission item written and read back; item 0 is home, so it goes after it
    local before = mission:num_commands()
    count("mission:num_commands", 1)
    local wp = mavlink_mission_item_int_t()
    count("mavlink_mission_item_int_t:new", 1)
    wp:command(16)   -- MAV_CMD_NAV_WAYPOINT
    wp:frame(3)      -- MAV_FRAME_GLOBAL_RELATIVE_ALT
    wp:x(-353632621)
    wp:y(1491652374)
    wp:z(25.5)
    count("mavlink_mission_item_int_t:command", 1)
    count("mavlink_mission_item_int_t:frame", 1)
    count("mavlink_mission_item_int_t:x", 1)
    count("mavlink_mission_item_int_t:y", 1)
    count("mavlink_mission_item_int_t:z", 1)
    if before == 0 then
      check(mission:set_item(0, wp), "mission:set_item home")   -- writes home to storage
      count("mission:set_item", 1)
    end
    local index = mission:num_commands()
    count("mission:num_commands", 1)
    check(mission:set_item(index, wp), "mission:set_item")
    count("mission:set_item", 1)
    local back = mission:get_item(index)
    count("mission:get_item", 1)
    check(back ~= nil, "mission:get_item")
    local command, frame, x, y, z = back:command(), back:frame(), back:x(), back:y(), back:z()
    count("mavlink_mission_item_int_t:command", 1)
    count("mavlink_mission_item_int_t:frame", 1)
    count("mavlink_mission_item_int_t:x", 1)
    count("mavlink_mission_item_int_t:y", 1)
    count("mavlink_mission_item_int_t:z", 1)
    check(command == 16 and frame == 3 and x == -353632621 and y == 1491652374, "mission item")
    check(math.abs(z - 25.5) < 0.01, "mission item z " .. tostring(z))   -- stored in cm
    if before <= 1 then
      check(mission:clear(), "mission:clear")   -- the mission was empty before
      count("mission:clear", 1)
    end
    local l = Location()
    l:lat(-353632621)
    l:lng(1491652374)
    l:alt(1000)
    l:relative_alt(true)
    local alias = l   -- a reference: the write-back shows through it
    local changed = l:change_alt_frame(0)   -- ABOVE_HOME to ABSOLUTE: reads home on the M7
    count("Location:change_alt_frame", 1)
    -- home is set above: the write-back decoded every field (a flag field reads as 0 or 1)
    check(changed, "change_alt_frame with home set")
    check(alias:lat() == -353632621 and alias:lng() == 1491652374 and alias:terrain_alt() == 0,
          "change_alt_frame kept the position")
    check(alias:relative_alt() == 0, "change_alt_frame wrote back")
    check(alias:alt() == 1000 + again:alt(), "absolute altitude " .. tostring(alias:alt()))
  end,
  function()   -- an origin for the GUIDED step
    if not ahrs:get_origin() then
      -- no GPS (Renode): an origin of its own, so the target converts to a position. A step
      -- before the target: the EKF takes the origin on a later update
      local origin = Location()
      origin:lat(-353632621)
      origin:lng(1491652374)
      origin:alt(58409)
      check(ahrs:set_origin(origin), "ahrs:set_origin")
      count("ahrs:set_origin", 1)
    end
    count("ahrs:get_origin", 1)
  end,
  function()   -- the deepest mode code on the M7's host thread: a GUIDED target (set-target-location.lua)
    local target = Location()
    target:lat(-353632621)
    target:lng(1491652374)
    target:alt(59409)
    target:offset(20, 10)
    local mode = vehicle:get_mode()
    count("vehicle:get_mode", 1)
    check(vehicle:set_mode(4), "set_mode GUIDED")
    check(vehicle:set_target_location(target), "vehicle:set_target_location")
    count("vehicle:set_target_location", 1)
    check(vehicle:set_mode(mode), "set_mode back")
    count("vehicle:set_mode", 2)
  end,
}

local step = 1

local function report()
  count("gcs:send_text", 0)
  count("gcs:send_text", #order + 2)   -- the count lines, the total and the pass
  local total = 0
  for _, name in ipairs(order) do
    total = total + counts[name]
  end
  for _, name in ipairs(order) do
    gcs:send_text(6, string.format("M4 count %s %d", name, counts[name]))
  end
  gcs:send_text(6, string.format("M4 total %d", total))
  gcs:send_text(6, "M4 bindings passed")
end

local function update()
  if step <= #steps then
    steps[step]()
    step = step + 1
    return update, 2000
  end
  report()
end

return update, 3000
