-- Generated bindings over the M4's RPC, one kind of binding per step. Steps run
-- two seconds apart, so one statistics window holds few methods and the SRPC
-- sums are exact. Every remote call is counted here; the tests compare these
-- counts with the SRPC rows. Runs unchanged in-process (no M4 needed for the checks).

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
  function()   -- constructors, fields and methods of a blob (Location)
    local a = Location()
    local b = Location()
    count("Location:new", 2)
    a:lat(-353632621)
    a:lng(1491652374)
    a:alt(58409)
    b:lat(-353632621)
    b:lng(1491652374)
    count("Location:lat", 2)
    count("Location:lng", 2)
    count("Location:alt", 1)
    b:offset(100, 0)
    count("Location:offset", 1)
    local d = a:get_distance(b)
    count("Location:get_distance", 1)
    check(math.abs(d - 100) < 0.5, "distance " .. tostring(d))
    check(b:lat() > a:lat(), "offset north")
    count("Location:lat", 2)
    local c = a   -- a reference: a write through c is seen through a
    c:alt(100)
    count("Location:alt", 1)
    check(a:alt() == 100, "reference semantics")
    count("Location:alt", 1)
  end,
  function()   -- operators and copies of a blob (Vector3f)
    local v = Vector3f()
    local w = Vector3f()
    count("Vector3f:new", 2)
    v:x(3)
    v:y(4)
    w:z(12)
    count("Vector3f:x", 1)
    count("Vector3f:y", 1)
    count("Vector3f:z", 1)
    local s = v + w
    count("Vector3f:__add", 1)
    check(math.abs(s:length() - 13) < 1e-4, "length")
    count("Vector3f:length", 1)
    local u = s:copy()
    count("Vector3f:copy", 1)
    u:normalize()
    count("Vector3f:normalize", 1)
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
  end,
  function()   -- argument errors keep their in-process texts; other vehicles' bindings are nil
    expect_error(function() gcs:send_text("x", "y") end, "bad argument #1 to 'send_text' (number expected, got string)")
    expect_error(function() gcs:send_text(9, "y") end, "bad argument #1 to 'send_text' (out of range)")
    expect_error(function() gcs:send_text(6) end, "bad argument #1 to 'send_text' (too few arguments)")
    expect_error(function() ahrs:set_home(Vector3f()) end, "bad argument #1 to 'set_home' (Location expected, got Vector3f)")
    count("ahrs:set_home", 1)
    count("Vector3f:new", 1)
    check(quadplane == nil and sub == nil, "bindings of other vehicles")
  end,
}

local step = 1

local function report()
  for _, name in ipairs(order) do
    gcs:send_text(6, string.format("M4 count %s %d", name, counts[name]))
  end
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

return update, 1000
