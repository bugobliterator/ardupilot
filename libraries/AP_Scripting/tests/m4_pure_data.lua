-- Pure-data userdata (Vector2f, Vector3f, Quaternion, Location) with fixed inputs.
-- Assumes ArduPilot's 32-bit lua_Integer and float. Every result becomes text: a float as its bit pattern (NaN as "nan": its sign and
-- payload are not compared), an integer in decimal, a boolean or string as is, an
-- error message as its length and FNV-1a. The same script runs in-process and on the
-- M4; the tests compare the lines one for one. Only gcs:send_text leaves the script,
-- four lines a second (the M4's Text quota is 20/s, burst 40).

local lines = {}

local function fnv(text)
  local h = -2128831035   -- 2166136261 as a 32-bit integer; integer arithmetic wraps
  for i = 1, #text do
    h = (h ~ text:byte(i)) * 16777619
  end
  return h
end

local function text(v)
  local kind = math.type(v)
  if kind == "float" then
    if v ~= v then
      return "nan"
    end
    return string.format("%08x", string.unpack("<i4", string.pack("<f", v)))
  elseif kind == "integer" then
    return string.format("%d", v)
  end
  return tostring(v)
end

local function add(...)
  local values = {...}   -- no value is nil; ArduPilot's Lua has no select()
  local parts = {}
  for i = 1, #values do
    parts[i] = text(values[i])
  end
  local line = string.format("PD %02d %s", #lines + 1, table.concat(parts, " "))
  -- one STATUSTEXT holds 50 characters: a longer line would arrive cut in both runs
  assert(#line <= 50, "PD line too long: " .. line)
  lines[#lines + 1] = line
end

local function add_error(fn)
  local ok, err = pcall(fn)
  if ok then
    add("no error")
  else
    err = tostring(err)
    add("err", #err, fnv(err))
  end
end

local home   -- shared by the stages

local function build_values()
  -- Vector2f
  local v = Vector2f()
  v:x(3)
  v:y(-4)
  add(v:x(), v:y(), v:length(), v:angle())
  local w = Vector2f()
  w:x(0.5)
  w:y(0.25)
  local s = v + w
  local d = v - w
  add(s:x(), s:y(), d:x(), d:y())
  local r = v:copy()
  r:rotate(0.7)
  add(r:x(), r:y(), v:x(), v:y())           -- the copy turned, the original did not
  r:normalize()
  add(r:x(), r:y(), r:length(), r:is_zero())
  local z = Vector2f()
  add(z:is_zero(), z:angle(), z:length())
  z:normalize()                              -- 0/0
  add(z:x(), z:is_nan(), z:is_inf(), z:is_zero())
  local big = Vector2f()
  big:x(1e38)
  big:y(1e38)
  add(big:length(), big:is_inf())            -- the float length overflows

  -- Vector3f
  local a = Vector3f()
  a:x(1.5)
  a:y(-2.25)
  a:z(3.125)
  local b = Vector3f()
  b:x(-0.5)
  b:y(4)
  b:z(0.75)
  local c = a:cross(b)
  add(c:x(), c:y(), c:z(), a:dot(b))
  add(a:angle(b), a:length(), (a + b):length(), (a - b):z())
  local sc = a:scale(1.7)
  local xy = a:xy()
  add(sc:x(), sc:y(), sc:z(), xy:y())
  local t = a:copy()
  t:rotate_xy(2.1)
  add(t:x(), t:y(), t:z(), a:x())
  t:normalize()
  add(t:x(), t:y(), t:z(), t:length())
  local tiny = Vector3f()
  tiny:x(1e-30)
  tiny:y(-2e-30)
  add(tiny:length(), tiny:is_zero(), a:angle(Vector3f()))
  local inf = Vector3f()
  inf:z(1 / 0)
  add(inf:is_inf(), inf:is_nan(), inf:length())
  local nan = Vector3f()
  nan:x(0 / 0)
  add(nan:is_nan(), nan:dot(a), nan:is_zero())

  -- Quaternion
  local q = Quaternion()
  q:from_euler(0.1, -0.2, 0.3)
  add(q:q1(), q:q2(), q:q3(), q:q4())
  add(q:get_euler_roll(), q:get_euler_pitch(), q:get_euler_yaw(), q:length())
  local q2 = Quaternion()
  q2:from_euler(-1.2, 0.4, 2.9)
  local p = q * q2
  add(p:q1(), p:q2(), p:q3(), p:q4())
  local inv = q:inverse()
  add(inv:q1(), inv:q2(), inv:q3(), inv:q4())
  local e = b:copy()
  q:earth_to_body(e)                         -- writes its argument, as in-process
  add(e:x(), e:y(), e:z(), b:x())
  local axis = Vector3f()
  p:to_axis_angle(axis)
  add(axis:x(), axis:y(), axis:z())
  local qa = Quaternion()
  qa:from_axis_angle(b, 0.8)
  add(qa:q1(), qa:q2(), qa:q3(), qa:q4())
  local qv = Quaternion()
  qv:q1(1)
  qv:from_angular_velocity(a, 0.05)
  add(qv:q1(), qv:q2(), qv:q3(), qv:q4())
  local gimbal = Quaternion()
  gimbal:from_euler(math.rad(10), math.rad(85), math.rad(20))   -- near, not at, gimbal lock
  add(gimbal:get_euler_roll(), gimbal:get_euler_pitch(), gimbal:get_euler_yaw())
  local unnorm = Quaternion()
  unnorm:q1(2)
  unnorm:q3(-3)
  unnorm:normalize()
  add(unnorm:q1(), unnorm:q2(), unnorm:q3(), unnorm:q4())

  -- Location: double precision on the M7 (ftype)
  home = Location()
  home:lat(-353632621)
  home:lng(1491652374)
  home:alt(58409)
  local there = home:copy()
  there:offset(100.5, -250.25)
  add(there:lat(), there:lng(), home:get_distance(there))
  add(home:get_bearing(there), there:get_bearing(home))
  local ne = home:get_distance_NE(there)
  local ned = home:get_distance_NED(there)
  add(ne:x(), ne:y(), ned:x(), ned:z())
  local b2 = home:copy()
  b2:offset_bearing(37.5, 1234.5)
  add(b2:lat(), b2:lng(), b2:alt())
  local b3 = home:copy()
  b3:offset_bearing_and_pitch(200, 15, 500)
  add(b3:lat(), b3:lng(), b3:alt())
  local east = Location()
  east:lng(1799999990)                       -- across the antimeridian
  east:offset(0, 5000)
  add(east:lat(), east:lng(), Location():get_distance(east))
  local pole = Location()
  pole:lat(899999000)                        -- past the pole: limit_lattitude
  pole:offset(2000, 10)
  add(pole:lat(), pole:lng(), pole:get_distance(Location()))
  local framed = home:copy()
  framed:set_alt_m(123.45, 2)                -- ABOVE_ORIGIN
  add(framed:alt(), framed:get_alt_frame(), framed:origin_alt(), framed:relative_alt())
  framed:set_alt_m(-7.5, 3)                  -- ABOVE_TERRAIN
  add(framed:alt(), framed:get_alt_frame(), framed:terrain_alt(), framed:relative_alt())
  local alias = framed                       -- a reference: a field write through it shows
  alias:loiter_xtrack(true)
  add(framed:loiter_xtrack())

  -- numbers as Lua prints them, one value per line (Location values apart from the rest)
  add(tostring(a:length()))
  add(tostring(home:get_distance(there)))
  add(tostring(q:get_euler_yaw()))
  add(tostring(ned:x()))
  add(string.format("%.3f", b2:lat() * 1e-7))
  add(string.format("%.2f", ne:y()))

end

-- the VM stops a script after a fixed instruction count: errors are hashed in two stages
local function build_errors_location()
  -- errors keep the in-process texts
  add_error(function() Location():lat(900000001) end)
  add_error(function() Location():lng(1800000001) end)
  add_error(function() Location():alt(2 ^ 31) end)
  add_error(function() Location():lat(1.5) end)
end

local function build_errors_other()
  add_error(function() Location():get_distance(Vector3f()) end)
  add_error(function() Vector3f():x("a") end)
  add_error(function() Vector2f():x({}) end)
  add_error(function() Vector3f():dot() end)
  add_error(function() Quaternion():from_euler(1, 2) end)
  add_error(function() Quaternion():q1("a") end)
  add_error(function() Location():set_alt_m(1, 7) end)
  add_error(function() return Vector2f() + Vector3f() end)

end

local function build_remote_and_names()
  -- boolean fields written directly
  local flags = Location()
  flags:relative_alt(true)
  flags:terrain_alt(true)
  flags:origin_alt(true)
  flags:loiter_xtrack(true)
  local before = flags:relative_alt()
  flags:relative_alt(false)
  add(before, flags:relative_alt(), flags:terrain_alt(), flags:origin_alt())

  -- a local value to a remote method and the remote result decoded locally; only values
  -- that do not depend on vehicle state (home, origin, EKF) are printed
  local absolute = home:copy()
  absolute:set_alt_m(5, 0)
  add(absolute:get_alt_m(0))                 -- remote: boolean and float, ABSOLUTE
  add(absolute:change_alt_frame(0), absolute:get_alt_frame(), absolute:alt())
  local moved = home:copy()
  local changed = moved:change_alt_frame(1)  -- needs home: only the types are printed
  add(type(changed), math.type(moved:alt()))
  local from_origin = home:get_vector_from_origin_NEU_m()   -- remote: a Vector3f or nil, by origin
  if from_origin ~= nil then
    from_origin:x()                          -- the decoded local vector works
  end
  add(from_origin == nil or type(from_origin) == "userdata")

  -- names: a local method, a remote row of a local type (looked up, not called), none
  add(type(Location().get_distance), type(Location().change_alt_frame), Location().no_such == nil)
  assert(#lines == 59, "PD line count " .. #lines)
end

local stages = {build_values, build_errors_location, build_errors_other, build_remote_and_names}
local next_stage = 1
local next_line = 1
local texts = 0

local function update()
  if next_stage <= #stages then
    stages[next_stage]()
    next_stage = next_stage + 1
    return update, 100
  end
  for _ = 1, 4 do
    if next_line > #lines then
      texts = texts + 1   -- this one
      gcs:send_text(6, string.format("PD done texts=%d", texts))
      return
    end
    gcs:send_text(6, lines[next_line])
    texts = texts + 1
    next_line = next_line + 1
  end
  return update, 1000
end

return update, 2000
