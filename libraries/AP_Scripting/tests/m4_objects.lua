-- Script-owned M7 objects on the M4 (ScriptingM4Objects, arducopter.py). Parameter and serial
-- ports live in the M7's object pool and are released when the M4 collects them; dot calls of
-- the serial singleton keep their arguments; analog:channel() runs its M7 replacement.
-- M4 only: in-process nothing limits the objects a script holds, so the limit stage fails.
-- Needs SCR_SDEV_EN 1 and SCR_SDEV1_PROTO 28 (a simulated device on the first scripting port).

local POOL_SLOTS = 64     -- ScriptingM4::ObjectPool::SLOTS

local failures = 0
local function check(ok, name)
  if not ok then
    failures = failures + 1
    gcs:send_text(3, "M4 objects FAIL " .. name)
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

local held = nil   -- objects a stage keeps until its run ends

local function parameters()
  -- without the M7 object these are nil, or an error
  local p = Parameter("SCR_HEAP_SIZE")
  check(p:get() == param:get("SCR_HEAP_SIZE"), "Parameter get")
  local empty = Parameter()
  check(empty:get() == nil, "Parameter() get")     -- no parameter yet: false, so nil
  check(empty:init("SCR_USER1"), "Parameter init")
  check(empty:set(5) and param:get("SCR_USER1") == 5, "Parameter set")
  local err = error_of(Parameter, "NO_SUCH_PARAM")
  check(has(err, "No parameter: NO_SUCH_PARAM"), "unknown name: " .. tostring(err))
  -- set_and_save writes storage: past the Storage quota (burst 100) it returns false, where
  -- without the quota all 150 return true
  local refused = 0
  for _ = 1, 150 do
    if not empty:set_and_save(5) then
      refused = refused + 1
    end
  end
  check(refused > 0, "set_and_save quota")
end

-- every slot held: the next object is refused with the M7's text, after the M4 collected and
-- retried. Without the pool bound all POOL_SLOTS + 8 are made
local function limit()
  held = {}
  local err = nil
  for _ = 1, POOL_SLOTS + 8 do
    local ok, object = pcall(Parameter, "SCR_USER1")
    if not ok then
      err = tostring(object)
      break
    end
    held[#held + 1] = object
  end
  -- other scripts' live objects take a few slots
  check(#held <= POOL_SLOTS and #held >= POOL_SLOTS - 8, "held " .. #held .. " objects")
  check(has(err, "M4: too many objects"), "limit text: " .. tostring(err))
  held = nil   -- collected after this run, and released before the next
end

-- the last stage's slots were released: as many again. Without the releases the first fails
local function released()
  held = {}
  for _ = 1, POOL_SLOTS - 8 do
    held[#held + 1] = Parameter("SCR_USER1")
  end
  check(#held == POOL_SLOTS - 8, "released slots")
  held = nil
end

-- objects dropped within one run do not count against the limit: the M4 collects and retries
-- when the pool is full. Without the retry the 65th creation of this loop fails
local function churn()
  local made = 0
  for _ = 1, 1000 do
    if Parameter("SCR_USER1") then
      made = made + 1
    end
  end
  check(made == 1000, "churn " .. made)
end

-- serial port objects are pool objects too: 200 of them in one run, 400 with the far ends, are
-- only made if the M4 releases the dropped ones. Without the release of that type the pool is
-- full after 64. (Dot calls, argument errors and readstring bounds: m4_serial_ports.lua)
local function serial_ports()
  local made = 0
  for _ = 1, 200 do
    local driver = serial.find_serial(0)
    local device = serial.find_simulated_device(28, 0)
    if driver ~= nil and device ~= nil then
      made = made + 1
    end
  end
  check(made == 200, "serial churn " .. made)
  local driver = serial:find_serial(0)
  check(driver ~= nil and driver:writestring("") == 0, "writestring empty")
end

local function analog_channel()
  -- through the M7 replacement (lua_m4_analog_channel): a source, usable as in-process
  local source = analog:channel()
  check(source ~= nil, "analog:channel()")
  if source then
    check(type(source:voltage_latest()) == "number", "voltage_latest")
  end
end

local function i2c_limits()
  -- a clock of 0 is accepted (the M7 clamps it; without the clamp ChibiOS divides by zero)
  local device = i2c:get_device(0, 0x10, 0)
  check(device ~= nil, "i2c clock 0")
  if device == nil then
    return
  end
  -- 200 sent + 100 received is over 256 bytes: refused (without the cap 21 attempts at 41+ ms each exceed the 900 ms bound)
  local ok, err = pcall(device.transfer, device, string.rep("x", 200), 100)
  check(not ok and has(err, "transfer longer than 256 bytes"), "transfer 300 bytes: " .. tostring(err))
  -- the retry count is capped at 18 (without the cap 20 is accepted)
  device:set_retries(18)
  ok, err = pcall(device.set_retries, device, 19)
  check(not ok, "set_retries(19): " .. tostring(err))
  check(device:write_register(0, 0) ~= nil, "write_register through the bounded take")
end

local stages = { parameters, limit, released, churn, serial_ports, analog_channel, i2c_limits }
local stage = 0

local function update()
  stage = stage + 1
  if stage <= #stages then
    local ok, err = pcall(stages[stage])
    if not ok then
      gcs:send_text(3, string.sub("M4 objects FAILED: " .. tostring(err), 1, 50))
      return
    end
    return update, 1000
  end
  if failures == 0 then
    gcs:send_text(6, "M4 objects passed")
  else
    gcs:send_text(3, "M4 objects failed " .. failures)
  end
end

return update, 1000
