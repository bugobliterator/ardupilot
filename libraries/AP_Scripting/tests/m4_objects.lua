-- Script-owned M7 objects on the M4 (ScriptingM4Objects, arducopter.py). Parameter and serial
-- ports live in the M7's object pool and are released when the M4 collects them; dot calls of
-- the serial singleton keep their arguments; analog:channel() runs its M7 replacement.
-- M4 only: in-process nothing limits the objects a script holds, so the limit stage fails.
-- Needs SCR_SDEV_EN 1 and SCR_SDEV1_PROTO 28 (a simulated device on the first scripting port).

local POOL_SLOTS = 64     -- ScriptingM4::ObjectPool::SLOTS
local IO_CHUNK = 768      -- ScriptingM4Remote::IO_CHUNK: bytes per readstring() reply

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

-- write 1000 bytes to the far end, then read count bytes from the port: what a call returns
-- and what is left. Without the 768-byte bound a count above it returns more than one reply
local function read_boundary(driver, device, count)
  local data = string.rep("0123456789", 100)
  check(driver:writestring(data) == #data, "write 1000 for " .. count)
  local first = device:readstring(count)
  local left = 0
  while true do
    local rest = device:readstring(IO_CHUNK)
    if rest == nil or #rest == 0 then
      break
    end
    left = left + #rest
  end
  return first, left
end

local function serial_ports()
  check(serial:find_serial(0) ~= nil, "serial:find_serial(0)")
  -- a dot call keeps its argument: without ROW_DOT_ACCESS the 0 is replaced by the nil self
  -- and the M7 answers "too few arguments"
  local ok, by_dot = pcall(serial.find_serial, 0)
  check(ok and by_dot ~= nil, "serial.find_serial(0): " .. tostring(by_dot))
  local driver = serial.find_serial(0)
  local device = serial.find_simulated_device(28, 0)
  check(device ~= nil, "serial.find_simulated_device(28, 0)")
  check(serial:find_serial(9) == nil, "no tenth scripting port")
  check(has(error_of(serial.find_serial), "too few arguments"), "dot call without arguments")
  local many = {}
  for i = 1, 16 do
    many[i] = i
  end
  -- 16 arguments and the nil self are 17 values, as in-process after the dot fix
  check(has(error_of(serial.find_serial, table.unpack(many)), "#17 ") and
        has(error_of(serial.find_serial, table.unpack(many)), "too many arguments"), "dot call with 16 arguments")
  if device and driver then
    -- 1000 bytes available: a call returns up to the count and never more than IO_CHUNK
    local first, left = read_boundary(driver, device, 1000)
    check(first ~= nil and #first == IO_CHUNK and left == 232, "readstring(1000): " .. tostring(first and #first) .. "+" .. left)
    first, left = read_boundary(driver, device, 65535)
    check(first ~= nil and #first == IO_CHUNK and left == 232, "readstring(65535)")
    first, left = read_boundary(driver, device, IO_CHUNK - 1)
    check(first ~= nil and #first == IO_CHUNK - 1 and left == 233, "readstring(767)")
    first, left = read_boundary(driver, device, IO_CHUNK)
    check(first ~= nil and #first == IO_CHUNK and left == 232, "readstring(768)")
    first, left = read_boundary(driver, device, IO_CHUNK + 1)
    check(first ~= nil and #first == IO_CHUNK and left == 232, "readstring(769)")
    first, left = read_boundary(driver, device, 0)
    check(first == "" and left == 1000, "readstring(0) leaves everything")
    -- a negative count is out of range before anything is read
    local data = string.rep("x", 10)
    check(driver:writestring(data) == 10, "write 10")
    check(has(error_of(device.readstring, device, -1), "out of range"), "readstring(-1)")
    check(device:readstring(10) == data, "readstring(-1) read nothing")
    check(driver:writestring("") == 0, "writestring empty")
  end
end

local function analog_channel()
  -- through the M7 replacement (lua_m4_analog_channel): a source, usable as in-process
  local source = analog:channel()
  check(source ~= nil, "analog:channel()")
  if source then
    check(type(source:voltage_latest()) == "number", "voltage_latest")
  end
end

local stages = { parameters, limit, released, churn, serial_ports, analog_channel }
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
