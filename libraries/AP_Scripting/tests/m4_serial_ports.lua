-- Serial ports on the M4 (ScriptingM4Bindings, arducopter.py): serial.find_serial(0) and
-- serial.find_simulated_device(28, 0) as dot calls and as method calls, binary data through the
-- ports, the 768-byte readstring chunk. Runs in-process too; the M4 reaches the same results.
-- Needs SCR_SDEV_EN 1 and SCR_SDEV1_PROTO 28 (the simulated device is the far end of the port).

local CHUNK = 768

local function check(condition, text)
  if not condition then
    error("M4 serial: " .. text, 2)
  end
end

local function raises(text, fn, ...)
  local ok, err = pcall(fn, ...)
  check(not ok, text .. " did not raise")
  return tostring(err)
end

local driver, device

local stages = {
  function()   -- dot calls keep their first argument: without it find_serial(0) is "too few arguments"
    driver = serial.find_serial(0)
    check(driver ~= nil, "serial.find_serial(0)")
    device = serial.find_simulated_device(28, 0)
    check(device ~= nil, "serial.find_simulated_device(28, 0)")
    check(serial.find_serial(7) == nil, "serial.find_serial(7) is nil")
    check(serial.find_simulated_device(28, 5) == nil, "serial.find_simulated_device(28, 5) is nil")
  end,
  function()   -- method calls give the same
    check(serial:find_serial(0) ~= nil, "serial:find_serial(0)")
    check(serial:find_simulated_device(28, 0) ~= nil, "serial:find_simulated_device(28, 0)")
    check(serial:find_serial(7) == nil, "serial:find_serial(7) is nil")
  end,
  function()   -- argument errors of a dot call name the stack as in-process, self included
    local err = raises("serial.find_serial()", serial.find_serial)
    check(string.find(err, "#1 ", 1, true) and string.find(err, "too few arguments", 1, true),
          "no arguments '" .. err .. "'")
    -- 16 arguments plus the self in-process: 17
    err = raises("16 arguments", serial.find_serial, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16)
    check(string.find(err, "#17 ", 1, true) and string.find(err, "too many arguments", 1, true),
          "16 arguments '" .. err .. "'")
    err = raises("find_serial(0, 1)", serial.find_serial, 0, 1)
    check(string.find(err, "#3 ", 1, true) and string.find(err, "too many arguments", 1, true),
          "two arguments '" .. err .. "'")
    -- another singleton where the serial is expected is an argument, not a self
    err = raises("find_serial(gcs)", serial.find_serial, gcs)
    check(string.find(err, "number expected, got", 1, true) ~= nil, "singleton argument '" .. err .. "'")
  end,
  function()   -- begin: no argument or nil is automatic baud
    driver:begin(115200)
    driver:begin()
    driver:begin(nil)
    local err = raises("begin(0)", driver.begin, driver, 0)
    check(string.find(err, "out of range", 1, true) ~= nil, "begin(0) '" .. err .. "'")
    err = raises("begin(1, 2)", driver.begin, driver, 1, 2)
    check(string.find(err, "too many arguments", 1, true) ~= nil, "begin(1, 2) '" .. err .. "'")
    err = raises("begin() on a number", driver.begin, 5)
    check(string.find(err, "expected", 1, true) ~= nil, "begin(5) '" .. err .. "'")
  end,
  function()   -- binary data both ways, NULs and high bytes included
    local message = "a\0b\255\1z"
    check(driver:writestring(message) == #message, "driver writestring")
    local received = device:readstring(#message)
    check(received == message, "device read " .. #received .. " bytes")
    check(device:writestring(message) == #message, "device writestring")
    received = driver:readstring(#message)
    check(received == message, "driver read " .. #received .. " bytes")
    check(driver:readstring(10) == "", "nothing left")
    check(driver:readstring(0) == "", "readstring(0)")
    check(driver:readstring(65535) == "", "readstring(65535) with nothing available")
    local err = raises("readstring(65536)", driver.readstring, driver, 65536)
    check(string.find(err, "out of range", 1, true) ~= nil, "readstring(65536) '" .. err .. "'")
    err = raises("writestring({})", driver.writestring, driver, {})
    check(string.find(err, "string expected", 1, true) ~= nil, "writestring({}) '" .. err .. "'")
  end,
  function()   -- readstring returns at most 768 bytes a call, whatever count it is given
    local data = string.rep("0123456789abcdef", 62)   -- 992 bytes
    check(driver:writestring(data) == #data, "device writes 992 bytes")
    local first = device:readstring(65535)
    check(#first == CHUNK, "first read " .. #first)
    local second = device:readstring(65535)
    check(#second == #data - CHUNK, "second read " .. #second)
    check(first .. second == data, "the chunks join to what was written")
    check(device:readstring(65535) == "", "nothing left after the chunks")
  end,
}
local stage = 0

local function update()
  stage = stage + 1
  if stage <= #stages then
    local ok, err = pcall(stages[stage])
    if not ok then
      gcs:send_text(3, string.sub("M4 serial FAILED: " .. tostring(err), 1, 50))
      return
    end
    return update, 200
  end
  gcs:send_text(6, "M4 serial ports passed")
end

return update, 1000
