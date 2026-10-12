-- Holds a session's devices for the whole M4 session: 4 I2C devices, 4 PWM sources, a UDP socket
-- bound to a fixed port, a CRSF menu with a sub-menu and a parameter, the CAN buffers of the
-- session with a full set of filters, 8 DroneCAN subscriptions, and a TCP listener with the
-- connection an outside peer made to it. ScriptingM4Devices restarts the M4: the new session
-- must get the same again. It could not if the M7 kept the old ones: no i2c devices available,
-- no PWMSources available, the port still bound, a first menu id one menu further on, no CAN
-- buffer past the cap and no filter free, no subscription free, the listener not bound, and the
-- peer's connection closed with a FIN where the M7 resets it.
-- Needs CAN_P1_DRIVER 1, CAN_P2_DRIVER 2, CAN_D1_PROTOCOL 1, CAN_D2_PROTOCOL 10 and NET_ENABLE 1.

local NODESTATUS_ID = 341
local NODESTATUS_SIGNATURE = uint64_t(0x0F0868D0, 0xC1A7C6F1)
local LISTEN_PORT = 15905

-- a global of the script: never collected
-- luacheck: globals held_devices
held_devices = {}
local function hold(object)
  held_devices[#held_devices + 1] = object
end

local i2c_count, pwm_count = 0, 0
for _ = 1, 4 do
  local ok, dev = pcall(i2c.get_device, i2c, 3, 0x0B)
  if ok and dev then
    i2c_count = i2c_count + 1
    hold(dev)
  end
  local made, source = pcall(PWMSource)
  if made and source then
    pwm_count = pwm_count + 1
    hold(source)
  end
end
local sock = Socket(1)
local bound = sock ~= nil and sock:bind("127.0.0.1", 15903)
hold(sock)
local menu = crsf:add_menu("M4 hold")
hold(menu)
gcs:send_text(6, string.format("M4 devices i2c %d pwm %d bound %s menu %s", i2c_count, pwm_count,
                               tostring(bound), tostring(menu and menu:id())))

-- a sub-menu and a parameter: the next session builds them again
local sub = menu and menu:add_menu("M4 sub")
hold(sub)
local parameter = menu and menu:add_parameter("hold")
hold(parameter)
gcs:send_text(6, string.format("M4 devices crsf sub %s params %s", tostring(sub and sub:id()),
                               tostring(menu and menu:num_params())))

-- buffers up to the cap, and the filters of the first: 8 each
local buffers = 0
local filters = 0
for _ = 1, 12 do
  local buffer = CAN.get_device(5)
  if buffer == nil then
    break
  end
  hold(buffer)
  buffers = buffers + 1
  if buffers == 1 then
    while buffer:add_filter(0x7FF, 0x100 + filters) do
      filters = filters + 1
    end
  end
end

-- subscriptions up to the cap: the ninth is refused
local subscriptions = 0
local ninth_refused = false
for _ = 1, 9 do
  local handle = DroneCAN_Handle(0, NODESTATUS_SIGNATURE, NODESTATUS_ID)
  hold(handle)
  if handle:subscribe() then
    subscriptions = subscriptions + 1
  elseif subscriptions == 8 then
    ninth_refused = true
  end
end
gcs:send_text(6, string.format("M4 devices can %d filt %d sub %d ninth refused %s", buffers, filters,
                               subscriptions, tostring(ninth_refused)))

local listener = Socket(0)
local listening = listener ~= nil and listener:bind("127.0.0.1", LISTEN_PORT) and listener:listen(1)
hold(listener)
gcs:send_text(6, "M4 devices listening " .. tostring(listening))

local accepted = nil

local function update()
  if accepted == nil then
    accepted = listener:accept()
    if accepted ~= nil then
      hold(accepted)
      gcs:send_text(6, "M4 devices accepted")
    end
  end
  return update, 500
end

return update, 500
