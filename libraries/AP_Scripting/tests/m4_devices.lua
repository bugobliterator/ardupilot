-- M7 devices M4 scripts make (ScriptingM4Devices, arducopter.py): I2C devices, PWM sources,
-- CAN buffers, DroneCAN handles, sockets and CRSF menus. M4 only: in-process sendto() sends
-- the length asked for, reading past the string, and CRSFParameter() exists. Each stage says
-- what it would see without the code it targets.
-- Needs CAN_P1_DRIVER 1, CAN_P2_DRIVER 2, CAN_D1_PROTOCOL 1 (DroneCAN), CAN_D2_PROTOCOL 10
-- (scripting) and NET_ENABLE 1.

local SMBUS_BUS, SMBUS_ADDRESS = 3, 0x0B                      -- SITL's generic SMBus battery
local DESIGN_VOLTAGE_REGISTER, DESIGN_VOLTAGE = 0x19, 50400   -- SITL/SIM_BattMonitor_SMBus.cpp
local NODESTATUS_ID = 341
local NODESTATUS_SIGNATURE = uint64_t(0x0F0868D0, 0xC1A7C6F1)
local UDP_PORT, TCP_PORT, STREAM_PORT = 15901, 15902, 15904
local IO_CHUNK = 768          -- the most one recv returns (AP_ScriptingM4_Remote.h)
local CAN_BUFFERS_PER_SESSION = 8
local CRSF_MENUS_PER_SESSION, CRSF_PARAMETERS_PER_MENU = 10, 20

local failures = 0
local function check(ok, name)
  if not ok then
    failures = failures + 1
    gcs:send_text(3, "M4 devices FAIL " .. name)
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

local held = {}   -- objects kept from one stage to the next
local send_path = nil

local function i2c_devices()
  -- a dot call: without ROW_DOT_ACCESS the bus number is replaced by the nil self
  local by_dot = i2c.get_device(SMBUS_BUS, SMBUS_ADDRESS)
  check(by_dot ~= nil, "i2c.get_device")
  local dev = i2c:get_device(SMBUS_BUS, SMBUS_ADDRESS)
  check(dev ~= nil, "i2c:get_device")
  if dev then
    -- a table of bytes, as in-process: without the M4 wrapper this is the M7's string
    local regs = dev:read_registers(DESIGN_VOLTAGE_REGISTER, 2)
    check(type(regs) == "table" and #regs == 2, "read_registers table: " .. type(regs))
    if type(regs) == "table" and #regs == 2 then
      check(regs[1] + 256 * regs[2] == DESIGN_VOLTAGE or regs[2] + 256 * regs[1] == DESIGN_VOLTAGE, "design voltage")
    end
    check(math.type(dev:read_registers(DESIGN_VOLTAGE_REGISTER)) == "integer", "read_registers one byte")
    local reply = dev:transfer(string.char(DESIGN_VOLTAGE_REGISTER), 2)
    check(reply ~= nil and #reply == 2, "transfer")
  end
  -- 4 devices per session (SCRIPTING_MAX_NUM_I2C_DEVICE): without the count a fifth is made
  local third = i2c:get_device(SMBUS_BUS, SMBUS_ADDRESS)
  local fourth = i2c:get_device(SMBUS_BUS, SMBUS_ADDRESS)
  check(third ~= nil and fourth ~= nil, "third and fourth i2c device")
  check(has(error_of(i2c.get_device, i2c, SMBUS_BUS, SMBUS_ADDRESS), "no i2c devices available"), "fifth i2c device")
end

local function pwm_sources()
  local sources = {}
  for i = 1, 4 do
    sources[i] = PWMSource()
    check(sources[i] ~= nil, "PWMSource " .. i)
  end
  check(has(error_of(PWMSource), "no PWMSources available"), "fifth PWMSource")
  check(sources[1] ~= nil and math.type(sources[1]:get_pwm_us()) == "integer", "get_pwm_us")
end

local function can_buffers()
  local buffer = CAN.get_device(5)   -- a dot call; driver 2 is the scripting driver
  check(buffer ~= nil, "CAN.get_device(5)")
  check(CAN:get_device2(5) == nil, "no second scripting driver")
  check(has(error_of(CAN.get_device, CAN, 26), "out of range"), "buffer length 26")
  if buffer then
    -- 8 filters (ScriptingCANBuffer::filter): the ninth is refused
    check(buffer:add_filter(0x7FF, 0x123), "add_filter")
    local frame = CANFrame()
    frame:id(0x123)
    frame:dlc(1)
    frame:data(0, 7)
    check(type(buffer:write_frame(frame, 10000)) == "boolean", "write_frame")
    -- the timeout is cut to 900 ms on the M7. Without the cut the M7 waits for the whole 4000 s
    -- and the script never gets past this call
    local start = millis()
    check(type(buffer:write_frame(frame, 4000000000)) == "boolean", "write_frame long timeout")
    local took = (millis() - start):toint()
    check(took < 950, "write_frame took " .. took)
    local got = buffer:read_frame()
    check(got == nil or got:id() == 0x123, "read_frame")
  end
  -- the session holds 8 buffers per sensor: the ninth is nil. Without the cap (take_at_most) it
  -- is made, as in-process
  local made = buffer and 1 or 0
  for _ = 1, 12 do
    if CAN.get_device(5) == nil then
      break
    end
    made = made + 1
  end
  check(made == CAN_BUFFERS_PER_SESSION, "CAN buffers " .. made)
end

local function dronecan()
  local handle = DroneCAN_Handle(0, NODESTATUS_SIGNATURE, NODESTATUS_ID)
  check(handle ~= nil, "DroneCAN_Handle on driver 0")
  check(DroneCAN_Handle(9, NODESTATUS_SIGNATURE, NODESTATUS_ID) == nil, "no driver 9")
  if handle then
    check(handle:subscribe(), "subscribe")
    check(type(handle:broadcast(string.rep("\0", 7))) == "boolean", "broadcast")
    local message = handle:check_message()
    check(message == nil or type(message) == "string", "check_message")
  end
end

-- handles dropped within one run do not count against the pool of 64: the M4 collects and
-- retries when it is full. Without the retry the 65th creation raises "too many objects". Their
-- subscribers go with them (the in-process __gc): the session holds at most 8, so more than 8
-- subscriptions in the run mean the collected handles released theirs
local function dronecan_churn()
  local created, subscribed = 0, 0
  for _ = 1, 200 do
    local h = DroneCAN_Handle(0, NODESTATUS_SIGNATURE, NODESTATUS_ID)
    if h then
      created = created + 1
      if h:subscribe() then
        subscribed = subscribed + 1
      end
    end
  end
  check(created == 200, "churn created " .. created)
  check(subscribed > 8, "churn subscribed " .. subscribed)
end

-- a transfer costs its frames against 800 frames/s (burst 800): 29 frames each for 200 bytes. The
-- calls are slower than the bus, so only the quota refuses them; without it every one is sent
local function dronecan_quota()
  local handle = DroneCAN_Handle(0, NODESTATUS_SIGNATURE, NODESTATUS_ID)
  local sent, refused = 0, 0
  for _ = 1, 100 do
    if handle:broadcast(string.rep("\0", 200)) then
      sent = sent + 1
    else
      refused = refused + 1
    end
  end
  check(sent > 0, "broadcast sent " .. sent)
  check(refused > 0, "CAN quota refused " .. refused)
end

local function udp_send()
  local rx = Socket(1)
  local tx = Socket(1)
  check(rx ~= nil and tx ~= nil, "Socket(1)")
  check(rx:bind("127.0.0.1", UDP_PORT), "bind")
  -- bounded by the string: in-process sendto() reads 100 bytes from a 6-byte string
  local sent = tx:sendto("hello", 100, string_to_ipv4_addr("127.0.0.1"), UDP_PORT)
  check(sent == 5, "sendto sent " .. tostring(sent))
  held.rx, held.tx = rx, tx
end

local function udp_receive()
  local data, addr, port = held.rx:recv(65535)   -- a count beyond one reply: up to 768 bytes
  check(data == "hello", "recv " .. tostring(data))
  check(addr ~= nil and ipv4_addr_to_string(addr) == "127.0.0.1", "recv address")
  check(math.type(port) == "integer", "recv port")
  -- an idle pollin asks 5 s: the M7 waits at most 900 ms, inside the call's 1 s timeout.
  -- Without the cut the M7 waits 5 s and the call fails
  local start = millis()
  local ok, err = pcall(held.rx.pollin, held.rx, 5000)
  local took = (millis() - start):toint()
  check(ok and err == false, "pollin " .. tostring(err))
  check(took >= 850 and took < 1000, "pollin took " .. took)
  ok, err = pcall(held.rx.pollout, held.rx, 5000)
  check(ok and err == true, "pollout " .. tostring(err))
  held.rx:close()
  -- a closed socket's handle is stale; in-process the next call dereferences null
  check(has(error_of(held.rx.recv, held.rx, 4), "M4: stale SocketAPM"), "recv after close")
  held.tx:close()
  held.rx, held.tx = nil, nil
end

local function tcp_connect()
  local listener = Socket(0)
  check(listener:bind("127.0.0.1", TCP_PORT) and listener:listen(4), "listen")
  local client = Socket(0)
  -- a new socket blocks, as in-process: connect waits for the handshake. Without the M4's wait
  -- the M7's non-blocking connect returns false while the handshake is pending
  check(client:connect("127.0.0.1", TCP_PORT), "blocking connect")
  check(client:is_connected(), "is_connected")
  send_path = scripts_dir() .. "/m4_devices_send.txt"
  local f = io.open(send_path, "w")
  f:write("sendfile body")
  f:close()
  held.listener, held.client = listener, client
end

-- a blocking send to a peer that never reads fills the buffers, then waits for 5 s without
-- progress and returns what was sent, never blocking the M7. Without the M4's wait the first
-- full buffer returns at once; a non-blocking socket always does
local function tcp_blocking_send()
  held.server = held.listener:accept()
  check(held.server ~= nil, "accept")
  local chunk = string.rep("x", IO_CHUNK)
  local last, took = 0, 0
  for _ = 1, 20000 do
    local start = millis()
    last = held.client:send(chunk, #chunk)
    took = (millis() - start):toint()
    if last ~= #chunk then
      break
    end
  end
  gcs:send_text(6, string.format("M4 devices blocking send %d in %d", last, took))
  check(last ~= #chunk, "buffers never filled")
  check(took >= 4500 and took < 6000, "blocking send took " .. took)
  check(held.client:set_blocking(false), "set_blocking false")
  local start = millis()
  local sent = held.client:send(chunk, #chunk)
  took = (millis() - start):toint()
  check(sent == -1 and took < 200, string.format("non-blocking send %d in %d", sent, took))
  held.client:close()
  held.server:close()
  held.client, held.server = nil, nil
end

local function tcp_sendfile()
  local client = Socket(0)
  check(client:connect("127.0.0.1", TCP_PORT), "connect again")
  held.client = client
  local server = held.listener:accept()
  check(server ~= nil, "accept again")
  if not server then
    return
  end
  held.server = server
  -- read 4 bytes: the M4 read 256 ahead, so the M7's file must be set back to byte 4, where
  -- in-process the descriptor is
  local f = io.open(send_path, "r")
  check(f:read(4) == "send", "read 4")
  check(server:sendfile(f), "sendfile")
  -- AP_Networking owns the descriptor now: the file's close fails, as in-process (fd -1)
  check(f:close() == nil, "close after sendfile")
  check(has(error_of(server.sendfile, server, "not a file"), "FILE* expected"), "sendfile of a string")
end

-- 2 hand-overs per session: without the cap the third is made. A refused one leaves the file
-- with the script
local function tcp_sendfile_cap()
  local data = held.client:recv(64)
  check(data == "file body", "sendfile data " .. tostring(data))
  local client2 = Socket(0)
  check(client2:connect("127.0.0.1", TCP_PORT), "connect third")
  local server2 = held.listener:accept()
  check(server2 ~= nil, "accept third")
  local f2 = io.open(send_path, "r")
  check(server2:sendfile(f2), "second sendfile")
  local server3 = Socket(0)
  local f3 = io.open(send_path, "r")
  check(not server3:sendfile(f3), "third sendfile refused")
  check(f3:close(), "file of the refused sendfile still open")
  client2:close()
  server3:close()
  held.server:close()
  held.client:close()
  held.listener:close()
  held.server, held.client, held.listener = nil, nil, nil
  remove(send_path)
end

local function receive_all(sock, want)
  local got = ""
  for _ = 1, 200 do
    local data = sock:recv(IO_CHUNK)
    if data then
      got = got .. data
    end
    if #got >= want then
      break
    end
  end
  return got
end

-- 1000 bytes sent over a TCP pair: one recv returns 768, the next the other 232. Without the
-- bound a single recv returns all 1000 (in-process it does)
local function stream_send()
  local listener = Socket(0)
  check(listener:bind("127.0.0.1", STREAM_PORT) and listener:listen(4), "listen stream")
  local client = Socket(0)
  check(client:connect("127.0.0.1", STREAM_PORT), "connect stream")
  local server = listener:accept()
  check(server ~= nil, "accept stream")
  held.listener, held.client, held.server = listener, client, server
  if not server then
    return
  end
  local thousand = string.rep("0123456789", 100)
  check(client:send(thousand, #thousand) == 1000, "send 1000")
  held.thousand = thousand
end

local function stream_receive()
  local server = held.server
  local first = server:recv(65535)
  check(first ~= nil and #first == IO_CHUNK, "first recv " .. tostring(first and #first))
  local second = server:recv(65535)
  check(second ~= nil and #second == 1000 - IO_CHUNK, "second recv " .. tostring(second and #second))
  check((first or "") .. (second or "") == held.thousand, "1000 bytes arrive whole")
  -- a blocking stream send longer than one piece goes out in pieces and arrives whole
  local big = string.rep("abcd", 1024)
  check(held.client:send(big, #big) == 4096, "blocking 4 KB send")
  check(receive_all(server, 4096) == big, "4 KB arrives whole")
  held.big = big
  -- a non-blocking send of 3000 bytes goes out in pieces too
  check(held.client:set_blocking(false), "set_blocking false stream")
  check(held.client:send(held.big, 3000) == 3000, "non-blocking 3000 send")
  check(receive_all(server, 3000) == string.sub(held.big, 1, 3000), "3000 arrives whole")
  -- Socket(1) marks the socket a datagram one: a send over one reply is refused, where a
  -- stream send of the same size succeeds. Without the mark the datagram is cut into pieces
  local udp = Socket(1)
  local err = error_of(udp.sendto, udp, string.rep("x", 1100), 1100, string_to_ipv4_addr("127.0.0.1"), UDP_PORT)
  check(has(err, "M4: call arguments too large"), "1100 B sendto " .. tostring(err))
  check(udp:connect("127.0.0.1", UDP_PORT), "udp connect")
  err = error_of(udp.send, udp, string.rep("x", 1100), 1100)
  check(has(err, "M4: call arguments too large"), "1100 B datagram send " .. tostring(err))
  udp:close()
  check(held.client:send(string.rep("y", 1100), 1100) == 1100, "1100 B stream send")
  held.client:close()
  server:close()
  held.listener:close()
  held.listener, held.client, held.server, held.thousand, held.big = nil, nil, nil, nil, nil
end

local function crsf_menus()
  local menu = crsf:add_menu("M4 menu")
  check(menu ~= nil, "crsf:add_menu")
  if not menu then
    return
  end
  check(menu:name() == "M4 menu", "menu name")
  local data = string.char(9) .. "M4 param" .. string.char(0)   -- any bytes: the M7 keeps a copy
  local p = menu:add_parameter(data)
  check(p ~= nil and math.type(p:id()) == "integer", "parameter id")
  check(menu:num_params() == 1, "num_params")
  -- no data, or more than the 255 bytes a length byte holds, is refused. In-process the empty
  -- one stores a wild pointer and 256 bytes wrap to a length of 0
  check(menu:add_parameter("") == nil, "empty parameter")
  check(menu:add_parameter(string.rep("x", 256)) == nil, "256 byte parameter")
  check(menu:add_parameter(string.rep("y", 255)) ~= nil, "255 byte parameter")
  local sub = menu:add_menu("M4 sub")
  check(sub ~= nil and sub:name() == "M4 sub", "submenu")
  -- 20 parameters in a menu (the sub-menu is one): without the cap all 30 are added, and 256
  -- would wrap num_params in-process
  for _ = 1, 30 do
    menu:add_parameter("z")
  end
  check(menu:num_params() == CRSF_PARAMETERS_PER_MENU, "parameters in a menu: " .. menu:num_params())
  check(sub ~= nil and sub:add_menu("deeper") ~= nil, "submenu of a submenu")
  -- 10 menus in the session, roots and sub-menus together: 3 are made, 7 more
  local menus = 3
  for _ = 1, 20 do
    if crsf:add_menu("M4 more") == nil then
      break
    end
    menus = menus + 1
  end
  check(menus == CRSF_MENUS_PER_SESSION, "menus in the session: " .. menus)
  check(has(error_of(CRSFMenu, "past the cap"), "out of memory"), "CRSFMenu() past the cap")
  -- the cap holds for a sub-menu too
  check(sub ~= nil and sub:add_menu("0123456789ABCDEFGHIJ") == nil, "menu past the cap is nil")
  local param, payload, events = crsf:get_menu_event(3)
  check(math.type(param) == "integer" and type(payload) == "string" and math.type(events) == "integer",
        "get_menu_event")
  -- no CRSFParameter(): its generated constructor leaves the menu pointer unset
  check(CRSFParameter == nil, "no CRSFParameter constructor")
end

local stages = { i2c_devices, pwm_sources, can_buffers, dronecan, dronecan_churn, dronecan_quota,
                 udp_send, udp_receive, tcp_connect, tcp_blocking_send, tcp_sendfile, tcp_sendfile_cap,
                 stream_send, stream_receive, crsf_menus }
local stage = 0

local function update()
  stage = stage + 1
  if stage <= #stages then
    local ok, err = pcall(stages[stage])
    if not ok then
      gcs:send_text(3, string.sub("M4 devices FAILED: " .. tostring(err), 1, 50))
      return
    end
    return update, 1000
  end
  if failures == 0 then
    gcs:send_text(6, "M4 devices passed")
  else
    gcs:send_text(3, "M4 devices failed " .. failures)
  end
end

return update, 1000
