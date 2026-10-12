-- Holds the pool's objects (Parameters and the two ends of a serial port), three open files, an
-- aux auth id and unread bytes in both directions of the port, for the whole M4 session.
-- ScriptingM4Objects restarts the M4: the new session must get 64 objects and id 0 again, find
-- no bytes left in the port, and open three files again. If the M7 kept the old objects the new
-- session would get fewer, if it kept the aux auths the id would be 1, if it kept the buffers
-- the leftovers would be 11, and if it kept the files lsof would show six.
-- Needs SCR_SDEV_EN 1 and SCR_SDEV1_PROTO 28.

local dir = nil
for template in string.gmatch(package.path, "[^;]+") do
  local found = string.match(template, "^(.-)/modules/%?%.lua$")
  if found and string.sub(found, 1, 1) ~= "@" then
    dir = found
  end
end

-- globals of the script: never collected
-- luacheck: globals held_objects held_files
held_objects = {}
held_files = {}

-- the port first: its two ends are pool objects, and the Parameters fill what is left
local driver = serial:find_serial(0)
local device = serial.find_simulated_device(28, 0)
held_objects[1] = driver
held_objects[2] = device

local leftover = #driver:readstring(100) + #device:readstring(100)
local port_ok = driver:writestring("0123456789a") == 11 and device:writestring("0123456789a") == 11

for i = 1, 3 do
  local f = dir and io.open(string.format("%s/m4_hold_objects_%d.txt", dir, i), "w")
  if f then
    held_files[#held_files + 1] = f
  end
end

while true do
  local ok, object = pcall(Parameter, "SCR_USER1")
  if not ok then
    break
  end
  held_objects[#held_objects + 1] = object
end
local auth_id = arming:get_aux_auth_id()
gcs:send_text(6, string.format("M4 holding %d objects, auth %s", #held_objects, tostring(auth_id)))
gcs:send_text(6, string.format("M4 hold: %d files, left %d, port %s", #held_files, leftover,
                               port_ok and "ok" or "bad"))

local function update()
  return update, 1000
end

return update, 1000
