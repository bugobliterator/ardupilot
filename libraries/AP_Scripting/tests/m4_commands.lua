-- Scripting stop and restart on the SITL M4 (ScriptingM4Commands, arducopter.py). Each session
-- counts itself in SCR_USER2 (set, not saved: the M7 keeps it across M4 restarts) and prints
-- "M4 commands session <n> loop <k>" every second, and at its start whether it registered the
-- HEARTBEAT filter (only the first session does: the filters stay on the M7, as in-process).
-- SCR_USER5 asks the script to: 1 call scripting:restart_all(); 2 stop scripting through
-- gcs:run_command_int(MAV_CMD_SCRIPTING); 3 restart at the start of each session while SCR_USER3,
-- counted down, is above 0; 4 call scripting:restart_all() every 100 ms.
-- SCR_USER6: 1 block MAV_CMD_USER_1; 2 read mission items; 3 leave them queued.
-- SCR_USER6 4: at its start, 120 param:set_and_save calls (the Storage quota: 10/s, burst 100)
-- and "M4 commands session <n> saves <k>", k the calls that returned true.

local MAV_CMD_USER_1, MAV_CMD_SCRIPTING, SCRIPTING_CMD_STOP = 31000, 42701, 2
local HEARTBEAT_ID = 0
local sessions = Parameter("SCR_USER2")
local loop_restarts = Parameter("SCR_USER3")
local action = Parameter("SCR_USER5")
local mode = Parameter("SCR_USER6")

local session = sessions:get() + 1
sessions:set(session)
-- a caught error each session: on the SITL M4 a caught error's longjmp once left the restart signal
-- blocked, and every later restart of the session timed out ("not responding")
pcall(error, "a caught error")
if mode:get() == 1 then
  mavlink.block_command(MAV_CMD_USER_1)
end
mavlink.init(5, 1)
gcs:send_text(6, string.format("M4 commands session %d registered %s", session,
                               tostring(mavlink.register_rx_msgid(HEARTBEAT_ID))))

-- the restart loop: restart_all() at the start of a session, while the count lasts. The M7 acts
-- on each after the session has run a second, as the in-process thread looks once a second
local restart_asked = false
local function restart_loop()
  local left = loop_restarts:get()
  if left > 0 and not restart_asked then
    restart_asked = true
    loop_restarts:set(left - 1)
    gcs:send_text(6, string.format("M4 commands restart loop %d", left - 1))
    scripting:restart_all()
  end
end
-- the Storage bucket, filled once per boot: the first session finds it full, later ones the refill
if mode:get() == 4 then
  local saved = 0
  for i = 1, 120 do
    if param:set_and_save("SCR_USER4", i) then
      saved = saved + 1
    end
  end
  gcs:send_text(6, string.format("M4 commands session %d saves %d", session, saved))
end
if action:get() == 3 then
  restart_loop()
end

local loops = 0
local fast_passes = 0
local mission_reported = false

local function update()
  local asked = action:get()
  if asked == 4 then
    -- a script that restarts scripting every 100 ms; it prints once a second, before it asks
    fast_passes = fast_passes + 1
    if fast_passes % 10 == 1 then
      loops = loops + 1
      gcs:send_text(6, string.format("M4 commands session %d loop %d", session, loops))
    end
    scripting:restart_all()
    return update, 100
  end
  loops = loops + 1
  gcs:send_text(6, string.format("M4 commands session %d loop %d", session, loops))
  if asked == 1 then
    action:set(0)
    scripting:restart_all()
  elseif asked == 2 then
    action:set(0)
    -- the stop acts at once on the M4: the update ends at this call, so the text never comes
    -- (in-process it does). ScriptingM4Commands checks that nothing follows the stop
    local result = gcs:run_command_int(MAV_CMD_SCRIPTING, {p1 = SCRIPTING_CMD_STOP})
    gcs:send_text(6, "M4 commands stop result " .. tostring(result))
  elseif asked == 3 then
    restart_loop()
  end
  if mode:get() == 2 then
    local t, id = mission_receive()
    if t then
      gcs:send_text(6, "M4 commands mission " .. id)
    elseif not mission_reported then
      mission_reported = true
      gcs:send_text(6, "M4 commands mission none")
    end
  end
  return update, 1000
end

return update, action:get() == 4 and 100 or 1000
