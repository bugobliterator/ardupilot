-- Holds as many Parameter objects as the M7 pool gives, and an aux auth id, for the whole M4
-- session. ScriptingM4Objects restarts the M4: the new session must get 64 objects and id 0
-- again. If the M7 kept the old objects the new session would get none, and if it kept the
-- aux auths the id would be 1

-- a global of the script: never collected
-- luacheck: globals held_objects
held_objects = {}
while #held_objects < 64 do
  local ok, object = pcall(Parameter, "SCR_USER1")
  if not ok then
    break
  end
  held_objects[#held_objects + 1] = object
end
local auth_id = arming:get_aux_auth_id()
gcs:send_text(6, string.format("M4 holding %d objects, auth %s", #held_objects, tostring(auth_id)))

local function update()
  return update, 1000
end

return update, 1000
