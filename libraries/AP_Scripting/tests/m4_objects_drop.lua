-- Holds the pool's 64 Parameters, then drops half of them and keeps allocating without a remote
-- call, so the collector queues their releases and nothing sends them: the M4's release queue
-- is not empty when the Renode test restarts the M4. The new session must get 64 objects again
-- and find no stale release in its queue (the queue is in .lua_bss, which the M4 zeroes at boot;
-- a stale one would reach the M7 as a protocol error). "M4 spin done" means the spin ended
-- before the restart, which the test refuses.

-- globals of the script: never collected
-- luacheck: globals held_objects
held_objects = {}

while true do
  local ok, object = pcall(Parameter, "SCR_USER1")
  if not ok then
    break
  end
  held_objects[#held_objects + 1] = object
end
gcs:send_text(6, string.format("M4 holding %d objects", #held_objects))

local function update()
  gcs:send_text(6, "M4 dropping")
  for i = 1, 32 do
    held_objects[i] = nil
  end
  -- garbage only: no binding is called until the spin ends, so no release is sent
  local length = 0
  for i = 1, 20000 do
    length = #(string.rep("g", 8000) .. i)
  end
  gcs:send_text(6, "M4 spin done " .. length)
  return update, 1000
end

return update, 1000
