-- M4 scripting tests (ScriptingM4FloodCap, arducopter.py): heavy frames for the M7's slow worker.
-- Reads @SYS/tasks.txt (the M7 generates it per open) as fast as the round trips allow, and
-- tries fs:crc32 on @SYS/threads.txt, which the M7 refuses (nil). Once a second the counts go
-- out as text
local reads = 0
local crc_tries = 0
local crc_refused = 0
local last_report_ms = millis()

local function update()
  for _ = 1, 10 do
    local file = io.open("@SYS/tasks.txt", "r")
    if file then
      file:read("a")
      file:close()
      reads = reads + 1
    end
  end
  crc_tries = crc_tries + 1
  if fs:crc32("@SYS/threads.txt") == nil then
    crc_refused = crc_refused + 1
  end
  if millis() - last_report_ms >= 1000 then
    last_report_ms = millis()
    -- reads of tasks.txt, then fs:crc32 refusals of tries (a statustext holds 50 characters)
    gcs:send_text(6, string.format("M4 heavy %d crc refused %d of %d", reads, crc_refused, crc_tries))
  end
  return update, 1
end

return update()
