-- M4 scripting tests (ScriptingM4FloodCap, arducopter.py): opens @SYS/threads.txt every 10 ms,
-- as a script would to reset the thread statistics the M7 host's duty cap charges. The M7
-- refuses it (EACCES); once a second the counts go out as text
local EACCES = 13
local tries = 0
local refused = 0
local last_report_ms = millis()

local function update()
  local file, _, code = io.open("@SYS/threads.txt", "r")
  tries = tries + 1
  if file then
    file:close()
  elseif code == EACCES then
    refused = refused + 1
  end
  if millis() - last_report_ms >= 1000 then
    last_report_ms = millis()
    gcs:send_text(6, string.format("M4 threads.txt refused %d of %d", refused, tries))
  end
  return update, 10
end

return update()
