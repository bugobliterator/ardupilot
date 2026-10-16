-- M4 scripting tests (ScriptingM4FloodCap, arducopter.py): while SCR_USER1 is 1, loops fs:crc32
-- over a large file (scripts/m4_big.bin, which the test writes) and over @SYS/flash.bin (the
-- whole program flash on ChibiOS; absent in SITL). Each call is a heavy frame for the M7's slow
-- worker, whose CPU the duty cap charges: the M7 pauses the M4 to its share, and while its debt
-- is beyond 50 ms refuses binding calls for the worker (these, param:get, gcs:send_text) with a
-- Lua error, counted here. Once a second the counts go out as text
local calls = 0
local refused = 0
local last_report_ms = millis()

-- a call on the slow worker; false when the duty cap refused it, any other error is raised
local function worker_call(fn, ...)
  local ok, result = pcall(fn, ...)
  if not ok then
    if string.find(tostring(result), "duty cap", 1, true) == nil then
      error(result, 0)
    end
    refused = refused + 1
    return false, nil
  end
  return true, result
end

local function update()
  local ok, enabled = worker_call(param.get, param, "SCR_USER1")
  if ok and enabled == 1 then
    worker_call(fs.crc32, fs, "scripts/m4_big.bin")
    worker_call(fs.crc32, fs, "@SYS/flash.bin")
    calls = calls + 1
    if millis() - last_report_ms >= 1000 then
      last_report_ms = millis()
      worker_call(gcs.send_text, gcs, 6, string.format("M4 attack crc32 %d refused %d", calls, refused))
    end
  end
  return update, 1
end

return update()
