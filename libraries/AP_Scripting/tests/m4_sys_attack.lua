-- M4 scripting tests (ScriptingM4FloodCap, arducopter.py): while SCR_USER1 is 1, loops fs:crc32
-- over a large file (scripts/m4_big.bin, which the test writes) and over @SYS/flash.bin (the
-- whole program flash on ChibiOS; absent in SITL). Each call is a heavy frame for the M7's slow
-- worker, whose CPU the duty cap charges: the M7 pauses the M4 to its share. Once a second the
-- count goes out as text
local calls = 0
local last_report_ms = millis()

local function update()
  if param:get("SCR_USER1") == 1 then
    fs:crc32("scripts/m4_big.bin")
    fs:crc32("@SYS/flash.bin")
    calls = calls + 1
    if millis() - last_report_ms >= 1000 then
      last_report_ms = millis()
      gcs:send_text(6, string.format("M4 attack crc32 %d", calls))
    end
  end
  return update, 1
end

return update()
