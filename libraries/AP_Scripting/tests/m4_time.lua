-- M4 scripting tests (test_m4_local_time.py): the M7's millis() and micros() as the M4 reads
-- them, sent every 200 ms; the board logs each text with its own time. Each text also carries
-- micros() read after the previous text's call returned, so that call's log time lies between
-- its own micros and that one, whatever the call's latency
local after = 0

local function update()
  local ms = millis()
  local us = micros()
  gcs:send_text(6, "T " .. tostring(ms) .. " " .. tostring(us) .. " " .. tostring(after))
  after = micros()
  return update, 200
end

return update()
