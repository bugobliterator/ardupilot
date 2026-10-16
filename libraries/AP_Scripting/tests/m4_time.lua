-- M4 scripting tests (test_m4_local_time.py): the M7's millis() and micros() as the M4 reads
-- them, sent every 200 ms; the board logs each text with its own time
local function update()
  local ms = millis()
  local us = micros()
  gcs:send_text(6, "T " .. tostring(ms) .. " " .. tostring(us))
  return update, 200
end

return update()
