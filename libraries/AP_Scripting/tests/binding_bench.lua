-- Calls a fixed set of representative bindings N times per run and reports the average
-- cost per call measured in Lua. Runs identically on single-core and M4 remote builds.
local N = 50
local PERIOD_MS = 1000
local REPORT_MS = 10000

local cases = {
  { "ahrs:get_location", function() return ahrs:get_location() end },           -- AHRS semaphore getter
  { "vehicle:get_mode", function() return vehicle:get_mode() end },             -- scheduler semaphore
  { "param:get", function() return param:get("SCR_ENABLE") end },
  { "millis", function() return millis() end },                                 -- manual binding
  { "gps:status", function() return gps:status(0) end },
  { "battery:voltage", function() return battery:voltage(0) end },
  { "SRV_Channels:get_output_pwm", function() return SRV_Channels:get_output_pwm(33) end },
}
if mount then
  cases[#cases+1] = { "mount:get_mode", function() return mount:get_mode(0) end }
end

local acc = {}
for i = 1, #cases do acc[i] = { n = 0, us = 0 } end
local last_report = millis()

local function update()
  for i, c in ipairs(cases) do
    local t0 = micros()
    for _ = 1, N do c[2]() end
    local dt = (micros() - t0):toint()
    acc[i].n = acc[i].n + N
    acc[i].us = acc[i].us + dt
  end
  if (millis() - last_report):toint() >= REPORT_MS then
    for i, c in ipairs(cases) do
      if acc[i].n > 0 then
        gcs:send_text(6, string.format("BENCH %s n=%d avg_us=%.1f", c[1], acc[i].n, acc[i].us / acc[i].n))
      end
      acc[i] = { n = 0, us = 0 }
    end
    last_report = millis()
  end
  return update, PERIOD_MS
end

return update()
