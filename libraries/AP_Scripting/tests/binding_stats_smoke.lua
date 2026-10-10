-- run with a stats build (AP_SCRIPTING_BINDING_STATS_ENABLED) and SCR_DEBUG_OPTS bit 3 (LOG_RUNTIME); errors from a binding must not break stats
local ok = pcall(function() vehicle:set_mode("x") end)
assert(not ok)
for _ = 1, 100 do
  local _ = ahrs:get_location()
  local _ = millis()
end
gcs:send_text(6, "binding_stats_smoke done")
