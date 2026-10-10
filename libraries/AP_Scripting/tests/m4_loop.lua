-- M4 scripting tests: one call per second, so a run (and a run after an M4 restart) shows
local count = 0

local function update()
  count = count + 1
  gcs:send_text(6, string.format("M4 loop %d", count))
  return update, 1000
end

return update()
