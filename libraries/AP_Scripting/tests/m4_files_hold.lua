-- keeps eight files open for the whole M4 session; ScriptingM4Files counts them in the
-- M7 process (lsof) before and after an M4 restart, which must close the first eight
local dir = nil
for template in string.gmatch(package.path, "[^;]+") do
  local found = string.match(template, "^(.-)/modules/%?%.lua$")
  if found and string.sub(found, 1, 1) ~= "@" then
    dir = found
  end
end

held = {}   -- a global of the script: never collected
for i = 1, 8 do
  local f = dir and io.open(string.format("%s/m4_hold_%d.txt", dir, i), "w")
  if f then
    held[#held + 1] = f
  end
end
gcs:send_text(6, string.format("M4 holding %d files", #held))

local function update()
  return update, 1000
end

return update, 1000
