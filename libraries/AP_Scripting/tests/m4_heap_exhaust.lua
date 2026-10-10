-- M4 scripting tests: a script that runs out of heap is stopped; the others keep running
local hoard = {}

local function update()
  for _ = 1, 1000 do
    hoard[#hoard + 1] = string.rep("x", 1000) .. #hoard
  end
  return update, 10
end

return update()
