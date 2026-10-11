-- M4 scripting tests: fills the Lua heap, has newlib allocate outside it (float
-- formatting and a long number parse) while it is full, then runs out of memory.
-- Long strings only (not interned, so each is a new block), at lengths where the
-- SITL cost and newlib-nano's chunk agree: newlib's heap fills as far as the count
local hoard = {}
local sizes = { 1004, 100, 44 }
local digits = "1." .. string.rep("1", 600)

local function fill(size)
  while true do
    hoard[#hoard + 1] = string.rep("x", size)
  end
end

local function update()
  for _, size in ipairs(sizes) do
    pcall(fill, size)
  end
  pcall(string.format, "%.99f", 3.4e38)
  pcall(string.format, "%.99e", 1.4e-45)
  pcall(tonumber, digits)
  fill(1004)   -- not caught: the hog stops with the out-of-memory error
end

return update, 1000
