-- Long numerals while the Lua heap is full. On the M4 a numeral longer than
-- AP_LUA_MAX_NUMBER_TEXT (100) reads as not a number, so strtof never runs on
-- it and cannot take newlib's reserve; in-process the same text is inf.

local long = string.rep("9", 3000)
local medium = string.rep("1", 100)
local long_value
local medium_value
local short_value

local function check(condition, text)
  if not condition then
    error("M4 number text: " .. text, 2)
  end
end

local function report()
  check(long_value == nil, "a 3000-digit numeral converted to " .. tostring(long_value))
  check(medium_value ~= nil, "a 100-digit numeral did not convert")
  check(short_value == 12.5, "12.5 read as " .. tostring(short_value))
  gcs:send_text(6, "M4 number text ok")
end

local function run()
  local filler = {}
  pcall(function()
    while true do
      filler[#filler + 1] = string.rep("x", 1024)
    end
  end)
  -- the heap is full: only newlib's reserve is left for strtof
  long_value = tonumber(long)
  medium_value = tonumber(medium)
  short_value = tonumber("12.5")
  filler = nil
  return report, 1000
end

return run, 1000
