-- M4 scripting tests: 200 gcs:send_text in one run; the M7's Text quota drops most of them
local flooded = false

local function update()
  if not flooded then
    flooded = true
    for i = 1, 200 do
      gcs:send_text(6, string.format("M4 flood %d", i))
    end
    return update, 3000
  end
  -- the bucket has refilled: this one gets through
  gcs:send_text(6, "M4 text flood done")
end

return update()
