-- Owned M7 objects on the M4 (ScriptingM4Bindings, arducopter.py): every Parameter holds a
-- slot of the M7's 64-slot object pool until the M4 collects it and sends its release. M4
-- only: in-process there is no pool, so the limit stage would never be refused.

local TOO_MANY = "M4: too many objects"

local function check(condition, text)
  if not condition then
    error("M4 owned: " .. text, 2)
  end
end

-- collectgarbage is not in the sandbox: allocate until done() or the bound
local function allocate_until(done)
  for _ = 1, 200 do
    if done() then
      return true
    end
    local garbage = {}
    for i = 1, 100 do
      garbage[i] = string.rep("g", 256) .. i
    end
    check(#garbage == 100, "garbage filled")
  end
  return done()
end

local held = {}

local stages = {
  function()   -- many short-lived objects in one run: the pool only refuses when 64 are live
    for i = 1, 200 do
      local p = Parameter("SCR_USER3")
      check(p ~= nil, "churn " .. i)
    end
  end,
  function()   -- the limit: 64 live objects, then the M7's refusal (after the M4's collect and retry)
    local err
    for _ = 1, 100 do
      local ok, result = pcall(Parameter, "SCR_USER3")
      if not ok then
        err = tostring(result)
        break
      end
      held[#held + 1] = result
    end
    check(#held == 64, "held " .. #held)
    check(err ~= nil and string.find(err, TOO_MANY, 1, true) ~= nil, "limit '" .. tostring(err) .. "'")
  end,
  function()   -- a call from a finalizer at the limit is refused without a collection nested in it
    -- outer and inner finalize in the same cycle (inner is reachable only through outer), outer
    -- first (newest first). A collection inside outer's call would run inner's finalizer nested in it
    local in_outer = false
    local outer_result, inner_nested
    local inner = setmetatable({}, { __gc = function() inner_nested = in_outer end })
    local outer = setmetatable({ inner = inner }, { __gc = function()   -- luacheck: ignore 231
      in_outer = true
      local ok, result = pcall(Parameter, "SCR_USER3")
      in_outer = false
      outer_result = ok and "made" or tostring(result)
    end })
    inner = nil   -- luacheck: ignore 311
    outer = nil   -- luacheck: ignore 311
    check(allocate_until(function() return outer_result ~= nil and inner_nested ~= nil end), "finalizers ran")
    check(string.find(outer_result, TOO_MANY, 1, true) ~= nil, "finalizer call '" .. outer_result .. "'")
    check(inner_nested == false, "collection nested in a finalizer")
  end,
  function()   -- init_by_info is not served on the M4 (an M7 allocation per call that nothing frees)
    local ok, err = pcall(function() return held[1]:init_by_info(1, 0, 3) end)
    check(not ok and string.find(tostring(err), "init_by_info not supported on M4", 1, true) ~= nil,
          "init_by_info '" .. tostring(err) .. "'")
    check(held[1]:get() ~= nil, "Parameter after init_by_info")
  end,
  function()   -- dropped objects are released: 64 new ones fit again
    held = {}
    for i = 1, 64 do
      held[i] = Parameter("SCR_USER3")
    end
    check(held[64]:set(3), "set")
    check(held[1]:get() == 3, "get")
    held = {}
  end,
}
local stage = 0

local function update()
  stage = stage + 1
  if stage <= #stages then
    local ok, err = pcall(stages[stage])
    if not ok then
      gcs:send_text(3, string.sub("M4 owned FAILED: " .. tostring(err), 1, 50))
      return
    end
    return update, 500
  end
  gcs:send_text(6, "M4 owned objects passed")
end

return update, 1000
