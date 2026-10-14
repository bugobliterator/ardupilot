-- End-to-End Flights: a script that flies with an AUTO mission (ScriptingM4MissionFlight and
-- ScriptingM4MidAutoFaults in arducopter.py, test_m4_flight.py in Renode). Runs unchanged in-process.
--   SCR_USER3 1: in AUTO and disarmed, arm the vehicle (on the M4, arming:arm() runs on the M7 host thread)
--   SCR_USER1: the mission item to hold at (a LOITER_UNLIM); SCR_USER2 above 0: once that item is
--   current, move the mission on to item SCR_USER2. Without the script the vehicle loiters there for good.
-- Every binding call is counted. DONE_UPDATES updates after the release it prints the counts
-- ("M4 count <name> <n>", "M4 total <n>") and makes no further call, so SRPC can be checked against them.
local MODE_AUTO = 3
local UPDATE_MS = 200
local DONE_UPDATES = 25

local counts = {}
local order = {}

local function count(name)
  if not counts[name] then
    counts[name] = 0
    order[#order + 1] = name
  end
  counts[name] = counts[name] + 1
end

local hold_item = param:get("SCR_USER1")
count("param:get")
local arm_wanted = param:get("SCR_USER3") == 1
count("param:get")
local arm_done = false
local released = false
local after_release = 0

local function report()
  -- the report's own texts are counted first: the "done" text (which also lists gcs:send_text
  -- in order if no text came before), then the count lines and the total
  count("gcs:send_text")
  counts["gcs:send_text"] = counts["gcs:send_text"] + #order + 1
  local total = 0
  for _, name in ipairs(order) do
    total = total + counts[name]
  end
  for _, name in ipairs(order) do
    gcs:send_text(6, string.format("M4 count %s %d", name, counts[name]))
  end
  gcs:send_text(6, string.format("M4 total %d", total))
  gcs:send_text(6, "M4 mission done")
end

local function update()
  local mode = vehicle:get_mode()
  count("vehicle:get_mode")
  local index = mission:get_current_nav_index()
  count("mission:get_current_nav_index")
  local release_to = param:get("SCR_USER2")
  count("param:get")
  if arm_wanted and not arm_done and mode == MODE_AUTO then
    arm_done = true
    local armed = arming:is_armed()
    count("arming:is_armed")
    if not armed then   -- a session restarted in flight finds the vehicle armed
      local ok = arming:arm()
      count("arming:arm")
      gcs:send_text(6, "M4 mission armed " .. tostring(ok))
      count("gcs:send_text")
    end
  end
  if not released and index == hold_item and release_to and release_to > 0 then
    released = true
    local ok = mission:set_current_cmd(release_to)
    count("mission:set_current_cmd")
    gcs:send_text(6, string.format("M4 mission release %d to %d %s", index, release_to, tostring(ok)))
    count("gcs:send_text")
  end
  if released then
    after_release = after_release + 1
    if after_release >= DONE_UPDATES then
      report()
      return
    end
  end
  return update, UPDATE_MS
end

return update, UPDATE_MS
