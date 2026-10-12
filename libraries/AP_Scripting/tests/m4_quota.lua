-- More M7 work than the quotas allow (spec §7): statustexts past the Text burst are
-- dropped, as from a full queue; storage writes past the Storage burst return false;
-- file logs past the FileLog burst are dropped (each queued one is an M7 allocation).
local saved = 0

local function report()
  gcs:send_text(6, string.format("M4 quota saved %d", saved))
end

local function run()
  for _ = 1, 200 do
    logger:log_file_content("m4_quota_missing.txt")
  end
  for i = 1, 200 do
    gcs:send_text(6, string.format("M4 quota text %d", i))
  end
  for i = 1, 150 do
    if param:set_and_save("SCR_USER3", i) then
      saved = saved + 1
    end
  end
  return report, 3000   -- the Text bucket refills before the report
end

return run, 1000
