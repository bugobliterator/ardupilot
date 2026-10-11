-- Errors while SCR_USER1 is 1. The M7 keeps the error as the prearm failure until a
-- new M4 session; the restarted script then runs clean.
local function update()
  if param:get("SCR_USER1") == 1 then
    error("m4 error once")
  end
  gcs:send_text(6, "M4 error_once idle")
  return update, 1000
end

return update, 1000
