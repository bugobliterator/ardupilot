-- M4 scripting tests: errors of remote calls carry the in-process texts
local function expect_error(f, text)
  local ok, err = pcall(f)
  assert(not ok, "no error, expected: " .. text)
  assert(string.find(err, text, 1, true), "got: " .. tostring(err))
end

expect_error(function() gcs:send_text("x", "y") end, "bad argument #1 to 'send_text' (number expected, got string)")
expect_error(function() gcs:send_text(9, "y") end, "bad argument #1 to 'send_text' (out of range)")
expect_error(function() gcs:send_text(6, {}) end, "bad argument #2 to 'send_text' (string expected, got table)")
expect_error(function() gcs:send_text(6) end, "bad argument #1 to 'send_text' (too few arguments)")
expect_error(function() gcs:send_text(6, "a", "b") end, "bad argument #3 to 'send_text' (too many arguments)")
gcs:send_text(6, 42)   -- a number converts, as luaL_checkstring does
print("M4 print works")
gcs:send_text(6, "M4 errors passed")
