-- io, require's file calls, dirlist and remove on the M4, through the M7's file proxy
-- (ScriptingM4Files, arducopter.py): in-process results, relative names included;
-- refusals for ".." above the root and writes on the "@" filesystems. M4 only: in-process
-- the refusal lines would succeed, and the standard-stream lines dereference (FILE *)1..3.

local EACCES = 13
local EROFS = 30
local EMFILE = 24

local failures = 0
local function check(ok, name)
  if not ok then
    failures = failures + 1
    gcs:send_text(3, "M4 files FAIL " .. name)
  end
end

-- the scripts directory as the M7 names it: the package.path entry that is not ROMFS
local function scripts_dir()
  for template in string.gmatch(package.path, "[^;]+") do
    local dir = string.match(template, "^(.-)/modules/%?%.lua$")
    if dir and string.sub(dir, 1, 1) ~= "@" then
      return dir
    end
  end
  return nil
end

local dir = scripts_dir()
local path = dir and (dir .. "/m4_files.txt")

-- io.open results of a refused open: nil, text, errno
local function refused(name, errno, file, _, code)
  check(file == nil, name)
  check(code == errno, name .. " errno " .. tostring(code))
  if file then
    file:close()
  end
end

local function round_trip()
  local f = io.open(path, "w")
  check(f ~= nil, "open w")
  if not f then
    return
  end
  check(f:write("line1\n", 42, "\n", 1.5, "\n") == f, "write")
  check(f:close() == true, "close")
  local lines = {}
  for line in io.lines(path) do
    lines[#lines + 1] = line
  end
  check(#lines == 3 and lines[1] == "line1" and lines[2] == "42" and lines[3] == "1.5", "lines")
  f = io.open(path, "r")
  check(f:read("l") == "line1", "read l")
  check(f:read("n") == 42, "read n")
  check(f:seek("cur") == 9, "seek cur")              -- past "42\n": read("n") ungot the newline
  check(f:seek("set", 2) == 2 and f:read(3) == "ne1", "seek set")
  check(f:seek("end") == 13, "seek end")
  check(f:read("a") == "", "read a at end")
  check(f:read("l") == nil, "read l at end")
  f:close()
  -- larger than a frame and than the read-ahead: back whole
  local big = string.rep("0123456789abcdef", 200)
  f = io.open(path, "w")
  f:write(big)
  f:close()
  f = io.open(path, "rb")
  check(f:read("a") == big, "big read")
  f:close()
  f = io.open(path, "a")
  f:write("tail")
  f:close()
  f = io.open(path, "r")
  f:seek("end", -4)
  check(f:read("a") == "tail", "append")
  f:close()
end

local function reach()
  -- a relative name resolves as in-process: the working directory
  for _, name in ipairs({ "m4_files_cwd.txt", "miss.txt" }) do
    local f = io.open(name, "w")
    check(f ~= nil, "relative write " .. name)
    if f then
      f:write("cwd")
      f:close()
    end
    f = io.open(name, "r")
    check(f ~= nil and f:read("a") == "cwd", "relative read " .. name)
    if f then
      f:close()
    end
    check(remove(name) == true, "relative remove " .. name)
    check(io.open(name, "r") == nil, "relative removed " .. name)
  end
  check(type(dirlist(".")) == "table", "dirlist relative")
  -- no climbing above the root
  refused("dotdot", EACCES, io.open("../m4_escape.txt", "w"))
  refused("deep dotdot", EACCES, io.open(dir .. "/../../../../m4_escape.txt", "w"))
  refused("backslash dotdot", EACCES, io.open("m4_cwd\\..\\..\\m4_escape.txt", "w"))
  refused("dotdot read", EACCES, io.open("../m4_escape.txt", "r"))
  local listing, err = dirlist(dir .. "/../../../..")
  -- dirlist returns nil and strerror(errno), as in-process
  check(listing == nil and err == "Permission denied", "dirlist above root")
  -- no writes on the @ filesystems, with or without a leading "/": EROFS, not ENOENT.
  -- Proofs of the M7 check: @PARAM and @MISSION answer ENOENT to these names on their own,
  -- and the removes below return errno 0. @SYS and @ROMFS refuse writes themselves: parity checks.
  refused("param write", EROFS, io.open("@PARAM/m4_escape.txt", "w"))
  refused("slash param write", EROFS, io.open("/@PARAM/m4_escape.txt", "w"))
  refused("mission write", EROFS, io.open("@MISSION/m4_escape.txt", "w"))
  refused("slash mission write", EROFS, io.open("/@MISSION/m4_escape.txt", "w"))
  refused("sys write", EROFS, io.open("@SYS/m4_escape.txt", "w"))
  refused("slash sys write", EROFS, io.open("/@SYS/m4_escape.txt", "w"))
  refused("romfs write", EROFS, io.open("@ROMFS/scripts/m4_escape.lua", "w"))
  local ok, msg, code = remove("@SYS/threads.txt")
  check(ok == nil and type(msg) == "string" and code == EROFS, "sys remove")
  ok, msg, code = remove("/@PARAM/m4_escape.txt")
  check(ok == nil and type(msg) == "string" and code == EROFS, "slash param remove")
  local file, text, errno = io.open("../m4_escape.txt", "r")
  check(file == nil and string.find(text, "../m4_escape.txt", 1, true) == 1 and errno == EACCES, "refusal text")
  -- except @SYS/threads.txt: it resets the thread statistics the M7 host's duty cap charges
  refused("sys threads read", EACCES, io.open("@SYS/threads.txt", "r"))
  refused("slash sys threads read", EACCES, io.open("/@SYS//threads.txt", "r"))
  -- reads of the @ filesystems stay allowed
  local sys = io.open("@SYS/tasks.txt", "r")   -- SITL's threads.txt is empty, so ENOENT
  check(sys ~= nil, "sys read")
  if sys then
    sys:close()
  end
  -- the standard streams are no files on the M4
  check(io.write("x") == nil, "stdout")
  check(io.read("l") == nil, "stdin")
end

local function listing_and_remove()
  local f = io.open(path, "w")
  f:write("x")
  f:close()
  local found = false
  for _, name in ipairs(dirlist(dir) or {}) do
    if name == "m4_files.txt" then
      found = true
    end
  end
  check(found, "dirlist")
  check(remove(path) == true, "remove")
  local ok, msg, code = remove(path)
  check(ok == nil and type(msg) == "string" and math.type(code) == "integer", "remove twice")
  check(io.open(path, "r") == nil, "removed")
end

local function open_files()
  local function name(i)
    return string.format("%s/m4_files_%d.txt", dir, i)
  end
  local held = {}
  local opened = 0
  for i = 1, 8 do
    held[i] = io.open(name(i), "w")
    opened = opened + (held[i] and 1 or 0)
  end
  check(opened == 8, "eight open")
  local ninth, err, ninth_code = io.open(name(9), "w")
  check(ninth == nil and type(err) == "string" and ninth_code == EMFILE, "ninth refused")
  held = nil   -- luacheck: ignore 311
  -- collectgarbage is not in the sandbox: allocate until the collector has run the finalizers
  -- that close the dropped files on the M7 (re-opening for reading, which is not quota'd)
  local again = {}
  local reopened = 0
  for _ = 1, 200 do
    local garbage = {}
    for i = 1, 100 do
      garbage[i] = string.rep("g", 256) .. i
    end
    check(#garbage == 100, "garbage filled")
    for i = 1, 8 do
      if not again[i] then
        again[i] = io.open(name(i), "r") or false
        reopened = reopened + (again[i] and 1 or 0)
      end
    end
    if reopened == 8 then
      break
    end
  end
  check(reopened == 8, "eight again after collection")
  for i = 1, 8 do
    if again[i] then
      again[i]:close()
    end
    remove(name(i))
  end
end

local function quota()
  -- 400 writes, far past the FileIo burst (100, refilled at 50/s): some fail with EAGAIN
  local f = io.open(path, "w")
  local refusals = 0
  local errno = nil
  for _ = 1, 400 do
    local ok, _, code = f:write("x")
    if ok == nil then
      refusals = refusals + 1
      errno = code
    end
  end
  f:close()
  -- EAGAIN: 35 on macOS and 11 on Linux and newlib (SITL's M7 is the host, FatFs builds use newlib)
  check(refusals > 0 and (errno == 35 or errno == 11), "quota")
end

local function cleanup()
  remove(path)
  for i = 1, 8 do
    remove(string.format("%s/m4_files_%d.txt", dir, i))
  end
end

local stages = { round_trip, reach, listing_and_remove, open_files, quota }
local stage = 0

local function update()
  if not dir then
    gcs:send_text(3, "M4 files FAIL no scripts directory in package.path")
    gcs:send_text(3, "M4 files failed 1")
    return
  end
  stage = stage + 1
  if stage <= #stages then
    local ok, err = pcall(stages[stage])
    if not ok then
      gcs:send_text(3, string.sub("M4 files FAILED: " .. tostring(err), 1, 50))
      cleanup()
      return
    end
    return update, 2000   -- the FileIo bucket refills between stages
  end
  cleanup()
  if failures == 0 then
    gcs:send_text(6, "M4 files passed")
  else
    gcs:send_text(3, "M4 files failed " .. failures)
  end
end

return update, 1000
