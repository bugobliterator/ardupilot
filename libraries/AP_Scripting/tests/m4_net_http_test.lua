-- M4 networking tests: a small HTTP server. GET / gives a fixed page with this session's
-- request count, GET /file sends this script with sendfile. With SCR_DEBUG_OPTS 2 each run's
-- "Lua: Time: ... Mem: ..." text gives the Lua heap in use (no collectgarbage() in scripts)
local PARAM_TABLE_KEY = 97
assert(param:add_table(PARAM_TABLE_KEY, "MHT_", 1), "could not add MHT_ table")
assert(param:add_param(PARAM_TABLE_KEY, 1, "PORT", 8081), "could not add MHT_PORT")
local port = math.floor(Parameter("MHT_PORT"):get())

local FILE_PATHS = {"scripts/m4_net_http_test.lua", "APM/scripts/m4_net_http_test.lua"}

local listener = Socket(0)
if not listener or not listener:bind("0.0.0.0", port) or not listener:listen(2) then
  gcs:send_text(3, "M4 HTTP: no listener on " .. port)
  return
end
gcs:send_text(6, "M4 HTTP: listening on " .. port)

local requests = 0

local function open_file()
  for _, path in ipairs(FILE_PATHS) do
    local file = io.open(path, "rb")
    if file then
      return file
    end
  end
end

local function send(sock, text)
  sock:send(text, #text)
end

local function serve(sock, request)
  requests = requests + 1
  local path = request:match("^GET (%S+) HTTP")
  if path == "/" then
    local body = string.format("M4 HTTP test, request %d\n", requests)
    send(sock, string.format("HTTP/1.0 200 OK\r\nX-Request-Count: %d\r\nContent-Length: %d\r\n\r\n%s",
                             requests, #body, body))
  elseif path == "/file" then
    local file = open_file()
    if file then
      send(sock, string.format("HTTP/1.0 200 OK\r\nX-Request-Count: %d\r\n\r\n", requests))
      if not sock:sendfile(file) then
        gcs:send_text(3, "M4 HTTP: sendfile failed")
        file:close()
      end
    else
      send(sock, "HTTP/1.0 404 Not Found\r\n\r\n")
    end
  else
    send(sock, "HTTP/1.0 404 Not Found\r\n\r\n")
  end
  gcs:send_text(6, string.format("M4 HTTP: request %d", requests))
end

local function update()
  if listener:pollin(0) then
    local sock = listener:accept()
    if sock then
      sock:set_blocking(true)
      if sock:pollin(500) then
        local request = sock:recv(512)
        if request then
          serve(sock, request)
        end
      end
      sock:close()
    end
  end
  return update, 200
end

return update()
