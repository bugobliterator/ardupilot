--[[
   example script to test lua socket API
--]]

PARAM_TABLE_KEY = 47
PARAM_TABLE_PREFIX = "WEB_"

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- Setup Parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 6), 'net_test: could not add param table')

--[[
  // @Param: WEB_ENABLE
  // @DisplayName: enable web server
  // @Description: enable web server
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local WEB_ENABLE = bind_add_param('ENABLE',  1, 1)

--[[
  // @Param: WEB_BIND_PORT
  // @DisplayName: web server TCP port
  // @Description: web server TCP port
  // @Range: 1 65535
  // @User: Standard
--]]
local WEB_BIND_PORT = bind_add_param('BIND_PORT', 2, 80)

--[[
  // @Param: WEB_DEBUG
  // @DisplayName: web server debugging
  // @Description: web server debugging
  // @Values: 0:Disabled,1:Enabled
  // @User: Advanced
--]]
local WEB_DEBUG = bind_add_param('DEBUG', 3, 0)

--[[
  // @Param: WEB_BLOCK_SIZE
  // @DisplayName: web server block size
  // @Description: web server block size for download
  // @Range: 1 65535
  // @User: Advanced
--]]
local WEB_BLOCK_SIZE = bind_add_param('BLOCK_SIZE', 4, 10240)

--[[
  // @Param: WEB_TIMEOUT
  // @DisplayName: web server timeout
  // @Description: timeout for inactive connections
  // @Units: s
  // @Range: 0.1 60
  // @User: Advanced
--]]
local WEB_TIMEOUT = bind_add_param('TIMEOUT', 5, 2.0)

--[[
  // @Param: WEB_SENDFILE_MIN
  // @DisplayName: web server minimum file size for sendfile
  // @Description: sendfile is an offloading mechanism for faster file download. If this is non-zero and the file is larger than this size then sendfile will be used for file download
  // @Range: 0 10000000
  // @User: Advanced
--]]
local WEB_SENDFILE_MIN = bind_add_param('SENDFILE_MIN', 6, 100000)

if WEB_ENABLE:get() ~= 1 then
   gcs:send_text(0, "WebServer: disabled")
   return
end

gcs:send_text(0, string.format("WebServer: starting on port %u", WEB_BIND_PORT:get()))

local sock_listen = Socket(0)
local clients = {}

local DOCTYPE = "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 3.2 Final//EN\">"
local SERVER_VERSION = "net_webserver 1.0"
local CONTENT_TEXT_HTML = "text/html;charset=UTF-8"
local CONTENT_OCTET_STREAM = "application/octet-stream"

local HIDDEN_FOLDERS = { "@SYS", "@ROMFS", "@MISSION", "@PARAM" }

local MNT_PREFIX = "/mnt"
local MNT_PREFIX2 = MNT_PREFIX .. "/"

local MIME_TYPES = {
   ["apj"] = CONTENT_OCTET_STREAM,
   ["dat"] = CONTENT_OCTET_STREAM,
   ["o"] = CONTENT_OCTET_STREAM,
   ["obj"] = CONTENT_OCTET_STREAM,
   ["lua"] = "text/x-lua",
   ["py"] = "text/x-python",
   ["shtml"] = CONTENT_TEXT_HTML,
   ["js"] = "text/javascript",
   -- thanks to https://developer.mozilla.org/en-US/docs/Web/HTTP/Basics_of_HTTP/MIME_types/Common_types
   ["aac"] = "audio/aac",
   ["abw"] = "application/x-abiword",
   ["arc"] = "application/x-freearc",
   ["avif"] = "image/avif",
   ["avi"] = "video/x-msvideo",
   ["azw"] = "application/vnd.amazon.ebook",
   ["bin"] = "application/octet-stream",
   ["bmp"] = "image/bmp",
   ["bz"] = "application/x-bzip",
   ["bz2"] = "application/x-bzip2",
   ["cda"] = "application/x-cdf",
   ["csh"] = "application/x-csh",
   ["css"] = "text/css",
   ["csv"] = "text/csv",
   ["doc"] = "application/msword",
   ["docx"] = "application/vnd.openxmlformats-officedocument.wordprocessingml.document",
   ["eot"] = "application/vnd.ms-fontobject",
   ["epub"] = "application/epub+zip",
   ["gz"] = "application/gzip",
   ["gif"] = "image/gif",
   ["htm"] = CONTENT_TEXT_HTML,
   ["html"] = CONTENT_TEXT_HTML,
   ["ico"] = "image/vnd.microsoft.icon",
   ["ics"] = "text/calendar",
   ["jar"] = "application/java-archive",
   ["jpeg"] = "image/jpeg",
   ["json"] = "application/json",
   ["jsonld"] = "application/ld+json",
   ["mid"] = "audio/x-midi",
   ["mjs"] = "text/javascript",
   ["mp3"] = "audio/mpeg",
   ["mp4"] = "video/mp4",
   ["mpeg"] = "video/mpeg",
   ["mpkg"] = "application/vnd.apple.installer+xml",
   ["odp"] = "application/vnd.oasis.opendocument.presentation",
   ["ods"] = "application/vnd.oasis.opendocument.spreadsheet",
   ["odt"] = "application/vnd.oasis.opendocument.text",
   ["oga"] = "audio/ogg",
   ["ogv"] = "video/ogg",
   ["ogx"] = "application/ogg",
   ["opus"] = "audio/opus",
   ["otf"] = "font/otf",
   ["png"] = "image/png",
   ["pdf"] = "application/pdf",
   ["php"] = "application/x-httpd-php",
   ["ppt"] = "application/vnd.ms-powerpoint",
   ["pptx"] = "application/vnd.openxmlformats-officedocument.presentationml.presentation",
   ["rar"] = "application/vnd.rar",
   ["rtf"] = "application/rtf",
   ["sh"] = "application/x-sh",
   ["svg"] = "image/svg+xml",
   ["tar"] = "application/x-tar",
   ["tif"] = "image/tiff",
   ["tiff"] = "image/tiff",
   ["ts"] = "video/mp2t",
   ["ttf"] = "font/ttf",
   ["txt"] = "text/plain",
   ["vsd"] = "application/vnd.visio",
   ["wav"] = "audio/wav",
   ["weba"] = "audio/webm",
   ["webm"] = "video/webm",
   ["webp"] = "image/webp",
   ["woff"] = "font/woff",
   ["woff2"] = "font/woff2",
   ["xhtml"] = "application/xhtml+xml",
   ["xls"] = "application/vnd.ms-excel",
   ["xlsx"] = "application/vnd.openxmlformats-officedocument.spreadsheetml.sheet",
   ["xml"] = "default.",
   ["xul"] = "application/vnd.mozilla.xul+xml",
   ["zip"] = "application/zip",
   ["3gp"] = "video",
   ["3g2"] = "video",
   ["7z"] = "application/x-7z-compressed",
}

--[[
   builtin static pages
--]]
local STATIC_PAGES = {
["/"] = [[
<!doctype html>
<html lang="en">
<script src="https://cdn.plot.ly/plotly-latest.min.js"></script>
<script type="application/javascript">
    function resizeIFrameToFitContent(iFrame) {
        iFrame.width = iFrame.contentWindow.document.body.scrollWidth;
        iFrame.height = iFrame.contentWindow.document.body.scrollHeight;
    }
    function dynamic_load(div_id, uri, period_ms) {
        var xhr = new XMLHttpRequest();
        xhr.open('GET', uri);

        xhr.setRequestHeader("Cache-Control", "no-cache, no-store, max-age=0");
        xhr.setRequestHeader("Expires", "Tue, 01 Jan 1980 1:00:00 GMT");
        xhr.setRequestHeader("Pragma", "no-cache");

        xhr.onload = function () {
            if (xhr.status === 200) {
                var output = document.getElementById(div_id);
                if (uri.endsWith('.shtml') || uri.endsWith('.html')) {
                    output.innerHTML = xhr.responseText;
                } else if (uri.endsWith('.json')) {
                    var json = JSON.parse(xhr.responseText);
                    var table = "<table>";
                    for (var key in json) {
                        table += "<tr><td>" + key + "</td><td>" + JSON.stringify(json[key]) + "</td></tr>";
                    }
                    table += "</table>";
                    output.innerHTML = table;
                } else {
                    output.textContent = xhr.responseText;
                }
            }
            if (period_ms > 0) {
                setTimeout(function () { dynamic_load(div_id, uri, period_ms); }, period_ms);
            }
        }
        xhr.send();
    }
    var plots = {};
    colors = ['#80CAF6', '#DF56F1', '#F6A280', '#F1DF56', '#80F6CA', '#56F1DF', '#A280F6', '#56DF56', '#F6CA80', '#F1DF56'];
    var oldest_time;
    var latest_time;
    function dynamic_plot(div_id, title, yaxis_label, fields, uri, period_ms) {
        if (!plots[div_id]) {
            var plot_fields = [];
            var time = new Date();
            for (var i = 0; i < fields.length; i++) {
                plot_fields.push({
                    x: [],
                    y: [],
                    mode: 'lines',
                    name: fields[i],
                    line: { color: colors[i] }
                });
            }
            oldest_time = time.setMinutes(time.getMinutes() - 1);;
            latest_time = time.setMinutes(time.getMinutes() + 1);
            plots[div_id] = Plotly.newPlot(div_id, plot_fields, 
                {
                    title: title,
                    xaxis: {
                        title: 'Time',
                        type: 'date',
                        autorange: false,
                        range: [oldest_time, latest_time],
                    },
                    yaxis: {
                        title: yaxis_label,
                        fixedrange: true
                    }
                },
                {scrollZoom: true}
            );
        }
        var xhr = new XMLHttpRequest();
        xhr.open('GET', uri);
        xhr.setRequestHeader("Cache-Control", "no-cache, no-store, max-age=0");
        xhr.setRequestHeader("Expires", "Tue, 01 Jan 1980 1:00:00 GMT");
        xhr.setRequestHeader("Pragma", "no-cache");

        xhr.onload = function () {
            if (xhr.status === 200) {
                var json = JSON.parse(xhr.responseText);
                var data = json['data'];
                var plot_indices = [];
                var time = new Date();
                var times = [];
                // check current x-axis range
                var layout = document.getElementById(div_id).layout;
                var lower_range = new Date(layout.xaxis.range[0]);
                var upper_range = new Date(layout.xaxis.range[1]);
                var xaxis_delta = upper_range - lower_range;
                if (time > upper_range) {
                    oldest_time = time - xaxis_delta;
                    // relayout to keep x-axis range
                    Plotly.relayout(div_id, {
                        xaxis: {
                            type: 'date',
                            range: [oldest_time, time]
                        }
                    });
                }
                for (var i = 0; i < fields.length; i++) {
                    plot_indices.push(i);
                    times.push([time]);
                }
                Plotly.extendTraces(div_id, {
                    x: times,
                    y: data
                }, plot_indices);
            }
            if (period_ms > 0) {
                setTimeout(function () { dynamic_plot(div_id, title, yaxis_label, fields, uri, period_ms); }, period_ms);
            }
        }
        xhr.send();
    }

    window.addEventListener('DOMContentLoaded', function (e) {
        var iFrame = document.getElementById('transFrame');
        resizeIFrameToFitContent(iFrame);
    });
</script>

<head>
    <meta charset="utf-8">
    <title>CubeRed</title>
</head>

<h2>CubeRed Gateway</h2>

<body onload="javascript: dynamic_load('board_status','/@DYNAMIC/board_status.json',1000);
                          dynamic_load('health_status','/@DYNAMIC/health_status.json',1000);
                          dynamic_plot('temp_graph','Temperature', 'Temperature (C)', ['IMU0 Temperature', 'IMU1 Temperature', 'IMU2 Temperature', 'Baro Temperature'],'/@DYNAMIC/temperatures.json',1000);
                          dynamic_plot('baro_graph','Barometer', 'Pressure (Pa)', ['Barometer Pressure'],'/@DYNAMIC/barometer.json',1000);">
    <h2>Controller Status</h2>
    <div id="board_status"></div>
    <h2>Tests</h2>
    <div id="health_status"></div>
    <div id="temp_graph" style="width:50%; display:inline-block;"></div><div id="baro_graph" style="width:50%; display:inline-block;"></div>
    <iframe name="transFrame" id="transFrame" style="height:200px;width:100%;border:none;overflow:hidden;"></iframe>
</body>

</html>
]],
}

--[[
 builtin dynamic pages
--]]
local DYNAMIC_PAGES = {
-- board status section on front page
["@DYNAMIC/board_status.json"] = [[
{
   "Firmware":"<?lstr FWVersion:string() ?>",
   "GitHash":"<?lstr FWVersion:hash() ?>",
   "Uptime":"<?lstr hms_uptime() ?>"
}
]],
["@DYNAMIC/temperatures.json"] = [[
{
   "data":[
   [<?lstr ins:get_temperature(0) ?>],
   [<?lstr ins:get_temperature(1) ?>],
   [<?lstr ins:get_temperature(2) ?>],
   [<?lstr baro:get_temperature() ?>]
   ]
}
]],
["@DYNAMIC/barometer.json"] = [[
{
   "data":[
   [<?lstr baro:get_pressure() ?>]
   ]
}
]],
["@DYNAMIC/health_status.json"] = [[
{
   "IMU[0]":"<?lstr read_health(ins:get_accel_health(0)) ?>",
   "IMU[1]":"<?lstr read_health(ins:get_accel_health(1)) ?>",
   "IMU[2]":"<?lstr read_health(ins:get_accel_health(2)) ?>",
   "BARO":"<?lstr read_health(baro:healthy(0)) ?>",
   "SDCard":"<?lstr read_health(sdcard_healthy) ?>"
}
]],
}

if not sock_listen:bind("0.0.0.0", WEB_BIND_PORT:get()) then
   gcs:send_text(0, string.format("WebServer: failed to bind to TCP %u", WEB_BIND_PORT:get()))
   return
end

if not sock_listen:listen(20) then
   gcs:send_text(0, "WebServer: failed to listen")
   return
end

function read_health(is_healthy)
    if is_healthy then
        return "Healthy"
    end
    return "Unhealthy"
end

-- create json response object
function json_response(obj)
   local ret = "{"
   for k,v in pairs(obj) do
      if v == "null" then
         ret = ret .. string.format('"%s":null,', k)
      elseif type(v) == "string" then
         ret = ret .. string.format('"%s":"%s",', k, v)
      else
         ret = ret .. string.format('"%s":%s,', k, tostring(v))
      end
   end
   ret = string.sub(ret, 1, #ret-1) .. "}"
   return ret
end

function hms_uptime()
   local s = (millis()/1000):toint()
   local min = math.floor(s / 60) % 60
   local hr = math.floor(s / 3600)
   return string.format("%u hours %u minutes %u seconds", hr, min, s%60)
end

--[[
   split string by pattern
--]]
local function split(str, pattern)
   local ret = {}
   for s in string.gmatch(str, pattern) do
      table.insert(ret, s)
   end
   return ret
end

--[[
   return true if a string ends in the 2nd string
--]]
local function endswith(str, s)
   local len1 = #str
   local len2 = #s
   return string.sub(str,1+len1-len2,len1) == s
end

--[[
   return true if a string starts with the 2nd string
--]]
local function startswith(str, s)
   return string.sub(str,1,#s) == s
end

local debug_count=0

function DEBUG(txt)
   if WEB_DEBUG:get() ~= 0 then
      gcs:send_text(0, txt .. string.format(" [%u]", debug_count))
      debug_count = debug_count + 1
   end
end

--[[
   return index of element in a table
--]]
function table_index(t,el)
   for i,v in ipairs(t) do
      if v == el then
         return i
      end
   end
   return nil
end

--[[
   return true if a table contains a given element
--]]
function table_contains(t,el)
   local i = table_index(t, el)
   return i ~= nil
end

function is_hidden_dir(path)
   return table_contains(HIDDEN_FOLDERS, path)
end

--[[
   substitute variables of form {xxx} from a table
   from http://lua-users.org/wiki/StringInterpolation
--]]
function substitute_vars(s, vars)
   s = (string.gsub(s, "({([^}]+)})",
                    function(whole,i)
                       return vars[i] or whole
   end))
   return s
end

--[[
   client class for open connections
--]]
local function Client(_sock, _idx)
   local self = {}

   self.closed = false

   local sock = _sock
   local idx = _idx
   local have_header = false
   local header = ""
   local header_lines = {}
   local header_vars = {}
   local form_data = {}
   local boundary_str = nil
   local run = nil
   local protocol = nil
   local file = nil
   local start_time = millis()
   local offset = 0
   local file_size = 0
   local close_connection = false

   function self.read_header()
      local s = sock:recv(2048)
      if not s then
         local now = millis()
         if not sock:is_connected() then
            -- EOF while looking for header
            DEBUG(string.format("%u: EOF", idx))
            self.remove()
            return false
         end
         return false
      end
      if not s or #s == 0 then
         return false
      end
      header = header .. s
      local eoh = string.find(s, '\r\n\r\n')
      if eoh then
         DEBUG(string.format("%u: got header", idx))
         have_header = true
         header_lines = split(header, "[^\r\n]+")
         -- blocking for reply
         sock:set_blocking(true)
         return true
      end
      return false
   end

   function self.sendstring(s)
      sock:send(s, #s)
   end

   function self.sendline(s)
      self.sendstring(s .. "\r\n")
   end

   --[[
      send a string with variable substitution using {varname}
   --]]
   function self.sendstring_vars(s, vars)
      self.sendstring(substitute_vars(s, vars))
   end
   
   function self.send_header(code, codestr, vars)
      self.sendline(string.format("%s %u %s", protocol, code, codestr))
      self.sendline(string.format("Server: %s", SERVER_VERSION))
      for k,v in pairs(vars) do
         self.sendline(string.format("%s: %s", k, v))
      end
      self.sendline("Connection: close")
      self.sendline("")
   end

   function self.send_chunk_header(code, codestr, vars)
      local send_str = string.format("%s %u %s\r\n", protocol, code, codestr)
      send_str = send_str .. string.format("Server: %s\r\n", SERVER_VERSION)
      for k,v in pairs(vars) do
         send_str = send_str .. string.format("%s: %s\r\n", k, v)
      end
      send_str = send_str .. "Transfer-Encoding: chunked\r\nConnection: close\r\n\r\n"
      self.sendstring(send_str)
   end

   -- create json error response
   function self.json_error_response(msg, req)
      local json_str = json_response({["error"]=msg, ["request"]=req})
      self.send_header(500, "Internal Server Error", {["Content-Type"]="application/json"})
      self.sendline(json_str)
   end

   -- create json success response
   function self.json_success_response(req)
      local json_str = json_response({["error"]="OK", ["request"]=req})
      self.send_header(200, "OK", {["Content-Type"]="application/json"})
      self.sendline(json_str)
   end

   function self.json_status_chunk_response(msg, req)
      local json_str = json_response({["status"]=msg, ["request"]=req})
      self.sendline(string.format("%x\r\n%s", #json_str, json_str))
   end

   function self.json_error_chunk_response(msg, req)
      local json_str = json_response({["error"]=msg, ["request"]=req})
      self.sendline(string.format("%x\r\n%s", #json_str, json_str))
      self.sendline("0\r\n")
   end

   function self.json_success_chunk_response(req)
      local json_str = json_response({["error"]="OK", ["request"]=req})
      self.sendline(string.format("%x\r\n%s", #json_str, json_str))
      self.sendline("0\r\n")
   end

   --[[
      evaluate some lua code and return as a string
   --]]
   function self.evaluate(code)
      local eval_code = "function eval_func()\n" .. code .. "\nend\n"
      local f, errloc, err = load(eval_code, "eval_func", "t", _ENV)
      if not f then
         DEBUG(string.format("load failed: err=%s errloc=%s", err, errloc))
         return nil
      end
      local success, err2 = pcall(f)
      if not success then
         DEBUG(string.format("pcall failed: err=%s", err2))
         return nil
      end
      local ok, s2 = pcall(eval_func)
      eval_func = nil
      if ok then
         return s2
      end
      return nil
   end

   --[[
      process a file as a lua CGI
   --]]
   function self.send_cgi()
      sock:set_blocking(true)
      local contents = self.load_file()
      local s = self.evaluate(contents)
      if s then
         self.sendstring(s)
      end
      self.remove()
   end

   --[[
      send file content with server side processsing
      files ending in .shtml can have embedded lua lika this:
      <?lua return "foo" ?>
      <?lstr 2.6+7.2 ?>

      Using 'lstr' a return tostring(yourcode) is added to the code
      automatically
   --]]
   function self.send_processed_file(dynamic_page)
      DEBUG(string.format("%u: send_processed_file", idx))
      sock:set_blocking(true)
      local contents
      if dynamic_page then
         contents = file
      else
         contents = self.load_file()
      end
      while #contents > 0 do
         local pat1 = "(.-)[<][?]lua[ \n](.-)[?][>](.*)"
         local pat2 = "(.-)[<][?]lstr[ \n](.-)[?][>](.*)"
         local p1, p2, p3 = string.match(contents, pat1)
         if not p1 then
            p1, p2, p3 = string.match(contents, pat2)
            if not p1 then
               break
            end
            p2 = "return tostring(" .. p2 .. ")"
         end
         self.sendstring(p1)
         local s2 = self.evaluate(p2)
         if s2 then
            self.sendstring(s2)
         end
         contents = p3
      end
      self.sendstring(contents)
      self.remove()
   end
   
   -- return a content type
   function self.content_type(path)
      if path == "/" then
         return MIME_TYPES["html"]
      end
      local _, ext = string.match(path, '(.*[.])(.*)')
      if not ext then
         return CONTENT_TEXT_HTML
      end
      ext = string.lower(ext)
      local ret = MIME_TYPES[ext]
      if not ret then
         return CONTENT_OCTET_STREAM
      end
      return ret
   end

   -- perform a file download
   function self.file_download(path)
      if startswith(path, "/@") then
         path = string.sub(path, 2, #path)
      end
      DEBUG(string.format("%u: file_download(%s)", idx, path))
      file = DYNAMIC_PAGES[path]
      dynamic_page = file ~= nil
      local vars = {["Content-Type"]=self.content_type(path)}
      local server_side_processing = endswith(path, ".shtml")
      if STATIC_PAGES[path] then
         self.send_header(200, "OK", vars)
         self.sendstring(STATIC_PAGES[path])
      elseif server_side_processing or dynamic_page then
         DEBUG(string.format("%u: processing %s", idx, path))
         self.send_header(200, "OK", vars)
         run = self.send_processed_file(dynamic_page)
      else
         return false
      end
      return true
   end

   function self.not_found()
      self.send_header(500, "Not found", {})
   end

   -- refresh page
   function self.refresh_page()
      self.send_header(200, "OK", {["Content-Type"]=CONTENT_TEXT_HTML})
      self.sendline(DOCTYPE)
      self.sendstring([[
<html>
   <head>
      <meta http-equiv="refresh" content="0;url=/">
   </head>
</html>]])
   end

   function self.read_vars(header_lines, i)
      local key, var = string.match(header_lines[i], '(.*): (.*)')
      -- DEBUG(string.format(header_lines[i]))
      if key then
         if key == 'Content-Disposition' or key == 'content-disposition' then
            -- get form data from content-disposition
            local name = string.match(var, '.*name="(.*)"')
            if name then
               DEBUG(string.format("FormData[%s]: %s", name, header_lines[i+1]))
               form_data[name] = header_lines[i+1]
            end
         elseif (key == 'Content-Type' or key == 'content-type') and not boundary_str then
            DEBUG(string.format("Content-Type: %s", var))
            boundary_str = string.match(var, '.*boundary=(.*)')
         end
         header_vars[key] = var
      end
   end

   -- process a single request
   function self.process_request()
      local h1 = header_lines[1]
      if not h1 or #h1 == 0 then
         DEBUG(string.format("%u: empty request", idx))
         return
      end
      local cmd = split(header_lines[1], "%S+")
      if not cmd or #cmd < 3 then
         DEBUG(string.format("bad request: %s", header_lines[1]))
         return
      end
      if cmd[1] ~= "GET" and cmd[1] ~= "POST" then
         DEBUG(string.format("bad op: %s", cmd[1]))
         return
      end
      protocol = cmd[3]
      if protocol ~= "HTTP/1.0" and protocol ~= "HTTP/1.1" then
         DEBUG(string.format("bad protocol: %s", protocol))
         return
      end
      local path = cmd[2]
      DEBUG(string.format("%u: path='%s'", idx, path))
      boundary_str = nil
      -- extract header variables
      for i = 2,#header_lines do
         self.read_vars(header_lines, i)
      end

      if startswith(path, "/BOARD_STATUS") then
         -- create a board status json
         local board_status = {}
         board_status["uptime"] = hms_uptime()
         board_status["version"] = 
         self.sendstring(json_response(board_status)) 
      elseif startswith(path, "/THROW_ERROR") then
         assert(false, "Forced error")
         return
      end

      if DYNAMIC_PAGES[path] ~= nil then
         self.file_download(path)
         return
      end

      if startswith(path,"/@") then
         path = string.sub(path, 2, #path)
      end

      -- or a file
      if self.file_download(path) then
         return
      end
      self.not_found()
   end

   -- update the client
   function self.update()
      if run then
         run()
         return
      end
      if not close_connection then
         if not have_header then
            if not self.read_header() then
               return
            end
         end
         self.process_request()
      end
      if not run then
         -- nothing more to do
         DEBUG(string.format("%u: nothing to run", idx))
         self.remove()
      end
   end

   function self.remove()
      DEBUG(string.format("%u: removing client OFFSET=%u", idx, offset))
      if self.closed then
         return
      end
      sock:close()
      self.closed = true
   end

   -- return the instance
   return self
end

--[[
   see if any new clients want to connect
--]]
local function check_new_clients()
   while sock_listen:pollin(0) do
      local sock = sock_listen:accept()
      if not sock then
         return
      end
      -- non-blocking for header read
      sock:set_blocking(false)
      -- find free client slot
      for i = 1, #clients+1 do
         if clients[i] == nil then
            local idx = i
            local client = Client(sock, idx)
            DEBUG(string.format("%u: New client", idx))
            clients[idx] = client
         end
      end
   end
end

--[[
   check for client activity
--]]
local function check_clients()
   for idx,client in ipairs(clients) do
      if not client.closed then
         client.update()
      end
      if client.closed then
         table.remove(clients,idx)
      end
   end
end

file = nil
data = ""
sdcard_healthy = false

for i = 1,512 do
   data = data .. "a"
end

local function check_sdcard()
    if file ~= nil then
        -- read last 512 bytes from the file
        file:seek("end", -512)
        local data_read = file:read(512)
        if data_read ~= data then
            gcs:send_text(0, "Bad data in file " .. #data_read)
            sdcard_healthy = false
        else
            sdcard_healthy = true
        end
    end
    -- create a file if it does not exist
    if file == nil then
        file = io.open("test.txt", "w+")
        if not file then
            gcs:send_text(0, "Failed to open file")
            return
        end
    end
    file:write(data)
end

local function update()
   check_new_clients()
   check_clients()
   check_sdcard()
   return update,5
end

return update,100