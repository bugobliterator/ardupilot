--[[
   example script to test lua socket API
--]]

---@diagnostic disable: param-type-mismatch
---@diagnostic disable: need-check-nil
---@diagnostic disable: redundant-parameter
---@diagnostic disable: undefined-field

local web = require("web")

function printf(str)
   periph:can_printf(str)
end

web.init(printf)