-- Host-test loader for the GX12's restricted EdgeTX 2.11 Lua environment.
-- Source: EdgeTX v2.11.0 radio/src/thirdparty/Lua/src/linit.c and library tables.
-- Desktop facilities stay available to the test harness, never to hop.lua.
-- This models the exposed APIs, not the radio VM, heap size or scheduler.
local host = _G
local function readonly(names, lookup)
  local allowed = {}
  for _,name in ipairs(names) do allowed[name] = true end
  return setmetatable({}, {
    __index=function(_,key) if allowed[key] then return lookup(key) end end,
    __newindex=function() error("read-only EdgeTX library") end
  })
end
return function(path)
  local env = {}
  local base = {"assert","error","ipairs","pairs","next","pcall","select",
    "tonumber","tostring","type"}
  for _,name in ipairs(base) do env[name] = host[name] end
  env.math = readonly({"floor","max","min","huge","pi"},function(k) return math[k] end)
  env.string = readonly({"char","sub","format","match","gsub"},function(k) return string[k] end)
  env.io = readonly({"open","close","read","write","seek"},function(k) return host.io and host.io[k] end)
  local allowed = {}
  for _,name in ipairs({"getTime","getFieldInfo","getSourceValue","getDateTime",
    "crossfireTelemetryPop","playHaptic","killEvents","lcd","model",
    "crossfireTelemetryPush","SMLSIZE","MIDSIZE","INVERS","SOLID","FORCE",
    "EVT_VIRTUAL_NEXT","EVT_VIRTUAL_PREV","EVT_VIRTUAL_ENTER","EVT_VIRTUAL_ENTER_LONG"}) do
    allowed[name] = true
  end
  env._G = env
  setmetatable(env,{__index=function(_,key) if allowed[key] then return host[key] end end})
  assert(env.table==nil and env.os==nil and env.package==nil and env.debug==nil and env.coroutine==nil)
  return assert(loadfile(path or "radio/SCRIPTS/TELEMETRY/hop.lua","t",env))(),env
end
