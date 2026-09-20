-- EdgeTX's simplified io API differs from desktop Lua: io.write(file,data),
-- io.close(file) with no return value. Exercise it without touching an SD card.
SMLSIZE,MIDSIZE,INVERS,SOLID,FORCE=512,768,1,0,2
EVT_VIRTUAL_NEXT,EVT_VIRTUAL_PREV,EVT_VIRTUAL_ENTER,EVT_VIRTUAL_ENTER_LONG=100,101,102,103
local now,draws,opens,writes,closes,killed,fail=100,{},0,0,0,0,nil
local hostio=io
local chunks={}
function getTime() return now end
function getDateTime() return {year=2026,mon=9,day=20,hour=12,min=34,sec=56} end
function getFieldInfo(name) return {id=name} end
function getSourceValue(id) return id=="FM" and 'READY,"test"' or 0,true,true end
function crossfireTelemetryPop() end
function crossfireTelemetryPush() error("no commands") end
model=setmetatable({}, {__index=function() error("no model access") end})
function killEvents(e) assert(e==EVT_VIRTUAL_ENTER_LONG);killed=killed+1 end
lcd={clear=function() draws={} end,drawText=function(x,y,s) draws[#draws+1]=s end,
  drawLine=function() end,drawRectangle=function() end,drawFilledRectangle=function() end}
local function contains(s)
  for _,v in ipairs(draws) do if v:find(s,1,true) then return true end end
  return false
end
local handle={}
io={
  open=function(path,mode)
    opens=opens+1
    assert(path:match('^/LOGS/hop%-20260920%-123456%-%d+%.csv$') and mode=="a")
    if fail=="open" then return nil end
    if fail=="open-throw" then error("card removed") end
    return handle
  end,
  write=function(f,s)
    assert(f==handle);writes=writes+1
    if fail=="write" then return nil,"disk full" end
    if fail=="write-throw" then error("disk full") end
    chunks[#chunks+1]=s;return handle
  end,
  close=function(f)
    assert(f==handle);closes=closes+1
    if fail=="close" then error("removed on close") end
    -- No return value, as on EdgeTX 2.11.
  end
}
local function newApp()
  local app=dofile('radio/SCRIPTS/TELEMETRY/hop.lua')
  app.run(0)
  for _=1,5 do app.run(EVT_VIRTUAL_NEXT) end
  assert(contains("LOG OFF"));return app
end
local app=newApp();assert(opens==0)
app.run(EVT_VIRTUAL_ENTER_LONG);assert(contains("LOG ON") and killed==1 and opens==0)
now=101;app.background();assert(opens==1 and writes==1 and closes==1)
for _=1,20 do app.background();app.run(0) end
assert(writes==1) -- foreground + background cannot double-log
now=201;app.background();assert(writes==2)
app.run(EVT_VIRTUAL_ENTER_LONG);assert(contains("LOG OFF"))
now=500;app.background();assert(writes==2)
app.run(EVT_VIRTUAL_ENTER_LONG);now=501;app.background();assert(writes==3)
for i=1,597 do now=501+i*100;app.background() end
app.run(0);assert(contains("LOG LIMIT") and writes==600 and closes==600)
now=now+1000;app.run(EVT_VIRTUAL_ENTER_LONG);app.background();assert(writes==600)
local body=table.concat(chunks)
assert(select(2,body:gsub('tick_ms,page',''))==1) -- resume does not repeat header
assert(body:find('"READY,""test"""',1,true))
local f=assert(hostio.open('output/radio-diagnostic-test.csv','w'));f:write(body);f:close()
for _,failure in ipairs({'open','open-throw','write','write-throw','close'}) do
  fail=failure;now=now+1000;app=newApp();app.run(EVT_VIRTUAL_ENTER_LONG)
  now=now+1;app.run(0);assert(contains("LOG ERROR"),failure)
  local attempts=opens
  now=now+500;app.run(EVT_VIRTUAL_ENTER_LONG);app.background();assert(opens==attempts)
end
io=hostio
print('Lua logger passed: default off, opt-in, append/CSV escaping, 1 Hz bound, pause/resume, 600-row limit, SD errors and no retries')
