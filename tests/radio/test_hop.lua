-- Execute the production script with EdgeTX API doubles and real C++ payloads.
local loadRadio=dofile("tests/radio/edgetx_mono.lua")
SMLSIZE, MIDSIZE, INVERS, SOLID, FORCE = 512, 768, 1, 0, 2
EVT_VIRTUAL_NEXT, EVT_VIRTUAL_PREV, EVT_VIRTUAL_ENTER = 100,101,102
LCD_W,LCD_H=128,64
local now,queue,draws,buzzes,pops=100,{}, {},0,0
local values={FM={"DISARM",true,true},RxBt={25.2,true,true},RQly={100,true,true},
  TQly={99,true,true},["1RSS"]={-63,true,true},TPWR={100,true,true},Roll={1.5,true,true},Ptch={-0.2,true,true},Curr={1.2,true,true},["tx-voltage"]={7.8,true,true}}
function getTime() return now end
function getFieldInfo(name) if values[name] then return {id=name,unit=(name=="Roll" or name=="Ptch") and 21 or nil} end end
function getSourceValue(id) return table.unpack(values[id]) end
function crossfireTelemetryPop()
  pops=pops+1
  local p=table.remove(queue,1)
  if p then return p.command,p.data end
end
function playHaptic() buzzes=buzzes+1 end
-- Forbidden APIs fail the test if future changes accidentally add commands.
function crossfireTelemetryPush() error("telemetry must be read-only") end
model=setmetatable({}, {__index=function() error("model access forbidden") end})
lcd={clear=function() draws={} end}
for _,name in ipairs({"drawText","drawLine","drawRectangle","drawFilledRectangle"}) do
  lcd[name]=function(...) draws[#draws+1]={name,...} end
end
local fixtures=dofile("output/radio-fixtures.lua")
local function copy(t) local out={} for k,v in pairs(t) do out[k]=v end return out end
local function push(t, command) queue[#queue+1]={command=command or 0x7E,data=copy(t)} end
local function contains(s)
  for _,d in ipairs(draws) do if d[1]=="drawText" and string.find(d[4],s,1,true) then return true end end
  return false
end
local function save(name)
  local f=assert(io.open("output/radio-"..name..".tsv","w"))
  for _,d in ipairs(draws) do
    for i,v in ipairs(d) do f:write(i==1 and "" or "\t",tostring(v)) end f:write("\n")
  end
  f:close()
end
local app=loadRadio()
app.run(0); assert(contains("BASIC TELEMETRY") and contains("Drive / arms UNKNOWN")); save("basic")
-- Each Basic page has its own content; never infer motor state from FM.
app.run(EVT_VIRTUAL_NEXT)
assert(contains("ROLL  +85.9deg") and contains("PITCH -11.5deg") and contains("MOTOR IQ 1.2A") and not contains("BASIC TELEMETRY"));save("basic-health")
app.run(EVT_VIRTUAL_NEXT)
assert(contains("FR  --") and contains("RA  --") and contains("MOTOR DETAILS UNAVAILABLE") and not contains("FM:"));save("basic-motors")
app.run(EVT_VIRTUAL_NEXT)
assert(contains("NO RUN REPORT RECEIVED") and contains("FM DISARM"));save("basic-history")
app.run(EVT_VIRTUAL_NEXT);assert(contains("CONTROL LQ 100%"));save("basic-link")
app.run(EVT_VIRTUAL_NEXT);assert(contains("RX 0  HS 0"));save("basic-diagnostics")
app.run(EVT_VIRTUAL_NEXT);assert(contains("BASIC TELEMETRY"))
values.FM={"BALANCE",false,false};values.RxBt={25.2,false,false};now=121
app.run(0);assert(contains("FM: DISARM") and contains("ROBOT 25.2V"))
push(fixtures.drive); app.run(0)
assert(contains("LIVE") and contains("DRIVE ON") and contains("ARMS  OFF")); save("drive")
push(fixtures.balance); now=140; app.run(0)
assert(contains("BALANCE") and contains("ARMS  ON"));save("balance")
-- -0.45 lies on a decimal rounding boundary; binary32/binary64 land on opposite sides.
app.run(EVT_VIRTUAL_NEXT); assert(contains("+88.2deg") and (contains("-0.5deg") or contains("-0.4deg")) and contains("MOTOR IQ 12.3A"));save("health")
app.run(EVT_VIRTUAL_NEXT);assert(contains("FR  ON") and contains("RA  ON"));save("motors")
push(fixtures.detail);app.run(EVT_VIRTUAL_NEXT);assert(contains("tilt out of range") and contains("(fallen)"));save("history")
app.run(EVT_VIRTUAL_NEXT);assert(contains("CONTROL LQ 100%"));save("link")
app.run(EVT_VIRTUAL_NEXT);assert(contains("LUA v3"));save("diagnostics")
app.run(EVT_VIRTUAL_NEXT);now=291;app.run(0)
assert(contains("HOLD") and contains("DRIVE ON") and buzzes==1);save("hold")
now=441;app.run(0)
assert(contains("ROBOT DATA LOST") and not contains("DRIVE ON") and buzzes==1);save("lost")
push(fixtures.balance);app.run(0);assert(contains("ROBOT DATA LOST")) -- duplicate cannot revive
push(fixtures.detail);app.run(0);assert(contains("ROBOT DATA LOST")) -- detail cannot revive
push(fixtures.fault);now=1000;app.run(0);assert(contains("MOTOR FAULT") and buzzes==2);save("fault")
now=1020;app.run(0);assert(buzzes==2) -- no repeated alarm
local future=copy(fixtures.balance);future[5]=2;push(future);app.run(0)
assert(contains("UPDATE HOP LUA"));save("version")
-- A matching unknown schema never silently downgrades to cached standard sensors.
now=1100;app.run(0);assert(contains("UPDATE HOP LUA"))
local unknown=copy(fixtures.balance);unknown[8]=200;unknown[11]=77;unknown[13]=0x12;unknown[14]=0x34
push(unknown);app.run(0);assert(contains("MODE 77") and contains("M:4660"));save("future-motion")
-- Malformed frames, foreign identities, NaNs, partial payloads and bad masks.
local originalPops=pops
for i=1,20 do push({1,2,3}) end
app.background();assert(pops-originalPops==4 and #queue==16);queue={}
for _,index in ipairs({1,2,3,4,15,16,17,32}) do
  local bad=copy(fixtures.balance);bad[index]=254;bad[8]=201;push(bad)
  app.run(0);assert(contains("MODE 77"))
end
local bad=copy(fixtures.balance);bad[9]=0/0;push(bad);app.run(0);assert(contains("MODE 77"))
-- Sequence wrap and radio clock wrap recover on a new valid packet.
local wrap=copy(fixtures.balance);wrap[7]=255;wrap[8]=255;push(wrap);now=1200;app.run(0)
wrap[7]=0;wrap[8]=0;push(wrap);now=1;app.run(0);assert(contains("LIVE"))
-- Unknown sensor quantities stay unknown; no fake battery percentage or progress.
wrap[8]=1;wrap[9]=0;wrap[10]=19;wrap[18]=255;push(wrap);now=20;app.run(0)
assert(contains("25.2V") and contains("IMU STALE")) -- numbers held; stale flag immediate
-- Stale motor page must not show cached enabled states.
now=321;app.run(EVT_VIRTUAL_NEXT);app.run(EVT_VIRTUAL_NEXT)
assert(contains("ROBOT DATA LOST") and not contains("FR  ON"))
-- Long labels and signed sentinel rendering exercised for bounds previews.
wrap[8]=2;wrap[9]=13;wrap[10]=19;wrap[19]=128;wrap[20]=0;wrap[21]=128;wrap[22]=0
for i=33,48 do wrap[i]=string.byte("W") end
push(wrap);now=600;app.run(EVT_VIRTUAL_PREV);assert(contains("TILT --") and contains("ERROR --"))
save("unknown-values")

-- Alternating sensor updates retain each value independently, then expire.
queue={};now=2000;app=loadRadio()
values.FM={"READY",true,true};values.RxBt={25.2,true,true};app.run(0)
values.FM={"",true,true};values.RxBt={0/0,true,true};now=2021;app.run(0)
assert(contains("FM: READY") and contains("ROBOT 25.2V"))
values.FM={nil,false,false};values.RxBt={25.1,true,true};now=2280;app.run(0)
assert(contains("FM: READY") and contains("ROBOT 25.1V"))
values.RxBt={999,true,false};now=2501;app.run(0)
assert(contains("FM: --") and contains("ROBOT 25.1V")) -- voltage did not renew FM
now=2781;app.run(0);assert(contains("ROBOT --")) -- stale value cannot renew voltage
values.RxBt={0,true,true};now=2802;app.run(0);assert(contains("ROBOT 0.0V"))
values.RxBt={math.huge,true,true};now=2823;app.run(0);assert(contains("ROBOT 0.0V"))
values.RxBt={"invalid",true,true};now=2844;app.run(0);assert(contains("ROBOT 0.0V"))
values.RxBt={nil,false,false};now=1;app.run(0);assert(contains("ROBOT --")) -- clock wrap

-- Missing numeric fields are held independently while flags update immediately.
queue={};now=3000;app=loadRadio()
push(fixtures.balance);app.run(EVT_VIRTUAL_NEXT)
local missing=copy(fixtures.balance);missing[8]=30;missing[9]=0;missing[10]=16
missing[18]=255;missing[19]=128;missing[20]=0;missing[21]=128;missing[22]=0
now=3200;push(missing);app.run(0)
assert(contains("TILT +88.2deg") and contains("MOTOR IQ 12.3A") and contains("MAX TEMP 43C") and contains("IMU STALE"))
app.run(EVT_VIRTUAL_PREV);assert(contains("DRIVE OFF") and contains("ARMS  OFF"))
app.run(EVT_VIRTUAL_NEXT)
now=3500;missing[8]=31;push(missing);app.run(0);assert(contains("MOTOR IQ 12.3A"))
now=3501;app.run(0)
assert(contains("TILT --") and contains("ERROR --") and contains("MOTOR IQ --") and contains("MAX TEMP --"))
app.run(EVT_VIRTUAL_PREV);assert(contains("DRIVE OFF") and contains("ARMS  OFF"))
local restored=copy(fixtures.balance);restored[8]=32;now=3520;push(restored);app.run(0)
assert(contains("25.2V") and contains("DRIVE ON"))
now=3671;app.run(0);assert(contains("HOLD") and contains("25.2V"))
push(restored);push(fixtures.detail);now=3821;app.run(0)
assert(contains("ROBOT DATA LOST") and not contains("DRIVE ON")) -- neither renews hold

-- Reproduce the hardware freshness window: unchanged values arrive just after
-- each old 200 ms polling boundary and are fresh for only 160 ms. The old
-- sampler missed EVERY update. Exercise foreground and background operation.
local sourceAPI=getSourceValue
function getSourceValue(id)
  if id=="FM" or id=="RxBt" then
    local fresh=now>=4010 and (now-4010)%40<16
    return id=="FM" and "READY" or 24.9,true,fresh
  end
  return sourceAPI(id)
end
now=4009;queue={};app=loadRadio();app.run(0)
assert(contains("ROBOT --"))
for t=4010,5500 do
  now=t;app.background()
  if t>=4019 and t%10==9 then
    app.run(0);assert(contains("ROBOT 24.9V") and contains("FM: READY"),"Blank at "..t)
  end
end
function getSourceValue(id)
  if id=="FM" or id=="RxBt" then return id=="FM" and "READY" or 24.9,true,false end
  return sourceAPI(id)
end
now=5800;app.run(0);assert(contains("ROBOT 24.9V*"));save("basic-held")
now=6010;app.run(0);assert(contains("ROBOT --") and contains("FM: --"));save("basic-expired")
getSourceValue=sourceAPI
print("Lua tests passed: C++ fixtures, independent five-second readings/three-second status expiry, recovery, clock wrap, lifecycle, schemas, malformed data, queue bounds, navigation, alerts")
