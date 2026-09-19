-- Hopscotch / GX12 / EdgeTX 2.11, 128x64 monochrome. Read-only telemetry.
-- No model writes, channel overrides, CRSF commands, or safety decisions.
-- Install as a model telemetry screen; roller / ENTER changes pages.
local TTL = 150 -- getTime() units: 10 ms. Mark unknown after 1.5 seconds.
local HAPTIC = true -- one pulse on loss/fault transition, rate limited
local page, pages = 1, 5
local status, detail, lastSequence, received, incompatible
local everLive, wasLive, wasFault, lastBuzz = false, false, false, -1000
local sensorIds, sensors, events = {}, {}, {}
local nextDiscover, nextSample = 0, 0
local names = {"FM", "RxBt", "Curr", "Roll", "RQly", "TQly", "1RSS", "TPWR", "tx-voltage"}
local motions = {[0]="NONE", [1]="FORWARD", [2]="CENTER", [3]="BACKWARD", [4]="JUMP", [256]="BALANCE"}
local roles = {"FR", "BR", "BL", "FL", "LA", "RA"}
local function has(value, bit) return math.floor(value / bit) % 2 == 1 end
local function age(now, before)
  if before == nil or now < before then return 1e9 end
  return now - before
end
local function u16(d, i) return d[i] * 256 + d[i+1] end
local function signed(d, i)
  local n = u16(d, i)
  if n == 32768 then return nil end
  if n > 32767 then n = n - 65536 end
  return n / 100
end
local function str(d, first, last)
  local s = ""
  for i = first, last do
    if d[i] == 0 then break end
    s = s .. ((d[i] >= 32 and d[i] <= 126) and string.char(d[i]) or "?")
  end
  return s
end
local function event(text, now)
  table.insert(events, 1, {text=text, time=now})
  if #events > 4 then table.remove(events) end
end
local function decode(command, d, now)
  if command ~= 0x7E or type(d) ~= "table" or #d < 8 or #d > 60 then return end
  for i = 1, #d do
    if type(d[i]) ~= "number" or d[i] < 0 or d[i] > 255 or d[i] % 1 ~= 0 then return end
  end
  if d[1] ~= 0xEA or d[2] ~= 0xC8 or d[3] ~= 72 or d[4] ~= 83 then return end
  if d[5] ~= 1 then incompatible = now; return end
  if d[6] == 2 and #d >= 60 then
    detail = {ended=u16(d,9)*65536+u16(d,11), reason=str(d,13,60), time=now}
    return -- detail packets never refresh the robot-state heartbeat
  end
  if d[6] ~= 1 or #d < 48 then return end
  local sequence = u16(d,7)
  if sequence == lastSequence then return end -- repeated cached packet is not live
  if d[15] > 63 or d[16] > 63 or d[17] > 63 or (d[32] > 100 and d[32] ~= 255) then return end
  local new = {flags=u16(d,9), mode=d[11], phase=d[12], motion=u16(d,13),
    online=d[15], faults=d[16], enabled=d[17], temp=d[18], tilt=signed(d,19),
    error=signed(d,21), volts=u16(d,23)/10, amps=u16(d,25)/10, inner=u16(d,27),
    capabilities=u16(d,29), cal=d[31], progress=d[32], label=str(d,33,48)}
  if not status or new.label ~= status.label then event(new.label, now) end
  if status then
    if has(new.flags,1) ~= has(status.flags,1) then event(has(new.flags,1) and "DRIVE ARMED" or "DRIVE DISARMED", now) end
    if has(new.flags,2) ~= has(status.flags,2) then event(has(new.flags,2) and "ARMS ARMED" or "ARMS DISARMED", now) end
  end
  status, received, lastSequence, incompatible = new, now, sequence, nil
end
local function sample(now)
  if now < nextSample and nextSample - now < 100 then return end
  nextSample = now + 20
  if now >= nextDiscover or nextDiscover - now > 200 then
    nextDiscover = now + 200
    for _, name in ipairs(names) do
      local info = getFieldInfo(name)
      sensorIds[name] = info and info.id or nil
    end
  end
  for _, name in ipairs(names) do
    local value, current, fresh
    if sensorIds[name] and getSourceValue then value, current, fresh = getSourceValue(sensorIds[name]) end
    -- Never fall back to getValue's cached values with no freshness information.
    sensors[name] = (current and fresh) and value or nil
  end
end
local function live(now)
  return status ~= nil and incompatible == nil and age(now,received) <= TTL
end
local function buzz(now)
  if HAPTIC and playHaptic and age(now,lastBuzz) >= 500 then
    playHaptic(12, 0)
    lastBuzz = now
  end
end
local function background()
  local now = getTime()
  if crossfireTelemetryPop then
    for _ = 1, 4 do -- bound work even when another device floods the queue
      local command, data = crossfireTelemetryPop()
      if not command then break end
      decode(command, data, now)
    end
  end
  sample(now)
  local current = live(now)
  local fault = current and (status.faults ~= 0 or status.inner ~= 0 or not has(status.flags,16))
  if wasLive and not current then event("ROBOT DATA LOST", now); buzz(now) end
  if everLive and current and not wasLive then event("ROBOT DATA BACK", now) end
  if current and fault and not wasFault then buzz(now) end
  everLive, wasLive, wasFault = everLive or current, current, fault
end
local function text(x,y,s,flags) lcd.drawText(x,y,s,flags or SMLSIZE) end
local function line(y) lcd.drawLine(0,y,127,y,SOLID,FORCE) end
local function value(v, fmt, suffix)
  if type(v) ~= "number" or v ~= v then return "--" end
  return string.format(fmt,v) .. (suffix or "")
end
local function freshValue(flag, v)
  if has(status.flags,flag) then return v end
end
local function title(name, now)
  lcd.drawFilledRectangle(0,0,128,9)
  text(1,1,name, SMLSIZE+INVERS)
  local badge = live(now) and "LIVE" or (status and "OLD" or "BASIC")
  if incompatible then badge = "VER?" end
  text(91,1,badge, SMLSIZE+INVERS)
  text(117,1,tostring(page), SMLSIZE+INVERS)
end
local function headline()
  if status.faults ~= 0 then return "MOTOR FAULT" end
  if not has(status.flags,16) then return "RC INPUT LOST" end
  if status.inner ~= 0 then return "BALANCE FAULT" end
  if has(status.flags,512) then return "SIM / TEST MODE" end
  if has(status.flags,64) then return "RESET ARM SWITCHES" end
  if has(status.flags,128) then return "SAVING RUN" end
  if status.online ~= 63 then return "MOTOR OFFLINE" end
  if not has(status.flags,256) then return "IMU STALE" end
  if status.mode > 3 then return "MODE " .. status.mode end
  return status.label ~= "" and status.label or "ROBOT STATUS"
end
local function armState(armed, arming)
  if has(status.flags,arming) then return "WAIT" end
  return has(status.flags,armed) and "ON" or "OFF"
end
local function motor(index)
  local bit = 2^(index-1)
  if has(status.faults,bit) then return "FLT" end
  if not has(status.online,bit) then return "---" end
  return has(status.enabled,bit) and "ON" or "OFF"
end
local function robot()
  -- Top view, matching the chassis motor roles (left in drawing = robot left).
  lcd.drawRectangle(12,27,18,20)
  lcd.drawLine(21,41,21,30,SOLID,FORCE)
  lcd.drawLine(21,30,18,34,SOLID,FORCE)
  lcd.drawLine(21,30,24,34,SOLID,FORCE)
  local positions = {{32,27},{32,41},{3,41},{3,27},{12,49},{25,49}}
  for i,p in ipairs(positions) do
    lcd.drawRectangle(p[1],p[2],7,6)
    local m = motor(i)
    if m == "ON" then lcd.drawFilledRectangle(p[1]+1,p[2]+1,5,4)
    elseif m == "FLT" then
      lcd.drawLine(p[1],p[2],p[1]+6,p[2]+5,SOLID,FORCE)
      lcd.drawLine(p[1]+6,p[2],p[1],p[2]+5,SOLID,FORCE)
    elseif m == "---" then text(p[1]+2,p[2],"?",SMLSIZE) end
  end
end
local function home()
  text(0,12,headline(),SMLSIZE)
  if status.progress <= 100 then text(106,12,tostring(status.progress).."%") end
  robot()
  text(47,24,value(freshValue(1024,status.volts),"%.1f","V"),MIDSIZE)
  text(47,39,"DRIVE "..armState(1,4))
  text(47,48,"ARMS  "..armState(2,8))
  text(0,57,"LQ "..value(sensors.RQly,"%.0f","%"))
  text(48,57,"M:"..(motions[status.motion] or tostring(status.motion)))
end
local function health()
  text(0,12,"TILT "..value(freshValue(256,status.tilt),"%+.1f","deg"))
  text(0,22,"ERROR "..value(freshValue(256,status.error),"%+.1f","deg"))
  text(0,32,"MOTOR IQ "..value(freshValue(2048,status.amps),"%.1f","A"))
  text(0,42,"MAX TEMP "..value(status.temp ~= 255 and status.temp or nil,"%.0f","C"))
  text(0,52,"IMU "..(has(status.flags,256) and "LIVE" or "STALE").." FAULT "..string.format("%04X",status.inner))
end
local function motors()
  text(0,12,"MOTOR STATE  MOTOR STATE")
  for row=0,2 do
    local a,b=row*2+1,row*2+2
    text(2,24+row*10,roles[a].."  "..motor(a))
    text(68,24+row*10,roles[b].."  "..motor(b))
  end
  text(0,57,"ON=enabled  ---=lost")
end
local function history(now)
  if detail and detail.ended ~= 0 and detail.reason ~= "" then
    text(0,12,age(now,detail.time) > 250 and "LAST RUN (CACHED)" or "LAST RUN ENDED:")
    local rest = detail.reason
    for i=0,2 do
      local count = math.min(25,#rest)
      if #rest > count then count = string.match(string.sub(rest,1,count),"^.*() ") or count end
      text(0,22+i*8,string.sub(rest,1,count))
      rest = string.gsub(string.sub(rest,count+1),"^ +","")
    end
  else
    text(0,12,"NO RUN END THIS BOOT")
  end
  line(46)
  for i=1,2 do if events[i] then text(0,48+(i-1)*8,string.sub(events[i].text,1,25)) end end
end
local function radio()
  text(0,12,"CONTROL LQ "..value(sensors.RQly,"%.0f","%"))
  text(0,22,"RETURN  LQ "..value(sensors.TQly,"%.0f","%"))
  text(0,32,"RSSI "..value(sensors["1RSS"],"%.0f","dBm").."  "..value(sensors.TPWR,"%.0f","mW"))
  text(0,42,"RADIO "..value(sensors["tx-voltage"],"%.1f","V"))
  text(0,54,"Roll / ENTER: next page")
end
local function unavailable(now)
  if incompatible then
    text(0,14,"UPDATE HOP LUA",MIDSIZE)
    text(0,33,"Robot schema unsupported")
  elseif status then
    text(0,14,"ROBOT DATA LOST",MIDSIZE)
    text(0,33,"Arming / motion UNKNOWN")
    text(0,44,"Age "..value(age(now,received)/100,"%.1f","s"))
  else
    text(0,12,"BASIC TELEMETRY")
    text(0,23,"FM: "..(type(sensors.FM)=="string" and string.sub(sensors.FM,1,19) or "--"))
    text(0,33,"ROBOT "..value(sensors.RxBt,"%.1f","V"))
    text(0,43,"Drive / arms UNKNOWN")
    text(0,54,"New firmware: full status")
  end
end
local titles = {"HOPSCOTCH", "POSE + POWER", "SIX MOTORS", "RUN + EVENTS", "RADIO LINK"}
local function isEvent(e, constant) return constant ~= nil and e == constant end
local function run(e)
  background()
  if isEvent(e,EVT_VIRTUAL_NEXT) or isEvent(e,EVT_VIRTUAL_ENTER) then page = page % pages + 1 end
  if isEvent(e,EVT_VIRTUAL_PREV) then page = (page + pages - 2) % pages + 1 end
  local now = getTime()
  lcd.clear()
  title(titles[page],now)
  if page == 5 then radio()
  elseif page == 4 then history(now)
  elseif not live(now) then unavailable(now)
  elseif page == 1 then home()
  elseif page == 2 then health()
  elseif page == 3 then motors() end
  return 0
end
return {run=run, background=background}
