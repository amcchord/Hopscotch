-- Hopscotch / GX12 / EdgeTX 2.11, 128x64 monochrome. Receive-only telemetry.
-- No model writes, channel overrides, CRSF commands, or safety decisions.
-- Roller / ENTER changes pages; hold ENTER on Diagnostics for optional SD logging.
local LIVE_TTL = 150 -- getTime() units: 10 ms. Show HOLD after 1.5 seconds.
local STATUS_TTL = 300 -- Arming/motion become unknown after three seconds.
local VALUE_TTL = 500 -- Numeric/text readings bridge gaps for five seconds.
local HAPTIC = true -- one pulse on loss/fault transition, rate limited
local page, pages = 1, 6
local status, detail, lastSequence, received, incompatible
local everLive, wasLive, wasFault, lastBuzz = false, false, false, -1000
local sensorIds, sensorUnits, sensors, quantities, events = {}, {}, {}, {}, {}
local observed = {} -- EdgeTX flags, including samples rejected by the display.
local diag = {rx=0, custom=0, status=0, detail=0, duplicate=0, lastType=0, gap=0}
local lastSample
local log = {state="OFF", rows=0, last=nil, file=nil} -- opt-in, 600 rows per load
local nextDiscover, nextSample = 0, 0
local names = {"FM", "RxBt", "Curr", "Roll", "Ptch", "RQly", "TQly", "1RSS", "TPWR", "tx-voltage"}
local motions = {[0]="NONE", [1]="FORWARD", [2]="CENTER", [3]="BACKWARD", [4]="JUMP", [256]="BALANCE"}
local roles = {"FR", "BR", "BL", "FL", "LA", "RA"}
local function has(value, bit) return math.floor(value / bit) % 2 == 1 end
local function age(now, before)
  if before == nil or now < before then return 1e9 end
  return now - before
end
local function remember(cache, key, v, now)
  if (type(v) == "number" and v == v and v > -math.huge and v < math.huge)
      or (type(v) == "string" and v ~= "") then
    local entry = cache[key] or {}
    entry.value, entry.time = v, now
    cache[key] = entry
  end
end
local function reading(cache, key)
  local entry = cache[key]
  if entry and age(getTime(),entry.time) <= VALUE_TTL then return entry.value end
end
local function held(cache, key)
  local entry = cache[key]
  return entry and age(getTime(),entry.time) > 50 and reading(cache,key) ~= nil
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
  -- Monochrome EdgeTX omits the table library. Keep this bounded list using
  -- language-level indexing (ordinary Lua tables themselves are supported).
  for i=4,2,-1 do events[i] = events[i-1] end
  events[1] = {text=text, time=now}
end
local function decode(command, d, now)
  if command ~= 0x7E or type(d) ~= "table" or #d < 8 or #d > 60 then return end
  for i = 1, #d do
    if type(d[i]) ~= "number" or d[i] < 0 or d[i] > 255 or d[i] % 1 ~= 0 then return end
  end
  if d[1] ~= 0xEA or d[2] ~= 0xC8 or d[3] ~= 72 or d[4] ~= 83 then return end
  if d[5] ~= 1 then incompatible = now; return end
  if d[6] == 2 and #d >= 60 then
    diag.detail = diag.detail + 1
    detail = {ended=u16(d,9)*65536+u16(d,11), reason=str(d,13,60), time=now}
    return -- detail packets never refresh the robot-state heartbeat
  end
  if d[6] ~= 1 or #d < 48 then return end
  local sequence = u16(d,7)
  if sequence == lastSequence then diag.duplicate = diag.duplicate + 1; return end
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
  if has(new.flags,256) then
    remember(quantities,"tilt",new.tilt,now)
    remember(quantities,"error",new.error,now)
  end
  if has(new.flags,1024) then remember(quantities,"volts",new.volts,now) end
  if has(new.flags,2048) then remember(quantities,"amps",new.amps,now) end
  if new.temp ~= 255 then remember(quantities,"temp",new.temp,now) end
  status, received, lastSequence, incompatible = new, now, sequence, nil
  diag.status = diag.status + 1
end
local function sample(now)
  if now < nextSample and nextSample - now < 100 then return end
  -- EdgeTX 2.11's isFresh window can be as short as 160 ms. A 200 ms poll
  -- can miss an entire update. Poll at 50 ms, independent of screen changes.
  nextSample = now + 5
  if lastSample and now >= lastSample then diag.gap = math.max(diag.gap,now-lastSample) end
  lastSample = now
  if now >= nextDiscover or nextDiscover - now > 200 then
    nextDiscover = now + 200
    for _, name in ipairs(names) do
      local info = getFieldInfo(name)
      sensorIds[name] = info and info.id or nil
      sensorUnits[name] = info and info.unit or nil
    end
  end
  for _, name in ipairs(names) do
    local value, current, fresh
    if sensorIds[name] and getSourceValue then value, current, fresh = getSourceValue(sensorIds[name]) end
    local flags = observed[name] or {}
    flags.current, flags.fresh = current == true, fresh == true
    observed[name] = flags
    -- Never fall back to getValue's cached values with no freshness information.
    -- An update for one sensor must not erase another sensor's last reading.
    -- Stale/invalid samples do not extend the hold, even if their value repeats.
    local validType = type(value) == (name == "FM" and "string" or "number")
    if current and fresh and validType then
      if name == "FM" and value ~= "" and (not sensors.FM or value ~= sensors.FM.value) then
        event("FM "..string.sub(value,1,20),now)
      end
      remember(sensors,name,value,now)
    end
  end
end
local function live(now)
  return status ~= nil and incompatible == nil and age(now,received) <= LIVE_TTL
end
local function available(now)
  return status ~= nil and incompatible == nil and age(now,received) <= STATUS_TTL
end
local function csv(v)
  if v == nil then return "" end
  if type(v) == "string" then return '"'..string.gsub(string.sub(v,1,64),'"','""')..'"' end
  return tostring(v)
end
local function logAge(now, before)
  local a = age(now,before)
  return a < 1e9 and a * 10 or ""
end
local function record(now)
  if log.state ~= "ON" or age(now,log.last) < 100 then return end
  log.last = now
  local row = {now*10,page,diag.rx,diag.custom,diag.status,diag.detail,diag.duplicate,
    diag.lastType,lastSequence or "",logAge(now,received),diag.gap*10,
    status and status.flags or "",status and status.online or "",status and status.faults or ""}
  local header = "tick_ms,page,rx,custom,status,detail,duplicate,last_type,sequence,status_age_ms,max_sample_gap_ms,flags,online,faults"
  for _,name in ipairs(names) do
    local entry, flags = sensors[name], observed[name] or {}
    row[#row+1] = entry and entry.value or ""
    row[#row+1] = logAge(now,entry and entry.time)
    row[#row+1] = flags.current and 1 or 0
    row[#row+1] = flags.fresh and 1 or 0
    row[#row+1] = reading(sensors,name) ~= nil and 1 or 0
    if log.rows == 0 then header = header..","..name..","..name.."_age_ms,"..name.."_current,"..name.."_fresh,"..name.."_shown" end
  end
  -- Assemble one bounded row without table.concat, absent on the GX12.
  local data = log.rows == 0 and header.."\n" or ""
  for i=1,#row do data = data..(i > 1 and "," or "")..csv(row[i]) end
  data = data.."\n"
  local file
  local ok = pcall(function()
    file = io.open(log.file,"a") -- append only; never truncate existing logs
    if not file or not io.write(file,data) then error("SD write failed") end
  end)
  local closed = true
  if file then closed = pcall(io.close,file) end -- EdgeTX close returns no value
  if not ok or not closed then log.state="ERROR"; return end
  log.rows = log.rows + 1
  if log.rows >= 600 then log.state="LIMIT" end
end
local function toggleLog(now)
  if log.state == "ON" then log.state="OFF"; return end
  if log.state == "ERROR" or log.state == "LIMIT" then return end
  if not log.file then
    local stamp = tostring(now)
    if getDateTime then
      local d = getDateTime()
      if d then stamp=string.format("%04d%02d%02d-%02d%02d%02d-%d",d.year,d.mon,d.day,d.hour,d.min,d.sec,now) end
    end
    log.file = "/LOGS/hop-"..stamp..".csv"
  end
  log.state, log.last = "ON", nil
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
      diag.rx, diag.lastType = diag.rx + 1, command
      if command == 0x7E then diag.custom = diag.custom + 1 end
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
  record(now)
end
local function text(x,y,s,flags) lcd.drawText(x,y,s,flags or SMLSIZE) end
local function line(y) lcd.drawLine(0,y,127,y,SOLID,FORCE) end
local function value(v, fmt, suffix)
  if type(v) ~= "number" or v ~= v then return "--" end
  return string.format(fmt,v) .. (suffix or "")
end
local function measurement(cache, key, fmt, suffix)
  return value(reading(cache,key),fmt,suffix)..(held(cache,key) and "*" or "")
end
local function basicAngle(key)
  local v = reading(sensors,key)
  -- Standard CRSF attitude is radians; sensor settings may convert to degrees.
  if v and sensorUnits[key] == 21 then v = v * 180 / math.pi end
  local unit = sensorUnits[key] == 21 or sensorUnits[key] == 20
  return value(v,"%+.1f",unit and "deg" or "")..(held(sensors,key) and "*" or "")
end
local function title(name, now)
  lcd.drawFilledRectangle(0,0,128,9)
  text(1,1,name, SMLSIZE+INVERS)
  local badge = live(now) and "LIVE" or (available(now) and "HOLD" or (status and "OLD" or "BASIC"))
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
  text(47,24,measurement(quantities,"volts","%.1f","V"),MIDSIZE)
  text(47,39,"DRIVE "..armState(1,4))
  text(47,48,"ARMS  "..armState(2,8))
  text(0,57,"LQ "..measurement(sensors,"RQly","%.0f","%"))
  text(48,57,"M:"..(motions[status.motion] or tostring(status.motion)))
end
local function health()
  text(0,12,"TILT "..measurement(quantities,"tilt","%+.1f","deg"))
  text(0,22,"ERROR "..measurement(quantities,"error","%+.1f","deg"))
  text(0,32,"MOTOR IQ "..measurement(quantities,"amps","%.1f","A"))
  text(0,42,"MAX TEMP "..measurement(quantities,"temp","%.0f","C"))
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
    text(0,12,"NO RUN REPORT RECEIVED")
    text(0,25,"Recent observed changes:")
  end
  line(46)
  for i=1,2 do if events[i] then text(0,48+(i-1)*8,string.sub(events[i].text,1,25)) end end
end
local function radio()
  text(0,12,"CONTROL LQ "..measurement(sensors,"RQly","%.0f","%"))
  text(0,22,"RETURN  LQ "..measurement(sensors,"TQly","%.0f","%"))
  text(0,32,"RSSI "..measurement(sensors,"1RSS","%.0f","dBm").." "..measurement(sensors,"TPWR","%.0f","mW"))
  text(0,42,"RADIO "..measurement(sensors,"tx-voltage","%.1f","V"))
  text(0,54,"* held value / -- missing")
end
local function basic()
  if page == 1 then
    text(0,12,"BASIC TELEMETRY")
    local fm = reading(sensors,"FM")
    text(0,23,"FM: "..(type(fm)=="string" and string.sub(fm,1,18) or "--")..(held(sensors,"FM") and "*" or ""))
    text(0,33,"ROBOT "..measurement(sensors,"RxBt","%.1f","V"))
    text(0,43,"Drive / arms UNKNOWN")
    text(0,54,"* held / status not seen")
  elseif page == 2 then
    text(0,12,"ROLL  "..basicAngle("Roll"))
    text(0,22,"PITCH "..basicAngle("Ptch"))
    text(0,32,"ROBOT "..measurement(sensors,"RxBt","%.1f","V"))
    text(0,42,"MOTOR IQ "..measurement(sensors,"Curr","%.1f","A"))
    text(0,54,"* held value / -- missing")
  elseif page == 3 then
    for row=0,2 do
      text(2,13+row*10,roles[row*2+1].."  --")
      text(68,13+row*10,roles[row*2+2].."  --")
    end
    text(0,46,"MOTOR DETAILS UNAVAILABLE")
    text(0,56,"Need robot status packets")
  end
end
local function unavailable(now)
  if not status and not incompatible then basic()
  elseif incompatible then
    text(0,14,"UPDATE HOP LUA",MIDSIZE)
    text(0,33,"Robot schema unsupported")
  elseif status then
    text(0,14,"ROBOT DATA LOST",MIDSIZE)
    text(0,33,"Arming / motion UNKNOWN")
    text(0,44,"Age "..value(age(now,received)/100,"%.1f","s"))
  end
end
local function dataAge(time)
  local a = age(getTime(),time)
  return a < 99900 and string.format("%.1fs",a/100) or "--"
end
local function diagnostics()
  text(0,12,"LUA v3.1  LOG "..log.state)
  text(0,22,string.format("RX %d  HS %d",math.min(diag.rx,99999),math.min(diag.status,99999)))
  text(0,32,"Status age "..dataAge(received))
  text(0,42,"FM "..dataAge(sensors.FM and sensors.FM.time).." V "..dataAge(sensors.RxBt and sensors.RxBt.time))
  text(0,54,(log.state=="ERROR" or log.state=="LIMIT") and "Log stopped; reload Lua" or "Hold ENTER: toggle SD log")
end
local titles = {"HOPSCOTCH", "POSE + POWER", "SIX MOTORS", "RUN + EVENTS", "RADIO LINK", "DIAGNOSTICS"}
local function isEvent(e, constant) return constant ~= nil and e == constant end
local function run(e)
  background()
  if page == 6 and isEvent(e,EVT_VIRTUAL_ENTER_LONG) then
    if killEvents then killEvents(e) end -- consume release; do not change page
    toggleLog(getTime())
    e = 0
  end
  if isEvent(e,EVT_VIRTUAL_NEXT) or isEvent(e,EVT_VIRTUAL_ENTER) then page = page % pages + 1 end
  if isEvent(e,EVT_VIRTUAL_PREV) then page = (page + pages - 2) % pages + 1 end
  local now = getTime()
  lcd.clear()
  title(titles[page],now)
  if page == 6 then diagnostics()
  elseif page == 5 then radio()
  elseif page == 4 then history(now)
  elseif not available(now) then unavailable(now)
  elseif page == 1 then home()
  elseif page == 2 then health()
  elseif page == 3 then motors() end
  return 0
end
return {run=run, background=background}
