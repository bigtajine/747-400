--[[ B744 FMC - DESCENT FORECAST page (VNAV DES page 5R FORECAST>).
     Up to four altitudes with the forecast wind (true direction / speed), ISA deviation and destination
     QNH. 6L REQUEST WINDS fills the page from the SimBrief OFP (the destination's forecast winds by
     level, Output/B744_simbrief.xml) or, without one, from X-Plane's weather at the present position.
     Entries: altitude at the left keys (e.g. 24000, FL240), wind at the right keys (e.g. 270/45).
     The winds are sent to the AFDS (laminar/B747/fms/des_winds) and used for the descent path. ]]
B747DR_des_winds_fc = find_dataref("laminar/B747/fms/des_winds")
-- (dataref proxies as globals: XTLua returns dataref values through globals)
B747DR_fc_wxr_alt = find_dataref("sim/weather/aircraft/wind_altitude_msl_m")
B747DR_fc_wxr_dir = find_dataref("sim/weather/aircraft/wind_direction_degt")
B747DR_fc_wxr_spd = find_dataref("sim/weather/aircraft/wind_speed_kts")

local FC = { lv = {}, isa = nil, qnh = nil, src = nil }
for k = 1, 4 do FC.lv[k] = { alt = nil, dir = nil, spd = nil } end

local function fc_publish()
  local t = {}
  for _, l in ipairs(FC.lv) do
    if l.alt and l.dir and l.spd then t[#t + 1] = string.format("%d,%d,%d", l.alt, l.dir % 360, l.spd) end
  end
  B747DR_des_winds_fc = table.concat(t, ";")
end

local function fc_dest()
  local ok, t = pcall(function() return json.decode(fmsJSON) end)
  if ok and type(t) == "table" and #t > 0 then
    for i = #t, 1, -1 do if t[i][2] == 1 then return t[i][8] end end
  end
  return nil
end
-- the four forecast levels taken from a list of {alt, dir, spd}: spread over the descent
local function pick_levels(list)
  local crz = tonumber(fmsModules["data"]["crzalt"] or "") or 0
  if crz < 1000 then crz = 35000 end
  local want = { math.min(crz, 34000), 24000, 14000, 5000 }
  local out = {}
  for k, wa in ipairs(want) do
    local best, bd = nil, 1e9
    for _, l in ipairs(list) do
      local d = math.abs(l.alt - wa)
      if d < bd then best, bd = l, d end
    end
    if best then out[k] = best end
  end
  return out
end
local function from_simbrief(dest)
  local f = io.open("Output/B744_simbrief.xml", "r")
  if not f then return nil end
  local s = f:read("*a") or ""; f:close()
  local xdest = s:match("<destination>%s*<icao_code>(%w+)</icao_code>")
  if dest and xdest and xdest ~= dest then return nil, "OFP DEST " .. xdest end
  -- the last navlog fix (the destination) carries the forecast winds by level
  local nav = s:match("<navlog>(.*)</navlog>")
  if not nav then return nil end
  local last = nil
  for blk in nav:gmatch("<fix>(.-)</fix>") do last = blk end
  if not last then return nil end
  local list = {}
  for a, d, v in last:gmatch("<altitude>(%d+)</altitude>%s*<wind_dir>(%d+)</wind_dir>%s*<wind_spd>(%d+)</wind_spd>") do
    local alt = tonumber(a)
    if alt and alt >= 2000 then list[#list + 1] = { alt = alt, dir = tonumber(d), spd = tonumber(v) } end
  end
  if #list == 0 then return nil end
  local isa = tonumber(last:match("<oat_isa_dev>(%-?%d+)</oat_isa_dev>") or "")
  local metar = s:match("<destination>.-<metar>(.-)</metar>") or ""
  local q = tonumber(metar:match(" Q(%d%d%d%d)") or "")
  local a2 = metar:match(" A(%d%d%d%d)")
  return { lv = pick_levels(list), isa = isa, qnh = q or (a2 and (tonumber(a2) / 100)) }
end
local function from_xplane()
  local list = {}
  for i = 0, 12 do
    local okA, a = pcall(function() return B747DR_fc_wxr_alt[i] end)
    local okD, d = pcall(function() return B747DR_fc_wxr_dir[i] end)
    local okS, v = pcall(function() return B747DR_fc_wxr_spd[i] end)
    if okA and okD and okS and a and a > 0 then
      list[#list + 1] = { alt = math.floor(a * 3.28084 / 100 + 0.5) * 100, dir = math.floor(d + 0.5) % 360, spd = math.floor(v + 0.5) }
    end
  end
  if #list == 0 then return nil end
  return { lv = pick_levels(list) }
end
function fmsFunctions.desfcst(fmsO, value)
  local sp = fmsO["scratchpad"] or ""
  if value == "request" then
    local dest = fc_dest()
    local r, why = from_simbrief(dest)
    local src = "SIMBRIEF"
    if not r then r = from_xplane(); src = "X-PLANE WX" end
    if not r then fmsO["notify"] = why or "NO WIND DATA" return end
    for k = 1, 4 do
      local l = r.lv[k]
      FC.lv[k] = l and { alt = l.alt, dir = l.dir, spd = l.spd } or { }
    end
    if r.isa then FC.isa = r.isa end
    if r.qnh then FC.qnh = r.qnh end
    FC.src = src
    fc_publish()
    fmsO["notify"] = "WINDS FROM " .. src
    return
  end
  local k = tonumber(value:match("(%d)$"))
  if not k then return end
  if value:match("^alt") then
    if sp == "DELETE" then FC.lv[k] = {} fc_publish() fmsO["scratchpad"] = "" return end
    local fl = sp:match("^FL(%d%d%d?)$")
    local a = fl and tonumber(fl) * 100 or tonumber(sp)
    if not a or a < 1000 or a > 45000 then fmsO["notify"] = "INVALID ENTRY" return end
    FC.lv[k].alt = math.floor(a + 0.5)
  elseif value:match("^wind") then
    if sp == "DELETE" then FC.lv[k].dir = nil FC.lv[k].spd = nil fc_publish() fmsO["scratchpad"] = "" return end
    local d, v = sp:match("^(%d%d?%d?)/(%d%d?%d?)$")
    d, v = tonumber(d), tonumber(v)
    if not d or not v or d > 360 or v > 250 then fmsO["notify"] = "INVALID ENTRY" return end
    FC.lv[k].dir = d % 360; FC.lv[k].spd = v
  elseif value == "isa" then
    local v = tonumber(sp)
    if not v or math.abs(v) > 60 then fmsO["notify"] = "INVALID ENTRY" return end
    FC.isa = v
  elseif value == "qnh" then
    local v = tonumber(sp)
    if not v or not ((v > 900 and v < 1100) or (v > 27 and v < 32)) then fmsO["notify"] = "INVALID ENTRY" return end
    FC.qnh = v
  end
  FC.src = nil
  fmsO["scratchpad"] = ""
  fc_publish()
end

local function fmt_alt(a)
  if not a then return "-----" end
  local ta = tonumber(fmsModules["data"]["transalt"] or "") or 18000
  if a >= ta then return string.format("FL%03d", math.floor(a / 100 + 0.5)) end
  return string.format("%5d", a)
end
local function fmt_wind(l)
  if not (l and l.dir and l.spd) then return "---`/---" end
  return string.format("%03d`/%3d", l.dir, l.spd)
end
fmsPages["DESFCST"] = createPage("DESFCST")
fmsPages["DESFCST"].getPage = function(self, pgNo, fmsID)
  local tl = fmsModules["data"]["transalt"] or "-----"
  local rows = {}
  for k = 1, 4 do
    local l = FC.lv[k]
    rows[k] = string.format("%-5s", fmt_alt(l.alt)) .. string.rep(" ", 24 - 5 - 8) .. fmt_wind(l)
  end
  local isa = FC.isa and string.format("%+d`C", FC.isa) or "---"
  local qnh = FC.qnh and ((FC.qnh > 100) and string.format("%4d", math.floor(FC.qnh + 0.5)) or string.format("%5.2f", FC.qnh)) or "----"
  return {
    "      DES FORECASTS     ",
    "                        ",
    string.format("%-24s", "FL" .. string.format("%03d", math.floor((tonumber(tl) or 18000) / 100))),
    "                        ",
    rows[1],
    "                        ",
    rows[2],
    "                        ",
    rows[3],
    "                        ",
    rows[4],
    "                        ",
    "<REQUEST" .. string.format("%16s", isa .. "   " .. qnh):sub(-16),
  }
end
fmsPages["DESFCST"].getSmallPage = function(self, pgNo, fmsID)
  return {
    "                        ",
    " TRANS LVL              ",
    "                        ",
    " ALT        WIND DIR/SPD",
    "                        ",
    "                        ",
    "                        ",
    "                        ",
    "                        ",
    "                        ",
    "                        ",
    string.format(" %-10s  ISA DEV QNH", FC.src or ""),
    "                        ",
  }
end
fmsPages["DESFCST"].getNumPages = function(self) return 1 end
fmsFunctionsDefs["DESFCST"] = {}
fmsFunctionsDefs["DESFCST"]["L2"] = { "desfcst", "alt1" }
fmsFunctionsDefs["DESFCST"]["L3"] = { "desfcst", "alt2" }
fmsFunctionsDefs["DESFCST"]["L4"] = { "desfcst", "alt3" }
fmsFunctionsDefs["DESFCST"]["L5"] = { "desfcst", "alt4" }
fmsFunctionsDefs["DESFCST"]["R2"] = { "desfcst", "wind1" }
fmsFunctionsDefs["DESFCST"]["R3"] = { "desfcst", "wind2" }
fmsFunctionsDefs["DESFCST"]["R4"] = { "desfcst", "wind3" }
fmsFunctionsDefs["DESFCST"]["R5"] = { "desfcst", "wind4" }
fmsFunctionsDefs["DESFCST"]["L6"] = { "desfcst", "request" }
fmsFunctionsDefs["DESFCST"]["R6"] = { "desfcst", "qnh" }
