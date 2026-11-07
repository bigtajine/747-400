--[[ SimBrief flight plan request (RTE page 1, LSK 3L REQUEST, on the ground).
     The pilot ID (or username) is entered and saved on the MISC page (MENU, LSK 5R). The OFP opens
     the SIMBRIEF page: <APPLY ROUTE and <SET FUEL & PAYLOAD (block fuel set exactly). The b744_nd plugin downloads the latest OFP (XTLua has no
     network access); the FMC then
       - loads the route into the FMS: en-route waypoints and airways only - no runways, SIDs, STARs
         (selected afterwards on the DEP / ARR pages),
       - loads the aircraft: passengers by cabin zone, cargo in the lower holds (main deck when
         they are full), standard passenger weight from the OFP, block fuel via the fuel truck,
       - fills PERF INIT: ZFW, reserves, cost index, cruise altitude; flight number.
     Files (X-Plane folder): Output/B744_sb_request.txt, Output/B744_sb_status.txt (plugin),
     Output/B744_simbrief.xml (OFP), Output/B744_simbrief.fms (route), Output/B744_simbrief_id.txt.
     Original code. ]]

local SB = { id = nil, state = nil, t = 0, fmsO = nil, ofp = nil }
B744SIMBRIEF = SB

local REQ, STATUS, XML, FMSF, IDF = "Output/B744_sb_request.txt", "Output/B744_sb_status.txt",
  "Output/B744_simbrief.xml", "Output/B744_simbrief.fms", "Output/B744_simbrief_id.txt"

local function rd(path)
  local f = io.open(path, "rb")
  if not f then return nil end
  local s = f:read("*a"); f:close()
  return s
end
local function wr(path, s)
  local f = io.open(path, "wb")
  if not f then return false end
  f:write(s); f:close()
  return true
end

function SB.get_id()
  if SB.id == nil then
    local s = rd(IDF)
    SB.id = s and s:match("([%w_%-]+)") or ""
  end
  return SB.id
end

-- XML helpers (the OFP is flat, well-formed XML) ----------------------------------------------------
local function sect(x, name) return x and x:match("<" .. name .. ">(.-)</" .. name .. ">") end
local function tag(x, name)
  if not x then return nil end
  local v = x:match("<" .. name .. ">(.-)</" .. name .. ">")
  if v then v = v:gsub("^%s+", ""):gsub("%s+$", "") end
  return v
end
local function num(x, name) return tonumber(tag(x, name) or "") end

-- (v55) latitude/longitude waypoint name - the same rule as the LEGS page (fmc/legs.lua M.ll_name)
local function sb_ll_name(lat, lon)
  local ns, ew = (lat >= 0) and "N" or "S", (lon >= 0) and "E" or "W"
  local a, o = math.abs(lat), math.abs(lon)
  local fl = math.floor
  local wl, wo = math.abs(a - fl(a + 0.5)) < 1e-4, math.abs(o - fl(o + 0.5)) < 1e-4
  if wl and wo then return string.format("%s%02d%s%03d", ns, fl(a + 0.5), ew, fl(o + 0.5)) end
  if wo and ns == "N" and ew == "W" and o < 100 and math.abs(a * 2 - fl(a * 2 + 0.5)) < 1e-4 then
    return string.format("H%02d%02d", fl(a), fl(o + 0.5))
  end
  return string.format("%s%02d%s%03d", ns, fl(a), ew, fl(o))
end

function SB.parse(xml)
  if not xml or not xml:find("<OFP") then return nil, "NO OFP" end
  local fetch = sect(xml, "fetch")
  local st = tag(fetch, "status")
  if st and not st:lower():find("^success") then return nil, st end
  local o = {}
  local params, general, weights, fuel = sect(xml, "params"), sect(xml, "general"), sect(xml, "weights"), sect(xml, "fuel")
  local orig, dest = sect(xml, "origin"), sect(xml, "destination")
  if not (general and weights and fuel and orig and dest) then return nil, "INCOMPLETE OFP" end
  o.units = (tag(params, "units") or "kgs"):lower()
  local f = (o.units == "lbs") and (1 / 2.20462) or 1
  o.airac = tag(params, "airac") or ""
  o.origin = { icao = tag(orig, "icao_code"), lat = num(orig, "pos_lat"), lon = num(orig, "pos_long"), elev = num(orig, "elevation") or 0 }
  o.dest = { icao = tag(dest, "icao_code"), lat = num(dest, "pos_lat"), lon = num(dest, "pos_long"), elev = num(dest, "elevation") or 0 }
  o.fltno = (tag(general, "icao_airline") or "") .. (tag(general, "flight_number") or "")
  o.ci = num(general, "costindex")
  o.crz = num(general, "initial_altitude")
  o.sid = tag(general, "sid_ident"); o.star = tag(general, "star_ident")
  o.pax = math.floor((num(weights, "pax_count") or 0) + 0.5)
  o.paxw = (num(weights, "pax_weight") or 0) * f
  o.cargo = (num(weights, "cargo") or 0) * f
  o.payload = (num(weights, "payload") or 0) * f
  o.zfw = (num(weights, "est_zfw") or 0) * f
  o.ramp = (num(fuel, "plan_ramp") or 0) * f
  o.rsv = ((num(fuel, "reserve") or 0) + (num(fuel, "alternate_burn") or 0)) * f
  -- en-route fixes: no SID / STAR fixes, no TOC / TOD, no airports
  o.fixes = {}
  local nav = sect(xml, "navlog") or ""
  for fx in nav:gmatch("<fix>(.-)</fix>") do
    local id, ty = tag(fx, "ident"), (tag(fx, "type") or ""):lower()
    local sidstar = tag(fx, "is_sid_star") == "1"
    -- (v55) latitude/longitude points ("ltlg": oceanic / Pacific crossings, e.g. 55N040W, 5230N02000W)
    -- are route waypoints; SimBrief marks its TOC / TOD (and step climb) points the same way - those
    -- are left out by name
    local nm = (tag(fx, "name") or ""):upper()
    -- (only lat/lon-type points are checked: a real fix or navaid is never left out by its name,
    -- e.g. the Stephenville VOR)
    local pseudo = id == "TOC" or id == "TOD" or (ty == "ltlg" and ((id or ""):find("/") ~= nil
      or nm:find("^TOP OF") ~= nil or nm:find("STEP") ~= nil or (id or ""):match("^S/?C") ~= nil))
    if id and not sidstar and ty ~= "apt" and not pseudo then
      local via = tag(fx, "via_airway") or "DCT"
      if via == "DCT" or via == "" or via == o.sid or via == o.star or via:match("^SID") or via:match("^STAR") then via = "DRCT" end
      local la, lo = num(fx, "pos_lat"), num(fx, "pos_long")
      if ty == "ltlg" then
        -- named as on the LEGS page / ND (N55W040, H5230 ...)
        if not (la and lo) then id = nil
        else id = sb_ll_name(la, lo) end
      end
      if id then o.fixes[#o.fixes + 1] = { id = id, ty = ty, via = via, lat = la, lon = lo } end
    end
  end
  -- the first fix after the departure is reached direct (its airway starts there)
  if o.fixes[1] then o.fixes[1].via = "DRCT" end
  if not (o.origin.icao and o.dest.icao) then return nil, "NO AIRPORTS" end
  return o
end

-- X-Plane 11+ flight plan (no DEPRWY / SID / STAR / DESRWY lines) ----------------------------------
function SB.fms_text(o)
  local T = { wpt = 11, vor = 3, ndb = 2, ltlg = 28 }
  local crz = o.crz or 35000
  local L = { "I", "1100 Version", "CYCLE " .. (o.airac ~= "" and o.airac or "0000"),
    "ADEP " .. o.origin.icao, "ADES " .. o.dest.icao, "NUMENR " .. (#o.fixes + 2) }
  L[#L + 1] = string.format("1 %s ADEP %.6f %.6f %.6f", o.origin.icao, o.origin.elev, o.origin.lat or 0, o.origin.lon or 0)
  for _, x in ipairs(o.fixes) do
    -- (cruise altitude on every en-route fix, as SimBrief's own export: the FMC does not take the
    -- OFP's predicted altitudes as restrictions)
    L[#L + 1] = string.format("%d %s %s %.6f %.6f %.6f", T[x.ty] or 11, x.id, x.via, crz, x.lat or 0, x.lon or 0)
  end
  L[#L + 1] = string.format("1 %s ADES %.6f %.6f %.6f", o.dest.icao, o.dest.elev, o.dest.lat or 0, o.dest.lon or 0)
  return table.concat(L, "\n") .. "\n"
end

-- split n into parts proportional to caps (largest remainder), each capped
local function split(n, caps)
  local tot = 0
  for _, c in ipairs(caps) do tot = tot + c end
  local out, rem, used = {}, {}, 0
  for i, c in ipairs(caps) do
    local v = n * c / tot
    out[i] = math.min(c, math.floor(v)); used = used + out[i]; rem[i] = { i = i, r = v - math.floor(v) }
  end
  table.sort(rem, function(a, b) return a.r > b.r end)
  local k = 1
  while used < math.floor(n + 0.5) and k <= #rem * 2 do
    local i = rem[(k - 1) % #rem + 1].i
    if out[i] < caps[i] then out[i] = out[i] + 1; used = used + 1 end
    k = k + 1
  end
  return out
end
SB.split = split

local PAX_CAP = { 23, 80, 77, 104, 132 }
local HOLD_CAP = { 26490, 22938, 6749 }
local MAIN_CAP = { 11289, 30104, 22578, 45156, 3763 }

-- aircraft loading: the existing PAX / CARGO data and calculations (CG, trim) are used
function SB.load_aircraft(o, fmsO)
  local msg = nil
  local pax = math.min(416, math.max(0, o.pax))
  local p = split(pax, PAX_CAP)
  setFMSData("paxFirstClassA", string.format("%2d", p[1]))
  setFMSData("paxBusClassB", string.format("%2d", p[2]))
  setFMSData("paxEconClassC", string.format("%2d", p[3]))
  setFMSData("paxEconClassD", string.format("%3d", p[4]))
  setFMSData("paxEconClassE", string.format("%3d", p[5]))
  if pax > 0 and o.paxw > 40 and o.paxw < 200 then
    -- the OFP's standard passenger weight (bags are in the OFP cargo figure)
    simConfigData["data"].SIM.std_pax_weight = string.format("%5.1f", o.paxw)
    pushSimConfig(simConfigData["data"]["values"])
  end
  local cargo = math.max(0, o.cargo)
  local hold_total = HOLD_CAP[1] + HOLD_CAP[2] + HOLD_CAP[3]
  local in_holds = math.min(cargo, hold_total)
  local h = split(in_holds, HOLD_CAP)
  setFMSData("cargoFwd", string.format("%6d", h[1]))
  setFMSData("cargoAft", string.format("%6d", h[2]))
  setFMSData("cargoBulk", string.format("%5d", h[3]))
  local rest = cargo - in_holds
  local m = { 0, 0, 0, 0, 0 }
  if rest > 1 then
    if pax == 0 then m = split(math.min(rest, 112890), MAIN_CAP) else msg = "CARGO EXCEEDS HOLDS" end
  end
  setFMSData("freightZoneA", string.format("%5d", m[1]))
  setFMSData("freightZoneB", string.format("%5d", m[2]))
  setFMSData("freightZoneC", string.format("%6d", m[3]))
  setFMSData("freightZoneD", string.format("%6d", m[4]))
  setFMSData("freightZoneE", string.format("%4d", m[5]))
  calc_pax_cargo()
  -- block fuel: set exactly (more or less than on board), in the 747-400 loading order
  local cur = o.ramp
  local okf, fm = pcall(SB.set_fuel, o.ramp)
  if not okf then msg = msg or "FUEL NOT SET" elseif fm then msg = msg or fm end
  -- PERF INIT
  local loaded = pax * (tonumber(simConfigData["data"].SIM.std_pax_weight) or 120) + in_holds
  for i = 1, 5 do loaded = loaded + m[i] end
  local zfw = ((tonumber(simDR_empty_weight) or 0) + loaded) / 1000
  setFMSData("zfw", string.format("%5.1f", zfw))
  setFMSData("grwt", string.format("%5.1f", zfw + math.max(cur, o.ramp) / 1000))
  if o.rsv > 0 then setFMSData("reserves", string.format("%5.1f", o.rsv / 1000)) end
  if o.ci then setFMSData("costindex", string.format("%d", o.ci)) end
  if o.fltno ~= "" then setFMSData("fltno", o.fltno) end
  calc_CGMAC()
  return msg
end

-- 747-400 tank loading order (FCOM): mains 1-4 equally until mains 1 and 4 are full, then mains 2 and
-- 3, then the reserve tanks (2 and 3), then the center tank, then the horizontal stabilizer tank.
-- X-Plane tank index: 0 center, 1-4 mains, 5 / 6 reserves, 7 stabilizer (as in the fuel system).
local CAP = { [0] = 52167.0, [1] = 13622.0, [2] = 38132.0, [3] = 38132.0, [4] = 13622.0, [5] = 4018.0, [6] = 4018.0, [7] = 10030.0 }
function SB.fuel_split(total)
  local t = { [0] = 0, 0, 0, 0, 0, 0, 0, 0 }
  local f = math.max(0, total)
  local each = math.min(f / 4, CAP[1])
  t[1], t[2], t[3], t[4] = each, each, each, each
  f = f - 4 * each
  local m23 = math.min(f / 2, CAP[2] - each)
  t[2], t[3] = t[2] + m23, t[3] + m23
  f = f - 2 * m23
  local r = math.min(f / 2, CAP[5])
  t[5], t[6] = r, r
  f = f - 2 * r
  t[0] = math.min(f, CAP[0]); f = f - t[0]
  t[7] = math.min(f, CAP[7]); f = f - t[7]
  return t, f
end
function SB.set_fuel(total)
  local t, left = SB.fuel_split(total)
  B747DR_refuel = 0
  B747DR_fuel_add = 0
  for i = 0, 7 do simDR_fuel_qty[i] = t[i] end
  local set = 0
  for i = 0, 7 do set = set + t[i] end
  B747DR_fuel_preselect = set
  simDR_fuel_totalizer_kg = set
  if left > 1 then return "FUEL EXCEEDS CAPACITY" end
end

local function notify(m) if SB.fmsO then SB.fmsO["notify"] = m end end
local function finish() SB.state = nil; if is_timer_scheduled(SB_poll) then stop_timer(SB_poll) end end

function SB_set_crz()
  local o, f = SB.ofp, SB.fmsO
  if not (o and f and o.crz and o.crz >= 1000) then return end
  local keep = f["scratchpad"]
  f["scratchpad"] = string.format("FL%03d", math.floor(o.crz / 100 + 0.5))
  pcall(fmsFunctions.setdata, f, "crzalt")
  f["scratchpad"] = keep or ""
end

function SB_poll()
  if not SB.state then finish() return end
  local s = rd(STATUS) or ""
  if s:match("^ERR") then
    notify((s:match("^ERR (%u+)") == "NOAPI") and "SIMBRIEF NEEDS XP 12.1" or "SIMBRIEF NET ERROR")
    finish() return
  end
  if SB.state == "FETCH" and s:match("^OK FETCH") then
    local o, err = SB.parse(rd(XML))
    if not o then notify(("SIMBRIEF " .. tostring(err)):upper():sub(1, 24)) finish() return end
    SB.ofp = o
    SB.route_done, SB.load_done, SB.crz_pending = nil, nil, nil
    notify("SIMBRIEF OFP RECEIVED")
    finish()
    if SB.fmsO then pcall(fmsFunctions.setpage, SB.fmsO, "SIMBRIEF") end
    return
  end
  if SB.state == "LOAD" and s:match("^OK LOAD") then
    local o = SB.ofp
    setFMSData("fltdep", o.origin.icao)
    setFMSData("fltdst", o.dest.icao)
    run_after_time(SB_set_crz, 2.0)
    SB.route_done = true
    notify("SIMBRIEF ROUTE LOADED")
    finish() return
  end
  if simDRTime - SB.t > 30 then notify("SIMBRIEF NO RESPONSE") finish() end
end

-- REQUEST (RTE page 1, LSK 3L): the saved ID from the MISC page
function SB.request(fmsO)
  SB.fmsO = fmsO
  if SB.get_id() == "" then fmsO["notify"] = "SIMBRIEF ID - SEE MISC" return end
  if simDR_onGround ~= 1 then fmsO["notify"] = "NOT ON GROUND" return end
  if SB.state then fmsO["notify"] = "SIMBRIEF BUSY" return end
  wr(STATUS, "WAIT")
  if not wr(REQ, "FETCH " .. SB.id) then fmsO["notify"] = "SIMBRIEF FILE ERROR" return end
  SB.state = "FETCH"; SB.t = simDRTime
  fmsO["notify"] = "SIMBRIEF REQUESTED"
  run_at_interval(SB_poll, 0.5)
end

fmsFunctions["sbRequest"] = function(fmsO, value) SB.request(fmsO) end
-- MISC page: LSK 1L enters the SimBrief pilot ID (or username), LSK 1R saves it
fmsFunctions["sbId"] = function(fmsO, value)
  local sp = fmsO["scratchpad"] or ""
  if sp == "DELETE" then SB.pending = ""; fmsO["scratchpad"] = "" return end
  if sp == "" then fmsO["scratchpad"] = SB.pending or SB.get_id() return end
  local id = sp:match("^([%w_%-]+)$")
  if not id or #id > 24 then fmsO["notify"] = "INVALID ENTRY" return end
  SB.pending = id
  fmsO["scratchpad"] = ""
end
fmsFunctions["sbSave"] = function(fmsO, value)
  local id = SB.pending
  if id == nil then fmsO["notify"] = "ENTER SIMBRIEF ID" return end
  if not wr(IDF, id) then fmsO["notify"] = "SAVE FAILED" return end
  SB.id = id; SB.pending = nil
  fmsO["notify"] = (id ~= "") and "SIMBRIEF ID SAVED" or "SIMBRIEF ID CLEARED"
end

-- MISC page LSK 2L: hardware throttle lock. ON: while an autothrottle mode is active (incl. HOLD) the
-- thrust levers ignore the hardware throttle axis; after the autothrottle disconnects the hardware takes
-- over once it is moved or brought to the lever position. Saved in Output/preferences/B744_options.txt
-- (read by the engine EEC script).
local OPTF = "Output/preferences/B744_options.txt"
local function opt_thr_lock()
  local s = rd(OPTF)
  if s and s:match("thr_hw_lock%s+0") then return false end
  return true
end
SB.thr_lock = opt_thr_lock()
fmsFunctions["thrLock"] = function(fmsO, value)
  SB.thr_lock = not SB.thr_lock
  if not wr(OPTF, "thr_hw_lock " .. (SB.thr_lock and "1" or "0") .. "\n") then fmsO["notify"] = "SAVE FAILED" end
end

fmsPages["MISC"] = createPage("MISC")
fmsPages["MISC"].getPage = function(self, pgNo, fmsID)
  local id = SB.pending or SB.get_id()
  local shown = (id ~= "") and id or "--------"
  local save = SB.pending and "SAVE>" or "     "
  return {
    "          MISC          ",
    "                        ",
    shown .. string.rep(" ", math.max(0, 24 - #shown - #save)) .. save,
    "                        ",
    SB.thr_lock and "<ON                     " or "<OFF                    ",
    "                        ",
    "                        ",
    "                        ",
    "                        ",
    "                        ",
    "                        ",
    "------------------------",
    "<INDEX                  ",
  }
end
fmsPages["MISC"].getSmallPage = function(self, pgNo, fmsID)
  local st = SB.pending and "             (NOT SAVED)" or ((SB.get_id() ~= "") and "                 (SAVED)" or "                        ")
  return {
    "                        ",
    " SIMBRIEF ID            ",
    "                        ",
    st,
    " THR HW LOCK (A/T)      ",
    "                        ",
    "                        ",
    "                        ",
    "                        ",
    "                        ",
    "                        ",
    "                        ",
    "                        ",
  }
end
fmsFunctionsDefs["MISC"]["L1"] = { "sbId", "" }
fmsFunctionsDefs["MISC"]["R1"] = { "sbSave", "" }
fmsFunctionsDefs["MISC"]["L2"] = { "thrLock", "" }
fmsFunctionsDefs["MISC"]["L6"] = { "setpage", "INDEX" }

-- SIMBRIEF page (opened by REQUEST): the OFP and the two load options
function SB.apply_route(fmsO)
  SB.fmsO = fmsO
  local o = SB.ofp
  if not o then fmsO["notify"] = "NO SIMBRIEF OFP" return end
  if simDR_onGround ~= 1 then fmsO["notify"] = "NOT ON GROUND" return end
  if SB.state then fmsO["notify"] = "SIMBRIEF BUSY" return end
  -- the route becomes a modification (light blue on the ND, EXEC lit); EXEC loads it
  local T = { wpt = 512, vor = 4, ndb = 2, ltlg = 2048 }
  local list = { { id = o.origin.icao, lat = o.origin.lat, lon = o.origin.lon, ty = 1, alt = o.origin.elev } }
  for _, x in ipairs(o.fixes) do
    list[#list + 1] = { id = x.id, lat = x.lat, lon = x.lon, ty = T[x.ty] or 512, alt = o.crz or 0, kind = (x.ty == "ltlg") and "LL" or nil }
  end
  list[#list + 1] = { id = o.dest.icao, lat = o.dest.lat, lon = o.dest.lon, ty = 1, alt = o.dest.elev }
  if not (B744_legs_new_mod and B744_legs_new_mod(list)) then fmsO["notify"] = "SIMBRIEF ROUTE ERROR" return end
  setFMSData("fltdep", o.origin.icao)
  setFMSData("fltdst", o.dest.icao)
  SB.route_done = "MOD"
  SB.crz_pending = true
  fmsO["notify"] = "ROUTE MODIFIED - EXEC"
end
-- EXEC pressed (LEGS module): the route is in the flight plan - cruise altitude now
function B744_on_legs_exec()
  if SB.crz_pending then
    SB.crz_pending = nil
    SB.route_done = true
    run_after_time(SB_set_crz, 2.0)
  end
end
function SB.set_payload(fmsO)
  SB.fmsO = fmsO
  local o = SB.ofp
  if not o then fmsO["notify"] = "NO SIMBRIEF OFP" return end
  if simDR_onGround ~= 1 then fmsO["notify"] = "NOT ON GROUND" return end
  local ok, m = pcall(SB.load_aircraft, o, fmsO)
  if not ok then fmsO["notify"] = "SIMBRIEF LOAD ERROR" return end
  SB.load_done = true
  fmsO["notify"] = m or "FUEL & PAYLOAD SET"
end
fmsFunctions["sbApplyRoute"] = function(fmsO, value) SB.apply_route(fmsO) end
fmsFunctions["sbSetLoad"] = function(fmsO, value) SB.set_payload(fmsO) end

-- v45: OPTIMIZE CG - the lower-hold cargo (same total) is redistributed between the forward, aft and
-- bulk holds to bring the CG as close as possible to CG_TARGET % MAC (fuel and passengers unchanged)
local CG_TARGET = 24.0
function SB.optimize_cg(fmsO)
  calc_CGMAC()
  local f0, a0, b0 = wb.fwd_lower_cargo_weight or 0, wb.aft_lower_cargo_weight or 0, wb.bulk_lower_cargo_weight or 0
  local T = f0 + a0 + b0
  if T < 100 then fmsO["notify"] = "NO HOLD CARGO TO MOVE" return end
  local root, tip = 648.99996, 159.99996
  local tp = tip / root
  local MAC = (2 / 3) * root * ((1 + tp + tp * tp) / (1 + tp))
  local LEMAC = 1129.87932
  local GW = wb.OEW_weight + wb.fuel_CTR_0_weight + wb.fuel_L_main1_weight + wb.fuel_L_main2_weight + wb.fuel_R_main3_weight
    + wb.fuel_R_main4_weight + wb.fuel_L_rsv5_weight + wb.fuel_R_rsv6_weight + wb.fuel_stab7_weight
    + wb.passenger_zoneA_weight + wb.passenger_zoneB_weight + wb.passenger_zoneC_weight + wb.passenger_zoneD_weight
    + wb.passenger_zoneE_weight + T + wb.freight_zoneA_weight + wb.freight_zoneB_weight + wb.freight_zoneC_weight
    + wb.freight_zoneD_weight + wb.freight_zoneE_weight
  if GW < 1000 then return end
  local cg0 = LEMAC + (wb.cg_mac or 0) / 100 * MAC
  local df, da, db = wb.fwd_lower_cargo_distance, wb.aft_lower_cargo_distance, wb.bulk_lower_cargo_distance
  local best, bf, ba, bb = 1e9, f0, a0, b0
  local fmax = math.min(T, HOLD_CAP[1])
  local f = 0
  while f <= fmax + 0.1 do
    local rem = T - f
    local a = math.min(rem, HOLD_CAP[2])
    local b = rem - a
    if b <= HOLD_CAP[3] then
      local cg = cg0 + ((f - f0) * df + (a - a0) * da + (b - b0) * db) / GW
      local d = math.abs((cg - LEMAC) / MAC * 100 - CG_TARGET)
      if d < best then best, bf, ba, bb = d, f, a, b end
    end
    f = f + 50
  end
  setFMSData("cargoFwd", string.format("%6d", math.floor(bf + 0.5)))
  setFMSData("cargoAft", string.format("%6d", math.floor(ba + 0.5)))
  setFMSData("cargoBulk", string.format("%5d", math.floor(T - math.floor(bf + 0.5) - math.floor(ba + 0.5))))
  calc_pax_cargo()
  calc_CGMAC()
  SB.cg_done = string.format("%.1f", wb.cg_mac or 0)
  fmsO["notify"] = "CG " .. SB.cg_done .. "% MAC"
end
fmsFunctions["sbOptCG"] = function(fmsO, value)
  local ok, e = pcall(SB.optimize_cg, fmsO)
  if not ok then print("OPTIMIZE CG error " .. tostring(e)) fmsO["notify"] = "INVALID ENTRY" end
end

local function lr(l, r, w)
  w = w or 24
  l, r = l or "", r or ""
  return l .. string.rep(" ", math.max(1, w - #l - #r)) .. r
end
local function wt(kg)
  local f = (simConfigData["data"].SIM.weight_display_units == "LBS") and simConfigData["data"].SIM.kgs_to_lbs or 1
  return string.format("%.1f", kg * f / 1000)
end
fmsPages["SIMBRIEF"] = createPage("SIMBRIEF")
fmsPages["SIMBRIEF"].getPage = function(self, pgNo, fmsID)
  local o = SB.ofp
  if not o then
    return { "      SIMBRIEF OFP      ", "                        ", "NO OFP - REQUEST ON RTE 1",
      "                        ", "                        ", "                        ", "                        ",
      "                        ", "                        ", "                        ", "                        ",
      "------------------------", "<RTE 1                  " }
  end
  local loaded = (o.payload or 0)
  return {
    "      SIMBRIEF OFP      ",
    "                        ",
    lr(o.origin.icao, o.dest.icao),
    "                        ",
    lr((o.fltno ~= "" and o.fltno or "-----"):sub(1, 10), o.crz and string.format("FL%03d", math.floor(o.crz / 100 + 0.5)) or "-----"),
    "                        ",
    lr(wt(o.ramp), wt((tonumber(simDR_empty_weight) or 0) + loaded)),
    "                        ",
    lr("<APPLY ROUTE", ""),
    "                        ",
    lr("<SET FUEL & PAYLOAD", "CG>"),
    "------------------------",
    lr("<RTE 1", "REQUEST>"),
  }
end
fmsPages["SIMBRIEF"].getSmallPage = function(self, pgNo, fmsID)
  local o = SB.ofp
  local u = simConfigData["data"].SIM.weight_display_units or "KGS"
  local blank = "                        "
  if not o then return { blank, blank, blank, blank, blank, blank, blank, blank, blank, blank, blank, blank, blank } end
  return {
    blank,
    lr(" ORIGIN", "DEST"),
    blank,
    lr(" FLT NO", "CRZ ALT"),
    blank,
    lr(" BLOCK FUEL", "ZFW " .. u),
    blank,
    lr(" PAX " .. tostring(o.pax), "PAYLOAD " .. wt(o.payload or 0)),
    lr("", (SB.route_done == "MOD" and "EXEC") or (SB.route_done and "LOADED") or ""),
    lr((SB.load_done and " SET" or "") .. (SB.cg_done and ("  CG " .. SB.cg_done .. "%") or ""), "OPTIMIZE"),
blank,
    blank,
    blank,
  }
end
fmsFunctionsDefs["SIMBRIEF"]["L4"] = { "sbApplyRoute", "" }
fmsFunctionsDefs["SIMBRIEF"]["L5"] = { "sbSetLoad", "" }
fmsFunctionsDefs["SIMBRIEF"]["R5"] = { "sbOptCG", "" }
fmsFunctionsDefs["SIMBRIEF"]["L6"] = { "setpage", "RTE1" }
fmsFunctionsDefs["SIMBRIEF"]["R6"] = { "sbRequest", "" }
