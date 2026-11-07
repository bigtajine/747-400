--[[ RTE 1 LEGS page - native Boeing logic (fmc/legs.lua).
     Replaces the proxy of the X-Plane LEGS page: direct-to, insert/connect, delete, discontinuities,
     constraints and INTC CRS work on a MOD route; EXEC reloads the X-Plane FMS and publishes the
     extra data for the AFDS (laminar/B747/fms/legs_ext). ]]
if B744GEO == nil then dofile("fmc/geo.lua") end
if B744NAVDB == nil then dofile("fmc/navdb.lua") end
dofile("fmc/legs.lua")

local LG = B744LEGS
local NDB = B744NAVDB

B747DR_legs_ext            = find_dataref("laminar/B747/fms/legs_ext")
B747DR_legs_pred           = find_dataref("laminar/B747/fms/legs_pred")
B747DR_exec_light_pilot    = find_dataref("laminar/B747/fms/exec_light_pilot")
B747DR_nd_mod_geom         = find_dataref("laminar/B747/nd/mod_geom")
B747DR_exec_light_copilot  = find_dataref("laminar/B747/fms/exec_light_copilot")
B747DR_lnav_active         = find_dataref("laminar/B747/fms/lnav_active")
simDR_legs_vx              = find_dataref("sim/flightmodel/position/local_vx")
simDR_legs_vz              = find_dataref("sim/flightmodel/position/local_vz")
simDR_legs_hdg             = find_dataref("sim/flightmodel/position/true_psi")
simDR_legs_alt             = find_dataref("sim/cockpit2/gauges/indicators/altitude_ft_pilot")
simDR_xp_exec_light_pilot  = find_dataref("sim/cockpit2/radios/indicators/fms_exec_light_pilot")
simDR_xp_exec_light_copilot = find_dataref("sim/cockpit2/radios/indicators/fms_exec_light_copilot")

-- NOTE: state must stay in locals: XTLua wraps tables assigned to globals (copies on write, and
-- reading a missing key of a wrapped table recurses forever)
local S = { act = nil, mod = nil, dir = nil, serial = 0, pending = nil, xp_sig = "" }
B747DR_nd_plan_ctr_lat = find_dataref("laminar/B747/nd/plan_ctr_lat")
B747DR_nd_plan_ctr_lon = find_dataref("laminar/B747/nd/plan_ctr_lon")
local navdb_started = false
local last_sync = -10

-- transition altitude from the nav database (the origin's SIDs, e.g. RJTT 14000) while the pilot has not
-- entered one (the FMC default is 18000): altitudes above it are shown as flight levels
local ta_cache = {}
local function db_trans_alt()
  local R = S and S.act
  if not (R and R.wps and NDB and NDB.ready and NDB.procedures) then return nil end
  local org = nil
  for _, w in ipairs(R.wps) do if not w.disco and w.ty == 1 then org = w.id break end end
  if not org then return nil end
  if ta_cache[org] ~= nil then return ta_cache[org] or nil end
  local ok, P = pcall(NDB.procedures, org)
  local ta = false
  if ok and P then
    for _, sid in pairs(P.sid or {}) do
      for _, l in pairs(sid.rwy or {}) do for _, x in ipairs(l) do if x.trans_alt and x.trans_alt > 0 then ta = x.trans_alt break end end if ta then break end end
      if ta then break end
      for _, x in ipairs(sid.common or {}) do if x.trans_alt and x.trans_alt > 0 then ta = x.trans_alt break end end
      if ta then break end
    end
    ta_cache[org] = ta
  end
  return ta or nil
end
local function legs_ctx()
  local ta_s = getFMSData and getFMSData("transalt") or ""
  local ta = tonumber(ta_s) or 18000
  if ta_s == "18000" or ta_s == "" then
    local okd, d = pcall(db_trans_alt)
    if okd and d then ta = d end
  end
  local vx, vz = simDR_legs_vx or 0, simDR_legs_vz or 0
  local trk = simDR_legs_hdg or 0
  if vx * vx + vz * vz > 100 then trk = math.deg(math.atan2(vx, -vz)) % 360 end
  local gs = math.sqrt(vx * vx + vz * vz) * 1.94384
  -- VNAV predictions from the AFDS (altitude / speed at each waypoint ahead): small font on the page
  local pred = {}
  local ok, ps = pcall(function() return B747DR_legs_pred end)
  if ok and type(ps) == "string" and #ps > 3 then
    for rec in ps:gmatch("[^;]+") do
      local f = {}
      for v in (rec .. ","):gmatch("([^,]*),") do f[#f + 1] = tonumber(v) end
      if f[1] and f[2] and f[3] and f[4] then
        pred[#pred + 1] = { lat = f[1], lon = f[2], alt = f[3], spd = f[4], t = f[5], dtg = f[6] }
      end
    end
  end
  local plan = ((tonumber(simDR_nd_mode_dial_capt) or 0) == 3) or ((tonumber(simDR_nd_mode_dial_fo) or 0) == 3)
  return { inflight = (simDR_onGround ~= 1), ppos = { lat = simDR_latitude, lon = simDR_longitude, trk = trk, gs = gs }, ta = ta, pred = pred, plan = plan }
end

local function decode_fms()
  local s = fmsJSON
  if s == nil or string.len(s) < 3 then return nil end
  local ok, t = pcall(json.decode, s)
  if not ok or type(t) ~= "table" then return nil end
  return t
end

local function crz_alt()
  local ok, v = pcall(function() return B747BR_cruiseAlt end)
  return (ok and tonumber(v)) or 0
end

local function imp_opt()
  local ok, r = pcall(function() return getFMSData("runway") end)
  return { rwy = ok and r or nil }
end

-- SID climb leg for the AFDS and the ND: X-Plane's flight plan does not keep the SID's first leg
-- (a CA "climb on course to an altitude", e.g. PARRY1 RW12: 122 deg to 2400 ft, then direct PARRY) or
-- the departure runway once the route is executed. The departure runway and that leg, from the
-- nav database (CIFP), are attached to the first SID fix in the data for the AFDS (x.dep), which
-- puts them into its path ahead of that fix. X-Plane's flight plan is not changed.
local function sid_departure(R, o)
  if not (R and o and o.e and NDB and NDB.ready and NDB.procedures) then return end
  for _, en in ipairs(o.e) do if en[3] and (en[3].ca or en[3].dep or en[3].pca or en[3].proc) then return end end
  local org = nil
  for _, w in ipairs(R.wps) do if not w.disco and w.ty == 1 then org = w break end end
  if not org or not org.id then return end
  local okp, P = pcall(NDB.procedures, org.id)
  if not okp or not P or not P.rwys or not P.sid then return end
  -- departure runway: RTE page selection, else the runway in the route
  local rid = nil
  local okr, r = pcall(function() return getFMSData("runway") end)
  if okr and type(r) == "string" then
    r = r:gsub("%s", "")
    if r ~= "" and not r:find("%-") then rid = r:find("^RW") and r or ("RW" .. r) end
  end
  if not rid then for _, w in ipairs(R.wps) do if not w.disco and w.rw and w.id then rid = w.id break end end end
  if not rid then return end
  local rwy = nil
  for k, v in pairs(P.rwys) do if (tostring(k):gsub("%s", "")) == rid then rwy = v end end
  if not rwy then return end
  -- route entries (index in o.e) by ident, and the first route fix after the origin / runway
  local idx, first = {}, nil
  for i, en in ipairs(o.e) do
    local x = en[3]
    if x and x.id and not x.xd and not x.rw and x.id ~= org.id and not tostring(x.id):find("^RW") then
      idx[x.id] = idx[x.id] or i
      first = first or i
    end
  end
  if not first then return end
  -- the SID (runway transition + common part) with the most fixes in the route
  local nrw = rid:match("^RW(%d%d)")
  local best, bs = nil, 0
  for _, name in ipairs(P.sid_list or {}) do
    local sid = P.sid[name]
    for t, legs in pairs(sid and sid.rwy or {}) do
      local tt = tostring(t):gsub("%s", "")
      if tt == rid or (nrw and tt == ("RW" .. nrw .. "B")) then
        local L = {}
        for _, l in ipairs(legs) do L[#L + 1] = l end
        for _, l in ipairs(sid.common or {}) do L[#L + 1] = l end
        local sc = 0
        for _, l in ipairs(L) do if l.ident and idx[l.ident] then sc = sc + 1 end end
        if sc > bs then best, bs = L, sc end
      end
    end
  end
  if not best then return end
  local function ca_of(l)
    if (l.path == "CA" or l.path == "VA") and l.crs and l.alt1 and l.alt1 > 0 then
      return { l.crs_is_true and l.crs or B744GEO.mag_to_true(l.crs, rwy.lat, rwy.lon), l.alt1 }
    end
  end
  -- next route entry after entry i, nil when an open discontinuity is in between
  local function next_entry(i)
    for k = i + 1, #o.e do
      local x = o.e[k][3]
      if x and x.xd then
        if not x.xdc then return nil end
      elseif x then
        if x.db then return nil end
        return k
      end
    end
  end
  -- climb leg right after the runway: on the first route fix (with the runway)
  local c1 = best[1] and not best[1].ident and ca_of(best[1])
  if c1 then
    local x = o.e[first][3]
    x.dep = { rw = { rid, rwy.lat, rwy.lon }, ca = c1 }
    -- the next procedure leg is a direct-to (e.g. PARRY1: climb to 2400 ft, then direct PARRY)
    local nx = best[2]
    if nx and nx.path == "DF" and nx.ident == x.id then x.dep.df = true end
  end
  -- climb legs after a fix (e.g. BN4: DE19L, climb on 196 to 600 ft, vectors): on that fix; the
  -- waypoint after it (the procedure's direct-to, or the route once the vector discontinuity is
  -- cleared) is flown direct from the end of the climb
  for k = 1, #best do
    local l = best[k]
    local ei = l.ident and idx[l.ident]
    local c = best[k + 1] and not best[k + 1].ident and ca_of(best[k + 1])
    if ei and c and not (c1 and k == 1) then
      o.e[ei][3].pca = c
      local j = k + 2
      while best[j] and not best[j].ident do j = j + 1 end
      local ni = next_entry(ei)
      if ni then
        local nxl = best[j]
        if (nxl and nxl.path == "DF" and nxl.ident == o.e[ni][3].id) or (not nxl) then o.e[ni][3].pdf = true end
      end
    end
  end
end

-- PROCEDURE LEGS X-PLANE DOES NOT KEEP ---------------------------------------------------------------
-- X-Plane's flight plan drops the SID legs that end at an altitude rather than at a fix (CA "on course
-- to an altitude", VA "on heading to an altitude"), e.g. KALO1S RW36L: 002 deg to 520 ft, then SOORO.
-- They are generated from the nav database (CIFP) and shown on the LEGS page as on the aircraft:
-- "002`" / "(520)", in sequence (active until the altitude is passed). They are LEGS lines only: not
-- sent to X-Plane (fms_text skips them) and not in the AFDS data - the AFDS already flies them from
-- the same nav data (x.dep / x.pca above).
local function remove_plegs(R)
  local out, act = {}, R.active or 0
  local na = act
  for i, w in ipairs(R.wps) do
    if w.pleg then
      if i < act then na = na - 1 end
    else out[#out + 1] = w end
  end
  R.wps = out
  R.active = math.max(0, math.min(na, #out))
  R.proc_ann = nil
  R.rm_hit = nil
end

-- (v52) generated legs the pilot removed on the LEGS page (a later waypoint brought up over
-- (VECTORS) / (900) ..., or DELETE): kept on the route waypoint they follow (w.prm, set by
-- fmc/legs.lua), by the leg's definition. That waypoint is where the leg is generated after, so the
-- removal stays with it while it is in the route, and goes when it goes (another SID / runway).
local function psig(l)
  return string.format("%s:%s:%s:%s", tostring(l.path), tostring(l.crs or ""), tostring(l.alt1 or ""), tostring(l.dist or ""))
end
local function prm_removed(R, anchor, l)
  if anchor and type(anchor.prm) == "table" and anchor.prm[psig(l)] then
    R.rm_hit = R.rm_hit or {}
    R.rm_hit[anchor] = true
    return true
  end
  return false
end

-- altitude-terminated legs: CA (course to altitude), VA (heading to altitude), FA (from a fix on a
-- course to altitude). X-Plane keeps the fix-terminated legs only.
local ALT_LEG = { CA = true, VA = true, FA = true }
local function cat(...)
  local out = {}
  for _, l in ipairs({ ... }) do for _, x in ipairs(l or {}) do out[#out + 1] = x end end
  return out
end
-- (v52) in name order: where two procedures fit the route equally (YAMB RW33: AMB4 and BRISY3 both
-- start with D326D) the same one is always chosen - the first by name, as for the AFDS departure
-- data (sid_departure) - not whichever came first in the table (could differ from flight to flight)
local function skeys(t)
  local ks = {}
  for k in pairs(t or {}) do ks[#ks + 1] = k end
  table.sort(ks, function(x, y) return tostring(x) < tostring(y) end)
  return ks
end
local function vals(t)
  local o = {}
  for _, k in ipairs(skeys(t)) do o[#o + 1] = t[k] end
  if #o == 0 then o[1] = {} end
  return o
end
-- the procedure leg sequences of an airport: SIDs (runway + common + transition), STARs
-- (transition + common + runway), approaches (transition + final + missed)
local function sequences(P, kind, rid)
  local seqs = {}
  if kind == "SID" then
    local nrw = rid and rid:match("^RW(%d%d)")
    for _, sn in ipairs(skeys(P.sid)) do
      local sid = P.sid[sn]
      local rws = {}
      for _, t in ipairs(skeys(sid.rwy)) do
        local l = sid.rwy[t]
        local tt = tostring(t):gsub("%s", "")
        if (not rid) or tt == rid or (nrw and tt == ("RW" .. nrw .. "B")) then rws[#rws + 1] = l end
      end
      if next(sid.rwy or {}) == nil then rws[1] = {} end
      for _, r in ipairs(rws) do for _, e in ipairs(vals(sid.enrt)) do seqs[#seqs + 1] = cat(r, sid.common, e) end end
    end
  elseif kind == "STAR" then
    for _, sn in ipairs(skeys(P.star)) do
      local st = P.star[sn]
      for _, e in ipairs(vals(st.enrt)) do for _, r in ipairs(vals(st.rwy)) do seqs[#seqs + 1] = cat(e, st.common, r) end end
    end
  else
    for _, an in ipairs(skeys(P.app)) do
      local ap = P.app[an]
      for _, t in ipairs(vals(ap.trans)) do seqs[#seqs + 1] = cat(t, ap.final, ap.missed) end
    end
  end
  return seqs
end

local function insert_plegs(R)
  remove_plegs(R)
  if not (NDB and NDB.ready and NDB.procedures) then return end
  local G = B744GEO
  local wps = R.wps
  local org, org_i, dst, dst_i = nil, nil, nil, nil
  for i, w in ipairs(wps) do
    if not w.disco and w.ty == 1 then
      if not org then org, org_i = w, i end
      dst, dst_i = w, i
    end
  end
  if not org then return end
  -- route idents: first occurrence (departure side) and last occurrence (arrival side)
  local first_of, last_of = {}, {}
  for i, w in ipairs(wps) do
    if not w.disco and w.id and w.ty ~= 1 then
      first_of[w.id] = first_of[w.id] or i
      last_of[w.id] = i
    end
  end
  local adds = {}
  R.proc_ann = {}
  -- procedure legs for the AFDS (x.proc): per route fix of the matched procedure, the legs X-Plane
  -- does not keep before it (climb / heading / DME / intercept legs) and the fix's own leg type
  -- (CF course, AF DME arc, RF radius, DF direct) - so LNAV and the ND fly the procedure as charted
  local DYN = { CA = 1, VA = 1, FA = 1, CI = 1, VI = 1, VM = 1, FM = 1, CD = 1, VD = 1, FD = 1, CR = 1, VR = 1, FC = 1 }
  local OWN = { IF = 1, TF = 1, CF = 1, DF = 1, AF = 1, RF = 1 }
  local function navobj(ident, region, sec, icao, ref)
    if not ident or ident == "" or not NDB.resolve then return nil end
    local ok, o = pcall(NDB.resolve, ident, region, sec, icao, ref and ref.lat or 0, ref and ref.lon or 0)
    if not ok or not o then return nil end
    return { id = o.ident, lat = o.lat, lon = o.lon, kind = o.kind, sv = o.slaved_var }
  end
  local function ser(l, icao, ref)
    local o = { t = l.path, crs = l.crs, ti = l.crs_is_true or nil, turn = (l.turn ~= "" and l.turn) or nil,
      rho = l.rho, theta = l.theta, dist = l.dist, arc_r = l.arc_r, fo = l.flyover or nil }
    local a = l.alt1
    if a and a > 0 then o.alt = { d = (l.alt_desc ~= "" and l.alt_desc) or "@", a = a, b = l.alt2 } end
    o.nav = navobj(l.nav_ident, l.nav_region, l.nav_sec, icao, ref)
    o.ctr = navobj(l.ctr_ident, l.ctr_region, l.ctr_sec, icao, ref)
    if l.ident and DYN[l.path] then
      local f = navobj(l.ident, l.fix_region, l.fix_sec, icao, ref)
      if f then o.fix = f end
    end
    return o
  end
  local function annotate(best, idx, icao, rwy_info, start_i)
    local pending, prev, first = {}, 0, true
    for _, l in ipairs(best) do
      local i = l.ident and idx[l.ident]
      if DYN[l.path] then
        local ref = wps[prev] or org
        -- (v52: a generated leg the pilot removed is not flown / drawn either)
        local anchor = (prev > 0) and wps[prev] or (start_i and wps[start_i]) or nil
        if not prm_removed(R, anchor, l) then pending[#pending + 1] = ser(l, icao, ref) end
      elseif OWN[l.path] and i and i == prev and R.proc_ann[i] and l.path == "IF" then
        -- the same fix again where two parts of the procedure join (the common part or the transition
        -- starts with an "IF" at the fix just reached): one waypoint - its leg is the one flown to it
        local a = R.proc_ann[i]
        if a.leg and not a.leg.alt and l.alt1 and l.alt1 > 0 then
          a.leg.alt = { d = (l.alt_desc ~= "" and l.alt_desc) or "@", a = l.alt1, b = l.alt2 }
        end
        pending = {}
      elseif OWN[l.path] and i and i >= prev then
        local a = { pre = (#pending > 0) and pending or nil, leg = ser(l, icao, wps[i]) }
        if first and rwy_info then a.rwy = rwy_info end
        R.proc_ann[i] = a
        pending, prev, first = {}, i, false
      else
        pending = {}
        if l.ident and not i then first = false end
      end
    end
    -- legs after the last route fix of the procedure (a SID ending "climb to 3000 ft, then vectors",
    -- a STAR ending in vectors to final, a vectors-only SID such as SY4: 155 deg to 500 ft, vectors):
    -- flown after that fix - or after the departure runway when the procedure has no fix in the route
    if #pending > 0 then
      local at = (prev > 0) and prev or start_i
      if at and wps[at] then
        local a = R.proc_ann[at]
        if not a then
          a = { leg = { t = (prev > 0) and "TF" or "IF" } }
          R.proc_ann[at] = a
        end
        a.post = pending
        if prev == 0 and rwy_info then a.rwy = rwy_info end
      end
    end
  end
  local function plan(P, kind, rid, idx, elev, start_i)
    local best, bs = nil, 0
    for _, L in ipairs(sequences(P, kind, rid)) do
      -- (fixes in the route in order count; procedure fixes missing from the route count against:
      -- picks e.g. the ILS rather than the RNAV approach when the route has the ILS missed approach)
      local sc, last = 0, 0
      for _, l in ipairs(L) do
        local i = l.ident and idx[l.ident]
        if i and i >= last then sc = sc + 1; last = i
        elseif l.ident and not i then sc = sc - 0.5 end
      end
      if sc > bs then best, bs = L, sc end
    end
    if not best then return end
    local rwy_info = nil
    if kind == "SID" and rid and P.rwys and P.rwys[rid] then
      local r = P.rwys[rid]
      rwy_info = { id = rid, lat = r.lat, lon = r.lon, hdg = r.hdg_true, len = r.length_ft, elev = r.elev }
    end
    local oka, ea = pcall(annotate, best, idx, P.icao, rwy_info, (kind == "SID") and start_i or nil)
    if not oka then print("B744 LEGS: ERROR procedure annotation " .. tostring(ea)) end
    local prev_i = start_i
    local prev_pt = start_i and wps[start_i]
    local ref = prev_pt or org
    -- (a generated leg needs the leg before it in the procedure to be in the route: the fix it
    -- starts from, or the departure runway for a SID's first leg)
    local linked = start_i ~= nil
    for _, l in ipairs(best) do
      local i = l.ident and idx[l.ident]
      if l.ident and not ALT_LEG[l.path] then linked = false end
      if i and (not prev_i or i >= prev_i) then prev_i = i; prev_pt = wps[i]; linked = true end
      local COND = { CI = "INTC", VI = "INTC", VM = "VECTORS", FM = "VECTORS", CD = "D", VD = "D", FD = "D", CR = "R", VR = "R", FC = "FC" }
      local removed = prev_i and (COND[l.path] or ALT_LEG[l.path]) and prm_removed(R, wps[prev_i], l)
      if COND[l.path] and l.crs and prev_i and prev_pt and linked and not ALT_LEG[l.path] and not removed then
        -- other conditional legs (display only, as on the aircraft): heading / course to intercept
        -- "(INTC)", vectors "(VECTORS)", to a DME distance "(TTE9)", to a radial "(TTE017)"
        local k = COND[l.path]
        local id
        if k == "D" then id = string.format("(%s%d)", (l.nav_ident ~= "" and l.nav_ident) or "DME", math.floor((l.dist or 0) + 0.5))
        elseif k == "R" then id = string.format("(%s%03d)", l.nav_ident or "", math.floor((l.theta or 0) + 0.5))
        elseif k == "FC" then id = string.format("(%d)", math.floor((l.dist or 0) + 0.5))
        else id = "(" .. k .. ")" end
        local mag = l.crs_is_true and G.true_to_mag(l.crs, ref.lat, ref.lon) or l.crs
        local tru = l.crs_is_true and l.crs or G.mag_to_true(l.crs, ref.lat, ref.lon)
        local d = (k == "D" or k == "FC") and math.max(0.5, math.min(20, l.dist or 3)) or 3
        local la, lo = G.dest(prev_pt.lat, prev_pt.lon, tru, d)
        local w = { id = id, lat = la, lon = lo, ty = 2048, pleg = true, virt = true, cond = true,
          ic = math.floor(mag + 0.5) % 360, hdg = (l.path:sub(1, 1) == "V") or nil, psig = psig(l) }
        adds[#adds + 1] = { after = prev_i, w = w, seq = #adds + 1 }
        prev_pt = w
      end
      if ALT_LEG[l.path] and l.crs and l.alt1 and l.alt1 > 0 and prev_i and prev_pt and linked and not removed then
        -- (X-Plane kept it after all, e.g. "(600)": nothing to add)
        local nx = wps[prev_i + 1]
        local have = nx and not nx.disco and nx.id and tostring(nx.id):match("^%(%d+%)$")
        if not have then
          local mag = l.crs_is_true and G.true_to_mag(l.crs, ref.lat, ref.lon) or l.crs
          local tru = l.crs_is_true and l.crs or G.mag_to_true(l.crs, ref.lat, ref.lon)
          -- end of the climb (display only): about 500 ft per nm from the start of the leg
          local d = math.max(0.3, math.min(8, (l.alt1 - (elev or 0)) / 500))
          local la, lo = G.dest(prev_pt.lat, prev_pt.lon, tru, d)
          local w = { id = string.format("(%d)", math.floor(l.alt1 + 0.5)), lat = la, lon = lo, ty = 2048,
            alt = l.alt1, pleg = true, virt = true, ic = math.floor(mag + 0.5) % 360, pl_alt = l.alt1, tru = tru, psig = psig(l) }
          adds[#adds + 1] = { after = prev_i, w = w, seq = #adds + 1 }
          prev_pt = w
        end
      end
    end
  end
  -- departure
  local okp, P = pcall(NDB.procedures, org.id)
  if okp and P and P.sid then
    local rid = nil
    local okr, r = pcall(function() return getFMSData("runway") end)
    if okr and type(r) == "string" then
      r = r:gsub("%s", "")
      if r ~= "" and not r:find("%-") and not r:find("%*") then rid = r:find("^RW") and r or ("RW" .. r) end
    end
    local rw_i = nil
    for i, w in ipairs(wps) do if not w.disco and w.rw and i <= 3 then rw_i = i; rid = rid or w.id break end end
    local elev = (P.rwys and rid and P.rwys[rid] and P.rwys[rid].elev) or org.alt or 0
    plan(P, "SID", rid, first_of, tonumber(elev) or 0, rw_i or org_i)
  end
  -- arrival and approach (incl. the missed approach)
  if dst and dst_i and dst_i > org_i then
    local okd, D = pcall(NDB.procedures, dst.id)
    if okd and D then
      local ap = NDB.airport and NDB.airport(dst.id)
      local elev = (ap and ap.elev) or 0
      plan(D, "STAR", nil, last_of, elev, nil)
      plan(D, "APP", nil, last_of, elev, nil)
    end
  end
  -- insert (from the back so the indices stay valid); one generated leg per position
  table.sort(adds, function(x, y) if x.after ~= y.after then return x.after > y.after end return (x.seq or 0) > (y.seq or 0) end)
  local seen = {}
  local ins = {}
  for _, a in ipairs(adds) do
    local key = a.after .. a.w.id
    if not seen[key] then
      seen[key] = true
      table.insert(R.wps, a.after + 1, a.w)
      ins[#ins + 1] = a.after
      if (R.active or 0) > a.after then R.active = R.active + 1 end
    end
  end
  -- (procedure annotations follow their waypoints to the new positions)
  if #ins > 0 then
    local moved = {}
    for k, v in pairs(R.proc_ann) do
      local n = 0
      for _, af in ipairs(ins) do if af < k then n = n + 1 end end
      moved[k + n] = v
    end
    R.proc_ann = moved
  end
end
LG.insert_plegs = insert_plegs

-- fly-over waypoints of the origin's SIDs and the destination's STARs / approaches (nav database):
-- marked for the AFDS so the turn starts over the fix (e.g. DE19L, the departure end of 19L)
local flyover_cache = {}
local function proc_flyovers(R, o)
  if not (R and o and o.e and NDB and NDB.ready and NDB.procedures) then return end
  local org, dst = nil, nil
  for _, w in ipairs(R.wps) do if not w.disco and w.ty == 1 then org = org or w; dst = w end end
  local fo = {}
  for _, a in ipairs({ org, dst }) do
    if a and a.id then
      local set = flyover_cache[a.id]
      local ok, P
      if set == nil then
        set = {}
        ok, P = pcall(NDB.procedures, a.id)
        if ok and P then
          local function scan(list) for _, l in ipairs(list or {}) do if l.ident and l.flyover then set[l.ident] = true end end end
          for _, p in pairs(P.sid or {}) do
            for _, l in pairs(p.rwy or {}) do scan(l) end
            scan(p.common); for _, l in pairs(p.enrt or {}) do scan(l) end
          end
          for _, p in pairs(P.star or {}) do
            for _, l in pairs(p.rwy or {}) do scan(l) end
            scan(p.common); for _, l in pairs(p.enrt or {}) do scan(l) end
          end
          for _, p in pairs(P.app or {}) do
            for _, l in pairs(p.trans or {}) do scan(l) end
            scan(p.final)
          end
        end
        if ok and P then flyover_cache[a.id] = set end      -- (not cached until the database has it)
      end
      for k, _ in pairs(set) do fo[k] = true end
    end
  end
  for _, en in ipairs(o.e) do
    local x = en[3]
    if x and x.id and fo[x.id] then x.fo = true end
  end
end

-- generated altitude legs for the AFDS: climb on the leg's course to its altitude after the fix
-- before it (x.pca), then direct to the next fix (x.pdf) - e.g. a missed approach "334 to 500 ft,
-- then direct PY985". (The SID's first climb leg is given as x.dep by sid_departure.)
local function pleg_afds(R, o)
  if not (R and o and o.e) then return end
  local map, k = {}, 0
  for i, w in ipairs(R.wps) do
    if not w.virt and (w.xdc or not w.disco or w.xpd) then k = k + 1; map[i] = k end
  end
  for i, w in ipairs(R.wps) do
    if w.pleg and w.tru then
      local pi, ni = nil, nil
      for j = i - 1, 1, -1 do if map[j] then pi = map[j] break end end
      for j = i + 1, #R.wps do if map[j] then ni = map[j] break end end
      local px = pi and o.e[pi] and o.e[pi][3]
      local nxe = ni and o.e[ni] and o.e[ni][3]
      -- (v52: not when the procedure data already flies that climb leg after the fix - it was added a
      -- second time, after the vectors leg: AMB4 RW33 drew (VECTORS) before (900))
      local have = false
      if px and px.proc and px.proc.post then
        for _, q in ipairs(px.proc.post) do
          if (q.t == "CA" or q.t == "VA" or q.t == "FA") and q.alt and q.alt.a and math.abs(q.alt.a - (w.pl_alt or -1)) < 1 then have = true end
        end
      end
      if px and not have and not px.pca and not px.xd and not (nxe and (nxe.dep or nxe.proc)) and not px.dep then
        px.pca = { w.tru, w.pl_alt }
        if nxe and not nxe.xd and not nxe.db then nxe.pdf = true end
      end
    end
  end
end

LG.pleg_afds = pleg_afds
local function proc_attach(R, o)
  if not (R and R.proc_ann and o and o.e) then return end
  local k = 0
  for i, w in ipairs(R.wps) do
    if not w.virt and (w.xdc or not w.disco or w.xpd) then
      k = k + 1
      local a = R.proc_ann[i]
      local x = o.e[k] and o.e[k][3]
      if a and x and not x.xd and x.id == w.id then x.proc = a end
    end
  end
end
LG.proc_attach = proc_attach
-- (v52) the waypoint after generated legs the pilot removed: the AFDS flies to it from the end of the
-- leg before (direct, with the turn drawn) instead of the procedure's own leg type for it - e.g.
-- D326D, (900), GLENN: climb to 900 ft, then turn direct to GLENN; D326D, GLENN: D326D, then GLENN
local function prm_afds(R, o)
  if not (R and R.rm_hit and o and o.e) then return end
  local k, mark = 0, false
  for _, w in ipairs(R.wps) do
    if not w.virt and (w.xdc or not w.disco or w.xpd) then
      k = k + 1
      local x = o.e[k] and o.e[k][3]
      if mark and x and not x.xd then x.vdf = true; mark = false end
    end
    if not w.pleg and not w.disco and R.rm_hit[w] then mark = true end
  end
end
LG.prm_afds = prm_afds
local function publish(with_active)
  S.serial = S.serial + 1
  local o = LG.ext(S.act, S.serial, with_active)
  pcall(proc_attach, S.act, o)
  pcall(sid_departure, S.act, o)
  pcall(pleg_afds, S.act, o)
  pcall(proc_flyovers, S.act, o)
  pcall(prm_afds, S.act, o)
  o.ld = S.load_serial
  B747DR_legs_ext = json.encode(o)
end

-- keep the active route in step with the X-Plane FMS (RTE page edits, sequencing)
function B744_legs_sync()
  if not navdb_started then
    navdb_started = true
    NDB.start("")
  end
  if not NDB.ready then NDB.load_step(0.003) end
  if simDRTime - last_sync < 0.5 then return end
  last_sync = simDRTime
  if S.load_txt then
    -- (v52) the executed route goes to X-Plane once the AFDS has the new LEGS data
    if simDRTime >= (S.load_t or 0) then
      fmsLoadDref = S.load_txt
      S.load_txt = nil
      S.pending = simDRTime
    end
    return
  end
  LG.on_ground = (simDR_onGround == 1)
  local fms = decode_fms()
  if not fms or #fms < 1 then
    if S.act and #S.act.wps > 0 then S.act = { wps = {}, active = 0 }; publish(false) end
    return
  end
  if S.act == nil then S.act = LG.import(fms, nil, crz_alt(), imp_opt()); publish(false) end
  if S.pending then
    -- waiting for the X-Plane FMS to load the executed route
    if LG.matches(S.act, fms) then
      S.pending = nil
    elseif LG.matches(S.act, fms, true) then
      -- X-Plane put its discontinuity back where the pilot closed it: take its list (the entry is
      -- kept hidden) and keep the executed active waypoint
      S.pending = nil
      local aw = S.act.wps[S.act.active]
      local R = LG.import(fms, S.act, crz_alt(), imp_opt())
      if aw then
        for i, w in ipairs(R.wps) do
          if not w.disco and math.abs(w.lat - aw.lat) < 2e-3 and math.abs(w.lon - aw.lon) < 2e-3 then R.active = i break end
        end
      end
      R.proc_done = S.act.proc_done
      S.act = R
      publish(true)
    elseif simDRTime - S.pending > 10 then
      S.pending = nil
      S.act = LG.import(fms, S.act, crz_alt(), imp_opt())
      publish(false)
    else
      return
    end
  end
  if not LG.matches(S.act, fms) then
    S.act = LG.import(fms, S.act, crz_alt(), imp_opt())
    publish(false)
  end
  -- speed / altitude restrictions from the nav database procedures (once it is loaded)
  -- (re-applied whenever the waypoint list changes: a route loaded or executed after the first
  -- check - e.g. the SID / STAR added by X-Plane after the origin and destination - still gets its
  -- restrictions; waypoints the pilot has edited are never overwritten)
  if NDB.ready then
    local ids = {}
    for _, w in ipairs(S.act.wps) do if not w.pleg then ids[#ids + 1] = w.disco and "*" or tostring(w.id or "") end end
    local sig = table.concat(ids, ",") .. "|" .. tostring(getFMSData and getFMSData("runway") or "")
    if S.act.proc_sig ~= sig or not S.act.proc_done then
      local ok, e1 = pcall(LG.apply_procedures, S.act)
      if not ok then print("B744 LEGS: ERROR procedures " .. tostring(e1)) end
      local ok2, e2 = pcall(insert_plegs, S.act)
      if not ok2 then print("B744 LEGS: ERROR procedure legs " .. tostring(e2)) end
      S.act.proc_done = true
      S.act.proc_sig = sig
      publish(false)
    end
  end
  -- active waypoint: X-Plane active entry, or the discontinuity LNAV has reached
  local xa = 0
  for i, e in ipairs(fms) do if e[10] == true then xa = i end end
  local la = tonumber(B747DR_lnav_active) or 0
  local ei = 0
  if la > 0 and la ~= math.floor(la) then
    -- LNAV at the discontinuity before X-Plane entry ceil(la)
    local nx = LG.elem_of_xp(S.act, math.ceil(la))
    if nx > 1 and S.act.wps[nx - 1] and S.act.wps[nx - 1].disco then ei = nx - 1 end
  end
  if ei == 0 and xa > 0 then ei = LG.elem_of_xp(S.act, xa) end
  if ei > 0 and S.act.wps[ei] and S.act.wps[ei].xdc and S.act.wps[ei + 1] then ei = ei + 1 end
  -- active on the departure runway (X-Plane's first entry): the active waypoint is the first point
  -- after the runway (the runway is where the first leg starts)
  if ei > 0 and ei <= 3 then
    local w = S.act.wps[ei]
    if w and not w.disco and (w.rw or w.ty == 1) then
      local k = ei + 1
      while S.act.wps[k] and (S.act.wps[k].xdc or S.act.wps[k].rw or (not S.act.wps[k].disco and S.act.wps[k].ty == 1 and k < #S.act.wps)) do k = k + 1 end
      if S.act.wps[k] and not S.act.wps[k].disco and k < #S.act.wps then ei = k end
    end
  end
  -- generated altitude leg (e.g. (520)) before the active element: active until it is passed
  if ei > 1 then
    local pw = S.act.wps[ei - 1]
    local alt = tonumber(simDR_legs_alt) or 0
    if pw and pw.pleg then
      S.pl_done = S.pl_done or {}
      local key = string.format("%s%.4f%.4f", pw.id, pw.lat, pw.lon)
      if simDR_onGround ~= 1 and alt >= (pw.pl_alt or 0) - 50 then S.pl_done[key] = true end
      if not S.pl_done[key] then ei = ei - 1 end
    end
  end
  if simDR_onGround == 1 then S.pl_done = nil end
  if ei > 0 then S.act.active = ei end
end

local function legs_exec()
  if not LG.exec(S) then return false end
  local cyc = (NDB and NDB.cycle) or ""
  -- (v52) X-Plane's flight plan is only reloaded when its list changes: an edit of the generated legs
  -- only ((VECTORS), (900) ...) leaves it as it is. The load is sent a moment after the LEGS data
  -- (which tells the AFDS a load is coming - it then does not set X-Plane's active entry until the
  -- new list is in: an entry index of the old list on the new one crashed X-Plane).
  local fms = decode_fms()
  local same = false
  if fms and #fms > 0 then
    local ok, m = pcall(LG.matches, S.act, fms)
    same = ok and m
  end
  if not same then
    S.load_serial = (S.load_serial or 0) + 1
    S.load_txt = LG.fms_text(S.act, cyc)
    S.load_t = simDRTime + 1.0
    S.pending = simDRTime
  end
  publish(true)
  if B744_on_legs_exec then pcall(B744_on_legs_exec) end
  return true
end
function B744_legs_has_mod() return S.mod ~= nil end
function B744_legs_exec() return legs_exec() end

-- the modified route for the ND (light blue until EXEC)
local mod_geom_last, mod_geom_t = nil, -10
local function publish_mod_geom()
  if simDRTime - mod_geom_t < 0.5 then return end
  mod_geom_t = simDRTime
  local t = ""
  if S.mod then
    local parts = {}
    for _, w in ipairs(S.mod.wps) do
      if not w.disco and not w.xdc and w.lat and w.lon and not (w.lat == 0 and w.lon == 0) then
        parts[#parts + 1] = string.format("%s,%.6f,%.6f", (tostring(w.id or "WPT"):gsub("[,;]", "")), w.lat, w.lon)
      end
    end
    t = table.concat(parts, ";")
  end
  if t ~= mod_geom_last then mod_geom_last = t; B747DR_nd_mod_geom = t end
end

-- a new route as a MOD (EXEC loads it): e.g. the SimBrief route. list: { id, lat, lon, ty } with the
-- origin and destination airports first and last
function B744_legs_new_mod(list)
  local R = { wps = {}, active = 2, closed = {} }
  for _, e in ipairs(list) do
    R.wps[#R.wps + 1] = { id = e.id, lat = e.lat, lon = e.lon, ty = e.ty or 512, alt = e.alt or 0, kind = e.kind }
  end
  if #R.wps < 2 then return false end
  S.mod = R; S.dir = nil
  return true
end

function B744_legs_lights()
  pcall(publish_mod_geom)
  local m = (S.mod ~= nil) and 1 or 0
  B747DR_exec_light_pilot = math.max(m, tonumber(simDR_xp_exec_light_pilot) or 0)
  B747DR_exec_light_copilot = math.max(m, tonumber(simDR_xp_exec_light_copilot) or 0)
end

-- page ------------------------------------------------------------------------------------------
fmsPages["LEGS"] = createPage("LEGS")
local cache = {}
local legs_page_err_t = -100
fmsPages["LEGS"].getPage = function(self, pgNo, fmsID)
  local ok, L, Sm = pcall(function() return LG.render(S, pgNo, legs_ctx()) end)
  if not ok then
    if simDRTime - legs_page_err_t > 10 then legs_page_err_t = simDRTime print("B744 LEGS page error: " .. tostring(L)) end
    local b = "                        "
    L = { "     RTE 1 LEGS         ", b, b, b, b, b, "    LEGS PAGE ERROR     ", b, b, b, b, b, b }
    Sm = { b, b, b, b, b, b, b, b, b, b, b, b, b }
  end
  cache[fmsID] = Sm
  -- v42: PLAN mode map centre for the ND (the <CTR> waypoint)
  pcall(function()
    if S.ctr_lat and S.ctr_lon then B747DR_nd_plan_ctr_lat = S.ctr_lat; B747DR_nd_plan_ctr_lon = S.ctr_lon end
  end)
  return L
end
fmsPages["LEGS"].getSmallPage = function(self, pgNo, fmsID)
  return cache[fmsID] or self.templateSmall
end
fmsPages["LEGS"].getNumPages = function(self, fmsID)
  local R = S.mod or S.act
  if not R then return 1 end
  local ok, n = pcall(LG.num_pages, R)
  return (ok and n) or 1
end

function fmsFunctions.legs_lsk(fmsO, value)
  local side, row = string.sub(value, 1, 1), tonumber(string.sub(value, 2, 2))
  local sp = fmsO["scratchpad"] or ""
  local ok, nsp, msg = pcall(function()
    local ctx = legs_ctx()
    if side == "L" then return LG.lsk_left(S, row, fmsO["pgNo"] or 1, sp, ctx) end
    return LG.lsk_right(S, row, fmsO["pgNo"] or 1, sp, ctx)
  end)
  if not ok then print("B744 LEGS key error: " .. tostring(nsp)) nsp, msg = sp, "INVALID ENTRY" end
  if S.ctr_page then fmsO["targetpgNo"] = S.ctr_page; S.ctr_page = nil end
  fmsO["scratchpad"] = nsp or sp
  if msg then fmsO["notify"] = msg end
end
function fmsFunctions.legs_exec(fmsO, value)
  if not legs_exec() then
    -- nothing modified here: EXEC goes to the X-Plane FMS (RTE page edits)
    simCMD_FMS_key[fmsO.id]["exec"]:once()
  end
end

for _, k in ipairs({ "L1", "L2", "L3", "L4", "L5", "L6", "R1", "R2", "R3", "R4", "R5", "R6" }) do
  fmsFunctionsDefs["LEGS"][k] = { "legs_lsk", k }
end
fmsFunctionsDefs["LEGS"]["exec"] = { "legs_exec", "" }
