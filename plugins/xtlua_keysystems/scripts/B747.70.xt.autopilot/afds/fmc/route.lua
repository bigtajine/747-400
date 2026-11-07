--[[ B744 FMC route (flight plan) model.
     The leg list is authoritative (like the real FMC): RTE / DEP-ARR selections replace the
     legs of their own segment; LEGS page edits change the list directly.
     Each leg: t (ARINC path terminator), id (display), fix {lat,lon,...}, crs (mag), alt/spd
     constraints, seg ('RWY','SID','ENRT','STAR','APP','MISSED','DEST'), via (RTE page VIA).
     Pure Lua; registered as global B744ROUTE. ]]
local M = {}
local G = B744GEO
local N = B744NAVDB
local sformat, smatch, ssub, sfind, tonumber, floor, abs = string.format, string.match, string.sub, string.find, tonumber, math.floor, math.abs

local function copy_fix(o)
  if not o then return nil end
  return { ident = o.ident, lat = o.lat, lon = o.lon, region = o.region, kind = o.kind, apt = o.apt, freq = o.freq, elev = o.elev }
end
M.copy_fix = copy_fix

local function deep(o, seen)
  if type(o) ~= "table" then return o end
  seen = seen or {}
  if seen[o] then return seen[o] end
  local t = {}
  seen[o] = t
  for k, v in pairs(o) do t[k] = deep(v, seen) end
  return t
end
M.deepcopy = deep

function M.new()
  return {
    origin = nil, dest = nil, dep_rwy = nil, arr_rwy = nil,
    sid = nil, sid_trans = nil, star = nil, star_trans = nil, app = nil, app_trans = nil,
    flt_no = "", co_route = "",
    legs = {},
    active = 0,           -- index of active (TO) leg; 0 = none
    activated = false,
    serial = 0,           -- bumped on every change (path/vnav rebuild trigger)
    msg = nil,
  }
end

function M.copy(p)
  local c = deep(p)
  return c
end

local function touch(p) p.serial = (p.serial or 0) + 1 end
M.touch = touch

-- constraints ---------------------------------------------------------------------------------
local function conv_alt(desc, a1, a2)
  if a1 == nil then return nil end
  if desc == "+" or desc == "H" or desc == "J" or desc == "V" then return { d = "+", a = a1 } end
  if desc == "-" or desc == "Y" then return { d = "-", a = a1 } end
  if desc == "B" and a2 then return { d = "B", a = a1, b = a2 } end -- at/below a1, at/above a2
  return { d = "@", a = a1 }
end
local function conv_spd(desc, v)
  if not v then return nil end
  if desc == "+" then return { d = "+", v = v } end
  if desc == "-" then return { d = "-", v = v } end
  return { d = "@", v = v }
end

-- check constraint satisfied by alt
function M.alt_ok(c, alt, tol)
  if not c then return true end
  tol = tol or 0
  if c.d == "+" then return alt >= c.a - tol end
  if c.d == "-" then return alt <= c.a + tol end
  if c.d == "B" then return alt <= c.a + tol and alt >= c.b - tol end
  return abs(alt - c.a) <= tol + 1
end
function M.alt_hi(c) if not c then return nil end if c.d == "-" or c.d == "@" or c.d == "B" then return c.a end end
function M.alt_lo(c) if not c then return nil end if c.d == "+" or c.d == "@" then return c.a end if c.d == "B" then return c.b end end

-- procedure leg conversion ---------------------------------------------------------------------
local function pseudo_label(l)
  local t = l.t
  if t == "CA" or t == "FA" or t == "VA" or t == "HA" then
    return "(" .. (l.alt and floor(l.alt.a) or "ALT") .. ")"
  elseif t == "CI" or t == "VI" then return "(INTC)"
  elseif t == "VM" or t == "FM" then return "(VECTORS)"
  elseif t == "CD" or t == "VD" or t == "FD" then
    return "(" .. ((l.nav and l.nav.ident) or "DME") .. sformat("%d", floor((l.dist or 0) + 0.5)) .. ")"
  elseif t == "CR" or t == "VR" then
    return "(" .. ((l.nav and l.nav.ident) or "") .. sformat("%03d", floor((l.theta or 0) + 0.5)) .. ")"
  end
  return nil
end
M.pseudo_label = pseudo_label

function M.proc_leg(pl, P, seg, via, ref_lat, ref_lon)
  local l = {
    t = pl.path, seg = seg, via = via, turn = pl.turn ~= "" and pl.turn or nil,
    crs = pl.crs, crs_is_true = pl.crs_is_true, dist = pl.dist, time = pl.time, theta = pl.theta, rho = pl.rho,
    arc_r = pl.arc_r, fpa = pl.vangle, flyover = pl.flyover, faf = pl.is_faf, map = pl.is_map, iaf = pl.is_iaf,
    alt = conv_alt(pl.alt_desc, pl.alt1, pl.alt2), spd = conv_spd(pl.spd_desc, pl.spd), missed = pl.missed,
  }
  if l.t == "" then l.t = "TF" end
  local icao = P and P.icao
  if pl.ident then
    local o
    if pl.fix_sec == "PG" and P and P.rwys[pl.ident] then
      local r = P.rwys[pl.ident]
      o = { ident = pl.ident, lat = r.lat, lon = r.lon, kind = "RWY", elev = r.elev }
      l.rwy = true
    else
      o = N.resolve(pl.ident, pl.fix_region, pl.fix_sec, icao, ref_lat, ref_lon)
    end
    if o then l.fix = copy_fix(o) else l.fix = nil; l.unresolved = pl.ident end
  end
  if pl.nav_ident and pl.nav_ident ~= "" then
    local o
    if pl.nav_sec == "PI" and icao then o = N.loc_by_ident(icao, pl.nav_ident) end
    if not o then o = N.resolve(pl.nav_ident, pl.nav_region, pl.nav_sec, icao, ref_lat, ref_lon) end
    if o then l.nav = copy_fix(o) end
  end
  if pl.ctr_ident and pl.ctr_ident ~= "" then
    local o = N.resolve(pl.ctr_ident, pl.ctr_region, pl.ctr_sec, icao, ref_lat, ref_lon)
    if o then l.ctr = copy_fix(o) end
  end
  if (l.t == "HM" or l.t == "HA" or l.t == "HF") then
    l.hold = { inbd = pl.crs, turn = l.turn or "R", time = pl.time or (not pl.dist and 1.0) or nil, dist = pl.dist }
  end
  local dyn = { CA = 1, VA = 1, FA = 1, CI = 1, VI = 1, VM = 1, FM = 1, CD = 1, VD = 1, FD = 1, CR = 1, VR = 1, FC = 1 }
  if dyn[l.t] then
    l.id = pseudo_label(l) or (l.fix and l.fix.ident) or "-----"
    if l.t == "VM" or l.t == "VA" or l.t == "VI" or l.t == "VD" or l.t == "VR" or l.t == "CA" or l.t == "CI" or l.t == "CD" or l.t == "CR" then
      l.fix = nil
    end
  else
    l.id = (l.fix and l.fix.ident) or pseudo_label(l) or pl.ident or "-----"
  end
  if l.t == "IF" and not l.fix then l.t = "TF" end
  return l
end

local function append_proc(out, legs, P, seg, via, rlat, rlon)
  for _, pl in ipairs(legs or {}) do
    local l = M.proc_leg(pl, P, seg, via, rlat, rlon)
    if l.fix then rlat, rlon = l.fix.lat, l.fix.lon end
    out[#out + 1] = l
  end
  return rlat, rlon
end

-- leg factories ---------------------------------------------------------------------------------
function M.tf_leg(fix, seg, via)
  return { t = "TF", fix = copy_fix(fix), id = fix.ident, seg = seg or "ENRT", via = via or "DIRECT" }
end
function M.disco() return { t = "DISCO", id = "*****", seg = "DISCO" } end

local function same_fix(a, b)
  if not a or not b then return false end
  if a.ident ~= b.ident then return false end
  return G.dist(a.lat, a.lon, b.lat, b.lon) < 1.0
end
M.same_fix = same_fix

local function leg_end_fix(l)
  if l.t == "DISCO" then return nil end
  if l.t == "CA" or l.t == "VA" or l.t == "FA" or l.t == "VM" or l.t == "FM" or l.t == "CI" or l.t == "VI"
    or l.t == "CD" or l.t == "VD" or l.t == "FD" or l.t == "CR" or l.t == "VR" or l.t == "FC" then
    return nil
  end
  return l.fix
end
M.leg_end_fix = leg_end_fix

-- continuity: merge duplicate fixes, add / remove discontinuities ------------------------------------
function M.connect(p)
  local legs, out = p.legs, {}
  for i, l in ipairs(legs) do
    local prev = out[#out]
    if l.t == "DISCO" then
      if prev and prev.t ~= "DISCO" then out[#out + 1] = l end
    elseif prev == nil then
      out[#out + 1] = l
    else
      local pf = leg_end_fix(prev)
      if l.t == "IF" then
        if prev.t == "DISCO" then
          l.t = "TF"; out[#out + 1] = l
        elseif pf and same_fix(pf, l.fix) then
          -- merge: keep later procedure's constraints when present
          if l.alt then prev.alt = l.alt end
          if l.spd then prev.spd = l.spd end
          if l.faf then prev.faf = true end
          if l.iaf then prev.iaf = true end
          if l.seg ~= prev.seg and (l.seg == "STAR" or l.seg == "APP") then prev.seg_join = l.seg end
        elseif pf == nil and (prev.t == "VM" or prev.t == "FM") then
          out[#out + 1] = M.disco()      -- manual termination: always a discontinuity
          l.t = "TF"; out[#out + 1] = l
        elseif pf == nil and (prev.t == "CI" or prev.t == "VI") then
          l.t = "TF"; out[#out + 1] = l
        else
          out[#out + 1] = M.disco()
          l.t = "TF"
          out[#out + 1] = l
        end
      elseif (prev.t == "VM" or prev.t == "FM") and l.t ~= "DISCO" and l.seg ~= prev.seg then
        out[#out + 1] = M.disco()
        out[#out + 1] = l
      elseif (l.t == "TF" or l.t == "DF") and pf and same_fix(pf, l.fix) and l.seg ~= "HOLD" then
        -- duplicate consecutive fix
        if l.alt then prev.alt = l.alt end
        if l.spd then prev.spd = l.spd end
      else
        out[#out + 1] = l
      end
    end
  end
  while #out > 0 and out[#out].t == "DISCO" do out[#out] = nil end
  p.legs = out
  if p.active > #out then p.active = #out end
end

-- segment helpers ----------------------------------------------------------------------------------
local function remove_seg(p, set)
  local out = {}
  for _, l in ipairs(p.legs) do if not set[l.seg] then out[#out + 1] = l end end
  p.legs = out
end
local function first_index(p, set)
  for i, l in ipairs(p.legs) do if set[l.seg] then return i end end
  return nil
end
local function last_index(p, set)
  local r = nil
  for i, l in ipairs(p.legs) do if set[l.seg] then r = i end end
  return r
end
local function insert_at(p, idx, list)
  for k, l in ipairs(list) do table.insert(p.legs, idx + k - 1, l) end
end

-- DEPARTURE: runway + SID (+ transition) ---------------------------------------------------------
function M.build_departure(p)
  remove_seg(p, { RWY = true, SID = true })
  local out = {}
  if not p.origin then return end
  local P = N.procedures(p.origin.icao)
  local rlat, rlon = p.origin.lat, p.origin.lon
  if p.dep_rwy and P and P.rwys[p.dep_rwy] then
    local r = P.rwys[p.dep_rwy]
    out[#out + 1] = { t = "IF", id = p.dep_rwy, fix = { ident = p.dep_rwy, lat = r.lat, lon = r.lon, kind = "RWY", elev = r.elev },
      seg = "RWY", via = "", rwy = true, alt = nil }
    rlat, rlon = r.lat, r.lon
  end
  if p.sid and P and P.sid[p.sid] then
    local sid = P.sid[p.sid]
    local via = p.sid
    if p.dep_rwy then
      for t, legs in pairs(sid.rwy) do
        if N.rwy_match(t, p.dep_rwy) then rlat, rlon = append_proc(out, legs, P, "SID", via, rlat, rlon) break end
      end
    end
    if sid.common then rlat, rlon = append_proc(out, sid.common, P, "SID", via, rlat, rlon) end
    if p.sid_trans and sid.enrt[p.sid_trans] then
      rlat, rlon = append_proc(out, sid.enrt[p.sid_trans], P, "SID", via, rlat, rlon)
    end
  end
  insert_at(p, 1, out)
  M.connect(p)
  touch(p)
end

-- ARRIVAL: STAR (+trans, runway trans) then approach (+trans) + missed approach -----------------
function M.build_arrival(p)
  remove_seg(p, { STAR = true, APP = true, MISSED = true, DEST = true })
  if not p.dest then return end
  local P = N.procedures(p.dest.icao)
  local out = {}
  local rlat, rlon = p.dest.lat, p.dest.lon
  local rwy = p.arr_rwy
  if p.app and P and P.app[p.app] then rwy = P.app[p.app].rwy or rwy end
  if p.star and P and P.star[p.star] then
    local st = P.star[p.star]
    if p.star_trans and st.enrt[p.star_trans] then rlat, rlon = append_proc(out, st.enrt[p.star_trans], P, "STAR", p.star, rlat, rlon) end
    if st.common then rlat, rlon = append_proc(out, st.common, P, "STAR", p.star, rlat, rlon) end
    if rwy then
      for t, legs in pairs(st.rwy) do
        if N.rwy_match(t, rwy) then rlat, rlon = append_proc(out, legs, P, "STAR", p.star, rlat, rlon) break end
      end
    end
  end
  if p.app and P and P.app[p.app] then
    local ap = P.app[p.app]
    if p.app_trans and ap.trans[p.app_trans] then rlat, rlon = append_proc(out, ap.trans[p.app_trans], P, "APP", ap.display, rlat, rlon) end
    rlat, rlon = append_proc(out, ap.final, P, "APP", ap.display, rlat, rlon)
    -- final approach glide path angle propagated to legs after the FAF
    local fpa
    for _, l in ipairs(out) do
      if l.seg == "APP" and l.fpa then fpa = l.fpa end
    end
    if ap.missed and #ap.missed > 0 then
      rlat, rlon = append_proc(out, ap.missed, P, "MISSED", ap.display, rlat, rlon)
    end
  elseif rwy and P and P.rwys[rwy] then
    local r = P.rwys[rwy]
    out[#out + 1] = { t = "TF", id = rwy, fix = { ident = rwy, lat = r.lat, lon = r.lon, kind = "RWY", elev = r.elev },
      seg = "DEST", via = "", rwy = true, alt = { d = "@", a = floor(r.elev + 50) }, map = true }
  else
    local a = p.dest
    out[#out + 1] = { t = "TF", id = a.icao, fix = copy_fix(a), seg = "DEST", via = "", alt = nil, map = true }
  end
  -- STAR/approach joins with a discontinuity unless the route already ends at the first fix
  if #out > 0 and out[1].t ~= "IF" and out[1].fix then out[1].t = "IF" end
  p.legs[#p.legs + 1] = nil
  for _, l in ipairs(out) do p.legs[#p.legs + 1] = l end
  M.connect(p)
  touch(p)
end

-- the enroute segment list derived from legs (RTE page 2+) ----------------------------------------
function M.enroute_segments(p)
  local segs = {}
  local cur = nil
  for i, l in ipairs(p.legs) do
    if l.seg == "ENRT" then
      if l.t == "DISCO" then
        cur = nil
      elseif cur and cur.via == l.via and l.via ~= "DIRECT" then
        cur.to = l.fix; cur.last = i
      else
        cur = { via = l.via or "DIRECT", to = l.fix, first = i, last = i }
        segs[#segs + 1] = cur
      end
    else
      cur = nil
    end
  end
  return segs
end

-- index after which enroute legs go
local function enroute_anchor(p)
  local li = last_index(p, { RWY = true, SID = true })
  return li or 0
end

-- RTE page: add VIA airway/DIRECT TO fix at end of enroute.  returns errmsg on failure
function M.add_enroute(p, via, to_ident, ref_lat, ref_lon)
  local segs = M.enroute_segments(p)
  local start_fix
  local insert_after
  if #segs > 0 then
    start_fix = segs[#segs].to
    insert_after = segs[#segs].last
  else
    insert_after = enroute_anchor(p)
    local l = p.legs[insert_after]
    start_fix = l and leg_end_fix(l) or p.origin
  end
  local list = {}
  if via == "DIRECT" or via == nil then
    local o = N.find(to_ident, ref_lat or (start_fix and start_fix.lat), ref_lon or (start_fix and start_fix.lon))
    if not o then return "NOT IN DATA BASE" end
    list[1] = M.tf_leg(o, "ENRT", "DIRECT")
  else
    if not N.airway_exists(via) then return "NOT IN DATA BASE" end
    if not start_fix then return "INVALID ENTRY" end
    local path, err = N.airway_path(via, start_fix, to_ident)
    if not path then return err or "NOT ON AIRWAY" end
    for _, o in ipairs(path) do list[#list + 1] = M.tf_leg(o, "ENRT", via) end
  end
  insert_at(p, insert_after + 1, list)
  M.connect(p)
  touch(p)
  return nil
end

-- RTE page: change TO/VIA of an existing segment k (rebuild enroute from segment list) -----------
function M.rebuild_enroute(p, seglist)
  -- seglist: { {via=, to_ident=} ... }
  local keep_after = {}
  remove_seg(p, { ENRT = true })
  local anchor = enroute_anchor(p)
  local l = p.legs[anchor]
  local start_fix = l and leg_end_fix(l) or p.origin
  local list = {}
  for _, s in ipairs(seglist) do
    if s.via == "DIRECT" then
      local o = N.find(s.to_ident, start_fix and start_fix.lat, start_fix and start_fix.lon)
      if o then list[#list + 1] = M.tf_leg(o, "ENRT", "DIRECT"); start_fix = o end
    else
      local path = start_fix and N.airway_path(s.via, start_fix, s.to_ident)
      if path then
        for _, o in ipairs(path) do list[#list + 1] = M.tf_leg(o, "ENRT", s.via) end
        start_fix = path[#path]
      else
        list[#list + 1] = M.disco()
        local o = N.find(s.to_ident, start_fix and start_fix.lat, start_fix and start_fix.lon)
        if o then list[#list + 1] = M.tf_leg(o, "ENRT", "DIRECT"); start_fix = o end
      end
    end
  end
  insert_at(p, anchor + 1, list)
  M.connect(p)
  touch(p)
end

-- LEGS edits ------------------------------------------------------------------------------------------
-- region tag for a leg inserted before index idx
local function region_at(p, idx)
  local l = p.legs[idx] or p.legs[idx - 1]
  if not l then return "ENRT" end
  if l.seg == "DISCO" then
    local a = p.legs[idx - 1]
    return (a and a.seg ~= "RWY" and a.seg) or "ENRT"
  end
  if l.seg == "RWY" then return "SID" end
  return l.seg
end

-- find later occurrence of fix ident at or after index
function M.find_ident(p, ident, from)
  for i = from or 1, #p.legs do
    local l = p.legs[i]
    if l.id == ident or (l.fix and l.fix.ident == ident) then return i end
  end
  return nil
end

-- insert/connect a waypoint at LEGS line index idx (the line's current leg moves down)
-- returns nil or error
function M.legs_enter(p, idx, fix, is_active_line, ppos)
  local n = #p.legs
  if idx > n + 1 then idx = n + 1 end
  local later = nil
  for i = idx, n do
    local l = p.legs[i]
    if l.fix and l.fix.ident == fix.ident and G.dist(l.fix.lat, l.fix.lon, fix.lat, fix.lon) < 2 then later = i break end
  end
  if is_active_line then
    -- DIRECT TO
    if later then
      for _ = idx, later - 1 do table.remove(p.legs, idx) end
      local l = p.legs[idx]
      l.t = "DF"; l.dt_start = { lat = ppos.lat, lon = ppos.lon }; l.direct = true
      l.seg = (l.seg == "RWY" and "SID") or l.seg
    else
      local l = M.tf_leg(fix, region_at(p, idx), "DIRECT")
      l.t = "DF"; l.dt_start = { lat = ppos.lat, lon = ppos.lon }; l.direct = true
      table.insert(p.legs, idx, l)
      table.insert(p.legs, idx + 1, M.disco())
    end
    p.active = idx
  else
    if later then
      for _ = idx, later - 1 do table.remove(p.legs, idx) end
      local l = p.legs[idx]
      if l.t == "IF" or l.t == "CF" or l.t == "FA" then l.t = "TF" end
      if l.seg == "ENRT" then l.via = "DIRECT" end
    else
      local tgt = p.legs[idx]
      if tgt and tgt.t == "DISCO" then
        -- fill discontinuity
        local l = M.tf_leg(fix, region_at(p, idx), "DIRECT")
        p.legs[idx] = l
        table.insert(p.legs, idx + 1, M.disco())
      else
        local l = M.tf_leg(fix, region_at(p, idx), "DIRECT")
        table.insert(p.legs, idx, l)
        table.insert(p.legs, idx + 1, M.disco())
      end
      if p.active >= idx and p.active > 0 then p.active = p.active + 0 end
    end
  end
  M.connect(p)
  touch(p)
  return nil
end

function M.legs_delete(p, idx)
  local l = p.legs[idx]
  if not l then return "INVALID DELETE" end
  if idx == p.active and p.activated and p.inflight then return "INVALID DELETE" end
  if l.t == "DISCO" then
    table.remove(p.legs, idx)
    local nl = p.legs[idx]
    if nl and (nl.t == "IF") then nl.t = "TF" end
  else
    table.remove(p.legs, idx)
    local nl = p.legs[idx]
    if nl then
      if nl.t == "IF" or nl.t == "CF" then nl.t = "TF" end
      if nl.seg == "ENRT" then nl.via = "DIRECT" end
    end
  end
  if p.active > idx then p.active = p.active - 1 end
  M.connect(p)
  touch(p)
  return nil
end

-- intercept course to the fix of leg idx (direct-to with INTC CRS)
function M.legs_intercept(p, idx, crs_mag)
  local l = p.legs[idx]
  if not l or not l.fix then return "INVALID ENTRY" end
  l.t = "CF"; l.crs = crs_mag; l.dt_start = nil; l.direct = nil; l.intc = true
  touch(p)
end

-- constraints from scratchpad ("250/10000", "/FL240", "/12000A", "240/", "/15000A12000B")
function M.parse_constraint(s, ta)
  local sp, al = smatch(s, "^(%.?%d*)/(.*)$")
  if not sp then
    if smatch(s, "^%d+[AB]?$") or smatch(s, "^FL%d+[AB]?$") then sp, al = "", s else return nil end
  end
  local spd, alt
  if sp ~= "" then
    local v = tonumber(sp)
    if not v or v < 100 or v > 399 then return nil end
    spd = { d = "-", v = v }
  end
  if al ~= "" then
    local function pa(x)
      local fl = smatch(x, "^FL(%d+)$")
      if fl then return tonumber(fl) * 100 end
      local v = tonumber(x)
      if not v then return nil end
      if v < 1000 and #x <= 3 then v = v * 100 end
      return v
    end
    local a1, s1, a2, s2 = smatch(al, "^(F?L?%d+)([AB]?)(F?L?%d*)([AB]?)$")
    if not a1 then return nil end
    local v1 = pa(a1)
    if not v1 or v1 > 45100 then return nil end
    if a2 ~= "" then
      local v2 = pa(a2)
      if not v2 then return nil end
      if s1 == "A" and s2 == "B" then alt = { d = "B", a = v2, b = v1 }
      elseif s1 == "B" and s2 == "A" then alt = { d = "B", a = v1, b = v2 }
      else return nil end
    elseif s1 == "A" then alt = { d = "+", a = v1 }
    elseif s1 == "B" then alt = { d = "-", a = v1 }
    else alt = { d = "@", a = v1 } end
  end
  if not spd and not alt then return nil end
  return spd, alt
end

function M.legs_constraint(p, idx, spd, alt, delete)
  local l = p.legs[idx]
  if not l or l.t == "DISCO" then return "INVALID ENTRY" end
  if delete then
    l.alt = nil; l.spd = nil
  else
    if alt then l.alt = alt end
    if spd then
      if not alt and not l.alt then return "INVALID ENTRY" end
      l.spd = spd
    end
  end
  l.user_cnstr = true
  touch(p)
  return nil
end

-- holds ----------------------------------------------------------------------------------------------
function M.insert_hold(p, idx, fix, inbd_mag, turn)
  -- hold at waypoint of leg idx (or at a new fix inserted before idx)
  local l = {
    t = "HM", fix = copy_fix(fix), id = fix.ident, seg = "HOLD", via = "HOLD", user = true,
    hold = { inbd = inbd_mag, turn = turn or "R", time = nil, dist = nil },
  }
  table.insert(p.legs, idx + 1, l)
  touch(p)
  return idx + 1
end

-- pilot waypoint construction: PBD "SEA090/10", lat/lon "N4730.0W12218.5", "SEA/-10"? ---------------
local pilot_count = 0
function M.parse_waypoint(s, ref_lat, ref_lon, p)
  -- lat/lon N4730.0W12218.0 or N47W122
  local ns, la, lm, ew, lo, lom = smatch(s, "^([NS])(%d%d)(%d%d%.?%d*)([EW])(%d%d%d)(%d%d%.?%d*)$")
  if ns then
    local lat = tonumber(la) + tonumber(lm) / 60
    local lon = tonumber(lo) + tonumber(lom) / 60
    if ns == "S" then lat = -lat end
    if ew == "W" then lon = -lon end
    pilot_count = pilot_count + 1
    return { ident = sformat("%s%02d%s%03d", ns, floor(abs(lat)), ew, floor(abs(lon))):sub(1, 7), lat = lat, lon = lon, kind = "LL" }
  end
  ns, la, ew, lo = smatch(s, "^([NS])(%d%d)([EW])(%d%d%d)$")
  if ns then
    local lat, lon = tonumber(la), tonumber(lo)
    if ns == "S" then lat = -lat end
    if ew == "W" then lon = -lon end
    return { ident = s, lat = lat, lon = lon, kind = "LL" }
  end
  -- place/bearing/distance
  local base, brg, dist = smatch(s, "^(%w+)(%d%d%d)/(%d+%.?%d*)$")
  if base and #base >= 2 then
    local o = N.find(base, ref_lat, ref_lon)
    if not o then return nil, "NOT IN DATA BASE" end
    local tb = G.mag_to_true(tonumber(brg), o.lat, o.lon)
    local lat, lon = G.dest(o.lat, o.lon, tb, tonumber(dist))
    pilot_count = pilot_count + 1
    local nm = ssub(o.ident, 1, 3) .. sformat("%02d", pilot_count % 100)
    return { ident = nm, lat = lat, lon = lon, kind = "PBD" }
  end
  if smatch(s, "^%w+$") then
    local o = N.find(s, ref_lat, ref_lon)
    if not o then return nil, "NOT IN DATA BASE" end
    return o
  end
  return nil, "INVALID ENTRY"
end

-- find leg indices -----------------------------------------------------------------------------------
function M.last_route_index(p)
  -- last leg before the missed approach (runway / destination)
  local r = #p.legs
  for i, l in ipairs(p.legs) do if l.seg == "MISSED" then return i - 1 end end
  return r
end

B744ROUTE = M
