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

-- check constraint satisfied by alt
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



-- leg factories ---------------------------------------------------------------------------------
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
local function first_index(p, set)
  for i, l in ipairs(p.legs) do if set[l.seg] then return i end end
  return nil
end

-- DEPARTURE: runway + SID (+ transition) ---------------------------------------------------------

-- ARRIVAL: STAR (+trans, runway trans) then approach (+trans) + missed approach -----------------

-- the enroute segment list derived from legs (RTE page 2+) ----------------------------------------

-- index after which enroute legs go

-- RTE page: add VIA airway/DIRECT TO fix at end of enroute.  returns errmsg on failure

-- RTE page: change TO/VIA of an existing segment k (rebuild enroute from segment list) -----------

-- LEGS edits ------------------------------------------------------------------------------------------
-- region tag for a leg inserted before index idx

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


-- intercept course to the fix of leg idx (direct-to with INTC CRS)

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


-- holds ----------------------------------------------------------------------------------------------

-- pilot waypoint construction: PBD "SEA090/10", lat/lon "N4730.0W12218.5", "SEA/-10"? ---------------
local pilot_count = 0

-- find leg indices -----------------------------------------------------------------------------------
function M.last_route_index(p)
  -- last leg before the missed approach (runway / destination)
  local r = #p.legs
  for i, l in ipairs(p.legs) do if l.seg == "MISSED" then return i - 1 end end
  return r
end

B744ROUTE = M
