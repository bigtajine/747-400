--[[ B744 FMC navigation database.
     Reads X-Plane 12 nav data (Custom Data overrides Resources/default data):
       earth_aptmeta.dat (airports), earth_nav.dat (VOR/NDB/DME/ILS), earth_fix.dat (waypoints),
       earth_awy.dat (airways), CIFP/<ICAO>.dat (SID/STAR/approach, runways).
     Loading is incremental (call load_step every frame with a time budget) so the sim never stalls.
     Pure Lua, registered as global B744NAVDB. ]]
local M = {}
local G = B744GEO
local tonumber, tostring, sfind, ssub, smatch, sgmatch, sformat, supper =
  tonumber, tostring, string.find, string.sub, string.match, string.gmatch, string.format, string.upper
local floor, abs = math.floor, math.abs

M.ready = false
M.status = "NAV DATA NOT LOADED"
M.cycle = ""
M.base = ""          -- X-Plane root prefix ("" in xtlua)
local fixes = {}     -- ident -> "lat lon region apt;..."
local navs = {}      -- ident -> "type lat lon freq region apt elev magvar name;..."
local locs = {}      -- airport icao -> "rwy ident freq lat lon crsT crsM;..."
local gss = {}       -- loc ident..apt -> "angle lat lon elev"
local apts = {}      -- icao -> "lat lon elev ta tl region"
local awys = {}      -- airway name -> "f1 r1 t1 f2 r2 t2 dir|..."
local cifp_cache = {}
local loader = nil

local function exists(path)
  local f = io.open(path, "r")
  if f then f:close() return true end
  return false
end
local function datafile(name)
  local c = M.base .. "Custom Data/" .. name
  if exists(c) then return c end
  return M.base .. "Resources/default data/" .. name
end
M.datafile = datafile

local function add(tbl, key, rec)
  local cur = tbl[key]
  if cur == nil then tbl[key] = rec else tbl[key] = cur .. ";" .. rec end
end

-- loaders ------------------------------------------------------------------------------------
local function load_apts(yield)
  local f = io.open(datafile("earth_aptmeta.dat"), "r")
  if not f then return end
  local n = 0
  for line in f:lines() do
    local icao, reg, lat, lon, elev, rest = smatch(line, "^%s*(%S+)%s+(%S+)%s+(%S+)%s+(%S+)%s+(%S+)(.*)$")
    if icao and tonumber(lat) then
      local ta, tl = smatch(rest or "", "(%-?%d+)%s+(%S+)%s*$")
      apts[icao] = lat .. " " .. lon .. " " .. elev .. " " .. (ta or "-1") .. " " .. (tl or "-1") .. " " .. reg
    end
    n = n + 1
    if n % 500 == 0 then yield() end
  end
  f:close()
end

local function load_navs(yield)
  local f = io.open(datafile("earth_nav.dat"), "r")
  if not f then return end
  local n = 0
  for line in f:lines() do
    local t, lat, lon, elev, freq, rng, hdg, ident, term, reg, name =
      smatch(line, "^%s*(%d+)%s+(%S+)%s+(%S+)%s+(%S+)%s+(%S+)%s+(%S+)%s+(%S+)%s+(%S+)%s+(%S+)%s+(%S+)%s*(.*)$")
    if t then
      t = tonumber(t)
      if t == 2 or t == 3 or t == 13 then
        -- NDB / VOR / standalone DME
        local kind = (t == 2 and "NDB") or (t == 3 and "VOR") or "DME"
        add(navs, ident, kind .. " " .. lat .. " " .. lon .. " " .. freq .. " " .. reg .. " " .. term .. " " .. elev .. " " .. hdg .. " " .. (name or ""):gsub(";", ","))
      elseif t == 12 then
        -- DME co-located with VOR (VOR/DME, VORTAC) -> flag only, also allow DME-only lookups
        if not sfind(name or "", "ILS") and not sfind(name or "", "LOC") then
          local cur = navs[ident]
          if cur == nil or not sfind(cur, lat, 1, true) then
            add(navs, ident, "DME " .. lat .. " " .. lon .. " " .. freq .. " " .. reg .. " " .. term .. " " .. elev .. " 0 " .. (name or ""):gsub(";", ","))
          end
        end
      elseif t == 4 or t == 5 then
        local h = tonumber(hdg) or 0
        local mag = floor(h / 360)
        local tru = h - mag * 360
        local rwy = smatch(name or "", "^(%S+)") or ""
        add(locs, term, rwy .. " " .. ident .. " " .. freq .. " " .. lat .. " " .. lon .. " " .. sformat("%.3f", tru) .. " " .. mag .. " " .. elev)
      elseif t == 6 then
        local h = tonumber(hdg) or 0
        local ang = floor(h / 1000) / 100
        gss[ident .. term] = ang .. " " .. lat .. " " .. lon .. " " .. elev
      end
    end
    n = n + 1
    if n % 400 == 0 then yield() end
  end
  f:close()
end

local function load_fixes(yield)
  local f = io.open(datafile("earth_fix.dat"), "r")
  if not f then return end
  local n = 0
  for line in f:lines() do
    local lat, lon, ident, term, reg = smatch(line, "^%s*(%S+)%s+(%S+)%s+(%S+)%s+(%S+)%s+(%S+)")
    if reg and tonumber(lat) then
      add(fixes, ident, lat .. " " .. lon .. " " .. reg .. " " .. term)
    end
    n = n + 1
    if n % 800 == 0 then yield() end
  end
  f:close()
end

local function load_awys(yield)
  local f = io.open(datafile("earth_awy.dat"), "r")
  if not f then return end
  local n = 0
  for line in f:lines() do
    local f1, r1, t1, f2, r2, t2, dir, lvl, base, top, names =
      smatch(line, "^%s*(%S+)%s+(%S+)%s+(%d+)%s+(%S+)%s+(%S+)%s+(%d+)%s+(%S)%s+(%d)%s+(%d+)%s+(%d+)%s+(%S+)")
    if names then
      local rec = f1 .. " " .. r1 .. " " .. t1 .. " " .. f2 .. " " .. r2 .. " " .. t2 .. " " .. dir
      for nm in sgmatch(names, "[^%-]+") do
        local cur = awys[nm]
        if cur == nil then awys[nm] = rec else awys[nm] = cur .. "|" .. rec end
      end
    end
    n = n + 1
    if n % 600 == 0 then yield() end
  end
  f:close()
end

local function read_cycle()
  local f = io.open(M.base .. "Custom Data/cycle_info.txt", "r")
  if f then
    local txt = f:read("*a") or ""
    f:close()
    local c = smatch(txt, "AIRAC cycle%s*:%s*(%d+)")
    local v = smatch(txt, "Valid %(from/to%):%s*([^\n]+)")
    M.cycle = c or ""
    M.valid = v or ""
    return
  end
  local h = io.open(datafile("earth_fix.dat"), "r")
  if h then
    h:read("*l")
    local l2 = h:read("*l") or ""
    h:close()
    M.cycle = smatch(l2, "cycle%s+(%d+)") or ""
  end
end

function M.start(base)
  M.base = base or ""
  M.ready = false
  fixes, navs, locs, gss, apts, awys, cifp_cache = {}, {}, {}, {}, {}, {}, {}
  M.status = "NAV DATA LOADING"
  loader = coroutine.create(function()
    local y = coroutine.yield
    read_cycle()
    load_apts(y)
    load_navs(y)
    load_fixes(y)
    load_awys(y)
  end)
end

-- run the loader for at most `budget` seconds of CPU. returns true when complete
function M.load_step(budget)
  if M.ready then return true end
  if loader == nil then return false end
  local t0 = os.clock()
  while os.clock() - t0 < (budget or 0.004) do
    local ok, err = coroutine.resume(loader)
    if not ok then
      M.status = "NAV DATA ERROR"
      print("B744 NAVDB load error: " .. tostring(err))
      loader = nil
      M.ready = true
      return true
    end
    if coroutine.status(loader) == "dead" then
      loader = nil
      M.ready = true
      M.status = "NAV DATA LOADED"
      collectgarbage("collect")
      return true
    end
  end
  return false
end


-- lookups ------------------------------------------------------------------------------------
local function mkfix(ident, lat, lon, reg, term)
  return { ident = ident, lat = tonumber(lat), lon = tonumber(lon), region = reg, apt = term, kind = "WPT" }
end
local function mknav(ident, rec)
  local kind, lat, lon, freq, reg, term, elev, hdg, name =
    smatch(rec, "^(%S+) (%S+) (%S+) (%S+) (%S+) (%S+) (%S+) (%S+) ?(.*)$")
  if not kind then return nil end
  local o = { ident = ident, kind = kind, lat = tonumber(lat), lon = tonumber(lon), freq = tonumber(freq),
    region = reg, apt = term, elev = tonumber(elev), name = name }
  if kind == "VOR" then o.slaved_var = tonumber(hdg) end
  return o
end

function M.airport(icao)
  local r = apts[icao]
  if not r then return nil end
  local lat, lon, elev, ta, tl, reg = smatch(r, "^(%S+) (%S+) (%S+) (%S+) (%S+) (%S+)")
  local tln = tonumber(smatch(tl or "", "FL(%d+)") or "")
  return { ident = icao, icao = icao, kind = "APT", lat = tonumber(lat), lon = tonumber(lon), elev = tonumber(elev) or 0,
    ta = tonumber(ta) or -1, tl = tln and tln * 100 or -1, region = reg }
end

-- all database objects with this ident
function M.lookup(ident)
  local out = {}
  if ident == nil or ident == "" then return out end
  local r = fixes[ident]
  if r then
    for rec in sgmatch(r, "[^;]+") do
      local lat, lon, reg, term = smatch(rec, "^(%S+) (%S+) (%S+) (%S+)")
      out[#out + 1] = mkfix(ident, lat, lon, reg, term)
    end
  end
  r = navs[ident]
  if r then
    for rec in sgmatch(r, "[^;]+") do
      local o = mknav(ident, rec)
      if o then out[#out + 1] = o end
    end
  end
  local a = M.airport(ident)
  if a then out[#out + 1] = a end
  return out
end

function M.nearest(list, lat, lon)
  local best, bd = nil, 1e12
  for _, o in ipairs(list) do
    local d = (lat and G.dist(lat, lon, o.lat, o.lon)) or 0
    if d < bd then best, bd = o, d end
  end
  return best, bd
end

-- pilot entry: unique ident nearest to reference (terminal waypoints of other airports excluded)
function M.find(ident, lat, lon)
  local list = M.lookup(ident)
  local filt = {}
  for _, o in ipairs(list) do
    if o.kind ~= "WPT" or o.apt == "ENRT" or o.apt == nil then filt[#filt + 1] = o end
  end
  if #filt == 0 then filt = list end
  return (M.nearest(filt, lat, lon)), #filt
end

-- procedure fix resolution
function M.resolve(ident, region, sec, icao, nlat, nlon)
  local list = M.lookup(ident)
  local cands = {}
  local want
  if sec == "D" or sec == "DB" or sec == "PN" then want = "NAV" elseif sec == "EA" or sec == "PC" then want = "WPT" end
  for _, o in ipairs(list) do
    local ok = true
    if region and region ~= "" and o.region ~= region and o.kind ~= "APT" then ok = false end
    if want == "NAV" and o.kind == "WPT" then ok = false end
    if want == "WPT" and o.kind ~= "WPT" then ok = false end
    if sec == "PC" and o.apt ~= icao then ok = false end
    if sec == "EA" and o.apt ~= "ENRT" then ok = false end
    if ok then cands[#cands + 1] = o end
  end
  if #cands == 0 then
    for _, o in ipairs(list) do if o.kind ~= "APT" then cands[#cands + 1] = o end end
  end
  return (M.nearest(cands, nlat, nlon))
end

-- ILS/LOC for airport runway ("16C" or "RW16C")
function M.ils(icao, rwy)
  local r = locs[icao]
  if not r then return nil end
  rwy = (rwy or ""):gsub("^RW", "")
  for rec in sgmatch(r, "[^;]+") do
    local rw, ident, freq, lat, lon, crsT, crsM, elev = smatch(rec, "^(%S+) (%S+) (%S+) (%S+) (%S+) (%S+) (%S+) (%S+)")
    if rw == rwy then
      local o = { ident = ident, freq = tonumber(freq), lat = tonumber(lat), lon = tonumber(lon),
        crs_true = tonumber(crsT), crs_mag = tonumber(crsM), elev = tonumber(elev), rwy = rw, kind = "LOC" }
      local g = gss[ident .. icao]
      if g then
        local ang, glat, glon, gel = smatch(g, "^(%S+) (%S+) (%S+) (%S+)")
        o.gs_angle = tonumber(ang); o.gs_lat = tonumber(glat); o.gs_lon = tonumber(glon); o.gs_elev = tonumber(gel)
      end
      return o
    end
  end
  return nil
end
function M.loc_by_ident(icao, ident)
  local r = locs[icao]
  if not r then return nil end
  for rec in sgmatch(r, "[^;]+") do
    local rw, id = smatch(rec, "^(%S+) (%S+)")
    if id == ident then return M.ils(icao, rw) end
  end
end

-- airways ------------------------------------------------------------------------------------
-- returns list of fix objects after `from` up to and including `to` along airway `name`
function M.airway_path(name, from, to)
  local r = awys[name]
  if not r then return nil, "NOT IN DATA BASE" end
  local adj, pos = {}, {}
  local function key(f, rg) return f .. "/" .. rg end
  local function link(a, b)
    local l = adj[a]
    if not l then l = {}; adj[a] = l end
    l[#l + 1] = b
  end
  for rec in sgmatch(r, "[^|]+") do
    local f1, r1, t1, f2, r2, t2, dir = smatch(rec, "^(%S+) (%S+) (%S+) (%S+) (%S+) (%S+) (%S)")
    local k1, k2 = key(f1, r1), key(f2, r2)
    pos[k1] = { f1, r1, t1 }; pos[k2] = { f2, r2, t2 }
    if dir == "N" or dir == "F" then link(k1, k2) end
    if dir == "N" or dir == "B" then link(k2, k1) end
  end
  -- start nodes matching `from` ident (nearest region match if several)
  local starts = {}
  for k, p in pairs(pos) do
    if p[1] == from.ident and (from.region == nil or from.region == p[2] or from.kind == "APT") then starts[#starts + 1] = k end
  end
  if #starts == 0 then
    for k, p in pairs(pos) do if p[1] == from.ident then starts[#starts + 1] = k end end
  end
  if #starts == 0 then return nil, "NOT ON AIRWAY" end
  for _, s in ipairs(starts) do
    -- BFS
    local prev, q, qi = { [s] = false }, { s }, 1
    local goal = nil
    while qi <= #q do
      local c = q[qi]; qi = qi + 1
      if pos[c][1] == to and c ~= s then goal = c break end
      for _, nb in ipairs(adj[c] or {}) do
        if prev[nb] == nil then prev[nb] = c; q[#q + 1] = nb end
      end
    end
    if goal then
      local chain = {}
      local c = goal
      while c and c ~= s do table.insert(chain, 1, c); c = prev[c] end
      local out = {}
      local rlat, rlon = from.lat, from.lon
      for _, k in ipairs(chain) do
        local p = pos[k]
        local t = tonumber(p[3])
        local sec = (t == 11 and "EA") or (t == 2 and "DB") or "D"
        local o = M.resolve(p[1], p[2], sec, nil, rlat, rlon)
        if not o then return nil, "NOT IN DATA BASE" end
        out[#out + 1] = o
        rlat, rlon = o.lat, o.lon
      end
      return out
    end
  end
  return nil, "NOT ON AIRWAY"
end

function M.airway_exists(name) return awys[name] ~= nil end

-- CIFP procedures -----------------------------------------------------------------------------
local function dms(s, neg)
  -- N47274972 -> 47 27 49.72 ; W122183954 -> 122 18 39.54
  local h = ssub(s, 1, 1)
  local body = ssub(s, 2)
  local dl = (#body == 8) and 2 or 3
  local d = tonumber(ssub(body, 1, dl)) or 0
  local m = tonumber(ssub(body, dl + 1, dl + 2)) or 0
  local sec = (tonumber(ssub(body, dl + 3)) or 0) / 100
  local v = d + m / 60 + sec / 3600
  if h == "S" or h == "W" then v = -v end
  return v
end

local function num(s, scale)
  s = s and s:gsub("%s", "") or ""
  if s == "" then return nil end
  local v = tonumber(s)
  if v == nil then return nil end
  return v * (scale or 1)
end
local function alt_val(s)
  s = s and s:gsub("%s", "") or ""
  if s == "" then return nil end
  local fl = smatch(s, "^FL(%d+)")
  if fl then return tonumber(fl) * 100 end
  return tonumber(s)
end

local function split_csv(line)
  local t = {}
  for f in sgmatch(line .. ",", "([^,]*),") do t[#t + 1] = f end
  return t
end

local APP_PREFIX = { I = "ILS", L = "LOC", R = "RNV", P = "GPS", H = "RNV", D = "VDM", V = "VOR", S = "VOR",
  N = "NDB", Q = "NDB", X = "LDA", B = "BCR", J = "GLS", G = "IGS", U = "SDF", T = "TAC", F = "FMS" }

function M.app_display_name(id)
  local t = ssub(id, 1, 1)
  local rest = ssub(id, 2)
  local rw, sfx = smatch(rest, "^(%d%d[LRCB]?)%-?(.*)$")
  if not rw then return (APP_PREFIX[t] or t) .. rest end
  return (APP_PREFIX[t] or t) .. rw .. (sfx or "")
end
function M.app_runway(id)
  local rw = smatch(ssub(id, 2), "^(%d%d[LRC]?)")
  return rw and ("RW" .. rw) or nil
end

function M.procedures(icao)
  local c = cifp_cache[icao]
  if c ~= nil then return c or nil end
  local path = M.base .. "Custom Data/CIFP/" .. icao .. ".dat"
  local f = io.open(path, "r")
  if not f then
    f = io.open(M.base .. "Resources/default data/CIFP/" .. icao .. ".dat", "r")
  end
  if not f then cifp_cache[icao] = false return nil end
  local apt = M.airport(icao) or { lat = 0, lon = 0, elev = 0 }
  local P = { icao = icao, rwys = {}, rwy_list = {}, sid = {}, star = {}, app = {}, sid_list = {}, star_list = {}, app_list = {} }
  local raw = { SID = {}, STAR = {}, APPCH = {} }
  for line in f:lines() do
    local rt, rest = smatch(line, "^(%u+):(.*)$")
    if rt == "RWY" then
      local a, b = smatch(rest, "^([^;]*);([^;]*)")
      local fa = split_csv(a or "")
      local fb = split_csv(b or "")
      local id = fa[1]
      if id and fb[1] and fb[2] then
        local r = { ident = id, kind = "RWY", lat = dms(fb[1]), lon = dms(fb[2]), elev = num(fa[4]) or apt.elev,
          loc = (fa[6] or ""):gsub("%s", ""), dthr = num(fb[3]) or 0 }
        P.rwys[id] = r
        P.rwy_list[#P.rwy_list + 1] = id
      end
    elseif rt == "SID" or rt == "STAR" or rt == "APPCH" then
      raw[rt][#raw[rt] + 1] = rest:gsub(";%s*$", "")
    end
  end
  f:close()
  table.sort(P.rwy_list)
  -- runway headings from opposite thresholds
  for id, r in pairs(P.rwys) do
    local n, s = smatch(id, "^RW(%d%d)([LRC]?)")
    if n then
      local on = (tonumber(n) + 18 - 1) % 36 + 1
      local os = (s == "L" and "R") or (s == "R" and "L") or s
      local opp = P.rwys[sformat("RW%02d%s", on, os)]
      if opp then
        r.hdg_true = G.brg(r.lat, r.lon, opp.lat, opp.lon)
        r.length_ft = G.dist(r.lat, r.lon, opp.lat, opp.lon) * 6076.12
      else
        r.hdg_true = G.mag_to_true(tonumber(n) * 10, r.lat, r.lon)
        r.length_ft = 10000
      end
      r.hdg_mag = G.true_to_mag(r.hdg_true, r.lat, r.lon)
      r.num = tonumber(n)
      r.side = s
    end
  end

  local function mkleg(fl, kind, rtype)
    local fixid = (fl[5] or ""):gsub("%s", "")
    local leg = {
      path = (fl[12] or ""):gsub("%s", ""),
      ident = fixid ~= "" and fixid or nil,
      fix_region = (fl[6] or ""):gsub("%s", ""),
      fix_sec = ((fl[7] or "") .. (fl[8] or "")):gsub("%s", ""),
      desc = fl[9] or "    ",
      turn = (fl[10] or ""):gsub("%s", ""),
      rnp = num(fl[11], 0.1),
      tdv = (fl[13] or ""):gsub("%s", "") == "Y",
      nav_ident = (fl[14] or ""):gsub("%s", ""),
      nav_region = (fl[15] or ""):gsub("%s", ""),
      nav_sec = ((fl[16] or "") .. (fl[17] or "")):gsub("%s", ""),
      arc_r = num(fl[18], 0.001),
      theta = num(fl[19], 0.1),
      rho = num(fl[20], 0.1),
      crs = nil,
      alt_desc = (fl[23] or ""):gsub("%s", ""),
      alt1 = alt_val(fl[24]),
      alt2 = alt_val(fl[25]),
      trans_alt = alt_val(fl[26]),
      spd_desc = (fl[27] or ""):gsub("%s", ""),
      spd = num(fl[28]),
      vangle = num(fl[29], 0.01),
      ctr_ident = (fl[30] or ""):gsub("%s", ""),
      ctr_region = (fl[31] or ""):gsub("%s", ""),
      ctr_sec = ((fl[32] or "") .. (fl[33] or "")):gsub("%s", ""),
      kind = kind, rtype = rtype,
    }
    local c = (fl[21] or ""):gsub("%s", "")
    if c ~= "" then
      if smatch(c, "T$") then leg.crs = num(ssub(c, 1, -2), 0.1); leg.crs_is_true = true
      else leg.crs = num(c, 0.1) end
    end
    local d = (fl[22] or ""):gsub("%s", "")
    if smatch(d, "^T") then leg.time = num(ssub(d, 2), 0.1) elseif d ~= "" then leg.dist = num(d, 0.1) end
    if leg.ident == "" then leg.ident = nil end
    leg.flyover = ssub(leg.desc, 2, 2) == "Y" or ssub(leg.desc, 2, 2) == "B"
    leg.is_faf = ssub(leg.desc, 4, 4) == "F"
    leg.is_map = ssub(leg.desc, 4, 4) == "M"
    leg.is_iaf = ssub(leg.desc, 4, 4) == "A"
    if leg.spd_desc == "" and leg.spd then leg.spd_desc = "@" end
    return leg
  end

  local function group(list, kind)
    local procs, order = {}, {}
    for _, line in ipairs(list) do
      local fl = split_csv(line)
      local rtype = (fl[2] or ""):gsub("%s", "")
      local name = (fl[3] or ""):gsub("%s", "")
      local trans = (fl[4] or ""):gsub("%s", "")
      local p = procs[name]
      if not p then
        p = { name = name, kind = kind, parts = {}, part_order = {} }
        procs[name] = p; order[#order + 1] = name
      end
      local key = rtype .. "|" .. trans
      local part = p.parts[key]
      if not part then
        part = { rtype = rtype, trans = trans, legs = {} }
        p.parts[key] = part; p.part_order[#p.part_order + 1] = key
      end
      part.legs[#part.legs + 1] = mkleg(fl, kind, rtype)
    end
    return procs, order
  end

  local function rwy_match(trans, rwy)
    if trans == rwy or trans == "ALL" then return true end
    local n, s = smatch(trans, "^RW(%d%d)([LRCB]?)$")
    local rn, rs = smatch(rwy, "^RW(%d%d)([LRC]?)$")
    if not n or not rn then return false end
    if n ~= rn then return false end
    return s == "B" or s == "" or s == rs
  end
  M.rwy_match = rwy_match

  -- SIDs: rtypes 1/4/F/T = runway transitions, 2/5/M = common, 3/6/S/V = enroute transitions, 0 engine out
  local sids, sorder = group(raw.SID, "SID")
  for _, name in ipairs(sorder) do
    local p = sids[name]
    local sid = { name = name, rwy = {}, common = nil, enrt = {}, enrt_list = {}, rwys = {} }
    for _, key in ipairs(p.part_order) do
      local part = p.parts[key]
      local rt = part.rtype
      if rt == "1" or rt == "4" or rt == "F" or rt == "T" then
        sid.rwy[part.trans] = part.legs
      elseif rt == "2" or rt == "5" or rt == "M" then
        sid.common = part.legs
        if smatch(part.trans, "^RW") then sid.rwy[part.trans] = sid.rwy[part.trans] or {} ; sid.common_rwy = part.trans end
      elseif rt == "3" or rt == "6" or rt == "S" or rt == "V" then
        sid.enrt[part.trans] = part.legs
        sid.enrt_list[#sid.enrt_list + 1] = part.trans
      end
    end
    for _, rid in ipairs(P.rwy_list) do
      local ok = false
      for t, _ in pairs(sid.rwy) do if rwy_match(t, rid) then ok = true end end
      if next(sid.rwy) == nil and sid.common then ok = true end
      if ok then sid.rwys[#sid.rwys + 1] = rid end
    end
    table.sort(sid.enrt_list)
    P.sid[name] = sid
    P.sid_list[#P.sid_list + 1] = name
  end
  table.sort(P.sid_list)

  -- STARs: 1/4/S/7 enroute transitions, 2/5/M/8 common, 3/6/F/9 runway transitions
  local stars, torder = group(raw.STAR, "STAR")
  for _, name in ipairs(torder) do
    local p = stars[name]
    local st = { name = name, rwy = {}, common = nil, enrt = {}, enrt_list = {}, rwys = {} }
    for _, key in ipairs(p.part_order) do
      local part = p.parts[key]
      local rt = part.rtype
      if rt == "1" or rt == "4" or rt == "S" or rt == "7" then
        st.enrt[part.trans] = part.legs; st.enrt_list[#st.enrt_list + 1] = part.trans
      elseif rt == "2" or rt == "5" or rt == "M" or rt == "8" then
        st.common = part.legs
      elseif rt == "3" or rt == "6" or rt == "F" or rt == "9" then
        st.rwy[part.trans] = part.legs
      end
    end
    for _, rid in ipairs(P.rwy_list) do
      local ok = next(st.rwy) == nil
      for t, _ in pairs(st.rwy) do if rwy_match(t, rid) then ok = true end end
      if ok then st.rwys[#st.rwys + 1] = rid end
    end
    table.sort(st.enrt_list)
    P.star[name] = st
    P.star_list[#P.star_list + 1] = name
  end
  table.sort(P.star_list)

  -- approaches: A = transitions, anything else = final (+missed after MAP)
  local apps, aorder = group(raw.APPCH, "APP")
  for _, name in ipairs(aorder) do
    local p = apps[name]
    local ap = { name = name, display = M.app_display_name(name), trans = {}, trans_list = {}, final = nil,
      rwy = M.app_runway(name), type = ssub(name, 1, 1) }
    for _, key in ipairs(p.part_order) do
      local part = p.parts[key]
      if part.rtype == "A" then
        ap.trans[part.trans] = part.legs; ap.trans_list[#ap.trans_list + 1] = part.trans
      else
        ap.final = part.legs; ap.final_type = part.rtype
      end
    end
    if ap.final then
      -- split missed approach: legs after the MAP
      local fin, mis, seen = {}, {}, false
      for _, l in ipairs(ap.final) do
        if seen then mis[#mis + 1] = l; l.missed = true else fin[#fin + 1] = l end
        if l.is_map then seen = true end
      end
      ap.final, ap.missed = fin, mis
      table.sort(ap.trans_list)
      P.app[name] = ap
      P.app_list[#P.app_list + 1] = name
    end
  end
  -- order approaches like the CDU: by runway then type
  table.sort(P.app_list, function(a, b)
    local ra, rb = M.app_runway(a) or a, M.app_runway(b) or b
    if ra ~= rb then return ra < rb end
    return a < b
  end)
  cifp_cache[icao] = P
  return P
end


B744NAVDB = M
