--[[ B744 FMC - RTE 1 LEGS page (native, Boeing logic).
     The route is the X-Plane FMS list (built on the RTE / DEP ARR pages) plus the data the X-Plane
     FMS cannot hold: discontinuities, direct-to legs, intercept courses, pilot constraints.
     Editing works on a MOD copy (title MOD RTE 1 LEGS, EXEC lit, <ERASE); EXEC makes it active,
     reloads the X-Plane FMS (ND / map) and publishes the extra data for the AFDS (legs_ext).
       - LSK left, empty scratchpad: copies the waypoint to the scratchpad
       - waypoint at 1L in flight: DIRECT TO (route waypoint: legs before it deleted;
         new waypoint: inserted, followed by a ROUTE DISCONTINUITY)
       - waypoint at another line: route waypoint -> legs in between deleted (connect);
         new waypoint -> inserted at that line followed by a ROUTE DISCONTINUITY
       - DELETE: waypoint removed (route closes up), discontinuity removed; not the active waypoint
       - LSK right: speed/altitude constraint ("250/10000", "/FL240", "/12000A", "/13000A11000B")
       - 6R INTC CRS while a direct-to is pending: intercept that course to the fix
     Pure Lua; registered as global B744LEGS. ]]
local M = {}
local G = B744GEO
local floor, abs, max, min, ceil = math.floor, math.abs, math.max, math.min, math.ceil
local sformat, ssub, smatch, srep, supper = string.format, string.sub, string.match, string.rep, string.upper

M.ROWS = 5
M.SEND_DEPRWY = false
local DISCO_TXT = "-- ROUTE DISCONTINUITY -"
local BOXES = "*****"

local function navdb() return B744NAVDB end

local function is_rwy_ident(id)
  if type(id) ~= "string" then return false end
  return id:match("^RWY?%d%d?[LRC]?$") ~= nil
end
M.is_rwy_ident = is_rwy_ident
local function is_ll(id)
  return id == nil or id == "" or id == "latlon" or id:match("^[%+%-]?%d+%.%d+_") ~= nil
end

local function cp_tbl(t)
  if type(t) ~= "table" then return t end
  local o = {}
  for k, v in pairs(t) do o[k] = cp_tbl(v) end
  return o
end
function M.copy(R)
  local o = { wps = {}, active = R.active, closed = cp_tbl(R.closed), proc_done = R.proc_done }
  for i, w in ipairs(R.wps) do o.wps[i] = cp_tbl(w) end
  return o
end

-- X-Plane discontinuity entries the pilot has closed: keyed by the waypoint before them. X-Plane puts
-- its entry back whenever it rebuilds the list (e.g. after a runway without a SID); a closed one is
-- kept in the list (so it lines up with X-Plane's) but hidden and ignored (xdc).
local function pos_key(w) return sformat("%.3f,%.3f", w.lat or 0, w.lon or 0) end
M.pos_key = pos_key

local function same_pos(a, b, tol)
  tol = tol or 2e-3
  return abs(a.lat - b.lat) < tol and abs(a.lon - b.lon) < tol
end

-- X-Plane's own discontinuity entry (the X-Plane 12 FMS puts one where procedures do not join up):
-- unknown nav type, a DISCON name or no position. Shown and handled as a ROUTE DISCONTINUITY.
local function is_xp_disco(f)
  local id = type(f[8]) == "string" and supper(f[8]) or ""
  if (f[2] or 2048) == 0 then return true end
  if id:match("DISCON") or id:match("^%-*%s*DISC") then return true end
  if abs(f[5] or 0) < 1e-6 and abs(f[6] or 0) < 1e-6 then return true end
  return false
end
M.is_xp_disco = is_xp_disco
-- elements that are entries of the X-Plane list (waypoints and X-Plane's discontinuity entries)
-- (virt: the departure runway given to X-Plane as DEPRWY when X-Plane does not list it)
local function is_xp_entry(w) return (not w.virt) and ((not w.disco) or w.xpd) end
-- a discontinuity shown on the page (not a closed X-Plane entry)
local function is_disco(w) return w ~= nil and w.disco and not w.xdc end
M.is_disco = is_disco

-- IMPORT -------------------------------------------------------------------------------------------
-- X-Plane's names for unnamed positions ("(2400)", WPTnn)
local function o_unnamed(id) return id == nil or id:match("^%(") ~= nil or id:match("^WPT%d+$") ~= nil end
-- fms: decoded xtlua/fms entries {ref,type,freq,hdg,lat,lon,name,id,alt,active}
-- old: previous route; data the X-Plane FMS cannot hold is carried over to the same waypoints
M.on_ground = false     -- set by the page: on the ground the route is shown from the runway
function M.import(fms, old, crz, opt)
  local R = { wps = {}, active = 0, closed = cp_tbl(old and old.closed) or {} }
  local oldw = {}
  if old then
    local db = false
    for _, e in ipairs(old.wps) do
      if e.disco then db = db or not e.xdc else oldw[#oldw + 1] = { e = e, db = db }; db = false end
    end
  end
  local n = #fms
  local j = 1
  for i, f in ipairs(fms) do
    if is_xp_disco(f) then
      local d = { disco = true, xpd = true, lat = f[5] or 0, lon = f[6] or 0 }
      local pv = nil
      for k = #R.wps, 1, -1 do if not R.wps[k].disco then pv = R.wps[k] break end end
      -- (DEP: closed after the departure runway / origin airport, whichever X-Plane puts it after)
      if pv and (R.closed[pos_key(pv)] or (R.closed.DEP and (pv.rw or (#R.wps == 1 and pv.ty == 1)))) then d.xdc = true end
      R.wps[#R.wps + 1] = d
      if f[10] == true then R.active = #R.wps end
    else
    local w = { id = f[8], lat = f[5] or 0, lon = f[6] or 0, ty = f[2] or 2048, alt = f[9] or 0 }
    if is_ll(w.id) then w.id = nil end
    local found = nil
    for k = j, #oldw do
      if same_pos(oldw[k].e, w) then found = k break end
    end
    if found then
      local o = oldw[found].e
      j = found + 1
      if (not w.id) or o.pp then w.id = o.id end
      w.cn = cp_tbl(o.cn); w.cnd = o.cnd; w.der = o.der; w.spd = o.spd; w.df = cp_tbl(o.df); w.ic = o.ic; w.icd = o.icd
      w.pp = o.pp; w.rw = o.rw; w.rw_end = o.rw_end; w.kind = o.kind; w.user = o.user; w.dbc = o.dbc; w.auto_cnd = o.auto_cnd
      w.prm = cp_tbl(o.prm)
      if oldw[found].db and #R.wps > 0 and not R.wps[#R.wps].disco then R.wps[#R.wps + 1] = { disco = true } end
    elseif not w.cn then
      -- the X-Plane altitude of a descent waypoint is used as an at-or-above constraint by the AFDS
      local alt = w.alt
      if alt > 0 and w.ty ~= 1 and i > n / 2 and ((crz or 0) == 0 or alt < crz - 1000) then
        w.cn = { d = "+", a = floor(alt + 0.5) }; w.der = true
      end
    end
    -- departure runway without a procedure: X-Plane gives it as an unnamed position next to the
    -- origin airport - shown as the runway (name from the RTE page / nav database)
    if i == 2 and w.ty == 2048 and not is_rwy_ident(w.id) and fms[1] and fms[1][2] == 1
      and G.dist(fms[1][5] or 0, fms[1][6] or 0, w.lat, w.lon) < 4 then
      local nm = opt and opt.rwy
      if type(nm) == "string" then
        nm = supper(nm):gsub("%s", "")
        if nm:match("^%d%d?[LRC]?$") then nm = "RW" .. nm elseif nm:match("^RWY%d") then nm = "RW" .. ssub(nm, 4) end
      end
      if not (type(nm) == "string" and nm:match("^RW%d%d?[LRC]?$")) then nm = "RWY" end
      if (not w.id) or o_unnamed(w.id) then w.id = nm end
      w.rw = true
    end
    if not w.id then w.id = sformat("WPT%02d", i) end
    if w.rw == nil and is_rwy_ident(w.id) then w.rw = true end
    R.wps[#R.wps + 1] = w
    if f[10] == true then R.active = #R.wps end
    end
  end
  -- departure runway sent as DEPRWY that X-Plane does not list: kept on the LEGS page
  if old and R.wps[1] and R.wps[1].ty == 1 then
    local orw = nil
    for k = 1, min(3, #old.wps) do local e = old.wps[k] if e.rw and not e.disco then orw = e break end end
    if orw then
      local have = false
      -- (X-Plane lists a departure runway - the same one or a newly selected one: the old one is not kept)
      for k = 1, min(4, #R.wps) do local e = R.wps[k] if not e.disco and k > 1 and (e.rw or G.dist(e.lat, e.lon, orw.lat, orw.lon) < 1) then have = true end end
      if not have and G.dist(R.wps[1].lat, R.wps[1].lon, orw.lat, orw.lon) < 4 then
        local v = cp_tbl(orw); v.virt = true
        table.insert(R.wps, 2, v)
        if R.active >= 2 then R.active = R.active + 1 end
      end
    end
  end
  pcall(M.close_proc_discos, R)
  if R.wps[R.active] and R.wps[R.active].xdc and R.wps[R.active + 1] then R.active = R.active + 1 end
  return R
end

-- X-Plane puts its own discontinuity inside procedures where a Boeing FMC has none (e.g. after a
-- climb-to-altitude leg followed by a direct-to: SCOTT4 RW19L ... (600), DADAN). Such an entry is
-- closed (as if closed on the LEGS page) when the nav database shows the waypoints on both sides
-- are consecutive in one procedure (only unnamed legs such as CA / VA between them), or the
-- departure runway and the first fix of its SID. X-Plane's flight plan is not changed.
local function proc_sequences(P)
  local seqs = {}
  local function cat(...)
    local out = {}
    for _, l in ipairs({ ... }) do for _, x in ipairs(l or {}) do out[#out + 1] = x end end
    return out
  end
  for _, sid in pairs(P.sid or {}) do
    local enr = {}
    for _, l in pairs(sid.enrt or {}) do enr[#enr + 1] = l end
    if #enr == 0 then enr[1] = {} end
    local rws = {}
    for t, l in pairs(sid.rwy or {}) do rws[#rws + 1] = { t = t, l = l } end
    if #rws == 0 then rws[1] = { t = nil, l = {} } end
    for _, r in ipairs(rws) do
      for _, e in ipairs(enr) do seqs[#seqs + 1] = { rwy = r.t, legs = cat(r.l, sid.common, e) } end
    end
  end
  for _, st in pairs(P.star or {}) do
    local enr = {}
    for _, l in pairs(st.enrt or {}) do enr[#enr + 1] = l end
    if #enr == 0 then enr[1] = {} end
    local rws = {}
    for _, l in pairs(st.rwy or {}) do rws[#rws + 1] = l end
    if #rws == 0 then rws[1] = {} end
    for _, e in ipairs(enr) do for _, r in ipairs(rws) do seqs[#seqs + 1] = { legs = cat(e, st.common, r) } end end
  end
  for _, ap in pairs(P.app or {}) do
    local tr = {}
    for _, l in pairs(ap.trans or {}) do tr[#tr + 1] = l end
    if #tr == 0 then tr[1] = {} end
    for _, t in ipairs(tr) do seqs[#seqs + 1] = { legs = cat(t, ap.final) } end
  end
  return seqs
end

local function joined(seqs, pv, nx)
  local rid = pv.rw and pv.id and (pv.id:gsub("%s", "")) or nil
  local nrw = rid and rid:match("^RW(%d%d)")
  for _, sq in ipairs(seqs) do
    local L = sq.legs
    for k = 1, #L do
      if L[k].ident == nx.id then
        -- legs before nx back to the previous named leg
        local j, unnamed = k - 1, 0
        while j >= 1 and not L[j].ident do j = j - 1; unnamed = unnamed + 1 end
        if pv.rw then
          local t = sq.rwy and tostring(sq.rwy):gsub("%s", "")
          if j < 1 and t and (t == rid or (nrw and t == "RW" .. nrw .. "B")) then return true end
        elseif o_unnamed(pv.id) then
          if unnamed > 0 then return true end
        elseif j >= 1 and L[j].ident == pv.id then
          return true
        end
      end
    end
  end
  return false
end

function M.close_proc_discos(R)
  local N = navdb()
  if not (N and N.ready and N.procedures and R and #R.wps > 2) then return end
  local org, dst = nil, nil
  for _, w in ipairs(R.wps) do if not w.disco and w.ty == 1 then org = org or w; dst = w end end
  local seqs = {}
  for _, a in ipairs({ org, dst ~= org and dst or nil }) do
    if a and a.id then
      local ok, P = pcall(N.procedures, a.id)
      if ok and P then for _, sq in ipairs(proc_sequences(P)) do seqs[#seqs + 1] = sq end end
    end
  end
  if #seqs == 0 then return end
  for i, w in ipairs(R.wps) do
    if w.disco and w.xpd and not w.xdc then
      local pv, nx = nil, nil
      for k = i - 1, 1, -1 do if not R.wps[k].disco then pv = R.wps[k] break end end
      for k = i + 1, #R.wps do if not R.wps[k].disco then nx = R.wps[k] break end end
      if pv and nx and nx.id and joined(seqs, pv, nx) then
        w.xdc = true
        w.auto_xdc = true
      end
    end
  end
end

-- does the X-Plane list hold exactly the waypoints of route R (in order)?
function M.matches(R, fms, loose)
  local k = 0
  local closed = R.closed or {}
  local pv = nil
  -- (same position and, where both are named, the same name: parallel runways and fixes a few
  -- hundred metres apart - RW24L / RW24R, DLREY / FABRA - are different waypoints)
  local function at(f, w)
    if not (f and abs((f[5] or 0) - w.lat) <= 2e-3 and abs((f[6] or 0) - w.lon) <= 2e-3) then return false end
    local fid = f[8]
    if type(fid) == "string" and fid ~= "" and not is_ll(fid) and w.id and not w.pp and not o_unnamed(fid) and not o_unnamed(w.id)
      and (fid:gsub("%s", "")) ~= (tostring(w.id):gsub("%s", "")) then return false end
    return true
  end
  for i, w in ipairs(R.wps) do
    local skip = false
    -- loose: the departure runway sent as DEPRWY is not listed by X-Plane
    if loose and w.rw and i <= 3 and is_xp_entry(w) and not at(fms[k + 1], w) then skip = true end
    if is_xp_entry(w) and not skip then
      k = k + 1
      -- loose: X-Plane put its discontinuity back after a waypoint where the pilot closed it
      if loose and not w.xpd and fms[k] and is_xp_disco(fms[k]) and pv
        and (closed[pos_key(pv)] or (closed.DEP and (pv.rw or (pv.ty == 1 and k == 2)))) then k = k + 1 end
      local f = fms[k]
      if not at(f, w) then return false end
      if w.xpd and not is_xp_disco(f) then return false end
    end
    if not w.disco and not w.pleg then pv = w end   -- (v52: not a generated leg)
  end
  return k == #fms
end

-- PROCEDURE CONSTRAINTS -------------------------------------------------------------------------------
-- The X-Plane FMS list carries no speed/altitude restrictions. They are taken from the nav database
-- (CIFP) procedures of the origin (SID) and destination (STAR, approach): the procedure whose fixes
-- best match the route is used, and each route waypoint gets the restriction of its procedure leg.
local function conv_alt(desc, a1, a2)
  if not a1 or a1 <= 0 then return nil end
  if desc == "+" or desc == "H" or desc == "J" or desc == "V" then return { d = "+", a = a1 } end
  if desc == "-" or desc == "Y" then return { d = "-", a = a1 } end
  if desc == "B" and a2 and a2 > 0 then return { d = "B", a = max(a1, a2), b = min(a1, a2) } end
  if desc == "" or desc == "@" or desc == "X" or desc == "G" or desc == "I" then return { d = "@", a = a1 } end
  return { d = "@", a = a1 }
end
-- procedure leg sequences of an airport, as flown: SID = runway transition + common + en-route
-- transition; STAR = en-route transition + common + runway transition; approach = transition + final
local function seq_vals(t)
  local o = {}
  for k, v in pairs(t or {}) do o[#o + 1] = { k = k, v = v } end
  table.sort(o, function(a, b) return tostring(a.k) < tostring(b.k) end)
  if #o == 0 then o[1] = { k = nil, v = {} } end
  return o
end
local function seq_cat(...)
  local out = {}
  for _, l in ipairs({ ... }) do for _, x in ipairs(l or {}) do out[#out + 1] = x end end
  return out
end
function M.proc_sequences(P, kind, rid)
  local seqs = {}
  if kind == "SID" then
    local nrw = rid and rid:match("^RW(%d%d)")
    for _, s in ipairs(seq_vals(P.sid)) do
      local sid = s.v
      for _, r in ipairs(seq_vals(sid.rwy)) do
        local tt = r.k and tostring(r.k):gsub("%s", "")
        if (not rid) or (not tt) or tt == rid or (nrw and tt == ("RW" .. nrw .. "B")) then
          for _, e in ipairs(seq_vals(sid.enrt)) do seqs[#seqs + 1] = { name = s.k, legs = seq_cat(r.v, sid.common, e.v) } end
        end
      end
    end
  elseif kind == "STAR" then
    for _, s in ipairs(seq_vals(P.star)) do
      local st = s.v
      for _, e in ipairs(seq_vals(st.enrt)) do
        for _, r in ipairs(seq_vals(st.rwy)) do seqs[#seqs + 1] = { name = s.k, legs = seq_cat(e.v, st.common, r.v) } end
      end
    end
  else
    for _, s in ipairs(seq_vals(P.app)) do
      local ap = s.v
      for _, t in ipairs(seq_vals(ap.trans)) do seqs[#seqs + 1] = { name = s.k, legs = seq_cat(t.v, ap.final) } end
    end
  end
  return seqs
end
-- the procedure sequence that best matches route waypoints wps[a..b]: its fixes found in the route in
-- the same order score, its fixes missing from the route count against it (so a procedure sharing a
-- fix or two with the route is not taken for the one flown). Returns the sequence and, per route
-- index, the procedure leg ending at that waypoint.
function M.match_sequence(seqs, wps, a, b, from_end)
  local idx = {}
  for i = a, b do
    local w = wps[i]
    if w and not w.disco and w.id and not w.pleg then
      if from_end then idx[w.id] = i else idx[w.id] = idx[w.id] or i end
    end
  end
  local best, bs, bmap = nil, 0, nil
  for _, sq in ipairs(seqs) do
    local sc, last, map = 0, 0, {}
    for _, l in ipairs(sq.legs) do
      local i = l.ident and idx[l.ident]
      if i and i == last and map[i] then
        -- the same fix again where two parts of the procedure join (the last fix of the common part
        -- is the first, "IF", fix of the transition): one waypoint, keeping a restriction from either
        if (not map[i].alt1 or map[i].alt1 <= 0) and not (map[i].spd and map[i].spd > 0) then map[i] = l end
      elseif i and i >= last then
        sc = sc + 1; last = i; map[i] = l
      elseif l.ident and not i and l.path ~= "IF" then
        sc = sc - 0.5
      end
    end
    if sc > bs then best, bs, bmap = sq, sc, map end
  end
  if bs < 1 then return nil end
  return best, bmap
end
local function leg_cnstr(l)
  local c = conv_alt(l.alt_desc, l.alt1, l.alt2)
  -- (a Boeing FMC holds maximum speed restrictions: "at" and "at or below")
  local sp = (l.spd and l.spd > 0 and (l.spd_desc == "@" or l.spd_desc == "-" or l.spd_desc == "")) and l.spd or nil
  return c, sp
end

function M.apply_procedures(R)
  local N = navdb()
  if not (N and N.ready and R and #R.wps > 1) then return false end
  local wps = R.wps
  local first, last = nil, nil
  for i, w in ipairs(wps) do if not w.disco then first = first or i; last = i end end
  if not first then return false end
  local o, d = wps[first], wps[last]
  local function settable(w) return w and not w.disco and w.id and not w.user and not w.cnd end
  -- restrictions from an earlier procedure match (a runway / procedure changed since) are re-derived
  for _, w in ipairs(wps) do
    if w and not w.disco and w.auto_cnd and not w.user then w.cnd = nil; w.auto_cnd = nil end
    if w.dbc and settable(w) then w.cn = nil; w.spd = nil; w.dbc = nil end
  end
  local any = false
  local arr_first = last
  -- arrival + approach first (the approach restriction wins where both have the fix)
  if d.ty == 1 and d.id and last > first then
    local P = N.procedures(d.id)
    if P then
      local _, ma = M.match_sequence(M.proc_sequences(P, "APP"), wps, first + 1, last - 1, true)
      local _, ms = M.match_sequence(M.proc_sequences(P, "STAR"), wps, first + 1, last - 1, true)
      ma, ms = ma or {}, ms or {}
      local found = false
      for i = last - 1, first + 1, -1 do
        local w = wps[i]
        local la, lb = ma[i], ms[i]
        if settable(w) and (la or lb) then
          local ca, sa = nil, nil
          if la then ca, sa = leg_cnstr(la) end
          local cb, sb = nil, nil
          if lb then cb, sb = leg_cnstr(lb) end
          local cn, sp = ca or cb, sa or sb
          if cn then w.cn = cn; w.der = nil; w.dbc = true; found = true end
          if sp then w.spd = sp; w.dbc = true end
          arr_first = i
        end
      end
      if found then
        any = true
        -- the X-Plane altitudes of the other arrival waypoints are predictions, not restrictions
        for i = arr_first, last - 1 do
          local w = wps[i]
          if w and not w.disco and w.der then w.cn = nil; w.der = nil; w.cnd = true; w.auto_cnd = true end
        end
      end
    end
  end
  -- departure: SID fixes before the first arrival fix
  if o.ty == 1 and o.id and arr_first > first + 1 then
    local P = N.procedures(o.id)
    if P then
      local rid = nil
      for i = first, math.min(first + 3, last) do local w = wps[i] if w and not w.disco and w.rw and w.id then rid = (w.id:gsub("%s", "")) break end end
      local _, m = M.match_sequence(M.proc_sequences(P, "SID", rid), wps, first + 1, arr_first - 1, false)
      for i, l in pairs(m or {}) do
        local w = wps[i]
        if settable(w) and not w.rw then
          local cn, sp = leg_cnstr(l)
          if cn then w.cn = cn; w.der = nil; w.dbc = true; any = true end
          if sp then w.spd = sp; w.dbc = true end
        end
      end
    end
  end
  R.proc_done = true
  return any
end

-- ROUTE STRUCTURE ----------------------------------------------------------------------------------
-- first element shown on the LEGS page: the origin airport and PPOS are not shown; the departure
-- runway is (the first leg of the departure, as on the Boeing LEGS page)
function M.first_index(R)
  local n = #R.wps
  local k = 1
  while k <= n do
    local w = R.wps[k]
    -- (the departure runway is the "from" point of the first leg, not a LEGS line: the first line is
    -- the first point after the runway, as on the Boeing LEGS page; the runway is on the RTE page)
    local skip = w.xdc or (not w.disco and (w.pp or (k == 1 and w.ty == 1) or (w.rw and k < n)))
    if not skip then break end
    k = k + 1
  end
  return k
end

function M.vis(R)
  local s = M.first_index(R)
  if (R.active or 0) > s and not M.on_ground then s = R.active end
  local out = {}
  for i = s, #R.wps do if not R.wps[i].pp and not R.wps[i].xdc then out[#out + 1] = i end end
  return out
end

function M.num_pages(R)
  return max(1, ceil((#M.vis(R) + 1) / M.ROWS))
end

local function normalize(R)
  local out = {}
  local act = R.wps[R.active]
  for _, w in ipairs(R.wps) do
    local prev = out[#out]
    if w.xdc then
      -- closed X-Plane discontinuity: not part of the route
    elseif w.disco then
      if prev and not prev.disco and not (prev.ty == 1 and #out == 1) then out[#out + 1] = w end
    else
      out[#out + 1] = w
    end
  end
  while #out > 0 and out[#out].disco do out[#out] = nil end
  R.wps = out
  R.active = 0
  if act then for i, w in ipairs(out) do if w == act then R.active = i end end end
  if R.active == 0 and #out > 0 then R.active = min(M.first_index(R), #out) end
end
M.normalize = normalize

local function find_ident(R, ident, from)
  for i = from or 1, #R.wps do
    local w = R.wps[i]
    if not w.disco and not w.pp and w.id == ident then return i end
  end
  return nil
end

-- WAYPOINT ENTRY -----------------------------------------------------------------------------------
local pilot_n = 0
local function nav_ty(kind)
  if kind == "APT" then return 1 elseif kind == "NDB" then return 2 elseif kind == "VOR" then return 4
  elseif kind == "WPT" then return 512 end
  return 2048
end

-- (v55) name of a latitude/longitude waypoint (oceanic / Pacific crossings), the same on the LEGS
-- page, the ND and in X-Plane's flight plan (7 characters at most):
--   whole degrees           N55W040  (55N 40W)   S15W170 (15S 170W)
--   half degree latitude    H5520    (55.5N 20W - the North Atlantic track convention)
--   other positions         N52W020  (degrees only; the waypoint keeps its exact position)
function M.ll_name(lat, lon)
  local ns, ew = (lat >= 0) and "N" or "S", (lon >= 0) and "E" or "W"
  local a, o = abs(lat), abs(lon)
  local wl, wo = abs(a - floor(a + 0.5)) < 1e-4, abs(o - floor(o + 0.5)) < 1e-4
  if wl and wo then return sformat("%s%02d%s%03d", ns, floor(a + 0.5), ew, floor(o + 0.5)) end
  if wo and ns == "N" and ew == "W" and o < 100 and abs(a * 2 - floor(a * 2 + 0.5)) < 1e-4 then
    return sformat("H%02d%02d", floor(a), floor(o + 0.5))
  end
  return sformat("%s%02d%s%03d", ns, floor(a), ew, floor(o))
end

-- scratchpad text -> fix {id, lat, lon, ty, kind} or nil, message
function M.resolve(s, ref_lat, ref_lon)
  local N = navdb()
  -- latitude/longitude N4730.0W12218.5 / N47W122
  local ns, la, lm, ew, lo, lom = smatch(s, "^([NS])(%d%d)(%d%d%.?%d*)([EW])(%d%d%d)(%d%d%.?%d*)$")
  if ns then
    local lat = tonumber(la) + tonumber(lm) / 60
    local lon = tonumber(lo) + tonumber(lom) / 60
    if lat > 90 or lon > 180 then return nil, "INVALID ENTRY" end
    if ns == "S" then lat = -lat end
    if ew == "W" then lon = -lon end
    return { id = M.ll_name(lat, lon), lat = lat, lon = lon, ty = 2048, kind = "LL" }
  end
  ns, la, ew, lo = smatch(s, "^([NS])(%d%d)([EW])(%d%d%d)$")
  if ns then
    local lat, lon = tonumber(la), tonumber(lo)
    if lat > 90 or lon > 180 then return nil, "INVALID ENTRY" end
    if ns == "S" then lat = -lat end
    if ew == "W" then lon = -lon end
    return { id = s, lat = lat, lon = lon, ty = 2048, kind = "LL" }
  end
  -- place / bearing / distance  SEA090/10
  local base, brg, dist = smatch(s, "^(%u[%u%d]+)(%d%d%d)/(%d+%.?%d*)$")
  if base then
    if not (N and N.ready) then return nil, "NOT IN DATA BASE" end
    local o = N.find(base, ref_lat, ref_lon)
    if not o then return nil, "NOT IN DATA BASE" end
    local b, d = tonumber(brg), tonumber(dist)
    if b > 360 or d <= 0 or d > 700 then return nil, "INVALID ENTRY" end
    local lat, lon = G.dest(o.lat, o.lon, G.mag_to_true(b % 360, o.lat, o.lon), d)
    pilot_n = pilot_n % 99 + 1
    return { id = ssub(o.ident, 1, 3) .. sformat("%02d", pilot_n), lat = lat, lon = lon, ty = 2048, kind = "PBD" }
  end
  if smatch(s, "^[%u%d]+$") and #s <= 7 then
    if not (N and N.ready) then return nil, "NOT IN DATA BASE" end
    local o = N.find(s, ref_lat, ref_lon)
    if not o then return nil, "NOT IN DATA BASE" end
    return { id = o.ident, lat = o.lat, lon = o.lon, ty = nav_ty(o.kind), kind = o.kind }
  end
  return nil, "INVALID ENTRY"
end

local function new_wp(fix)
  return { id = fix.id, lat = fix.lat, lon = fix.lon, ty = fix.ty or 2048, kind = fix.kind, alt = 0, user = true }
end

local function ref_pos(R, idx, ctx)
  for k = (idx or #R.wps + 1) - 1, 1, -1 do
    local w = R.wps[k]
    if w and not w.disco then return w.lat, w.lon end
  end
  if ctx and ctx.ppos then return ctx.ppos.lat, ctx.ppos.lon end
  return nil, nil
end

-- DIRECT TO (entry at 1L in flight)
local function direct_to(S, R, later, fix, start, ctx)
  local o = (R.wps[1] and not R.wps[1].disco and R.wps[1].ty == 1) and 1 or 0
  local tgt
  if later then
    tgt = R.wps[later]
    for k = later - 1, o + 1, -1 do table.remove(R.wps, k) end
  else
    tgt = new_wp(fix)
    for k = start - 1, o + 1, -1 do table.remove(R.wps, k) end
    local nx = R.wps[o + 1]
    table.insert(R.wps, o + 1, tgt)
    if nx and not nx.disco then table.insert(R.wps, o + 2, { disco = true }) end
  end
  local pp = { id = "PPOS", lat = ctx.ppos.lat, lon = ctx.ppos.lon, ty = 2048, pp = true, alt = 0 }
  table.insert(R.wps, o + 1, pp)
  tgt.df = { lat = ctx.ppos.lat, lon = ctx.ppos.lon, trk = ctx.ppos.trk, gs = ctx.ppos.gs }
  tgt.ic = nil; tgt.icd = nil
  R.active = o + 2
  S.dir = tgt
end

-- (v52) a generated procedure leg ((VECTORS), (INTC), (900) ...) removed from the MOD route: noted on
-- the route waypoint it follows (the legs are generated again after that waypoint at every route
-- update; B744.fms.pages.legs.lua leaves the noted ones out, on the LEGS page and for the AFDS).
-- Only in the MOD: EXEC keeps it, ERASE discards it.
local function note_pleg_removed(Mr, k)
  local w = Mr.wps[k]
  if not (w and w.pleg and w.psig) then return end
  for j = k - 1, 1, -1 do
    local a = Mr.wps[j]
    if a and not a.pleg and not a.disco then
      a.prm = a.prm or {}
      a.prm[w.psig] = true
      return
    end
  end
end

-- left line select key: row 1..5 on page pg, or row 6 (ERASE / RTE 2 LEGS)
-- returns new scratchpad, message (nil = ok)
function M.lsk_left(S, row, pg, sp, ctx)
  if row == 6 then
    if S.mod then S.mod = nil; S.dir = nil return sp, nil end
    return sp, "KEY/FUNCTION INOP"
  end
  local R = S.mod or S.act
  if not R then return sp, "INVALID ENTRY" end
  local vis = M.vis(R)
  local vi = (pg - 1) * M.ROWS + row
  local idx = vis[vi]
  if idx == nil and vi > #vis + 1 then return sp, nil end
  if sp == "" then
    if not idx then return sp, nil end
    local w = R.wps[idx]
    if w.disco then return sp, nil end
    return w.id, nil
  end
  local Mr = S.mod or M.copy(S.act)
  if sp == "DELETE" then
    if not idx then return sp, "INVALID DELETE" end
    local w = Mr.wps[idx]
    if not w.disco and ctx.inflight and idx == Mr.active then return sp, "INVALID DELETE" end
    note_pleg_removed(Mr, idx)
    table.remove(Mr.wps, idx)
    if S.dir == w then S.dir = nil end
    normalize(Mr)
    S.mod = Mr
    return "", nil
  end
  local is_active_line = ctx.inflight and vi == 1 and ctx.ppos ~= nil
  local start = vis[1] or (#Mr.wps + 1)
  local later = find_ident(Mr, sp, idx or (#Mr.wps + 1))
  local fix = nil
  if not later then
    local rl, ro = ref_pos(Mr, idx, ctx)
    local msg
    fix, msg = M.resolve(sp, rl, ro)
    if not fix then
      -- a waypoint earlier in the route (or the nav database still loading): use the route's copy
      local up = find_ident(Mr, sp, 1)
      if not up then return sp, msg end
      local u = Mr.wps[up]
      fix = { id = u.id, lat = u.lat, lon = u.lon, ty = u.ty, kind = u.kind }
    end
  end
  if is_active_line then
    direct_to(S, Mr, later, fix, start, ctx)
  elseif later then
    if later == idx then return "", nil end
    local act = Mr.wps[Mr.active]
    for k = later - 1, idx, -1 do
      if Mr.wps[k] == act then act = Mr.wps[later] end
      note_pleg_removed(Mr, k)
      table.remove(Mr.wps, k)
    end
    local w = Mr.wps[idx]
    -- connected to the previous waypoint: no longer a direct-to / intercept from a stale position
    if w ~= act then w.df = nil; w.ic = nil end
    for i, e in ipairs(Mr.wps) do if e == act then Mr.active = i end end
  else
    local w = new_wp(fix)
    if idx == nil then
      Mr.wps[#Mr.wps + 1] = w
    else
      local act = Mr.wps[Mr.active]
      local nx = Mr.wps[idx]
      table.insert(Mr.wps, idx, w)
      if not nx.disco then table.insert(Mr.wps, idx + 1, { disco = true }) end
      for i, e in ipairs(Mr.wps) do if e == act then Mr.active = i end end
    end
  end
  normalize(Mr)
  S.mod = Mr
  return "", nil
end

-- CONSTRAINTS --------------------------------------------------------------------------------------
function M.parse_constraint(s)
  local sp, al = smatch(s, "^(%d*)/(.*)$")
  if not sp then
    if smatch(s, "^%d+[AB]?$") or smatch(s, "^FL%d+[AB]?$") then sp, al = "", s else return nil end
  end
  local spd, alt
  if sp ~= "" then
    local v = tonumber(sp)
    if not v or v < 100 or v > 399 then return nil end
    spd = v
  end
  if al ~= "" then
    local function pa(x)
      local fl = smatch(x, "^FL(%d+)$")
      if fl then return tonumber(fl) * 100 end
      local v = tonumber(x)
      if not v then return nil end
      if #x <= 3 then v = v * 100 end
      return v
    end
    local a1, s1, a2, s2 = smatch(al, "^(F?L?%d+)([AB]?)(F?L?%d*)([AB]?)$")
    if not a1 then return nil end
    local v1 = pa(a1)
    if not v1 or v1 > 45100 or v1 < -1000 then return nil end
    if a2 ~= "" then
      local v2 = pa(a2)
      if not v2 then return nil end
      if s1 == "A" and s2 == "B" and v2 >= v1 then alt = { d = "B", a = v2, b = v1 }
      elseif s1 == "B" and s2 == "A" and v1 >= v2 then alt = { d = "B", a = v1, b = v2 }
      else return nil end
    elseif s1 == "A" then alt = { d = "+", a = v1 }
    elseif s1 == "B" then alt = { d = "-", a = v1 }
    else alt = { d = "@", a = v1 } end
  end
  if not spd and not alt then return nil end
  return spd, alt
end

local function fmt_alt(a, ta)
  if a >= (ta or 18000) then return sformat("FL%03d", floor(a / 100 + 0.5)) end
  return sformat("%d", floor(a + 0.5))
end
local function fmt_cn(c, ta)
  if not c then return "" end
  if c.d == "+" then return fmt_alt(c.a, ta) .. "A" end
  if c.d == "-" then return fmt_alt(c.a, ta) .. "B" end
  if c.d == "B" then return fmt_alt(c.b, ta) .. "A" .. fmt_alt(c.a, ta) .. "B" end
  return fmt_alt(c.a, ta)
end
M.fmt_cn = fmt_cn

-- predicted speed: CAS in kt, or Mach (sent as Mach * 1000)
local function fmt_spd(v)
  if v >= 400 then return sformat(".%03d", floor(v + 0.5)) end
  return sformat("%d", floor(v + 0.5))
end
-- the AFDS prediction for a LEGS waypoint (same position; a generated climb leg "(700)" is matched
-- to the nearest prediction close by)
local function pred_at(list, w)
  if not list or #list == 0 or not w or not w.lat then return nil end
  local best, bd = nil, w.pleg and 1.5 or 0.05
  for _, q in ipairs(list) do
    local d = G.dist(q.lat, q.lon, w.lat, w.lon)
    if d < bd then best, bd = q, d end
  end
  return best
end
local function intc_crs_default(S, ctx)
  local t = S.dir
  if not t or not ctx.ppos then return nil end
  local b = G.brg(ctx.ppos.lat, ctx.ppos.lon, t.lat, t.lon)
  return floor(G.true_to_mag(b, ctx.ppos.lat, ctx.ppos.lon) + 0.5) % 360
end

function M.lsk_right(S, row, pg, sp, ctx)
  if row == 6 and ctx.plan and not S.mod then
    -- STEP: centre on the next waypoint (wraps to the first), the page follows it
    local R = S.act
    if not R then return sp, nil end
    local vis = M.vis(R)
    if #vis == 0 then return sp, nil end
    local v = (S.ctr_vi or ((pg - 1) * M.ROWS)) + 1
    while vis[v] and R.wps[vis[v]].disco do v = v + 1 end
    if v > #vis then v = 1 end
    S.ctr_vi = v
    S.ctr_page = ceil(v / M.ROWS)
    return sp, nil
  end
  if row == 6 then
    if S.mod and S.dir then
      if sp == "" then return sp, nil end
      local v = tonumber(sp)
      if not v or not smatch(sp, "^%d%d?%d?$") or v > 360 then return sp, "INVALID ENTRY" end
      local t = S.dir
      t.ic = v % 360
      t.icd = floor((ctx.ppos and G.dist(ctx.ppos.lat, ctx.ppos.lon, t.lat, t.lon) or 0) + 20.5)
      return "", nil
    end
    return sp, "KEY/FUNCTION INOP"
  end
  local R = S.mod or S.act
  if not R then return sp, "INVALID ENTRY" end
  local vis = M.vis(R)
  local idx = vis[(pg - 1) * M.ROWS + row]
  if not idx or R.wps[idx].disco or R.wps[idx].ty == 1 then
    if sp == "" then return sp, nil end
    return sp, "INVALID ENTRY"
  end
  if sp == "" then
    local w = R.wps[idx]
    local s = ""
    if w.spd then s = sformat("%d/", w.spd) end
    if w.cn and not w.der then s = s .. (w.spd and "" or "/") .. fmt_cn(w.cn, ctx.ta) end
    return s, nil
  end
  local Mr = S.mod or M.copy(S.act)
  local w = Mr.wps[idx]
  if sp == "DELETE" then
    if not w.cn and not w.spd then return sp, "INVALID DELETE" end
    w.cn = nil; w.spd = nil; w.der = nil; w.cnd = true
    S.mod = Mr
    return "", nil
  end
  local spd, alt = M.parse_constraint(sp)
  if not spd and not alt then return sp, "INVALID ENTRY" end
  if spd and not alt and not (w.cn and not w.der) then return sp, "INVALID ENTRY" end
  if alt then w.cn = alt; w.der = nil; w.cnd = nil end
  if spd then w.spd = spd end
  w.user = true
  S.mod = Mr
  return "", nil
end

-- EXEC ----------------------------------------------------------------------------------------------
function M.exec(S)
  if not S.mod then return false end
  local Mr = S.mod
  Mr.closed = Mr.closed or {}
  -- X-Plane discontinuities the pilot closed (the waypoint before it is now followed by a waypoint):
  -- remembered, so X-Plane putting its entry back does not reopen them
  if S.act then
    local pv = nil
    for _, w in ipairs(S.act.wps) do
      if w.xpd and not w.xdc and pv then
        for k, m in ipairs(Mr.wps) do
          if not m.disco and same_pos(m, pv, 1e-4) then
            -- (v52: the next element after the generated legs that follow the waypoint)
            local j = k + 1
            while Mr.wps[j] and Mr.wps[j].pleg do j = j + 1 end
            local nx = Mr.wps[j]
            if nx and not nx.disco then
              Mr.closed[pos_key(pv)] = true
              if pv.rw or (pv.ty == 1 and k == 1) then Mr.closed.DEP = true end
            end
            break
          end
        end
      end
      if not w.disco and not w.pleg then pv = w end   -- (v52: not a generated leg)
    end
  end
  -- a discontinuity left after such a waypoint (or re-entered by the pilot) opens it again
  for k, m in ipairs(Mr.wps) do
    if m.disco and not m.xdc and k > 1 and not Mr.wps[k - 1].disco then
      local pw = Mr.wps[k - 1]
      Mr.closed[pos_key(pw)] = nil
      if pw.rw or (pw.ty == 1 and k == 2) then Mr.closed.DEP = nil end
    end
  end
  -- X-Plane's open discontinuity entries cannot be written back: they become our own discontinuities
  local out = {}
  for _, w in ipairs(Mr.wps) do
    if not w.xdc then w.xpd = nil; out[#out + 1] = w end
  end
  local act = Mr.wps[Mr.active]
  Mr.wps = out
  for i, w in ipairs(out) do if w == act then Mr.active = i end end
  -- departure runway: X-Plane gets it as a point at the far END of the runway, so its map draws
  -- the route from the runway end to the first waypoint (as the runway point at the threshold it
  -- drew a leg out from the airport and a loop back at the runway)
  pcall(M.runway_to_end, Mr)
  S.act = S.mod
  S.mod = nil
  S.dir = nil
  return true
end

-- EXPORT --------------------------------------------------------------------------------------------
local function fms_type(w)
  if w.ty == 1 then return 1 elseif w.ty == 2 then return 2 elseif w.ty == 4 then return 3 elseif w.ty == 512 then return 11 end
  return 28
end

-- moves the departure runway element (after the origin airport) to the far end of the runway, from
-- the nav database (runway of that name, else the nearest threshold); once only
function M.runway_to_end(R)
  local o, w = R.wps[1], R.wps[2]
  if not (o and w and not o.disco and o.ty == 1 and not w.disco and w.rw and not w.rw_end) then return end
  if G.dist(o.lat, o.lon, w.lat, w.lon) > 4 then return end
  local N = navdb()
  if not (N and N.ready and N.procedures) then return end
  local ok, P = pcall(N.procedures, o.id)
  if not ok or not P or not P.rwys then return end
  local byid = {}
  for k, r in pairs(P.rwys) do byid[(tostring(k):gsub("%s", ""))] = r end
  local id = byid[w.id] and w.id or nil
  if not id then
    local bd = 1.0
    for k, r in pairs(byid) do
      local d = G.dist(r.lat, r.lon, w.lat, w.lon)
      if d < bd then id, bd = k, d end
    end
  end
  if not id then return end
  local n, sfx = smatch(id, "^RW(%d%d)([LRC]?)$")
  if not n then return end
  local on = (tonumber(n) + 18 - 1) % 36 + 1
  local os = (sfx == "L" and "R") or (sfx == "R" and "L") or sfx
  local e = byid[sformat("RW%02d%s", on, os)]
  if not e then return end
  -- the runway point stays at its threshold (where the SID starts); it is only marked to be kept in
  -- X-Plane's plan (it used to be moved to the far end, which put the first turn in the wrong place)
  w.rw_end = true
  if not is_rwy_ident(w.id) then w.id = id end
end

-- X-Plane .fms (v1100) text for the route
function M.fms_text(R, cycle)
  local list = {}
  for _, w in ipairs(R.wps) do if not w.disco and not w.pleg then list[#list + 1] = w end end
  local out = { "I", "1100 Version", "CYCLE " .. ((cycle and cycle ~= "") and cycle or "2306") }
  local first, last = list[1], list[#list]
  -- departure runway: given to X-Plane as its departure runway (DEPRWY), not as a waypoint - X-Plane
  -- then draws the runway and starts the route at it (as a waypoint it draws a leg from the airport
  -- reference point to the runway and a turn back at it)
  local deprwy = nil
  local r2 = list[2]
  if first and first.ty == 1 and r2 and r2.rw and type(r2.id) == "string"
    and G.dist(first.lat, first.lon, r2.lat, r2.lon) < 4 then
    -- X-Plane only takes a DEPRWY that is in its nav data (CIFP) for the airport - an unknown runway
    -- (e.g. one only in the scenery, with no procedures) crashes X-Plane when the plan is loaded.
    -- Otherwise the runway is left out of X-Plane's plan (the route starts at the airport; the LEGS
    -- page keeps showing the runway) - never sent as a waypoint (that draws a loop back at it)
    local N = navdb()
    local P = nil
    if N and N.ready and N.procedures then
      local ok, r = pcall(N.procedures, first.id)
      if ok then P = r end
    end
    local rr = nil
    if P and P.rwys then
      for k, r in pairs(P.rwys) do if (tostring(k):gsub("%s", "")) == r2.id then rr = r end end
    end
    -- (DEPRWY is not sent for now: a plan loaded with a DEPRWY and no SID may be what crashed X-Plane;
    -- M.SEND_DEPRWY re-enables it for runways that are in the nav data)
    if M.SEND_DEPRWY and rr and r2.id:match("^RW%d%d?[LRC]?$") and G.dist(rr.lat, rr.lon, r2.lat, r2.lon) < 1.5 then deprwy = r2.id end
    -- (the runway moved to its far end stays in the plan: the route starts there)
    if not r2.rw_end then table.remove(list, 2) end
  end
  if first and first.ty == 1 then out[#out + 1] = "ADEP " .. first.id end
  if deprwy then out[#out + 1] = "DEPRWY " .. deprwy end
  if last and last.ty == 1 and #list > 1 then out[#out + 1] = "ADES " .. last.id end
  out[#out + 1] = "NUMENR " .. #list
  for i, w in ipairs(list) do
    local via = "DRCT"
    if i == 1 and w.ty == 1 then via = "ADEP" elseif i == #list and w.ty == 1 and i > 1 then via = "ADES" end
    local alt = w.alt or 0
    if w.cn and not w.der then alt = (w.cn.d == "B") and w.cn.a or w.cn.a end
    local id = w.id or "WPT"
    if fms_type(w) == 28 then
      -- lat/lon entries: plain identifier only (X-Plane does not take names such as "(2400)")
      id = w.pp and "PPOS" or supper(tostring(id)):gsub("[^%u%d]", "")
      if id == "" then id = sformat("WPT%02d", i) end
      id = ssub(id, 1, 7)
    end
    out[#out + 1] = sformat("%d %s %s %.6f %.6f %.6f", fms_type(w), id, via, alt, w.lat, w.lon)
  end
  return table.concat(out, "\n") .. "\n"
end

-- extra data for the AFDS, one entry per X-Plane FMS entry
function M.ext(R, serial, with_active)
  local e = {}
  local db = false
  local a, ad = nil, nil
  for i, w in ipairs(R.wps) do
    if w.virt then
      if i == R.active then a = #e + 1 end
    elseif w.xdc then
      -- closed X-Plane discontinuity entry: lines up with X-Plane's list, not a discontinuity
      e[#e + 1] = { w.lat, w.lon, { xd = true, xdc = true } }
      if i == R.active then a = #e + 1 end
    elseif w.disco and w.xpd then
      -- X-Plane's own discontinuity entry: kept so the list lines up with X-Plane's
      e[#e + 1] = { w.lat, w.lon, { xd = true } }
      db = true
      if i == R.active then a = #e + 1; ad = true end
    elseif w.disco then
      db = true
      if i == R.active then a = #e + 1; ad = true end
    else
      local x = { id = w.id }
      if db then x.db = true end
      if w.cn and not w.der then x.cn = w.cn end
      if w.cnd then x.cnd = true end
      if w.spd then x.sp = w.spd end
      if w.df then x.df = { w.df.lat, w.df.lon, w.df.trk, w.df.gs } end
      if w.ic then x.ic = w.ic; x.icd = w.icd end
      if w.pp then x.pp = true end
      if w.rw then x.rw = true end
      e[#e + 1] = { w.lat, w.lon, x }
      db = false
      if i == R.active then a = #e end
    end
  end
  local o = { s = serial, e = e }
  if with_active then o.a = a; o.ad = ad end
  return o
end

-- index of the element for the X-Plane active entry k (1-based in the X-Plane list)
function M.elem_of_xp(R, k)
  local c = 0
  for i, w in ipairs(R.wps) do
    if is_xp_entry(w) then c = c + 1; if c == k then return i end end
  end
  return 0
end

-- RENDER --------------------------------------------------------------------------------------------
local function pad(s, n) s = s or "" if #s >= n then return ssub(s, 1, n) end return s .. srep(" ", n - #s) end
local function rpad(s, n) s = s or "" if #s >= n then return ssub(s, #s - n + 1) end return srep(" ", n - #s) .. s end
local function overlay(base, s, col)
  -- put s into base at column col (1-based)
  return ssub(base, 1, col - 1) .. s .. ssub(base, col + #s)
end
local BLANK = srep(" ", 24)

local function fmt_dist(d)
  if d < 9.95 then return sformat("%.1fNM", d) end
  return sformat("%dNM", floor(d + 0.5))
end

-- returns large[13], small[13]
function M.render(S, pg, ctx)
  local R = S.mod or S.act
  local L, Sm = {}, {}
  for i = 1, 13 do L[i] = BLANK; Sm[i] = BLANK end
  local np = R and M.num_pages(R) or 1
  if pg > np then pg = np end
  local title = (S.mod and "MOD" or "ACT") .. " RTE 1 LEGS"
  L[1] = pad("    " .. title, 19) .. rpad(sformat("%d/%d", pg, np), 4) .. " "
  L[12] = "------------------------"
  if S.mod then L[13] = "<ERASE                  " else L[13] = "<RTE 2 LEGS    RTE DATA>" end
  -- v42: PLAN mode on an ND: 6R STEP moves the map centre (<CTR>) from waypoint to waypoint
  if ctx.plan and not S.mod then L[13] = "<RTE 2 LEGS        STEP>" end
  if S.mod and S.dir then
    L[12] = "-------------           "
    Sm[12] = overlay(BLANK, "INTC CRS", 16)
    local c = S.dir.ic or intc_crs_default(S, ctx)
    L[13] = overlay(L[13], c and sformat("%03d`", c) or "---`", 21)
  end
  if not R then return L, Sm end
  local vis = M.vis(R)
  -- PLAN centre: a waypoint of this page (the first one after a page change)
  local ctr_vi = nil
  if ctx.plan then
    local first, last = (pg - 1) * M.ROWS + 1, min(pg * M.ROWS, #vis)
    ctr_vi = S.ctr_vi
    if not ctr_vi or ctr_vi < first or ctr_vi > last then ctr_vi = first end
    while ctr_vi <= last and vis[ctr_vi] and R.wps[vis[ctr_vi]].disco do ctr_vi = ctr_vi + 1 end
    if ctr_vi > last or not vis[ctr_vi] then ctr_vi = nil end
    S.ctr_vi = ctr_vi
    local cw = ctr_vi and R.wps[vis[ctr_vi]]
    S.ctr_lat, S.ctr_lon = cw and cw.lat or nil, cw and cw.lon or nil
  end
  for row = 1, M.ROWS do
    local vi = (pg - 1) * M.ROWS + row
    local idx = vis[vi]
    local hl, ll = 2 * row, 2 * row + 1
    if idx then
      local w = R.wps[idx]
      if w.disco then
        Sm[hl] = pad(" THEN", 24)
        L[ll] = pad(BOXES, 24)
      else
        -- header: course and distance of the leg to this waypoint
        local pk = idx - 1
        while R.wps[pk] and R.wps[pk].xdc do pk = pk - 1 end
        local prev = R.wps[pk]
        if prev and prev.disco then
          L[hl] = DISCO_TXT
        else
          local fl, fo
          if vi == 1 and ctx.inflight and ctx.ppos then fl, fo = ctx.ppos.lat, ctx.ppos.lon
          elseif w.df and idx == R.active and ctx.ppos then fl, fo = w.df.lat, w.df.lon
          elseif prev then fl, fo = prev.lat, prev.lon end
          if fl then
            local d = G.dist(fl, fo, w.lat, w.lon)
            local c
            if w.ic then c = w.ic
            else c = floor(G.true_to_mag(G.brg(fl, fo, w.lat, w.lon), fl, fo) + 0.5) % 360 end
            if w.cond then
              -- conditional leg (intercept, vectors, DME / radial): course or heading only
              Sm[hl] = overlay(BLANK, sformat("%03d`", c) .. (w.hdg and " HDG" or ""), 2)
            elseif d < 0.05 and not w.ic then
              Sm[hl] = overlay(BLANK, fmt_dist(d), 8)
            else
              Sm[hl] = overlay(overlay(BLANK, sformat("%03d`", c), 2), rpad(fmt_dist(d), 7), 7)
            end
          end
        end
        L[ll] = pad(w.id, 24)
        if ctr_vi and vi == ctr_vi then Sm[ll] = overlay(Sm[ll], "<CTR>", 9) end
        -- right side: restrictions in large font, VNAV predictions in small font (as on the aircraft)
        local pr = pred_at(ctx.pred, w)
        local p_spd = pr and pr.spd and pr.spd > 0 and fmt_spd(pr.spd) or nil
        local p_alt = pr and pr.alt and fmt_alt(pr.alt, ctx.ta) or nil
        local has_cn = w.cn and not w.der
        if has_cn or w.spd then
          local cn_t = has_cn and rpad(fmt_cn(w.cn, ctx.ta), w.cn.d == "B" and 12 or 6) or nil
          local alt_t = cn_t or rpad(p_alt or "----", 6)
          local spd_t = w.spd and sformat("%d", w.spd) or (p_spd or "")
          local right = spd_t .. "/" .. alt_t
          local c0 = 25 - #right
          -- (large: the restriction parts; small: the predicted parts)
          if w.spd then L[ll] = overlay(L[ll], spd_t .. "/", c0) else
            if p_spd then Sm[ll] = overlay(Sm[ll], spd_t, c0) end
            L[ll] = overlay(L[ll], "/", c0 + #spd_t)
          end
          if cn_t then L[ll] = overlay(L[ll], cn_t, c0 + #spd_t + 1)
          elseif p_alt then Sm[ll] = overlay(Sm[ll], alt_t, c0 + #spd_t + 1)
          else L[ll] = overlay(L[ll], alt_t, c0 + #spd_t + 1) end
        elseif p_spd or p_alt then
          local t = (p_spd or "") .. "/" .. rpad(p_alt or "", 6)
          Sm[ll] = overlay(Sm[ll], t, 25 - #t)
        elseif w.cn and w.der then
          local t = "/" .. rpad(fmt_cn({ d = "@", a = w.cn.a }, ctx.ta), 6)
          Sm[ll] = overlay(BLANK, t, 25 - #t)
        end
      end
    end
  end
  return L, Sm
end

B744LEGS = M
return M
