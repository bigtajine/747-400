--[[
 ap_vnav.lua  --  pure vertical-navigation computations for the rebuilt Sparky 747 autopilot.

 Builds a descent path BACKWARDS from the destination through the altitude constraints, finds the
 top of descent against the cruise altitude, and reports path deviation / path V/S.  It does not
 touch X-Plane and does not decide modes -- ap_modes.lua does that using these numbers.

 Path definition (distance E is measured from the END of the route, nm):
   * vertex 0       : destination elevation at E = 0
   * constraint wps : (E_k, max(constraint, previous vertex alt))   ("AT" semantics: the FMS data
                      carries no AT-OR-ABOVE/BELOW flag, so every number is treated as AT)
   * TOD            : idle-slope line (slope_ftnm) back up from the last vertex to the cruise altitude
 Same published meaning as stock: top_of_descent = E_tod,  remaining_distance = total route nm,
 dist_to_TOD = total - E_tod.
--]]
local U = AP_LOAD("ap_util.lua")

local M = {}

M.CFG = {
  slope_ftnm      = 290,     -- idle descent gradient (~2.75 deg), same figure the stock code used
  corr_gain       = 1.2,     -- fpm of correction per ft of path error
  corr_max        = 1500,    -- fpm
  bias_ft         = 50,      -- aim this far below the path so constraints are met, not missed
  vs_min          = -2500,
  vs_max          = 1500,
  end_clamp_nm    = 10,
  toc_min_gs_kt   = 280,
  toc_min_vs      = 2000,
}

function M.new()
  return { prof = nil, last_calc = -100, sig = nil }
end

local function airport_alt(w)
  if w == nil then return 0 end
  local a = w[9]
  if a == nil or a < 0 then a = w[3] end
  return a or 0
end

-- distance (nm) from the present position to each waypoint i >= start along the route
local function cum_distances(route, start, d_next)
  local D = {}
  D[start] = d_next
  for i = start, #route - 1 do
    D[i + 1] = D[i] + U.distance_nm(route[i][5], route[i][6], route[i + 1][5], route[i + 1][6])
  end
  return D
end

-- lat/lon of the point `dist` nm ahead of the aircraft along the route
local function point_ahead(route, D, start, dist, pos_lat, pos_lon)
  local n = #route
  if dist < 0 then return nil end
  if dist <= D[start] then
    local brg = U.bearing(pos_lat, pos_lon, route[start][5], route[start][6])
    return U.destination(pos_lat, pos_lon, dist, brg)
  end
  for i = start, n - 1 do
    if dist <= D[i + 1] then
      local seg = D[i + 1] - D[i]
      if seg < 1e-6 then return route[i + 1][5], route[i + 1][6] end
      local brg = U.bearing(route[i][5], route[i][6], route[i + 1][5], route[i + 1][6])
      return U.destination(route[i][5], route[i][6], dist - D[i], brg)
    end
  end
  return route[n][5], route[n][6]
end

--[[
 build(route, start, inp, cruise_alt, cfg)  ->  profile or nil
 inp: lat, lon, palt, gs, vs
--]]
function M.build(route, start, inp, cruise_alt, cfg)
  cfg = cfg or M.CFG
  local n = #route
  if n < 3 or start == nil or start < 1 then return nil end
  if start > n then start = n end
  local last = route[n]
  if last[2] ~= 1 then return nil end              -- VNAV needs the route to end at an airport

  local d_next = U.distance_nm(inp.lat, inp.lon, route[start][5], route[start][6])
  local D = cum_distances(route, start, d_next)
  local total = D[n]
  local alt_end = airport_alt(last)

  -- vertices from the end backwards
  local verts = { { E = 0, A = alt_end } }
  for i = n - 1, start, -1 do
    local c = route[i][9] or 0
    if c > 0 then
      local E = total - D[i]
      local prev = verts[#verts]
      if E > prev.E + 0.05 then
        local a = math.max(c, prev.A)
        if a > cruise_alt and cruise_alt > 0 then a = cruise_alt end
        verts[#verts + 1] = { E = E, A = a, idx = i }
      end
    end
  end

  local top = verts[#verts]
  local E_tod
  if cruise_alt > top.A then
    E_tod = top.E + (cruise_alt - top.A) / cfg.slope_ftnm
  else
    E_tod = top.E
  end
  verts[#verts + 1] = { E = E_tod, A = math.max(cruise_alt, top.A), tod = true }

  local prof = {
    start = start, n = n, D = D, total = total, verts = verts, E_tod = E_tod,
    cruise = cruise_alt, alt_end = alt_end,
  }

  -- eod index (first wp within 10 nm of the airport), kept for the ND
  prof.eod = n
  for i = start, n do
    if U.distance_nm(route[i][5], route[i][6], last[5], last[6]) < cfg.end_clamp_nm then
      prof.eod = i
      break
    end
  end

  -- TOD point for the ND
  local d_to_tod = total - E_tod
  if d_to_tod > 0 then
    prof.tod_lat, prof.tod_lon = point_ahead(route, D, start, d_to_tod, inp.lat, inp.lon)
  end
  return prof
end

-- path altitude at distance-from-end E
function M.path_alt(prof, E)
  local v = prof.verts
  if E <= 0 then return v[1].A, 0 end
  for k = 1, #v - 1 do
    local a, b = v[k], v[k + 1]
    if E <= b.E then
      local span = b.E - a.E
      if span < 1e-6 then return b.A, 0 end
      local slope = (b.A - a.A) / span           -- ft per nm, >= 0 going away from the end
      return a.A + slope * (E - a.E), slope
    end
  end
  return v[#v].A, 0                              -- beyond TOD: cruise
end

-- next vertex AHEAD of E that is lower than the path here: (alt, distance_nm) or nil
function M.next_target(prof, E)
  local v = prof.verts
  local here = M.path_alt(prof, E)
  local best
  for k = 1, #v do
    if v[k].E < E - 0.05 and not v[k].tod and v[k].A < here - 50 then
      if best == nil or v[k].E > best.E then best = v[k] end
    end
  end
  if best then return best.A, E - best.E end
  return nil
end

-- lat/lon of the point `dist` nm ahead along the route (used for the TOC marker on the ND)
function M.point_on_route(route, start, inp, dist)
  if #route < 2 or start < 1 or start > #route or dist == nil or dist < 0 then return nil end
  local d_next = U.distance_nm(inp.lat, inp.lon, route[start][5], route[start][6])
  local D = cum_distances(route, start, d_next)
  return point_ahead(route, D, start, dist, inp.lat, inp.lon)
end

function M.toc_distance(inp, cruise_alt, cfg)
  cfg = cfg or M.CFG
  if cruise_alt <= 0 then return -1 end
  local diff = cruise_alt - inp.palt
  if diff <= 0 then return 0 end
  local fpm = math.max(inp.vs or 0, cfg.toc_min_vs)
  local gs_kt = math.max(inp.gs * U.MPS_KT, cfg.toc_min_gs_kt)
  return gs_kt * (diff / fpm) / 60
end

--[[
 step(V, inp, route, start, cruise_alt, cfg)
 Recomputes the profile (cheap numeric work, throttled to every 0.5 s or on route change) and
 returns the current numbers.
--]]
function M.step(V, inp, route, start, cruise_alt, cfg)
  cfg = cfg or M.CFG
  local res = { valid = false, toc = -1 }
  res.toc = M.toc_distance(inp, cruise_alt, cfg)
  local t = inp.time or 0
  if V.prof == nil or (t - V.last_calc) >= 0.5 or t < V.last_calc or V.cruise ~= cruise_alt or V.start ~= start then
    V.prof = M.build(route, start, inp, cruise_alt, cfg)
    V.last_calc, V.cruise, V.start = t, cruise_alt, start
  end
  local p = V.prof
  if p == nil then return res end

  -- the profile distances were measured at last_calc; roll them forward by the distance flown
  local d_next_now = U.distance_nm(inp.lat, inp.lon, route[p.start][5], route[p.start][6])
  local total_now = p.total - p.D[p.start] + d_next_now
  local E = total_now

  local pa, slope = M.path_alt(p, E)
  local gs_kt = inp.gs * U.MPS_KT
  local path_vs = -slope * gs_kt / 60
  local fpe = inp.palt - pa

  local corr = U.clamp(-cfg.corr_gain * (fpe + cfg.bias_ft), -cfg.corr_max, cfg.corr_max)
  local vs_cmd = U.clamp(path_vs + corr, cfg.vs_min, cfg.vs_max)

  local tgt_alt, tgt_dist = M.next_target(p, E)

  res.valid         = true
  res.total         = total_now
  res.tod           = p.E_tod                     -- distance from END (published as top_of_descent)
  res.dist_to_tod   = total_now - p.E_tod
  res.path_alt      = pa
  res.path_vs       = path_vs
  res.fpe           = fpe
  res.vs_cmd        = vs_cmd
  res.slope         = slope
  res.target_alt    = tgt_alt
  res.target_dist   = tgt_dist
  res.eod           = p.eod
  res.next_dist_ft  = d_next_now * U.NM_FT
  res.dist_to_dest  = U.distance_nm(inp.lat, inp.lon, route[#route][5], route[#route][6])
  res.tod_lat, res.tod_lon = p.tod_lat, p.tod_lon
  res.profile       = p
  return res
end

AP_MODULES = AP_MODULES or {}
AP_MODULES["ap_vnav.lua"] = M
return M
