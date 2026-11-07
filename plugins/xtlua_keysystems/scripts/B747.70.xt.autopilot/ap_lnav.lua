--[[
 ap_lnav.lua  --  pure lateral-navigation guidance for the rebuilt Sparky 747 autopilot.

 Route format (same as xtlua/fms json, 1-based):
   r[i][5]=lat  r[i][6]=lon  r[i][2]=type (1 = airport)  r[i][8]=ident  r[i][9]=altitude constraint
   r[i][10]=true when X-Plane's FMS says this is the active waypoint
 The active leg is r[idx-1] -> r[idx].

 What this replaces: the stock code re-searched every leg on every frame (heading match + xtk)
 and could skip or flip legs.  Here sequencing is deterministic: advance when the turn-anticipation
 distance is reached, and only search for a leg once, when the route actually changes.
--]]
local U = AP_LOAD("ap_util.lua")

local M = {}

M.CFG = {
  bank_nom        = 25,    -- deg, used for turn-radius / anticipation
  roll_lag_s      = 3.0,   -- s of extra lead for roll-in
  lead_max_nm     = 20,
  lead_leg_frac   = 0.45,  -- never anticipate more than this fraction of the leg
  la_time_s       = 30,    -- L1 look-ahead time
  la_min_nm       = 2.0,
  la_max_nm       = 6.0,
  int_max_deg     = 45,    -- max intercept angle
  cap_xtk_nm      = 2.5,   -- armed LNAV captures anywhere inside this
  reacq_xtk_nm    = 15,    -- farther than this from the active leg => search once
  min_turn_deg    = 2.0,   -- below this just sequence at the waypoint
}

function M.new()
  return { idx = 0, sig = nil, reacq_sig = nil, last_seq_time = -100 }
end

local function wp(route, i)
  local w = route[i]
  if w == nil then return nil end
  return w[5], w[6]
end

-- course (true) of the great circle a->b *at the aircraft's along-track position*
-- (matters on long oceanic legs where the initial bearing differs from the local one)
local function local_course(lat1, lon1, lat2, lon2, atk)
  if atk ~= atk then atk = 0 end
  local c0 = U.bearing(lat1, lon1, lat2, lon2)
  local len = U.distance_nm(lat1, lon1, lat2, lon2)
  if len < 30 then return c0 end
  local p_lat, p_lon = U.destination(lat1, lon1, U.clamp(atk, 0, len - 0.01), c0)
  return U.bearing(p_lat, p_lon, lat2, lon2)
end

-- Turn anticipation distance (nm) for a fly-by turn of `delta` degrees at ground speed gs (m/s)
function M.turn_lead(delta, gs_mps, cfg)
  cfg = cfg or M.CFG
  if delta < cfg.min_turn_deg then return 0.3 end
  local bank = math.rad(cfg.bank_nom)
  local r_m  = (gs_mps * gs_mps) / (9.81 * math.tan(bank))
  local r_nm = r_m / 1852
  local d = r_nm * math.tan(math.rad(math.min(delta, 150)) / 2)
  d = d + (gs_mps / 1852) * cfg.roll_lag_s
  return U.clamp(d, 0.3, cfg.lead_max_nm)
end

-- cheap structural signature so we notice the pilot editing the route / direct-to
function M.route_sig(route)
  local n = #route
  if n == 0 then return "0" end
  local a, b = route[1], route[n]
  local mid = route[math.floor(n / 2) + 1] or a
  return string.format("%d:%s:%s:%.4f:%.4f:%.4f", n, tostring(a[8]), tostring(b[8]),
                       mid[5] or 0, mid[6] or 0, b[5] or 0)
end

-- One-shot best leg search.  Returns idx (2..n) or nil.
function M.find_leg(route, lat, lon, hdg_true, cfg)
  cfg = cfg or M.CFG
  local n = #route
  local best, best_cost
  for i = 2, n do
    local a_lat, a_lon = wp(route, i - 1)
    local b_lat, b_lon = wp(route, i)
    if a_lat and b_lat then
      local len = U.distance_nm(a_lat, a_lon, b_lat, b_lon)
      if len > 0.1 then
        local xtk, atk = U.xtk_atk(a_lat, a_lon, b_lat, b_lon, lat, lon)
        local crs = U.bearing(a_lat, a_lon, b_lat, b_lon)
        local hd = math.abs(U.hdg_diff(hdg_true, crs))
        if atk >= -1 and atk <= len + 1 and hd < 100 then
          local cost = math.abs(xtk) + 0.03 * hd
          if best == nil or cost < best_cost then best, best_cost = i, cost end
        end
      end
    end
  end
  return best
end

-- The FMS marks one waypoint as active ([10]==true).  Used when we are not in override mode.
function M.fms_active_index(route)
  for i = 1, #route do
    if route[i][10] == true then return i end
  end
  return nil
end

--[[
 step(L, inp, route, cfg)
 inp: lat, lon, gs (m/s), trk (true ground track deg), hdg_true, mag_var (true - magnetic),
      wind_bearing, wind_kts, tas_kts, xtk_target (nm offset, <=-99 means none)
 returns result table (res.valid == false when there is nothing to fly)
--]]
function M.step(L, inp, route, cfg)
  cfg = cfg or M.CFG
  local n = #route
  local res = { valid = false, idx = L.idx, seq = false, route_end = false }
  if n < 2 then L.idx = 0; res.idx = 0; return res end

  -- route changed? re-sync once
  local sig = M.route_sig(route)
  if sig ~= L.sig then
    L.sig = sig
    local fi = M.fms_active_index(route)
    if fi and fi >= 2 then
      L.idx = fi
    elseif L.idx < 2 or L.idx > n then
      L.idx = M.find_leg(route, inp.lat, inp.lon, inp.hdg_true, cfg) or 2
    end
  end
  if L.idx < 2 then L.idx = 2 end
  if L.idx > n then L.idx = n end

  local gs = math.max(inp.gs, 40 * U.KT_MPS)
  local gs_kt = gs * U.MPS_KT
  local offset = 0
  if inp.xtk_target and inp.xtk_target > -99 then offset = inp.xtk_target end

  for _ = 1, 4 do      -- allow a few legs to be skipped in one frame (very short legs)
    local idx = L.idx
    local a_lat, a_lon = wp(route, idx - 1)
    local b_lat, b_lon = wp(route, idx)
    if not (a_lat and b_lat) then return res end
    local len = U.distance_nm(a_lat, a_lon, b_lat, b_lon)
    local xtk, atk = U.xtk_atk(a_lat, a_lon, b_lat, b_lon, inp.lat, inp.lon)
    local togo = len - atk
    local course = local_course(a_lat, a_lon, b_lat, b_lon, atk)
    local advanced = false

    -- far from this leg and we have not searched yet for this route => look once
    if math.abs(xtk) > cfg.reacq_xtk_nm and L.reacq_sig ~= sig then
      L.reacq_sig = sig
      local found = M.find_leg(route, inp.lat, inp.lon, inp.hdg_true, cfg)
      if found and found ~= idx then L.idx = found; advanced = true end
    end

    -- sequencing
    local lead = 0.3
    if not advanced then
      if idx < n then
        local c_end  = U.wrap360(U.bearing(b_lat, b_lon, a_lat, a_lon) + 180)
        local nx_lat, nx_lon = wp(route, idx + 1)
        if nx_lat then
          local c_next = U.bearing(b_lat, b_lon, nx_lat, nx_lon)
          local delta = math.abs(U.hdg_diff(c_next, c_end))
          lead = M.turn_lead(delta, gs, cfg)
        end
        lead = math.min(lead, len * cfg.lead_leg_frac)
        if togo <= lead then
          L.idx = idx + 1
          L.last_seq_time = inp.time or 0
          res.seq = true
          advanced = true
        end
      elseif togo < 0.2 then
        res.route_end = true
      end
    end

    if not advanced then
      local xtk_eff = xtk - offset
      local la = U.clamp(gs_kt * cfg.la_time_s / 3600, cfg.la_min_nm, cfg.la_max_nm)
      local eta = math.deg(math.atan2(math.abs(xtk_eff), la))
      eta = math.min(eta, cfg.int_max_deg)
      if xtk_eff > 0 then eta = -eta end          -- right of course => steer left
      local trk_cmd = U.wrap360(course + eta)
      local mag_var = inp.mag_var or 0
      local hdg_cmd = U.wca_heading(trk_cmd - mag_var, inp.wind_bearing or 0,
                                    inp.wind_kts or 0, inp.tas_kts or gs_kt)

      -- capture test for armed LNAV
      local trk = inp.trk or inp.hdg_true
      local ta = U.hdg_diff(trk, course)
      local converging = (xtk_eff > 0 and ta < 0) or (xtk_eff < 0 and ta > 0)
      local cap = math.abs(xtk_eff) <= cfg.cap_xtk_nm
      if not cap and converging and math.abs(ta) <= 90 then
        local r_nm = (gs * gs) / (9.81 * math.tan(math.rad(cfg.bank_nom))) / 1852
        local need = r_nm * (1 - math.cos(math.rad(math.abs(ta)))) +
                     (gs / 1852) * cfg.roll_lag_s * math.sin(math.rad(math.abs(ta)))
        cap = math.abs(xtk_eff) <= need
      end

      res.valid        = true
      res.idx          = idx
      res.len          = len
      res.atk          = atk
      res.togo         = togo
      res.xtk          = xtk
      res.xtk_eff      = xtk_eff
      res.course       = course
      res.trk_cmd      = trk_cmd
      res.hdg_cmd_mag  = hdg_cmd
      res.lead         = lead
      res.capture_ok   = cap
      res.dist_to_wp   = U.distance_nm(inp.lat, inp.lon, b_lat, b_lon)
      return res
    end
  end
  res.idx = L.idx
  return res
end

AP_MODULES = AP_MODULES or {}
AP_MODULES["ap_lnav.lua"] = M
return M
