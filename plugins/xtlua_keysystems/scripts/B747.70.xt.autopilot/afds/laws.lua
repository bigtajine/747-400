--[[ B747-400 AFDS guidance laws (rebuilt).
     Produce flight-director / autopilot pitch and roll ATTITUDE commands (deg) that the existing
     Sparky surface servos (hydraulics_override.lua) track, plus rudder for align/rollout and an
     A/T lever target for the THR mode.
     Pitch: flight-path-angle command loop  theta_c = theta_ref + Kp*(gamma_c - gamma),
            theta_ref integrates Ki*(gamma_c - gamma).  gamma_c comes from V/S, altitude (with a
            constant-g capture law), speed-on-elevator, glideslope, VNAV path or flare laws.
     Roll:  heading / track error laws with bank limit and roll-rate limiting.
     Registered as global B744LAWS. ]]
local M = {}
local abs, min, max, floor, sqrt, atan, asin, sin, cos, tan, pi =
  math.abs, math.min, math.max, math.floor, math.sqrt, math.atan, math.asin, math.sin, math.cos, math.tan, math.pi
local D2R, R2D = pi / 180, 180 / pi
local GRAV = 9.80665
local C_LNAV_BANK = 25

M.P = {
  kp_g = 1.0,       -- deg pitch per deg flight path error
  ki_g = 0.15,      -- 1/s (speed-on-elevator modes)
  ki_path = 0.30,   -- 1/s (G/S, V/S, ALT, VNAV PTH)
  pitch_lead_path = 4.0,
  pitch_lead_max = 2.5,
  pitch_rate = 1.0, -- deg/s command rate limit
  fd_kp = 0.6,      -- FD only: deg pitch per deg flight path error
  fd_rate = 3.0,    -- FD only: deg/s bar rate limit
  fd_tau_v = 12.0,  -- FD only: s, speed error -> acceleration wanted
  pitch_min = -10, pitch_max = 20,
  kv = 0.08,        -- deg gamma per kt speed error (speed on elevator)
  kvd = 1.2,        -- deg gamma per kt/s speed rate
  k_alt = 8.0,      -- fpm per ft (linear part of altitude law)
  cap_a = 1.6,      -- ft/s^2 capture accel
  kpsi = 1.0,       -- deg bank per deg heading error
  roll_rate = 3.0,  -- deg/s
  roll_rate_loc = 3.0,
  flare_tau = 4.0,
  flare_kp = 2.5, flare_ki = 0.8,
  flare_td_fpm = 120,
  flare_lead = 0.5,
  gs_dot_deg = 0.35 / 1.0, -- deg per dot of glideslope deviation (2 dots = 0.7 deg)
  loc_dot_deg = 1.0,
  kloc = 1.25, loc_la = 2.6, loc_cap_bank = 22,  -- deg per dot of localizer deviation (approx. 2.5 dots = 2.5 deg)
  k_gs = 1.7,       -- fpm per ft of glideslope deviation
  at_tau = 6.0, at_tau_appr = 4.0, at_tau_acc = 2.5,   -- A/T SPD: speed and acceleration loop time constants
  roll_acc = 5.0,   -- deg/s^2: roll rate onset (smooth roll-in / roll-out)
  roll_tau = 0.5,   -- s: roll command smoothing
  ki_alt = 0.30,    -- 1/s: level hold integral
  alpha_ff = 1.0,   -- angle-of-attack feed-forward on speed changes in level flight
}
local P = M.P

local function clamp(x, a, b) if x < a then return a elseif x > b then return b end return x end
local function wrap180(a) a = (a + 180) % 360 - 180 return a end

local function gamma_of(vs, tas) return asin(clamp(vs / (max(60, tas) * 101.27), -0.5, 0.5)) * R2D end
M.gamma_of = gamma_of

-- (high altitude: the aircraft answers a pitch change more slowly - thin air, high TAS - so the level
-- hold is softer up there; the stiff low-level gains made the cruise level hunt)
local function hi_alt_frac(alt) return clamp(((alt or 0) - 25000) / 14000, 0, 1) end
M.hi_alt_frac = hi_alt_frac
-- (v54) low true airspeed: the same vertical speed is a much steeper path (200 kt: twice the angle of
-- 400 kt), so the linear level-hold gain is reduced below 400 kt TAS (the hold at 3000 ft / 200 kt
-- overshot further each time). Unchanged at 400 kt TAS and above.
local function lo_tas_k(tas) if not tas then return 1 end return clamp(tas / 400, 0.5, 1) end
M.lo_tas_k = lo_tas_k
function M.alt_law_vs(err, alt, tas)
  local a = 60 * sqrt(2 * P.cap_a * abs(err))
  local l = P.k_alt * (1 - 0.5 * hi_alt_frac(alt)) * lo_tas_k(tas) * abs(err)
  local v = min(a, l)
  return (err >= 0) and v or -v
end

function M.bank_limit(inp)
  local sel = inp.bank_sel or 0
  if sel >= 1 and sel <= 5 then return sel * 5 end
  local tas = inp.tas or 250
  if tas <= 300 then return 25 end
  if tas >= 450 then return 15 end
  return 25 - (tas - 300) / 150 * 10
end

-- airspeed rate estimate: complementary filter, inertial (ground speed derivative scaled to IAS) for the
-- fast part and the (turbulent) indicated airspeed for the slow part -> no lag and little gust noise
function M.speed_rate(st, ias, gs, tas, dt)
  dt = max(0.005, dt or 0.05)
  if st.vh == nil or abs(ias - st.vh) > 40 then st.vh = ias; st.gs = gs; st.a = 0 end
  local a = (gs - st.gs) / dt
  st.gs = gs
  st.a = st.a + (a - st.a) * min(1, dt / 0.3)
  local k = 0.4
  local a_ias = st.a * clamp(ias / max(60, tas), 0.4, 1.2)
  local r = a_ias + k * (ias - st.vh)
  st.vh = st.vh + r * dt
  st.rate = r
  return r
end

-- speed-on-elevator: gamma command
local function speed_gamma(S, inp, vt)
  local g = inp.gamma or 0
  local e = (inp.ias or vt) - vt
  if S.fd_hand then
    -- flight director only: total-energy speed law. Energy path angle = path + acceleration/g
    -- (3 deg per kt/s) does not change when the nose is moved (speed and climb trade), so the bar
    -- shows the pitch the speed needs, independent of where the pilot has the nose right now.
    local acmd = clamp(-e / P.fd_tau_v, -1.0, 1.0)              -- kt/s wanted
    return g + 3.0 * ((inp.ias_rate or 0) - acmd)
  end
  local gc = g + P.kv * e + P.kvd * (inp.ias_rate or 0)
  -- v42: accelerating in a climb (flap retraction schedule after the acceleration height) more than
  -- 5 kt slow: the nose is lowered enough for about 1 kt/s (energy split: 3 deg per kt/s) - it was
  -- holding most of the climb and accelerating at about 0.3 kt/s
  if e < -5 and (inp.vs or 0) > 300 then
    local a_want = clamp(-e / 12, 0, P.clb_accel or 1.0)
    gc = math.min(gc, g + 3.0 * ((inp.ias_rate or 0) - a_want))
  end
  return gc
end

-- LOC capture turn point from the runway geometry: true when the capture turn (bank P.loc_cap_bank)
-- has to start now to end on the centreline. Used to capture before the beam leaves full scale.
function M.loc_turn_point(inp)
  local nav = inp.nav or {}
  local x = nav.xtk_geo
  if not x or not nav.crs then return false end
  local trk = inp.trk_mag or inp.hdg_mag or 0
  local th = wrap180(trk - nav.crs)
  if abs(th) < 4 or x * th >= 0 then return false end
  local gs = max(60, inp.gs or 150)
  local v = gs * 0.5144
  local t_lead = P.loc_cap_bank / (2 * P.roll_rate_loc) + 1.5
  local xp = abs(x) - gs / 3600 * t_lead * abs(sin(th * D2R))
  if xp <= 0.001 then return true end
  local R = xp * 1852 / (1 - cos(abs(th) * D2R))
  return atan(v * v / (GRAV * R)) * R2D >= P.loc_cap_bank * 0.9
end

-- LOC intercept geometry: intercept angle (track - course, deg) and the bank that makes a turn
-- started now end exactly on the centreline (nil when not converging on the beam)
function M.loc_turn_bank(inp)
  local nav = inp.nav or {}
  if not nav.crs then return nil end
  local x = nav.xtk_geo
  if not x and nav.hdef then
    local dist = nav.dme and nav.dme > 0.2 and nav.dme or 10
    x = -dist * tan(clamp(nav.hdef * P.loc_dot_deg, -10, 10) * D2R)
  end
  if not x then return nil end
  local trk = inp.trk_mag or inp.hdg_mag or 0
  local th = wrap180(trk - nav.crs)
  if abs(th) < 1 or x * th >= 0 then return nil end
  local gs = max(60, inp.gs or 150)
  local v = gs * 0.5144
  local xp = abs(x) - gs / 3600 * 1.0 * abs(sin(th * D2R))
  if xp <= 0.001 then return th, 90 end
  local R = xp * 1852 / max(1e-4, 1 - cos(abs(th) * D2R))
  return th, atan(v * v / (GRAV * R)) * R2D
end

function M.compute(S, inp, ctx)
  local dt = max(0.005, inp.dt or 0.05)
  S.fd_hand = not (ctx.nap and ctx.nap > 0) and not inp.onground
  local out = { pitch = inp.pitch or 0, roll = 0, rudder = nil, thr = -1 }
  local theta = inp.pitch or 0
  local tas = inp.tas or 250
  local vs = inp.vs or 0
  local g_now = inp.gamma or gamma_of(vs, tas)
  local gc = nil           -- flight path command (deg)
  local fixed_theta = nil  -- direct pitch command
  local pm = S.pitch
  local air = not inp.onground
  local vnav = inp.vnav or {}

  if pm ~= "GS" then S.vdef_f = nil; S.gs_v = nil end
  if S.roll ~= "LOC" and S.roll ~= "ROLLOUT" then S.hdef_f = nil end
  -- PITCH -------------------------------------------------------------------------------------------
  if pm == "TOGA" then
    if not air then
      fixed_theta = 8.0
    else
      gc = speed_gamma(S, inp, ctx.spd_tgt)
      if S.toga == "GA" then
        gc = max(gc, 1.0)
      else
        gc = max(gc, 2.0)
      end
    end
  elseif pm == "VS" then
    gc = gamma_of(S.vs_target or 0, tas)
  elseif pm == "ALT" then
    local e = (S.alt_hold or inp.alt) - inp.alt
    local v = M.alt_law_vs(e, inp.alt, tas)
    -- capture: do not exceed the entry vertical speed
    if S.capturing then
      local lim = max(300, abs(S.cap_vs or vs))
      v = clamp(v, -lim, lim)
    else
      S.cap_vs = vs
    end
    gc = gamma_of(v, tas)
    out.alt_tgt = S.alt_hold
  elseif pm == "FLCH" then
    gc = speed_gamma(S, inp, ctx.spd_tgt)
    if (S.flch_dir or 1) > 0 then gc = max(gc, 0.3) else gc = min(gc, -0.3) end
  elseif pm == "VNAV" then
    local p = vnav.pitch or "SPD"
    if p == "SPD" then
      gc = speed_gamma(S, inp, ctx.spd_tgt)
      local lvl = vnav.alt_tgt
      if lvl then
        if lvl > inp.alt then gc = max(gc, 0.3) elseif lvl < inp.alt then gc = min(gc, vnav.spd_des_floor and -1.0 or -0.3) end
        -- capture by the altitude law when close
        local e = lvl - inp.alt
        local v = M.alt_law_vs(e)
        if (e > 0 and v < vs) or (e < 0 and v > vs) then gc = gamma_of(v, tas) end
      end
    elseif p == "VS" then
      local v = vnav.vs or 0
      local lvl = vnav.alt_tgt
      if lvl and lvl > -1000 then
        local e = lvl - inp.alt
        local va = M.alt_law_vs(e)
        if v < 0 and va > v then v = va end
        if v > 0 and va < v then v = va end
      end
      gc = gamma_of(v, tas)
    else -- ALT
      local e = (vnav.alt_tgt or inp.alt) - inp.alt
      local v = M.alt_law_vs(e, inp.alt, tas)
      -- (v54) climbing into the level (capture) while well below the target speed: the climb is
      -- flattened so the speed is not lost (the thrust is in SPD during the capture)
      local st_ = ctx.spd_tgt
      if e > 0 and v > 0 and st_ and inp.ias and inp.ias < st_ - 10 then
        v = v * clamp(1 - (st_ - 10 - inp.ias) / 20, 0.25, 1)
      end
      gc = gamma_of(v, tas)
    end
    out.alt_tgt = vnav.alt_tgt
  elseif pm == "GS" then
    local nav = inp.nav or {}
    local gsang = nav.gs_angle or 3.0
    local gs_kt = inp.gs or 140
    local vs_gs = -tan(gsang * D2R) * gs_kt * 101.27
    local h = max(100, inp.ra or 1000)
    local dist_ft = h / tan(gsang * D2R)
    if nav.dme and nav.dme > 0.3 then dist_ft = max(dist_ft * 0.5, nav.dme * 6076.12) end
    -- beam deviation smoothed (no chasing of beam noise / small excursions)
    local vd = nav.vdef or 0
    if S.vdef_f == nil then S.vdef_f = vd end
    S.vdef_f = S.vdef_f + (vd - S.vdef_f) * min(1, dt / 1.2)
    local dev_ft = S.vdef_f * P.gs_dot_deg * D2R * dist_ft   -- + = above glideslope
    local v = vs_gs - P.k_gs * dev_ft
    if S.gs_v == nil then S.gs_v = clamp(vs, -2500, 500) end   -- from the present descent rate: smooth capture
    S.gs_v = S.gs_v + (v - S.gs_v) * min(1, dt / 1.5)
    gc = gamma_of(clamp(S.gs_v, -2500, 500), tas)
  elseif pm == "FLARE" then
    -- exponential flare on predicted height (lead compensates for the surface/airframe lag)
    local h = max(0, (inp.ra or 0) + (inp.vs or 0) / 60 * P.flare_lead)
    local v = -(h * 60 / P.flare_tau) - P.flare_td_fpm
    gc = gamma_of(v, tas)
  else
    gc = nil
  end
  if S.rollout then fixed_theta = max(0, theta - 1.0 * dt * 2) ; if not air then fixed_theta = max(0, (S.pitch_cmd or theta) - 1.0 * dt) end end

  local pc
  if fixed_theta then
    pc = fixed_theta
    S.theta_ref = nil
  elseif gc and air and not (ctx.nap and ctx.nap > 0) and pm ~= "FLARE" then
    -- FLIGHT DIRECTOR ONLY (hand flying): the bar shows, live, the attitude that gives the commanded
    -- flight path: commanded path + the angle of attack being flown (pitch - path, lightly smoothed)
    -- + a proportional term for the path error. No integrator: nothing winds up while the pilot is
    -- not on the bars, so the bar never sticks at a limit or jumps when the nose is moved.
    local a_now = clamp(theta - g_now, -5, 20)
    if S.fd_alpha == nil then S.fd_alpha = a_now end
    S.fd_alpha = S.fd_alpha + (a_now - S.fd_alpha) * min(1, dt / 0.8)
    pc = gc + S.fd_alpha + P.fd_kp * (gc - g_now)
    pc = clamp(pc, theta - 10, theta + 10)
    S.fd_only = true
    -- keep the autopilot law in step for a smooth engagement
    S.theta_ref = clamp(gc + S.fd_alpha, theta - 6, theta + 6)
  elseif gc then
    if S.theta_ref == nil then S.theta_ref = theta end
    local e = gc - g_now
    local kp, ki, lead = P.kp_g, P.ki_g, P.pitch_lead_max
    -- path modes (no speed-on-elevator loop to destabilise): stronger integral and more authority
    if pm == "GS" then
      ki = P.ki_path; lead = P.pitch_lead_path
    end
    -- level holding (ALT, VNAV ALT / level PTH): stronger integral, and the attitude change the
    -- speed change needs is fed forward - accelerating at constant altitude needs less angle of attack
    -- (alpha ~ 1/V^2), so the nose is lowered as the speed builds instead of after a climb develops
    local level = (pm == "ALT") or (pm == "VNAV" and (vnav.pitch == "ALT"))
    if level then
      ki = P.ki_alt * (1 - 0.5 * hi_alt_frac(inp.alt)) * lo_tas_k(tas)
      local alpha = clamp(theta - g_now, 0, 12)
      local v = max(100, inp.ias or 250)
      S.theta_ref = S.theta_ref - P.alpha_ff * 2 * alpha / v * (inp.ias_rate or 0) * dt
    end
    if pm == "FLARE" then kp, ki, lead = P.flare_kp, P.flare_ki, 6 end
    S.theta_ref = S.theta_ref + ki * e * dt
    S.theta_ref = clamp(S.theta_ref, theta - 6, theta + 6)
    pc = S.theta_ref + kp * e
    pc = clamp(pc, theta - lead, theta + lead)
  else
    pc = theta
    S.theta_ref = nil
  end
  pc = clamp(pc, P.pitch_min, P.pitch_max)
  -- approach modes: a 744 on an ILS never needs more than a few degrees nose down
  if pm == "GS" or pm == "FLARE" then pc = clamp(pc, -4, 12) end
  if (inp.flaps or 0) >= 20 then pc = max(pc, -5) end
  if pm == "TOGA" and not air then pc = 8.0 end
  -- command rate limit
  local prev = S.pitch_cmd or theta
  local rl = P.pitch_rate * dt
  if S.fd_only then rl = P.fd_rate * dt end
  if not S.fd_only then S.fd_alpha = nil end
  S.fd_only = nil
  if pm == "FLARE" then rl = 3.0 * dt end
  pc = clamp(pc, prev - rl, prev + rl)
  if pm == "FLARE" then pc = max(pc, prev - 0.5 * dt) end   -- no abrupt nose-down once the flare has started
  if not ctx.nap or ctx.nap == 0 then
    -- FD only: command follows (no servo); keep it close to the aircraft so re-engage is smooth
    if pm == "NONE" then pc = theta end
  end
  S.pitch_cmd = pc
  out.pitch = pc
  out.gamma_cmd = gc

  -- ROLL --------------------------------------------------------------------------------------------
  local rm = S.roll
  local blim = M.bank_limit(inp)
  local rc = 0
  local roll = inp.roll or 0
  local hdg = inp.hdg_mag or 0
  local trk = inp.trk_mag or hdg
  local rate = P.roll_rate
  if rm == "HDG_SEL" then
    rc = clamp(P.kpsi * wrap180((inp.mcp_hdg or hdg) - hdg), -blim, blim)
  elseif rm == "HDG_HOLD" then
    if S.hdg_hold == nil then
      rc = 0
      if abs(roll) < 3 then S.hdg_hold = hdg end
    else
      rc = clamp(P.kpsi * wrap180(S.hdg_hold - hdg), -blim, blim)
    end
  elseif rm == "ATT" then
    if S.att_bank == nil then S.att_bank = clamp(roll, -30, 30) end
    rc = S.att_bank
    if abs(roll) < 5 and abs(S.att_bank) < 5 then S.roll = "HDG_HOLD"; S.hdg_hold = nil; S.att_bank = nil end
  elseif rm == "LNAV" then
    local l = inp.lnav or {}
    if l.valid then
      rc = clamp(l.roll or 0, -C_LNAV_BANK, C_LNAV_BANK)
      S.lnav_hold_hdg = nil
    else
      -- discontinuity / end of route: hold heading
      if S.lnav_hold_hdg == nil then S.lnav_hold_hdg = hdg end
      rc = clamp(P.kpsi * wrap180(S.lnav_hold_hdg - hdg), -blim, blim)
    end
  elseif rm == "TOGA" then
    if not air then rc = 0
    else
      if S.track_hold == nil then S.track_hold = trk end
      rc = clamp(P.kpsi * wrap180(S.track_hold - trk), -15, 15)
    end
  elseif rm == "LOC" or rm == "ROLLOUT" then
    local nav = inp.nav or {}
    local crs = nav.crs or trk
    local d_raw = nav.hdef or 0
    if S.hdef_f == nil then S.hdef_f = d_raw end
    S.hdef_f = S.hdef_f + (d_raw - S.hdef_f) * min(1, dt / 1.0)
    local d = S.hdef_f
    local dist = nav.dme and nav.dme > 0.2 and nav.dme or max(0.5, (max(50, inp.ra or 1000) / tan(3 * D2R)) / 6076.12 + 1.5)
    local xtk = -dist * tan(clamp(d * P.loc_dot_deg, -10, 10) * D2R)   -- nm, + right of course
    local gs = max(60, inp.gs or 150)
    local K = P.kloc
    local L = max(0.4, P.loc_la * (gs * 0.5144) ^ 2 / (GRAV * K) / 1852)
    local ia = clamp(-atan(xtk / L) * R2D, -45, 45)
    local dtk = crs + ia
    rc = clamp(K * wrap180(dtk - trk), -blim, blim)
    rate = P.roll_rate_loc
    -- capture turn: while still on the intercept heading (track more than 4 deg off the course and
    -- converging), hold the heading until the turn must start, then fly the bank that makes the turn
    -- end exactly on the centreline (radius from the remaining offset and intercept angle; the offset
    -- is predicted ahead by the roll-in time). Wind is included as it works on ground track/speed.
    local th = wrap180(trk - crs)
    local v = gs * 0.5144
    local xc = nav.xtk_geo or xtk     -- runway-geometry offset when known (independent of beam width)
    if abs(th) > 4 and xc * th < 0 and not S.flare and rm == "LOC" then
      xtk = xc
      -- before the turn: lead by the roll-in time; once turning only by the response lag
      local t_lead = S.loc_turn and 1.0 or (P.loc_cap_bank / (2 * P.roll_rate_loc) + 1.5)
      local xp = abs(xtk) - gs / 3600 * t_lead * abs(sin(th * D2R))
      local need = 1 - cos(abs(th) * D2R)
      local bank_req = 0
      if xp > 0.001 then
        local R = xp * 1852 / need
        bank_req = atan(v * v / (GRAV * R)) * R2D
      else
        bank_req = blim
      end
      -- already turning towards the centreline (e.g. LNAV turn onto final): continue, do not level out
      if S.loc_turn == nil and roll * (th > 0 and -1 or 1) > 2 then S.loc_turn = true end
      if S.loc_turn or bank_req >= P.loc_cap_bank * 0.9 then
        S.loc_turn = true
        rc = -(th > 0 and 1 or -1) * clamp(bank_req, 0, blim)
      else
        rc = 0          -- keep the intercept heading/track until the turn point
      end
    else
      S.loc_turn = nil
      -- established: gentle corrections only
      if rm == "LOC" and abs(th) < 6 then rc = clamp(rc, -12, 12) end
    end
    if (inp.ra or 9999) < 200 then rc = clamp(rc, -8, 8) end
    if S.flare or rm == "ROLLOUT" then
      -- align (decrab) with rudder and rollout steering on the localizer
      local hdg_err = wrap180(crs - hdg)
      local rud = 0.06 * hdg_err - 0.25 * d_raw
      if not air then rud = 0.08 * hdg_err - 0.35 * d_raw - 0.0 end
      out.rudder = clamp(rud, -1, 1)
      if not air then rc = 0 end
    end
  else
    rc = 0
  end
  if rm ~= "ATT" then S.att_bank = nil end
  local prevr = S.roll_cmd or roll
  if rm == "NONE" then prevr = roll end
  local loc_trk = (rm == "LOC") and not S.loc_turn and not S.flare
  if (rm == "LNAV" or rm == "HDG_SEL" or rm == "HDG_HOLD" or loc_trk) and air then
    -- smooth roll: the bank target is low-pass filtered (no chasing of every small correction) and
    -- the roll rate builds up / dies away gradually (roll acceleration limit) - the aircraft rolls
    -- to a bank and holds it instead of making many small jerky corrections
    if S.rc_f == nil then S.rc_f = prevr; S.roll_rate_cmd = 0 end
    S.rc_f = S.rc_f + (rc - S.rc_f) * min(1, dt / (loc_trk and 1.0 or P.roll_tau))
    local want = clamp((S.rc_f - prevr) / 0.6, -rate, rate)
    local cur = S.roll_rate_cmd or 0
    cur = clamp(want, cur - P.roll_acc * dt, cur + P.roll_acc * dt)
    S.roll_rate_cmd = cur
    rc = prevr + cur * dt
  else
    S.rc_f = nil; S.roll_rate_cmd = nil
    local rr = rate * dt
    rc = clamp(rc, prevr - rr, prevr + rr)
  end
  S.roll_cmd = rc
  out.roll = rc

  -- A/T SPD (lever target for the EEC) ---------------------------------------------------------------
  -- speed -> commanded acceleration (time constant 6 s, 4 s on approach), acceleration error
  -- integrated into the lever (the lever moves until the aircraft accelerates as wanted), plus a
  -- feed-forward of flight path changes (thrust for climb/descent gradient: pitch-over onto the G/S,
  -- level-off) so the speed does not wait for an error to build up. Speed trend is the inertial-
  -- complemented IAS rate (little gust noise).
  if S.at == "SPD" and S.at_engaged and air and not S.flare then
    local V = inp.ias or 0
    local Vt = ctx.spd_tgt or V
    local appr = (inp.flaps or 0) >= 20
    local e = Vt - V
    local tau_v = appr and P.at_tau_appr or P.at_tau
    -- small errors: gentle (about 1 kt/s); large errors (e.g. accelerating to cruise speed):
    -- up to the thrust limit - the lever goes to the FMC maximum thrust until the speed nears target
    local amax = appr and 0.9 or 3.0
    local acmd = clamp(e / tau_v, -amax, amax)
    local a = inp.ias_rate or 0
    local m = max(150000, inp.gw or 300000)
    local lapse = clamp(1 - (inp.alt or 0) / 55000, 0.3, 1)
    local tmax = 4 * 250000 * lapse                       -- N, lever 1.0
    local Ga = tmax / m * 1.944                           -- kt/s per unit lever
    if S.spd_u == nil then S.spd_u = inp.thr_lever or 0.5; S.spd_g = g_now; S.spd_gp = g_now end
    S.spd_g = S.spd_g + (g_now - S.spd_g) * min(1, dt / 1.0)
    local dg = S.spd_g - S.spd_gp
    S.spd_gp = S.spd_g
    local ff = dg * D2R * m * GRAV / tmax
    S.spd_u = clamp(S.spd_u + (acmd - a) / (Ga * P.at_tau_acc) * dt + ff, 0, 1)
    out.thr = S.spd_u
  else
    S.spd_u = nil
  end

  -- A/T THR (lever target for the EEC) ---------------------------------------------------------------
  if S.at == "THR" and S.at_engaged then
    local tgt_vs
    if S.pitch == "FLCH" then
      if (S.flch_dir or 1) > 0 then
        local dh = (inp.mcp_alt or inp.alt) - inp.alt
        tgt_vs = clamp(dh * 1.0, 1000, 6000)
      else
        tgt_vs = nil
      end
    elseif S.pitch == "TOGA" and S.toga == "GA" then
      tgt_vs = 2000
    end
    if tgt_vs then
      S.thr_int = clamp((S.thr_int or (inp.thr_lever or 0.6)) + 0.00004 * (tgt_vs - vs) * dt * 20, 0.05, 1.0)
      out.thr = S.thr_int
    else
      out.thr = 0
      S.thr_int = inp.thr_lever or 0
    end
  else
    S.thr_int = inp.thr_lever or 0.6
  end
  return out
end

B744LAWS = M
