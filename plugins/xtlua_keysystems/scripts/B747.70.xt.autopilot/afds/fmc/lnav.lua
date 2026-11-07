--[[ B744 FMC lateral navigation guidance.
     Tracks the active leg's path elements, sequences legs (fixed fixes at the turn bisector,
     altitude/DME/radial/intercept terminations, holds), computes XTK / track error and the
     LNAV roll command (track-error law with curvature feed-forward, wind is handled because
     the law works on ground track).  Pure Lua; registered as global B744LNAV. ]]
local M = {}
local G = B744GEO
local PA = B744PATH
local RT = B744ROUTE
local abs, min, max, floor, atan, tan, sqrt, pi, cos, sin = math.abs, math.min, math.max, math.floor, math.atan, math.tan, math.sqrt, math.pi, math.cos, math.sin
local D2R, R2D = pi / 180, 180 / pi
local GRAV = 9.80665
local KROLL = 1.5           -- deg bank per deg of track error
M.KROLL = KROLL
M.ROLL_RATE = 3.0      -- deg/s, AFDS roll command rate limit
M.ROLL_LAG = 1.5       -- s, aircraft roll response lag
M.LA = 2.0            -- lookahead factor (cross-track loop time constant / track loop time constant)

function M.new_state() return { ei = 1, hold = nil, last_active = 0, seq_msg = nil } end

-- first leg to fly after the runway
function M.first_active(plan)
  for i, l in ipairs(plan.legs) do
    if l.t ~= "IF" and l.t ~= "DISCO" and l.seg ~= "RWY" then return i end
  end
  return 0
end

local function lookahead_nm(gs_kt)
  local v = max(60, gs_kt) * 0.514444
  return max(0.6, M.LA * v * v / (GRAV * M.KROLL) / 1852)
end
M.lookahead_nm = lookahead_nm

-- evaluate guidance on one element
local function eval_elem(e, ac)
  local atk, xtk, ptrk, rem = PA.elem_track(e, ac.lat, ac.lon)
  return atk, xtk, ptrk, rem
end

local function bank_for_radius(gs, r, dir)
  local v = gs * 0.514444
  return dir * atan(v * v / (max(0.1, r) * 1852 * GRAV)) * R2D
end

-- command for element
local function steer(e, ac, xtk, ptrk, bank_lim)
  local gs = max(80, ac.gs or 250)
  if e.k == "H" then
    local err = G.angdiff(e.hdg, ac.hdg_true or ac.trk_true)
    return max(-bank_lim, min(bank_lim, M.KROLL * err)), e.hdg
  end
  local L = lookahead_nm(gs)
  local ia = -atan(xtk / L) * R2D
  -- v34: slow cross-track trim (wind/steady offset of a few tenths of a nm), only close to the track
  local dt = min(0.5, max(0, ac.dt or 0.05))
  if abs(xtk) < 0.5 and not ac.onground then
    M._xi = (M._xi or 0) * (1 - 0.01 * dt) - xtk * 0.8 * dt
    if M._xi > 4 then M._xi = 4 elseif M._xi < -4 then M._xi = -4 end
  else
    M._xi = 0
  end
  ia = ia + M._xi
  if ia > 45 then ia = 45 elseif ia < -45 then ia = -45 end
  local dtk = G.wrap360(ptrk + ia)
  local ff = 0
  if e.k == "A" then ff = bank_for_radius(gs, e.r, e.dir) end
  local err = G.angdiff(dtk, ac.trk_true)
  local roll = ff + M.KROLL * err
  return max(-bank_lim, min(bank_lim, roll)), dtk
end

-- termination checks for dynamic legs
local function dyn_done(l, ac, plan, i)
  local t = l.t
  if t == "CA" or t == "FA" or t == "VA" then
    local tgt = l.alt and l.alt.a or 0
    return (ac.alt or 0) >= tgt - 20
  elseif t == "CD" or t == "VD" or t == "FD" then
    if not l.nav then return false end
    local d = G.dist(ac.lat, ac.lon, l.nav.lat, l.nav.lon)
    local e = l.elems[1]
    local d0 = e and G.dist(e.a.lat, e.a.lon, l.nav.lat, l.nav.lon) or 0
    if d0 < (l.dist or 0) then return d >= (l.dist or 0) else return d <= (l.dist or 0) end
  elseif t == "CR" or t == "VR" then
    if not l.nav or not l.theta then return false end
    local rad = (l.nav.slaved_var and G.wrap360(l.theta + l.nav.slaved_var)) or G.mag_to_true(l.theta, l.nav.lat, l.nav.lon)
    local b = G.brg(l.nav.lat, l.nav.lon, ac.lat, ac.lon)
    local e = l.elems[1]
    local b0 = e and G.brg(l.nav.lat, l.nav.lon, e.a.lat, e.a.lon) or b
    local d0, d1 = G.angdiff(b0, rad), G.angdiff(b, rad)
    return (d0 >= 0) ~= (d1 >= 0) and abs(d1) < 30
  elseif t == "CI" or t == "VI" then
    local nl = plan.legs[i + 1]
    if not nl or not nl.elems or not nl.elems[1] then return true end
    local ne = nl.elems[1]
    if ne.k == "A" and nl.elems[2] then ne = nl.elems[2] end
    local _, x, ptrk = PA.elem_track(ne, ac.lat, ac.lon)
    local dlt = abs(G.angdiff(ptrk, ac.trk_true))
    local r = G.turn_radius(max(120, ac.gs or 200), 25)
    local lead = r * (1 - cos(min(90, dlt) * D2R)) + 0.1
    return abs(x) <= lead
  elseif t == "VM" or t == "FM" then
    return false
  end
  return false
end

-- ACTIVE LEG UPDATE + GUIDANCE ----------------------------------------------------------------------
--[[ ac = { lat, lon, trk_true, hdg_true, gs, alt, onground, goaround, bank_lim }
     returns out = { valid, roll, xtk, tke, dtk, active, dtg (to active wpt), disco, eor, msg, s_now, cap_ok, intc } ]]
function M.update(st, plan, ac)
  local out = { valid = false, roll = 0, xtk = 0, tke = 0, active = plan.active or 0 }
  local legs = plan.legs
  local n = #legs
  if n == 0 or not plan.activated then return out end
  local bank_lim = ac.bank_lim or 25
  if (plan.active or 0) < 1 then plan.active = M.first_active(plan) end
  if plan.active < 1 or plan.active > n then out.eor = true return out end
  if st.last_active ~= plan.active then st.ei = 1; st.last_active = plan.active; st.hold = nil end
  local last = RT.last_route_index(plan)

  local guard = 0
  local i, l, e, atk, xtk, ptrk, rem
  while guard < 20 do
    guard = guard + 1
    i = plan.active
    l = legs[i]
    if not l then out.eor = true return out end
    if l.t == "DISCO" then
      out.disco = true; out.active = i
      out.msg = "DISCONTINUITY"
      out.s_now = (legs[i - 1] and legs[i - 1].cum) or 0
      return out
    end
    if (l.t == "IF") then
      plan.active = i + 1; st.ei = 1
    elseif not l.elems or #l.elems == 0 then
      -- leg without geometry (zero length) -> sequence
      if i >= n then out.eor = true return out end
      plan.active = i + 1; st.ei = 1
    else
      if st.ei > #l.elems then st.ei = #l.elems end
      e = l.elems[st.ei]
      -- dynamic course legs re-anchor at activation (fly the course from present position)
      if l.t == "CA" or l.t == "CD" or l.t == "CR" or l.t == "CI" or l.t == "VA" or l.t == "VD" or l.t == "VR" or l.t == "VI" or l.t == "VM" then
        if not st.anchored then
          if e.k == "L" or e.k == "H" then
            local la, lo = G.dest(ac.lat, ac.lon, e.crs_fixed or e.crs or e.hdg, 30)
            if l.t ~= "CA" or (ac.onground or (ac.alt or 0) - (ac.elev or 0) > 50) then
              -- keep the runway centerline for CA off the runway until airborne
            end
            -- (the guidance line is LNAV's own copy: the path element - drawn on the ND from the leg's
            -- start to its predicted end - is not changed)
            if e.k == "H" then
              st.hanch = { leg = i, ei = st.ei, e = { k = "H", a = { lat = ac.lat, lon = ac.lon }, b = { lat = la, lon = lo },
                hdg = e.hdg, crs = e.crs, len = 30, leg = e.leg, dyn = e.dyn } }
            end
          end
          st.anchored = true
        end
        -- (a heading element is steered on its heading alone; its along-track distance - the distance
        -- still to go and the profile position - is measured on the path element itself, from the leg's
        -- start to its predicted end, as drawn on the ND)
      else
        st.anchored = nil
        st.hanch = nil
      end
      atk, xtk, ptrk, rem = eval_elem(e, ac)
      -- hold handling
      if (l.t == "HM" or l.t == "HA" or l.t == "HF") and l.hold and l.hold.circuit then
        return M.hold_guidance(st, plan, ac, i, l, out, bank_lim)
      end
      local done = false
      if PA.DYN[l.t] then
        if st.ei < #l.elems and rem <= 0 then st.ei = st.ei + 1
        elseif dyn_done(l, ac, plan, i) then done = true end
        -- course legs past their predicted end keep flying the course (extend)
      else
        if rem <= 0.02 then
          if st.ei < #l.elems then
            st.ei = st.ei + 1
          else
            done = true
          end
        end
      end
      -- missed approach only after go-around
      if done and i >= last and not ac.goaround then
        out.eor = (i >= last)
        done = false
        if rem <= 0 then
          out.msg = "END OF ROUTE"
          out.valid = false
          out.active = i
          return out
        end
      end
      if done then
        if i >= n then
          out.eor = true; out.msg = "END OF ROUTE"
          out.active = i
          return out
        end
        plan.active = i + 1
        st.ei = 1; st.anchored = nil
        out.sequenced = true
        local nx = legs[i + 1]
        if nx and nx.elems and nx.elems[1] and nx.elems[1].trans then
          -- second half of the transition arc already being flown
        end
      else
        break
      end
    end
  end
  if not e then return out end
  local roll, dtk = steer(e, ac, xtk, ptrk, bank_lim)
  -- lead into the next element's curvature (roll-in anticipation)
  local nxt = l.elems[st.ei + 1]
  if not nxt and legs[i + 1] and legs[i + 1].elems then nxt = legs[i + 1].elems[1] end
  -- turn anticipation: the roll command is rate limited (M.ROLL_RATE deg/s) and the aircraft lags it
  -- (M.ROLL_LAG s), so the bank for the arc is commanded early enough that the roll-in is centred on
  -- the start of the arc (and the roll-out on its end). Uses the actual ground speed (wind included).
  local gsn = max(80, ac.gs or 250)
  local function lead_nm(bank) return gsn / 3600 * (min(bank_lim, abs(bank)) / (2 * M.ROLL_RATE) + M.ROLL_LAG) end
  if nxt and nxt.k == "A" and e.k ~= "A" then
    local ff = bank_for_radius(gsn, nxt.r, nxt.dir)
    local lead = lead_nm(ff)
    if rem < lead then
      roll = max(-bank_lim, min(bank_lim, roll + ff))
    end
  elseif e.k == "A" and nxt and nxt.k ~= "A" then
    local ff = bank_for_radius(gsn, e.r, e.dir)
    local lead = lead_nm(ff)
    if rem < lead then
      roll = roll - ff
    end
  elseif e.k == "A" and nxt and nxt.k == "A" then
    -- arc to arc (DME arc into a fly-by turn, S-turns): the bank change (a reversal from one side to
    -- the other on an S-turn) is started early enough to be centred on the junction
    local f0 = bank_for_radius(gsn, e.r, e.dir)
    local f1 = bank_for_radius(gsn, nxt.r, nxt.dir)
    local db = min(2 * bank_lim, abs(f1 - f0))
    local lead = gsn / 3600 * (db / (2 * M.ROLL_RATE) + M.ROLL_LAG)
    if rem < lead and db > 2 then
      roll = max(-bank_lim, min(bank_lim, roll - f0 + f1))
    end
  end
  out.valid = true
  -- in or approaching a turn: the path geometry must not be rebuilt under the aircraft
  out.turn = (e.k == "A") or (nxt ~= nil and nxt.k == "A" and rem < lead_nm(25) * 2 + 1)
  out.roll = roll
  out.xtk = xtk
  out.dtk = dtk
  out.ptrk = ptrk
  out.tke = G.angdiff(ac.trk_true, ptrk)
  out.active = plan.active
  out.ei = st.ei
  -- distance to go on active leg (along path)
  local dtg = max(0, rem or 0)
  for k = st.ei + 1, #l.elems do dtg = dtg + (l.elems[k].len or 0) end
  if PA.DYN[l.t] then dtg = max(dtg, 0) end
  out.dtg = dtg
  out.s_now = (l.cum or 0) - dtg
  -- capture criteria (for LNAV arming)
  local r = G.turn_radius(max(120, ac.gs or 200), 25)
  local te = abs(out.tke)
  out.cap_ok = abs(xtk) <= 2.5 or abs(xtk) <= r * (1 - cos(min(90, te) * D2R)) + 0.3
  local conv = (xtk > 0 and G.angdiff(ac.trk_true, ptrk) < -1) or (xtk < 0 and G.angdiff(ac.trk_true, ptrk) > 1)
  out.intc = conv and te < 160
  if e.k == "H" then out.cap_ok = true; out.intc = true end
  return out
end

-- HOLDS -------------------------------------------------------------------------------------------------
-- entry (direct / teardrop / parallel) then racetrack circuits
function M.hold_guidance(st, plan, ac, i, l, out, bank_lim)
  local h = l.hold
  local hs = st.hold
  local f = h.pts.f
  if not hs then
    hs = { phase = "TOFIX", circuits = 0 }
    st.hold = hs
  end
  local gs = max(120, ac.gs or 220)
  local function steer_to(e)
    local atk, xtk, ptrk, rem = PA.elem_track(e, ac.lat, ac.lon)
    local roll, dtk = steer(e, ac, xtk, ptrk, bank_lim)
    return roll, rem, xtk, ptrk, dtk
  end
  local roll, rem, xtk, ptrk, dtk
  if hs.phase == "TOFIX" then
    local d = G.dist(ac.lat, ac.lon, f.lat, f.lon)
    local b = G.brg(ac.lat, ac.lon, f.lat, f.lon)
    local e = { k = "L", a = { lat = ac.lat, lon = ac.lon }, b = f, len = d, crs = b }
    if l.elems[1] and not l.elems[1].hold and st.ei == 1 then e = l.elems[1] end
    roll, rem, xtk, ptrk, dtk = steer_to(e)
    if rem <= 0.05 or d < 0.3 then
      -- arrival at the fix: choose entry by the angle between our track and the inbound course
      local rel = G.angdiff(ac.trk_true, h.inbd_true) * h.dir
      if rel >= -70 and rel <= 110 then hs.phase = "CIRCUIT"; hs.k = 1
      elseif rel > 110 then hs.phase = "TEARDROP"; hs.t0 = ac.t or 0
      else hs.phase = "PARALLEL"; hs.t0 = ac.t or 0 end
      hs.entered = true
      if l.t == "HF" then hs.exit_at_fix = true end
    end
  elseif hs.phase == "TEARDROP" then
    -- fly 30 deg offset outbound on the holding side for one leg time, then turn in
    local hdg = G.wrap360(h.inbd_true + 180 - 30 * h.dir)
    local err = G.angdiff(hdg, ac.trk_true)
    roll = max(-bank_lim, min(bank_lim, M.KROLL * err))
    hs.dist = (hs.dist or 0) + gs / 3600 * (ac.dt or 0.05)
    if hs.dist >= h.legd then hs.phase = "CIRCUIT"; hs.k = 4; hs.dist = 0 end
    rem = 1
  elseif hs.phase == "PARALLEL" then
    local hdg = G.wrap360(h.inbd_true + 180)
    local err = G.angdiff(hdg, ac.trk_true)
    roll = max(-bank_lim, min(bank_lim, M.KROLL * err))
    hs.dist = (hs.dist or 0) + gs / 3600 * (ac.dt or 0.05)
    if hs.dist >= h.legd then
      -- turn toward the holding side back to the fix
      hs.phase = "PARRET"; hs.dist = 0
    end
    rem = 1
  elseif hs.phase == "PARRET" then
    local b = G.brg(ac.lat, ac.lon, f.lat, f.lon)
    local err = G.angdiff(b, ac.trk_true)
    roll = max(-bank_lim, min(bank_lim, M.KROLL * err))
    if abs(err) < 20 then hs.phase = "PARFIX" end
    rem = 1
  elseif hs.phase == "PARFIX" then
    local d = G.dist(ac.lat, ac.lon, f.lat, f.lon)
    local b = G.brg(ac.lat, ac.lon, f.lat, f.lon)
    local err = G.angdiff(b, ac.trk_true)
    roll = max(-bank_lim, min(bank_lim, M.KROLL * err))
    if d < 0.4 then hs.phase = "CIRCUIT"; hs.k = 1 end
    rem = 1
  else -- CIRCUIT
    local e = h.circuit[hs.k]
    roll, rem, xtk, ptrk, dtk = steer_to(e)
    if rem <= 0.03 then
      hs.k = hs.k + 1
      if hs.k > 4 then
        hs.k = 1
        hs.circuits = hs.circuits + 1
        -- at the fix
        local exit = false
        if h.exit or hs.exit_at_fix then exit = true end
        if l.t == "HA" and l.alt and abs((ac.alt or 0) - l.alt.a) < 200 then exit = true end
        if exit then
          if plan.legs[i + 1] then
            plan.active = i + 1; st.ei = 1; st.hold = nil
            out.sequenced = true
          end
        end
      end
    end
  end
  out.valid = true
  out.roll = roll or 0
  out.xtk = xtk or 0
  out.ptrk = ptrk or h.inbd_true
  out.tke = ptrk and G.angdiff(ac.trk_true, ptrk) or 0
  out.active = plan.active
  out.dtg = G.dist(ac.lat, ac.lon, f.lat, f.lon)
  out.s_now = (l.cum or 0) - out.dtg
  out.in_hold = true
  out.cap_ok = true
  out.intc = true
  return out
end

B744LNAV = M
