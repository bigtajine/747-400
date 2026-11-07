--[[ B747-400 AFDS (autopilot / flight director / autothrottle mode logic) — rebuilt.
     Pure Lua (no X-Plane API): the IO script fills `inp`, queues button events with B744AFDS.event(),
     calls B744AFDS.step() every frame and writes `out` to datarefs.
     Behaviour follows the 747-400 FCOM (chapter 4 Automatic Flight) as closely as the sim allows.
     Registered as global B744AFDS. ]]
local M = {}
local abs, min, max, floor, sqrt, atan, asin, sin, cos, tan, pi =
  math.abs, math.min, math.max, math.floor, math.sqrt, math.atan, math.asin, math.sin, math.cos, math.tan, math.pi
local D2R, R2D = pi / 180, 180 / pi

-- FMA codes (unchanged from the stock cockpit displays)
M.ROLL = { NONE = 0, TOGA = 1, LNAV = 2, LOC = 3, ROLLOUT = 4, ATT = 5, HDG_SEL = 6, HDG_HOLD = 7 }
M.PITCH = { NONE = 0, TOGA = 1, GS = 2, FLARE = 3, VNAV_SPD = 4, VNAV_ALT = 5, VNAV_PTH = 6, VS = 7, FLCH = 8, ALT = 9 }
M.AT = { NONE = 0, HOLD = 1, IDLE = 2, SPD = 3, THR = 4, THR_REF = 5 }
M.STATUS = { NONE = 0, FD = 1, CMD = 2, LAND2 = 3, LAND3 = 4, NO_AUTOLAND = 5 }

M.CFG = {
  ap_engage_ra = 250,     -- ft RA after takeoff before CMD can engage
  lnav_engage_ra = 50,
  vnav_engage_ra = 400,
  toga_exit_ra = 400,
  at_hold_kt = 65,        -- THR REF -> HOLD on the takeoff roll
  land_ra = 1500,         -- LAND 2/3, FLARE/ROLLOUT armed
  flare_ra = 50,
  retard_ra = 25,
  rollout_ra = 5,
  mode_box_s = 10,
  bank_max = 25,
  cap_a = 1.6,            -- ft/s^2 capture normal acceleration (~0.05 g)
}
local C = M.CFG

local function clamp(x, a, b) if x < a then return a elseif x > b then return b end return x end
M.clamp = clamp
local function wrap180(a) a = (a + 180) % 360 - 180 return a end
local function wrap360(a) return a % 360 end

function M.new()
  return {
    events = {},
    ap = { false, false, false },    -- L C R
    ap_on = false,
    roll = "NONE", roll_arm = "NONE",
    pitch = "NONE", pitch_arm = "NONE",
    lnav_arm = false, vnav_arm = false, loc_arm = false, gs_arm = false, app = false,
    at = "NONE", at_engaged = false,
    hdg_hold = nil, alt_hold = nil, track_hold = nil,
    vs_win = false, ias_win = true, spd_intv = false,
    toga = nil,              -- "TO" or "GA"
    toga_presses = 0,
    ga_thr_ref = false,
    fd_prev = false,
    status = "NONE",
    land_status = nil,
    chg = { roll = -100, pitch = -100, at = -100, status = -100 },
    prev = { roll = "NONE", pitch = "NONE", at = "NONE", status = "NONE" },
    theta_ref = nil,         -- pitch law integrator
    roll_cmd = 0, pitch_cmd = 0,
    flare = false, rollout = false, retard = false,
    warn_ap = false, warn_at = false,
    t = 0,
    liftoff_t = nil, liftoff_spd = nil,
    alt_push = 0, des_now = 0,
    msg = {},
    flight_phase = 0,
    thr_int = 0.5,
    was_onground = true,
  }
end

function M.event(S, name, a) S.events[#S.events + 1] = { name, a } end

-- helpers -----------------------------------------------------------------------------------------
local function n_ap(S) local n = 0 for i = 1, 3 do if S.ap[i] then n = n + 1 end end return n end
M.n_ap = n_ap
local function fd_on(inp) return (inp.fd_capt == 1) or (inp.fd_fo == 1) end

local function set_roll(S, m) if S.roll ~= m then S.roll = m end end
local function set_pitch(S, m) if S.pitch ~= m then S.pitch = m; S.theta_ref = nil end end

local function engage_at(S, inp, mode)
  if inp.at_arm_sw == 1 and not inp.at_fail then
    S.at_engaged = true
    S.at = mode
  end
end

local function vnav_engage(S, inp)
  if inp.vnav and inp.vnav.valid then
    S.vnav_arm = false
    set_pitch(S, "VNAV")
    S.toga = (S.toga == "TO" or S.toga == "GA") and (S.roll == "TOGA" and S.toga or nil) or nil
    engage_at(S, inp, "VNAV")
    return true
  end
  S.msg.vnav_unavail = true
  return false
end

local function basic_modes(S, inp)
  -- default modes when the first FD is switched on / AP engaged with no FD: HDG HOLD or ATT, V/S
  if abs(inp.roll) > 5 then set_roll(S, "ATT") else set_roll(S, "HDG_HOLD"); S.hdg_hold = nil end
  set_pitch(S, "VS")
  S.vs_target = floor((inp.vs or 0) / 100 + 0.5) * 100
  S.vs_win = true
  S.vs_sync = true
end

local function leave_vnav(S)
  if S.pitch == "VNAV" then S.pitch = "NONE" end
  S.vnav_arm = false
  S.spd_intv = false
end

local function open_ias_window(S, inp)
  if not S.ias_win then
    S.ias_win = true
    S.ias_sync = true      -- IO copies the current/target speed into the window
  end
end

-- EVENTS -------------------------------------------------------------------------------------------
local function handle(S, inp, ev, arg)
  local air = not inp.onground
  local ra = inp.ra or 0
  if ev == "CMD" then
    local i = arg
    if inp.bar == 1 then return end
    if not air or ra < C.ap_engage_ra then return end
    local multi = S.app or S.roll == "LOC" or S.pitch == "GS"
    if S.ap[i] then
      return
    end
    if not multi then
      for k = 1, 3 do S.ap[k] = false end
    end
    local first = n_ap(S) == 0
    S.ap[i] = true
    S.warn_ap = false
    if first then
      if not fd_on(inp) or (S.roll == "NONE" and S.pitch == "NONE") then
        basic_modes(S, inp)
      end
      if S.roll == "NONE" then if abs(inp.roll) > 5 then set_roll(S, "ATT") else set_roll(S, "HDG_HOLD"); S.hdg_hold = nil end end
      if S.pitch == "NONE" then set_pitch(S, "VS"); S.vs_target = floor((inp.vs or 0) / 100 + 0.5) * 100; S.vs_win = true end
    end
  elseif ev == "DISCONNECT" then
    if n_ap(S) > 0 then S.warn_ap = true else S.warn_ap = false end
    for k = 1, 3 do S.ap[k] = false end
    S.land_status = nil
    if not fd_on(inp) then
      S.roll, S.pitch = "NONE", "NONE"; S.lnav_arm = false; S.vnav_arm = false
      S.loc_arm = false; S.gs_arm = false; S.app = false
    end
  elseif ev == "WARN_RESET" then
    S.warn_ap = false
  elseif ev == "LNAV" then
    if S.lnav_arm or (S.roll_arm == "LNAV") then
      S.lnav_arm = false                          -- second push disarms
    elseif S.roll ~= "LNAV" then
      if not (inp.lnav and inp.lnav.route) then S.msg.no_route = true return end
      S.lnav_arm = true
      if S.roll ~= "LOC" and S.pitch ~= "GS" then end
    end
  elseif ev == "VNAV" then
    if S.pitch == "VNAV" then return end
    if S.vnav_arm then S.vnav_arm = false return end
    if not (inp.vnav and inp.vnav.valid) then S.msg.vnav_unavail = true return end
    if not air or ra < C.vnav_engage_ra then S.vnav_arm = true
    else
      if S.pitch == "GS" or S.pitch == "FLARE" then return end
      vnav_engage(S, inp)
    end
  elseif ev == "FLCH" then
    if not air then return end
    if S.pitch == "GS" or S.pitch == "FLARE" then return end
    leave_vnav(S)
    set_pitch(S, "FLCH")
    if S.toga then S.toga = nil; if S.roll == "TOGA" then set_roll(S, "HDG_HOLD"); S.hdg_hold = nil end end
    open_ias_window(S, inp)
    S.flch_dir = (inp.mcp_alt >= inp.alt) and 1 or -1
    if abs(inp.mcp_alt - inp.alt) < 150 then set_pitch(S, "ALT"); S.alt_hold = inp.mcp_alt end
    engage_at(S, inp, "THR")
    S.thr_hold = false
  elseif ev == "VS" then
    if not air then return end
    if S.pitch == "GS" or S.pitch == "FLARE" then return end
    leave_vnav(S)
    set_pitch(S, "VS")
    S.vs_target = floor((inp.vs or 0) / 100 + 0.5) * 100
    S.vs_win = true
    S.vs_sync = true
    if S.toga then S.toga = nil; if S.roll == "TOGA" then set_roll(S, "HDG_HOLD"); S.hdg_hold = nil end end
    open_ias_window(S, inp)
    if S.at_engaged or inp.at_arm_sw == 1 then if S.at_engaged then S.at = "SPD" end end
  elseif ev == "ALT_HOLD" then
    if not air then return end
    if S.pitch == "GS" or S.pitch == "FLARE" then return end
    leave_vnav(S)
    set_pitch(S, "ALT")
    S.alt_hold = floor(inp.alt / 10 + 0.5) * 10
    if S.toga then S.toga = nil; if S.roll == "TOGA" then set_roll(S, "HDG_HOLD"); S.hdg_hold = nil end end
    open_ias_window(S, inp)
    if S.at_engaged then S.at = "SPD" end
  elseif ev == "HDG_SEL" then
    if not air then return end
    if S.roll == "LOC" and S.pitch == "GS" then return end
    if S.roll == "ROLLOUT" then return end
    set_roll(S, "HDG_SEL"); S.lnav_arm = false
    if S.roll_arm == "LOC" and not S.app then end
    if S.toga then S.toga_roll_end = true end
  elseif ev == "HDG_HOLD" then
    if not air then return end
    if S.roll == "LOC" and S.pitch == "GS" then return end
    if S.roll == "ROLLOUT" then return end
    set_roll(S, "HDG_HOLD"); S.hdg_hold = nil; S.lnav_arm = false
  elseif ev == "LOC" then
    if S.pitch == "GS" then return end
    if S.loc_arm and not S.app then S.loc_arm = false
    elseif S.roll == "LOC" and not S.app then
      -- LOC captured: cancel only by selecting another roll mode
    else
      S.loc_arm = true; S.app = false; S.gs_arm = false
    end
  elseif ev == "APP" then
    if S.pitch == "GS" and S.roll == "LOC" then
      if ra > C.land_ra then S.app = false; S.gs_arm = false end
      return
    end
    if S.app then
      S.app = false; S.gs_arm = false
      if S.roll ~= "LOC" then S.loc_arm = false end
    else
      S.app = true; S.loc_arm = (S.roll ~= "LOC"); S.gs_arm = true
    end
  elseif ev == "SPD" then
    if inp.at_arm_sw == 1 then
      S.at_engaged = true; S.at = "SPD"
      open_ias_window(S, inp)
    end
  elseif ev == "THR" then
    if inp.at_arm_sw == 1 and air then S.at_engaged = true; S.at = "THR_REF" end
  elseif ev == "AT_DISC" then
    if S.at_engaged then S.at_engaged = false; S.at = "NONE"; S.warn_at = true
    else S.warn_at = false end
  elseif ev == "SPD_INTV" then
    if S.pitch == "VNAV" or S.at == "VNAV" then
      S.spd_intv = not S.spd_intv
      if S.spd_intv then S.ias_win = true; S.ias_sync = true else S.ias_win = false end
    end
  elseif ev == "ALT_INTV" then
    S.alt_push = S.alt_push + 1
    if S.pitch == "ALT" and S.alt_hold and abs(inp.mcp_alt - inp.alt) > 200 and S.vnav_was then
      -- nothing: ALT hold is not left by the knob outside VNAV
    end
  elseif ev == "TOGA" then
    if not air then
      -- takeoff: THR REF, TO/GA roll/pitch (if FDs on)
      if inp.at_arm_sw == 1 then S.at_engaged = true; S.at = "THR_REF"; S.to_thrust = true end
      if fd_on(inp) then S.toga = "TO"; set_roll(S, "TOGA"); set_pitch(S, "TOGA") end
    else
      local armed = (inp.flaps or 0) > 0 or S.pitch == "GS" or S.pitch == "FLARE"
      if not armed then return end
      if S.toga == "GA" then
        S.ga_thr_ref = true
        if S.at_engaged or inp.at_arm_sw == 1 then S.at_engaged = true; S.at = "THR_REF" end
      else
        S.toga = "GA"; S.ga_thr_ref = false
        set_roll(S, "TOGA"); set_pitch(S, "TOGA")
        S.track_hold = inp.trk_mag
        S.ga_spd = max(inp.mcp_spd or 0, inp.ias or 0)
        S.app = false; S.loc_arm = false; S.gs_arm = false; S.lnav_arm = false; S.vnav_arm = false
        S.flare = false; S.rollout = false; S.retard = false
        S.goaround = true
        S.land_status = nil
        if inp.at_arm_sw == 1 then S.at_engaged = true; S.at = "THR" end
      end
    end
  elseif ev == "DES_NOW" then
    S.des_now = S.des_now + 1
  end
end

-- MAIN STEP ------------------------------------------------------------------------------------------
--[[ returns out: { pitch_cmd, roll_cmd, valid, servos, fma = {roll, roll_arm, pitch, pitch_arm, at}, status,
       spd_target (CAS), at_mode code, thr_target (0..1 or -1), alt_target (display), vs_win, ias_win, ... } ]]
function M.step(S, inp, laws)
  local dt = inp.dt or 0.05
  S.t = inp.t or (S.t + dt)
  S.msg = {}
  local air = not inp.onground
  local ra = inp.ra or 0
  local fdon = fd_on(inp)
  local lnav = inp.lnav or {}
  local vnav = inp.vnav or {}

  -- FD switch on in flight with no AP and no modes -> basic modes
  if fdon and not S.fd_prev then
    if air and n_ap(S) == 0 and S.roll == "NONE" and S.pitch == "NONE" then basic_modes(S, inp)
    elseif not air then S.toga = "TO"; set_roll(S, "TOGA"); set_pitch(S, "TOGA") end
  end
  if not fdon and S.fd_prev and n_ap(S) == 0 then
    S.roll, S.pitch = "NONE", "NONE"
    S.lnav_arm, S.vnav_arm, S.loc_arm, S.gs_arm, S.app = false, false, false, false, false
    if S.at == "VNAV" or S.at == "THR" then S.at = S.at_engaged and "SPD" or "NONE" end
  end
  S.fd_prev = fdon

  -- events
  local evs = S.events
  S.events = {}
  for _, e in ipairs(evs) do handle(S, inp, e[1], e[2]) end

  -- A/T arm switch off -> A/T disconnected
  if inp.at_arm_sw ~= 1 or inp.at_fail then
    if S.at_engaged then S.warn_at = inp.at_arm_sw == 1 end
    S.at_engaged = false; S.at = "NONE"
  end

  -- override / AP disconnect by yoke force (not in LAND modes below 1500)
  if n_ap(S) > 0 and inp.yoke_override and not (S.land_status and ra < C.land_ra) then
    handle(S, inp, "DISCONNECT")
  end
  if inp.bar == 1 and n_ap(S) > 0 then handle(S, inp, "DISCONNECT") end

  -- ground / takeoff -----------------------------------------------------------------------------
  if not air then
    if S.was_onground == false then
      -- touchdown
      S.touchdown_t = S.t
    end
    if S.at == "THR_REF" and S.to_thrust and (inp.ias or 0) >= C.at_hold_kt then S.at = "HOLD"; S.to_thrust = false end
    if S.toga == "TO" then S.liftoff_spd = inp.ias end
    if S.rollout or S.flare then
      -- landing roll
    elseif S.toga ~= "TO" and n_ap(S) == 0 then
      if not fdon then S.roll, S.pitch = "NONE", "NONE" end
    end
    if fdon and S.toga == nil and S.roll == "NONE" and S.pitch == "NONE" and (inp.gs or 0) < 40 then
      S.toga = "TO"; set_roll(S, "TOGA"); set_pitch(S, "TOGA")
    end
  else
    if S.was_onground and S.toga == "TO" then
      S.liftoff_t = S.t
      S.track_hold = inp.trk_mag
      local v2 = inp.v2 or 160
      S.to_spd = clamp(max(v2 + 10, (S.liftoff_spd or 0)), v2 + 10, v2 + 25)
    end
  end
  S.was_onground = inp.onground

  -- LNAV arm -> engage ----------------------------------------------------------------------------
  if S.lnav_arm and air and ra >= C.lnav_engage_ra then
    if lnav.valid and lnav.cap_ok then
      S.lnav_arm = false
      set_roll(S, "LNAV")
      if S.toga == "TO" or S.toga == "GA" then S.toga_roll_end = true end
    elseif lnav.valid and not lnav.intc and (lnav.xtk and abs(lnav.xtk) > 2.5) then
      S.msg.not_intc = true
    end
  end
  if S.roll == "LNAV" and not lnav.route then set_roll(S, "HDG_HOLD"); S.hdg_hold = nil end

  -- VNAV arm -> engage
  if S.vnav_arm and air and ra >= C.vnav_engage_ra and vnav.valid then
    if S.pitch ~= "GS" and S.pitch ~= "FLARE" then vnav_engage(S, inp) end
  end
  if S.pitch == "VNAV" and not vnav.valid then
    -- VNAV lost: hold V/S... FCOM: pitch reverts, use V/S
    set_pitch(S, "VS"); S.vs_target = floor((inp.vs or 0) / 100 + 0.5) * 100; S.vs_win = true
    S.msg.vnav_lost = true
  end

  -- TOGA termination after takeoff
  if S.toga == "TO" and air then
    if S.pitch == "VNAV" or (S.pitch ~= "TOGA" and S.pitch ~= "NONE") then
      if S.roll == "TOGA" then
        if ra > C.toga_exit_ra then set_roll(S, "HDG_HOLD"); S.hdg_hold = nil end
      end
    end
    if S.roll ~= "TOGA" and S.pitch ~= "TOGA" then S.toga = nil end
  end
  if S.toga == "GA" then
    if S.roll ~= "TOGA" and S.pitch ~= "TOGA" then S.toga = nil; S.goaround = false end
  end
  if S.toga_roll_end then S.toga_roll_end = nil end

  -- LOC capture --------------------------------------------------------------------------------------
  local nav = inp.nav or {}
  if S.loc_arm and air and nav.loc_valid and nav.loc_cap_ok ~= false then
    local d = nav.hdef or 9
    local crs_err = abs(wrap180((nav.crs or 0) - (inp.trk_mag or 0)))
    local closing = (d > 0 and (nav.hdef_rate or 0) < 0) or (d < 0 and (nav.hdef_rate or 0) > 0)
    local early = laws and laws.loc_turn_point and laws.loc_turn_point(inp)
    -- already in a turn onto the localizer (LNAV / HDG SEL / bank turning towards the beam): the
    -- localizer takes over the turn as it is - same direction, the bank eased to the one that ends the
    -- turn on the centreline - instead of rolling out and turning in again. Taken over close to the
    -- course (within 15 deg) or when the turn has to carry on to reach the centreline.
    local inturn = false
    -- (v48: taken over closer in - within 1.5 dots - it was taking control a little early)
    if laws and laws.loc_turn_bank and S.roll ~= "LOC" and abs(d) < 1.5 then
      local th, breq = laws.loc_turn_bank(inp)
      if th and abs(th) > 2 then
        local toward = (inp.roll or 0) * (th > 0 and -1 or 1)
        if toward > 5 and (abs(th) <= 15 or breq >= 0.5 * toward) then inturn = true end
      end
    end
    local defer = false
    if not defer and crs_err <= 120 and (abs(d) < 1.0 or (abs(d) < 1.5 and closing) or early or inturn) then
      S.loc_arm = false
      set_roll(S, "LOC")
      S.lnav_arm = false
      if S.toga then S.toga = nil end
    end
  end
  if S.roll == "LOC" and not nav.loc_valid and air and ra > 200 then
    -- localizer lost: hold heading
    set_roll(S, "HDG_HOLD"); S.hdg_hold = nil
  end
  -- G/S capture (after LOC)
  if S.gs_arm and air and S.roll == "LOC" and nav.gs_valid then
    local v = nav.vdef or 9
    -- capture only on the real beam: close to it, or crossing it with a small deviation either side
    -- (a crossing far out is a false glideslope lobe above the beam). The IO also rejects the signal
    -- when the aircraft's elevation angle from the runway is not near the glideslope angle.
    local crossing = S.gs_prev and (S.gs_prev > 0) ~= (v > 0) and abs(S.gs_prev) < 1.5 and abs(v) < 1.5
    if abs(v) < 0.6 or crossing then
      S.gs_arm = false
      leave_vnav(S)
      set_pitch(S, "GS")
      if S.at_engaged and S.at ~= "THR_REF" then S.at = "SPD" end
      open_ias_window(S, inp)
    end
    S.gs_prev = v
  end

  -- G/S signal lost / implausible while in G/S above 1500 ft: hold the vertical speed (V/S)
  if S.pitch == "GS" and air and not nav.gs_valid and ra > 1500 and not S.land_status then
    S.vs_target = math.floor(math.max(-1000, math.min(0, inp.vs or 0)) / 100) * 100; set_pitch(S, "VS"); S.vs_win = true; S.vs_sync = true; S.gs_arm = true
  end

  -- LAND 2 / LAND 3 / FLARE / ROLLOUT -----------------------------------------------------------------
  local nap = n_ap(S)
  if S.roll == "LOC" and S.pitch == "GS" and nap >= 2 and ra < C.land_ra and air then
    if not S.land_status then S.land_status = (nap >= 3) and "LAND3" or "LAND2" end
    if ra > 200 then S.land_status = (nap >= 3) and "LAND3" or "LAND2" end
  elseif S.land_status and not (S.flare or S.rollout) and (S.pitch ~= "GS" or S.roll ~= "LOC") then
    S.land_status = nil
  end
  if S.land_status and air then
    if not S.flare and ra <= C.flare_ra and ra > 2 then S.flare = true; set_pitch(S, "FLARE") end
    if S.flare and ra <= C.retard_ra and S.at_engaged then S.at = "IDLE"; S.retard = true end
  end
  if S.land_status and S.flare and not air then
    if not S.rollout then S.rollout = true; set_roll(S, "ROLLOUT"); S.pitch = "NONE" end
  end
  if S.rollout and inp.reverse and S.at_engaged then S.at_engaged = false; S.at = "NONE" end
  if S.rollout and (inp.gs or 0) < 30 and nap > 0 then
    -- autopilot disengages at low speed after rollout (pilot normally does this)
  end
  if S.land_status == nil and nap >= 2 and not S.app and S.roll ~= "LOC" then
    -- multiple autopilots only with approach: keep the first
    local kept = false
    for k = 1, 3 do if S.ap[k] and not kept then kept = true elseif S.ap[k] then S.ap[k] = false end end
  end

  -- ALT capture from V/S, FLCH, TOGA -----------------------------------------------------------------
  local mcp_alt = inp.mcp_alt or 0
  local err = mcp_alt - inp.alt
  if S.cap_mode ~= S.pitch then S.cap_mode = S.pitch; S.cap_armed = abs(err) > 100 end
  if abs(err) > 100 then S.cap_armed = true end
  if (S.pitch == "VS" or S.pitch == "FLCH" or S.pitch == "TOGA") and air and ra > 400 and S.cap_armed then
    local vs = inp.vs or 0
    local toward = (err > 0 and vs > 50) or (err < 0 and vs < -50)
    local need = 60 * sqrt(2 * C.cap_a * abs(err))
    if toward and need <= abs(vs) + 150 or abs(err) < 40 then
      if S.pitch ~= "TOGA" or S.toga == "GA" or ra > C.toga_exit_ra then
        set_pitch(S, "ALT"); S.alt_hold = mcp_alt; S.capturing = true
        if S.at_engaged and (S.at == "THR" or S.at == "HOLD" or S.at == "THR_REF" or S.at == "IDLE") then S.at = "SPD" end
        if S.toga == "GA" and S.roll == "TOGA" then end
      end
    end
  end
  if S.pitch == "ALT" and S.capturing and abs(inp.alt - (S.alt_hold or inp.alt)) < 30 then S.capturing = false end
  -- MCP altitude changed while in ALT (not VNAV): stays ALT (744)

  -- FLCH A/T: THR -> HOLD when thrust reached (descent: idle)
  if S.pitch == "FLCH" and S.at_engaged then
    if S.at == "THR" and S.flch_dir < 0 and (inp.thr_lever or 1) < 0.03 then S.at = "HOLD" end
  end

  -- VNAV: sub-mode, A/T request from FMC -------------------------------------------------------------
  local vsub = nil
  if S.pitch == "VNAV" then
    vsub = vnav.sub or "SPD"
    if S.at_engaged and (S.at == "VNAV" or S.at == "THR_REF" or S.at == "SPD" or S.at == "IDLE" or S.at == "HOLD" or S.at == "THR") then
      local r = vnav.at or "SPD"
      if r == "THR_REF" then S.at = "THR_REF" elseif r == "IDLE" then S.at = "IDLE" elseif r == "HOLD" then S.at = "HOLD" else S.at = "SPD" end
    end
    if S.at == "VNAV" then S.at = "SPD" end
    if not S.spd_intv then S.ias_win = false end
  else
    if S.at == "VNAV" then S.at = "SPD" end
  end
  if S.at_engaged and S.at == "NONE" then S.at = "SPD" end

  -- after takeoff A/T HOLD with no VNAV -> stays HOLD until a mode is selected
  -- speed target -------------------------------------------------------------------------------------
  local spd_tgt = inp.mcp_spd or 250
  local spd_src = "MCP"
  if S.pitch == "VNAV" and not S.spd_intv and vnav.spd then spd_tgt = vnav.spd; spd_src = "VNAV" end
  if S.pitch == "TOGA" and S.toga == "TO" and air then spd_tgt = S.to_spd or spd_tgt; spd_src = "TOGA" end
  if S.pitch == "TOGA" and S.toga == "GA" then spd_tgt = max(S.ga_spd or spd_tgt, inp.mcp_spd or 0); spd_src = "TOGA" end
  -- speed protection
  if inp.vmax then spd_tgt = min(spd_tgt, inp.vmax - 5) end
  if inp.vmin and air then spd_tgt = max(spd_tgt, inp.vmin) end
  S.spd_tgt = spd_tgt

  -- status ------------------------------------------------------------------------------------------
  local status = "NONE"
  if nap > 0 then status = "CMD" elseif fdon then status = "FD" end
  if S.land_status and nap > 0 then status = S.land_status end
  if S.land_status and nap < 2 and air and ra < C.land_ra then status = "NO_AUTOLAND" end
  S.status = status

  -- mode change boxes
  local function boxchg(k, v) if S.prev[k] ~= v then S.prev[k] = v; S.chg[k] = S.t end end
  local pitch_disp = S.pitch
  if S.pitch == "VNAV" then pitch_disp = "VNAV_" .. (vsub or "SPD") end
  boxchg("roll", S.roll); boxchg("pitch", pitch_disp); boxchg("at", S.at); boxchg("status", status)

  -- GUIDANCE (laws) ---------------------------------------------------------------------------------
  local g = laws.compute(S, inp, { vsub = vsub, spd_tgt = spd_tgt, fdon = fdon, nap = nap })

  -- outputs ------------------------------------------------------------------------------------------
  local out = {
    pitch_cmd = g.pitch, roll_cmd = g.roll, rudder = g.rudder, valid = (nap > 0 or fdon) and (S.roll ~= "NONE" or S.pitch ~= "NONE"),
    servos = nap > 0, nap = nap, ap = { S.ap[1], S.ap[2], S.ap[3] },
    roll = M.ROLL[S.roll] or 0,
    pitch = (S.pitch == "VNAV") and ((vsub == "PTH" and 6) or (vsub == "ALT" and 5) or 4) or (M.PITCH[S.pitch] or 0),
    roll_arm = (S.lnav_arm and 2) or (S.loc_arm and 3) or ((S.land_status and not S.rollout) and 4) or 0,
    pitch_arm = (S.gs_arm and 2) or ((S.land_status and not S.flare) and 3) or (S.vnav_arm and 4) or 0,
    at = M.AT[S.at] or 0,
    at_engaged = S.at_engaged,
    status = M.STATUS[status] or 0,
    box_roll = (S.t - S.chg.roll) < C.mode_box_s and S.roll ~= "NONE",
    box_pitch = (S.t - S.chg.pitch) < C.mode_box_s and S.pitch ~= "NONE",
    box_at = (S.t - S.chg.at) < C.mode_box_s and S.at ~= "NONE",
    box_status = (S.t - S.chg.status) < C.mode_box_s and status ~= "NONE",
    spd_tgt = spd_tgt, spd_src = spd_src,
    thr_target = g.thr or -1,
    vs_win = (S.pitch == "VS"),
    ias_win = S.ias_win,
    alt_push = S.alt_push, des_now = S.des_now,
    msg = S.msg,
    warn_ap = S.warn_ap, warn_at = S.warn_at,
    land = S.land_status ~= nil, flare = S.flare, rollout = S.rollout,
    vnav_engaged = (S.pitch == "VNAV"), lnav_engaged = (S.roll == "LNAV"),
    goaround = S.goaround, toga = S.toga,
    spd_intv = S.spd_intv,
    alt_target = g.alt_tgt,
    vs_target = S.vs_target,
  }
  return out
end

B744AFDS = M
