--[[
 ap_modes.lua  --  mode state machine for the rebuilt Sparky 747 autopilot (AFDS).

 PURE LOGIC: no X-Plane API.  The IO layer (ap_io.lua / main script) builds an input snapshot,
 feeds button presses in as events, calls step(), and writes `out` back to datarefs.

 Why this exists: the stock code never owned the modes.  It inferred FMA state from X-Plane's
 FLCH/VS/ALT/NAV/GS flags and patched them from 25 places.  Here the machine owns the modes and
 the sim flags are *outputs*.  The only thing observed back from the sim is altitude capture
 (alt_hold_status becoming 2), which X-Plane performs itself.

 FMA codes written to the cockpit are unchanged from stock (see ROLL / PITCH / ATM tables).
--]]
local U  = AP_LOAD("ap_util.lua")

local M = {}

-- codes published on laminar/B747/autopilot/FMA/* (identical to stock)
M.ROLL  = { NONE = 0, TOGA = 1, LNAV = 2, LOC = 3, ROLLOUT = 4, ATT = 5, HDG_SEL = 6, HDG_HOLD = 7 }
M.PITCH = { NONE = 0, TOGA = 1, GS = 2, FLARE = 3, VNAV_SPD = 4, VNAV_ALT = 5, VNAV_PTH = 6,
            VS = 7, FLCH = 8, ALT = 9 }
M.ATM   = { NONE = 0, HOLD = 1, IDLE = 2, SPD = 3, THR = 4, THR_REF = 5 }

M.CFG = {
  engage_min_ra    = 400,    -- ft RA: CMD cannot engage below this (stock value)
  lnav_arm_ra      = 50,     -- ft RA: armed LNAV may capture above this
  vnav_engage_ra   = 400,    -- ft RA: armed VNAV engages above this
  accel_clear_ra   = 1500,   -- ft RA: TOGA engine mode is released in VNAV above this
  flch_alt_band    = 150,    -- FLCH selected within this of MCP alt => go straight to ALT
  des_to_spd_ft    = 600,    -- above path by more than this => VNAV SPD descent (idle)
  des_to_pth_ft    = 100,    -- back on path (within this) => VNAV PTH
  des_dwell_s      = 20,     -- minimum time in PTH before falling back to SPD
  mode_box_s       = 10,     -- mode-change box duration
  loc_valid_s      = 1.0,    -- localizer signal must be valid this long before capture
  gs_cap_dots      = 1.5,
  appr_arm_ra_gs   = 3500,   -- APP arms the glideslope below this RA
  appr_hdg_window  = 120,    -- course must be within this of our heading to arm
  alt_sync_s       = 2.0,    -- MCP altitude knob must be still this long before FLCH/VS window follows
  mcp_alt_cruise_margin = 100,
  min_n1_running   = 15.0,
  engines_needed   = 3,
  hyd_min          = 1000,
}

local to_manual_from_vnav
local function sum3(c) return c[1] + c[2] + c[3] end

function M.new()
  return {
    cmd = { 0, 0, 0 }, servos = false, dis_bar = false, switch_time = -100,
    roll = "NONE", roll_armed = "NONE", pitch = "NONE", pitch_armed = "NONE",
    lnav = 0, vnav = 0, vdesc = 0, mcp_hold = 0,
    loc = 0, gs = 0, appr = 0, on_approach = false, loc_valid_since = nil,
    at_active = false, thrust_mode = 0, phase = 0, des_mode = "SPD", des_levelled = false,
    alt_target = 0, hold_alt = 0, vs_cmd = 0, limit_is_mcp = false,
    last_mcp_alt = nil, mcp_alt_changed = -100,
    last_cmd = -100, warn = 0, ap_was_on = false,
    events = {}, notif = {}, clear_toga = false, set_cruise = nil, early_desc = false,
    afds = 0, last_afds = 0, noautoland = false,
    chg = { roll = { v = 0, t = -100 }, pitch = { v = 0, t = -100 }, at = { v = 0, t = -100 },
            afds = { v = 0, t = -100 } },
    actions = {}, new_mode_flags = {},
    heading_cmd = nil, last_dots = nil,
  }
end

-- Another script (engines init/shutdown, vnavspd) wrote one of the AP state datarefs.
-- Stock treated those datarefs as the truth; here we adopt the external reset.
function M.external(S, inp, field, value)
  if field == "vnav" then
    if value == 0 and S.vnav > 0 then to_manual_from_vnav(S, inp) end
  elseif field == "vdesc" then
    if value == 0 then S.vdesc = 0; S.des_levelled = false end
  elseif field == "phase" then
    S.phase = value
  elseif field == "appr" then
    if value == 0 then S.appr = 0 end
  elseif field == "lnav" then
    if value == 0 and S.lnav > 0 then S.lnav = 0 end
  end
end

function M.event(S, name, arg)
  S.events[#S.events + 1] = { name = name, arg = arg }
end

-- ---------------------------------------------------------------------------------------------
-- helpers
-- ---------------------------------------------------------------------------------------------
local function act(S, a) S.actions[#S.actions + 1] = a end

local function at_enabled(inp)
  return inp.at_switch and not inp.at_fail
end

local function any_fd(inp)
  return inp.fd_L or inp.fd_R
end

local function can_select_modes(S, inp)
  return true      -- stock allowed arming/selecting with nothing engaged (e.g. LNAV/VNAV armed on the ground)
end

local function alt_window(inp)
  return U.rescale(0, 200, 3000, 1000, math.abs(inp.vs))
end

local function clear_vnav(S)
  S.vnav, S.vdesc, S.mcp_hold = 0, 0, 0
  S.early_desc, S.des_levelled = false, false
end

local function set_phase_by_mcp(S, inp)
  if inp.mcp_alt > inp.palt then S.phase = 1 else S.phase = 3 end
end

local function engage_at(S, inp)
  if not S.at_active and at_enabled(inp) and inp.onground == 0 then
    S.at_active = true
    act(S, "at_engaged")
  end
end

to_manual_from_vnav = function(S, inp)
  -- VNAV switched off: keep flying what we were flying, as a manual mode
  local p = S.pitch
  if p == "VNAV_SPD" then
    S.pitch = "FLCH"
    S.thrust_mode = 2
  elseif p == "VNAV_PTH" then
    if inp.sim_alt_hold == 2 then
      S.pitch = "ALT"; S.hold_alt = inp.palt
    else
      S.pitch = "VS"; S.vs_cmd = U.clamp(U.round(inp.vs / 100) * 100, -8000, 6000)
    end
  elseif p == "VNAV_ALT" then
    S.pitch = "ALT"
    if S.hold_alt == 0 then S.hold_alt = inp.palt end
  end
  clear_vnav(S)
end

-- ---------------------------------------------------------------------------------------------
-- engagement / disengagement
-- ---------------------------------------------------------------------------------------------
local function disengage(S, inp, warn)
  local was_on = sum3(S.cmd) > 0
  S.cmd[1], S.cmd[2], S.cmd[3] = 0, 0, 0
  S.servos = false
  S.switch_time = inp.time
  if was_on and warn then S.warn = 1 end
  act(S, "servos_off")
  if S.at_active then act(S, "at_reengage_soon") end
end

local function engage_cmd(S, inp, i)
  if S.dis_bar then return end
  if S.cmd[i] == 1 then return end
  if inp.radalt <= M.CFG.engage_min_ra then return end
  if sum3(S.cmd) > 0 then return end                 -- one CMD at a time until approach multi-engage
  S.cmd[i] = 1
  S.warn = 0
  if not S.servos then
    S.servos = true
    S.switch_time = inp.time
    act(S, "servos_on")
  end
  -- first engagement with no modes: hold what we are doing (stock left this to X-Plane defaults)
  if S.roll == "NONE" then
    S.roll = "HDG_HOLD"; act(S, "heading_hold")
  end
  if S.pitch == "NONE" then
    if math.abs(inp.vs) < 200 then
      S.pitch = "ALT"; S.hold_alt = inp.palt; S.phase = 2
    else
      S.pitch = "VS"; S.vs_cmd = U.clamp(U.round(inp.vs / 100) * 100, -8000, 6000)
    end
    engage_at(S, inp)
  end
end

-- ---------------------------------------------------------------------------------------------
-- event handling (button presses)
-- ---------------------------------------------------------------------------------------------
local function handle_event(S, inp, e)
  local n = e.name
  S.last_cmd = inp.time

  if n == "cmd_L" then engage_cmd(S, inp, 1)
  elseif n == "cmd_C" then engage_cmd(S, inp, 2)
  elseif n == "cmd_R" then engage_cmd(S, inp, 3)

  elseif n == "dis_bar" then
    S.dis_bar = not S.dis_bar
    if S.dis_bar then
      if sum3(S.cmd) > 0 then disengage(S, inp, true) else disengage(S, inp, false) end
      S.engine_toga_clear = true
    else
      S.warn = 0
    end

  elseif n == "yoke_disc" then
    S.engine_toga_clear = true
    if sum3(S.cmd) > 0 then
      disengage(S, inp, true)
    else
      S.warn = 0                                       -- second press cancels the warning
    end

  elseif n == "reset" then
    disengage(S, inp, false)
    S.engine_toga_clear = true

  elseif n == "at_disco" then
    S.at_active = false
    S.at_fail_toggle = true

  elseif n == "thr" then
    S.at_active = false
    if inp.radalt > 400 then S.thrust_mode = 3 end

  elseif n == "spd" then
    engage_at(S, inp)
    clear_vnav(S)
    if S.pitch == "FLCH" or S.pitch == "VNAV_SPD" then
      S.pitch = "VS"; S.vs_cmd = U.clamp(U.round(inp.vs / 100) * 100, -8000, 6000)
    end
    S.thrust_mode = 0
    if inp.mcp_alt > inp.palt then S.phase = 1 else S.phase = 3 end

  -- ---------------- vertical mode buttons ----------------
  elseif n == "flch" then
    if not can_select_modes(S, inp) then return end
    S.clear_toga = true
    clear_vnav(S)
    if math.abs(inp.mcp_alt - inp.palt) < M.CFG.flch_alt_band then
      S.pitch = "ALT"; S.hold_alt = inp.mcp_alt
    else
      S.pitch = "FLCH"; S.thrust_mode = 2
    end
    set_phase_by_mcp(S, inp)
    S.alt_target = inp.mcp_alt
    engage_at(S, inp)

  elseif n == "vs" then
    if not can_select_modes(S, inp) then return end
    S.clear_toga = true
    clear_vnav(S)
    S.pitch = "VS"; S.thrust_mode = 0
    S.vs_cmd = U.clamp(U.round(inp.vs / 100) * 100, -8000, 6000)
    set_phase_by_mcp(S, inp)
    S.alt_target = inp.mcp_alt
    engage_at(S, inp)

  elseif n == "alt_hold" then
    if not can_select_modes(S, inp) then return end
    S.clear_toga = true
    clear_vnav(S)
    S.pitch = "ALT"; S.hold_alt = inp.palt; S.thrust_mode = 0; S.phase = 2
    engage_at(S, inp)

  elseif n == "vnav" then
    if not can_select_modes(S, inp) then return end
    if S.vnav > 0 then
      to_manual_from_vnav(S, inp)
      S.thrust_mode = 0
    else
      local ok = inp.vnav_available
      if not ok or inp.cruise_alt < 10 then
        S.notif[30] = 1                                -- "NO VNAV" style advisory (stock index 30)
        return
      end
      S.vnav = 1
      S.vdesc = 0
      S.mcp_hold = 0
      S.thrust_mode = 0
      S.new_mode_flags.vnav_armed = true
      engage_at(S, inp)
    end

  elseif n == "alt_knob" then                          -- ALT intervention knob
    if S.vnav > 0 then
      if inp.mcp_alt > inp.cruise_alt + M.CFG.mcp_alt_cruise_margin then
        S.set_cruise = inp.mcp_alt                     -- step climb: new cruise altitude
        S.mcp_hold = 0
      elseif S.mcp_hold == 1 or S.pitch == "VNAV_ALT" then
        S.mcp_hold = 0                                 -- release MCP altitude hold
      elseif inp.mcp_alt < inp.palt - 100 and S.vdesc == 0 then
        S.early_desc = true                            -- start descent now (before TOD)
      end
      S.release_hold = true
    end

  -- ---------------- lateral mode buttons ----------------
  elseif n == "lnav" then
    if S.lnav > 0 then
      S.lnav = 0
      if S.roll == "LNAV" then S.roll = "HDG_HOLD"; act(S, "heading_hold") end
    else
      if not can_select_modes(S, inp) then return end
      S.lnav = 1
    end

  elseif n == "hdg_sel" then
    if not can_select_modes(S, inp) then return end
    S.lnav = 0
    if S.loc == 2 then S.loc = 1 end
    S.roll = "HDG_SEL"
    S.new_mode_flags.hdg_sel = true

  elseif n == "hdg_hold" then
    if not can_select_modes(S, inp) then return end
    S.lnav = 0
    S.roll = "HDG_HOLD"

  elseif n == "loc" or n == "appr" then
    if not inp.ils_valid then
      S.notif[40] = 1
      return
    end
    S.notif[40] = 0
    if n == "loc" then
      if S.appr >= 0 then S.appr = -1 end
    else
      if S.appr <= 0 then
        S.appr = 1
        -- approach with CMD already on and above 1500 ft: arm all engaged channels' redundancy
        if sum3(S.cmd) > 0 and inp.radalt > 1500 then S.cmd[1], S.cmd[2], S.cmd[3] = 1, 1, 1 end
      elseif S.gs ~= 2 and S.loc ~= 2 then
        S.appr = 0
      end
    end
    S.new_mode_flags.set_hdg_to_course = true

  elseif n == "toga" then
    -- engine/FD TOGA handled by the manipulators script; we only observe TOGA_status
  end
end

-- ---------------------------------------------------------------------------------------------
-- approach (LOC / GS) logic
-- ---------------------------------------------------------------------------------------------
local function loc_command(S, inp)
  -- steering law kept from stock (gains tuned in the sim): 4 deg per dot (two receivers summed)
  -- plus 2x the off-course angle to the beacon, clamped to +/-60
  local bearing = (inp.nav1_bearing + inp.nav2_bearing) / 2
  local diffap = U.hdg_diff(bearing, inp.nav_obs) * 2
  diffap = U.clamp(diffap, -60, 60)
  local mod = (inp.hdef1 + inp.hdef2) * 4 + diffap
  return U.wca_heading(inp.nav_obs + mod, inp.wind_bearing, inp.wind_kts, inp.tas_kts), diffap, mod
end

local function approach_logic(S, inp)
  local C = M.CFG
  -- cancel everything on go-around / ground
  if S.appr == 0 or inp.toga_mode == 1 or inp.onground == 1 then
    if S.appr ~= 0 or S.loc ~= 0 or S.gs ~= 0 then
      if inp.toga_mode == 1 or inp.onground == 1 then S.appr = 0 end
    end
    if S.appr == 0 then
      S.loc, S.gs, S.on_approach, S.loc_valid_since = 0, 0, false, nil
      if S.roll == "LOC" then S.roll = "HDG_SEL" end
      return
    end
  end

  local diff = U.hdg_diff(inp.nav_obs, inp.hdg_mag)
  local can_arm = math.abs(diff) < C.appr_hdg_window

  -- arm
  if S.loc == 0 and can_arm then
    S.loc = 1
    if S.appr == 1 and inp.radalt < C.appr_arm_ra_gs then S.gs = 1 end
  end
  if S.appr == 1 and S.gs == 0 and S.loc >= 1 and inp.radalt < C.appr_arm_ra_gs then S.gs = 1 end
  if S.appr == -1 and S.gs ~= 0 and S.gs ~= 2 then S.gs = 0 end

  -- localizer capture: both receivers show a valid horizontal signal for a moment,
  -- or X-Plane itself reports captured (stock mirrored that flag too)
  local sig_ok = inp.hsig1 == 1 and inp.hsig2 == 1
  if S.loc == 1 then
    if (inp.sim_nav or 0) == 2 and sig_ok then
      S.loc = 2; S.lnav = 0; S.on_approach = false
    elseif sig_ok then
      S.loc_valid_since = S.loc_valid_since or inp.time
      if inp.time - S.loc_valid_since >= C.loc_valid_s then
        S.loc = 2
        S.lnav = 0
        S.on_approach = false
      end
    else
      S.loc_valid_since = nil
    end
  elseif S.loc == 2 then
    if not sig_ok then
      S.loc = 1; S.loc_valid_since = nil
    else
      local cmd, diffap = loc_command(S, inp)
      S.heading_cmd = cmd
      if not S.on_approach and math.abs(diffap) < 5 then S.on_approach = true end
      if S.on_approach and math.abs(inp.hdef1 + inp.hdef2) > 4 then
        S.on_approach = false
        S.loc = 1
      end
    end
  end

  -- glideslope capture
  local gs_ok = inp.gsflag1 == 0 and inp.gsflag2 == 0 and inp.vsig1 == 1 and inp.vsig2 == 1
  if S.gs == 1 and S.loc == 2 and gs_ok and
     (math.abs(inp.vdef) < C.gs_cap_dots or (inp.sim_gs or 0) == 2) then
    S.gs = 2
    S.vnav, S.vdesc, S.mcp_hold = 0, 0, 0
    S.thrust_mode = 0
    S.phase = 4
    S.pitch = "GS"
  end
  if S.gs == 2 and (inp.gsflag1 ~= 0 and inp.gsflag2 ~= 0) then
    S.gs = 1
    if S.pitch == "GS" then S.pitch = "VS"; S.vs_cmd = U.clamp(U.round(inp.vs / 100) * 100, -8000, 6000) end
  end
end

-- ---------------------------------------------------------------------------------------------
-- lateral
-- ---------------------------------------------------------------------------------------------
local function lateral(S, inp, ln)
  local C = M.CFG
  -- route available?
  if S.lnav > 0 and not (ln and ln.valid) and inp.route_len < 2 then
    S.lnav = 0
    if S.roll == "LNAV" then S.roll = "HDG_HOLD"; act(S, "heading_hold") end
    S.notif[54] = 1
  end

  -- armed LNAV captures when airborne and the leg can be captured (or TOGA roll was active)
  if S.lnav == 1 and inp.onground == 0 and inp.radalt > C.lnav_arm_ra and ln and ln.valid then
    if ln.capture_ok and S.loc ~= 2 then
      S.lnav = 2
      S.new_mode_flags.lnav_active = true
    end
  end
  if S.lnav == 2 and S.loc == 2 then S.lnav = 0 end

  -- roll mode
  local toga = inp.toga_status ~= 0
  if sum3(S.cmd) == 0 and not any_fd(inp) and S.ap_was_on then
    S.roll = "NONE"
  elseif math.abs(inp.att) >= 5 then
    S.roll = "ATT"
  elseif toga and S.lnav ~= 2 then
    S.roll = "TOGA"
  elseif inp.onground == 1 then
    S.roll = "NONE"
    S.thrust_mode = 0
  elseif S.loc == 2 then
    S.roll = "LOC"
  elseif S.lnav == 2 then
    S.roll = "LNAV"
  elseif S.roll == "LOC" or S.roll == "LNAV" or S.roll == "TOGA" or S.roll == "ATT" then
    S.roll = "HDG_HOLD"; act(S, "heading_hold")       -- lost a roll mode: wings level hold
  end

  -- armed roll mode
  if S.lnav == 1 then S.roll_armed = "LNAV"
  elseif S.loc == 1 or (S.appr ~= 0 and S.loc ~= 2) then S.roll_armed = "LOC"
  elseif inp.radalt > 200 and (S.afds == 3 or S.afds == 4) then S.roll_armed = "ROLLOUT"
  else S.roll_armed = "NONE" end

  -- heading target for the sim
  if S.roll == "HDG_SEL" then
    S.heading_cmd = inp.mcp_hdg
  elseif S.roll == "LNAV" and ln and ln.valid then
    S.heading_cmd = ln.hdg_cmd_mag
  elseif S.roll == "LOC" then
    -- set by approach_logic
  else
    S.heading_cmd = nil
  end
end

-- ---------------------------------------------------------------------------------------------
-- vertical
-- ---------------------------------------------------------------------------------------------
local function vnav_is_valid(inp, vn)
  return vn ~= nil and vn.valid
end

local function vertical(S, inp, vn)
  local C = M.CFG
  local win = alt_window(inp)
  local palt = inp.palt

  -- MCP altitude knob: remember when it last moved (FLCH/VS window follows after it settles)
  if S.last_mcp_alt == nil or S.last_mcp_alt ~= inp.mcp_alt then
    S.last_mcp_alt = inp.mcp_alt
    S.mcp_alt_changed = inp.time
  end

  -- TOGA pitch: owned by the manipulators script via TOGA_status, we just show it
  if inp.toga_status == 1 and S.pitch ~= "GS" then
    S.pitch = "TOGA"
  elseif S.pitch == "TOGA" and inp.toga_status == 0 then
    S.pitch = (inp.onground == 1) and "NONE" or "VS"
    if S.pitch == "VS" then S.vs_cmd = U.clamp(U.round(inp.vs / 100) * 100, -8000, 6000) end
  end
  if S.clear_toga and inp.toga_status ~= 0 then act(S, "clear_toga") end
  S.clear_toga = false

  -- go-around: TOGA engine mode cancels approach & GS
  if inp.toga_mode == 1 and S.gs ~= 0 then
    S.gs, S.loc, S.appr = 0, 0, 0
    if S.pitch == "GS" then S.pitch = "TOGA" end
  end

  -- on the ground nothing but TOGA shows
  if inp.onground == 1 and S.pitch ~= "TOGA" then
    S.pitch = "NONE"
    if S.vnav == 2 then clear_vnav(S) end
  end

  -- ---------- VNAV ----------
  if S.vnav > 0 then
    if not vnav_is_valid(inp, vn) or inp.cruise_alt < 10 or inp.vnav_available == false then
      -- keep VNAV armed on the ground / TOGA until the route is complete, cancel once airborne
      if inp.onground == 0 and inp.radalt > C.vnav_engage_ra then
        clear_vnav(S); to_manual_from_vnav(S, inp); S.notif[54] = 1
      end
    elseif S.vnav == 1 then
      -- armed: engage above 400 ft RA (or immediately when already flying)
      if inp.onground == 0 and inp.radalt > C.vnav_engage_ra then
        S.vnav = 2
        S.clear_toga = true
        act(S, "clear_toga")
        if inp.toga_mode == 1 and inp.radalt > C.accel_clear_ra then act(S, "clear_toga_mode") end
        S.new_mode_flags.vnav_active = true
        S.chg.pitch.t = inp.time
        S.pitch = "VNAV_SPD"                          -- refined below
      end
    end

    if S.vnav == 2 then
      if inp.toga_mode == 1 and inp.radalt > C.accel_clear_ra then act(S, "clear_toga_mode") end
      local cruise = inp.cruise_alt
      if S.set_cruise then
        act(S, { "set_cruise", S.set_cruise }); cruise = S.set_cruise; S.set_cruise = nil
      end
      local dist_tod = vn.dist_to_tod
      -- decide vertical phase
      if S.vdesc == 0 then
        local mcp_below = inp.mcp_alt <= palt - 100
        local past_tod = dist_tod <= 0
        if (past_tod and mcp_below and inp.radalt > 1000 and vn.fpe > -400) or
           (S.early_desc and mcp_below) then
          S.vdesc = 1
          S.phase = 3
          S.des_levelled = false
          S.early_desc = false
          S.des_mode = (vn.fpe > C.des_to_pth_ft) and "SPD" or "PTH"
          S.des_mode_t = inp.time
          S.chg.pitch.t = inp.time
          act(S, "descent_begin")
        end
      end

      local limit_is_mcp = false
      local target
      if S.vdesc > 0 then
        -- DESCENT
        S.phase = 3
        local tgt = vn.target_alt
        if tgt == nil then tgt = vn.profile and vn.profile.alt_end or 0 end
        target = math.max(tgt, inp.mcp_alt)
        limit_is_mcp = inp.mcp_alt >= tgt
        if inp.mcp_alt > palt + 100 then                -- MCP above us: cannot descend
          target = palt; limit_is_mcp = true; S.notif[33] = 1
        end
        -- energy / path mode with hysteresis
        if S.des_mode == "SPD" and vn.fpe <= C.des_to_pth_ft then
          S.des_mode = "PTH"; S.des_mode_t = inp.time
        elseif S.des_mode == "PTH" and vn.fpe > C.des_to_spd_ft and
               (inp.time - (S.des_mode_t or -100)) >= C.des_dwell_s then
          S.des_mode = "SPD"; S.des_mode_t = inp.time
        end
      else
        -- CLIMB or CRUISE
        if palt < cruise - math.max(win, 500) and dist_tod > 10 or inp.radalt < 5000 and dist_tod > 0 and palt < cruise - 200 then
          S.phase = 1
        else
          S.phase = 2
        end
        -- MCP only limits a CLIMB; an MCP below us in cruise does not pull VNAV down before TOD
        if inp.mcp_alt >= cruise or inp.mcp_alt < palt - 100 then
          target = cruise
        else
          target = inp.mcp_alt; limit_is_mcp = true
        end
        -- RESET MCP ALT reminder shortly before TOD
        local eta_h = (dist_tod > 0 and inp.gs > 1) and (dist_tod / (inp.gs * U.MPS_KT)) or 0
        if S.phase == 2 and dist_tod > 0 and eta_h < 0.0334 and inp.mcp_alt > cruise - 1000 then
          S.notif[33] = 1
        else
          S.notif[33] = 0
        end
      end
      S.limit_is_mcp = limit_is_mcp
      S.alt_target = target

      -- release of MCP hold (ALT knob)
      if S.release_hold then
        S.mcp_hold = 0
        S.release_hold = false
        if S.pitch == "VNAV_ALT" or S.pitch == "ALT" then S.pitch = "VNAV_SPD" end
      end

      -- pitch mode selection
      local at_target = math.abs(palt - target) <= math.max(30, win * 0.15) and math.abs(inp.vs) < 400
      local near = math.abs(palt - target) <= math.max(300, win * 1.5)      -- a latched flag far from the
      local captured = (inp.sim_alt_hold == 2 and near and (inp.time - S.chg.pitch.t) > 0.5 and  -- target is stale
                        (S.pitch == "VNAV_SPD" or S.pitch == "VNAV_PTH"))
      if S.mcp_hold == 1 then
        S.pitch = "VNAV_ALT"
        S.hold_alt = S.hold_alt ~= 0 and S.hold_alt or target
      elseif at_target or captured then
        if limit_is_mcp and inp.mcp_alt ~= cruise then
          S.mcp_hold = 1
          S.pitch = "VNAV_ALT"; S.hold_alt = target
          if S.vdesc > 0 then S.des_levelled = true end
        else
          S.pitch = "VNAV_PTH"                         -- level at FMC cruise / constraint
          S.hold_alt = target
          if S.vdesc == 0 then S.phase = 2 end
        end
      else
        if S.vdesc > 0 then
          if S.des_mode == "PTH" then
            S.pitch = "VNAV_PTH"
            S.vs_cmd = math.min(vn.vs_cmd, 0)          -- never climb back up to the path
            S.hold_alt = 0
          else
            S.pitch = "VNAV_SPD"
          end
        else
          S.pitch = "VNAV_SPD"
          S.hold_alt = 0
        end
      end
      S.thrust_mode = (S.pitch == "VNAV_SPD") and 2 or 0
    end
  else
    -- ---------- manual vertical modes ----------
    if S.pitch == "FLCH" or S.pitch == "VS" then
      -- sim window follows the MCP once the knob has been still
      local settled = (inp.time - S.mcp_alt_changed) >= C.alt_sync_s
      if settled then S.alt_target = inp.mcp_alt end
      local tgt = S.alt_target ~= 0 and S.alt_target or inp.mcp_alt
      local settled_mode = (inp.time - S.chg.pitch.t) > 0.5      -- sim needs a frame to clear stale flags
      local cap = settled_mode and ((inp.sim_alt_hold == 2 and math.abs(palt - tgt) <= math.max(300, win * 1.5))
                                  or (math.abs(palt - tgt) < 30 and math.abs(inp.vs) < 300))
      if cap and inp.onground == 0 then
        S.pitch = "ALT"; S.hold_alt = tgt
        if S.phase < 3 then S.phase = 2 end
        S.thrust_mode = 0
      end
    elseif S.pitch == "ALT" then
      if S.hold_alt == 0 then S.hold_alt = palt end
    end
  end

  -- armed pitch mode
  if S.vnav == 1 then S.pitch_armed = "VNAV"
  elseif S.gs == 1 or (S.appr > 0 and S.gs ~= 2) then S.pitch_armed = "GS"
  elseif inp.radalt > 200 and (S.afds == 3 or S.afds == 4) then S.pitch_armed = "FLARE"
  else S.pitch_armed = "NONE" end
end

-- ---------------------------------------------------------------------------------------------
-- autothrottle FMA / flight phase
-- ---------------------------------------------------------------------------------------------
local function autothrottle(S, inp)
  local C = M.CFG
  local running = 0
  for i = 1, 4 do if (inp.n1[i] or 0) > C.min_n1_running then running = running + 1 end end
  if running < C.engines_needed or inp.at_fail then
    if S.at_active then S.at_active = false end
    return M.ATM.NONE, true            -- second value: AT failed
  end
  if not at_enabled(inp) then
    S.at_active = false
    return M.ATM.NONE, false
  end

  local m
  if inp.autoland_idle then
    m = M.ATM.IDLE
  elseif (inp.toga_mode > 0 and inp.ias < 65) or inp.autoland < 0 or (S.vnav == 0 and S.thrust_mode > 0) then
    if inp.toga_mode == 1 then m = M.ATM.NONE; S.toga_set = 0 else m = M.ATM.THR_REF end
  elseif inp.toga_mode > 0 and inp.radalt < 50 then
    m = M.ATM.HOLD; S.toga_set = 1          -- 'reached hold state' (stock behaviour)
  elseif S.vnav == 2 and S.pitch == "VNAV_SPD" and S.vdesc == 0 then
    m = M.ATM.THR_REF                   -- VNAV climb
  elseif S.at_active then
    if S.vnav > 0 and inp.throttle < 0.1 and S.vdesc > 0 then
      m = M.ATM.IDLE
    elseif inp.onground == 0 and S.pitch == "TOGA" then
      m = M.ATM.THR_REF
    elseif S.vnav == 2 and S.pitch == "VNAV_SPD" and S.vdesc > 0 then
      m = M.ATM.IDLE
    elseif S.pitch == "FLCH" then
      m = M.ATM.THR_REF
    elseif inp.onground == 0 then
      m = M.ATM.SPD
    else
      m = M.ATM.NONE
    end
  else
    if S.vnav > 0 and inp.throttle < 0.1 and inp.onground == 0 then
      m = M.ATM.IDLE
    elseif (S.vnav > 0 or at_enabled(inp)) and inp.onground == 0 and inp.radalt < 50 then
      m = M.ATM.HOLD
    else
      m = M.ATM.NONE
    end
  end
  return m, false
end

-- ---------------------------------------------------------------------------------------------
-- AFDS status annunciation (FD / CMD / LAND 2 / LAND 3 / NO AUTOLAND)
-- ---------------------------------------------------------------------------------------------
local function afds_status(S, inp)
  local n = sum3(S.cmd)
  local land_assist = ((S.loc == 2 and S.gs == 2) or inp.approach_status > 1 or inp.autoland == 1)
                      and inp.radalt < 1500
  local noal = false
  local v
  if n > 0 and S.appr ~= 0 and inp.radalt > 200 and inp.radalt < 1000 and S.loc ~= 2 and S.gs ~= 2 then
    v = 5; noal = true
  elseif n == 1 then
    if land_assist and inp.radalt < 1500 then v = 5 else v = 2 end
  elseif n == 2 and land_assist then v = 3
  elseif n == 3 and land_assist then v = 4
  elseif n >= 1 then v = 2
  elseif inp.fd_L or inp.fd_R then v = 1
  else v = 0 end
  S.noautoland = noal
  return v
end

-- ---------------------------------------------------------------------------------------------
-- main step
-- ---------------------------------------------------------------------------------------------
local function track_change(S, key, value, t)
  local c = S.chg[key]
  if c.v ~= value then c.v = value; c.t = t end
end

local function box(S, key, active_nonzero, now)
  local c = S.chg[key]
  if not active_nonzero or c.v == 0 then return 0 end
  return (now - c.t) < M.CFG.mode_box_s and 1 or 0
end

function M.step(S, inp, ln, vn)
  S.actions = {}
  S.new_mode_flags = {}
  S.toga_set = nil
  if inp.at_active_dr ~= nil then S.at_active = inp.at_active_dr end
  S.engine_toga_clear = false
  S.at_fail_toggle = false

  -- 1. button events
  local evs = S.events
  S.events = {}
  local pitch_before = S.pitch
  for i = 1, #evs do handle_event(S, inp, evs[i]) end
  if S.pitch ~= pitch_before then S.chg.pitch.t = inp.time end

  -- 2. servo / hydraulic faults
  if inp.hyd3 < M.CFG.hyd_min then S.cmd[1] = 0 end
  if inp.hyd1 < M.CFG.hyd_min then S.cmd[2] = 0 end
  if inp.hyd2 < M.CFG.hyd_min then S.cmd[3] = 0 end
  local n_cmd = sum3(S.cmd)
  if S.servos and n_cmd == 0 and (inp.time - S.switch_time) > 1 then
    S.servos = false
    S.warn = 1
    act(S, "servos_off")
  end
  if n_cmd > 0 then S.ap_was_on = true end
  -- sim dropped the servos on us (autopilot fail etc.)
  if S.servos and inp.sim_servos == 0 and (inp.time - S.switch_time) > 1 then
    S.cmd[1], S.cmd[2], S.cmd[3] = 0, 0, 0
    S.servos = false
    S.warn = 1
  end

  -- 3. everything off when neither AP nor FD
  if n_cmd == 0 and not any_fd(inp) and S.ap_was_on then
    S.roll, S.pitch = "NONE", "NONE"
    clear_vnav(S); S.lnav = 0; S.appr = 0; S.loc = 0; S.gs = 0
    S.ap_was_on = false
    S.engine_toga_clear = true
    act(S, "fd_off")
  end

  -- 4. approach, lateral, vertical
  if S.vnav == 0 and S.pitch == "NONE" and sum3(S.cmd) == 0 and not any_fd(inp) then
    -- idle
  end
  approach_logic(S, inp)
  lateral(S, inp, ln)
  vertical(S, inp, vn)

  -- 5. flight phase bookkeeping
  if inp.onground == 1 then
    S.phase = 0
  elseif S.pitch == "TOGA" then
    S.phase = 0
  elseif S.gs == 2 then
    S.phase = 4
  elseif S.phase == 0 and inp.radalt > 100 then
    S.phase = (inp.mcp_alt > inp.palt + 500) and 1 or 2
  end
  if (S.pitch == "ALT" or S.pitch == "VNAV_ALT") and S.phase < 3 then S.phase = 2 end

  -- 6. autothrottle
  local at_mode, at_failed = autothrottle(S, inp)
  if at_failed then S.at_failed = true else S.at_failed = false end

  -- 7. AFDS annunciation
  S.afds = afds_status(S, inp)

  -- 8. mode-change boxes
  local roll_code  = M.ROLL[S.roll]
  local pitch_code = M.PITCH[S.pitch]
  track_change(S, "roll", roll_code, inp.time)
  track_change(S, "pitch", pitch_code, inp.time)
  track_change(S, "at", at_mode, inp.time)
  track_change(S, "afds", S.afds, inp.time)

  local out = {
    cmd = { S.cmd[1], S.cmd[2], S.cmd[3] },
    servos_on = S.servos, warn = S.warn,
    roll = roll_code, roll_armed = M.ROLL[S.roll_armed] or 0,
    pitch = pitch_code,
    pitch_armed = ({ NONE = 0, TOGA = 1, GS = 2, FLARE = 3, VNAV = 4 })[S.pitch_armed] or 0,
    at_mode = at_mode, at_active = S.at_active, at_failed = at_failed,
    thrust_mode = S.thrust_mode, phase = S.phase,
    lnav_state = S.lnav, vnav_state = S.vnav, vnav_descent = S.vdesc, after_tod = (S.vdesc > 0 or (vn and vn.valid and vn.dist_to_tod <= 0)) and 1 or 0,
    mcp_hold = S.mcp_hold, approach_mode = S.appr, nav_status = S.loc, gs_status = S.gs,
    afds = S.afds, noautoland = S.noautoland,
    alt_target = S.alt_target, hold_alt = S.hold_alt, vs_cmd = S.vs_cmd,
    heading_cmd = S.heading_cmd, des_mode = S.des_mode,
    roll_box = box(S, "roll", roll_code ~= 0, inp.time),
    pitch_box = box(S, "pitch", pitch_code ~= 0, inp.time),
    at_box = box(S, "at", at_mode ~= 0, inp.time),
    afds_box = (S.afds >= 1 and S.afds <= 4) and box(S, "afds", true, inp.time) or 0,
    afds_box2 = (S.afds == 5) and box(S, "afds", true, inp.time) or 0,
    notif = S.notif, actions = S.actions, flags = S.new_mode_flags,
    engine_toga_clear = S.engine_toga_clear, at_fail_toggle = S.at_fail_toggle, toga_mode_set = S.toga_set,
    last_cmd = S.last_cmd, show_fma = (n_cmd > 0 or any_fd(inp)) and 1 or 0,
  }
  S.notif = {}
  return out
end

AP_MODULES = AP_MODULES or {}
AP_MODULES["ap_modes.lua"] = M
return M
