--[[
****************************************************************************************
* Program Script Name	:	B747.42.xt.EEC.GE.lua
* Author Name			:	Marauder28
*                   (with SIGNIFICANT contributions from @kudosi for aeronautic formulas)
*   Revisions:
*   -- DATE --	--- REV NO ---		--- DESCRIPTION ---
*   2021-01-11	0.01a				      Start of Dev
*	  2021-05-27	0.1					      Initial Release
*   2021-06-16  0.2               Added Engine Idle Control
*	  2021-12-11	0.3				        New Throttle Resolver Angle formula, various bug fixes
*
*****************************************************************************************
]]

--This function calculates the target position of the throttle target on the Upper EICAS
function throttle_resolver_angle_GE(engine_in)
  local throttle_angle = 0.0
  local thrust_ratio_factor = 1.0

  thrust_ratio_factor = B747DR_display_N1_max[engine_in] / 116.0  --117.5

  -- lever -> commanded N1: idle N1 at the idle stop, the maximum rated N1 (takeoff / go-around) at
  -- full forward (was 25% .. 116%, past the rating: full lever showed ~109% where the rating was less)
  local idle = tonumber(engine_idle_control_GE(simDR_altitude or 0)) or 25.9
  local nmax = GE_N1_FULL or 105.0
  if nmax < idle + 20 then nmax = 105.0 end
  local N1_target = idle + (nmax - idle) * math.max(0, math.min(1, B747DR_throttle[engine_in] or 0))
  if B747DR_log_level == -1 then
    print("Thrust Factor = ", thrust_ratio_factor)
    print("TRA = ", throttle_angle)
    print("N1_target = ", N1_target)
  end
  return N1_target
end

function engine_idle_control_GE(altitude_ft_in)
  local N1_low_idle = 25.9
  local N1_high_idle_ratio = 2.625  --target ~42% N1 at SL / 15c

  --Information from FCOM
    --Chapter 7 - Engines, APU
    --Section 20 - Engine System Description
    --Sub-Section - Electronic Engine Control (EEC) / EEC Idle Selection

  --N1 Idle Display (Currently only high idle is implemented for use in the 747 in XP so manipulate the high idle dataref for low idle)

  --------------------
  --MINIMUM (LOW) Idle
  --------------------
  --When on ground and flaps not in landing configuration, low idle fluctuates based on temperature
  if simDR_onGround == 1 then
    if simDR_temperature < 15.0 then
      simDR_engine_high_idle_ratio = B747_rescale(-75.0, 1.04, 14.99, 1.249, simDR_temperature)
    else
      simDR_engine_high_idle_ratio = B747_rescale(15.0, 1.25, 75.0, 1.49, simDR_temperature)
    end
  end

  ge_approach_idle = false
  --Calc in-flight LOW Idle
  if simDR_onGround == 0 then
    --N1_low_idle = -1.03E-08 * altitude_ft_in^2 + 8.85E-04 * altitude_ft_in + 2.52E+01
    N1_low_idle = -1.45E-08 * altitude_ft_in^2 + 0.00121 * altitude_ft_in + 25.9
  end

  ----------------------
  --APPROACH (HIGH) Idle
  ----------------------
  --Conditions:
    --Landing Flaps Selected (25 or 30)
    --CON-tinuous Ignition Selected
    --Engine Anti-Ice Selected
    --Reversers deployed

  if (simDR_onGround == 0 and simDR_flap_ratio > 0.667)
    or (simDR_onGround == 0 and B747DR_button_switch_position[44] == 1)  --CONTinuous Ignition
    or (simDR_onGround == 0 and math.max(B747DR_nacelle_ai_valve_pos[0], B747DR_nacelle_ai_valve_pos[1], B747DR_nacelle_ai_valve_pos[2], B747DR_nacelle_ai_valve_pos[3]) == 1)  --Engine A/I
    or (simDR_onGround == 1 and math.max(simDR_reverser_on[0], simDR_reverser_on[1], simDR_reverser_on[2], simDR_reverser_on[3]) == 1) then  --Reversers deployed
      simDR_engine_high_idle_ratio = N1_high_idle_ratio
      ge_approach_idle = true
      
      --Reset to LOW Idle 5 seconds after touchdown (TBD)
  end
  
  if B747DR_log_level >= 1 then
    print("XP High Idle Ratio = ", simDR_engine_high_idle_ratio)
    print("N1 Low Idle - Flight = ", N1_low_idle)
  end

  return N1_low_idle
end



--[[function take_off_thrust_assumed(altitude_ft_in, temperature_K_in)
  local TOGA_corrected_thrust_lbf = 0.0
  local TOGA_actual_thrust_lbf = 0.0
  local TOGA_actual_thrust_N = 0.0
  local approximate_max_TO_thrust_lbf = 0
  local temperature_K = 0.0

  temperature_K = fmsModules["data"]["thrustsel"] + 274.15

  if temperature_K_in > corner_temperature_K then
      TOGA_corrected_thrust_lbf = (-1.79545 * (temperature_K / corner_temperature_K) + 2.7874) * (-0.0000546 * altitude_ft_in^2 + 1.37 * altitude_ft_in + approximate_max_TO_thrust_lbf)
  else
      TOGA_corrected_thrust_lbf = (-0.0000546 * altitude_ft_in^2 + 1.37 * altitude_ft_in + approximate_max_TO_thrust_lbf)
  end

  TOGA_actual_thrust_lbf = TOGA_corrected_thrust_lbf * pressure_ratio
  TOGA_actual_thrust_N = TOGA_actual_thrust_lbf * lbf_to_N

  if B747DR_log_level >= 1 then
    print("\t\t\t\t\t<<<--- Assumed Temp Takeoff Calcs --->>>")
    print("Altitude IN = ", altitude_ft_in)
    print("Temperature K IN = ", temperature_K_in)
    print("Approximate Takeoff Thrust Required = ", approximate_max_TO_thrust_lbf)
    print("TOGA Corrected LBF = ", TOGA_corrected_thrust_lbf)
    print("TOGA Actual LBF = ", TOGA_actual_thrust_lbf)
    print("TOGA Actual N = ", TOGA_actual_thrust_N)
  end

  return TOGA_corrected_thrust_lbf  --, TOGA_actual_thrust_lbf, TOGA_actual_thrust_N
end]]

function take_off_N1_GE(altitude_ft_in)
  local N1_corrected = 0.0
  local N1_actual = 0.0
  local temperature_K = 0.0
  local temperature_ratio = 0.0

  --locals to hold returned values from take_off_thrust_corrected() function
  local TOGA_corrected_thrust_lbf = 0.0
  local TOGA_actual_thrust_N = 0.0

  --For Takeoff, use actual Sim temperature instead of atmosphere() temperature_K
  --If a derate temperature is entered in the THRUST LIM page of the FMC, use that instead
  --if fmsModules["data"]["thrustsel"] ~= "  " then
  --  temperature_K = tonumber(fmsModules["data"]["thrustsel"]) + 273.15
  --else
    temperature_K = simDR_temperature + 273.15
  --end

  temperature_ratio = temperature_K / 288.15

  --get take_off_thrust_corrected() data
  TOGA_corrected_thrust_lbf, _, TOGA_actual_thrust_N = take_off_thrust_corrected(altitude_ft_in, temperature_K)

  --Mach should always be 0 for purposes of this calculation
  mach = 0.0

  N1_corrected = (-1.4446E-12 * mach^2 + 1.2102E-12 * mach + 4.8911E-13) * TOGA_corrected_thrust_lbf^3 + (1.1197E-07 * mach^2 - 7.8717E-08 * mach - 6.498200000000002E-08)
                  * TOGA_corrected_thrust_lbf^2 + (-0.0027 * mach^2 + 0.0011 * mach + 0.0036) * TOGA_corrected_thrust_lbf + (36.93 * mach + 20.96)

  N1_actual = string.format("%4.1f", N1_corrected * math.sqrt(temperature_ratio))

  if B747DR_log_level >= 1 then
    print("\t\t\t\t\t<<<--- TAKEOFF N1 (GE) --->>>")
    print("Altitude IN = ", altitude_ft_in)
    print("Temperature K = ", temperature_K)
    print("Temperature Ratio = ", temperature_ratio)
    print("Mach = ", mach)
    print("TOGA Corrected Thrust = ", TOGA_corrected_thrust_lbf)
    print("N1 Corrected = ", N1_corrected)
    print("N1 Actual = ", N1_actual)
  end

  return N1_actual, TOGA_actual_thrust_N
end

function in_flight_N1_GE(altitude_ft_in, delta_t_isa_K_in)
    local N1_corrected = 0.0
    local N1_actual = 0.0
    local N1_corrected_raw_max_climb = 0.0
    local N1_corrected_mod_max_climb = 0.0
    local N1_real_max_climb = 0.0
    local N1_corrected_raw_max_cruise = 0.0
    local N1_corrected_mod_max_cruise = 0.0
    local N1_real_max_cruise = 0.0
    local climb_rate_fpm = 0
    local climb_angle_deg = 0.0

    local thrust_per_engine_N = 0.0
    local corrected_thrust_lbf = 0.0

    --Due to modeling differences / deficiencies, the climb rates below are tweaked (higher) to
    --allow for more closely matched real-world performance.  Ideal generic target climb rates are:
    --  >> 0 - 10000 ft = 2200fpm
    --  >> 10000 - 20000ft = 2100fpm
    --  >> 20000 - 30000ft = 1700fpm
    if string.match(simConfigData["data"].PLANE.engines, "B1F$") then
      if simDR_altitude < 10000 then
        climb_rate_fpm = 2750
      elseif simDR_altitude <= 20000 then
        --climb_rate_fpm = 2750
        climb_rate_fpm = B747_rescale(10000.0, 2750.0, 20000.0, 2500.0, simDR_altitude)
      elseif simDR_altitude <= 30000 then
        --climb_rate_fpm = 2500
        climb_rate_fpm = B747_rescale(20000.0, 2500.0, 30000.0, 2250.0, simDR_altitude)
      elseif simDR_altitude <= 50000 then
        --climb_rate_fpm = 2000
        climb_rate_fpm = B747_rescale(30000.0, 2250.0, 50000.0, 2250.0, simDR_altitude)
      elseif simDR_altitude > 50000 then
        climb_rate_fpm = 2250
      end
    elseif string.match(simConfigData["data"].PLANE.engines, "B1F1") then
      --For now, use the same climb rates as the B1F until we have specific information for B5F and others
      if simDR_altitude < 10000 then
        climb_rate_fpm = 2875
      elseif simDR_altitude <= 20000 then
        --climb_rate_fpm = 2750
        climb_rate_fpm = B747_rescale(10000.0, 2875.0, 20000.0, 2625.0, simDR_altitude)
      elseif simDR_altitude <= 30000 then
        --climb_rate_fpm = 2500
        climb_rate_fpm = B747_rescale(20000.0, 2625.0, 30000.0, 2375.0, simDR_altitude)
      elseif simDR_altitude <= 50000 then
        --climb_rate_fpm = 2000
        climb_rate_fpm = B747_rescale(30000.0, 2375.0, 50000.0, 2375.0, simDR_altitude)
      elseif simDR_altitude > 50000 then
        climb_rate_fpm = 2375
      end
    elseif string.match(simConfigData["data"].PLANE.engines, "B5F") then
      --For now, use the same climb rates as the B1F until we have specific information for B5F and others
      if simDR_altitude < 10000 then
        climb_rate_fpm = 3000
      elseif simDR_altitude <= 20000 then
        --climb_rate_fpm = 2750
        climb_rate_fpm = B747_rescale(10000.0, 3000.0, 20000.0, 2750.0, simDR_altitude)
      elseif simDR_altitude <= 30000 then
        --climb_rate_fpm = 2500
        climb_rate_fpm = B747_rescale(20000.0, 2750.0, 30000.0, 2500.0, simDR_altitude)
      elseif simDR_altitude <= 50000 then
        --climb_rate_fpm = 2000
        climb_rate_fpm = B747_rescale(30000.0, 2500.0, 50000.0, 2500.0, simDR_altitude)
      elseif simDR_altitude > 50000 then
        climb_rate_fpm = 2250
      end
    else --failsafe option / assume B1F
      if simDR_altitude < 10000 then
        climb_rate_fpm = 2750
      elseif simDR_altitude <= 20000 then
        --climb_rate_fpm = 2750
        climb_rate_fpm = B747_rescale(10000.0, 2750.0, 20000.0, 2500.0, simDR_altitude)
      elseif simDR_altitude <= 30000 then
        --climb_rate_fpm = 2500
        climb_rate_fpm = B747_rescale(20000.0, 2500.0, 30000.0, 2250.0, simDR_altitude)
      elseif simDR_altitude <= 50000 then
        --climb_rate_fpm = 2000
        climb_rate_fpm = B747_rescale(30000.0, 2250.0, 50000.0, 2250.0, simDR_altitude)
      elseif simDR_altitude > 50000 then
        climb_rate_fpm = 2250
      end
  end

    if fmc_alt >= (altitude_ft_in - 200) and B747DR_ref_thr_limit_mode == "CRZ" then
      climb_rate_fpm = 0
    end

    climb_angle_deg = math.asin((0.00508 * climb_rate_fpm) / tas_mtrs_sec) * 180 / math.pi

    --get in_flight_thrust() data
    _, thrust_per_engine_N, _, corrected_thrust_lbf = in_flight_thrust(simDR_acf_weight_total_kg, climb_angle_deg)

    N1_corrected = (-1.4446E-12 * mach^2 + 1.2102E-12 * mach + 4.8911E-13) * corrected_thrust_lbf^3 + (1.1197E-07 * mach^2 - 7.8717E-08 * mach -6.498200000000001E-08)
                    * corrected_thrust_lbf^2 + (-0.0027 * mach^2 + 0.0011 * mach + 0.0036) * corrected_thrust_lbf + (36.93 * mach + 20.96)

    N1_actual = N1_corrected * math.sqrt(temperature_ratio)

    if delta_t_isa_K_in > 0 then
        if delta_t_isa_K_in > 10 then
            N1_corrected_raw_max_climb = -0.25 * delta_t_isa_K_in + 95.8 + (-5.8239E-09 * altitude_ft_in^2 + 0.00077693 * altitude_ft_in)
            N1_corrected_raw_max_cruise = -0.255 * delta_t_isa_K_in + 96 + (-7.5969E-09 * altitude_ft_in^2 + 0.00075761 * altitude_ft_in)
        else
            N1_corrected_raw_max_climb = -0.04 * delta_t_isa_K_in + 96 + (-5.8239E-09 * altitude_ft_in^2 + 0.00077693 * altitude_ft_in)
        end
    else
      N1_corrected_raw_max_climb = 96 + (-5.8239E-09 * altitude_ft_in^2 + 0.00077693 * altitude_ft_in)
      N1_corrected_raw_max_cruise = 93.5 + (-7.5969E-09 * altitude_ft_in^2 + 0.00075761 * altitude_ft_in)
    end
    
    if altitude_ft_in > 24000 then
      N1_corrected_mod_max_climb = N1_corrected_raw_max_climb -3.5722E-08 * altitude_ft_in^2 + 0.002463 * altitude_ft_in - 38.585
      N1_corrected_mod_max_cruise = N1_corrected_raw_max_cruise -3.5722E-08 * altitude_ft_in^2 + 0.002463 * altitude_ft_in - 38.585
    else
      N1_corrected_mod_max_climb = N1_corrected_raw_max_climb
      N1_corrected_mod_max_cruise = N1_corrected_raw_max_cruise
    end
  
    N1_real_max_climb = math.min(N1_corrected_mod_max_climb * math.sqrt(temperature_ratio_adapted), 117.5)
    N1_real_max_cruise = math.min(N1_corrected_mod_max_cruise * math.sqrt(temperature_ratio_adapted), 117.5)
    if N1_actual<89 then
      N1_actual=89
    end
    if B747DR_log_level >= 1 then
      print("\t\t\t\t\t<<<--- IN FLIGHT N1 (GE) --->>>")
      print("Altitude IN = ", altitude_ft_in)
      print("Delta T ISA IN = ", delta_t_isa_K_in)
      print("Temperature Ratio = ", temperature_ratio)
      print("Temperature Ratio Adapted = ", temperature_ratio_adapted)
      print("Mach = ", mach)
      print("Req'd Thrust per Engine N = ", thrust_per_engine_N)
      print("Corrected Thrust LBF = ", corrected_thrust_lbf)
      print("Climb Rate FPM = ", climb_rate_fpm)
      print("Climb Angle = ", climb_angle_deg)
      print("N1 Corrected Raw MAX Climb = ", N1_corrected_raw_max_climb)
      print("N1 Corrected MOD MAX Climb = ", N1_corrected_mod_max_climb)
      print("N1 Real MAX Climb = ", N1_real_max_climb)
      print("N1 Corrected Raw MAX Cruise = ", N1_corrected_raw_max_cruise)
      print("N1 Corrected MOD MAX Cruise = ", N1_corrected_mod_max_cruise)
      print("N1 Real MAX Cruise = ", N1_real_max_cruise)
      print("N1 Corrected = ", N1_corrected)
      print("N1 Actual = ", N1_actual)
    end

    return N1_corrected, N1_actual, N1_corrected_raw_max_climb, N1_corrected_mod_max_climb, N1_real_max_climb, N1_corrected_raw_max_cruise, N1_corrected_mod_max_cruise, N1_real_max_cruise
end

--[[ IN-FLIGHT THRUST -> N1 (EICAS), anchored to the FCOM:
     - at the climb rating thrust (no derate) N1 = FCOM MAXIMUM CLIMB N1 for the TAT and altitude
       (MAX_CLIMB_N1_GE, tbls file),
     - at idle thrust N1 = the idle N1 schedule (engine_idle_control_GE),
     - in between and above (go-around / takeoff thrust) the fan law: N1 - idle grows with the thrust
       fraction ^ p, with p set so that near the rating N1 ~ thrust ^ 0.4 (thrust ~ N1 ^ 2.5).
     The climb rating thrust comes from climb_rating_thrust_lbf (air density, Mach, flat rating). ]]
local function tbl_interp2(T, x, y)
  -- MAX_CLIMB_N1_GE[TAT][altitude]: TAT -40 .. 50 C (5 C), altitude 0 .. 45 000 ft (5 000 ft);
  -- indexed directly (pairs() does not walk XTLua's wrapped global tables); clamped bilinear
  local function br(v, lo, hi, st)
    v = math.max(lo, math.min(hi, v))
    local k0 = lo + math.floor((v - lo) / st) * st
    if k0 >= hi then k0 = hi - st end
    return k0, k0 + st, (v - k0) / st
  end
  local x0, x1, fx = br(x, -40, 50, 5)
  local y0, y1, fy = br(y, 0, 45000, 5000)
  local function at(xk, yk) return tonumber(T[xk][yk]) end
  local a0 = at(x0, y0) + (at(x0, y1) - at(x0, y0)) * fy
  local a1 = at(x1, y0) + (at(x1, y1) - at(x1, y0)) * fy
  return a0 + (a1 - a0) * fx
end
-- (FCOM tables are fixed: interpolation results cached by rounded TAT / altitude)
local tbl_cache = {}
local function clb_table_n1(tat, alt)
  local key = math.floor(tat * 2 + 0.5) * 100000 + math.floor(alt / 100 + 0.5)
  local v = tbl_cache[key]
  if v == nil then
    local ok, r = pcall(tbl_interp2, MAX_CLIMB_N1_GE, math.floor(tat * 2 + 0.5) / 2, math.floor(alt / 100 + 0.5) * 100)
    v = ok and r or false
    tbl_cache[key] = v
  end
  return v or nil
end
local ge_anchor = { t = -1 }
local function ge_anchors(altitude_ft_in)
  if ge_anchor.t == simDRTime and ge_anchor.alt == altitude_ft_in then return ge_anchor end
  ge_anchor.t, ge_anchor.alt = simDRTime, altitude_ft_in
  local Mf = (tas_mtrs_sec or 0) / math.max(200, speed_of_sound or 340)
  local rating = (engine_max_thrust_n or 258400) / lbf_to_N
  local F, d = climb_rating_thrust_lbf(rating, altitude_ft_in, Mf, (simDR_temperature or 15) + 273.15)
  if d > 0 then F = F / B747_rescale(10000.0, 1 - d, 15000.0, 1.0, altitude_ft_in) end
  ge_anchor.f_clb = math.max(1000, F)
  local n1c = clb_table_n1(tonumber(simDR_TAT) or 15, altitude_ft_in)
  local ok = n1c ~= nil
  ge_anchor.n1_clb = n1c or 100
  -- engine specific: the FCOM table is for the CF6-80C2B1F (58 090 lbf). The B1F1 / B5F are the same
  -- fan and core at a higher rating, so the same thrust fraction needs a higher fan speed: the
  -- rating ratio by the fan law (thrust ~ N1 ^ 2.5)
  ge_anchor.n1_clb = math.min(117.5, ge_anchor.n1_clb * (rating / 58090) ^ 0.4)
  ge_anchor.idle = tonumber(engine_idle_control_GE(altitude_ft_in)) or 25.9
  local span = math.max(10, ge_anchor.n1_clb - ge_anchor.idle)
  ge_anchor.p = 0.4 * ge_anchor.n1_clb / span
  return ge_anchor
end
function GE_N1_of_thrust(F_lbf, altitude_ft_in)
  local A = ge_anchors(altitude_ft_in)
  local x = math.max(0, F_lbf) / A.f_clb
  return math.min(117.5, A.idle + (A.n1_clb - A.idle) * x ^ A.p)
end
-- climb rating N1 (as displayed): with derate, and without
function climb_rating_N1_GE(altitude_ft_in, rating_lbf)
  local A = ge_anchors(altitude_ft_in)
  local Mf = (tas_mtrs_sec or 0) / math.max(200, speed_of_sound or 340)
  local F = climb_rating_thrust_lbf(rating_lbf, altitude_ft_in, Mf, (simDR_temperature or 15) + 273.15)
  return GE_N1_of_thrust(F, altitude_ft_in), A.n1_clb
end

local last_thrust_n = {0.0, 0.0, 0.0, 0.0}
local last_N1 = {0.0, 0.0, 0.0, 0.0}
-- the displayed N1 the engine shows at idle in X-Plane, learned per engine while the lever is at
-- idle (X-Plane's idle N1 is not the FCOM idle): the display is mapped so idle reads the FCOM idle
-- and blends smoothly into the normal display by 60 % N1 - no jump when the lever leaves idle
local idle_raw = {}
local last_raw = {}
local idle_t = {}
local GE_IDLE_BLEND_TOP = 60.0
local ge_approach_idle = false
function N1_display_GE(altitude_ft_in, thrust_N_in, n1_factor_in, engine_in)
    local N1_corrected = 0.0
    local N1_actual = 0.0
    local corrected_thrust_N = 0.0
    local corrected_thrust_lbf = 0.0
    local actual_thrust_lbf = 0.0
    local N1_low_idle = 0.0
    if last_N1[engine_in] == nil then last_N1[engine_in] = 0.0 end
    if last_thrust_n[engine_in] == nil then
      last_thrust_n[engine_in] = 0.0
    end

    --Handle display of an engine shutdown
    --[[if simDR_engine_running[engine_in] == 0 then
      thrust_N_in = last_thrust_n[engine_in]
      n1_factor_in = 1.0
      if last_thrust_n[engine_in] > 0 then
        last_thrust_n[engine_in] = last_thrust_n[engine_in] - 100
      elseif last_thrust_n[engine_in] < 0 then
        last_thrust_n[engine_in] = 0.0
      end
    else
      last_thrust_n[engine_in] = thrust_N_in
    end]]
    last_thrust_n[engine_in] = thrust_N_in
    if thrust_N_in < 0.0 then
      thrust_N_in = 0.0
    end

    if n1_factor_in == 0 or n1_factor_in == nil then
      --Failsafe incase n1_factor_in is 0 or nil
      n1_factor_in = 105.0
      takeoff_TOGA_n1 = n1_factor_in
    end

    corrected_thrust_N = thrust_N_in / pressure_ratio
    corrected_thrust_lbf = corrected_thrust_N / lbf_to_N
    
    actual_thrust_lbf = corrected_thrust_lbf * pressure_ratio

    N1_corrected = (-1.4446E-12 * mach^2 + 1.2102E-12 * mach + 4.8911E-13) * corrected_thrust_lbf^3 + (1.1197E-07 * mach^2 - 7.8717E-08 * mach - 6.498200000000002E-08)
    * corrected_thrust_lbf^2 + (-0.0027 * mach^2 + 0.0011 * mach + 0.0036) * corrected_thrust_lbf + (36.93 * mach + 20.96)

    N1_actual = N1_corrected * math.sqrt(temperature_ratio)
    -- in flight: FCOM-anchored thrust -> N1 (see GE_N1_of_thrust)
    if simDR_onGround == 0 then
      local okn, n1a = pcall(GE_N1_of_thrust, thrust_N_in / lbf_to_N, altitude_ft_in)
      if okn and n1a then N1_actual = n1a end
    end

    --Keep the N1 display steady during TO until we manage thrust or 2000 AGL unmanaged
    if (string.match(B747DR_ref_thr_limit_mode, "TO") or (simDR_onGround == 1 and B747DR_ref_thr_limit_mode == "GA"))
      or ((B747DR_ref_thr_limit_mode == "NONE" or B747DR_ref_thr_limit_mode == "") and B747DR_radio_altitude < 2000) then
        N1_actual = simDR_N1[engine_in] * n1_factor_in
    end

    --Engine Idle Logic (Minimum / Approach)
    N1_low_idle = engine_idle_control_GE(altitude_ft_in)
    if simDR_engine_running[engine_in] == 1 and simDR_reverser_on[engine_in] == 0 then
      -- (was: display pinned to the idle value while the lever is at idle, then a jump to the engine's
      -- real value as soon as it moves) - continuous mapping of the real value instead
      local at_idle = (B747DR_throttle[engine_in] or 0) < 0.02
      local ir = idle_raw[engine_in]
      -- settled at idle: X-Plane throttle at idle and N1 steady (< 0.2 %/s) for 3 s - never while
      -- the engine is still running down (that taught a high idle and read below idle on the EICAS)
      local dtp = math.max(0.005, SIM_PERIOD or 0.05)
      local steady = last_raw[engine_in] ~= nil and math.abs(N1_actual - last_raw[engine_in]) / dtp < 0.2
      last_raw[engine_in] = N1_actual
      if at_idle and steady and (simDR_engn_thro_use[engine_in] or 0) < 0.001 then
        idle_t[engine_in] = (idle_t[engine_in] or 0) + dtp
      else
        idle_t[engine_in] = 0
      end
      local settled = idle_t[engine_in] > 3.0
      if at_idle and not ge_approach_idle and settled then
        -- learn slowly (about 3 s) while settled at idle
        if ir == nil then ir = N1_actual else ir = ir + (N1_actual - ir) * 0.03 end
        idle_raw[engine_in] = math.max(tonumber(N1_low_idle) - 4, math.min(tonumber(N1_low_idle) + 10, ir))
        ir = idle_raw[engine_in]
      end
      if ir ~= nil and ir < GE_IDLE_BLEND_TOP - 5 then
        local raw = N1_actual
        local w = (GE_IDLE_BLEND_TOP - N1_actual) / (GE_IDLE_BLEND_TOP - ir)
        if w > 1 then w = 1 elseif w < 0 then w = 0 end
        N1_actual = N1_actual + (tonumber(N1_low_idle) - ir) * w
        -- a running engine at or above its idle never reads below idle
        if raw >= ir - 0.3 and N1_actual < tonumber(N1_low_idle) then N1_actual = tonumber(N1_low_idle) end
      end
    elseif simDR_engine_running[engine_in] == 0 then
      N1_actual = B747_animate_value(last_N1[engine_in],simDR_N1[engine_in],0,115,0.1) 
      --print(" pin N1_actual 1 "..N1_actual.. " n1 "..simDR_N1[engine_in])
    end

    if B747DR_log_level >= 1 then
      print("\t\t\t\t\t<<<--- N1 DISPLAY (GE) --->>>".."\t\tEngine # "..engine_in + 1)
      print("Altitude IN = ", altitude_ft_in)
      print("Thrust IN = ", thrust_N_in)
      print("Pressure Ratio = ", pressure_ratio)
      print("Temperature K = ", temperature_K)
      print("Temperature Ratio = ", temperature_ratio)
      print("Mach = ", mach)
      print("N1_low_idle = ", N1_low_idle)
      print("Corrected Thrust LBF = ", corrected_thrust_lbf)
      print("Actual Thrust LBF = ", actual_thrust_lbf)
      print("TO Factor = ", n1_factor_in)
      print("N1 Actual = ", N1_actual)
      print("Last Thrust In = ", last_thrust_n[engine_in])
    end
    last_N1[engine_in] = N1_actual
    return N1_actual
end
local last_N2 = {0.0, 0.0, 0.0, 0.0}
function N2_display_GE(engine_N1_in, engine_in)
  -- X-Plane's N2 (it spools with the engine, so N2 lags exactly with N1) scaled to the CF6-80C2:
  -- X-Plane idle 71.2% -> 63%, 100% -> 102% (the old N1 formula topped out near 92% and the display
  -- then jumped to X-Plane's value)
  local x = simDR_N2[engine_in] or 0
  local N2_display
  if x <= 71.2 then N2_display = x * (63.0 / 71.2)
  else N2_display = 63.0 + (x - 71.2) * (102.0 - 63.0) / (100.0 - 71.2) end
  if last_N2[engine_in] == nil then last_N2[engine_in] = N2_display end
  last_N2[engine_in] = N2_display
  return N2_display
end

function EGT_display_GE(engine_in)
  local EGT_display = 0.0

  --Use a scaled approach from the default XP (PW) EGT calcs to computing EGT.
  if simDR_engn_EGT_c[engine_in] <= simDR_temperature then
    EGT_display = simDR_temperature
  elseif simDR_engn_EGT_c[engine_in] < 375 then
    EGT_display = simDR_engn_EGT_c[engine_in]
  else
    EGT_display = B747_rescale(0.0, 0.0, 985.0, 985.0, simDR_engn_EGT_c[engine_in])
  end
  -- running: CF6-80C2 EGT from the power (X-Plane's EGT reached only ~460 C at takeoff) - about
  -- 420 C at idle, 880 C at takeoff N1, plus the outside air temperature effect; blended in from the
  -- start (X-Plane's start EGT) as N2 reaches idle
  local n2 = B747DR_display_N2[engine_in] or 0
  if simDR_engine_running[engine_in] == 1 and n2 > 45 then
    local n1 = tonumber(B747DR_display_N1[engine_in]) or 0
    -- power as a fraction of the rated takeoff N1 (never past it: the N1 rises with altitude
    -- at a fixed lever, the EGT margin does not shrink with it)
    local full = math.max(90, tonumber(GE_N1_FULL) or 104)
    local nn = math.max(0, math.min(1.0, (n1 - 26.0) / (full - 26.0)))
    local prof = 420.0 + 440.0 * nn ^ 1.5 + ((simDR_temperature or 15) - 15.0) * 1.2
    local w = math.max(0, math.min(1, (n2 - 45) / 15))
    EGT_display = EGT_display * (1 - w) + prof * w
  end

  if B747DR_log_level >= 1 then
    print("EGT = ", EGT_display)
  end

  return EGT_display
end

local takeoff_TOGA_n1 = 0.0
local orig_thrust_n = 0.0
local N1_target_bug={}
local display_N1_ref={}
local display_N1_max={}
function GE(altitude_ft_in)
	local altitude = 0.0  --round_thrustcalc(simDR_altitude, "ALT")
	local temperature = 0
	local nbr_packs_on = 0
	local packs_adjustment_value = 0.0
	local engine_anti_ice_adjustment_value = 0.0
	local wing_anti_ice_adjustment_value = 0.0
	local takeoff_thrust_n1 = 0.0  --100.0
	local takeoff_thrust_n1_throttle = 0.00
  local N1_display = {}
  local N2_display = {}
  local EGT_display = {}
  

  
  --In-flight variables
  local N1_actual = 0.0
  local N1_real_max_climb = 0.0
  local N1_real_max_cruise = 0.0

  --Setup engine factors based on engine type
	if string.match(simConfigData["data"].PLANE.engines, "B1F$")  then
    engine_max_thrust_n = 258400
    simDR_throttle_max = 1.0
    if orig_thrust_n == 0.0 or B747DR_newsimconfig_data == 1 then
      B747_setMaxThrust(258400)  --(58,090 lbf, flat rated 32C)
    end
    simDR_compressor_area = 4.38251 --(93-inch fan -- 47.17 sq. ft)
	elseif string.match(simConfigData["data"].PLANE.engines, "B5F") then
    engine_max_thrust_n = 276200
    simDR_throttle_max = 1.0
    if orig_thrust_n == 0.0 or B747DR_newsimconfig_data == 1 then
      B747_setMaxThrust(276200)  --(62,100 lbf, 747-400ER rating)
    end
    simDR_compressor_area = 4.38251 --(93-inch fan -- 47.17 sq. ft)
	elseif string.match(simConfigData["data"].PLANE.engines, "B1F1")  then
    engine_max_thrust_n = 267000
    simDR_throttle_max = 1.0
    if orig_thrust_n == 0.0 or B747DR_newsimconfig_data == 1 then
      B747_setMaxThrust(267000)  --(60,030 lbf; B1F uprate, rating not verified)
    end
    simDR_compressor_area = 4.38251 --(93-inch fan -- 47.17 sq. ft)
  else  --Assume CF6-802C-B1F if all else fails
    engine_max_thrust_n = 258400
    simDR_throttle_max = 1.0
    if orig_thrust_n == 0.0 or B747DR_newsimconfig_data == 1 then
      B747_setMaxThrust(258400)  --(58,090 lbf, flat rated 32C)
    end
    simDR_compressor_area = 4.38251 --(93-inch fan -- 47.17 sq. ft)
	end

  --Find current altitude rounded to the closest 1000 feet (for use in table lookups)
  altitude = round_thrustcalc(altitude_ft_in, "ALT")

	--Packs Adjustment
  nbr_packs_on = B747DR_pack_ctrl_sel_pos[0] + B747DR_pack_ctrl_sel_pos[1] + B747DR_pack_ctrl_sel_pos[2]

	if nbr_packs_on == 0 then
		packs_adjustment_value = TOGA_N1_GE_adjustment["3PACKS_OFF"][altitude]
	elseif nbr_packs_on == 1 then
		packs_adjustment_value = TOGA_N1_GE_adjustment["2PACKS_OFF"][altitude]
	else
		packs_adjustment_value = 0.00
	end

	--Engine Anti-Ice Adjustment
	for i = 0, 3 do
		if simDR_engine_anti_ice[i] == 1 then
			engine_anti_ice_adjustment_value = TOGA_N1_GE_adjustment["NACELLE_AI_ON"][altitude]
		end
	end

	--print("Alt = "..altitude)
	--print("Temp = "..temperature)

  if simDR_onGround == 1 or takeoff_thrust_n1==0.0 or B747DR_ref_thr_limit_mode == "" or B747DR_ref_thr_limit_mode == "NONE" then
		takeoff_thrust_n1, _ = take_off_N1_GE(altitude_ft_in)   --TOGA_N1_GE[temperature][altitude] + packs_adjustment_value + engine_anti_ice_adjustment_value
    
    if B747DR_toderate == 0 then
      takeoff_TOGA_n1 = takeoff_thrust_n1
    end

    -- lever for the takeoff N1 on the lever scale (idle N1 .. maximum rated N1)
    local idle_n1 = tonumber(engine_idle_control_GE(altitude_ft_in)) or 25.9
    local togan = tonumber(takeoff_TOGA_n1) or 105
    if togan > idle_n1 + 20 then
      takeoff_thrust_n1_throttle = math.max(0, math.min(1, ((tonumber(takeoff_thrust_n1) or togan) - idle_n1) / (togan - idle_n1)))
    else
      takeoff_thrust_n1_throttle = B747_rescale(0.0, 0.0, togan, 1.0, tonumber(takeoff_thrust_n1))
    end
    
		-- Set N1 Target Bugs & Reference Indicator
    for i = 0, 3 do
      N1_target_bug[i] = math.min(string.format("%4.1f",takeoff_thrust_n1) + packs_adjustment_value + engine_anti_ice_adjustment_value, 117.5)
      display_N1_ref[i] = math.min(string.format("%4.1f",takeoff_thrust_n1) + packs_adjustment_value + engine_anti_ice_adjustment_value, 117.5)
      display_N1_max[i] = math.min(string.format("%4.1f",takeoff_thrust_n1) + packs_adjustment_value + engine_anti_ice_adjustment_value, 117.5)
    end

		B747DR_TO_throttle = takeoff_thrust_n1_throttle
    --print("did init")
    --simCMD_pause:once() 
  end

  if string.match(B747DR_ref_thr_limit_mode, "TO") or B747DR_ref_thr_limit_mode == "" then
    --Store original max engine output
    if orig_thrust_n == 0.0 or B747DR_newsimconfig_data == 1 then
      orig_thrust_n = simDR_thrust_max
    end
  elseif string.match(B747DR_ref_thr_limit_mode, "CLB") then
    -- climb rating (not a target climb rate): the rated climb thrust for the air the engine is in,
    -- given as the N1 the display shows for that thrust (same thrust -> N1 relation as N1_display_GE)
    local _, n1_max_full = climb_rating_N1_GE(altitude_ft_in, engine_max_thrust_n / lbf_to_N)
    N1_real_max_climb = n1_max_full
    local n1_clb = climb_rating_N1_GE(altitude_ft_in, engine_max_thrust_n / lbf_to_N)

		--Set target bugs
		for i = 0, 3 do
      N1_target_bug[i] = math.min(string.format("%4.1f", n1_clb) + packs_adjustment_value + engine_anti_ice_adjustment_value, 117.5)
      display_N1_ref[i] = math.min(string.format("%4.1f", n1_clb) + packs_adjustment_value + engine_anti_ice_adjustment_value, 117.5)
      --pprint(i.." display_N1_ref="..display_N1_ref[i].. "N1_actual "..N1_actual.." N1_real_max_climb ".. N1_real_max_climb .." packs_adjustment_value ".. packs_adjustment_value .." engine_anti_ice_adjustment_value ".. engine_anti_ice_adjustment_value .." takeoff_TOGA_n1 "..takeoff_TOGA_n1)
      if B747DR_display_N1_max[i] < B747DR_display_N1_ref[i] then
        display_N1_max[i] = B747DR_display_N1_ref[i]
      else
        display_N1_max[i] = math.max(display_N1_ref[i],math.min(string.format("%4.1f", N1_real_max_climb) + packs_adjustment_value + engine_anti_ice_adjustment_value, 117.5))
      end
    end
  elseif string.match(B747DR_ref_thr_limit_mode, "CRZ") then
      _, N1_actual, _, _, N1_real_max_climb, _, _, N1_real_max_cruise = in_flight_N1_GE(altitude_ft_in, 0)
      -- maximum climb / maximum cruise from the same climb rating (max cruise a little below max climb)
      do
        local _, n1_full = climb_rating_N1_GE(altitude_ft_in, engine_max_thrust_n / lbf_to_N)
        N1_real_max_climb = n1_full
        N1_real_max_cruise = n1_full - 1.0
      end

      --Set target bugs
      for i = 0, 3 do
          N1_target_bug[i] = math.min(string.format("%4.1f", N1_real_max_cruise) + packs_adjustment_value + engine_anti_ice_adjustment_value, 117.5)
          display_N1_ref[i] = math.min(string.format("%4.1f", N1_real_max_cruise) + packs_adjustment_value + engine_anti_ice_adjustment_value, 117.5)
          display_N1_max[i] = math.min(string.format("%4.1f", N1_real_max_climb), 117.5)
          
      end
  elseif B747DR_ref_thr_limit_mode == "GA" or (simDR_onGround == 0 and B747DR_ref_thr_limit_mode == "" or B747DR_ref_thr_limit_mode == "NONE") then
    --Find current temperature rounded to the closest 5 degrees (for use in table lookups)
    --temperature = round_thrustcalc(simDR_temperature, "TEMP")

    --Find G/A N1 based on current temperature
    temperature = find_closest_temperature(TOGA_N1_GE, simDR_temperature)

    takeoff_TOGA_n1, _ = take_off_N1_GE(altitude_ft_in)

    --Set G/A N1 targets
    for i = 0, 3 do
			N1_target_bug[i], _ = takeoff_TOGA_n1  --TOGA_N1_GE[temperature][altitude] + packs_adjustment_value + engine_anti_ice_adjustment_value
      display_N1_ref[i] = math.min(simDR_N1_target_bug[i], 117.5)
      display_N1_max[i] = math.min(simDR_N1_target_bug[i], 117.5)
		end
  end

    --Handle minor engine thrust differences where calculated (requested) thrust is less than the defined XP engine model can produce
    --Boost engine output as necessary
    if math.max(simDR_engn_thro_use[0], simDR_engn_thro_use[1], simDR_engn_thro_use[2], simDR_engn_thro_use[3]) == 1
      and math.max(simDR_N1[0], simDR_N1[1], simDR_N1[2], simDR_N1[3]) > 99.99
      and (B747DR_display_N1[0] < simDR_N1_target_bug[0] or B747DR_display_N1[1] < simDR_N1_target_bug[1]
          or B747DR_display_N1[2] < simDR_N1_target_bug[2] or B747DR_display_N1[3] < simDR_N1_target_bug[3]) then
            B747_setMaxThrust(simDR_thrust_max + 200)
    elseif simDR_thrust_max > orig_thrust_n and simDR_onGround == 0 and B747DR_radio_altitude > 1500 then  --slowly reset max engine thrust to normal
      if simDR_thrust_max > orig_thrust_n + 10 then
        B747_setMaxThrust(simDR_thrust_max - 10)
      else
        B747_setMaxThrust(simDR_thrust_max - 1)
      end
    end

	--Failsafe option if takeoff_thrust_n1 isn't set
  if takeoff_thrust_n1 == nil then
    takeoff_thrust_n1 = 105.0
    takeoff_TOGA_n1 = 105.0
  end
  
  for i = 0, 3 do
    if B747DR_ap_FMA_autothrottle_mode==5 and B747DR_ap_flightPhase>2 then
      simDR_N1_target_bug[i]=25
    else
      simDR_N1_target_bug[i] = N1_target_bug[i]
    end
    B747DR_display_N1_ref[i] = display_N1_ref[i]
    B747DR_display_N1_max[i] = display_N1_max[i]
  end
	for i = 0, 3 do
    --takeoff_TOGA_n1 (max TO) is used as a factor to compute N1 for TO based on the simDR_N1 to prevent N1 display loss during TO -- in flight it is calculated based on Newtons
    N1_display[i] = string.format("%4.1f", math.min(N1_display_GE(altitude_ft_in, simDR_thrust_n[i], takeoff_TOGA_n1 / 100, i), 118.0))  --use i as a reference for engine number
    B747DR_display_N1[i] = math.max(N1_display[i], 0.0)

    N2_display[i] = string.format("%4.1f", math.min(N2_display_GE(N1_display[i], i), 112.5))  --use N1 as input for calcs
    B747DR_display_N2[i] = math.max(N2_display[i], 0.0)

    EGT_display[i] = EGT_display_GE(i)
    B747DR_display_GE_EGT[i] =egt_thermal_profile(EGT_display[i],i,B747DR_display_GE_EGT[i])--math.max(EGT_display[i], 0.0)

    B747DR_throttle_resolver_angle[i] = throttle_resolver_angle_GE(i)
	end
  -- maximum rated N1 (full forward lever)
  local tgn = tonumber(takeoff_TOGA_n1)
  if tgn and tgn > 60 then GE_N1_FULL = tgn end

  if B747DR_log_level >= 1 then
    print("Takeoff TOGA = ", takeoff_TOGA_n1)
  end
  --Manage Thrust
  throttle_management()
  --thrust_ref_control_N1()
end