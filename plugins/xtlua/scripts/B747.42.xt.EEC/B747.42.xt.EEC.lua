--[[
****************************************************************************************
* Program Script Name	:	B747.42.xt.EEC.lua
* Author Name			:	Marauder28
*							(with SIGNIFICANT contributions from @kudosi for aeronautic formulas)
*   Revisions:
*   -- DATE --	--- REV NO ---		--- DESCRIPTION ---
*   2021-01-11	0.01a				Start of Dev
*	2021-05-27	0.1					Initial Release
*
*
*
*****************************************************************************************
]]

--Load helper files
dofile("json/json.lua")
dofile("B747.42.xt.EEC.tbls.lua")
simDR_version=find_dataref("sim/version/xplane_internal_version")
dofile("pid.lua")
dofile("spool.lua")
local SP = B744SPOOL
local spool_p = { [0] = 0, 0, 0, 0 }
local spool_st = { [0] = {}, {}, {}, {} }
local spool_init = false
simDR_spool_turbine = find_dataref("sim/aircraft/engine/acf_spooltime_turbine")
simDR_spool_jet     = find_dataref("sim/aircraft/engine/acf_spooltime_jet")
local computeRate=0.0333 -- handle low FPS
local lastCompute=0
local lastThrottleCompute=0
local doCompute=0
local throttlePid = newPid()

throttlePid.minout=0
throttlePid.maxout=1.05
throttlePid.target=0
throttlePid.input = 0
throttlePid:compute()
B747DR_pidthrottleP = find_dataref("laminar/B747/flt_ctrls/pid/throttle/p")
B747DR_pidthrottleHP = find_dataref("laminar/B747/flt_ctrls/pid/throttle/highp")
B747DR_pidthrottleLP = find_dataref("laminar/B747/flt_ctrls/pid/throttle/lowp")
B747DR_pidthrottleI = find_dataref("laminar/B747/flt_ctrls/pid/throttle/i")
B747DR_pidthrottleD = find_dataref("laminar/B747/flt_ctrls/pid/throttle/d")
B747DR_pideccI = find_dataref("laminar/B747/flt_ctrls/pid/ecc/N1/i")
B747DR_pideccD = find_dataref("laminar/B747/flt_ctrls/pid/ecc/N1/d")
B747DR_pideccP = find_dataref("laminar/B747/flt_ctrls/pid/ecc/N1/p")
B747DR_pidepr_eccI = find_dataref("laminar/B747/flt_ctrls/pid/ecc/epr/i")
B747DR_pidepr_eccD = find_dataref("laminar/B747/flt_ctrls/pid/ecc/epr/d")
B747DR_pidepr_eccP = find_dataref("laminar/B747/flt_ctrls/pid/ecc/epr/p")
local eccPid ={}
for i = 0, 3 do
	eccPid[i]=newPid()
	eccPid[i].minout=0
	eccPid[i].maxout=1.2
	eccPid[i].target=0
	eccPid[i].input = 0
	eccPid[i]:compute()
end

simDR_ind_airspeed_kts_pilot        = find_dataref("laminar/B747/gauges/indicators/airspeed_kts_pilot")

--B747DR_ap_ias_bug_value            	= find_dataref("laminar/B747/autopilot/ias_bug_value")

--[[
*************************************************************************************
** 				              MISC FUNCTIONS              		    	           **
*************************************************************************************
]]
function deferred_dataref(name,nilType,callFunction)
    if callFunction~=nil then
        print("WARN:" .. name .. " is trying to wrap a function to a dataref -> use xlua")
    end
    return find_dataref(name)
end

function B747_rescale(in1, out1, in2, out2, x)
    if x < in1 then
        return out1
    end
    if x > in2 then
        return out2
    end
    return out1 + (out2 - out1) * (x - in1) / (in2 - in1)
end

function round_thrustcalc(x, input_type)
    --print("x IN = "..x)

    if input_type == "ALT" then
      if x < -2000 then
        x = -2000
      elseif x > 10000 then
        x = 10000
      end
      --print("x Corrected = "..x)
      return x >= 0 and math.floor((x / 1000) + 0.5) * 1000 or math.ceil((x / 1000) - 0.5) * 1000  --Alts in 1000s
    elseif input_type == "ALT-CLB" then
      if x < 0 then
        x = 0
      elseif x > 45000 then
        x = 45000
      end
      --print("x Corrected = "..x)
      return x >= 0 and math.floor((x / 5000) + 0.5) * 5000 or math.ceil((x / 5000) - 0.5) * 5000  --Alts in 5000s
    elseif input_type == "TEMP" then
  --    if x < 10 then
  --      x = 10
  --    elseif x > 70 then
  --      x = 70
  --    end
      --print("x Corrected = "..x)
      return x >= 0 and math.floor((x / 5) + 0.5) * 5 or math.ceil((x / 5) - 0.5) * 5  --Temps by 5's
    end
  end

  function find_closest_temperature(table_in, temp_target)
	local temp_table = {}
	local closest_match
	local last_difference = 99999999
	local counter = 1

	for i, v in pairs(table_in) do
		for i, v in pairs(v) do
			table.insert(temp_table, i)
		end
	end
	table.sort(temp_table)

	for i,v in pairs(temp_table) do
		local distance = math.abs(temp_target - v) --Distance away
		if distance < last_difference then
			closest_match = v
			last_difference = distance
		end
	end

	return closest_match
  end

	function find_closest_weight(table_in, weight_target)
		local temp_table = {}
		local closest_match
		local last_difference = 99999999
		local counter = 1

		for i, v in pairs(table_in) do
			for i, v in pairs(v) do
				table.insert(temp_table, i)
			end
		end
		table.sort(temp_table)

		for i,v in pairs(temp_table) do
			--print("Weight i, v = ", i, v)
			local distance = math.abs(weight_target - v) --Distance away
			if distance < last_difference then
				closest_match = v
				last_difference = distance
			end
		end

		return closest_match
	end

	function find_closest_altitude(table_in, alt_target)
		local temp_table = {}
		local closest_match
		local last_difference = 99999999
		local counter = 1

		for i, v in pairs(table_in) do
			for i, v in pairs(v) do
				for i, v in pairs(v) do
					table.insert(temp_table, i)
				end
			end
--			table.insert(temp_table, i)
		end
		table.sort(temp_table)

		for i,v in pairs(temp_table) do
			--print("Alt i, v = ", i, v)
			local distance = math.abs(alt_target - v) --Distance away
			if distance < last_difference then
				closest_match = v
				last_difference = distance
			end
		end

		return closest_match
	end

--[[
*************************************************************************************
** 				              FIND X-PLANE DATAREFS              		    	   **
*************************************************************************************
]]
--simDR_EEC_button			= find_dataref("laminar/B747/button_switch/position")  --array positions 7,8,9,10
simDRTime					= find_dataref("sim/time/total_running_time_sec")
simDR_onGround				= find_dataref("sim/flightmodel/failures/onground_any")
simDR_altitude				= find_dataref("sim/cockpit2/gauges/indicators/altitude_ft_pilot")
simDR_temperature			= find_dataref("sim/cockpit2/temperature/outside_air_temp_degc")
simDR_TAT					= find_dataref("sim/cockpit2/temperature/outside_air_LE_temp_degc")
simDR_EPR					= find_dataref("sim/flightmodel/engine/ENGN_EPR")
simDR_EPR_target_bug		= find_dataref("sim/cockpit2/engine/actuators/EPR_target_bug")
simDR_N1					= find_dataref("sim/flightmodel/engine/ENGN_N1_")
simDR_N1_target_bug			= find_dataref("sim/cockpit2/engine/actuators/N1_target_bug")
simDR_N2					= find_dataref("sim/flightmodel/engine/ENGN_N2_")
simDR_heating				= find_dataref("sim/flightmodel2/engines/engine_is_burning_fuel")
simDR_throttle_ratio		= find_dataref("sim/cockpit2/engine/actuators/throttle_ratio")
--simDR_throttle_ratio	= find_dataref("sim/cockpit2/engine/actuators/hardware_throttle_ratio")
simDR_engine_throttle_jet_all   = find_dataref("sim/cockpit2/engine/actuators/throttle_jet_rev_ratio_all")
simDR_hardware_throttle_ratio	= find_dataref("sim/cockpit2/engine/actuators/hardware_throttle_ratio")
simDR_has_hardware_throttle	= find_dataref("sim/joystick/has_throttle")
simDR_override_throttles	= find_dataref("sim/operation/override/override_throttles")
simDR_engn_thro				= find_dataref("sim/flightmodel/engine/ENGN_thro")
simDR_engn_thro_use			= find_dataref("sim/flightmodel/engine/ENGN_thro_use")
simDR_throttle_max			= find_dataref("sim/aircraft/engine/acf_throtmax_FWD")
simDR_acf_weight_total_kg   = find_dataref("sim/flightmodel/weight/m_total")
simDR_ias_pilot				= find_dataref("sim/cockpit2/gauges/indicators/airspeed_kts_pilot")
simDR_vvi_fpm_pilot        	= find_dataref("sim/cockpit2/gauges/indicators/vvi_fpm_pilot")
simDR_tas_pilot				= find_dataref("sim/cockpit2/gauges/indicators/true_airspeed_kts_pilot")
simDR_flap_ratio			= find_dataref("sim/cockpit2/controls/flap_ratio")

simDR_flap_handle_ratio		= find_dataref("sim/cockpit2/controls/flap_handle_deploy_ratio")
simDR_thrust_max			= find_dataref("sim/aircraft/engine/acf_tmax")
simDR_thrust_max_engine			= find_dataref("sim/aircraft/engine/acf_tmax_per_engine")
simDR_thrust_n				= find_dataref("sim/cockpit2/engine/indicators/thrust_dry_n")
simDR_engine_anti_ice		= find_dataref("laminar/B747/antiice/nacelle/valve_pos")
simDR_acceleration_kts_sec_pilot    = find_dataref("sim/cockpit2/gauges/indicators/airspeed_acceleration_kts_sec_pilot")
simDR_engn_EGT_c			= find_dataref("sim/flightmodel/engine/ENGN_EGT_c")
simDR_engine_high_idle_ratio	= find_dataref("sim/aircraft2/engine/high_idle_ratio")
simDR_rpm					= find_dataref("sim/cockpit2/engine/indicators/engine_speed_rpm")
simDR_reverser_on			= find_dataref("sim/cockpit2/annunciators/reverser_on")
simDR_reverser_deploy_ratio = find_dataref("sim/flightmodel2/engines/thrust_reverser_deploy_ratio")
simDR_reverser_max			= find_dataref("sim/aircraft/engine/acf_throtmax_REV")
simDR_prop_mode                 = find_dataref("sim/cockpit2/engine/actuators/prop_mode")
simDR_engine_running		= find_dataref("sim/flightmodel/engine/ENGN_running")
simDR_compressor_area		= find_dataref("sim/aircraft/engine/acf_face_jet")
B747DR_autothrottle_active	= find_dataref("laminar/B747/engines/autothrottle_active")
--simDR_autothrottle_on		= find_dataref("sim/cockpit2/autopilot/autothrottle_on")
simDR_engine_starter_status	= find_dataref("sim/flightmodel2/engines/starter_is_running")
B747DR_ap_autoland            	= deferred_dataref("laminar/B747/autopilot/autoland", "number")
B747DR_airspeed_Vmc                             = deferred_dataref("laminar/B747/airspeed/Vmc", "number")
debug_ecc     = deferred_dataref("laminar/B747/debug/ecc", "number")
--[[
*************************************************************************************
** 				              FIND X-PLANE COMMANDS              		    	   **
*************************************************************************************
]]
--simCMD_autopilot_autothrottle_off		= find_command("sim/autopilot/autothrottle_off")
simCMD_ThrottleUp			= find_command("sim/engines/throttle_up")
simCMD_ThrottleDown			= find_command("sim/engines/throttle_down")

--[[
*************************************************************************************
** 				              CUSTOM READ/WRITE DATAREFS           		    	   **
*************************************************************************************
]]
B747DR_button_switch_position	= deferred_dataref("laminar/B747/button_switch/position")
--[[
	7 - 10	= EEC buttons
	44		= CONT button
]]
--array positions 7,8,9,10
B747DR_pack_ctrl_sel_pos		= deferred_dataref("laminar/B747/air/pack_ctrl/sel_dial_pos", "array[3]")
B747DR_nacelle_ai_valve_pos     = deferred_dataref("laminar/B747/antiice/nacelle/valve_pos", "array[4)")
B747DR_TO_throttle				= deferred_dataref("laminar/B747/engines/thrustref_throttle", "number")
B747DR_ap_FMA_autothrottle_mode	= deferred_dataref("laminar/B747/autopilot/FMA/autothrottle_mode", "number")
B747DR_afds_thr_target			= find_dataref("laminar/B747/afds/thr_target")
--[[
    0 = NONE
    1 = HOLD
    2 = IDLE
    3 = SPD
    4 = THR
    5 = THR REF
--]]
B747DR_ap_autothrottle_armed	= deferred_dataref("laminar/B747/autothrottle/armed", "number")
B747DR_engine_TOGA_mode			= deferred_dataref("laminar/B747/engines/TOGA_mode", "number")
B747DR_ref_thr_limit_mode		= deferred_dataref("laminar/B747/engines/ref_thr_limit_mode", "string")
--[[
    ["NONE"]
    ["TO"]
    ["TO 1"]
    ["TO 2"]
    ["D-TO"]
    ["D-TO 1"]
    ["D-TO 2"]
    ["CLB"]
    ["CLB 1"]
    ["CLB 2"]
    ["CRZ"]
    ["CON"]
    ["GA"]
]]
B747DR_ap_FMA_active_pitch_mode     	= deferred_dataref("laminar/B747/autopilot/FMA/active_pitch_mode", "number")
--[[
    0 = NONE
    1 = TOGA
    2 = G/S
    3 = FLARE
    4 = VNAV SPD
    5 = VNAV ALT
    6 = VNAV PATH
    7 = V/S
    8 = FLCH SPD
    9 = ALT
--]]
--[[mSparks, placeholder for RR displays]]
simDR_engine_N1_pct                 = find_dataref("sim/cockpit2/engine/indicators/N1_percent")
simDR_engine_N2_pct                 = find_dataref("sim/cockpit2/engine/indicators/N2_percent")
simDR_engine_EPR					= find_dataref("sim/cockpit2/engine/indicators/EPR_ratio")
--[[end mSparks]]

B747DR_display_N1					= deferred_dataref("laminar/B747/engines/display_N1", "array[4]")
B747DR_display_N1_ref				= deferred_dataref("laminar/B747/engines/display_N1_ref", "array[4]")
B747DR_display_N1_max				= deferred_dataref("laminar/B747/engines/display_N1_max", "array[4]")
B747DR_display_N2					= deferred_dataref("laminar/B747/engines/display_N2", "array[4]")
B747DR_display_N3					= deferred_dataref("laminar/B747/engines/display_N3", "array[4]")
B747DR_display_EPR					= deferred_dataref("laminar/B747/engines/display_EPR", "array[4]")
B747DR_display_EPR_ref				= deferred_dataref("laminar/B747/engines/display_EPR_ref", "array[4]")
B747DR_display_EPR_max				= deferred_dataref("laminar/B747/engines/display_EPR_max", "array[4]")
B747DR_display_GE_EGT				= deferred_dataref("laminar/B747/engines/display_GE_EGT", "array[4]")
B747DR_display_EGT					= deferred_dataref("laminar/B747/engines/display_EGT", "array[4]")
B747DR_FMSdata						= deferred_dataref("laminar/B747/fms/data", "string")
B747DR_radio_altitude				= deferred_dataref("laminar/B747/efis/radio_altitude")
simDR_radarAlt1           	= find_dataref("sim/cockpit2/gauges/indicators/radio_altimeter_height_ft_pilot")
B747DR_altitude_dial				= deferred_dataref("laminar/B747/autopilot/heading/altitude_dial_ft")
B747DR_ap_flightPhase 				= deferred_dataref("laminar/B747/autopilot/flightPhase", "number")
B747DR_toderate						= deferred_dataref("laminar/B747/engine/derate/TO","number")
B747DR_clbderate					= deferred_dataref("laminar/B747/engine/derate/CLB","number")
B747DR_ref_line_magenta				= deferred_dataref("laminar/B747/engines/display_ref_line_magenta", "number")
B747DR_throttle_resolver_angle 		= deferred_dataref("laminar/B747/engines/TRA", "array[4]")
B747DR_throttle						= deferred_dataref("laminar/B747/engine/throttle", "array[4]")
B747DR_throttle_reversor			= deferred_dataref("laminar/B747/engine/throttle_reversor", "array[4]")
B747DR_engineType					= deferred_dataref("laminar/B747/engines/type", "number")

-- Holds all SimConfig options
B747DR_simconfig_data				= deferred_dataref("laminar/B747/simconfig", "string")
B747DR_newsimconfig_data				= deferred_dataref("laminar/B747/newsimconfig", "number")
--FMS data
B747DR_FMSdata						= deferred_dataref("laminar/B747/fms/data", "string")
B747DR_autothrottle_fail			= deferred_dataref("laminar/B747/engines/autothrottle_fail", "number")
B747DR_ap_vvi_fpm					= deferred_dataref("laminar/B747/autopilot/vvi_fpm")
simDR_autopilot_airspeed_kts = find_dataref("sim/cockpit2/autopilot/airspeed_dial_kts")
B747DR_airspeed_pilot				= deferred_dataref("laminar/B747/gauges/indicators/airspeed_kts_pilot")

--[[
*************************************************************************************
** 				              GLOBAL VARIABLES           		    	 		   **
*************************************************************************************
]]
--Constants
lbf_to_N = 4.4482216
mtrs_per_sec = 1.94384

--Logging On/Off
B747DR_log_level = deferred_dataref("laminar/B747/engines/logging", "number")  --true / false

--Simulator Config Options
simConfigData = {}

--FMS data
fmsModules = {}
fms_data = {}
fmc_alt = 0

--Atmosphere
altitude_ft = 0.0 --simDR_altitude
altitude_mtrs = 0.0 --altitude_ft * 0.3048
pressure_pa = 0.0  --pascals
pressure_ratio = 0.0
density = 0.0  --kg per m^3
temperature_K = 0.0  --kelvin
temperature_C = 0.0  --celcius
corner_temperature_K = 0.0  --kelvin
temperature_ratio = 0.0
temperature_ratio_adapted = 0.0
speed_of_sound = 0.0  --mtrs per sec
sigma_density_ratio = 0.0

--Flight Coefficients
mach = 0.0
cL = 0.0
cD = 0.0
--climb_angle_deg = 0.0
tas_mtrs_sec = 0.0
--acceleration_kts_sec = simDR_acceleration_kts_sec_pilot / 1.94384
--acceleration_mtrs_sec = 0.0  --mtrs per sec^2

--EEC Status Flag
EEC_status = 0

--General Engine Parameters
engine_max_thrust_n = 0

takeoff_TOGA_n1 = 0.0
takeoff_TOGA_EPR = 0.0

--[[
*************************************************************************************
** 				              LOCAL VARIABLES           		    	 		   **
*************************************************************************************
]]


--[[
*************************************************************************************
** 				              GLOBAL CODE                  		    	 		   **
*************************************************************************************
]]
if string.len(B747DR_simconfig_data) > 1 then
	simConfigData["data"] = json.decode(B747DR_simconfig_data)
else
	simConfigData["data"] = json.decode("[]")
end

if string.len(B747DR_FMSdata) > 1 then
	fmsModules["data"] = json.decode(B747DR_FMSdata)
else
	fmsModules["data"] = json.decode("[]")
end

B747DR_ref_line_magenta = 0
function B747_animate_value(current_value, target, min, max, speed)

    local fps_factor = math.min(0.1, speed * SIM_PERIOD)
	local nextValue=current_value + ((target - current_value) * fps_factor)
    if nextValue >= (max - 0.001) then
        return max
    elseif nextValue <= (min + 0.001) then
       return min
    else
        return nextValue
    end

end
function B747_setMaxThrust(value)
	--[[simDR_thrust_max_engine[0]=value
	simDR_thrust_max_engine[1]=value
	simDR_thrust_max_engine[2]=value
	simDR_thrust_max_engine[3]=value]]--
	simDR_thrust_max=value
end
--[[
*************************************************************************************
** 				              AERODYNAMIC FUNCTIONS (GLOBAL)	    	           **
*************************************************************************************
]]
function atmosphere(altitude_ft_in, delta_t_isa_K_in)
    local altitude_mtrs = altitude_ft_in * 0.3048

    if altitude_mtrs < 11000 then
        pressure_pa = (1-0.00651 * altitude_mtrs / 288.15)^5.255 * 101325
        temperature_K = 288.15 - 6.5 / 1000 * altitude_mtrs
    else
        pressure_pa = 22632 * 2.71828^(-9.81 * (altitude_mtrs - 11000) / (287 *216.65))
        temperature_K = 216.65
    end

	temperature_K = simDR_temperature + 273.15
    pressure_ratio = pressure_pa / 101325
    density = pressure_pa / (287 * temperature_K)
    temperature_C = temperature_K - 273.15
    corner_temperature_K = 7.31E-12 * altitude_ft_in^3 - 7.73E-08 * altitude_ft_in^2 - 0.00216 * altitude_ft_in + 305
    temperature_ratio = (temperature_K + delta_t_isa_K_in) / 288.15
    temperature_ratio_adapted = (temperature_K + delta_t_isa_K_in + 5) / 288.15
    speed_of_sound = math.sqrt(287 * 1.4 * temperature_K)
	sigma_density_ratio = pressure_pa / (287.058 * temperature_K * 1.225)

    if B747DR_log_level >= 1 then
		print("\t\t\t\t\t<<<--- ATMOSPHERE --->>>")
		print("Altitude IN = ", altitude_ft_in)
		print("Delta T ISA K IN = ", delta_t_isa_K_in)
		print("Altitude MTRS = ", altitude_mtrs)
		print("Pressure PA = ", pressure_pa)
		print("Pressure Ratio = ", pressure_ratio)
		print("Density = ", density)
		print("Sigma Density Ratio", sigma_density_ratio)
		print("Temperature K = ", temperature_K)
		print("Temperature C = ", temperature_C)
		print("Corner Temperature K = ", corner_temperature_K)
		print("Temperature Ratio = ", temperature_ratio)
		print("Temperature Ratio Adapted = ", temperature_ratio_adapted)
		print("Speed of Sound = ", speed_of_sound)
    --print("\n")
	end

    --return altitude_mtrs, pressure_pa, pressure_ratio, density, temperature_K, temperature_C, corner_temperature_K, temperature_ratio, temperature_ratio_adapted, speed_of_sound
end

function flight_coefficients(gw_kg_in, tas_kts_in)
	local flaps_incremental_drag = 0.0

	if tonumber(string.format("%4.3f", simDR_flap_ratio)) == 0.0 then
		flaps_incremental_drag = 0.0
	elseif tonumber(string.format("%4.3f", simDR_flap_ratio)) == 0.167 then  --Flaps 1
		flaps_incremental_drag = 0.008
	elseif tonumber(string.format("%4.3f", simDR_flap_ratio)) == 0.333 then  --Flaps 5
		flaps_incremental_drag = 0.018
	elseif tonumber(string.format("%4.3f", simDR_flap_ratio)) == 0.5 then  --Flaps 10
		flaps_incremental_drag = 0.018
	elseif tonumber(string.format("%4.3f", simDR_flap_ratio)) == 0.667 then  --Flaps 20
		flaps_incremental_drag = 0.028
	elseif tonumber(string.format("%4.3f", simDR_flap_ratio)) == 0.833 then  --Flaps 25 & Gear Down
		flaps_incremental_drag = 0.088
	elseif tonumber(string.format("%4.3f", simDR_flap_ratio)) == 1.0 then  --Flaps 30  & Gear Down
		flaps_incremental_drag = 0.108
	end

	--Removed the above flaps drag code since it was doing weird things to the initial climb calculations.  Now just use minimal flaps drag regardless of the flaps setting.
	--if tonumber(string.format("%4.3f", simDR_flap_ratio)) > 0.0 then
	--	flaps_incremental_drag = 0.008
	--else
	--	flaps_incremental_drag = 0.0
	--end

	tas_mtrs_sec = tas_kts_in / mtrs_per_sec
    mach = tas_mtrs_sec / speed_of_sound
    cL = (gw_kg_in * 9.81) / (0.5 * density * tas_mtrs_sec^2 * 511)

    if mach > 0.7 then
        cD = -1.881 + 7.115 * mach + 0.5293 * cL - 8.928 * mach^2 -1.091 * mach * cL -0.2917 * cL^2 +3.746 * mach^3 + 0.5522 * mach^2 * cL + 0.3291 * mach * cL^2 + 0.1029 * cL^3 + flaps_incremental_drag
    else
        cD = 0.106 * cL^2 - 0.0463 * cL + 0.0219 + flaps_incremental_drag
    end

    --climb_angle_deg = math.asin(0.00508 * climb_rate_fpm_in / tas_mtrs_sec) * 180 / math.pi
    --acceleration_mtrs_sec = acceleration_kts_sec_in / 1.94384

	if B747DR_log_level >= 1 then
		print("\t\t\t\t\t<<<--- FLIGHT COEFFICIENTS --->>>")
		print("Gross Weight IN = ", gw_kg_in)
		print("Mach = ", mach)
		print("Coefficient of Lift = ", cL)
		print("Coefficient of Drag = ", cD)
		print("Flap Handle = ", simDR_flap_ratio)
		print("Flaps Incremental Drag = ", flaps_incremental_drag)
		--print("Climb Angle Degrees = ", climb_angle_deg)
		print("TAS MTRS Sec = ", tas_mtrs_sec)
		--print("Acceleration MTRS Sec = ", acceleration_mtrs_sec)
		--print("\n")
	end

    --return mach, cL, cD, climb_angle_deg, tas_mtrs_sec, acceleration_mtrs_sec
	--return mach, cL, cD, tas_mtrs_sec
end


--[[
*************************************************************************************
** 				              EEC FUNCTIONS              		    	           **
*************************************************************************************
]]
function egt_thermal_profile(tempIn,engineid,current_value)
	--B747DR_display_EGT[i]

	local thermalIn=tempIn
	--[[if tempIn<320 and tempIn>120 then
		thermalIn=320
	else]]--
	if tempIn<100 and simDR_engine_starter_status[engineid]<0.5 and simDR_heating[engineid]<0.5 then
		thermalIn=current_value-10
	elseif tempIn<150 and tempIn>100 and simDR_heating[engineid]>0.5 then
			thermalIn=150
	end
	local div=120
	if simDR_N1[engineid]<29 and simDR_N2[engineid]>25 and simDR_heating[engineid]>0 then
		div=1050
	end

	--freeze engine
	if simDR_N2[engineid]<22 and simDR_engine_starter_status[engineid]<0.5 and simDR_heating[engineid]<0.5 then
	 simDR_N2[engineid]=0
	end

	-- (time based: the per-frame divisor made the lag depend on the frame rate)
	local rate
	if div == 120 then rate = (thermalIn - current_value) * math.min(1, (SIM_PERIOD or 0.05) / 2.5)
	else rate = (thermalIn - current_value) / div end
	local tempOut=current_value+rate
	--print(engineid.."="..tempIn.." to "..tempOut.." at ".. rate)
	return math.max(simDR_temperature,tempOut)--current_value+rate
end
function clear_thrust_targets()
	-- Clear Thrust Target Bugs
	for i = 0, 8 do
		simDR_EPR_target_bug[i] = 0.0
		simDR_N1_target_bug[i] = 0.0
	end
end

function take_off_thrust_corrected(altitude_ft_in, temperature_K_in)
	local TOGA_corrected_thrust_lbf = 0.0
	local TOGA_actual_thrust_lbf = 0.0
	local TOGA_actual_thrust_N = 0.0
	local approximate_max_TO_thrust_lbf = 0

	--Approximate TOGA Max thrust
	--  GE CFG-802C-B1F = 57160 lbf (Engine Max = 58000 lbf / 258000 Newtons)
	--  GE CFG-802C-B5F = 60030 lbf (Engine Max = 60800 lbf / 270500 Newtons)
	--  PW4056 = 56750 lbf (Engine Max = 56750 lbf / 252500 Newtons)
	--  PW4060 = 60000 lbf (Engine Max = 60000 lbf / 266900 Newtons)
	--  PW4062 = 62000 lbf (Engine Max = 62000 lbf / 275800 Newtons)
	--  RR RB211-524G = 56870 lbf (Engine Max = 58000 lbf / 258000 Newtons)
	--  RR RB211-524H = 59450 lbf (Engine Max = 60600 lbf / 269600 Newtons)

	-- (B1F1 tested before B1F: "B1F" also matches "B1F1")
	if string.match(simConfigData["data"].PLANE.engines, "B1F1")  then
	  approximate_max_TO_thrust_lbf = 60030
	elseif string.match(simConfigData["data"].PLANE.engines, "B1F")  then
		approximate_max_TO_thrust_lbf = 57160
	elseif string.match(simConfigData["data"].PLANE.engines, "B5F")  then
		approximate_max_TO_thrust_lbf = 60030
	elseif string.match(simConfigData["data"].PLANE.engines, "4056") then
	    approximate_max_TO_thrust_lbf = 56750
	elseif string.match(simConfigData["data"].PLANE.engines, "4060")  then
	    approximate_max_TO_thrust_lbf = 60000
	elseif string.match(simConfigData["data"].PLANE.engines, "4062")  then
	    approximate_max_TO_thrust_lbf = 62000
	elseif string.match(simConfigData["data"].PLANE.engines, "524G")  then
	    approximate_max_TO_thrust_lbf = 56870
	elseif string.match(simConfigData["data"].PLANE.engines, "524H")  then
	    approximate_max_TO_thrust_lbf = 59450
	else
	    approximate_max_TO_thrust_lbf = 56500  --failsafe option
	end

	if temperature_K_in > corner_temperature_K then
		TOGA_corrected_thrust_lbf = (-1.79545 * (temperature_K_in / corner_temperature_K) + 2.7874) * (-0.0000546 * altitude_ft_in^2 + 1.37 * altitude_ft_in + approximate_max_TO_thrust_lbf)
	else
		TOGA_corrected_thrust_lbf = (-0.0000546 * altitude_ft_in^2 + 1.37 * altitude_ft_in + approximate_max_TO_thrust_lbf)
	end

	if B747DR_toderate == 1 then
	  TOGA_corrected_thrust_lbf = TOGA_corrected_thrust_lbf * 0.9
	elseif B747DR_toderate == 2 then
	  TOGA_corrected_thrust_lbf = TOGA_corrected_thrust_lbf * 0.8
	end

	TOGA_actual_thrust_lbf = TOGA_corrected_thrust_lbf * sigma_density_ratio  --pressure_ratio
	TOGA_actual_thrust_N = TOGA_actual_thrust_lbf * lbf_to_N

	if B747DR_log_level >= 1 then
	  print("\t\t\t\t\t<<<--- Takeoff Calcs --->>>")
	  print("Altitude IN = ", altitude_ft_in)
	  print("Temperature K IN = ", temperature_K_in)
	  print("Approximate Takeoff Thrust Required = ", approximate_max_TO_thrust_lbf)
	  print("TOGA Corrected LBF = ", TOGA_corrected_thrust_lbf)
	  print("TOGA Actual LBF = ", TOGA_actual_thrust_lbf)
	  print("TOGA Actual N = ", TOGA_actual_thrust_N)
	end

	return TOGA_corrected_thrust_lbf, TOGA_actual_thrust_lbf, TOGA_actual_thrust_N
end

--[[ CLIMB RATING THRUST (per engine, lbf).
     Climb thrust is a rating, not a target climb rate: the EEC holds the climb rating and the climb
     rate is whatever weight, drag and the air allow. Lapse with the air (Daidzic, high-bypass
     turbofan): F = F_TO * k_clb * sigma^m * (1 + b1 M sqrt(theta) + b2 M^2 theta), sigma^m in the
     troposphere and proportional to density above it; flat rated (no more thrust on a cold day) up to
     ISA + 10 C, less above it. CLB 1 / CLB 2 (-10 % / -20 %) wash out between 10 000 and 15 000 ft
     (FCOM 7/11.32.3). Calibrated to the published 747-400 climb (390 t, about 18-20 min to the optimum
     altitude) - the lapse exponent and climb rating fraction are the calibration. ]]
CLB_K, CLB_M, CLB_CORNER = 0.88, 0.80, 10.0
local function isa_T(alt_ft)
  local h = alt_ft * 0.3048
  if h < 11000 then return 288.15 - 0.0065 * h end
  return 216.65
end
local function isa_sigma(alt_ft, dT)
  local h = alt_ft * 0.3048
  local T, p
  if h < 11000 then T = 288.15 - 0.0065 * h; p = 101325 * (T / 288.15) ^ 5.25588
  else T = 216.65; p = 22632 * math.exp(-9.80665 * (h - 11000) / (287.05 * 216.65)) end
  return p / (287.05 * (T + dT)) / 1.225, (T + dT) / 288.15
end
function climb_rating_thrust_lbf(rating_lbf, alt_ft, mach_in, oat_K)
  local dT = (oat_K or isa_T(alt_ft)) - isa_T(alt_ft)
  local sig, th = isa_sigma(alt_ft, math.max(0, dT - CLB_CORNER))
  local s
  if alt_ft < 36089 then s = sig ^ CLB_M
  else
    local s11 = isa_sigma(36089, math.max(0, dT - CLB_CORNER))
    s = s11 ^ CLB_M * (sig / s11)
  end
  local M = math.max(0, mach_in or 0)
  local b1, b2 = -8.5e-4 * 1116.45, 5.5e-7 * 1116.45 ^ 2
  local F = rating_lbf * CLB_K * s * (1 + b1 * M * math.sqrt(th) + b2 * M * M * th)
  local d = 0
  if B747DR_clbderate == 1 then d = 0.10 elseif B747DR_clbderate == 2 then d = 0.20 end
  if d > 0 then F = F * B747_rescale(10000.0, 1 - d, 15000.0, 1.0, alt_ft) end
  return math.max(0, F), d
end

function in_flight_thrust(gw_kg_in, climb_angle_deg_in)
	local total_thrust_required_N = 0.0
	local thrust_per_engine_N = 0.0
	local corrected_thrust_N = 0.0
	local corrected_thrust_lbf = 0.0

	total_thrust_required_N = 0.5 * cD * tas_mtrs_sec^2 * density * 511 + math.sin(climb_angle_deg_in / 180 * math.pi) * gw_kg_in * 9.81

	if B747DR_clbderate == 1 then
		total_thrust_required_N = total_thrust_required_N * B747_rescale(10000.0, 0.9, 15000.0, 1.0, simDR_altitude)  --0.9  --Scale linearly from CLB1 to CLB from 10K to 15K ft  FCOM 7/11.32.3
	elseif B747DR_clbderate == 2 then
		total_thrust_required_N = total_thrust_required_N * B747_rescale(10000.0, 0.8, 15000.0, 1.0, simDR_altitude)  --0.8  --Scale linearly from CLB1 to CLB from 10K to 15K ft
	end

	thrust_per_engine_N = total_thrust_required_N / 4

	corrected_thrust_N = thrust_per_engine_N / pressure_ratio
	corrected_thrust_lbf = corrected_thrust_N / lbf_to_N

	if B747DR_log_level >= 1 then
		print("\t\t\t\t<<<--- IN FLIGHT THRUST --->>>")
		print("Gross Weight IN = ", gw_kg_in)
		print("Climb Angle IN = ", climb_angle_deg_in)
		print("Pressure Ratio = ", pressure_ratio)
		print("Density = ", density)
		print("Coefficient of Drag = ", cD)
		print("TAS MTRS Sec = ", tas_mtrs_sec)
		print("Total Thrust Required N = ", total_thrust_required_N)
		print("Thrust per Engine N = ", thrust_per_engine_N)
		print("Corrected Thrust N = ", corrected_thrust_N)
		print("Corrected Thrust LBF = ", corrected_thrust_lbf)
	end

	return total_thrust_required_N, thrust_per_engine_N, corrected_thrust_N, corrected_thrust_lbf
end
lastNewTargetModeTime=0
lastNewTarget=""
function ecc_mode_set()
	newTarget=""
	--Set Specific sub-mode for TO or CLB
	--if (B747DR_ap_autoland==-2 or B747DR_ap_FMA_active_roll_mode ==3 ) and simDR_flap_ratio_control>0 then
	--if simDR_altitude>32000 then
	--	newTarget = "CRZ"
	--else
	if B747DR_ap_autoland==-2 or (B747DR_ap_flightPhase >= 2 and simDR_flap_ratio > 0 and simDR_radarAlt1<1500) then
		newTarget = "GA"
	elseif B747DR_ap_flightPhase==0 then
		if B747DR_toderate == 0 then
			newTarget = "TO"
		elseif B747DR_toderate == 1 then
			newTarget = "TO 1"
		elseif B747DR_toderate == 2 then
			newTarget = "TO 2"
		end
	elseif B747DR_ap_flightPhase==1 then
		if B747DR_clbderate == 0 then
			newTarget = "CLB"
		elseif B747DR_clbderate == 1 then
			newTarget = "CLB 1"
		elseif B747DR_clbderate == 2 then
			newTarget = "CLB 2"
		end
	elseif B747DR_ap_flightPhase>=2 then
		newTarget = "CRZ"
	end
	if newTarget~=lastNewTarget then
		lastNewTarget=newTarget
		lastNewTargetModeTime=simDRTime
	end

	if simDRTime-lastNewTargetModeTime>0.25 then
		B747DR_ref_thr_limit_mode=newTarget
	end
end
function B747_interpolate_value(current_value, target, min, max, speed)--speed in sex min->max
	local change = ((max-min)/speed)*(SIM_PERIOD)
	newValue=current_value

	  if newValue<=target then
	  newValue=newValue+change
	  if newValue >= target then
		newValue = B747_animate_value(current_value,target,min,max,100)
	  end
	elseif newValue>target then
	  newValue=newValue-change
	  if newValue <= target then
		newValue = B747_animate_value(current_value,target,min,max,100)
	  end
	end
	if newValue <= min then
	  newValue = B747_animate_value(current_value,min,min,max,100)
	elseif newValue >= max then
	  newValue = B747_animate_value(current_value,max,min,max,100)
	else
	  if math.abs(current_value-target) < change then
		newValue = B747_animate_value(current_value,target,min,max,100)
	  end
	end
	return newValue
  end

function round(x)
	return x>=0 and math.floor(x+0.5) or math.ceil(x-0.5)
end
-- X-Plane's jet throttle dead band: below it the engine stays at idle (on the ground about 0.08).
-- Learned per engine and per ground / flight from X-Plane's N1 (linear in throttle past the band):
-- a settled point (throttle, N1) and the settled idle N1 give the band edge.
local db = { gi = { 0.08, 0.08, 0.08, 0.08 }, fi = { 0.04, 0.04, 0.04, 0.04 } }
local db_idle = { gi = {}, fi = {} }
local db_last = {}
function eng_deadband(i, ctx)
	local c = (ctx == "gi") and "gi" or "fi"
	return db[c][i + 1]
end
function eng_deadband_learn(i, ctx)
	local c = (ctx == "gi") and "gi" or "fi"
	local thr, n1 = simDR_engn_thro_use[i] or 0, simDR_N1[i] or 0
	local L = db_last[i] or { n1 = n1, t = simDRTime, st = 0 }
	db_last[i] = L
	local dt = simDRTime - L.t
	if dt <= 0 then return end
	-- settled: X-Plane N1 steady for 1.5 s
	if math.abs(n1 - L.n1) < 0.05 * dt / 0.05 * 0.05 and math.abs(n1 - L.n1) < 0.02 then L.st = L.st + dt else L.st = 0 end
	L.n1, L.t = n1, simDRTime
	if L.st < 1.5 or simDR_engine_running[i] ~= 1 or simDR_prop_mode[i] ~= 1 then return end
	if thr <= 0.0005 then
		db_idle[c][i + 1] = n1
	elseif db_idle[c][i + 1] and thr > 0.15 and thr < 0.36 then
		-- low part of X-Plane's N1 / throttle curve: about 57% N1 per unit throttle past the band
		local n1i = db_idle[c][i + 1]
		if n1 > n1i + 3 then
			local slope = 0.81 * (100 - n1i)
			local edge = thr - (n1 - n1i) / slope
			edge = math.max(0, math.min(0.2, edge))
			db[c][i + 1] = db[c][i + 1] + (edge - db[c][i + 1]) * 0.05
		end
	end
end

local spd_target_throttle=0.5
local REV_N1_MAX = 93.0             -- (v47) maximum N1 in reverse thrust
local rev_n1_cap = { [0] = 1.0, 1.0, 1.0, 1.0 }
-- (v56) reverse thrust timing: power 1.0 s after reverse is selected, then a quick spool-up
-- (twice the forward acceleration schedule: about 4 s from idle to the 93 % N1 limit)
local REV_DELAY = 1.0
local REV_RATE = 2.0
local rev_sel_t = { [0] = nil, nil, nil, nil }

function ecc_spd()
	--print("---ECC SPD---")
	    local input=1
		local target=1
		if simDR_override_throttles == 0 then
			simDR_override_throttles = 1
		end
		local diffSpeed=2
		--print("ecc spd")
		for i = 0, 3 do

			if B747DR_display_N2[i]<60 and simDR_onGround == 1 then
				eccPid[i].kp=0.06
				eccPid[i].ki=0
				eccPid[i].kd=0
				eccPid[i].input = 100
				eccPid[i].target = 1
				--print("cull fuel")
			elseif B747DR_engineType==1 then --GE, n1 target
				eccPid[i].kp=B747DR_pideccP
				eccPid[i].ki=B747DR_pideccI
				eccPid[i].kd=B747DR_pideccD
				eccPid[i].input = B747DR_display_N1[i]
				eccPid[i].target = B747DR_throttle_resolver_angle[i]
			else --PW or RR, EPR target
				if B747DR_display_N1[i]>110.1 then
					eccPid[i].input = 50.0*B747DR_display_EPR[i]
					eccPid[i].target = 1.5 --cull fuel if exceeding N1 limit
				else
					eccPid[i].input = 50.0*B747DR_display_EPR[i]
					eccPid[i].target = 50.0*B747DR_throttle_resolver_angle[i]
				end
				eccPid[i].kp=100*B747DR_pidepr_eccP
				eccPid[i].ki=B747DR_pidepr_eccI
				eccPid[i].kd=50*B747DR_pidepr_eccD
				diffSpeed=4
			end

		end

		--local diffSpeed=30/(0.1+math.abs(input-target))
		--print(diffSpeed)
        if (simDRTime-lastCompute)>computeRate then
			for i = 0, 3 do
            	eccPid[i]:compute()
			end
			lastCompute=simDRTime

        end
		-- engine acceleration / deceleration schedules (spool.lua): the EEC demand is followed at
		-- the rate the engine can accelerate - slow off idle, rapid above ~40% power
		local kind = SP.kind_of(B747DR_engineType)
		local ctx = "gi"
		if simDR_onGround ~= 1 then ctx = (simDR_flap_ratio > 0.8) and "ai" or "fi" end
		local all_run = simDR_engine_running[0] == 1 and simDR_engine_running[1] == 1 and simDR_engine_running[2] == 1 and simDR_engine_running[3] == 1
		if all_run then
			-- X-Plane's own spool lag kept short: the schedule sets the response
			if simDR_spool_turbine ~= SP.XP_LAG then simDR_spool_turbine = SP.XP_LAG end
			if simDR_spool_jet ~= SP.XP_LAG then simDR_spool_jet = SP.XP_LAG end
		end
		if not spool_init then
			-- start from the power the engines already have (in-flight start / script reload)
			spool_init = true
			for i = 0, 3 do
				local dz = eng_deadband(i, ctx)
				spool_p[i] = math.max(0, ((simDR_engn_thro_use[i] or 0) - dz) / (1 - dz)); spool_st[i].p = spool_p[i]
			end
		end
		-- (dead band fixed: nothing in the thrust path is learned)
		for i = 0, 3 do
			if eccPid[i].output~=nil then
				local tValue=round(eccPid[i].output*100)/100
				if simDR_engine_running[i] ==1 then
					if simDR_prop_mode[i]==1 then
						rev_sel_t[i] = nil
						-- learned feedforward + trim, driven along the acceleration schedule
						local st = spool_st[i]
						if spool_p[i] < 0 then st.p = 0 end
						local isGE = (B747DR_engineType == 1)
						-- N1-rate scheduled EEC control (spool.lua control2): steady idle at the idle stop,
						-- never past X-Plane's maximum throttle
						spool_p[i] = SP.control2(st, {
							target = eccPid[i].target, input = eccPid[i].input, dt = SIM_PERIOD, time = simDRTime, kind = kind, ctx = ctx,
							n1 = tonumber(B747DR_display_N1[i]) or 0, idle = (B747DR_throttle[i] or 0) < 0.02,
							lo = isGE and 25.9 or 50, hi = isGE and 108 or 85, eng = i, toga = isGE and (tonumber(GE_N1_FULL) or 105) or nil,
							hold_zero = (B747DR_display_N2[i] < 60 and simDR_onGround == 1) })
						-- power past X-Plane's throttle dead band: the engine responds as soon as it is asked
						local dz = eng_deadband(i, ctx)
						simDR_engn_thro_use[i] = (spool_p[i] > 0.0005) and (dz + spool_p[i] * (1 - dz)) or 0
					else
						rev_sel_t[i] = rev_sel_t[i] or simDRTime
						if simDRTime - rev_sel_t[i] >= REV_DELAY and simDR_reverser_deploy_ratio[i] > 0.5 then
							-- reverse thrust spools on the same schedule (reverse lever: 0 idle reverse .. -1 max
							-- reverse; X-Plane's throttle in reverse mode is 0 .. 1 reverse power)
							-- (v47: power from 75 % deployed - was 98 % - and limited to 93 % N1 in reverse)
							local want = math.max(0, math.min(1.0, -(B747DR_throttle_reversor[i] or 0)))
							local n1r = tonumber(B747DR_display_N1[i]) or 0
							local cap = rev_n1_cap[i] or 1.0
							cap = cap + (REV_N1_MAX - 0.5 - n1r) * 0.03 * SIM_PERIOD
							if n1r > REV_N1_MAX then cap = math.min(cap, (spool_p[i] and -spool_p[i] or cap)) end
							cap = math.max(0.3, math.min(1.0, cap))
							rev_n1_cap[i] = cap
							want = math.min(want, cap)
							spool_p[i] = SP.step(math.max(0, -spool_p[i]), want, SIM_PERIOD, kind, ctx, REV_RATE) * -1
							spool_st[i].p = 0
							simDR_engn_thro_use[i]=-spool_p[i]
						else
							spool_p[i] = 0
							spool_st[i].p = 0
							simDR_engn_thro_use[i]=0
							rev_n1_cap[i] = 1.0
						end
					end
				else
					spool_p[i] = 0
					spool_st[i].p = 0
					simDR_engn_thro_use[i]=0
				end
				--print(i.." throttle target="..eccPid[i].target.. " current "..eccPid[i].input.." AT retval "..eccPid[i].output .. " t="..simDR_engn_thro_use[i].. " diffSpeed="..diffSpeed)
			end
		end
end


last_simDR_ind_airspeed_kts_pilot=0
local lastspd_throttleTime=0
local last_speed_delta=0
local lastModeChange=0
local lastMode=""
local lastThrustTargets={0,0,0,0}
function getThrustTargets()
	local thrustTargets={0,0,0,0}
	for i = 1, 4 do
		if B747DR_engineType==1 then
			thrustTargets[i] = throttle_resolver_angle_GE(i-1)
		elseif B747DR_engineType==0 then
			thrustTargets[i] = throttle_resolver_angle_PW(i-1)*50
		elseif B747DR_engineType==2 then
			thrustTargets[i] = throttle_resolver_angle_RR(i-1)*50
		else  --Assume PW engine if all else fails
			thrustTargets[i] = throttle_resolver_angle_PW(i-1)*50
		end
	end
	return thrustTargets
end
function normalise_throttle()
	thrustTargets = getThrustTargets()
	local input=0
	local target=0
	for i = 1, 4 do
		input=math.max(input,thrustTargets[i])
		target=math.max(target,lastThrustTargets[i],input)
	end
	--print("thrustTargets was "..lastThrustTargets[1].." now "..thrustTargets[1])
	last_speed_delta=input-target
	return input,target,0.001
end
function get_spd_input_target_rog()
	local modeChangeDiff=simDRTime-lastModeChange
	if modeChangeDiff< 1.0 then
		return normalise_throttle()
	end
	if B747DR_ref_thr_limit_mode==lastMode then
		lastThrustTargets = getThrustTargets()
	end
	local input=simDR_ind_airspeed_kts_pilot
	local target=simDR_autopilot_airspeed_kts
	local diff=simDRTime-lastspd_throttleTime
	if diff>0.1 then
		last_speed_delta=simDR_ind_airspeed_kts_pilot-last_simDR_ind_airspeed_kts_pilot
		lastspd_throttleTime=simDRTime
		last_simDR_ind_airspeed_kts_pilot=simDR_ind_airspeed_kts_pilot
	end
	local speed_delta=last_speed_delta
	local rog=math.max(0.0001+0.0001*math.abs(input-target),0.0001+0.005*math.abs(speed_delta))
	if(rog>0.005) then
		rog=0.005
	end
	return input,target,rog
end
function spd_throttle()
	if B747DR_ref_thr_limit_mode~=lastMode then
		lastMode=B747DR_ref_thr_limit_mode
		lastModeChange=simDRTime
	end


	-- REBUILT AFDS: in SPD the AFDS speed law gives the lever target (acceleration-based, tight
	-- speed hold); the levers follow it at the A/T servo rate
	if B747DR_ap_FMA_autothrottle_mode==3 and B747DR_afds_thr_target~=nil and B747DR_afds_thr_target>=0 and simDR_radarAlt1>=25.1 then
		spd_target_throttle=math.max(0, math.min(1, B747DR_afds_thr_target))
		local average_throttle=0
		for i = 0, 3 do
			B747DR_throttle[i]=B747_interpolate_value(B747DR_throttle[i],spd_target_throttle,0,1.00,3)
			average_throttle=average_throttle+B747DR_throttle[i]
		end
		simDR_engine_throttle_jet_all = average_throttle / 4
		return
	end
	input,target,rog = get_spd_input_target_rog()
	local min_speedDelta=0
    local max_speedDelta=0
	local speedError=math.abs(input-target)
	local speed_delta=last_speed_delta
	if  speedError > 1 then
		--if (simDR_autopilot_airspeed_kts< simDR_ind_airspeed_kts_pilot-1) then
		max_speedDelta=0.001+speedError/100
		--else
		--	max_speedDelta=0.001+speedError/25
		--end
		min_speedDelta=max_speedDelta/5
	end
	if ((target> input+0.5) and speed_delta<min_speedDelta
            or (target< input-0.5) and speed_delta<-max_speedDelta)
        then

            spd_target_throttle= (spd_target_throttle+rog)
        elseif ((target< input-0.5) and speed_delta>-min_speedDelta
            or (target> input+0.5) and speed_delta>max_speedDelta )
        then

            spd_target_throttle= (spd_target_throttle-rog)
    end
	if simDR_radarAlt1<25.1 then
		B747DR_ap_FMA_autothrottle_mode=2
		spd_target_throttle=0
	elseif spd_target_throttle<0  then
		spd_target_throttle=0
	elseif 	spd_target_throttle>1 then
		spd_target_throttle=1
	end
	--print("THRO SPD simDR_radarAlt1="..simDR_radarAlt1.." rog="..rog.." min_speedDelta="..min_speedDelta.. " max_speedDelta="..max_speedDelta.. " speed_delta="..speed_delta .." spd_target_throttle="..spd_target_throttle)
	--[[ ]]--
	local average_throttle=0
	for i = 0, 3 do
		--simDR_engn_thro[i]=B747_interpolate_value(simDR_engn_thro[i],spd_target_throttle,0,1.00,2)
		-- REBUILT AFDS: realistic A/T lever rate in SPD (about 8 s for the full lever range)
		B747DR_throttle[i]=B747_interpolate_value(B747DR_throttle[i],spd_target_throttle,0,1.00,8)
		average_throttle=average_throttle+B747DR_throttle[i]
		--simDR_throttle_ratio[i]	= B747DR_throttle[i]
	end
	simDR_engine_throttle_jet_all = average_throttle / 4
end
local previous_throttleTime=0
function ecc_throttle()
	--print("---ECC Throttle---")
	    local input=1
		local target=1
		local minSafeSpeed = math.max(B747DR_airspeed_Vmc + 10,simDR_autopilot_airspeed_kts-5)
		--if simDR_radarAlt1<25.1 then minSafeSpeed=0 end
		local previous_pitchTime=0

		time=simDRTime-previous_throttleTime
		previous_throttleTime=simDRTime
		if time>1 or time==0 then
			return
		end
		if B747DR_ap_FMA_autothrottle_mode==3 --SPD
		or (B747DR_ap_FMA_autothrottle_mode==2 and simDR_ind_airspeed_kts_pilot<math.max(B747DR_airspeed_Vmc + 10, simDR_autopilot_airspeed_kts-15)) --IDLE: only speed protection (VNAV changes to SPD itself)
		then
				spd_throttle()
				return

		else
			--some kind of thrust target
			if B747DR_engineType==1 then --GE, n1 target
				input=5*math.max(B747DR_throttle_resolver_angle[0],B747DR_throttle_resolver_angle[1],B747DR_throttle_resolver_angle[2],B747DR_throttle_resolver_angle[3])
				target=5*simDR_N1_target_bug[0]
			else --PW or RR, EPR target
				local targetBug=math.min(simDR_EPR_target_bug[0],simDR_EPR_target_bug[1],simDR_EPR_target_bug[2],simDR_EPR_target_bug[3],
				B747DR_display_EPR_max[0],B747DR_display_EPR_max[1],B747DR_display_EPR_max[2],B747DR_display_EPR_max[3])
				local inputBug=math.max(B747DR_throttle_resolver_angle[0],B747DR_throttle_resolver_angle[1],B747DR_throttle_resolver_angle[2],B747DR_throttle_resolver_angle[3])
				input=200.0*inputBug
				target=200.0*targetBug
				--B747DR_pidthrottleP=B747DR_pidthrottleLP
				--print("throttle target="..target.. " current "..input.." targetBug "..targetBug.." inputBug "..inputBug)
			end
		end
	local rog=0.0001+(math.abs(input-target))*time*0.0005
	
	--print("rog"..rog.." B747DR_throttle_resolver_angle[0] "..B747DR_throttle_resolver_angle[0].." spd_target_throttle "..spd_target_throttle.. " simDR_N1_target_bug[0] "..simDR_N1_target_bug[0])
	--[[if(rog>0.005) then
		rog=0.005
	end]]--
	if (target> input+1) and math.abs(simDR_engine_throttle_jet_all-spd_target_throttle)<0.02 then
		spd_target_throttle= (spd_target_throttle+rog)
	elseif target< input-1 and math.abs(simDR_engine_throttle_jet_all-spd_target_throttle)<0.02 then
		spd_target_throttle= (spd_target_throttle-rog)
	end
	-- REBUILT AFDS: A/T THR mode lever target (FLCH / go-around), capped by the thrust reference
	if B747DR_ap_FMA_autothrottle_mode==4 and B747DR_afds_thr_target~=nil and B747DR_afds_thr_target>=0 then
		if B747DR_afds_thr_target < spd_target_throttle then spd_target_throttle=B747DR_afds_thr_target end
	end
	if spd_target_throttle<0 or B747DR_ap_FMA_autothrottle_mode==2 then
		spd_target_throttle=0
	elseif 	spd_target_throttle>1 then
		spd_target_throttle=1
	end

	local refreshThro=0
	if B747DR_ap_FMA_autothrottle_mode>1 --AT active
		then
		local average_throttle=0
		-- REBUILT AFDS: realistic A/T lever rates (seconds for the full lever range)
		local animTime=5                                  -- THR / THR REF advance
		if B747DR_ap_FMA_autothrottle_mode==2 then
			animTime=7                                    -- IDLE: smooth retard in descent
			if simDR_radarAlt1 < 60 then animTime=3 end   -- flare retard
		end
		if simDR_onGround == 1 and simDR_ias_pilot<30 then
			animTime=5
		end	
		for i = 0, 3 do
			--simDR_engn_thro[i]=B747_interpolate_value(simDR_engn_thro[i],spd_target_throttle,0,1.00,2)
			B747DR_throttle[i]=B747_interpolate_value(B747DR_throttle[i],spd_target_throttle,0,1.00,animTime)
			average_throttle=average_throttle+B747DR_throttle[i]
			--simDR_throttle_ratio[i]	= B747DR_throttle[i]
		end
		simDR_engine_throttle_jet_all = average_throttle / 4
	else
		spd_target_throttle=simDR_engine_throttle_jet_all	
	--[[else
		--shouldn't get here!, done in throttle_management()
		print("shouldn't get here!, done in throttle_management()")
		for i = 0, 3 do
			--refreshThro=simDR_engn_thro[i]
			B747DR_throttle[i]=B747_interpolate_value(B747DR_throttle[i],simDR_engn_thro[i],0,1.00,1)
		end]]--
	end

end
local previous_altitude = 0
local lastUserThrottleTargets={}
-- hardware throttle lock option (FMC MISC page; Output/preferences/B744_options.txt, default on)
thr_hw_lock = true
local hw_pickup = {}   -- (local: XTLua wraps global bare tables, and reading a missing key of one overflows the stack)
local thr_lock_t = -100
function thr_lock_poll()
	if simDRTime - thr_lock_t < 3 and simDRTime >= thr_lock_t then return end
	thr_lock_t = simDRTime
	local ok, f = pcall(io.open, "Output/preferences/B744_options.txt", "r")
	if ok and f then
		local s = f:read("*a") or ""; f:close()
		thr_hw_lock = not s:match("thr_hw_lock%s+0")
	else
		thr_hw_lock = true
	end
end
lastUserThrottleTargets[0]=-2
lastUserThrottleTargets[1]=-2
lastUserThrottleTargets[2]=-2
lastUserThrottleTargets[3]=-2
function throttle_management()

	--Get FMC data for CRZ ALT
	--[[if string.len(B747DR_FMSdata) > 2 then
		fms_data["data"] = json.decode(B747DR_FMSdata)
	else
		return
		--fms_data["data"].crzalt = B747DR_altitude_dial
	end]]--

	--Disconnect A/T if any of the EEC buttons move from NORMAL to ALTERNATE
	if (B747DR_button_switch_position[7] == 0 or B747DR_button_switch_position[8] == 0 or B747DR_button_switch_position[9] == 0 or B747DR_button_switch_position[10] == 0) and EEC_status == 0 then
		B747DR_autothrottle_fail = 1
		EEC_status = 1
		B747DR_autothrottle_active = 0
	elseif (B747DR_button_switch_position[7] == 1 and B747DR_button_switch_position[8] == 1 and B747DR_button_switch_position[9] == 1 and B747DR_button_switch_position[10] == 1) then
		EEC_status = 0
	end



	ecc_mode_set()

	--After landing and reversers stowed reset mode to TO
	--print("-------< ATTEMPTING TO CLEAN-UP GA MODE >-----")
	--print("On Ground = ", simDR_onGround)
	--print("Speed = ", simDR_ias_pilot)
	--print("Reverser On = ", simDR_reverser_on[1])
	--print("Reverser Deployed = ", simDR_reverser_deploy_ratio[1])
	if simDR_onGround == 1 and simDR_ias_pilot<30 -- B747DR_ref_thr_limit_mode == "GA"
		and math.max(simDR_reverser_on[0], simDR_reverser_on[1], simDR_reverser_on[2], simDR_reverser_on[3]) == 0
		and math.max(simDR_reverser_deploy_ratio[0], simDR_reverser_deploy_ratio[1], simDR_reverser_deploy_ratio[2], simDR_reverser_deploy_ratio[3]) <= 0.1 then
			--B747DR_ref_thr_limit_mode = "TO"
			B747DR_ap_flightPhase=0
	end
	--print("Flight Phase = ", B747DR_ap_flightPhase)

	--Determine FMA Mode
	--THR REF Mode
	if B747DR_ap_autothrottle_armed == 1 and B747DR_ap_FMA_autothrottle_mode == 5
	and B747DR_ap_flightPhase<2 and EEC_status == 0 then
		--Take control of the throttles from the user and manage via Thrust Ref targets
		--hold_mode = 0
		--ecc_spd()

		--Thrust ref target line should stay GREEN when in TOGA mode
		if string.match(B747DR_ref_thr_limit_mode, "TO") then
			B747DR_ref_line_magenta = 0
		else
			B747DR_ref_line_magenta = 1
		end

		if B747DR_log_level >= 1 then
			print("THR REF MODE")
			print("Override Throttles = ", simDR_override_throttles)
		end

	-- HOLD Mode
	elseif (B747DR_ap_autothrottle_armed == 1  or simDR_override_throttles == 1 ) and B747DR_ap_FMA_autothrottle_mode == 1 and EEC_status == 0 then
		--Give throttle control back to the user
		--simDR_override_throttles = 0

		B747DR_ref_line_magenta = 0
		--hold_mode = 1
		--ecc_spd()
		if B747DR_log_level >= 1 then
			print("HOLD MODE")
			print("Override REMOVED = ", simDR_override_throttles)
		end

	--SPEED Mode
	elseif B747DR_ap_autothrottle_armed == 0 and B747DR_ap_FMA_autothrottle_mode == 3 and EEC_status == 0 then
		--Give throttle control back to the user
		--hold_mode = 0
		--simDR_override_throttles = 0
		if B747DR_autothrottle_active == 1 then
			B747DR_autothrottle_active = 0

		end
		B747DR_ref_line_magenta = 0
		--speed_mode = 1
		--ecc_spd()
		if B747DR_log_level >= 1 then
			print("SPEED MODE")
			print("Override Throttles = ", simDR_override_throttles)
		end
	end
	if B747DR_autothrottle_fail == 1 then
		--Autothrottle has been disabled for some reason
		B747DR_autothrottle_active = 0
	end
	ecc_spd() --also handles reverse throttle
	-- hardware throttle lock (FMC MISC page option, default on): while any autothrottle mode is active,
	-- HOLD included, the hardware axis does not move the thrust levers (an idle hardware lever after
	-- takeoff no longer pulls the levers back when the A/T goes to HOLD)
	thr_lock_poll()
	local at_any = B747DR_ap_autothrottle_armed == 1 and B747DR_ap_FMA_autothrottle_mode >= 1
	if at_any and thr_hw_lock then
		for i = 0, 3 do hw_pickup[i] = { hw = getUserThrottleTarget(i), wait = true } end
	end
	if B747DR_ap_autothrottle_armed == 1 and B747DR_ap_FMA_autothrottle_mode > 1 then --not none or HOLD
		--new SPD
		ecc_throttle()
		for i = 0, 3 do
			lastUserThrottleTargets[i]=getUserThrottleTarget(i)
		end
	else
		for i = 0, 3 do
			if at_any and thr_hw_lock then
				-- (A/T HOLD: the levers stay where the autothrottle left them)
			elseif simDR_reverser_deploy_ratio[i]<0.01 then --reversers are not deployed
					manipulate_throttle_axis(i)
			else
			    B747DR_throttle[i]=B747_interpolate_value(B747DR_throttle[i],0,0,1.00,1)
			end
		end
		B747DR_ref_line_magenta = 0
	end

end



function getUserThrottleTarget(engine_id)
	if simDR_has_hardware_throttle == 1 then
		return simDR_hardware_throttle_ratio[engine_id]
	else
		return simDR_throttle_ratio[engine_id]
	end
end

function manipulate_throttle_axis(engine_id)
	local throttleTarget=getUserThrottleTarget(engine_id)
	-- after the autothrottle disconnects (hardware lock on): the hardware takes the lever over once it is
	-- moved (more than 4 %) or brought to the lever position - no jump to wherever the hardware rests
	local pk = hw_pickup[engine_id]
	if pk and pk.wait then
		if math.abs(throttleTarget - pk.hw) > 0.04 or math.abs(throttleTarget - (B747DR_throttle[engine_id] or 0)) < 0.03 then
			pk.wait = false
			lastUserThrottleTargets[engine_id] = -2
		else
			return
		end
	end
	if math.abs(lastUserThrottleTargets[engine_id]-throttleTarget)<0.01 then
		return --no user movement since AT deactivated, just skip.
	end
	lastUserThrottleTargets[engine_id]=-2
	B747DR_throttle[engine_id]=B747_interpolate_value(B747DR_throttle[engine_id],throttleTarget,0,1.00,1)
end
dofile("B747.42.xt.EEC.GE.lua")
dofile("B747.42.xt.EEC.PW.lua")
dofile("B747.42.xt.EEC.RR.lua")

function set_engines()
	--Engine Thrust Parameters based on selected engine
	if B747DR_engineType==1 then
		GE(simDR_altitude)
	elseif B747DR_engineType==0 then
		PW(simDR_altitude)
	elseif B747DR_engineType==2 then
		RR(simDR_altitude)
	else  --Assume PW engine if all else fails
		PW(simDR_altitude)
	end
end


--[[
*************************************************************************************
** 				              XP CALLBACKS              		    	           **
*************************************************************************************
]]
function aircraft_load()
	clear_thrust_targets()  --Set all thrust target bugs to 0
end

local setSimConfig=false
function hasSimConfig()
	if B747DR_newsimconfig_data==1 then
		if string.len(B747DR_simconfig_data) > 1 then
			simConfigData["data"] = json.decode(B747DR_simconfig_data)
			setSimConfig=true
		else
			return false
		end
	end
	return setSimConfig
end
function flight_start()
	B747DR_pidthrottleP = 1.00
	B747DR_pidthrottleI = 0.01
	B747DR_pidthrottleD = 0.01

	B747DR_pideccI = 0.01
	B747DR_pideccD = 0.001
	B747DR_pideccP = 0.06

	B747DR_pidepr_eccI = 0.01
	B747DR_pidepr_eccD = 0.01
	B747DR_pidepr_eccP = 0.001
end
-- ENGINE RECORDER: Output/B744_engines.csv (5 per second, overwritten each session) - per engine:
-- thrust lever, scheduled power (X-Plane throttle), thrust, EICAS N1/N2/EGT/EPR, X-Plane N1/N2/EGT, fuel flow
simDR_rec_ff = find_dataref("sim/cockpit2/engine/indicators/fuel_flow_kg_sec")
local erec_f, erec_t, erec_n = nil, -1, 0
local function engine_record()
	if simDRTime - erec_t < 0.2 then return end
	erec_t = simDRTime
	if erec_f == nil then
		erec_f = io.open("Output/B744_engines.csv", "w")
		if not erec_f then erec_f = false return end
		local h = { "t", "gnd", "alt", "ias", "mode", "type" }
		for e = 1, 4 do
			for _, k in ipairs({ "lever", "p", "thrust_lbf", "N1", "xpN1", "N2", "xpN2", "EGT", "xpEGT", "EPR", "FF_kgh" }) do h[#h + 1] = k .. e end
		end
		erec_f:write(table.concat(h, ",") .. "\n")
	end
	if not erec_f then return end
	local r = { string.format("%.2f", simDRTime), tostring(simDR_onGround), string.format("%.0f", simDR_altitude),
		string.format("%.1f", simDR_ind_airspeed_kts_pilot or 0), tostring(B747DR_ref_thr_limit_mode), tostring(B747DR_engineType) }
	for i = 0, 3 do
		r[#r + 1] = string.format("%.3f", B747DR_throttle[i] or 0)
		r[#r + 1] = string.format("%.3f", simDR_engn_thro_use[i] or 0)
		r[#r + 1] = string.format("%.0f", (simDR_thrust_n[i] or 0) / 4.44822)
		r[#r + 1] = string.format("%.1f", tonumber(B747DR_display_N1[i]) or 0)
		r[#r + 1] = string.format("%.1f", simDR_N1[i] or 0)
		r[#r + 1] = string.format("%.1f", tonumber(B747DR_display_N2[i]) or 0)
		r[#r + 1] = string.format("%.1f", simDR_N2[i] or 0)
		local egt_d = (B747DR_engineType == 1 and B747DR_display_GE_EGT) and B747DR_display_GE_EGT[i] or B747DR_display_EGT[i]
		r[#r + 1] = string.format("%.0f", tonumber(egt_d) or 0)
		r[#r + 1] = string.format("%.0f", simDR_engn_EGT_c[i] or 0)
		r[#r + 1] = string.format("%.3f", tonumber(B747DR_display_EPR[i]) or 0)
		local okf, ff = pcall(function() return simDR_rec_ff[i] end)
		r[#r + 1] = string.format("%.0f", ((okf and tonumber(ff)) or 0) * 3600)
	end
	erec_f:write(table.concat(r, ",") .. "\n")
	erec_n = erec_n + 1
	if erec_n % 25 == 0 then erec_f:flush() end
end

function after_physics()
	collectgarbage("collect")
	if hasSimConfig()==false then return end
	if debug_ecc>0 then return end
	local okr, er = pcall(engine_record) if not okr and not erec_err then erec_err = true print("B744 engine recorder: " .. tostring(er)) end


    atmosphere(simDR_altitude, 0)
    flight_coefficients(simDR_acf_weight_total_kg, simDR_tas_pilot)

	--fmsModules["data"] = json.decode(B747DR_FMSdata)
	set_engines()

	--[[if string.len(B747DR_simconfig_data) > 1 then
		set_engines()
	end]]
end
