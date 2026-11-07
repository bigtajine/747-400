--[[
*****************************************************************************************
* B747-400 AUTOPILOT / FLIGHT DIRECTOR / AUTOTHROTTLE  -  REBUILT 2026
*
*  Mode logic: afds/afds.lua      (747-400 FCOM behaviour, FMA, arming, captures, autoland)
*  Guidance  : afds/laws.lua      (pitch/roll attitude commands flown by the Sparky servos)
*  LNAV/VNAV : afds/glue.lua + afds/fmc/*.lua (path, LNAV, VNAV profile and guidance)
*  This file : X-Plane IO only (datarefs, commands, displays).
*
*  Until the new FMC is installed the route comes from the X-Plane FMS (xtlua/fms).
*
*  Based on the Sparky 744 by Mark Parker (mSparks), Jim Gregory/Laminar Research, Marauder28
*  and contributors.  Licence CC BY-NC 4.0.
*****************************************************************************************
--]]
NUM_AUTOPILOT_BUTTONS = 18
function null_command(phase, duration) end
function deferred_command(name, desc, realFunc) return wrap_command(name, realFunc, null_command) end
function deferred_dataref(name, nilType, callFunction) return find_dataref(name) end
function replace_char(pos, str, r) return str:sub(1, pos - 1) .. r .. str:sub(pos + 1) end

dofile("json/json.lua")
dofile("afds/fmc/geo.lua")
dofile("afds/fmc/route.lua")
dofile("afds/fmc/path.lua")
dofile("afds/fmc/perf.lua")
dofile("afds/fmc/vnav.lua")
dofile("afds/fmc/lnav.lua")
dofile("afds/afds.lua")
dofile("afds/laws.lua")
dofile("afds/glue.lua")

local AF, LW, GL, RT, GEO, PF = B744AFDS, B744LAWS, B744GLUE, B744ROUTE, B744GEO, B744PERF

--*************************************************************************************--
--**                                 DATAREFS                                         **--
--*************************************************************************************--
simDRTime                         = find_dataref("sim/time/total_running_time_sec")
simDR_version                     = find_dataref("sim/version/xplane_internal_version")
simDR_startup_running             = find_dataref("sim/operation/prefs/startup_running")
simDR_onGround                    = find_dataref("sim/flightmodel/failures/onground_any")
simDR_all_wheels_on_ground        = find_dataref("sim/flightmodel/failures/onground_all")
simDR_latitude                    = find_dataref("sim/flightmodel/position/latitude")
simDR_longitude                   = find_dataref("sim/flightmodel/position/longitude")
simDR_true_heading                = find_dataref("sim/flightmodel/position/psi")
simDR_variation                   = find_dataref("sim/flightmodel/position/magnetic_variation")
simDR_groundspeed                 = find_dataref("sim/flightmodel/position/groundspeed")
simDR_TAS_mps                     = find_dataref("sim/flightmodel/position/true_airspeed")
simDR_vpath                       = find_dataref("sim/flightmodel/position/vpath")
simDR_hpath                       = find_dataref("sim/flightmodel/position/hpath")
simDR_gwt                         = find_dataref("sim/flightmodel/weight/m_total")
simDR_fuel_total                  = find_dataref("sim/flightmodel/weight/m_fuel_total")
simDR_airspeed_mach               = find_dataref("sim/flightmodel/misc/machno")
simDR_ind_airspeed_kts_pilot      = find_dataref("laminar/B747/gauges/indicators/airspeed_kts_pilot")
simDR_pressureAlt1                = find_dataref("sim/cockpit2/gauges/indicators/altitude_ft_pilot")
simDR_radarAlt1                   = find_dataref("sim/cockpit2/gauges/indicators/radio_altimeter_height_ft_pilot")
simDR_vvi_fpm_pilot               = find_dataref("sim/cockpit2/gauges/indicators/vvi_fpm_pilot")
simDR_vh_ind_fpm                  = find_dataref("sim/flightmodel/position/vh_ind_fpm")
simDR_AHARS_roll_deg_pilot        = find_dataref("sim/cockpit2/gauges/indicators/roll_AHARS_deg_pilot")
simDR_AHARS_pitch_deg_pilot       = find_dataref("sim/cockpit2/gauges/indicators/pitch_AHARS_deg_pilot")
simDR_AHARS_heading_deg_pilot     = find_dataref("sim/cockpit2/gauges/indicators/heading_AHARS_deg_mag_pilot")
simDR_ground_track_mag            = find_dataref("sim/cockpit2/gauges/indicators/ground_track_mag_pilot")
simDR_capt_roll                   = find_dataref("sim/cockpit2/gauges/indicators/roll_AHARS_deg_pilot")
simDR_fo_roll                     = find_dataref("sim/cockpit2/gauges/indicators/roll_AHARS_deg_copilot")
simDR_wind_speed_kts              = find_dataref("sim/cockpit2/gauges/indicators/wind_speed_kts")
simDR_wind_dir                    = find_dataref("sim/cockpit2/gauges/indicators/wind_heading_deg_mag")
simDR_flap_ratio_control          = find_dataref("sim/cockpit2/controls/flap_ratio")
simDR_yoke_pitch                  = find_dataref("sim/cockpit2/controls/yoke_pitch_ratio")
simDR_yoke_roll                   = find_dataref("sim/cockpit2/controls/yoke_roll_ratio")
simDR_speedbrake_ratio            = find_dataref("sim/cockpit2/controls/speedbrake_ratio")
simDR_local_vx                    = find_dataref("sim/flightmodel/position/local_vx")
simDR_local_vz                    = find_dataref("sim/flightmodel/position/local_vz")
simDR_gear_handle                 = find_dataref("sim/cockpit2/controls/gear_handle_down")
B747DR_sim_pitch_ratio            = find_dataref("laminar/B747/flt_ctrls/sim_pitch_ratio")
B747DR_sim_roll_ratio             = find_dataref("laminar/B747/flt_ctrls/sim_roll_ratio")
B747DR_yoke_anim_pitch            = find_dataref("laminar/B747/flt_ctrls/yoke_anim_pitch")
B747DR_yoke_anim_roll             = find_dataref("laminar/B747/flt_ctrls/yoke_anim_roll")
simDR_reverser_deploy             = find_dataref("sim/flightmodel2/engines/thrust_reverser_deploy_ratio")
simDR_rudder                      = find_dataref("sim/flightmodel2/controls/rudder_trim")

-- sim autopilot (only used as display/target carriers now)
simDR_autopilot_airspeed_is_mach  = find_dataref("sim/cockpit2/autopilot/airspeed_is_mach")
simDR_autopilot_airspeed_kts      = find_dataref("sim/cockpit2/autopilot/airspeed_dial_kts")
simDR_autopilot_airspeed_kts_mach = find_dataref("sim/cockpit2/autopilot/airspeed_dial_kts_mach")
simDR_autopilot_vs_fpm            = find_dataref("sim/cockpit2/autopilot/vvi_dial_fpm")
simDR_autopilot_heading_deg       = find_dataref("sim/cockpit2/autopilot/heading_dial_deg_mag_pilot")
simDR_autopilot_heading_deg_fo    = find_dataref("sim/cockpit2/autopilot/heading_dial_deg_mag_copilot")
simDR_disabled_autopilot_altitude_ft = find_dataref("sim/cockpit2/autopilot/altitude_dial_ft")
simDR_autopilot_bank_limit        = find_dataref("sim/cockpit2/autopilot/bank_angle_mode")
simDR_ap_target_roll              = find_dataref("sim/cockpit2/autopilot/flight_director_roll_deg")
simDR_override_fms_progress       = find_dataref("sim/operation/override/override_fms_advance")
simCMD_autopilot_servos_off       = find_command("sim/autopilot/servos_off_any")
simCMD_autopilot_fdir_off         = find_command("sim/autopilot/servos_fdir_off")

-- nav radios
simDR_nav1Freq                    = find_dataref("sim/cockpit/radios/nav1_freq_hz")
simDR_nav2Freq                    = find_dataref("sim/cockpit/radios/nav2_freq_hz")
simDR_radio_nav_obs_deg           = find_dataref("sim/cockpit2/radios/actuators/nav_obs_deg_mag_pilot")
simDR_nav1_hdef                   = find_dataref("sim/cockpit2/radios/indicators/nav1_hdef_dots_pilot")
simDR_nav1_vdef                   = find_dataref("sim/cockpit2/radios/indicators/nav1_vdef_dots_pilot")
simDR_nav1_horizontal             = find_dataref("sim/cockpit2/radios/indicators/nav1_display_horizontal")
simDR_nav1_vertical               = find_dataref("sim/cockpit2/radios/indicators/nav1_display_vertical")
simDR_nav1_gs_flag                = find_dataref("sim/cockpit2/radios/indicators/nav1_flag_glideslope")
simDR_nav1_type                   = find_dataref("sim/cockpit2/radios/indicators/nav1_type")
simDR_nav1_dme                    = find_dataref("sim/cockpit2/radios/indicators/nav1_dme_distance_nm")
simDR_nav1_has_dme                = find_dataref("sim/cockpit2/radios/indicators/nav1_has_dme")
simDR_nav1_course                 = find_dataref("sim/cockpit2/radios/actuators/nav1_course_deg_mag_pilot")
simDR_nav1_bearing                = find_dataref("sim/cockpit2/radios/indicators/nav1_bearing_deg_mag")
simDR_hsi_source_pilot            = find_dataref("sim/cockpit2/radios/actuators/HSI_source_select_pilot")
simDR_hsi_source_copilot          = find_dataref("sim/cockpit2/radios/actuators/HSI_source_select_copilot")
simDR_nav2_hdef                   = find_dataref("sim/cockpit2/radios/indicators/nav2_hdef_dots_pilot")
simDR_nav2_vdef                   = find_dataref("sim/cockpit2/radios/indicators/nav2_vdef_dots_pilot")
simDR_nav2_horizontal             = find_dataref("sim/cockpit2/radios/indicators/nav2_display_horizontal")
simDR_nav2_vertical               = find_dataref("sim/cockpit2/radios/indicators/nav2_display_vertical")
simDR_variation                   = find_dataref("sim/flightmodel/position/magnetic_variation")
navAidsJSON                       = find_dataref("xtlua/navaids")
fmsJSON                           = find_dataref("xtlua/fms")
B747DR_fms_setCurrent             = find_dataref("xtlua/currentFMSID")
B747DR_legs_ext                   = find_dataref("laminar/B747/fms/legs_ext")
B747DR_lnav_active                = find_dataref("laminar/B747/fms/lnav_active")
B747DR_nd_route_geom              = find_dataref("laminar/B747/nd/route_geom")
B747DR_legs_pred                  = find_dataref("laminar/B747/fms/legs_pred")
B747DR_des_winds                  = find_dataref("laminar/B747/fms/des_winds")
targetILS                         = find_dataref("laminar/B747/radio/ilsData")
B747DR_radioModes                 = find_dataref("laminar/B747/radio/tuningmodes")

-- aircraft systems
B747DR_airspeed_V2                = find_dataref("laminar/B747/airspeed/V2")
B747DR_airspeed_Vref              = find_dataref("laminar/B747/airspeed/Vref")
B747DR_airspeed_Vmc               = find_dataref("laminar/B747/airspeed/Vmc")
B747DR_airspeed_Vmo               = find_dataref("laminar/B747/airspeed/Vmo")
B747DR_airspeed_Vf0               = find_dataref("laminar/B747/airspeed/Vf0")
B747DR_airspeed_Vf1               = find_dataref("laminar/B747/airspeed/Vf1")
B747DR_airspeed_Vf5               = find_dataref("laminar/B747/airspeed/Vf5")
B747DR_airspeed_Vf10              = find_dataref("laminar/B747/airspeed/Vf10")
B747DR_airspeed_Vf20              = find_dataref("laminar/B747/airspeed/Vf20")
B747DR_airspeed_Vf30              = find_dataref("laminar/B747/airspeed/Vf30")
B747DR_throttle                   = find_dataref("laminar/B747/engine/throttle")
B747DR_engine_TOGA_mode           = find_dataref("laminar/B747/engines/TOGA_mode")
B747DR_autothrottle_fail          = find_dataref("laminar/B747/engines/autothrottle_fail")
B747DR_autothrottle_active        = find_dataref("laminar/B747/engines/autothrottle_active")
B747DR_toggle_switch_position     = find_dataref("laminar/B747/toggle_switch/position")
B747DR_CAS_warning_status         = find_dataref("laminar/B747/CAS/warning_status")
B747DR_CAS_caution_status         = find_dataref("laminar/B747/CAS/caution_status")
B747DR_CAS_advisory_status        = find_dataref("laminar/B747/CAS/advisory_status")
B747DR_fmc_notifications          = find_dataref("laminar/B747/fms/notification")
B747DR_FMSdata                    = find_dataref("laminar/B747/fms/data")
B747DR_speedbrake_lever           = find_dataref("laminar/B747/flt_ctrls/speedbrake_lever")
B747DR_hyd_sys_pressure_1         = find_dataref("laminar/B747/hydraulics/pressure_1")
B747DR_hyd_sys_pressure_2         = find_dataref("laminar/B747/hydraulics/pressure_2")
B747DR_hyd_sys_pressure_3         = find_dataref("laminar/B747/hydraulics/pressure_3")
B747DR_simconfig_data             = find_dataref("laminar/B747/simconfig")
B747DR_newsimconfig_data          = find_dataref("laminar/B747/newsimconfig")
B747DR_ref_thr_limit_mode         = find_dataref("laminar/B747/engines/ref_thr_limit_mode")

-- autopilot (displays / interface)
simDR_autopilot_servos_on         = find_dataref("laminar/B747/autopilot/servos_on")
B747DR_ap_button_switch_position  = find_dataref("laminar/B747/autopilot/button_switch/position")
B747DR_ap_bank_limit_sel_dial_pos = find_dataref("laminar/B747/autopilot/bank_limit/sel_dial_pos")
B747DR_ap_ias_mach_window_open    = find_dataref("laminar/B747/autopilot/ias_mach/window_open")
B747DR_ap_vs_window_open          = find_dataref("laminar/B747/autopilot/vert_spd/window_open")
B747DR_ap_vs_show_thousands       = find_dataref("laminar/B747/autopilot/vert_speed/show_thousands")
B747DR_autopilot_altitude_ft      = find_dataref("laminar/B747/autopilot/heading/altitude_dial_ft")
B747DR_autopilot_altitude_ft_pfd  = find_dataref("laminar/B747/autopilot/heading/altitude_dial_ft_pfd")
B747DR_ap_heading_deg             = find_dataref("laminar/B747/autopilot/heading/degrees")
B747DR_ap_activate_target_heading_deg = find_dataref("laminar/B747/autopilot/heading/active_target")
B747DR_ap_target_heading_deg      = find_dataref("laminar/B747/autopilot/heading/target")
B747DR_ap_target_altitude         = find_dataref("laminar/B747/autopilot/altitude/target")
B747DR_ap_target_vertspeed        = find_dataref("laminar/B747/autopilot/vertspeed/target")
B747DR_ap_target_speed            = find_dataref("laminar/B747/autopilot/speed/target")
B747DR_ap_ias_dial_value          = find_dataref("laminar/B747/autopilot/ias_dial_value")
B747DR_ap_ias_bug_value           = find_dataref("laminar/B747/autopilot/ias_bug_value")
B747DR_ap_ias_bug_value_pfd       = find_dataref("laminar/B747/autopilot/ias_bug_value_pfd")
B747DR_ap_ias_mach_dial_value     = find_dataref("laminar/B747/autopilot/ias_mach_dial_value")
B747DR_ils_dots                   = find_dataref("laminar/B747/autopilot/ils_dots")
B747DR_gs_ptr_show                = find_dataref("laminar/B747/autopilot/gs_ptr_show")
B747DR_ap_vnav_system             = find_dataref("laminar/B747/autopilot/vnav_system")
B747DR_ap_vnav_target_alt         = find_dataref("laminar/B747/autopilot/vnav_target_alt")
B747DR_ap_lnav_xtk_error          = find_dataref("laminar/B747/autopilot/lnav/xtk_error")
B747DR_ap_lnav_xtk_target         = find_dataref("laminar/B747/autopilot/lnav/xtk_target")
B747DR_ap_vnav_state              = find_dataref("laminar/B747/autopilot/vnav_state")
B747DR_ap_lnav_state              = find_dataref("laminar/B747/autopilot/lnav_state")
B747DR_ap_inVNAVdescent           = find_dataref("laminar/B747/autopilot/vnav_descent")
B747DR_ap_inDescent               = find_dataref("laminar/B747/autopilot/vnav/after_tod")
B747DR_ap_flightPhase             = find_dataref("laminar/B747/autopilot/flightPhase")
B747BR_vnavProfile                = find_dataref("laminar/B747/autopilot/dist/vnavProfile")
B747BR_totalDistance              = find_dataref("laminar/B747/autopilot/dist/remaining_distance")
B747BR_distance_to_dest           = find_dataref("laminar/B747/autopilot/dist/distance_to_dest")
B747BR_eod_index                  = find_dataref("laminar/B747/autopilot/dist/eod_index")
B747BR_nextDistanceInFeet         = find_dataref("laminar/B747/autopilot/dist/next_distance_feet")
B747BR_cruiseAlt                  = find_dataref("laminar/B747/autopilot/dist/cruise_alt")
B747BR_fpe                        = find_dataref("laminar/B747/autopilot/dist/flight_path_error")
B747BR_showFMA                    = find_dataref("laminar/B747/autopilot/FMA/showFMA")
B747BR_tod                        = find_dataref("laminar/B747/autopilot/dist/top_of_descent")
B747BR_todLat                     = find_dataref("laminar/B747/autopilot/dist/top_of_descent_lat")
B747BR_todLong                    = find_dataref("laminar/B747/autopilot/dist/top_of_descent_long")
B747BR_tocLat                     = find_dataref("laminar/B747/autopilot/dist/top_of_climb_lat")
B747BR_tocLong                    = find_dataref("laminar/B747/autopilot/dist/top_of_climb_long")
B747BR_toc                        = find_dataref("laminar/B747/autopilot/dist/distance_to_toc")
B747DR_autopilot_vs_fpm           = find_dataref("laminar/B747/cockpit2/autopilot/vvi_dial_fpm")
B747DR_ap_vvi_fpm                 = find_dataref("laminar/B747/autopilot/vvi_fpm")
B747DR_ap_alt_show_thousands      = find_dataref("laminar/B747/autopilot/altitude/show_thousands")
B747DR_ap_alt_show_tenThousands   = find_dataref("laminar/B747/autopilot/altitude/show_tenThousands")
B747DR_ap_cmd_L_mode              = find_dataref("laminar/B747/autopilot/cmd_L_mode/status")
B747DR_ap_cmd_C_mode              = find_dataref("laminar/B747/autopilot/cmd_C_mode/status")
B747DR_ap_cmd_R_mode              = find_dataref("laminar/B747/autopilot/cmd_R_mode/status")
B747DR_ap_autothrottle_armed      = find_dataref("laminar/B747/autothrottle/armed")
B747DR_ap_mach_decimal_visibiilty = find_dataref("laminar/B747/autopilot/mach_dec_vis")
B747DR_ap_fpa                     = find_dataref("laminar/B747/autopilot/navadata/fpa")
B747DR_ap_vb                      = find_dataref("laminar/B747/autopilot/navadata/vb")
B747DR_ap_capt_roll               = find_dataref("laminar/B747/autopilot/capt_roll_ref")
B747DR_ap_fo_roll                 = find_dataref("laminar/B747/autopilot/fo_roll_ref")
B747DR_ap_FMA_autothrottle_mode_box_status = find_dataref("laminar/B747/autopilot/FMA/autothrottle/mode_box_status")
B747DR_ap_roll_mode_box_status    = find_dataref("laminar/B747/autopilot/FMA/roll/mode_box_status")
B747DR_ap_pitch_mode_box_status   = find_dataref("laminar/B747/autopilot/FMA/pitch/mode_box_status")
B747DR_ap_approach_mode           = find_dataref("laminar/B747/autopilot/approach_mode")
B747DR_autopilot_nav_status       = find_dataref("laminar/B747/autopilot/nav_status")
B747DR_autopilot_gs_status        = find_dataref("laminar/B747/autopilot/glideslope_status")
B747DR_autopilot_acutal_nav_status = find_dataref("laminar/B747/autopilot/actual_nav_status")
B747DR_autopilot_acutal_gs_status = find_dataref("laminar/B747/autopilot/actual_glideslope_status")
B747DR_ap_FMA_autothrottle_mode   = find_dataref("laminar/B747/autopilot/FMA/autothrottle_mode")
B747DR_ap_ATT                     = find_dataref("laminar/B747/autopilot/att_rate")
B747DR_ap_active_land             = find_dataref("laminar/B747/autopilot/active_land")
B747DR_ap_FMA_armed_roll_mode     = find_dataref("laminar/B747/autopilot/FMA/armed_roll_mode")
B747DR_ap_FMA_active_roll_mode    = find_dataref("laminar/B747/autopilot/FMA/active_roll_mode")
B747DR_ap_FMA_armed_pitch_mode    = find_dataref("laminar/B747/autopilot/FMA/armed_pitch_mode")
B747DR_ap_FMA_active_pitch_mode   = find_dataref("laminar/B747/autopilot/FMA/active_pitch_mode")
B747DR_ap_AFDS_mode_box_status_pilot   = find_dataref("laminar/B747/autopilot/AFDS/mode_box_status_pilot")
B747DR_ap_AFDS_mode_box_status_copilot = find_dataref("laminar/B747/autopilot/AFDS/mode_box_status_copilot")
B747DR_ap_AFDS_mode_box2_status_pilot  = find_dataref("laminar/B747/autopilot/AFDS/mode_box2_status_pilot")
B747DR_ap_AFDS_mode_box2_status_copilot = find_dataref("laminar/B747/autopilot/AFDS/mode_box2_status_copilot")
B747DR_ap_AFDS_status_annun_pilot = find_dataref("laminar/B747/autopilot/AFDS/status_annun_pilot")
B747DR_ap_AFDS_status_annun_copilot = find_dataref("laminar/B747/autopilot/AFDS/status_annun_copilot")
B747DR_ap_autoland                = find_dataref("laminar/B747/autopilot/autoland")
B744DR_autolandPitch              = find_dataref("laminar/B747/autopilot/autolandPitch")
B747DR_switching_servos_on        = find_dataref("laminar/B747/autopilot/ap_monitor/servos_on_time")
B747DR_fmscurrentIndex            = find_dataref("laminar/B747/autopilot/ap_monitor/fmscurrentIndex")
B747DR_fmstargetDistance          = find_dataref("laminar/B747/autopilot/ap_monitor/fmstargetDistance")
B747DR_alt_capture_window         = find_dataref("laminar/B747/autopilot/ap_monitor/alt_capture_window")
B747DR_ap_thrust_mode             = find_dataref("laminar/B747/autopilot/thrust_mode")
B747DR_ap_lastCommand             = find_dataref("laminar/B747/autopilot/lastCommand")
B747DR_lastap_dial_airspeed       = find_dataref("laminar/B747/autopilot/ap_monitor/last_airspeed")
-- custom "sim" autopilot state carriers still read by other systems
simDR_autopilot_altitude_ft       = find_dataref("laminar/B747/autopilot/altitude_dial_ft")
simDR_autopilot_hold_altitude_ft  = find_dataref("laminar/B747/autopilot/altitude_hold_ft")
simDR_autopilot_alt_hold_status   = find_dataref("laminar/B747/autopilot/altitude_hold_status")
simDR_autopilot_vs_status         = find_dataref("laminar/B747/autopilot/vvi_status")
simDR_autopilot_flch_status       = find_dataref("laminar/B747/autopilot/speed_status")
B747DR_autopilot_TOGA_status      = find_dataref("laminar/B747/autopilot/TOGA_status")
B747DR_flight_director_pitch      = find_dataref("laminar/B747/autopilot/flight_director_pitch_deg")
-- rebuilt AFDS interface
B747DR_afds_pitch_cmd             = find_dataref("laminar/B747/afds/pitch_cmd")
B747DR_afds_roll_cmd              = find_dataref("laminar/B747/afds/roll_cmd")
B747DR_afds_cmd_valid             = find_dataref("laminar/B747/afds/cmd_valid")
B747DR_afds_thr_target            = find_dataref("laminar/B747/afds/thr_target")
B747DR_afds_rudder_cmd            = find_dataref("laminar/B747/afds/rudder_cmd")
B747DR_afds_rudder_valid          = find_dataref("laminar/B747/afds/rudder_valid")
B747DR_afds_toga_count            = find_dataref("laminar/B747/afds/toga_count")
B747DR_afds_mcp_lights            = find_dataref("laminar/B747/afds/mcp_lights")
B747DR_afds_debug                 = find_dataref("laminar/B747/afds/debug")
B747DR_afds_lnav_roll             = find_dataref("laminar/B747/afds/lnav_roll")
B747DR_afds_vnav_sub              = find_dataref("laminar/B747/afds/vnav_sub")

B747CMD_fdr_log_lnav     = find_command("laminar/B747/fdr/lnav")
B747CMD_fdr_log_vnav     = find_command("laminar/B747/fdr/vnav")
B747CMD_fdr_log_apon     = find_command("laminar/B747/fdr/apon")
B747CMD_fdr_log_apdisconnect = find_command("laminar/B747/fdr/apdisconnect")
B747CMD_fdr_log_flch     = find_command("laminar/B747/fdr/flch")
B747CMD_fdr_log_vs       = find_command("laminar/B747/fdr/vs")
B747CMD_fdr_log_alt      = find_command("laminar/B747/fdr/alt")
B747CMD_fdr_log_toga     = find_command("laminar/B747/fdr/toga")
B747CMD_fdr_log_loc      = find_command("laminar/B747/fdr/loc")
B747CMD_fdr_log_app      = find_command("laminar/B747/fdr/app")
B747CMD_fdr_log_altmod   = find_command("laminar/B747/fdr/altmod")
B747CMD_fdr_log_spdmod   = find_command("laminar/B747/fdr/spdmod")
B747CMD_fdr_log_spd      = find_command("laminar/B747/fdr/spd")
B747CMD_fdr_log_headsel  = find_command("laminar/B747/fdr/headsel")
B747CMD_fdr_log_headhold = find_command("laminar/B747/fdr/headhold")

--*************************************************************************************--
--**                                 STATE                                            **--
--*************************************************************************************--
local S = AF.new()
local glue = GL.new()
local btn_target = {}
for i = 0, NUM_AUTOPILOT_BUTTONS - 1 do btn_target[i] = 0 end
local last_toga_count = nil
local last_t = 0
local ias_prev, ias_rate = 0, 0
local wind_vx_f, wind_vy_f = nil, nil
local spd_filt = {}
local hdef_prev, vdef_prev = 0, 0
local loc_crs_est, ils_freq_est = nil, nil
local loc_ok_t, loc_ok_freq = nil, nil     -- localizer signal: time it has been continuously valid
local loc_inbeam_t = nil                   -- time the needle has been continuously off full scale
local last_nav = nil
gs_ang_geo = nil
ils_on = false
local fms_sig = ""
local xp_active_set = -1
local xp_active_seen = -1
local dyn_seq_seen = nil      -- (active leg last seen: a climb / heading leg sequenced into a DF)
local ext_seen = nil          -- serial of the LEGS page data last applied
local ext_lat_sig = ""
local xp_guard_t = -1         -- ignore X-Plane active-entry changes until (after a LEGS EXEC)
local lnav_active_pub = -1
local legs_wait_t = nil
local ld_seen = nil           -- (v52) LEGS EXEC load serial last seen
local sc_hold = -1            -- (v52) no X-Plane active-entry change until (after a LEGS EXEC load)
local sc_hold_sig = ""
local thr_red_done = false
local last_out = nil
local clb_tab_loaded, clb_tab_saved = false, 0
local wind_comp_f, wind_alt_f = 0, 0
local fd_capt_prev, fd_fo_prev = 0, 0
local ap_disc_warn_t = 0

local function safe_print(s) print("B744 AFDS: " .. s) end
local function fdr(c) pcall(function() c:once() end) end
local function btn(i, phase) if phase == 0 then btn_target[i] = 1 elseif phase == 2 then btn_target[i] = 0 end end

local function fmsdata_get(key)
  local ok, d = pcall(function() return json.decode(B747DR_FMSdata) end)
  if ok and type(d) == "table" then return d[key] end
  return nil
end
local fmsdata_cache, fmsdata_t = nil, -100
local function fmsdata()
  if simDRTime - fmsdata_t > 2 then
    fmsdata_t = simDRTime
    local ok, d = pcall(function() return json.decode(B747DR_FMSdata) end)
    if ok and type(d) == "table" then fmsdata_cache = d end
  end
  return fmsdata_cache or {}
end
local function num(s)
  if s == nil then return nil end
  s = tostring(s):gsub("%s", "")
  local fl = s:match("^FL(%d+)")
  if fl then return tonumber(fl) * 100 end
  return tonumber(s)
end

--*************************************************************************************--
--**                          MCP COMMAND HANDLERS                                    **--
--*************************************************************************************--
function B747_ap_thrust_mode_CMDhandler(phase, duration)
  btn(0, phase)
  if phase == 0 then AF.event(S, "THR") end
end
function B747_ap_switch_speed_mode_CMDhandler(phase, duration)
  btn(1, phase)
  if phase == 0 then fdr(B747CMD_fdr_log_spd); AF.event(S, "SPD") end
end
function B747_ap_switch_vnavspeed_mode_CMDhandler(phase, duration)  -- IAS/MACH knob push
  btn(15, phase)
  if phase == 0 then fdr(B747CMD_fdr_log_spdmod); AF.event(S, "SPD_INTV") end
end
function B747_ap_switch_vnavalt_mode_CMDhandler(phase, duration)    -- ALT knob push
  btn(16, phase)
  if phase == 0 then fdr(B747CMD_fdr_log_altmod); AF.event(S, "ALT_INTV") end
end
function B747_ap_ias_mach_sel_button_CMDhandler(phase, duration) end
function B747_ap_switch_flch_mode_CMDhandler(phase, duration)
  btn(4, phase)
  if phase == 0 then fdr(B747CMD_fdr_log_flch); AF.event(S, "FLCH") end
end
function B747_ap_switch_vs_mode_CMDhandler(phase, duration)
  btn(6, phase)
  if phase == 0 then fdr(B747CMD_fdr_log_vs); AF.event(S, "VS") end
end
function B747_ap_alt_hold_mode_CMDhandler(phase, duration)
  btn(7, phase)
  if phase == 0 then fdr(B747CMD_fdr_log_alt); AF.event(S, "ALT_HOLD") end
end
function B747_ap_VNAV_mode_CMDhandler(phase, duration)
  btn(3, phase)
  if phase == 0 then fdr(B747CMD_fdr_log_vnav); AF.event(S, "VNAV") end
end
function B747_ap_LNAV_mode_afterCMDhandler(phase, duration)
  btn(2, phase)
  if phase == 0 then fdr(B747CMD_fdr_log_lnav); AF.event(S, "LNAV") end
end
function B747_ap_switch_hdg_sel_mode_CMDhandler(phase, duration)
  if phase == 0 then fdr(B747CMD_fdr_log_headsel); AF.event(S, "HDG_SEL") end
end
function B747_ap_heading_hold_mode_beforeCMDhandler(phase, duration) end
function B747_ap_heading_hold_mode_afterCMDhandler(phase, duration)
  btn(5, phase)
  if phase == 0 then fdr(B747CMD_fdr_log_headhold); AF.event(S, "HDG_HOLD") end
end
function B747_ap_switch_loc_mode_CMDhandler(phase, duration)
  btn(8, phase)
  if phase == 0 then fdr(B747CMD_fdr_log_loc); AF.event(S, "LOC") end
end
function B747_ap_appr_mode_beforeCMDhandler(phase, duration)
  btn(9, phase)
  if phase == 0 then fdr(B747CMD_fdr_log_app); AF.event(S, "APP") end
end
local function cmd_press(i, anim, phase)
  btn(anim, phase)
  if phase == 0 then
    fdr(B747CMD_fdr_log_apon)
    B747DR_CAS_warning_status[0] = 0
    AF.event(S, "CMD", i)
  end
end
function B747_ap_switch_cmd_L_CMDhandler(phase, duration) cmd_press(1, 10, phase) end
function B747_ap_switch_cmd_C_CMDhandler(phase, duration) cmd_press(2, 11, phase) end
function B747_ap_switch_cmd_R_CMDhandler(phase, duration) cmd_press(3, 12, phase) end
function B747_ap_switch_disengage_bar_CMDhandler(phase, duration)
  if phase == 0 then
    fdr(B747CMD_fdr_log_apdisconnect)
    btn_target[14] = 1.0 - btn_target[14]
    if btn_target[14] == 1.0 then AF.event(S, "DISCONNECT") else AF.event(S, "WARN_RESET") end
  end
end
local function yoke_disc(phase)
  if phase == 0 then
    fdr(B747CMD_fdr_log_apdisconnect)
    if AF.n_ap(S) > 0 then AF.event(S, "DISCONNECT") else AF.event(S, "WARN_RESET") end
  end
end
function B747_ap_switch_yoke_disengage_capt_CMDhandler(phase, duration) yoke_disc(phase) end
function B747_ap_switch_yoke_disengage_fo_CMDhandler(phase, duration) yoke_disc(phase) end
local function at_disc(phase) if phase == 0 then AF.event(S, "AT_DISC") end end
function B747_ap_switch_autothrottle_disco_L_CMDhandler(phase, duration) at_disc(phase) end
function B747_ap_switch_autothrottle_disco_R_CMDhandler(phase, duration) at_disc(phase) end
function B747_ap_reset_CMDhandler(phase, duration)
  if phase == 0 then
    -- used by other systems (e.g. engine shutdown): drop everything
    for k = 1, 3 do S.ap[k] = false end
    S.roll, S.pitch = "NONE", "NONE"
    S.lnav_arm, S.vnav_arm, S.loc_arm, S.gs_arm, S.app = false, false, false, false, false
  end
end
function B747_ai_ap_quick_start_CMDhandler(phase, duration)
  if phase == 0 then B747_set_ap_all_modes() end
end

-- knobs ----------------------------------------------------------------------------------------
local function ias_step(d)
  if B747DR_ap_ias_mach_window_open == 1 then
    if simDR_autopilot_airspeed_is_mach == 0 then
      B747DR_ap_ias_dial_value = math.floor(math.max(100, math.min(399, B747DR_ap_ias_dial_value + d)))
    else
      B747DR_ap_ias_dial_value = math.floor(math.max(40, math.min(95, B747DR_ap_ias_dial_value + d)))
    end
  end
end
function B747_ap_airspeed_up_CMDhandler(phase, duration) if phase == 0 or (phase == 1 and duration > 0.5) then ias_step(1) end end
function B747_ap_airspeed_down_CMDhandler(phase, duration) if phase == 0 or (phase == 1 and duration > 0.5) then ias_step(-1) end end
function B747_ap_knots_mach_toggle_CMDhandler(phase, duration)
  if phase == 0 then
    btn_target[13] = 1
    if B747DR_ap_ias_mach_window_open == 1 then
      local h = simDR_pressureAlt1
      if simDR_autopilot_airspeed_is_mach == 0 and simDR_airspeed_mach > 0.4 then
        local m = PF.cas_to_mach(B747DR_ap_ias_dial_value, h)
        simDR_autopilot_airspeed_is_mach = 1
        B747DR_ap_ias_dial_value = math.floor(m * 100 + 0.5)
      elseif simDR_autopilot_airspeed_is_mach == 1 then
        local c = PF.mach_to_cas(B747DR_ap_ias_dial_value / 100, h)
        simDR_autopilot_airspeed_is_mach = 0
        B747DR_ap_ias_dial_value = math.floor(c + 0.5)
      end
    end
  elseif phase == 2 then
    btn_target[13] = 0
  end
end
local function hdg_step(d)
  B747DR_ap_heading_deg = math.floor((B747DR_ap_heading_deg + d) % 360)
end
function B747_ap_heading_up_CMDhandler(phase, duration) if phase == 0 or (phase == 1 and duration > 0.5) then hdg_step(1) end end
function B747_ap_heading_down_CMDhandler(phase, duration) if phase == 0 or (phase == 1 and duration > 0.5) then hdg_step(-1) end end
function B747_ap_bank_limit_up_CMDhandler(phase, duration)
  if phase == 0 then B747DR_ap_bank_limit_sel_dial_pos = math.min(B747DR_ap_bank_limit_sel_dial_pos + 1, 5) end
end
function B747_ap_bank_limit_down_CMDhandler(phase, duration)
  if phase == 0 then B747DR_ap_bank_limit_sel_dial_pos = math.max(B747DR_ap_bank_limit_sel_dial_pos - 1, 0) end
end
local function vs_step(d)
  if B747DR_ap_vs_window_open == 1 then
    local v = simDR_autopilot_vs_fpm + d
    if math.abs(v) < 1000 then v = math.floor(v / 50 + 0.5) * 50 end
    v = math.max(-8000, math.min(6000, v))
    simDR_autopilot_vs_fpm = v
    B747DR_autopilot_vs_fpm = v
    S.vs_target = v
  end
end
function B747_ap_vertical_speed_up_CMDhandler(phase, duration)
  if phase == 0 or (phase == 1 and duration > 0.5) then vs_step(math.abs(simDR_autopilot_vs_fpm) < 1000 and 50 or 100) end
end
function B747_ap_vertical_speed_down_CMDhandler(phase, duration)
  if phase == 0 or (phase == 1 and duration > 0.5) then vs_step(math.abs(simDR_autopilot_vs_fpm) <= 1000 and -50 or -100) end
end
function B747_ap_altitude_up_CMDhandler(phase, duration)
  if phase == 0 or (phase == 1 and duration > 0.5) then
    B747DR_autopilot_altitude_ft = math.min(50000.0, B747DR_autopilot_altitude_ft + 100)
    B747DR_ap_lastCommand = simDRTime
  end
end
function B747_ap_altitude_down_CMDhandler(phase, duration)
  if phase == 0 or (phase == 1 and duration > 0.5) then
    B747DR_autopilot_altitude_ft = math.max(0.0, B747DR_autopilot_altitude_ft - 100)
    B747DR_ap_lastCommand = simDRTime
  end
end
function roundToIncrement(numberToRound, increment)
  local y = numberToRound / increment
  return math.floor(y + 0.5) * increment
end

--*************************************************************************************--
--**                          COMMAND REGISTRATION                                    **--
--*************************************************************************************--
B747CMD_ap_thrust_mode          = deferred_command("laminar/B747/autopilot/button_switch/thrust_mode", "Autopilot THR Mode", B747_ap_thrust_mode_CMDhandler)
B747CMD_ap_switch_speed_mode    = deferred_command("laminar/B747/autopilot/button_switch/speed_mode", "A/P Speed Mode Switch", B747_ap_switch_speed_mode_CMDhandler)
B747CMD_ap_press_airspeed       = deferred_command("laminar/B747/button_switch/press_airspeed", "Airspeed Dial Press", B747_ap_switch_vnavspeed_mode_CMDhandler)
B747CMD_ap_press_altitude       = deferred_command("laminar/B747/button_switch/press_altitude", "Altitude Dial Press", B747_ap_switch_vnavalt_mode_CMDhandler)
B747CMD_ap_ias_mach_sel_button  = deferred_command("laminar/B747/autopilot/ias_mach/sel_button", "Autopilot IAS/Mach Selector Button", B747_ap_ias_mach_sel_button_CMDhandler)
B747CMD_ap_switch_flch_mode     = deferred_command("laminar/B747/autopilot/button_switch/flch_mode", "Autopilot Fl CH Mode Switch", B747_ap_switch_flch_mode_CMDhandler)
B747CMD_ap_switch_vs_mode       = deferred_command("laminar/B747/autopilot/button_switch/vs_mode", "A/P V/S Mode Switch", B747_ap_switch_vs_mode_CMDhandler)
B747CMD_ap_switch_alt_hold_mode = deferred_command("laminar/B747/autopilot/button_switch/alt_hold_mode", "A/P Altitude Hold Mode Switch", B747_ap_alt_hold_mode_CMDhandler)
B747CMD_autopilot_VNAV_mode     = deferred_command("laminar/B747/autopilot/button_switch/VNAV", "A/P VNAV Mode Switch", B747_ap_VNAV_mode_CMDhandler)
B747CMD_autopilot_LNAV_mode     = deferred_command("laminar/B747/autopilot/button_switch/LNAV", "A/P LNAV Mode Switch", B747_ap_LNAV_mode_afterCMDhandler)
B747CMD_ap_switch_hdg_sel_mode  = deferred_command("laminar/B747/autopilot/button_switch/heading_select", "A/P Heading Select Mode Switch", B747_ap_switch_hdg_sel_mode_CMDhandler)
B747CMD_ap_switch_cmd_L         = deferred_command("laminar/B747/autopilot/button_switch/cmd_L", "A/P CMD L Switch", B747_ap_switch_cmd_L_CMDhandler)
B747CMD_ap_switch_cmd_C         = deferred_command("laminar/B747/autopilot/button_switch/cmd_C", "A/P CMD C Switch", B747_ap_switch_cmd_C_CMDhandler)
B747CMD_ap_switch_cmd_R         = deferred_command("laminar/B747/autopilot/button_switch/cmd_R", "A/P CMD R Switch", B747_ap_switch_cmd_R_CMDhandler)
B747CMD_ap_switch_disengage_bar = deferred_command("laminar/B747/autopilot/slide_switch/disengage_bar", "A/P Disenage Bar", B747_ap_switch_disengage_bar_CMDhandler)
B747CMD_ap_switch_yoke_disengage_capt = deferred_command("laminar/B747/autopilot/button_switch/yoke_disengage_capt", "A/P Capt Yoke Disengage Switch", B747_ap_switch_yoke_disengage_capt_CMDhandler)
B747CMD_ap_switch_yoke_disengage_fo = deferred_command("laminar/B747/autopilot/button_switch/yoke_disengage_fo", "A/P F/O Yoke Disengage Switch", B747_ap_switch_yoke_disengage_fo_CMDhandler)
B747CMD_ap_switch_autothrottle_disco_L = deferred_command("laminar/B747/autopilot/button_switch/autothrottle_disco_L", "A/P Autothrottle Disco Left", B747_ap_switch_autothrottle_disco_L_CMDhandler)
B747CMD_ap_switch_autothrottle_disco_R = deferred_command("laminar/B747/autopilot/button_switch/autothrottle_disco_R", "A/P Autothrottle Disco Right", B747_ap_switch_autothrottle_disco_R_CMDhandler)
B747CMD_ap_reset                = deferred_command("laminar/B747/autopilot/mode_reset", "Autopilot Mode Reset", B747_ap_reset_CMDhandler)
B747CMD_ai_ap_quick_start       = deferred_command("laminar/B747/ai/autopilot_quick_start", "Autopilot Quick Start", B747_ai_ap_quick_start_CMDhandler)
B747CMD_autopilot_appr_mode     = deferred_command("laminar/B747/autopilot/approach", "APP Switch", B747_ap_appr_mode_beforeCMDhandler)
B747CMD_ap_switch_loc_mode      = deferred_command("laminar/B747/autopilot/button_switch/loc_mode", "A/P Localizer Mode Switch", B747_ap_switch_loc_mode_CMDhandler)

simCMD_ap_knots_mach_toggle = replace_command("sim/autopilot/knots_mach_toggle", B747_ap_knots_mach_toggle_CMDhandler)
simCMD_ap_airspeed_up       = replace_command("sim/autopilot/airspeed_up", B747_ap_airspeed_up_CMDhandler)
simCMD_ap_airspeed_down     = replace_command("sim/autopilot/airspeed_down", B747_ap_airspeed_down_CMDhandler)
simCMD_ap_heading_up        = replace_command("sim/autopilot/heading_up", B747_ap_heading_up_CMDhandler)
simCMD_ap_heading_down      = replace_command("sim/autopilot/heading_down", B747_ap_heading_down_CMDhandler)
simCMD_ap_bank_limit_up     = replace_command("sim/autopilot/bank_limit_up", B747_ap_bank_limit_up_CMDhandler)
simCMD_ap_bank_limit_down   = replace_command("sim/autopilot/bank_limit_down", B747_ap_bank_limit_down_CMDhandler)
simCMD_ap_vertical_speed_up = replace_command("sim/autopilot/vertical_speed_up", B747_ap_vertical_speed_up_CMDhandler)
simCMD_ap_vertical_speed_down = replace_command("sim/autopilot/vertical_speed_down", B747_ap_vertical_speed_down_CMDhandler)
simCMD_ap_altitude_up       = replace_command("sim/autopilot/altitude_up", B747_ap_altitude_up_CMDhandler)
simCMD_ap_altitude_down     = replace_command("sim/autopilot/altitude_down", B747_ap_altitude_down_CMDhandler)
simCMD_autopilot_heading_hold_mode = wrap_command("sim/autopilot/heading_hold", B747_ap_heading_hold_mode_beforeCMDhandler, B747_ap_heading_hold_mode_afterCMDhandler)

--*************************************************************************************--
--**                    ROUTE FROM THE X-PLANE FMS (until the new FMC)                 **--
--*************************************************************************************--
local function fms_entries()
  local s = fmsJSON
  if s == nil or string.len(s) < 3 then return nil end
  local ok, t = pcall(function() return json.decode(s) end)
  if not ok or type(t) ~= "table" then return nil end
  return t
end

-- LEGS page data (discontinuities, direct-to, intercept course, pilot constraints): one entry per
-- X-Plane FMS entry; only used while it describes exactly the list X-Plane holds
local ext_str, ext_tbl = nil, nil
local function attach_ext(fms)
  local s = B747DR_legs_ext
  if s == nil or string.len(s) < 3 then return nil end
  if s ~= ext_str then
    ext_str = s
    local ok, t = pcall(function() return json.decode(s) end)
    ext_tbl = (ok and type(t) == "table" and type(t.e) == "table") and t or nil
  end
  local x = ext_tbl
  if not x or #x.e ~= #fms then return nil end
  for i, e in ipairs(x.e) do
    local f = fms[i]
    if math.abs((f[5] or 0) - (e[1] or 0)) > 2e-3 or math.abs((f[6] or 0) - (e[2] or 0)) > 2e-3 then return nil end
    -- (parallel runways / nearby fixes: the LEGS data must be for the same named waypoint)
    local xid, fid = type(e[3]) == "table" and e[3].id, f[8]
    if type(xid) == "string" and type(fid) == "string" and fid ~= "" and fid ~= "latlon" and not fid:find("^%(")
      and not xid:find("^%(") and not (e[3].pp) and (fid:gsub("%s", "")) ~= (xid:gsub("%s", "")) then return nil end
  end
  for i, e in ipairs(x.e) do fms[i][11] = (type(e[3]) == "table") and e[3] or nil end
  return x
end
-- the parts of the LEGS data that change the lateral path
local function ext_lateral_sig(fms)
  local parts = {}
  for i, e in ipairs(fms) do
    local x = e[11]
    -- (x.proc: the nav-database procedure legs - climb / heading / DME / intercept legs before the fix
    -- and the fix's own leg type - change the path too; the LEGS data usually arrives after X-Plane's
    -- new list, so without it here those legs would be missing until the next route change)
    local pq = ""
    if x and x.proc then
      local q = x.proc
      local tt = {}
      for _, pl in ipairs(q.pre or {}) do tt[#tt + 1] = tostring(pl.t) .. tostring(pl.crs or "") .. tostring(pl.alt and pl.alt.a or "") end
      -- (v52: and the legs after the fix - a removed (VECTORS) / (900) changes the path)
      local tp = {}
      for _, pl in ipairs(q.post or {}) do tp[#tp + 1] = tostring(pl.t) .. tostring(pl.crs or "") .. tostring(pl.alt and pl.alt.a or "") end
      pq = "p" .. table.concat(tt, ".") .. ">" .. tostring(q.leg and q.leg.t or "") .. (q.rwy and "r" or "") .. "<" .. table.concat(tp, ".")
    end
    if x and (x.db or x.df or x.ic or x.pp or x.rw or x.xdc or x.ca or x.dep or x.fo or x.pca or x.pdf or x.proc or x.vdf) then
      parts[#parts + 1] = string.format("%d:%s%s%s%s", i, x.db and "D" or "", x.df and string.format("F%.4f,%.4f", x.df[1], x.df[2]) or "",
        x.ic and ("C" .. tostring(x.ic)) or "", (x.pp and "P" or "") .. (x.rw and "R" or "") .. (x.xdc and "X" or "") .. ((x.ca or x.dep) and "C" or "") .. (x.fo and "O" or "") .. (x.pca and ("A" .. tostring(x.pca[2])) or "") .. (x.pdf and "Q" or "") .. (x.vdf and "V" or "") .. pq)
    end
  end
  return table.concat(parts, "|")
end

local function is_rwy_ident(id)
  if type(id) ~= "string" then return false end
  return id:match("^RWY?%d%d?[LRC]?$") ~= nil or id:match("^RW%d%d?[LRC]?$") ~= nil
end
local function ent_ident(e)
  local id = e[8]
  if (id == nil or id == "" or id == "latlon") and e[11] and e[11].id then id = e[11].id end
  return id
end
local function ent_is_rwy(e) return is_rwy_ident(ent_ident(e)) or (e[11] ~= nil and e[11].rw == true) end
-- index of the first entry LNAV may fly to: after the origin airport and departure runway entries
local function first_flyable(fms)
  local k = 2
  local o = fms[1]
  while fms[k] and k < #fms do
    local e = fms[k]
    local near = o and GEO.dist(o[5], o[6], e[5], e[6]) < 6
    -- (X-Plane gives a departure runway without a procedure as an unnamed position next to the airport)
    if e[11] and e[11].ca then break end
    local unnamed_rwy = k == 2 and o and o[2] == 1 and e[2] == 2048 and GEO.dist(o[5], o[6], e[5], e[6]) < 4
    if (ent_is_rwy(e) and near) or unnamed_rwy or (e[2] == 1 and k == 2) or (e[11] and e[11].pp) then k = k + 1 else break end
  end
  return k
end
-- procedure leg from the LEGS data (x.proc entries)
function proc_nav_of(n)
  if not n then return nil end
  return { ident = n.id, lat = n.lat, lon = n.lon, kind = n.kind, slaved_var = n.sv }
end
function proc_leg_of(q, seg)
  local l = { t = q.t, crs = q.crs, crs_is_true = q.ti, turn = q.turn, rho = q.rho, theta = q.theta,
    dist = q.dist, arc_r = q.arc_r, flyover = q.fo, seg = seg, via = "" }
  l.nav = proc_nav_of(q.nav)
  l.ctr = proc_nav_of(q.ctr)
  if q.fix then l.fix = { ident = q.fix.id, lat = q.fix.lat, lon = q.fix.lon, kind = "WPT" } end
  if q.alt and q.alt.a and (l.t == "CA" or l.t == "VA" or l.t == "FA") then l.alt = { d = "+", a = q.alt.a } end
  if l.t == "CA" or l.t == "VA" or l.t == "FA" then l.id = string.format("(%d)", (l.alt and l.alt.a) or 0)
  elseif l.t == "CI" or l.t == "VI" then l.id = "(INTC)"
  elseif l.t == "VM" or l.t == "FM" then l.id = "(VECTORS)"
  elseif l.t == "CD" or l.t == "VD" or l.t == "FD" then l.id = string.format("(%s%d)", (l.nav and l.nav.ident) or "DME", math.floor((l.dist or 0) + 0.5))
  elseif l.t == "CR" or l.t == "VR" then l.id = string.format("(%s%03d)", (l.nav and l.nav.ident) or "", math.floor((l.theta or 0) + 0.5))
  else l.id = (l.fix and l.fix.ident) or "-----" end
  return l
end
-- on the ground the departure starts with the first procedure leg after the runway: X-Plane makes
-- the first route fix active, but the legs before it that X-Plane does not hold (climb, heading,
-- DME, intercept legs - SYE4: 338 to 700 ft, heading 100 to TTE 9 DME, heading 017 to intercept)
-- are flown first
local function ground_first_active(p)
  if not (p and p.ca_idx and p.active and p.active > p.ca_idx) then return end
  for k = p.ca_idx, p.active - 1 do
    local l = p.legs[k]
    if not l or l.t == "DISCO" or not (B744PATH.DYN[l.t]) then return end
  end
  p.active = p.ca_idx
end
local function build_plan_from_xp(fms, ppos)
  local p = RT.new()
  local n = #fms
  local active = 0
  local crz = B747BR_cruiseAlt or 0
  local ff = first_flyable(fms)
  -- landing runway: the last runway entry in the second half of the list; anything after it is the
  -- missed approach (not part of the descent); the destination airport is the last airport entry
  local dest_rwy, dest_apt = nil, nil
  for i = n, 1, -1 do if fms[i][2] == 1 and i > 1 then dest_apt = i break end end
  for i = n, math.max(2, math.floor(n / 2)), -1 do
    if ent_is_rwy(fms[i]) and i >= ff then dest_rwy = i break end
  end
  local xp_map = {}
  -- arrival part of the route: waypoints nearer the destination than the origin (a short route has
  -- arrival fixes in the first half of the list - their restrictions must not cap the climb)
  local oe, de = fms[1], fms[dest_apt or n]
  local function arrival(e)
    if not oe or not de then return false end
    return GEO.dist(e[5], e[6], de[5], de[6]) < GEO.dist(e[5], e[6], oe[5], oe[6])
  end
  local function xp_disco(e)
    if e[11] and e[11].xd then return true end
    local id = type(e[8]) == "string" and string.upper(e[8]) or ""
    return (e[2] or 2048) == 0 or id:find("DISCON") ~= nil or (math.abs(e[5] or 0) < 1e-6 and math.abs(e[6] or 0) < 1e-6)
  end
  local goto_next = false
  p.ca_idx = nil
  for i = 1, n do
    local e = fms[i]
    local x = e[11]
    if x and x.xdc then
      -- X-Plane discontinuity entry closed on the LEGS page: no discontinuity, the route goes on
      -- to the next waypoint (the entry maps to that leg)
      xp_map[i] = #p.legs + 1
    elseif xp_disco(e) then
      -- X-Plane's own discontinuity entry
      if #p.legs > 0 and p.legs[#p.legs].t ~= "DISCO" then p.legs[#p.legs + 1] = RT.disco() end
      xp_map[i] = #p.legs
    else
    local ident = ent_ident(e)
    if ident == nil or ident == "latlon" or ident == "" then ident = string.format("WPT%02d", i) end
    local fix = { ident = ident, lat = e[5], lon = e[6], kind = (e[2] == 1 and "APT") or (ent_is_rwy(e) and "RWY") or "WPT" }
    local l
    -- procedure legs from the nav database (LEGS data x.proc): the legs X-Plane's plan does not hold
    -- before this fix (climb / heading / DME / intercept legs) and, below, this fix's own leg type
    local prc = x and x.proc or nil
    if prc then
      local seg0 = (dest_rwy and i > dest_rwy) and "MISSED" or (arrival(e) and "STAR" or "SID")
      if prc.rwy then
        -- departure runway: the departure starts at the runway end, on the runway heading
        local rl = nil
        for k = #p.legs, 1, -1 do local q = p.legs[k] if q.t == "IF" and q.fix and q.fix.kind == "RWY" then rl = q break end end
        if not rl then
          rl = { t = "IF", id = prc.rwy.id, fix = { ident = prc.rwy.id, lat = prc.rwy.lat, lon = prc.rwy.lon, kind = "RWY" }, seg = "RWY", via = "" }
          p.legs[#p.legs + 1] = rl
        end
        rl.rwy_hdg = prc.rwy.hdg; rl.rwy_len = prc.rwy.len
      end
      local ALTL = { CA = true, VA = true, FA = true }
      for k, q in ipairs(prc.pre or {}) do
        local nl = proc_leg_of(q, seg0)
        if k == 1 and prc.rwy and (nl.t == "CA" or nl.t == "VA") then
          nl.from_elev = prc.rwy.elev
        end
        -- the same climb leg already in the plan (X-Plane kept it as an entry, e.g. "(520)", or the
        -- older LEGS climb-leg data): one leg, with the nav-database definition
        local dup = nil
        if ALTL[nl.t] and nl.alt and nl.alt.a then
          for m = #p.legs, 1, -1 do
            local o = p.legs[m]
            if ALTL[o.t] and o.alt and o.alt.a and math.abs(o.alt.a - nl.alt.a) < 1 then dup = o break end
            if not ALTL[o.t] then break end
          end
        end
        if dup then
          for kk, vv in pairs(nl) do if kk ~= "id" then dup[kk] = vv end end
          if k == 1 and prc.rwy and (nl.t == "CA" or nl.t == "VA") then
            for m = #p.legs, 1, -1 do if p.legs[m] == dup then p.ca_idx = p.ca_idx or m break end end
          end
        else
          if k == 1 and prc.rwy and (nl.t == "CA" or nl.t == "VA") then p.ca_idx = p.ca_idx or (#p.legs + 1) end
          p.legs[#p.legs + 1] = nl
        end
      end
    end
    -- departure runway and climb on runway heading to an altitude (X-Plane's "(2400)" leg) that the
    -- X-Plane plan does not hold: from the LEGS data
    local dep = x and (x.dep or (x.ca and {})) or nil
    if dep and dep.rw then
      p.legs[#p.legs + 1] = { t = "IF", id = dep.rw[1], fix = { ident = dep.rw[1], lat = dep.rw[2], lon = dep.rw[3], kind = "RWY" }, seg = "RWY", via = "" }
    end
    local ca = x and (x.ca or (x.dep and x.dep.ca)) or nil
    if ca and ca[1] and ca[2] then
      local cal = { t = "CA", id = string.format("(%d)", ca[2]), crs = ca[1], crs_is_true = true,
        alt = { d = "+", a = ca[2] }, seg = "SID", via = "" }
      if x.ca then
        -- the X-Plane entry itself is the climb leg
        if not (dep and dep.rw) then
          p.legs[#p.legs + 1] = { t = "IF", id = "RWY", fix = { ident = "RWY", lat = e[5], lon = e[6], kind = "RWY" }, seg = "RWY", via = "" }
        end
        p.legs[#p.legs + 1] = cal
        p.ca_idx = #p.legs
        xp_map[i] = #p.legs
        if e[10] == true then active = i end
        goto_next = true
      else
        p.legs[#p.legs + 1] = cal
        p.ca_idx = #p.legs
      end
    end
    if goto_next then
      goto_next = false
    else
    if i < ff then
      -- origin airport / departure runway: the departure starts here, never flown back to
      l = { t = "IF", id = ident, fix = fix, seg = "RWY", via = "" }
      if i == 1 and e[2] == 1 then p.origin = { icao = ident, ident = ident, lat = e[5], lon = e[6], elev = e[9] or 0 } end
    else
      l = { t = "TF", id = ident, fix = fix, seg = arrival(e) and "STAR" or "ENRT", via = "DIRECT" }
      -- (a departure procedure fix: part of the SID - its restrictions are climb restrictions, never
      -- the end of the descent)
      if prc and not arrival(e) then l.seg = "SID" end
      local alt = e[9] or 0
      if x and x.cn and x.cn.a then
        -- constraint entered / kept on the LEGS page
        l.alt = { d = x.cn.d or "@", a = x.cn.a, b = x.cn.b }; l.user_cnstr = true
      elseif x and x.cnd then
        l.alt = nil
      elseif alt > 0 and e[2] ~= 1 and arrival(e) and (crz == 0 or alt < crz - 1000) then
        l.alt = { d = "+", a = alt }
      end
      if x and x.sp then l.spd = { d = "-", v = x.sp } end
      -- fly-over waypoint (nav database, e.g. a departure-end fix): the turn starts over the fix
      if x and x.fo then l.flyover = true end
      -- procedure direct-to (after a climb leg): flown as a turn from the climb track onto the fix
      if x and (x.pdf or (x.dep and x.dep.df)) then l.t = "DF"; l.proc_df = true end
      if x and x.vdf and not x.db then
        -- (v52) generated legs removed on the LEGS page before this fix (e.g. (VECTORS) replaced by
        -- GLENN): direct to it from the end of the leg before - the end of a climb / heading leg, or
        -- over the procedure fix before (D326D: over it, then the turn to GLENN, as the procedure's
        -- own legs would have done; not a fly-by turn started back near the runway)
        local pl = p.legs[#p.legs]
        if pl and pl.t ~= "DISCO" and pl.t ~= "IF" then l.t = "DF"; l.proc_df = true else l.t = "TF"; l.proc_df = nil end
      end
      if x and x.df then
        -- DIRECT TO from the LEGS page: from the position where it was executed
        l.t = "DF"; l.dt_start = { lat = x.df[1], lon = x.df[2] }; l.direct = true
        l.dt_trk = x.df[3]; l.dt_gs = x.df[4]
      end
      if x and x.ic then
        -- INTC CRS: intercept the entered course to the fix
        l.t = "CF"; l.crs = x.ic; l.dist = x.icd or 30; l.intc = true; l.dt_start = nil
      end
      if dest_rwy and i > dest_rwy and i ~= dest_apt then l.seg = "MISSED"; l.alt = nil end
      if i == dest_rwy then
        -- landing runway: end of descent at threshold height on a 3 degree final
        local th = (dest_apt and fms[dest_apt][9]) or 0
        if alt > 0 and alt < th + 200 then th = alt end
        l.alt = { d = "@", a = th + 50 }; l.fpa = -3.0; l.seg = "APP"
      end
      if i == dest_apt then
        p.dest = { icao = ident, ident = ident, lat = e[5], lon = e[6], elev = e[9] or 0 }
        l.seg = "DEST"; l.alt = nil; l.map = true
        if dest_rwy then
          l.seg = "MISSED"          -- beyond the runway threshold: not part of the LNAV route
        else
          -- no runway in the X-Plane list: end the descent path at the airport (threshold height) on a
          -- 3 degree final from the last fix when it is close in, so VNAV arrives on a normal approach path
          -- (farther out: an idle path down to the airport, never an E/D 1500 ft above the field)
          local pe = fms[i - 1]
          l.alt = { d = "@", a = (e[9] or 0) + 50 }; l.seg = "APP"
          if pe and pe[2] ~= 1 and GEO.dist(pe[5], pe[6], e[5], e[6]) < 20 then l.fpa = -3.0 end
        end
      end
    end
    if x and x.db and #p.legs > 0 and p.legs[#p.legs].t ~= "DISCO" then
      -- ROUTE DISCONTINUITY before this waypoint (LEGS page)
      p.legs[#p.legs + 1] = RT.disco()
    end
    -- this fix's own procedure leg (CF course, AF DME arc, RF radius, DF direct), not for a pilot
    -- direct-to / intercept or the landing runway
    if prc and prc.leg and l and l.t ~= "IF" and not x.df and not x.ic and not (x.vdf and not x.db) and i ~= dest_rwy then
      local q = prc.leg
      local lastpre = prc.pre and prc.pre[#prc.pre]
      if lastpre and (lastpre.t == "CI" or lastpre.t == "VI") and q.t == "CF" and q.crs then
        -- heading / course to intercept, then the course to the fix (SYE4: heading 017 to intercept
        -- 347 to SYE): the course is intercepted, not flown direct
        l.t = "CF"; l.crs = q.crs; l.crs_is_true = q.ti; l.dist = q.dist
      elseif prc.pre and #prc.pre > 0 and (q.t == "CF" or q.t == "TF") then
        -- after a climb / heading / DME leg (ends where it ends, no fly-by): turn from that track
        -- straight towards the fix (a smooth turn drawn on the ND and flown by LNAV)
        l.t = "DF"; l.proc_df = true
      elseif q.t == "CF" and q.crs then
        l.t = "CF"; l.crs = q.crs; l.crs_is_true = q.ti; l.dist = q.dist
      elseif q.t == "AF" and q.nav then
        l.t = "AF"; l.nav = proc_nav_of(q.nav); l.rho = q.rho; l.turn = q.turn
      elseif q.t == "RF" and q.ctr then
        l.t = "RF"; l.ctr = proc_nav_of(q.ctr); l.arc_r = q.arc_r; l.turn = q.turn
      elseif q.t == "DF" then
        l.t = "DF"; l.proc_df = true
      end
      if q.fo then l.flyover = true end
    end
    p.legs[#p.legs + 1] = l
    xp_map[i] = #p.legs
    if e[10] == true then active = i end
    -- procedure legs after this fix (nav database, x.proc.post): the procedure continues with legs
    -- that end at an altitude / DME distance / intercept, or in vectors - e.g. SY4: 155 deg to 500 ft,
    -- then vectors; a STAR ending in vectors to final. Vectors (VM / FM) end in a ROUTE DISCONTINUITY
    -- as on the aircraft (LNAV holds the heading / track until the pilot continues the route).
    if prc and prc.post and #prc.post > 0 and not (x and (x.df or x.ic)) then
      local segp = (dest_rwy and i >= dest_rwy) and "MISSED" or (arrival(e) and "STAR" or "SID")
      local ALT1 = { CA = true, VA = true, FA = true }
      for k, q in ipairs(prc.post) do
        local nl = proc_leg_of(q, segp)
        if k == 1 and l.t == "IF" and prc.rwy and (nl.t == "CA" or nl.t == "VA") then
          nl.from_elev = prc.rwy.elev
          l.rwy_hdg = l.rwy_hdg or prc.rwy.hdg; l.rwy_len = l.rwy_len or prc.rwy.len
          p.ca_idx = p.ca_idx or (#p.legs + 1)
        end
        -- (the same climb leg X-Plane kept as an entry, e.g. "(500)": not added twice)
        local nx = fms[i + 1]
        local nxid = nx and type(nx[8]) == "string" and nx[8] or ""
        if not (ALT1[nl.t] and nl.alt and nxid == string.format("(%d)", nl.alt.a)) then
          p.legs[#p.legs + 1] = nl
        end
      end
      local lastq = prc.post[#prc.post]
      local nx = fms[i + 1]
      if lastq and (lastq.t == "VM" or lastq.t == "FM") and nx and not xp_disco(nx) and segp ~= "MISSED" then
        p.legs[#p.legs + 1] = RT.disco()
      end
    end
    -- SID climb leg after this fix (e.g. BN4: DE19L, then climb on 196 to 600 ft): from the LEGS data
    if x and x.pca and x.pca[1] and x.pca[2] then
      p.legs[#p.legs + 1] = { t = "CA", id = string.format("(%d)", x.pca[2]), crs = x.pca[1], crs_is_true = true,
        alt = { d = "+", a = x.pca[2] }, seg = "SID", via = "" }
    end
    end
    end
  end
  if not p.dest and dest_rwy then
    local e = fms[dest_rwy]
    p.dest = { icao = e[8], ident = e[8], lat = e[5], lon = e[6], elev = math.max(0, (e[9] or 0)) }
  end
  -- end of descent: the last altitude constraint before the destination (the approach fix where the
  -- ILS is intercepted, e.g. GLENN 3000A) is flown AT its altitude, level by the fix - VNAV then holds
  -- it for the intercept. The airport fallback path is only used when there is no such fix.
  local last_c = nil
  for i = #p.legs, 1, -1 do
    local l = p.legs[i]
    -- (only on the arrival side: a route without a STAR must not end its descent at a SID restriction)
    -- (not the destination airport itself: with no arrival selected yet it is the end of the path)
    if l.alt and l.fix and (l.seg == "STAR" or l.seg == "APP") and l.fix.kind ~= "RWY" and l.fix.kind ~= "APT" and not l.fpa then last_c = i break end
  end
  if last_c then
    -- several final restrictions at the same altitude (e.g. GLENN 3000A then GOGEX 3000 on the ILS):
    -- the end of descent is the first of them, so the aircraft is level at that altitude from there
    local a0 = p.legs[last_c].alt.a
    for i = last_c - 1, 1, -1 do
      local l = p.legs[i]
      if l.t == "DISCO" or l.seg == "RWY" then break end
      if l.alt then
        if math.abs(l.alt.a - a0) <= 50 and (l.alt.d == "+" or l.alt.d == "@") then last_c = i else break end
      end
    end
    local l = p.legs[last_c]
    l.alt = { d = "@", a = l.alt.a }
    local d = p.legs[#p.legs]
    if d and d.seg == "APP" and d.fix and d.fix.kind == "APT" then
      local dd = GEO.dist(l.fix.lat, l.fix.lon, d.fix.lat, d.fix.lon)
      if dd < 40 then d.alt = nil; d.fpa = nil; d.seg = "MISSED" end
    end
  end
  p.activated = true
  p.xp_map = xp_map
  p.first_flyable = xp_map[math.min(ff, n)] or ff
  -- the climb-to-altitude departure leg is the first leg flown
  if p.ca_idx and p.ca_idx < p.first_flyable then p.first_flyable = p.ca_idx end
  p.active = math.max(xp_map[active] or 0, p.first_flyable)
  if p.ca_idx and ppos and ppos.onground and p.active <= p.ca_idx + 1 then p.active = p.ca_idx end
  if ppos and ppos.onground then ground_first_active(p) end
  return p
end

local function fms_signature(fms)
  local parts = {}
  for i, e in ipairs(fms) do parts[#parts + 1] = (e[8] or "") .. string.format("%.3f%.3f", e[5] or 0, e[6] or 0) end
  return table.concat(parts, "|")
end

local fms_alt_sig = ""
local alt_sig_t = 0
local function alt_signature(fms)
  local parts = {}
  for i, e in ipairs(fms) do parts[#parts + 1] = tostring(math.floor((e[9] or 0) + 0.5)) end
  return table.concat(parts, ",")
end
local function sync_route(ac)
  local fms = fms_entries()
  if not fms or #fms < 2 then
    if glue.plan then GL.set_plan(glue, nil) end
    fms_sig = ""
    return
  end
  local ext = attach_ext(fms)
  -- a LEGS EXEC is waiting for X-Plane to load the new list: X-Plane's entry indices are about to
  -- change, so the active entry must not be set until the new list is in (setting the X-Plane
  -- destination entry to a stale / invalid index can crash X-Plane)
  local legs_loading = (ext == nil) and ext_tbl ~= nil and ext_tbl.s ~= ext_seen
  if legs_loading then
    legs_wait_t = legs_wait_t or simDRTime
    if simDRTime - legs_wait_t > 5 then legs_loading = false end
  else
    legs_wait_t = nil
  end
  -- (v52) a LEGS EXEC reloads X-Plane's flight plan (announced in the LEGS data, sent a moment
  -- later): X-Plane's active entry is not set until the new list is in and has stayed the same for a
  -- moment - an index of the old list given while X-Plane loads crashed X-Plane
  local ld = ext_tbl and (ext_tbl.ld or 0)
  if ld ~= nil and ld ~= ld_seen then
    if ld_seen ~= nil then sc_hold = math.max(sc_hold, simDRTime + 4) end
    ld_seen = ld
  end
  local ext_new = ext ~= nil and ext.s ~= ext_seen
  local xp_raw = 0
  for i, e in ipairs(fms) do if e[10] == true then xp_raw = i end end
  local sig = fms_signature(fms)
  if sig ~= sc_hold_sig then
    if sc_hold_sig ~= "" and simDRTime < sc_hold + 6 then sc_hold = math.max(sc_hold, simDRTime + 1.5) end
    sc_hold_sig = sig
  end
  -- X-Plane / the CDU removed passed waypoints (from the front or after the origin): if every remaining
  -- entry is in the current plan in the same order and nothing at/after the active leg is missing,
  -- it is the same route: keep the plan (rebuilding it would remove the turn being flown)
  if sig ~= fms_sig and glue.plan and not ac.onground and not (ext_new and ext.a) then
    local old = glue.plan
    local function same(e, l) return l and l.fix and math.abs(e[5] - l.fix.lat) < 1e-4 and math.abs(e[6] - l.fix.lon) < 1e-4 end
    local map, j, ok = {}, 1, true
    for i = 1, #fms do
      while j <= #old.legs and not same(fms[i], old.legs[j]) do j = j + 1 end
      if j > #old.legs then ok = false break end
      map[i] = j; j = j + 1
    end
    local nfix = 0
    for _, l in ipairs(old.legs) do if l.t ~= "DISCO" then nfix = nfix + 1 end end
    if ok and #fms < nfix then
      -- every plan leg from the active one on must still be in the list
      local present = {}
      for i = 1, #fms do present[map[i]] = true end
      for k = math.max(1, old.active), #old.legs do
        if old.legs[k].fix and old.legs[k].t ~= "IF" and not present[k] then ok = false break end
      end
      if ok then
        old.xp_map = map
        fms_sig = sig
        fms_alt_sig = alt_signature(fms)
        if ext then ext_seen = ext.s; ext_lat_sig = ext_lateral_sig(fms) else ext_seen = nil; ext_lat_sig = "" end
        ext_new = false
      end
    end
  end
  local function rebuild(use_ext_active)
    local old = glue.plan
    local old_id = old and old.legs[old.active] and old.legs[old.active].id
    local p = build_plan_from_xp(fms, ac)
    local xa = (xp_raw > 0) and (p.xp_map[xp_raw] or xp_raw) or 0
    if use_ext_active and ext and ext.a and p.xp_map[ext.a] then
      -- active leg chosen on the LEGS page (EXEC)
      p.active = p.xp_map[ext.a]
      if ext.ad and p.legs[p.active - 1] and p.legs[p.active - 1].t == "DISCO" then p.active = p.active - 1 end
      if p.ca_idx and ac.onground and p.active <= p.ca_idx + 1 then p.active = p.ca_idx end
      if ac.onground then ground_first_active(p) end
      xp_guard_t = simDRTime + 4
      xp_active_set = -1                  -- tell X-Plane the active entry
    else
      if xa > 1 then p.active = math.max(xa, p.first_flyable or 2) end
      -- on the ground the departure climb leg (not in X-Plane's plan) stays active
      if p.ca_idx and ac.onground and p.active <= p.ca_idx + 1 then p.active = p.ca_idx end
      if ac.onground then ground_first_active(p) end
      local new_id = p.legs[p.active] and p.legs[p.active].id
      if not ext and old and old_id ~= new_id and not ac.onground then
        -- active waypoint changed by the pilot (DIRECT TO on an X-Plane page): fly direct from present position
        local l = p.legs[p.active]
        if l then l.t = "DF"; l.dt_start = { lat = ac.lat, lon = ac.lon }; l.dt_trk = ac.trk_true; l.dt_gs = ac.gs end
      end
      xp_active_set = p.active
    end
    GL.set_plan(glue, p, {})
    fms_sig = sig
    fms_alt_sig = alt_signature(fms)
    xp_active_seen = xa
    if ext then ext_seen = ext.s; ext_lat_sig = ext_lateral_sig(fms) else ext_seen = nil; ext_lat_sig = "" end
  end
  if sig ~= fms_sig then
    -- route changed in the FMS
    rebuild(ext_new)
    return
  end
  local p = glue.plan
  if not p then return end
  local xmap = p.xp_map
  local xp_act = (xp_raw > 0) and ((xmap and xmap[xp_raw]) or xp_raw) or 0
  if ext_new then
    -- LEGS page EXEC on the same X-Plane list (discontinuity removed, constraint, intercept course ...)
    local ls = ext_lateral_sig(fms)
    if ls ~= ext_lat_sig or ext.a then
      if ls ~= ext_lat_sig then rebuild(true) return end
    end
    ext_seen = ext.s
    fms_alt_sig = ""                      -- vertical data changed: update below
  end
  -- altitude constraints edited (X-Plane LEGS page or our LEGS page): update them in place (keeps LNAV state)
  local asig = fms_alt_sig
  if simDRTime - alt_sig_t > 5 or asig == "" then alt_sig_t = simDRTime; asig = alt_signature(fms) end
  if asig ~= fms_alt_sig then
    fms_alt_sig = asig
    local np = build_plan_from_xp(fms, ac)
    for xi = 1, #fms do
      local pi_ = xmap and xmap[xi] or xi
      local ni = np.xp_map and np.xp_map[xi] or xi
      local l, nl_ = p.legs[pi_], np.legs[ni]
      if l and nl_ and nl_.id == l.id then l.alt = nl_.alt; l.spd = nl_.spd; l.fpa = nl_.fpa; l.alt_deleted = nil end
    end
    p.vserial = (p.vserial or 0) + 1      -- vertical change only: re-plan VNAV, keep the lateral path
  end
  -- pilot changed the active waypoint on the CDU without changing the list
  -- (X-Plane sometimes advances one waypoint on its own: that is not a DIRECT TO)
  if simDRTime > xp_guard_t and xp_act > 0 and xp_act >= (p.first_flyable or 2) and xp_act ~= xp_active_seen and xp_act ~= xp_active_set
    and xp_act ~= p.active + 1 and xp_act ~= p.active and p.legs[p.active] and p.legs[p.active].t ~= "DISCO" then
    p.active = xp_act
    local l = p.legs[p.active]
    if l and not ac.onground then l.t = "DF"; l.dt_start = { lat = ac.lat, lon = ac.lon }; l.dt_trk = ac.trk_true; l.dt_gs = ac.gs end
    RT.touch(p)
  end
  xp_active_seen = xp_act
  -- a leg that ends at an altitude / DME distance / intercept (VA, CA, VD, ...) has just been
  -- sequenced into a DIRECT TO fix leg (e.g. ROVER4B: 338 to 700 ft, then DF ARAKA): as on the
  -- aircraft, the direct-to is computed from where the aircraft actually is, on its present track -
  -- not from the predicted end of the climb leg - so the turn is drawn and flown from here
  if p.active ~= dyn_seq_seen then
    local prv, cur = p.legs[(p.active or 0) - 1], p.legs[p.active or 0]
    if dyn_seq_seen and p.active == dyn_seq_seen + 1 and prv and B744PATH.DYN[prv.t] and cur and cur.t == "DF"
      and not cur.dt_start and not ac.onground and ac.lat and ac.trk_true then
      cur.dt_start = { lat = ac.lat, lon = ac.lon }; cur.dt_trk = ac.trk_true; cur.dt_gs = ac.gs
      RT.touch(p)
    end
    dyn_seq_seen = p.active
  end
  -- our LNAV sequencing -> X-Plane FMS active entry (keeps the CDU and ND in step)
  local function xp_index_of(pi_)
    if not xmap then return pi_ end
    -- the waypoint entry (not a discontinuity entry mapped to the same leg)
    local any = nil
    for k = 1, #fms do
      if xmap[k] == pi_ then
        local x = fms[k][11]
        if not (x and x.xd) then return k end
        any = any or k
      end
    end
    return any
  end
  if p.active ~= xp_active_set then
    xp_active_set = p.active
    local xi = xp_index_of(p.active)
    -- only a real waypoint entry of the list X-Plane holds now (never its discontinuity entry)
    local fe = xi and fms[xi]
    local fe_ok = fe ~= nil and (fe[2] or 0) ~= 0 and not (math.abs(fe[5] or 0) < 1e-6 and math.abs(fe[6] or 0) < 1e-6)
      and not (type(fe[8]) == "string" and string.upper(fe[8]):find("DISCON") ~= nil)
    if xi and xi >= 1 and xi <= #fms and fe_ok and not legs_loading and simDRTime >= sc_hold then B747DR_fms_setCurrent = tostring(xi)
    else xp_active_set = -1 end
  end
  -- LEGS page: LNAV at a discontinuity (X-Plane has no such entry)
  local la = 0
  local al = p.legs[p.active]
  if al and al.t == "DISCO" then
    local nx = xp_index_of(p.active + 1)
    if nx then la = nx - 0.5 end
  end
  if la ~= lnav_active_pub then lnav_active_pub = la; B747DR_lnav_active = la end
end

-- point at along-track distance s on the plan
local function point_at(p, s)
  if not p or not p.legs then return nil end
  for i, l in ipairs(p.legs) do
    if l.cum and l.cum >= s and l.elems and #l.elems > 0 then
      local back = l.cum - s
      for k = #l.elems, 1, -1 do
        local e = l.elems[k]
        if back <= (e.len or 0) then
          local frac = 1 - back / math.max(0.001, e.len)
          if e.k == "A" then local q = B744PATH.arc_point(e, frac) return q.lat, q.lon end
          local la, lo = GEO.dest(e.a.lat, e.a.lon, GEO.brg(e.a.lat, e.a.lon, e.b.lat, e.b.lon), e.len * frac)
          return la, lo
        end
        back = back - (e.len or 0)
      end
      return l.pe and l.pe.lat, l.pe and l.pe.lon
    end
  end
  return nil
end

--*************************************************************************************--
--**                      ILS AUTOTUNE (from the stock FMS logic)                     **--
--*************************************************************************************--
local navAids = {}
local nSize = 0
local lastILSUpdate = 0
local targetILSS = ""
function getHeading(lat1, lon1, lat2, lon2) return GEO.brg(lat1, lon1, lat2, lon2) end
function getHeadingDifference(desireddirection, current_heading) return GEO.angdiff(current_heading, desireddirection) end
function getDistance(lat1, lon1, lat2, lon2) return GEO.dist(lat1, lon1, lat2, lon2) end
local function setILS(fmsO, dist_to_dest)
  local modes = B747DR_radioModes or " "
  if modes:sub(1, 1) == " " then B747DR_radioModes = replace_char(1, modes, "A") return end
  if modes:sub(1, 1) == "M" then return end
  if simDRTime - lastILSUpdate < 2 then return end
  lastILSUpdate = simDRTime
  if not fmsO or #fmsO < 3 then return end
  local dst = fmsO[#fmsO]
  if dst[2] ~= 1 then return end
  if (dist_to_dest or 999) > 45 then
    if S.roll ~= "LOC" then targetILSS = "" end
    return
  end
  if string.len(targetILSS) < 2 then
    if string.len(navAidsJSON) ~= nSize then
      local ok, t = pcall(function() return json.decode(navAidsJSON) end)
      if ok and type(t) == "table" then navAids = t end
      nSize = string.len(navAidsJSON)
    end
    local best, bestDiff = nil, 3
    for i = #fmsO - 1, 2, -1 do
      local d = getDistance(fmsO[i][5], fmsO[i][6], dst[5], dst[6])
      if d < 12 then
        local hdgNext = getHeading(fmsO[i][5], fmsO[i][6], fmsO[i + 1][5], fmsO[i + 1][6])
        for n = #navAids, 1, -1 do
          local na = navAids[n]
          if na[2] == 8 and getDistance(dst[5], dst[6], na[5], na[6]) < 10 then
            local diff = math.abs(getHeadingDifference(na[4], hdgNext))
            local toILS = getHeading(fmsO[i][5], fmsO[i][6], na[5], na[6])
            if diff < bestDiff and math.abs(getHeadingDifference(toILS, hdgNext)) < 6 then best = na; bestDiff = diff end
          end
        end
      end
      if best then break end
    end
    if best then targetILSS = json.encode(best); targetILS = targetILSS; safe_print("tuned ILS " .. targetILSS) end
  end
  if string.len(targetILSS) > 2 then
    local ok, ilsNav = pcall(function() return json.decode(targetILSS) end)
    if ok and ilsNav then
      simDR_nav1Freq = ilsNav[3]
      simDR_nav2Freq = ilsNav[3]
      local course = (ilsNav[4] + simDR_variation) % 360
      simDR_radio_nav_obs_deg[0] = course
      simDR_radio_nav_obs_deg[1] = course
    end
  end
end

--*************************************************************************************--
--**                                 HELPERS                                          **--
--*************************************************************************************--
function B747_animate_value(current_value, target, min, max, speed)
  local fps_factor = math.min(1.0, speed * SIM_PERIOD)
  if target >= (max - 0.001) and current_value >= (max - 0.01) then return max
  elseif target <= (min + 0.001) and current_value <= (min + 0.01) then return min
  else return current_value + ((target - current_value) * fps_factor) end
end
local function button_animation()
  for i = 0, NUM_AUTOPILOT_BUTTONS - 1 do
    local cur = B747DR_ap_button_switch_position[i]
    local tgt = btn_target[i]
    if math.abs(cur - tgt) > 0.001 then
      B747DR_ap_button_switch_position[i] = cur + (tgt - cur) * math.min(1, 30 * SIM_PERIOD)
    else
      B747DR_ap_button_switch_position[i] = tgt
    end
  end
end
local function flap_deg()
  return PF.flap_detent(simDR_flap_ratio_control)
end
function isATEnabled() return S.at_engaged end
function B747_ap_all_cmd_modes_off() for k = 1, 3 do S.ap[k] = false end end
function ap_reset() B747_ap_reset_CMDhandler(0, 0) end
function B747_set_ap_all_modes()
  simDR_autopilot_airspeed_is_mach = 0
  B747DR_ap_alt_show_thousands = 1
  B747DR_ap_ias_dial_value = 200
  B747DR_autopilot_altitude_ft = 10000
  B747DR_ap_heading_deg = 0
  simDR_autopilot_vs_fpm = 0
  S = AF.new()
end

--*************************************************************************************--
--**                                 MAIN LOOP                                        **--
--*************************************************************************************--
local function speed_kt_from_window()
  if simDR_autopilot_airspeed_is_mach == 1 then
    return PF.mach_to_cas(B747DR_ap_ias_dial_value / 100, simDR_pressureAlt1), true
  end
  return B747DR_ap_ias_dial_value, false
end

local B747_FMCAlertMsg_names = {
  "ATC COMM ESTABLISHED", "ATC MSG NOT ACKNOWLEDGED", "ATC REPORT LIST FULL", "CHECK ALT TGT", "CYCLE IRS OFF-NAV",
  "DESCENT PATH DELETED", "DES FORECST UPLINK READY", "DISCONTINUITY - LNAV", "DRAG REQUIRED", "END OF OFFSET",
  "END OF ROUTE", "ENTER IRS POSITION", "FLT NUMBER UPLINK", "FMC L/R OUTPUT DATA LOSS", "FUEL DISAGREE",
  "ILS TUNE INHIBITED", "INSUFFICIENT FUEL", "INVALID ATC UPLINK", "INVALID FLT NO UPLINK", "INVALID FORECAST UPLINK",
  "INVALID ROUTE UPLINK", "INVALID WIND DATA UPLINK", "IRS POS/ORIGIN DISAGREE", "LIMIT ALT FL___", "MESSAGE LIMIT EXCEEDED",
  "NAV INVALID-TUNE ___", "NO ACTIVE ROUTE", "PARTIAL CLEARANCE LOADED", "PARTIAL ROUTE UPLINK", "PERF/VNAV UNAVAILABLE",
  "PURGE UPDATES", "RE-LOGON TO ATC COMM", "RESET MCP ALT", "RESPOND TO ATC UPLINKS", "RESYNC FAIL", "RESYNCING OTHER FMC",
  "ROUTE UPLINK LOADING", "ROUTE UPLINK READY", "RTA FIX DELETED", "RW/ILS CRS ERROR", "SET CLOCK TO UTC TIME",
  "SINGLE FMC OPERATION", "SPLIT IRS OPERATION", "TAKEOFF SPEEDS DELETED", "THRUST REQUIRED", "UNABLE FL___ AT RTA FIX",
  "UNABLE NEXT ALT", "UNABLE RTA", "UNABLE TO LOAD CLEARANCE", "UNABLE TO SEND MESSAGE", "VERIFY POSITION", "VERIFY RNP",
  "WIND DATA UPLINK READY", "DESTINATION NOT SET" }

--*************************************************************************************--
--**  FLIGHT RECORDER: X-Plane 12\Output\B744_AFDS_flight.csv, one line every 2 s (for tuning)  **--
--*************************************************************************************--
local rec_f, rec_t, rec_failed = nil, -100, false
local function rec_num(x, f) if x == nil then return "" end return string.format(f or "%.1f", x) end
local function recorder(t, d)
  if rec_failed or t - rec_t < 2 then return end
  rec_t = t
  if not rec_f then
    if not (io and io.open) then rec_failed = true return end
    local ok, f = pcall(io.open, "Output/B744_AFDS_flight.csv", "w")
    if not ok or not f then rec_failed = true return end
    rec_f = f
    rec_f:write("t,lat,lon,alt,ra,ias,tas,gs,mach,vs,pitch,roll,trk,fma_r,fma_p,fma_at,ph,sub,vat,spd_tgt,path_alt,dev,s_now,s_tod,ed_s,ed_h,leg,xtk,turn,roll_cmd,pitch_cmd,lever,spdbrk,flaps,gear,wind,mcp_alt,mcp_spd,crz,drag,ap,gs_ang,vdef,hdef\n")
  end
  local ok = pcall(function()
    rec_f:write(table.concat({ rec_num(t), rec_num(d.lat, "%.5f"), rec_num(d.lon, "%.5f"), rec_num(d.alt, "%.0f"), rec_num(d.ra, "%.0f"),
      rec_num(d.ias), rec_num(d.tas), rec_num(d.gs), rec_num(d.mach, "%.3f"), rec_num(d.vs, "%.0f"), rec_num(d.pitch), rec_num(d.roll),
      rec_num(d.trk), tostring(d.fr), tostring(d.fp), tostring(d.fat), tostring(d.ph), tostring(d.sub), tostring(d.vat),
      rec_num(d.spd), rec_num(d.pa, "%.0f"), rec_num(d.dev, "%.0f"), rec_num(d.s_now, "%.2f"), rec_num(d.s_tod, "%.2f"),
      rec_num(d.ed_s, "%.2f"), rec_num(d.ed_h, "%.0f"), tostring(d.leg), rec_num(d.xtk, "%.3f"), tostring(d.turn),
      rec_num(d.rc), rec_num(d.pc), rec_num(d.lev, "%.3f"), rec_num(d.sb, "%.2f"), rec_num(d.flaps, "%.0f"), rec_num(d.gear, "%.0f"),
      rec_num(d.wind), rec_num(d.mcp_alt, "%.0f"), rec_num(d.mcp_spd, "%.0f"), rec_num(d.crz, "%.0f"), tostring(d.drag), tostring(d.ap), rec_num(d.gsang, "%.2f"), rec_num(d.vdef, "%.2f"), rec_num(d.hdef, "%.2f") }, ",") .. "\n")
    rec_f:flush()
  end)
  if not ok then rec_failed = true end
end

local function core()
  local t = simDRTime
  local dt = math.max(0.005, math.min(0.25, SIM_PERIOD or 0.05))
  local onground = simDR_onGround == 1
  local ra = simDR_radarAlt1
  local alt = simDR_pressureAlt1
  local ias = simDR_ind_airspeed_kts_pilot
  local tas = simDR_TAS_mps * 1.94384
  local gs = simDR_groundspeed * 1.94384
  ias_rate = LW.speed_rate(spd_filt, ias, gs, tas, dt)   -- complementary (inertial + IAS) speed trend
  -- along-track wind component (+ tailwind), filtered: used for the VNAV predictions
  if not onground and tas > 100 then
    local wc = gs - tas * math.cos(math.rad(((simDR_true_heading - (math.deg(math.atan2(simDR_local_vx or 0, -(simDR_local_vz or 0))) % 360)) + 540) % 360 - 180))
    wind_comp_f = wind_comp_f + (wc - wind_comp_f) * math.min(1, dt / 60)
    wind_alt_f = alt
  end
  local mv = simDR_variation
  local hdg_mag = simDR_AHARS_heading_deg_pilot
  local hdg_true = simDR_true_heading
  -- instantaneous ground track from the inertial velocity vector (OpenGL frame: x east, z south);
  -- the cockpit ground-track gauge is filtered and hpath is the air-mass track (no wind)
  local vx, vz = simDR_local_vx or 0, simDR_local_vz or 0
  local trk_true = hdg_true
  if gs > 30 and (vx * vx + vz * vz) > 100 then trk_true = math.deg(math.atan2(vx, -vz)) % 360
  elseif gs > 30 then trk_true = (simDR_ground_track_mag + (hdg_true - hdg_mag)) % 360 end
  -- in flight: ground track = air vector (true heading, TAS) + wind, with the wind vector filtered
  -- (3 s): turbulence no longer shakes the track the roll laws steer on, turns are not lagged
  if not onground and tas > 80 and (vx * vx + vz * vz) > 100 then
    local tm = tas * 0.514444
    local ax, ay = tm * math.sin(math.rad(hdg_true)), tm * math.cos(math.rad(hdg_true))
    local wx, wy = vx - ax, -vz - ay
    if wind_vx_f == nil then wind_vx_f, wind_vy_f = wx, wy end
    local k = math.min(1, dt / 3.0)
    wind_vx_f = wind_vx_f + (wx - wind_vx_f) * k
    wind_vy_f = wind_vy_f + (wy - wind_vy_f) * k
    trk_true = math.deg(math.atan2(ax + wind_vx_f, ay + wind_vy_f)) % 360
  else
    wind_vx_f, wind_vy_f = nil, nil
  end
  local trk_mag = (gs > 30) and ((trk_true - (hdg_true - hdg_mag)) % 360) or hdg_mag
  local vs = simDR_vh_ind_fpm       -- instantaneous (the cockpit VSI needle is lagged)
  local gamma = (tas > 40) and math.deg(math.asin(math.max(-0.5, math.min(0.5, vs / (tas * 101.27))))) or 0
  local fl = flap_deg()
  -- MCP
  local mcp_spd, mcp_is_mach = speed_kt_from_window()
  local fd_capt = B747DR_toggle_switch_position[23] > 0.5 and 1 or 0
  local fd_fo = B747DR_toggle_switch_position[24] > 0.5 and 1 or 0
  local at_sw = (B747DR_toggle_switch_position[29] > 0.5) and 1 or 0
  B747DR_ap_autothrottle_armed = at_sw
  -- TO/GA presses (counted by the engines script)
  local tc = B747DR_afds_toga_count or 0
  if last_toga_count == nil then last_toga_count = tc end
  if tc ~= last_toga_count then
    last_toga_count = tc
    fdr(B747CMD_fdr_log_toga)
    AF.event(S, "TOGA")
  end
  -- route / FMC data
  local fd = fmsdata()
  local crz = B747BR_cruiseAlt or 0
  if crz <= 0 then crz = num(fd.crzalt) or 0 end
  local ci = num(fd.costindex) or 100
  local v2 = (B747DR_airspeed_V2 and B747DR_airspeed_V2 < 998 and B747DR_airspeed_V2 > 100) and B747DR_airspeed_V2 or nil
  local ac = { lat = simDR_latitude, lon = simDR_longitude, trk_true = trk_true, hdg_true = hdg_true, gs = gs, alt = alt,
    ra = ra, onground = onground, goaround = S.goaround, t = t, dt = dt, ias = ias, mach = simDR_airspeed_mach, vs = vs,
    tas = tas, agl = ra, wind_comp = wind_comp_f, ias_rate = ias_rate }
  sync_route(ac)
  local w = simDR_gwt
  local orig = glue.plan and glue.plan.origin
  local dest = glue.plan and glue.plan.dest
  local ctx = { wind_comp = wind_comp_f, wind_alt = wind_alt_f, crz = crz, ci = ci, w = w, fuel = simDR_fuel_total, v2 = v2 or 160, accel_ht = 1000, thr_red = 1500,
    orig_elev = orig and orig.elev or 0, dest_elev = dest and dest.elev or 0, vmo = B747DR_airspeed_Vmo or 365,
    flaps_up = (B747DR_airspeed_Vf0 and B747DR_airspeed_Vf0 > 100) and B747DR_airspeed_Vf0 or nil }
  -- DES FORECAST winds (FMC): forecast list and the descent track (towards the destination)
  do
    local ws = B747DR_des_winds
    if type(ws) == "string" and #ws > 4 then
      local list = {}
      for a_, d_, v_ in ws:gmatch("(%-?%d+),(%d+),(%d+)") do list[#list + 1] = { alt = tonumber(a_), dir = tonumber(d_), spd = tonumber(v_) } end
      local pl = glue.plan
      if #list > 0 and pl and pl.dest and pl.legs then
        -- (from the last route fix 40 nm or more before the destination: the general descent track)
        local from = nil
        for k = #pl.legs, 1, -1 do
          local l = pl.legs[k]
          if l.fix and l.seg ~= "MISSED" and l.fix.kind ~= "APT" and GEO.dist(l.fix.lat, l.fix.lon, pl.dest.lat, pl.dest.lon) > 40 then from = l.fix break end
        end
        if from then
          ctx.des_winds = list
          ctx.des_trk = GEO.brg(from.lat, from.lon, pl.dest.lat, pl.dest.lon)
          ctx.des_winds_sig = ws .. string.format("@%.0f", ctx.des_trk)
        end
      end
    end
  end
  -- speeds selected on the FMC VNAV pages (CRZ .810, DES .805/270, DES transition 240/10000 ...)
  local function mach_of(x) local n = num(x) if n and n > 1 then n = n / 1000 end if n and n >= 0.5 and n <= 0.92 then return n end return nil end
  local function cas_of(x) local n = num(x) if n and n >= 150 and n <= 360 then return n end return nil end
  ctx.sel_crz_mach = mach_of(fd.crzspd)
  ctx.sel_des_mach = mach_of(fd.desspdmach)
  ctx.sel_des_cas = cas_of(fd.desspd)
  local csel = cas_of(fd.clbspd)
  if csel and csel > 260 then ctx.sel_clb_cas = csel end
  ctx.des_trans = { spd = cas_of(fd.destranspd) or 240, alt = num(fd.desspdtransalt) or 10000 }
  local flags = { vnav_engaged = (S.pitch == "VNAV"), mcp_alt = B747DR_autopilot_altitude_ft, mcp_spd = mcp_spd,
    spd_intv = S.spd_intv, alt_push = S.alt_push, des_now = S.des_now, flaps = fl,
    -- flap maneuver speeds (PFD flap bugs): VNAV flies the flap retraction schedule after takeoff
    vf = { [0] = B747DR_airspeed_Vf0, [1] = B747DR_airspeed_Vf1, [5] = B747DR_airspeed_Vf5, [10] = B747DR_airspeed_Vf10,
      [20] = B747DR_airspeed_Vf20 },
    thr_idle = math.max(B747DR_throttle[0], B747DR_throttle[1], B747DR_throttle[2], B747DR_throttle[3]) < 0.03 }
  -- climb performance learning (T/C prediction fitted to this aircraft; kept between flights)
  flags.clb_learn = glue.vst and glue.vst.phase == "CLB" and S.at == "THR_REF" and fl == 0 and (simDR_gear_handle or 0) == 0
    and ra > 3000 and math.abs(simDR_AHARS_roll_deg_pilot) < 10 and (S.pitch == "VNAV" or S.pitch == "FLCH")
  flags.des_learn = glue.vst and glue.vst.phase == "DES" and (S.at == "IDLE" or S.at == "HOLD") and flags.thr_idle
    and (simDR_speedbrake_ratio or 0) < 0.05 and fl == 0 and (simDR_gear_handle or 0) == 0 and math.abs(simDR_AHARS_roll_deg_pilot) < 10
    and (S.pitch == "VNAV" or (S.pitch == "FLCH" and (S.flch_dir or 1) < 0))
  flags.spdbrk = simDR_speedbrake_ratio or 0
  if not clb_tab_loaded then
    clb_tab_loaded = true
    if io and io.open then
      local ok, f = pcall(io.open, "Output/preferences/B744_AFDS_climb.txt", "r")
      if ok and f then
        local t, i = {}, 0
        for v in (f:read("*l") or ""):gmatch("[%d%.]+") do local x = tonumber(v) if x and x >= 0.2 and x <= 2 then t[i] = x end i = i + 1 end
        f:close()
        glue.clb_tab = t
      end
      local ok2, f2 = pcall(io.open, "Output/preferences/B744_AFDS_descent2.txt", "r")
      if ok2 and f2 then
        local t, i = {}, 0
        for v in (f2:read("*l") or ""):gmatch("[%d%.]+") do local x = tonumber(v) if x and x >= 0.4 and x <= 1.6 then t[i] = x end i = i + 1 end
        f2:close()
        glue.des_tab = t
      end
    end
  end
  if (glue.clb_learned or glue.des_learned) and simDRTime - clb_tab_saved > 60 and io and io.open then
    clb_tab_saved = simDRTime
    if glue.des_learned then
      local okd, fd_ = pcall(io.open, "Output/preferences/B744_AFDS_descent2.txt", "w")
      if okd and fd_ then
        local parts = {}
        -- (v41: a band not learned is saved as 0 and skipped on loading - it used to be saved as 1)
        for i = 0, 9 do parts[#parts + 1] = string.format("%.3f", (glue.des_tab or {})[i] or 0) end
        fd_:write(table.concat(parts, " ")); fd_:close()
      end
    end
    local ok, f = pcall(io.open, "Output/preferences/B744_AFDS_climb.txt", "w")
    if ok and f then
      local parts = {}
      for i = 0, 9 do parts[#parts + 1] = string.format("%.3f", (glue.clb_tab or {})[i] or 1) end
      f:write(table.concat(parts, " ")); f:close()
    end
  end
  local lout, vout = GL.update(glue, ac, ctx, flags)
  if glue.new_crz then
    -- altitude intervention changed the cruise altitude
    B747BR_cruiseAlt = glue.new_crz
    glue.new_crz = nil
  end
  -- nav receivers (NAV1)
  -- an ILS is tuned when NAV1 is on an ILS/LOC frequency (108.10-111.95 MHz, odd tenths)
  local f = simDR_nav1Freq or 0
  local ils_tuned = f >= 10810 and f <= 11195 and (math.floor(f / 10) % 2 == 1)
  local loc_valid = ils_tuned and simDR_nav1_horizontal == 1
  local gs_valid = loc_valid and simDR_nav1_vertical == 1 and simDR_nav1_gs_flag == 0
  -- glideslope plausibility: X-Plane models the false glideslope lobes above the beam (6, 9 deg ..).
  -- Reject the signal when the aircraft's elevation angle from the landing runway is well above the
  -- glideslope (needs the runway or destination in the route; DME distance when available)
  if gs_valid and glue.plan then
    local rp = nil
    for k = #glue.plan.legs, 1, -1 do
      local l = glue.plan.legs[k]
      if l.fix and l.fix.kind == "RWY" and l.seg ~= "RWY" then rp = l.fix break end
    end
    rp = rp or glue.plan.dest
    if rp and rp.lat then
      local d = GEO.dist(simDR_latitude, simDR_longitude, rp.lat, rp.lon)
      if simDR_nav1_has_dme == 1 and simDR_nav1_dme > 0.3 then d = simDR_nav1_dme end
      local elev = (glue.plan.dest and glue.plan.dest.elev) or 0
      local ang = math.deg(math.atan((alt - elev) / math.max(600, d * 6076.12)))
      gs_ang_geo = ang
      if ang > 4.4 then gs_valid = false end
    end
  end
  -- localizer offset from the runway geometry (landing runway in the route): lets the capture turn be
  -- planned without knowing the localizer's beam width
  local xtk_geo = nil
  local hd_ = simDR_nav1_hdef
  local mvar = hdg_true - hdg_mag
  -- landing runway and the approach course into it from the route (the leg before the runway)
  local rwy_p, plan_crs_t = nil, nil
  if glue.plan then
    for k = #glue.plan.legs, 1, -1 do
      local l = glue.plan.legs[k]
      if l.fix and l.fix.kind == "RWY" and l.seg ~= "RWY" then
        rwy_p = l.fix
        for m = k - 1, math.max(1, k - 4), -1 do
          local q = glue.plan.legs[m]
          if q.fix and q.fix.lat and GEO.dist(q.fix.lat, q.fix.lon, rwy_p.lat, rwy_p.lon) > 2 then
            plan_crs_t = GEO.brg(q.fix.lat, q.fix.lon, rwy_p.lat, rwy_p.lon); break
          end
        end
        break
      end
    end
  end
  -- glideslope only within 17 nm of the landing runway (distance to the runway in the route, else the
  -- ILS DME, else the distance to the destination): the G/S pointer is shown and G/S can capture only
  -- there - farther out the signal is unusable / false lobes, and on short routes it would always show
  local gs_dist = nil
  if rwy_p and rwy_p.lat then gs_dist = GEO.dist(simDR_latitude, simDR_longitude, rwy_p.lat, rwy_p.lon)
  elseif simDR_nav1_has_dme == 1 and simDR_nav1_dme > 0.3 then gs_dist = simDR_nav1_dme
  elseif B747BR_distance_to_dest and B747BR_distance_to_dest >= 0 then gs_dist = B747BR_distance_to_dest end
  local gs_in_range = gs_dist ~= nil and gs_dist <= 17.0
  if not gs_in_range and S.pitch ~= "GS" and S.pitch ~= "FLARE" then gs_valid = false end
  -- course reference for the geometry checks (true): the localizer course in use; the route's final
  -- approach course when it agrees with the selected course (independent of the receiver)
  local function loc_ref_crs_t()
    local c = ((loc_crs_est or simDR_nav1_course) + mvar) % 360
    local sel = (simDR_nav1_course + mvar) % 360
    if plan_crs_t and math.abs(GEO.angdiff(plan_crs_t, sel)) < 10 then return plan_crs_t end
    return c
  end
  if loc_valid and glue.plan then
    local rp = rwy_p
    if rp then
      local d = GEO.dist(simDR_latitude, simDR_longitude, rp.lat, rp.lon)
      local crs_t = loc_ref_crs_t()
      -- the runway must be ahead on the localizer course and the beam consistent with the geometry
      if d < 30 then
        local bla, blo = GEO.dest(rp.lat, rp.lon, (crs_t + 180) % 360, 20)
        local x = GEO.xtk_atk(bla, blo, rp.lat, rp.lon, simDR_latitude, simDR_longitude)
        local ok = (x > 0) == (hd_ < 0) or math.abs(hd_) < 0.3
        if ok and math.abs(hd_) > 0.3 and math.abs(hd_) < 2.3 then
          -- an offset localizer (not on the runway centreline) shows up as a mismatch: ignore geometry
          local xh = -d * math.tan(math.rad(hd_ * LW.P.loc_dot_deg))
          local r = x / xh
          if r < 0.4 or r > 2.5 then ok = false end
        end
        if ok then xtk_geo = x end
      end
    end
  end
  -- only the real localizer beam is captured:
  --  * the signal must have been valid for 3 s on this frequency (just tuned / coming into range the
  --    receiver reports a centred needle before it has a real deviation)
  --  * with the landing runway in the route, the needle must agree with where the aircraft actually is:
  --    more than 6 deg off the course line (beyond full scale) or outside the +-35 deg coverage /
  --    behind the antenna, an on-course or small deviation is a false signal (side lobe, ghost)
  if loc_valid then
    if loc_ok_freq ~= f or not loc_ok_t then loc_ok_t = t; loc_ok_freq = f end
  else
    loc_ok_t = nil
  end
  local loc_cap_ok = loc_valid and loc_ok_t ~= nil and (t - loc_ok_t) >= 3.0
  if loc_valid and glue.plan then
    local rp = rwy_p
    if rp then
      local d = GEO.dist(simDR_latitude, simDR_longitude, rp.lat, rp.lon)
      local crs_t = loc_ref_crs_t()
      local brg_from = GEO.brg(rp.lat, rp.lon, simDR_latitude, simDR_longitude)
      local off = math.abs(GEO.angdiff(brg_from, (crs_t + 180) % 360))   -- angle off the approach course line
      if d > 1.0 then
        if off > 35 then loc_cap_ok = false
        elseif off > 6 and math.abs(simDR_nav1_hdef) < 2.0 then loc_cap_ok = false end
      end
    end
  end
  local hdef = simDR_nav1_hdef
  local vdef = simDR_nav1_vdef
  -- localizer course: from the bearing to the LOC antenna corrected by the deviation (independent of the
  -- course the pilot / autotune set); the selected course is used when it agrees
  -- (only from a real beam: a single false reading - a side lobe / ghost giving an in-beam needle far off
  --  the course - must not move the course. Taken when the estimate agrees with the selected course or
  --  the route's final approach course, or after the needle has been in the beam for 10 s)
  if loc_valid and math.abs(hdef) < 2.0 then loc_inbeam_t = loc_inbeam_t or t else loc_inbeam_t = nil end
  if loc_valid and loc_cap_ok and math.abs(hdef) < 2.0 then
    local c = (simDR_nav1_bearing - hdef * LW.P.loc_dot_deg) % 360
    local agree = math.abs(GEO.angdiff(c, simDR_nav1_course)) < 10
      or (plan_crs_t and math.abs(GEO.angdiff(c, (plan_crs_t - mvar) % 360)) < 10)
      or (loc_inbeam_t and t - loc_inbeam_t >= 10)
    if agree then
      if loc_crs_est == nil or ils_freq_est ~= f then loc_crs_est = c; ils_freq_est = f
      else loc_crs_est = (loc_crs_est + GEO.angdiff(c, loc_crs_est) * math.min(1, dt / 6)) % 360 end
    end
  elseif not ils_tuned then loc_crs_est = nil end
  if loc_crs_est and ils_freq_est ~= f then loc_crs_est = nil end
  local crs = simDR_nav1_course
  if loc_crs_est and math.abs(GEO.angdiff(loc_crs_est, crs)) > 4 then crs = loc_crs_est end
  ils_on = ils_tuned
  local nav = { xtk_geo = xtk_geo, loc_valid = loc_valid, loc_cap_ok = loc_cap_ok, gs_valid = gs_valid, hdef = hdef, vdef = vdef, crs = crs,
    hdef_rate = (hdef - hdef_prev) / dt, dme = (simDR_nav1_has_dme == 1) and simDR_nav1_dme or nil, gs_angle = 3.0 }
  hdef_prev, vdef_prev = hdef, vdef
  last_nav = nav
  -- yoke override (pilot force)
  local yo = math.abs(simDR_yoke_pitch) > 0.55 or math.abs(simDR_yoke_roll) > 0.55
  local vmax = (PF.FLAP_PLACARD[fl] or 365)
  if fl == 0 then vmax = B747DR_airspeed_Vmo or 365 end
  local thr = math.max(B747DR_throttle[0], B747DR_throttle[1], B747DR_throttle[2], B747DR_throttle[3])
  local rev = math.max(simDR_reverser_deploy[0], simDR_reverser_deploy[1], simDR_reverser_deploy[2], simDR_reverser_deploy[3]) > 0.1
  local inp = { t = t, dt = dt, onground = onground, ra = ra, alt = alt, vs = vs, ias = ias, mach = simDR_airspeed_mach, tas = tas,
    gs = gs, pitch = simDR_AHARS_pitch_deg_pilot, roll = simDR_AHARS_roll_deg_pilot, hdg_mag = hdg_mag, trk_mag = trk_mag,
    gamma = gamma, ias_rate = ias_rate, flaps = fl, fd_capt = fd_capt, fd_fo = fd_fo, at_arm_sw = at_sw,
    at_fail = false, bar = (btn_target[14] > 0.5) and 1 or 0, yoke_override = yo,
    mcp_alt = B747DR_autopilot_altitude_ft, mcp_spd = mcp_spd, mcp_hdg = B747DR_ap_heading_deg,
    bank_sel = B747DR_ap_bank_limit_sel_dial_pos, nav = nav, lnav = lout, vnav = vout, thr_lever = thr,
    v2 = v2 or 160, vmax = vmax, reverse = rev, gw = simDR_gwt }
  local out = AF.step(S, inp, LW)
  last_out = out

  -- WINDOWS -----------------------------------------------------------------------------------------
  if S.ias_sync then
    S.ias_sync = false
    local v = out.spd_tgt or ias
    if out.spd_src == "VNAV" and vout.spd then v = vout.spd end
    if simDR_autopilot_airspeed_is_mach == 1 or (vout.ism and S.spd_intv) then
      simDR_autopilot_airspeed_is_mach = 1
      B747DR_ap_ias_dial_value = math.floor(PF.cas_to_mach(v, alt) * 100 + 0.5)
    else
      B747DR_ap_ias_dial_value = math.floor(v + 0.5)
    end
  end
  B747DR_ap_ias_mach_window_open = out.ias_win and 1 or 0
  -- automatic IAS/MACH changeover (744 MCP)
  if out.ias_win then
    if simDR_autopilot_airspeed_is_mach == 0 and simDR_airspeed_mach >= 0.84 and vs > 0 then
      simDR_autopilot_airspeed_is_mach = 1
      B747DR_ap_ias_dial_value = math.floor(simDR_airspeed_mach * 100 + 0.5)
    elseif simDR_autopilot_airspeed_is_mach == 1 and ias >= 310 and vs < 0 then
      simDR_autopilot_airspeed_is_mach = 0
      B747DR_ap_ias_dial_value = math.floor(ias + 0.5)
    end
  end
  B747DR_ap_mach_decimal_visibiilty = simDR_autopilot_airspeed_is_mach
  if S.vs_sync then
    S.vs_sync = false
    simDR_autopilot_vs_fpm = S.vs_target or 0
    B747DR_autopilot_vs_fpm = simDR_autopilot_vs_fpm
  end
  if out.vs_win then S.vs_target = simDR_autopilot_vs_fpm end
  B747DR_ap_vs_window_open = out.vs_win and 1 or 0
  -- (the thousands digit only while the V/S window is open: with the window blank the digit of the
  -- last V/S - e.g. "6" - stayed in the window)
  B747DR_ap_vs_show_thousands = (out.vs_win and math.abs(simDR_autopilot_vs_fpm) >= 1000) and 1 or 0
  B747DR_ap_vvi_fpm = simDR_autopilot_vs_fpm

  -- SPEED TARGET for the A/T (EEC uses the sim speed dial) and the PFD bug ---------------------------
  local tgt = out.spd_tgt or mcp_spd
  simDR_autopilot_airspeed_kts = tgt
  B747DR_ap_ias_bug_value = tgt
  B747DR_ap_ias_bug_value_pfd = tgt
  -- MCP window closed (VNAV): the PFD readout shows the FMC target speed
  if not out.ias_win and out.spd_src == "VNAV" and vout.spd then
    if vout.ism then
      simDR_autopilot_airspeed_is_mach = 1
      B747DR_ap_ias_dial_value = math.floor((vout.mach or PF.cas_to_mach(vout.spd, alt)) * 100 + 0.5)
    else
      simDR_autopilot_airspeed_is_mach = 0
      B747DR_ap_ias_dial_value = math.floor(vout.spd + 0.5)
    end
    B747DR_ap_mach_decimal_visibiilty = simDR_autopilot_airspeed_is_mach
  end
  B747DR_ap_ias_mach_dial_value = (simDR_autopilot_airspeed_is_mach == 1) and (B747DR_ap_ias_dial_value * 0.01) or B747DR_ap_ias_dial_value

  -- ALTITUDE displays / carriers --------------------------------------------------------------------
  B747DR_autopilot_altitude_ft_pfd = B747DR_autopilot_altitude_ft
  simDR_autopilot_altitude_ft = B747DR_autopilot_altitude_ft
  simDR_disabled_autopilot_altitude_ft = 150000
  B747DR_ap_alt_show_thousands = 1
  B747DR_ap_alt_show_tenThousands = (B747DR_autopilot_altitude_ft >= 10000) and 1 or 0
  if S.pitch == "ALT" then simDR_autopilot_hold_altitude_ft = S.alt_hold or alt end
  -- heading bug follows the MCP heading
  simDR_autopilot_heading_deg = B747DR_ap_heading_deg
  simDR_autopilot_heading_deg_fo = B747DR_ap_heading_deg
  if B747DR_ap_activate_target_heading_deg == 1 then
    B747DR_ap_activate_target_heading_deg = 0
    AF.event(S, "HDG_SEL")
  end

  -- FMA ---------------------------------------------------------------------------------------------
  B747DR_ap_FMA_active_roll_mode = out.roll
  B747DR_ap_FMA_armed_roll_mode = out.roll_arm
  B747DR_ap_FMA_active_pitch_mode = out.pitch
  B747DR_ap_FMA_armed_pitch_mode = out.pitch_arm
  B747DR_ap_FMA_autothrottle_mode = out.at_engaged and out.at or 0
  B747DR_ap_roll_mode_box_status = out.box_roll and 1 or 0
  B747DR_ap_pitch_mode_box_status = out.box_pitch and 1 or 0
  B747DR_ap_FMA_autothrottle_mode_box_status = out.box_at and 1 or 0
  B747DR_ap_AFDS_status_annun_pilot = out.status
  B747DR_ap_AFDS_status_annun_copilot = out.status
  local sbox = out.box_status and 1 or 0
  B747DR_ap_AFDS_mode_box_status_pilot = sbox
  B747DR_ap_AFDS_mode_box_status_copilot = sbox
  B747DR_ap_AFDS_mode_box2_status_pilot = 0
  B747DR_ap_AFDS_mode_box2_status_copilot = 0
  B747BR_showFMA = 1

  -- AP / A/T state ----------------------------------------------------------------------------------
  B747DR_ap_cmd_L_mode = S.ap[1] and 1 or 0
  B747DR_ap_cmd_C_mode = S.ap[2] and 1 or 0
  B747DR_ap_cmd_R_mode = S.ap[3] and 1 or 0
  local servo = out.servos and 1 or 0
  if servo ~= simDR_autopilot_servos_on then B747DR_switching_servos_on = t end
  simDR_autopilot_servos_on = servo
  B747DR_autothrottle_active = out.at_engaged and 1 or 0
  if out.warn_ap then B747DR_CAS_warning_status[0] = 1 else B747DR_CAS_warning_status[0] = 0 end
  B747DR_CAS_caution_status[5] = out.warn_at and 1 or 0
  -- legacy state carriers read by other scripts
  B747DR_ap_lnav_state = (S.roll == "LNAV") and 2 or (S.lnav_arm and 1 or 0)
  B747DR_ap_vnav_state = (S.pitch == "VNAV") and 2 or (S.vnav_arm and 1 or 0)
  B747DR_ap_inVNAVdescent = (S.pitch == "VNAV" and vout.phase == "DES") and 1 or 0
  B747DR_ap_inDescent = (vout.phase == "DES") and 1 or 0
  simDR_autopilot_alt_hold_status = (S.pitch == "ALT" or (S.pitch == "VNAV" and vout.pitch == "ALT")) and 2 or 0
  simDR_autopilot_vs_status = (S.pitch == "VS") and 2 or 0
  simDR_autopilot_flch_status = (S.pitch == "FLCH") and 2 or 0
  B747DR_autopilot_TOGA_status = (S.roll == "TOGA" or S.pitch == "TOGA") and 1 or 0
  B747DR_autopilot_nav_status = (S.roll == "LOC") and 2 or (S.loc_arm and 1 or 0)
  B747DR_autopilot_gs_status = (S.pitch == "GS") and 2 or (S.gs_arm and 1 or 0)
  B747DR_autopilot_acutal_nav_status = B747DR_autopilot_nav_status
  B747DR_autopilot_acutal_gs_status = B747DR_autopilot_gs_status
  B747DR_ap_approach_mode = S.app and 1 or ((S.loc_arm or S.roll == "LOC") and -1 or 0)
  B747DR_ap_autoland = S.goaround and -2 or 0
  B747DR_ap_active_land = S.rollout and 1 or 0
  B747DR_ap_ATT = 0
  B747DR_ap_thrust_mode = (S.at == "THR" or S.at == "THR_REF") and 2 or 0

  -- MCP switch lights -------------------------------------------------------------------------------
  local L = B747DR_afds_mcp_lights
  L[0] = (out.at_engaged and S.at == "THR_REF") and 1 or 0
  L[1] = (out.at_engaged and S.at == "SPD" and S.pitch ~= "VNAV") and 1 or 0
  L[2] = (S.roll == "LNAV" or S.lnav_arm) and 1 or 0
  L[3] = (S.pitch == "VNAV" or S.vnav_arm) and 1 or 0
  L[4] = (S.pitch == "FLCH") and 1 or 0
  L[5] = (S.roll == "HDG_HOLD") and 1 or 0
  L[6] = (S.pitch == "VS") and 1 or 0
  L[7] = (S.pitch == "ALT") and 1 or 0
  L[8] = ((S.loc_arm or S.roll == "LOC") and not S.app) and 1 or 0
  L[9] = (S.app or (S.roll == "LOC" and S.pitch == "GS")) and 1 or 0
  L[10] = S.ap[1] and 1 or 0
  L[11] = S.ap[2] and 1 or 0
  L[12] = S.ap[3] and 1 or 0

  -- GUIDANCE -> servos / flight directors -------------------------------------------------------------
  B747DR_afds_pitch_cmd = out.pitch_cmd or 0
  B747DR_afds_roll_cmd = out.roll_cmd or 0
  B747DR_afds_cmd_valid = out.valid and 1 or 0
  -- control wheel / column back-drive (the 744 autopilot servos move the column and wheel)
  if servo == 1 and out.valid then
    B747DR_yoke_anim_pitch = B747_animate_value(B747DR_yoke_anim_pitch, B747DR_sim_pitch_ratio, -1, 1, 8)
    B747DR_yoke_anim_roll = B747_animate_value(B747DR_yoke_anim_roll, B747DR_sim_roll_ratio, -1, 1, 8)
  else
    B747DR_yoke_anim_pitch = B747_animate_value(B747DR_yoke_anim_pitch, simDR_yoke_pitch, -1, 1, 20)
    B747DR_yoke_anim_roll = B747_animate_value(B747DR_yoke_anim_roll, simDR_yoke_roll, -1, 1, 20)
  end
  B747DR_afds_thr_target = ((S.at == "THR" or S.at == "SPD") and out.at_engaged) and (out.thr_target or -1) or -1
  if out.rudder and (S.flare or S.rollout) and servo == 1 then
    B747DR_afds_rudder_cmd = out.rudder
    B747DR_afds_rudder_valid = 1
    simDR_rudder = B747_animate_value(simDR_rudder, out.rudder, -1, 1, 3)
  elseif B747DR_afds_rudder_valid == 1 then
    B747DR_afds_rudder_valid = 0
    simDR_rudder = 0
  end
  -- PFD flight director bars
  -- roll bar: commanded minus actual bank - the same roll command the autopilot is flying (AP
  -- engaged) or the pilot should fly (FD only), so the bars and the autopilot always agree
  local fdk = 1.0
  local fdroll = (out.valid and S.roll ~= "NONE") and fdk * (out.roll_cmd - simDR_capt_roll) or 0
  B747DR_ap_capt_roll = B747_animate_value(B747DR_ap_capt_roll, math.max(-25, math.min(25, fdroll)), -25, 25, 4)
  local fdroll2 = (out.valid and S.roll ~= "NONE") and fdk * (out.roll_cmd - simDR_fo_roll) or 0
  B747DR_ap_fo_roll = B747_animate_value(B747DR_ap_fo_roll, math.max(-25, math.min(25, fdroll2)), -25, 25, 4)

  B747DR_afds_lnav_roll = lout.roll or 0
  B747DR_afds_vnav_sub = tostring(vout.phase or "-") .. "/" .. tostring(vout.sub or "-") .. "/" .. tostring(vout.at or "-")

  -- FLIGHT PHASE for the EEC thrust reference ---------------------------------------------------------
  local ph = B747DR_ap_flightPhase
  if onground and gs < 40 then ph = 0; thr_red_done = false
  elseif not onground then
    local agl = ra
    if not thr_red_done and agl > 1500 then thr_red_done = true end
    if vout.valid and vout.phase then
      if vout.phase == "TO" then ph = thr_red_done and 1 or 0
      elseif vout.phase == "CLB" then ph = thr_red_done and 1 or 0
      elseif vout.phase == "CRZ" then ph = 2
      else ph = 3 end
    else
      if not thr_red_done then ph = 0
      elseif vs > 400 and (crz == 0 or alt < crz - 500) then ph = 1
      elseif vs < -400 then ph = 3
      elseif alt > 20000 then ph = 2 end
    end
  end
  B747DR_ap_flightPhase = ph

  -- ROUTE / VNAV data for ND, EICAS, FMC pages --------------------------------------------------------
  local P = glue.prof
  local plan = glue.plan
  if plan and P and P.ok and lout.s_now then
    local remain = (P.ed_s or P.total or 0) - lout.s_now
    B747BR_totalDistance = math.max(0, remain)
    B747BR_tod = math.max(0, (P.ed_s or 0) - (P.s_tod or 0))
    local la, lo = point_at(plan, P.s_tod or 0)
    if la then B747BR_todLat = la; B747BR_todLong = lo end
    if vout.phase == "CLB" or vout.phase == "TO" then
      B747BR_toc = math.max(0, (P.s_toc or 0) - lout.s_now)
      local la2, lo2 = point_at(plan, P.s_toc or 0)
      if la2 then B747BR_tocLat = la2; B747BR_tocLong = lo2 end
    else
      B747BR_toc = -1
    end
    B747BR_fpe = vout.path_dev or 0
    B747BR_eod_index = P.ed_i or 0
    B747BR_distance_to_dest = dest and GEO.dist(simDR_latitude, simDR_longitude, dest.lat, dest.lon) or -1
    B747DR_fmscurrentIndex = plan.active
    B747DR_fmstargetDistance = lout.dtg or 0
    B747BR_nextDistanceInFeet = (lout.dtg or 0) * 6076.12
  else
    B747BR_totalDistance = 0
    B747BR_toc = -1
    B747BR_distance_to_dest = -1
  end
  B747DR_ap_lnav_xtk_error = lout.xtk or 0
  B747DR_ap_vnav_target_alt = vout.alt_tgt or B747DR_autopilot_altitude_ft
  -- FMC messages (shown on the CDU scratchpad, EICAS FMC MESSAGE)
  local function note(name, on)
    for i = 1, 54 do
      if B747_FMCAlertMsg_names[i] == name then B747DR_fmc_notifications[i] = on and 1 or 0 return end
    end
  end
  local vm = vout.msg or {}
  note("DRAG REQUIRED", vm.drag and S.pitch == "VNAV")
  note("RESET MCP ALT", vm.reset_mcp and S.pitch == "VNAV")
  note("DISCONTINUITY - LNAV", lout.disco and S.roll == "LNAV")
  note("END OF ROUTE", lout.eor and S.roll == "LNAV")
  note("PERF/VNAV UNAVAILABLE", out.msg and out.msg.vnav_unavail or false)
  -- ILS autotune
  local fmsO = fms_entries()
  setILS(fmsO, B747BR_distance_to_dest)
  -- PFD localizer / glideslope scales and pointers: shown when the approach ILS is in use - tuned,
  -- its signal received, and either close to the destination (35 nm) or the approach armed/active.
  -- (a frequency left in the receiver en route no longer puts the G/S diamond on the PFD)
  local appr_act = S.loc_arm or S.gs_arm or S.roll == "LOC" or S.roll == "ROLLOUT" or S.pitch == "GS" or S.pitch == "FLARE"
  local d_dest = B747BR_distance_to_dest or -1
  local near = d_dest >= 0 and d_dest < 35
  local rcv = (simDR_nav1_horizontal == 1) or (simDR_nav1_vertical == 1)
  B747DR_ils_dots = (ils_on and rcv and (near or appr_act or onground)) and 1 or 0
  -- (G/S pointer: airborne, and within 17 nm of the runway or G/S captured)
  -- (v48: and only once the localizer is detected - its deviation alive, within 2.4 dots - or captured)
  local loc_seen = S.roll == "LOC" or S.roll == "ROLLOUT" or S.pitch == "GS" or S.pitch == "FLARE"
    or (simDR_nav1_horizontal == 1 and math.abs(simDR_nav1_hdef or 9) < 2.4)
  B747DR_gs_ptr_show = ((not onground) and loc_seen and (gs_in_range or S.pitch == "GS" or S.pitch == "FLARE")) and 1 or 0
  if ils_on then simDR_hsi_source_pilot = 0; simDR_hsi_source_copilot = 1 end

  do
    local P_ = glue.prof
    recorder(t, { lat = simDR_latitude, lon = simDR_longitude, alt = alt, ra = ra, ias = ias, tas = tas, gs = gs, mach = simDR_airspeed_mach,
      vs = vs, pitch = simDR_AHARS_pitch_deg_pilot, roll = simDR_AHARS_roll_deg_pilot, trk = trk_true, fr = S.roll, fp = S.pitch, fat = S.at,
      ph = vout.phase, sub = vout.sub, vat = vout.at, spd = out.spd_tgt, pa = vout.path_dev and (alt - vout.path_dev) or nil, dev = vout.path_dev,
      s_now = lout.s_now, s_tod = P_ and P_.s_tod, ed_s = P_ and P_.ed_s, ed_h = P_ and P_.ed_h,
      leg = glue.plan and glue.plan.legs[glue.plan.active] and glue.plan.legs[glue.plan.active].id, xtk = lout.xtk, turn = lout.turn,
      rc = out.roll_cmd, pc = out.pitch_cmd, lev = thr, sb = simDR_speedbrake_ratio, flaps = fl, gear = simDR_gear_handle,
      wind = wind_comp_f, mcp_alt = B747DR_autopilot_altitude_ft, mcp_spd = mcp_spd, crz = crz, drag = vout.msg and vout.msg.drag,
      ap = AF.n_ap(S), gsang = gs_ang_geo, vdef = simDR_nav1_vdef, hdef = simDR_nav1_hdef })
  end
  button_animation()
  B747DR_afds_debug = string.format("%s %s r=%s/%s p=%s/%s at=%s st=%s lnav=%s v=%s dev=%s spd=%s xtk=%s loc=%s/%s crs=%s",
    tostring(vout.phase), tostring(vout.sub), S.roll, tostring(S.lnav_arm), S.pitch, tostring(S.vnav_arm), S.at, S.status,
    tostring(lout.valid), tostring(vout.valid), vout.path_dev and string.format("%.0f", vout.path_dev) or "-",
    vout.spd and string.format("%.0f", vout.spd) or "-", lout.xtk and string.format("%.2f", lout.xtk) or "-",
    tostring(last_nav and last_nav.loc_valid), tostring(last_nav and last_nav.gs_valid),
    last_nav and string.format("%.0f", last_nav.crs or -1) or "-")
end



-- legacy target helpers used by other systems (ACARS, AI, FMC pages) ---------------------------
local last_target_t = 0
local function targets()
  if simDRTime - last_target_t < 0.05 then return end
  last_target_t = simDRTime
  if B747DR_ap_target_heading_deg ~= -1 then
    local tgt = math.floor(B747DR_ap_target_heading_deg)
    if tgt < 0 or tgt > 360 then B747DR_ap_target_heading_deg = -1
    elseif B747DR_ap_heading_deg == tgt % 360 then B747DR_ap_target_heading_deg = -1
    else B747DR_ap_heading_deg = tgt % 360 end
  end
  if B747DR_ap_target_altitude ~= -1 then
    if B747DR_ap_target_altitude >= 100 and B747DR_ap_target_altitude <= 50000 then
      B747DR_autopilot_altitude_ft = math.floor(B747DR_ap_target_altitude / 100 + 0.5) * 100
    end
    B747DR_ap_target_altitude = -1
  end
  if B747DR_ap_target_speed ~= -1 then
    if B747DR_ap_target_speed >= 100 and B747DR_ap_target_speed <= 350 then
      B747DR_ap_ias_dial_value = math.floor(B747DR_ap_target_speed)
    end
    B747DR_ap_target_speed = -1
  end
  if B747DR_ap_target_vertspeed ~= -1 then
    if B747DR_ap_target_vertspeed >= -8000 and B747DR_ap_target_vertspeed <= 8000 then
      if S.pitch ~= "VS" then AF.event(S, "VS") end
      simDR_autopilot_vs_fpm = B747DR_ap_target_vertspeed
      S.vs_target = B747DR_ap_target_vertspeed
    end
    B747DR_ap_target_vertspeed = -1
  end
end

--*************************************************************************************--
--**                                 EVENT CALLBACKS                                  **--
--*************************************************************************************--
local initialised = false
function flight_start()
  S = AF.new()
  glue = GL.new()
  fms_sig = ""
  simDR_override_fms_progress = 1          -- the FMC sequences the route, not X-Plane
  pcall(function() simCMD_autopilot_servos_off:once() end)
  B747DR_ap_target_heading_deg = -1
  B747DR_ap_target_altitude = -1
  B747DR_ap_target_speed = -1
  B747DR_ap_target_vertspeed = -1
  B747DR_ap_vnav_system = 2
  if B747DR_ap_ias_dial_value < 100 then B747DR_ap_ias_dial_value = 200 end
  if B747DR_autopilot_altitude_ft < 100 then B747DR_autopilot_altitude_ft = 10000 end
  simDR_disabled_autopilot_altitude_ft = 150000
  B747BR_vnavProfile = "[]"
  last_toga_count = nil
  initialised = true
  safe_print("rebuilt AFDS loaded")
end

local err_count = 0
-- ND route (drawn by the b744_nd plugin): the LNAV path from the active leg on, as text -
-- "L,lat,lon,lat,lon" straight pieces, "A,clat,clon,r,a0,sweep,dir" arcs, "B" discontinuity
local nd_geom_t = -10
local nd_geom_err = nil
local function nd_publish_route()
  if simDRTime - nd_geom_t < 1.0 then return end
  nd_geom_t = simDRTime
  local p = glue.plan
  if not p or not p.legs then B747DR_nd_route_geom = "S,noplan" return end
  local f6 = function(v) return string.format("%.6f", v) end
  local ne = 0
  for i = 1, #p.legs do ne = ne + (p.legs[i].elems and #p.legs[i].elems or 0) end
  -- status token first (diagnostics, ignored by the drawing)
  -- v43: vectors legs (FM / VM - heading legs into a discontinuity) are not drawn on the ND, and once
  -- LNAV is on one (or at the discontinuity) the ND's active waypoint is the next fix after it
  -- (e.g. RIVET5 NASHO -> (VECTORS) -> disco -> SOSIJ: SOSIJ). ND only - LNAV / the FMC are unchanged.
  local function vec_leg(l, i)
    if not (l and (l.t == "FM" or l.t == "VM") and l.seg ~= "SID") then return false end
    -- (only a vectors leg that ends in a discontinuity - the arrival / approach case)
    if not i then for k = 1, #p.legs do if p.legs[k] == l then i = k break end end end
    local nx = i and p.legs[i + 1]
    return nx ~= nil and nx.t == "DISCO"
  end
  local nd_act = p.active or 1
  do
    local al = p.legs[nd_act]
    if al and (vec_leg(al) or al.t == "DISCO") then
      for j = nd_act + 1, #p.legs do
        local l = p.legs[j]
        if l.seg == "MISSED" then break end
        if l.t ~= "DISCO" and not vec_leg(l) and (l.fix or l.pe) then nd_act = j break end
      end
    end
  end
  -- (5th field: the leg LNAV is really on - the FMC shows "(VECTOR)" in the ND header for a vectors leg)
  local out = { string.format("S,%d,%s,%d,%s", #p.legs, tostring(nd_act), ne, tostring(p.active or 1)) }
  -- waypoint list (all legs): "W,index,id,type,lat,lon,elems" (used for diagnostics now, symbols later)
  for i = 1, math.min(#p.legs, 300) do
    local l = p.legs[i]
    local fx = l.fix or l.pe          -- conditional legs ((2400) etc.): the predicted end point
    if vec_leg(l, i) then fx = nil end   -- (no symbol / label for a vectors leg)
    if fx or not vec_leg(l, i) then out[#out + 1] = string.format("W,%d,%s,%s,%s,%s,%d", i, (tostring(l.id or ""):gsub("[,;]", "")), tostring(l.t or ""),
      fx and f6(fx.lat) or "", fx and f6(fx.lon) or "", l.elems and #l.elems or 0) end
  end
  -- T/C and T/D (only when still ahead)
  local P, lo_ = glue.prof, glue.lout
  local s_now = lo_ and lo_.s_now
  if P and P.ok and s_now then
    -- (a short flight that cannot reach the cruise level has T/C and T/D at the same point: both shown)
    if P.s_toc and P.s_toc > s_now + 0.1 and P.s_toc <= (P.s_tod or 1e9) + 0.01 then
      local la, lo = point_at(p, P.s_toc)
      if la then out[#out + 1] = "T,TC," .. f6(la) .. "," .. f6(lo) end
    end
    if P.s_tod and P.s_tod > s_now + 0.1 then
      local la, lo = point_at(p, P.s_tod)
      if la then out[#out + 1] = "T,TD," .. f6(la) .. "," .. f6(lo) end
    end
  end
  -- v48 (ND display only - LNAV's path is unchanged): a corner between two straight pieces (e.g. the
  -- ROVER4B (700) -> direct ARAKA turn) is drawn as a turn of the radius the aircraft flies
  local pendL = nil
  local function emitL(a1, o1, a2, o2) out[#out + 1] = "L," .. f6(a1) .. "," .. f6(o1) .. "," .. f6(a2) .. "," .. f6(o2) end
  local function flushL() if pendL then emitL(pendL[1], pendL[2], pendL[3], pendL[4]); pendL = nil end end
  local gs_nd = math.max(180, (simDR_groundspeed or 0) * 1.94384)
  local R_nd = math.max(0.8, math.min(4.0, gs_nd * gs_nd / (11.26 * 0.4663 * 6076.12)))
  local function addL(a1, o1, a2, o2)
    if pendL and math.abs(pendL[3] - a1) < 1e-5 and math.abs(pendL[4] - o1) < 1e-5 then
      local cl = math.cos(math.rad(a1))
      local function xy(la, lo) return (lo - o1) * 60 * cl, (la - a1) * 60 end
      local x0, y0 = xy(pendL[1], pendL[2])
      local x2, y2 = xy(a2, o2)
      local l1, l2 = math.sqrt(x0 * x0 + y0 * y0), math.sqrt(x2 * x2 + y2 * y2)
      if l1 > 0.05 and l2 > 0.05 then
        local u1x, u1y = -x0 / l1, -y0 / l1          -- incoming direction (towards the corner)
        local u2x, u2y = x2 / l2, y2 / l2            -- outgoing direction
        local cross = u1x * u2y - u1y * u2x
        local dot = u1x * u2x + u1y * u2y
        local dlt = math.atan2(cross, dot)           -- + left (counter-clockwise), - right
        local ad = math.abs(dlt)
        if ad > math.rad(3) and ad < math.rad(150) then
          local R = R_nd
          local d = R * math.tan(ad / 2)
          local function ll0(x, y) return a1 + y / 60, o1 + x / (60 * cl) end
          if d > 0.45 * l1 and l2 > 2.5 * R then
            -- short piece before the corner (a climb-to-altitude leg ending at (700), then direct to a
            -- fix): drawn as flown - the turn starts at the corner and rolls out on a line to the fix
            local right = dlt < 0
            local nx, ny = u1y, -u1x
            if not right then nx, ny = -nx, -ny end
            local cx, cy = nx * R, ny * R
            local bx, by = x2 - cx, y2 - cy
            local D = math.sqrt(bx * bx + by * by)
            if D > R * 1.05 then
              local beta = math.deg(math.atan2(bx, by))
              local phi = math.deg(math.atan(math.sqrt(D * D - R * R) / R))
              local th = right and (beta - phi) or (beta + phi)
              local th0 = math.deg(math.atan2(-cx, -cy))
              local sw = right and ((th - th0) % 360) or ((th0 - th) % 360)
              if sw > 1 and sw < 200 then
                local tx, ty = cx + R * math.sin(math.rad(th)), cy + R * math.cos(math.rad(th))
                local cla, clo = ll0(cx, cy)
                local tla, tlo = ll0(tx, ty)
                emitL(pendL[1], pendL[2], a1, o1)
                out[#out + 1] = "A," .. f6(cla) .. "," .. f6(clo) .. "," .. string.format("%.4f,%.3f,%.3f,%d", R, th0 % 360, sw, right and 1 or -1)
                pendL = { tla, tlo, a2, o2 }
                return
              end
            end
          end
          local dmax = 0.45 * math.min(l1, l2)
          if d > dmax then d = dmax; R = d / math.tan(ad / 2) end
          local t1x, t1y = -u1x * d, -u1y * d
          local t2x, t2y = u2x * d, u2y * d
          local right = dlt < 0
          local nx, ny = u1y, -u1x                   -- right of the incoming direction
          if not right then nx, ny = -nx, -ny end
          local cx, cy = t1x + nx * R, t1y + ny * R
          local function ll(x, y) return a1 + y / 60, o1 + x / (60 * cl) end
          local t1la, t1lo = ll(t1x, t1y)
          local t2la, t2lo = ll(t2x, t2y)
          local cla, clo = ll(cx, cy)
          local a0 = math.deg(math.atan2(t1x - cx, t1y - cy)) % 360
          emitL(pendL[1], pendL[2], t1la, t1lo)
          out[#out + 1] = "A," .. f6(cla) .. "," .. f6(clo) .. "," .. string.format("%.4f,%.3f,%.3f,%d", R, a0, math.deg(ad), right and 1 or -1)
          pendL = { t2la, t2lo, a2, o2 }
          return
        end
      end
    end
    flushL()
    pendL = { a1, o1, a2, o2 }
  end
  for i = math.max(1, p.active or 1), #p.legs do
    local l = p.legs[i]
    if l.t == "DISCO" or vec_leg(l, i) then flushL(); out[#out + 1] = "B"
    elseif l.seg == "MISSED" then break
    elseif l.elems then
      for _, el in ipairs(l.elems) do
        if (el.k == "L" or el.k == "H") and el.a and el.b then
          addL(el.a.lat, el.a.lon, el.b.lat, el.b.lon)
        elseif el.k == "A" and el.c then
          flushL()
          out[#out + 1] = "A," .. f6(el.c.lat) .. "," .. f6(el.c.lon) .. "," .. string.format("%.4f,%.3f,%.3f,%d", el.r or 0, el.a0 or 0, el.sweep or 0, (el.dir or 1) < 0 and -1 or 1)
        end
      end
    end
    if #out > 1500 then break end
  end
  flushL()
  B747DR_nd_route_geom = table.concat(out, ";")
  -- LEGS page predictions: the VNAV profile's altitude and speed at each waypoint still ahead
  local pp = {}
  if P and P.ok and P.legs then
    for i = math.max(1, p.active or 1), #p.legs do
      local l, pr = p.legs[i], P.legs[i]
      local fx = l.fix or l.pe
      if fx and pr and pr.alt and l.t ~= "DISCO" and l.seg ~= "MISSED" then
        -- (a climb-to-altitude leg ends at its altitude)
        if (l.t == "CA" or l.t == "VA" or l.t == "FA") and l.alt and l.alt.a then pr = { alt = l.alt.a, cas = pr.cas, mach = pr.mach } end
        local spd = pr.mach and math.floor(pr.mach * 1000 + 0.5) or (pr.cas and math.floor(pr.cas + 0.5)) or 0
        -- (time from now in s and distance to go along the path in nm: ND / PROGRESS estimates)
        local dtg = (l.cum and s_now) and math.max(0, l.cum - s_now) or -1
        pp[#pp + 1] = string.format("%.4f,%.4f,%d,%d,%d,%.1f", fx.lat, fx.lon, math.floor(pr.alt + 0.5), spd,
          math.floor((P.legs[i] and P.legs[i].t) or -1), dtg)
      end
      if #pp > 300 then break end
    end
  end
  B747DR_legs_pred = table.concat(pp, ";")
end

-- v46: EICAS advisory SET BARO STD - in the climb / cruise above the transition altitude (FMC, PERF /
-- CLB page, default 18000 ft) with the captain's altimeter not on STD; clears once STD is selected
B747DR_CAS_advisory_status_ap = find_dataref("laminar/B747/CAS/advisory_status")
B747DR_baro_std_capt_ap = find_dataref("laminar/B747/efis/baro_std/capt/switch_pos")
local baro_msg = 0
local function baro_std_advisory()
  local ta = num(fmsdata().transalt) or 18000
  local alt = simDR_pressureAlt1 or 0
  local std = (B747DR_baro_std_capt_ap or 0) == 1
  local ph = glue.vst and glue.vst.phase or ""
  local on = 0
  if not std and (simDR_onGround or 0) == 0 and ph ~= "DES" and alt > ta + 200 then on = 1 end
  if on ~= baro_msg then
    baro_msg = on
    B747DR_CAS_advisory_status_ap[304] = on
  end
end

function after_physics()
  if not initialised then flight_start() end
  pcall(baro_std_advisory)
  local ok, e = pcall(core)
  if not ok then
    err_count = err_count + 1
    if err_count < 20 or err_count % 500 == 0 then safe_print("ERROR " .. tostring(e)) end
    B747DR_afds_cmd_valid = 0
  end
  targets()
  local okg, eg = pcall(nd_publish_route)
  if not okg then
    nd_geom_err = "E," .. string.gsub(tostring(eg), "[;\n]", " "):sub(1, 200)
    B747DR_nd_route_geom = nd_geom_err
  end
end
