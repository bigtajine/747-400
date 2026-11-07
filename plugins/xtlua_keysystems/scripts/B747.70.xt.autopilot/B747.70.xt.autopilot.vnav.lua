--[[ legacy VNAV helpers kept for speedctl/vnavspd: deceleratedDesent(), getDescentTarget().
     The old setDescentVSpeed() (per-frame, path math tangled with mode flags) is replaced by ap_vnav.lua. ]]
--[[
*****************************************************************************************
* Program Script Name	:	B747.70.autopilot.vnav
* Author Name			:	Mark Parker (mSparks)
*
*   Revisions:
*   -- DATE --	--- REV NO ---		--- DESCRIPTION ---
*   2021-01-27	0.01a				Start of Dev
*
*
*
*
--]]
dofile("B747.70.xt.autopilot.speedctl.lua")
dofile("B747.70.xt.autopilot.vnavspd.lua")

function deceleratedDesent(targetvspeed)
  if simDR_autopilot_airspeed_is_mach == 1 then return targetvspeed end --can't do this in mach mode, slow tf down already

  local meet = B747_rescale(0,0,400,500,B747BR_fpe)
  local upperAlt=math.max(tonumber(getFMSData("desspdtransalt")),tonumber(getFMSData("desrestalt")))
  if simDR_pressureAlt1>upperAlt+1000 then 
    return targetvspeed -meet
  end --nowhere near a restriction yet
  local lowerAlt=math.min(tonumber(getFMSData("desspdtransalt")),tonumber(getFMSData("desrestalt")))
  local upperAltspdval=tonumber(getFMSData("destranspd"))
  local lowerAltspdval=tonumber(getFMSData("desrestspd"))

  if simDR_ind_airspeed_kts_pilot<=(lowerAltspdval+5) then return targetvspeed end --already low enough
  --less than upperAlt+1000
   -- greater than lowerAltspdval
  if simDR_ind_airspeed_kts_pilot>(upperAltspdval+5) then 
    --print("upperAltspdval deceleratedDesent upperAlt"..upperAlt.." lowerAlt=".. lowerAlt .." upperAltspdval=".. upperAltspdval .." simDR_pressureAlt1="..simDR_pressureAlt1.." simDR_ind_airspeed_kts_pilot="..simDR_ind_airspeed_kts_pilot)
    return -500 
  end --approximate 500fpm

  if simDR_pressureAlt1>lowerAlt+1000 then return targetvspeed end --not at next restriction yet

  if simDR_ind_airspeed_kts_pilot>(lowerAltspdval+5) then 
    --print("lowerAltspdval deceleratedDesent upperAlt"..upperAlt.." lowerAlt=".. lowerAlt .." upperAltspdval=".. upperAltspdval .." simDR_pressureAlt1="..simDR_pressureAlt1.." simDR_ind_airspeed_kts_pilot="..simDR_ind_airspeed_kts_pilot)
    return -500 
  end --approximate 500fpm

  return targetvspeed-meet
  --print("oob deceleratedDesent upperAlt"..upperAlt.." lowerAlt=".. lowerAlt .." upperAltspdval=".. upperAltspdval .." simDR_pressureAlt1="..simDR_pressureAlt1.." simDR_ind_airspeed_kts_pilot="..simDR_ind_airspeed_kts_pilot)


end
-- setDescentVSpeed() removed: the path V/S is now computed by ap_vnav.lua (VN.step -> vs_cmd).

  
  function getDescentTarget()
    B747DR_target_descentSpeed=tonumber(getFMSData("destranspd"))
    B747DR_target_descentAlt=tonumber(getFMSData("desspdtransalt"))
    if B747DR_target_descentAlt>simDR_pressureAlt1 
      or simDR_autopilot_airspeed_kts<B747DR_target_descentSpeed 
      then 
      B747DR_descentSpeedGradient=0 
      return 
    end
    B747DR_descentSpeedGradient=(simDR_autopilot_airspeed_kts-B747DR_target_descentSpeed)/(simDR_pressureAlt1-B747DR_target_descentAlt)
    print("set descentSpeedGradient to " .. B747DR_descentSpeedGradient)
  end
  