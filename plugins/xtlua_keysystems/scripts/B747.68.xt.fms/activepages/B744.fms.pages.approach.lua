-- Very minor modification by crazytimtimtim
-- v48: 744 APPROACH REF layout - VREF for flaps 25 / 30 (unchanged values), and after an arrival
-- runway is selected: the destination ILS (from the navigation database) - runway length, ILS
-- frequency / front course (PARK when the NAV receiver is not tuned to it) - FLAP/SPD and G/S.
simDR_acf_weight_total_kg          				= find_dataref("sim/flightmodel/weight/m_total")
B747DR_airspeed_Vf25                            = find_dataref("laminar/B747/airspeed/Vf25")
B747DR_airspeed_Vf30                            = find_dataref("laminar/B747/airspeed/Vf30")
B747DR_appr_nav1_hz                             = find_dataref("sim/cockpit/radios/nav1_freq_hz")

local appr_cache = { t = -100, key = "", info = nil }
-- destination ICAO and arrival runway from X-Plane's FMS list (the runway entry before the airport)
local function appr_arrival()
  if type(fmsJSON) ~= "string" or string.len(fmsJSON) < 3 then return nil end
  local ok, fms = pcall(function() return json.decode(fmsJSON) end)
  if not ok or type(fms) ~= "table" or #fms < 2 then return nil end
  local dst = fms[#fms]
  if not dst or dst[2] ~= 1 or type(dst[8]) ~= "string" then return nil end
  local icao = dst[8]
  for i = #fms - 1, math.max(1, #fms - 25), -1 do
    local id = fms[i] and fms[i][8]
    if type(id) == "string" then
      local rw = id:match("^RW(%d%d[LRC]?)%s*$")
      if rw then return icao, rw end
    end
  end
  return icao, nil
end
local function appr_info()
  local icao, rw = appr_arrival()
  if not icao or not rw then return nil end
  local key = icao .. rw
  if key == appr_cache.key and appr_cache.info then return appr_cache.info end
  local N = B744NAVDB
  if not N then return nil end
  local okI, ils = pcall(N.ils, icao, rw)
  local info = { icao = icao, rw = rw }
  if okI and ils then info.freq = ils.freq; info.crs = ils.crs_mag end
  local okP, pr = pcall(N.procedures, icao)
  local r = okP and pr and pr.rwys and (pr.rwys["RW" .. rw] or pr.rwys["RW" .. rw .. " "])
  if r and r.length_ft then info.len = r.length_ft end
  appr_cache.key = key; appr_cache.info = info
  return info
end
local function lr24(l, r)
  l, r = l or "", r or ""
  return l .. string.rep(" ", math.max(1, 24 - #l - #r)) .. r
end
local appr_small = nil

fmsPages["APPROACH"]=createPage("APPROACH")
fmsPages["APPROACH"].getPage=function(self,pgNo,fmsID)--dynamic pages need to be this way
  local acf_weight = simDR_acf_weight_total_kg
  if simConfigData["data"].SIM.weight_display_units == "LBS" then
	acf_weight = simDR_acf_weight_total_kg * simConfigData["data"].SIM.kgs_to_lbs
  end
  local okI, info = pcall(appr_info)
  if not okI then info = nil end
  local blank = "                        "
  local l7, l9, s6, s8 = blank, blank, blank, " "
  if info then
    s6 = " " .. info.icao .. "ILS " .. info.rw
    if info.len then l7 = string.format("%dFT %dM", math.floor(info.len + 0.5), math.floor(info.len * 0.3048 + 0.5)) end
    s8 = " ILS" .. info.rw .. "/CRS"
    if info.freq and info.crs then
      local f = info.freq > 1000 and info.freq / 100 or info.freq
      local park = (math.abs((tonumber(B747DR_appr_nav1_hz) or 0) - (info.freq > 1000 and info.freq or info.freq * 100)) > 1) and " PARK" or ""
      l9 = string.format("%6.2f/%03d`", f, math.floor(info.crs + 0.5) % 360) .. park
    else
      l9 = "---.--/---`"
    end
  end
  local fs = fmsModules["data"]["flapspeed"] or "**/***"
  appr_small = { s6 = s6, s8 = s8 }
  return {
"      APPROACH REF      ",
"                        ",
string.format("%5.1f", acf_weight/1000) .. "        25`     " .. string.format("%3d", B747DR_airspeed_Vf25),
"                        ",
"             30`     " .. string.format("%3d", B747DR_airspeed_Vf30),
"                        ",
lr24(l7 ~= blank and l7 or "", ""),
"                        ",
lr24(l9 ~= blank and l9 or "", fs),
"                        ",
"<ON                     ",
"------------------------",
"<INDEX       THRUST LIM>"
  }
end

fmsPages["APPROACH"].getSmallPage=function(self,pgNo,fmsID)
  local s6 = (appr_small and appr_small.s6) or ""
  local s8 = (appr_small and appr_small.s8) or ""
  return{
"                        ",
" GROSS WT   FLAPS   VREF",
"                        ",
"                        ",
"                        ",
lr24(s6, ""),
"                        ",
lr24(s8, "FLAP/SPD"),
"                        ",
" G/S                    ",
"   <->OFF               ",
"------------------------",
"                        "
  }
end

fmsFunctionsDefs["APPROACH"]={}
fmsFunctionsDefs["APPROACH"]["L6"]={"setpage","INITREF"}
fmsFunctionsDefs["APPROACH"]["R6"]={"setpage","THRUSTLIM"}
fmsFunctionsDefs["APPROACH"]["L1"]={"setdata","grosswt"}
fmsFunctionsDefs["APPROACH"]["R1"]={"setdata","vref1"}
fmsFunctionsDefs["APPROACH"]["R2"]={"setdata","vref2"}
fmsFunctionsDefs["APPROACH"]["R4"]={"setdata","flapspeed"}
