--[[
 ap_ils.lua -- ILS auto-tune / geometry helpers.
 Extracted UNCHANGED from the stock B747.70.xt.autopilot.lua (original authors: Jim Gregory,
 Mark Parker "mSparks", Marauder28; CC BY-NC 4.0).  The logic here is radio tuning, not autopilot
 mode logic, so it is intentionally not rewritten.  It calls setDistances(fms), which the new
 main script provides as a thin shim over the new VNAV module.
--]]
dofile("json/json.lua")
simDR_variation = find_dataref("sim/flightmodel/position/magnetic_variation")
simDR_nav1Freq = find_dataref("sim/cockpit/radios/nav1_freq_hz")
simDR_nav2Freq = find_dataref("sim/cockpit/radios/nav2_freq_hz")
--simDR_nav1crs=find_dataref("sim/cockpit/radios/nav1_course_degm")
--simDR_nav2crs=find_dataref("sim/cockpit/radios/nav2_course_degm")
simDR_radio_nav_obs_deg = find_dataref("sim/cockpit2/radios/actuators/nav_obs_deg_mag_pilot")
simDR_radio_nav1_obs_deg = find_dataref("sim/cockpit/radios/nav1_obs_degt")
simDR_radio_nav2_obs_deg = find_dataref("sim/cockpit/radios/nav2_obs_degt")
simDR_radio_nav1_bearing_deg = find_dataref("sim/cockpit2/radios/indicators/nav1_bearing_deg_mag")
simDR_radio_nav2_bearing_deg = find_dataref("sim/cockpit2/radios/indicators/nav2_bearing_deg_mag")

navAidsJSON = find_dataref("xtlua/navaids")
fmsJSON = find_dataref("xtlua/fms")
nSize = 0
local navAids = {}
targetILS = find_dataref("laminar/B747/radio/ilsData")
local targetILSS = ""
local targetFMS = ""
local targetFMSnum = -1
targetFix = 0
bestDiff = 180
function getHeading(lat1, lon1, lat2, lon2)
	b10 = math.rad(lat1)
	b11 = math.rad(lon1)
	b12 = math.rad(lat2)
	b13 = math.rad(lon2)
	retVal =
		math.atan2(
		math.sin(b13 - b11) * math.cos(b12),
		math.cos(b10) * math.sin(b12) - math.sin(b10) * math.cos(b12) * math.cos(b13 - b11)
	)
	return math.deg(retVal)
end
function getHeadingDifference(desireddirection, current_heading)
	error = current_heading - desireddirection
	if (error > 180) then
		error = error - 360
	end
	if (error < -180) then
		error = error + 360
	end
	return error
end

function getDistance(lat1, lon1, lat2, lon2)
	if lat1 == lat2 and lon1 == lon2 then
		return 0
	end
	alat = math.rad(lat1)
	alon = math.rad(lon1)
	blat = math.rad(lat2)
	blon = math.rad(lon2)
	av = math.sin(alat) * math.sin(blat) + math.cos(alat) * math.cos(blat) * math.cos(blon - alon)
	if av > 1 then
		av = 1
	end
	retVal = math.acos(av) * 3440
	return retVal
end

function movePoint(lat1,lon1,distanceInNM,bearing)
	local brngRad = math.rad(bearing)
	local latRad = math.rad(lat1)
	local lonRad = math.rad(lon1)
	local distFrac = distanceInNM / 3440;
	local latitudeResult = math.asin(math.sin(latRad) * math.cos(distFrac) + math.cos(latRad) * math.sin(distFrac) * math.cos(brngRad))
	local a = math.atan2(math.sin(brngRad) * math.sin(distFrac) * math.cos(latRad), math.cos(distFrac) - math.sin(latRad) * math.sin(latitudeResult))
	local longitudeResult = (lonRad + a + 3 * math.pi) % (2 * math.pi) - math.pi;
	lat2 = math.deg(latitudeResult)
    lon2 = math.deg(longitudeResult)

    return lat2, lon2							   
end

local lastILSUpdate = 0
original_distance = -1
function round(x)
	return x >= 0 and math.floor(x + 0.5) or math.ceil(x - 0.5)
end
function int(x)
	return x >= 0 and math.floor(x) or math.ceil(x)
end
function B747_fltmgmt_setILS(fmsO)
	local modes = B747DR_radioModes
	local targetDots=0
	--print("B747_fltmgmt_setILS")
	if modes:sub(1, 1) == " " then
		targetFMSnum = -1
		B747DR_radioModes = replace_char(1, modes, "A")
	elseif modes:sub(1, 1) == "M" then
		--local fms=json.decode(fmsSTR)

		setDistances(fmsO)
		--print("Dist B747DR_ils_dots = "..B747DR_ils_dots)
		--print("Dist targetILSS = "..targetILS)
		--print("Dist B747BR_distance_to_dest = "..B747BR_distance_to_dest)
		if B747DR_ils_dots==0 and ((B747BR_totalDistance>0 and B747BR_totalDistance< 50) or (B747BR_distance_to_dest>=0 and B747BR_distance_to_dest<50)) then
			if string.len(targetILS) > 2 then
				B747DR_ils_dots=1
				--print("do Dist B747DR_ils_dots = "..B747DR_ils_dots)
			end
		end
		return
	end
	local diff = simDRTime - lastILSUpdate
	if diff < 2 then
		return
	end
	--collectgarbage("collect")
	lastILSUpdate = simDRTime
	local n1 = simDR_nav1Freq
	local n2 = simDR_nav2Freq
	local d1 = simDR_radio_nav1_obs_deg
	local d2 = simDR_radio_nav2_obs_deg
	 --continually get latest
	local d1 = simDR_radio_nav_obs_deg[0]
	local d2 = simDR_radio_nav_obs_deg[1]
	 --continually get latest
	--print("fmsJSON=".. fmsSTR)


	setDistances(fmsO)

	local newTargetFix = 0
	local hitI = -1
	--print(navAidsJSON)
	if table.getn(fmsO) > 4 and (fmsO[targetFMSnum] == nil or targetFMS ~= fmsO[targetFMSnum][8]) then
		if string.len(navAidsJSON) ~= nSize then
			navAids = json.decode(navAidsJSON)
			nSize = string.len(navAidsJSON)
		end
		if fmsO[table.getn(fmsO)][2] == 1 then
			--we have an airport as our dst
			local apdistance = getDistance(simDR_latitude, simDR_longitude, fmsO[table.getn(fmsO)][5], fmsO[table.getn(fmsO)][6])
			if apdistance > 45 then
				return
			end

			--Marauder28
			if original_distance == -1 then
				original_distance = B747BR_totalDistance --capture original flightplan distance
			end
			local pct_remaining = 100 - (((original_distance - B747BR_totalDistance) / original_distance) * 100)
			local dist_to_TOD = B747BR_totalDistance - B747BR_tod
			--print("Orig Dist = "..original_distance)
			--print("Dist Remain = "..B747BR_totalDistance)
			--if dist_to_TOD > 0 then print("Dist TOD = "..dist_to_TOD) end
			--print("Pct Remain = "..pct_remaining)
			--if (apdistance>=200) or (pct_remaining > 50) then
			--if dist_to_TOD >= 200 then --or pct_remaining > 50 then
			--return    --more than 200nm away from TOD or less than half-way to DEST (used in NAV RAD auto-tune scenario #2)
			--end
			--Marauder28

			found = false
			local runwayHeading = -999
			for i = table.getn(fmsO) - 1, 2, -1 do --last is the airport, before that go-around [may] be dup
				--we have a fix coming in to the airport
				--if fms[i][2] == 512 then
				local ap1Heading = getHeading(fmsO[i][5], fmsO[i][6], fmsO[table.getn(fmsO)][5], fmsO[table.getn(fmsO)][6])
				local ap2Heading = getHeading(fmsO[i - 1][5], fmsO[i - 1][6], fmsO[table.getn(fmsO)][5], fmsO[table.getn(fmsO)][6])
				local diffap = getHeadingDifference(ap1Heading, ap2Heading)
				local distance = getDistance(fmsO[i][5], fmsO[i][6], fmsO[table.getn(fmsO)][5], fmsO[table.getn(fmsO)][6])
				print("finding ils FMS i=" .. i .. ":" .. ap1Heading .. ":" .. ap2Heading .. ":" .. diffap .. ":" .. distance.. ":" .. fmsO[i][8].. ":" .. fmsO[i - 1][8] )
				if diffap < 10 and diffap > -10 and distance < 11 then
					print("potential matched from" .. i)
					for n = table.getn(navAids), 1, -1 do
						--now find an ils

						if navAids[n][2] == 8 then
							local headingToILS = getHeading(fmsO[i][5], fmsO[i][6], navAids[n][5], navAids[n][6])
							local headingToNext = getHeading(fmsO[i][5], fmsO[i][6], fmsO[i + 1][5], fmsO[i + 1][6])
							local diff = getHeadingDifference(navAids[n][4], headingToILS)
							local diff2 = getHeadingDifference(headingToILS, headingToNext)
							if diff < 0 then
								diff = diff * -1
							end

							if diff2 < 5 and diff2 > -5 then
								local distance2 = getDistance(fmsO[table.getn(fmsO)][5], fmsO[table.getn(fmsO)][6], navAids[n][5], navAids[n][6])
								if diff < 1 and diff <= bestDiff and distance2 < 10 then
									--print("navaid "..n.."->"..fms[i][8].."="..diff.." ".. navAids[n][1].." ".. navAids[n][2].." ".. navAids[n][3].." ".. navAids[n][4].." ".. navAids[n][5].." ".. navAids[n][6].." ".. navAids[n][7].." ".. navAids[n][8])
									bestDiff = diff
									runwayHeading = headingToNext
									print("bestdiff = " .. bestDiff .. " on heading " .. headingToILS .. " course=" .. headingToNext)
									--if targetFix == newTargetFix then
									found = true
									newTargetFix = n
									print("newTargetFix = " .. newTargetFix)
									hitI = i
								end
							end
						end
					end
				end
			end
			targetFix = newTargetFix
			-- if found==false then
			bestDiff = 180

			--else
			if targetFix ~= 0 then
				--print("set targetILS")
				--navAids[targetFix][4]=runwayHeading
				targetILSS = json.encode(navAids[targetFix])
				targetFMS = fmsO[hitI][8]
				targetFMSnum = hitI
				targetILS = targetILSS
				print("Tune ILS" .. targetILSS)

				if string.len(targetILSS) > 0 then
					print("Tuning ILS" .. targetILSS)
					local ilsNav = json.decode(targetILSS)
					simDR_nav1Freq = ilsNav[3]
					simDR_nav2Freq = ilsNav[3]
					local course = (ilsNav[4] + int(simDR_variation))
					if course < 0 then
						course = course + 360
					end
					simDR_radio_nav_obs_deg[0] = course
					simDR_radio_nav_obs_deg[1] = course
					print("Tuned ILS " .. course)
					targetDots = 1 -- make PFD loc and GS dots visible
				end
			elseif B747DR_ap_FMA_active_roll_mode ~= 3 then
				--print("cleared targetILS")
				targetILS = " "
				targetILSS = " "
			end
		end
	elseif string.len(targetILSS) > 1 and string.len(targetILS) > 2 then
		--print("Tuned ILS "..course)
		--print("Tuning ILS".. targetILSS)
		local ilsNav = json.decode(targetILSS)
		simDR_nav1Freq = ilsNav[3]
		simDR_nav2Freq = ilsNav[3]
		local course = (ilsNav[4] + simDR_variation)
		if course < 0 then
			course = course + 360
		elseif course > 360 then
			course = course - 360
		end
		--course=round(course)
		simDR_radio_nav_obs_deg[0] = course
		simDR_radio_nav_obs_deg[1] = course
		targetDots = 1 -- make PFD loc and GS dots visible
	else
		if B747DR_ap_FMA_active_roll_mode == 3 then
			targetDots = 1 -- make PFD loc and GS dots visible
		elseif string.len(targetILS) < 2 and B747DR_ap_FMA_active_roll_mode ~= 3 then
			targetDots = 0
		end
	end
	local tDots=B747DR_ils_dots
	if (B747BR_totalDistance>0 and B747BR_totalDistance< 50) or (B747BR_distance_to_dest>=0 and B747BR_distance_to_dest<50) then
		B747DR_ils_dots=targetDots
	else
		B747DR_ils_dots=0
	end
	--print("target="..targetILS.."= "..targetILSS.."= "..targetFix.. " "..nSize.. " "..table.getn(navAids))
	--[[for i=table.getn(fms)-1,1,-1 do
    if fms[i][10]==true then
      print("fms"..i.."=".. fms[i][1].." "..fms[i][2].." "..fms[i][3].." "..fms[i][4].." "..fms[i][5].." "..fms[i][6].." "..fms[i][7].." "..fms[i][8].." "..fms[i][9].." active")
    else
      print("fms"..i.."=".. fms[i][1].." "..fms[i][2].." "..fms[i][3].." "..fms[i][4].." "..fms[i][5].." "..fms[i][6].." "..fms[i][7].." "..fms[i][8].." "..fms[i][9].." inactive")
    end
  end]]
end
