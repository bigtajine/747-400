--[[
*****************************************************************************************
* Program Script Name	:	B747.70.autopilot.speedctl
* Purpose		:	VNAV speed-on-pitch improvements for the XP12 747-400 autopilot
*
*   Adds, without replacing any existing interlock:
*     1. FLCH target shaping   - lead (acceleration) compensation plus an overspeed
*                                "rescue" bias applied to the speed target that X-Plane's
*                                FLCH pitch loop sees.  The MCP/FMA/bug values are NOT
*                                changed, only the internal sim target.
*     2. Descent speed protect - replaces the crude "deceleratedDesent" -500fpm hack.
*                                Path VS (VNAV PTH) is limited by an energy-based VS limit
*                                so speed is held at idle thrust (VNAV SPD behaviour).
*     3. Climb speed schedule  - faster refresh and a larger step-down limit, so the
*                                target can catch up with a restriction.
*     4. CSV logger            - writes VNAV_speedctl_log.csv for tuning.
*
*   Every entry point is wrapped in pcall and falls back to the original behaviour on
*   error (5 errors in a session disables the module).  Set SPDCTL_CFG.enabled=false
*   below to restore stock behaviour completely.
*
*   Derived from work (c) 2016 Jim Gregory / Laminar Research and (c) 2020-21 Mark Parker
*   (mSparks), CC-BY-NC 4.0.  Modifications are non-commercial and share the same licence.
*****************************************************************************************
--]]

SPDCTL_CFG = {
	enabled        = true,   -- master switch (false = stock behaviour)
	shapeFLCH      = true,   -- target shaping while in VNAV + FLCH (climb and high descent)
	descentProtect = true,   -- energy-based speed protection in VNAV VS descent
	legacyDecel    = false,  -- true = use original deceleratedDesent instead of protection
	log            = true,   -- CSV logging while VNAV is active
	logPeriod      = 0.5,    -- seconds between log rows

	-- climb speed schedule (vnavspd.lua)
	climbStepDown  = 25,     -- kt: max target reduction below current IAS per update (stock: 15)
	refreshSec     = 10,     -- s: speed schedule re-evaluation interval (stock: 30)
	overspdTrigger = 8,      -- kt above dial that forces an immediate re-evaluation

	-- FLCH target shaping
	minAlt   = 400,          -- ft AGL below which shaping is off
	tLead    = 5.0,          -- s: lead time applied to IAS/Mach acceleration
	kRescue  = 0.8,          -- extra bias per kt above (target + rescueDb)
	rescueDb = 2.0,          -- kt dead band before the rescue bias starts
	kInt     = 0.25,         -- kt/s of extra bias per kt of overspeed (integral, independent of sim FLCH gain)
	riTau    = 2.0,          -- s: the integral bias unwinds with this time constant once speed is back in band
	maxDown  = 30,           -- kt: max amount virtual target is pushed BELOW real target
	maxUp    = 8,            -- kt: max amount virtual target is pushed ABOVE real target
	offTau   = 1.0,          -- s: low-pass on the applied offset (keeps FD bars smooth)
	accTau   = 1.5,          -- s: low-pass on measured acceleration

	-- descent speed protection (VS branch)
	dGain       = 0.10,      -- 1/s: integrator gain (closed-loop speed time constant ~10 s)
	dTau        = 12.0,      -- s: speed error -> desired deceleration time constant
	dAccMin     = -1.2,      -- kt/s: max commanded deceleration
	dAccMax     = 1.0,       -- kt/s: max commanded acceleration
	dLowerBand  = 600,       -- fpm: limit may sit at most this far below the path VS (anti-windup)
	dUpperLimit = 0,         -- fpm: protection never commands a climb in descent
	dRate       = 600,       -- fpm/s: rate limit on the commanded VS (smooth FD bars)
}

local cfg = SPDCTL_CFG
local S = { t = nil, ias = 0, mach = 0, aIas = 0, aMach = 0, off = 0, ri = 0, errors = 0,
            rawKts = nil, rawStamp = -99, tShape = nil, virt = 0 }
local D = { t = nil, lim = 0, cmd = 0 }
local logFile = nil
local logTried = false
local logHeaderDone = false
local lastLog = -99
local lastPrint = -99

local function clamp(x, lo, hi)
	if x < lo then return lo elseif x > hi then return hi else return x end
end

local function num(x)
	return tonumber(x) or 0
end

local function fail(where, err)
	S.errors = S.errors + 1
	print("SPDCTL error in " .. where .. ": " .. tostring(err) .. " (" .. S.errors .. "/5)")
	if S.errors >= 5 then
		cfg.enabled = false
		print("SPDCTL disabled after repeated errors - stock VNAV speed behaviour restored")
	end
end

--------------------------------------------------------------------------------------
-- speed / acceleration estimator (called at most once per 0.1 s of sim time)
--------------------------------------------------------------------------------------
local function update()
	local now = simDRTime
	if S.t == nil or now < S.t or now - S.t > 1.0 then
		S.t = now
		S.ias = simDR_ind_airspeed_kts_pilot
		S.mach = simDR_airspeed_mach
		S.aIas, S.aMach, S.off, S.ri = 0, 0, 0, 0
		return
	end
	local dt = now - S.t
	if dt < 0.1 then return end
	local ias = simDR_ind_airspeed_kts_pilot
	local mach = simDR_airspeed_mach
	local k = 1 - math.exp(-dt / cfg.accTau)
	S.aIas = S.aIas + (((ias - S.ias) / dt) - S.aIas) * k
	S.aMach = S.aMach + (((mach - S.mach) / dt) - S.aMach) * k
	S.ias, S.mach, S.t = ias, mach, now
end

local function soundKts()
	local tas = simDR_TAS_mps * 1.94384
	if S.mach > 0.1 then return tas / S.mach end
	return 600
end

--------------------------------------------------------------------------------------
-- 1. FLCH target shaping
--------------------------------------------------------------------------------------
local function shapingActive()
	return cfg.enabled and cfg.shapeFLCH
		and B747DR_ap_vnav_state > 0
		and simDR_autopilot_flch_status > 0
		and simDR_onGround == 0
		and simDR_radarAlt1 > cfg.minAlt
		and B747DR_autopilot_gs_status < 1
end

-- returns the offset in kt to add to the real target (negative = pitch up sooner)
local function computeOffset(errKts, accKts)
	local now = simDRTime
	local dtf = now - (S.tShape or now)
	S.tShape = now
	dtf = clamp(dtf, 0, 0.5)

	local want = -cfg.tLead * accKts
	if errKts > cfg.rescueDb then
		want = want - cfg.kRescue * (errKts - cfg.rescueDb)
		S.ri = S.ri - cfg.kInt * (errKts - cfg.rescueDb) * dtf      -- wind up while overspeeding
	else
		S.ri = S.ri * math.exp(-dtf / cfg.riTau)                    -- unwind once back in band
	end
	S.ri = clamp(S.ri, -cfg.maxDown, 0)
	want = clamp(want + S.ri, -cfg.maxDown, cfg.maxUp)
	S.off = S.off + (want - S.off) * (1 - math.exp(-dtf / cfg.offTau))
	return S.off
end

local function shapeIAS_impl(raw, minSafe, maxSafe)
	update()
	if not shapingActive() then
		S.off, S.ri = 0, 0
		S.rawKts = nil
		return nil
	end
	local off = computeOffset(S.ias - raw, S.aIas)
	local virt = clamp(raw + off, minSafe, maxSafe)
	S.rawKts = raw
	S.rawStamp = simDRTime
	S.virt = virt
	return virt
end

function SPDCTL_shapeIAS(raw, minSafe, maxSafe)
	if not cfg.enabled then return raw end
	local ok, res = pcall(shapeIAS_impl, raw, minSafe, maxSafe)
	if not ok then
		fail("shapeIAS", res)
		return raw
	end
	return res or raw
end

local function shapeMach_impl(rawMach, maxMach)
	update()
	if not shapingActive() then
		S.off, S.ri = 0, 0
		S.rawKts = nil
		return nil
	end
	local as = soundKts()
	local errKts = (S.mach - rawMach) * as
	local accKts = S.aMach * as
	local offKts = computeOffset(errKts, accKts)
	local virt = clamp(rawMach + offKts / as, 0.40, maxMach)
	S.rawStamp = simDRTime
	S.virt = virt
	return virt
end

function SPDCTL_shapeMach(rawMach, maxMach)
	if not cfg.enabled then return rawMach end
	local ok, res = pcall(shapeMach_impl, rawMach, maxMach)
	if not ok then
		fail("shapeMach", res)
		return rawMach
	end
	return res or rawMach
end

-- The un-shaped speed dial value, for code that reads the sim target back as "the dial".
function SPDCTL_dialKts()
	if S.rawKts ~= nil and (simDRTime - S.rawStamp) < 0.5 then
		return S.rawKts
	end
	return simDR_autopilot_airspeed_kts
end

--------------------------------------------------------------------------------------
-- 3. Climb speed schedule helpers (used by vnavspd.lua)
--------------------------------------------------------------------------------------
function SPDCTL_stepLimit(spdval)
	local step = 15
	if cfg.enabled then step = cfg.climbStepDown end
	return math.max(spdval, simDR_ind_airspeed_kts_pilot - step)
end

function SPDCTL_refreshSec()
	if cfg.enabled then return cfg.refreshSec end
	return 30
end

-- true when IAS is well above the selected speed and the schedule should be re-evaluated
function SPDCTL_overspeedTrigger(lastEval)
	if not cfg.enabled then return false end
	if simDR_autopilot_airspeed_is_mach ~= 0 then return false end
	if simDRTime - lastEval < 3 then return false end
	return simDR_ind_airspeed_kts_pilot > (B747DR_ap_ias_dial_value + cfg.overspdTrigger)
end

--------------------------------------------------------------------------------------
-- 2. Descent speed protection (VNAV PTH limited by VNAV SPD)
--   vsPath : path vertical speed (negative = descending) from setDescentVSpeed
--   returns: vertical speed to put in the VS window
--------------------------------------------------------------------------------------
local function descentVS_impl(vsPath)
	update()
	local now = simDRTime

	-- path intercept aid from the original deceleratedDesent: catch the path from above
	local meet = B747_rescale(0, 0, 400, 500, B747BR_fpe)
	local vp = vsPath - meet

	local dt = now - (D.t or -99)
	if dt > 2.0 or dt <= 0 then
		-- (re)initialise: limit starts out of the way so descent is never blocked
		D.lim = vp - cfg.dLowerBand
		D.cmd = simDR_autopilot_vs_fpm
		D.t = now
		return vp
	end
	dt = clamp(dt, 0, 0.5)

	local tas = simDR_TAS_mps * 1.94384
	local ktas = tas / math.max(S.ias, 60)
	local err, acc
	if simDR_autopilot_airspeed_is_mach == 1 then
		local as = soundKts()
		err = (S.mach - simDR_autopilot_airspeed_kts_mach) * as
		acc = S.aMach * as
	else
		err = S.ias - SPDCTL_dialKts()
		acc = S.aIas
	end

	-- energy: 1 kt/s of TAS acceleration = 5.313*TAS*(TAS/IAS) fpm of climb-rate change at fixed thrust
	local k0 = 5.313 * tas * ktas
	local accDes = clamp(-err / cfg.dTau, cfg.dAccMin, cfg.dAccMax)
	D.lim = D.lim + dt * cfg.dGain * k0 * (acc - accDes)
	D.lim = clamp(D.lim, vp - cfg.dLowerBand, cfg.dUpperLimit)

	local target = math.max(vp, D.lim)          -- protection can only reduce the descent rate
	local maxStep = cfg.dRate * dt
	D.cmd = D.cmd + clamp(target - D.cmd, -maxStep, maxStep)
	D.t = now
	return D.cmd
end

function SPDCTL_descentVS(vsPath, legacyFn)
	if not cfg.enabled or not cfg.descentProtect or cfg.legacyDecel then
		return legacyFn(vsPath)
	end
	local ok, res = pcall(descentVS_impl, vsPath)
	if not ok then
		fail("descentVS", res)
		return legacyFn(vsPath)
	end
	return res
end

--------------------------------------------------------------------------------------
-- 4. Logger + per-frame hook (called from B747_ap_ias_mach_mode)
--------------------------------------------------------------------------------------
local function openLog()
	logTried = true
	local ok, path = pcall(function()
		local dr = find_dataref("sim/aircraft/view/acf_livery_path")
		return tostring(dr)
	end)
	if not ok or path == nil or path == "" or path == "nil" then path = "" end
	local f = io.open(path .. "VNAV_speedctl_log.csv", "a")
	if f ~= nil then
		logFile = f
		print("SPDCTL logging to " .. path .. "VNAV_speedctl_log.csv")
	else
		print("SPDCTL could not open a log file; logging rows to Log.txt with prefix SPDCTL,")
	end
end

local HEADER = "t,vnav_state,in_descent,flch,vs_status,alt_ft,mcp_alt,ias,dial_kts,sim_tgt_kts,mach,sim_tgt_mach,"
	.. "vs_meas,vs_dial,fpe,throttle,n1,pitch,flaps,acc_ias,offset,vs_limit,thrust_mode,at_active"

local function logRow()
	if not logTried then openLog() end
	local row = string.format(
		"%.1f,%d,%d,%d,%d,%.0f,%.0f,%.1f,%.1f,%.1f,%.3f,%.3f,%.0f,%.0f,%.0f,%.2f,%.1f,%.1f,%.2f,%.2f,%.1f,%.0f,%d,%d",
		simDRTime, num(B747DR_ap_vnav_state), num(B747DR_ap_inVNAVdescent), num(simDR_autopilot_flch_status),
		num(simDR_autopilot_vs_status), num(simDR_pressureAlt1), num(simDR_autopilot_altitude_ft),
		num(simDR_ind_airspeed_kts_pilot), num(SPDCTL_dialKts()), num(simDR_autopilot_airspeed_kts),
		num(simDR_airspeed_mach), num(simDR_autopilot_airspeed_kts_mach), num(simDR_vvi_fpm_pilot),
		num(simDR_autopilot_vs_fpm), num(B747BR_fpe), num(simDR_allThrottle),
		math.max(num(B747DR_display_N1[0]), num(B747DR_display_N1[1]), num(B747DR_display_N1[2]), num(B747DR_display_N1[3])),
		num(simDR_AHARS_pitch_heading_deg_pilot), num(simDR_flap_ratio_control), S.aIas, S.off, D.lim,
		num(B747DR_ap_thrust_mode), num(B747DR_autothrottle_active))
	if logFile ~= nil then
		if not logHeaderDone then
			logFile:write(HEADER .. "\n")
			logHeaderDone = true
		end
		logFile:write(row .. "\n")
		logFile:flush()
	elseif simDRTime - lastPrint > 2 then
		print("SPDCTL," .. row)
		lastPrint = simDRTime
	end
end

local function frame_impl()
	update()
	if cfg.log and B747DR_ap_vnav_state > 0 and (simDRTime - lastLog) >= cfg.logPeriod then
		lastLog = simDRTime
		logRow()
	end
end

function SPDCTL_frame()
	if not cfg.enabled then return end
	local ok, err = pcall(frame_impl)
	if not ok then fail("frame", err) end
end

print("SPDCTL speed control module loaded (enabled=" .. tostring(cfg.enabled) .. ")")
