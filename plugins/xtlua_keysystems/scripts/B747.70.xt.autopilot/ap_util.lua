--[[
 ap_util.lua  --  shared pure helpers for the rebuilt Sparky 747 autopilot.
 No X-Plane API calls in this file: it runs unchanged in the offline test harness.
 Lua 5.1 / LuaJIT compatible (X-Plane xlua).
--]]
local U = {}

U.NM_FT   = 6076.12
U.R_NM    = 3440.065      -- mean earth radius in nm
U.KT_MPS  = 0.514444
U.MPS_KT  = 1.94384449

function U.clamp(x, lo, hi)
  if x < lo then return lo end
  if x > hi then return hi end
  return x
end

-- identical semantics to the stock B747_rescale (clamped linear map)
function U.rescale(in1, out1, in2, out2, x)
  if x < in1 then return out1 end
  if x > in2 then return out2 end
  return out1 + (out2 - out1) * (x - in1) / (in2 - in1)
end

function U.wrap360(a)
  a = a % 360
  return a
end

function U.wrap180(a)
  a = (a + 180) % 360 - 180
  return a
end

-- signed shortest angle from b to a  (a - b) in (-180,180]
function U.hdg_diff(a, b)
  return U.wrap180(a - b)
end

function U.round(x)
  if x >= 0 then return math.floor(x + 0.5) end
  return math.ceil(x - 0.5)
end

-- great-circle distance (nm); haversine is numerically stable for short legs
function U.distance_nm(lat1, lon1, lat2, lon2)
  if lat1 == lat2 and lon1 == lon2 then return 0 end
  local p1, p2 = math.rad(lat1), math.rad(lat2)
  local dp = p2 - p1
  local dl = math.rad(lon2 - lon1)
  local a = math.sin(dp / 2) ^ 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ^ 2
  if a > 1 then a = 1 end
  return 2 * U.R_NM * math.asin(math.sqrt(a))
end

-- initial true bearing 0..360
function U.bearing(lat1, lon1, lat2, lon2)
  local p1, p2 = math.rad(lat1), math.rad(lat2)
  local dl = math.rad(lon2 - lon1)
  local y = math.sin(dl) * math.cos(p2)
  local x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
  return U.wrap360(math.deg(math.atan2(y, x)))
end

function U.destination(lat, lon, dist_nm, brg)
  local d = dist_nm / U.R_NM
  local b = math.rad(brg)
  local p1, l1 = math.rad(lat), math.rad(lon)
  local p2 = math.asin(math.sin(p1) * math.cos(d) + math.cos(p1) * math.sin(d) * math.cos(b))
  local l2 = l1 + math.atan2(math.sin(b) * math.sin(d) * math.cos(p1),
                             math.cos(d) - math.sin(p1) * math.sin(p2))
  return math.deg(p2), U.wrap180(math.deg(l2))
end

-- cross-track (+ = aircraft RIGHT of course) and along-track distance (nm, from leg start;
-- negative = before the start) of a position relative to the great circle lat1/lon1 -> lat2/lon2
function U.xtk_atk(lat1, lon1, lat2, lon2, lat, lon)
  local d13 = U.distance_nm(lat1, lon1, lat, lon)
  if d13 < 1e-6 then return 0, 0 end
  local th13 = math.rad(U.bearing(lat1, lon1, lat, lon))
  local th12 = math.rad(U.bearing(lat1, lon1, lat2, lon2))
  local dd = d13 / U.R_NM
  local s = U.clamp(math.sin(dd) * math.sin(th13 - th12), -1, 1)
  local xtk_ang = math.asin(s)
  local c = math.cos(dd) / math.cos(xtk_ang)
  local atk = math.acos(U.clamp(c, -1, 1)) * U.R_NM
  if math.cos(th13 - th12) < 0 then atk = -atk end
  return xtk_ang * U.R_NM, atk
end

-- Wind-correction, ported from the stock getWCAforHeading so the sign convention that is
-- already tuned in the sim is preserved.  windBearing = laminar/B747/nd/wind_bearing.
function U.wca_heading(theading, wind_bearing, wind_kts, tas_kts)
  if tas_kts < 30 then return U.wrap360(theading) end
  local heading = U.wrap360(theading)
  local wca
  local r = wind_kts / tas_kts
  if wind_bearing < -90 then
    wca = r * math.sin(math.rad(180 - (wind_bearing * -1)))
  elseif wind_bearing < 0 then
    wca = r * math.sin(math.rad(wind_bearing * -1))
  elseif wind_bearing > 90 then
    wca = -r * math.sin(math.rad(180 - wind_bearing))
  else
    wca = -r * math.sin(math.rad(wind_bearing))
  end
  wca = U.clamp(wca, -1, 1)
  return U.wrap360(heading + math.deg(math.asin(wca)))
end

-- first-order lag, time-constant tau (s)
function U.lag(prev, target, dt, tau)
  if tau <= 0 or prev == nil then return target end
  return prev + (target - prev) * (1 - math.exp(-dt / tau))
end

AP_MODULES = AP_MODULES or {}
AP_MODULES["ap_util.lua"] = U
return U
