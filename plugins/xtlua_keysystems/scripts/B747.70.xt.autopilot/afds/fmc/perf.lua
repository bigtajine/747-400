--[[ B744 FMC performance model (approximation of the 747-400 PDB used by the real FMC).
     Standard atmosphere, CAS/Mach/TAS conversions, ECON speeds vs cost index, climb rate,
     idle descent gradient, fuel flow, max/opt altitude, flap placard speeds.
     Pure Lua; registered as global B744PERF. ]]
local M = {}
local sqrt, exp, min, max, floor, abs, tan, atan, pi = math.sqrt, math.exp, math.min, math.max, math.floor, math.abs, math.tan, math.atan, math.pi

-- ISA ---------------------------------------------------------------------------------------------
function M.isa(h)               -- h ft -> T(K), delta (p/p0), a (kt)
  local T, d
  if h < 36089 then
    T = 288.15 - 0.0019812 * h
    d = (T / 288.15) ^ 5.255876
  else
    T = 216.65
    d = 0.223361 * exp(-(h - 36089) / 20806)
  end
  return T, d, 38.967854 * sqrt(T)
end
local A0 = 661.4786
function M.cas_to_mach(cas, h)
  local _, d = M.isa(h)
  local qc = (1 + 0.2 * (cas / A0) ^ 2) ^ 3.5 - 1
  return sqrt(5 * ((qc / d + 1) ^ (2 / 7) - 1))
end
function M.mach_to_cas(m, h)
  local _, d = M.isa(h)
  local qc = d * ((1 + 0.2 * m * m) ^ 3.5 - 1)
  return A0 * sqrt(5 * ((qc + 1) ^ (2 / 7) - 1))
end
function M.tas(cas, h, isadev)
  local T, _, a = M.isa(h)
  if isadev then a = 38.967854 * sqrt(T + isadev) end
  return M.cas_to_mach(cas, h) * a
end
function M.tas_mach(m, h) local _, _, a = M.isa(h) return m * a end
-- altitude where CAS equals Mach (crossover)
function M.crossover(cas, mach)
  local lo, hi = 0, 45000
  for _ = 1, 30 do
    local mid = (lo + hi) / 2
    if M.cas_to_mach(cas, mid) < mach then lo = mid else hi = mid end
  end
  return (lo + hi) / 2
end

-- ECON speeds ------------------------------------------------------------------------------------
local function interp(tbl, x)
  if x <= tbl[1][1] then return tbl[1][2] end
  for i = 2, #tbl do
    if x <= tbl[i][1] then
      local a, b = tbl[i - 1], tbl[i]
      return a[2] + (b[2] - a[2]) * (x - a[1]) / (b[1] - a[1])
    end
  end
  return tbl[#tbl][2]
end
M.interp = interp
local CLB_CAS = { { 0, 290 }, { 40, 305 }, { 100, 318 }, { 200, 330 }, { 500, 340 }, { 9999, 340 } }
local CLB_M = { { 0, 0.820 }, { 60, 0.835 }, { 150, 0.845 }, { 400, 0.855 }, { 9999, 0.86 } }
local CRZ_M = { { 0, 0.835 }, { 40, 0.845 }, { 100, 0.855 }, { 200, 0.862 }, { 500, 0.87 }, { 9999, 0.875 } }
local DES_M = { { 0, 0.800 }, { 50, 0.830 }, { 100, 0.845 }, { 300, 0.855 }, { 9999, 0.86 } }
local DES_CAS = { { 0, 265 }, { 50, 290 }, { 100, 305 }, { 200, 320 }, { 500, 335 }, { 9999, 340 } }

-- weight adjustment (heavier -> slightly faster econ)
local function wadj(w) return (w - 300000) / 100000 end
function M.econ_clb(ci, w) return floor(interp(CLB_CAS, ci) + 5 * wadj(w) + 0.5), interp(CLB_M, ci) end
function M.econ_crz(ci, w, h)
  local m = interp(CRZ_M, ci) + 0.005 * wadj(w)
  return min(0.88, m)
end
function M.lrc_crz(w) return min(0.87, 0.840 + 0.008 * wadj(w)) end
function M.econ_des(ci, w) return floor(interp(DES_CAS, ci) + 0.5), interp(DES_M, ci) end

-- limits -----------------------------------------------------------------------------------------
M.VMO, M.MMO = 365, 0.92
-- flap placards (deg -> kt)
M.FLAP_PLACARD = { [0] = 365, [1] = 280, [5] = 260, [10] = 240, [20] = 230, [25] = 205, [30] = 180 }
function M.flap_detent(ratio)
  -- sim flap handle ratio (0..1) for 0,1,5,10,20,25,30
  if ratio < 0.05 then return 0 elseif ratio < 0.25 then return 1 elseif ratio < 0.42 then return 5
  elseif ratio < 0.58 then return 10 elseif ratio < 0.75 then return 20 elseif ratio < 0.92 then return 25 end
  return 30
end
function M.max_alt(w)  -- ft (stock FCOM-derived fit)
  local kft = 58.16363636 - 0.06154545 * (w / 1000)
  return floor(min(45100, max(29000, kft * 1000)) / 100) * 100
end
function M.opt_alt(w)
  return floor((M.max_alt(w) - 2000) / 100) * 100
end

-- climb rate (fpm) at climb thrust ---------------------------------------------------------------
local ROC = { { 0, 3300 }, { 5000, 3000 }, { 10000, 2700 }, { 15000, 2300 }, { 20000, 2000 }, { 25000, 1700 },
  { 30000, 1300 }, { 33000, 1000 }, { 36000, 700 }, { 39000, 450 }, { 42000, 250 }, { 45000, 100 } }
function M.climb_rate(h, w, isadev)
  local r = interp(ROC, h) * (300000 / w) ^ 1.6
  if isadev and isadev > 0 then r = r * (1 - 0.01 * isadev) end
  return max(100, r)
end
-- ft per nm in climb
function M.climb_grad(h, w, gs)
  return M.climb_rate(h, w) * 60 / max(100, gs)
end

-- idle descent flight path angle (deg, air mass) -> ft/nm over ground with gs
-- v41: physics model (drag polar + idle thrust) calibrated to X-Plane's 747-400 from flight data
-- (idle descents 3500-17000 ft, 225-265 kt, 214 t: CD0 0.0273 +/- 0.002, about 3.5-4.4 deg) - the
-- old book fit (2.6-2.8 deg) was 35-60 % too shallow, so VNAV planned paths it could not fly at idle
M.IDLE_CD0, M.IDLE_K, M.IDLE_T = 0.0273, 0.045, 0.035
function M.idle_fpa(cas, h, w)
  local T, d, a = M.isa(h)
  local rho = 1.225 * d * 288.15 / T
  local mach = M.cas_to_mach(cas, h)
  local v = mach * a * 0.514444
  local q = 0.5 * rho * v * v
  local W = max(100000, w or 300000) * 9.80665
  local cl = W / max(1, q * 511)
  local cd = M.IDLE_CD0 + M.IDLE_K * cl * cl + 12 * max(0, mach - 0.80) ^ 3
  local ti = 4 * 282000 * (rho / 1.225) ^ 0.85 * (1 - 0.35 * mach) * M.IDLE_T
  local s = (q * 511 * cd - ti) / W
  local g = math.asin(max(-0.2, min(0.2, s))) * 180 / pi
  return max(1.8, min(6.5, g))
end
function M.idle_grad(cas, h, w, gs, tas)
  local fpa = M.idle_fpa(cas, h, w)
  local g = tan(fpa * pi / 180) * 6076.12
  if gs and tas and gs > 50 then g = g * tas / gs end
  return g
end
-- v41: idle descent gradient at the descent speed schedule (constant CAS, or constant Mach above the
-- crossover): with a constant CAS the true airspeed falls in the descent and that kinetic energy
-- makes the descent shallower (about 25 % at FL300, 5 % below 10000 ft); at a constant Mach it rises
-- (steeper). idle_grad is the constant-TAS air-mass gradient.
function M.idle_grad_des(cas, h, w, gs, tas, mach_lim)
  local g = M.idle_grad(cas, h, w, gs, tas)
  local function v_at(hh)
    local c = cas
    if mach_lim and M.cas_to_mach(c, hh) > mach_lim then c = M.mach_to_cas(mach_lim, hh) end
    return M.tas(c, hh) * 1.6878
  end
  local v1, v2 = v_at(h - 500), v_at(h + 500)
  local f = 1 + (v1 + v2) / 2 * (v2 - v1) / 1000 / 32.174
  return g / max(0.5, min(1.6, f))
end
-- deceleration at idle: kt per nm (level) / in descent with reduced gradient
function M.decel_kt_per_nm(gs) return 0.9 * 3600 / max(150, gs) end

-- fuel flow kg/h (total) -------------------------------------------------------------------------
function M.ff_cruise(w, mach, h)
  local f = w * 0.0310 * (mach / 0.85) ^ 2
  if h < 30000 then f = f * (1 + (30000 - h) / 60000) end
  return f
end
function M.ff_climb(h, w) return max(9000, (19500 - 0.20 * h) * (w / 330000) ^ 0.5) end
function M.ff_descent() return 2200 end

-- maneuver speeds (if not supplied by the aircraft): VREF30 approx from weight
function M.vref30(w) return floor(0.00032 * w + 67 + 0.5) end   -- 200t -> 131, 285t -> 158
function M.flaps_up_speed(w) return M.vref30(w) + 80 end

B744PERF = M
