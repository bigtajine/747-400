--[[ B744 FMC geodesy helpers + WMM2025 magnetic variation.
     Pure Lua (no X-Plane API). Registered as global B744GEO (table with functions so the
     xtlua namespace stores it raw). Units: degrees, nautical miles, feet unless noted. ]]
local M = {}
local sin, cos, tan, asin, acos, atan2, sqrt, abs, floor, pi =
  math.sin, math.cos, math.tan, math.asin, math.acos, math.atan2, math.sqrt, math.abs, math.floor, math.pi
local D2R, R2D = pi / 180, 180 / pi
local RE = 3440.065 -- earth radius nm
M.RE = RE
M.D2R, M.R2D = D2R, R2D

function M.wrap360(a) a = a % 360 if a < 0 then a = a + 360 end return a end
function M.wrap180(a) a = (a + 180) % 360 - 180 if a < -180 then a = a + 360 end return a end
function M.angdiff(a, b) return M.wrap180(a - b) end -- a minus b, -180..180

function M.dist(lat1, lon1, lat2, lon2)
  local p1, p2 = lat1 * D2R, lat2 * D2R
  local dp, dl = p2 - p1, (lon2 - lon1) * D2R
  local a = sin(dp / 2) ^ 2 + cos(p1) * cos(p2) * sin(dl / 2) ^ 2
  if a > 1 then a = 1 end
  return 2 * RE * asin(sqrt(a))
end

function M.brg(lat1, lon1, lat2, lon2)
  local p1, p2 = lat1 * D2R, lat2 * D2R
  local dl = (lon2 - lon1) * D2R
  local y = sin(dl) * cos(p2)
  local x = cos(p1) * sin(p2) - sin(p1) * cos(p2) * cos(dl)
  if x == 0 and y == 0 then return 0 end
  return M.wrap360(atan2(y, x) * R2D)
end

function M.dest(lat, lon, brg, d)
  local p1, l1, t, dr = lat * D2R, lon * D2R, brg * D2R, d / RE
  local p2 = asin(sin(p1) * cos(dr) + cos(p1) * sin(dr) * cos(t))
  local l2 = l1 + atan2(sin(t) * sin(dr) * cos(p1), cos(dr) - sin(p1) * sin(p2))
  return p2 * R2D, M.wrap180(l2 * R2D)
end

-- cross track (nm, + = right of A->B) and along track (nm from A) of P
function M.xtk_atk(alat, alon, blat, blon, plat, plon)
  local d13 = M.dist(alat, alon, plat, plon) / RE
  local t13 = M.brg(alat, alon, plat, plon) * D2R
  local t12 = M.brg(alat, alon, blat, blon) * D2R
  local x = asin(sin(d13) * sin(t13 - t12))
  local c = cos(d13) / cos(x)
  if c > 1 then c = 1 elseif c < -1 then c = -1 end
  local a = acos(c)
  if cos(t13 - t12) < 0 then a = -a end
  return x * RE, a * RE
end

-- intersection of two great-circle courses (true deg); returns lat, lon or nil
function M.intersect(lat1, lon1, brg1, lat2, lon2, brg2)
  local p1, l1, p2, l2 = lat1 * D2R, lon1 * D2R, lat2 * D2R, lon2 * D2R
  local t13, t23 = brg1 * D2R, brg2 * D2R
  local dp, dl = p2 - p1, l2 - l1
  local d12 = 2 * asin(sqrt(sin(dp / 2) ^ 2 + cos(p1) * cos(p2) * sin(dl / 2) ^ 2))
  if d12 < 1e-9 then return lat1, lon1 end
  local ca = (sin(p2) - sin(p1) * cos(d12)) / (sin(d12) * cos(p1))
  local cb = (sin(p1) - sin(p2) * cos(d12)) / (sin(d12) * cos(p2))
  if ca > 1 then ca = 1 elseif ca < -1 then ca = -1 end
  if cb > 1 then cb = 1 elseif cb < -1 then cb = -1 end
  local ta, tb = acos(ca), acos(cb)
  local t12, t21
  if sin(l2 - l1) > 0 then t12, t21 = ta, 2 * pi - tb else t12, t21 = 2 * pi - ta, tb end
  local a1 = (t13 - t12 + pi) % (2 * pi) - pi
  local a2 = (t21 - t23 + pi) % (2 * pi) - pi
  if sin(a1) == 0 and sin(a2) == 0 then return nil end
  if sin(a1) * sin(a2) < 0 then return nil end
  a1, a2 = abs(a1), abs(a2)
  local a3 = acos(-cos(a1) * cos(a2) + sin(a1) * sin(a2) * cos(d12))
  local d13 = atan2(sin(d12) * sin(a1) * sin(a2), cos(a2) + cos(a1) * cos(a3))
  if d13 < 0 then return nil end
  local p3 = asin(sin(p1) * cos(d13) + cos(p1) * sin(d13) * cos(t13))
  local l3 = l1 + atan2(sin(t13) * sin(d13) * cos(p1), cos(d13) - sin(p1) * sin(p3))
  return p3 * R2D, M.wrap180(l3 * R2D)
end

-- local tangent plane (nm east, nm north) about lat0/lon0; fine for < ~100 nm
function M.to_local(lat0, lon0, lat, lon)
  local dl = M.wrap180(lon - lon0)
  return dl * 60 * cos(lat0 * D2R), (lat - lat0) * 60
end
function M.from_local(lat0, lon0, x, y)
  return lat0 + y / 60, M.wrap180(lon0 + x / (60 * cos(lat0 * D2R)))
end

-- turn radius nm for groundspeed kt and bank deg
function M.turn_radius(gs, bank)
  local v = gs * 0.514444
  return (v * v / (9.80665 * tan((bank > 1 and bank or 1) * D2R))) / 1852
end

----------------------------------------------------------------------------------------------
-- WMM magnetic variation (east positive)
----------------------------------------------------------------------------------------------
-- WMM2025 coefficients (NOAA/BGS World Magnetic Model, public domain): n, m, g, h, gdot, hdot
local WMM_EPOCH = 2025.0
local WMM_COF = {
  {1,0,-29351.8,0.0,12.0,0.0},
  {1,1,-1410.8,4545.4,9.7,-21.5},
  {2,0,-2556.6,0.0,-11.6,0.0},
  {2,1,2951.1,-3133.6,-5.2,-27.7},
  {2,2,1649.3,-815.1,-8.0,-12.1},
  {3,0,1361.0,0.0,-1.3,0.0},
  {3,1,-2404.1,-56.6,-4.2,4.0},
  {3,2,1243.8,237.5,0.4,-0.3},
  {3,3,453.6,-549.5,-15.6,-4.1},
  {4,0,895.0,0.0,-1.6,0.0},
  {4,1,799.5,278.6,-2.4,-1.1},
  {4,2,55.7,-133.9,-6.0,4.1},
  {4,3,-281.1,212.0,5.6,1.6},
  {4,4,12.1,-375.6,-7.0,-4.4},
  {5,0,-233.2,0.0,0.6,0.0},
  {5,1,368.9,45.4,1.4,-0.5},
  {5,2,187.2,220.2,0.0,2.2},
  {5,3,-138.7,-122.9,0.6,0.4},
  {5,4,-142.0,43.0,2.2,1.7},
  {5,5,20.9,106.1,0.9,1.9},
  {6,0,64.4,0.0,-0.2,0.0},
  {6,1,63.8,-18.4,-0.4,0.3},
  {6,2,76.9,16.8,0.9,-1.6},
  {6,3,-115.7,48.8,1.2,-0.4},
  {6,4,-40.9,-59.8,-0.9,0.9},
  {6,5,14.9,10.9,0.3,0.7},
  {6,6,-60.7,72.7,0.9,0.9},
  {7,0,79.5,0.0,-0.0,0.0},
  {7,1,-77.0,-48.9,-0.1,0.6},
  {7,2,-8.8,-14.4,-0.1,0.5},
  {7,3,59.3,-1.0,0.5,-0.8},
  {7,4,15.8,23.4,-0.1,0.0},
  {7,5,2.5,-7.4,-0.8,-1.0},
  {7,6,-11.1,-25.1,-0.8,0.6},
  {7,7,14.2,-2.3,0.8,-0.2},
  {8,0,23.2,0.0,-0.1,0.0},
  {8,1,10.8,7.1,0.2,-0.2},
  {8,2,-17.5,-12.6,0.0,0.5},
  {8,3,2.0,11.4,0.5,-0.4},
  {8,4,-21.7,-9.7,-0.1,0.4},
  {8,5,16.9,12.7,0.3,-0.5},
  {8,6,15.0,0.7,0.2,-0.6},
  {8,7,-16.8,-5.2,-0.0,0.3},
  {8,8,0.9,3.9,0.2,0.2},
  {9,0,4.6,0.0,-0.0,0.0},
  {9,1,7.8,-24.8,-0.1,-0.3},
  {9,2,3.0,12.2,0.1,0.3},
  {9,3,-0.2,8.3,0.3,-0.3},
  {9,4,-2.5,-3.3,-0.3,0.3},
  {9,5,-13.1,-5.2,0.0,0.2},
  {9,6,2.4,7.2,0.3,-0.1},
  {9,7,8.6,-0.6,-0.1,-0.2},
  {9,8,-8.7,0.8,0.1,0.4},
  {9,9,-12.9,10.0,-0.1,0.1},
  {10,0,-1.3,0.0,0.1,0.0},
  {10,1,-6.4,3.3,0.0,0.0},
  {10,2,0.2,0.0,0.1,-0.0},
  {10,3,2.0,2.4,0.1,-0.2},
  {10,4,-1.0,5.3,-0.0,0.1},
  {10,5,-0.6,-9.1,-0.3,-0.1},
  {10,6,-0.9,0.4,0.0,0.1},
  {10,7,1.5,-4.2,-0.1,0.0},
  {10,8,0.9,-3.8,-0.1,-0.1},
  {10,9,-2.7,0.9,-0.0,0.2},
  {10,10,-3.9,-9.1,-0.0,-0.0},
  {11,0,2.9,0.0,0.0,0.0},
  {11,1,-1.5,0.0,-0.0,-0.0},
  {11,2,-2.5,2.9,0.0,0.1},
  {11,3,2.4,-0.6,0.0,-0.0},
  {11,4,-0.6,0.2,0.0,0.1},
  {11,5,-0.1,0.5,-0.1,-0.0},
  {11,6,-0.6,-0.3,0.0,-0.0},
  {11,7,-0.1,-1.2,-0.0,0.1},
  {11,8,1.1,-1.7,-0.1,-0.0},
  {11,9,-1.0,-2.9,-0.1,0.0},
  {11,10,-0.2,-1.8,-0.1,0.0},
  {11,11,2.6,-2.3,-0.1,0.0},
  {12,0,-2.0,0.0,0.0,0.0},
  {12,1,-0.2,-1.3,0.0,-0.0},
  {12,2,0.3,0.7,-0.0,0.0},
  {12,3,1.2,1.0,-0.0,-0.1},
  {12,4,-1.3,-1.4,-0.0,0.1},
  {12,5,0.6,-0.0,-0.0,-0.0},
  {12,6,0.6,0.6,0.1,-0.0},
  {12,7,0.5,-0.1,-0.0,-0.0},
  {12,8,-0.1,0.8,0.0,0.0},
  {12,9,-0.4,0.1,0.0,-0.0},
  {12,10,-0.2,-1.0,-0.1,-0.0},
  {12,11,-1.3,0.1,-0.0,0.0},
  {12,12,-0.7,0.2,-0.1,-0.1},
}

local MAXORD = 12
local c, cd, k, snorm, fn, fm = {}, {}, {}, {}, {}, {}
local wmm_ready = false
local function idx(m, n) return m * 13 + n end
local function wmm_init()
  for i = 0, 13 * 13 do c[i] = 0; cd[i] = 0; k[i] = 0; snorm[i] = 0 end
  for _, r in ipairs(WMM_COF) do
    local n, m, g, h, gd, hd = r[1], r[2], r[3], r[4], r[5], r[6]
    c[idx(m, n)] = g; cd[idx(m, n)] = gd
    if m ~= 0 then c[idx(n, m - 1)] = h; cd[idx(n, m - 1)] = hd end
  end
  snorm[idx(0, 0)] = 1
  fm[0] = 0
  for n = 1, MAXORD do
    snorm[idx(0, n)] = snorm[idx(0, n - 1)] * (2 * n - 1) / n
    local j = 2
    for m = 0, n do
      k[idx(m, n)] = ((n - 1) * (n - 1) - m * m) / ((2 * n - 1) * (2 * n - 3))
      if m > 0 then
        local flnmj = ((n - m + 1) * j) / (n + m)
        snorm[idx(m, n)] = snorm[idx(m - 1, n)] * sqrt(flnmj)
        j = 1
        c[idx(n, m - 1)] = snorm[idx(m, n)] * c[idx(n, m - 1)]
        cd[idx(n, m - 1)] = snorm[idx(m, n)] * cd[idx(n, m - 1)]
      end
      c[idx(m, n)] = snorm[idx(m, n)] * c[idx(m, n)]
      cd[idx(m, n)] = snorm[idx(m, n)] * cd[idx(m, n)]
    end
    fn[n] = n + 1
    fm[n] = n
  end
  k[idx(1, 1)] = 0
  wmm_ready = true
end

local p, dp, tc, sp, cp = {}, {}, {}, {}, {}
function M.magvar(glat, glon, alt_ft, year)
  if not wmm_ready then wmm_init() end
  year = year or 2026.5
  local alt = (alt_ft or 0) * 0.0003048
  local dt = year - WMM_EPOCH
  local a, b, re = 6378.137, 6356.7523142, 6371.2
  local a2, b2 = a * a, b * b
  local c2, a4, b4 = a2 - b2, a2 * a2, b2 * b2
  local c4 = a4 - b4
  local rlon, rlat = glon * D2R, glat * D2R
  local srlon, srlat, crlon, crlat = sin(rlon), sin(rlat), cos(rlon), cos(rlat)
  local srlat2, crlat2 = srlat * srlat, crlat * crlat
  sp[0], cp[0] = 0, 1
  sp[1], cp[1] = srlon, crlon
  local q = sqrt(a2 - c2 * srlat2)
  local q1 = alt * q
  local q2 = ((q1 + a2) / (q1 + b2)) ^ 2
  local ct = srlat / sqrt(q2 * crlat2 + srlat2)
  local st = sqrt(1 - ct * ct)
  local r2 = alt * alt + 2 * q1 + (a4 - c4 * srlat2) / (q * q)
  local r = sqrt(r2)
  local d = sqrt(a2 * crlat2 + b2 * srlat2)
  local ca = (alt + d) / r
  local sa = c2 * crlat * srlat / (r * d)
  for m = 2, MAXORD do
    sp[m] = sp[1] * cp[m - 1] + cp[1] * sp[m - 1]
    cp[m] = cp[1] * cp[m - 1] - sp[1] * sp[m - 1]
  end
  local aor = re / r
  local ar = aor * aor
  local br, bt, bp, bpp = 0, 0, 0, 0
  p[idx(0, 0)], dp[idx(0, 0)] = 1, 0
  local pp = {}; pp[0] = 1
  for n = 1, MAXORD do
    ar = ar * aor
    for m = 0, n do
      local mn = idx(m, n)
      if n == m then
        p[mn] = st * p[idx(m - 1, n - 1)]
        dp[mn] = st * dp[idx(m - 1, n - 1)] + ct * p[idx(m - 1, n - 1)]
      elseif n == 1 and m == 0 then
        p[mn] = ct * p[idx(m, n - 1)]
        dp[mn] = ct * dp[idx(m, n - 1)] - st * p[idx(m, n - 1)]
      elseif n > 1 and n ~= m then
        if m > n - 2 then p[idx(m, n - 2)] = 0; dp[idx(m, n - 2)] = 0 end
        p[mn] = ct * p[idx(m, n - 1)] - k[mn] * p[idx(m, n - 2)]
        dp[mn] = ct * dp[idx(m, n - 1)] - st * p[idx(m, n - 1)] - k[mn] * dp[idx(m, n - 2)]
      end
      tc[mn] = c[mn] + dt * cd[mn]
      if m ~= 0 then tc[idx(n, m - 1)] = c[idx(n, m - 1)] + dt * cd[idx(n, m - 1)] end
      local par = ar * p[mn]
      local temp1, temp2
      if m == 0 then
        temp1 = tc[mn] * cp[m]; temp2 = tc[mn] * sp[m]
      else
        temp1 = tc[mn] * cp[m] + tc[idx(n, m - 1)] * sp[m]
        temp2 = tc[mn] * sp[m] - tc[idx(n, m - 1)] * cp[m]
      end
      bt = bt - ar * temp1 * dp[mn]
      bp = bp + fm[m] * temp2 * par
      br = br + fn[n] * temp1 * par
      if st == 0 and m == 1 then
        if n == 1 then pp[n] = pp[n - 1] else pp[n] = ct * pp[n - 1] - k[mn] * pp[n - 2] end
        bpp = bpp + fm[m] * temp2 * ar * pp[n]
      end
    end
  end
  if st == 0 then bp = bpp else bp = bp / st end
  local bx = -bt * ca - br * sa
  local by = bp
  return atan2(by, bx) * R2D
end

-- cached coarse magvar (2 deg grid) for speed in loops
local mv_cache = {}
function M.magvar_fast(lat, lon)
  local kx, ky = floor(lon / 2 + 0.5), floor(lat / 2 + 0.5)
  local key = kx * 1000 + ky
  local v = mv_cache[key]
  if v == nil then v = M.magvar(ky * 2, kx * 2, 0); mv_cache[key] = v end
  return v
end
function M.true_to_mag(tru, lat, lon) return M.wrap360(tru - M.magvar_fast(lat, lon)) end
function M.mag_to_true(mag, lat, lon) return M.wrap360(mag + M.magvar_fast(lat, lon)) end

B744GEO = M
