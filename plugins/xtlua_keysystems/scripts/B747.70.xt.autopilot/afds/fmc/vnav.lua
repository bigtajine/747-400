--[[ B744 FMC vertical navigation: profile prediction and guidance.
     profile(): climb prediction (forward), descent path (backward from E/D with constraint anchoring,
     idle / geometric / deceleration segments), T/C, T/D, waypoint altitude/speed/ETA/fuel predictions.
     guide(): VNAV sub-mode logic (SPD / PTH / ALT), A/T mode request, pitch law and targets,
     altitude intervention, DES NOW, cruise climb/descent, speed intervention, path speed protection.
     Pure Lua; registered as global B744VNAV. ]]
local M = {}
local PF = B744PERF
local RT = B744ROUTE
local abs, min, max, floor, tan, atan, pi = math.abs, math.min, math.max, math.floor, math.tan, math.atan, math.pi
local cos = math.cos
local R2D = 180 / pi

-- helpers -------------------------------------------------------------------------------------------
local function speed_cas_at(cas, mach, h)
  -- the commanded speed (CAS) for a CAS/Mach schedule at altitude h
  local mc = PF.mach_to_cas(mach, h)
  if mc < cas then return mc, true end
  return cas, false
end
M.speed_cas_at = speed_cas_at

local function descent_region_start(plan)
  for i, l in ipairs(plan.legs) do
    if l.seg == "STAR" or l.seg == "APP" or l.seg == "DEST" then return i end
  end
  return #plan.legs + 1
end
M.descent_region_start = descent_region_start

local function first_app(plan)
  for i, l in ipairs(plan.legs) do if l.seg == "APP" then return i end end
  return nil
end

-- E/D: last leg (before missed approach) with an altitude constraint
function M.find_ed(plan)
  local last = RT.last_route_index(plan)
  for i = last, 1, -1 do
    local l = plan.legs[i]
    if l.alt and l.seg ~= "SID" and l.seg ~= "RWY" then
      local a = l.alt
      local h = (a.d == "B") and a.b or a.a
      return i, h
    end
  end
  return nil
end

-- learned climb performance factors by 5000 ft band (table index 0..9), 1 = performance model
M.DES_DEFAULT = 1.0    -- v41: the idle model is calibrated to X-Plane's 747 (perf.lua); learning trims it
function M.clb_factor(tab, h, default)
  default = default or 1
  if not tab then return default end
  local x = math.max(0, h) / 5000 - 0.5
  local i = math.floor(x)
  local f = x - i
  local a = tab[math.max(0, math.min(9, i))] or default
  local b = tab[math.max(0, math.min(9, i + 1))] or default
  return a + (b - a) * math.max(0, math.min(1, f))
end

-- PROFILE --------------------------------------------------------------------------------------------
function M.profile(plan, ctx)
  local legs = plan.legs
  local P = { ok = false, legs = {} }
  local n = #legs
  if n < 2 or not plan.total or not ctx.crz or ctx.crz <= 0 then return P end
  local w = ctx.w or 300000
  local ci = ctx.ci or 100
  local crz = ctx.crz
  local ta = ctx.ta or 18000
  local clb_cas, clb_mach = PF.econ_clb(ci, w)
  if ctx.sel_clb_cas then clb_cas = ctx.sel_clb_cas end
  if ctx.sel_clb_mach then clb_mach = ctx.sel_clb_mach end
  local crz_mach = ctx.sel_crz_mach or PF.econ_crz(ci, w, crz)
  local des_cas, des_mach = PF.econ_des(ci, w)
  if ctx.sel_des_cas then des_cas = ctx.sel_des_cas end
  if ctx.sel_des_mach then des_mach = ctx.sel_des_mach end
  P.clb_cas, P.clb_mach, P.crz_mach, P.des_cas, P.des_mach = clb_cas, clb_mach, crz_mach, des_cas, des_mach
  P.crz_cas = ctx.sel_crz_cas or math.max(clb_cas, des_cas, 300)
  -- speed limit: a cruise level below 10000 ft is flown at no more than 250 kt
  if (ctx.crz or 0) > 0 and ctx.crz < M.SPD_LIM_ALT then P.crz_cas = math.min(P.crz_cas, M.SPD_LIM_KT) end
  local clbT = ctx.clb_trans or { spd = 250, alt = 10000 }
  local desT = ctx.des_trans or { spd = 240, alt = 10000 }
  local fus = ctx.flaps_up or PF.flaps_up_speed(w)
  local last = RT.last_route_index(plan)
  local total = legs[last] and legs[last].cum or plan.total
  P.total = total
  local s_now = ctx.s_now or 0
  local h_now = ctx.alt_now or ctx.orig_elev or 0
  local dreg = descent_region_start(plan)
  local wind = ctx.wind_comp or 0
  -- descent winds: the measured along-track wind, reducing towards the ground (FMC style forecast
  -- when no descent winds are entered): 100 % at/above the reference altitude, 30 % at sea level
  local wref_h = max(5000, ctx.wind_alt or h_now)
  local function wind_at(hh) return wind * max(0.3, min(1, hh / wref_h)) end
  -- DES FORECAST winds (FMC page: entered, or requested from SimBrief / X-Plane weather): the
  -- along-track component on the descent track, interpolated between the forecast altitudes
  local fw = ctx.des_winds
  if fw and #fw >= 1 and ctx.des_trk then
    local comp = {}
    for _, w_ in ipairs(fw) do
      -- (wind FROM dir: a tailwind is wind from behind the track)
      comp[#comp + 1] = { h = w_.alt, c = -w_.spd * cos((w_.dir - ctx.des_trk) * pi / 180) }
    end
    table.sort(comp, function(a, b) return a.h < b.h end)
    wind_at = function(hh)
      if hh <= comp[1].h then return comp[1].c end
      for k = 2, #comp do
        if hh <= comp[k].h then
          local a, b = comp[k - 1], comp[k]
          return a.c + (b.c - a.c) * (hh - a.h) / max(1, b.h - a.h)
        end
      end
      return comp[#comp].c
    end
    P.des_winds_used = true
  end

  -- CLIMB (forward) -------------------------------------------------------------------------------
  local cons_clb = {}
  for i = 1, min(dreg - 1, last) do
    local l = legs[i]
    if l.alt and l.cum and l.cum > s_now - 0.5 then cons_clb[#cons_clb + 1] = { s = l.cum, alt = l.alt, i = i, spd = l.spd } end
  end
  local function clb_cap(s)
    local cap = crz
    for _, c in ipairs(cons_clb) do
      if c.s >= s - 0.01 then
        local hi = RT.alt_hi(c.alt)
        if hi and hi < cap then cap = hi end
      end
    end
    return cap
  end
  local function clb_speed(s, h)
    local agl = h - (ctx.orig_elev or 0)
    local v
    if agl < (ctx.accel_ht or 1000) and ctx.v2 then v = ctx.v2 + 10
    else
      v = clb_cas
      if h < clbT.alt then v = min(v, clbT.spd) end
      if ctx.clb_restr and h < ctx.clb_restr.alt then v = min(v, ctx.clb_restr.spd) end
      for _, c in ipairs(cons_clb) do
        if c.spd and c.s >= s - 0.01 and (c.spd.d == "-" or c.spd.d == "@") then v = min(v, c.spd.v) end
      end
    end
    local cas, ism = speed_cas_at(v, clb_mach, h)
    return cas, ism
  end
  local clb = {}
  local s, h = s_now, h_now
  local t, fuel = 0, ctx.fuel or 0
  local s_toc = nil
  local in_climb = (ctx.phase == nil or ctx.phase == "TO" or ctx.phase == "CLB") and h_now < crz - 50
  if in_climb then
    clb[#clb + 1] = { s = s, h = h, v = clb_speed(s, h), t = 0, f = fuel }
    local guard = 0
    while s < total and guard < 3000 do
      guard = guard + 1
      local ds = (h < 10000) and 1 or 2
      local cas, ism = clb_speed(s, h)
      local tas = PF.tas(cas, h)
      local gs = max(80, tas + wind)
      local roc = PF.climb_rate(h, w) * M.clb_factor(ctx.clb_tab, h)
      local dh = roc * 60 / gs * ds
      local cap = clb_cap(s + ds)
      local hn = min(cap, h + dh)
      if hn >= crz - 1 then
        -- top of climb inside this step
        local frac = (h + dh > h) and (crz - h) / dh or 1
        frac = max(0, min(1, frac))
        s = s + ds * frac
        t = t + ds * frac / gs * 3600
        fuel = fuel - PF.ff_climb(h, w) * (ds * frac / gs)
        h = crz
        clb[#clb + 1] = { s = s, h = h, v = cas, ism = ism, t = t, f = fuel }
        s_toc = s
        break
      end
      s = s + ds
      t = t + ds / gs * 3600
      fuel = fuel - PF.ff_climb(h, w) * (ds / gs)
      h = hn
      clb[#clb + 1] = { s = s, h = h, v = cas, ism = ism, t = t, f = fuel }
    end
    if not s_toc then s_toc = total end
  else
    s_toc = s_now
    clb[1] = { s = s_now, h = h_now, v = 0, t = 0, f = fuel }
  end
  P.clb = clb
  P.s_toc = s_toc

  -- DESCENT (backward from E/D) ------------------------------------------------------------------------
  local ed_i, ed_h = M.find_ed(plan)
  local ed_s
  if ed_i then ed_s = legs[ed_i].cum else ed_s = total; ed_h = (ctx.dest_elev or 0) + 1500 end
  P.ed_i, P.ed_h, P.ed_s = ed_i, ed_h, ed_s
  local app_i = first_app(plan)
  local s_appr = (app_i and legs[app_i - 1] and legs[app_i - 1].cum) or (ed_s - 15)
  s_appr = min(s_appr, ed_s - 8)
  local cons_des = {}
  for i = max(1, dreg - 1), (ed_i or last) do
    local l = legs[i]
    if l.cum then cons_des[#cons_des + 1] = { s = l.cum, alt = l.alt, spd = l.spd, i = i } end
  end
  local function des_target(sp, hh)
    local v = des_cas
    if hh < desT.alt + 50 then v = min(v, desT.spd) end
    if ctx.des_restr and hh < ctx.des_restr.alt then v = min(v, ctx.des_restr.spd) end
    for _, c in ipairs(cons_des) do
      if c.spd and c.s <= sp + 0.01 and (c.spd.d == "-" or c.spd.d == "@") then v = min(v, c.spd.v) end
    end
    if sp >= s_appr then v = min(v, fus) end
    return speed_cas_at(v, des_mach, hh)
  end
  -- legs with a glide path angle (final approach)
  local function fpa_at(sp)
    for i = (app_i or n + 1), (ed_i or last) do
      local l = legs[i]
      if l.fpa and l.cum and l.cum - l.len <= sp + 0.001 and sp <= l.cum + 0.001 then return abs(l.fpa) end
    end
    return nil
  end
  local pts = {}
  local sd, hd = ed_s, ed_h
  local vd = des_target(ed_s, ed_h)
  pts[1] = { s = sd, h = hd, v = vd, k = "ed" }
  local anchor = 1
  -- constraint list sorted by s descending
  local cl = {}
  for _, c in ipairs(cons_des) do if c.alt and c.s < ed_s - 0.01 then cl[#cl + 1] = c end end
  table.sort(cl, function(a, b) return a.s > b.s end)
  local ci_ = 1
  local s_tod = nil
  local guard = 0
  local s_floor = max(0, (ctx.phase == "DES") and -1e9 or s_toc - 400)
  while guard < 4000 do
    guard = guard + 1
    local ds = (hd < 10000) and 1 or 2
    local sn = sd - ds
    -- next constraint crossing
    local c = cl[ci_]
    if c and c.s >= sn then ds = sd - c.s; sn = c.s end
    if ds < 1e-6 then ds = 1e-6 end
    local tv = des_target(sn, hd)
    local grad, kind
    local tas = PF.tas(vd, hd)
    local gs = max(100, tas + wind_at(hd))
    local fpa = fpa_at(sd - ds * 0.5)
    local vn = vd
    if fpa then
      grad = tan(fpa * pi / 180) * 6076.12; kind = "gp"
      if tv > vd then vn = min(tv, vd + PF.decel_kt_per_nm(gs) * ds * 0.5) end
    elseif tv > vd + 0.5 then
      -- deceleration at idle while descending at 30 % of the idle gradient: the other 70 % of the
      -- energy loss slows the aircraft (energy consistent, 15 % margin)
      local G = PF.idle_grad(vd, hd, w, gs, tas) * M.clb_factor(ctx.des_tab, hd, M.DES_DEFAULT)
      local vt = PF.tas(vd, hd) * 1.6878
      local dv = 0.85 * 0.70 * G * 32.174 / max(200, vt) / 1.6878 * (tas / gs)   -- kt per nm over ground
      vn = min(tv, vd + dv * ds)
      grad = G * 0.30; kind = "decel"
    else
      vn = tv
      grad = PF.idle_grad_des(vn, hd, w, gs, tas, des_mach) * M.clb_factor(ctx.des_tab, hd, M.DES_DEFAULT); kind = "idle"
    end
    local hn = hd + grad * ds
    -- v41: slower descent speed than the cruise speed (e.g. 320 kt cruise at FL170, 270 kt descent):
    -- the speed is bled off in the first part of the descent (deceleration segment at about 30 % of
    -- the idle gradient, as the real FMC plans it) instead of a level deceleration at the cruise level
    -- several miles before T/D
    if kind == "idle" and not P.no_top_decel then
      local vcz = math.min(PF.mach_to_cas(crz_mach, crz), (P.crz_cas or 320))
      if vcz > vn + 3 then
        local Gt = PF.idle_grad(vn, hd, w, gs, tas) * M.clb_factor(ctx.des_tab, hd, M.DES_DEFAULT)
        local vtt = PF.tas(vn, hd) * 1.6878
        local dvpn = 0.85 * 0.70 * Gt * 32.174 / max(200, vtt) / 1.6878 * (tas / gs)
        local gdec = Gt * 0.30
        local dist_dec = (vcz - vn) / max(0.5, dvpn)
        local dh_dec = gdec * dist_dec
        if hn + dh_dec >= crz then
          local h1 = max(hd, crz - dh_dec)
          local s1 = (grad > 1) and (sd - (h1 - hd) / grad) or sd
          if h1 > hd + 1 then pts[#pts + 1] = { s = s1, h = h1, v = vn, k = "idle" } end
          local dist = (crz - h1) / max(30, gdec)
          local vtop = math.min(vcz, vn + dvpn * dist)
          local st = s1 - dist
          pts[#pts + 1] = { s = st, h = crz, v = vtop, k = "decel" }
          s_tod = st
          if vtop < vcz - 3 then
            -- (not enough height for all of it: the rest level at the cruise level before T/D)
            local G = PF.idle_grad(vtop, crz, w, gs, tas) * M.clb_factor(ctx.des_tab, crz, M.DES_DEFAULT)
            local v1, v2 = PF.tas(vtop, crz) * 1.6878, PF.tas(vcz, crz) * 1.6878
            local dkin = (v2 * v2 - v1 * v1) / (2 * 32.174)
            local dl = dkin / max(50, 0.85 * G) * (gs / max(60, tas))
            P.s_decel = st - dl
            P.v_tod = vtop
            pts[#pts + 1] = { s = st - dl, h = crz, v = vcz, k = "decel" }
          end
          break
        end
      end
    end
    -- mach above crossover: speed is CAS equivalent already
    if hn >= crz then
      local frac = (crz - hd) / max(1e-6, hn - hd)
      local st = sd - ds * frac
      local vtd = vd + (vn - vd) * frac
      pts[#pts + 1] = { s = st, h = crz, v = vtd, k = kind }
      s_tod = st
      -- the descent starts slower than the cruise speed (low cruise level, speed limit below 10000 or
      -- a speed restriction): decelerate level at the cruise level before T/D (idle, energy based)
      local vcrz = math.min(PF.mach_to_cas(crz_mach, crz), (P.crz_cas or 320))
      if vtd < vcrz - 3 then
        local G = PF.idle_grad(vtd, crz, w, gs, tas) * M.clb_factor(ctx.des_tab, crz, M.DES_DEFAULT)
        local v1, v2 = PF.tas(vtd, crz) * 1.6878, PF.tas(vcrz, crz) * 1.6878
        -- kinetic energy to lose (ft) / idle energy loss per nm (ft/nm, 85 % margin) -> nm over ground
        local dkin = (v2 * v2 - v1 * v1) / (2 * 32.174)
        local dist = dkin / max(50, 0.85 * G) * (gs / max(60, tas))
        P.s_decel = st - dist
        P.v_tod = vtd
        pts[#pts + 1] = { s = st - dist, h = crz, v = vcrz, k = "decel" }
      end
      break
    end
    sd, hd, vd = sn, hn, vn
    pts[#pts + 1] = { s = sd, h = hd, v = vd, k = kind }
    if c and abs(c.s - sd) < 1e-6 then
      -- apply constraint
      local lo, hi = RT.alt_lo(c.alt), RT.alt_hi(c.alt)
      -- a lower limit above the cruise level cannot be met in the descent: the cruise level applies
      if lo and lo > crz then lo = crz end
      if hi and hi > crz then hi = nil end
      local fixh = nil
      if hi and hd > hi then fixh = hi end
      if lo and hd < lo then fixh = lo end
      if fixh then
        local a = pts[anchor]
        if hi and fixh == hi and a.h > fixh + 1 then
          -- v41: an at-or-below restriction cutting the idle path where the path from the next
          -- restriction comes up higher than it allows (RIVET5: TAXAT 9000B, BOOGI 6000A 6 nm on):
          -- after the fix a continuous shallower descent (JOIN_FRAC of the idle gradient) until it
          -- meets the idle path again. Before: a straight line from the fix to the next restriction
          -- - here a climb - and the guidance levelled at the restriction until the fix.
          local gj = M.JOIN_FRAC * PF.idle_grad_des(vd, fixh, w, gs, tas, des_mach) * M.clb_factor(ctx.des_tab, fixh, M.DES_DEFAULT)
          -- (no steeper than the steepest straight line to a point of the path up to the next
          -- restriction it is held at - its lower limit - or E/D, so it always meets the path there)
          local gbest = -1e9
          for k = #pts - 1, 1, -1 do
            local p = pts[k]
            local g_ = (fixh - p.h) / max(0.05, p.s - sd)
            if g_ > gbest then gbest = g_ end
            if p.cns and p.lo and p.h <= p.lo + 50 then break end
          end
          gj = max(0, min(gj, gbest))
          pts[#pts].h = fixh
          for k = #pts - 1, 1, -1 do
            local p = pts[k]
            local line = fixh - gj * (p.s - sd)
            if p.h > line + 1 then
              if p.lo and line < p.lo then line = p.lo end
              p.h = line; p.k = "geo"
            else break end
          end
        else
          for k = anchor + 1, #pts do
            local p = pts[k]
            local fr = (a.s - p.s) / max(1e-6, a.s - sd)
            p.h = a.h + (fixh - a.h) * fr
            p.k = "geo"
          end
        end
        hd = fixh
      end
      pts[#pts].cns = true
      pts[#pts].lo = lo
      anchor = #pts
      ci_ = ci_ + 1
    end
    if sd <= s_floor then break end
  end
  -- ascending order
  local des = {}
  for k = #pts, 1, -1 do des[#des + 1] = pts[k] end
  P.des = des
  P.s_tod = s_tod or (des[1] and des[1].s) or total
  P.crz_eff = crz
  -- T/C after T/D (short route)
  if P.s_toc and P.s_tod and P.s_toc > P.s_tod and in_climb then
    local best = nil
    for _, cp in ipairs(clb) do
      local dh = M.path_alt(P, cp.s)
      if dh and cp.h >= dh then best = cp break end
    end
    if best then
      P.s_toc = best.s; P.s_tod = best.s; P.crz_eff = best.h; P.unable_crz = true
    end
  end
  -- leg predictions -------------------------------------------------------------------------------------
  local crz_tas = PF.tas_mach(crz_mach, P.crz_eff)
  local crz_gs = max(100, crz_tas + wind)
  local t_toc = clb[#clb].t or 0
  local f_toc = clb[#clb].f or fuel
  local crz_dist = max(0, P.s_tod - P.s_toc)
  local t_tod = t_toc + crz_dist / crz_gs * 3600
  local f_tod = f_toc - PF.ff_cruise(w, crz_mach, P.crz_eff) * crz_dist / crz_gs
  -- descent time/fuel along des pts
  local dt_acc, df_acc = {}, {}
  local tt, ff = t_tod, f_tod
  for k = 1, #des do
    if k > 1 then
      local a, b = des[k - 1], des[k]
      local ds_ = b.s - a.s
      local gs = max(100, PF.tas((a.v + b.v) / 2, (a.h + b.h) / 2) + wind_at((a.h + b.h) / 2))
      tt = tt + ds_ / gs * 3600
      ff = ff - PF.ff_descent() * ds_ / gs
    end
    dt_acc[k], df_acc[k] = tt, ff
  end
  local function interp_pts(list, sp, fld)
    if #list == 0 then return nil end
    if sp <= list[1].s then return list[1][fld], 1 end
    for k = 2, #list do
      if sp <= list[k].s then
        local a, b = list[k - 1], list[k]
        local fr = (sp - a.s) / max(1e-6, b.s - a.s)
        return a[fld] + (b[fld] - a[fld]) * fr, k
      end
    end
    return list[#list][fld], #list
  end
  for i = 1, n do
    local l = legs[i]
    local sp = l.cum
    local pr = {}
    if sp and sp >= s_now - 0.01 and i <= last then
      if sp <= P.s_toc then
        pr.alt = interp_pts(clb, sp, "h") or crz
        local v = interp_pts(clb, sp, "v")
        pr.cas = v
        pr.t = interp_pts(clb, sp, "t")
        pr.f = interp_pts(clb, sp, "f")
        pr.ph = "CLB"
      elseif sp < P.s_tod then
        pr.alt = P.crz_eff
        pr.mach = crz_mach
        pr.t = t_toc + (sp - P.s_toc) / crz_gs * 3600
        pr.f = f_toc - PF.ff_cruise(w, crz_mach, P.crz_eff) * (sp - P.s_toc) / crz_gs
        pr.ph = "CRZ"
      else
        local hh, k = interp_pts(des, sp, "h")
        pr.alt = hh
        pr.cas = interp_pts(des, sp, "v")
        if k then
          local a = des[max(1, k - 1)]
          local tb, fb = dt_acc[max(1, k - 1)], df_acc[max(1, k - 1)]
          local gs = max(100, PF.tas(pr.cas or 250, hh or 0) + wind)
          pr.t = (tb or t_tod) + max(0, sp - a.s) / gs * 3600
          pr.f = (fb or f_tod) - PF.ff_descent() * max(0, sp - a.s) / gs
        end
        pr.ph = "DES"
      end
      if pr.cas and pr.alt then
        local mc = PF.cas_to_mach(pr.cas, pr.alt)
        local lim = (pr.ph == "CLB") and clb_mach or des_mach
        if mc >= lim - 0.002 then pr.mach = lim; pr.cas = nil end
      end
    end
    P.legs[i] = pr
  end
  P.t_tod, P.t_toc = t_tod, t_toc
  -- destination predictions
  local lastl = legs[last]
  if lastl then
    P.dest_t = P.legs[last] and P.legs[last].t
    P.dest_f = P.legs[last] and P.legs[last].f
  end
  P.ok = true
  return P
end

function M.path_alt(P, s)
  local d = P.des
  if not d or #d == 0 then return nil end
  if s <= d[1].s then return d[1].h, 0, d[1] end
  for k = 2, #d do
    if s <= d[k].s then
      local a, b = d[k - 1], d[k]
      local fr = (s - a.s) / max(1e-6, b.s - a.s)
      local grad = (a.h - b.h) / max(1e-6, b.s - a.s)   -- ft per nm (positive = descending)
      return a.h + (b.h - a.h) * fr, grad, a, b
    end
  end
  return d[#d].h, 0, d[#d]
end
function M.path_speed(P, s)
  local d = P.des
  if not d or #d == 0 then return nil end
  if s <= d[1].s then return d[1].v end
  for k = 2, #d do
    if s <= d[k].s then
      local a, b = d[k - 1], d[k]
      local fr = (s - a.s) / max(1e-6, b.s - a.s)
      return a.v + (b.v - a.v) * fr, a.k
    end
  end
  return d[#d].v
end

-- GUIDANCE --------------------------------------------------------------------------------------------
--[[ st: persistent vnav state;  inp: {
       engaged, s_now, alt, ias, mach, gs, vs, mcp_alt, mcp_spd, spd_intv, onground, ra, agl, flaps (deg),
       alt_push (counter edge), crz, phase, accel_ht, thr_red, v2, w, ci, active leg index, legs, on_final,
       vmo, des_now (edge) }
   returns out: { phase, sub ('SPD','PTH','ALT'), pitch ('SPD','VS','ALT'), at ('THR_REF','SPD','IDLE','HOLD'),
       spd (cas), mach, ism, alt_tgt, vs, msg = {...}, crz (new crz alt if changed), path_dev, ...} ]]
function M.new_state()
  return { phase = "TO", sub = "SPD", cap = false, des_now = false, crz_des = false, crz_clb = false,
    idle_reached = false, last_alt_push = 0, last_des_now = 0, deleted = {}, drag_req = false }
end

local function next_climb_constraint(plan, s_now, dreg)
  for i = 1, dreg - 1 do
    local l = plan.legs[i]
    if l.cum and l.cum >= s_now - 0.2 and l.alt and not l.alt_deleted then
      local hi = RT.alt_hi(l.alt)
      if hi then return hi, i end
    end
  end
  return nil
end
M.next_climb_constraint = next_climb_constraint

local function next_des_constraint(plan, s_now, dreg, last)
  for i = 1, last do
    local l = plan.legs[i]
    if l.cum and l.cum >= s_now - 0.2 and l.alt and not l.alt_deleted and (l.seg ~= "SID" and l.seg ~= "RWY") then
      -- (v41: only a lower limit is a floor - an at-or-below restriction such as TAXAT 9000B is not
      -- levelled at; it used to hold the aircraft at 9000 ft until the fix, far above the path)
      local lo = RT.alt_lo(l.alt)
      if lo then return lo, i end
    end
  end
  return nil
end

M.DECEL_LVL = 0.6   -- kt/s, level deceleration used to meet speed restrictions
function M.guide(st, P, plan, inp)
  local out = { msg = {} }
  local alt, ias = inp.alt, inp.ias
  local crz = inp.crz or 0
  local mcp = inp.mcp_alt
  local legs = plan.legs
  local dreg = descent_region_start(plan)
  local last = RT.last_route_index(plan)
  local w = inp.w or 300000
  local clb_cas, clb_mach = P.clb_cas or 320, P.clb_mach or 0.84
  local des_cas, des_mach = P.des_cas or 300, P.des_mach or 0.84
  local crz_mach = P.crz_mach or 0.85
  local vmo = inp.vmo or 365
  local s = inp.s_now or 0
  local push = (inp.alt_push or 0) ~= st.last_alt_push
  st.last_alt_push = inp.alt_push or 0
  local desnow = (inp.des_now or 0) ~= st.last_des_now
  st.last_des_now = inp.des_now or 0

  -- PHASE -------------------------------------------------------------------------------------------
  if inp.onground then
    if st.phase ~= "TO" and (inp.gs or 0) < 40 then st.phase = "TO" end
    st.sub = "SPD"; st.des_now = false; st.crz_des = false
  else
    if st.phase == "TO" and inp.agl and inp.agl > (inp.thr_red or 1500) then st.phase = "CLB" end
    if st.phase == "TO" and alt > crz - 100 and crz > 0 then st.phase = "CLB" end
    if st.phase == "CLB" and crz > 0 and abs(alt - crz) < 60 and abs(inp.vs or 0) < 300 then st.phase = "CRZ" end
    if st.phase == "CLB" and crz > 0 and alt > crz + 60 then st.phase = "CRZ" end
    if st.phase == "CRZ" and crz > 0 and alt < crz - 600 and not st.crz_des and not st.des_now and (s < (P.s_tod or 1e9) - 5) then
      st.phase = "CLB"
    end
    if inp.goaround then st.phase = "CLB"; st.des_now = false end
  end
  local s_tod = P.s_tod or 1e9
  local to_tod = s_tod - s

  -- ALTITUDE INTERVENTION (ALT knob push) --------------------------------------------------------------
  if push and inp.engaged and not inp.onground then
    if st.phase == "CLB" or st.phase == "TO" then
      if mcp > crz + 50 then
        out.new_crz = mcp; crz = mcp
      else
        -- delete next climb constraint below MCP altitude
        local hi, i = next_climb_constraint(plan, s, dreg)
        if hi and hi < mcp and i then legs[i].alt_deleted = true; out.deleted = i end
      end
      st.sub = "SPD"
    elseif st.phase == "CRZ" then
      if mcp > alt + 50 then
        out.new_crz = mcp; crz = mcp; st.phase = "CLB"; st.sub = "SPD"; st.crz_clb = true
      elseif mcp < alt - 50 then
        if to_tod < 50 then
          st.des_now = true; st.phase = "DES"
        else
          out.new_crz = mcp; crz = mcp; st.crz_des = true
        end
      end
    elseif st.phase == "DES" then
      if mcp < alt - 50 then
        local lo, i = next_des_constraint(plan, s, dreg, last)
        if lo and lo > mcp and i and alt - lo < 300 then legs[i].alt_deleted = true; out.deleted = i
        elseif s < s_tod then st.des_now = true end
      end
      st.resume = true
    end
  end
  if desnow and not inp.onground and (st.phase == "CRZ" or st.phase == "CLB") then
    st.des_now = true; st.phase = "DES"
  end
  local tod_lead = max(0.1, (inp.gs or 450) * M.TOD_LEAD_S / 3600)
  if st.phase == "CRZ" and s >= s_tod - tod_lead and mcp < alt - 300 and not st.crz_des then
    st.phase = "DES"
  end
  if st.phase == "CRZ" and s >= s_tod - 0.1 and mcp >= alt - 300 then
    out.msg.reset_mcp = true
  elseif st.phase == "CRZ" and to_tod < (inp.gs or 450) / 30 and mcp >= alt - 300 then
    out.msg.reset_mcp = true   -- ~2 minutes before T/D
  end
  if st.phase == "DES" and st.des_now and alt <= (M.path_alt(P, s) or 0) + 150 then st.des_now = false end
  if st.crz_des and abs(alt - crz) < 80 then st.crz_des = false end

  -- SPEED TARGET ----------------------------------------------------------------------------------------
  local tgt, mach_lim
  local flapsdeg = inp.flaps or 0
  if st.phase == "TO" or (st.phase == "CLB" and (inp.agl or 99999) < (inp.accel_ht or 1000) and not inp.goaround) then
    tgt = (inp.v2 or 160) + 10
    if inp.toga_spd then tgt = inp.toga_spd end
    mach_lim = clb_mach
  elseif st.phase == "CLB" then
    tgt = clb_cas
    if alt < (inp.clb_trans_alt or 10000) - 50 then tgt = min(tgt, inp.clb_trans_spd or 250) end
    if inp.clb_restr and alt < inp.clb_restr.alt then tgt = min(tgt, inp.clb_restr.spd) end
    for i = 1, dreg - 1 do
      local l = legs[i]
      if l.cum and l.cum >= s - 0.2 and l.spd and (l.spd.d == "-" or l.spd.d == "@") then tgt = min(tgt, l.spd.v) end
    end
    mach_lim = clb_mach
  elseif st.phase == "CRZ" then
    tgt = 999; mach_lim = crz_mach
    tgt = min(tgt, PF.mach_to_cas(crz_mach, alt), P.crz_cas or 320)
    -- level deceleration before a low T/D
    if P.s_decel and P.v_tod and s >= P.s_decel then tgt = min(tgt, P.v_tod); out.crz_decel = true end
  else
    local pv = M.path_speed(P, s)
    tgt = pv or des_cas
    -- (v43: started a few seconds before T/D - the T/D lead for the pitch-over - the speed stays at the
    -- path's T/D speed (the cruise speed when the descent starts with a deceleration): stepping to the
    -- descent speed there pulled the thrust to idle level before T/D, the aircraft then fell behind
    -- the planned deceleration and the A/T came in with full thrust)
    if st.des_now then tgt = des_cas
    elseif s < (P.s_tod or 0) then tgt = (P.des and P.des[1] and P.des[1].v) or des_cas end
    if alt < (inp.des_trans_alt or 10000) + 50 then tgt = min(tgt, inp.des_trans_spd or 240) end
    mach_lim = des_mach
  end
  -- takeoff flap retraction: after the acceleration height VNAV holds the retraction speed of the
  -- flaps set (the maneuver speed of the next position: F20/10 -> "5" bug, F5 -> "1", F1 -> "UP");
  -- once the flaps are up it accelerates to the climb speed (speed restrictions, 250 kt below 10000 ft)
  local flap_cap = nil
  if (st.phase == "CLB" or st.phase == "CRZ") and not inp.goaround and (inp.agl or 99999) >= (inp.accel_ht or 1000) and not inp.spd_intv then
    local vref30 = PF.vref30(inp.w or 300000)
    local function vf(f)
      local v = inp.vf and tonumber(inp.vf[f])
      if v and v > 100 then return v end
      local off = { [0] = 80, [1] = 60, [5] = 40, [10] = 20, [20] = 20 }
      return vref30 + (off[f] or 80)
    end
    if flapsdeg > 0 and alt < 20000 then
      local nxt = (flapsdeg >= 10) and 5 or ((flapsdeg >= 5) and 1 or 0)
      if flapsdeg >= 20 then nxt = 10 end
      local ft = vf(nxt) + M.FLAP_RETR_MARGIN
      -- capped by the climb speed: speed restrictions and 250 kt below 10000 ft (a restriction
      -- below the flap retraction speed is flown - the pilot leaves the flaps out)
      tgt = min(ft, tgt)
      flap_cap = min(ft, (PF.FLAP_PLACARD[flapsdeg] or 365) - 5)
      out.flap_sched = nxt
    end
  end
  -- flap placard / maneuver speeds
  local plac = PF.FLAP_PLACARD[flapsdeg] or 365
  if flapsdeg > 0 then tgt = min(tgt, plac - 5) end
  if inp.vman and flapsdeg > 0 and st.phase == "DES" then tgt = max(min(tgt, inp.vman), inp.vman) end
  local cas, ism = speed_cas_at(tgt, mach_lim, alt)
  if st.phase == "CRZ" then
    -- cruise at the FMC cruise Mach; below the Mach/CAS crossover the cruise CAS limit applies
    -- (a low cruise level must not give a near-Vmo target)
    local c = PF.mach_to_cas(crz_mach, alt)
    local lim = P.crz_cas or 320
    if c > lim then cas = lim; ism = false else cas = c; ism = true end
    if alt < M.SPD_LIM_ALT and cas > M.SPD_LIM_KT then cas = M.SPD_LIM_KT; ism = false end
    if out.crz_decel and P.v_tod and P.v_tod < cas then cas = P.v_tod; ism = false end
    -- (flaps still out at the cruise level: their retraction speed / placard)
    if flap_cap and cas > flap_cap then cas = flap_cap; ism = false end
  end
  -- speed restrictions ahead in level flight and in the climb (a low cruise level, a level segment
  -- before the approach - e.g. GLENN max 185 kt on the YBBN ILS 01R): VNAV slows down in time to
  -- cross the fix at the restriction (level deceleration about 0.6 kt/s, plus a 2 nm margin). In the
  -- descent the descent path already plans the deceleration.
  if (st.phase == "CRZ" or st.phase == "CLB") and not inp.spd_intv and inp.s_now and not inp.goaround then
    local s_now = inp.s_now
    local gsk = max(120, inp.gs or 250)
    local iasn = ias or cas
    for i = max(1, plan.active or 1), #legs do
      local l = legs[i]
      if l.cum and l.cum >= s_now - 0.05 then
        local togo = l.cum - s_now
        if togo > 40 then break end
        -- (restrictions after T/D: the descent path plans their deceleration)
        if st.phase == "CRZ" and P.s_tod and l.cum > P.s_tod + 0.5 then break end
        local sp = l.spd
        if sp and sp.v and (sp.d == "-" or sp.d == "@") and sp.v < cas then
          local need = max(0, max(iasn, cas) - sp.v) / M.DECEL_LVL * gsk / 3600 + 2.0
          if togo <= need then cas = sp.v; ism = false; out.restr_decel = true end
        end
      end
    end
  end
  -- turn speed limit (glue): a fly-by turn ahead that its geometry only allows at a lower speed
  if inp.turn_spd and cas > inp.turn_spd and (st.phase == "CLB" or st.phase == "CRZ" or st.phase == "DES") then
    cas = inp.turn_spd; ism = false; out.turn_lim = true
  end
  cas = min(cas, vmo - 10)
  -- v34: climb speed target increases are rate-limited (about 1 kt/s) so the climb continues while it
  -- accelerates (e.g. 250 kt -> climb speed at 10000 ft) instead of levelling off to accelerate
  if st.phase == "CLB" and not inp.spd_intv and not inp.onground then
    local d = min(1, max(0, inp.dt or 0.05))
    local r = st.clb_spd_r or (ias or cas)
    if cas > r then cas = min(cas, r + 1.0 * d); ism = false end
    st.clb_spd_r = cas
  else
    st.clb_spd_r = nil
  end
  if inp.spd_intv and inp.mcp_spd then cas = inp.mcp_spd; ism = false end
  out.spd = cas
  out.ism = ism
  out.mach = ism and mach_lim or PF.cas_to_mach(cas, alt)
  if st.phase == "CRZ" and not inp.spd_intv and ism then out.mach = crz_mach end

  -- LEVEL TARGET ----------------------------------------------------------------------------------------
  local floor_mcp = mcp
  if inp.on_final and mcp > alt then floor_mcp = -1000 end
  local function capture(level, kind)
    out.alt_tgt = level
    out.level_kind = kind
  end

  -- SUB MODES ----------------------------------------------------------------------------------------
  local sub, pitch, at = st.sub, "SPD", "THR_REF"
  local vs_cmd = 0
  if st.phase == "TO" or st.phase == "CLB" then
    local lvl, lkind = crz, "CRZ"
    local hi = next_climb_constraint(plan, s, dreg)
    -- (a restriction already well below the aircraft is not descended to in the climb)
    if hi and hi < lvl and hi >= alt - 300 then lvl, lkind = hi, "CNSTR" end
    if mcp < lvl then lvl, lkind = mcp, "MCP" end
    if lvl < alt - 200 and lkind == "MCP" then
      -- MCP set below us during climb: VNAV ALT hold present
      lvl = alt
    end
    capture(lvl, lkind)
    if st.sub == "ALT" and lkind == "MCP" and abs(alt - mcp) < 250 then
      sub = "ALT"; pitch = "ALT"; at = "SPD"
    elseif abs(alt - lvl) < 100 + abs(inp.vs or 0) * 0.08 then
      if lkind == "MCP" then sub = "ALT" else sub = "PTH" end
      pitch = "ALT"; at = "SPD"
    elseif alt < lvl and (inp.vs or 0) > 300 and 60 * math.sqrt(2 * 1.6 * (lvl - alt)) < (inp.vs or 0) then
      -- (v54) the level capture has started (the climb rate is above the capture curve - from 1300 ft
      -- below at 4000 fpm): the thrust holds the speed during the capture (A/T SPD, as on the 744)
      -- instead of fixed climb thrust - the speed fell 35 kt below target on the YAMB departure
      sub = "PTH"; pitch = "ALT"; at = "SPD"
    elseif alt < lvl then
      sub = "SPD"; pitch = "SPD"; at = "THR_REF"
    else
      sub = "PTH"; pitch = "ALT"; at = "SPD"
    end
    if st.sub == "ALT" and lkind == "MCP" and mcp > alt + 250 and not push then
      sub = "ALT"; pitch = "ALT"; at = "SPD"   -- 744: push the selector to resume the climb
      out.alt_tgt = st.hold_alt or alt
    end
    if sub == "ALT" then st.hold_alt = out.alt_tgt end
  elseif st.phase == "CRZ" then
    local lvl = crz
    if mcp > alt + 50 and mcp < crz then lvl = mcp end
    if st.crz_des then
      lvl = max(crz, floor_mcp)
      capture(lvl, "CRZ")
      if alt > lvl + 100 then
        sub = "PTH"; pitch = "VS"; vs_cmd = -1250; at = "SPD"
      else sub = "PTH"; pitch = "ALT"; at = "SPD" end
    else
      if floor_mcp > alt + 50 and mcp < crz then
        capture(alt, "MCP"); sub = "ALT"; pitch = "ALT"; at = "SPD"
      else
        capture(crz, "CRZ"); sub = "PTH"; pitch = "ALT"; at = "SPD"
        -- (v54: only well below the cruise altitude - a new, higher CRZ ALT. A small deviation is
        -- corrected by the level hold: switching to a climb at climb thrust 150 ft below the level
        -- threw the aircraft 500 ft above it and the level hold kept hunting)
        if alt < crz - 500 and mcp >= crz - 50 then sub = "SPD"; pitch = "SPD"; at = "THR_REF" end
      end
    end
  else -- DES
    local pa, grad = M.path_alt(P, s)
    pa = pa or alt
    grad = grad or 0
    local gs = max(100, inp.gs or 250)
    local path_vs = -grad * gs / 60
    -- v41: feed-forward from the path over the next 8 s (anticipates a gradient change - T/D, the end
    -- of a shallow segment - instead of reacting once the aircraft is off the path)
    do
      local dl = gs * 8 / 3600
      local pa2 = M.path_alt(P, s + dl)
      if pa2 and pa then path_vs = min(0, (pa2 - pa) / dl * gs / 60) end
    end
    out.path_dev = alt - pa
    out.path_vs = path_vs
    local pv, pk = M.path_speed(P, s)
    local idle_seg = (pk == "idle" or pk == "decel" or pk == nil) and grad > 150
    local level_floor = max(floor_mcp, -2000)
    capture(level_floor, "MCP")
    if alt <= floor_mcp + 100 + abs(inp.vs or 0) * 0.08 and floor_mcp > -1000 then
      sub = "ALT"; pitch = "ALT"; at = "SPD"; out.alt_tgt = floor_mcp
    elseif st.des_now and alt > pa + 150 then
      sub = "PTH"; pitch = "VS"; vs_cmd = -1250; at = "SPD"
    elseif inp.spd_intv and alt > pa - 300 then
      sub = "SPD"; pitch = "SPD"; at = "IDLE"
    -- (VNAV SPD only when well above the path: as on the 744, VNAV PTH stays engaged above the path -
    -- the idle descent is flown a little faster to regain it, DRAG REQUIRED if that is not enough. The
    -- old 400 ft limit reverted to VNAV SPD at almost every T/D and restriction: OP DES-like hunting)
    elseif alt > pa + M.SPD_ABOVE or (st.sub == "SPD" and alt > pa + M.SPD_ABOVE_EXIT) then
      sub = "SPD"; pitch = "SPD"; at = "IDLE"
      out.spd_des_floor = true          -- VNAV SPD descent: keep descending while slowing
      -- DRAG REQUIRED: well above the path and not able to get back to it at idle before the
      -- next constraint / E/D (idle path gradient vs the gradient needed)
      local s_next = (P.ed_s or s) - s
      for _, d in ipairs(P.des or {}) do if d.cns and d.s > s + 0.5 then s_next = min(s_next, d.s - s) break end end
      local need = (alt - pa) / max(1, s_next)
      if (alt - pa > 500 and need > 80) or alt - pa > 1500 then out.msg.drag = (inp.spdbrk or 0) < 0.1 end
    else
      -- VNAV PTH (v41). The nose follows the path; on idle and deceleration segments the thrust stays
      -- at idle (IDLE -> HOLD) and the speed floats about the planned path speed - as on the 744.
      --  * slow (the path is shallower than this aircraft's idle descent, or below the path): A/T SPD
      --    adds the thrust needed and stays in SPD until it has brought the thrust back to idle by
      --    itself with the speed on target (the path steepened again) - no IDLE / SPD cycling
      --  * fast (the path is steeper than the idle descent): the nose does not steepen beyond the path
      --    rate to correct a high deviation, DRAG REQUIRED; well above the path VNAV SPD (above)
      -- (v40 and before: speed on pitch with a path-trimmed target and thrust once 400 ft low - the
      -- level-then-sink cycles seen on the RIVET5 descent)
      sub = "PTH"; pitch = "VS"
      local dev = alt - pa
      local tnow = inp.t or 0
      local sb = (inp.spdbrk or 0) > 0.1
      -- path tracking: feed-forward of the path vertical speed plus a correction on the predicted
      -- error 12 s ahead (damps the response), authority at least 1200 fpm / 80 % of the path rate
      local vsn = inp.vs or path_vs
      local perr = (pa + 0.2 * path_vs) - (alt + 0.2 * vsn)
      -- (below the path: up to the full path rate - level off well below it until the path comes
      -- down; above it: a gentler extra descent, the idle path itself is steep)
      local x = max(-1, min(1, perr / 300))
      local auth = (x > 0) and max(1200, abs(path_vs)) or max(800, 0.35 * abs(path_vs))
      local corr = math.sin(x * pi / 2) * auth
      -- fast: a correction down to the path is faded out between 8 and 15 kt over the target
      if corr < 0 and ias > cas + 8 then corr = corr * max(0, min(1, (cas + 15 - ias) / 7)) end
      vs_cmd = path_vs + corr
      -- speed protection: more than 10 kt fast (this aircraft glides better than planned) the descent
      -- is made up to 25 % shallower than the path (from 12 kt fast) - the excess goes into height above the path
      -- (VNAV SPD beyond SPD_ABOVE), DRAG REQUIRED shown
      if ias > cas + 12 and not sb then vs_cmd = vs_cmd * (1 - min(0.25, (ias - cas - 12) * 0.03)) end
      vs_cmd = max(min(vs_cmd, 0), -6000)
      local dt_ = max(0.01, inp.dt or 0.1)
      if st.vs_slew == nil or st.sub ~= "PTH" then st.vs_slew = inp.vs or vs_cmd end
      st.vs_slew = st.vs_slew + max(-500 * dt_, min(500 * dt_, vs_cmd - st.vs_slew))
      vs_cmd = st.vs_slew
      local idle_like = idle_seg or pk == "decel"
      if idle_like then
        at = "IDLE"
        -- (on a deceleration segment the target itself falls: slowing faster than planned is fine)
        -- (v43: on a deceleration segment no thrust unless well slow - 25 kt: slowing a little faster
        -- than planned is harmless, and the speed target catches up)
        local slow_lim = (pk == "decel") and 25 or M.THR_SLOW
        if not st.thr and not sb and (ias < cas - slow_lim or dev < -M.THR_BELOW) then
          st.thr = true; st.thr_idle_t = nil
        end
        if st.thr then
          at = "SPD"
          -- back to IDLE once the A/T itself has had the thrust at idle for a few seconds with the
          -- speed on target (path steeper again), or the aircraft is high / fast
          if inp.thr_idle and ias >= cas - 2 and dev > -M.THR_BELOW then st.thr_idle_t = st.thr_idle_t or tnow else st.thr_idle_t = nil end
          if (st.thr_idle_t and tnow - st.thr_idle_t > 6) or dev > 200 or ias > cas + 6 then
            st.thr = false; st.thr_idle_t = nil; at = "IDLE"
          end
        end
      else
        at = "SPD"; st.thr = false
      end
      st.idle_thr = st.thr
      -- path speed protection: DRAG REQUIRED once the speed runs 10 kt over
      if ias > cas + 10 and ias < vmo - 11 and not sb and dev > -200 then out.msg.drag = true end
      if dev > 300 and ias > cas + 5 and not sb then out.msg.drag = true end
      if ias > vmo - 11 then sub = "SPD"; pitch = "SPD"; at = "IDLE"; out.msg.drag = true end
    end
    -- capture of E/D / level segments handled by the VS law (grad 0)
    -- v41: in every descent sub-mode, never below the lower limit of the next restriction before the
    -- fix (at-or-above / window restrictions): level at it if the aircraft gets there early
    if sub ~= "ALT" then
      local lo = next_des_constraint(plan, s, dreg, last)
      if lo and lo > level_floor and alt < lo + 60 + abs(inp.vs or 0) * 0.06 then
        capture(lo, "CNSTR"); sub = "ALT"; pitch = "ALT"; at = "SPD"; vs_cmd = 0; st.thr = false
      end
    end
  end
  -- idle -> hold logic: once thrust reached idle the A/T goes HOLD
  if at == "IDLE" then
    if st.at_prev == "IDLE" or st.at_prev == "HOLD" then
      if (inp.thr_idle or false) then at = "HOLD" end
      if st.at_prev == "HOLD" then at = "HOLD" end
    end
  end
  if at ~= "IDLE" and at ~= "HOLD" then st.at_prev = at else st.at_prev = at end
  st.sub = sub
  out.phase = st.phase
  out.sub = sub
  out.pitch = pitch
  out.at = at
  out.vs = vs_cmd
  out.crz = crz
  out.to_tod = to_tod
  return out
end

M.SPD_BELOW = 500     -- ft below the idle path that pitch may use to regain speed
M.SPD_ABOVE = 1200    -- ft above the path before VNAV SPD (idle, speed on pitch) takes over
M.SPD_ABOVE_EXIT = 500 -- ft above the path where VNAV SPD hands back to VNAV PTH
M.TOD_LEAD_S = 8      -- s before T/D the descent starts (the pitch-over takes time: on the path at T/D)
M.FLAP_RETR_MARGIN = 5 -- kt above the next flap position's maneuver speed (VNAV flap retraction speed)
M.SPD_LIM_ALT = 10000  -- ft: cruise below this is limited to SPD_LIM_KT
M.SPD_LIM_KT = 250
M.IDLE_DES = true      -- idle descent with speed on pitch (path-trimmed speed target)
M.IDLE_BAND_LO = 400   -- ft below the path before thrust is used (idle and level off at the next restriction instead)
M.IDLE_SPD_LO = 12     -- kt the idle descent speed target may go below the descent speed
M.IDLE_FT_PER_KT = 30  -- path deviation per kt of speed target trim
M.JOIN_FRAC = 0.6      -- gradient (fraction of idle) joining the idle path after an at-or-below restriction
M.THR_SLOW = 5         -- kt slow on an idle path before the A/T adds thrust (A/T SPD)
M.THR_BELOW = 250      -- ft below the path before the A/T adds thrust
B744VNAV = M
