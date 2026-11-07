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
M.DES_DEFAULT = 0.92   -- X-Plane's 747 glides a little better than the book idle gradient (until learned)
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
      grad = PF.idle_grad(vn, hd, w, gs, tas) * M.clb_factor(ctx.des_tab, hd, M.DES_DEFAULT); kind = "idle"
    end
    local hn = hd + grad * ds
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
        for k = anchor + 1, #pts do
          local p = pts[k]
          local fr = (a.s - p.s) / max(1e-6, a.s - sd)
          p.h = a.h + (fixh - a.h) * fr
          p.k = "geo"
        end
        hd = fixh
      end
      pts[#pts].cns = true
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
      local gs = max(100, PF.tas((a.v + b.v) / 2, (a.h + b.h) / 2) + wind)
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
      return RT.alt_lo(l.alt) or RT.alt_hi(l.alt), i
    end
  end
  return nil
end

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
  if st.phase == "CRZ" and s >= s_tod - 0.1 and mcp < alt - 300 and not st.crz_des then
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
    if st.des_now or s < (P.s_tod or 0) then tgt = des_cas end
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
  cas = min(cas, vmo - 10)
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
        if alt < crz - 150 and mcp >= crz - 50 then sub = "SPD"; pitch = "SPD"; at = "THR_REF" end
      end
    end
  else -- DES
    local pa, grad = M.path_alt(P, s)
    pa = pa or alt
    grad = grad or 0
    local gs = max(100, inp.gs or 250)
    local path_vs = -grad * gs / 60
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
    elseif alt > pa + 400 or (st.sub == "SPD" and alt > pa + 150) then
      sub = "SPD"; pitch = "SPD"; at = "IDLE"
      out.spd_des_floor = true          -- VNAV SPD descent: keep descending while slowing
      -- DRAG REQUIRED: well above the path and not able to get back to it at idle before the
      -- next constraint / E/D (idle path gradient vs the gradient needed)
      local s_next = (P.ed_s or s) - s
      for _, d in ipairs(P.des or {}) do if d.cns and d.s > s + 0.5 then s_next = min(s_next, d.s - s) break end end
      local need = (alt - pa) / max(1, s_next)
      if (alt - pa > 500 and need > 80) or alt - pa > 1500 then out.msg.drag = (inp.spdbrk or 0) < 0.1 end
    elseif idle_seg and M.IDLE_DES and not st.idle_thr and alt > pa - M.IDLE_BAND_LO then
      -- IDLE DESCENT: T/D -> idle -> steady descent -> level off. Thrust stays at idle and the nose
      -- holds the descent speed; the speed target is trimmed by the path deviation (+/-10 kt for
      -- 300 ft) so the aircraft stays close to the path without thrust (above: a little faster,
      -- below: a little slower). Restrictions ahead are flown by levelling at them.
      sub = "PTH"; pitch = "SPD"; at = "IDLE"; out.spd_des_floor = true
      local dev = alt - pa
      local restricted = (pv or des_cas) < des_cas - 2 or alt < (inp.des_trans_alt or 10000) + 500
      local up = restricted and 0 or 10
      local adj = max(-M.IDLE_SPD_LO, min(up, dev / M.IDLE_FT_PER_KT))
      local dt_ = max(0.01, inp.dt or 0.1)
      st.spd_adj = (st.spd_adj or adj) + (adj - (st.spd_adj or adj)) * min(1, dt_ / 8)
      if not inp.spd_intv then
        out.spd = min(cas + st.spd_adj, vmo - 10)
        out.ism = false
        out.mach = PF.cas_to_mach(out.spd, alt)
      end
      -- next restriction with a lower limit ahead: level at it until the fix is passed
      local lo = next_des_constraint(plan, s, dreg, last)
      if lo and lo > level_floor and lo < alt + 100 then
        capture(lo, "CNSTR")
        if alt <= lo + 100 + abs(inp.vs or 0) * 0.08 then sub = "ALT"; pitch = "ALT"; at = "SPD" end
      end
      -- well above the path: DRAG REQUIRED (the speedbrake, as on the aircraft)
      if dev > 300 and ias > cas + 5 and (inp.spdbrk or 0) < 0.1 then out.msg.drag = true end
      -- well below the path and slow: thrust and back to the path (VNAV PTH with A/T SPD)
      -- (the speed target is already up to IDLE_SPD_LO slow: thrust only once the speed falls well
      -- below that, or the aircraft is at the bottom of the band)
      if (dev < -M.IDLE_BAND_LO + 100 and ias < cas - M.IDLE_SPD_LO - 3) or ias < cas - M.IDLE_SPD_LO - 10 then st.idle_thr = true end
    else
      if st.idle_thr and alt > pa - 300 then st.idle_thr = false end
      if not idle_seg then st.idle_thr = false
      elseif M.IDLE_DES and alt <= pa - M.IDLE_BAND_LO then st.idle_thr = true end
      if st.idle_thr and not inp.spd_intv then
        -- back from an idle descent that ended up well below the path: keep the trimmed (slower)
        -- speed, so the A/T only adds the thrust a shallow descent needs (no thrust surge)
        out.spd = min(cas + min(0, st.spd_adj or 0), vmo - 10)
        out.ism = false
        out.mach = PF.cas_to_mach(out.spd, alt)
      end
      sub = "PTH"; pitch = "VS"
      -- path tracking: feed-forward of the path vertical speed plus a
      -- correction on the PREDICTED error 12 s ahead (damps the response), authority at least
      -- 1200 fpm / 80 % of the path rate, smooth near the path; VS target slewed
      local vsn = inp.vs or path_vs
      local perr = (pa + 0.2 * path_vs) - (alt + 0.2 * vsn)
      local auth = max(1200, 0.8 * abs(path_vs))
      local x = max(-1, min(1, perr / 300))
      local corr = math.sin(x * pi / 2) * auth * 0.75
      vs_cmd = path_vs + corr
      -- idle path descent losing speed (the path is shallower than the idle descent at this speed):
      -- the nose is lowered to regain the speed (extra descent rate from the speed deficit and its
      -- trend) while within SPD_BELOW ft below the path - the room shrinks from half of it; thrust
      -- (A/T SPD) is only added once pitch has used that room up or the speed is well low
      local spd_prio = false
      local below = pa - alt
      -- the room below the path closes towards the next altitude constraint (the path meets it there)
      local room_ft = M.SPD_BELOW
      local s_c = (P.ed_s or s) - s
      for _, d in ipairs(P.des or {}) do if d.cns and d.s > s + 0.1 then s_c = min(s_c, d.s - s) break end end
      room_ft = max(0, min(room_ft, 40 * (s_c - 2)))
      if idle_seg and ias < cas - 1 and below < room_ft and (inp.spdbrk or 0) < 0.1 then
        local def = cas - ias
        local extra = min(2000, 120 * (def - 1) - 250 * min(0, inp.ias_rate or 0))
        local room = max(0, min(1, (room_ft - max(0, below)) / (0.5 * M.SPD_BELOW)))
        if extra > 0 then
          vs_cmd = min(vs_cmd, path_vs - extra * room + min(0, corr))
          spd_prio = true
        end
      end
      st.spd_prio = spd_prio
      if alt < pa - 100 and not spd_prio and not (idle_seg and below < room_ft and ias < cas + 2) then
        vs_cmd = min(0, vs_cmd); vs_cmd = 0
        -- (below the idle path after an idle descent: a shallow descent back up to the path - 40 % of
        -- the path rate - instead of a level-off with a thrust surge)
        if st.idle_thr and below < 2.5 * M.IDLE_BAND_LO then
          local lo = next_des_constraint(plan, s, dreg, last)
          if not lo or alt > lo + 500 then vs_cmd = 0.4 * min(0, path_vs) end
        end
      end
      vs_cmd = max(min(vs_cmd, 0), -6000)
      local dt_ = max(0.01, inp.dt or 0.1)
      if st.vs_slew == nil or st.sub ~= "PTH" then st.vs_slew = inp.vs or vs_cmd end
      st.vs_slew = st.vs_slew + max(-500 * dt_, min(500 * dt_, vs_cmd - st.vs_slew))
      vs_cmd = st.vs_slew
      local sb = (inp.spdbrk or 0) > 0.1
      if alt < pa - (idle_seg and max(100, room_ft) or 100) then at = "SPD"
      elseif idle_seg then
        at = "IDLE"
        -- thrust only if well slow (with the speedbrake out the pilot is managing the energy);
        -- once in SPD stay there until the speed is back near the target (no IDLE/SPD hunting)
        -- the idle path is shallower than this aircraft's idle descent: SPD (thrust) and stay there for
        -- at least 45 s and until the speed is back on target, instead of cycling IDLE/HOLD <-> SPD
        local tnow = inp.t or 0
        -- (thrust only once pitch has used up its room below the path, or well slow)
        if not st.slow and ((ias < cas - 8 and below > room_ft * 0.7) or ias < cas - (sb and 25 or 15)) then st.slow = true; st.slow_t = tnow end
        if st.slow and ias >= cas - 1 and tnow - (st.slow_t or 0) > 45 then st.slow = false end
        if st.slow then at = "SPD" end
      else
        at = "SPD"
      end
      -- path speed protection: DRAG REQUIRED once the speed runs 10 kt over on the idle path
      if ias > cas + 10 and ias < vmo - 11 and not sb then out.msg.drag = true end
      if ias > vmo - 11 then sub = "SPD"; pitch = "SPD"; at = "IDLE"; out.msg.drag = true end
      if ias > cas + 5 and at == "SPD" and idle_seg and alt >= pa - 100 then at = "IDLE" end
    end
    -- capture of E/D / level segments handled by the VS law (grad 0)
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
M.FLAP_RETR_MARGIN = 5 -- kt above the next flap position's maneuver speed (VNAV flap retraction speed)
M.SPD_LIM_ALT = 10000  -- ft: cruise below this is limited to SPD_LIM_KT
M.SPD_LIM_KT = 250
M.IDLE_DES = true      -- idle descent with speed on pitch (path-trimmed speed target)
M.IDLE_BAND_LO = 1200  -- ft below the path before thrust is used (idle and level off at the next restriction instead)
M.IDLE_SPD_LO = 12     -- kt the idle descent speed target may go below the descent speed
M.IDLE_FT_PER_KT = 30  -- path deviation per kt of speed target trim
B744VNAV = M
