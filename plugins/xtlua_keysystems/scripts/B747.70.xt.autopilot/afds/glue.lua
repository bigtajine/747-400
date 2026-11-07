--[[ Glue between a flight plan (B744ROUTE plan) and the AFDS: runs path build, LNAV guidance,
     VNAV profile + guidance at sensible rates.  Pure Lua; registered as global B744GLUE. ]]
local M = {}
local PA, LN, VN, RT, PF, G = B744PATH, B744LNAV, B744VNAV, B744ROUTE, B744PERF, B744GEO
local abs, min, max, floor = math.abs, math.min, math.max, math.floor

function M.new()
  return { plan = nil, serial = -1, prof = nil, lst = LN.new_state(), vst = VN.new_state(),
    t_path = -100, t_prof = -100, lout = {}, vout = {}, ctx_key = "" }
end

local function prof_fn(G_)
  return function(i)
    local P = G_.prof
    local pr = P and P.legs and P.legs[i]
    local alt = (pr and pr.alt) or 10000
    local cas = (pr and pr.cas) or ((pr and pr.mach) and PF.mach_to_cas(pr.mach, alt)) or 280
    local act = G_.plan and G_.plan.active or 0
    local legs = G_.plan and G_.plan.legs or {}
    -- a turn the aircraft is already flying (or about to roll into) keeps its geometry: rebuilding
    -- it under the aircraft would make LNAV re-aim at a different arc
    local fz = G_.freeze
    if fz and (fz[i] or i < act) and legs[i] and legs[i].pr and legs[i].pr.gs then
      local o = legs[i].pr
      return { alt = o.alt, gs = o.gs, grad = o.grad }
    end
    -- the next turns are planned for the speed the aircraft will really have there (as the FMC turn
    -- prediction does): the present speed, accelerating (~1.2 kt/s) or decelerating (~0.7 kt/s)
    -- towards the commanded speed (in the climb: the climb speed once restrictions are passed), and
    -- the altitude it will really be at - the predicted profile can lag a fast climb or an
    -- acceleration after a speed restriction (KALO1S: 230 kt at SOORO, then 320+ kt at D342Q)
    local nw = G_.now
    if nw and i >= act and i <= act + 5 and nw.ias and nw.ias > 100 then
      local lp, l = legs[i - 1], legs[i]
      local s_turn = (lp and lp.cum) or (l and l.cum) or nw.s or 0
      local tsec = max(0, s_turn - (nw.s or s_turn)) / max(120, nw.gs or 250) * 3600
      if nw.climb then alt = max(alt, min(nw.alt_cap or alt, nw.alt + max(0, nw.vs or 0) * tsec / 60)) end
      local tgt = nw.tgt or nw.ias
      if nw.climb and G_.prof and G_.prof.clb_cas and nw.vnav then
        local free = G_.prof.clb_cas
        if alt < 10000 then free = min(free, 250) end
        tgt = max(tgt, free)
      end
      local v
      if tgt >= nw.ias then v = min(tgt, nw.ias + 1.2 * tsec) else v = max(tgt, nw.ias - 0.7 * tsec) end
      -- a max speed restriction at the turn fix (or the fix after it) caps the speed in the turn
      for qi = 1, 2 do
        local q = (qi == 1) and lp or l
        local sp = q and q.spd
        if sp and sp.v and (sp.d == "-" or sp.d == "@") then v = min(v, sp.v) end
      end
      if v > cas then cas = v end
    end
    -- the turn at the end of the previous leg is flown at that fix's speed restriction (the profile
    -- value of this leg is the speed at its own end, after the acceleration)
    do
      local lp = legs[i - 1]
      local sp = lp and lp.spd
      if sp and sp.v and (sp.d == "-" or sp.d == "@") and sp.v < cas then cas = sp.v end
    end
    local tas = PF.tas(cas, alt)
    -- turn radii are planned for the worst case (tailwind in the turn) so the bank stays within limits
    local gs = max(140, tas + 0.8 * (G_.wspd or 0))
    -- the next turns: never smaller than for the speed actually flown now (a radius planned for a lower
    -- speed cannot be flown within the bank limit: late roll-out and overshoot)
    if G_.gs_now and i >= act and i <= act + 3 then gs = max(gs, G_.gs_now + 5) end
    return { alt = alt, gs = gs, grad = PF.climb_grad(alt, G_.w or 300000, tas) }
  end
end

function M.set_plan(G_, plan, opts)
  G_.plan = plan
  G_.opts = opts or G_.opts or {}
  G_.serial = -1
  G_.lst = LN.new_state()
  G_.prof = nil
end

function M.update(G_, ac, ctx, fl)
  local plan = G_.plan
  local lout, vout = { valid = false, route = false }, { valid = false }
  if not plan or #plan.legs < 1 then G_.lout, G_.vout = lout, vout return lout, vout end
  local t = ac.t or 0
  G_.w = ctx.w
  if ac.gs and ac.gs > 100 and not ac.onground then G_.gs_now = (G_.gs_now or ac.gs) + (ac.gs - (G_.gs_now or ac.gs)) * min(1, (ac.dt or 0.1) / 10) end
  -- wind speed from the ground and air velocity vectors (filtered)
  if ac.tas and ac.gs and ac.hdg_true and ac.trk_true and not ac.onground then
    local r = math.pi / 180
    local wx = ac.gs * math.sin(ac.trk_true * r) - ac.tas * math.sin(ac.hdg_true * r)
    local wy = ac.gs * math.cos(ac.trk_true * r) - ac.tas * math.cos(ac.hdg_true * r)
    local w = math.sqrt(wx * wx + wy * wy)
    G_.wspd = (G_.wspd or w) + (w - (G_.wspd or w)) * min(1, (ac.dt or 0.1) / 20)
  end
  -- present state for the turn predictions (speed / altitude the aircraft will have at the next turns)
  local pl = G_.lout or {}
  local pv = G_.vout or {}
  local vph = G_.vst and G_.vst.phase
  local tgt = (fl.vnav_engaged and pv.valid and pv.spd) or fl.mcp_spd or ac.ias
  local cap = ctx.crz or 45000
  if fl.mcp_alt and fl.mcp_alt > (ac.alt or 0) then cap = min(cap, fl.mcp_alt) end
  G_.now = (not ac.onground) and { s = pl.s_now, gs = ac.gs, ias = ac.ias, alt = ac.alt or 0, vs = ac.vs, tgt = tgt,
    climb = (vph == "TO" or vph == "CLB") and (ac.vs or 0) > 300, alt_cap = cap, vnav = fl.vnav_engaged } or nil
  -- periodic rebuild (speed / wind changes, every 5 s): the turn being flown (or about to be rolled
  -- into) keeps its geometry - rebuilding that arc under the aircraft makes LNAV re-aim - while the
  -- turns further ahead are re-planned for the speed the aircraft now has (accelerating after a
  -- restriction: wider turns, anticipated earlier, instead of an overshoot)
  local act = plan.active or 0
  local al = plan.legs[act]
  local dyn = al and PA.DYN and PA.DYN[al.t]
  if plan.serial ~= G_.serial or (t - G_.t_path > 5 and not dyn) then
    local fz = nil
    if plan.serial == G_.serial then
      fz = { [act] = true }
      local tr = al and al.trans
      local gsn = max(120, ac.gs or 250)
      if (pl.turn and (pl.dtg or 99) < 6) or (tr and (pl.dtg or 99) < (tr.d or 0) * 1.3 + gsn * 15 / 3600 + 1) then
        fz[act + 1] = true
      end
      if al and (al.t == "AF" or al.t == "RF") then fz[act + 1] = fz[act + 1] or (pl.dtg or 99) < 6 end
    end
    G_.freeze = fz
    local opts = G_.opts or {}
    opts.force_r = nil
    if fz then
      local f = {}
      for k in pairs(fz) do
        local q = plan.legs[k - 1]
        if q and q.trans and q.trans.r then f[q] = q.trans.r end
      end
      opts.force_r = f
    end
    PA.build(plan, prof_fn(G_), opts)
    opts.force_r = nil
    G_.freeze = nil
    G_.serial = plan.serial
    G_.t_path = t
    G_.t_prof = -100
  end
  ac.bank_lim = 25
  local lo = LN.update(G_.lst, plan, ac)
  lo.route = plan.activated and #plan.legs > 1
  lout = lo
  -- profile
  local key = tostring(ctx.crz) .. "/" .. tostring(ctx.ci) .. "/" .. tostring(G_.serial) .. "/" .. tostring(plan.vserial) .. "/" .. tostring(ctx.sel_crz_mach) .. tostring(ctx.sel_des_cas) .. tostring(ctx.sel_des_mach) .. tostring(ctx.sel_clb_cas) .. "/" .. tostring(ctx.des_winds_sig)
  local s_now = lo.s_now or (ac.onground and 0) or nil
  -- (once descending, the descent path is kept - as the FMC does - unless the route or the vertical
  -- plan changes: re-planning it every few seconds moved the path under the aircraft, by hundreds of
  -- feet at a time, and VNAV chased it - pitch / thrust hunting)
  local des_fixed = G_.vst and G_.vst.phase == "DES" and G_.prof and G_.prof.ok
  if s_now and ((t - G_.t_prof > 4 and not des_fixed) or key ~= G_.ctx_key) then
    ctx.s_now = s_now
    ctx.alt_now = ac.alt
    ctx.phase = G_.vst.phase
    ctx.wind_comp = ac.wind_comp or 0
    G_.prof = VN.profile(plan, ctx)
    G_.t_prof = t
    G_.ctx_key = key
  end
  -- climb performance learning: the achieved climb (energy-equivalent, at climb thrust) is compared
  -- with the performance model in 5000 ft bands, so T/C and the climb predictions fit this aircraft
  -- in X-Plane (bands are kept between flights by the IO script)
  G_.clb_tab = G_.clb_tab or {}
  -- energy height h + V^2/2g sampled every 10 s; the rate over the last 60 s (all at climb thrust)
  -- is compared with the model's average over the same altitudes (gust noise averages out)
  if fl.clb_learn and ac.alt and ac.tas and ac.t then
    local smp = G_.clb_smp or {}
    G_.clb_smp = smp
    if #smp == 0 or ac.t - smp[#smp].t >= 10 then
      local v = ac.tas * 1.6878
      smp[#smp + 1] = { t = ac.t, he = ac.alt + v * v / (2 * 32.174), h = ac.alt, ias = ac.ias or 0 }
      if #smp > 7 then table.remove(smp, 1) end
      if #smp == 7 then
        local a, b = smp[1], smp[7]
        -- the model gives the climb rate at the schedule speed: use windows with a steady IAS
        local rate = (b.h - a.h) / (b.t - a.t) * 60
        local hm = (a.h + b.h) / 2
        local pred = PF.climb_rate(hm, G_.w or 300000)
        if pred > 100 and math.abs(b.ias - a.ias) < 4 then
          local band = math.max(0, math.min(9, math.floor(hm / 5000)))
          local r = max(0.2, min(2.0, rate / pred))
          local k = G_.clb_tab[band] or 1
          G_.clb_tab[band] = k + (r - k) * 0.25
          G_.clb_learned = true
        end
      end
    end
  else
    G_.clb_smp = nil
  end
  ctx.clb_tab = G_.clb_tab
  -- idle descent learning (same idea, descent tables measured in X-Plane): steady-
  -- speed idle descents (VNAV SPD / FLCH, no speedbrake, clean) give the real idle gradient per band
  G_.des_tab = G_.des_tab or {}
  -- (v41: 15 s windows sampled every 5 s, starting 8 s after the thrust reached idle - spool-down -
  -- so short idle stretches are measured too; the 30 s windows rarely completed in a real descent)
  if fl.des_learn then G_.des_idle_t = G_.des_idle_t or (ac.t or 0) else G_.des_idle_t = nil end
  if fl.des_learn and ac.alt and ac.t and ac.gs and ac.gs > 120 and ac.t - (G_.des_idle_t or ac.t) >= 8 then
    local smp = G_.des_smp or {}
    G_.des_smp = smp
    if #smp == 0 or ac.t - smp[#smp].t >= 5 then
      smp[#smp + 1] = { t = ac.t, h = ac.alt, ias = ac.ias or 0, gs = ac.gs, tas = ac.tas or ac.gs }
      if #smp > 4 then table.remove(smp, 1) end
      if #smp == 4 then
        local a, b = smp[1], smp[4]
        local gs_m = (a.gs + b.gs) / 2
        -- energy-equivalent descent: a speed gain is altitude not lost (CAS change, as the model is
        -- a constant-CAS idle gradient)
        local tas_m = (a.tas + b.tas) / 2
        local vt = tas_m * 1.6878
        local dcas_t = (b.ias - a.ias) * tas_m / math.max(60, (a.ias + b.ias) / 2) * 1.6878
        local dh_e = (b.h - a.h) + vt * dcas_t / 32.174
        local act = -dh_e / ((b.t - a.t) / 3600 * gs_m)                 -- ft per nm over ground
        local hm = (a.h + b.h) / 2
        local pred = PF.idle_grad_des((a.ias + b.ias) / 2, hm, G_.w or 300000, gs_m, tas_m, G_.prof and G_.prof.des_mach)   -- model value (factor 1)
        if pred > 80 and act > 0 and math.abs(b.ias - a.ias) < 30 then
          local band = math.max(0, math.min(9, math.floor(hm / 5000)))
          local r = max(0.4, min(1.6, act / pred))
          local k = G_.des_tab[band] or (VN.DES_DEFAULT or 1)
          G_.des_tab[band] = k + (r - k) * 0.15
          G_.des_learned = true
        end
      end
    end
  else
    G_.des_smp = nil
  end
  -- bands not learned yet: the mean of the learned ones (the model error is much the same at all
  -- altitudes), the model itself (factor 1) if none
  local dt_f, sum_, n_ = {}, 0, 0
  for b = 0, 9 do local v = G_.des_tab[b] if v then sum_ = sum_ + v; n_ = n_ + 1 end end
  for b = 0, 9 do dt_f[b] = G_.des_tab[b] or (n_ > 0 and sum_ / n_) or nil end
  ctx.des_tab = dt_f
  -- turn speed limit: a fly-by turn ahead whose geometry (short legs, a DME arc join) only allows a
  -- radius smaller than the speed needs is flown at the speed that radius allows at 22 deg of bank
  -- (held from far enough ahead to slow down), instead of overshooting it
  local turn_spd = nil
  if s_now and not ac.onground and (ac.agl or 9999) > 1500 then
    local w80 = 1.0 * (G_.wspd or 0)                 -- worst case: full tailwind in the turn
    local tb = math.tan(22 * math.pi / 180)            -- margin below the 25 deg LNAV bank limit
    for k = max(1, (plan.active or 1) - 1), min(#plan.legs - 1, (plan.active or 1) + 4) do
      local a, b = plan.legs[k], plan.legs[k + 1]
      local tr = a and a.trans
      if tr and tr.rmax and b and a.cum then
        -- (the turn: the second half of the fly-by arc on leg a, the first half on leg b)
        local h1 = a.elems and a.elems[#a.elems]
        local h2 = b.elems and b.elems[1]
        local l1 = (h1 and h1.trans and h1.len) or (tr.d or 0)
        local l2 = (h2 and h2.trans and h2.len) or (tr.d or 0)
        local s_turn = a.cum - l1
        local togo = s_turn - s_now
        if s_now < a.cum + l2 + 0.3 then                      -- held until the turn is completed
          local gsm = math.sqrt(tr.rmax * 1852 * 9.80665 * tb) / 0.514444
          local alt_t = (b.pr and b.pr.alt) or ac.alt or 10000
          local _, _, asnd = PF.isa(alt_t)
          local cas = PF.mach_to_cas(max(0.2, (gsm - w80) / asnd), alt_t)
          local vmin = PF.vref30(ctx.w or 300000) + 80
          cas = max(cas, vmin)
          local need = max(0, (ac.ias or 0) - cas) / 0.7 * max(150, ac.gs or 250) / 3600 + 4
          if togo < need and cas < (ac.ias or 0) + 60 and (not turn_spd or cas < turn_spd) then turn_spd = cas end
        end
      end
    end
  end
  -- hysteresis: once limited, the limit only rises in 5 kt steps (no hunting between rebuilds)
  if turn_spd and G_.turn_spd and turn_spd > G_.turn_spd and turn_spd - G_.turn_spd < 5 then turn_spd = G_.turn_spd end
  G_.turn_spd = turn_spd
  -- v34: the turn speed limit is disabled; procedure speed/altitude restrictions are flown per waypoint instead
  turn_spd = nil
  local P = G_.prof
  if P and P.ok and s_now and ctx.crz and ctx.crz > 0 then
    local inp = {
      engaged = fl.vnav_engaged, s_now = s_now, alt = ac.alt, ias = ac.ias, mach = ac.mach, gs = ac.gs, vs = ac.vs,
      mcp_alt = fl.mcp_alt, mcp_spd = fl.mcp_spd, spd_intv = fl.spd_intv, onground = ac.onground, agl = ac.agl,
      flaps = fl.flaps, vf = fl.vf, alt_push = fl.alt_push, des_now = fl.des_now, crz = ctx.crz, accel_ht = ctx.accel_ht,
      thr_red = ctx.thr_red, v2 = ctx.v2, w = ctx.w, vmo = ctx.vmo, goaround = ac.goaround,
      clb_trans_alt = 10000, clb_trans_spd = 250, des_trans_alt = (ctx.des_trans and ctx.des_trans.alt) or 10000,
      des_trans_spd = (ctx.des_trans and ctx.des_trans.spd) or 240,
      on_final = lo.on_final, thr_idle = fl.thr_idle, toga_spd = fl.toga_spd,
      dt = ac.dt, t = ac.t, spdbrk = fl.spdbrk or 0, ias_rate = ac.ias_rate, turn_spd = turn_spd,
    }
    vout = VN.guide(G_.vst, P, plan, inp)
    vout.valid = true
    if vout.new_crz then G_.new_crz = vout.new_crz end
  else
    vout = { valid = false }
  end
  G_.lout, G_.vout = lout, vout
  return lout, vout
end

B744GLUE = M
