--[[ B744 engine acceleration / deceleration schedules (EEC fuel scheduling).
     A high-bypass engine is slow to leave idle (small core, big fan, surge margin and EGT limit
     the extra fuel the EEC may add) and then accelerates rapidly from roughly 30-40% power.
     The EEC's commanded power (the X-Plane throttle) follows these schedules; X-Plane's own
     spool lag is set short so the schedule dominates.

     Times are idle -> 95% of rated thrust (static, sea level), and 95% -> idle:
       gi  : from ground (minimum) idle
       fi  : from flight idle (14 CFR 33.73(b): not more than 5 s)
       ai  : from approach idle (flaps in landing position: raised idle so a go-around
             reaches go-around thrust quickly)
       dec : deceleration from takeoff thrust to idle
     Boeing / the engine makers do not publish per-variant figures; the values below are set
     inside the certification limit, from the 8 s go-around thrust credit (14 CFR 25.119) with
     approach idle, and from line-pilot reports on the 747-400:
       - CF6-80C2: quickest off idle of the three (fuel can be introduced early, light core)
       - PW4000-94: slower to get going from ground idle, similar once above ~40% N1
       - RB211-524: three-spool, little residual thrust at idle; accelerates well but its
         thrust decays fastest when the levers are retarded
     Pure Lua; registered as global B744SPOOL. ]]
local M = {}
local min, max = math.min, math.max

M.XP_LAG = 0.4        -- s, X-Plane spool time left on top of the schedule
-- p is the engine power past X-Plane's throttle dead band (the EEC adds the learned dead band),
-- which moves X-Plane's N1 from idle (about 29.5%) to 100% (a little steeper at the top): about
-- 40% N1 at p 0.20, 50% at p 0.37
M.P40 = 0.20

-- per-engine character (engines of different age): spool rate, overshoot / settling, wander
M.ENG = {
  [0] = { rate = 1.00, os = 1.00, ph = 0.0 },
  [1] = { rate = 0.95, os = 1.25, ph = 1.7 },
  [2] = { rate = 1.04, os = 0.85, ph = 3.1 },
  [3] = { rate = 0.97, os = 1.10, ph = 4.6 },
}

-- times (s): t40 = idle -> 40% N1, t95 = idle -> 95% thrust; gi ground idle, fi flight idle,
-- ai approach idle; dec = takeoff thrust -> 40% N1, -> near idle (2% thrust).  High-bypass engines creep off idle (small core,
-- big fan, surge margin) and accelerate briskly from about 40% N1.
M.TYPES = {
  GE = { gi = { 4.0, 8.0 }, fi = { 2.2, 4.8 }, ai = { 1.2, 3.6 }, dec = { 3.0, 6.0 } },
  PW = { gi = { 4.6, 8.6 }, fi = { 2.5, 5.0 }, ai = { 1.3, 3.8 }, dec = { 3.3, 6.5 } },
  RR = { gi = { 4.3, 8.3 }, fi = { 2.3, 4.9 }, ai = { 1.25, 3.7 }, dec = { 2.8, 5.6 } },
}

local function smooth(a, b, x)
  local t = max(0, min(1, (x - a) / (b - a)))
  return t * t * (3 - 2 * t)
end

-- acceleration capability vs power: a slow creep below 40% N1, then rising to full by ~65% N1
local function g_acc(p, lo) return lo + (1 - lo) * smooth(M.P40, M.P40 + 0.22, p) end
-- deceleration: quick from high power, slow tail below about 40% N1
local function g_dec(p, dlo) return dlo + (1 - dlo) * smooth(M.P40 - 0.03, M.P40 + 0.15, p) end

-- simulated times (schedule + X-Plane lag): to 40% N1 and to 95% thrust from idle
local function sim_acc(lo, k)
  local p, th, t, dt = 0, 0, 0, 0.05
  local t40 = nil
  while th < 0.95 and t < 40 do
    p = min(1, p + k * g_acc(p, lo) * dt)
    th = th + (p - th) * dt / M.XP_LAG
    t = t + dt
    if not t40 and th >= M.P40 then t40 = t end
  end
  return t40 or 40, t
end
local function sim_dec(kd, dlo)
  local p, th, t, dt = 1, 1, 0, 0.05
  local t40 = nil
  while th > 0.02 and t < 40 do
    p = max(0, p - kd * g_dec(p, dlo) * dt)
    th = th + (p - th) * dt / M.XP_LAG
    t = t + dt
    if not t40 and th <= M.P40 then t40 = t end
  end
  return t40 or 40, t
end
-- k giving t95 for a given lo (bisection; time falls as k rises)
local function solve_k(lo, t95)
  local a, b = 0.02, 20
  for _ = 1, 30 do
    local m = (a + b) / 2
    local _, t = sim_acc(lo, m)
    if t > t95 then a = m else b = m end
  end
  return (a + b) / 2
end

local cache = {}
local function params(kind, ctx)
  local key = kind .. ctx
  if cache[key] then return cache[key] end
  local T = M.TYPES[kind] or M.TYPES.GE
  local tt = T[ctx] or T.fi
  local t40, t95 = tt[1], tt[2]
  -- lo (creep below 40% N1) for the time to 40%, k for the time to 95% (nested bisection)
  local a, b = 0.02, 1.0
  local k = 1
  for _ = 1, 24 do
    local lo = (a + b) / 2
    k = solve_k(lo, t95)
    local t4 = sim_acc(lo, k)
    if t4 > t40 then a = lo else b = lo end
  end
  local lo = (a + b) / 2
  k = solve_k(lo, t95)
  -- deceleration: kd for the time to 40% N1, dlo (tail) for the time to near idle
  local function solve_kd(dlo)
    local da, db = 0.05, 20
    for _ = 1, 30 do
      local m = (da + db) / 2
      local t4 = sim_dec(m, dlo)
      if t4 > T.dec[1] then da = m else db = m end
    end
    return (da + db) / 2
  end
  local la, lb = 0.01, 1.0
  for _ = 1, 24 do
    local dlo = (la + lb) / 2
    local _, tt2 = sim_dec(solve_kd(dlo), dlo)
    if tt2 > T.dec[2] then la = dlo else lb = dlo end
  end
  local dlo = (la + lb) / 2
  local P = { lo = lo, k = k, kd = solve_kd(dlo), dlo = dlo }
  cache[key] = P
  return P
end
M.params = params

--[[ one engine step.  p: current scheduled power (throttle 0..1.1), want: EEC demand,
     dt: s, kind: "GE"/"PW"/"RR", ctx: "gi" ground idle, "fi" flight idle, "ai" approach idle.
     returns new p, limited (+1 accel limited, -1 decel limited, 0 free) ]]
function M.step(p, want, dt, kind, ctx, rate)
  local P = params(kind, ctx)
  rate = rate or 1
  if want > p then
    local np = p + P.k * rate * g_acc(max(0, p), P.lo) * dt
    if np >= want then return want, 0 end
    return np, 1
  elseif want < p then
    local np = p - P.kd * rate * g_dec(max(0, p), P.dlo) * dt
    if np <= want then return want, 0 end
    return np, -1
  end
  return p, 0
end

--[[ EEC power control.  The X-Plane throttle needed for a given N1 / EPR is learned while the
     engine is steady (it changes with altitude and Mach), so the schedule drives the throttle
     straight to the power the target needs and a small trim removes what is left - no slow
     integrator crawl at the end of a thrust change.
     st: per-engine state table (create with {}).  inp: target, input (N1 % or 50*EPR), dt,
     kind, ctx, lo/hi (input at idle / at full throttle, initial guess), hold_zero ]]
local NB = 12                     -- throttle bins 0, 0.1 .. 1.1
local function tbl_init(st, lo, hi)
  -- X-Plane's N1 (and so N1 / EPR) rises a little faster towards the top of the throttle range:
  -- about 0.66 p + 0.34 p^2 of the way from idle to maximum (fitted to the CF6 in X-Plane)
  st.tbl = {}
  for b = 0, NB - 1 do local p = b / 10 st.tbl[b] = lo + (hi - lo) * (0.66 * p + 0.34 * p * p) end
end
local function tbl_input(st, thr)
  local x = max(0, min(NB - 1.001, thr * 10))
  local b = math.floor(x)
  local f = x - b
  return st.tbl[b] + (st.tbl[b + 1] - st.tbl[b]) * f
end
local function tbl_throttle(st, val)
  local t = st.tbl
  if val <= t[0] then return 0 end
  for b = 0, NB - 2 do
    if val <= t[b + 1] then
      local d = t[b + 1] - t[b]
      local f = (d > 1e-6) and (val - t[b]) / d or 0
      return (b + f) / 10
    end
  end
  return (NB - 1) / 10
end
local function tbl_learn(st, thr, val, w)
  local x = max(0, min(NB - 1.001, thr * 10))
  local b = math.floor(x)
  local f = x - b
  local err = val - tbl_input(st, thr)
  st.tbl[b] = st.tbl[b] + err * w * (1 - f)
  st.tbl[b + 1] = st.tbl[b + 1] + err * w * f
  -- keep it monotonic
  for k = 1, NB - 1 do if st.tbl[k] < st.tbl[k - 1] + 0.05 then st.tbl[k] = st.tbl[k - 1] + 0.05 end end
end
M.tbl_throttle = tbl_throttle

function M.control(st, inp)
  local dt = max(0.001, inp.dt)
  if not st.tbl then tbl_init(st, inp.lo, inp.hi) end
  st.p = st.p or 0
  st.trim = st.trim or 0
  if inp.hold_zero then
    st.p = 0; st.trim = 0
    return 0
  end
  -- learn the throttle -> N1/EPR relation when the engine is settled
  local rate = (st.last_in and (inp.input - st.last_in) / dt) or 0
  st.last_in = inp.input
  st.rate_f = (st.rate_f or 0) + (rate - (st.rate_f or 0)) * min(1, dt / 0.5)
  if math.abs(st.p - (st.p_settle or -1)) > 0.004 then st.p_settle = st.p; st.settle_t = 0 end
  st.settle_t = (st.settle_t or 0) + dt
  local span = math.abs(inp.hi - inp.lo)
  if st.settle_t > 0.8 and math.abs(st.rate_f) < 0.004 * span then
    tbl_learn(st, st.p, inp.input, min(1, dt / 1.0))
  end
  -- feedforward + trim
  local ff = tbl_throttle(st, inp.target)
  local slope = 1.0 / max(1, span)               -- throttle per input unit (nominal)
  local err = inp.target - inp.input
  -- lead: aim with where the input is heading (its rate over the remaining X-Plane lag), so a large
  -- thrust change does not overshoot the target
  local err_l = inp.target - (inp.input + max(0, st.rate_f) * M.XP_LAG * 0.5)
  if err > 0 and err_l < err and err < 0.15 * span then err = max(err_l, -0.05 * span) end
  local want = max(0, min(1.1, ff + st.trim + 1.2 * slope * err))
  local E = M.ENG[inp.eng or 0] or M.ENG[0]
  local np, lim = M.step(st.p, want, dt, inp.kind, inp.ctx, E.rate)
  st.p = np
  local raw_err = inp.target - inp.input
  if st.p <= 0.001 and raw_err < 0 then
    -- at idle with the target at / below idle: no wind-up (it made the next acceleration crawl)
    st.trim = st.trim * (1 - min(1, dt / 3))
  elseif lim == 0 and math.abs(err) < 0.15 * span then
    st.trim = max(-0.15, min(0.15, st.trim + 2.5 * slope * err * dt))
  end
  -- settling: an engine does not arrive clinically - after an acceleration / deceleration it
  -- overshoots / undershoots a little and settles (looser below about 50% N1, tighter above)
  st.t = (st.t or 0) + dt
  if st.lim_prev and st.lim_prev ~= 0 and lim == 0 then
    local low = st.p < M.P40 + 0.12
    st.os_a = st.lim_prev * (low and 0.040 or 0.020) * E.os
    st.os_t = 0
  end
  st.lim_prev = lim
  local out = st.p
  if st.os_a then
    st.os_t = st.os_t + dt
    local d = st.os_a * math.exp(-st.os_t / 1.4) * math.sin(st.os_t * 2 * math.pi / 3.6 + 0.6) / math.sin(0.6)
    if st.os_t > 8 then st.os_a = nil else out = out + d end
  end
  -- slight wander while steady (fuel control / bleed), different per engine
  if st.p > 0.001 then
    out = out + 0.0025 * math.sin(st.t * 0.9 + E.ph) * math.sin(st.t * 0.37 + 2 * E.ph)
  end
  return max(0, min(1.1, out))
end


--[[ EEC control (2): an acceleration / deceleration schedule in N1 terms, as the user feels it.
     A reference moves to the lever command at a rate set by the present N1 (on the ground: slow
     below 30% N1, ramping hard through 40-50%, brisk above; deceleration with a gentle tail near
     idle); the throttle follows the reference through the learned throttle -> input table plus a
     small integral, never past X-Plane's maximum (X-Plane runs N1 past 100% above it).  At the idle
     stop the throttle is held at idle once N1 is down (a steady idle).  Engines of different age:
     each overshoots / undershoots a mid-range setting by its own 1-3% N1 and settles; none at high
     power.  Inputs as M.control plus n1 (displayed N1) and idle (lever at the idle stop). ]]
local function interp(tbl, x)
  if x <= tbl[1][1] then return tbl[1][2] end
  for k = 2, #tbl do
    if x <= tbl[k][1] then
      local a, b = tbl[k - 1], tbl[k]
      return a[2] + (b[2] - a[2]) * (x - a[1]) / (b[1] - a[1])
    end
  end
  return tbl[#tbl][2]
end
-- Reference N1 rate (% per s) vs the N1 the reference stands at: one continuous schedule -
-- sluggish off idle and through 30-45% (the fuel control's acceleration limit), building to a
-- brisk rate past 50% where the engine follows the lever closely
M.TOGA_K = 1.05   -- (v48) acceleration factor to TO/GA thrust
M.ACC = { { 0, 2.0 }, { 26, 2.0 }, { 30, 2.6 }, { 35, 3.6 }, { 40, 5.2 }, { 45, 8.0 }, { 50, 13.0 }, { 55, 20.0 }, { 60, 28.0 }, { 130, 30.0 } }
M.CTX = { gi = 1.0, fi = 1.4, ai = 2.0 }      -- low end only (higher idle N2 in the air)
M.DEC = { { 0, 3.0 }, { 27, 3.0 }, { 32, 5.0 }, { 40, 10.0 }, { 55, 20.0 }, { 70, 30.0 }, { 130, 30.0 } }
M.FAM = { GE = { acc = 1.00, dec = 1.00, ky = 1.0 }, PW = { acc = 0.92, dec = 0.95, ky = 0.47 }, RR = { acc = 0.96, dec = 1.05, ky = 0.47 } }
local function interp(tbl, x)
  if x <= tbl[1][1] then return tbl[1][2] end
  for k = 2, #tbl do
    if x <= tbl[k][1] then
      local a, b = tbl[k - 1], tbl[k]
      return a[2] + (b[2] - a[2]) * (x - a[1]) / (b[1] - a[1])
    end
  end
  return tbl[#tbl][2]
end
local function sstep(a, b, x) local t = max(0, min(1, (x - a) / (b - a))) return t * t * (3 - 2 * t) end
function M.acc_rate(ctx, n1)
  local k = M.CTX[ctx] or 1
  return interp(M.ACC, n1) * (1 + (k - 1) * (1 - sstep(40, 60, n1)))
end
function M.dec_rate(n1) return interp(M.DEC, n1) end

--[[ control2: the thrust lever commands the thrust.  A reference moves from where it is to the
     lever's command along the acceleration / deceleration schedule and the throttle is set
     straight from the reference through a fixed throttle -> N1 (or EPR) map - nothing is
     learned, there is no feedback during a thrust change, so the engine cannot overshoot or
     hunt.  Only once the engine has settled at its command does a slow, bounded trim remove what
     the fixed map is out by (altitude / Mach), far slower than the engine so it cannot
     overshoot either.  At the idle stop the throttle is at idle.  inp.time (s) gives real dt. ]]
-- GE (CF6) map: X-Plane N1 vs power past the dead band (fitted to the CF6 in X-Plane, checked
-- against recorded steady points) and the EICAS N1 (x takeoff N1 / 100, idle blended to the
-- idle value below 60%)
function M.ge_display(p, toga, idle, top)
  top = top or 60
  local xp = 29.5 + 46.37 * p + 24.12 * p * p
  local d, di = xp * toga / 100, 29.5 * toga / 100
  local w = max(0, min(1, (top - d) / (top - di)))
  return d + (idle - di) * w
end
local function tbl_ge(st, toga, idle)
  st.tbl = {}
  for b = 0, NB - 1 do st.tbl[b] = M.ge_display(b / 10, toga, idle) end
  for k = 1, NB - 1 do if st.tbl[k] < st.tbl[k - 1] + 0.05 then st.tbl[k] = st.tbl[k - 1] + 0.05 end end
end

function M.control2(st, inp)
  local dt = inp.dt
  if inp.time then
    if st.t and inp.time > st.t then dt = inp.time - st.t end
    st.t = inp.time
  end
  dt = max(0.001, min(0.25, dt or 0.05))
  if inp.toga then
    local key = math.floor(inp.toga * 10 + 0.5) * 1000 + math.floor(inp.lo * 10 + 0.5)
    if st.tkey ~= key then tbl_ge(st, inp.toga, inp.lo); st.tkey = key end
  elseif not st.tbl then tbl_init(st, inp.lo, inp.hi) end
  local F = M.FAM[inp.kind] or M.FAM.GE
  local ky = F.ky
  local y, tgt = inp.input, inp.target
  local lo = inp.lo
  st.trim = st.trim or 0
  if inp.hold_zero then st.p = 0; st.r = nil; st.trim = 0 return 0 end
  if inp.idle then tgt = lo end
  tgt = max(lo, tgt)
  -- start from where the engine is (first call / after a start / after reverse)
  if st.r == nil then st.r = max(lo, y) end
  local n1r = 26 + (st.r - lo) / ky              -- N1 the reference stands for
  local r0 = st.r
  -- (the governor eases the engine onto the command: the rate tapers over the last few %)
  -- just off idle the fuel control lets the acceleration build up gradually (about 1.5 s)
  if tgt > st.r then st.acc_t = (st.acc_t or 0) + dt else st.acc_t = 0 end
  local onset = 1 - (1 - (0.25 + 0.75 * sstep(0, 1.5, st.acc_t))) * (1 - sstep(32, 40, n1r))
  if tgt > st.r then
    -- (v48: to takeoff / go-around thrust - a target above 88 % N1 - 5 % faster)
    local togak = ((26 + (tgt - lo) / ky) >= 88) and M.TOGA_K or 1.0
    local rate = min(M.acc_rate(inp.ctx, n1r) * F.acc * ky * onset * togak, 6.0 * (tgt - st.r) + 0.2 * ky)
    st.r = min(tgt, st.r + rate * dt)
  elseif tgt < st.r then
    local rate = min(M.dec_rate(n1r) * F.dec * ky, 4.0 * (st.r - tgt) + 0.2 * ky)
    st.r = max(tgt, st.r - rate * dt)
  end
  -- reference rate for the lead on X-Plane's own spool lag (the engine then follows the
  -- reference itself instead of trailing it); smoothed on the way up, never held on the way down
  local raw = (st.r - r0) / dt
  st.rd = (st.rd or 0) + (raw - (st.rd or 0)) * min(1, dt / 0.12)
  if math.abs(raw) < math.abs(st.rd) then st.rd = raw end
  if st.r == tgt then st.arr_t = (st.arr_t or 0) + dt else st.arr_t = 0 end
  local at_idle = (st.r <= lo + 1e-6)
  if at_idle then st.p = 0; st.rd = 0 return 0 end
  -- slow bounded trim, only once the reference has been at the command for a while
  if st.arr_t > 1.5 then
    local e = st.r - y
    if math.abs(e) > 0.15 * ky then
      local rate = max(-0.015, min(0.015, 0.006 * e / ky))
      st.trim = max(-0.25, min(0.25, st.trim + rate * dt))
    end
  end
  local u = tbl_throttle(st, st.r + st.rd * M.XP_LAG) + st.trim
  st.p = max(0, min(1.0, u))
  return st.p
end

function M.kind_of(engine_type)
  if engine_type == 0 then return "PW" elseif engine_type == 2 then return "RR" end
  return "GE"
end

B744SPOOL = M
return M
