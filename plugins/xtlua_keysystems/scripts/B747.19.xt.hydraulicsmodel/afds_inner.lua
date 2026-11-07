--[[ REBUILT AFDS (2026) - autopilot inner loops (attitude -> elevator / aileron ratio).
     Replaces the stock PI + rate-limited interpolation for the new AFDS: proportional + rate damping
     (+ slow integral that the stabilizer auto-trim offloads), gains scheduled with dynamic pressure.
     Pure Lua (also used by the offline tests).  Registered as global B744INNER.
     Live tuning: laminar/B747/afds/tune/* datarefs are gain multipliers (0 = default 1.0).        ]]
local M = {}
local abs, min, max = math.abs, math.min, math.max
local function clamp(x, a, b) if x < a then return a elseif x > b then return b end return x end

M.P = {
  -- pitch: elevator ratio per deg of attitude error / per deg/s of pitch-rate error / per deg*s
  p_kp = 0.055, p_kd = 0.10, p_ki = 0.012, p_rate = 0.9, p_ff = 0.6,
  -- roll: aileron ratio per deg of bank error / per deg/s of roll rate / per deg*s
  r_kp = 0.070, r_kd = 0.070, r_ki = 0.004, r_rate = 1.4, r_ff = 1.0,
  v_ref = 250,          -- kt, gains are nominal at this IAS
}

function M.new() return { p = {}, r = {} } end

local function qscale(ias, P)
  local s = (max(110, ias or P.v_ref) / P.v_ref)
  return clamp(s * s, 0.25, 2.5)
end

local function mult(x) if x == nil or x <= 0 then return 1 end return x end

-- inp: dt, theta (deg), cmd (deg), ias (kt), cur (current elevator ratio), tune = {kp,kd,ki}
function M.pitch(st, inp)
  local P = M.P
  local s = st.p
  local dt = max(0.005, inp.dt or 0.05)
  local th, c = inp.theta, inp.cmd
  if s.th == nil then s.th = th; s.c = th; s.thd = 0; s.cd = 0; s.I = nil; s.cref = th; s.t = 0 end
  -- engagement: pitch target from the attitude being held to the command (about 1 deg/s at first)
  s.t = (s.t or 99) + dt
  if not s.free then
    local rmax = min(2.5, 1.0 + s.t * 0.3)
    s.cref = s.cref + clamp(c - s.cref, -rmax * dt, rmax * dt)
    if s.t > 1.0 and abs(c - s.cref) < 0.2 then s.free = true end
    if s.t > 30 then s.free = true end
    c = s.cref
  end
  -- attitude rate (Euler pitch rate: no nose-down bias in turns) and command rate, lightly filtered
  local thd = (th - s.th) / dt
  s.thd = s.thd + (thd - s.thd) * min(1, dt / 0.12)
  local cd = (c - s.c) / dt
  s.cd = s.cd + (cd - s.cd) * min(1, dt / 0.3)
  s.th, s.c = th, c
  local q = qscale(inp.ias, P)
  local tu = inp.tune or {}
  local kp = P.p_kp * mult(tu.kp) / q
  local kd = P.p_kd * mult(tu.kd) / q
  local ki = P.p_ki * mult(tu.ki) / q
  local e = c - th
  local de = P.p_ff * s.cd - s.thd
  if s.I == nil then s.I = clamp((inp.cur or 0) - kp * e - kd * de, -1, 1) end   -- bumpless engagement
  s.I = clamp(s.I + ki * e * dt, -0.8, 0.8)
  local u = s.I + kp * e + kd * de
  local prev = s.u or (inp.cur or u)
  local pr = P.p_rate * min(1, 0.3 + s.t / 3)             -- elevator rate builds up after engagement
  u = clamp(u, prev - pr * dt, prev + pr * dt)
  u = clamp(u, -1, 1)
  s.u = u
  return u
end

-- auto-trim moved the stabilizer: take the same amount out of the integrator (keeps the elevator steady)
function M.pitch_trim_moved(st, d_elev_equiv)
  local s = st.p
  if s.I then s.I = clamp(s.I - d_elev_equiv, -0.8, 0.8) end
end

function M.roll(st, inp)
  local P = M.P
  local s = st.r
  local dt = max(0.005, inp.dt or 0.05)
  local ph, c = inp.phi, clamp(inp.cmd or 0, -25, 25)       -- v34: bank command never beyond 25 deg
  if s.ph == nil then s.ph = ph; s.c = ph; s.phd = 0; s.cd = 0; s.I = nil; s.cref = ph; s.t = 0; s.cs = ph end
  -- v34: bank command slewed at up to 4 deg/s (smoother roll-in, no over-input in tight turns)
  s.cs = (s.cs or ph) + clamp(c - (s.cs or ph), -4 * dt, 4 * dt)
  c = s.cs
  -- engagement: the bank target starts at the bank being held and moves to the guidance command at a
  -- limited rate (about 2 deg/s, rising to unrestricted over 8 s), so the aircraft is taken over smoothly
  s.t = (s.t or 99) + dt
  if not s.free then
    -- bank target rate: 2 deg/s at engagement, up to 4 deg/s; normal guidance once caught up
    local rmax = min(4.0, 2.0 + s.t * 0.4)
    s.cref = s.cref + clamp(c - s.cref, -rmax * dt, rmax * dt)
    if s.t > 1.0 and abs(c - s.cref) < 0.3 then s.free = true end
    if s.t > 30 then s.free = true end
    c = s.cref
  end
  local phd = (ph - s.ph) / dt
  s.phd = s.phd + (phd - s.phd) * min(1, dt / 0.1)
  local cd = (c - s.c) / dt
  s.cd = s.cd + (cd - s.cd) * min(1, dt / 0.3)
  s.ph, s.c = ph, c
  local q = qscale(inp.ias, P)
  local tu = inp.tune or {}
  local kp = P.r_kp * mult(tu.kp) / q
  local kd = P.r_kd * mult(tu.kd) / q
  local ki = P.r_ki * mult(tu.ki) / q
  local e = c - ph
  local de = P.r_ff * s.cd - s.phd
  if s.I == nil then s.I = clamp((inp.cur or 0) - kp * e - kd * de, -0.3, 0.3) end
  if abs(e) < 5 then s.I = clamp(s.I + ki * e * dt, -0.3, 0.3) end
  local u = s.I + kp * e + kd * de
  -- v34: bank protection - beyond 26 deg the aileron is driven back toward wings level
  if abs(ph) > 26 then
    local over = abs(ph) - 26
    local sg = ph > 0 and 1 or -1
    u = u - sg * min(0.4, over * 0.08)
    if u * sg > 0 then u = 0 end
  end
  local prev = s.u or (inp.cur or u)
  local rr = P.r_rate * min(1, 0.25 + s.t / 4)            -- aileron rate builds up after engagement
  u = clamp(u, prev - rr * dt, prev + rr * dt)
  u = clamp(u, -1, 1)
  s.u = u
  return u
end


B744INNER = M
return M
