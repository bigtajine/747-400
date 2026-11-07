--[[ B744 FMC lateral path builder.
     Converts the leg list into geometry: straight elements (great circle 'L'), arcs ('A', for RF/AF legs,
     fly-by turn transitions and holds) and heading elements ('H' for VA/VI/VM/VD/VR legs).
     Every leg gets: ps/pe (start/end point), len (nm), cum (distance from route start), tin/tout
     (true track at end/start), elems (guidance elements), trk_mag for the LEGS page.
     prof(i) supplies predicted {alt, gs, climb_grad} for dynamic legs and turn radii.
     Pure Lua; registered as global B744PATH. ]]
local M = {}
local G = B744GEO
local R = B744ROUTE
local abs, floor, min, max, sqrt, tan, atan2, sin, cos, pi =
  math.abs, math.floor, math.min, math.max, math.sqrt, math.tan, math.atan2, math.sin, math.cos, math.pi
local D2R, R2D = pi / 180, 180 / pi

local function pt(lat, lon) return { lat = lat, lon = lon } end

local function crs_true(l, ref)
  if l.crs == nil then return nil end
  if l.crs_is_true then return l.crs end
  local p = ref or l.fix or l.nav
  if not p then return l.crs end
  -- prefer navaid slaved variation when the course is defined from a navaid
  if l.nav and l.nav.kind == "VOR" and l.nav.slaved_var then return G.wrap360(l.crs + l.nav.slaved_var) end
  return G.mag_to_true(l.crs, p.lat, p.lon)
end
M.crs_true = crs_true

local function line_elem(a, b, i)
  local len = G.dist(a.lat, a.lon, b.lat, b.lon)
  return { k = "L", a = a, b = b, len = len, crs = G.brg(a.lat, a.lon, b.lat, b.lon), leg = i }
end

-- arc from start point s around center c (radius r) to end point e, dir 1 = right/cw, -1 = left/ccw
local function arc_elem(c, r, s, e, dir, i)
  local a0 = G.brg(c.lat, c.lon, s.lat, s.lon)
  local a1 = G.brg(c.lat, c.lon, e.lat, e.lon)
  local sweep
  if dir > 0 then sweep = (a1 - a0) % 360 else sweep = (a0 - a1) % 360 end
  if sweep < 0.01 then sweep = 0 end
  return { k = "A", c = c, r = r, a0 = a0, sweep = sweep, dir = dir, len = r * sweep * D2R, leg = i, s = s, e = e }
end
M.arc_elem = arc_elem

function M.arc_point(el, frac)
  local a = el.a0 + el.dir * el.sweep * frac
  local lat, lon = G.dest(el.c.lat, el.c.lon, a, el.r)
  return pt(lat, lon)
end

local function heading_elem(s, hdg, len, i)
  local lat, lon = G.dest(s.lat, s.lon, hdg, len)
  return { k = "H", a = s, b = pt(lat, lon), hdg = hdg, len = len, crs = hdg, leg = i }
end

-- distance along course line from s until DME distance d from navaid n
local function dme_point(s, crs, n, d)
  local step = 0.25
  local prev = G.dist(s.lat, s.lon, n.lat, n.lon)
  for k = 1, 400 do
    local lat, lon = G.dest(s.lat, s.lon, crs, k * step)
    local dd = G.dist(lat, lon, n.lat, n.lon)
    if (prev - d) * (dd - d) <= 0 then
      return k * step, pt(lat, lon)
    end
    prev = dd
  end
  local lat, lon = G.dest(s.lat, s.lon, crs, 5)
  return 5, pt(lat, lon)
end

-- end point of the "next leg line" for intercept legs
local function next_line(legs, i)
  local nl = legs[i + 1]
  if not nl or not nl.fix then return nil end
  if nl.t == "CF" and nl.crs then
    return nl.fix, crs_true(nl)
  end
  local nn = legs[i + 2]
  -- (a direct-to has no course to intercept: the intercept leg ends where the turn to the fix starts)
  if nl.t == "DF" then return nl.fix, nil end
  if nl.t == "TF" or nl.t == "IF" then
    if nn and nn.fix and (nn.t == "TF") then
      return nl.fix, G.brg(nl.fix.lat, nl.fix.lon, nn.fix.lat, nn.fix.lon)
    end
    return nl.fix, nil
  end
  return nl.fix, nil
end

local DYN = { CA = true, VA = true, FA = true, CD = true, VD = true, FD = true, CR = true, VR = true, CI = true, VI = true,
  VM = true, FM = true, FC = true }
M.DYN = DYN

-- build geometry for all legs.  start = {lat,lon} where the route begins (runway end / ppos)
function M.build(plan, prof, opts)
  local legs = plan.legs
  opts = opts or {}
  local pos = nil          -- current end point
  local trk = nil          -- current track (true)
  local cum = 0
  for i, l in ipairs(legs) do
    l.elems = {}
    l.len = 0
    l.ps = pos and pt(pos.lat, pos.lon) or nil
    local pr = prof and prof(i) or { alt = 0, gs = 250, grad = 500 }
    local t = l.t
    if t == "DISCO" then
      l.pe = nil
      pos = nil; trk = nil
    elseif t == "IF" then
      if l.fix then
        pos = pt(l.fix.lat, l.fix.lon)
        l.ps = pos; l.pe = pos
        if (l.rwy and opts.rwy_hdg and l.fix.ident == opts.dep_rwy) or l.rwy_hdg then
          trk = l.rwy_hdg or opts.rwy_hdg
          -- departure starts at the runway end
          local lat, lon = G.dest(pos.lat, pos.lon, trk, (l.rwy_len or opts.rwy_len or 9000) / 6076.12)
          pos = pt(lat, lon)
          l.pe = pos
        else
          trk = nil
        end
      end
    elseif t == "TF" or t == "DF" or (t == "CF" and not l.crs) or t == "PI" then
      local s = (t == "DF" and l.dt_start) and pt(l.dt_start.lat, l.dt_start.lon) or pos
      if not l.fix then
        l.pe = pos
      elseif not s then
        l.ps = pt(l.fix.lat, l.fix.lon); l.pe = l.ps
        pos = l.pe; trk = nil
      else
        l.ps = s
        local e = pt(l.fix.lat, l.fix.lon)
        local arc = nil
        -- procedure direct-to (SID: climb leg, then direct to a fix): turn from the climb track
        local p_trk = (t == "DF" and l.proc_df and not l.dt_start and trk) or nil
        l.dt_lead = nil
        if t == "DF" and ((l.dt_start and l.dt_trk) or p_trk) and G.dist(s.lat, s.lon, e.lat, e.lon) > 1.0 then
          -- DIRECT TO: turn from the track flown at execution onto the direct course, then straight
          local d_trk = l.dt_start and l.dt_trk or p_trk
          local dlt = G.angdiff(G.brg(s.lat, s.lon, e.lat, e.lon), d_trk)
          if abs(dlt) > 3 then
            local dir = dlt > 0 and 1 or -1
            local bank = (pr.alt or 0) > 25000 and 20 or 23
            -- turn radius from the predicted ground speed there (VNAV profile: FMC performance data)
            local gsk = math.max(150, l.dt_gs or pr.gs or 250)
            local r = G.turn_radius(gsk, bank)
            -- roll-in: the turn starts after the bank is established (3 deg/s, ~1.5 s lag) and the
            -- route is loaded (~1 s): straight on the present track until then (procedure turns:
            -- the bank build-up only)
            local lead = gsk * (bank / 3.0 / 2 + (l.dt_start and 2.5 or 0)) / 3600
            local sla, slo = G.dest(s.lat, s.lon, d_trk, lead)
            local s0 = s
            s = pt(sla, slo)
            l.dt_lead = line_elem(s0, s, i)
            local lac, loc = G.dest(s.lat, s.lon, G.wrap360(d_trk + dir * 90), r)
            local c = pt(lac, loc)
            local d = G.dist(c.lat, c.lon, e.lat, e.lon)
            if d > r * 1.05 then
              local th = math.deg(math.acos(r / d))
              local la, lo = G.dest(c.lat, c.lon, G.wrap360(G.brg(c.lat, c.lon, e.lat, e.lon) - dir * th), r)
              arc = arc_elem(c, r, s, pt(la, lo), dir, i)
              arc.tp = pt(la, lo)
            end
          end
        end
        local el
        if arc then
          el = line_elem(arc.tp, e, i)
          l.elems[1] = l.dt_lead
          l.elems[2] = arc
          l.elems[3] = el
          l.len = l.dt_lead.len + arc.len + el.len
          s = l.dt_lead.a
        else
          if l.dt_lead then s = l.dt_lead.a end
          el = line_elem(s, e, i)
          l.elems[1] = el
          l.len = el.len
        end
        l.pe = e
        pos = e
        trk = G.wrap360(G.brg(e.lat, e.lon, el.a.lat, el.a.lon) + 180)
        if el.len < 0.05 then trk = el.crs end
      end
    elseif t == "CF" then
      local e = pt(l.fix.lat, l.fix.lon)
      local c = crs_true(l)
      local s = pos
      if s then
        local b = G.brg(s.lat, s.lon, e.lat, e.lon)
        if G.dist(s.lat, s.lon, e.lat, e.lon) < 0.3 or abs(G.angdiff(b, c)) < 4 then
          -- previous fix is on the course
        else
          -- line starts on the inbound course, abeam the previous end
          local back = l.dist or min(15, G.dist(s.lat, s.lon, e.lat, e.lon))
          local lat, lon = G.dest(e.lat, e.lon, G.wrap360(c + 180), max(back, 1))
          s = pt(lat, lon)
          l.intercept = true
        end
      else
        local lat, lon = G.dest(e.lat, e.lon, G.wrap360(c + 180), l.dist or 5)
        s = pt(lat, lon)
      end
      l.ps = s
      local el = line_elem(s, e, i)
      el.crs_fixed = c
      l.elems[1] = el
      l.len = el.len
      if l.intercept and pos then
        l.len = G.dist(pos.lat, pos.lon, e.lat, e.lon)
      end
      l.pe = e; pos = e; trk = c
    elseif t == "RF" and l.fix and l.ctr and pos then
      local e = pt(l.fix.lat, l.fix.lon)
      local r = l.arc_r or G.dist(l.ctr.lat, l.ctr.lon, e.lat, e.lon)
      local dir = (l.turn == "L") and -1 or 1
      local el = arc_elem(pt(l.ctr.lat, l.ctr.lon), r, pos, e, dir, i)
      l.elems[1] = el
      l.len = el.len
      l.pe = e; pos = e
      trk = G.wrap360(G.brg(l.ctr.lat, l.ctr.lon, e.lat, e.lon) + dir * 90)
      l.arc = { r = r, dir = dir }
    elseif t == "AF" and l.fix and l.nav and pos then
      local e = pt(l.fix.lat, l.fix.lon)
      local r = l.rho or G.dist(l.nav.lat, l.nav.lon, e.lat, e.lon)
      local dir = (l.turn == "L") and -1 or 1
      local el = arc_elem(pt(l.nav.lat, l.nav.lon), r, pos, e, dir, i)
      l.elems[1] = el
      l.len = el.len
      l.pe = e; pos = e
      trk = G.wrap360(G.brg(l.nav.lat, l.nav.lon, e.lat, e.lon) + dir * 90)
      l.arc = { r = r, dir = dir }
    elseif t == "HM" or t == "HA" or t == "HF" then
      -- hold: racetrack at fix.  (entry line to the fix comes from the previous leg end)
      local f = pt(l.fix.lat, l.fix.lon)
      local h = l.hold or {}
      l.hold = h
      local inbd = G.mag_to_true(h.inbd or l.crs or (trk or 0), f.lat, f.lon)
      local gs = pr.gs or 220
      local tleg = h.time or ((pr.alt or 0) > 14000 and 1.5 or 1.0)
      local legd = h.dist or (gs * tleg / 60)
      local rr = G.turn_radius(gs, 25)
      local dir = (h.turn == "L") and -1 or 1
      h.inbd_true = inbd; h.legd = legd; h.r = rr; h.dir = dir
      -- racetrack points (local plane at fix)
      local ox, oy = sin(inbd * D2R), cos(inbd * D2R)     -- inbound unit vector
      local px, py = dir * oy, -dir * ox                   -- unit vector to the hold side
      local function P(x, y) local la, lo = G.from_local(f.lat, f.lon, x, y) return pt(la, lo) end
      local c1 = P(px * rr, py * rr)                                  -- outbound-turn center (at fix)
      local o1 = P(px * 2 * rr, py * 2 * rr)                          -- abeam fix, start outbound leg
      local o2 = P(px * 2 * rr - ox * legd, py * 2 * rr - oy * legd)  -- end outbound leg
      local c2 = P(px * rr - ox * legd, py * rr - oy * legd)          -- inbound-turn center
      local i1 = P(-ox * legd, -oy * legd)                             -- start of inbound leg
      h.pts = { f = f, c1 = c1, o1 = o1, o2 = o2, c2 = c2, i1 = i1 }
      local e0 = nil
      if pos and G.dist(pos.lat, pos.lon, f.lat, f.lon) > 0.2 then
        e0 = line_elem(pos, f, i)
        l.elems[#l.elems + 1] = e0
      end
      h.circuit = {
        arc_elem(c1, rr, f, o1, dir, i),
        line_elem(o1, o2, i),
        arc_elem(c2, rr, o2, i1, dir, i),
        line_elem(i1, f, i),
      }
      for _, e in ipairs(h.circuit) do e.hold = true end
      l.len = (e0 and e0.len or 0)
      l.hold_len = 2 * legd + 2 * pi * rr
      l.ps = pos or f
      l.pe = f; pos = f; trk = inbd
    elseif DYN[t] then
      local s = pos
      if t == "FA" or t == "FC" or t == "FD" or t == "FM" then
        if l.fix then s = pt(l.fix.lat, l.fix.lon) end
      end
      if l.dt_start then s = pt(l.dt_start.lat, l.dt_start.lon) end
      if not s then
        l.pe = nil; pos = nil
      else
        local c
        if t == "VA" or t == "VD" or t == "VI" or t == "VM" or t == "VR" then
          c = crs_true(l, s)
        else
          c = crs_true(l, s)
        end
        c = c or trk or 0
        local len = 2
        local e
        if t == "CA" or t == "FA" or t == "VA" then
          local tgt = l.alt and l.alt.a or 1500
          local a0 = l.from_elev or pr.alt or 0
          len = max(0.3, (tgt - a0) / max(150, pr.grad or 500))
        elseif t == "CD" or t == "VD" or t == "FD" then
          if l.nav then len, e = dme_point(s, c, l.nav, l.dist or 5) else len = l.dist or 5 end
        elseif t == "FC" then
          len = l.dist or 5
        elseif t == "CR" or t == "VR" then
          if l.nav and l.theta then
            local rad = (l.nav.slaved_var and G.wrap360(l.theta + l.nav.slaved_var)) or G.mag_to_true(l.theta, l.nav.lat, l.nav.lon)
            local la, lo = G.intersect(s.lat, s.lon, c, l.nav.lat, l.nav.lon, rad)
            if la then len = G.dist(s.lat, s.lon, la, lo); e = pt(la, lo) end
          end
        elseif t == "CI" or t == "VI" then
          local f, nc = next_line(legs, i)
          if f and nc then
            local la, lo = G.intersect(s.lat, s.lon, c, f.lat, f.lon, G.wrap360(nc + 180))
            if not la then la, lo = G.intersect(s.lat, s.lon, c, f.lat, f.lon, nc) end
            -- (an intercept further than the fix itself is not this course: the procedure's next course
            -- is behind - the leg ends close in and the next leg takes over)
            local dfix = G.dist(s.lat, s.lon, f.lat, f.lon)
            if la and G.dist(s.lat, s.lon, la, lo) < math.max(15, 1.5 * dfix) then len = G.dist(s.lat, s.lon, la, lo); e = pt(la, lo)
            else
              -- already (almost) on the next course: short intercept
              local la2, lo2 = G.dest(f.lat, f.lon, nc, 10)
              local x = G.xtk_atk(f.lat, f.lon, la2, lo2, s.lat, s.lon)
              if abs(x) < 2 then len = 0.5 end
            end
          elseif f then
            len = min(5, G.dist(s.lat, s.lon, f.lat, f.lon) * 0.5)
          end
        elseif t == "VM" or t == "FM" then
          len = 3
        end
        if not e then local la, lo = G.dest(s.lat, s.lon, c, len) e = pt(la, lo) end
        local el
        if t == "VA" or t == "VD" or t == "VI" or t == "VM" or t == "VR" then
          el = heading_elem(s, c, len, i)
          el.b = e
        else
          el = line_elem(s, e, i)
          el.crs_fixed = c
        end
        el.dyn = t
        l.elems[1] = el
        l.ps = s; l.len = len; l.pe = e; pos = e; trk = c
        l.dyn_crs = c
      end
    else
      -- unknown leg type: treat as direct to fix
      if l.fix and pos then
        local e = pt(l.fix.lat, l.fix.lon)
        local el = line_elem(pos, e, i)
        l.elems[1] = el; l.len = el.len; l.pe = e; pos = e; trk = el.crs
      elseif l.fix then
        pos = pt(l.fix.lat, l.fix.lon); l.pe = pos
      end
    end
    -- tracks at start and end of leg
    local e1 = l.elems[1]
    if e1 then
      if e1.k == "A" then l.tout = G.wrap360(e1.a0 + e1.dir * 90) else l.tout = e1.crs_fixed or e1.crs end
      local el = l.elems[#l.elems]
      if el.k == "A" then l.tin = G.wrap360(el.a0 + el.dir * (el.sweep + 90))
      elseif el.k == "L" then l.tin = el.crs_fixed or G.wrap360(G.brg(el.b.lat, el.b.lon, el.a.lat, el.a.lon) + 180)
      else l.tin = el.hdg end
    end
    l.pr = pr
  end

  -- fly-by turn transitions -----------------------------------------------------------------------
  -- each turn gets the radius its predicted speed needs (design bank); a straight segment shared by
  -- two turns is split in proportion to what each needs, so a fast turn is not squeezed into half the
  -- leg (a radius too small for the speed cannot be flown within the bank limit: overshoot)
  local function trans_need(k)
    local a, b = legs[k], legs[k + 1]
    if not (a and b) or a.flyover or not a.pe or not b.ps then return 0 end
    local ea, eb = a.elems and a.elems[#a.elems], b.elems and b.elems[1]
    if not (ea and eb) then return 0 end
    local t1, t2
    if ea.k == "L" then t1 = a.tin or ea.crs elseif ea.k == "A" and ea.c and ea.e then t1 = G.wrap360(G.brg(ea.c.lat, ea.c.lon, ea.e.lat, ea.e.lon) + ea.dir * 90) end
    if eb.k == "L" then t2 = eb.crs_fixed or eb.crs elseif eb.k == "A" and eb.c and eb.s then t2 = G.wrap360(G.brg(eb.c.lat, eb.c.lon, eb.s.lat, eb.s.lon) + eb.dir * 90) end
    if not t1 or not t2 then return 0 end
    local dl = abs(G.angdiff(t2, t1))
    if dl <= 2 or dl >= 150 then return 0 end
    local pr = b.pr or {}
    local r = G.turn_radius(pr.gs or 250, (pr.alt or 0) > 25000 and 20 or 23)
    return r * tan(dl * D2R / 2)
  end
  local function trans_lim(k, ea, eb, d)
    local la = 0.95 * (ea.len or 0)
    local L = 0.95 * (eb.len or 0)
    local lb = L
    local nb = legs[k + 1]
    if nb and nb.elems and #nb.elems == 1 then
      local nn = trans_need(k + 1)
      if d + nn > L then lb = L * d / math.max(1e-6, d + nn) end
    end
    return min(la, lb)
  end
  -- the largest turn radius the geometry allows here (whatever the speed): VNAV holds a speed this
  -- radius can be flown at within the LNAV bank limit (turn speed limit)
  local function trans_rmax(k, ea, eb, hd)
    local la = 0.95 * (ea.len or 0)
    local L = 0.95 * (eb.len or 0)
    local lb = L
    local nb = legs[k + 1]
    if nb and nb.elems and #nb.elems == 1 then lb = max(L - trans_need(k + 1), 0.5 * L) end
    return min(la, lb) / math.max(1e-3, tan(hd))
  end
  for i = 1, #legs - 1 do
    local a, b = legs[i], legs[i + 1]
    a.trans = nil
    local ea, eb = a.elems[#a.elems], b.elems[1]
    if ea and eb and ea.k == "L" and eb.k == "L" and not a.flyover and a.pe and b.ps
      and a.t ~= "DISCO" and b.t ~= "DISCO" and not DYN[a.t] and a.t ~= "HM" and a.t ~= "HA" and a.t ~= "HF"
      and b.t ~= "HM" and b.t ~= "HA" and b.t ~= "HF" and not b.dt_start
      and G.dist(a.pe.lat, a.pe.lon, b.ps.lat, b.ps.lon) < 0.2 then
      local t1 = a.tin
      local t2 = eb.crs_fixed or eb.crs
      local dlt = G.angdiff(t2, t1)
      if abs(dlt) > 2 and abs(dlt) < 150 then
        local pr = b.pr or {}
        local gs = pr.gs or 250
        -- design bank below the 25 deg LNAV limit, so the aircraft has margin to correct
        local bank = (pr.alt or 0) > 25000 and 20 or 23
        local r = G.turn_radius(gs, bank)
        local hd = abs(dlt) * D2R / 2
        local d = r * tan(hd)
        local rmax = trans_rmax(i, ea, eb, hd)
        local lim = trans_lim(i, ea, eb, d)
        if d > lim then d = lim; r = d / tan(hd) end
        -- a turn being flown keeps the radius it was built with (rebuild under the aircraft)
        local fr = opts and opts.force_r and opts.force_r[a]
        if fr then r = fr; d = r * tan(hd); lim = 1e9 end
        if d > 0.05 then
          local f = a.pe
          local la1, lo1 = G.dest(f.lat, f.lon, G.wrap360(t1 + 180), d)
          local la2, lo2 = G.dest(f.lat, f.lon, t2, d)
          local p1, p2 = pt(la1, lo1), pt(la2, lo2)
          local dir = dlt > 0 and 1 or -1
          local lac, loc = G.dest(la1, lo1, G.wrap360(t1 + dir * 90), r)
          local c = pt(lac, loc)
          local arc = arc_elem(c, r, p1, p2, dir, i)
          -- split arc at the bisector: first half belongs to leg a, second to leg b
          local mid = M.arc_point(arc, 0.5)
          local h1 = arc_elem(c, r, p1, mid, dir, i)
          local h2 = arc_elem(c, r, mid, p2, dir, i + 1)
          h1.trans = true; h2.trans = true
          -- shorten lines
          ea.b = p1; ea.len = G.dist(ea.a.lat, ea.a.lon, p1.lat, p1.lon)
          eb.a = p2; eb.len = G.dist(p2.lat, p2.lon, eb.b.lat, eb.b.lon)
          a.elems[#a.elems + 1] = h1
          table.insert(b.elems, 1, h2)
          a.trans = { d = d, r = r, dlt = dlt, rmax = rmax }
        end
      end
    end
  end
  -- fly-by turns onto and off DME arcs / radius-to-fix legs (e.g. KALO1S: SOORO, then the KMH 12 DME
  -- arc): a turn of the normal radius joining the straight line and the arc instead of a corner
  for i = 1, #legs - 1 do
    local a, b = legs[i], legs[i + 1]
    local ea, eb = a.elems[#a.elems], b.elems[1]
    local ARC = { AF = true, RF = true }
    if ea and eb and not a.flyover and a.pe and b.ps and not a.trans and a.t ~= "DISCO" and b.t ~= "DISCO"
      and not DYN[a.t] and G.dist(a.pe.lat, a.pe.lon, b.ps.lat, b.ps.lon) < 0.2
      and ((ea.k == "L" and eb.k == "A" and ARC[b.t]) or (ea.k == "A" and ARC[a.t] and eb.k == "L" and not b.dt_start)) then
      local t1, t2
      if ea.k == "L" then t1 = ea.crs else t1 = G.wrap360(G.brg(ea.c.lat, ea.c.lon, ea.e.lat, ea.e.lon) + ea.dir * 90) end
      if eb.k == "L" then t2 = eb.crs_fixed or eb.crs else t2 = G.wrap360(G.brg(eb.c.lat, eb.c.lon, eb.s.lat, eb.s.lon) + eb.dir * 90) end
      local dlt = G.angdiff(t2, t1)
      if abs(dlt) > 2 and abs(dlt) < 150 then
        local pr = b.pr or {}
        local gs = pr.gs or 250
        local bank = (pr.alt or 0) > 25000 and 20 or 23
        local r = G.turn_radius(gs, bank)
        local hd = abs(dlt) * D2R / 2
        local d = r * tan(hd)
        local rmax = trans_rmax(i, ea, eb, hd)
        local lim = trans_lim(i, ea, eb, d)
        if d > lim then d = lim; r = d / tan(hd) end
        -- a turn being flown keeps the radius it was built with (rebuild under the aircraft)
        local fr = opts and opts.force_r and opts.force_r[a]
        if fr then r = fr; d = r * tan(hd); lim = 1e9 end
        -- exact tangency to the arc: the fillet centre lies at r from the straight line (turn side)
        -- and at R -+ r from the arc centre; a large fillet on a DME arc must start where the arc
        -- tangent - not the arc's end tangent - meets it
        local dir = dlt > 0 and 1 or -1
        local f = a.pe
        local lin_l2a = (ea.k == "L")
        local arc = lin_l2a and eb or ea
        local th = (lin_l2a and t1 or t2) * D2R
        local ux, uy = sin(th), cos(th)
        local nx, ny = dir * uy, -dir * ux                    -- turn-side normal of the line
        local cx, cy = G.to_local(f.lat, f.lon, arc.c.lat, arc.c.lon)
        local exact = nil
        for _ = 1, 8 do
          local D = (dir == arc.dir) and (arc.r - r) or (arc.r + r)
          if D <= 0.01 then break end
          local wx, wy = nx * r - cx, ny * r - cy
          local bq = wx * ux + wy * uy
          local disc = bq * bq - (wx * wx + wy * wy - D * D)
          if disc < 0 then break end
          local sq = math.sqrt(disc)
          local want = lin_l2a and -d or d
          local l1, l2 = -bq - sq, -bq + sq
          local lam = (abs(l1 - want) < abs(l2 - want)) and l1 or l2
          if abs(lam) <= lim + 1e-6 and ((lin_l2a and lam <= 0.05) or (not lin_l2a and lam >= -0.05)) then
            local ox, oy = nx * r + ux * lam, ny * r + uy * lam
            local vx, vy = ox - cx, oy - cy
            local vl = math.sqrt(vx * vx + vy * vy)
            exact = { lam = lam, ox = ox, oy = oy, ax = cx + arc.r * vx / vl, ay = cy + arc.r * vy / vl }
            break
          end
          r = r * 0.85; d = r * tan(hd)
        end
        if d > 0.05 and exact then
          local la, lo = G.from_local(f.lat, f.lon, ux * exact.lam, uy * exact.lam)
          local pl = pt(la, lo)
          la, lo = G.from_local(f.lat, f.lon, exact.ax, exact.ay)
          local pa = pt(la, lo)
          la, lo = G.from_local(f.lat, f.lon, exact.ox, exact.oy)
          local c = pt(la, lo)
          local p1, p2
          if lin_l2a then p1, p2 = pl, pa else p1, p2 = pa, pl end
          d = abs(exact.lam)
          local fil = arc_elem(c, r, p1, p2, dir, i)
          local mid = M.arc_point(fil, 0.5)
          local h1 = arc_elem(c, r, p1, mid, dir, i)
          local h2 = arc_elem(c, r, mid, p2, dir, i + 1)
          h1.trans = true; h2.trans = true
          if ea.k == "L" then
            ea.b = p1; ea.len = G.dist(ea.a.lat, ea.a.lon, p1.lat, p1.lon)
          else
            a.elems[#a.elems] = arc_elem(ea.c, ea.r, ea.s, p1, ea.dir, i)
          end
          if eb.k == "L" then
            eb.a = p2; eb.len = G.dist(p2.lat, p2.lon, eb.b.lat, eb.b.lon)
          else
            b.elems[1] = arc_elem(eb.c, eb.r, p2, eb.e, eb.dir, i + 1)
          end
          a.elems[#a.elems + 1] = h1
          table.insert(b.elems, 1, h2)
          a.trans = { d = d, r = r, dlt = dlt, rmax = rmax }
        elseif d > 0.05 then
          local p1, p2          local lac, loc = G.dest(p1.lat, p1.lon, G.wrap360(t1 + dir * 90), r)
          local c = pt(lac, loc)
          local fil = arc_elem(c, r, p1, p2, dir, i)
          local mid = M.arc_point(fil, 0.5)
          local h1 = arc_elem(c, r, p1, mid, dir, i)
          local h2 = arc_elem(c, r, mid, p2, dir, i + 1)
          h1.trans = true; h2.trans = true
          if ea.k == "L" then
            ea.b = p1; ea.len = G.dist(ea.a.lat, ea.a.lon, p1.lat, p1.lon)
          else
            a.elems[#a.elems] = arc_elem(ea.c, ea.r, ea.s, p1, ea.dir, i)
          end
          if eb.k == "L" then
            eb.a = p2; eb.len = G.dist(p2.lat, p2.lon, eb.b.lat, eb.b.lon)
          else
            b.elems[1] = arc_elem(eb.c, eb.r, p2, eb.e, eb.dir, i + 1)
          end
          a.elems[#a.elems + 1] = h1
          table.insert(b.elems, 1, h2)
          a.trans = { d = d, r = r, dlt = dlt, rmax = rmax }
        end
      end
    end
  end
  -- leg lengths / cumulative distance ------------------------------------------------------------------
  cum = 0
  for i, l in ipairs(legs) do
    local len = 0
    for _, e in ipairs(l.elems or {}) do len = len + (e.len or 0) end
    if l.intercept and l.ps and l.pe then len = max(len, 0) end
    l.len = len
    cum = cum + len
    l.cum = cum
    -- magnetic track to display on LEGS page
    if l.t == "DISCO" then
      l.trk_mag = nil
    else
      local tt = (l.crs and (l.t == "CF" or DYN[l.t])) and nil or l.tin
      if l.crs and (l.t == "CF" or DYN[l.t]) then
        l.trk_mag = G.wrap360(floor(l.crs + 0.5))
      elseif tt and l.pe then
        l.trk_mag = G.true_to_mag(tt, l.pe.lat, l.pe.lon)
      end
    end
  end
  plan.total = cum
  return plan
end

-- distance-to-go and position helpers ---------------------------------------------------------------
-- along-track progress inside element: returns (atk from start, xtk(+right), track at that point)
function M.elem_track(e, lat, lon)
  if e.k == "A" then
    local d = G.dist(e.c.lat, e.c.lon, lat, lon)
    local b = G.brg(e.c.lat, e.c.lon, lat, lon)
    local ang
    if e.dir > 0 then ang = (b - e.a0) % 360 else ang = (e.a0 - b) % 360 end
    if ang > 180 + e.sweep / 2 then ang = ang - 360 end
    local atk = ang * D2R * e.r
    local xtk = (d - e.r) * (e.dir > 0 and -1 or 1)  -- right turn: outside of circle is left of path
    local trk = G.wrap360(b + e.dir * 90)
    return atk, xtk, trk, e.len - atk
  else
    local xtk, atk = G.xtk_atk(e.a.lat, e.a.lon, e.b.lat, e.b.lon, lat, lon)
    local trk
    if e.k == "H" then trk = e.hdg else trk = G.brg(e.a.lat, e.a.lon, e.b.lat, e.b.lon) end
    -- great-circle track at the abeam point
    if e.k == "L" and e.len > 20 and atk > 0 then
      local la, lo = G.dest(e.a.lat, e.a.lon, G.brg(e.a.lat, e.a.lon, e.b.lat, e.b.lon), math.min(atk, e.len))
      trk = G.brg(la, lo, e.b.lat, e.b.lon)
    end
    return atk, xtk, trk, e.len - atk
  end
end

B744PATH = M
