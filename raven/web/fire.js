/** Fire field + hunt. Numbers match raven/sim/fire.py */

const AMBIENT = 28;
const CORE = 820;
const THERMAL_LEN = 1.85;
const SMOKE_LEN = 2.4;
const VOC_K = 420;
const FLUX_K = 48;
const UV_RANGE = 2.3;
const RGB_RANGE = 1.55;
const TRACK = 0.3;
const WHEEL_R = 0.05;
const STANDOFF = 0.95;
const NEIGHBORS = [
  [-1, -1], [0, -1], [1, -1],
  [-1, 0], [1, 0],
  [-1, 1], [0, 1], [1, 1],
];

export function wrap(a) {
  const two = Math.PI * 2;
  let t = (a + Math.PI) % two;
  if (t < 0) t += two;
  return t - Math.PI;
}

export function parseGrid(g) {
  if (!g || !g.rows) return null;
  const rows = g.rows.map((r) => [...r].map((c) => c === "1"));
  return { nx: g.nx, ny: g.ny, res: g.res || 0.1, rows };
}

function occupied(grid, x, y) {
  if (!grid) return false;
  const ix = Math.floor(x / grid.res);
  const iy = Math.floor(y / grid.res);
  if (ix < 0 || iy < 0 || ix >= grid.nx || iy >= grid.ny) return true;
  return grid.rows[iy][ix];
}

export function lineOfSight(x0, y0, x1, y1, grid) {
  const dist = Math.hypot(x1 - x0, y1 - y0);
  if (dist < 1e-6) return 1;
  const steps = Math.max(2, Math.floor(dist / 0.08));
  for (let i = 1; i < steps; i += 1) {
    const u = i / steps;
    if (occupied(grid, x0 + (x1 - x0) * u, y0 + (y1 - y0) * u)) return 0.12;
  }
  return 1;
}

export function sampleSensors(rx, ry, yaw, fx, fy, intensity, t, grid) {
  const dx = fx - rx;
  const dy = fy - ry;
  const dist = Math.hypot(dx, dy);
  const bearing = Math.atan2(dy, dx);
  const facing = Math.cos(wrap(bearing - yaw));
  const los = lineOfSight(rx, ry, fx, fy, grid);
  const flicker = 0.92 + 0.08 * Math.sin(t * 11);
  const thermal =
    AMBIENT +
    intensity *
      (CORE - AMBIENT) *
      Math.exp(-dist / THERMAL_LEN) *
      (0.55 + 0.45 * Math.max(0, facing)) *
      los *
      flicker;
  const smoke = intensity * Math.exp(-dist / SMOKE_LEN) * los * (0.85 + 0.15 * Math.sin(t * 3.1));
  const voc = (intensity * VOC_K) / (dist * dist + 0.35);
  const uv = intensity > 0.35 && dist < UV_RANGE && facing > 0.15 && los > 0.5 ? 1 : 0;
  const flux = (intensity * FLUX_K) / (dist + 0.35) ** 2 * los;
  const rgb = intensity > 0.4 && dist < RGB_RANGE && facing > 0.25 && los > 0.4 ? 1 : 0;
  const pile = [];
  for (let i = 0; i < 8; i += 1) {
    const a = yaw + (-0.7 + i * (1.4 / 7));
    const f = Math.cos(wrap(bearing - a));
    pile.push(
      AMBIENT + intensity * (CORE - AMBIENT) * Math.exp(-dist / THERMAL_LEN) * Math.max(0, f) * los
    );
  }
  const activated = {
    "IR-TPA": thermal > 55,
    SMOKE: smoke > 0.12,
    VOC: voc > 80,
    "UV-FLAME": uv >= 0.4,
    HF: flux > 2,
    RGB: Boolean(rgb),
  };
  return {
    d: dist,
    bearing,
    facing,
    los,
    thermal,
    smoke: Math.max(0, smoke),
    voc,
    uv,
    flux,
    rgb,
    pile,
    activated,
    alarm: Object.values(activated).some(Boolean),
    located: activated["IR-TPA"] && activated["UV-FLAME"] && (activated.VOC || activated.RGB),
  };
}

export function snapFree(x, y, grid) {
  if (!grid) return [x, y];
  const ix0 = Math.floor(x / grid.res);
  const iy0 = Math.floor(y / grid.res);
  const tryCell = (ix, iy) => {
    if (ix < 1 || iy < 1 || ix >= grid.nx - 1 || iy >= grid.ny - 1) return null;
    if (grid.rows[iy][ix]) return null;
    return [(ix + 0.5) * grid.res, (iy + 0.5) * grid.res];
  };
  const hit = tryCell(ix0, iy0);
  if (hit) return hit;
  for (let r = 1; r < 14; r += 1) {
    for (let dx = -r; dx <= r; dx += 1) {
      for (let dy = -r; dy <= r; dy += 1) {
        const found = tryCell(ix0 + dx, iy0 + dy);
        if (found) return found;
      }
    }
  }
  return null;
}

export function standoffPose(fx, fy, from = { x: 1, y: 1 }) {
  const dx = fx - from.x;
  const dy = fy - from.y;
  const n = Math.hypot(dx, dy) || 1;
  const x = Math.min(7.6, Math.max(0.4, fx - (STANDOFF * dx) / n));
  const y = Math.min(5.6, Math.max(0.4, fy - (STANDOFF * dy) / n));
  return { x, y, yaw: Math.atan2(fy - y, fx - x) };
}

function octile(a, b) {
  const dx = Math.abs(a[0] - b[0]);
  const dy = Math.abs(a[1] - b[1]);
  return dx + dy + (Math.SQRT2 - 2) * Math.min(dx, dy);
}

function heapPush(heap, node) {
  heap.push(node);
  let i = heap.length - 1;
  while (i > 0) {
    const p = (i - 1) >> 1;
    if (heap[p][0] <= heap[i][0]) break;
    const tmp = heap[p];
    heap[p] = heap[i];
    heap[i] = tmp;
    i = p;
  }
}

function heapPop(heap) {
  const top = heap[0];
  const last = heap.pop();
  if (!heap.length) return top;
  heap[0] = last;
  let i = 0;
  for (;;) {
    const l = i * 2 + 1;
    const r = l + 1;
    let s = i;
    if (l < heap.length && heap[l][0] < heap[s][0]) s = l;
    if (r < heap.length && heap[r][0] < heap[s][0]) s = r;
    if (s === i) break;
    const tmp = heap[i];
    heap[i] = heap[s];
    heap[s] = tmp;
    i = s;
  }
  return top;
}

export function planPath(grid, startXy, goalXy) {
  if (!grid) return [goalXy];
  const toCell = (p) => [Math.floor(p[0] / grid.res), Math.floor(p[1] / grid.res)];
  const center = (c) => [(c[0] + 0.5) * grid.res, (c[1] + 0.5) * grid.res];
  const start = toCell(startXy);
  const goal = toCell(goalXy);
  if (start[0] === goal[0] && start[1] === goal[1]) return [goalXy];
  const blocked = (ix, iy) =>
    ix < 0 || iy < 0 || ix >= grid.nx || iy >= grid.ny || grid.rows[iy][ix];
  if (blocked(start[0], start[1]) || blocked(goal[0], goal[1])) return [goalXy];
  const key = (c) => `${c[0]},${c[1]}`;
  const gscore = new Map([[key(start), 0]]);
  const came = new Map();
  const heap = [];
  heapPush(heap, [octile(start, goal), start]);
  const closed = new Set();
  while (heap.length) {
    const current = heapPop(heap)[1];
    const ck = key(current);
    if (closed.has(ck)) continue;
    if (current[0] === goal[0] && current[1] === goal[1]) {
      const path = [current];
      let cur = ck;
      while (came.has(cur)) {
        const prev = came.get(cur);
        path.push(prev);
        cur = key(prev);
      }
      path.reverse();
      return path.map(center).concat([goalXy]);
    }
    closed.add(ck);
    for (const [dx, dy] of NEIGHBORS) {
      const nx = current[0] + dx;
      const ny = current[1] + dy;
      if (blocked(nx, ny)) continue;
      const neigh = [nx, ny];
      const nk = key(neigh);
      const step = dx && dy ? Math.SQRT2 : 1;
      const tentative = gscore.get(ck) + step;
      if (tentative >= (gscore.get(nk) ?? Infinity)) continue;
      came.set(nk, current);
      gscore.set(nk, tentative);
      heapPush(heap, [tentative + octile(neigh, goal), neigh]);
    }
  }
  return [goalXy];
}

export function wheelSpeeds(pose, xy) {
  const dx = xy[0] - pose.x;
  const dy = xy[1] - pose.y;
  const desired = Math.atan2(dy, dx);
  const err = wrap(desired - pose.yaw);
  let v;
  let w;
  if (Math.abs(err) > 0.45) {
    v = 0;
    w = Math.max(-1.2, Math.min(1.2, 2.2 * err));
  } else {
    const dist = Math.hypot(dx, dy);
    v = Math.min(0.4, 1.5 * dist);
    w = Math.max(-1.2, Math.min(1.2, 2.2 * err));
  }
  const half = TRACK / 2;
  return [(v - w * half) / WHEEL_R, (v + w * half) / WHEEL_R];
}

export function stepPose(pose, wl, wr, dt) {
  const v = (WHEEL_R * (wr + wl)) / 2;
  const w = (WHEEL_R * (wr - wl)) / TRACK;
  return {
    x: pose.x + v * Math.cos(pose.yaw) * dt,
    y: pose.y + v * Math.sin(pose.yaw) * dt,
    yaw: pose.yaw + w * dt,
  };
}

export function estimateSeat(pose, sense) {
  let peak = 0;
  let best = sense.pile[0];
  sense.pile.forEach((v, i) => {
    if (v > best) {
      best = v;
      peak = i;
    }
  });
  const ang = pose.yaw + (-0.7 + peak * (1.4 / 7));
  const rise = Math.max(0.02, (sense.thermal - AMBIENT) / (CORE - AMBIENT));
  const dest = Math.min(6, Math.max(0.6, -THERMAL_LEN * Math.log(rise)));
  return [pose.x + dest * Math.cos(ang), pose.y + dest * Math.sin(ang)];
}

export function rangeRays(pose, grid) {
  const bearings = [-0.7, -0.35, 0, 0.35, 0.7];
  return bearings.map((b) => {
    const ang = pose.yaw + b;
    let d = 0.08;
    while (d <= 2) {
      const x = pose.x + d * Math.cos(ang);
      const y = pose.y + d * Math.sin(ang);
      if (occupied(grid, x, y)) return d;
      d += 0.08;
    }
    return 2;
  });
}

function heatColor(t) {
  const u = Math.max(0, Math.min(1, (t - 28) / 500));
  if (u < 0.33) {
    const k = u / 0.33;
    return `rgb(${Math.round(41 + k * 180)},${Math.round(80 + k * 80)},${Math.round(180 - k * 80)})`;
  }
  if (u < 0.66) {
    const k = (u - 0.33) / 0.33;
    return `rgb(${Math.round(221)},${Math.round(160 - k * 80)},${Math.round(40)})`;
  }
  const k = (u - 0.66) / 0.34;
  return `rgb(239,${Math.round(80 - k * 40)},${Math.round(40 + k * 20)})`;
}

export function drawIR(ctx, sense, intensity) {
  const s = ctx.canvas.width;
  const img = ctx.createImageData(s, s);
  for (let y = 0; y < s; y += 1) {
    for (let x = 0; x < s; x += 1) {
      const nx = (x / s - 0.5) * 2;
      const ny = (y / s - 0.5) * 2;
      const r = Math.hypot(nx, ny * 1.1);
      const look = sense.facing;
      const hot = AMBIENT + (sense.thermal - AMBIENT) * Math.exp(-r * (2.2 - look)) * intensity;
      const u = Math.max(0, Math.min(1, (hot - 28) / 500));
      const i = (y * s + x) * 4;
      let cr;
      let cg;
      let cb;
      if (u < 0.33) {
        const k = u / 0.33;
        cr = 20 + k * 20;
        cg = 40 + k * 90;
        cb = 80 + k * 100;
      } else if (u < 0.66) {
        const k = (u - 0.33) / 0.33;
        cr = 40 + k * 200;
        cg = 130 - k * 40;
        cb = 180 - k * 140;
      } else {
        const k = (u - 0.66) / 0.34;
        cr = 240;
        cg = 90 - k * 50;
        cb = 40;
      }
      if (sense.rgb && r < 0.22) {
        cr = 255;
        cg = 180;
        cb = 40;
      }
      img.data[i] = cr;
      img.data[i + 1] = cg;
      img.data[i + 2] = cb;
      img.data[i + 3] = 255;
    }
  }
  ctx.putImageData(img, 0, 0);
  ctx.strokeStyle = "rgba(225,225,228,0.25)";
  ctx.strokeRect(0.5, 0.5, s - 1, s - 1);
}

export function drawFlameCam(ctx, sense, intensity) {
  const s = ctx.canvas.width;
  ctx.fillStyle = "#15151a";
  ctx.fillRect(0, 0, s, s);
  ctx.strokeStyle = "rgba(41,190,231,0.18)";
  for (let y = 0; y < s; y += 8) {
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(s, y);
    ctx.stroke();
  }
  if (intensity > 0.05 && sense.facing > -0.1) {
    const cx = 32 + (1 - Math.max(0, sense.facing)) * 8;
    const cy = 28;
    const rad = 6 + 10 * intensity * Math.max(0.2, sense.facing);
    const g = ctx.createRadialGradient(cx, cy, 1, cx, cy, rad);
    g.addColorStop(0, `rgba(255,220,80,${0.4 + 0.5 * intensity})`);
    g.addColorStop(0.45, `rgba(239,131,58,${0.5 * intensity})`);
    g.addColorStop(1, "rgba(245,73,102,0)");
    ctx.fillStyle = g;
    ctx.beginPath();
    ctx.arc(cx, cy, rad, 0, Math.PI * 2);
    ctx.fill();
  }
}

export function drawPile(root, pile) {
  if (!root) return;
  if (!root.childElementCount) {
    pile.forEach(() => root.append(document.createElement("i")));
  }
  [...root.children].forEach((el, i) => {
    const u = Math.max(0.08, Math.min(1, (pile[i] - 28) / 400));
    el.style.transform = `scaleY(${u})`;
    el.classList.toggle("hot", pile[i] > 55);
  });
}

export function drawGraph(ctx, history) {
  if (!ctx) return;
  const w = ctx.canvas.width;
  const h = ctx.canvas.height;
  ctx.fillStyle = "#121217";
  ctx.fillRect(0, 0, w, h);
  ctx.strokeStyle = "#2d2d33";
  ctx.beginPath();
  ctx.moveTo(36, 8);
  ctx.lineTo(36, h - 8);
  ctx.lineTo(w - 6, h - 8);
  ctx.stroke();
  const traces = [
    { key: "thermal", color: "#EF833A", max: 400, label: "IR" },
    { key: "voc", color: "#29bee7", max: 800, label: "VOC" },
    { key: "smoke", color: "#a7a6af", max: 1, label: "OD" },
    { key: "flux", color: "#f54966", max: 40, label: "HF" },
  ];
  const n = history.length;
  traces.forEach((tr) => {
    ctx.strokeStyle = tr.color;
    ctx.lineWidth = 1.4;
    ctx.beginPath();
    history.forEach((s, i) => {
      const x = 36 + (i / Math.max(1, n - 1)) * (w - 44);
      const y = h - 8 - (Math.min(1, s[tr.key] / tr.max) * (h - 16));
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.stroke();
  });
  ctx.font = "10px 'IBM Plex Mono', monospace";
  ctx.fillStyle = "#a7a6af";
  ctx.fillText("IR °C  VOC ppm  smoke  kW/m²", 40, 14);
}

export { heatColor, AMBIENT, STANDOFF };
