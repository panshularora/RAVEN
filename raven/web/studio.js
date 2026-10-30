import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

const storyEl = document.getElementById("story");
const alarmEl = document.getElementById("alarm");
const clockEl = document.getElementById("clock");
const logEl = document.getElementById("log");
const blurbEl = document.getElementById("blurb");
const posEl = document.getElementById("pos");
const yawEl = document.getElementById("yaw");
const gripEl = document.getElementById("grip");
const wheelsEl = document.getElementById("wheels");
const hsvEl = document.getElementById("hsv");
const ikEl = document.getElementById("ik");
const valveNum = document.getElementById("valve-num");
const needle = document.getElementById("needle");
const playBtn = document.getElementById("play");
const restartBtn = document.getElementById("restart");
const followBtn = document.getElementById("follow");
const scrub = document.getElementById("scrub");
const camCanvas = document.getElementById("cam");
const camCtx = camCanvas.getContext("2d");
const stage = document.getElementById("stage");
const view = document.getElementById("view");
const phaseEls = [...document.querySelectorAll("#phases li")];
const briefingEl = document.getElementById("briefing");
const ordersEl = document.getElementById("orders");
const debriefEl = document.getElementById("debrief");
const minimap = document.getElementById("minimap");
const mapCtx = minimap.getContext("2d");
const sonarBars = document.getElementById("sonar-bars");
const sonarVals = document.getElementById("sonar-vals");
const hudPhase = document.getElementById("hud-phase");
const hudOrder = document.getElementById("hud-order");
const incSub = document.getElementById("inc-sub");
const howEl = document.getElementById("how");

const SONAR_BEARINGS = [-0.7, -0.35, 0, 0.35, 0.7];
const UP = new THREE.Vector3(0, 1, 0);

/** Plant yaw=0 faces +x. Object3D local forward is -z. Must match raven.sim.frames. */
export function yawToThree(yaw) {
  return -Math.PI / 2 - yaw;
}

export function plantToThree(x, y, z = 0) {
  return new THREE.Vector3(x, z, y);
}

function selfTestFrames(robot) {
  const y0 = yawToThree(0);
  const fx = -Math.sin(y0);
  const fz = -Math.cos(y0);
  console.assert(Math.abs(fx - 1) < 1e-8 && Math.abs(fz) < 1e-8, "yaw0 must face +x");
  const y90 = yawToThree(Math.PI / 2);
  const fx2 = -Math.sin(y90);
  const fz2 = -Math.cos(y90);
  console.assert(Math.abs(fx2) < 1e-8 && Math.abs(fz2 - 1) < 1e-8, "yaw90 must face +z");
  const saved = { x: robot.position.x, y: robot.position.y, z: robot.position.z, ry: robot.rotation.y };
  robot.position.set(1, 0.07, 1);
  robot.rotation.y = yawToThree(0);
  robot.updateMatrixWorld(true);
  const nose = robot.getObjectByName("nose");
  const nw = new THREE.Vector3();
  nose.getWorldPosition(nw);
  console.assert(nw.x > robot.position.x + 0.1, "nose must point +x at yaw 0");
  robot.position.set(saved.x, saved.y, saved.z);
  robot.rotation.y = saved.ry;
  window.__ravenFrameTest = true;
}

function unwrap(a, b) {
  let d = b - a;
  while (d > Math.PI) d -= Math.PI * 2;
  while (d < -Math.PI) d += Math.PI * 2;
  return d;
}

function lerp(a, b, u) {
  return a + (b - a) * u;
}

function sample(frames, t) {
  const last = frames[frames.length - 1];
  if (t >= last.t) return cloneFrame(last);
  let lo = 0;
  let hi = frames.length - 1;
  let i = 0;
  while (lo <= hi) {
    const mid = (lo + hi) >> 1;
    if (frames[mid].t <= t) {
      i = mid;
      lo = mid + 1;
    } else {
      hi = mid - 1;
    }
  }
  const a = frames[i];
  const b = frames[Math.min(i + 1, frames.length - 1)];
  if (b === a) return cloneFrame(a);
  const span = b.t - a.t;
  const u = span <= 1e-9 ? 1 : (t - a.t) / span;
  const arm = a.arm.map((p, idx) => p.map((v, k) => lerp(v, b.arm[idx][k], u)));
  const raysA = a.rays || [2, 2, 2, 2, 2];
  const raysB = b.rays || raysA;
  return {
    t,
    phase: u < 1 ? a.phase : b.phase,
    story: u < 1 ? a.story : b.story,
    x: lerp(a.x, b.x, u),
    y: lerp(a.y, b.y, u),
    yaw: a.yaw + unwrap(a.yaw, b.yaw) * u,
    q: a.q.map((v, idx) => lerp(v, b.q[idx], u)),
    grip: u < 1 ? a.grip : b.grip,
    leak: u < 1 ? a.leak : b.leak,
    valve: lerp(a.valve, b.valve, u),
    arm,
    wl: lerp(a.wl || 0, b.wl || 0, u),
    wr: lerp(a.wr || 0, b.wr || 0, u),
    rays: raysA.map((v, idx) => lerp(v, raysB[idx], u)),
    lc: u < 1 ? a.lc : b.lc,
    hc: u < 1 ? a.hc : b.hc,
    ik: u < 1 ? a.ik : b.ik,
  };
}

function cloneFrame(f) {
  return {
    ...f,
    q: f.q.slice(),
    arm: f.arm.map((p) => p.slice()),
    rays: (f.rays || []).slice(),
  };
}

function fmtClock(t) {
  const m = Math.floor(t / 60);
  const s = t - m * 60;
  return `T+${String(m).padStart(2, "0")}:${s.toFixed(1).padStart(4, "0")}`;
}

function tapeTexture() {
  const c = document.createElement("canvas");
  c.width = c.height = 64;
  const g = c.getContext("2d");
  g.fillStyle = "#1a1a12";
  g.fillRect(0, 0, 64, 64);
  g.strokeStyle = "#c9a227";
  g.lineWidth = 10;
  for (let i = -64; i < 128; i += 18) {
    g.beginPath();
    g.moveTo(i, 64);
    g.lineTo(i + 64, 0);
    g.stroke();
  }
  const t = new THREE.CanvasTexture(c);
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.repeat.set(1, 4);
  t.colorSpace = THREE.SRGBColorSpace;
  return t;
}

function stencilTexture(text, color) {
  const c = document.createElement("canvas");
  c.width = 1024;
  c.height = 256;
  const g = c.getContext("2d");
  g.clearRect(0, 0, 1024, 256);
  g.font = "700 168px Oswald, sans-serif";
  g.fillStyle = color;
  g.textAlign = "center";
  g.textBaseline = "middle";
  g.fillText(text, 512, 128);
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  t.needsUpdate = true;
  return t;
}

function makeLabel(text, color) {
  const c = document.createElement("canvas");
  c.width = 256;
  c.height = 64;
  const g = c.getContext("2d");
  g.font = "600 34px Oswald, sans-serif";
  g.fillStyle = color;
  g.textAlign = "center";
  g.textBaseline = "middle";
  g.fillText(text, 128, 32);
  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  const mat = new THREE.SpriteMaterial({ map: tex, transparent: true, depthTest: false });
  const s = new THREE.Sprite(mat);
  s.scale.set(1.15, 0.29, 1);
  return s;
}

function makeRobot() {
  const g = new THREE.Group();
  const body = new THREE.Mesh(
    new THREE.BoxGeometry(0.3, 0.14, 0.4),
    new THREE.MeshStandardMaterial({
      color: 0xc56a12,
      metalness: 0.35,
      roughness: 0.48,
      emissive: 0x4a2208,
      emissiveIntensity: 0.22,
    })
  );
  body.castShadow = true;
  body.receiveShadow = true;
  g.add(body);
  const mast = new THREE.Mesh(
    new THREE.CylinderGeometry(0.018, 0.018, 0.22, 8),
    new THREE.MeshStandardMaterial({ color: 0x2a2a24, metalness: 0.4, roughness: 0.5 })
  );
  mast.position.set(0, 0.16, 0.04);
  g.add(mast);
  const cam = new THREE.Mesh(
    new THREE.BoxGeometry(0.07, 0.05, 0.08),
    new THREE.MeshStandardMaterial({ color: 0x111110, metalness: 0.3, roughness: 0.4 })
  );
  cam.position.set(0, 0.28, 0.0);
  g.add(cam);
  const nose = new THREE.Mesh(
    new THREE.BoxGeometry(0.08, 0.05, 0.1),
    new THREE.MeshStandardMaterial({ color: 0x161612, metalness: 0.2, roughness: 0.7 })
  );
  nose.position.set(0, 0.03, -0.22);
  nose.name = "nose";
  nose.castShadow = true;
  g.add(nose);
  const beacon = new THREE.Mesh(
    new THREE.SphereGeometry(0.03, 10, 10),
    new THREE.MeshStandardMaterial({ color: 0xe23a2a, emissive: 0xe23a2a, emissiveIntensity: 0.9 })
  );
  beacon.position.set(0, 0.16, -0.12);
  beacon.name = "beacon";
  g.add(beacon);
  const wheelMat = new THREE.MeshStandardMaterial({ color: 0x1c1c18, roughness: 0.92 });
  const wheels = [];
  for (const x of [-0.16, 0.16]) {
    for (const z of [-0.1, 0.1]) {
      const w = new THREE.Mesh(new THREE.CylinderGeometry(0.05, 0.05, 0.04, 16), wheelMat);
      w.rotation.z = Math.PI / 2;
      w.position.set(x, -0.05, z);
      w.castShadow = true;
      g.add(w);
      wheels.push(w);
    }
  }
  g.userData.wheels = wheels;
  return g;
}

const _a = new THREE.Vector3();
const _b = new THREE.Vector3();
const _d = new THREE.Vector3();

function placeSegment(mesh, ax, ay, az, bx, by, bz) {
  _a.set(ax, ay, az);
  _b.set(bx, by, bz);
  _d.copy(_b).sub(_a);
  const len = _d.length();
  if (len < 1e-4) {
    mesh.visible = false;
    return;
  }
  mesh.visible = true;
  mesh.position.copy(_a).add(_b).multiplyScalar(0.5);
  mesh.quaternion.setFromUnitVectors(UP, _d.normalize());
  mesh.scale.set(1, len, 1);
}

function drawCam(frame) {
  const s = 64;
  camCtx.fillStyle = "#1e1e24";
  camCtx.fillRect(0, 0, s, s);
  if (frame.leak === 1) {
    camCtx.fillStyle = "#dc1414";
    camCtx.beginPath();
    const lx = frame.lc ? frame.lc[0] : 32;
    const ly = frame.lc ? frame.lc[1] : 16;
    camCtx.arc(lx, ly, 8, 0, Math.PI * 2);
    camCtx.fill();
  }
  camCtx.fillStyle = "#14c814";
  const hx = frame.hc ? frame.hc[0] : 48;
  const hy = frame.hc ? frame.hc[1] : 48;
  camCtx.fillRect(hx - 5, hy - 5, 10, 10);
  camCtx.strokeStyle = "rgba(94,224,168,0.25)";
  for (let y = 0; y < s; y += 8) {
    camCtx.beginPath();
    camCtx.moveTo(0, y);
    camCtx.lineTo(s, y);
    camCtx.stroke();
  }
}

function buildPlant(scene, world) {
  const floor = new THREE.Mesh(
    new THREE.BoxGeometry(world.w, 0.04, world.h),
    new THREE.MeshStandardMaterial({ color: 0x3d423c, roughness: 0.92, metalness: 0.05 })
  );
  floor.position.set(world.w / 2, -0.02, world.h / 2);
  floor.receiveShadow = true;
  scene.add(floor);

  const wallMat = new THREE.MeshStandardMaterial({ color: 0x161a16, roughness: 0.85 });
  const walls = [
    [world.w / 2, 0.7, 0, world.w, 1.4, 0.08],
    [world.w / 2, 0.7, world.h, world.w, 1.4, 0.08],
    [0, 0.7, world.h / 2, 0.08, 1.4, world.h],
    [world.w, 0.7, world.h / 2, 0.08, 1.4, world.h],
  ];
  for (const [x, y, z, sx, sy, sz] of walls) {
    const m = new THREE.Mesh(new THREE.BoxGeometry(sx, sy, sz), wallMat);
    m.position.set(x, y, z);
    m.castShadow = true;
    m.receiveShadow = true;
    scene.add(m);
  }

  const grid = new THREE.GridHelper(8, 8, 0x3a433c, 0x222820);
  grid.position.set(4, 0.005, 3);
  grid.material.transparent = true;
  grid.material.opacity = 0.55;
  scene.add(grid);

  const [ox, oy] = world.obstacle.c;
  const [sx, sy] = world.obstacle.s;
  const obstacle = new THREE.Mesh(
    new THREE.BoxGeometry(sx, 0.8, sy),
    new THREE.MeshStandardMaterial({ map: tapeTexture(), roughness: 0.7 })
  );
  obstacle.position.set(ox, 0.4, oy);
  obstacle.castShadow = true;
  obstacle.receiveShadow = true;
  scene.add(obstacle);

  const [tx, ty] = world.tank;
  const tank = new THREE.Mesh(
    new THREE.CylinderGeometry(0.26, 0.26, 0.9, 28),
    new THREE.MeshStandardMaterial({ color: 0x3d6e8c, metalness: 0.55, roughness: 0.32 })
  );
  tank.position.set(tx, 0.45, ty);
  tank.castShadow = true;
  tank.receiveShadow = true;
  scene.add(tank);

  const pipeLen = Math.abs(ty - world.valve[1]);
  const pipe = new THREE.Mesh(
    new THREE.CylinderGeometry(0.04, 0.04, pipeLen, 12),
    new THREE.MeshStandardMaterial({ color: 0x6a7370, metalness: 0.7, roughness: 0.28 })
  );
  pipe.position.set(tx, 0.25, (ty + world.valve[1]) / 2);
  pipe.rotation.x = Math.PI / 2;
  scene.add(pipe);

  const leak = new THREE.Mesh(
    new THREE.SphereGeometry(0.11, 16, 16),
    new THREE.MeshStandardMaterial({
      color: 0xe23a2a,
      emissive: 0x7a120c,
      emissiveIntensity: 0.8,
      roughness: 0.4,
    })
  );
  leak.position.set(tx, 0.28, ty - 0.28);
  leak.name = "leak";
  scene.add(leak);

  const spray = new THREE.Mesh(
    new THREE.ConeGeometry(0.09, 0.28, 12),
    new THREE.MeshStandardMaterial({
      color: 0xe23a2a,
      transparent: true,
      opacity: 0.55,
      roughness: 0.3,
    })
  );
  spray.position.set(tx, 0.12, ty - 0.38);
  spray.rotation.x = 0.7;
  spray.name = "spray";
  scene.add(spray);

  const valve = new THREE.Group();
  const green = new THREE.MeshStandardMaterial({ color: 0x2db34a, metalness: 0.4, roughness: 0.35 });
  const stem = new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.03, 0.18, 12), green);
  stem.position.y = 0.09;
  const handle = new THREE.Mesh(new THREE.BoxGeometry(0.04, 0.035, 0.22), green);
  handle.position.y = 0.2;
  handle.name = "handle";
  handle.castShadow = true;
  valve.add(stem, handle);
  valve.position.set(world.valve[0], 0, world.valve[1]);
  scene.add(valve);

  const home = new THREE.Mesh(
    new THREE.PlaneGeometry(0.9, 0.9),
    new THREE.MeshBasicMaterial({ color: 0x3a3420, transparent: true, opacity: 0.55, side: THREE.DoubleSide })
  );
  home.rotation.x = -Math.PI / 2;
  home.position.set(world.home[0], 0.01, world.home[1]);
  scene.add(home);

  const bay = makeLabel("BAY 3", "#d7d3c4");
  bay.scale.set(1.4, 0.35, 1);
  bay.position.set(4.0, 1.25, 0.22);
  scene.add(bay);

  const stencil = makeLabel("LEAK", "#e23a2a");
  stencil.scale.set(1.8, 0.45, 1);
  stencil.position.set(5.55, 0.62, 5.05);
  stencil.name = "stencil";
  scene.add(stencil);

  const homeLabel = makeLabel("HOME", "#d7d3c4");
  homeLabel.position.set(world.home[0], 0.55, world.home[1]);
  scene.add(homeLabel);
  const tankLabel = makeLabel("T-04", "#8eb4c8");
  tankLabel.position.set(tx, 1.15, ty);
  scene.add(tankLabel);
  const valveLabel = makeLabel("V-17", "#6fe07a");
  valveLabel.position.set(world.valve[0], 0.7, world.valve[1] - 0.35);
  scene.add(valveLabel);

  const leakLight = new THREE.PointLight(0xe23a2a, 1.4, 4.5, 2);
  leakLight.position.set(tx, 0.6, ty - 0.2);
  leakLight.name = "leakLight";
  scene.add(leakLight);

  return { leak, spray, handle, stencil, leakLight };
}

function drawMinimap(mission, frame, pathPts) {
  const g = mission.grid;
  const w = minimap.width;
  const h = minimap.height;
  mapCtx.fillStyle = "#080a08";
  mapCtx.fillRect(0, 0, w, h);
  const worldW = mission.world.w;
  const worldH = mission.world.h;
  const sx = w / worldW;
  const sy = h / worldH;
  const toX = (x) => x * sx;
  const toY = (y) => h - y * sy;
  if (g && g.rows) {
    const cellW = w / g.nx;
    const cellH = h / g.ny;
    mapCtx.fillStyle = "#2a322c";
    for (let iy = 0; iy < g.ny; iy += 1) {
      const row = g.rows[iy];
      for (let ix = 0; ix < g.nx; ix += 1) {
        if (row[ix] === "1") {
          mapCtx.fillRect(ix * cellW, h - (iy + 1) * cellH, cellW + 0.4, cellH + 0.4);
        }
      }
    }
  }
  if (pathPts && pathPts.length > 1) {
    mapCtx.strokeStyle = "#e0a21a";
    mapCtx.lineWidth = 1.5;
    mapCtx.beginPath();
    pathPts.forEach(([x, y], i) => {
      if (i === 0) mapCtx.moveTo(toX(x), toY(y));
      else mapCtx.lineTo(toX(x), toY(y));
    });
    mapCtx.stroke();
  }
  const [tx, ty] = mission.world.tank;
  mapCtx.fillStyle = "#3d6e8c";
  mapCtx.beginPath();
  mapCtx.arc(toX(tx), toY(ty), 5, 0, Math.PI * 2);
  mapCtx.fill();
  const [vx, vy] = mission.world.valve;
  mapCtx.fillStyle = "#2db34a";
  mapCtx.fillRect(toX(vx) - 3, toY(vy) - 3, 6, 6);
  mapCtx.fillStyle = "#c56a12";
  const px = toX(frame.x);
  const py = toY(frame.y);
  mapCtx.save();
  mapCtx.translate(px, py);
  mapCtx.rotate(-frame.yaw);
  mapCtx.beginPath();
  mapCtx.moveTo(8, 0);
  mapCtx.lineTo(-5, 5);
  mapCtx.lineTo(-5, -5);
  mapCtx.closePath();
  mapCtx.fill();
  mapCtx.restore();
  if (frame.rays) {
    mapCtx.strokeStyle = "rgba(94,224,168,0.7)";
    mapCtx.lineWidth = 1;
    frame.rays.forEach((r, i) => {
      const ang = frame.yaw + SONAR_BEARINGS[i];
      mapCtx.beginPath();
      mapCtx.moveTo(px, py);
      mapCtx.lineTo(toX(frame.x + r * Math.cos(ang)), toY(frame.y + r * Math.sin(ang)));
      mapCtx.stroke();
    });
  }
}

function fillSonar(rays) {
  const vals = rays || [2, 2, 2, 2, 2];
  if (!sonarBars.childElementCount) {
    vals.forEach(() => sonarBars.append(document.createElement("i")));
  }
  [...sonarBars.children].forEach((el, i) => {
    const pct = Math.max(6, Math.min(100, (vals[i] / 2) * 100));
    el.style.height = `${pct}%`;
    el.style.opacity = vals[i] < 0.6 ? "1" : "0.7";
    el.style.background = vals[i] < 0.5 ? "#ff4d3a" : "#5ee0a8";
  });
  sonarVals.textContent = vals.map((v) => v.toFixed(2)).join("  ") + " m";
}

function outcomeCopy(result) {
  const reason = result.reason;
  if (reason === "valve_closed") {
    return {
      title: "SEALED",
      kicker: "Isolation complete",
      body: "V-17 is at 90°. Leak flag is false. Tank T-04 is isolated. Cell can stay locked.",
    };
  }
  if (reason === "leak_confirmed") {
    return {
      title: "HOLD",
      kicker: "Inspect only",
      body: "Leak is confirmed on the wrist camera. The valve was left open on purpose. Evidence is in the log.",
    };
  }
  if (reason === "operator_request") {
    return {
      title: "HUMAN",
      kicker: "Operator abort",
      body: "Autonomy stopped on your order. Valve is still open. A person has to enter the cell.",
    };
  }
  if (reason === "valve_not_closed") {
    return {
      title: "SEIZED",
      kicker: "Two failed closes",
      body: "The handle would not seat. Two INTERVENE attempts ran. HUMAN_ASSIST is the safe stop.",
    };
  }
  return {
    title: result.status,
    kicker: result.reason,
    body: `Mission ended ${result.status} / ${result.reason}. Valve ${result.valve_angle.toFixed(0)}°.`,
  };
}

try {
  const res = await fetch("./mission.json");
  if (!res.ok) throw new Error(`mission.json ${res.status}`);
  const mission = await res.json();
  window.__ravenMission = mission;

  const isStory = mission.mode === "story" && mission.branches;
  const gateT = isStory ? Number(mission.gate_t) : Infinity;
  const briefing = mission.briefing || {};

  blurbEl.textContent = mission.blurb || "";
  storyEl.textContent = mission.frames[0].story;
  if (briefing.incident_id) {
    incSub.textContent = `${briefing.incident_id} · ${briefing.site || "Bay 3"}`;
  }
  (mission.methods || []).forEach((line) => {
    const li = document.createElement("li");
    li.textContent = line;
    howEl.append(li);
  });

  if (isStory) {
    document.getElementById("ticket-when").textContent = briefing.when || "";
    document.getElementById("ticket-site").textContent = briefing.site || "";
    document.getElementById("ticket-solving").textContent = briefing.solving || "";
    document.getElementById("ticket-problem").textContent = briefing.problem || mission.blurb;
    const methods = document.getElementById("ticket-methods");
    (mission.methods || []).forEach((line) => {
      const li = document.createElement("li");
      li.textContent = line;
      methods.append(li);
    });
    const list = document.getElementById("order-list");
    (mission.actions || []).forEach((act) => {
      const btn = document.createElement("button");
      btn.type = "button";
      btn.className = "order-btn";
      btn.dataset.id = act.id;
      btn.innerHTML = `<strong>${act.title}</strong><span>${act.detail}</span>`;
      list.append(btn);
    });
  }

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0a0c0a);
  scene.fog = new THREE.Fog(0x0a0c0a, 16, 34);

  const camera = new THREE.PerspectiveCamera(46, 1, 0.05, 80);
  camera.position.set(0.25, 5.4, -2.5);

  const renderer = new THREE.WebGLRenderer({ canvas: view, antialias: true, alpha: false });
  renderer.setPixelRatio(Math.min(2, window.devicePixelRatio || 1));
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.05;
  renderer.outputColorSpace = THREE.SRGBColorSpace;

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;
  controls.target.set(3.8, 0.2, 2.7);
  controls.maxPolarAngle = Math.PI / 2 - 0.05;
  controls.minDistance = 2.2;
  controls.maxDistance = 18;

  scene.add(new THREE.HemisphereLight(0xc5c8bc, 0x2a2e28, 0.85));
  const sun = new THREE.DirectionalLight(0xf2efe4, 1.35);
  sun.position.set(2.4, 9.5, 1.2);
  sun.castShadow = true;
  sun.shadow.mapSize.set(2048, 2048);
  sun.shadow.camera.left = -8;
  sun.shadow.camera.right = 8;
  sun.shadow.camera.top = 8;
  sun.shadow.camera.bottom = -8;
  sun.shadow.camera.near = 0.5;
  sun.shadow.camera.far = 24;
  scene.add(sun);

  const plant = buildPlant(scene, mission.world);
  const robot = makeRobot();
  scene.add(robot);
  selfTestFrames(robot);

  const steel = new THREE.MeshStandardMaterial({ color: 0xb9b6ad, metalness: 0.62, roughness: 0.3 });
  const jointMat = new THREE.MeshStandardMaterial({ color: 0x3e3e38, metalness: 0.5, roughness: 0.4 });
  const armLinks = [];
  const armJoints = [];
  for (let i = 0; i < 6; i += 1) {
    const link = new THREE.Mesh(new THREE.CylinderGeometry(0.022, 0.022, 1, 8), steel);
    link.castShadow = true;
    scene.add(link);
    armLinks.push(link);
    const joint = new THREE.Mesh(new THREE.SphereGeometry(0.032, 12, 12), jointMat);
    scene.add(joint);
    armJoints.push(joint);
  }
  const gripMat = new THREE.MeshStandardMaterial({ color: 0x222220, metalness: 0.3, roughness: 0.55 });
  const gripL = new THREE.Mesh(new THREE.BoxGeometry(0.02, 0.05, 0.08), gripMat);
  const gripR = new THREE.Mesh(new THREE.BoxGeometry(0.02, 0.05, 0.08), gripMat);
  gripL.castShadow = true;
  gripR.castShadow = true;
  scene.add(gripL, gripR);

  const sonarMat = new THREE.LineBasicMaterial({ color: 0x5ee0a8, transparent: true, opacity: 0.85 });
  const sonarLines = SONAR_BEARINGS.map(() => {
    const geom = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3(1, 0, 0)]);
    const line = new THREE.Line(geom, sonarMat);
    scene.add(line);
    return line;
  });

  const pathGeom = new THREE.BufferGeometry();
  const pathLine = new THREE.Line(
    pathGeom,
    new THREE.LineBasicMaterial({ color: 0xe0a21a, transparent: true, opacity: 0.7 })
  );
  scene.add(pathLine);

  const trailMax = 400;
  const trailPos = new Float32Array(trailMax * 3);
  const trailGeom = new THREE.BufferGeometry();
  trailGeom.setAttribute("position", new THREE.BufferAttribute(trailPos, 3));
  trailGeom.setDrawRange(0, 0);
  const trail = new THREE.Line(
    trailGeom,
    new THREE.LineBasicMaterial({ color: 0x8be36a, transparent: true, opacity: 0.85 })
  );
  scene.add(trail);

  const tape = {
    frames: mission.frames,
    paths: mission.paths,
    path: mission.path,
    result: mission.result,
    inspect: mission.inspect,
  };

  function durationOf(frames) {
    return frames[frames.length - 1].t;
  }

  function pathAt(t) {
    const paths = tape.paths && tape.paths.length ? tape.paths : [{ t: 0, pts: tape.path || [] }];
    let pts = paths[0].pts || [];
    for (const p of paths) {
      if (p.t <= t) pts = p.pts;
    }
    return pts || [];
  }
  function drawPath(t) {
    const pts = pathAt(t);
    if (pts.length > 1) {
      pathGeom.setFromPoints(pts.map(([x, y]) => plantToThree(x, y, 0.03)));
    }
    return pts;
  }

  const params = new URLSearchParams(location.search);
  const tParam = params.has("t") ? Number(params.get("t")) : null;
  const skipBrief = params.get("skip") === "1" || Number.isFinite(tParam);
  const play = {
    t: Number.isFinite(tParam) ? tParam : 0,
    speed: 1,
    playing: !isStory || skipBrief,
    follow: true,
    hold: skipBrief ? 0 : 0.4,
    lastStory: "",
    lastPhase: "",
    chosen: null,
    gated: false,
    debriefed: false,
  };

  if (isStory && !skipBrief) {
    briefingEl.hidden = false;
    play.playing = false;
  }

  const order = ["NAVIGATE", "INSPECT", "APPROACH", "PLAN", "INTERVENE", "VERIFY", "DONE", "HUMAN_ASSIST"];

  function markPhases(phase) {
    const idx = order.indexOf(phase);
    const retryIdx = phase === "RETRY" ? order.indexOf("INTERVENE") : idx;
    phaseEls.forEach((el) => {
      const p = el.dataset.phase;
      el.classList.toggle("on", p === phase || (phase === "RETRY" && p === "INTERVENE"));
      const pi = order.indexOf(p);
      el.classList.toggle("did", retryIdx >= 0 && pi >= 0 && pi < retryIdx);
    });
  }

  function setScrubMax() {
    scrub.max = String(Math.max(1, Math.round(durationOf(tape.frames) * 1000)));
  }
  setScrubMax();

  function apply(frame, dt = 0.016) {
    robot.position.set(frame.x, 0.07, frame.y);
    robot.rotation.y = yawToThree(frame.yaw);
    const wheels = robot.userData.wheels || [];
    const spin = ((frame.wl || 0) + (frame.wr || 0)) * 0.5 * dt;
    wheels.forEach((w) => {
      w.rotation.x += spin;
    });
    const beacon = robot.getObjectByName("beacon");
    if (beacon) {
      beacon.material.emissiveIntensity = frame.leak ? 0.6 + 0.6 * Math.sin(frame.t * 8) : 0.15;
    }

    const pts = frame.arm.map(([x, y, z]) => plantToThree(x, y, z));
    for (let i = 0; i < armJoints.length; i += 1) {
      armJoints[i].position.copy(pts[i]);
    }
    for (let i = 0; i < armLinks.length; i += 1) {
      const a = pts[i];
      const b = pts[i + 1];
      placeSegment(armLinks[i], a.x, a.y, a.z, b.x, b.y, b.z);
    }
    const tip = pts[pts.length - 1];
    const open = frame.grip ? 0.015 : 0.04;
    gripL.position.set(tip.x, tip.y, tip.z + open);
    gripR.position.set(tip.x, tip.y, tip.z - open);

    plant.handle.rotation.y = THREE.MathUtils.degToRad(-frame.valve);
    const leaking = frame.leak === 1;
    plant.leak.visible = leaking;
    plant.spray.visible = leaking;
    plant.spray.scale.setScalar(leaking ? 1 + 0.12 * Math.sin(frame.t * 9) : 1);
    plant.leakLight.intensity = leaking ? 1.4 : 0.0;
    if (plant.stencil.userData.leak !== leaking) {
      const map = plant.stencil.material.map;
      map?.dispose();
      plant.stencil.material.map = stencilTexture(leaking ? "LEAK" : "SEALED", leaking ? "#e23a2a" : "#8be36a");
      plant.stencil.material.needsUpdate = true;
      plant.stencil.userData.leak = leaking;
    }

    const rays = frame.rays || [2, 2, 2, 2, 2];
    rays.forEach((r, i) => {
      const ang = frame.yaw + SONAR_BEARINGS[i];
      const ox = frame.x;
      const oy = frame.y;
      const ex = ox + r * Math.cos(ang);
      const ey = oy + r * Math.sin(ang);
      sonarLines[i].geometry.setFromPoints([
        plantToThree(ox, oy, 0.12),
        plantToThree(ex, ey, 0.12),
      ]);
    });
    fillSonar(rays);

    alarmEl.textContent = leaking ? "LEAK" : "SEALED";
    alarmEl.classList.toggle("ok", !leaking);
    alarmEl.classList.toggle("alarm", leaking);
    clockEl.textContent = fmtClock(frame.t);
    posEl.textContent = `${frame.x.toFixed(2)}, ${frame.y.toFixed(2)} m`;
    yawEl.textContent = `${THREE.MathUtils.radToDeg(frame.yaw).toFixed(0)}°`;
    gripEl.textContent = frame.grip ? "CLOSED" : "OPEN";
    wheelsEl.textContent = `L ${Number(frame.wl || 0).toFixed(1)}  R ${Number(frame.wr || 0).toFixed(1)} rad/s`;
    const camLive = !["NAVIGATE", "IDLE"].includes(frame.phase);
    hsvEl.textContent = camLive
      ? `leak ${frame.lc ? "yes @" + frame.lc.map((n) => n.toFixed(0)).join(",") : "no"} · handle ${
          frame.hc ? "yes @" + frame.hc.map((n) => n.toFixed(0)).join(",") : "no"
        }`
      : "mast driving";
    ikEl.textContent = frame.ik == null ? "—" : `${Number(frame.ik).toFixed(3)} m`;
    valveNum.textContent = `${frame.valve.toFixed(0)}°`;
    needle.style.transform = `rotate(${-90 + frame.valve}deg)`;
    drawCam(camLive ? frame : { leak: 0, lc: null, hc: null });
    markPhases(frame.phase);
    hudPhase.textContent = frame.phase;
    hudOrder.textContent = play.chosen
      ? play.chosen
      : isStory
        ? frame.t >= gateT
          ? "awaiting order"
          : "en route"
        : "live tape";

    if (frame.story !== play.lastStory) {
      play.lastStory = frame.story;
      storyEl.textContent = frame.story;
      const li = document.createElement("li");
      const time = document.createElement("time");
      time.textContent = fmtClock(frame.t);
      li.append(time, document.createTextNode(frame.story));
      logEl.append(li);
      logEl.scrollTop = logEl.scrollHeight;
    }
    play.lastPhase = frame.phase;

    let n = 0;
    let lastX = null;
    let lastY = null;
    for (const f of tape.frames) {
      if (f.t > frame.t) break;
      if (lastX !== null && Math.hypot(f.x - lastX, f.y - lastY) < 0.05) continue;
      trailPos[n * 3] = f.x;
      trailPos[n * 3 + 1] = 0.04;
      trailPos[n * 3 + 2] = f.y;
      lastX = f.x;
      lastY = f.y;
      n += 1;
      if (n >= trailMax) break;
    }
    trailGeom.attributes.position.needsUpdate = true;
    trailGeom.setDrawRange(0, n);

    const pathPts = drawPath(frame.t);
    drawMinimap(mission, frame, pathPts);
    if (play.follow && play.hold <= 0 && briefingEl.hidden) {
      const back = new THREE.Vector3(0, 0, 1).applyQuaternion(robot.quaternion);
      const want = robot.position.clone().addScaledVector(back, 4.4).add(new THREE.Vector3(0, 3.1, 0));
      camera.position.lerp(want, 0.06);
      controls.target.lerp(robot.position.clone().add(new THREE.Vector3(0, 0.35, 0)), 0.1);
    }
  }

  function resize() {
    const r = stage.getBoundingClientRect();
    const w = Math.max(1, r.width);
    const h = Math.max(1, r.height);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    renderer.setSize(w, h, false);
  }
  resize();
  window.addEventListener("resize", resize);

  apply(sample(tape.frames, play.t));
  scrub.value = String(Math.round(play.t * 1000));
  playBtn.textContent = play.playing ? "Pause" : "Play";

  function showDebrief() {
    if (play.debriefed) return;
    play.debriefed = true;
    const copy = outcomeCopy(tape.result);
    document.getElementById("debrief-kicker").textContent = copy.kicker;
    document.getElementById("debrief-title").textContent = copy.title;
    document.getElementById("debrief-body").textContent = copy.body;
    const stats = document.getElementById("debrief-stats");
    stats.replaceChildren();
    const rows = [
      ["status", tape.result.status],
      ["reason", tape.result.reason],
      ["attempts", String(tape.result.attempts)],
      ["valve", `${Number(tape.result.valve_angle).toFixed(1)}°`],
      ["order", play.chosen || "close"],
    ];
    rows.forEach(([k, v]) => {
      const div = document.createElement("div");
      const dt = document.createElement("dt");
      const dd = document.createElement("dd");
      dt.textContent = k;
      dd.textContent = v;
      div.append(dt, dd);
      stats.append(div);
    });
    debriefEl.hidden = false;
  }

  function chooseOrder(id) {
    const branch = mission.branches[id];
    if (!branch) return;
    play.chosen = id;
    play.gated = true;
    tape.frames = branch.frames;
    tape.paths = branch.paths;
    tape.path = branch.path;
    tape.result = branch.result;
    tape.inspect = branch.inspect;
    setScrubMax();
    play.t = Math.min(gateT, durationOf(tape.frames));
    play.playing = true;
    play.debriefed = false;
    ordersEl.hidden = true;
    debriefEl.hidden = true;
    hudOrder.textContent = id;
    const li = document.createElement("li");
    const time = document.createElement("time");
    time.textContent = fmtClock(play.t);
    li.append(time, document.createTextNode(`Operator order: ${id}`));
    logEl.append(li);
  }

  let last = performance.now();
  function tick(now) {
    const dt = Math.min(0.05, (now - last) / 1000);
    last = now;
    const dur = durationOf(tape.frames);
    if (play.hold > 0) {
      play.hold -= dt;
    } else if (play.playing) {
      play.t = Math.min(dur, play.t + dt * play.speed);
      if (isStory && !play.chosen && play.t >= gateT) {
        play.t = gateT;
        play.playing = false;
        ordersEl.hidden = false;
      }
      if (play.t >= dur) {
        play.playing = false;
        if (play.chosen || !isStory) showDebrief();
      }
      playBtn.textContent = play.playing ? "Pause" : "Play";
      scrub.value = String(Math.round(play.t * 1000));
    }
    apply(sample(tape.frames, play.t), dt);
    controls.update();
    renderer.render(scene, camera);
    requestAnimationFrame(tick);
  }
  requestAnimationFrame(tick);

  document.getElementById("deploy").addEventListener("click", () => {
    briefingEl.hidden = true;
    play.playing = true;
    play.hold = 1.4;
    playBtn.textContent = "Pause";
  });
  document.getElementById("order-list").addEventListener("click", (e) => {
    const btn = e.target.closest("[data-id]");
    if (btn) chooseOrder(btn.dataset.id);
  });
  document.getElementById("debrief-again").addEventListener("click", () => {
    debriefEl.hidden = true;
    play.chosen = null;
    play.gated = false;
    play.debriefed = false;
    tape.frames = mission.frames;
    tape.paths = mission.paths;
    tape.path = mission.path;
    tape.result = mission.result;
    tape.inspect = mission.inspect;
    setScrubMax();
    play.t = 0;
    play.playing = true;
    play.lastStory = "";
    logEl.replaceChildren();
    ordersEl.hidden = true;
  });

  playBtn.addEventListener("click", () => {
    if (!briefingEl.hidden) return;
    if (isStory && !play.chosen && play.t >= gateT - 1e-3) {
      ordersEl.hidden = false;
      return;
    }
    play.playing = !play.playing;
    if (play.t >= durationOf(tape.frames)) play.t = 0;
    playBtn.textContent = play.playing ? "Pause" : "Play";
  });
  restartBtn.addEventListener("click", () => {
    briefingEl.hidden = true;
    debriefEl.hidden = true;
    ordersEl.hidden = true;
    play.t = 0;
    play.hold = 0.3;
    play.playing = true;
    play.chosen = null;
    play.debriefed = false;
    tape.frames = mission.frames;
    tape.paths = mission.paths;
    tape.path = mission.path;
    tape.result = mission.result;
    setScrubMax();
    play.lastStory = "";
    logEl.replaceChildren();
    playBtn.textContent = "Pause";
  });
  followBtn.addEventListener("click", () => {
    play.follow = !play.follow;
    followBtn.setAttribute("aria-pressed", play.follow ? "true" : "false");
  });
  scrub.addEventListener("input", () => {
    if (!briefingEl.hidden) return;
    let t = Number(scrub.value) / 1000;
    if (isStory && !play.chosen) t = Math.min(t, gateT);
    play.t = t;
    play.playing = false;
    play.hold = 0;
    playBtn.textContent = "Play";
  });
  document.querySelectorAll("#speeds button").forEach((btn) => {
    btn.addEventListener("click", () => {
      play.speed = Number(btn.dataset.speed);
      document.querySelectorAll("#speeds button").forEach((b) => b.classList.toggle("on", b === btn));
    });
  });
  window.addEventListener("keydown", (e) => {
    if (e.code === "Space") {
      e.preventDefault();
      playBtn.click();
    } else if (e.code === "KeyR") {
      restartBtn.click();
    } else if (e.code === "KeyF") {
      followBtn.click();
    } else if (e.code === "ArrowRight") {
      play.t = Math.min(durationOf(tape.frames), play.t + 0.5);
      if (isStory && !play.chosen) play.t = Math.min(play.t, gateT);
      scrub.value = String(Math.round(play.t * 1000));
    } else if (e.code === "ArrowLeft") {
      play.t = Math.max(0, play.t - 0.5);
      scrub.value = String(Math.round(play.t * 1000));
    }
  });
} catch (err) {
  storyEl.textContent =
    "Could not load the control room. Need a network connection once for the 3D library, and python -m raven must keep running.";
  console.error(err);
}
