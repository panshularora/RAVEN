import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import {
  parseGrid,
  sampleSensors,
  snapFree,
  standoffPose,
  planPath,
  wheelSpeeds,
  stepPose,
  estimateSeat,
  rangeRays,
  wrap,
  drawIR,
  drawFlameCam,
  drawPile,
  drawGraph,
} from "./fire.js";
import { publish } from "./bus.js";

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
const spark = document.getElementById("spark");
const sparkCtx = spark ? spark.getContext("2d") : null;
const sparkBuf = [];
const irCanvas = document.getElementById("ir");
const irCtx = irCanvas ? irCanvas.getContext("2d") : null;
const graphCanvas = document.getElementById("graph");
const graphCtx = graphCanvas ? graphCanvas.getContext("2d") : null;
const pileEl = document.getElementById("pile");
const chipsEl = document.getElementById("chips");
const igniteHint = document.getElementById("ignite-hint");
const padsEl = document.getElementById("pads");
const smokeRead = document.getElementById("smoke-read");
const fluxRead = document.getElementById("flux-read");
const fireD = document.getElementById("fire-d");

const SONAR_BEARINGS = [-0.7, -0.35, 0, 0.35, 0.7];
const UP = new THREE.Vector3(0, 1, 0);
const C = {
  bg: 0x15151a,
  paper: 0x27272b,
  bar: 0x35363a,
  ink: 0xe1e1e4,
  amber: 0xef833a,
  cyan: 0x29bee7,
  ok: 0x92c353,
  alarm: 0xf54966,
  steel: 0x585861,
};

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
  g.fillStyle = "#1c1c22";
  g.fillRect(0, 0, 64, 64);
  g.strokeStyle = "#eba800";
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
  g.font = "700 168px 'IBM Plex Sans Condensed', sans-serif";
  g.fillStyle = color;
  g.textAlign = "center";
  g.textBaseline = "middle";
  g.fillText(text, 512, 128);
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  t.needsUpdate = true;
  return t;
}

function canvasText(text, color, w, h, size) {
  const c = document.createElement("canvas");
  c.width = w;
  c.height = h;
  const g = c.getContext("2d");
  g.clearRect(0, 0, w, h);
  g.font = `700 ${size}px 'IBM Plex Sans Condensed', sans-serif`;
  g.fillStyle = color;
  g.textAlign = "center";
  g.textBaseline = "middle";
  g.fillText(text, w / 2, h / 2);
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  t.needsUpdate = true;
  return t;
}

function concreteMap() {
  const c = document.createElement("canvas");
  c.width = c.height = 512;
  const g = c.getContext("2d");
  g.fillStyle = "#2a2a32";
  g.fillRect(0, 0, 512, 512);
  for (let i = 0; i < 1200; i += 1) {
    g.fillStyle = `rgba(0,0,0,${Math.random() * 0.12})`;
    g.fillRect(Math.random() * 512, Math.random() * 512, 3, 2);
  }
  g.strokeStyle = "#EF833A";
  g.lineWidth = 6;
  g.strokeRect(18, 18, 476, 476);
  g.strokeStyle = "rgba(41,190,231,0.22)";
  g.lineWidth = 2;
  g.beginPath();
  g.moveTo(256, 18);
  g.lineTo(256, 494);
  g.moveTo(18, 256);
  g.lineTo(494, 256);
  g.stroke();
  const t = new THREE.CanvasTexture(c);
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.repeat.set(2, 1.5);
  t.anisotropy = 8;
  t.colorSpace = THREE.SRGBColorSpace;
  return t;
}

function makeRobot() {
  const g = new THREE.Group();
  const bodyMat = new THREE.MeshStandardMaterial({
    color: C.amber,
    metalness: 0.42,
    roughness: 0.4,
    emissive: 0x4a220c,
    emissiveIntensity: 0.22,
  });
  const body = new THREE.Mesh(new THREE.BoxGeometry(0.34, 0.15, 0.46), bodyMat);
  body.castShadow = true;
  body.receiveShadow = true;
  g.add(body);
  const skirt = new THREE.Mesh(
    new THREE.BoxGeometry(0.36, 0.04, 0.48),
    new THREE.MeshStandardMaterial({ color: 0x1c1c22, metalness: 0.3, roughness: 0.7 })
  );
  skirt.position.y = -0.08;
  skirt.castShadow = true;
  g.add(skirt);
  const bumper = new THREE.Mesh(
    new THREE.BoxGeometry(0.3, 0.06, 0.04),
    new THREE.MeshStandardMaterial({ color: 0x121217, roughness: 0.65 })
  );
  bumper.position.set(0, -0.02, -0.25);
  g.add(bumper);
  const mast = new THREE.Mesh(
    new THREE.CylinderGeometry(0.02, 0.024, 0.26, 10),
    new THREE.MeshStandardMaterial({ color: C.bar, metalness: 0.45, roughness: 0.45 })
  );
  mast.position.set(0, 0.18, 0.05);
  g.add(mast);
  const cam = new THREE.Mesh(
    new THREE.BoxGeometry(0.08, 0.05, 0.09),
    new THREE.MeshStandardMaterial({ color: 0x15151a, metalness: 0.35, roughness: 0.35 })
  );
  cam.position.set(0, 0.32, -0.02);
  g.add(cam);
  const lens = new THREE.Mesh(
    new THREE.CylinderGeometry(0.016, 0.016, 0.02, 12),
    new THREE.MeshStandardMaterial({ color: 0x0e3a48, metalness: 0.7, roughness: 0.2, emissive: C.cyan, emissiveIntensity: 0.35 })
  );
  lens.rotation.x = Math.PI / 2;
  lens.position.set(0, 0.32, -0.07);
  g.add(lens);
  const nose = new THREE.Mesh(
    new THREE.BoxGeometry(0.08, 0.05, 0.1),
    new THREE.MeshStandardMaterial({ color: 0x1c1c22, metalness: 0.2, roughness: 0.7 })
  );
  nose.position.set(0, 0.03, -0.22);
  nose.name = "nose";
  nose.castShadow = true;
  g.add(nose);
  const sonarMat = new THREE.MeshStandardMaterial({ color: C.cyan, emissive: C.cyan, emissiveIntensity: 0.45 });
  for (const x of [-0.12, -0.06, 0, 0.06, 0.12]) {
    const puck = new THREE.Mesh(new THREE.CylinderGeometry(0.012, 0.012, 0.018, 8), sonarMat);
    puck.rotation.x = Math.PI / 2;
    puck.position.set(x, 0.02, -0.24);
    g.add(puck);
  }
  const beacon = new THREE.Mesh(
    new THREE.SphereGeometry(0.028, 12, 12),
    new THREE.MeshStandardMaterial({ color: C.alarm, emissive: C.alarm, emissiveIntensity: 0.9 })
  );
  beacon.position.set(0, 0.18, -0.14);
  beacon.name = "beacon";
  g.add(beacon);
  const wheelMat = new THREE.MeshStandardMaterial({ color: 0x1c1c22, roughness: 0.92 });
  const wheels = [];
  for (const x of [-0.17, 0.17]) {
    for (const z of [-0.12, 0.12]) {
      const w = new THREE.Mesh(new THREE.CylinderGeometry(0.055, 0.055, 0.045, 18), wheelMat);
      w.rotation.z = Math.PI / 2;
      w.position.set(x, -0.06, z);
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
  camCtx.fillStyle = "#15151a";
  camCtx.fillRect(0, 0, s, s);
  if (frame.leak === 1 && frame.lc) {
    camCtx.fillStyle = "#f54966";
    camCtx.beginPath();
    camCtx.arc(frame.lc[0], frame.lc[1], 8, 0, Math.PI * 2);
    camCtx.fill();
  }
  if (frame.hc) {
    camCtx.fillStyle = "#92c353";
    camCtx.fillRect(frame.hc[0] - 5, frame.hc[1] - 5, 10, 10);
  }
  camCtx.strokeStyle = "rgba(41,190,231,0.22)";
  for (let y = 0; y < s; y += 8) {
    camCtx.beginPath();
    camCtx.moveTo(0, y);
    camCtx.lineTo(s, y);
    camCtx.stroke();
  }
}

function makeMist(tx, ty) {
  const n = 90;
  const positions = new Float32Array(n * 3);
  const seeds = [];
  for (let i = 0; i < n; i += 1) {
    seeds.push(Math.random());
    positions[i * 3] = 0;
    positions[i * 3 + 1] = 0;
    positions[i * 3 + 2] = 0;
  }
  const geo = new THREE.BufferGeometry();
  geo.setAttribute("position", new THREE.BufferAttribute(positions, 3));
  const mist = new THREE.Points(
    geo,
    new THREE.PointsMaterial({
      color: C.alarm,
      size: 0.05,
      transparent: true,
      opacity: 0.82,
      depthWrite: false,
    })
  );
  mist.position.set(tx, 0.3, ty - 0.26);
  mist.userData.seeds = seeds;
  mist.name = "mist";
  return mist;
}

function tickMist(mist, leaking, t) {
  mist.visible = leaking;
  if (!leaking) return;
  const arr = mist.geometry.attributes.position.array;
  mist.userData.seeds.forEach((s, i) => {
    const u = (t * 0.85 + s) % 1;
    const spread = u * 0.28;
    arr[i * 3] = (s - 0.5) * spread * 1.4;
    arr[i * 3 + 1] = -u * 0.5;
    arr[i * 3 + 2] = -u * 0.62;
  });
  mist.geometry.attributes.position.needsUpdate = true;
}

function buildPlant(scene, world) {
  const pad = new THREE.Mesh(
    new THREE.BoxGeometry(22, 0.08, 18),
    new THREE.MeshStandardMaterial({ color: 0x121217, roughness: 0.95 })
  );
  pad.position.set(world.w / 2, -0.08, world.h / 2);
  pad.receiveShadow = true;
  scene.add(pad);

  const floor = new THREE.Mesh(
    new THREE.BoxGeometry(world.w, 0.05, world.h),
    new THREE.MeshStandardMaterial({ map: concreteMap(), roughness: 0.9, metalness: 0.04 })
  );
  floor.position.set(world.w / 2, -0.02, world.h / 2);
  floor.receiveShadow = true;
  scene.add(floor);

  const wallMat = new THREE.MeshStandardMaterial({ color: C.bar, roughness: 0.84, metalness: 0.06 });
  const walls = [
    [world.w / 2, 0.85, 0, world.w, 1.7, 0.1],
    [world.w / 2, 0.85, world.h, world.w, 1.7, 0.1],
    [0, 0.85, world.h / 2, 0.1, 1.7, world.h],
    [world.w, 0.85, world.h / 2, 0.1, 1.7, world.h],
  ];
  for (const [x, y, z, sx, sy, sz] of walls) {
    const m = new THREE.Mesh(new THREE.BoxGeometry(sx, sy, sz), wallMat);
    m.position.set(x, y, z);
    m.castShadow = true;
    m.receiveShadow = true;
    scene.add(m);
  }

  const baySign = new THREE.Mesh(
    new THREE.PlaneGeometry(3.2, 0.48),
    new THREE.MeshBasicMaterial({ map: canvasText("BAY 3  ·  CELL LOCKED", "#e1e1e4", 1024, 180, 92), transparent: true })
  );
  baySign.rotation.y = Math.PI;
  baySign.position.set(4.2, 1.22, world.h - 0.06);
  scene.add(baySign);

  const [ox, oy] = world.obstacle.c;
  const [sx, sy] = world.obstacle.s;
  const obstacle = new THREE.Mesh(
    new THREE.BoxGeometry(sx, 0.85, sy),
    new THREE.MeshStandardMaterial({ map: tapeTexture(), roughness: 0.68 })
  );
  obstacle.position.set(ox, 0.42, oy);
  obstacle.castShadow = true;
  obstacle.receiveShadow = true;
  scene.add(obstacle);

  const [tx, ty] = world.tank;
  const tankMat = new THREE.MeshStandardMaterial({ color: 0x4f8eaa, metalness: 0.22, roughness: 0.46 });
  const tank = new THREE.Mesh(new THREE.CylinderGeometry(0.28, 0.3, 0.95, 32), tankMat);
  tank.position.set(tx, 0.48, ty);
  tank.castShadow = true;
  tank.receiveShadow = true;
  scene.add(tank);
  const rim = new THREE.Mesh(
    new THREE.CylinderGeometry(0.3, 0.3, 0.05, 32),
    new THREE.MeshStandardMaterial({ color: 0x2a4a5c, metalness: 0.7, roughness: 0.25 })
  );
  rim.position.set(tx, 0.98, ty);
  scene.add(rim);
  for (const a of [0, Math.PI * 0.5, Math.PI, Math.PI * 1.5]) {
    const leg = new THREE.Mesh(
      new THREE.BoxGeometry(0.05, 0.22, 0.05),
      new THREE.MeshStandardMaterial({ color: 0x2a2e2c, metalness: 0.4, roughness: 0.5 })
    );
    leg.position.set(tx + Math.cos(a) * 0.22, 0.11, ty + Math.sin(a) * 0.22);
    scene.add(leg);
  }
  const t04 = new THREE.Mesh(
    new THREE.PlaneGeometry(0.5, 0.16),
    new THREE.MeshBasicMaterial({ map: canvasText("T-04", "#29bee7", 256, 80, 64), transparent: true })
  );
  t04.position.set(tx, 0.7, ty - 0.29);
  scene.add(t04);

  const pipeMat = new THREE.MeshStandardMaterial({ color: 0x6a7370, metalness: 0.74, roughness: 0.26 });
  const pipeLen = Math.abs(ty - world.valve[1]);
  const pipe = new THREE.Mesh(new THREE.CylinderGeometry(0.045, 0.045, pipeLen, 14), pipeMat);
  pipe.position.set(tx, 0.25, (ty + world.valve[1]) / 2);
  pipe.rotation.x = Math.PI / 2;
  scene.add(pipe);
  const flange = new THREE.Mesh(new THREE.CylinderGeometry(0.07, 0.07, 0.03, 14), pipeMat);
  flange.position.set(tx, 0.25, world.valve[1] + 0.08);
  flange.rotation.x = Math.PI / 2;
  scene.add(flange);

  const leak = new THREE.Mesh(
    new THREE.SphereGeometry(0.07, 14, 14),
    new THREE.MeshStandardMaterial({
      color: C.alarm,
      emissive: 0x7a1230,
      emissiveIntensity: 0.95,
      roughness: 0.35,
    })
  );
  leak.position.set(tx, 0.26, ty - 0.3);
  leak.name = "leak";
  scene.add(leak);

  const spray = new THREE.Mesh(
    new THREE.ConeGeometry(0.08, 0.26, 12),
    new THREE.MeshStandardMaterial({
      color: C.alarm,
      transparent: true,
      opacity: 0.32,
      roughness: 0.3,
      depthWrite: false,
    })
  );
  spray.position.set(tx, 0.12, ty - 0.4);
  spray.rotation.x = 0.75;
  spray.name = "spray";
  scene.add(spray);
  const mist = makeMist(tx, ty);
  scene.add(mist);

  const valve = new THREE.Group();
  const green = new THREE.MeshStandardMaterial({ color: C.ok, metalness: 0.45, roughness: 0.32 });
  const steel = new THREE.MeshStandardMaterial({ color: 0x6a7370, metalness: 0.7, roughness: 0.28 });
  const body = new THREE.Mesh(new THREE.CylinderGeometry(0.055, 0.055, 0.1, 14), steel);
  body.position.y = 0.05;
  const stem = new THREE.Mesh(new THREE.CylinderGeometry(0.022, 0.022, 0.16, 12), green);
  stem.position.y = 0.14;
  const handle = new THREE.Mesh(new THREE.BoxGeometry(0.035, 0.03, 0.24), green);
  handle.position.y = 0.23;
  handle.name = "handle";
  handle.castShadow = true;
  const cross = new THREE.Mesh(new THREE.BoxGeometry(0.24, 0.03, 0.035), green);
  cross.position.y = 0.23;
  valve.add(body, stem, handle, cross);
  valve.position.set(world.valve[0], 0, world.valve[1]);
  scene.add(valve);
  const v17 = new THREE.Mesh(
    new THREE.PlaneGeometry(0.42, 0.12),
    new THREE.MeshBasicMaterial({ map: canvasText("V-17", "#92c353", 256, 80, 72), transparent: true })
  );
  v17.position.set(world.valve[0], 0.42, world.valve[1] - 0.18);
  scene.add(v17);

  const home = new THREE.Mesh(
    new THREE.PlaneGeometry(0.95, 0.95),
    new THREE.MeshBasicMaterial({
      map: canvasText("HOME", "#29bee7", 256, 256, 64),
      transparent: true,
      opacity: 0.85,
      side: THREE.DoubleSide,
    })
  );
  home.rotation.x = -Math.PI / 2;
  home.position.set(world.home[0], 0.012, world.home[1]);
  scene.add(home);

  const stencil = new THREE.Mesh(
    new THREE.PlaneGeometry(1.15, 0.32),
    new THREE.MeshBasicMaterial({
      map: stencilTexture("LEAK", "#f54966"),
      transparent: true,
      side: THREE.DoubleSide,
      depthWrite: false,
    })
  );
  stencil.rotation.x = -Math.PI / 2;
  stencil.position.set(tx - 0.05, 0.02, ty - 0.85);
  stencil.name = "stencil";
  scene.add(stencil);

  const leakLight = new THREE.PointLight(C.alarm, 1.6, 5.0, 2);
  leakLight.position.set(tx, 0.7, ty - 0.25);
  leakLight.name = "leakLight";
  scene.add(leakLight);

  const bay1 = new THREE.PointLight(0xc8d4dc, 0.55, 8, 2);
  bay1.position.set(2.2, 2.4, 2.0);
  scene.add(bay1);
  const bay2 = new THREE.PointLight(0xc8d4dc, 0.5, 8, 2);
  bay2.position.set(6.2, 2.4, 4.2);
  scene.add(bay2);

  dressCell(scene, world);
  return { leak, spray, mist, handle, stencil, leakLight };
}

function box(scene, x, y, z, sx, sy, sz, color, metal = 0.15) {
  const m = new THREE.Mesh(
    new THREE.BoxGeometry(sx, sy, sz),
    new THREE.MeshStandardMaterial({ color, metalness: metal, roughness: 0.55 })
  );
  m.position.set(x, y, z);
  m.castShadow = true;
  m.receiveShadow = true;
  scene.add(m);
  return m;
}

function cyl(scene, x, y, z, r, h, color, rx = 0, rz = 0) {
  const m = new THREE.Mesh(
    new THREE.CylinderGeometry(r, r, h, 16),
    new THREE.MeshStandardMaterial({ color, metalness: 0.45, roughness: 0.4 })
  );
  m.position.set(x, y, z);
  m.rotation.x = rx;
  m.rotation.z = rz;
  m.castShadow = true;
  scene.add(m);
  return m;
}

function dressCell(scene, world) {
  const steel = 0x585861;
  const paper = 0x27272b;
  for (let x = 0.5; x < world.w; x += 1) {
    const line = new THREE.Mesh(
      new THREE.BoxGeometry(0.02, 0.01, world.h - 0.2),
      new THREE.MeshBasicMaterial({ color: 0x2a2a32 })
    );
    line.position.set(x, 0.01, world.h / 2);
    scene.add(line);
  }
  for (const x of [1.8, 4.0, 6.4]) {
    cyl(scene, x, 1.72, 0.18, 0.04, world.h - 0.4, steel, Math.PI / 2, 0);
    cyl(scene, x, 1.72, world.h - 0.18, 0.04, 0.01, steel);
  }
  for (let i = 0; i < 5; i += 1) {
    const lx = 1.1 + i * 1.45;
    const lamp = new THREE.Mesh(
      new THREE.BoxGeometry(0.55, 0.04, 0.18),
      new THREE.MeshStandardMaterial({ color: 0xd8dce4, emissive: 0x8aa0b0, emissiveIntensity: 0.35 })
    );
    lamp.position.set(lx, 1.62, 3.0);
    scene.add(lamp);
  }
  cyl(scene, -1.35, 0.7, 2.2, 0.38, 1.4, 0x3a6070);
  cyl(scene, -1.35, 0.7, 3.4, 0.38, 1.4, 0x3a6070);
  const t01 = new THREE.Mesh(
    new THREE.PlaneGeometry(0.7, 0.18),
    new THREE.MeshBasicMaterial({ map: canvasText("T-01  T-02  FARM", "#29bee7", 512, 80, 48), transparent: true })
  );
  t01.position.set(-1.35, 1.5, 2.8);
  t01.rotation.y = Math.PI / 2;
  scene.add(t01);
  box(scene, -2.4, 0.55, 5.0, 1.6, 1.1, 1.2, paper, 0.05);
  const dock = new THREE.Mesh(
    new THREE.PlaneGeometry(1.4, 0.2),
    new THREE.MeshBasicMaterial({ map: canvasText("DOCK  B", "#e1e1e4", 256, 64, 48), transparent: true })
  );
  dock.position.set(-2.4, 1.2, 5.0);
  dock.rotation.y = Math.PI / 2;
  scene.add(dock);
  box(scene, 0.28, 0.45, 2.35, 0.38, 0.9, 0.7, 0x35363a, 0.2);
  const mcc = new THREE.Mesh(
    new THREE.PlaneGeometry(0.62, 0.14),
    new THREE.MeshBasicMaterial({ map: canvasText("MCC-3", "#EF833A", 256, 64, 48), transparent: true })
  );
  mcc.position.set(0.5, 0.72, 2.35);
  mcc.rotation.y = Math.PI / 2;
  scene.add(mcc);
  box(scene, 0.28, 0.28, 4.55, 0.32, 0.55, 0.32, 0x8a2a32, 0.1);
  cyl(scene, 7.72, 0.35, 1.15, 0.16, 0.7, 0x4a4e58);
  cyl(scene, 7.72, 0.35, 1.5, 0.16, 0.7, 0x4a4e58);
  box(scene, 7.72, 0.4, 4.65, 0.36, 0.8, 0.5, 0x2a3040, 0.3);
  const cab = new THREE.Mesh(
    new THREE.PlaneGeometry(0.5, 0.14),
    new THREE.MeshBasicMaterial({ map: canvasText("N2", "#92c353", 128, 48, 40), transparent: true })
  );
  cab.position.set(7.52, 0.55, 4.65);
  cab.rotation.y = -Math.PI / 2;
  scene.add(cab);
  for (const z of [0.7, 1.1, 1.5]) {
    cyl(scene, 0.22, 0.18, z, 0.11, 0.36, 0x3e3a32);
  }
  cyl(scene, world.tank[0], 0.55, 0.25, 0.04, 5.4, 0x6a7370, 0, Math.PI / 2);
  cyl(scene, 5.4, 0.55, 5.75, 0.04, 4.2, 0x6a7370, 0, Math.PI / 2);
  box(scene, 3.5, 0.95, 3.0, 0.08, 0.12, 1.6, 0xeba800, 0.05);
  const aisle = new THREE.Mesh(
    new THREE.PlaneGeometry(1.8, 0.16),
    new THREE.MeshBasicMaterial({ map: canvasText("AISLE A  ·  KEEP CLEAR", "#eba800", 512, 64, 40), transparent: true })
  );
  aisle.rotation.x = -Math.PI / 2;
  aisle.position.set(2.2, 0.03, 5.35);
  scene.add(aisle);
  const haz = new THREE.Mesh(
    new THREE.PlaneGeometry(1.4, 0.16),
    new THREE.MeshBasicMaterial({ map: canvasText("HYDROCARBON", "#f54966", 512, 64, 44), transparent: true })
  );
  haz.rotation.x = -Math.PI / 2;
  haz.position.set(6.2, 0.03, 5.35);
  scene.add(haz);
}



function drawMinimap(mission, frame, pathPts) {
  const g = mission.grid;
  const w = minimap.width;
  const h = minimap.height;
  mapCtx.fillStyle = "#121217";
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
    mapCtx.fillStyle = "#35363a";
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
    mapCtx.strokeStyle = "#EF833A";
    mapCtx.lineWidth = 1.5;
    mapCtx.beginPath();
    pathPts.forEach(([x, y], i) => {
      if (i === 0) mapCtx.moveTo(toX(x), toY(y));
      else mapCtx.lineTo(toX(x), toY(y));
    });
    mapCtx.stroke();
  }
  const [tx, ty] = mission.world.tank;
  mapCtx.fillStyle = "#4f8eaa";
  mapCtx.beginPath();
  mapCtx.arc(toX(tx), toY(ty), 5, 0, Math.PI * 2);
  mapCtx.fill();
  const [vx, vy] = mission.world.valve;
  mapCtx.fillStyle = "#92c353";
  mapCtx.fillRect(toX(vx) - 3, toY(vy) - 3, 6, 6);
  mapCtx.fillStyle = "#EF833A";
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
    mapCtx.strokeStyle = "rgba(41,190,231,0.75)";
    mapCtx.lineWidth = 1;
    frame.rays.forEach((r, i) => {
      const ang = frame.yaw + SONAR_BEARINGS[i];
      mapCtx.beginPath();
      mapCtx.moveTo(px, py);
      mapCtx.lineTo(toX(frame.x + r * Math.cos(ang)), toY(frame.y + r * Math.sin(ang)));
      mapCtx.stroke();
    });
  }
  const reduceMotion = window.matchMedia("(prefers-reduced-motion: reduce)").matches;
  if (reduceMotion) return;
  const sweep = (frame.t * 2.4) % (Math.PI * 2);
  mapCtx.save();
  mapCtx.translate(px, py);
  mapCtx.rotate(-sweep);
  const sweepGrad = mapCtx.createLinearGradient(0, 0, 36, 0);
  sweepGrad.addColorStop(0, "rgba(41,190,231,0.28)");
  sweepGrad.addColorStop(1, "rgba(41,190,231,0)");
  mapCtx.fillStyle = sweepGrad;
  mapCtx.beginPath();
  mapCtx.moveTo(0, 0);
  mapCtx.arc(0, 0, 36, 0, 0.55);
  mapCtx.closePath();
  mapCtx.fill();
  mapCtx.restore();
}

function fillSonar(rays) {
  const vals = rays || [2, 2, 2, 2, 2];
  if (!sonarBars.childElementCount) {
    vals.forEach(() => sonarBars.append(document.createElement("i")));
  }
  [...sonarBars.children].forEach((el, i) => {
    const scale = Math.max(0.08, Math.min(1, vals[i] / 2));
    el.style.transform = `scaleY(${scale})`;
    el.style.opacity = vals[i] < 0.6 ? "1" : "0.78";
    el.classList.toggle("close", vals[i] < 0.5);
  });
  sonarVals.textContent = vals.map((v) => v.toFixed(2)).join("  ") + " m";
}

function drawSpark(wl, wr) {
  if (!sparkCtx || !spark) return;
  sparkBuf.push([wl, wr]);
  const w = spark.width;
  const h = spark.height;
  if (sparkBuf.length > w) sparkBuf.shift();
  sparkCtx.fillStyle = "#121217";
  sparkCtx.fillRect(0, 0, w, h);
  sparkCtx.strokeStyle = "#2d2d33";
  sparkCtx.beginPath();
  sparkCtx.moveTo(0, h / 2);
  sparkCtx.lineTo(w, h / 2);
  sparkCtx.stroke();
  const max = 4;
  const plot = (idx, color) => {
    sparkCtx.strokeStyle = color;
    sparkCtx.lineWidth = 1.4;
    sparkCtx.beginPath();
    sparkBuf.forEach((pair, i) => {
      const x = i;
      const y = h / 2 - (pair[idx] / max) * (h / 2 - 4);
      if (i === 0) sparkCtx.moveTo(x, y);
      else sparkCtx.lineTo(x, y);
    });
    sparkCtx.stroke();
  };
  plot(0, "#29bee7");
  plot(1, "#EF833A");
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
  if (reason === "co2_extinguished") {
    return {
      title: "OUT",
      kicker: "CO₂ flood",
      body: "Wrist dump starved the flame of oxygen. IR is back at ambient. VOC is falling.",
    };
  }
  if (reason === "foam_blanket") {
    return {
      title: "BLANKETED",
      kicker: "AFFF foam",
      body: "Fuel surface is covered. Vapour cannot feed the flame. Sensors below trip.",
    };
  }
  if (reason === "fuel_isolated") {
    return {
      title: "STARVED",
      kicker: "V-17 closed",
      body: "Isolation valve cut the tank feed. The pool burned out. Cell can stay locked.",
    };
  }
  if (reason === "remote_fuel") {
    return {
      title: "STILL BURNING",
      kicker: "Wrong valve",
      body: "V-17 does not feed this seat. The spill keeps burning. A human has to enter.",
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

  const fireSpec = mission.fire;
  const fireMode = Boolean(fireSpec && fireSpec.enabled);
  const grid = parseGrid(mission.grid);
  const isStory = mission.mode === "story" && mission.branches;
  const gateT = isStory && !fireMode ? Number(mission.gate_t) : Infinity;
  const briefing = fireMode && fireSpec.briefing ? fireSpec.briefing : mission.briefing || {};
  const fireActions = fireMode ? fireSpec.actions : mission.actions || [];
  const methodLines = fireMode ? fireSpec.methods : mission.methods || [];

  blurbEl.textContent = (fireMode && fireSpec.briefing ? fireSpec.briefing.problem : mission.blurb) || "";
  storyEl.textContent = fireMode
    ? "Cell locked. Click a free bay to ignite. RAVEN does not know the seat."
    : mission.frames[0].story;
  if (briefing.incident_id) {
    incSub.textContent = `${briefing.incident_id} · ${briefing.site || "Bay 3"}`;
  }
  methodLines.forEach((line) => {
    const li = document.createElement("li");
    li.textContent = line;
    howEl.append(li);
  });
  if (chipsEl && fireSpec && fireSpec.sensors) {
    fireSpec.sensors.forEach((s) => {
      const b = document.createElement("b");
      b.className = "chip";
      b.dataset.id = s.id;
      b.textContent = s.id;
      b.title = s.use;
      chipsEl.append(b);
    });
  }
  if (padsEl && fireSpec && fireSpec.sites) {
    fireSpec.sites.forEach((site) => {
      const btn = document.createElement("button");
      btn.type = "button";
      btn.className = "pad";
      btn.dataset.x = String(site.xy[0]);
      btn.dataset.y = String(site.xy[1]);
      btn.innerHTML = `<strong>${site.label}</strong><span>${site.fuel}</span>`;
      padsEl.append(btn);
    });
  }

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
    fireActions.forEach((act, i) => {
      const btn = document.createElement("button");
      btn.type = "button";
      btn.className = `order-btn kind-${act.id}`;
      btn.dataset.id = act.id;
      btn.innerHTML = `<kbd>${i + 1}</kbd><div><strong>${act.title}</strong><span>${act.detail}</span></div>`;
      list.append(btn);
    });
  }

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(C.bg);
  scene.fog = new THREE.Fog(C.bg, 16, 32);

  const camera = new THREE.PerspectiveCamera(48, 1, 0.05, 80);
  camera.position.set(-1.6, 6.6, -3.8);

  const renderer = new THREE.WebGLRenderer({ canvas: view, antialias: true, alpha: false });
  renderer.setPixelRatio(Math.min(2, window.devicePixelRatio || 1));
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.08;
  renderer.outputColorSpace = THREE.SRGBColorSpace;

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;
  controls.target.set(4.1, 0.2, 2.9);
  controls.maxPolarAngle = Math.PI / 2 - 0.04;
  controls.minDistance = 1.05;
  controls.maxDistance = 32;
  controls.zoomSpeed = 1.35;
  controls.rotateSpeed = 0.72;
  controls.panSpeed = 0.9;
  controls.screenSpacePanning = true;
  const camState = { follow: false };
  const camKeys = Object.create(null);
  const _fwd = new THREE.Vector3();
  const _right = new THREE.Vector3();
  const _up = new THREE.Vector3(0, 1, 0);
  function markFollow(on) {
    camState.follow = on;
    if (followBtn) followBtn.setAttribute("aria-pressed", on ? "true" : "false");
    const followCamBtn = document.querySelector('#cam-bar [data-cam="follow"]');
    if (followCamBtn) followCamBtn.classList.toggle("on", on);
  }
  controls.addEventListener("start", () => {
    if (camState.follow) markFollow(false);
  });
  window.addEventListener("keydown", (e) => {
    if (["KeyW", "KeyA", "KeyS", "KeyD", "KeyQ", "KeyE"].includes(e.code) && briefingEl.hidden) {
      camKeys[e.code] = true;
      e.preventDefault();
      if (camState.follow) markFollow(false);
    }
  });
  window.addEventListener("keyup", (e) => {
    camKeys[e.code] = false;
  });
  function driveCamera(dt) {
    const speed = (camKeys.ShiftLeft ? 10 : 4.4) * dt;
    camera.getWorldDirection(_fwd);
    _fwd.y = 0;
    if (_fwd.lengthSq() < 1e-6) _fwd.set(0, 0, -1);
    else _fwd.normalize();
    _right.crossVectors(_fwd, _up).normalize();
    let moved = false;
    if (camKeys.KeyW) {
      camera.position.addScaledVector(_fwd, speed);
      controls.target.addScaledVector(_fwd, speed);
      moved = true;
    }
    if (camKeys.KeyS) {
      camera.position.addScaledVector(_fwd, -speed);
      controls.target.addScaledVector(_fwd, -speed);
      moved = true;
    }
    if (camKeys.KeyD) {
      camera.position.addScaledVector(_right, speed);
      controls.target.addScaledVector(_right, speed);
      moved = true;
    }
    if (camKeys.KeyA) {
      camera.position.addScaledVector(_right, -speed);
      controls.target.addScaledVector(_right, -speed);
      moved = true;
    }
    if (camKeys.KeyE) {
      camera.position.y += speed;
      controls.target.y += speed * 0.25;
      moved = true;
    }
    if (camKeys.KeyQ) {
      camera.position.y = Math.max(0.4, camera.position.y - speed);
      moved = true;
    }
    return moved;
  }
  function setCam(kind) {
    document.querySelectorAll("#cam-bar button").forEach((b) => b.classList.toggle("on", b.dataset.cam === kind));
    if (kind === "follow") {
      markFollow(true);
      return;
    }
    markFollow(false);
    if (kind === "crane") {
      camera.position.set(-1.6, 6.6, -3.8);
      controls.target.set(4.1, 0.2, 2.9);
    } else if (kind === "top") {
      camera.position.set(4.0, 13.5, 3.05);
      controls.target.set(4.0, 0.0, 3.0);
    } else if (kind === "seat") {
      const h = window.__ravenHunt;
      const fx = h && h.fire ? h.fire.x : 6.2;
      const fy = h && h.fire ? h.fire.y : 4.2;
      camera.position.set(fx - 1.6, 2.1, fy - 2.2);
      controls.target.set(fx, 0.4, fy);
    } else if (kind === "robot") {
      const p = robot.position;
      const cx = Math.max(0.7, Math.min(7.3, p.x - 2.1));
      const cz = Math.max(0.7, Math.min(5.3, p.z - 2.1));
      camera.position.set(cx, 2.8, cz);
      controls.target.set(p.x, 0.35, p.z);
    }
    controls.update();
  }
  const camBar = document.getElementById("cam-bar");
  if (camBar) {
    camBar.addEventListener("click", (e) => {
      const btn = e.target.closest("[data-cam]");
      if (btn) setCam(btn.dataset.cam);
    });
  }

  scene.add(new THREE.HemisphereLight(0xd8dce4, 0x27272b, 0.95));
  const sun = new THREE.DirectionalLight(0xd8dce4, 1.15);
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

  const fireRig = new THREE.Group();
  fireRig.name = "fire";
  fireRig.visible = false;
  const fireLight = new THREE.PointLight(0xf54966, 0, 7, 2);
  fireLight.position.set(0, 0.55, 0);
  fireRig.add(fireLight);
  const fireGlow = new THREE.Mesh(
    new THREE.SphereGeometry(0.16, 16, 16),
    new THREE.MeshBasicMaterial({ color: 0xef833a, transparent: true, opacity: 0.85 })
  );
  fireGlow.position.y = 0.22;
  fireRig.add(fireGlow);
  const fireCore = new THREE.Mesh(
    new THREE.ConeGeometry(0.12, 0.42, 10),
    new THREE.MeshBasicMaterial({ color: 0xf54966, transparent: true, opacity: 0.7 })
  );
  fireCore.position.y = 0.28;
  fireRig.add(fireCore);
  const emberGeo = new THREE.BufferGeometry();
  const emberN = 120;
  const emberPos = new Float32Array(emberN * 3);
  emberGeo.setAttribute("position", new THREE.BufferAttribute(emberPos, 3));
  const embers = new THREE.Points(
    emberGeo,
    new THREE.PointsMaterial({ color: 0xef833a, size: 0.04, transparent: true, opacity: 0.9, depthWrite: false })
  );
  fireRig.add(embers);
  const co2Mat = new THREE.PointsMaterial({ color: 0xd8dce4, size: 0.05, transparent: true, opacity: 0.0, depthWrite: false });
  const co2 = new THREE.Points(emberGeo.clone(), co2Mat);
  fireRig.add(co2);
  scene.add(fireRig);
  const padMarkers = [];
  (fireSpec?.sites || []).forEach((site) => {
    const ring = new THREE.Mesh(
      new THREE.RingGeometry(0.16, 0.22, 24),
      new THREE.MeshBasicMaterial({ color: 0xf54966, side: THREE.DoubleSide, transparent: true, opacity: 0.7 })
    );
    ring.rotation.x = -Math.PI / 2;
    ring.position.set(site.xy[0], 0.02, site.xy[1]);
    ring.userData.site = site;
    scene.add(ring);
    padMarkers.push(ring);
  });

  function tickFireFx(t, intensity, kind) {
    fireRig.visible = intensity > 0.02 || kind === "co2" || kind === "foam";
    fireLight.intensity = 2.4 * intensity;
    fireGlow.scale.setScalar(0.7 + 0.5 * intensity + 0.08 * Math.sin(t * 14));
    fireCore.scale.setScalar(0.6 + 0.55 * intensity);
    fireCore.material.opacity = 0.35 + 0.5 * intensity;
    const arr = embers.geometry.attributes.position.array;
    for (let i = 0; i < emberN; i += 1) {
      const s = (i * 0.137) % 1;
      const u = (t * 0.9 + s) % 1;
      arr[i * 3] = (s - 0.5) * 0.35 * u;
      arr[i * 3 + 1] = u * (0.3 + 0.9 * intensity);
      arr[i * 3 + 2] = (Math.sin(s * 12) * 0.2) * u;
    }
    embers.geometry.attributes.position.needsUpdate = true;
    embers.material.opacity = 0.2 + 0.7 * intensity;
    co2Mat.opacity = kind === "co2" || kind === "foam" ? 0.55 * (1 - intensity) : 0;
  }

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

  const sonarLines = SONAR_BEARINGS.map(() => {
    const geom = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3(1, 0, 0)]);
    const line = new THREE.Line(
      geom,
      new THREE.LineBasicMaterial({ color: C.cyan, transparent: true, opacity: 0.85 })
    );
    scene.add(line);
    return line;
  });

  const pathGeom = new THREE.BufferGeometry();
  const pathLine = new THREE.Line(
    pathGeom,
    new THREE.LineBasicMaterial({ color: C.amber, transparent: true, opacity: 0.78 })
  );
  scene.add(pathLine);

  const trailMax = 400;
  const trailPos = new Float32Array(trailMax * 3);
  const trailGeom = new THREE.BufferGeometry();
  trailGeom.setAttribute("position", new THREE.BufferAttribute(trailPos, 3));
  trailGeom.setDrawRange(0, 0);
  const trail = new THREE.Line(
    trailGeom,
    new THREE.LineBasicMaterial({ color: C.cyan, transparent: true, opacity: 0.85 })
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
    playing: fireMode ? false : !isStory || skipBrief,
    follow: false,
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
  } else if (fireMode) {
    hunt.waiting = true;
    if (igniteHint) igniteHint.hidden = false;
    stage.classList.add("ignite");
  }

  const hunt = {
    waiting: true,
    pose: { x: mission.world.home[0], y: mission.world.home[1], yaw: 0 },
    wl: 0,
    wr: 0,
    fire: null,
    intensity: 0,
    phase: "IDLE",
    path: [],
    pi: 0,
    patrolI: 0,
    located: false,
    kind: null,
    history: [],
    tripped: {},
    valve: 0,
    estimate: null,
    replan: 0,
    log: [],
  };
  window.__ravenHunt = hunt;
  let lastPub = 0;
  function publishTel(s) {
    const now = performance.now();
    if (now - lastPub < 80) return;
    lastPub = now;
    const logItems = [...logEl.querySelectorAll("li")].slice(-20).map((li) => li.textContent);
    publish({
      t: play.t,
      clock: clockEl.textContent,
      phase: hunt.phase,
      story: storyEl.textContent,
      waiting: hunt.waiting,
      located: hunt.located,
      intensity: hunt.intensity,
      alarm: alarmEl.textContent,
      pose: hunt.pose,
      sense: s,
      history: hunt.history.slice(-240),
      log: hunt.log.length ? hunt.log.slice(-24) : logItems,
    });
  }

  const order = fireMode
    ? ["IDLE", "SEARCH", "LOCATE", "INSPECT", "MITIGATE", "DONE", "HUMAN_ASSIST"]
    : ["NAVIGATE", "INSPECT", "APPROACH", "PLAN", "INTERVENE", "VERIFY", "DONE", "HUMAN_ASSIST"];

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

  function logLine(text) {
    if (text === play.lastStory) return;
    play.lastStory = text;
    storyEl.textContent = text;
    const li = document.createElement("li");
    const time = document.createElement("time");
    time.textContent = fmtClock(play.t);
    li.append(time, document.createTextNode(text));
    logEl.append(li);
    logEl.scrollTop = logEl.scrollHeight;
    hunt.log.push(`${fmtClock(play.t)} ${text}`);
    if (hunt.log.length > 40) hunt.log.shift();
  }

  function resetHunt() {
    hunt.waiting = true;
    hunt.pose = { x: mission.world.home[0], y: mission.world.home[1], yaw: 0 };
    hunt.wl = 0;
    hunt.wr = 0;
    hunt.fire = null;
    hunt.intensity = 0;
    hunt.phase = "IDLE";
    hunt.path = [];
    hunt.pi = 0;
    hunt.patrolI = 0;
    hunt.located = false;
    hunt.kind = null;
    hunt.history = [];
    hunt.tripped = {};
    hunt.valve = 0;
    hunt.estimate = null;
    hunt.replan = 0;
    hunt.log = [];
    play.t = 0;
    play.chosen = null;
    play.debriefed = false;
    play.lastStory = "";
    fireRig.visible = false;
    padMarkers.forEach((m) => {
      m.visible = true;
    });
    if (padsEl) [...padsEl.querySelectorAll("button")].forEach((b) => { b.disabled = false; });
  }

  function igniteAt(x, y) {
    if (!fireMode || !hunt.waiting || !briefingEl.hidden) return false;
    const snap = snapFree(x, y, grid);
    if (!snap) {
      logLine("That cell is occupied. Pick a free bay.");
      return false;
    }
    hunt.fire = { x: snap[0], y: snap[1] };
    hunt.intensity = 1;
    hunt.waiting = false;
    hunt.phase = "SEARCH";
    hunt.path = fireSpec.patrol && fireSpec.patrol.length ? fireSpec.patrol.map((p) => p.slice()) : planPath(grid, [hunt.pose.x, hunt.pose.y], snap);
    hunt.pi = 0;
    fireRig.position.set(snap[0], 0, snap[1]);
    fireRig.visible = true;
    padMarkers.forEach((m) => {
      m.visible = false;
    });
    if (igniteHint) igniteHint.hidden = true;
    stage.classList.remove("ignite");
    if (padsEl) [...padsEl.querySelectorAll("button")].forEach((b) => { b.disabled = true; });
    play.playing = true;
    logLine(`Ignition at ${snap[0].toFixed(2)}, ${snap[1].toFixed(2)} m. RAVEN begins a coverage search.`);
    return true;
  }

  function startMitigate(id) {
    hunt.kind = id;
    hunt.phase = id === "human" ? "HUMAN_ASSIST" : "MITIGATE";
    play.chosen = id;
    play.gated = true;
    ordersEl.hidden = true;
    debriefEl.hidden = true;
    play.playing = true;
    logLine(`Operator order: ${id}`);
    if (id === "human") {
      hunt.wl = 0;
      hunt.wr = 0;
      play.playing = false;
      tape.result = { status: "HUMAN_ASSIST", reason: "operator_request", attempts: 0, valve_angle: hunt.valve };
      showDebrief();
    }
  }

  function finishFire(status, reason, bodyTitle) {
    hunt.phase = status === "HUMAN_ASSIST" ? "HUMAN_ASSIST" : "DONE";
    play.playing = false;
    tape.result = { status, reason, attempts: 1, valve_angle: hunt.valve };
    play.debriefed = false;
    showDebrief();
    if (bodyTitle) document.getElementById("debrief-title").textContent = bodyTitle;
  }

  function driveAlong(dt) {
    if (!hunt.path.length) {
      hunt.wl = 0;
      hunt.wr = 0;
      return true;
    }
    while (hunt.pi < hunt.path.length) {
      const wp = hunt.path[hunt.pi];
      const dist = Math.hypot(wp[0] - hunt.pose.x, wp[1] - hunt.pose.y);
      if (dist >= 0.2) break;
      hunt.pi += 1;
    }
    if (hunt.pi >= hunt.path.length) {
      hunt.wl = 0;
      hunt.wr = 0;
      return true;
    }
    const speeds = wheelSpeeds(hunt.pose, hunt.path[hunt.pi]);
    hunt.wl = speeds[0];
    hunt.wr = speeds[1];
    hunt.pose = stepPose(hunt.pose, hunt.wl, hunt.wr, dt);
    return false;
  }

  function senseNow() {
    if (!hunt.fire) {
      return sampleSensors(hunt.pose.x, hunt.pose.y, hunt.pose.yaw, 6.2, 4.2, 0, play.t, grid);
    }
    return sampleSensors(
      hunt.pose.x,
      hunt.pose.y,
      hunt.pose.yaw,
      hunt.fire.x,
      hunt.fire.y,
      hunt.intensity,
      play.t,
      grid
    );
  }

  function huntTick(dt) {
    const step = dt * play.speed;
    if (play.playing && !hunt.waiting) play.t += step;
    const s = senseNow();
    Object.entries(s.activated).forEach(([id, on]) => {
      if (on && !hunt.tripped[id]) {
        hunt.tripped[id] = true;
        logLine(`${id} tripped · ${id === "VOC" ? s.voc.toFixed(0) + " ppm" : id === "IR-TPA" ? s.thermal.toFixed(0) + " °C" : id === "HF" ? s.flux.toFixed(1) + " kW/m²" : "looks at the seat"}`);
      }
    });
    hunt.history.push({ thermal: s.thermal, voc: s.voc, smoke: s.smoke, flux: s.flux });
    if (hunt.history.length > 240) hunt.history.shift();

    if (hunt.kind === "co2" && hunt.phase === "MITIGATE") {
      hunt.intensity = Math.max(0, hunt.intensity - step * 0.55);
      hunt.wl = 0;
      hunt.wr = 0;
      if (hunt.intensity <= 0.02) finishFire("DONE", "co2_extinguished", "OUT");
    } else if (hunt.kind === "foam" && hunt.phase === "MITIGATE") {
      hunt.intensity = Math.max(0, hunt.intensity - step * 0.42);
      hunt.wl = 0;
      hunt.wr = 0;
      if (hunt.intensity <= 0.02) finishFire("DONE", "foam_blanket", "BLANKETED");
    } else if (hunt.kind === "isolate" && hunt.phase === "MITIGATE") {
      const tank = mission.world.tank;
      const valve = mission.world.valve;
      const nearFuel =
        Math.hypot(hunt.fire.x - tank[0], hunt.fire.y - tank[1]) < 1.35 ||
        Math.hypot(hunt.fire.x - valve[0], hunt.fire.y - valve[1]) < 1.1;
      hunt.valve = Math.min(90, hunt.valve + step * 40);
      if (nearFuel) {
        hunt.intensity = Math.max(0, hunt.intensity - step * 0.28);
        if (hunt.intensity <= 0.02 && hunt.valve >= 80) finishFire("DONE", "fuel_isolated", "STARVED");
      } else if (hunt.valve >= 90) {
        hunt.intensity = 1;
        finishFire("HUMAN_ASSIST", "remote_fuel", "STILL BURNING");
      }
    } else if (hunt.waiting) {
      hunt.wl = 0;
      hunt.wr = 0;
    } else if (hunt.phase === "SEARCH") {
      if (s.thermal > 42 || s.voc > 35 || s.alarm) {
        hunt.phase = "LOCATE";
        hunt.estimate = estimateSeat(hunt.pose, s);
        const hold = standoffPose(hunt.fire.x, hunt.fire.y, hunt.pose);
        hunt.path = planPath(grid, [hunt.pose.x, hunt.pose.y], [hold.x, hold.y]);
        hunt.pi = 0;
        logLine("Sensor fusion has a seat estimate. Driving a standoff.");
      } else {
        driveAlong(step);
        if (hunt.pi >= hunt.path.length && fireSpec.patrol) {
          hunt.path = fireSpec.patrol.map((p) => p.slice());
          hunt.pi = 0;
        }
      }
    } else if (hunt.phase === "LOCATE") {
      hunt.replan += step;
      if (hunt.replan > 2.5) {
        hunt.replan = 0;
        const hold = standoffPose(hunt.fire.x, hunt.fire.y, hunt.pose);
        hunt.path = planPath(grid, [hunt.pose.x, hunt.pose.y], [hold.x, hold.y]);
        hunt.pi = 0;
      }
      const arrived = driveAlong(step);
      const face = wrap(s.bearing - hunt.pose.yaw);
      if ((arrived || s.d < 1.15) && Math.abs(face) < 0.35) {
        hunt.wl = 0;
        hunt.wr = 0;
        hunt.phase = "INSPECT";
        logLine("On scene. Wrist RGB and UV looking at the flame front.");
      } else if (Math.abs(face) > 0.45 && s.d < 1.4) {
        const w = Math.max(-1.2, Math.min(1.2, 2.2 * face));
        const half = 0.15;
        hunt.wl = (-w * half) / 0.05;
        hunt.wr = (w * half) / 0.05;
        hunt.pose = stepPose(hunt.pose, hunt.wl, hunt.wr, step);
      }
    } else if (hunt.phase === "INSPECT") {
      hunt.wl = 0;
      hunt.wr = 0;
      const lock = s.located || (s.d < 1.3 && s.thermal > 70 && s.voc > 50);
      if (lock && !hunt.located) {
        hunt.located = true;
        play.playing = false;
        ordersEl.hidden = false;
        logLine("Fire located. IR + UV and vapour agree. Awaiting mitigation order.");
      }
    }

    applyHunt(s, dt);
  }

  function applyHunt(s, dt) {
    robot.position.set(hunt.pose.x, 0.11, hunt.pose.y);
    robot.rotation.y = yawToThree(hunt.pose.yaw);
    const wheels = robot.userData.wheels || [];
    const spin = ((hunt.wl || 0) + (hunt.wr || 0)) * 0.5 * dt;
    wheels.forEach((w) => {
      w.rotation.x += spin;
    });
    const beacon = robot.getObjectByName("beacon");
    if (beacon) beacon.material.emissiveIntensity = hunt.intensity > 0.2 ? 0.6 + 0.6 * Math.sin(play.t * 8) : 0.15;
    if (hunt.fire) tickFireFx(play.t, hunt.intensity, hunt.kind);
    const rays = rangeRays(hunt.pose, grid);
    rays.forEach((r, i) => {
      const ang = hunt.pose.yaw + SONAR_BEARINGS[i];
      const ex = hunt.pose.x + r * Math.cos(ang);
      const ey = hunt.pose.y + r * Math.sin(ang);
      sonarLines[i].geometry.setFromPoints([
        plantToThree(hunt.pose.x, hunt.pose.y, 0.12),
        plantToThree(ex, ey, 0.12),
      ]);
      sonarLines[i].material.color.setHex(r < 0.5 ? C.alarm : C.cyan);
    });
    fillSonar(rays);
    drawSpark(hunt.wl, hunt.wr);
    wheelsEl.textContent = `L ${hunt.wl.toFixed(1)}  R ${hunt.wr.toFixed(1)} rad/s`;
    if (irCtx) drawIR(irCtx, s, hunt.intensity);
    if (camCtx) drawFlameCam(camCtx, s, hunt.intensity);
    if (pileEl) drawPile(pileEl, s.pile);
    if (graphCtx) drawGraph(graphCtx, hunt.history);
    if (chipsEl) {
      [...chipsEl.querySelectorAll(".chip")].forEach((el) => {
        const on = Boolean(s.activated[el.dataset.id]);
        el.classList.toggle("on", on);
        el.classList.toggle("ok", on && hunt.intensity < 0.05);
      });
    }
    alarmEl.textContent = hunt.intensity > 0.2 ? "FIRE" : hunt.fire ? "CLEAR" : "CELL";
    alarmEl.classList.toggle("ok", hunt.intensity <= 0.2);
    alarmEl.classList.toggle("alarm", hunt.intensity > 0.2);
    clockEl.textContent = fmtClock(play.t);
    posEl.textContent = `${hunt.pose.x.toFixed(2)}, ${hunt.pose.y.toFixed(2)} m`;
    yawEl.textContent = `${((hunt.pose.yaw * 180) / Math.PI).toFixed(0)}°`;
    hsvEl.textContent = `${s.thermal.toFixed(0)} °C`;
    ikEl.textContent = `${s.voc.toFixed(0)} ppm`;
    gripEl.textContent = s.uv ? "FLAME" : "dark";
    if (smokeRead) smokeRead.textContent = `${s.smoke.toFixed(2)} OD`;
    if (fluxRead) fluxRead.textContent = `${s.flux.toFixed(1)} kW/m²`;
    if (fireD) fireD.textContent = hunt.fire ? `${s.d.toFixed(2)} m` : "—";
    valveNum.textContent = `${hunt.valve.toFixed(0)}°`;
    needle.style.transform = `rotate(${-90 + hunt.valve}deg)`;
    const rec = document.getElementById("cam-rec");
    if (rec) rec.style.opacity = hunt.phase === "INSPECT" || hunt.phase === "MITIGATE" ? "1" : "0.25";
    markPhases(hunt.phase);
    hudPhase.textContent = hunt.phase;
    hudOrder.textContent = hunt.waiting ? "awaiting ignition" : play.chosen || (hunt.located ? "awaiting order" : "hunting");
    plant.handle.rotation.y = THREE.MathUtils.degToRad(-hunt.valve);
    const leaking = hunt.intensity > 0.35 && hunt.fire && Math.hypot(hunt.fire.x - mission.world.tank[0], hunt.fire.y - mission.world.tank[1]) < 1.2;
    plant.leak.visible = leaking;
    plant.spray.visible = leaking;
    plant.leakLight.intensity = leaking ? 1.2 : 0;
    const pathPts = hunt.path;
    if (pathPts.length > 1) pathGeom.setFromPoints(pathPts.map(([x, y]) => plantToThree(x, y, 0.03)));
    drawMinimap(
      mission,
      { x: hunt.pose.x, y: hunt.pose.y, yaw: hunt.pose.yaw, rays, t: play.t },
      pathPts
    );
    if (hunt.fire) {
      mapCtx.fillStyle = "#f54966";
      mapCtx.beginPath();
      const sx = minimap.width / mission.world.w;
      const sy = minimap.height / mission.world.h;
      mapCtx.arc(hunt.fire.x * sx, minimap.height - hunt.fire.y * sy, 4 + 2 * hunt.intensity, 0, Math.PI * 2);
      mapCtx.fill();
    }
    if (camState.follow && play.hold <= 0 && briefingEl.hidden) {
      const p = robot.position;
      const want = new THREE.Vector3(
        Math.max(0.7, Math.min(7.3, p.x - 2.1)),
        2.8,
        Math.max(0.7, Math.min(5.3, p.z - 2.1))
      );
      camera.position.lerp(want, 0.08);
      controls.target.lerp(p.clone().add(new THREE.Vector3(0.15, 0.25, 0.1)), 0.12);
    }
    publishTel(s);
  }

  function apply(frame, dt = 0.016) {
    robot.position.set(frame.x, 0.11, frame.y);
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
    plant.spray.scale.setScalar(leaking ? 1 + 0.1 * Math.sin(frame.t * 9) : 1);
    if (plant.mist) tickMist(plant.mist, leaking, frame.t);
    plant.leakLight.intensity = leaking ? 1.55 : 0.0;
    if (plant.stencil.userData.leak !== leaking) {
      const map = plant.stencil.material.map;
      map?.dispose();
      plant.stencil.material.map = stencilTexture(leaking ? "LEAK" : "SEALED", leaking ? "#f54966" : "#92c353");
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
      sonarLines[i].material.color.setHex(r < 0.5 ? C.alarm : C.cyan);
      sonarLines[i].material.opacity = r < 0.6 ? 1 : 0.72;
    });
    fillSonar(rays);
    drawSpark(frame.wl || 0, frame.wr || 0);

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
    const rec = document.getElementById("cam-rec");
    if (rec) rec.style.opacity = camLive ? "1" : "0.25";
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
    if (camState.follow && play.hold <= 0 && briefingEl.hidden) {
      const want = new THREE.Vector3(robot.position.x - 1.8, 2.2, robot.position.z - 2.4);
      camera.position.lerp(want, 0.08);
      controls.target.lerp(robot.position.clone().add(new THREE.Vector3(0.2, 0.25, 0.15)), 0.12);
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

  if (fireMode) {
    applyHunt(senseNow(), 0);
    scrub.max = "120000";
  } else {
    apply(sample(tape.frames, play.t));
  }
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
    if (fireMode) {
      startMitigate(id);
      return;
    }
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
    if (fireMode) {
      if (play.hold > 0) play.hold -= dt;
      huntTick(dt);
      playBtn.textContent = play.playing ? "Pause" : "Play";
      scrub.value = String(Math.round(play.t * 1000));
    } else {
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
    }
    driveCamera(dt);
    controls.update();
    renderer.render(scene, camera);
    requestAnimationFrame(tick);
  }
  requestAnimationFrame(tick);

  document.getElementById("deploy").addEventListener("click", () => {
    briefingEl.hidden = true;
    if (fireMode) {
      play.playing = false;
      hunt.waiting = true;
      if (igniteHint) igniteHint.hidden = false;
      stage.classList.add("ignite");
      logLine("RAVEN is on station at home. Ignite a free cell.");
      playBtn.textContent = "Play";
    } else {
      play.playing = true;
      play.hold = 1.4;
      playBtn.textContent = "Pause";
    }
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
    logEl.replaceChildren();
    sparkBuf.length = 0;
    ordersEl.hidden = true;
    if (fireMode) {
      resetHunt();
      if (igniteHint) igniteHint.hidden = false;
      stage.classList.add("ignite");
      play.playing = false;
      logLine("Cell reset. Ignite a free cell.");
    } else {
      tape.frames = mission.frames;
      tape.paths = mission.paths;
      tape.path = mission.path;
      tape.result = mission.result;
      tape.inspect = mission.inspect;
      setScrubMax();
      play.t = 0;
      play.playing = true;
      play.lastStory = "";
    }
  });

  playBtn.addEventListener("click", () => {
    if (!briefingEl.hidden) return;
    if (fireMode) {
      if (hunt.waiting) return;
      if (hunt.located && !play.chosen) {
        ordersEl.hidden = false;
        return;
      }
      play.playing = !play.playing;
      playBtn.textContent = play.playing ? "Pause" : "Play";
      return;
    }
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
    play.chosen = null;
    play.debriefed = false;
    play.lastStory = "";
    logEl.replaceChildren();
    sparkBuf.length = 0;
    if (fireMode) {
      resetHunt();
      if (igniteHint) igniteHint.hidden = false;
      stage.classList.add("ignite");
      play.playing = false;
      playBtn.textContent = "Play";
      logLine("Cell reset. Ignite a free cell.");
    } else {
      play.t = 0;
      play.hold = 0.3;
      play.playing = true;
      tape.frames = mission.frames;
      tape.paths = mission.paths;
      tape.path = mission.path;
      tape.result = mission.result;
      setScrubMax();
      playBtn.textContent = "Pause";
    }
  });
  followBtn.addEventListener("click", () => {
    markFollow(!camState.follow);
  });
  const pickRay = new THREE.Raycaster();
  const pickPlane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
  const pickHit = new THREE.Vector3();
  view.addEventListener("click", (ev) => {
    if (!fireMode || !hunt.waiting || !briefingEl.hidden) return;
    const rect = view.getBoundingClientRect();
    const mouse = new THREE.Vector2(
      ((ev.clientX - rect.left) / rect.width) * 2 - 1,
      -((ev.clientY - rect.top) / rect.height) * 2 + 1
    );
    pickRay.setFromCamera(mouse, camera);
    if (!pickRay.ray.intersectPlane(pickPlane, pickHit)) return;
    igniteAt(pickHit.x, pickHit.z);
  });
  minimap.addEventListener("click", (ev) => {
    if (!fireMode || !hunt.waiting || !briefingEl.hidden) return;
    const rect = minimap.getBoundingClientRect();
    const x = ((ev.clientX - rect.left) / rect.width) * mission.world.w;
    const y = (1 - (ev.clientY - rect.top) / rect.height) * mission.world.h;
    igniteAt(x, y);
  });
  if (padsEl) {
    padsEl.addEventListener("click", (ev) => {
      const btn = ev.target.closest("[data-x]");
      if (!btn) return;
      igniteAt(Number(btn.dataset.x), Number(btn.dataset.y));
    });
  }
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
    } else if (!ordersEl.hidden && ["Digit1", "Digit2", "Digit3", "Digit4"].includes(e.code)) {
      const btn = document.querySelectorAll(".order-btn")[Number(e.code.slice(-1)) - 1];
      if (btn) chooseOrder(btn.dataset.id);
    }
  });
} catch (err) {
  storyEl.textContent =
    "Could not load the control room. Need a network connection once for the 3D library, and python -m raven must keep running.";
  console.error(err);
}
