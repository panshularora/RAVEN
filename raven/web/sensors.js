import { subscribe } from "./bus.js";
import { drawIR, drawFlameCam, drawPile } from "./fire.js";

const ir = document.getElementById("ir").getContext("2d");
const cam = document.getElementById("cam").getContext("2d");
const pile = document.getElementById("pile");
const alarm = document.getElementById("alarm");
const clock = document.getElementById("clock");
const cards = document.getElementById("cards");
const note = document.getElementById("live-note");

const COPY = {
  "IR-TPA": {
    now: (s) => `${s.thermal.toFixed(0)} °C`,
    why: "8-pixel thermopile ±40°. Ambient 28 °C. Inverse-range heat with flicker. Trips above 55 °C. Looking away cuts the cosine term; an occupied cell on the ray leaves only 12% leak-through.",
  },
  SMOKE: {
    now: (s) => `${s.smoke.toFixed(2)} OD`,
    why: "Optical density on the look ray, 0.9 µm scatter. Trips above 0.12. Slow sine models plume wobble. Not required for locate; it tells you the aisle is filling.",
  },
  VOC: {
    now: (s) => `${s.voc.toFixed(0)} ppm`,
    why: "Photo-ionisation detector. Inverse-square from the seat, 420 / (d² + 0.35). Trips above 80 ppm. Works around corners because vapour does not need line of sight.",
  },
  "UV-FLAME": {
    now: (s) => (s.uv ? "FLAME" : "dark"),
    why: "185–260 nm. Binary. Needs intensity, range under 2.3 m, facing cosine > 0.15, and a clear look. This is the ‘I see fire’ bit, not heat.",
  },
  HF: {
    now: (s) => `${s.flux.toFixed(1)} kW/m²`,
    why: "Mast radiometer. 48 / (d + 0.35)² times line of sight. Trips above 2.0 kW/m². Used for standoff, not for the locate vote.",
  },
  RGB: {
    now: (s) => (s.rgb ? "blob" : "no blob"),
    why: "Wrist 64×64 HSV. Orange/red disk only if the camera is inside 1.55 m and facing the seat. Same idea as the leak HSV in the 10-point brief.",
  },
};

let specs = [];

fetch("./mission.json")
  .then((r) => r.json())
  .then((m) => {
    specs = (m.fire && m.fire.sensors) || [];
    specs.forEach((s) => {
      const art = document.createElement("article");
      art.className = "sensor-card";
      art.dataset.id = s.id;
      art.innerHTML = `
        <header>
          <b class="chip" data-id="${s.id}">${s.id}</b>
          <h2>${s.name}</h2>
          <p class="read" id="read-${s.id}">—</p>
        </header>
        <p>${s.use}</p>
        <p class="output">${s.output}</p>
        <p class="why">${(COPY[s.id] && COPY[s.id].why) || ""}</p>
      `;
      cards.append(art);
    });
  })
  .catch(() => {
    note.textContent = "Could not load mission.json. Keep python -m raven running.";
  });

subscribe((tel) => {
  if (!tel || !tel.sense) return;
  note.textContent = tel.waiting
    ? "Plant is live. Ignite a cell on the Plant page."
    : `Live · ${tel.phase} · ${tel.story || ""}`;
  const s = tel.sense;
  drawIR(ir, s, tel.intensity ?? 1);
  drawFlameCam(cam, s, tel.intensity ?? 1);
  drawPile(pile, s.pile || [28, 28, 28, 28, 28, 28, 28, 28]);
  alarm.textContent = tel.alarm || "CELL";
  alarm.classList.toggle("ok", tel.alarm === "CLEAR" || tel.alarm === "CELL");
  alarm.classList.toggle("alarm", tel.alarm === "FIRE");
  clock.textContent = tel.clock || "";
  document.getElementById("fire-d").textContent = s.d != null ? `${Number(s.d).toFixed(2)} m` : "—";
  document.getElementById("los").textContent = s.los != null ? `${Number(s.los).toFixed(2)}` : "—";
  document.getElementById("facing").textContent = s.facing != null ? Number(s.facing).toFixed(2) : "—";
  document.getElementById("phase").textContent = tel.phase || "—";
  specs.forEach((spec) => {
    const el = document.getElementById(`read-${spec.id}`);
    const card = document.querySelector(`.sensor-card[data-id="${spec.id}"]`);
    const on = Boolean(s.activated && s.activated[spec.id]);
    if (el && COPY[spec.id]) el.textContent = COPY[spec.id].now(s);
    if (card) card.classList.toggle("hot", on);
    const chip = card && card.querySelector(".chip");
    if (chip) chip.classList.toggle("on", on);
  });
});
