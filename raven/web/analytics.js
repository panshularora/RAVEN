import { subscribe } from "./bus.js";
import { drawGraph } from "./fire.js";

const graph = document.getElementById("graph").getContext("2d");
const body = document.getElementById("fusion-body");
const logEl = document.getElementById("log");
const alarm = document.getElementById("alarm");
const clock = document.getElementById("clock");
const note = document.getElementById("live-note");

const ROWS = [
  ["IR-TPA", "thermal", "55 °C", (s) => `${s.thermal.toFixed(0)} °C`],
  ["SMOKE", "smoke", "0.12 OD", (s) => `${s.smoke.toFixed(2)} OD`],
  ["VOC", "voc", "80 ppm", (s) => `${s.voc.toFixed(0)} ppm`],
  ["UV-FLAME", "uv", "1 inside 2.3 m", (s) => (s.uv ? "1" : "0")],
  ["HF", "flux", "2.0 kW/m²", (s) => `${s.flux.toFixed(1)}`],
  ["RGB", "rgb", "blob < 1.55 m", (s) => (s.rgb ? "seen" : "none")],
];

ROWS.forEach(([id, , trip]) => {
  const tr = document.createElement("tr");
  tr.dataset.id = id;
  tr.innerHTML = `<td>${id}</td><td class="v">—</td><td>${trip}</td><td class="st">quiet</td>`;
  body.append(tr);
});

subscribe((tel) => {
  if (!tel || !tel.sense) return;
  note.textContent = tel.waiting
    ? "Plant is live. Ignite on the Plant page to fill the traces."
    : `Live · ${tel.phase} · located ${tel.located ? "yes" : "no"}`;
  drawGraph(graph, tel.history || []);
  alarm.textContent = tel.alarm || "CELL";
  alarm.classList.toggle("ok", tel.alarm === "CLEAR" || tel.alarm === "CELL");
  alarm.classList.toggle("alarm", tel.alarm === "FIRE");
  clock.textContent = tel.clock || "";
  const s = tel.sense;
  ROWS.forEach(([id, , , fmt]) => {
    const tr = body.querySelector(`tr[data-id="${id}"]`);
    if (!tr) return;
    tr.querySelector(".v").textContent = fmt(s);
    const on = Boolean(s.activated && s.activated[id]);
    tr.querySelector(".st").textContent = on ? "TRIP" : "quiet";
    tr.classList.toggle("hot", on);
  });
  if (Array.isArray(tel.log)) {
    logEl.replaceChildren();
    tel.log.slice(-24).forEach((line) => {
      const li = document.createElement("li");
      li.textContent = line;
      logEl.append(li);
    });
  }
});
