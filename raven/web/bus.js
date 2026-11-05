/** Live telemetry from the plant tab to Sensors and Analytics. */

export const KEY = "raven.telemetry";
export const CHANNEL = "raven-telemetry";

let channel = null;

function ch() {
  if (channel) return channel;
  try {
    channel = new BroadcastChannel(CHANNEL);
  } catch {
    channel = null;
  }
  return channel;
}

export function publish(payload) {
  const json = JSON.stringify(payload);
  try {
    localStorage.setItem(KEY, json);
  } catch {
    /* quota */
  }
  const bus = ch();
  if (bus) bus.postMessage(payload);
}

export function subscribe(fn) {
  const bus = ch();
  const onMsg = (e) => fn(e.data);
  if (bus) bus.addEventListener("message", onMsg);
  const onStore = (e) => {
    if (e.key === KEY && e.newValue) {
      try {
        fn(JSON.parse(e.newValue));
      } catch {
        /* ignore */
      }
    }
  };
  window.addEventListener("storage", onStore);
  try {
    const raw = localStorage.getItem(KEY);
    if (raw) fn(JSON.parse(raw));
  } catch {
    /* ignore */
  }
  return () => {
    if (bus) bus.removeEventListener("message", onMsg);
    window.removeEventListener("storage", onStore);
  };
}
