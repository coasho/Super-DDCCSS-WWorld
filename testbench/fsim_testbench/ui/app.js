// The testbench window: the catalog on the left, the chosen scenario's options, and the session - every run in it, its
// aircraft and their activities - polled from the server every 300 ms.
"use strict";

const $ = (id) => document.getElementById(id);
let catalog = { suites: [], scenarios: [] };
let selected = null;
let lastStatus = { runs: [] };
let lastLogSeq = 0;
let results = {};
const seenEnded = new Set(); // runs whose end was recorded in the results
try { results = JSON.parse(localStorage.getItem("testbench.results") || "{}"); } catch (e) { results = {}; }

function saveResults() { try { localStorage.setItem("testbench.results", JSON.stringify(results)); } catch (e) { /* private window */ } }
function savedOptions(id) { try { return JSON.parse(localStorage.getItem("testbench.options." + id) || "{}"); } catch (e) { return {}; } }
function saveOptions(id, o) { try { localStorage.setItem("testbench.options." + id, JSON.stringify(o)); } catch (e) { /* ignore */ } }

async function api(path, body) {
  const r = await fetch(path, body === undefined ? {} : { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) });
  return r.json();
}

function el(tag, attrs, ...children) {
  const e = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs || {})) {
    if (k === "class") e.className = v;
    else if (k.startsWith("on")) e.addEventListener(k.slice(2), v);
    else if (v !== undefined && v !== null && v !== false) e.setAttribute(k, v === true ? "" : v);
  }
  for (const c of children.flat()) if (c !== null && c !== undefined) e.append(c.nodeType ? c : document.createTextNode(String(c)));
  return e;
}

// --- the catalog ---------------------------------------------------------------------------------------------
function renderTree() {
  const filter = $("search").value.trim().toLowerCase();
  const tree = $("tree");
  tree.textContent = "";
  const runningIds = new Set((lastStatus.runs || []).filter((r) => r.status === "running").map((r) => r.scenario));
  for (const suite of catalog.suites) {
    const inSuite = catalog.scenarios.filter((s) => s.suite === suite.name &&
      (!filter || (s.id + " " + s.title + " " + s.group + " " + s.summary).toLowerCase().includes(filter)));
    if (!inSuite.length) continue;
    tree.append(el("div", { class: "suite" }, suite.name, el("small", {}, suite.description)));
    const groups = [...new Set(inSuite.map((s) => s.group))];
    for (const g of groups) {
      const items = inSuite.filter((s) => s.group === g);
      const passed = items.filter((s) => results[s.id] === "pass").length;
      const d = el("details", { class: "group", open: filter || items.some((s) => selected && s.id === selected.id) ? true : null },
        el("summary", {}, el("span", {}, g), el("span", { class: "count" }, passed + "/" + items.length)));
      for (const s of items) {
        const dot = runningIds.has(s.id) ? "run" : results[s.id] || "";
        d.append(el("div", { class: "item" + (selected && selected.id === s.id ? " selected" : ""), title: s.id, onclick: () => select(s) },
          el("span", { class: "dot " + dot }), el("span", {}, s.title)));
      }
      tree.append(d);
    }
  }
}

function select(s) {
  selected = s;
  try { localStorage.setItem("testbench.selected", s.id); } catch (e) { /* ignore */ }
  const saved = savedOptions(s.id);
  const form = el("form", { class: "options", onsubmit: (e) => { e.preventDefault(); run(false); } });
  for (const o of s.options) {
    const value = saved[o.name] !== undefined ? saved[o.name] : o.default;
    let input;
    if (o.kind === "choice") {
      input = el("select", { name: o.name }, o.choices.map((c) => el("option", { value: c, selected: String(c) === String(value) }, c)));
    } else if (o.kind === "bool") {
      input = el("input", { type: "checkbox", name: o.name, checked: !!value });
      form.append(el("label", { class: "check", title: o.help }, input, o.label));
      continue;
    } else if (o.kind === "number") {
      input = el("input", { type: "number", name: o.name, value, min: o.min, max: o.max, step: o.step || "any" });
    } else {
      input = el("input", { type: "text", name: o.name, value });
    }
    form.append(el("label", { title: o.help }, o.label + (o.unit ? " (" + o.unit + ")" : ""), input, o.help ? el("span", { class: "help" }, o.help) : null));
  }
  const card = $("scenario");
  card.className = "card";
  card.textContent = "";
  card.append(el("h1", {}, s.title), el("div", { class: "refs" }, s.group + " · " + s.id + (s.refs ? " · " + s.refs : "")),
    el("p", { class: "summary" }, s.summary), form,
    el("div", { class: "buttons" },
      el("button", { class: "primary", onclick: () => run(false), title: "End what is flying and start a new session with this scenario" }, "Run (new session)"),
      el("button", { class: "secondary", id: "addbtn", onclick: () => run(true), title: "Run it beside what is already flying: one more set of aircraft in the same world" }, "Add to session"),
      el("button", { onclick: () => { saveOptions(s.id, {}); select(s); } }, "Defaults")));
  updateAddButton();
  renderTree();
}

function updateAddButton() {
  const b = $("addbtn");
  if (b) b.disabled = !(lastStatus.runs || []).length;
}

function readOptions() {
  const o = {};
  const form = document.querySelector("form.options");
  if (!form || !selected) return o;
  for (const opt of selected.options) {
    const input = form.elements[opt.name];
    if (!input) continue;
    o[opt.name] = opt.kind === "bool" ? input.checked : opt.kind === "number" ? Number(input.value) : input.value;
  }
  return o;
}

async function run(add) {
  if (!selected) return;
  const options = readOptions();
  saveOptions(selected.id, options);
  if (!add) {
    lastLogSeq = 0;
    $("log").textContent = "";
  }
  await api("/api/run", { id: selected.id, options, add });
}

// --- the session ---------------------------------------------------------------------------------------------
const fmt = (x, d = 0) => (x === null || x === undefined ? "–" : typeof x === "number" ? x.toFixed(d) : x);
const RUN_STATE = { running: "running", finished: "passed", failed: "failed", rejected: "rejected", error: "error" };

function renderRun(r) {
  const status = el("span", { class: "pill " + r.status }, RUN_STATE[r.status] || r.status);
  const head = el("div", { class: "run-head" },
    el("span", { class: "run-no" }, "#" + r.run), el("span", { class: "run-title" }, r.title), status,
    el("span", { class: "muted" }, r.runway + (r.lane ? " · lane " + r.lane : "")),
    el("button", { class: "small", title: "Stop this run and remove its aircraft", onclick: () => api("/api/stop_run", { run: r.run }) }, "Stop run"));
  const box = el("div", { class: "run" }, head);
  if (r.wait && r.status === "running") {
    const pct = Math.min(100, (100 * r.wait.elapsed) / Math.max(1, r.wait.timeout));
    box.append(el("div", { class: "wait" }, el("div", { class: "bar" }, el("div", { style: "width:" + pct + "%" })),
      el("span", {}, "waiting " + (r.wait.label || "") + " – " + r.wait.elapsed.toFixed(0) + " of " + r.wait.timeout.toFixed(0) + " s")));
  }
  if (r.note) box.append(el("div", { class: "run-note" }, r.note));
  const cols = el("div", { class: "run-cols" });
  const checks = el("ul", { class: "checks" });
  if (!r.checks.length) checks.append(el("li", { class: "muted" }, "No checks decided yet"));
  for (const c of r.checks) checks.append(el("li", { class: c.state }, c.label, c.detail ? el("span", { class: "detail" }, c.detail) : null));
  const metrics = el("dl", { class: "metrics" });
  for (const x of r.metrics) metrics.append(el("dt", {}, x.label), el("dd", {}, typeof x.value === "number" ? x.value.toFixed(2) : x.value ?? "–"));
  cols.append(el("div", {}, checks), el("div", {}, metrics));
  box.append(cols);
  if (r.actions.length) {
    const actions = el("div", { class: "actions" });
    r.actions.forEach((a, i) => actions.append(el("button", { onclick: () => api("/api/action", { run: r.run, index: i }) }, a)));
    box.append(actions);
  }
  return box;
}

function renderStatus(s) {
  const prev = lastStatus;
  lastStatus = s;
  const runs = s.runs || [];
  const status = s.paused && s.status === "running" ? "paused" : s.status;
  $("status").className = "pill " + status;
  $("status").textContent = status === "finished" ? "passed" : status;
  $("clock").textContent = s.time === null || s.time === undefined ? "-" :
    runs.length + (runs.length === 1 ? " run" : " runs") + " · " + (s.vehicles || []).length + " aircraft · t = " + s.time.toFixed(1) + " s";
  $("pause").textContent = s.paused ? "Resume" : "Pause";
  $("pause").disabled = $("stop").disabled = !runs.length;
  $("viewer").disabled = !catalog.viewer || s.viewer_running;
  $("viewer").title = !catalog.viewer ? "flightsim-viewer.exe not found (fsim build)" : s.viewer_running ? "The viewer is running" : "Start flightsim-viewer";
  const speed = $("speed");
  if (document.activeElement !== speed) speed.value = String(s.speed);
  if (document.activeElement !== $("tod") && s.clock) $("tod").value = s.clock;
  if (document.activeElement !== $("sky") && s.sky) $("sky").value = s.sky;
  updateAddButton();

  // the viewer's aircraft
  const follow = $("follow");
  if (document.activeElement !== follow) {
    const names = (s.vehicles || []).map((v) => v.name);
    const want = ["", ...names].join("\n");
    if (follow.dataset.names !== want) {
      follow.dataset.names = want;
      follow.textContent = "";
      follow.append(el("option", { value: "" }, names.length ? "-" : "no aircraft"));
      for (const v of s.vehicles || []) follow.append(el("option", { value: v.name }, v.name + " (#" + v.run + ", " + v.type + ")"));
    }
    follow.value = s.focused || "";
  }

  // the runs
  const box = $("runs");
  box.textContent = "";
  if (!runs.length) box.append(el("p", { class: "muted" }, "No run yet"));
  for (const r of runs) box.append(renderRun(r));
  const verdict = $("verdict");
  const ended = runs.filter((r) => r.status !== "running");
  const failed = ended.filter((r) => r.status !== "finished");
  verdict.className = "verdict " + (failed.length ? "fail" : ended.length ? "pass" : "");
  verdict.textContent = ended.length ? ended.length - failed.length + " of " + ended.length + " ended runs passed" : "";
  for (const r of ended) {
    const key = (s.time_origin || "") + r.run + ":" + r.scenario;
    if (!seenEnded.has(key) && (prev.runs || []).some((p) => p.run === r.run && p.status === "running")) {
      seenEnded.add(key);
      results[r.scenario] = r.status === "finished" ? "pass" : "fail";
      saveResults();
    }
  }

  // activities
  const ab = $("activities").tBodies[0];
  ab.textContent = "";
  for (const a of s.activities || []) {
    ab.append(el("tr", {}, el("td", { title: a.id + " " + (a.source || "") }, a.label), el("td", {}, "#" + a.run), el("td", { class: "state-" + a.state }, a.state),
      el("td", {}, a.reason && a.reason !== "none" ? a.reason : ""), el("td", {}, a.segment || ""), el("td", {}, fmt(a.percent)),
      el("td", {}, a.to_go_m === null || a.to_go_m === undefined ? "" : fmt(a.to_go_m / 1000, 2) + " km" + (a.to_go_s ? " / " + fmt(a.to_go_s) + " s" : "")),
      el("td", {}, a.xtrack === null || a.xtrack === undefined ? "" : fmt(a.xtrack, 1) + " m")));
  }
  if (!(s.activities || []).length) ab.append(el("tr", {}, el("td", { colspan: 8, class: "muted" }, "None")));

  // vehicles: View turns the viewer to one
  const vb = $("vehicles").tBodies[0];
  vb.textContent = "";
  for (const v of s.vehicles || []) {
    const flags = v.crashed ? "CRASHED" : v.diverged ? "DIVERGED" : v.ground ? "on ground" : "flying";
    const followed = s.focused === v.name;
    vb.append(el("tr", { class: followed ? "followed" : "" },
      el("td", {}, el("button", { class: "small view" + (followed ? " on" : ""), title: "The viewer follows this aircraft",
        onclick: () => api("/api/focus", { name: v.name }) }, followed ? "Viewing" : "View"), " " + v.name + " (" + v.type + ")"),
      el("td", {}, "#" + v.run), el("td", {}, v.level || ""),
      el("td", {}, fmt(v.alt)), el("td", {}, fmt(v.agl, 1)), el("td", {}, fmt(v.cas, 1)), el("td", {}, fmt(v.gs, 1)), el("td", {}, fmt(v.vs, 1)),
      el("td", {}, fmt(v.hdg)), el("td", {}, fmt(v.pitch, 1)), el("td", {}, fmt(v.roll, 1)), el("td", {}, fmt(v.g, 2)),
      el("td", {}, v.gear === null ? "" : v.gear > 0.99 ? "down" : v.gear < 0.01 ? "up" : fmt(v.gear, 2)), el("td", {}, fmt(v.flaps)),
      el("td", {}, fmt(v.throttle, 2)), el("td", {}, fmt(v.fuel)), el("td", {}, v.runway || ""), el("td", {}, fmt(v.along)), el("td", {}, fmt(v.across, 1)),
      el("td", { class: v.crashed || v.diverged ? "flag-bad" : "" }, flags), el("td", { class: v.radar ? "" : "muted" }, v.radar || "none")));
  }
  if (!(s.vehicles || []).length) vb.append(el("tr", {}, el("td", { colspan: 21, class: "muted" }, "None")));

  // the log: only what is new
  const log = $("log");
  const lines = (s.log || []).filter(([seq]) => seq > lastLogSeq);
  if ((s.log || []).length && s.log[s.log.length - 1][0] < lastLogSeq) { log.textContent = ""; lastLogSeq = 0; }
  if (lines.length) {
    const atBottom = log.scrollTop + log.clientHeight >= log.scrollHeight - 4;
    log.textContent += lines.map(([, t]) => t).join("\n") + "\n";
    lastLogSeq = lines[lines.length - 1][0];
    if (atBottom) log.scrollTop = log.scrollHeight;
  }
  const changed = runs.length !== (prev.runs || []).length || runs.some((r, i) => !prev.runs[i] || prev.runs[i].status !== r.status);
  if (changed) renderTree();
}

async function poll() {
  try {
    renderStatus(await api("/api/status"));
  } catch (e) {
    $("status").className = "pill error";
    $("status").textContent = "disconnected";
  }
  setTimeout(poll, 300);
}

async function init() {
  catalog = await api("/api/catalog");
  const speed = $("speed");
  for (const x of catalog.speeds) speed.append(el("option", { value: String(x) }, x === 0 ? "max" : x + "×"));
  speed.addEventListener("change", () => api("/api/speed", { speed: Number(speed.value) }));
  const tod = $("tod");
  for (const c of catalog.clocks || []) tod.append(el("option", { value: c }, c === "now" ? "wall clock" : c));
  tod.addEventListener("change", () => api("/api/clock", { clock: tod.value }));
  const sky = $("sky");
  for (const c of catalog.skies || []) sky.append(el("option", { value: c }, c));
  sky.addEventListener("change", () => api("/api/sky", { sky: sky.value }));
  $("follow").addEventListener("change", (e) => { if (e.target.value) api("/api/focus", { name: e.target.value }); });
  $("pause").addEventListener("click", () => api("/api/pause", { paused: !lastStatus.paused }));
  $("stop").addEventListener("click", () => api("/api/stop", {}));
  $("viewer").addEventListener("click", () => api("/api/viewer", {}));
  $("search").addEventListener("input", renderTree);
  let first = null;
  try { first = localStorage.getItem("testbench.selected"); } catch (e) { /* ignore */ }
  const s = catalog.scenarios.find((x) => x.id === first) || null;
  if (s) select(s); else renderTree();
  poll();
}

init();
