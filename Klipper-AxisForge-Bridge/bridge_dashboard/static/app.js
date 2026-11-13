const byId = (id) => document.getElementById(id);

function setBadge(id, text, good) {
  const badge = byId(id);
  badge.textContent = text;
  badge.className = `badge ${good === true ? "good" : good === false ? "bad" : ""}`;
}

function displayTemperature(value, target) {
  if (value == null) return "—";
  return `${Number(value).toFixed(1)} °C / ${Number(target || 0).toFixed(1)} °C`;
}

let latestSystemsReady = false;
let latestSystemsReason = "Checking Klipper and robot connections.";
let latestAxisForgeIdle = false;
let selectedPreflightState = null;
let syncSnapshot = { state: "idle", armed: false, job: null, control_ready: false };
let syncActionInFlight = false;
let loadCalibrationSnapshot = {state:"idle", busy:false};
let loadCalibrationAvailable = false, loadCalibrationInFlight = false, setupRefreshInFlight = false;
let robotConnectionSnapshot = {busy:false}, robotConnectInFlight = false;
let tcpSnapshot = null, tcpAvailable = false, tcpSaveInFlight = false, tcpRefreshInFlight = false;
let tcpDirty = false, tcpConflict = false, tcpRequestVersion = 0;
let setupRequestVersion = 0;
let calibrationVisibleToken = null, calibrationReadyAt = 0, uncertainCalibrationToken = null;
const consumedCalibrationTokens = new Set();
function setupBusy() { return tcpSaveInFlight || loadCalibrationSnapshot.busy || loadCalibrationInFlight || robotConnectionSnapshot.busy || robotConnectInFlight; }

let bedProbeSnapshot = {state:"idle", busy:false, confirmation:null};
let bedProbeAvailable = false;
let bedProbeActionInFlight = false;
let bedProbeRefreshInFlight = false;
let bedProbeRequestVersion = 0;
const consumedProbeConfirmations = new Set();
let visibleProbeConfirmation = null;
let probeConfirmationReadyAt = 0;
let uncertainProbeConfirmation = null;

function showMessage(id, text, isError = false) {
  const element = byId(id);
  element.textContent = text && typeof text === "object" ? JSON.stringify(text) : (text || "");
  element.className = `inline-message${text ? (isError ? " error" : " success") : ""}`;
}

async function request(path, options = {}) {
  const response = await fetch(path, {
    ...options,
    headers: { "Content-Type": "application/json", ...(options.headers || {}) },
  });
  const body = await response.json().catch(() => ({}));
  if (!response.ok) throw new Error(body.detail || `Request failed (${response.status})`);
  return body;
}

async function refreshStatus() {
  try {
    const status = await request("/api/status");
    const ros = status.ros;
    const klipper = status.klipper;
    const job = status.axisforge;
    const allReady = ros.connected && klipper.connected && klipper.state === "ready";
    latestSystemsReady = Boolean(allReady && ros.joint_state_fresh);
    latestSystemsReason = !ros.connected ? "Reconnect ROS / the robot driver." :
      !klipper.connected ? "Reconnect to Klipper / Moonraker." :
      klipper.state !== "ready" ? `Klipper is not ready: ${klipper.state_message || klipper.state}.` :
      !ros.joint_state_fresh ? "Wait for fresh robot joint feedback." : "";
    latestAxisForgeIdle = !job.running && !job.planning_check_running;
    byId("overall-dot").className = `dot ${allReady ? "good" : "bad"}`;
    byId("overall-state").textContent = allReady ? "Systems connected" : "Check system status";

    setBadge("ros-badge", ros.connected ? "Connected" : "Offline", ros.connected);
    byId("joint-state").textContent = ros.joint_state_fresh
      ? `Fresh · ${ros.joint_state_age_s}s ago`
      : ros.connected ? "No fresh feedback" : "Unavailable";
    byId("joint-count").textContent = ros.joint_names?.length
      ? `${ros.joint_names.length} joints`
      : "—";

    setBadge(
      "klipper-badge",
      klipper.connected ? klipper.state : "Offline",
      klipper.connected && klipper.state === "ready",
    );
    byId("klipper-state").textContent = klipper.connected
      ? klipper.state_message || klipper.state
      : klipper.error || "Unavailable";
    byId("extruder-temp").textContent = displayTemperature(
      klipper.extruder?.temperature, klipper.extruder?.target,
    );
    byId("bed-temp").textContent = displayTemperature(
      klipper.bed?.temperature, klipper.bed?.target,
    );

    setBadge("job-badge", job.running ? "Running" : "Idle", job.running);
    byId("job-name").textContent = job.job || "No active job";
    byId("job-mode").textContent = job.dry_run == null
      ? (job.return_code == null ? "—" : `Exited (${job.return_code})`)
      : job.dry_run ? "Planning only" : "Live";
    byId("start-job").disabled = job.running || !ros.connected || !byId("job-select").value;
    byId("stop-job").disabled = !job.running;
    updateSyncControls();
  } catch (error) {
    byId("overall-dot").className = "dot bad";
    byId("overall-state").textContent = error.message;
    latestSystemsReady = false;
    latestSystemsReason = `Could not check system status: ${error.message}`;
    latestAxisForgeIdle = false;
    updateSyncControls();
  }
}

async function loadJobs() {
  const select = byId("job-select");
  try {
    const result = await request("/api/jobs");
    select.replaceChildren();
    if (!result.jobs.length) {
      select.add(new Option("No staged G-code files", ""));
      showMessage("jobs-message", result.error || "No G-code jobs found", true);
      return;
    }
    select.add(new Option("Select a job…", ""));
    for (const job of result.jobs) select.add(new Option(job, job));
    showMessage("jobs-message", "");
  } catch (error) {
    select.replaceChildren(new Option("Could not load jobs", ""));
    showMessage("jobs-message", error.message, true);
  }
}

async function loadLinks() {
  try {
    const links = await request("/api/links");
    const fluidd = byId("fluidd-link");
    const mainsail = byId("mainsail-link");
    const placeholder = byId("printer-ui-placeholder");
    if (links.fluidd) {
      fluidd.href = links.fluidd;
      fluidd.hidden = false;
    }
    if (links.mainsail) {
      mainsail.href = links.mainsail;
      mainsail.hidden = false;
    }
    if (!links.fluidd && !links.mainsail && links.printer_ui) {
      placeholder.href = links.printer_ui;
      placeholder.target = "_blank";
      placeholder.rel = "noopener noreferrer";
      placeholder.classList.remove("unavailable");
      placeholder.removeAttribute("aria-disabled");
      placeholder.title = "Open the printer web UI on the Moonraker host";
    } else {
      placeholder.hidden = Boolean(links.fluidd || links.mainsail);
    }
    byId("alps-link").href = links.alps_logger;
  } catch (error) {
    showMessage("jobs-message", `Could not load tool links: ${error.message}`, true);
  }
}

const chartColors = ["#c9a7ff", "#58a6ff", "#4ade80", "#fbbf24", "#fb7185", "#22d3ee"];
let temperatureHistory = {};
let temperatureColors = new Map();
let telemetryInFlight = false;

function percentage(value) {
  return Number.isFinite(value) ? `${(value * 100).toFixed(0)}%` : "Unavailable";
}

function temperature(value) {
  return Number.isFinite(value) ? `${value.toFixed(1)} °C` : "Unavailable";
}

function renderTelemetryList(id, entries, emptyMessage = "None reported", colors = new Map()) {
  const list = byId(id);
  list.replaceChildren();
  if (!entries.length) {
    const row = document.createElement("div");
    const label = document.createElement("dt");
    label.textContent = emptyMessage;
    row.append(label);
    list.append(row);
    return;
  }
  for (const [name, value] of entries) {
    const row = document.createElement("div");
    const label = document.createElement("dt");
    const detail = document.createElement("dd");
    if (colors.has(name)) {
      const marker = document.createElement("span");
      marker.className = "sensor-color";
      marker.style.backgroundColor = colors.get(name);
      marker.setAttribute("aria-hidden", "true");
      label.append(marker);
    }
    label.append(document.createTextNode(name));
    detail.textContent = value;
    row.append(label, detail);
    list.append(row);
  }
}

function drawTemperatureHistory() {
  const canvas = byId("temperature-chart");
  const width = canvas.clientWidth;
  if (!width) return;
  const height = 200;
  const scale = Math.min(window.devicePixelRatio || 1, 2);
  canvas.width = width * scale;
  canvas.height = height * scale;
  const ctx = canvas.getContext("2d");
  ctx.scale(scale, scale);
  const series = Object.entries(temperatureHistory)
    .filter(([, values]) => Array.isArray(values) && values.some(Number.isFinite));
  if (!series.length) return;

  const samples = series.flatMap(([, values]) => values.filter(Number.isFinite));
  const min = Math.min(0, Math.floor(Math.min(...samples) / 10) * 10);
  const max = Math.max(min + 10, Math.ceil(Math.max(...samples) / 10) * 10);
  const left = 44, right = width - 12, top = 12, bottom = height - 20;
  ctx.font = "11px sans-serif";
  ctx.fillStyle = "#8b949e";
  ctx.strokeStyle = "#30363d";
  for (let i = 0; i <= 4; i++) {
    const y = top + (bottom - top) * i / 4;
    ctx.beginPath();
    ctx.moveTo(left, y);
    ctx.lineTo(right, y);
    ctx.stroke();
    ctx.fillText(`${(max - (max - min) * i / 4).toFixed(0)}°`, 4, y + 4);
  }
  const longest = Math.max(...series.map(([, values]) => values.length), 2);
  series.forEach(([name, values]) => {
    ctx.strokeStyle = temperatureColors.get(name);
    ctx.lineWidth = 2;
    ctx.beginPath();
    let active = false;
    values.forEach((value, i) => {
      if (!Number.isFinite(value)) {
        active = false;
        return;
      }
      const x = left + (right - left) * (longest - values.length + i) / (longest - 1);
      const y = bottom - (bottom - top) * (value - min) / (max - min);
      if (active) ctx.lineTo(x, y);
      else ctx.moveTo(x, y);
      active = true;
    });
    ctx.stroke();
  });
}

function renderHistory(history, error, sensorNames) {
  const series = sensorNames
    .filter((name) => Array.isArray(history[name]) && history[name].some(Number.isFinite));
  temperatureHistory = Object.fromEntries(series.map((name) => [name, history[name]]));
  temperatureColors = new Map(series.map((name, index) => [name, chartColors[index % chartColors.length]]));
  byId("temperature-chart-message").textContent =
    error || (series.length ? "" : "No temperature history available.");
  byId("temperature-chart-message").className = `inline-message${error ? " error" : ""}`;
  drawTemperatureHistory();
}

function formatDuration(seconds) {
  if (!Number.isFinite(seconds) || seconds < 0) return "—";
  const total = Math.floor(seconds);
  const hours = Math.floor(total / 3600);
  const minutes = String(Math.floor(total % 3600 / 60)).padStart(2, "0");
  return `${hours}:${minutes}:${String(total % 60).padStart(2, "0")}`;
}

function renderPrintStats(data, metadataError) {
  byId("print-status").textContent = [data.state || "Unavailable", data.filename]
    .filter(Boolean).join(" · ");
  byId("print-speed").textContent = Number.isFinite(data.velocity)
    ? `${data.velocity.toFixed(1)} mm/s` : "—";
  const flow = Number.isFinite(data.extruder_velocity) && Number.isFinite(data.filament_diameter)
    && data.filament_diameter > 0
    ? Math.max(0, data.extruder_velocity) * Math.PI * (data.filament_diameter / 2) ** 2 : null;
  byId("print-flow").textContent = flow == null ? "—" : `${flow.toFixed(1)} mm³/s`;
  byId("print-filament").textContent = Number.isFinite(data.filament_used)
    ? `${data.filament_used.toFixed(2)} mm` : "—";
  byId("print-layer").textContent = Number.isInteger(data.current_layer) && Number.isInteger(data.total_layer)
    ? `${data.current_layer} of ${data.total_layer}` : "—";
  const progress = data.progress;
  const remaining = ["printing", "paused"].includes(data.state)
    && Number.isFinite(progress) && progress >= 0.01 && progress < 1
    && Number.isFinite(data.print_duration) && data.print_duration > 0
    ? data.print_duration * (1 - progress) / progress : null;
  byId("print-estimate").textContent = formatDuration(remaining);
  byId("print-slicer").textContent = data.slicer_estimate > 0
    ? formatDuration(data.slicer_estimate) : "—";
  byId("print-total").textContent = formatDuration(data.total_duration);
  byId("print-eta").textContent = remaining != null && data.state === "printing"
    ? new Date(Date.now() + remaining * 1000).toLocaleTimeString() : "—";
  byId("print-info-message").textContent = metadataError
    ? `Slicer metadata unavailable: ${metadataError}` : "";
  byId("print-info-message").className = `inline-message${metadataError ? " error" : ""}`;
}

async function refreshTelemetry() {
  if (telemetryInFlight) return;
  telemetryInFlight = true;
  try {
    const data = await request("/api/klipper/telemetry");
    renderHistory(data.history, data.history_error, data.temperatures.map((item) => item.name));
    renderTelemetryList("telemetry-temperatures", data.temperatures.map((item) => [
      item.name,
      item.target == null
        ? temperature(item.temperature)
        : `${temperature(item.temperature)} / ${temperature(item.target)} target`,
    ]), "None reported", temperatureColors);
    renderTelemetryList("telemetry-fans", data.fans.map((item) => [item.name, percentage(item.speed)]));
    renderPrintStats(data.print, data.metadata_error);
    byId("telemetry-message").textContent = "";
    byId("telemetry-updated").textContent = `Updated ${new Date().toLocaleTimeString()} · refreshes every 5 seconds`;
  } catch (error) {
    renderTelemetryList("telemetry-temperatures", [], "Unavailable");
    renderTelemetryList("telemetry-fans", [], "Unavailable");
    renderPrintStats({}, null);
    byId("telemetry-updated").textContent = "";
    byId("telemetry-message").textContent = `Toolhead data unavailable: ${error.message}`;
    byId("telemetry-message").className = "inline-message error";
    renderHistory({}, "Temperature history unavailable.", []);
  } finally {
    telemetryInFlight = false;
  }
}

const toolFrames = new Map();
const toolTabs = [
  byId("setup-tab"),
  byId("dashboard-tab"),
  byId("printer-ui-placeholder"),
  byId("fluidd-link"),
  byId("mainsail-link"),
  byId("alps-link"),
];

function selectTab(tab) {
  for (const item of toolTabs) {
    item.classList.toggle("active", item === tab);
    if (item === tab) item.setAttribute("aria-current", "page");
    else item.removeAttribute("aria-current");
  }
}

byId("dashboard-tab").addEventListener("click", (event) => {
  event.preventDefault();
  selectTab(event.currentTarget);
  byId("setup-view").hidden = true;
  byId("dashboard-view").hidden = false;
  byId("embedded-view").hidden = true;
  drawTemperatureHistory();
});

byId("setup-tab").addEventListener("click", event => {
  event.preventDefault(); selectTab(event.currentTarget);
  byId("setup-view").hidden = false; byId("dashboard-view").hidden = true; byId("embedded-view").hidden = true;
});
byId("setup-continue").addEventListener("click", () => byId("dashboard-tab").click());
byId("setup-home-link").addEventListener("click", () => byId("bed-probe-start").closest("section").scrollIntoView({behavior:"smooth"}));

for (const tab of toolTabs.slice(2)) {
  tab.addEventListener("click", (event) => {
    if (event.button !== 0 || event.ctrlKey || event.metaKey || event.shiftKey || event.altKey) return;
    event.preventDefault();
    if (tab.getAttribute("aria-disabled") === "true" || !tab.href || tab.hidden) return;

    selectTab(tab);
    byId("setup-view").hidden = true;
    byId("dashboard-view").hidden = true;
    byId("embedded-view").hidden = false;
    byId("embedded-title").textContent = tab.textContent.replace("↗", "").trim();
    byId("embedded-external").href = tab.href;
    byId("printer-warning").hidden = tab.id === "alps-link";
    for (const frame of toolFrames.values()) frame.hidden = true;

    const url = new URL(tab.href);
    if (location.protocol === "https:" && url.protocol === "http:") {
      byId("embedded-message").textContent =
        "An HTTPS page cannot embed an HTTP tool. Open it in a new tab or use HTTPS.";
      return;
    }

    byId("embedded-message").textContent = tab.id === "alps-link"
      ? "Allow local network access when prompted. If blocked, enable it in this dashboard's browser site settings, then reconnect."
      : "If the tool cannot be displayed or a device permission fails, open it in a new tab.";
    let frame = toolFrames.get(tab.id);
    if (!frame) {
      frame = document.createElement("iframe");
      frame.className = "tool-frame";
      frame.title = byId("embedded-title").textContent;
      if (tab.id === "alps-link") {
        // ALPS fetches Moonraker and its load-cell stream from the printer's LAN IP.
        // Delegate to this frame's origin; the browser still asks for user consent.
        // Keep the original LNA token for Chrome versions before the split permissions.
        frame.allow = "serial; local-network-access; local-network; loopback-network";
      }
      frame.src = url.href;
      byId("embedded-frames").append(frame);
      toolFrames.set(tab.id, frame);
    }
    frame.hidden = false;
  });
}

byId("start-job").addEventListener("click", async () => {
  showMessage("job-message", "");
  const job = byId("job-select").value;
  if (!job) return showMessage("job-message", "Select a G-code file first.", true);
  try {
    const result = await request("/api/axisforge/start", {
      method: "POST",
      body: JSON.stringify({ job, dry_run: true }),
    });
    showMessage("job-message", `Started planning check for ${result.job}.`);
    refreshStatus();
  } catch (error) {
    showMessage("job-message", error.message, true);
  }
});

byId("job-select").addEventListener("change", () => {
  refreshStatus();
  loadPreflight();
});

let syncRefreshInFlight = false;
let heaterOptionsInitialized = false;
let selectedHeaterChecks = null;
let selectedMotionOptions = null;
let motionOptionsInitialized = false;
let clockOptionsInitialized = false;
let selectedClockOptions = null;
const clockInputs = {max_uncertainty_ms:"clock-uncertainty", max_alignment_error_ms:"clock-alignment"};
const clockPreferences = (() => {
  try { return JSON.parse(localStorage.getItem("axisforge.clockOptions.v1")) || {}; }
  catch { return {}; }
})();

function clockOptions() {
  return Object.fromEntries(Object.entries(clockInputs).map(([key,id]) => [key,Number(byId(id).value)]));
}

function clockOptionsMatch() {
  const chosen = clockOptions();
  return selectedClockOptions && Object.keys(chosen).every(key => chosen[key] === selectedClockOptions[key]);
}

function clockOptionsValid() {
  const chosen = clockOptions();
  // Explicit bounds also apply to readonly preset inputs (which skip browser validation).
  return syncSnapshot.clock_options_supported === true &&
    Object.values(clockInputs).every(id => byId(id).value !== "" && byId(id).checkValidity()) &&
    Object.values(chosen).every(value => Number.isFinite(value) && value >= 2 && value <= 20) &&
    chosen.max_uncertainty_ms <= chosen.max_alignment_error_ms;
}

function updateClockOptions() {
  const profiles = syncSnapshot.clock_profiles;
  if (!Array.isArray(profiles)) return;
  if (!clockOptionsInitialized) {
    const select = byId("clock-profile");
    select.replaceChildren();
    for (const profile of [...profiles,{id:"custom",label:"Custom — 2 to 20 ms"}]) {
      const option = document.createElement("option");
      option.value = profile.id; option.textContent = profile.label; select.append(option);
    }
    const chosen = syncSnapshot.busy ? syncSnapshot.clock_options : {...syncSnapshot.clock_defaults,...clockPreferences};
    for (const [key,id] of Object.entries(clockInputs)) {
      if (typeof chosen?.[key] === "number" && Number.isFinite(chosen[key])) byId(id).value = chosen[key];
    }
    select.value = profiles.find(p => Object.keys(clockInputs).every(key => p.options[key] === clockOptions()[key]))?.id || "custom";
    clockOptionsInitialized = true;
  }
  const custom = byId("clock-profile").value === "custom";
  for (const id of Object.values(clockInputs)) byId(id).readOnly = !custom;
  const chosen = clockOptions();
  const profile = profiles.find(p => p.id === byId("clock-profile").value);
  byId("clock-options-note").textContent = `${profile?.network || "Custom commissioning limits. Use a measured local connection; both hosts on Ethernet is preferred."} Raising a limit accepts more uncertainty; it does not improve synchronization. Clock profiles do not change heartbeat protection.`;
  const budget = syncSnapshot.clock_guidance_distance_mm;
  const speed = budget*1000/chosen.max_alignment_error_ms;
  let guidance = clockOptionsValid()
    ? `Suggested peak XYZ speed: up to ${speed.toFixed(2)} mm/s for an illustrative ${Number(budget).toFixed(2)} mm timing-displacement allowance (distance ÷ time). This is guidance, not a speed override or a certified safe speed; verify extrusion and rotary motion on the cell.`
    : "Use finite limits from 2 to 20 ms, with scheduling uncertainty no greater than running alignment. Update the worker if timing options are unavailable.";
  const planned = syncSnapshot.clock_speed_guidance;
  if (planned && syncSnapshot.job === byId("job-select").value &&
      Object.keys(chosen).every(key => chosen[key] === syncSnapshot.clock_options?.[key])) {
    guidance += ` Planned peak: ${planned.peak_xyz_speed_mm_s.toFixed(2)} mm/s → ${planned.timing_displacement_mm.toFixed(3)} mm at this limit.${planned.exceeds_guidance ? " Above the illustrative allowance; reduce G-code feed or master velocity limits and rerun preflight if you need that allowance." : ""}`;
  }
  byId("clock-speed-note").textContent = guidance;
  const measurement = syncSnapshot.clock_measurement;
  byId("clock-measurement").textContent = measurement
    ? `Last clock check (${syncSnapshot.job || "job"}, ${measurement.age_s.toFixed(1)} s ago): uncertainty ${measurement.uncertainty_ms.toFixed(2)} ms${measurement.alignment_error_ms == null ? "" : `; running alignment ${measurement.alignment_error_ms.toFixed(2)} ms`}. Recorded limits: ${syncSnapshot.clock_options.max_uncertainty_ms}/${syncSnapshot.clock_options.max_alignment_error_ms} ms.`
    : "Clock measurements appear during preflight and execution.";
  const heartbeat = syncSnapshot.heartbeat;
  byId("heartbeat-measurement").textContent = heartbeat?.last_reply_ms == null ? ""
    : `Klipper heartbeat: ${heartbeat.active ? (heartbeat.lease_mode === "cold_preflight" ? "cold preflight (bounded recovery)" : "strict control") : "inactive"}; last exchange ${heartbeat.last_reply_ms.toFixed(1)} ms; cold retries ${heartbeat.cold_retries || 0}; maximum dashboard scheduling delay ${(heartbeat.max_event_loop_delay_ms || 0).toFixed(1)} ms.${heartbeat.active && heartbeat.retrying ? " Reconnecting within the cold-preflight deadline…" : ""}`;
  const robotHeartbeat = syncSnapshot.robot_heartbeat;
  if (robotHeartbeat?.last_reply_ms != null) {
    byId("heartbeat-measurement").textContent += ` Robot heartbeat: ${robotHeartbeat.active ? "strict control" : "inactive"}; last exchange ${robotHeartbeat.last_reply_ms.toFixed(1)} ms; maximum ${robotHeartbeat.max_reply_ms.toFixed(1)} ms.`;
  }
}

for (const id of ["clock-profile",...Object.values(clockInputs)]) {
  byId(id).addEventListener("change", () => {
    if (id === "clock-profile") {
      const profile = (syncSnapshot.clock_profiles || []).find(p => p.id === byId(id).value);
      if (profile) for (const [key,input] of Object.entries(clockInputs)) byId(input).value = profile.options[key];
    }
    if (clockOptionsValid()) {
      Object.assign(clockPreferences,clockOptions());
      try { localStorage.setItem("axisforge.clockOptions.v1",JSON.stringify(clockPreferences)); } catch {}
    }
    updateClockOptions();
    showMessage("preflight-message","Clock timing changed. Run preflight again before Start.");
    loadPreflight();
    updateSyncControls();
  });
}
const motionInputs = {square_corner_velocity:"corner-scv", junction_deviation_mm:"corner-jd",
  blend_tolerance_mm:"corner-tolerance", blend_speed_mm_s:"corner-speed"};
const motionPreferences = (() => {
  try { return JSON.parse(localStorage.getItem("axisforge.motionOptions.v1")) || {}; }
  catch { return {}; }
})();

function motionOptions() {
  return {policy:byId("corner-policy").value,
    ...Object.fromEntries(Object.entries(motionInputs).map(([key,id]) => [key,Number(byId(id).value)]))};
}

function motionOptionsMatch() {
  const chosen = motionOptions();
  return selectedMotionOptions && Object.keys(chosen).every(key => chosen[key] === selectedMotionOptions[key]);
}

function motionOptionsValid() {
  return Object.values(motionInputs).every(id => byId(id).checkValidity()) &&
    (syncSnapshot.corner_policies || ["exact"]).includes(byId("corner-policy").value);
}

function updateMotionOptions() {
  const policies = syncSnapshot.corner_policies || ["exact"];
  if (!motionOptionsInitialized) {
    const chosen = {...syncSnapshot.motion_defaults,...motionPreferences};
    if (["exact","klipper","marlin","blend"].includes(chosen.policy)) byId("corner-policy").value = chosen.policy;
    for (const [key,id] of Object.entries(motionInputs)) {
      if (typeof chosen[key] === "number" && Number.isFinite(chosen[key])) byId(id).value = chosen[key];
    }
    motionOptionsInitialized = true;
  }
  for (const option of byId("corner-policy").options) option.disabled = !policies.includes(option.value);
  const policy = byId("corner-policy").value;
  for (const name of ["klipper","marlin","blend"]) byId(`corner-${name}`).hidden = policy !== name;
  const descriptions = {
    exact:"Preserves the programmed path and stops at sharp corners. Straight moves remain connected.",
    klipper:"Sets corner speed using Klipper’s square-corner model. No explicit fillets; robot preflight may require a lower corner speed.",
    marlin:"Sets corner speed using Marlin’s Junction Deviation model. This is a speed parameter, not a geometric rounding tolerance.",
    blend:"Rounds eligible XYZ corners within the selected deviation. Preserves source filament amounts. Tight tolerances can require slower turns; changing orientation and reversals still stop."
  };
  const pa = syncSnapshot.pressure_advance;
  byId("motion-options-note").textContent = `${descriptions[policy]} Native Klipper pressure advance${pa == null ? "" : ` (${pa})`} is retained in every mode. Changing options requires preflight again.`;
}

for (const id of ["corner-policy",...Object.values(motionInputs)]) {
  byId(id).addEventListener("change", () => {
    if (motionOptionsValid()) {
      Object.assign(motionPreferences,motionOptions());
      try { localStorage.setItem("axisforge.motionOptions.v1",JSON.stringify(motionPreferences)); } catch {}
    }
    updateMotionOptions();
    showMessage("preflight-message","Corner options changed. Run preflight to validate this motion.");
    loadPreflight();
    updateSyncControls();
  });
}
const heaterPreferences = (() => {
  try { return JSON.parse(localStorage.getItem("axisforge.heaterChecks.v1")) || {}; }
  catch { return {}; }
})();

function heaterChecks() {
  return {extruder:byId("check-extruder").checked, heater_bed:byId("check-bed").checked};
}

function heaterChecksMatch() {
  const chosen = heaterChecks();
  return selectedHeaterChecks && Object.keys(chosen).every(name => chosen[name] === selectedHeaterChecks[name]);
}

function updateHeaterOptions() {
  const configured = syncSnapshot.configured_heaters;
  if (!Array.isArray(configured)) return;
  for (const [heater, id, statusId] of [["extruder","check-extruder","extruder-check-state"], ["heater_bed","check-bed","bed-check-state"]]) {
    const available = configured.includes(heater);
    if (!heaterOptionsInitialized) {
      byId(id).checked = typeof heaterPreferences[heater] === "boolean" ? heaterPreferences[heater] : available;
    }
    byId(statusId).textContent = available ? "Configured in Klipper" : "Not configured in Klipper · leave off to ignore its commands";
  }
  heaterOptionsInitialized = true;
}

for (const id of ["check-extruder", "check-bed"]) {
  byId(id).addEventListener("change", () => {
    const choices = heaterChecks();
    Object.assign(heaterPreferences, choices);
    try { localStorage.setItem("axisforge.heaterChecks.v1", JSON.stringify(choices)); } catch {}
    showMessage("preflight-message", "Heater options changed. Run preflight to validate these choices.");
    loadPreflight();
    updateSyncControls();
  });
}

function syncStartBlocker(job, preflightReady) {
  if (setupBusy()) return "Finish setup or scale calibration before starting a job.";
  if (bedProbeSnapshot.busy || bedProbeActionInFlight) return "Finish or stop bed homing/probing before starting a job.";
  if (syncActionInFlight) return "Waiting for the dashboard request to finish.";
  if (syncSnapshot.busy) return syncSnapshot.message || "Wait for the current operation to finish.";
  if (!job) return "Select a G-code file and run preflight.";
  if (!latestSystemsReady) return latestSystemsReason;
  if (!latestAxisForgeIdle) return "Wait for the standalone AxisForge job or planning check to finish.";
  if (!syncSnapshot.control_ready) return syncSnapshot.reason || "Synchronized services are unavailable.";
  if (!heaterOptionsInitialized) return "Checking the heater configuration.";
  if (!motionOptionsValid()) return "Check corner values and update both services if the selected policy is unavailable.";
  if (!clockOptionsValid()) return "Check clock timing limits and update the AxisForge worker if timing options are unavailable.";
  if (selectedHeaterChecks && !heaterChecksMatch()) return "Heater options differ from the passed preflight. Restore those choices or run preflight again.";
  if (selectedMotionOptions && !motionOptionsMatch()) return "Corner options differ from the passed preflight. Restore those choices or run preflight again.";
  if (selectedClockOptions && !clockOptionsMatch()) return "Clock timing differs from the passed preflight. Restore those limits or run preflight again.";
  if (!preflightReady) return "Run preflight on the selected file. Results expire after 15 minutes.";
  if (!syncSnapshot.robot_ready) {
    const reason = syncSnapshot.robot_reason;
    if (reason === "Start the UR External Control program") {
      return "On the UR teach pendant, run the program containing External Control and set the speed slider to 100%.";
    }
    return reason || "The robot is not ready for motion. Check the UR controller.";
  }
  return "";
}

function updateSyncControls() {
  const job = byId("job-select").value;
  const busy = Boolean(syncSnapshot.busy || bedProbeSnapshot.busy || bedProbeActionInFlight || setupBusy());
  const ready = selectedPreflightState === "preflight_passed" && syncSnapshot.job === job && heaterChecksMatch() && motionOptionsMatch() && clockOptionsMatch();
  const blocker = syncStartBlocker(job, ready);
  byId("heater-check-options").disabled = busy || syncActionInFlight || !heaterOptionsInitialized;
  byId("motion-options").disabled = busy || syncActionInFlight || !motionOptionsInitialized;
  byId("clock-options").disabled = busy || syncActionInFlight || !clockOptionsInitialized;
  byId("job-select").disabled = busy || syncActionInFlight;
  byId("upload-job").disabled = busy || syncActionInFlight;
  byId("preflight-both").disabled = busy || syncActionInFlight || !heaterOptionsInitialized || !motionOptionsInitialized || !motionOptionsValid() || !clockOptionsInitialized || !clockOptionsValid() || !job || !latestSystemsReady || !latestAxisForgeIdle || !syncSnapshot.control_ready;
  byId("sync-start").disabled = Boolean(blocker);
  byId("sync-abort").disabled = !busy;
  byId("start-job").disabled = busy || !job || !latestSystemsReady || !latestAxisForgeIdle;
  const label = bedProbeSnapshot.busy || bedProbeActionInFlight ? "Bed homing" : busy ? syncSnapshot.stage : !syncSnapshot.control_ready ? "Services unavailable" : !blocker ? "Ready to start" : ready ? "Start blocked" : "Preflight required";
  setBadge("sync-availability-badge", label || "Working", !blocker ? true : ready && !busy ? false : undefined);
  byId("step-sync").textContent = blocker || "Ready to start";
  const startStatus = byId("sync-start-status");
  startStatus.textContent = busy ? blocker : blocker ? `Start blocked: ${blocker}` : "Ready to start synchronized heating and motion.";
  startStatus.className = `inline-message${!blocker ? " success" : ready && !busy ? " error" : ""}`;
  updateBedProbeControls();
  updateSetupControls();
}

function updateBedProbeControls() {
  const body = bedProbeSnapshot;
  const busy = Boolean(body.busy);
  const occupied = busy || setupBusy() || bedProbeActionInFlight || syncActionInFlight || syncSnapshot.busy || !latestAxisForgeIdle;
  const liveReason = !bedProbeAvailable ? "Bed-probe service is unavailable." :
    !body.configured ? "Configure AxisForge before homing." :
    occupied ? "Finish or stop the current operation before starting another routine." :
    !latestSystemsReady ? latestSystemsReason :
    !syncSnapshot.robot_ready ? (syncSnapshot.robot_reason || "Check the robot connection and External Control program.") : "";
  byId("bed-probe-start").disabled = Boolean(liveReason);
  byId("bed-probe-simulate").disabled = !bedProbeAvailable || !body.configured || occupied;
  byId("bed-probe-abort").disabled = !busy || bedProbeActionInFlight;
  byId("bed-probe-readiness").textContent = busy ? "Heater targets must stay off. Stop probing cancels motion without returning the arm." :
    liveReason || "Ready. The first click checks readiness; motion waits for your confirmation below.";
  const labels = {idle:"Not run",starting:"Checking",waiting_surface:"Confirm surface",homing_fast:"Finding bed",homing_slow:"Fine contact",moving_front_left:"Moving to first point",waiting_front_left:"Confirm front-left",meshing:"Measuring mesh",returning:"Returning",complete:body.dry_run ? "Simulation complete" : "Mesh complete",failed:"Failed",aborted:"Stopped",stopping:"Stopping"};
  const label = labels[body.state] || body.state;
  setBadge("bed-probe-badge", label, body.state === "complete" && !body.dry_run ? true : ["failed","aborted"].includes(body.state) ? false : undefined);
  byId("step-home").textContent = body.state === "idle" ? "Locate the surface and measure its mesh" : `${body.dry_run ? "Simulation · " : ""}${label}`;
  byId("bed-probe-message").textContent = `${body.dry_run ? "Simulation: " : ""}${body.message || ""}`;
  byId("bed-probe-message").className = `inline-message${["failed","aborted"].includes(body.state) ? " error" : ""}`;
  const threshold = Number(body.trigger_value);
  const maximum = Number(body.max_value);
  const unit = body.trigger_unit || (body.trigger_mode === "raw" ? "counts" : "g");
  byId("bed-probe-sensor").textContent = Number.isFinite(threshold) ?
    `${body.trigger_mode === "raw" ? "Raw sensor mode" : "Calibrated sensor mode"}: ${threshold.toLocaleString()} ${unit} contact threshold${Number.isFinite(maximum) ? ` · ${maximum.toLocaleString()} ${unit} stop threshold` : ""}${body.trigger_mode === "raw" ? ". Uses the unloaded starting baseline; no conversion to grams." : "."}` : "";
  byId("bed-probe-progress-container").hidden = !busy && body.state !== "complete";
  byId("bed-probe-progress").value = body.progress || 0;
  byId("bed-probe-progress-label").textContent = `${label} · ${body.current_point || 0}/${body.total_points || 9} mesh points`;
  const confirm = body.confirmation;
  const requested = busy && confirm && !consumedProbeConfirmations.has(confirm.token) && ["surface","front_left"].includes(confirm.stage);
  if (requested && visibleProbeConfirmation !== confirm.token) {
    visibleProbeConfirmation = confirm.token;
    probeConfirmationReadyAt = performance.now() + 750;
    window.setTimeout(updateBedProbeControls, 760);
  }
  byId("bed-probe-confirmation").hidden = !requested;
  byId("bed-probe-confirm").disabled = !requested || bedProbeActionInFlight || performance.now() < probeConfirmationReadyAt;
  if (requested) {
    const first = confirm.stage === "surface";
    byId("bed-probe-confirm-title").textContent = first ? "Confirm the surface before descending" : "Confirm the front-left mesh point";
    byId("bed-probe-confirm-detail").textContent = `${body.dry_run ? "This is a simulation; no hardware will move. " : ""}${first ?
      "Confirm the flat build surface is underneath the upright nozzle. The nozzle must be clean, cool, clear of the surface and unloaded. Keep the area clear; confirmation starts a vertical descent." :
      "Confirm the nozzle is above the intended first mesh point and this is the front-leftmost position of the mesh. Confirmation starts the automatic nine-point scan."}`;
    byId("bed-probe-confirm").textContent = first ? "Surface confirmed — begin Z homing" : "Front-left confirmed — probe mesh";
    const position = body.position_mm;
    const grid = body.first_point;
    const positionText = Array.isArray(position) && position.length === 3 ?
      `Nozzle position in robot Base: X ${Number(position[0]).toFixed(2)}, Y ${Number(position[1]).toFixed(2)}, Z ${Number(position[2]).toFixed(2)} mm.` : "";
    const gridText = !first && Array.isArray(grid) && grid.length === 2 ?
      ` Mesh point in G-code: X ${Number(grid[0]).toFixed(2)}, Y ${Number(grid[1]).toFixed(2)} mm.` : "";
    byId("bed-probe-position").textContent = positionText + gridText;
  }
}

async function refreshBedProbe() {
  if (bedProbeRefreshInFlight || bedProbeActionInFlight) return;
  bedProbeRefreshInFlight = true;
  const version = bedProbeRequestVersion;
  try {
    const result = await request("/api/bed-probe/status");
    if (version !== bedProbeRequestVersion) return;
    if (uncertainProbeConfirmation) {
      // A dropped HTTP request may never reach the server. Only a fresh status
      // with the same still-pending nonce permits another explicit click.
      if (result.confirmation?.stage === uncertainProbeConfirmation.stage &&
          result.confirmation?.token === uncertainProbeConfirmation.token) {
        consumedProbeConfirmations.delete(uncertainProbeConfirmation.token);
      }
      uncertainProbeConfirmation = null;
    }
    const finished = bedProbeSnapshot.busy && !result.busy;
    bedProbeSnapshot = result;
    bedProbeAvailable = true;
    updateSyncControls();
    if (finished) { selectedPreflightState = null; await loadPreflight(); refreshSync(); }
  } catch (error) {
    if (version !== bedProbeRequestVersion) return;
    bedProbeAvailable = false;
    bedProbeSnapshot.confirmation = null;
    bedProbeSnapshot.message = `Could not check bed probing: ${error.message}`;
    updateSyncControls();
  } finally { bedProbeRefreshInFlight = false; }
}

async function bedProbeAction(action, data = {}) {
  if (bedProbeActionInFlight) return;
  bedProbeActionInFlight = true;
  bedProbeRequestVersion++;
  if (action === "start") { selectedPreflightState = null; uncertainProbeConfirmation = null; }
  updateSyncControls();
  try {
    bedProbeSnapshot = await request(`/api/bed-probe/${action}`, {method:"POST",body:JSON.stringify(data)});
    bedProbeAvailable = true;
  } catch (error) {
    if (action === "confirm") uncertainProbeConfirmation = data;
    bedProbeSnapshot.message = error.message;
    showMessage("bed-probe-message",error.message,true);
  } finally {
    bedProbeActionInFlight = false;
    updateSyncControls();
    refreshBedProbe();
    refreshSync();
  }
}

byId("bed-probe-start").addEventListener("click", () => bedProbeAction("start", {dry_run:false}));
byId("bed-probe-simulate").addEventListener("click", () => bedProbeAction("start", {dry_run:true}));
byId("bed-probe-abort").addEventListener("click", () => bedProbeAction("abort"));
byId("bed-probe-confirm").addEventListener("click", () => {
  const confirmation = bedProbeSnapshot.confirmation;
  if (!confirmation || bedProbeActionInFlight || consumedProbeConfirmations.has(confirmation.token)) return;
  consumedProbeConfirmations.add(confirmation.token);
  bedProbeAction("confirm", {stage:confirmation.stage, token:confirmation.token});
});

async function loadPreflight() {
  const job = byId("job-select").value;
  selectedPreflightState = null;
  selectedHeaterChecks = null;
  selectedMotionOptions = null;
  selectedClockOptions = null;
  if (job) {
    try {
      const result = await request(`/api/jobs/${encodeURIComponent(job)}/preflight`);
      if (byId("job-select").value === job) {
        selectedHeaterChecks = result.heater_checks;
        selectedMotionOptions = result.motion_options;
        selectedClockOptions = result.clock_options;
        selectedPreflightState = result.state === "preflight_passed" && (!heaterChecksMatch() || !motionOptionsMatch() || !clockOptionsMatch()) ? "staged" : result.state;
      }
    } catch (error) {
      showMessage("preflight-message", error.message, true);
    }
  }
  setBadge("preflight-badge", selectedPreflightState === "preflight_passed" ? "Passed" : job ? "Preflight required" : "Select a job", selectedPreflightState === "preflight_passed" ? true : undefined);
  updateSyncControls();
}

byId("preflight-both").addEventListener("click", async () => {
  const job = byId("job-select").value;
  if (!job) return;
  syncActionInFlight = true;
  selectedPreflightState = null;
  showMessage("preflight-message", "");
  updateSyncControls();
  try {
    syncSnapshot = await request(`/api/jobs/${encodeURIComponent(job)}/preflight/both`, {
      method:"POST", body:JSON.stringify({heater_checks:heaterChecks(),motion_options:motionOptions(),clock_options:clockOptions()}),
    });
  } catch (error) {
    showMessage("preflight-message", error.message, true);
  } finally {
    syncActionInFlight = false;
    updateSyncControls();
    refreshSync();
  }
});

for (const action of ["start", "abort"]) {
  byId(`sync-${action}`).addEventListener("click", async () => {
    syncActionInFlight = true;
    updateSyncControls();
    try {
      syncSnapshot = await request(`/api/sync/${action}`, {
        method:"POST", body:JSON.stringify({job:byId("job-select").value, heater_checks:heaterChecks(),motion_options:motionOptions(),clock_options:clockOptions()}),
      });
    } catch (error) {
      showMessage("sync-message", error.message, true);
    } finally {
      syncActionInFlight = false;
      updateSyncControls();
      refreshSync();
    }
  });
}

async function refreshSync() {
  if (syncRefreshInFlight) return;
  syncRefreshInFlight = true;
  try {
    syncSnapshot = await request("/api/sync/status");
    updateHeaterOptions();
    updateMotionOptions();
    updateClockOptions();
    const body = syncSnapshot;
    showMessage("sync-message", body.message || body.reason || "Upload a file and run preflight.", ["failed","aborted"].includes(body.state));
    const list = byId("preflight-checks");
    list.replaceChildren();
    if (body.job === byId("job-select").value) {
      for (const check of body.checks || []) {
        const item = document.createElement("li");
        item.className = `preflight-${check.status}`;
        item.textContent = `${check.status === "pass" ? "✓" : check.status === "skip" ? "Skipped:" : "!"} ${check.participant}: ${check.name} — ${check.detail}`;
        list.append(item);
      }
    }
    byId("preflight-progress-container").hidden = !body.busy;
    byId("preflight-progress").max = 1;
    byId("preflight-progress").value = body.progress || 0;
    byId("preflight-progress-label").textContent = `${body.job || "Job"} · ${body.stage || body.state}`;
    const readings = Object.entries(body.temperatures || {}).map(([name,t]) => `${name}: ${displayTemperature(t.temperature,t.target)}`);
    byId("preflight-progress-detail").textContent = [body.message, ...readings].filter(Boolean).join(" · ");
    byId("step-klipper").textContent = (body.checks || []).some(c => c.name === "Heaters off") ? "Selected checks complete; heater targets off for robot preflight" : "Check selected heaters, then switch all off";
    byId("step-robot").textContent = body.stage === "robot" ? "Validating trajectory…" : (body.checks || []).some(c => c.participant === "robot") ? "Timed trajectory validated" : "Validate robot motion and timing";
    if (body.busy) {
      setBadge("job-badge", body.state, true);
      byId("job-name").textContent = body.job;
      byId("job-mode").textContent = body.stage;
    }
    await loadPreflight();
  } catch (error) {
    syncSnapshot.control_ready = false;
    syncSnapshot.robot_ready = false;
    showMessage("sync-message", `Could not refresh sync state: ${error.message}`, true);
    updateSyncControls();
  } finally {
    syncRefreshInFlight = false;
  }
}
window.setInterval(refreshSync, 1000);

byId("upload-form").addEventListener("submit", async (event) => {
  event.preventDefault();
  const file = byId("gcode-upload").files[0];
  if (!file) return showMessage("upload-message", "Choose a .gcode file.", true);
  const button = byId("upload-job");
  button.disabled = true;
  showMessage("upload-message", "Uploading to Moonraker and staging for AxisForge…");
  const form = new FormData();
  form.append("file", file);
  try {
    const response = await fetch("/api/jobs/upload", { method: "POST", body: form });
    const body = await response.json().catch(() => ({}));
    if (!response.ok) throw new Error(body.detail || `Upload failed (${response.status})`);
    byId("gcode-upload").value = "";
    showMessage("upload-message", `${body.job} uploaded. Run the Klipper and robot preflights before starting.`);
    await loadJobs();
    byId("job-select").value = body.job;
    await loadPreflight();
  } catch (error) {
    showMessage("upload-message", error.message, true);
  } finally {
    updateSyncControls();
  }
});

byId("stop-job").addEventListener("click", async () => {
  if (!window.confirm("Request AxisForge to stop the robot job?")) return;
  showMessage("job-message", "");
  try {
    const result = await request("/api/axisforge/stop", { method: "POST" });
    showMessage("job-message", result.message, result.running);
    refreshStatus();
  } catch (error) {
    showMessage("job-message", error.message, true);
  }
});

byId("klipper-estop").addEventListener("click", async () => {
  if (!window.confirm("Request emergency stop of Klipper and AxisForge? Use physical E-stops if needed.")) return;
  showMessage("safety-message", "");
  try {
    await request("/api/klipper/emergency-stop", { method: "POST" });
    showMessage("safety-message", "Stop requests accepted by Klipper and AxisForge.");
    refreshStatus();
  } catch (error) {
    showMessage("safety-message", error.message, true);
  }
});

loadJobs().then(loadPreflight).then(refreshSync);
loadLinks();
refreshStatus();
refreshTelemetry();
refreshBedProbe();
window.setInterval(refreshBedProbe, 1000);
window.setInterval(refreshStatus, 2000);
window.setInterval(refreshTelemetry, 5000);
window.addEventListener("resize", drawTemperatureHistory);

function updateSetupControls() {
  updateTcpControls();
  const c = robotConnectionSnapshot, load = loadCalibrationSnapshot;
  const otherBusy = syncSnapshot.busy || bedProbeSnapshot.busy || bedProbeActionInFlight || syncActionInFlight || !latestAxisForgeIdle;
  byId("robot-connect").disabled = !c.available || !c.configured || Boolean(otherBusy || setupBusy());
  byId("robot-connection-message").textContent = c.message || "Checking PC robot connection service…";
  setBadge("robot-connection-badge", c.ready ? "Ready" : (c.state || "Checking"), c.ready ? true : undefined);
  const cfg = c.configuration;
  byId("robot-configuration").textContent = cfg ? `Robot: ${cfg.robot_type} at ${cfg.robot_ip} · AxisForge nozzle offset: ${cfg.axisforge_tcp_mm?.join(", ") || "unavailable"} mm · These saved values still need to match the installed tool.` : "";
  byId("robot-readiness").replaceChildren(...(c.checks || []).map(check => {
    const item = document.createElement("li");
    item.textContent = `${check.status === "pass" ? "✓" : "○"} ${check.name}: ${check.detail}`;
    return item;
  }));
  const occupied = Boolean(otherBusy || setupBusy());
  const unavailable = !loadCalibrationAvailable || !load.configured;
  const reason = unavailable ? "Scale routine service is unavailable." : occupied ? "Finish or stop the current operation first." :
    !latestSystemsReady ? latestSystemsReason : !syncSnapshot.robot_ready ? (syncSnapshot.robot_reason || "Complete the robot checks first.") : "";
  byId("load-calibration-start").disabled = Boolean(reason);
  byId("load-calibration-simulate").disabled = unavailable || occupied;
  byId("load-calibration-abort").disabled = !load.busy || loadCalibrationInFlight;
  byId("load-calibration-readiness").textContent = load.busy ? "Keep heater targets off and the nozzle cool. Stop cancels motion and holds position." : reason || "Ready. Motion starts only after the home and scale confirmations.";
  setBadge("load-calibration-badge", `${load.dry_run ? "Simulation · " : ""}${load.state.replaceAll("_", " ")}`, ["failed","aborted"].includes(load.state) ? false : undefined);
  byId("load-calibration-message").textContent = load.message || "";
  byId("load-calibration-sensor").textContent = Number.isFinite(load.trigger_value) ? `Contact: ${load.trigger_value.toFixed(0)} raw counts · Stop limit: ${load.max_value} counts. Live counts are refreshed after each move and while waiting.` : "Contact uses uncalibrated raw counts; no grams conversion is assumed.";
  const confirmation = load.confirmation;
  const requested = loadCalibrationAvailable && load.busy && confirmation && !consumedCalibrationTokens.has(confirmation.token);
  if (requested && calibrationVisibleToken !== confirmation.token) {
    calibrationVisibleToken = confirmation.token; calibrationReadyAt = performance.now()+750;
    window.setTimeout(updateSetupControls, 760);
  }
  byId("load-calibration-confirmation").hidden = !requested;
  byId("load-calibration-confirm").disabled = !requested || loadCalibrationInFlight || performance.now() < calibrationReadyAt;
  if (requested) {
    const titles = {home:"Confirm the arm is at home", scale:"Confirm the scale is under the nozzle", tare:"Confirm the weighing scale is tared"};
    const labels = {home:"Home position reached — continue", scale:"Scale placed — begin slow Z contact", tare:"Scale tared — enable Z baby steps"};
    byId("load-calibration-confirm-title").textContent = titles[confirmation.stage];
    byId("load-calibration-confirm-detail").textContent = `${load.dry_run ? "Simulation only. " : ""}${load.message}`;
    byId("load-calibration-confirm").textContent = labels[confirmation.stage];
  }
  byId("load-calibration-steps").hidden = !load.busy || !["adjusting_load","stepping","step_rejected"].includes(load.state);
  const enabled = loadCalibrationAvailable && load.busy && !loadCalibrationInFlight && load.state === "adjusting_load" && load.action_token && !consumedCalibrationTokens.has(load.action_token);
  for (const id of ["load-step-up","load-step-down","load-calibration-finish"]) byId(id).disabled = !enabled;
  byId("load-calibration-position").textContent = Array.isArray(load.position_mm) ? `Nozzle Z: ${load.position_mm[2].toFixed(3)} mm · Allowed Z: ${load.lower_z_mm?.toFixed(3)} to ${load.upper_z_mm?.toFixed(3)} mm · Raw delta: ${load.delta_counts?.toFixed(0) || "—"} counts` : "";
}

async function refreshSetup() {
  if (setupRefreshInFlight || loadCalibrationInFlight || robotConnectInFlight) return;
  setupRefreshInFlight = true;
  const version = setupRequestVersion;
  try {
    const results = await Promise.allSettled([request("/api/robot/connection"), request("/api/load-calibration/status")]);
    if (version !== setupRequestVersion) return;
    if (results[0].status === "fulfilled") robotConnectionSnapshot = results[0].value;
    else robotConnectionSnapshot = {...robotConnectionSnapshot, available:false, message:"PC connection service unavailable."};
    if (results[1].status === "fulfilled") {
      const finished = loadCalibrationSnapshot.busy && !results[1].value.busy;
      loadCalibrationSnapshot = results[1].value; loadCalibrationAvailable = true;
      if (uncertainCalibrationToken) {
        if (loadCalibrationSnapshot.confirmation?.token === uncertainCalibrationToken || loadCalibrationSnapshot.action_token === uncertainCalibrationToken) consumedCalibrationTokens.delete(uncertainCalibrationToken);
        uncertainCalibrationToken = null;
      }
      if (finished) { selectedPreflightState = null; loadPreflight(); refreshSync(); }
    } else {
      loadCalibrationAvailable = false;
      loadCalibrationSnapshot.message = "Cannot check calibration status. Controls disabled; use the physical stop if needed.";
    }
    updateSyncControls();
  } finally { setupRefreshInFlight = false; }
}

async function calibrationAction(action, data={}) {
  if (loadCalibrationInFlight) return;
  loadCalibrationInFlight = true; setupRequestVersion++;
  if (data.token) consumedCalibrationTokens.add(data.token);
  if (action === "start") selectedPreflightState = null;
  updateSyncControls();
  try {
    loadCalibrationSnapshot = await request(`/api/load-calibration/${action}`, {method:"POST",body:JSON.stringify(data)});
    loadCalibrationAvailable = true;
  } catch (error) {
    if (data.token) uncertainCalibrationToken = data.token;
    loadCalibrationSnapshot.message = error.message;
  } finally { loadCalibrationInFlight = false; updateSyncControls(); refreshSetup(); }
}
byId("robot-connect").addEventListener("click", async () => {
  if (robotConnectInFlight) return;
  robotConnectInFlight = true; setupRequestVersion++; selectedPreflightState = null; updateSyncControls();
  try { robotConnectionSnapshot = await request("/api/robot/connect", {method:"POST",body:"{}"}); }
  catch (error) { robotConnectionSnapshot.message = error.message; }
  finally { robotConnectInFlight = false; updateSyncControls(); refreshSetup(); }
});
byId("load-calibration-start").addEventListener("click", () => calibrationAction("start", {dry_run:false}));
byId("load-calibration-simulate").addEventListener("click", () => calibrationAction("start", {dry_run:true}));
byId("load-calibration-abort").addEventListener("click", () => calibrationAction("abort"));
byId("load-calibration-confirm").addEventListener("click", () => {
  const value = loadCalibrationSnapshot.confirmation;
  if (value && !consumedCalibrationTokens.has(value.token)) calibrationAction("confirm", value);
});
for (const [id, direction] of [["load-step-up",1],["load-step-down",-1]]) {
  byId(id).addEventListener("click", () => {
    const token = loadCalibrationSnapshot.action_token;
    if (token && !consumedCalibrationTokens.has(token)) calibrationAction("action", {command:"step", token, delta_mm:direction*Number(byId("load-step-size").value)});
  });
}
byId("load-calibration-finish").addEventListener("click", () => {
  const token = loadCalibrationSnapshot.action_token;
  if (token && !consumedCalibrationTokens.has(token)) calibrationAction("action", {command:"finish", token});
});
byId("calibration-open-logger").addEventListener("click", () => byId("alps-link").click());
refreshSetup();
window.setInterval(refreshSetup, 1000);

function tcpValues() {
  return ["tcp-x","tcp-y","tcp-z"].map(id => byId(id).value.trim() === "" ? NaN : Number(byId(id).value));
}

function updateTcpControls() {
  const occupied = Boolean(setupBusy() || syncSnapshot.busy || syncActionInFlight || bedProbeSnapshot.busy || bedProbeActionInFlight || !latestAxisForgeIdle);
  const valid = tcpValues().every(value => Number.isFinite(value) && Math.abs(value) <= 1000);
  byId("tcp-fields").disabled = occupied || !tcpAvailable;
  byId("tcp-save").disabled = occupied || !tcpAvailable || !valid || !tcpDirty || tcpConflict;
  byId("tcp-reload").disabled = tcpSaveInFlight || tcpRefreshInFlight;
  setBadge("tcp-badge", tcpSaveInFlight ? "Saving…" : !tcpAvailable ? "Unavailable" : occupied ? "Operation active" : tcpConflict ? "Reload required" : tcpDirty ? "Unsaved changes" : "Saved", tcpConflict ? false : undefined);
  byId("tcp-saved").textContent = tcpSnapshot ? `Loaded X / Y / Z: ${tcpSnapshot.tcp_offset_mm.join(" / ")} mm` : "Reading the saved AxisForge nozzle offset…";
  byId("tcp-bed-mesh").textContent = tcpSnapshot?.bed_mesh?.message || "";
}

function acceptTcpSettings(body, replaceInputs=false) {
  const revisionChanged = tcpSnapshot && tcpSnapshot.revision !== body.revision;
  if (replaceInputs || !tcpDirty) {
    ["tcp-x","tcp-y","tcp-z"].forEach((id,index) => { byId(id).value = body.tcp_offset_mm[index]; });
    tcpDirty = false; tcpConflict = false;
    tcpSnapshot = body;
  } else if (revisionChanged) {
    tcpConflict = true;
    showMessage("tcp-message", "Saved configuration changed while you were editing. Reload saved values, then review your offsets.", true);
  } else {
    tcpSnapshot = body;
  }
  tcpAvailable = true;
  updateTcpControls();
}

async function refreshTcp(replaceInputs=false) {
  if (tcpRefreshInFlight || tcpSaveInFlight) return;
  tcpRefreshInFlight = true;
  const version = tcpRequestVersion;
  updateTcpControls();
  try {
    const body = await request("/api/setup/tcp");
    if (version !== tcpRequestVersion) return;
    acceptTcpSettings(body, replaceInputs);
    if (replaceInputs) showMessage("tcp-message", "Loaded the saved nozzle TCP.");
  } catch (error) {
    if (version !== tcpRequestVersion) return;
    tcpAvailable = false;
    showMessage("tcp-message", error.message, true);
  } finally { tcpRefreshInFlight = false; updateTcpControls(); }
}

for (const id of ["tcp-x","tcp-y","tcp-z"]) byId(id).addEventListener("input", () => {
  tcpDirty = !tcpSnapshot || tcpValues().some((value,index) => value !== tcpSnapshot.tcp_offset_mm[index]);
  updateTcpControls();
});
byId("tcp-reload").addEventListener("click", () => refreshTcp(true));
byId("tcp-form").addEventListener("submit", async event => {
  event.preventDefault();
  if (byId("tcp-save").disabled || !byId("tcp-form").reportValidity()) return;
  const [x_mm,y_mm,z_mm] = tcpValues();
  tcpSaveInFlight = true; tcpRequestVersion++;
  updateSyncControls();
  try {
    const result = await request("/api/setup/tcp", {method:"POST", body:JSON.stringify({x_mm,y_mm,z_mm,revision:tcpSnapshot.revision})});
    acceptTcpSettings(result, true);
    showMessage("tcp-message", result.message);
    if (result.changed) {
      selectedPreflightState = null;
      loadPreflight(); refreshSync(); refreshSetup();
    }
  } catch (error) {
    // A lost response may follow a committed save. Require an explicit reload
    // before retrying, so the displayed version always describes the real file.
    tcpConflict = true;
    showMessage("tcp-message", `${error.message} Reload saved values before retrying.`, true);
  } finally { tcpSaveInFlight = false; updateSyncControls(); }
});
refreshTcp();
window.setInterval(refreshTcp, 5000);
