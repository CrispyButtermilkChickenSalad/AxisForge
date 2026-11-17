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
let latestKlipperTemperatures = {};
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
let probingSnapshot = null, probingAvailable = false, probingSaveInFlight = false, probingRefreshInFlight = false;
let probingDirty = false, probingConflict = false, probingRequestVersion = 0, probingEditVersion = 0;
let bedSettingsSnapshot = null, bedSettingsAvailable = false, bedSettingsSaveInFlight = false, bedSettingsRefreshInFlight = false;
let bedSettingsDirty = false, bedSettingsConflict = false, bedSettingsRequestVersion = 0;
let jobTransferInFlight = false;
let setupRequestVersion = 0;
let calibrationVisibleToken = null, calibrationReadyAt = 0, uncertainCalibrationToken = null;
const consumedCalibrationTokens = new Set();
function setupBusy() { return tcpSaveInFlight || probingSaveInFlight || bedSettingsSaveInFlight || jobTransferInFlight || loadCalibrationSnapshot.busy || loadCalibrationInFlight || robotConnectionSnapshot.busy || robotConnectInFlight; }

let bedProbeSnapshot = {state:"idle", busy:false, confirmation:null};
let bedProbeAvailable = false;
let bedProbeActionInFlight = false;
let bedProbeRefreshInFlight = false;
let bedProbeRequestVersion = 0;
const consumedProbeConfirmations = new Set();
let visibleProbeConfirmation = null;
let probeConfirmationReadyAt = 0;
let uncertainProbeConfirmation = null;
let uncertainProbeActionToken = null;

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
  if (!response.ok) {
    const detail = Array.isArray(body.detail) ? body.detail.map(error =>
      [error.loc?.filter(part => part !== "body").join("."), error.msg].filter(Boolean).join(": ")
    ).join("; ") : body.detail;
    throw new Error(detail || `Request failed (${response.status})`);
  }
  return body;
}

async function refreshStatus() {
  try {
    const status = await request("/api/status");
    const ros = status.ros;
    const klipper = status.klipper;
    latestKlipperTemperatures = {extruder:klipper.extruder, heater_bed:klipper.bed};
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
  const tracking = syncSnapshot.tracking;
  byId("print-tracking").textContent = tracking?.samples ?
    `Measured robot tracking: maximum nozzle error ${Number(tracking.max_position_error_mm).toFixed(3)} mm · maximum Z error ${Number(tracking.max_z_error_mm).toFixed(3)} mm${tracking.mean_estimated_lag_ms == null ? "" : ` · mean estimated motion lag ${Number(tracking.mean_estimated_lag_ms).toFixed(1)} ms`}. These compare joint feedback with the commanded path; they are separate from network clock alignment. They do not measure filament flow or nozzle deflection.` :
    "Physical nozzle tracking will be measured during the next synchronized print. Clock alignment alone does not measure robot response delay.";
  byId("clock-measurement").textContent = measurement
    ? `Last clock check (${syncSnapshot.job || "job"}, ${measurement.age_s.toFixed(1)} s ago): uncertainty ${measurement.uncertainty_ms.toFixed(2)} ms${measurement.alignment_error_ms == null ? "" : `; running alignment ${measurement.alignment_error_ms.toFixed(2)} ms`}${Number.isFinite(measurement.print_clock_rate_ppm) ? `; print clock rate ${measurement.print_clock_rate_ppm.toFixed(1)} ppm` : ""}${measurement.rate_source === "measured_between_hosts" ? ` (measured between hosts over ${measurement.host_rate_span_s.toFixed(0)} s)` : ""}. Recorded limits: ${syncSnapshot.clock_options.max_uncertainty_ms}/${syncSnapshot.clock_options.max_alignment_error_ms} ms.`
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
  if (bedSettingsDirty || bedSettingsConflict) return "Save or reload the bed dimensions before preflighting or starting a job.";
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
  byId("copy-job-klipper").disabled = busy || syncActionInFlight || !job || !latestAxisForgeIdle;
  byId("preflight-both").disabled = busy || syncActionInFlight || bedSettingsDirty || bedSettingsConflict || !heaterOptionsInitialized || !motionOptionsInitialized || !motionOptionsValid() || !clockOptionsInitialized || !clockOptionsValid() || !job || !latestSystemsReady || !latestAxisForgeIdle || !syncSnapshot.control_ready;
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

function zStage(body) {
  return ["positioning_z", "recovering_z", "z_stepping", "z_step_rejected"].includes(body.state);
}

function zControlsReason(body, available, inFlight, consumed) {
  if (!available || !body.configured) return "Z controls are unavailable because the routine status cannot be checked.";
  if (!body.busy || body.stop_unconfirmed) return "This routine has stopped. Z controls require an active session with a confirmed stop and valid ALPS feedback.";
  if (inFlight) return "Waiting for the current request; another Z step cannot be queued.";
  if (!["positioning_z", "recovering_z"].includes(body.state)) return "Wait for the current movement or check to finish before choosing a Z step.";
  const z = body.z_controls;
  if (!z || z.phase !== (body.state === "positioning_z" ? "positioning" : "recovery") ||
      !Array.isArray(z.position_mm) || z.position_mm.length !== 3 || !z.position_mm.every(Number.isFinite) ||
      ![z.lower_z_mm, z.upper_z_mm, z.delta_counts].every(Number.isFinite) || z.lower_z_mm > z.upper_z_mm ||
      ![z.loaded, z.upward_only, z.can_continue, z.can_finish].every(value => typeof value === "boolean") ||
      !Array.isArray(z.step_sizes) || !z.step_sizes.length || !z.step_sizes.every(value => Number.isFinite(value) && value > 0)) {
    return "Waiting for valid Z travel limits and load feedback.";
  }
  if (!body.action_token || consumed.has(body.action_token)) return "Waiting for fresh feedback after the last action before enabling another Z step.";
  if (z.release_blocked) return "Up step stopped: ALPS load increased. Waiting for a fresh stable window below the original release ceiling. If this persists, stop and inspect the toolhead/contact direction.";
  return "";
}

function zStepReason(body, delta) {
  const z = body.z_controls;
  if (!z || !Number.isFinite(delta) || !z.step_sizes?.includes(Math.abs(delta))) return "Choose a supported Z step size.";
  if ((z.phase === "recovery" || z.upward_only || z.loaded) && delta < 0) return "Only upward release is available; Down is disabled.";
  if (z.loaded && delta > .1) return "Load is applied. Use Up 0.1 mm or finer until fresh readings confirm release.";
  const end = z.position_mm?.[2] + delta;
  if (!Number.isFinite(end) || end < z.lower_z_mm - 1e-8 || end > z.upper_z_mm + 1e-8) return "This step exceeds the allowed Z range; choose a smaller step.";
  return "";
}

function zStatusText(body) {
  const z = body.z_controls;
  if (z.release_blocked) return "Recovery is still active, but further Z steps are blocked until the load guard clears. No step repeats automatically.";
  if (z.phase === "recovery") return `${z.failure ? `${z.failure} ` : ""}Upward recovery is available. ${z.loaded ? "Use Up 0.1 mm or finer to release contact." : "The nozzle is unloaded; lift clear, then finish recovery and start a fresh attempt."}`;
  return z.loaded ? "Contact detected. Use Up 0.1 mm or finer to unload before continuing." :
    "Z steps are available. Position the unloaded nozzle, then Continue to start the automatic search. Downward steps stop on contact.";
}

function zPositionText(body) {
  const z = body.z_controls;
  return z && Array.isArray(z.position_mm) && [z.position_mm[2], z.lower_z_mm, z.upper_z_mm, z.delta_counts].every(Number.isFinite) ?
    `Nozzle Z: ${z.position_mm[2].toFixed(3)} mm · Allowed Z: ${z.lower_z_mm.toFixed(3)} to ${z.upper_z_mm.toFixed(3)} mm · Raw delta: ${z.delta_counts.toFixed(0)} counts` : "";
}

function probeThermalOptions() {
  if (!byId("probe-heated").checked) return null;
  return {nozzle_target_c:Number(byId("probe-nozzle-target").value), bed_target_c:Number(byId("probe-bed-target").value), soak_s:Number(byId("probe-soak").value)};
}

function probeThermalValid() {
  const t=probeThermalOptions();
  return t===null || [t.nozzle_target_c,t.bed_target_c,t.soak_s].every(Number.isFinite) &&
    (t.nozzle_target_c===0 || t.nozzle_target_c>=50 && t.nozzle_target_c<=300) &&
    (t.bed_target_c===0 || t.bed_target_c>=30 && t.bed_target_c<=150) && t.soak_s>=5 && t.soak_s<=1800 &&
    t.nozzle_target_c+t.bed_target_c>0;
}

function probeStartPayload(dryRun) {
  const thermal=probeThermalOptions();
  return {dry_run:dryRun,...(thermal ? {thermal} : {})};
}

function updateBedProbeControls() {
  const body = bedProbeSnapshot;
  const busy = Boolean(body.busy);
  const occupied = busy || body.stop_unconfirmed || setupBusy() || bedProbeActionInFlight || syncActionInFlight || syncSnapshot.busy || !latestAxisForgeIdle;
  if (busy) {
    const selected=body.thermal_options;
    byId("probe-heated").checked=Boolean(selected && (selected.nozzle_target_c>0 || selected.bed_target_c>0));
    if (selected) {
      byId("probe-nozzle-target").value=selected.nozzle_target_c;
      byId("probe-bed-target").value=selected.bed_target_c;
      byId("probe-soak").value=selected.soak_s;
    }
  }
  const thermal=probeThermalOptions();
  const noBed=Array.isArray(syncSnapshot.configured_heaters) && !syncSnapshot.configured_heaters.includes("heater_bed");
  byId("probe-thermal-fields").disabled=occupied;
  byId("probe-thermal-inputs").hidden=!byId("probe-heated").checked;
  byId("probe-bed-target").disabled=occupied;
  byId("probe-copy-temperatures").disabled=occupied || !latestSystemsReady;
  const liveReason = !bedProbeAvailable ? "Bed-probe service is unavailable." :
    !body.configured ? "Configure AxisForge before homing." :
    body.stop_unconfirmed ? "Robot stop is unconfirmed. Use the pendant to stop the arm and restore a verified stopped state before retrying." :
    occupied ? "Finish or stop the current operation before starting another routine." :
    bedSettingsDirty || bedSettingsConflict ? "Save or reload the bed dimensions before starting the routine." :
    probingDirty || probingConflict ? "Save or reload the homing settings before starting the routine." :
    !probeThermalValid() ? "Enter nozzle 50–300°C, bed 30–150°C (0 means off), and a 5–1800 second soak." :
    noBed && thermal?.bed_target_c>0 ? "Klipper has no configured bed heater. Set the probing bed target to 0." :
    robotConnectionSnapshot.available === true && robotConnectionSnapshot.ready === false ?
      (robotConnectionSnapshot.message || "Complete the robot connection checks before retrying.") :
    !latestSystemsReady ? latestSystemsReason :
    !syncSnapshot.robot_ready ? (syncSnapshot.robot_reason || "Check the robot connection and External Control program.") : "";
  byId("bed-probe-start").disabled = Boolean(liveReason);
  byId("bed-probe-start").textContent=body.saved_mesh?.available || body.state==="complete" && !body.dry_run ? "Redo homing & mesh" : "Home & probe bed";
  const saved=body.saved_mesh;
  const savedThermal=saved?.temperature;
  const savedTemperatures=savedThermal?.options ? `Nozzle ${savedThermal.options.nozzle_target_c}°C · bed ${savedThermal.options.bed_target_c}°C · ${savedThermal.mode}.` : "Temperature was not recorded for this mesh.";
  byId("bed-probe-saved-mesh").textContent=saved?.available ? `Saved mesh: ${saved.points} points · ${saved.stamp}. ${savedTemperatures} You can redo it while the robot is ready and idle. The new completed scan replaces it and keeps a backup; failed runs leave it unchanged.` : "A completed scan creates a new active mesh. You can repeat homing and meshing whenever needed.";
  const temperatures=Object.entries(latestKlipperTemperatures).filter(([,v])=>Number.isFinite(v?.temperature)).map(([name,v])=>`${name === "extruder" ? "Nozzle" : "Bed"}: ${displayTemperature(v.temperature,v.target)}`).join(" · ");
  const soaking=body.state==="waiting_temperature" && body.thermal ? ` · Stable ${Math.floor(body.thermal.stable_for_s || 0)}/${body.thermal.required_stable_s} s` : "";
  byId("probe-thermal-status").textContent=temperatures+soaking+(noBed ? " · Bed heater not configured; use target 0." : "");
  byId("bed-probe-simulate").disabled = !bedProbeAvailable || !body.configured || occupied || !probeThermalValid() || probingDirty || probingConflict || bedSettingsDirty || bedSettingsConflict;
  const retryVisible = ["failed", "aborted"].includes(body.state);
  const retry = byId("bed-probe-retry");
  byId("bed-probe-recovery").hidden = !retryVisible;
  retry.textContent = body.dry_run ? "Retry simulation" : "Retry homing";
  retry.disabled = !retryVisible || Boolean(body.stop_unconfirmed) ||
    (body.dry_run ? byId("bed-probe-simulate").disabled : byId("bed-probe-start").disabled);
  byId("bed-probe-retry-guidance").textContent = body.dry_run ?
    "Retry starts a fresh simulation with new confirmations. No hardware will move and the physical bed map stays unchanged." :
    "Correct the reported fault and leave the nozzle clear of the surface and unloaded. Review the maximum Z search for the current position. Retry starts fresh checks and asks for surface confirmation again; it does not lift the arm or resume the interrupted movement.";
  byId("bed-probe-retry-status").textContent = retry.disabled ?
    (body.dry_run && !body.stop_unconfirmed ? "Finish the current operation and save any setup changes; the homing service must be available before retrying." : liveReason) :
    "Ready to start a fresh attempt with the saved settings.";
  byId("bed-probe-abort").disabled = !busy || bedProbeActionInFlight;
  const heated=body.thermal?.mode==="heated" || body.thermal_options && (body.thermal_options.nozzle_target_c>0 || body.thermal_options.bed_target_c>0);
  byId("bed-probe-readiness").textContent = busy ? `${heated ? "Maintain the selected Klipper heater targets throughout this run." : "Heater targets must stay off."} Stop probing cancels motion without returning the arm.` :
    liveReason || "Ready. The first click checks readiness; motion waits for your confirmation below.";
  const labels = {idle:"Not run",starting:"Checking",waiting_temperature:"Temperature soak",aligning_nozzle:"Aligning nozzle",checking_robot_limits:"Checking robot limits",waiting_surface:"Confirm surface",positioning_z:"Position nozzle",recovering_z:"Lift clear",z_stepping:"Moving Z",z_step_rejected:"Z step rejected",homing_fast:"Finding bed",homing_slow:"Fine contact",moving_front_left:"Moving to first point",waiting_front_left:"Confirm front-left",meshing:"Measuring mesh",returning:"Returning",recovering_home:"Returning Home after failure",complete:body.dry_run ? "Simulation complete" : "Mesh complete",failed:"Failed",aborted:"Stopped",stopping:"Stopping"};
  const label = labels[body.state] || body.state;
  setBadge("bed-probe-badge", label, body.state === "complete" && !body.dry_run ? true : ["failed","aborted"].includes(body.state) ? false : undefined);
  byId("step-home").textContent = body.state === "idle" ? "Locate the surface and measure its mesh" : `${body.dry_run ? "Simulation · " : ""}${label}`;
  byId("bed-probe-message").textContent = `${body.dry_run ? "Simulation: " : ""}${body.message || ""}`;
  byId("bed-probe-message").className = `inline-message${["failed","aborted"].includes(body.state) ? " error" : ""}`;
  const alignment = body.alignment;
  byId("bed-probe-alignment").textContent = alignment?.state === "aligned" ?
    `Upright alignment complete: ${Number(alignment.angle_deg).toFixed(3)}° corrected; remaining error ${Number(alignment.residual_deg).toFixed(3)}°. The mesh returns to the corrected starting pose.` : "";
  const issues = Array.isArray(body.failure_history) ? body.failure_history : body.last_failure ? [body.last_failure] : [];
  byId("bed-probe-last-error").hidden = !issues.length;
  byId("bed-probe-last-error").textContent = issues.length ? `Recent reported issues: ${issues.slice(-3).join(" · ")}` : "";
  const recovery = body.failure_recovery;
  byId("bed-probe-home-recovery").textContent = recovery?.state === "returned" ?
    "The scan failed and the arm returned to its established Home. The previous valid mesh is unchanged. Retry homing starts a fresh scan." : recovery?.state === "blocked" ?
    `Automatic Home return could not finish: ${recovery.reason}. The scan remains failed; follow the reported recovery instructions.` : recovery ?
    "The scan failed. Automatic recovery is releasing contact and returning Home under the existing sensor and robot guards. Stop probing cancels recovery too." : "";
  const threshold = Number(body.trigger_value);
  const maximum = Number(body.max_value);
  const unit = body.trigger_unit || (body.trigger_mode === "raw" ? "counts" : "g");
  const nativeTrigger = body.trigger_source === "klipper_probe";
  const nativeForce = Number(body.trigger_force_g);
  byId("bed-probe-sensor").textContent = nativeTrigger ?
    `Native Klipper probe: ${Number.isFinite(nativeForce) && nativeForce > 0 ? `${nativeForce.toLocaleString()} g contact threshold` : "contact threshold unavailable; check native probe calibration"}${Number.isFinite(maximum) && maximum > 0 ? ` · independent ${unit === "counts" ? "raw " : ""}overload stop ${maximum.toLocaleString()} ${unit}` : ""}. Requires load-cell calibration; the ALPS stream still monitors freshness and overload.` : Number.isFinite(threshold) ?
    `${body.trigger_mode === "raw" ? "Raw sensor mode" : "Calibrated sensor mode"}: ${threshold.toLocaleString()} ${unit} contact threshold${Number.isFinite(maximum) ? ` · ${maximum.toLocaleString()} ${unit} stop threshold` : ""}${body.trigger_mode === "raw" ? ". Uses the unloaded starting baseline; no conversion to grams." : "."}` : "";
  if (Number.isFinite(threshold) && Number.isFinite(body.baseline_noise_fraction)) {
    byId("bed-probe-sensor").textContent += ` Unloaded sample-range limit: ${(threshold*body.baseline_noise_fraction).toLocaleString()} ${unit}.`;
  }
  byId("bed-probe-progress-container").hidden = !busy && body.state !== "complete";
  byId("bed-probe-progress").value = body.progress || 0;
  const plannedPoints = body.total_points || (probingSnapshot ? probingSnapshot.settings.mesh_columns * probingSnapshot.settings.mesh_rows : "—");
  byId("bed-probe-progress-label").textContent = `${label} · ${body.current_point || 0}/${plannedPoints} mesh points`;
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
    const needsAlignment = first && alignment?.required && alignment.state === "pending";
    const searchLimit = Number(body.max_descent_mm);
    byId("bed-probe-confirm-title").textContent = first ? needsAlignment ? "Confirm surface and upright alignment" : "Confirm the surface before descending" : "Confirm the front-left mesh point";
    byId("bed-probe-confirm-detail").textContent = `${body.dry_run ? "This is a simulation; no hardware will move. " : ""}${first ?
      `Confirm the nozzle is above the CENTRE of the flat bed. Its starting X/Y defines the mesh centre. The nozzle must be clean, ${heated ? "at the selected temperature," : "cool,"} clear of the surface and unloaded.${heated ? " The bed must tolerate the selected nozzle temperature." : ""}${needsAlignment ? ` Leave space around the toolhead for rotation. Confirmation authorizes a ${Number(alignment.angle_deg).toFixed(3)}° upright correction around the nozzle tip over at least ${Number(alignment.duration_s)} seconds (maximum ${Number(alignment.maximum_deg)}°). The tip is intended to stay in place. Z positioning enables after alignment is verified.` : " Confirmation enables Z positioning."} Choose Continue afterward to start the automatic search.${Number.isFinite(searchLimit) && searchLimit > 0 ? ` Maximum downward search from the adjusted position: ${searchLimit} mm.` : ""}` :
      "Confirm the nozzle is above the intended first mesh point and this is the front-leftmost position of the mesh. Confirmation starts the selected mesh scan."}`;
    byId("bed-probe-confirm").textContent = first ? needsAlignment ? "Surface clear — align nozzle" : "Surface confirmed — enable Z positioning" : "Front-left confirmed — probe mesh";
    const position = body.position_mm;
    const grid = body.first_point;
    const positionText = Array.isArray(position) && position.length === 3 ?
      `Nozzle position in robot Base: X ${Number(position[0]).toFixed(2)}, Y ${Number(position[1]).toFixed(2)}, Z ${Number(position[2]).toFixed(2)} mm.` : "";
    const gridText = !first && Array.isArray(grid) && grid.length === 2 ?
      ` Mesh point in G-code: X ${Number(grid[0]).toFixed(2)}, Y ${Number(grid[1]).toFixed(2)} mm.` : "";
    const offset = body.first_point_offset_mm;
    const offsetText = !first && Array.isArray(offset) && offset.length === 2 ?
      ` Offset from homed centre in Base: X ${Number(offset[0]).toFixed(2)}, Y ${Number(offset[1]).toFixed(2)} mm.` : "";
    byId("bed-probe-position").textContent = positionText + gridText + offsetText;
  }
  const zReason = zControlsReason(body, bedProbeAvailable, bedProbeActionInFlight, consumedProbeConfirmations);
  byId("bed-z-status").textContent = zStage(body) ? zReason || zStatusText(body) :
    busy && alignment?.state === "aligning" ? "Aligning upright; Z controls enable after the stopped pose and unloaded sensor are verified." :
    ["failed", "aborted"].includes(body.state) ? "This attempt has stopped; its Z controls are closed. Dashboard release is offered only while a recoverable failure keeps the monitored session active." :
    busy ? "Z positioning enables after the unloaded surface confirmation. Automatic homing and mesh motion must finish or stop before manual adjustment." :
    "Start homing and confirm the unloaded surface to enable Z positioning.";
  byId("bed-z-position").textContent = zStage(body) ? zPositionText(body) : "";
  for (const button of document.querySelectorAll("[data-bed-z-step]")) {
    const reason = zReason || zStepReason(body, Number(button.dataset.bedZStep));
    button.disabled = Boolean(reason);
    button.title = reason || "One click requests one Z move";
  }
  const z = zStage(body) ? body.z_controls : null;
  byId("bed-z-continue").hidden = z?.phase !== "positioning";
  byId("bed-z-continue").disabled = Boolean(zReason) || z?.can_continue !== true || z?.loaded !== false;
  byId("bed-z-finish").hidden = z?.phase !== "recovery";
  byId("bed-z-finish").disabled = Boolean(zReason) || z?.can_finish !== true || z?.loaded !== false;
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
    if (uncertainProbeActionToken) {
      if (result.action_token === uncertainProbeActionToken) consumedProbeConfirmations.delete(uncertainProbeActionToken);
      uncertainProbeActionToken = null;
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
  if (action === "start") { selectedPreflightState = null; uncertainProbeConfirmation = null; uncertainProbeActionToken = null; }
  if (action === "action" && data.token) consumedProbeConfirmations.add(data.token);
  updateSyncControls();
  try {
    bedProbeSnapshot = await request(`/api/bed-probe/${action}`, {method:"POST",body:JSON.stringify(data)});
    bedProbeAvailable = true;
  } catch (error) {
    if (action === "confirm") uncertainProbeConfirmation = data;
    if (action === "action" && data.token) uncertainProbeActionToken = data.token;
    bedProbeSnapshot.message = error.message;
    showMessage("bed-probe-message",error.message,true);
  } finally {
    bedProbeActionInFlight = false;
    updateSyncControls();
    refreshBedProbe();
    refreshSync();
  }
}

byId("bed-probe-start").addEventListener("click", () => bedProbeAction("start", probeStartPayload(false)));
byId("bed-probe-simulate").addEventListener("click", () => bedProbeAction("start", probeStartPayload(true)));
byId("bed-probe-retry").addEventListener("click", () => {
  if (byId("bed-probe-retry").disabled || !["failed", "aborted"].includes(bedProbeSnapshot.state)) return;
  bedProbeAction("start", probeStartPayload(bedProbeSnapshot.dry_run === true));
});
for (const id of ["probe-heated","probe-nozzle-target","probe-bed-target","probe-soak"]) byId(id).addEventListener("input",updateBedProbeControls);
byId("probe-copy-temperatures").addEventListener("click",()=>{
  const nozzle=Number(latestKlipperTemperatures.extruder?.target)||0, bed=Number(latestKlipperTemperatures.heater_bed?.target)||0;
  byId("probe-nozzle-target").value=nozzle;byId("probe-bed-target").value=bed;
  byId("probe-heated").checked=nozzle>0 || bed>0;updateBedProbeControls();
});
byId("bed-probe-abort").addEventListener("click", () => bedProbeAction("abort"));
byId("bed-probe-confirm").addEventListener("click", () => {
  const confirmation = bedProbeSnapshot.confirmation;
  if (!confirmation || bedProbeActionInFlight || consumedProbeConfirmations.has(confirmation.token)) return;
  consumedProbeConfirmations.add(confirmation.token);
  bedProbeAction("confirm", {stage:confirmation.stage, token:confirmation.token});
});
for (const button of document.querySelectorAll("[data-bed-z-step]")) {
  button.addEventListener("click", () => {
    const delta = Number(button.dataset.bedZStep);
    if (button.disabled || zControlsReason(bedProbeSnapshot, bedProbeAvailable, bedProbeActionInFlight, consumedProbeConfirmations) || zStepReason(bedProbeSnapshot, delta)) return;
    bedProbeAction("action", {command:"step", token:bedProbeSnapshot.action_token, delta_mm:delta});
  });
}
for (const [id, command] of [["bed-z-continue", "continue"], ["bed-z-finish", "finish_recovery"]]) {
  byId(id).addEventListener("click", () => {
    if (byId(id).disabled || zControlsReason(bedProbeSnapshot, bedProbeAvailable, bedProbeActionInFlight, consumedProbeConfirmations)) return;
    bedProbeAction("action", {command, token:bedProbeSnapshot.action_token});
  });
}

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
  if (byId("upload-job").disabled || jobTransferInFlight) return;
  const file = byId("gcode-upload").files[0];
  if (!file) return showMessage("upload-message", "Choose a .gcode file.", true);
  jobTransferInFlight = true;
  updateSyncControls();
  showMessage("upload-message", "Uploading to Moonraker and staging for AxisForge…");
  const form = new FormData();
  form.append("file", file);
  try {
    const response = await fetch("/api/jobs/upload", { method: "POST", body: form });
    const body = await response.json().catch(() => ({}));
    if (!response.ok) throw new Error(body.detail || `Upload failed (${response.status})`);
    byId("gcode-upload").value = "";
    showMessage("upload-message", `${body.original_name} saved as ${body.job}. ${body.message}`);
    await loadJobs();
    byId("job-select").value = body.job;
    await loadPreflight();
  } catch (error) {
    showMessage("upload-message", error.message, true);
  } finally {
    jobTransferInFlight = false;
    updateSyncControls();
  }
});

byId("copy-job-klipper").addEventListener("click", async () => {
  if (byId("copy-job-klipper").disabled || jobTransferInFlight) return;
  const job = byId("job-select").value;
  jobTransferInFlight = true;
  updateSyncControls();
  showMessage("copy-job-message", "Copying the selected file under a new version name on both hosts…");
  try {
    const body = await request("/api/jobs/copy-to-klipper", {method:"POST", body:JSON.stringify({job})});
    showMessage("copy-job-message", `${body.job}: ${body.message}`);
    selectedPreflightState = null;
    await loadJobs();
    byId("job-select").value = body.job;
    await loadPreflight();
  } catch (error) {
    showMessage("copy-job-message", error.message, true);
  } finally {
    jobTransferInFlight = false;
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

function calibrationStartOptions() {
  const value = id => byId(id).value.trim() === "" ? NaN : Number(byId(id).value);
  return {calibration_max_counts: value("load-calibration-max-counts"),
    calibration_hot_target: byId("load-calibration-hot").checked ? value("load-calibration-hot-target") : null};
}

function validCalibrationOptions(options) {
  return Number.isFinite(options.calibration_max_counts) && options.calibration_max_counts >= 90000 && options.calibration_max_counts <= 250000 &&
    (options.calibration_hot_target === null || (Number.isFinite(options.calibration_hot_target) && options.calibration_hot_target >= 50 && options.calibration_hot_target <= 300));
}

function calibrationMeasuredGrams() {
  const value = byId("load-calibration-grams").value.trim();
  return value === "" ? NaN : Number(value);
}

function updateSetupControls() {
  updateTcpControls();
  updateProbingControls();
  updateBedGeometryControls();
  const c = robotConnectionSnapshot, load = loadCalibrationSnapshot;
  const otherBusy = syncSnapshot.busy || bedProbeSnapshot.busy || bedProbeActionInFlight || syncActionInFlight || !latestAxisForgeIdle;
  byId("robot-connect").disabled = !c.available || !c.configured || Boolean(otherBusy || setupBusy());
  byId("robot-connection-message").textContent = c.message || "Checking PC robot connection service…";
  setBadge("robot-connection-badge", c.ready ? "Ready" : (c.state || "Checking"), c.ready ? true : undefined);
  const cfg = c.configuration;
  byId("robot-configuration").textContent = cfg ? `Robot: ${cfg.robot_type} at ${cfg.robot_ip} · AxisForge nozzle offset: ${cfg.axisforge_tcp_mm?.join(", ") || "unavailable"} mm · These saved values still need to match the installed tool.` : "";
  byId("robot-orientation").textContent = Array.isArray(cfg?.probing_rpy_deg) && cfg.probing_rpy_deg.length === 3 ?
    `Saved AxisForge orientation at A/B/C = 0: roll ${cfg.probing_rpy_deg[0]}°, pitch ${cfg.probing_rpy_deg[1]}°, yaw ${cfg.probing_rpy_deg[2]}° in Base.` :
    "Saved AxisForge orientation is unavailable; check the robot configuration before setting the initial pose.";
  byId("robot-readiness").replaceChildren(...(c.checks || []).map(check => {
    const item = document.createElement("li");
    item.textContent = `${check.status === "pass" ? "✓" : "○"} ${check.name}: ${check.detail}`;
    return item;
  }));
  const occupied = Boolean(otherBusy || setupBusy());
  const unavailable = !loadCalibrationAvailable || !load.configured;
  if (load.busy && Number.isFinite(load.calibration_max_counts)) {
    byId("load-calibration-max-counts").value = load.calibration_max_counts;
    byId("load-calibration-hot").checked = Number.isFinite(load.calibration_hot_target);
    if (Number.isFinite(load.calibration_hot_target)) byId("load-calibration-hot-target").value = load.calibration_hot_target;
  }
  const hotSelected = byId("load-calibration-hot").checked;
  const optionsValid = validCalibrationOptions(calibrationStartOptions());
  byId("load-calibration-options").disabled = occupied;
  byId("load-calibration-hot-target-label").hidden = !hotSelected;
  byId("load-calibration-hot-guidance").hidden = !hotSelected;
  byId("load-calibration-hot-target").disabled = !hotSelected || occupied;
  const reason = unavailable ? "Scale routine service is unavailable." : occupied ? "Finish or stop the current operation first." :
    !optionsValid ? "Enter a calibration raw limit of 90,000–250,000 counts and, for hot mode, a target of 50–300°C." :
    !latestSystemsReady ? latestSystemsReason : !syncSnapshot.robot_ready ? (syncSnapshot.robot_reason || "Complete the robot checks first.") : "";
  byId("load-calibration-start").disabled = Boolean(reason);
  byId("load-calibration-simulate").disabled = unavailable || occupied || !optionsValid;
  byId("load-calibration-abort").disabled = !load.busy || loadCalibrationInFlight || ["saving_config","verifying_config"].includes(load.state);
  byId("load-calibration-clear-session").disabled = unavailable || occupied || !latestSystemsReady;
  const hotActive = load.hot_calibration || Number.isFinite(load.calibration_hot_target);
  const savingCalibration = ["ready_to_save","saving_config","verifying_config"].includes(load.state);
  byId("load-calibration-readiness").textContent = load.busy ? (savingCalibration ? "Saving calibration with heaters off. Keep the scale unloaded; the robot remains stationary." : hotActive ? "Maintain the selected manual nozzle target and keep clear of the hot contact area. Stop cancels motion and holds position." : "Keep heater targets off and the nozzle cool. Stop cancels motion and holds position.") : reason || "Ready. Confirm the current start pose and unloaded scale placement, then use Z positioning before continuing.";
  setBadge("load-calibration-badge", `${load.dry_run ? "Simulation · " : ""}${load.state.replaceAll("_", " ")}`, ["failed","aborted","save_failed"].includes(load.state) ? false : load.state === "complete" ? true : undefined);
  byId("load-calibration-message").textContent = load.message || "";
  byId("load-calibration-sensor").textContent = Number.isFinite(load.trigger_value) ? `Contact: ${load.trigger_value.toFixed(0)} raw counts · Stop limit: ${load.max_value} counts. Live counts are refreshed after each move and while waiting.` : "Contact uses uncalibrated raw counts; no grams conversion is assumed.";
  byId("load-calibration-temperature").textContent = load.busy && Number.isFinite(load.temperature_c) ?
    `Nozzle: ${load.temperature_c.toFixed(1)}°C${Number.isFinite(load.target_c) ? ` · Target: ${load.target_c.toFixed(1)}°C` : ""}${load.state === "waiting_temperature" && Number.isFinite(load.stable_for_s) ? ` · Stable: ${load.stable_for_s.toFixed(0)}/${Number.isFinite(load.required_stable_s) ? load.required_stable_s : 30} s` : ""}.` : "";
  const confirmation = load.confirmation;
  const requested = loadCalibrationAvailable && load.busy && confirmation && !consumedCalibrationTokens.has(confirmation.token);
  if (requested && calibrationVisibleToken !== confirmation.token) {
    calibrationVisibleToken = confirmation.token; calibrationReadyAt = performance.now()+750;
    window.setTimeout(updateSetupControls, 760);
  }
  byId("load-calibration-confirmation").hidden = !requested;
  byId("load-calibration-confirm").disabled = !requested || loadCalibrationInFlight || performance.now() < calibrationReadyAt;
  if (requested) {
    const titles = {home:"Confirm the current home/start pose", scale:"Confirm the scale is under the nozzle", tare:"Confirm the weighing scale is tared"};
    const labels = {home:"Start pose confirmed — continue", scale:"Scale placed — enable Z positioning", tare:"Scale tared — enable load adjustment"};
    byId("load-calibration-confirm-title").textContent = titles[confirmation.stage];
    byId("load-calibration-confirm-detail").textContent = `${load.dry_run ? "Simulation only. " : ""}${load.message}`;
    byId("load-calibration-confirm").textContent = labels[confirmation.stage];
  }
  byId("load-calibration-steps").hidden = !load.busy || !["adjusting_load","stepping","step_rejected","load_limit","recording_load"].includes(load.state);
  const controlsReason = calibrationControlsReason();
  const enabled = !controlsReason;
  byId("load-calibration-z-status").textContent = controlsReason || (zStage(load) ? zStatusText(load) : load.upward_only || load.load_limit_reached ?
    "Z release is available: use Up 0.1 mm or finer until the load clears." :
    "Z steps are available. Use one click at a time; use 0.1 mm or finer to apply load.");
  byId("load-calibration-z-status").className = `inline-message${["failed","aborted","save_failed"].includes(load.state) ? " error" : ""}`;
  const step = Number(byId("load-step-size").value);
  const upReason = calibrationStepReason(step), downReason = calibrationStepReason(-step);
  byId("load-step-size").disabled = !enabled;
  byId("load-step-up").disabled = !enabled || Boolean(upReason);
  byId("load-step-down").disabled = !enabled || Boolean(downReason);
  for (const button of document.querySelectorAll("[data-calibration-step]")) {
    const reason = calibrationStepReason(Number(button.dataset.calibrationStep));
    button.disabled = !enabled || Boolean(reason);
    button.title = controlsReason || reason || "One click requests one Z move";
  }
  const z = zStage(load) ? load.z_controls : null;
  byId("load-z-navigation").hidden = !zStage(load);
  byId("load-z-position").textContent = zStage(load) ? zPositionText(load) : "";
  byId("load-z-continue").hidden = z?.phase !== "positioning";
  byId("load-z-continue").disabled = !enabled || z?.can_continue !== true || z?.loaded !== false;
  byId("load-z-finish").hidden = z?.phase !== "recovery";
  byId("load-z-finish").disabled = !enabled || z?.can_finish !== true || z?.loaded !== false;
  const releaseOnly = Boolean(load.upward_only || load.load_limit_reached);
  const recordingEnabled = enabled && load.state === "adjusting_load";
  const grams = calibrationMeasuredGrams();
  byId("load-calibration-grams").disabled = !recordingEnabled || releaseOnly || load.calibration_recorded === true;
  byId("load-calibration-record").disabled = !recordingEnabled || releaseOnly || load.calibration_recorded === true || load.unloaded !== false || !Number.isFinite(grams) || grams < 50 || grams > 25000;
  byId("load-calibration-finish").disabled = !recordingEnabled || releaseOnly || load.calibration_recorded !== true || load.unloaded !== true;
  byId("load-calibration-save-guidance").textContent = `${load.calibration_recorded !== true ? "Record a measured load first." : load.unloaded !== true || releaseOnly ? "Use Up until fresh readings confirm the scale is unloaded." : "The measured load is recorded and the scale is unloaded; calibration is ready to save."} Save turns heaters off and runs Klipper SAVE_CONFIG, which restarts Klipper. It does not move the robot.`;
  const recordedGrams = Number.isFinite(load.recorded_grams) ? load.recorded_grams : load.calibrated_grams;
  const calibrationDetails = [Number.isFinite(recordedGrams) && `${recordedGrams.toLocaleString()} g measured load`,
    Number.isFinite(load.calibration_counts_per_gram) && `${load.calibration_counts_per_gram.toFixed(3)} counts/g`,
    Number.isFinite(load.calibration_tare_counts) && `tare ${load.calibration_tare_counts.toFixed(0)} counts`].filter(Boolean).join(" · ");
  byId("load-calibration-result").textContent = load.state === "complete" ? `${load.dry_run ? "Simulation complete; no calibration was saved." : "Calibration saved and verified in Klipper."}${calibrationDetails ? ` ${calibrationDetails}.` : ""}` : load.state === "save_failed" ? `Calibration save was not verified. ${load.message || "Keep the robot stationary and inspect the Klipper configuration before retrying."}` : ["ready_to_save","saving_config","verifying_config"].includes(load.state) ? "Saving and checking Klipper calibration. Keep the scale unloaded; Klipper will restart and the robot remains stationary." : load.calibration_recorded === true ? `Measured load recorded${calibrationDetails ? `: ${calibrationDetails}` : ""}. Release the load before saving.` : "";
  byId("load-calibration-result").className = `inline-message${load.state === "save_failed" ? " error" : load.state === "complete" ? " success" : ""}`;
  byId("load-step-guidance").textContent = [upReason && `Up: ${upReason}`, downReason && `Down: ${downReason}`].filter(Boolean).join(" ") ||
    "Both directions are available within the current travel limits. Use 0.1 mm or finer near contact.";
  byId("load-calibration-position").textContent = Array.isArray(load.position_mm) ? `Nozzle Z: ${load.position_mm[2].toFixed(3)} mm · Allowed Z: ${load.lower_z_mm?.toFixed(3)} to ${load.upper_z_mm?.toFixed(3)} mm · Raw delta: ${load.delta_counts?.toFixed(0) || "—"} counts` : "";
}

function calibrationControlsReason() {
  const load = loadCalibrationSnapshot;
  if (!loadCalibrationAvailable || !load.configured) return "Z controls are unavailable: calibration status cannot be checked.";
  if (!load.busy) {
    if (["failed", "aborted", "save_failed"].includes(load.state)) {
      const recovery = /native calibration is open|unfinished.*calibration/i.test(load.message || "") ?
        " Use Clear unfinished Klipper calibration while idle, then start the scale routine again." :
        " Resolve this error, then start the scale routine again.";
      return `Z controls are unavailable because the routine stopped: ${load.message || load.state}.${recovery}`;
    }
    return "Start the scale routine and complete its confirmations to enable Z steps.";
  }
  if (load.stop_unconfirmed) return "Robot stop is unconfirmed; Z controls are unavailable.";
  if (zStage(load)) return zControlsReason(load, loadCalibrationAvailable, loadCalibrationInFlight, consumedCalibrationTokens);
  if (loadCalibrationInFlight) return "Waiting for the current calibration request; another Z step cannot be queued.";
  const stages = {
    starting: "Checking ALPS and Klipper before the scale routine. Z steps are not enabled yet.",
    waiting_temperature: "Waiting for the selected temperature to stabilize. Z steps are not enabled yet.",
    waiting_home: "Confirm the home/start pose above. Dashboard Z positioning follows unloaded scale placement confirmation.",
    waiting_scale: "Confirm the scale is under the clear, unloaded nozzle. Confirmation enables Z positioning; Continue starts the automatic contact search.",
    finding_scale: "Finding the scale automatically at 0.1 mm/s, with at most 20 mm descent. Z buttons enable after the lift and tare confirmation.",
    retracting: "Lifting 5 mm at 0.5 mm/s to unload the scale. Wait for the tare confirmation before using Z steps.",
    waiting_tare: "Tare the unloaded weighing scale, then confirm above. Klipper must record the unloaded reference before Z steps enable.",
    stepping: "A Z step is in progress. Wait for it to finish before requesting another.",
    step_rejected: "The requested step was rejected. Waiting for fresh position and load feedback before enabling another step.",
    load_limit: "The calibration load limit was reached. Waiting for a confirmed stop before enabling small upward release steps.",
    recording_load: "Recording the measured load in Klipper. Keep the arm and scale stationary.",
    accepting_calibration: "Validating calibration with the load released. Z steps are finished for this routine.",
    ready_to_save: "The robot routine has stopped. Preparing to save calibration; Z steps are finished.",
    saving_config: "Saving calibration and restarting Klipper. Z steps are finished for this routine.",
    verifying_config: "Verifying the saved calibration after Klipper restarts. Z steps are finished for this routine.",
  };
  if (load.state !== "adjusting_load") return stages[load.state] || "Waiting for the scale routine to reach Z adjustment.";
  if (!load.action_token || consumedCalibrationTokens.has(load.action_token)) return "Waiting for fresh feedback after the last action before enabling another Z step.";
  return "";
}

function calibrationStepReason(delta) {
  const load = loadCalibrationSnapshot;
  if (zStage(load)) return zStepReason(load, delta);
  const releaseOnly = load.upward_only || load.load_limit_reached;
  if (releaseOnly && delta < 0) return "Calibration load limit reached; use Up to release the load.";
  if (releaseOnly && delta > .1) return "Choose 0.1 mm or finer for upward load release.";
  const z = load.position_mm?.[2];
  if (![z, load.lower_z_mm, load.upper_z_mm, load.contact_z_mm].every(Number.isFinite)) return "Waiting for contact and travel limits.";
  if (![.001, .01, .05, .1, .5, 1, 10].includes(Math.abs(delta))) return "Choose a supported step size.";
  const end = z + delta;
  if (end < load.lower_z_mm - 1e-8 || end > load.upper_z_mm + 1e-8) return "This step exceeds the allowed Z range; choose a smaller step.";
  if (delta < -.1 && end < load.contact_z_mm + .5 - 1e-8) return "Use 0.1 mm or finer within 0.5 mm of contact.";
  return "";
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
for (const id of ["load-calibration-max-counts", "load-calibration-hot", "load-calibration-hot-target"]) byId(id).addEventListener("input", updateSetupControls);
for (const [id, dryRun] of [["load-calibration-start", false], ["load-calibration-simulate", true]]) {
  byId(id).addEventListener("click", () => {
    const options = calibrationStartOptions();
    if (!byId(id).disabled && validCalibrationOptions(options)) calibrationAction("start", {dry_run:dryRun, ...options});
  });
}
byId("load-calibration-abort").addEventListener("click", () => calibrationAction("abort"));
byId("load-calibration-clear-session").addEventListener("click", () => {
  if (!byId("load-calibration-clear-session").disabled) calibrationAction("clear-session");
});
byId("load-calibration-confirm").addEventListener("click", () => {
  const value = loadCalibrationSnapshot.confirmation;
  if (value && !consumedCalibrationTokens.has(value.token)) calibrationAction("confirm", value);
});
for (const [id, direction] of [["load-step-up",1],["load-step-down",-1]]) {
  byId(id).addEventListener("click", () => {
    const token = loadCalibrationSnapshot.action_token;
    const delta = direction * Number(byId("load-step-size").value);
    if (!byId(id).disabled && !calibrationStepReason(delta) && token && !consumedCalibrationTokens.has(token)) calibrationAction("action", {command:"step", token, delta_mm:delta});
  });
}
for (const button of document.querySelectorAll("[data-calibration-step]")) {
  button.addEventListener("click", () => {
    const token = loadCalibrationSnapshot.action_token;
    const delta = Number(button.dataset.calibrationStep);
    if (!button.disabled && !calibrationStepReason(delta) && token && !consumedCalibrationTokens.has(token)) calibrationAction("action", {command:"step", token, delta_mm:delta});
  });
}
for (const [id, command] of [["load-z-continue", "continue"], ["load-z-finish", "finish_recovery"]]) {
  byId(id).addEventListener("click", () => {
    const token = loadCalibrationSnapshot.action_token;
    if (byId(id).disabled || calibrationControlsReason() || !token || consumedCalibrationTokens.has(token)) return;
    calibrationAction("action", {command, token});
  });
}
byId("load-step-size").addEventListener("change", updateSetupControls);
byId("load-calibration-grams").addEventListener("input", updateSetupControls);
byId("load-calibration-record").addEventListener("click", () => {
  const token = loadCalibrationSnapshot.action_token;
  if (!byId("load-calibration-record").disabled && token && !consumedCalibrationTokens.has(token)) calibrationAction("action", {command:"record", token, grams:calibrationMeasuredGrams()});
});
byId("load-calibration-finish").addEventListener("click", () => {
  const token = loadCalibrationSnapshot.action_token;
  if (!byId("load-calibration-finish").disabled && token && !consumedCalibrationTokens.has(token)) calibrationAction("action", {command:"finish", token});
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

const probingInputs = {
  first_speed_mm_s: "probing-search-speed",
  max_descent_mm: "probing-search-limit",
  second_speed_mm_s: "probing-fine-speed",
  lift_speed_mm_s: "probing-lift-speed",
  mesh_columns: "probing-mesh-columns",
  mesh_rows: "probing-mesh-rows",
  mesh_yaw_deg: "probing-mesh-yaw",
  mesh_speed_mm_s: "probing-mesh-speed",
  mesh_samples: "probing-mesh-samples",
  mesh_repeatability_mm: "probing-repeatability",
};
const probingBounds = {
  first_speed_mm_s: [.5, 10], second_speed_mm_s: [.02, 1],
  max_descent_mm: [5, 350],
  lift_speed_mm_s: [1, 20], mesh_columns: [2, 9], mesh_rows: [2, 9],
  mesh_yaw_deg: [-180, 180],
  mesh_speed_mm_s: [.02, .5], mesh_samples: [1, 5], mesh_repeatability_mm: [.005, .05],
};

function probingValues() {
  return Object.fromEntries(Object.entries(probingInputs).map(([key, id]) =>
    [key, byId(id).value.trim() === "" ? NaN : Number(byId(id).value)]));
}

function validProbingSettings(values) {
  return Object.entries(probingBounds).every(([key, [min, max]]) =>
    Number.isFinite(values[key]) && values[key] >= min && values[key] <= max) &&
    [values.mesh_samples, values.mesh_columns, values.mesh_rows].every(Number.isInteger) &&
    values.second_speed_mm_s < values.first_speed_mm_s;
}

function updateProbingControls() {
  const occupied = Boolean(setupBusy() || syncSnapshot.busy || syncActionInFlight || bedProbeSnapshot.busy || bedProbeActionInFlight || !latestAxisForgeIdle);
  const values = probingValues();
  const initialValid = Number.isFinite(values.first_speed_mm_s) && values.first_speed_mm_s >= .5 && values.first_speed_mm_s <= 10;
  const speedsReversed = Number.isFinite(values.second_speed_mm_s) && Number.isFinite(values.first_speed_mm_s) &&
    values.second_speed_mm_s >= values.first_speed_mm_s;
  byId("probing-fine-speed").setCustomValidity(speedsReversed ? "Fine rehoming must be slower than the initial search." : "");
  const speedGuidance = byId("probing-speed-guidance");
  speedGuidance.textContent = !initialValid ? "Enter an initial search speed from 0.5–10 mm/s. 5 mm/s is the default, not the minimum." :
    speedsReversed ? "Fine rehoming must be slower than the initial search. Lower the fine speed before saving." :
    "Initial search: 0.5–10 mm/s; 5 mm/s is the default, not the minimum. Fine rehoming must be slower. Click Save homing settings to apply changes to the next run.";
  speedGuidance.className = !initialValid || speedsReversed ? "inline-message error" : "chart-caption";
  byId("probing-fields").disabled = occupied || !probingAvailable;
  byId("probing-save").disabled = occupied || !probingAvailable || !validProbingSettings(values) || !probingDirty || probingConflict;
  byId("probing-reload").disabled = probingSaveInFlight || probingRefreshInFlight;
  setBadge("probing-settings-badge", probingSaveInFlight ? "Saving…" : !probingAvailable ? "Unavailable" : occupied ? "Operation active" :
    probingConflict ? "Reload required" : probingDirty ? "Unsaved changes" : "Saved", probingConflict ? false : undefined);
  const saved = probingSnapshot?.settings;
  byId("probing-saved").textContent = saved ? `Saved: initial ${saved.first_speed_mm_s} mm/s · maximum Z search ${saved.max_descent_mm} mm · fine ${saved.second_speed_mm_s} mm/s · upward ${saved.lift_speed_mm_s} mm/s · ${saved.mesh_columns}×${saved.mesh_rows} mesh (${saved.mesh_columns * saved.mesh_rows} points) at ${saved.mesh_speed_mm_s} mm/s · ${saved.mesh_samples} contacts per point · maximum spread ${saved.mesh_repeatability_mm} mm · bed direction ${saved.mesh_yaw_deg}° from Base.` :
    "Reading the saved homing and mesh settings…";
}

function acceptProbingSettings(body, replaceInputs=false, preserveInputs=false) {
  if (!body || typeof body.revision !== "string" || !body.settings || !validProbingSettings(body.settings)) {
    throw new Error("The saved homing settings are invalid or unavailable.");
  }
  const revisionChanged = probingSnapshot && probingSnapshot.revision !== body.revision;
  if (replaceInputs || (!preserveInputs && !probingDirty && !probingConflict)) {
    for (const [key, id] of Object.entries(probingInputs)) byId(id).value = body.settings[key];
    probingDirty = false; probingConflict = false; probingSnapshot = body;
  } else if (revisionChanged) {
    probingConflict = true;
    showMessage("probing-message", "Saved homing settings changed while you were editing. Reload saved settings, then review your changes.", true);
  } else {
    probingSnapshot = body;
  }
  probingAvailable = true;
}

async function refreshProbing(replaceInputs=false) {
  if (probingRefreshInFlight || probingSaveInFlight) return;
  probingRefreshInFlight = true;
  const version = probingRequestVersion;
  const editVersion = probingEditVersion;
  updateProbingControls();
  try {
    const body = await request("/api/setup/probing");
    if (version !== probingRequestVersion) return;
    // A manual reload may return after the operator starts another edit. Keep
    // those newer inputs instead of snapping them back to the saved values.
    const newerEdits = editVersion !== probingEditVersion;
    const replaceCurrentInputs = replaceInputs && !newerEdits;
    acceptProbingSettings(body, replaceCurrentInputs, newerEdits);
    if (replaceInputs && !probingConflict) showMessage("probing-message", replaceCurrentInputs ?
      "Loaded the saved homing and mesh settings." : "Saved settings checked; your newer edits were kept.");
  } catch (error) {
    if (version !== probingRequestVersion) return;
    probingAvailable = false;
    showMessage("probing-message", error.message, true);
  } finally { probingRefreshInFlight = false; updateSyncControls(); }
}

for (const id of Object.values(probingInputs)) byId(id).addEventListener("input", () => {
  probingEditVersion++;
  const values = probingValues();
  probingDirty = !probingSnapshot || Object.keys(probingInputs).some(key => values[key] !== probingSnapshot.settings[key]);
  updateSyncControls();
});
byId("probing-reload").addEventListener("click", () => refreshProbing(true));
byId("probing-form").addEventListener("submit", async event => {
  event.preventDefault();
  if (byId("probing-save").disabled || !byId("probing-form").reportValidity()) return;
  const settings = probingValues();
  probingSaveInFlight = true; probingRequestVersion++;
  updateSyncControls();
  try {
    const result = await request("/api/setup/probing", {method:"POST", body:JSON.stringify({revision:probingSnapshot.revision, settings})});
    acceptProbingSettings(result, true);
    showMessage("probing-message", result.message || "Homing and mesh settings saved for the next routine.");
    if (result.changed) { selectedPreflightState = null; loadPreflight(); refreshSync(); }
  } catch (error) {
    // The server may have committed a write before a response was lost. Do not
    // automatically retry against a revision that may no longer be current.
    probingConflict = true;
    showMessage("probing-message", `${error.message} Reload saved settings before retrying.`, true);
  } finally { probingSaveInFlight = false; updateSyncControls(); }
});
refreshProbing();
window.setInterval(refreshProbing, 5000);

function bedGeometryValues() {
  const values = ["bed-width", "bed-depth", "bed-margin"].map(id =>
    byId(id).value.trim() === "" ? NaN : Number(byId(id).value));
  return {width_mm: values[0], depth_mm: values[1], probe_margin_mm: values[2]};
}

function validBedGeometry(v) {
  return Object.values(v).every(Number.isFinite) && v.width_mm >= 10 && v.width_mm <= 2000 &&
    v.depth_mm >= 10 && v.depth_mm <= 2000 && v.probe_margin_mm >= 0 &&
    2*v.probe_margin_mm < Math.min(v.width_mm, v.depth_mm);
}

function updateBedGeometryControls() {
  const occupied = Boolean(setupBusy() || syncSnapshot.busy || syncActionInFlight || bedProbeSnapshot.busy || bedProbeActionInFlight || !latestAxisForgeIdle);
  const v = bedGeometryValues(), valid = validBedGeometry(v);
  byId("bed-margin").setCustomValidity(Number.isFinite(v.probe_margin_mm) &&
    2*v.probe_margin_mm >= Math.min(v.width_mm, v.depth_mm) ? "Margin must leave a positive mesh area." : "");
  byId("bed-settings-fields").disabled = occupied || !bedSettingsAvailable;
  byId("bed-settings-save").disabled = occupied || !bedSettingsAvailable || !valid || !bedSettingsDirty || bedSettingsConflict;
  byId("bed-settings-reload").disabled = bedSettingsSaveInFlight || bedSettingsRefreshInFlight;
  setBadge("bed-settings-badge", bedSettingsSaveInFlight ? "Saving…" : !bedSettingsAvailable ? "Unavailable" : occupied ? "Operation active" :
    bedSettingsConflict ? "Reload required" : bedSettingsDirty ? "Unsaved changes" : "Saved", bedSettingsConflict ? false : undefined);
  const saved = bedSettingsSnapshot;
  byId("bed-settings-saved").textContent = saved ? `Saved bed: ${saved.bed_size_mm.join(" × ")} mm · edge margin ${saved.probe_margin_mm} mm.` : "Reading saved bed dimensions…";
  byId("bed-settings-area").textContent = valid ? `${bedSettingsDirty ? "Preview" : "Mesh area"}: G-code X ${v.probe_margin_mm}–${v.width_mm-v.probe_margin_mm} mm, Y ${v.probe_margin_mm}–${v.depth_mm-v.probe_margin_mm} mm. First point relative to homed centre: X ${v.probe_margin_mm-v.width_mm/2}, Y ${v.probe_margin_mm-v.depth_mm/2} mm before bed-direction rotation.` : "Enter dimensions and a margin that leave a positive probing area.";
  byId("bed-settings-mesh").textContent = saved?.bed_mesh?.message || "";
}

function acceptBedGeometry(body, replaceInputs=false) {
  const values = {width_mm:body?.bed_size_mm?.[0], depth_mm:body?.bed_size_mm?.[1], probe_margin_mm:body?.probe_margin_mm};
  if (!validBedGeometry(values) || typeof body.revision !== "string") throw new Error("Saved bed dimensions are invalid or unavailable.");
  if (replaceInputs || (!bedSettingsDirty && !bedSettingsConflict)) {
    byId("bed-width").value = values.width_mm;
    byId("bed-depth").value = values.depth_mm;
    byId("bed-margin").value = values.probe_margin_mm;
    bedSettingsDirty = bedSettingsConflict = false;
    bedSettingsSnapshot = body;
  } else if (bedSettingsSnapshot && body.revision !== bedSettingsSnapshot.revision) {
    bedSettingsConflict = true;
    showMessage("bed-settings-message", "AxisForge configuration changed while you were editing. Reload dimensions and review your changes.", true);
  } else {
    bedSettingsSnapshot = body;
  }
  bedSettingsAvailable = true;
}

async function refreshBedGeometry(replaceInputs=false) {
  if (bedSettingsRefreshInFlight || bedSettingsSaveInFlight) return;
  bedSettingsRefreshInFlight = true;
  const version = bedSettingsRequestVersion;
  updateBedGeometryControls();
  try {
    const body = await request("/api/setup/bed");
    if (version !== bedSettingsRequestVersion) return;
    acceptBedGeometry(body, replaceInputs);
    if (replaceInputs) showMessage("bed-settings-message", "Loaded saved bed dimensions.");
  } catch (error) {
    if (version !== bedSettingsRequestVersion) return;
    bedSettingsAvailable = false;
    showMessage("bed-settings-message", error.message, true);
  } finally { bedSettingsRefreshInFlight = false; updateSyncControls(); }
}

for (const id of ["bed-width", "bed-depth", "bed-margin"]) byId(id).addEventListener("input", () => {
  const values = bedGeometryValues(), saved = bedSettingsSnapshot;
  bedSettingsDirty = !saved || values.width_mm !== saved.bed_size_mm[0] || values.depth_mm !== saved.bed_size_mm[1] || values.probe_margin_mm !== saved.probe_margin_mm;
  updateSyncControls();
});
byId("bed-settings-reload").addEventListener("click", () => refreshBedGeometry(true));
byId("bed-settings-form").addEventListener("submit", async event => {
  event.preventDefault();
  if (byId("bed-settings-save").disabled || !byId("bed-settings-form").reportValidity()) return;
  const values = bedGeometryValues();
  bedSettingsSaveInFlight = true; bedSettingsRequestVersion++;
  updateSyncControls();
  try {
    const body = await request("/api/setup/bed", {method:"POST", body:JSON.stringify({...values, revision:bedSettingsSnapshot.revision})});
    acceptBedGeometry(body, true);
    showMessage("bed-settings-message", body.message);
    if (body.changed) { selectedPreflightState = null; loadPreflight(); refreshSync(); refreshTcp(); }
  } catch (error) {
    bedSettingsConflict = true;
    showMessage("bed-settings-message", `${error.message} Reload saved dimensions before retrying.`, true);
  } finally { bedSettingsSaveInFlight = false; updateSyncControls(); }
});
refreshBedGeometry();
window.setInterval(refreshBedGeometry, 5000);
