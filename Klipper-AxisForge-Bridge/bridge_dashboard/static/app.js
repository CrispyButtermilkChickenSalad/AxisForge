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
let latestAxisForgeIdle = false;
let selectedPreflightState = null;
let syncSnapshot = { state: "idle", armed: false, job: null, control_ready: false };
let syncActionInFlight = false;

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
  byId("dashboard-view").hidden = false;
  byId("embedded-view").hidden = true;
  drawTemperatureHistory();
});

for (const tab of toolTabs.slice(1)) {
  tab.addEventListener("click", (event) => {
    if (event.button !== 0 || event.ctrlKey || event.metaKey || event.shiftKey || event.altKey) return;
    event.preventDefault();
    if (tab.getAttribute("aria-disabled") === "true" || !tab.href || tab.hidden) return;

    selectTab(tab);
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

function updateSyncControls() {
  const job = byId("job-select").value;
  const busy = Boolean(syncSnapshot.busy);
  const ready = selectedPreflightState === "preflight_passed" && syncSnapshot.job === job && heaterChecksMatch();
  byId("heater-check-options").disabled = busy || syncActionInFlight || !heaterOptionsInitialized;
  byId("job-select").disabled = busy || syncActionInFlight;
  byId("upload-job").disabled = busy || syncActionInFlight;
  byId("preflight-both").disabled = busy || syncActionInFlight || !heaterOptionsInitialized || !job || !latestSystemsReady || !latestAxisForgeIdle || !syncSnapshot.control_ready;
  byId("sync-start").disabled = busy || syncActionInFlight || !ready || !latestSystemsReady || !latestAxisForgeIdle || !syncSnapshot.control_ready || !syncSnapshot.robot_ready;
  byId("sync-abort").disabled = !busy;
  byId("start-job").disabled = busy || !job || !latestSystemsReady || !latestAxisForgeIdle;
  const label = busy ? syncSnapshot.stage : !syncSnapshot.control_ready ? "Services unavailable" : ready ? "Preflight passed" : "Preflight required";
  setBadge("sync-availability-badge", label || "Working", ready && !busy);
  byId("step-sync").textContent = busy ? syncSnapshot.message : !syncSnapshot.control_ready ? syncSnapshot.reason : ready ? (syncSnapshot.robot_ready ? "Ready to start" : syncSnapshot.robot_reason) : "Run preflight on the selected file";
}

async function loadPreflight() {
  const job = byId("job-select").value;
  selectedPreflightState = null;
  selectedHeaterChecks = null;
  if (job) {
    try {
      const result = await request(`/api/jobs/${encodeURIComponent(job)}/preflight`);
      if (byId("job-select").value === job) {
        selectedHeaterChecks = result.heater_checks;
        selectedPreflightState = result.state === "preflight_passed" && !heaterChecksMatch() ? "staged" : result.state;
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
      method:"POST", body:JSON.stringify({heater_checks:heaterChecks()}),
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
        method:"POST", body:JSON.stringify({job:byId("job-select").value, heater_checks:heaterChecks()}),
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
    button.disabled = false;
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
window.setInterval(refreshStatus, 2000);
window.setInterval(refreshTelemetry, 5000);
window.addEventListener("resize", drawTemperatureHistory);
