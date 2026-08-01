"""Loopback live display for measured-state Gazebo certificate frames."""

from __future__ import annotations

from datetime import datetime, timezone
from functools import partial
from hashlib import sha256
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from io import BytesIO
import json
from math import isfinite
from pathlib import Path
from threading import Thread
from time import sleep
from typing import Any, Mapping

from cbf_rover.domain.dynamic_road import DynamicRoadScenario

from .differential_road_contract import (
    GazeboCameraRecordingSpec,
    GazeboFrontCameraSpec,
    GazeboPhysicalRoverSpec,
)


_VIEWER_HTML = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>CBF Rover live certificate</title>
  <style>
    :root {
      color-scheme: dark;
      --bg: #0b0f12;
      --panel: #12191d;
      --line: #344148;
      --muted: #9aa6a8;
      --text: #e8e3d8;
      --blue: #43c7d9;
      --violet: #a792d4;
      --green: #66c78f;
      --amber: #ffb000;
      --red: #ff5c4d;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      background: var(--bg);
      color: var(--text);
      font-family: Bahnschrift, "Segoe UI", ui-sans-serif, system-ui, sans-serif;
    }
    main { width: min(1720px, 96vw); margin: 0 auto; padding: 18px 0 30px; }
    header { display: flex; gap: 18px; justify-content: space-between; align-items: end; }
    h1 { margin: 0; font-size: clamp(22px, 2.4vw, 34px); letter-spacing: -0.025em; }
    .eyebrow { color: var(--violet); font: 700 12px/1.4 ui-monospace, monospace; }
    .sub { color: var(--muted); margin: 7px 0 0; }
    .status {
      border: 1px solid var(--line);
      border-radius: 2px;
      padding: 8px 13px;
      font: 700 12px/1 ui-monospace, monospace;
    }
    .running { color: var(--green); border-color: var(--green); }
    .completed { color: var(--blue); border-color: var(--blue); }
    .failed, .disconnected { color: var(--red); border-color: var(--red); }
    .metrics {
      display: grid;
      grid-template-columns: repeat(3, minmax(140px, 1fr));
      gap: 10px;
      margin: 18px 0 12px;
    }
    .metric, .panel {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 3px;
    }
    .metric { padding: 12px 13px; min-height: 76px; }
    .label { color: var(--muted); font: 600 10px/1.3 ui-monospace, monospace; text-transform: uppercase; }
    .value { margin-top: 8px; font: 700 18px/1.1 ui-monospace, monospace; }
    .layout { display: grid; grid-template-columns: minmax(0, 2.35fr) minmax(330px, 1fr); gap: 12px; align-items: start; }
    .panel { padding: 12px; min-width: 0; }
    .panel h2 { margin: 0 0 9px; font-size: 14px; }
    canvas { display: block; width: 100%; height: auto; background: #090d0f; border-radius: 2px; }
    .camera-panel { padding: 0; overflow: hidden; }
    .camera-panel h2 { padding: 12px 14px 0; }
    .camera-frame { position: relative; margin: 10px 12px 0; border: 1px solid #27343a; background: #070a0c; aspect-ratio: 16 / 9; overflow: hidden; }
    .camera-image { display: block; width: 100%; height: 100%; object-fit: fill; }
    #camera { position: absolute; inset: 0; width: 100%; height: 100%; background: transparent; }
    .camera-caption { display: flex; justify-content: space-between; gap: 12px; padding: 10px 14px 13px; color: var(--muted); font-size: 11px; }
    .stack { display: grid; gap: 12px; }
    .facts { display: grid; grid-template-columns: 1fr auto; gap: 8px 14px; font-size: 12px; }
    .facts dt { color: var(--muted); }
    .facts dd { margin: 0; font-family: ui-monospace, monospace; text-align: right; }
    .legend { display: flex; flex-wrap: wrap; gap: 13px; color: var(--muted); font-size: 11px; margin-top: 9px; }
    .dot { display: inline-block; width: 9px; height: 9px; border-radius: 50%; margin-right: 5px; }
    .note { color: var(--muted); font-size: 11px; line-height: 1.5; margin-top: 11px; }
    details.panel { padding: 0; }
    summary { cursor: pointer; list-style: none; padding: 13px 14px; color: var(--text); font-size: 13px; font-weight: 700; }
    summary::-webkit-details-marker { display: none; }
    summary::after { content: "+"; float: right; color: var(--amber); font: 700 16px/1 ui-monospace, monospace; }
    details[open] summary::after { content: "−"; }
    .telemetry-body { border-top: 1px solid var(--line); padding: 12px; }
    :focus-visible { outline: 2px solid var(--amber); outline-offset: 3px; }
    @media (max-width: 980px) {
      .metrics { grid-template-columns: repeat(3, 1fr); }
      .layout { grid-template-columns: 1fr; }
    }
    @media (max-width: 560px) {
      header { align-items: start; flex-direction: column; }
      .metrics { grid-template-columns: repeat(2, 1fr); }
    }
    @media (prefers-reduced-motion: reduce) {
      * { scroll-behavior: auto !important; }
    }
  </style>
</head>
<body>
<main>
  <header>
    <div>
      <div class="eyebrow">GAZEBO CLASSIC 11 · INDOOR SAFETY ROOM · MEASURED STATE</div>
      <h1>Rover live obstacle certificate</h1>
      <p class="sub">车载 RGB 主视角；激活障碍的证书边界直接投影到相机画面。</p>
    </div>
    <div id="status" class="status">STARTING</div>
  </header>
  <section class="metrics">
    <div class="metric"><div class="label">simulated time</div><div id="time" class="value">—</div></div>
    <div class="metric"><div class="label">minimum lifted h</div><div id="barrier" class="value">—</div></div>
    <div class="metric"><div class="label">active obstacles</div><div id="obstacles" class="value">—</div></div>
  </section>
  <section class="layout">
    <div class="panel camera-panel">
      <h2>FRONT RGB / CERTIFICATE PROJECTION</h2>
      <div class="camera-frame">
        <img id="cameraImage" class="camera-image" src="front_camera.png" alt="Gazebo front RGB camera">
        <canvas id="camera" width="960" height="540" role="img" aria-label="Gazebo front camera with active obstacle-certificate boundaries"></canvas>
      </div>
      <div class="camera-caption">
        <span>琥珀虚线：当前激活的 lifted obstacle certificate</span>
        <span id="cameraState">WAITING FOR CAMERA</span>
      </div>
    </div>
    <div class="stack">
      <div class="panel">
        <h2>TOP-DOWN SAFETY MAP</h2>
        <canvas id="scene" width="690" height="300" role="img" aria-label="Top-down road and active certificate map"></canvas>
        <div class="legend">
          <span><i class="dot" style="background:#43c7d9"></i>trajectory</span>
          <span><i class="dot" style="background:#ff5c4d"></i>obstacle</span>
          <span><i class="dot" style="background:#ffb000"></i>certificate</span>
        </div>
      </div>
      <details class="panel" id="telemetryDetails">
        <summary>完整遥测与证书曲线</summary>
        <div class="telemetry-body">
          <canvas id="chart" width="640" height="260"></canvas>
          <dl class="facts">
            <dt>CBF intervention</dt><dd id="intervention">—</dd>
            <dt>linear command</dt><dd id="linear">—</dd>
            <dt>angular command</dt><dd id="angular">—</dd>
            <dt>sequence / step</dt><dd><span id="sequence">—</span> / <span id="step">—</span></dd>
            <dt>center pose</dt><dd id="pose">—</dd>
            <dt>lookahead point</dt><dd id="lookahead">—</dd>
            <dt>contraction residual</dt><dd id="contraction">—</dd>
            <dt>new obstacle</dt><dd id="newObstacle">—</dd>
            <dt>termination</dt><dd id="termination">—</dd>
            <dt>last server update</dt><dd id="updated">—</dd>
          </dl>
          <p class="note">
            The display is an observer only and never feeds data back to the controller.
            The online distance CBF may modify the lookahead velocity before differential mapping.
            This is empirical Gazebo evidence, not a q-SBC probability claim.
          </p>
        </div>
      </details>
    </div>
  </section>
</main>
<script>
  const $ = (id) => document.getElementById(id);
  const fmt = (v, n = 3, suffix = "") => v == null ? "n/a" : `${Number(v).toFixed(n)}${suffix}`;
  const set = (id, value) => { $(id).textContent = value; };
  let lastStatus = null;
  const scene = $("scene");
  const sceneContext = scene.getContext("2d");
  const cameraCanvas = $("camera");
  const cameraContext = cameraCanvas.getContext("2d");
  const cameraImage = $("cameraImage");
  const chart = $("chart");
  const chartContext = chart.getContext("2d");
  let lastSnapshot = null;
  let requestedCameraVersion = -1;

  function line(context, points, color, width = 2, dash = []) {
    if (points.length < 2) return;
    context.beginPath();
    context.setLineDash(dash);
    context.strokeStyle = color;
    context.lineWidth = width;
    points.forEach(([x, y], index) => index ? context.lineTo(x, y) : context.moveTo(x, y));
    context.stroke();
    context.setLineDash([]);
  }

  function cameraFallback(message) {
    const context = cameraContext;
    context.clearRect(0, 0, cameraCanvas.width, cameraCanvas.height);
    context.fillStyle = "#070a0c";
    context.fillRect(0, 0, cameraCanvas.width, cameraCanvas.height);
    context.fillStyle = "#9aa6a8";
    context.font = "600 18px Bahnschrift, Segoe UI, sans-serif";
    context.textAlign = "center";
    context.fillText(message, cameraCanvas.width / 2, cameraCanvas.height / 2);
    context.textAlign = "left";
  }

  function projectCameraPoint(snapshot, worldX, worldY, worldZ) {
    const camera = snapshot.camera;
    const state = camera && camera.alignment && camera.alignment.state;
    if (!camera || !state) return null;
    const dx = worldX - state.x_m;
    const dy = worldY - state.y_m;
    const c = Math.cos(state.heading_rad);
    const s = Math.sin(state.heading_rad);
    const baseForward = c * dx + s * dy - camera.mount_x_m;
    const left = -s * dx + c * dy - camera.mount_y_m;
    const baseUp = worldZ - camera.world_height_m;
    const pitchCos = Math.cos(camera.pitch_rad);
    const pitchSin = Math.sin(camera.pitch_rad);
    const forward = pitchCos * baseForward - pitchSin * baseUp;
    if (forward <= camera.near_clip_m + 0.01) return null;
    const up = pitchSin * baseForward + pitchCos * baseUp;
    const focal = cameraCanvas.width /
      (2 * Math.tan(camera.horizontal_fov_rad / 2));
    return [
      cameraCanvas.width / 2 - focal * left / forward,
      cameraCanvas.height / 2 - focal * up / forward,
      forward,
    ];
  }

  function drawCameraCertificates(snapshot) {
    if (!snapshot.frame || !snapshot.camera) return;
    const context = cameraContext;
    const alignment = snapshot.camera.alignment;
    if (!alignment || !alignment.state) return;
    const activeIds = new Set(alignment.active_obstacle_ids || []);
    const alignedBarriers = alignment.obstacle_barrier_m || {};
    snapshot.scenario.obstacles.filter((item) => activeIds.has(item.id)).forEach((obstacle) => {
      const samples = 96;
      let drawing = false;
      context.beginPath();
      for (let index = 0; index <= samples; index += 1) {
        const angle = 2 * Math.PI * index / samples;
        const point = projectCameraPoint(
          snapshot,
          obstacle.x_m + obstacle.lifted_radius_m * Math.cos(angle),
          obstacle.y_m + obstacle.lifted_radius_m * Math.sin(angle),
          0.018
        );
        if (!point) {
          drawing = false;
          continue;
        }
        if (drawing) context.lineTo(point[0], point[1]);
        else context.moveTo(point[0], point[1]);
        drawing = true;
      }
      const barrier = alignedBarriers[obstacle.id];
      const danger = barrier != null && barrier < 0.10;
      context.strokeStyle = danger ? "#ff5c4d" : "#ffb000";
      context.lineWidth = danger ? 5 : 4;
      context.setLineDash([14, 9]);
      context.stroke();
      context.setLineDash([]);

      const labelPoint = projectCameraPoint(
        snapshot,
        obstacle.x_m,
        obstacle.y_m,
        0.58
      );
      if (labelPoint && labelPoint[0] > 10 && labelPoint[0] < cameraCanvas.width - 10) {
        const label = `${obstacle.id.replace("obstacle_", "O")}  h=${fmt(barrier, 3, "m")}`;
        context.font = "700 15px ui-monospace, monospace";
        const textWidth = context.measureText(label).width;
        context.fillStyle = "rgba(7, 10, 12, 0.86)";
        context.fillRect(labelPoint[0] - textWidth / 2 - 7, labelPoint[1] - 19, textWidth + 14, 25);
        context.fillStyle = danger ? "#ff5c4d" : "#ffb000";
        context.textAlign = "center";
        context.fillText(label, labelPoint[0], labelPoint[1]);
        context.textAlign = "left";
      }
    });
  }

  function drawCamera(snapshot) {
    const camera = snapshot.camera;
    if (!camera || !camera.configured) {
      set("cameraState", "CAMERA NOT CONFIGURED");
      cameraFallback("Front camera is not configured for this run");
      return;
    }
    if (!camera.available) {
      set("cameraState", "WAITING FOR GAZEBO FRAME");
      cameraFallback("Waiting for the first Gazebo RGB frame…");
      return;
    }
    if (cameraCanvas.width !== camera.image_width || cameraCanvas.height !== camera.image_height) {
      cameraCanvas.width = camera.image_width;
      cameraCanvas.height = camera.image_height;
    }
    const version = camera.frame_version;
    if (requestedCameraVersion === version) {
      cameraContext.clearRect(0, 0, cameraCanvas.width, cameraCanvas.height);
      drawCameraCertificates(snapshot);
      return;
    }
    requestedCameraVersion = version;
    cameraImage.dataset.version = String(version);
    cameraImage.src = `${camera.frame_file}?v=${version}`;
  }

  cameraImage.addEventListener("load", () => {
    cameraContext.clearRect(0, 0, cameraCanvas.width, cameraCanvas.height);
    if (!lastSnapshot || !lastSnapshot.camera) {
      set("cameraState", "RGB FRAME LOADED");
      return;
    }
    const version = Number(cameraImage.dataset.version || -1);
    if (version !== lastSnapshot.camera.frame_version) return;
    drawCameraCertificates(lastSnapshot);
    set("cameraState", `RGB FRAME ${version}`);
  });
  cameraImage.addEventListener("error", () => {
    requestedCameraVersion = -1;
    set("cameraState", "FRAME LOAD ERROR");
    cameraFallback("Gazebo camera frame could not be loaded");
  });

  function drawScene(snapshot) {
    const context = sceneContext;
    const road = snapshot.scenario.road;
    const padding = 42;
    const width = scene.width - 2 * padding;
    const height = scene.height - 2 * padding;
    const yMin = road.physical_y_min_m - 0.48;
    const yMax = road.physical_y_max_m + 0.48;
    const sx = (x) => padding + (x - road.x_min_m) / (road.x_max_m - road.x_min_m) * width;
    const sy = (y) => padding + (yMax - y) / (yMax - yMin) * height;
    const radius = (r) => r / (road.x_max_m - road.x_min_m) * width;
    context.clearRect(0, 0, scene.width, scene.height);
    context.fillStyle = "#344054";
    context.fillRect(
      sx(road.x_min_m),
      sy(road.physical_y_max_m),
      sx(road.x_max_m) - sx(road.x_min_m),
      sy(road.physical_y_min_m) - sy(road.physical_y_max_m)
    );
    road.lane_centers_y_m.forEach((lane) => {
      line(context, [[sx(road.x_min_m), sy(lane)], [sx(road.x_max_m), sy(lane)]], "#6c7789", 1);
    });
    line(context, [[sx(road.x_min_m), sy(0)], [sx(road.x_max_m), sy(0)]], "#d0d5dd", 1, [8, 8]);
    line(
      context,
      [[sx(road.finish_line_x_m), sy(road.physical_y_min_m)],
       [sx(road.finish_line_x_m), sy(road.physical_y_max_m)]],
      "#32d583",
      3
    );
    const history = snapshot.history;
    line(context, history.map((item) => [sx(item.x_m), sy(item.y_m)]), "#2e90fa", 3);
    line(
      context,
      history.map((item) => [sx(item.lookahead_x_m), sy(item.lookahead_y_m)]),
      "#b692f6",
      2,
      [5, 5]
    );
    snapshot.scenario.obstacles.filter((item) => item.active).forEach((obstacle) => {
      context.beginPath();
      context.strokeStyle = obstacle.barrier_m != null && obstacle.barrier_m < 0.10 ? "#f04438" : "#fdb022";
      context.lineWidth = 3;
      context.setLineDash([7, 5]);
      context.arc(sx(obstacle.x_m), sy(obstacle.y_m), radius(obstacle.lifted_radius_m), 0, 2 * Math.PI);
      context.stroke();
      context.setLineDash([]);
      context.beginPath();
      context.fillStyle = "#f04438";
      context.strokeStyle = "#fda29b";
      context.lineWidth = 2;
      context.arc(sx(obstacle.x_m), sy(obstacle.y_m), radius(obstacle.physical_radius_m), 0, 2 * Math.PI);
      context.fill();
      context.stroke();
      context.fillStyle = "#f2f4f7";
      context.font = "12px ui-monospace, monospace";
      context.textAlign = "center";
      context.fillText(obstacle.id.replace("obstacle_", "O"), sx(obstacle.x_m), sy(obstacle.y_m) - 12);
    });
    if (!snapshot.frame) return;
    const state = snapshot.frame.state;
    context.beginPath();
    context.fillStyle = "#f2f4f7";
    context.strokeStyle = "#2e90fa";
    context.lineWidth = 3;
    context.arc(sx(state.x_m), sy(state.y_m), radius(road.rover_radius_m), 0, 2 * Math.PI);
    context.fill();
    context.stroke();
    line(
      context,
      [[sx(state.x_m), sy(state.y_m)], [sx(state.lookahead_x_m), sy(state.lookahead_y_m)]],
      "#2e90fa",
      4
    );
    context.beginPath();
    context.fillStyle = "#b692f6";
    context.arc(sx(state.lookahead_x_m), sy(state.lookahead_y_m), 6, 0, 2 * Math.PI);
    context.fill();
    context.fillStyle = "#98a2b3";
    context.font = "11px ui-monospace, monospace";
    context.textAlign = "left";
    context.fillText("road x [m]", scene.width - 102, scene.height - 13);
  }

  function drawChart(snapshot) {
    const context = chartContext;
    const history = snapshot.history;
    const padding = 38;
    const width = chart.width - 2 * padding;
    const height = chart.height - 2 * padding;
    context.clearRect(0, 0, chart.width, chart.height);
    context.fillStyle = "#98a2b3";
    context.font = "11px ui-monospace, monospace";
    if (!history.length) {
      context.fillText("Waiting for the first certificate frame…", padding, padding);
      return;
    }
    const timeMax = Math.max(history.at(-1).time_s, 0.05);
    const values = history.flatMap((item) => [item.minimum_barrier_m, item.intervention_mps || 0]);
    const valueMin = Math.min(0, ...values);
    const valueMax = Math.max(0.2, ...values);
    const sx = (x) => padding + x / timeMax * width;
    const sy = (y) => padding + (valueMax - y) / (valueMax - valueMin || 1) * height;
    line(context, [[padding, sy(0)], [padding + width, sy(0)]], "#f04438", 1, [6, 5]);
    line(
      context,
      history.map((item) => [sx(item.time_s), sy(item.minimum_barrier_m)]),
      "#32d583",
      3
    );
    line(
      context,
      history.map((item) => [sx(item.time_s), sy(item.intervention_mps || 0)]),
      "#fdb022",
      2
    );
    context.fillStyle = "#32d583";
    context.fillText("minimum lifted h [m]", padding, 16);
    context.fillStyle = "#fdb022";
    context.fillText("CBF intervention [m/s]", padding + 185, 16);
    context.fillStyle = "#98a2b3";
    context.fillText(`0 s`, padding, chart.height - 10);
    context.textAlign = "right";
    context.fillText(`${timeMax.toFixed(1)} s`, chart.width - padding, chart.height - 10);
    context.textAlign = "left";
  }

  function update(snapshot) {
    lastSnapshot = snapshot;
    lastStatus = snapshot.status;
    const status = $("status");
    status.textContent = snapshot.status.toUpperCase();
    status.className = `status ${snapshot.status}`;
    set("sequence", snapshot.sequence);
    set("updated", snapshot.emitted_at_utc);
    if (!snapshot.frame) return;
    const frame = snapshot.frame;
    set("time", fmt(frame.simulated_time_s, 2, " s"));
    set("barrier", fmt(frame.certificate.minimum_barrier_m, 4, " m"));
    set("obstacles", `${frame.obstacles.active_count}/${frame.obstacles.planned_count}`);
    set("intervention", fmt(frame.control.intervention_mps, 4, " m/s"));
    set("linear", fmt(frame.control.linear_mps, 3, " m/s"));
    set("angular", fmt(frame.control.angular_radps, 3, " rad/s"));
    set("step", frame.step_index);
    set("pose", `(${fmt(frame.state.x_m)}, ${fmt(frame.state.y_m)}, ${fmt(frame.state.heading_rad)})`);
    set("lookahead", `(${fmt(frame.state.lookahead_x_m)}, ${fmt(frame.state.lookahead_y_m)}) m`);
    set("contraction", fmt(frame.certificate.minimum_realized_contraction_residual_m, 5, " m"));
    set("newObstacle", frame.obstacles.newly_activated_ids.join(", ") || "none");
    set("termination", frame.termination_reason || "running");
    drawCamera(snapshot);
    drawScene(snapshot);
    if ($("telemetryDetails").open) drawChart(snapshot);
  }

  $("telemetryDetails").addEventListener("toggle", () => {
    if ($("telemetryDetails").open && lastSnapshot) drawChart(lastSnapshot);
  });
  cameraFallback("Waiting for the first Gazebo RGB frame…");

  async function poll() {
    try {
      const response = await fetch(`snapshot.json?t=${Date.now()}`, { cache: "no-store" });
      if (!response.ok) throw new Error(`HTTP ${response.status}`);
      update(await response.json());
    } catch (error) {
      if (!["completed", "failed"].includes(lastStatus)) {
        const status = $("status");
        status.textContent = "DISCONNECTED";
        status.className = "status disconnected";
      }
    } finally {
      window.setTimeout(poll, __REFRESH_MS__);
    }
  }
  poll();
</script>
</body>
</html>
"""


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _json_bytes(payload: Mapping[str, Any], *, pretty: bool) -> bytes:
    return (
        json.dumps(
            payload,
            indent=2 if pretty else None,
            sort_keys=True,
            ensure_ascii=False,
            allow_nan=False,
            separators=None if pretty else (",", ":"),
        )
        + "\n"
    ).encode("utf-8")


def _atomic_write(path: Path, content: bytes) -> None:
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_bytes(content)
    temporary.replace(path)


def _file_sha256(path: Path) -> str:
    digest = sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _number(value: Any) -> float | None:
    return None if value is None else float(value)


class _QuietHandler(SimpleHTTPRequestHandler):
    def log_message(self, format: str, *args: object) -> None:
        del format, args


class _LoopbackServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True


class GazeboLiveDisplay:
    """Publish bounded live telemetry and serve a loopback Canvas viewer."""

    def __init__(
        self,
        *,
        directory: str | Path,
        scenario: DynamicRoadScenario,
        display_stride: int,
        host: str = "127.0.0.1",
        port: int = 0,
        refresh_interval_s: float = 0.20,
        camera_source_path: str | Path | None = None,
        camera_queue_directory: str | Path | None = None,
        camera_spec: GazeboFrontCameraSpec | None = None,
        rover_spec: GazeboPhysicalRoverSpec | None = None,
        state_sample_period_s: float | None = None,
        camera_recording_spec: GazeboCameraRecordingSpec | None = None,
    ) -> None:
        if display_stride <= 0:
            raise ValueError("display_stride must be positive")
        if host not in {"127.0.0.1", "localhost"}:
            raise ValueError("live display must bind to a loopback host")
        if port < 0 or port > 65535:
            raise ValueError("live display port must be between 0 and 65535")
        if not isfinite(refresh_interval_s) or refresh_interval_s <= 0.0:
            raise ValueError("live display refresh interval must be finite and positive")
        camera_values = (
            camera_source_path is not None,
            camera_spec is not None,
            rover_spec is not None,
            state_sample_period_s is not None,
        )
        if any(camera_values) and not all(camera_values):
            raise ValueError(
                "camera source, camera spec, and rover spec must be configured together"
            )
        if state_sample_period_s is not None:
            if (
                not isfinite(state_sample_period_s)
                or state_sample_period_s <= 0.0
            ):
                raise ValueError(
                    "camera state sample period must be finite and positive"
                )
        if camera_recording_spec is not None and camera_spec is None:
            raise ValueError("camera recording requires a configured camera")
        if camera_queue_directory is not None and camera_recording_spec is None:
            raise ValueError("camera queue requires a recording contract")
        if camera_recording_spec is not None and camera_queue_directory is None:
            raise ValueError("camera recording requires its lossless frame queue")
        self.directory = Path(directory).resolve()
        self.scenario = scenario
        self.display_stride = int(display_stride)
        self.host = host
        self.requested_port = int(port)
        self.refresh_interval_s = float(refresh_interval_s)
        self.camera_source_path = (
            None
            if camera_source_path is None
            else Path(camera_source_path).resolve()
        )
        self.camera_queue_directory = (
            None
            if camera_queue_directory is None
            else Path(camera_queue_directory).resolve()
        )
        self.camera_spec = camera_spec
        self.rover_spec = rover_spec
        self.state_sample_period_s = state_sample_period_s
        self.camera_recording_spec = camera_recording_spec
        self.snapshot_path = self.directory / "snapshot.json"
        self.events_path = self.directory / "display_events.jsonl"
        self.viewer_path = self.directory / "index.html"
        self.connection_path = self.directory / "connection.json"
        self.manifest_path = self.directory / "live_display_manifest.json"
        self.camera_frame_path = (
            None
            if camera_spec is None
            else self.directory / Path(camera_spec.frame_file).with_suffix(".png").name
        )
        self.camera_keyframe_directory = self.directory / "camera_keyframes"
        self.camera_recording_directory = self.directory / "camera_recording"
        self.camera_recording_frames_directory = (
            self.camera_recording_directory / "frames"
        )
        self.camera_recording_index_path = (
            self.camera_recording_directory / "frame_index.jsonl"
        )
        self.camera_recording_manifest_path = (
            self.camera_recording_directory / "recording_manifest.json"
        )
        self._history: list[dict[str, Any]] = []
        self._frame_history: list[dict[str, Any]] = []
        self._pending_newly_activated_ids: list[str] = []
        self._pending_camera_keyframe_ids: dict[str, int] = {}
        self._snapshot: dict[str, Any] | None = None
        self._events_stream: Any = None
        self._server: _LoopbackServer | None = None
        self._server_thread: Thread | None = None
        self._sequence = 0
        self._update_count = 0
        self._max_snapshot_bytes = 0
        self._camera_source_signature: tuple[int, int] | None = None
        self._camera_frame_version = 0
        self._camera_published_frame_version = 0
        self._camera_latest_png_content: bytes | None = None
        self._camera_update_count = 0
        self._camera_invalid_frame_count = 0
        self._camera_keyframes: list[dict[str, Any]] = []
        self._camera_recording_records: list[dict[str, Any]] = []
        self._camera_recording_manifest: dict[str, Any] | None = None
        self._camera_queue_consumed_frame_count = 0
        self._camera_capture_simulated_time_s: float | None = None
        self._camera_first_capture_simulated_time_s: float | None = None
        self._camera_last_capture_simulated_time_s: float | None = None
        self._camera_frame_counter: int | None = None
        self._camera_first_frame_counter: int | None = None
        self._camera_alignment_frame: dict[str, Any] | None = None
        self._camera_aligned_update_count = 0
        self._camera_maximum_alignment_error_s = 0.0
        self._started_at_utc: str | None = None
        self._closed = False
        self.url: str | None = None

    def start(self) -> str:
        if self._started_at_utc is not None:
            raise RuntimeError("live display has already started")
        self.directory.mkdir(parents=True, exist_ok=False)
        if self.camera_spec is not None:
            self.camera_keyframe_directory.mkdir()
        if self.camera_recording_spec is not None:
            self.camera_recording_frames_directory.mkdir(parents=True)
        refresh_ms = max(10, round(1000.0 * self.refresh_interval_s))
        viewer_html = _VIEWER_HTML.replace("__REFRESH_MS__", str(refresh_ms))
        self.viewer_path.write_text(viewer_html, encoding="utf-8", newline="\n")
        self._events_stream = self.events_path.open("w", encoding="utf-8", newline="\n")
        self._started_at_utc = _utc_now()
        self._write_snapshot(status="starting", frame=None)
        handler = partial(_QuietHandler, directory=str(self.directory))
        self._server = _LoopbackServer((self.host, self.requested_port), handler)
        actual_host, actual_port = self._server.server_address[:2]
        display_host = "127.0.0.1" if actual_host == "localhost" else actual_host
        self.url = f"http://{display_host}:{actual_port}/"
        connection = {
            "schema_version": 1,
            "kind": "gazebo_live_certificate_display_connection",
            "url": self.url,
            "snapshot_url": f"{self.url}snapshot.json",
            "loopback_only": True,
            "started_at_utc": self._started_at_utc,
        }
        _atomic_write(
            self.connection_path,
            _json_bytes(connection, pretty=True),
        )
        self._server_thread = Thread(
            target=self._server.serve_forever,
            name="cbf-rover-live-display",
            daemon=True,
        )
        self._server_thread.start()
        return self.url

    def publish(self, row: Mapping[str, Any]) -> None:
        if self._started_at_utc is None or self._closed:
            raise RuntimeError("live display is not running")
        frame = self._frame_payload(row)
        self._frame_history.append(frame)
        step_index = int(row["step_index"])
        for obstacle_id in row["obstacles"]["newly_activated_ids"]:
            if obstacle_id not in self._pending_newly_activated_ids:
                self._pending_newly_activated_ids.append(str(obstacle_id))
            self._pending_camera_keyframe_ids.setdefault(
                str(obstacle_id),
                step_index,
            )
        if self.camera_recording_spec is not None:
            self._publish_camera_frame()
        terminal = row.get("termination_reason") is not None
        if step_index % self.display_stride != 0 and not terminal:
            return
        if self.camera_recording_spec is None:
            self._publish_camera_frame()
        self._publish_camera_display_frame()
        frame["obstacles"]["newly_activated_ids"] = list(
            self._pending_newly_activated_ids
        )
        self._pending_newly_activated_ids.clear()
        camera_alignment = self._camera_alignment_payload()
        aligned_active_ids = set(
            []
            if camera_alignment is None
            else camera_alignment["active_obstacle_ids"]
        )
        ready_keyframe_ids = [
            obstacle_id
            for obstacle_id in self._pending_camera_keyframe_ids
            if obstacle_id in aligned_active_ids
        ]
        if ready_keyframe_ids:
            keyframe = self._archive_camera_keyframe(
                step_index=step_index,
                obstacle_ids=ready_keyframe_ids,
                activation_step_indices={
                    obstacle_id: self._pending_camera_keyframe_ids[obstacle_id]
                    for obstacle_id in ready_keyframe_ids
                },
            )
            if keyframe is not None:
                for obstacle_id in ready_keyframe_ids:
                    del self._pending_camera_keyframe_ids[obstacle_id]
                frame["camera_keyframe_file"] = keyframe["frame_file"]
                frame["camera_keyframe_snapshot_file"] = keyframe[
                    "snapshot_file"
                ]
        history_item = {
            "time_s": frame["simulated_time_s"],
            "x_m": frame["state"]["x_m"],
            "y_m": frame["state"]["y_m"],
            "lookahead_x_m": frame["state"]["lookahead_x_m"],
            "lookahead_y_m": frame["state"]["lookahead_y_m"],
            "minimum_barrier_m": frame["certificate"]["minimum_barrier_m"],
            "intervention_mps": frame["control"]["intervention_mps"],
        }
        if not self._history or self._history[-1]["time_s"] != history_item["time_s"]:
            self._history.append(history_item)
        else:
            self._history[-1] = history_item
        status = "completed" if terminal else "running"
        self._write_snapshot(status=status, frame=frame)

    def drain_camera_frames(self) -> None:
        """Consume frames published during the final Gazebo shutdown boundary."""

        if self._started_at_utc is None or self._closed:
            return
        self._publish_camera_frame()

    def fail(self, error: BaseException) -> None:
        if self._started_at_utc is None or self._closed:
            return
        frame = None if self._snapshot is None else self._snapshot.get("frame")
        self._write_snapshot(
            status="failed",
            frame=frame,
            error=f"{type(error).__name__}: {error}"[:500],
        )

    def close(self) -> Path:
        if self._closed:
            return self.manifest_path
        if self._started_at_utc is None:
            raise RuntimeError("live display has not started")
        if self._snapshot is not None and self._snapshot["status"] not in {"completed", "failed"}:
            self._write_snapshot(status="stopped", frame=self._snapshot.get("frame"))
        terminal_grace_s = max(0.30, self.refresh_interval_s + 0.10)
        if self._server is not None:
            sleep(terminal_grace_s)
            self._server.shutdown()
            self._server.server_close()
        if self._server_thread is not None:
            self._server_thread.join(timeout=5.0)
        if self._events_stream is not None:
            self._events_stream.flush()
            self._events_stream.close()
        if self.camera_recording_spec is not None:
            self._finalize_camera_recording()
        self._closed = True
        manifest: dict[str, Any] = {
            "schema_version": 1,
            "kind": "gazebo_live_certificate_display",
            "status": None if self._snapshot is None else self._snapshot["status"],
            "started_at_utc": self._started_at_utc,
            "closed_at_utc": _utc_now(),
            "display_stride": self.display_stride,
            "refresh_interval_s": self.refresh_interval_s,
            "layout": "camera_primary_map_secondary_telemetry_collapsed",
            "camera_certificate_projection": (
                "timestamp_aligned_measured_pose_pinhole_ground_plane"
                if self.camera_spec is not None
                else "none"
            ),
            "update_count": self._update_count,
            "maximum_snapshot_bytes": self._max_snapshot_bytes,
            "final_snapshot": {
                "sequence": (
                    None if self._snapshot is None else self._snapshot["sequence"]
                ),
                "step_index": (
                    None
                    if self._snapshot is None or self._snapshot.get("frame") is None
                    else self._snapshot["frame"]["step_index"]
                ),
                "simulated_time_s": (
                    None
                    if self._snapshot is None or self._snapshot.get("frame") is None
                    else self._snapshot["frame"]["simulated_time_s"]
                ),
            },
            "transport": {
                "protocol": "loopback_http_polling",
                "bind_host": self.host,
                "requested_port": self.requested_port,
                "runtime_url": self.url,
                "terminal_snapshot_grace_s": terminal_grace_s,
                "server_stopped_after_episode": True,
            },
            "semantics": {
                "certificate_source": (
                    "camera_embedded_sim_timestamp_nearest_measured_gazebo_pose"
                ),
                "display_sink_control_feedback": False,
                "display_sink_action_modification": False,
                "online_filter_precedes_display_observation": True,
                "obstacle_display_policy": "visible_after_controller_activation",
                "q_sbc_probability_claimed": False,
                "front_camera_observer_only": True,
            },
            "camera": self._camera_manifest_payload(),
            "artifacts": {
                "viewer_file": self.viewer_path.name,
                "viewer_file_sha256": _file_sha256(self.viewer_path),
                "connection_file": self.connection_path.name,
                "connection_file_sha256": _file_sha256(
                    self.connection_path
                ),
                "snapshot_file": self.snapshot_path.name,
                "snapshot_file_sha256": _file_sha256(self.snapshot_path),
                "events_file": self.events_path.name,
                "events_file_sha256": _file_sha256(self.events_path),
            },
        }
        if self.camera_frame_path is not None and self.camera_frame_path.is_file():
            manifest["artifacts"].update(
                {
                    "camera_frame_file": self.camera_frame_path.name,
                    "camera_frame_file_sha256": _file_sha256(
                        self.camera_frame_path
                    ),
                    "camera_frame_file_bytes": self.camera_frame_path.stat().st_size,
                }
            )
        if self._camera_recording_manifest is not None:
            manifest["artifacts"].update(
                {
                    "camera_recording_manifest_file": str(
                        self.camera_recording_manifest_path.relative_to(
                            self.directory
                        )
                    ),
                    "camera_recording_manifest_file_sha256": _file_sha256(
                        self.camera_recording_manifest_path
                    ),
                    "camera_recording_index_file": str(
                        self.camera_recording_index_path.relative_to(
                            self.directory
                        )
                    ),
                    "camera_recording_index_file_sha256": _file_sha256(
                        self.camera_recording_index_path
                    ),
                }
            )
        manifest["manifest_content_sha256"] = sha256(
            _json_bytes(manifest, pretty=False)
        ).hexdigest()
        _atomic_write(self.manifest_path, _json_bytes(manifest, pretty=True))
        return self.manifest_path

    def manifest_payload(self) -> dict[str, Any]:
        if not self._closed:
            raise RuntimeError("live display must be closed before collecting its manifest")
        return json.loads(self.manifest_path.read_text(encoding="utf-8"))

    def _write_snapshot(
        self,
        *,
        status: str,
        frame: Mapping[str, Any] | None,
        error: str | None = None,
    ) -> None:
        self._sequence += 1
        snapshot: dict[str, Any] = {
            "schema_version": 1,
            "kind": "gazebo_live_certificate_snapshot",
            "status": status,
            "sequence": self._sequence,
            "emitted_at_utc": _utc_now(),
            "error": error,
            "scenario": {
                "scenario_id": self.scenario.scenario_id,
                "road": self._road_payload(),
                "obstacles": self._obstacle_payload(frame),
            },
            "camera": self._camera_payload(),
            "frame": frame,
            "history": list(self._history),
            "runtime_contract": {
                "display_sink_control_feedback": False,
                "certificate_updates_from_measured_state": True,
                "post_projection_command_clipping_applied": False,
            },
        }
        content = _json_bytes(snapshot, pretty=False)
        _atomic_write(self.snapshot_path, content)
        self._max_snapshot_bytes = max(self._max_snapshot_bytes, len(content))
        self._snapshot = snapshot
        if frame is not None and frame.get("camera_keyframe_snapshot_file"):
            replay_snapshot = dict(snapshot)
            replay_snapshot["camera"] = dict(snapshot["camera"])
            replay_snapshot["camera"]["frame_file"] = frame[
                "camera_keyframe_file"
            ]
            replay_path = self.directory / str(
                frame["camera_keyframe_snapshot_file"]
            )
            _atomic_write(
                replay_path,
                _json_bytes(replay_snapshot, pretty=False),
            )
        if frame is not None:
            self._update_count += 1
            event = {
                "schema_version": 1,
                "kind": "gazebo_live_certificate_display_event",
                "sequence": self._sequence,
                "emitted_at_utc": snapshot["emitted_at_utc"],
                "status": status,
                "step_index": frame["step_index"],
                "simulated_time_s": frame["simulated_time_s"],
                "minimum_barrier_m": frame["certificate"]["minimum_barrier_m"],
                "active_obstacle_count": frame["obstacles"]["active_count"],
                "newly_activated_ids": frame["obstacles"]["newly_activated_ids"],
                "display_sink_control_feedback": False,
            }
            assert self._events_stream is not None
            self._events_stream.write(_json_bytes(event, pretty=False).decode("utf-8"))
            self._events_stream.flush()

    def _publish_camera_frame(self) -> None:
        source = self.camera_source_path
        target = self.camera_frame_path
        camera = self.camera_spec
        if source is None or target is None or camera is None:
            return
        if self.camera_queue_directory is not None:
            self._consume_camera_queue()
            return
        try:
            stat = source.stat()
            signature = (int(stat.st_mtime_ns), int(stat.st_size))
            if signature == self._camera_source_signature:
                return
            content = source.read_bytes()
        except OSError:
            return
        self._ingest_camera_content(content, signature=signature)

    def _consume_camera_queue(self) -> None:
        queue = self.camera_queue_directory
        if queue is None or not queue.is_dir():
            return
        for frame_path in sorted(queue.glob("frame_*.bmp")):
            try:
                content = frame_path.read_bytes()
            except OSError:
                break
            if not self._ingest_camera_content(content, signature=None):
                break
            frame_path.unlink()
            self._camera_queue_consumed_frame_count += 1

    def _ingest_camera_content(
        self,
        content: bytes,
        *,
        signature: tuple[int, int] | None,
    ) -> bool:
        camera = self.camera_spec
        if camera is None:
            return False
        metadata = self._camera_bmp_metadata(content, camera=camera)
        if metadata is None:
            self._camera_invalid_frame_count += 1
            return False
        capture_time_s = float(metadata["capture_simulated_time_s"])
        frame_counter = int(metadata["frame_counter"])
        if (
            self._camera_frame_counter is not None
            and frame_counter <= self._camera_frame_counter
        ) or (
            self._camera_last_capture_simulated_time_s is not None
            and capture_time_s
            < self._camera_last_capture_simulated_time_s - 1.0e-12
        ):
            self._camera_invalid_frame_count += 1
            return False
        alignment_frame = self._nearest_frame_for_camera_time(capture_time_s)
        if alignment_frame is None:
            self._camera_invalid_frame_count += 1
            return False
        try:
            from PIL import Image

            with Image.open(BytesIO(content)) as image:
                image.load()
                converted = image.convert("RGB")
                stream = BytesIO()
                converted.save(stream, format="PNG", compress_level=1)
                png_content = stream.getvalue()
        except Exception:
            self._camera_invalid_frame_count += 1
            return False
        if signature is not None:
            self._camera_source_signature = signature
        self._camera_frame_version += 1
        self._camera_update_count += 1
        self._camera_aligned_update_count += 1
        self._camera_capture_simulated_time_s = capture_time_s
        self._camera_last_capture_simulated_time_s = capture_time_s
        self._camera_frame_counter = frame_counter
        self._camera_alignment_frame = alignment_frame
        if self._camera_first_capture_simulated_time_s is None:
            self._camera_first_capture_simulated_time_s = capture_time_s
            self._camera_first_frame_counter = frame_counter
        alignment_error_s = abs(
            float(alignment_frame["simulated_time_s"]) - capture_time_s
        )
        self._camera_maximum_alignment_error_s = max(
            self._camera_maximum_alignment_error_s,
            alignment_error_s,
        )
        if self.camera_recording_spec is not None:
            self._archive_camera_recording_frame(png_content)
        self._camera_latest_png_content = png_content
        return True

    def _publish_camera_display_frame(self) -> None:
        target = self.camera_frame_path
        content = self._camera_latest_png_content
        if target is None or content is None:
            return
        if self._camera_published_frame_version == self._camera_frame_version:
            return
        _atomic_write(target, content)
        self._camera_published_frame_version = self._camera_frame_version

    def _nearest_frame_for_camera_time(
        self,
        capture_time_s: float,
    ) -> dict[str, Any] | None:
        if not self._frame_history:
            return None
        return min(
            self._frame_history,
            key=lambda frame: abs(
                float(frame["simulated_time_s"]) - capture_time_s
            ),
        )

    def _archive_camera_recording_frame(self, png_content: bytes) -> None:
        alignment = self._camera_alignment_payload()
        frame = self._camera_alignment_frame
        if alignment is None or frame is None:
            raise RuntimeError("camera recording requires an aligned measured state")
        sequence_index = len(self._camera_recording_records) + 1
        frame_relative = f"frames/frame_{sequence_index:06d}.png"
        frame_path = self.camera_recording_directory / frame_relative
        _atomic_write(frame_path, png_content)
        record = {
            "schema_version": 1,
            "kind": "gazebo_timestamp_aligned_camera_frame",
            "sequence_index": sequence_index,
            "frame_file": frame_relative,
            "frame_file_sha256": sha256(png_content).hexdigest(),
            "frame_file_bytes": len(png_content),
            "frame_counter": self._camera_frame_counter,
            "capture_simulated_time_s": self._camera_capture_simulated_time_s,
            "alignment": alignment,
            "minimum_barrier_m": float(
                frame["certificate"]["minimum_barrier_m"]
            ),
            "termination_reason": frame.get("termination_reason"),
        }
        self._camera_recording_records.append(record)

    def _finalize_camera_recording(self) -> None:
        if self._camera_recording_manifest is not None:
            return
        recording = self.camera_recording_spec
        camera = self.camera_spec
        rover = self.rover_spec
        if recording is None or camera is None or rover is None:
            raise RuntimeError("camera recording contract is incomplete")
        index_content = b"".join(
            _json_bytes(record, pretty=False)
            for record in self._camera_recording_records
        )
        _atomic_write(self.camera_recording_index_path, index_content)
        capture_times = [
            float(record["capture_simulated_time_s"])
            for record in self._camera_recording_records
        ]
        frame_counters = [
            int(record["frame_counter"])
            for record in self._camera_recording_records
        ]
        gaps = [
            later - earlier
            for earlier, later in zip(capture_times, capture_times[1:])
        ]
        sorted_gaps = sorted(gaps)
        median_gap_s = (
            None
            if not sorted_gaps
            else (
                sorted_gaps[len(sorted_gaps) // 2]
                if len(sorted_gaps) % 2
                else 0.5
                * (
                    sorted_gaps[len(sorted_gaps) // 2 - 1]
                    + sorted_gaps[len(sorted_gaps) // 2]
                )
            )
        )
        missing_frame_count = sum(
            max(0, later - earlier - 1)
            for earlier, later in zip(frame_counters, frame_counters[1:])
        )
        final_simulated_time_s = (
            0.0
            if not self._frame_history
            else float(self._frame_history[-1]["simulated_time_s"])
        )
        capture_span_s = (
            0.0
            if len(capture_times) < 2
            else capture_times[-1] - capture_times[0]
        )
        capture_coverage_ratio = (
            0.0
            if final_simulated_time_s <= 0.0
            else min(1.0, capture_span_s / final_simulated_time_s)
        )
        timestamps_monotonic_nondecreasing = all(gap >= 0.0 for gap in gaps)
        frame_counter_strictly_increasing = all(
            later > earlier
            for earlier, later in zip(frame_counters, frame_counters[1:])
        )
        frame_hash_closure = all(
            (
                self.camera_recording_directory / str(record["frame_file"])
            ).is_file()
            and _file_sha256(
                self.camera_recording_directory / str(record["frame_file"])
            )
            == record["frame_file_sha256"]
            for record in self._camera_recording_records
        )
        maximum_gap_s = None if not gaps else max(gaps)
        queued_frames_remaining = (
            []
            if self.camera_queue_directory is None
            else sorted(self.camera_queue_directory.glob("frame_*.bmp"))
        )
        queued_temporary_files = (
            []
            if self.camera_queue_directory is None
            else sorted(self.camera_queue_directory.glob("*.tmp"))
        )
        gates = {
            "source_frames_present": bool(self._camera_recording_records),
            "source_frame_hash_closure": frame_hash_closure,
            "source_timestamps_monotonic_nondecreasing": (
                timestamps_monotonic_nondecreasing
            ),
            "source_frame_counters_strictly_increasing": (
                frame_counter_strictly_increasing
            ),
            "source_frame_counters_contiguous": missing_frame_count == 0,
            "source_frame_gap_within_bound": (
                maximum_gap_s is not None
                and maximum_gap_s
                <= recording.maximum_source_frame_gap_s + 1.0e-12
            ),
            "source_capture_coverage": (
                capture_coverage_ratio
                >= recording.minimum_capture_coverage_ratio - 1.0e-12
            ),
            "all_source_frames_timestamp_aligned": all(
                record.get("alignment") is not None
                for record in self._camera_recording_records
            ),
            "camera_queue_fully_consumed": not queued_frames_remaining,
            "camera_queue_has_no_temporary_files": (
                not queued_temporary_files
            ),
        }
        mount = camera.pose_on_base_link
        manifest: dict[str, Any] = {
            "schema_version": 1,
            "kind": "gazebo_timestamp_aligned_camera_recording",
            "status": "completed" if all(gates.values()) else "failed",
            "capture_policy": recording.capture_policy,
            "archive_format": recording.archive_format,
            "observer_only": True,
            "frame_count": len(self._camera_recording_records),
            "first_frame_counter": (
                None if not frame_counters else frame_counters[0]
            ),
            "last_frame_counter": (
                None if not frame_counters else frame_counters[-1]
            ),
            "missing_frame_count": missing_frame_count,
            "camera_queue_consumed_frame_count": (
                self._camera_queue_consumed_frame_count
            ),
            "camera_queue_remaining_frame_count": len(
                queued_frames_remaining
            ),
            "camera_queue_temporary_file_count": len(
                queued_temporary_files
            ),
            "first_capture_simulated_time_s": (
                None if not capture_times else capture_times[0]
            ),
            "last_capture_simulated_time_s": (
                None if not capture_times else capture_times[-1]
            ),
            "capture_span_s": capture_span_s,
            "run_simulated_duration_s": final_simulated_time_s,
            "capture_coverage_ratio": capture_coverage_ratio,
            "median_source_frame_gap_s": median_gap_s,
            "maximum_source_frame_gap_s": maximum_gap_s,
            "maximum_alignment_error_s": (
                None
                if not self._camera_recording_records
                else max(
                    float(record["alignment"]["absolute_time_error_s"])
                    for record in self._camera_recording_records
                )
            ),
            "index_file": self.camera_recording_index_path.name,
            "index_file_sha256": _file_sha256(
                self.camera_recording_index_path
            ),
            "camera": {
                "image_width": camera.image_width,
                "image_height": camera.image_height,
                "horizontal_fov_rad": camera.horizontal_fov_rad,
                "near_clip_m": camera.near_clip_m,
                "mount_x_m": mount[0],
                "mount_y_m": mount[1],
                "pitch_rad": mount[4],
                "world_height_m": (
                    rover.wheel_radius_m
                    + rover.base_height_m / 2.0
                    + mount[2]
                ),
            },
            "scenario": {
                "scenario_id": self.scenario.scenario_id,
                "road": self._road_payload(),
                "obstacles": [
                    {
                        "id": obstacle.obstacle_id,
                        "x_m": obstacle.center.x_m,
                        "y_m": obstacle.center.y_m,
                        "physical_radius_m": obstacle.physical_radius_m,
                        "certificate_radius_m": obstacle.certificate_radius_m,
                        "activation_x_m": obstacle.activation_x_m,
                    }
                    for obstacle in self.scenario.obstacles
                ],
            },
            "playback": {
                "fps": recording.playback_fps,
                "width": recording.video_width,
                "height": recording.video_height,
                "video_file": recording.video_file,
                "encoder_backend": recording.encoder_backend,
                "bitrate_kbps": recording.bitrate_kbps,
                "minimum_overlay_visible_frame_count": (
                    recording.minimum_overlay_visible_frame_count
                ),
                "require_all_activated_obstacles_visible": (
                    recording.require_all_activated_obstacles_visible
                ),
                "playback_rate": "1x_simulated_time",
                "interpolation": "none",
            },
            "gates": gates,
        }
        manifest["manifest_content_sha256"] = sha256(
            _json_bytes(manifest, pretty=False)
        ).hexdigest()
        _atomic_write(
            self.camera_recording_manifest_path,
            _json_bytes(manifest, pretty=True),
        )
        self._camera_recording_manifest = manifest

    def _archive_camera_keyframe(
        self,
        *,
        step_index: int,
        obstacle_ids: list[str],
        activation_step_indices: Mapping[str, int],
    ) -> dict[str, Any] | None:
        source = self.camera_frame_path
        if source is None or not source.is_file():
            return None
        suffix = "_".join(
            obstacle_id.replace("obstacle_", "o")
            for obstacle_id in obstacle_ids
        )
        basename = f"step_{step_index:04d}_{suffix}"
        frame_relative = f"camera_keyframes/{basename}.png"
        snapshot_relative = f"camera_keyframes/{basename}.snapshot.json"
        destination = self.directory / frame_relative
        _atomic_write(destination, source.read_bytes())
        record = {
            "step_index": step_index,
            "obstacle_ids": list(obstacle_ids),
            "activation_step_index_by_obstacle": {
                obstacle_id: int(activation_step_indices[obstacle_id])
                for obstacle_id in obstacle_ids
            },
            "frame_file": frame_relative,
            "snapshot_file": snapshot_relative,
            "camera_frame_version": self._camera_frame_version,
            "camera_frame_counter": self._camera_frame_counter,
            "camera_capture_simulated_time_s": (
                self._camera_capture_simulated_time_s
            ),
            "camera_alignment": self._camera_alignment_payload(),
        }
        self._camera_keyframes.append(record)
        return record

    def _camera_keyframe_manifest(self) -> list[dict[str, Any]]:
        payload: list[dict[str, Any]] = []
        for record in self._camera_keyframes:
            frame_path = self.directory / record["frame_file"]
            snapshot_path = self.directory / record["snapshot_file"]
            payload.append(
                {
                    **record,
                    "frame_file_sha256": (
                        _file_sha256(frame_path)
                        if frame_path.is_file()
                        else None
                    ),
                    "snapshot_file_sha256": (
                        _file_sha256(snapshot_path)
                        if snapshot_path.is_file()
                        else None
                    ),
                }
            )
        return payload

    @staticmethod
    def _valid_camera_bmp(
        content: bytes,
        *,
        camera: GazeboFrontCameraSpec,
    ) -> bool:
        if len(content) < 54 or content[:2] != b"BM":
            return False
        width = int.from_bytes(content[18:22], "little", signed=True)
        height = int.from_bytes(content[22:26], "little", signed=True)
        bits_per_pixel = int.from_bytes(content[28:30], "little")
        declared_size = int.from_bytes(content[2:6], "little")
        return (
            width == camera.image_width
            and height == camera.image_height
            and bits_per_pixel == 24
            and declared_size == len(content)
        )

    @staticmethod
    def _camera_bmp_metadata(
        content: bytes,
        *,
        camera: GazeboFrontCameraSpec,
    ) -> dict[str, int | float] | None:
        if not GazeboLiveDisplay._valid_camera_bmp(content, camera=camera):
            return None
        pixel_offset = int.from_bytes(content[10:14], "little")
        row_bytes = camera.image_width * 3
        row_stride = (row_bytes + 3) & ~3
        metadata_offset = pixel_offset + row_stride * camera.image_height
        if pixel_offset != 54 or metadata_offset >= len(content):
            return None
        try:
            fields = content[metadata_offset:].decode("ascii").strip().split()
            if len(fields) != 4 or fields[0] != "CBF_ROVER_CAMERA_FRAME_V1":
                return None
            seconds = int(fields[1])
            nanoseconds = int(fields[2])
            frame_counter = int(fields[3])
        except (UnicodeDecodeError, ValueError):
            return None
        if seconds < 0 or not 0 <= nanoseconds < 1_000_000_000:
            return None
        if frame_counter <= 0:
            return None
        capture_time_s = seconds + nanoseconds * 1.0e-9
        if not isfinite(capture_time_s):
            return None
        return {
            "capture_simulated_time_s": capture_time_s,
            "seconds": seconds,
            "nanoseconds": nanoseconds,
            "frame_counter": frame_counter,
        }

    def _camera_payload(self) -> dict[str, Any]:
        camera = self.camera_spec
        rover = self.rover_spec
        if camera is None or rover is None or self.camera_frame_path is None:
            return {
                "configured": False,
                "available": False,
                "observer_only": True,
            }
        mount = camera.pose_on_base_link
        return {
            "configured": True,
            "available": self.camera_frame_path.is_file(),
            "observer_only": True,
            "frame_file": self.camera_frame_path.name,
            "frame_version": self._camera_published_frame_version,
            "frame_counter": self._camera_frame_counter,
            "capture_simulated_time_s": (
                self._camera_capture_simulated_time_s
            ),
            "image_width": camera.image_width,
            "image_height": camera.image_height,
            "horizontal_fov_rad": camera.horizontal_fov_rad,
            "near_clip_m": camera.near_clip_m,
            "mount_x_m": mount[0],
            "mount_y_m": mount[1],
            "pitch_rad": mount[4],
            "world_height_m": (
                rover.wheel_radius_m
                + rover.base_height_m / 2.0
                + mount[2]
            ),
            "transport": camera.transport,
            "projection": (
                "timestamp_aligned_measured_pose_pinhole_ground_plane"
            ),
            "alignment": self._camera_alignment_payload(),
        }

    def _camera_alignment_payload(self) -> dict[str, Any] | None:
        frame = self._camera_alignment_frame
        capture_time_s = self._camera_capture_simulated_time_s
        if frame is None or capture_time_s is None:
            return None
        barrier_by_id = {
            str(item["barrier_id"]).split(":", 1)[1]: float(item["barrier_m"])
            for item in frame["certificate"]["constraints"]
            if item["barrier_kind"] == "obstacle"
        }
        aligned_time_s = float(frame["simulated_time_s"])
        return {
            "policy": "nearest_measured_state_by_embedded_gazebo_sim_timestamp",
            "capture_simulated_time_s": capture_time_s,
            "aligned_step_index": int(frame["step_index"]),
            "aligned_simulated_time_s": aligned_time_s,
            "absolute_time_error_s": abs(aligned_time_s - capture_time_s),
            "state_sample_period_s": self.state_sample_period_s,
            "state": dict(frame["state"]),
            "active_obstacle_ids": list(frame["obstacles"]["active_ids"]),
            "obstacle_barrier_m": barrier_by_id,
            "additional_body_inflation_m": float(
                frame["certificate"]["additional_body_inflation_m"]
            ),
        }

    def _camera_manifest_payload(self) -> dict[str, Any]:
        capture_span_s = 0.0
        if (
            self._camera_first_capture_simulated_time_s is not None
            and self._camera_last_capture_simulated_time_s is not None
        ):
            capture_span_s = max(
                0.0,
                self._camera_last_capture_simulated_time_s
                - self._camera_first_capture_simulated_time_s,
            )
        final_simulated_time_s = 0.0
        if self._snapshot is not None and self._snapshot.get("frame") is not None:
            final_simulated_time_s = float(
                self._snapshot["frame"]["simulated_time_s"]
            )
        coverage_ratio = (
            0.0
            if final_simulated_time_s <= 0.0
            else min(1.0, capture_span_s / final_simulated_time_s)
        )
        return {
            "configured": self.camera_spec is not None,
            "source_path": (
                None
                if self.camera_source_path is None
                else str(self.camera_source_path)
            ),
            "frame_file": (
                None
                if self.camera_frame_path is None
                else self.camera_frame_path.name
            ),
            "transport": (
                None if self.camera_spec is None else self.camera_spec.transport
            ),
            "update_count": self._camera_update_count,
            "aligned_update_count": self._camera_aligned_update_count,
            "invalid_frame_count": self._camera_invalid_frame_count,
            "first_frame_counter": self._camera_first_frame_counter,
            "last_frame_counter": self._camera_frame_counter,
            "first_capture_simulated_time_s": (
                self._camera_first_capture_simulated_time_s
            ),
            "last_capture_simulated_time_s": (
                self._camera_last_capture_simulated_time_s
            ),
            "capture_span_s": capture_span_s,
            "capture_coverage_ratio": coverage_ratio,
            "maximum_alignment_error_s": (
                None
                if self._camera_aligned_update_count == 0
                else self._camera_maximum_alignment_error_s
            ),
            "state_sample_period_s": self.state_sample_period_s,
            "final_frame_available": bool(
                self.camera_frame_path is not None
                and self.camera_frame_path.is_file()
            ),
            "activation_keyframes": self._camera_keyframe_manifest(),
            "pending_activation_keyframe_obstacle_ids": list(
                self._pending_camera_keyframe_ids
            ),
            "recording": (
                None
                if self._camera_recording_manifest is None
                else {
                    "directory": self.camera_recording_directory.name,
                    "manifest_file": (
                        self.camera_recording_manifest_path.name
                    ),
                    "manifest_file_sha256": _file_sha256(
                        self.camera_recording_manifest_path
                    ),
                    "manifest": self._camera_recording_manifest,
                }
            ),
        }

    def _road_payload(self) -> dict[str, Any]:
        geometry = self.scenario.geometry
        return {
            "x_min_m": geometry.x_min_m,
            "x_max_m": geometry.x_max_m,
            "physical_y_min_m": geometry.physical_y_min_m,
            "physical_y_max_m": geometry.physical_y_max_m,
            "lane_centers_y_m": list(geometry.lane_centers_y_m),
            "finish_line_x_m": geometry.finish_line_x_m,
            "rover_radius_m": geometry.rover_radius_m,
        }

    def _obstacle_payload(
        self,
        frame: Mapping[str, Any] | None,
    ) -> list[dict[str, Any]]:
        active_ids: set[str] = set()
        barrier_by_id: dict[str, float] = {}
        inflation = 0.0
        if frame is not None:
            active_ids = set(frame["obstacles"]["active_ids"])
            inflation = float(frame["certificate"]["additional_body_inflation_m"])
            barrier_by_id = {
                str(item["barrier_id"]).split(":", 1)[1]: float(item["barrier_m"])
                for item in frame["certificate"]["constraints"]
                if item["barrier_kind"] == "obstacle"
            }
        return [
            {
                "id": obstacle.obstacle_id,
                "x_m": obstacle.center.x_m,
                "y_m": obstacle.center.y_m,
                "physical_radius_m": obstacle.physical_radius_m,
                "lifted_radius_m": obstacle.certificate_radius_m + inflation,
                "activation_x_m": obstacle.activation_x_m,
                "active": obstacle.obstacle_id in active_ids,
                "barrier_m": barrier_by_id.get(obstacle.obstacle_id),
            }
            for obstacle in self.scenario.obstacles
        ]

    @staticmethod
    def _frame_payload(row: Mapping[str, Any]) -> dict[str, Any]:
        state = row["state"]
        action = row["action"]
        certificate = row["certificate"]
        return {
            "step_index": int(row["step_index"]),
            "simulated_time_s": float(row["simulated_time_s"]),
            "termination_reason": row["termination_reason"],
            "state": {
                "x_m": float(state["x_m"]),
                "y_m": float(state["y_m"]),
                "heading_rad": float(state["heading_rad"]),
                "lookahead_x_m": float(state["lookahead_point_m"][0]),
                "lookahead_y_m": float(state["lookahead_point_m"][1]),
            },
            "obstacles": {
                "active_ids": list(row["obstacles"]["active_ids"]),
                "newly_activated_ids": list(row["obstacles"]["newly_activated_ids"]),
                "active_count": int(row["obstacles"]["active_count"]),
                "planned_count": int(row["obstacles"]["planned_count"]),
            },
            "certificate": {
                "certificate_id": certificate["certificate_id"],
                "family": certificate["family"],
                "additional_body_inflation_m": float(
                    certificate["additional_body_inflation_m"]
                ),
                "minimum_barrier_m": float(certificate["minimum_barrier_m"]),
                "minimum_active_obstacle_barrier_m": _number(
                    certificate["minimum_active_obstacle_barrier_m"]
                ),
                "minimum_constraint_residual_mps": _number(
                    certificate["minimum_constraint_residual_mps"]
                ),
                "minimum_realized_contraction_residual_m": _number(
                    certificate["minimum_realized_contraction_residual_m"]
                ),
                "constraints": list(certificate["constraints"]),
                "q_sbc_probability_bound_claimed": False,
            },
            "control": {
                "linear_mps": (
                    None
                    if action is None
                    else float(action["differential_command"]["linear_mps"])
                ),
                "angular_radps": (
                    None
                    if action is None
                    else float(action["differential_command"]["angular_radps"])
                ),
                "intervention_mps": (
                    None if action is None else float(action["intervention_norm_mps"])
                ),
                "display_sink_modified_action": False,
            },
            "runtime": {
                "backend": row["runtime"]["backend"],
                "certificate_mode": row["runtime"]["certificate_mode"],
                "display_sink_control_feedback": False,
                "post_projection_command_clipping_applied": False,
            },
        }
