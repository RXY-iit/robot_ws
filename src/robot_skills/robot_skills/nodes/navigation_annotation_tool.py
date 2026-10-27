from __future__ import annotations

import argparse
import json
import math
import os
import shutil
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

import yaml


WORKSPACE_ROOT = Path(os.environ.get("ROBOT_WS", "/home/matsunaga-h/robot_ws"))
DEFAULT_MAP_YAML = WORKSPACE_ROOT / "maps" / "l402_2d_map_clean_0509.yaml"
DEFAULT_ANNOTATION_YAML = WORKSPACE_ROOT / "maps" / "l402_2d_map_clean_0509" / "navigation_annotations.yaml"


def _bundle_dir_from_env() -> Path | None:
    map_id = os.environ.get("ROBOT_MAP_ID", "").strip()
    if not map_id:
        return None
    if map_id == "real_lab":
        map_id = "l402_2d_map_clean_0509"
    bundle_dir = WORKSPACE_ROOT / "maps" / map_id
    return bundle_dir if bundle_dir.exists() else None


def _default_map_yaml() -> Path:
    bundle_dir = _bundle_dir_from_env()
    if bundle_dir and (bundle_dir / "nav2.yaml").exists():
        return bundle_dir / "nav2.yaml"
    return DEFAULT_MAP_YAML


def _default_annotation_yaml() -> Path:
    bundle_dir = _bundle_dir_from_env()
    if bundle_dir and (bundle_dir / "navigation_annotations.yaml").exists():
        return bundle_dir / "navigation_annotations.yaml"
    return DEFAULT_ANNOTATION_YAML


def _load_yaml(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def _write_yaml(path: Path, data: dict[str, Any]):
    if path.exists():
        shutil.copy2(path, path.with_suffix(path.suffix + ".bak"))
    with path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)


def _map_image_path(map_yaml_path: Path, map_data: dict[str, Any]) -> Path:
    image = map_data.get("image", "")
    path = Path(image)
    if not path.is_absolute():
        path = map_yaml_path.parent / path
    return path


def _upsert_zone(zones: list[dict[str, Any]], zone: dict[str, Any]) -> list[dict[str, Any]]:
    zone_id = zone.get("id")
    for index, existing in enumerate(zones):
        if existing.get("id") == zone_id:
            zones[index] = zone
            return zones
    zones.append(zone)
    return zones


def _normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


class NavigationAnnotationToolServer:
    def __init__(self, map_yaml: Path, annotation_yaml: Path):
        self.map_yaml = map_yaml
        self.annotation_yaml = annotation_yaml
        self.map_data = _load_yaml(map_yaml)
        self.map_image = _map_image_path(map_yaml, self.map_data)

    def state(self) -> dict[str, Any]:
        annotations = _load_yaml(self.annotation_yaml)
        return {
            "map": {
                "yaml_path": str(self.map_yaml),
                "image_path": str(self.map_image),
                "resolution": float(self.map_data.get("resolution", 0.05)),
                "origin": self.map_data.get("origin", [0.0, 0.0, 0.0]),
            },
            "annotation_yaml": str(self.annotation_yaml),
            "annotations": annotations,
            "narrow_passages": annotations.get("narrow_passages", []),
        }

    def save_alignment_zone(self, payload: dict[str, Any]) -> dict[str, Any]:
        passage_id = str(payload.get("passage_id") or "").strip()
        zone_id = str(payload.get("id") or "").strip()
        if not passage_id:
            raise ValueError("passage_id is required")
        if not zone_id:
            raise ValueError("alignment zone id is required")

        annotations = _load_yaml(self.annotation_yaml)
        annotations.setdefault("map_id", os.environ.get("ROBOT_MAP_ID", "l402_2d_map_clean_0509") or "l402_2d_map_clean_0509")
        annotations.setdefault("frame_id", "map")
        passages = annotations.setdefault("narrow_passages", [])
        passage = next((item for item in passages if item.get("id") == passage_id), None)
        if passage is None:
            raise ValueError(f"passage not found: {passage_id}")

        polygon = payload.get("polygon") or []
        pose = payload.get("pose") or {}
        if len(polygon) < 3:
            raise ValueError("alignment zone polygon needs at least 3 points")
        if not {"x", "y", "yaw"}.issubset(pose):
            raise ValueError("pose requires x, y, yaw")

        zone = {
            "id": zone_id,
            "enabled": bool(payload.get("enabled", False)),
            "side": str(payload.get("side") or "entry"),
            "type": "navigation_alignment_zone",
            "description": str(payload.get("description") or ""),
            "polygon": [
                {"x": float(point["x"]), "y": float(point["y"])}
                for point in polygon
                if "x" in point and "y" in point
            ],
            "pose": {
                "x": float(pose["x"]),
                "y": float(pose["y"]),
                "yaw": _normalize_angle(float(pose["yaw"])),
            },
            "xy_goal_tolerance_m": float(payload.get("xy_goal_tolerance_m", 0.25)),
            "yaw_goal_tolerance_rad": float(payload.get("yaw_goal_tolerance_rad", 0.35)),
            "timeout_sec": float(payload.get("timeout_sec", 16.0)),
            "use_policy": {
                "skip_if_robot_near_pose_m": float(payload.get("skip_if_robot_near_pose_m", 0.12)),
            },
        }
        passage["alignment_zones"] = _upsert_zone(passage.get("alignment_zones") or [], zone)
        path_updated = _calibrate_passage_path_from_alignment_zones(passage)
        _write_yaml(self.annotation_yaml, annotations)
        return {
            "ok": True,
            "passage_id": passage_id,
            "zone_id": zone_id,
            "path_updated": path_updated,
            "annotation_yaml": str(self.annotation_yaml),
        }


def _make_handler(app: NavigationAnnotationToolServer):
    class Handler(BaseHTTPRequestHandler):
        def _send_json(self, data: dict[str, Any], status: int = 200):
            body = json.dumps(data, ensure_ascii=False, indent=2).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = urlparse(self.path).path
            if path == "/":
                body = HTML.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            if path == "/api/state":
                self._send_json(app.state())
                return
            if path == "/map.pgm":
                if not app.map_image.exists():
                    self._send_json({"error": f"map image not found: {app.map_image}"}, 404)
                    return
                body = app.map_image.read_bytes()
                self.send_response(200)
                self.send_header("Content-Type", "application/octet-stream")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            self._send_json({"error": "not found"}, 404)

        def do_POST(self):
            path = urlparse(self.path).path
            if path != "/api/save_alignment_zone":
                self._send_json({"error": "not found"}, 404)
                return
            try:
                length = int(self.headers.get("Content-Length", "0"))
                payload = json.loads(self.rfile.read(length).decode("utf-8"))
                result = app.save_alignment_zone(payload)
            except Exception as exc:
                self._send_json({"ok": False, "error": str(exc)}, 400)
                return
            self._send_json(result)

        def log_message(self, fmt, *args):
            print(f"[navigation_annotation_tool] {self.address_string()} - {fmt % args}")

    return Handler


def _calibrate_passage_path_from_alignment_zones(passage: dict[str, Any]) -> bool:
    zones = passage.get("alignment_zones") or []
    entry_zone = next(
        (
            zone
            for zone in zones
            if zone.get("enabled", False)
            and str(zone.get("side") or "").lower() == "entry"
            and isinstance(zone.get("pose"), dict)
        ),
        None,
    )
    exit_zone = next(
        (
            zone
            for zone in zones
            if zone.get("enabled", False)
            and str(zone.get("side") or "").lower() == "exit"
            and isinstance(zone.get("pose"), dict)
        ),
        None,
    )
    if entry_zone is None or exit_zone is None:
        return False
    entry_pose = entry_zone["pose"]
    exit_pose = exit_zone["pose"]
    if not {"x", "y"}.issubset(entry_pose) or not {"x", "y"}.issubset(exit_pose):
        return False
    entry_x = float(entry_pose["x"])
    entry_y = float(entry_pose["y"])
    exit_x = float(exit_pose["x"])
    exit_y = float(exit_pose["y"])
    yaw = _normalize_angle(math.atan2(exit_y - entry_y, exit_x - entry_x))
    passage["entry"] = {"x": entry_x, "y": entry_y, "yaw": yaw}
    passage["exit"] = {"x": exit_x, "y": exit_y, "yaw": yaw}
    passage["centerline"] = [{"x": entry_x, "y": entry_y}, {"x": exit_x, "y": exit_y}]
    return True


HTML = r"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>Navigation Annotation Tool</title>
  <style>
    :root { color-scheme: light; font-family: Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; }
    body { margin: 0; background: #f4f6f8; color: #17202a; }
    main { display: grid; grid-template-columns: minmax(560px, 1fr) 420px; height: 100vh; }
    #mapWrap { position: relative; overflow: hidden; background: #c7ccd2; }
    canvas { display: block; width: 100%; height: 100%; cursor: crosshair; }
    aside { border-left: 1px solid #cbd3dc; background: #fff; padding: 14px; overflow: auto; }
    h1 { margin: 0 0 10px; font-size: 18px; }
    h2 { margin: 16px 0 8px; font-size: 13px; color: #53606b; text-transform: uppercase; letter-spacing: .04em; }
    label { display: grid; gap: 4px; margin: 8px 0; font-size: 13px; color: #3b4650; }
    input, select, textarea, button { font: inherit; }
    input, select, textarea { width: 100%; box-sizing: border-box; border: 1px solid #bcc7d2; border-radius: 6px; padding: 7px 8px; background: #fff; }
    textarea { min-height: 58px; resize: vertical; }
    button { border: 1px solid #aeb9c4; background: #f7f9fb; border-radius: 6px; padding: 7px 10px; cursor: pointer; }
    button.primary { background: #1f6feb; color: white; border-color: #1f6feb; }
    button.danger { color: #a42323; }
    .row { display: flex; gap: 8px; align-items: center; }
    .row > * { flex: 1; }
    .toolbar { position: absolute; left: 12px; top: 12px; display: flex; gap: 6px; padding: 6px; background: rgba(255,255,255,.92); border: 1px solid #cbd3dc; border-radius: 8px; box-shadow: 0 2px 10px rgba(30,40,50,.12); }
    .status { position: absolute; left: 12px; bottom: 12px; padding: 6px 8px; background: rgba(255,255,255,.92); border: 1px solid #cbd3dc; border-radius: 8px; font-size: 12px; }
    pre { max-height: 230px; overflow: auto; background: #f1f4f7; border: 1px solid #d8e0e8; border-radius: 6px; padding: 8px; font-size: 12px; }
    .hint { font-size: 12px; color: #64717d; line-height: 1.45; }
    .pill { display: inline-block; border: 1px solid #cbd3dc; border-radius: 999px; padding: 2px 7px; margin: 2px; font-size: 12px; background: #f8fafc; }
  </style>
</head>
<body>
<main>
  <section id="mapWrap">
    <canvas id="mapCanvas"></canvas>
    <div class="toolbar">
      <button id="modePolygon" class="primary">Polygon</button>
      <button id="modePose">Pose</button>
      <button id="undo">Undo</button>
      <button id="clear" class="danger">Clear polygon</button>
    </div>
    <div class="status" id="cursor">loading map...</div>
  </section>
  <aside>
    <h1>Navigation Annotation Tool</h1>
    <div class="hint">
      这个工具只编辑 navigation_annotations.yaml。alignment zone 是导航功能区域，不会写入 semantic object/place。
      Polygon 表示可转身开放区域，Pose 表示推荐站位和正对 narrow path 的 yaw。
    </div>

    <h2>Passage / Zone</h2>
    <label>Narrow passage <select id="passageSelect"></select></label>
    <label>Alignment zone <select id="zoneSelect"></select></label>
    <div class="row">
      <button id="newEntry">New entry zone</button>
      <button id="newExit">New exit zone</button>
    </div>
    <label>ID <input id="zoneId" /></label>
    <label>Description <textarea id="description"></textarea></label>
    <div class="row">
      <label>Side
        <select id="side">
          <option value="entry">entry</option>
          <option value="exit">exit</option>
        </select>
      </label>
      <label>Enabled
        <select id="enabled">
          <option value="false">false</option>
          <option value="true">true</option>
        </select>
      </label>
    </div>

    <h2>Pose</h2>
    <div class="row">
      <label>x <input id="poseX" type="number" step="0.01" /></label>
      <label>y <input id="poseY" type="number" step="0.01" /></label>
    </div>
    <div class="row">
      <label>yaw deg <input id="yawDeg" type="number" step="1" /></label>
      <button id="facePassage">Face passage</button>
    </div>

    <h2>Tolerance</h2>
    <div class="row">
      <label>xy tol m <input id="xyTol" type="number" value="0.25" step="0.01" /></label>
      <label>yaw tol rad <input id="yawTol" type="number" value="0.35" step="0.01" /></label>
    </div>
    <div class="row">
      <label>timeout sec <input id="timeoutSec" type="number" value="16" step="1" /></label>
      <label>skip near m <input id="skipNear" type="number" value="0.12" step="0.01" /></label>
    </div>

    <h2>Current Polygon</h2>
    <div id="points"></div>
    <pre id="snippet"></pre>

    <div class="row">
      <button id="save" class="primary">Save alignment zone</button>
      <button id="reload">Reload</button>
    </div>
  </aside>
</main>
<script>
const canvas = document.getElementById('mapCanvas');
const ctx = canvas.getContext('2d');
let state = null, mapBitmap = null, mapImageData = null;
let imgW = 0, imgH = 0;
let mode = 'polygon';
let polygon = [];
let view = {scale: 1, ox: 0, oy: 0};

function $(id) { return document.getElementById(id); }
function radToDeg(v) { return Number(v || 0) * 180 / Math.PI; }
function degToRad(v) { return Number(v || 0) * Math.PI / 180; }
function round4(v) { return Number(Number(v).toFixed(4)); }
function currentPassage() {
  return (state.narrow_passages || []).find(p => p.id === $('passageSelect').value) || {};
}
function currentZone() {
  const p = currentPassage();
  return (p.alignment_zones || []).find(z => z.id === $('zoneSelect').value) || null;
}

function resize() {
  const r = canvas.parentElement.getBoundingClientRect();
  canvas.width = Math.floor(r.width * devicePixelRatio);
  canvas.height = Math.floor(r.height * devicePixelRatio);
  draw();
}
window.addEventListener('resize', resize);

async function loadState() {
  state = await (await fetch('/api/state')).json();
  const buf = await (await fetch('/map.pgm')).arrayBuffer();
  mapImageData = parsePgm(new Uint8Array(buf));
  imgW = mapImageData.width; imgH = mapImageData.height;
  const off = new OffscreenCanvas(imgW, imgH);
  off.getContext('2d').putImageData(mapImageData.imageData, 0, 0);
  mapBitmap = await createImageBitmap(off);
  populatePassages();
  populateZones();
  loadZone(currentZone());
  fitMap();
  updateSnippet();
  resize();
}

function parsePgm(bytes) {
  let i = 0;
  function token() {
    while (i < bytes.length && bytes[i] <= 32) i++;
    if (bytes[i] === 35) { while (i < bytes.length && bytes[i] !== 10) i++; return token(); }
    let s = '';
    while (i < bytes.length && bytes[i] > 32) s += String.fromCharCode(bytes[i++]);
    return s;
  }
  const magic = token();
  const width = parseInt(token(), 10);
  const height = parseInt(token(), 10);
  const max = parseInt(token(), 10);
  while (i < bytes.length && bytes[i] <= 32) i++;
  const imageData = new ImageData(width, height);
  for (let p = 0; p < width * height; p++) {
    const v = magic === 'P5' ? bytes[i++] : parseInt(token(), 10);
    const g = Math.round((v / max) * 255);
    imageData.data[p * 4] = g;
    imageData.data[p * 4 + 1] = g;
    imageData.data[p * 4 + 2] = g;
    imageData.data[p * 4 + 3] = 255;
  }
  return {width, height, imageData};
}

function fitMap() {
  const wrap = canvas.parentElement.getBoundingClientRect();
  const scale = Math.min(wrap.width / imgW, wrap.height / imgH);
  view.scale = scale * devicePixelRatio;
  view.ox = (wrap.width * devicePixelRatio - imgW * view.scale) / 2;
  view.oy = (wrap.height * devicePixelRatio - imgH * view.scale) / 2;
}
function screenToPixel(e) {
  const rect = canvas.getBoundingClientRect();
  const sx = (e.clientX - rect.left) * devicePixelRatio;
  const sy = (e.clientY - rect.top) * devicePixelRatio;
  return {px: (sx - view.ox) / view.scale, py: (sy - view.oy) / view.scale};
}
function pixelToMap(px, py) {
  const res = state.map.resolution;
  const [ox, oy] = state.map.origin;
  return {x: ox + px * res, y: oy + (imgH - py) * res};
}
function mapToPixel(x, y) {
  const res = state.map.resolution;
  const [ox, oy] = state.map.origin;
  return {px: (x - ox) / res, py: imgH - (y - oy) / res};
}
function screenPoint(p) {
  const pix = mapToPixel(p.x, p.y);
  return {x: view.ox + pix.px * view.scale, y: view.oy + pix.py * view.scale};
}

function draw() {
  if (!mapBitmap) return;
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(mapBitmap, view.ox, view.oy, imgW * view.scale, imgH * view.scale);
  drawZones();
  drawSelectedPath();
  drawCurrent();
}
function drawPoly(poly, color, width = 2, fill = null) {
  if (!poly || poly.length === 0) return;
  ctx.beginPath();
  poly.forEach((p, i) => {
    const s = screenPoint(p);
    if (i === 0) ctx.moveTo(s.x, s.y); else ctx.lineTo(s.x, s.y);
  });
  if (poly.length >= 3) ctx.closePath();
  if (fill && poly.length >= 3) { ctx.fillStyle = fill; ctx.fill(); }
  ctx.strokeStyle = color; ctx.lineWidth = width * devicePixelRatio; ctx.stroke();
}
function drawLine(a, b, color, width = 2) {
  const sa = screenPoint(a), sb = screenPoint(b);
  ctx.beginPath();
  ctx.moveTo(sa.x, sa.y);
  ctx.lineTo(sb.x, sb.y);
  ctx.strokeStyle = color;
  ctx.lineWidth = width * devicePixelRatio;
  ctx.stroke();
}
function drawPose(p, yaw, color) {
  if (!p || p.x === undefined || p.y === undefined) return;
  const s = screenPoint(p);
  ctx.beginPath();
  ctx.arc(s.x, s.y, 5 * devicePixelRatio, 0, Math.PI * 2);
  ctx.fillStyle = color;
  ctx.fill();
  const len = 26 * devicePixelRatio;
  ctx.beginPath();
  ctx.moveTo(s.x, s.y);
  ctx.lineTo(s.x + Math.cos(yaw) * len, s.y - Math.sin(yaw) * len);
  ctx.strokeStyle = color;
  ctx.lineWidth = 2 * devicePixelRatio;
  ctx.stroke();
}
function drawSelectedPath() {
  const p = currentPassage();
  if (!p.entry || !p.exit) return;
  drawLine(p.entry, p.exit, 'rgba(31,111,235,.55)', 1.5);
}
function drawZones() {
  for (const p of state.narrow_passages || []) {
    for (const z of p.alignment_zones || []) {
      const selected = p.id === $('passageSelect').value && z.id === $('zoneSelect').value;
      const color = selected ? '#e03131' : (z.enabled ? '#159947' : '#b08900');
      drawPoly(z.polygon, color, selected ? 2.5 : 1.5, selected ? 'rgba(224,49,49,.12)' : null);
      if (z.pose) drawPose(z.pose, z.pose.yaw || 0, color);
    }
  }
}
function drawCurrent() {
  drawPoly(polygon, '#e03131', 2.5, 'rgba(224,49,49,.10)');
  for (const p of polygon) {
    const s = screenPoint(p);
    ctx.beginPath();
    ctx.arc(s.x, s.y, 4 * devicePixelRatio, 0, Math.PI * 2);
    ctx.fillStyle = '#e03131';
    ctx.fill();
  }
  const pose = currentPayload().pose;
  drawPose(pose, pose.yaw, '#7b2cbf');
}

canvas.addEventListener('mousemove', e => {
  if (!state) return;
  const {px, py} = screenToPixel(e);
  const m = pixelToMap(px, py);
  $('cursor').textContent = `map x=${m.x.toFixed(3)}, y=${m.y.toFixed(3)} | mode=${mode}`;
});
canvas.addEventListener('click', e => {
  const {px, py} = screenToPixel(e);
  if (px < 0 || py < 0 || px > imgW || py > imgH) return;
  const p = pixelToMap(px, py);
  const rounded = {x: round4(p.x), y: round4(p.y)};
  if (mode === 'pose') {
    $('poseX').value = rounded.x;
    $('poseY').value = rounded.y;
  } else {
    polygon.push(rounded);
  }
  updateSnippet();
  draw();
});

$('modePolygon').onclick = () => { mode = 'polygon'; $('modePolygon').className = 'primary'; $('modePose').className = ''; };
$('modePose').onclick = () => { mode = 'pose'; $('modePose').className = 'primary'; $('modePolygon').className = ''; };
$('undo').onclick = () => { polygon.pop(); updateSnippet(); draw(); };
$('clear').onclick = () => { polygon = []; updateSnippet(); draw(); };
$('reload').onclick = () => location.reload();
$('passageSelect').onchange = () => { populateZones(); loadZone(currentZone()); draw(); };
$('zoneSelect').onchange = () => { loadZone(currentZone()); draw(); };
$('newEntry').onclick = () => newZone('entry');
$('newExit').onclick = () => newZone('exit');
$('facePassage').onclick = () => { $('yawDeg').value = radToDeg(recommendedYaw()).toFixed(1); updateSnippet(); draw(); };

function populatePassages() {
  $('passageSelect').innerHTML = (state.narrow_passages || []).map(p => `<option value="${p.id}">${p.id}</option>`).join('');
}
function populateZones() {
  const p = currentPassage();
  $('zoneSelect').innerHTML = (p.alignment_zones || []).map(z => `<option value="${z.id}">${z.id} (${z.side}, enabled=${!!z.enabled})</option>`).join('');
}
function recommendedYaw() {
  const p = currentPassage();
  if (!p.entry || !p.exit) return 0;
  const side = $('side').value;
  const from = side === 'entry' ? p.entry : p.exit;
  const to = side === 'entry' ? p.exit : p.entry;
  return Math.atan2(to.y - from.y, to.x - from.x);
}
function newZone(side) {
  const passage = currentPassage();
  $('zoneId').value = `${passage.id}_${side}_alignment`;
  $('description').value = `Open area before entering ${passage.id} from ${side} side.`;
  $('side').value = side;
  $('enabled').value = 'false';
  polygon = [];
  const base = side === 'entry' ? passage.entry : passage.exit;
  if (base) {
    $('poseX').value = base.x;
    $('poseY').value = base.y;
  }
  $('yawDeg').value = radToDeg(recommendedYaw()).toFixed(1);
  $('xyTol').value = 0.25;
  $('yawTol').value = 0.35;
  $('timeoutSec').value = 16;
  $('skipNear').value = 0.12;
  updateSnippet();
  draw();
}
function loadZone(zone) {
  if (!zone) {
    newZone('entry');
    return;
  }
  $('zoneId').value = zone.id || '';
  $('description').value = zone.description || '';
  $('side').value = zone.side || 'entry';
  $('enabled').value = String(!!zone.enabled);
  polygon = (zone.polygon || []).map(p => ({x: Number(p.x), y: Number(p.y)}));
  $('poseX').value = zone.pose?.x ?? '';
  $('poseY').value = zone.pose?.y ?? '';
  $('yawDeg').value = radToDeg(zone.pose?.yaw || 0).toFixed(1);
  $('xyTol').value = zone.xy_goal_tolerance_m ?? 0.25;
  $('yawTol').value = zone.yaw_goal_tolerance_rad ?? 0.35;
  $('timeoutSec').value = zone.timeout_sec ?? 16;
  $('skipNear').value = zone.use_policy?.skip_if_robot_near_pose_m ?? 0.12;
  updateSnippet();
}
function currentPayload() {
  return {
    passage_id: $('passageSelect').value,
    id: $('zoneId').value.trim(),
    enabled: $('enabled').value === 'true',
    side: $('side').value,
    description: $('description').value.trim(),
    polygon,
    pose: {
      x: Number($('poseX').value || 0),
      y: Number($('poseY').value || 0),
      yaw: degToRad($('yawDeg').value || 0),
    },
    xy_goal_tolerance_m: Number($('xyTol').value || 0.25),
    yaw_goal_tolerance_rad: Number($('yawTol').value || 0.35),
    timeout_sec: Number($('timeoutSec').value || 16),
    skip_if_robot_near_pose_m: Number($('skipNear').value || 0.12),
  };
}
function updateSnippet() {
  $('points').innerHTML = polygon.map((p, i) => `<span class="pill">${i + 1}: x=${p.x}, y=${p.y}</span>`).join('') || '<span class="hint">还没有 polygon 点</span>';
  $('snippet').textContent = JSON.stringify(currentPayload(), null, 2);
}
['zoneId','description','side','enabled','poseX','poseY','yawDeg','xyTol','yawTol','timeoutSec','skipNear'].forEach(id => {
  $(id).addEventListener('input', () => { updateSnippet(); draw(); });
  $(id).addEventListener('change', () => { updateSnippet(); draw(); });
});
$('save').onclick = async () => {
  const body = currentPayload();
  if (!body.id) return alert('zone id is required');
  if (body.polygon.length < 3) return alert('polygon needs at least 3 points');
  const res = await fetch('/api/save_alignment_zone', {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body)});
  const data = await res.json();
  if (!data.ok) return alert(data.error || 'save failed');
  alert(`Saved ${data.zone_id}`);
  await loadState();
  $('passageSelect').value = data.passage_id;
  populateZones();
  $('zoneSelect').value = data.zone_id;
  loadZone(currentZone());
  draw();
};

loadState().catch(err => {
  $('cursor').textContent = err.toString();
  console.error(err);
});
</script>
</body>
</html>
"""


def main():
    parser = argparse.ArgumentParser(description="Local map-level navigation annotation calibration tool.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8772)
    parser.add_argument("--map-yaml", default=None)
    parser.add_argument("--annotation-yaml", default=None)
    args = parser.parse_args()

    app = NavigationAnnotationToolServer(
        map_yaml=Path(args.map_yaml) if args.map_yaml else _default_map_yaml(),
        annotation_yaml=Path(args.annotation_yaml) if args.annotation_yaml else _default_annotation_yaml(),
    )
    server = ThreadingHTTPServer((args.host, args.port), _make_handler(app))
    print(f"Navigation annotation tool: http://{args.host}:{args.port}")
    print(f"Map YAML: {app.map_yaml}")
    print(f"Map image: {app.map_image}")
    print(f"Annotation YAML: {app.annotation_yaml}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
