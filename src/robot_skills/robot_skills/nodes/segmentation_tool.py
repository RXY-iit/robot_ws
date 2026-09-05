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
DEFAULT_SEGMENTS_YAML = WORKSPACE_ROOT / "maps" / "l402_2d_map_clean_0509" / "semantic_segments.yaml"
DEFAULT_SEMANTIC_MAP_YAML = WORKSPACE_ROOT / "maps" / "l402_2d_map_clean_0509" / "semantic_map.yaml"


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


def _default_segments_yaml() -> Path:
    bundle_dir = _bundle_dir_from_env()
    if bundle_dir and (bundle_dir / "semantic_segments.yaml").exists():
        return bundle_dir / "semantic_segments.yaml"
    return DEFAULT_SEGMENTS_YAML


def _default_semantic_map_yaml() -> Path:
    bundle_dir = _bundle_dir_from_env()
    if bundle_dir and (bundle_dir / "semantic_map.yaml").exists():
        return bundle_dir / "semantic_map.yaml"
    return DEFAULT_SEMANTIC_MAP_YAML


def _load_yaml(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def _write_yaml(path: Path, data: dict[str, Any]):
    if path.exists():
        backup = path.with_suffix(path.suffix + ".bak")
        shutil.copy2(path, backup)
    with path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)


def _upsert(items: list[dict[str, Any]], item: dict[str, Any]) -> list[dict[str, Any]]:
    item_id = item.get("id")
    for index, existing in enumerate(items):
        if existing.get("id") == item_id:
            items[index] = item
            return items
    items.append(item)
    return items


def _map_image_path(map_yaml_path: Path, map_data: dict[str, Any]) -> Path:
    image = map_data.get("image", "")
    path = Path(image)
    if not path.is_absolute():
        path = map_yaml_path.parent / path
    return path


def _segment_from_payload(payload: dict[str, Any]) -> dict[str, Any]:
    segment_type = payload.get("type", "object_region")
    polygon = payload.get("polygon") or []
    point = payload.get("point")
    names = payload.get("names") or [payload["id"]]
    tags = payload.get("tags") or []

    segment: dict[str, Any] = {
        "id": payload["id"],
        "type": segment_type,
        "names": names,
        "tags": tags,
        "annotation": {
            "color": payload.get("color", "user"),
            "shape": "polygon" if polygon else "point",
            "meaning": payload.get("meaning", payload["id"]),
        },
        "map_polygon": polygon,
        "map_pose": point,
        "calibrated": bool(polygon or point),
        "navigation_ready": bool(len(polygon) >= 3 or point),
        "notes": payload.get("notes", ""),
    }
    if polygon:
        yaw_mode = payload.get("yaw_mode", "face_region_center")
        segment["approach_policy"] = {
            "generator": "region_side_offset",
            "preferred_side": payload.get("front_side", "west"),
            "distance_m": float(payload.get("distance_m", 0.75)),
            "yaw_mode": yaw_mode,
            "yaw": math.radians(float(payload.get("fixed_yaw_deg", 0.0))),
            "yaw_offset_deg": float(payload.get("yaw_offset_deg", 0.0)),
            "face_region_center": True,
        }
    return segment


def _semantic_object_from_payload(payload: dict[str, Any]) -> dict[str, Any]:
    polygon = payload.get("polygon") or []
    point = payload.get("point")
    names = payload.get("names") or [payload["id"]]
    category = payload.get("category", "object_region")

    item: dict[str, Any] = {
        "id": payload["id"],
        "names": names,
        "category": category,
        "static": True,
        "segment_id": payload["id"],
        "navigation_ready": bool(len(polygon) >= 3 or point),
        "affordances": ["navigate_near", "inspect", "visual_arrival_confirm"],
        "safety_zone_m": float(payload.get("safety_zone_m", 0.5)),
        "confidence": float(payload.get("confidence", 0.7)),
        "note": payload.get("notes", ""),
    }

    if polygon:
        yaw_mode = payload.get("yaw_mode", "face_region_center")
        item["map_region"] = {
            "type": "polygon",
            "polygon": polygon,
            "front_side": payload.get("front_side", "west"),
        }
        item["approach_policy"] = {
            "generator": "region_side_offset",
            "preferred_side": payload.get("front_side", "west"),
            "distance_m": float(payload.get("distance_m", 0.75)),
            "clearance_mode": payload.get("clearance_mode", "narrow"),
            "speed_profile": payload.get("speed_profile", "cautious"),
            "yaw_mode": yaw_mode,
            "yaw": math.radians(float(payload.get("fixed_yaw_deg", 0.0))),
            "yaw_offset_deg": float(payload.get("yaw_offset_deg", 0.0)),
            "face_region_center": True,
        }
    elif point:
        item["pose"] = point
        item["approach_poses"] = [{"id": "near", "pose": point}]

    if payload.get("visual_confirm", True):
        item["visual_arrival_confirm"] = {
            "enabled": True,
            "camera_control_policy": "scan_after_stop",
            "detector_skill": "detect_known_object",
            "detector_candidates": ["yolo", "gpt_api", "local_vlm", "manual_confirm"],
            "expected_query": payload.get("expected_query", payload["id"]),
        }
    return item


class SegmentationToolServer:
    def __init__(self, map_yaml: Path, segments_yaml: Path, semantic_map_yaml: Path):
        self.map_yaml = map_yaml
        self.segments_yaml = segments_yaml
        self.semantic_map_yaml = semantic_map_yaml
        self.map_data = _load_yaml(map_yaml)
        self.map_image = _map_image_path(map_yaml, self.map_data)

    def state(self) -> dict[str, Any]:
        return {
            "map": {
                "yaml_path": str(self.map_yaml),
                "image_path": str(self.map_image),
                "resolution": float(self.map_data.get("resolution", 0.05)),
                "origin": self.map_data.get("origin", [0.0, 0.0, 0.0]),
                "mode": self.map_data.get("mode", "trinary"),
                "occupied_thresh": self.map_data.get("occupied_thresh"),
                "free_thresh": self.map_data.get("free_thresh"),
            },
            "segments": _load_yaml(self.segments_yaml).get("segments", []),
            "semantic_objects": _load_yaml(self.semantic_map_yaml).get("objects", []),
        }

    def save_annotation(self, payload: dict[str, Any]) -> dict[str, Any]:
        if not payload.get("id"):
            raise ValueError("id is required")

        segments_data = _load_yaml(self.segments_yaml)
        segments_data.setdefault("map_id", "sim_l402_seed_v0")
        segments_data.setdefault("frame_id", "map")
        segments_data.setdefault("segments", [])
        segments_data["segments"] = _upsert(segments_data["segments"], _segment_from_payload(payload))
        _write_yaml(self.segments_yaml, segments_data)

        semantic_data = _load_yaml(self.semantic_map_yaml)
        semantic_data.setdefault("map_id", "sim_l402_seed_v0")
        semantic_data.setdefault("frame_id", "map")
        semantic_data.setdefault("places", [])
        semantic_data.setdefault("objects", [])
        semantic_data["objects"] = _upsert(semantic_data["objects"], _semantic_object_from_payload(payload))
        _write_yaml(self.semantic_map_yaml, semantic_data)

        return {
            "ok": True,
            "segment_id": payload["id"],
            "segments_yaml": str(self.segments_yaml),
            "semantic_map_yaml": str(self.semantic_map_yaml),
        }


def _make_handler(app: SegmentationToolServer):
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
            if path != "/api/save":
                self._send_json({"error": "not found"}, 404)
                return
            try:
                length = int(self.headers.get("Content-Length", "0"))
                payload = json.loads(self.rfile.read(length).decode("utf-8"))
                result = app.save_annotation(payload)
            except Exception as exc:
                self._send_json({"ok": False, "error": str(exc)}, 400)
                return
            self._send_json(result)

        def log_message(self, fmt, *args):
            print(f"[segmentation_tool] {self.address_string()} - {fmt % args}")

    return Handler


HTML = r"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>Robot Semantic Segmentation Tool</title>
  <style>
    :root { color-scheme: light; font-family: Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; }
    body { margin: 0; background: #f4f6f8; color: #182026; }
    main { display: grid; grid-template-columns: minmax(520px, 1fr) 380px; height: 100vh; }
    #mapWrap { position: relative; overflow: hidden; background: #c9cdd1; }
    canvas { display: block; width: 100%; height: 100%; cursor: crosshair; }
    aside { border-left: 1px solid #cbd3dc; background: #ffffff; padding: 14px; overflow: auto; }
    h1 { margin: 0 0 12px; font-size: 18px; }
    h2 { margin: 18px 0 8px; font-size: 13px; color: #53606b; text-transform: uppercase; letter-spacing: .04em; }
    label { display: grid; gap: 4px; margin: 9px 0; font-size: 13px; color: #3b4650; }
    input, select, textarea, button { font: inherit; }
    input, select, textarea { width: 100%; box-sizing: border-box; border: 1px solid #bcc7d2; border-radius: 6px; padding: 7px 8px; background: #fff; }
    textarea { min-height: 64px; resize: vertical; }
    button { border: 1px solid #aeb9c4; background: #f7f9fb; border-radius: 6px; padding: 7px 10px; cursor: pointer; }
    button.primary { background: #1f6feb; color: white; border-color: #1f6feb; }
    button.danger { color: #a42323; }
    .row { display: flex; gap: 8px; align-items: center; }
    .row > * { flex: 1; }
    .toolbar { position: absolute; left: 12px; top: 12px; display: flex; gap: 6px; padding: 6px; background: rgba(255,255,255,.92); border: 1px solid #cbd3dc; border-radius: 8px; box-shadow: 0 2px 10px rgba(30,40,50,.12); }
    .status { position: absolute; left: 12px; bottom: 12px; padding: 6px 8px; background: rgba(255,255,255,.92); border: 1px solid #cbd3dc; border-radius: 8px; font-size: 12px; }
    pre { max-height: 220px; overflow: auto; background: #f1f4f7; border: 1px solid #d8e0e8; border-radius: 6px; padding: 8px; font-size: 12px; }
    .hint { font-size: 12px; color: #64717d; line-height: 1.45; }
    .pill { display: inline-block; border: 1px solid #cbd3dc; border-radius: 999px; padding: 2px 7px; margin: 2px; font-size: 12px; background: #f8fafc; }
  </style>
</head>
<body>
<main>
  <section id="mapWrap">
    <canvas id="mapCanvas"></canvas>
    <div class="toolbar">
      <button id="modePoint">Point</button>
      <button id="modePoly" class="primary">Polygon</button>
      <button id="undo">Undo</button>
      <button id="clear" class="danger">Clear</button>
    </div>
    <div class="status" id="cursor">loading map...</div>
  </section>
  <aside>
    <h1>Semantic Segmentation Tool</h1>
    <div class="hint">点击地图记录 map-frame 坐标。polygon 至少 3 个点后，可以生成区域驱动 approach pose。</div>

    <h2>选择 / 切换 Segment</h2>
    <div class="row">
      <label>Segment
        <select id="segmentSelect"></select>
      </label>
      <button id="newSegment">New</button>
    </div>
    <div class="hint">选择已有 segment 会把它的 polygon 和表单信息载入当前编辑区；保存时按同名 id 覆盖。</div>

    <h2>目标信息</h2>
    <label>ID <input id="id" value="shelf_06" /></label>
    <label>Category <input id="category" value="shelf" /></label>
    <label>Names / aliases <input id="names" value="shelf_06,06架子,架子06,6号架子,06 shelf,shelf 06" /></label>
    <label>Tags <input id="tags" placeholder="rack, number_label, high_shelf" /></label>
    <label>Meaning <input id="meaning" value="06 shelf" /></label>

    <h2>Approach Policy</h2>
    <div class="row">
      <label>Front side
        <select id="frontSide">
          <option>west</option><option>east</option><option>north</option><option>south</option>
        </select>
      </label>
      <label>Distance m <input id="distance" type="number" value="0.75" step="0.05" /></label>
    </div>
    <div class="row">
      <label>Yaw mode
        <select id="yawMode">
          <option value="face_region_center">face region center</option>
          <option value="fixed_yaw">fixed yaw</option>
        </select>
      </label>
      <label>Yaw offset deg <input id="yawOffset" type="number" value="0" step="5" /></label>
    </div>
    <label>Fixed yaw deg <input id="fixedYaw" type="number" value="0" step="5" /></label>
    <div class="row">
      <label>Safety zone m <input id="safety" type="number" value="0.5" step="0.05" /></label>
      <label>Confidence <input id="confidence" type="number" value="0.7" step="0.05" min="0" max="1" /></label>
    </div>
    <label>Expected visual query <input id="expected" value="06 shelf label" /></label>
    <label>Notes <textarea id="notes">Annotated from Nav2 map. Approach pose should be generated from polygon, not remembered from one manual stop.</textarea></label>

    <h2>当前选点</h2>
    <div id="points"></div>
    <pre id="snippet"></pre>

    <div class="row">
      <button id="save" class="primary">Save to YAML</button>
      <button id="reload">Reload</button>
    </div>

    <h2>已有 Segments</h2>
    <div id="segments"></div>
  </aside>
</main>
<script>
const canvas = document.getElementById('mapCanvas');
const ctx = canvas.getContext('2d');
const cursor = document.getElementById('cursor');
let state = null;
let mapBitmap = null;
let mapImageData = null;
let imgW = 0, imgH = 0;
let mode = 'polygon';
let points = [];
let view = {scale: 1, ox: 0, oy: 0};

function $(id) { return document.getElementById(id); }
function listValue(id) { return $(id).value.split(',').map(x => x.trim()).filter(Boolean); }

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
  fitMap();
  renderSegments();
  populateSegmentSelect();
  loadSegment($('segmentSelect').value || 'shelf_06', true);
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

function draw() {
  if (!mapBitmap) return;
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(mapBitmap, view.ox, view.oy, imgW * view.scale, imgH * view.scale);
  drawExisting();
  drawCurrent();
}

function drawPoly(poly, color, width = 2) {
  if (!poly || poly.length === 0) return;
  ctx.beginPath();
  poly.forEach((p, idx) => {
    const pix = mapToPixel(p.x, p.y);
    const sx = view.ox + pix.px * view.scale;
    const sy = view.oy + pix.py * view.scale;
    if (idx === 0) ctx.moveTo(sx, sy); else ctx.lineTo(sx, sy);
  });
  if (poly.length >= 3) ctx.closePath();
  ctx.strokeStyle = color; ctx.lineWidth = width * devicePixelRatio; ctx.stroke();
}

function drawExisting() {
  for (const seg of state.segments || []) {
    const color = seg.id === $('id').value.trim() ? '#e03131' : '#159947';
    drawPoly(seg.map_polygon, color, seg.id === $('id').value.trim() ? 2.5 : 1.5);
  }
}

function drawCurrent() {
  drawPoly(points, '#e03131', 2.5);
  for (const p of points) {
    const pix = mapToPixel(p.x, p.y);
    const sx = view.ox + pix.px * view.scale;
    const sy = view.oy + pix.py * view.scale;
    ctx.beginPath();
    ctx.arc(sx, sy, 4 * devicePixelRatio, 0, Math.PI * 2);
    ctx.fillStyle = '#e03131';
    ctx.fill();
  }
}

canvas.addEventListener('mousemove', e => {
  if (!state) return;
  const {px, py} = screenToPixel(e);
  const m = pixelToMap(px, py);
  cursor.textContent = `pixel ${px.toFixed(1)}, ${py.toFixed(1)} | map x=${m.x.toFixed(3)}, y=${m.y.toFixed(3)} | origin ${state.map.origin.join(', ')} | res ${state.map.resolution}`;
});

canvas.addEventListener('click', e => {
  const {px, py} = screenToPixel(e);
  if (px < 0 || py < 0 || px > imgW || py > imgH) return;
  const p = pixelToMap(px, py);
  const rounded = {x: Number(p.x.toFixed(4)), y: Number(p.y.toFixed(4))};
  if (mode === 'point') points = [rounded]; else points.push(rounded);
  updateSnippet();
  draw();
});

$('modePoint').onclick = () => { mode = 'point'; $('modePoint').className = 'primary'; $('modePoly').className = ''; };
$('modePoly').onclick = () => { mode = 'polygon'; $('modePoly').className = 'primary'; $('modePoint').className = ''; };
$('undo').onclick = () => { points.pop(); updateSnippet(); draw(); };
$('clear').onclick = () => { points = []; updateSnippet(); draw(); };
$('reload').onclick = () => location.reload();
$('newSegment').onclick = () => {
  points = [];
  $('id').value = '';
  $('category').value = 'shelf';
  $('names').value = '';
  $('tags').value = '';
  $('meaning').value = '';
  $('frontSide').value = 'west';
  $('distance').value = '0.75';
  $('yawMode').value = 'face_region_center';
  $('yawOffset').value = '0';
  $('fixedYaw').value = '0';
  $('safety').value = '0.5';
  $('confidence').value = '0.7';
  $('expected').value = '';
  $('notes').value = 'Annotated from Nav2 map. Approach pose should be generated from polygon, not remembered from one manual stop.';
  updateSnippet();
  draw();
};
$('segmentSelect').onchange = () => loadSegment($('segmentSelect').value);

function payload() {
  const isPolygon = points.length >= 3 && mode === 'polygon';
  return {
    id: $('id').value.trim(),
    type: isPolygon ? 'object_region' : 'point',
    category: $('category').value.trim() || 'object_region',
    names: listValue('names'),
    tags: listValue('tags'),
    meaning: $('meaning').value.trim(),
    polygon: isPolygon ? points : [],
    point: !isPolygon && points[0] ? {...points[0], yaw: 0.0, frame_id: 'map'} : null,
    front_side: $('frontSide').value,
    distance_m: Number($('distance').value),
    yaw_mode: $('yawMode').value,
    yaw_offset_deg: Number($('yawOffset').value),
    fixed_yaw_deg: Number($('fixedYaw').value),
    safety_zone_m: Number($('safety').value),
    confidence: Number($('confidence').value),
    expected_query: $('expected').value.trim(),
    notes: $('notes').value.trim(),
    visual_confirm: true
  };
}

function updateSnippet() {
  $('points').innerHTML = points.map((p, i) => `<span class="pill">${i + 1}: x=${p.x}, y=${p.y}</span>`).join('') || '<span class="hint">还没有选点</span>';
  $('snippet').textContent = JSON.stringify(payload(), null, 2);
}
['id','category','names','tags','meaning','frontSide','distance','yawMode','yawOffset','fixedYaw','safety','confidence','expected','notes'].forEach(id => $(id).addEventListener('input', updateSnippet));

$('save').onclick = async () => {
  const body = payload();
  if (!body.id) return alert('ID is required');
  if (!body.point && body.polygon.length < 3) return alert('Point 模式需要 1 个点；Polygon 模式需要至少 3 个点');
  const res = await fetch('/api/save', {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body)});
  const data = await res.json();
  if (!data.ok) return alert(data.error || 'save failed');
  alert(`Saved ${data.segment_id}`);
  await loadState();
  loadSegment(data.segment_id, true);
};

function renderSegments() {
  $('segments').innerHTML = (state.segments || []).map(s => {
    const n = (s.names || []).join(', ');
    const p = (s.map_polygon || []).length;
    return `<button class="pill" data-segment-id="${s.id}">${s.id} · ${s.type} · ${p} pts · ${n}</button>`;
  }).join('');
  document.querySelectorAll('[data-segment-id]').forEach(el => {
    el.addEventListener('click', () => loadSegment(el.getAttribute('data-segment-id')));
  });
}

function populateSegmentSelect() {
  const current = $('id').value.trim();
  $('segmentSelect').innerHTML = (state.segments || []).map(s => {
    const selected = s.id === current ? ' selected' : '';
    const p = (s.map_polygon || []).length;
    return `<option value="${s.id}"${selected}>${s.id} (${p} pts)</option>`;
  }).join('');
}

function semanticObjectFor(id) {
  return (state.semantic_objects || []).find(obj => obj.id === id || obj.segment_id === id) || {};
}

function loadSegment(id, quiet = false) {
  const seg = (state.segments || []).find(item => item.id === id);
  if (!seg) return;
  const obj = semanticObjectFor(id);
  const policy = seg.approach_policy || obj.approach_policy || {};
  const visual = obj.visual_arrival_confirm || {};
  points = (seg.map_polygon || obj.map_region?.polygon || []).map(p => ({x: Number(p.x), y: Number(p.y)}));
  if (!points.length && seg.map_pose) points = [{x: Number(seg.map_pose.x), y: Number(seg.map_pose.y)}];
  mode = points.length <= 1 ? 'point' : 'polygon';
  $('modePoint').className = mode === 'point' ? 'primary' : '';
  $('modePoly').className = mode === 'polygon' ? 'primary' : '';
  $('id').value = seg.id || obj.id || '';
  $('category').value = obj.category || (seg.type === 'object_region' ? 'shelf' : seg.type || 'object_region');
  $('names').value = (seg.names || obj.names || [seg.id]).join(',');
  $('tags').value = (seg.tags || []).join(',');
  $('meaning').value = seg.annotation?.meaning || seg.id || '';
  $('frontSide').value = policy.preferred_side || obj.map_region?.front_side || 'west';
  $('distance').value = policy.distance_m ?? 0.75;
  $('yawMode').value = policy.yaw_mode || 'face_region_center';
  $('yawOffset').value = policy.yaw_offset_deg ?? 0;
  $('fixedYaw').value = policy.yaw ? (Number(policy.yaw) * 180 / Math.PI).toFixed(1) : 0;
  $('safety').value = obj.safety_zone_m ?? 0.5;
  $('confidence').value = obj.confidence ?? 0.7;
  $('expected').value = visual.expected_query || `${seg.id} label`;
  $('notes').value = seg.notes || obj.note || '';
  $('segmentSelect').value = seg.id;
  updateSnippet();
  draw();
  if (!quiet) cursor.textContent = `loaded ${seg.id}: ${points.length} point(s)`;
}

loadState().catch(err => {
  cursor.textContent = err.toString();
  console.error(err);
});
</script>
</body>
</html>
"""


def main():
    parser = argparse.ArgumentParser(description="Local semantic segmentation map annotation tool.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8770)
    parser.add_argument("--map-yaml", default=None)
    parser.add_argument("--segments-yaml", default=None)
    parser.add_argument("--semantic-map-yaml", default=None)
    args = parser.parse_args()

    app = SegmentationToolServer(
        map_yaml=Path(args.map_yaml) if args.map_yaml else _default_map_yaml(),
        segments_yaml=Path(args.segments_yaml) if args.segments_yaml else _default_segments_yaml(),
        semantic_map_yaml=Path(args.semantic_map_yaml) if args.semantic_map_yaml else _default_semantic_map_yaml(),
    )
    server = ThreadingHTTPServer((args.host, args.port), _make_handler(app))
    print(f"Segmentation tool: http://{args.host}:{args.port}")
    print(f"Map YAML: {app.map_yaml}")
    print(f"Map image: {app.map_image}")
    print(f"Segments YAML: {app.segments_yaml}")
    print(f"Semantic map YAML: {app.semantic_map_yaml}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
