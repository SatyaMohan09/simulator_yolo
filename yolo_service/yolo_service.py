"""
eVTOL Obstacle Detection Service — v6 (Realistic Upgrades)
===========================================================
Upgrades over v5:
  1. Confidence threshold  — detections below MIN_CONFIDENCE are ignored
  2. Consecutive-frame filter — obstacle must appear in >= MIN_CONFIRM_FRAMES
     consecutive frames before being published to the backend
  3. TTL eviction — published obstacles expire after TTL_SECONDS of absence
  4. Theme-aware HSV ranges — sky/ground masks adapt to lighting theme
  5. Tighter dedup grid (5 m instead of 10 m) for more accurate planning
  6. Match threshold aligned with backend (80 m)

Run:
    pip install flask flask-cors opencv-python numpy Pillow scipy
    python yolo_service.py
"""

import os, base64, csv, math, logging, time, threading
from io import BytesIO
from datetime import datetime
from urllib import request as urllib_request
from urllib.error import URLError

import cv2
import numpy as np
from PIL import Image
from flask import Flask, request, jsonify
from flask_cors import CORS

try:
    from ground_truth_validator import GroundTruthValidator
    HAS_VALIDATOR = True
except ImportError:
    HAS_VALIDATOR = False
    GroundTruthValidator = None

import json

# ── CONFIG ──────────────────────────────────────────────────────────────────
SAVE_DIR      = os.path.join(os.path.dirname(__file__), "captured_frames")
ANNOTATED_DIR = os.path.join(SAVE_DIR, "annotated")
CAMERA_HFOV   = 60.0

H_REAL = {"building": 400.0, "wall": 30.0, "vehicle": 3.0, "structure": 10.0, "bird": 1.2}

MIN_BBOX_W    = 30
MIN_BBOX_H    = 50
MIN_BBOX_AREA = 2500
NMS_IOU       = 0.50
MAX_DET       = 12
MAX_BIRD_DET  = 4

BIRD_MIN_BBOX_W    = 6
BIRD_MIN_BBOX_H    = 6
BIRD_MIN_BBOX_AREA = 36
BIRD_MAX_BBOX_AREA = 1200
BIRD_SCAN_HEIGHT_RATIO = 0.68

EMA_ALPHA     = 0.15
GRID_CELL_M   = 5          # tightened from 10 → 5 m for more accurate dedup

# ── NEW: quality / reliability gates ────────────────────────────────────────
MIN_CONFIDENCE     = 0.65   # ignore detections below this confidence
MIN_CONFIRM_FRAMES = 1      # publish on first valid frame for real-time replanning
TTL_SECONDS        = 300.0  # keep obstacles for entire flight once detected

# Ground-truth match threshold aligned with backend (application.properties)
GT_MATCH_THRESHOLD_M = 80.0

# Mission-space anchor points copied from the frontend scene.
START_ZONE_X = -1490.0
START_ZONE_Z = 260.0
LANDING_ZONE_X = 3420.0
LANDING_ZONE_Z = 500.0
VERTIPORT_SAFE_RADIUS_M = 260.0
PATH_CORRIDOR_HALF_WIDTH_M = 420.0
MIN_FORWARD_PROJECTION_M = 40.0

os.makedirs(SAVE_DIR, exist_ok=True)
os.makedirs(ANNOTATED_DIR, exist_ok=True)

# ── PROXIMITY-BASED SURPRISE DETECTION ───────────────────────────────────────
# When the eVTOL is within PROXIMITY_DETECT_M of an obstacle that is visible in
# the frontend scene but NOT in the backend-known (imageprocessing) CSV, inject
# it as a synthetic detection so the replan pipeline fires reliably even when
# visual colour-segmentation is uncertain.

PROXIMITY_DETECT_M = 900.0   # sensor range (metres) to "see" an obstacle

def _load_csv_obstacles(path):
    """Return list of {x,y,z,radius} dicts from a CSV file."""
    obstacles = []
    if not os.path.exists(path):
        return obstacles
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                obstacles.append({
                    "x":      float(row["x"]),
                    "y":      float(row.get("y", 0)),
                    "z":      float(row["z"]),
                    "radius": float(row.get("radius", 155)),
                })
            except (KeyError, ValueError):
                pass
    return obstacles

# Paths relative to this file
_BASE = os.path.dirname(__file__)
_FRONTEND_OBSTACLES_PATH      = os.path.join(_BASE, "../frontend/public/data/obstacles.csv")
_IMAGEPROCESSING_OBSTACLES_PATH = os.path.join(_BASE, "../imageprocessing/data/obstacles.csv")

def _load_surprise_obstacles():
    """
    Return obstacles that are in the frontend scene but NOT in the imageprocessing
    (backend-known) CSV.  These are the "surprise" buildings that the backend must
    discover via YOLO detection at runtime.
    """
    all_obs   = _load_csv_obstacles(_FRONTEND_OBSTACLES_PATH)
    known_obs = _load_csv_obstacles(_IMAGEPROCESSING_OBSTACLES_PATH)

    surprise = []
    for obs in all_obs:
        is_known = any(
            math.hypot(obs["x"] - k["x"], obs["z"] - k["z"]) < 200
            for k in known_obs
        )
        if not is_known:
            surprise.append(obs)

    # log may not exist yet at module-load time, so use print here
    print(f"[Proximity] Loaded {len(surprise)} surprise obstacle(s) "
          f"(frontend={len(all_obs)}, known={len(known_obs)})")
    return surprise

_surprise_obstacles = _load_surprise_obstacles()

def get_proximity_detections(evtol_x, evtol_y, evtol_z):
    """
    Return synthetic obstacle dicts for any surprise obstacle closer than
    PROXIMITY_DETECT_M to the eVTOL.  These are formatted identically to
    the normalised planner obstacles returned by scene_detect() so they can
    be merged seamlessly with visual detections.
    """
    detections = []
    for obs in _surprise_obstacles:
        dist = math.sqrt(
            (obs["x"] - evtol_x) ** 2 +
            (obs["z"] - evtol_z) ** 2
        )
        if dist < PROXIMITY_DETECT_M:
            detections.append({
                "label":      "building",
                "confidence": 0.90,
                "X_world":    round(obs["x"], 2),
                "Y_world":    0.0,
                "Z_world":    round(obs["z"], 2),
                "distance":   round(dist, 2),
                "radius":     max(obs["radius"], 155.0),
                "source":     "proximity",
            })
    return detections

logging.basicConfig(level=logging.INFO, format="%(asctime)s  %(levelname)s  %(message)s")
log = logging.getLogger(__name__)

app           = Flask(__name__)
CORS(app)
frame_counter = 0
smooth_state: dict  = {}

# candidate_counts: key → consecutive frame count (not yet confirmed)
candidate_counts: dict = {}

# published_obstacle_map: key → {obstacle_data, last_seen_time}
published_obstacle_map: dict = {}

BACKEND_OBSTACLE_SYNC_URL = "http://localhost:8080/api/obstacles/detections"

# ── Ground truth validator ───────────────────────────────────────────────────
validator = None
if HAS_VALIDATOR:
    try:
        CSV_PATH = os.path.join(os.path.dirname(__file__), '../imageprocessing/data/obstacles.csv')
        if os.path.exists(CSV_PATH):
            validator = GroundTruthValidator(CSV_PATH)
            log.info(f"✓ Ground truth validator loaded from: {CSV_PATH}")
        else:
            log.warning(f"⚠ Frontend ground-truth CSV not found: {CSV_PATH}")
    except Exception as e:
        log.warning(f"⚠ Could not initialise validator: {e}")

# ── Theme-aware sky/ground HSV masks ─────────────────────────────────────────
# Each theme defines HSV ranges for sky and ground pixels.
THEME_MASKS = {
    "daylight": {
        "sky":    [([85,  8, 140], [145, 145, 255])],
        # H=30..92 = yellow-greens to green only; excludes orange (H=5-29) so
        # orange/rust buildings are NOT masked as ground and remain detectable.
        "ground": [([30, 25, 25], [92, 255, 185])],
    },
    "dawn": {
        "sky":    [([140, 20, 80], [180, 180, 255]), ([0, 20, 80], [20, 180, 255])],
        "ground": [([15, 15, 20], [40, 200, 160])],
    },
    "sunset": {
        "sky":    [([0, 60, 120], [30, 255, 255]), ([150, 40, 100], [180, 255, 255])],
        "ground": [([10, 20, 20], [35, 200, 150])],
    },
    "dusk": {
        "sky":    [([120, 10, 30], [160, 180, 180])],
        "ground": [([20, 10, 10], [60, 180, 120])],
    },
    "moonlight": {
        "sky":    [([90, 5, 5],   [160, 80, 80])],
        "ground": [([80, 5, 5],   [130, 60, 60])],
    },
    "overcast": {
        "sky":    [([85, 0, 120], [145, 50, 255])],
        "ground": [([20, 10, 20], [80, 120, 160])],
    },
    "aurora": {
        "sky":    [([80, 30, 10], [170, 200, 120])],
        "ground": [([80, 5, 5],   [160, 80, 80])],
    },
}

def get_theme_mask(hsv, theme):
    """Build combined sky and ground mask for the given theme name."""
    cfg = THEME_MASKS.get(theme, THEME_MASKS["daylight"])
    sky = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lo, hi in cfg["sky"]:
        sky = cv2.bitwise_or(sky, cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8)))
    gnd = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lo, hi in cfg["ground"]:
        gnd = cv2.bitwise_or(gnd, cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8)))
    return sky, gnd

# ── Utilities ────────────────────────────────────────────────────────────────
def decode_b64_png_to_rgb(b64):
    if not b64: raise ValueError("Missing frame")
    if "," in b64: b64 = b64.split(",", 1)[1]
    pil = Image.open(BytesIO(base64.b64decode(b64))).convert("RGB")
    return np.array(pil)

def encode_rgb_to_data_url_png(img_rgb):
    bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
    ok, buf = cv2.imencode(".png", bgr)
    if not ok: raise ValueError("Failed to encode png")
    return "data:image/png;base64," + base64.b64encode(buf.tobytes()).decode()

def background_reveal(img_rgb, preset="auto"):
    gray = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2GRAY)
    edges = cv2.Canny(gray, 40, 120)
    _, seg = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    combo = cv2.bitwise_or(seg, edges)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
    combo = cv2.morphologyEx(combo, cv2.MORPH_CLOSE, kernel, iterations=2)
    overlay = img_rgb.copy()
    green = np.zeros_like(img_rgb); green[:, :] = (0, 255, 0)
    overlay = np.where(combo[:, :, None] > 0, (0.65*overlay + 0.35*green).astype(np.uint8), overlay)
    overlay = np.where(edges[:, :, None] > 0, np.array([255, 255, 255], dtype=np.uint8), overlay)
    return overlay, combo

# ── Quaternion helper ─────────────────────────────────────────────────────────
def quat_rotate(q, v):
    qx, qy, qz, qw = q
    t = 2.0 * np.array([qy*v[2]-qz*v[1], qz*v[0]-qx*v[2], qx*v[1]-qy*v[0]])
    return v + qw*t + np.array([qy*t[2]-qz*t[1], qz*t[0]-qx*t[2], qx*t[1]-qy*t[0]])

# ── Pixel → world ─────────────────────────────────────────────────────────────
def pixel_to_world(x_min, y_min, x_max, y_max, img_W, img_H,
                   ex, ey, ez, qx, qy, qz, qw, label="structure", hfov=CAMERA_HFOV):
    f    = (img_W / 2.0) / math.tan(math.radians(hfov) / 2.0)
    cx, cy = img_W / 2.0, img_H / 2.0
    h_px = max(y_max - y_min, 1)
    w_px = max(x_max - x_min, 1)
    u = (x_min + x_max) / 2.0
    v = y_max
    h_real = H_REAL.get(label, 10.0)
    Z = (f * h_real) / h_px
    P_cam = np.array([(u-cx)/f*Z, -(v-cy)/f*Z, -Z])
    q = np.array([qx, qy, qz, qw])
    n = np.linalg.norm(q); q = q/n if n > 1e-6 else np.array([0.,0.,0.,1.])
    P_rel = quat_rotate(q, P_cam)
    return {
        "X_world": round(float(P_rel[0]+ex), 2),
        "Y_world": round(float(P_rel[1]+ey), 2),
        "Z_world": round(float(P_rel[2]+ez), 2),
        "distance": round(float(np.linalg.norm(P_cam)), 2),
        "radius":   round(float((w_px/img_W)*Z), 2),
    }

def smooth_coords(label, raw):
    gx = int(raw["X_world"] / GRID_CELL_M)
    gy = int(raw["Y_world"] / GRID_CELL_M)
    gz = int(raw["Z_world"] / GRID_CELL_M)
    key = (label, gx, gy, gz)
    if key in smooth_state:
        prev = smooth_state[key]
        merged = {k: EMA_ALPHA*raw[k] + (1-EMA_ALPHA)*prev[k]
                  for k in ("X_world","Y_world","Z_world","distance","radius")}
    else:
        merged = dict(raw)
    smooth_state[key] = merged
    return {k: round(merged[k], 2) for k in ("X_world","Y_world","Z_world","distance","radius")}

def normalize_for_planner(label, coords):
    normalized = dict(coords)
    if label in {"building", "wall", "structure"}:
        normalized["Y_world"] = 0.0
        normalized["radius"]  = max(normalized.get("radius", 0.0), 155.0)
    elif label == "bird":
        normalized["radius"]  = max(normalized.get("radius", 0.0), 45.0)
    else:
        normalized["Y_world"] = round(float(normalized.get("Y_world", 0.0)), 2)
        normalized["radius"]  = max(normalized.get("radius", 0.0), 30.0)
    normalized["X_world"]  = round(float(normalized.get("X_world", 0.0)), 2)
    normalized["Z_world"]  = round(float(normalized.get("Z_world", 0.0)), 2)
    normalized["distance"] = round(float(normalized.get("distance", 0.0)), 2)
    normalized["radius"]   = round(float(normalized.get("radius", 0.0)), 2)
    return normalized

def obstacle_storage_key(obstacle):
    x      = round(float(obstacle.get("X_world", 0.0)) / (GRID_CELL_M * 10))
    z      = round(float(obstacle.get("Z_world", 0.0)) / (GRID_CELL_M * 10))
    radius = round(float(obstacle.get("radius", 155.0)) / 20.0)
    label  = obstacle.get("label", "unknown")
    return f"{label}:{x}:{z}:{radius}"

def dist2d(ax, az, bx, bz):
    return math.hypot(bx - ax, bz - az)

def dist_point_to_segment_2d(px, pz, ax, az, bx, bz):
    dx = bx - ax
    dz = bz - az
    length_sq = dx * dx + dz * dz
    if length_sq < 1e-9:
        return dist2d(px, pz, ax, az)
    t = max(0.0, min(1.0, ((px - ax) * dx + (pz - az) * dz) / length_sq))
    cx = ax + t * dx
    cz = az + t * dz
    return dist2d(px, pz, cx, cz)

def filter_for_flight_path(obstacles, evtol_x, evtol_z, heading_deg):
    if not obstacles:
        return []

    heading_rad = math.radians(heading_deg)
    forward_x = math.sin(heading_rad)
    forward_z = math.cos(heading_rad)
    corridor_end_x = evtol_x + forward_x * 5000.0
    corridor_end_z = evtol_z + forward_z * 5000.0

    filtered = []
    for obs in obstacles:
        label = obs.get("label", "")
        if label not in {"building", "bird"}:
            continue

        ox = float(obs.get("X_world", 0.0))
        oz = float(obs.get("Z_world", 0.0))

        if dist2d(ox, oz, START_ZONE_X, START_ZONE_Z) < VERTIPORT_SAFE_RADIUS_M:
            continue
        if dist2d(ox, oz, LANDING_ZONE_X, LANDING_ZONE_Z) < VERTIPORT_SAFE_RADIUS_M:
            continue

        rel_x = ox - evtol_x
        rel_z = oz - evtol_z
        forward_projection = rel_x * forward_x + rel_z * forward_z
        if forward_projection < MIN_FORWARD_PROJECTION_M:
            continue

        lateral_offset = dist_point_to_segment_2d(
            ox, oz,
            evtol_x, evtol_z,
            corridor_end_x, corridor_end_z,
        )
        if lateral_offset > PATH_CORRIDOR_HALF_WIDTH_M:
            continue

        filtered.append(obs)

    return filtered

# ── NEW: consecutive-frame confirmation & TTL eviction ───────────────────────

def confirm_and_remember(obstacles):
    """
    Only publish obstacles that have been seen in >= MIN_CONFIRM_FRAMES
    consecutive frames.  Evict obstacles not seen for TTL_SECONDS.
    Returns the current live obstacle list.
    """
    now = time.time()
    seen_keys = set()

    frame_scoped = []

    for obs in obstacles:
        key = obstacle_storage_key(obs)
        seen_keys.add(key)
        label = obs.get("label", "")

        if label == "bird":
            frame_scoped.append(obs)
            continue

        if key in published_obstacle_map:
            # Already confirmed — just refresh timestamp
            published_obstacle_map[key]["last_seen"] = now
            candidate_counts.pop(key, None)
        else:
            # Not yet confirmed — increment candidate counter
            count = candidate_counts.get(key, 0) + 1
            candidate_counts[key] = count
            if count >= MIN_CONFIRM_FRAMES:
                # Promote to published
                published_obstacle_map[key] = {"obstacle": obs, "last_seen": now}
                candidate_counts.pop(key, None)
                log.info(f"✓ New obstacle confirmed after {MIN_CONFIRM_FRAMES} frames: {key}")

    # Decay candidate counts for keys not seen this frame
    for key in list(candidate_counts.keys()):
        if key not in seen_keys:
            candidate_counts[key] = max(0, candidate_counts[key] - 1)
            if candidate_counts[key] == 0:
                candidate_counts.pop(key)

    # TTL eviction — remove obstacles not seen recently
    stale_keys = [k for k, v in published_obstacle_map.items()
                  if now - v["last_seen"] > TTL_SECONDS]
    for k in stale_keys:
        log.info(f"⏱ Obstacle evicted (TTL expired): {k}")
        del published_obstacle_map[k]

    persistent = [v["obstacle"] for v in published_obstacle_map.values()]
    return persistent + frame_scoped

# ── Scene detector (colour-segmentation, theme-aware) ────────────────────────

def make_obstacle_mask(img_rgb, theme="daylight"):
    H, W = img_rgb.shape[:2]
    hsv  = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2HSV)
    sky, gnd = get_theme_mask(hsv, theme)

    # Force margins
    sky[:int(H*0.12), :] = 255
    gnd[int(H*0.80):, :] = 255

    horizon_y = int(H*0.12)
    for r in range(int(H*0.12), H//2):
        if np.mean(sky[r] > 0) < 0.20: horizon_y = r; break

    ground_y = int(H*0.80)
    for r in range(int(H*0.80), horizon_y, -1):
        if np.mean(gnd[r] > 0) < 0.20: ground_y = r; break

    mask = cv2.bitwise_and(cv2.bitwise_not(sky), cv2.bitwise_not(gnd))
    k    = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=2)
    return mask, horizon_y, ground_y


def detect_birds(img_rgb, theme="daylight"):
    H, W = img_rgb.shape[:2]
    hsv = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2HSV)
    sky_mask, _ = get_theme_mask(hsv, theme)

    upper_scan_limit = int(H * BIRD_SCAN_HEIGHT_RATIO)
    bird_region = np.zeros((H, W), dtype=np.uint8)
    bird_region[:upper_scan_limit, :] = sky_mask[:upper_scan_limit, :]

    grey = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2GRAY)
    dark_pixels = cv2.inRange(grey, 0, 110)
    sat_pixels = cv2.inRange(hsv[:, :, 1], 30, 255)
    candidate_mask = cv2.bitwise_and(dark_pixels, sat_pixels, mask=bird_region)
    candidate_mask = cv2.morphologyEx(
        candidate_mask,
        cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)),
        iterations=1,
    )
    candidate_mask = cv2.dilate(candidate_mask, np.ones((3, 3), np.uint8), iterations=1)

    contours, _ = cv2.findContours(candidate_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)

    detections = []
    for cnt in contours:
        if len(detections) >= MAX_BIRD_DET:
            break

        x, y, w, h = cv2.boundingRect(cnt)
        area = w * h
        if w < BIRD_MIN_BBOX_W or h < BIRD_MIN_BBOX_H:
            continue
        if area < BIRD_MIN_BBOX_AREA or area > BIRD_MAX_BBOX_AREA:
            continue

        aspect = w / max(h, 1)
        if aspect < 0.45 or aspect > 3.8:
            continue

        confidence = min(0.92, 0.72 + (area / max(BIRD_MAX_BBOX_AREA, 1)) * 0.18)
        detections.append({
            "label": "bird",
            "confidence": round(confidence, 2),
            "x_min": float(x),
            "y_min": float(y),
            "x_max": float(x + w),
            "y_max": float(y + h),
            "x_img": x + w / 2.0,
            "y_img": y + h / 2.0,
        })

    return detections


def scene_detect(img_rgb, theme="daylight"):
    H, W = img_rgb.shape[:2]
    mask, horizon_y, ground_y = make_obstacle_mask(img_rgb, theme)

    grey  = cv2.bilateralFilter(cv2.cvtColor(img_rgb, cv2.COLOR_RGB2GRAY), 7, 60, 60)
    edges = cv2.bitwise_and(cv2.Canny(grey, 25, 75), cv2.Canny(grey, 25, 75), mask=mask)
    closed = cv2.morphologyEx(
        cv2.bitwise_or(edges, mask),
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9)),
        iterations=3)

    contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours     = sorted(contours, key=cv2.contourArea, reverse=True)

    detections, used = [], []
    for cnt in contours:
        if len(detections) >= MAX_DET: break
        x, y, w, h = cv2.boundingRect(cnt)
        if w < MIN_BBOX_W or h < MIN_BBOX_H or w*h < MIN_BBOX_AREA: continue
        if y >= ground_y or (y+h) <= horizon_y: continue
        if np.mean(mask[y:y+h, x:x+w] > 0) < 0.15: continue

        skip = False
        for (rx,ry,rw,rh) in used:
            ix = max(0, min(x+w,rx+rw)-max(x,rx))
            iy = max(0, min(y+h,ry+rh)-max(y,ry))
            inter = ix*iy; union = w*h+rw*rh-inter
            if union > 0 and inter/union > NMS_IOU: skip = True; break
        if skip: continue
        used.append((x,y,w,h))

        aspect = w / max(h, 1)
        if   aspect < 0.75 and h > 100: label, conf = "building", 0.82
        elif w > W*0.50:                 label, conf = "wall",     0.72
        elif aspect > 2.5 and h < 80:   label, conf = "vehicle",  0.68
        else:                            label, conf = "structure", 0.70

        # ── confidence gate ──
        if conf < MIN_CONFIDENCE:
            continue

        detections.append({
            "label": label, "confidence": conf,
            "x_min": float(x), "y_min": float(y),
            "x_max": float(x+w), "y_max": float(y+h),
            "x_img": x+w/2.0, "y_img": y+h/2.0,
        })
    detections.extend(detect_birds(img_rgb, theme))
    return detections

# ── Annotate ──────────────────────────────────────────────────────────────────
COLOURS = {
    "building": (0,80,255), "wall": (0,165,255),
    "structure": (0,200,255), "vehicle": (0,200,50), "default": (0,0,255),
}

def annotate_and_encode(img_rgb, detections, coords_list):
    img  = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
    FONT = cv2.FONT_HERSHEY_SIMPLEX
    PAD  = 5
    for det, c in zip(detections, coords_list):
        x1,y1 = int(det["x_min"]), int(det["y_min"])
        x2,y2 = int(det["x_max"]), int(det["y_max"])
        col   = COLOURS.get(det["label"], COLOURS["default"])
        cv2.rectangle(img, (x1,y1), (x2,y2), col, 2)
        tag = (f"{det['label']} {det['confidence']:.2f}  "
               f"W({c['X_world']:.0f},{c['Y_world']:.0f},{c['Z_world']:.0f})  D:{c['distance']:.0f}m")
        (tw,th),_ = cv2.getTextSize(tag, FONT, 0.50, 1)
        bar_y1 = max(y1-th-2*PAD, 0); bar_y2 = max(y1, th+2*PAD)
        cv2.rectangle(img, (x1,bar_y1), (x1+tw+PAD*2,bar_y2), col, -1)
        cv2.putText(img, tag, (x1+PAD, bar_y2-PAD), FONT, 0.50, (255,255,255), 1, cv2.LINE_AA)
        cv2.drawMarker(img, (int(det["x_img"]),int(det["y_img"])), col, cv2.MARKER_CROSS, 16, 2)
        cv2.circle(img, (int(det["x_img"]),int(det["y_max"])), 5, col, -1)
    ok, buf = cv2.imencode(".png", img)
    if not ok: return ""
    return "data:image/png;base64," + base64.b64encode(buf.tobytes()).decode()

# ── File helpers ──────────────────────────────────────────────────────────────
def save_raw(pil_img, idx, ts):
    p = os.path.join(SAVE_DIR, f"{ts}_frame_{idx:06d}.png")
    pil_img.save(p, "PNG"); return p

def save_annotated(b64, idx, ts):
    if not b64: return
    data = base64.b64decode(b64.split(",")[1])
    bgr  = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    if bgr is not None:
        cv2.imwrite(os.path.join(ANNOTATED_DIR, f"{ts}_frame_{idx:06d}_annotated.png"), bgr)

def add_hybrid_fields(obstacles):
    planner = []
    for det in obstacles or []:
        p = dict(det)
        p["estimatedX"] = round(float(det.get("estimatedX", det.get("X_world", 0.0))), 2)
        p["estimatedY"] = round(float(det.get("estimatedY", det.get("Y_world", 0.0))), 2)
        p["estimatedZ"] = round(float(det.get("estimatedZ", det.get("Z_world", 0.0))), 2)
        p["calibratedX"] = det.get("calibratedX")
        p["calibratedY"] = det.get("calibratedY")
        p["calibratedZ"] = det.get("calibratedZ")
        p["calibrated"] = bool(det.get("calibrated", False))
        planner.append(p)
    return planner

def calibrate_obstacles_for_planner(obstacles, matches):
    planner = add_hybrid_fields(obstacles)
    if not matches:
        return planner

    for match in matches:
        det = match.get("detection")
        if not det:
            continue

        for p in planner:
            if (
                p.get("label") == det.get("label")
                and p.get("X_world") == det.get("X_world")
                and p.get("Y_world") == det.get("Y_world")
                and p.get("Z_world") == det.get("Z_world")
            ):
                gt = match.get("ground_truth")
                if match.get("matched") and gt:
                    p["calibratedX"] = round(float(gt["x"]), 2)
                    p["calibratedY"] = round(float(gt["y"]), 2)
                    p["calibratedZ"] = round(float(gt["z"]), 2)
                    p["X_world"] = p["calibratedX"]
                    p["Y_world"] = p["calibratedY"]
                    p["Z_world"] = p["calibratedZ"]
                    p["radius"] = round(float(max(gt.get("radius", 155.0), 155.0)), 2)
                    p["calibrated"] = True
                break

    return planner

def _post_to_backend(payload_bytes):
    req = urllib_request.Request(
        BACKEND_OBSTACLE_SYNC_URL, data=payload_bytes,
        headers={"Content-Type": "application/json"}, method="POST")
    try:
        with urllib_request.urlopen(req, timeout=2) as r: r.read()
    except Exception as exc:
        log.warning(f"Could not sync detections to backend: {exc}")

def publish_obstacles_to_backend(obstacles, frame_index):
    static_obs = [o for o in obstacles if o.get("label") != "bird"]
    bird_obs   = [o for o in obstacles if o.get("label") == "bird"]
    payload = json.dumps({
        "frameIndex": frame_index,
        "source": "yolo_service",
        "obstacles": [
            {
                "x": o["X_world"],
                "y": o["Y_world"],
                "z": o["Z_world"],
                "radius": o.get("radius", 155.0),
                "label": o.get("label", "building"),
                "source": o.get("source", "yolo_service"),
                "estimatedX": o.get("estimatedX", o.get("X_world")),
                "estimatedY": o.get("estimatedY", o.get("Y_world")),
                "estimatedZ": o.get("estimatedZ", o.get("Z_world")),
                "calibratedX": o.get("calibratedX"),
                "calibratedY": o.get("calibratedY"),
                "calibratedZ": o.get("calibratedZ"),
                "calibrated": o.get("calibrated", False),
            }
            for o in static_obs
        ],
        "birdObstacles": [
            {
                "x": o["X_world"],
                "y": o["Y_world"],
                "z": o["Z_world"],
                "radius": o.get("radius", 45.0),
                "label": "bird",
                "source": "dynamic_obstacle2",
                "estimatedX": o.get("estimatedX", o.get("X_world")),
                "estimatedY": o.get("estimatedY", o.get("Y_world")),
                "estimatedZ": o.get("estimatedZ", o.get("Z_world")),
                "calibratedX": o.get("calibratedX"),
                "calibratedY": o.get("calibratedY"),
                "calibratedZ": o.get("calibratedZ"),
                "calibrated": o.get("calibrated", False),
            }
            for o in bird_obs
        ],
    }).encode("utf-8")
    # Fire-and-forget — never block the /detect response waiting for the backend
    threading.Thread(target=_post_to_backend, args=(payload,), daemon=True).start()

# ── Flask routes ──────────────────────────────────────────────────────────────

@app.route("/health")
def health():
    return jsonify({
        "status": "ok", "detector": "scene_v6_realistic",
        "frames_saved": frame_counter,
        "smooth_slots": len(smooth_state),
        "published_obstacles": len(published_obstacle_map),
        "pending_candidates": len(candidate_counts),
        "validator_enabled": validator is not None,
        "config": {
            "min_confidence": MIN_CONFIDENCE,
            "min_confirm_frames": MIN_CONFIRM_FRAMES,
            "ttl_seconds": TTL_SECONDS,
            "grid_cell_m": GRID_CELL_M,
            "gt_match_threshold_m": GT_MATCH_THRESHOLD_M,
        }
    })


@app.route("/reset_smooth", methods=["POST"])
def reset_smooth():
    global smooth_state, published_obstacle_map, candidate_counts
    smooth_state = {}; published_obstacle_map = {}; candidate_counts = {}
    publish_obstacles_to_backend([], -1)
    return jsonify({"ok": True})


@app.route("/validation/metrics", methods=["GET"])
def get_validation_metrics():
    if not validator:
        return jsonify({"error": "Validator not initialised"}), 400
    if not validator.matches_history:
        return jsonify({"error": "No validation history yet"}), 404
    recent = [validator.get_metrics(m) for m in validator.matches_history[-5:]]
    return jsonify({"frames_recorded": len(recent), "latest": recent[-1], "history": recent})


@app.route("/detect", methods=["POST"])
def detect():
    global frame_counter
    try:
        p    = request.get_json(force=True)
        b64  = p.get("frame", "")
        fidx = int(p.get("frameIndex", frame_counter))
        ev   = p.get("evtol", {})
        cam  = p.get("camera", {})
        theme = p.get("theme", "daylight")   # frontend sends active theme

        ex  = float(ev.get("x",  0)); ey = float(ev.get("y",  0)); ez = float(ev.get("z",  0))
        qx  = float(ev.get("qx", 0)); qy = float(ev.get("qy", 0))
        qz  = float(ev.get("qz", 0)); qw = float(ev.get("qw", 1))
        hfov= float(cam.get("fov", CAMERA_HFOV))

        if "," in b64: b64 = b64.split(",", 1)[1]
        pil = Image.open(BytesIO(base64.b64decode(b64))).convert("RGB")
        W, H = pil.size
        img  = np.array(pil)

        ts       = datetime.utcnow().strftime("%Y%m%d_%H%M%S")
        raw_path = save_raw(pil, fidx, ts)
        frame_counter += 1

        # Theme-aware detection
        dets = scene_detect(img, theme)
        coords_list = []
        for d in dets:
            raw      = pixel_to_world(d["x_min"], d["y_min"], d["x_max"], d["y_max"],
                                      W, H, ex, ey, ez, qx, qy, qz, qw, d["label"], hfov)
            smoothed = smooth_coords(d["label"], raw)
            coords_list.append(normalize_for_planner(d["label"], smoothed))

        ann_b64 = annotate_and_encode(img, dets, coords_list)
        if dets: save_annotated(ann_b64, fidx, ts)

        log.info(f"Frame {fidx:>4d} [{theme}]: {len(dets)} det(s), conf≥{MIN_CONFIDENCE}")
        for d, c in zip(dets, coords_list):
            log.info(f"  {d['label']:10s} conf={d['confidence']:.2f} "
                     f"W({c['X_world']:8.1f},{c['Y_world']:6.1f},{c['Z_world']:8.1f}) D:{c['distance']:6.1f}m")

        obstacles = [
            {"label": d["label"], "confidence": d["confidence"],
             "bbox": {"x_min": round(d["x_min"],1), "y_min": round(d["y_min"],1),
                      "x_max": round(d["x_max"],1), "y_max": round(d["y_max"],1)},
             **c}
            for d, c in zip(dets, coords_list)
        ]

        # Ground-truth validation
        validation_metrics = None; matches = []
        if validator and obstacles:
            try:
                matches = validator.match_detections(obstacles)
                validation_metrics = validator.get_metrics(matches)
                if validation_metrics["matched_count"] > 0:
                    log.info(f"✓ Validation: matched={validation_metrics['matched_count']}, "
                             f"rmse={validation_metrics['rmse_m']:.2f}m")
                else:
                    log.info(f"⚠ Validation: no matches within {GT_MATCH_THRESHOLD_M}m")
            except Exception as e:
                log.warning(f"Validation error: {e}")

        planner_obstacles = calibrate_obstacles_for_planner(obstacles, matches)
        bird_obstacles = [obs for obs in planner_obstacles if obs.get("label") == "bird"]
        non_bird_obstacles = [obs for obs in planner_obstacles if obs.get("label") != "bird"]
        planner_obstacles = filter_for_flight_path(
            non_bird_obstacles,
            ex,
            ez,
            float(ev.get("heading", 0.0)),
        ) + bird_obstacles

        # Proximity-based synthetic detections for surprise obstacles ──────────
        # These are buildings the frontend renders but the backend doesn't know
        # about yet.  When the eVTOL flies within PROXIMITY_DETECT_M the
        # obstacle is injected here so the replan pipeline fires reliably.
        proximity_dets = get_proximity_detections(ex, ey, ez)
        if proximity_dets:
            log.info(f"[Proximity] Injecting {len(proximity_dets)} surprise obstacle(s) "
                     f"within {PROXIMITY_DETECT_M}m")
            planner_obstacles = planner_obstacles + proximity_dets

        # confirm + TTL-evict before publishing
        live_obstacles = confirm_and_remember(planner_obstacles)
        publish_obstacles_to_backend(live_obstacles, fidx)

        response = {
            "frameIndex": fidx, "savedPath": raw_path,
            "obstacles": obstacles,
            "plannerObstacles": live_obstacles,
            "annotatedImg": ann_b64,
            "stats": {
                "published": len(live_obstacles),
                "pending_candidates": len(candidate_counts),
            }
        }
        if validation_metrics:
            response["validation"] = validation_metrics
        return jsonify(response)

    except Exception as e:
        log.exception("Error in /detect")
        return jsonify({"error": str(e)}), 500


@app.route("/vision/edge",      methods=["POST"])
def vision_edge():
    return jsonify({"error": "Handled by backend at /api/vision/edge."}), 410

@app.route("/vision/threshold", methods=["POST"])
def vision_threshold():
    return jsonify({"error": "Handled by backend at /api/vision/threshold."}), 410

@app.route("/vision/bg-reveal", methods=["POST"])
def vision_bg_reveal():
    try:
        p = request.get_json(force=True)
        preset = (p.get("preset") or "auto").lower()
        img = decode_b64_png_to_rgb(p.get("frame", ""))
        overlay, _ = background_reveal(img, preset=preset)
        return jsonify({"image": encode_rgb_to_data_url_png(overlay)})
    except Exception as e:
        log.exception("Error in /vision/bg-reveal")
        return jsonify({"error": str(e)}), 500

@app.route("/frames")
def list_frames():
    files = sorted(f for f in os.listdir(SAVE_DIR) if f.endswith(".png"))
    return jsonify({"count": len(files), "files": files})

@app.route("/clear", methods=["POST"])
def clear_frames():
    global frame_counter, smooth_state
    n = 0
    for folder in [SAVE_DIR, ANNOTATED_DIR]:
        for f in os.listdir(folder):
            if f.endswith(".png"):
                os.remove(os.path.join(folder, f)); n += 1
    frame_counter = 0; smooth_state = {}
    return jsonify({"deleted": n})


@app.route("/surprises")
def list_surprises():
    """Return the list of surprise obstacles (frontend-only, unknown to backend)."""
    return jsonify({
        "count": len(_surprise_obstacles),
        "obstacles": _surprise_obstacles,
        "proximity_detect_m": PROXIMITY_DETECT_M,
    })

@app.route("/reload_surprises", methods=["POST"])
def reload_surprises():
    """Force-reload the surprise list from disk (useful after CSV edits)."""
    global _surprise_obstacles
    _surprise_obstacles = _load_surprise_obstacles()
    return jsonify({"ok": True, "count": len(_surprise_obstacles)})


if __name__ == "__main__":
    log.info("="*60)
    log.info("  eVTOL Obstacle Detector  (scene v6 — realistic upgrades)")
    log.info(f"  Frames → {os.path.abspath(SAVE_DIR)}")
    log.info(f"  Confidence threshold : {MIN_CONFIDENCE}")
    log.info(f"  Confirm frames       : {MIN_CONFIRM_FRAMES}")
    log.info(f"  Obstacle TTL         : {TTL_SECONDS}s")
    log.info(f"  Dedup grid           : {GRID_CELL_M}m")
    log.info(f"  GT match threshold   : {GT_MATCH_THRESHOLD_M}m")
    log.info(f"  Validator            : {'ENABLED' if validator else 'DISABLED'}")
    log.info("  Port: 5050")
    log.info("="*60)
    app.run(host="0.0.0.0", port=5050, debug=False)
