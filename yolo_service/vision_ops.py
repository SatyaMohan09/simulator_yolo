import base64
from io import BytesIO
from typing import Literal, Tuple

import cv2
import numpy as np
from PIL import Image


def decode_b64_png_to_rgb(b64: str) -> np.ndarray:
    if not b64:
        raise ValueError("Missing frame")
    if "," in b64:
        b64 = b64.split(",", 1)[1]
    pil = Image.open(BytesIO(base64.b64decode(b64))).convert("RGB")
    return np.array(pil)


def encode_rgb_to_data_url_png(img_rgb: np.ndarray) -> str:
    bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
    ok, buf = cv2.imencode(".png", bgr)
    if not ok:
        raise ValueError("Failed to encode png")
    return "data:image/png;base64," + base64.b64encode(buf.tobytes()).decode()


def _apply_clahe_l_channel(img_rgb: np.ndarray) -> np.ndarray:
    lab = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    l2 = clahe.apply(l)
    lab2 = cv2.merge([l2, a, b])
    return cv2.cvtColor(lab2, cv2.COLOR_LAB2RGB)


def _gamma(img_rgb: np.ndarray, gamma: float) -> np.ndarray:
    if gamma <= 0:
        return img_rgb
    lut = np.array([((i / 255.0) ** (1.0 / gamma)) * 255 for i in range(256)], dtype=np.uint8)
    return cv2.LUT(img_rgb, lut)


def edge_detection(
    img_rgb: np.ndarray,
    preset: Literal["day", "dawn", "dusk", "night", "auto"] = "auto",
) -> Tuple[np.ndarray, np.ndarray]:
    norm = _apply_clahe_l_channel(img_rgb)

    if preset == "night":
        norm = _gamma(norm, 2.2)
        blur = cv2.GaussianBlur(cv2.cvtColor(norm, cv2.COLOR_RGB2GRAY), (5, 5), 0)
    elif preset in ("dawn", "dusk"):
        norm = _gamma(norm, 1.6)
        blur = cv2.GaussianBlur(cv2.cvtColor(norm, cv2.COLOR_RGB2GRAY), (5, 5), 0)
    else:
        blur = cv2.bilateralFilter(cv2.cvtColor(norm, cv2.COLOR_RGB2GRAY), 7, 60, 60)

    med = float(np.median(blur))
    lo = int(max(0, 0.66 * med))
    hi = int(min(255, 1.33 * med))
    edges = cv2.Canny(blur, lo, hi)

    edges_rgb = np.zeros_like(img_rgb)
    edges_rgb[edges > 0] = (255, 255, 255)
    return edges_rgb, edges


def threshold_segmentation(
    img_rgb: np.ndarray,
    preset: Literal["day", "dawn", "dusk", "night", "auto"] = "auto",
) -> Tuple[np.ndarray, np.ndarray]:
    gray = np.clip(
        0.299 * img_rgb[:, :, 0] + 0.587 * img_rgb[:, :, 1] + 0.114 * img_rgb[:, :, 2],
        0,
        255,
    ).astype(np.uint8)
    _, mask = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    out = np.zeros_like(img_rgb)
    out[mask > 0] = (255, 255, 255)
    return out, mask


def background_reveal(
    img_rgb: np.ndarray,
    preset: Literal["day", "dawn", "dusk", "night", "auto"] = "auto",
) -> Tuple[np.ndarray, np.ndarray]:
    edges_rgb, edges = edge_detection(img_rgb, preset=preset)
    seg_rgb, seg = threshold_segmentation(img_rgb, preset=preset)

    combo = cv2.bitwise_or(seg, edges)
    k = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
    combo = cv2.morphologyEx(combo, cv2.MORPH_CLOSE, k, iterations=2)

    overlay = img_rgb.copy()
    green = np.zeros_like(img_rgb)
    green[:, :] = (0, 255, 0)
    overlay = np.where(combo[:, :, None] > 0, (0.65 * overlay + 0.35 * green).astype(np.uint8), overlay)
    overlay = np.where(edges[:, :, None] > 0, np.array([255, 255, 255], dtype=np.uint8), overlay)
    return overlay, combo
