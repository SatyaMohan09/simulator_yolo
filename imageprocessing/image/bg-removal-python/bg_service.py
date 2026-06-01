import ssl
ssl._create_default_https_context = ssl._create_unverified_context
from flask import Flask, request, send_file
from flask_cors import CORS
from rembg import remove, new_session
from PIL import Image
import numpy as np
import cv2
import io

app = Flask(__name__)
CORS(app)

# Load AI models once
u2net_session = new_session("u2net")
isnet_session = new_session("isnet-general-use")

# SKY DOMINANCE DETECTION
def is_sky_dominant(image: np.ndarray) -> bool:
    hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    _, s, v = cv2.split(hsv)

    sky_pixels = np.sum((v > 185) & (s < 80))
    ratio = sky_pixels / (image.shape[0] * image.shape[1])
    return ratio > 0.35

# DETECT SMALL OBJECTS ONLY (BIRDS)
def has_only_small_sky_objects(image: np.ndarray) -> bool:
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    dark_pixels = gray < 90
    dark_ratio = np.sum(dark_pixels) / dark_pixels.size
    return dark_ratio < 0.08

# PRESERVE SMALL DARK OBJECTS (BIRDS)
def preserve_small_dark_objects(image: np.ndarray, alpha: np.ndarray) -> np.ndarray:
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    edges = cv2.Canny(gray, 80, 160)
    dark = gray < 90

    restore = (edges > 0) & dark
    restore = cv2.dilate(
        restore.astype(np.uint8),
        np.ones((3, 3), np.uint8)
    )

    alpha[restore > 0] = 255
    return alpha

# WINDOWS‑LIKE SKY REMOVAL

def remove_sky(image: np.ndarray) -> Image.Image:
    h, w, _ = image.shape
    hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    _, s, v = cv2.split(hsv)

    sky_candidate = np.zeros((h, w), dtype=np.uint8)
    sky_candidate[(v > 180) & (s < 90)] = 255

    # Keep only sky connected to the top of the image
    flood_mask = np.zeros((h + 2, w + 2), np.uint8)
    connected = sky_candidate.copy()
    cv2.floodFill(connected, flood_mask, (0, 0), 255)
    sky_mask = cv2.bitwise_and(connected, sky_candidate)

    sky_mask = cv2.GaussianBlur(sky_mask, (7, 7), 0)

    alpha = 255 - sky_mask
    alpha[alpha > 200] = 255
    alpha[alpha <= 200] = 0

    # Restore birds
    alpha = preserve_small_dark_objects(image, alpha)

    rgba = np.dstack((image, alpha))
    return Image.fromarray(rgba, "RGBA")

# ISNET CONFIDENCE CHECK
def is_isnet_confident(rgba: Image.Image) -> bool:
    np_img = np.array(rgba)
    alpha = np_img[:, :, 3]
    fg_ratio = np.sum(alpha > 240) / alpha.size

    # Small sharp foreground → birds or objects
    return fg_ratio < 0.15

# API ENDPOINT
@app.route("/remove-bg", methods=["POST"])
def remove_bg():
    if "image" not in request.files:
        return "No image uploaded", 400

    img_rgb = Image.open(request.files["image"].stream).convert("RGB")
    np_img = np.array(img_rgb)

    # SKY‑DOMINANT IMAGES
    if is_sky_dominant(np_img):
        if has_only_small_sky_objects(np_img):
            print("Birds‑only sky detected → ISNet")
            output = remove(img_rgb, session=isnet_session).convert("RGBA")
        else:
            print("Skyline detected → sky removal")
            output = remove_sky(np_img)

    # NON‑SKY IMAGES

    else:
        print("Trying ISNet")
        isnet_out = remove(img_rgb, session=isnet_session).convert("RGBA")

        if is_isnet_confident(isnet_out):
            print(" ISNet confident → using ISNet output")
            output = isnet_out
        else:
            print("Fallback → U²‑Net")
            output = remove(img_rgb, session=u2net_session).convert("RGBA")

    buf = io.BytesIO()
    output.save(buf, format="PNG")
    buf.seek(0)

    return send_file(buf, mimetype="image/png")

# RUN SERVER
if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5001)
