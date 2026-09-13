#!/usr/bin/env python3
"""yolo_jetson_server.py -- draait OP DE JETSON. Ontvangt af en toe een
kleur+diepte-frame van de Pi (geen continue stream, zelfde les als het
afgeschreven RTAB-Map-Pi/Jetson/WiFi-plan, zie muto_rtabmap_abandoned_for_
load-memory), draait YOLO op het kleurbeeld (GPU-versneld via ultralytics),
en zoekt voor elke detectie de afstand op in het (geregistreerde) dieptebeeld.

Wire-formaat: multipart/form-data POST naar /detect met twee bestanden:
  color: JPEG
  depth: PNG, 16-bit grayscale, mm (zelfde resolutie/uitlijning als color --
         vereist depth_registration:=true bij het starten van astra_camera
         op de Pi-kant, anders kloppen de pixelcoordinaten niet).
Antwoord: JSON lijst van {label, confidence, bbox:[x1,y1,x2,y2], distance_m}.
"""
import io
import time

import numpy as np
from flask import Flask, request, jsonify
from PIL import Image
from ultralytics import YOLO

app = Flask(__name__)
print("YOLO-model laden...")
model = YOLO("yolov8n.pt")  # kleinste model -- snel genoeg voor incidentele snapshots
print("Model geladen, klaar om te ontvangen.")


def median_depth_mm(depth_arr: np.ndarray, x1: int, y1: int, x2: int, y2: int) -> float | None:
    """Mediaan van de geldige (niet-nul) dieptewaarden binnen de bounding
    box -- robuuster dan het centrum-pixel alleen (dat kan toevallig op een
    gat/rand vallen)."""
    region = depth_arr[max(0, y1):y2, max(0, x1):x2]
    valid = region[region > 0]
    if valid.size == 0:
        return None
    return float(np.median(valid))


@app.route("/detect", methods=["POST"])
def detect():
    t0 = time.time()
    if "color" not in request.files:
        return jsonify({"error": "geen 'color'-bestand in de request"}), 400

    color_img = Image.open(io.BytesIO(request.files["color"].read())).convert("RGB")
    color_arr = np.array(color_img)

    depth_arr = None
    if "depth" in request.files:
        depth_img = Image.open(io.BytesIO(request.files["depth"].read()))
        depth_arr = np.array(depth_img)  # verwacht uint16, mm

    results = model(color_arr, verbose=False)[0]

    detections = []
    for box in results.boxes:
        x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]
        label = model.names[int(box.cls[0])]
        confidence = float(box.conf[0])

        distance_m = None
        if depth_arr is not None and depth_arr.shape[:2] == color_arr.shape[:2]:
            d_mm = median_depth_mm(depth_arr, x1, y1, x2, y2)
            if d_mm is not None:
                distance_m = round(d_mm / 1000.0, 2)

        detections.append({
            "label": label,
            "confidence": round(confidence, 2),
            "bbox": [x1, y1, x2, y2],
            "distance_m": distance_m,
        })

    elapsed_ms = round((time.time() - t0) * 1000)
    print(f"[detect] {len(detections)} object(en), {elapsed_ms}ms -- {[d['label'] for d in detections]}")
    return jsonify({"detections": detections, "elapsed_ms": elapsed_ms})


@app.route("/health", methods=["GET"])
def health():
    return jsonify({"ok": True, "model": "yolov8n"})


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8600)
