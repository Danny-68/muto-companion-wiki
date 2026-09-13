#!/usr/bin/env python3
"""yolo_snapshot_sender.py -- draait in de humble_run-container op de Pi.
Pakt PERIODIEK (niet continu) een kleur+diepte-frame van de Astra-camera en
stuurt ze naar yolo_jetson_server.py op de Jetson. Bewust geen streaming --
zelfde les als het afgeschreven RTAB-Map-plan (Pi/Jetson/WiFi-belasting),
zie muto_rtabmap_abandoned_for_load-memory.

Vereist dat astra_camera gestart is MET depth_registration:=true, anders
komen kleur- en diepte-pixelcoordinaten niet overeen en klopt de
afstandsschatting per detectie niet.
"""
import argparse
import io
import json
import socket
import subprocess
import time

import requests
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from PIL import Image as PILImage

JETSON_URL = "http://192.168.68.86:8600/detect"
SNAPSHOT_INTERVAL_S = 5.0
MUTOD_HOST, MUTOD_PORT = "127.0.0.1", 8420
BEEP_TIMEOUT = 3  # 300ms -- kort "ik zag iets nieuws"-piepje
# 13 sep 2026: espeak-ng (formant-synthese) was slecht verstaanbaar, ook in het
# Engels en met een lagere spreeksnelheid -- vervangen door Piper (neurale TTS,
# lokaal/offline, zoals Reachy's cloud-TTS maar zonder de cloud-afhankelijkheid,
# zie muto_reachy_tts_research_2026-09-13-memory), user-confirmed "veel beter!!".
PIPER_BIN = "/root/piper_venv/bin/piper"
PIPER_MODEL = "/root/piper_voices/en_US-amy-medium.onnx"
PIPER_SAMPLE_RATE = 22050
# USB-speaker kaartnummer kan per boot wisselen (zie muto_audio_module_fix_2026-09-13-memory)
# -- vandaar hier als losse, makkelijk te updaten constante i.p.v. hardcoded verderop.
TTS_ALSA_DEVICE = "plughw:2,0"


class MutodBuzzer:
    """Minimale TCP-client naar mutod's buzzer-op -- zelfde newline-JSON-
    kanaal als overal elders, hier alleen voor het piepje. Faalt stil (met
    een warning) als mutod niet bereikbaar is -- een gemiste piep mag nooit
    de detectie-pijplijn zelf breken."""

    def __init__(self, logger):
        self._logger = logger

    def beep(self, timeout: int = BEEP_TIMEOUT):
        try:
            sock = socket.create_connection((MUTOD_HOST, MUTOD_PORT), timeout=2)
            sock.sendall((json.dumps({"op": "buzzer", "timeout": timeout}) + "\n").encode())
            sock.recv(256)
            sock.close()
        except OSError as exc:
            self._logger.warn(f"kon niet piepen (mutod niet bereikbaar?): {exc}")


class Tts:
    """13 sep 2026: de USB-speaker bleek geen kapotte chip, een udev-regel
    blokkeerde de ALSA-driver (zie muto_audio_module_fix_2026-09-13-memory) --
    nu gefixed, dus echte spraak i.p.v. alleen een piepje is een reele optie.
    Piper (lokale neurale TTS, zie /root/piper_venv) i.p.v. espeak-ng --
    espeak-ng's formant-synthese was zelfs in het Engels slecht verstaanbaar,
    Piper is user-confirmed "veel beter!!". Piper -> aplay via een pipe,
    fire-and-forget (niet gewacht op het resultaat) zodat een trage/falende
    TTS-aanroep de detectie-cyclus (die elke SNAPSHOT_INTERVAL_S al een
    netwerk-call naar de Jetson doet) nooit blokkeert."""

    def __init__(self, logger):
        self._logger = logger

    def say(self, text: str):
        try:
            piper = subprocess.Popen(
                [PIPER_BIN, "--model", PIPER_MODEL, "--output-raw"],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
            )
            subprocess.Popen(
                ["aplay", "-r", str(PIPER_SAMPLE_RATE), "-f", "S16_LE", "-t", "raw", "-D", TTS_ALSA_DEVICE],
                stdin=piper.stdout,
            )
            piper.stdout.close()
            piper.stdin.write(text.encode())
            piper.stdin.close()
        except OSError as exc:
            self._logger.warn(f"kon niet spreken (piper/aplay niet gevonden?): {exc}")


class YoloSnapshotSender(Node):
    def __init__(self, interval_s: float, beep_enabled: bool = False, speak_enabled: bool = False):
        super().__init__("yolo_snapshot_sender")
        self.bridge = CvBridge()
        self.interval_s = interval_s
        self.beep_enabled = beep_enabled
        self.speak_enabled = speak_enabled
        self.latest_color = None
        self.latest_depth = None
        self.buzzer = MutodBuzzer(self.get_logger())
        self.tts = Tts(self.get_logger())
        self.previous_labels = set()

        self.create_subscription(Image, "/camera/color/image_raw", self._color_cb, qos_profile_sensor_data)
        self.create_subscription(Image, "/camera/depth/image_raw", self._depth_cb, qos_profile_sensor_data)
        self.create_timer(self.interval_s, self._tick)
        self.get_logger().info(f"yolo_snapshot_sender gestart -- elke {self.interval_s:.0f}s een snapshot naar {JETSON_URL}")

    def _color_cb(self, msg):
        self.latest_color = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")

    def _depth_cb(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def _tick(self):
        if self.latest_color is None:
            self.get_logger().warn("nog geen kleurbeeld ontvangen, sla deze snapshot over")
            return

        color_buf = io.BytesIO()
        PILImage.fromarray(self.latest_color).save(color_buf, format="JPEG", quality=85)
        color_buf.seek(0)
        files = {"color": ("color.jpg", color_buf, "image/jpeg")}

        if self.latest_depth is not None and self.latest_depth.shape[:2] == self.latest_color.shape[:2]:
            depth_buf = io.BytesIO()
            PILImage.fromarray(self.latest_depth).save(depth_buf, format="PNG")
            depth_buf.seek(0)
            files["depth"] = ("depth.png", depth_buf, "image/png")
        else:
            self.get_logger().warn("dieptebeeld ontbreekt of komt niet overeen in resolutie -- snapshot zonder afstand")

        t0 = time.time()
        try:
            resp = requests.post(JETSON_URL, files=files, timeout=8)
            resp.raise_for_status()
            data = resp.json()
        except requests.RequestException as exc:
            self.get_logger().warn(f"Jetson niet bereikbaar/fout: {exc}")
            return

        elapsed_ms = round((time.time() - t0) * 1000)
        detections = data.get("detections", [])
        current_labels = {d["label"] for d in detections}

        if detections:
            summary = ", ".join(f"{d['label']}({d['distance_m']}m)" if d["distance_m"] else d["label"] for d in detections)
            self.get_logger().info(f"[{elapsed_ms}ms round-trip] gezien: {summary}")
        else:
            self.get_logger().info(f"[{elapsed_ms}ms round-trip] niets gedetecteerd")

        new_labels = current_labels - self.previous_labels
        if new_labels:
            if self.beep_enabled:
                self.get_logger().info(f"nieuw gezien t.o.v. vorige keer: {sorted(new_labels)} -- piep")
                self.buzzer.beep()
            if self.speak_enabled:
                label_to_distance = {d["label"]: d["distance_m"] for d in detections}
                parts = []
                for label in sorted(new_labels):
                    dist = label_to_distance.get(label)
                    parts.append(f"{label} at {dist} meters" if dist else label)
                phrase = "I see " + " and ".join(parts)
                self.get_logger().info(f"spreek: {phrase!r}")
                self.tts.say(phrase)
            if not self.beep_enabled and not self.speak_enabled:
                self.get_logger().info(f"nieuw gezien t.o.v. vorige keer: {sorted(new_labels)} (audio uit)")
        self.previous_labels = current_labels


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--interval", type=float, default=SNAPSHOT_INTERVAL_S)
    p.add_argument("--beep", action="store_true", help="piep via mutod's buzzer bij een nieuw gedetecteerd object (default: uit)")
    p.add_argument("--speak", action="store_true", help="spreek (espeak-ng, nl) bij een nieuw gedetecteerd object (default: uit)")
    args = p.parse_args()

    rclpy.init()
    node = YoloSnapshotSender(args.interval, beep_enabled=args.beep, speak_enabled=args.speak)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
