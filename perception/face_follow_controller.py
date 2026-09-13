#!/usr/bin/env python3
"""face_follow_controller.py -- Fase 6 "camera-gerichtheid": sluit de lus
tussen face_tracker.py's /face_bearing en een echt draaicommando via mutod's
Fase 3 robot.move (STM32-firmware-gait TURN_LEFT/TURN_RIGHT, dezelfde,
al fysiek geverifieerde weg als de eerdere TURN-test).

VEILIGHEIDSONTWERP (nieuwe categorie t.o.v. eerdere Fase 3/6-tests: continu,
camera-reactief, onbepaalde duur zolang iemand off-center blijft):
- --dry-run (DEFAULT) logt alleen wat er verstuurd ZOU worden, stuurt niets.
  Pas met --live stuurt hij echte robot.move-commando's.
- Dead-band (BEARING_DEADBAND_DEG): kleine afwijkingen worden genegeerd, geen
  voortdurend micro-jitteren.
- Rate-limit (CORRECTION_INTERVAL_S): hooguit 1 correctie per ~0.6s (mutod's
  eigen deadman-venster), niet op elke camera-frame (~13Hz) reageren.
- Geklemde, kleine correctiesnelheid (MAX_VYAW), dezelfde ordegrootte als de
  al fysiek bevestigde TURN-test uit Fase 3 -- nooit opschalen met de
  bearing-grootte voorbij dit plafond.
- Watchdog (MAX_CONTINUOUS_CORRECTION_S): als er langer dan dit continu
  gecorrigeerd wordt zonder centrering, stopt de lus zichzelf en meldt een
  waarschuwing -- voorkomt onbeheerd doordraaien bij een detectiefout.
- Expliciete robot.stop() zodra het gezicht weg is, gecentreerd is, of de
  watchdog aanslaat -- nooit stilzwijgend laten uitdoven op de deadman alleen.
"""
import argparse
import json
import socket
import threading
import time

MUTOD_HOST, MUTOD_PORT = "127.0.0.1", 8420

BEARING_DEADBAND_DEG = 8.0
CORRECTION_INTERVAL_S = 0.6
MAX_VYAW = 0.10          # rad/s-achtige input voor robot.move, zelfde ordegrootte als de bevestigde Fase 3 TURN-test
GAIN = 0.01              # vyaw = clamp(-GAIN * bearing_deg, -MAX_VYAW, MAX_VYAW)
MAX_CONTINUOUS_CORRECTION_S = 15.0


def rpc_call(sock, rfile, method, params=None, req_id=1):
    req = {"jsonrpc": "2.0", "id": req_id, "method": method}
    if params is not None:
        req["params"] = params
    sock.sendall((json.dumps(req) + "\n").encode())
    return json.loads(rfile.readline())


class FaceFollowController:
    def __init__(self, live: bool):
        self.live = live
        self._sock = None
        self._rfile = None
        self._last_correction_ts = 0.0
        self._correcting_since = None
        self._watchdog_tripped = False

    def _ensure_connected(self):
        if self._sock is not None:
            return
        self._sock = socket.create_connection((MUTOD_HOST, MUTOD_PORT), timeout=5)
        self._rfile = self._sock.makefile("rb")

    def _send_move(self, vyaw: float):
        if not self.live:
            print(f"[DRY-RUN] zou sturen: robot.move vyaw={vyaw:+.3f}")
            return
        self._ensure_connected()
        resp = rpc_call(self._sock, self._rfile, "robot.move", {"vx": 0.0, "vy": 0.0, "vyaw": vyaw})
        print(f"[LIVE] robot.move vyaw={vyaw:+.3f} -> {resp.get('result') or resp.get('error')}")

    def _send_stop(self, reason: str):
        print(f"[STOP] {reason}")
        if not self.live:
            print("[DRY-RUN] zou sturen: robot.stop")
            return
        self._ensure_connected()
        resp = rpc_call(self._sock, self._rfile, "robot.stop")
        print(f"[LIVE] robot.stop -> {resp.get('result') or resp.get('error')}")

    def on_bearing(self, msg: dict):
        now = time.monotonic()

        if not msg.get("face_found"):
            if self._correcting_since is not None:
                self._send_stop("geen gezicht meer, stoppen")
            self._correcting_since = None
            self._watchdog_tripped = False
            return

        bearing = msg["bearing_deg"]

        if abs(bearing) <= BEARING_DEADBAND_DEG:
            if self._correcting_since is not None:
                self._send_stop(f"gecentreerd (bearing={bearing:+.1f} graden binnen dead-band)")
            self._correcting_since = None
            self._watchdog_tripped = False
            return

        if self._watchdog_tripped:
            return  # blijft genegeerd tot het gezicht weg is/gecentreerd raakt (ziet bovenstaande takken)

        if self._correcting_since is None:
            self._correcting_since = now
        elif now - self._correcting_since > MAX_CONTINUOUS_CORRECTION_S:
            self._send_stop(
                f"WATCHDOG: langer dan {MAX_CONTINUOUS_CORRECTION_S:.0f}s continu aan het corrigeren "
                f"zonder centrering (laatste bearing={bearing:+.1f} graden) -- stoppen, mogelijk detectiefout")
            self._watchdog_tripped = True
            return

        if now - self._last_correction_ts < CORRECTION_INTERVAL_S:
            return  # rate-limit, nog niet aan de beurt voor een nieuwe correctie

        vyaw = max(-MAX_VYAW, min(MAX_VYAW, -GAIN * bearing))
        self._last_correction_ts = now
        self._send_move(vyaw)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--live", action="store_true",
                    help="stuur ECHTE robot.move-commando's (default: dry-run, alleen loggen)")
    args = p.parse_args()

    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    rclpy.init()
    node = Node("face_follow_controller")
    controller = FaceFollowController(live=args.live)

    mode = "LIVE (stuurt echte bewegingscommando's)" if args.live else "DRY-RUN (logt alleen, stuurt niets)"
    node.get_logger().info(f"face_follow_controller gestart -- modus: {mode}")

    def cb(msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        controller.on_bearing(data)

    node.create_subscription(String, "face_bearing", cb, 10)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        controller._send_stop("node afgesloten")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
