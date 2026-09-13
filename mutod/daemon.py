"""mutod hoofdproces: enige eigenaar van /dev/myserial.

Start:
    python3 -m mutod.daemon [--port /dev/myserial] [--listen-host 127.0.0.1] [--listen-port 8420]

Faalt hard (met duidelijke foutmelding) als een ander proces de seriele
poort al open heeft -- dat vangt een app_muto.py/robot_bridge.py-conflict
meteen op (Fase 1 briefing §2, architectuurbeslissing 1).

Commandokanaal: newline-delimited JSON over TCP op 127.0.0.1 (niet een Unix-
socket -- de humble_run ROS2-container draait met NetworkMode=host maar heeft
geen /tmp van de host gemount, dus een Unix-socket-pad zou vanuit de
container onbereikbaar zijn; TCP op localhost werkt in beide namespaces
identiek). Voorbeeld: {"op": "gait", "direction": "FORWARD", "step": 10}
Sinds Fase 3 (12 sep 2026) accepteert hetzelfde kanaal ook echte JSON-RPC 2.0-
berichten in duck-ipc-proto-spelling (robot.move/robot.state/robot.health/
robot.do/robot.stop/robot.subscribe, zie ipc_types.py) -- herkend aan het
"jsonrpc": "2.0"-veld, naast (niet i.p.v.) Fase 2's kortere {"op": ...}-vorm
die mutod_client.py/phoenix_driver.py's raw_frames-relay blijft gebruiken.
"""
import argparse
import json
import logging
import os
import socketserver
import subprocess
import threading
import time

from . import ipc_types, protocol
from .behavior import BehaviorStateMachine
from .hal import MutoHAL, MutoHALError
from .safety import Deadman

log = logging.getLogger("mutod")

CONTROL_HZ = 25
TICK_S = 1.0 / CONTROL_HZ
DEADMAN_TIMEOUT_S = 0.6

# Achtergrond-telemetrie (read_all_angles) valt terug van 25Hz naar 2Hz zodra
# er recent bewegingscommando's binnenkomen (gait/write_servo/write_leg/
# raw_frames) -- phoenix_driver.py's eigen tests toonden dat zelfs 10Hz
# interleaved reads merkbare hapering gaven in de servo-aansturing op
# dezelfde seriele lijn; 2Hz is hun eigen, al gevalideerde veilige waarde.
READ_INTERVAL_IDLE_S = TICK_S
READ_INTERVAL_ACTIVE_S = 0.5
MOVEMENT_ACTIVE_WINDOW_S = 0.3

# -- Fase 3: robot.move (vx,vy,vyaw) -> STM32-firmware-gait (0x12-0x17) --
# Dit is bewust een ANDERE, eenvoudigere aanstuurweg dan phoenix_driver.py's
# cmd_vel-pad (dat gaat via raw_frames/tripod-gait, niet via mutod's eigen
# `gait`-op) -- zie roadmap Fase 3: "vertaalt naar gait-commando's, niet naar
# losse joint-writes". Bedoeld voor de latere gedragslaag (Fase 5)/companion-
# besturing, niet voor Nav2. De onderstaande schaling naar step (10-25) is
# NIET snelheidsgekalibreerd zoals phoenix_driver.py's MAX_ANGULAR_SPEED_RADPS
# dat wel is -- firmware-gait-snelheid meten is nog niet gedaan, dit is puur
# functionele plumbing voor Fase 3's berichtenschema.
MOVE_DEADBAND_MPS = 0.02
MOVE_DEADBAND_RADPS = 0.02
MOVE_REFERENCE_MAX_MPS = 0.06  # orde-grootte van phoenix_driver's MAX_LINEAR_SPEED_MPS
MOVE_REFERENCE_MAX_RADPS = 0.13  # orde-grootte van phoenix_driver's herkalibreerde MAX_ANGULAR_SPEED_RADPS
HEALTH_STALE_S = 2.0  # angles ouder dan dit -> niet meer "healthy"


def _scale_to_step(magnitude: float, reference_max: float) -> int:
    frac = min(1.0, abs(magnitude) / reference_max) if reference_max > 0 else 0.0
    return int(round(10 + frac * 15))  # hal.gait() klemt toch nogmaals op [10, 25]


def move_to_gait(vx: float, vy: float, vyaw: float):
    """Kiest de dominante as (grootste genormaliseerde component) en vertaalt
    die naar een (direction, step) voor hal.gait(). Geeft None terug als alles
    binnen de deadband valt (geen bewegingsintent)."""
    norm_x = vx / MOVE_REFERENCE_MAX_MPS if MOVE_REFERENCE_MAX_MPS else 0.0
    norm_y = vy / MOVE_REFERENCE_MAX_MPS if MOVE_REFERENCE_MAX_MPS else 0.0
    norm_yaw = vyaw / MOVE_REFERENCE_MAX_RADPS if MOVE_REFERENCE_MAX_RADPS else 0.0

    if abs(vx) < MOVE_DEADBAND_MPS:
        norm_x = 0.0
    if abs(vy) < MOVE_DEADBAND_MPS:
        norm_y = 0.0
    if abs(vyaw) < MOVE_DEADBAND_RADPS:
        norm_yaw = 0.0

    candidates = {
        "x": (abs(norm_x), norm_x, vx),
        "y": (abs(norm_y), norm_y, vy),
        "yaw": (abs(norm_yaw), norm_yaw, vyaw),
    }
    axis, (mag, sign_norm, raw) = max(candidates.items(), key=lambda kv: kv[1][0])
    if mag == 0.0:
        return None

    if axis == "x":
        direction = "FORWARD" if raw > 0 else "BACKWARD"
        step = _scale_to_step(raw, MOVE_REFERENCE_MAX_MPS)
    elif axis == "y":
        # Teken-conventie voor SHIFT_LEFT/RIGHT is nog niet fysiek geverifieerd
        # (in tegenstelling tot voor/achter en linksom/rechtsom, zie
        # muto_front_convention.py) -- eerste keer echt gebruiken vereist een
        # aparte fysieke check, niet aannemen dat dit al klopt.
        direction = "SHIFT_LEFT" if raw > 0 else "SHIFT_RIGHT"
        step = _scale_to_step(raw, MOVE_REFERENCE_MAX_MPS)
    else:
        # Zelfde teken-conventie als localization_spin2.py (angular.z>0 ==
        # linksom, empirisch bevestigd via de 12-sep-kalibratie).
        direction = "TURN_LEFT" if raw > 0 else "TURN_RIGHT"
        step = _scale_to_step(raw, MOVE_REFERENCE_MAX_RADPS)
    return direction, step


def _other_holders(device_path: str) -> set:
    """Best-effort: PIDs die het device al open hebben (via fuser)."""
    try:
        out = subprocess.run(
            ["fuser", device_path], capture_output=True, text=True, timeout=2
        )
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return set()
    pids = set()
    for tok in out.stdout.split():
        try:
            pids.add(int(tok))
        except ValueError:
            pass
    pids.discard(os.getpid())
    return pids


class ThreadingTCPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    daemon_threads = True
    allow_reuse_address = True


class SharedState:
    def __init__(self):
        self._lock = threading.Lock()
        self.angles = {}
        self.angles_ts = 0.0
        self.last_error = None
        self.last_move = {"vx": 0.0, "vy": 0.0, "vyaw": 0.0}
        self.last_move_ts = 0.0

    def set_angles(self, angles):
        with self._lock:
            self.angles = angles
            self.angles_ts = time.monotonic()
            self.last_error = None

    def set_error(self, err):
        with self._lock:
            self.last_error = str(err)

    def snapshot(self):
        with self._lock:
            return dict(self.angles), self.angles_ts, self.last_error

    def set_last_move(self, vx, vy, vyaw):
        with self._lock:
            self.last_move = {"vx": vx, "vy": vy, "vyaw": vyaw}
            self.last_move_ts = time.monotonic()

    def get_last_move(self):
        with self._lock:
            return dict(self.last_move), self.last_move_ts


class Daemon:
    def __init__(self, port: str, listen_host: str, listen_port: int):
        self.port = port
        self.listen_host = listen_host
        self.listen_port = listen_port
        self.state = SharedState()
        self.deadman = Deadman(DEADMAN_TIMEOUT_S)
        self.cmd_lock = threading.Lock()
        self._stop = threading.Event()
        self.hal = None
        self.movement_active_until = 0.0
        # Fase 5: alleen voor mode-RAPPORTAGE via robot.state -- ticken hier
        # roept nooit hal.gait() of iets bewegends aan, zie behavior.py's
        # module-docstring. peers_present staat hardcoded op False tot Fase 4's
        # DiscoveryService bewust aan de daemon gekoppeld wordt (nog niet
        # gedaan -- geen Fase-5-deliverable, zie muto_fase4_discovery-memory).
        self.behavior = BehaviorStateMachine()

    def _open_hal(self):
        holders = _other_holders(self.port)
        if holders:
            raise SystemExit(
                f"mutod: {self.port} is al open door proces(sen) {sorted(holders)} "
                f"(vermoedelijk app_muto.py of robot_bridge.py) -- sluit dat proces "
                f"eerst af. mutod weigert te starten met een gedeelde seriele poort."
            )
        self.hal = MutoHAL(self.port)
        self.hal.drain_stale()

    def _mark_movement_active(self):
        self.movement_active_until = time.monotonic() + MOVEMENT_ACTIVE_WINDOW_S

    # -- commando-verwerking ------------------------------------------------

    def _handle_command(self, cmd: dict) -> dict:
        op = cmd.get("op")
        try:
            if op == "get_state":
                angles, ts, err = self.state.snapshot()
                age_s = (time.monotonic() - ts) if ts else None
                return {"ok": True, "angles": angles, "age_s": age_s, "last_error": err}
            elif op == "gait":
                self.deadman.touch()
                self._mark_movement_active()
                with self.cmd_lock:
                    self.hal.gait(cmd["direction"], cmd.get("step", 10))
                return {"ok": True}
            elif op == "write_servo":
                self.deadman.touch()
                self._mark_movement_active()
                with self.cmd_lock:
                    self.hal.write_servo(cmd["servo_id"], cmd["angle_deg"], cmd.get("runtime_ms", 100))
                return {"ok": True}
            elif op == "write_leg":
                self.deadman.touch()
                self._mark_movement_active()
                with self.cmd_lock:
                    self.hal.write_leg(cmd["leg"], cmd["angles"], cmd.get("runtime_ms", 100))
                return {"ok": True}
            elif op == "raw_frames":
                frames = []
                for hexstr in cmd["frames_hex"]:
                    frame = bytes.fromhex(hexstr)
                    parsed = protocol.parse_write_frame(frame)
                    if parsed is None:
                        return {"ok": False, "error": f"ongeldig frame: {hexstr}"}
                    addr, _ = parsed
                    if addr not in protocol.RAW_FRAME_ALLOWED_ADDRESSES:
                        return {"ok": False, "error": f"adres 0x{addr:02x} niet toegestaan via raw_frames"}
                    frames.append(frame)
                self.deadman.touch()
                self._mark_movement_active()
                with self.cmd_lock:
                    self.hal.write_raw_frames(frames)
                return {"ok": True, "count": len(frames)}
            elif op == "read_attitude":
                with self.cmd_lock:
                    attitude = self.hal.read_attitude()
                return {"ok": True, **attitude}
            elif op == "torque":
                with self.cmd_lock:
                    self.hal.torque(bool(cmd.get("on", True)), cmd.get("servo_id", 0))
                return {"ok": True}
            elif op == "reset_posture":
                self.deadman.touch()
                with self.cmd_lock:
                    self.hal.reset_posture()
                return {"ok": True}
            elif op == "buzzer":
                with self.cmd_lock:
                    self.hal.buzzer(cmd["timeout"])
                return {"ok": True, "timeout": cmd["timeout"]}
            elif op == "action":
                self.deadman.touch()
                self._mark_movement_active()
                with self.cmd_lock:
                    self.hal.action(cmd["action_id"])
                return {"ok": True, "action_id": cmd["action_id"], "name": protocol.ACTION_NAMES.get(cmd["action_id"])}
            elif op == "stop":
                with self.cmd_lock:
                    self.hal.stay_put()
                self.deadman.touch()
                return {"ok": True}
            else:
                return {"ok": False, "error": f"onbekende op: {op!r}"}
        except (KeyError, ValueError, MutoHALError) as exc:
            return {"ok": False, "error": str(exc)}

    # -- Fase 3: duck-ipc-proto JSON-RPC --------------------------------

    def _build_robot_state(self) -> dict:
        angles, ts, err = self.state.snapshot()
        move, move_ts = self.state.get_last_move()
        age_s = (time.monotonic() - ts) if ts else None
        battery = self._read_battery_cache()
        mode = self.behavior.tick(
            battery_pct=battery["percent"] if battery else None,
            peers_present=False,  # zie __init__: Fase 4-koppeling nog niet gedaan
        )
        return {
            "t": time.time(),
            "mode": mode,
            "servos": {str(sid): deg for sid, deg in angles.items()},
            "move": {"requested": [move["vx"], move["vy"], move["vyaw"]]},
            "safety": {"deadman_active": self.deadman.expired()},
            "loop": {"hz": CONTROL_HZ, "age_s": age_s, "missed": 1 if err else 0},
        }

    def _build_robot_health(self) -> dict:
        angles, ts, err = self.state.snapshot()
        age_s = (time.monotonic() - ts) if ts else None
        healthy = err is None and age_s is not None and age_s < HEALTH_STALE_S
        result = {"healthy": healthy}
        if not healthy:
            result["degraded"] = True
            result["reason"] = err or (
                f"laatste read is {age_s:.1f}s oud" if age_s is not None else "nog geen read gedaan"
            )
        battery = self._read_battery_cache()
        if battery is not None:
            result["battery"] = battery
        return result

    @staticmethod
    def _read_battery_cache():
        """Best-effort: /tmp/battery_pct en /tmp/battery_volt zijn app_muto.py's
        cache-bestanden (zie Fase 2b, yahboom_oled.py-bevinding) -- mutod
        schrijft ze zelf niet, dus dit is alleen zinvol zolang app_muto.py
        recent gedraaid heeft. Geen fout als ze ontbreken of verouderd zijn."""
        try:
            pct_mtime = os.path.getmtime("/tmp/battery_pct")
            if time.time() - pct_mtime > 30.0:
                return None
            with open("/tmp/battery_pct") as f:
                pct = float(f.read().strip())
            with open("/tmp/battery_volt") as f:
                volt = float(f.read().strip())
            return {"percent": pct, "volts": volt}
        except (OSError, ValueError):
            return None

    def _handle_rpc(self, req: dict) -> dict:
        req_id = req.get("id")
        method = req.get("method")
        params = req.get("params") or {}

        if method not in ipc_types.SUPPORTED_METHODS:
            return ipc_types.make_error(
                req_id, ipc_types.METHOD_NOT_FOUND, f"onbekende methode: {method!r}"
            )

        try:
            if method == ipc_types.ROBOT_HEALTH:
                return ipc_types.make_result(req_id, self._build_robot_health())

            elif method == ipc_types.ROBOT_STATE:
                return ipc_types.make_result(req_id, self._build_robot_state())

            elif method == ipc_types.ROBOT_STOP:
                with self.cmd_lock:
                    self.hal.stay_put()
                self.deadman.touch()
                self.state.set_last_move(0.0, 0.0, 0.0)
                return ipc_types.make_result(req_id, {"accepted": True})

            elif method == ipc_types.ROBOT_MOVE:
                try:
                    vx = float(params.get("vx", 0.0))
                    vy = float(params.get("vy", 0.0))
                    vyaw = float(params.get("vyaw", 0.0))
                except (TypeError, ValueError):
                    return ipc_types.make_error(req_id, ipc_types.INVALID_PARAMS, "vx/vy/vyaw moeten getallen zijn")
                for v in (vx, vy, vyaw):
                    if v != v or v in (float("inf"), float("-inf")):  # NaN/inf
                        return ipc_types.make_error(req_id, ipc_types.INVALID_PARAMS, "NaN/inf niet toegestaan")

                self.deadman.touch()
                self._mark_movement_active()
                self.state.set_last_move(vx, vy, vyaw)
                choice = move_to_gait(vx, vy, vyaw)
                if choice is None:
                    return ipc_types.make_result(req_id, {"accepted": True, "direction": None, "note": "binnen deadband, geen gait verstuurd"})
                direction, step = choice
                with self.cmd_lock:
                    self.hal.gait(direction, step)
                return ipc_types.make_result(req_id, {"accepted": True, "direction": direction, "step": step})

            elif method == ipc_types.ROBOT_DO:
                skill = params.get("skill")
                if skill not in ipc_types.KNOWN_SKILLS:
                    return ipc_types.make_error(
                        req_id, ipc_types.INVALID_PARAMS,
                        f"skill {skill!r} nog niet geimplementeerd (Fase 6, zie ACTION/0x3E-validatie)",
                    )
                return ipc_types.make_error(req_id, ipc_types.INTERNAL_ERROR, "onbereikbaar: lege KNOWN_SKILLS")

            elif method == ipc_types.ROBOT_SUBSCRIBE:
                # LET OP: start hier bewust GEEN achtergrondthread -- de
                # aanroeper (Handler.handle()) start die pas na het schrijven
                # van dit antwoord, anders kan de eerste robot.state-
                # notificatie de subscribe-bevestiging zelf inhalen op de
                # socket (race gevonden door mutod_ipc_live_check.py, 12 sep 2026).
                hz = ipc_types.clamp_hz(params.get("hz", ipc_types.SUBSCRIBE_DEFAULT_HZ))
                return ipc_types.make_result(req_id, {"accepted": True, "hz": hz})

        except (KeyError, ValueError, MutoHALError) as exc:
            return ipc_types.make_error(req_id, ipc_types.INTERNAL_ERROR, str(exc))

    # -- control loop -----------------------------------------------------

    def _control_loop(self):
        last_read_ts = 0.0
        while not self._stop.is_set():
            tick_start = time.monotonic()

            active = tick_start < self.movement_active_until
            read_interval = READ_INTERVAL_ACTIVE_S if active else READ_INTERVAL_IDLE_S
            if tick_start - last_read_ts >= read_interval:
                last_read_ts = tick_start
                try:
                    with self.cmd_lock:
                        angles = self.hal.read_all_angles()
                    self.state.set_angles(angles)
                except MutoHALError as exc:
                    self.state.set_error(exc)
                    log.warning("read_all_angles mislukt: %s", exc)

            if self.deadman.trip_once():
                log.warning("deadman: geen intent binnen %.1fs, stay_put()", DEADMAN_TIMEOUT_S)
                try:
                    with self.cmd_lock:
                        self.hal.stay_put()
                except MutoHALError as exc:
                    log.error("deadman stay_put() mislukt: %s", exc)

            elapsed = time.monotonic() - tick_start
            time.sleep(max(0.0, TICK_S - elapsed))

    # -- socket-server ------------------------------------------------------

    def _make_handler(self):
        daemon = self

        class Handler(socketserver.StreamRequestHandler):
            def setup(self):
                super().setup()
                self._write_lock = threading.Lock()
                self._sub_stop = threading.Event()
                self._sub_thread = None

            def _write_line(self, obj):
                with self._write_lock:
                    self.wfile.write((json.dumps(obj) + "\n").encode())

            def _start_subscription(self, hz):
                if self._sub_thread is not None:
                    return  # al een subscriptie op deze verbinding, geen tweede thread

                def push_loop():
                    interval = 1.0 / hz
                    while not self._sub_stop.is_set():
                        try:
                            state = daemon._build_robot_state()
                            self._write_line(ipc_types.make_notification(ipc_types.ROBOT_STATE, state))
                        except (OSError, ValueError):
                            break  # verbinding weg -- handle()'s for-loop stopt vanzelf ook
                        self._sub_stop.wait(interval)

                self._sub_thread = threading.Thread(target=push_loop, daemon=True)
                self._sub_thread.start()

            def handle(self):
                try:
                    for line in self.rfile:
                        line = line.strip()
                        if not line:
                            continue
                        try:
                            cmd = json.loads(line)
                        except json.JSONDecodeError as exc:
                            self._write_line({"ok": False, "error": f"ongeldige JSON: {exc}"})
                            continue
                        if ipc_types.is_jsonrpc_request(cmd):
                            resp = daemon._handle_rpc(cmd)
                            self._write_line(resp)
                            if cmd.get("method") == ipc_types.ROBOT_SUBSCRIBE and "result" in resp:
                                self._start_subscription(resp["result"]["hz"])
                        else:
                            resp = daemon._handle_command(cmd)
                            self._write_line(resp)
                finally:
                    self._sub_stop.set()
                    if self._sub_thread is not None:
                        self._sub_thread.join(timeout=1.0)

        return Handler

    def run(self):
        self._open_hal()
        log.info("mutod: %s exclusief geopend", self.port)

        server = ThreadingTCPServer((self.listen_host, self.listen_port), self._make_handler())
        server_thread = threading.Thread(target=server.serve_forever, daemon=True)
        server_thread.start()
        log.info("mutod: luistert op %s:%d", self.listen_host, self.listen_port)

        control_thread = threading.Thread(target=self._control_loop, daemon=True)
        control_thread.start()

        try:
            while control_thread.is_alive():
                control_thread.join(timeout=1.0)
        except KeyboardInterrupt:
            log.info("mutod: afsluiten...")
        finally:
            self._stop.set()
            server.shutdown()
            self.hal.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/myserial")
    parser.add_argument("--listen-host", default="127.0.0.1")
    parser.add_argument("--listen-port", type=int, default=8420)
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    Daemon(args.port, args.listen_host, args.listen_port).run()


if __name__ == "__main__":
    main()
