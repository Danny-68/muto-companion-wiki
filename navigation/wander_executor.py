#!/usr/bin/env python3
"""wander_executor.py -- Fase 5's novelty-grid-wandelroutine, DAADWERKELIJK
uitgevoerd. Dit is de eerste autonome, doorlopende, zelfgekozen beweging in
dit project (nieuwe risicocategorie t.o.v. losse commando's) -- vereist een
aparte, expliciete aankondiging voor de eerste live run, zie
muto_announce_before_movement-memory.

VEILIGHEIDSONTWERP:
- --dry-run (DEFAULT) logt alleen de gekozen actie, stuurt niets. Pas --live
  stuurt echte robot.move-commando's naar mutod.
- Draait ALLEEN als mutod's eigen behavior-state-machine mode=="wander"
  rapporteert (robot.state, Fase 5) -- respecteert dus vanzelf Rust/
  Interactie/Laag-batterij-prioriteiten zonder die hier te dupliceren.
- Combineert LiDAR (360, lidar_obstacle.py) + dieptecamera (voorwaarts,
  depth_obstacle.py, optioneel als de camera niet draait) via
  merge_lidar_and_depth_clearance() -- conservatief, minimum wint.
- choose_heading() (behavior.py) kiest de richting; als niets vrij genoeg is
  (None), wordt er NIET geraden -- expliciet stilstaan.
- Extra front-veiligheidscheck vlak voor elk FORWARD-commando (naast
  choose_heading()'s eigen filter) -- defense in depth.
- Snelheden/stappen zijn klein en van dezelfde ordegrootte als de al fysiek
  bevestigde Fase 3-tests (TURN/FORWARD), niet nieuw verzonnen.
- Cyclus-rate-limit (WANDER_CYCLE_S) -- reageert niet op elke sensor-tick.
- Watchdog: sessie stopt zichzelf hard na MAX_SESSION_S, ongeacht mode --
  vereist een herstart van dit script om door te gaan (geen onbeheerd
  oneindig rondlopen bij een eerste test).
- Stuurt altijd een expliciete robot.stop() bij mode-wissel weg van wander,
  bij "geen veilige richting", en bij afsluiten.
"""
import argparse
import json
import math
import socket
import sys
import time

sys.path.insert(0, "/root")
from behavior import NoveltyGrid, choose_heading
from lidar_obstacle import estimate_clearance_by_heading as lidar_clearance_by_heading
from depth_obstacle import estimate_clearance_by_heading as depth_clearance_by_heading
from depth_obstacle import merge_lidar_and_depth_clearance

MUTOD_HOST, MUTOD_PORT = "127.0.0.1", 8420

WANDER_CYCLE_S = 2.0
CANDIDATE_HEADINGS_DEG = [-90, -60, -30, 0, 30, 60, 90]
STEP_M = 0.3
MIN_CLEAR_M = 0.45
FORWARD_ALIGN_DEG = 25.0
FORWARD_SAFETY_MIN_M = 0.45   # extra check vlak voor het FORWARD-commando zelf
TURN_VYAW = 0.09              # zelfde ordegrootte als de bevestigde Fase 3 TURN-test
FORWARD_VX = 0.03             # bewust klein, eerste live wander-test
MAX_SESSION_S = 60.0          # harde stop, ongeacht mode -- eerste test, conservatief


def rpc_call(sock, rfile, method, params=None, req_id=1):
    req = {"jsonrpc": "2.0", "id": req_id, "method": method}
    if params is not None:
        req["params"] = params
    sock.sendall((json.dumps(req) + "\n").encode())
    line = rfile.readline()
    if not line:
        raise ConnectionError("mutod sloot de verbinding")
    return json.loads(line)


class MutodLink:
    def __init__(self):
        self._sock = None
        self._rfile = None

    def _ensure(self):
        if self._sock is None:
            self._sock = socket.create_connection((MUTOD_HOST, MUTOD_PORT), timeout=5)
            self._rfile = self._sock.makefile("rb")

    def get_mode(self) -> str:
        self._ensure()
        resp = rpc_call(self._sock, self._rfile, "robot.state")
        return resp.get("result", {}).get("mode", "onbekend")

    def move(self, vx=0.0, vy=0.0, vyaw=0.0):
        self._ensure()
        return rpc_call(self._sock, self._rfile, "robot.move", {"vx": vx, "vy": vy, "vyaw": vyaw})

    def stop(self):
        self._ensure()
        return rpc_call(self._sock, self._rfile, "robot.stop")


def quat_to_yaw_deg(z, w):
    return math.degrees(2.0 * math.atan2(z, w))


class SensorCache:
    """Simpele houder voor de laatste ROS-berichten -- geen rclpy-logica hier,
    puur data, zodat de besluitvormingslus in main() overzichtelijk blijft."""

    def __init__(self):
        self.scan = None  # (ranges, angle_min, angle_increment, range_min, range_max)
        self.odom = None  # (x, y, yaw_deg)
        self.depth_frame = None
        self.depth_fx = None
        self.depth_cx = None


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--live", action="store_true",
                    help="stuur ECHTE robot.move-commando's (default: dry-run, alleen loggen)")
    args = p.parse_args()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan, Image, CameraInfo
    from nav_msgs.msg import Odometry
    from cv_bridge import CvBridge

    rclpy.init()
    node = Node("wander_executor")
    bridge = CvBridge()
    cache = SensorCache()
    mutod = MutodLink()

    mode_label = "LIVE (stuurt echte bewegingscommando's)" if args.live else "DRY-RUN (logt alleen, stuurt niets)"
    node.get_logger().info(f"wander_executor gestart -- modus: {mode_label}")

    def scan_cb(msg: LaserScan):
        cache.scan = (msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max)

    def odom_cb(msg: Odometry):
        q = msg.pose.pose.orientation
        cache.odom = (msg.pose.pose.position.x, msg.pose.pose.position.y, quat_to_yaw_deg(q.z, q.w))

    def depth_info_cb(msg: CameraInfo):
        if cache.depth_fx is None and msg.k[0] > 0:
            cache.depth_fx, cache.depth_cx = msg.k[0], msg.k[2]

    def depth_image_cb(msg: Image):
        cache.depth_frame = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    node.create_subscription(LaserScan, "/scan_fixed", scan_cb, qos_profile_sensor_data)
    node.create_subscription(Odometry, "/odom_fused", odom_cb, 10)
    node.create_subscription(CameraInfo, "/camera/depth/camera_info", depth_info_cb, 10)
    node.create_subscription(Image, "/camera/depth/image_raw", depth_image_cb, qos_profile_sensor_data)

    grid = NoveltyGrid(cell_m=0.20)
    session_start = None
    watchdog_tripped = False
    was_wandering = False
    target_world_heading_deg = None  # vastgehouden doel, i.p.v. elke cyclus opnieuw kiezen (zie fix 12 sep 2026)

    def normalize_deg(d):
        return ((d + 180.0) % 360.0) - 180.0

    def send_move(vx=0.0, vy=0.0, vyaw=0.0, why=""):
        if not args.live:
            node.get_logger().info(f"[DRY-RUN] zou sturen: robot.move vx={vx:+.3f} vyaw={vyaw:+.3f} ({why})")
            return
        resp = mutod.move(vx=vx, vy=vy, vyaw=vyaw)
        node.get_logger().info(f"[LIVE] robot.move vx={vx:+.3f} vyaw={vyaw:+.3f} ({why}) -> {resp.get('result') or resp.get('error')}")

    def send_stop(why=""):
        node.get_logger().info(f"[STOP] {why}")
        if not args.live:
            node.get_logger().info("[DRY-RUN] zou sturen: robot.stop")
            return
        resp = mutod.stop()
        node.get_logger().info(f"[LIVE] robot.stop -> {resp.get('result') or resp.get('error')}")

    def cycle():
        nonlocal session_start, watchdog_tripped, was_wandering, target_world_heading_deg

        try:
            mode = mutod.get_mode()
        except (ConnectionError, OSError) as exc:
            node.get_logger().warn(f"kan mutod niet bereiken: {exc}")
            return

        if mode != "wander":
            if was_wandering:
                send_stop(f"mode is nu {mode!r}, niet meer wander")
            was_wandering = False
            session_start = None
            watchdog_tripped = False
            target_world_heading_deg = None
            return

        if watchdog_tripped:
            return  # blijft genegeerd tot het script herstart wordt (bewuste, harde grens)

        if session_start is None:
            session_start = time.monotonic()
            node.get_logger().info("wander-sessie gestart")
        elif time.monotonic() - session_start > MAX_SESSION_S:
            send_stop(f"WATCHDOG: {MAX_SESSION_S:.0f}s wander-sessie voorbij, hard stoppen (herstart dit script om door te gaan)")
            watchdog_tripped = True
            return

        was_wandering = True

        if cache.scan is None or cache.odom is None:
            node.get_logger().warn("nog geen scan/odom ontvangen, wacht...")
            return

        ranges, angle_min, angle_increment, range_min, range_max = cache.scan
        lidar_clear = lidar_clearance_by_heading(ranges, angle_min, angle_increment, range_min, range_max,
                                                   heading_step_deg=15.0, useful_range_m=4.0)

        depth_clear = {}
        if cache.depth_frame is not None and cache.depth_fx is not None:
            depth_clear = depth_clearance_by_heading(cache.depth_frame, cache.depth_fx, cache.depth_cx,
                                                        heading_step_deg=15.0)
        merged = merge_lidar_and_depth_clearance(lidar_clear, depth_clear)

        # Top-level veiligheidsgate (12 sep 2026-fix, zie verderop bij de
        # FORWARD-tak voor de volledige uitleg): iets ergens heel dichtbij
        # stopt alles, ongeacht welke actie overwogen werd.
        global_min_clear = min(merged.values()) if merged else None
        if global_min_clear is not None and global_min_clear < FORWARD_SAFETY_MIN_M * 0.7:
            send_stop(f"KRITIEK dichtbij iets rondom (min={global_min_clear:.2f}m) -- alles stoppen, nieuw doel volgende cyclus")
            target_world_heading_deg = None
            return

        odom_x, odom_y, odom_yaw_deg = cache.odom
        grid.visit(odom_x, odom_y)

        # Alleen een NIEUW doel kiezen als er nog geen is, of als het huidige
        # doel niet meer veilig blijkt -- voorkomt het heen-en-weer-wisselen
        # tussen bijna-gelijke kandidaten door sensorruis (gevonden 12 sep 2026,
        # live wander-test: robot bleef besluiteloos heen-en-weer draaien).
        need_new_target = target_world_heading_deg is None
        if not need_new_target:
            offset_to_target = normalize_deg(target_world_heading_deg - odom_yaw_deg)
            nearest_bucket = min(merged.keys(), key=lambda h: abs(h - offset_to_target)) if merged else None
            current_target_clear = merged.get(nearest_bucket) if nearest_bucket is not None else None
            if current_target_clear is None or current_target_clear < MIN_CLEAR_M:
                need_new_target = True
                node.get_logger().info(f"vastgehouden doel niet meer vrij ({current_target_clear}), nieuw doel kiezen")

        if need_new_target:
            chosen = choose_heading(CANDIDATE_HEADINGS_DEG, merged, odom_x, odom_y, odom_yaw_deg,
                                      grid, step_m=STEP_M, min_clear_m=MIN_CLEAR_M)
            if chosen is None:
                send_stop("geen richting voldoende vrij (min_clear_m=%.2f) -- stilstaan" % MIN_CLEAR_M)
                target_world_heading_deg = None
                return
            target_world_heading_deg = normalize_deg(odom_yaw_deg + chosen)
            node.get_logger().info(f"nieuw doel gekozen: {chosen:+.0f} graden relatief (wereld {target_world_heading_deg:+.0f})")

        offset_to_target = normalize_deg(target_world_heading_deg - odom_yaw_deg)

        if abs(offset_to_target) <= FORWARD_ALIGN_DEG:
            # VEILIGHEIDSFIX 12 sep 2026: een "front"-richting-opzoeking bleek
            # fout gecalibreerd (raw scan-hoek 0 komt NIET betrouwbaar overeen
            # met de werkelijke looprichting -- ontdekt doordat de robot een
            # kast recht vooruit niet detecteerde en een mens moest ingrijpen,
            # zie muto_wander_frontcheck_bug_2026-09-12-memory). Tot een echte
            # hoek-kalibratie gedaan is: gebruik het GLOBALE minimum over ALLE
            # richtingen als veiligheidsgrens, niet een vermoede "voorkant"-
            # bucket -- conservatiever (kan ook stoppen voor iets opzij), maar
            # blijft veilig ongeacht of de richtingtoewijzing zelf klopt.
            global_min_clear = min(merged.values()) if merged else None
            if global_min_clear is None or global_min_clear < FORWARD_SAFETY_MIN_M:
                send_stop(f"veiligheidscheck faalt (laagste clearance rondom={global_min_clear}), niet vooruit")
                target_world_heading_deg = None  # opnieuw kiezen volgende cyclus
                return
            send_move(vx=FORWARD_VX, why=f"doel {offset_to_target:+.0f} graden ~voorwaarts, lopen")
        else:
            vyaw = math.copysign(TURN_VYAW, offset_to_target)
            send_move(vyaw=vyaw, why=f"draaien naar vastgehouden doel ({offset_to_target:+.0f} graden resterend)")

    node.create_timer(WANDER_CYCLE_S, cycle)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        send_stop("node afgesloten")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
