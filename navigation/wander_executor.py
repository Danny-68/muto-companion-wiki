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
- 13 sep 2026: bij elke veiligheidsstop wordt niet meer blind een nieuwe
  uitwijkrichting gekozen -- eerst een YOLO-check op het kleurbeeld (zelfde
  Jetson-server als yolo_snapshot_sender.py). Bij een herkende persoon: meldt
  interactie aan mutod (nieuwe robot.notify_interaction-IPC, sluit Fase 5's
  al bestaande notify_interaction()-hook eindelijk aan) i.p.v. te blijven
  uitwijken/draaien, bewaart een foto, en laat twee korte herkennings-
  piepjes horen via de STM32-buzzer (onderscheidbaar van yolo_snapshot_
  sender.py's ENE "nieuw object"-piepje). Bij iets anders: bestaande
  uitwijklogica ongewijzigd.
- 13 sep 2026 (vervolg): de vooruit-veiligheidscheck gebruikt nu een smalle
  corridor recht vooruit (FORWARD_CORRIDOR_HALF_DEG) i.p.v. het globale
  minimum over ALLE richtingen -- anders zou de robot nooit door een
  deuropening durven lopen (posten aan weerszijden dichtbij, pad zelf vrij).
  Het globale minimum blijft wel een apart, laatste vangnet specifiek tegen
  vooruitlopen (niet tegen draaien -- draaien brengt de robot niet dichter
  bij iets, dus hoeft niet bevroren te worden zodra iets ergens dichtbij is).
- 13 sep 2026 (vervolg): white_v_detector.py's losse "volg de witte V"-gedrag
  (zelfde bearing-conventie als face_follow_controller.py) is hier
  samengevoegd -- dit script kijkt nu ELKE cyclus eerst of er een witte
  V-vorm zichtbaar is; zo ja, dan krijgt dat voorrang boven het gewone
  wander-gedrag (draait ernaartoe als niet gecentreerd, stapt vooruit als
  wel gecentreerd), ongeacht de huidige mutod-modus.
"""
import argparse
import io
import json
import math
import socket
import sys
import time

import cv2
import requests
from PIL import Image as PILImage

sys.path.insert(0, "/root")
from behavior import NoveltyGrid, choose_heading
from lidar_obstacle import estimate_clearance_by_heading as lidar_clearance_by_heading
from depth_obstacle import estimate_clearance_by_heading as depth_clearance_by_heading
from depth_obstacle import merge_lidar_and_depth_clearance

MUTOD_HOST, MUTOD_PORT = "127.0.0.1", 8420

# 13 sep 2026 (Fase 5-vervolg): bij een veiligheidsstop niet zomaar blind een
# nieuwe uitwijkrichting kiezen -- eerst met YOLO checken WAT er voor staat.
# Bij een persoon: meld interactie aan mutod's centrale gedragslaag (bestaande,
# tot nu toe ongebruikte notify_interaction()-hook, zie muto_fase5_behavior_
# 2026-09-12-memory) i.p.v. te blijven uitwijken/draaien. Zelfde Jetson-server
# als yolo_snapshot_sender.py, hier los aangeroepen op het moment van de stop
# i.p.v. op een vast interval.
YOLO_JETSON_URL = "http://192.168.68.86:8600/detect"
INTERACTION_SNAPSHOT_PATH = "/root/interaction_snapshot.jpg"

# Herkennings-piepjes bij een gezicht: TWEE korte piepjes (i.p.v. het ENE
# piepje dat yolo_snapshot_sender.py al gebruikt voor "nieuw object gezien")
# -- moet zelf onderscheidbaar zijn, niet dezelfde betekenis overloaden.
# Gaat via mutod's oudere {"op": "buzzer"}-kanaal (Fase 2, geen JSON-RPC-
# methode hiervoor) -- zelfde patroon als yolo_snapshot_sender.py's
# MutodBuzzer, hier lokaal opnieuw omdat dat een apart script/proces is.
RECOGNITION_BEEP_TIMEOUT = 3   # 300ms per piepje
RECOGNITION_BEEP_GAP_S = 0.5   # stilte tussen de twee piepjes


def send_buzzer(timeout: int):
    try:
        s = socket.create_connection((MUTOD_HOST, MUTOD_PORT), timeout=2)
        s.sendall((json.dumps({"op": "buzzer", "timeout": timeout}) + "\n").encode())
        s.recv(256)
        s.close()
    except OSError:
        pass  # een gemiste piep mag de rest van de afhandeling niet breken


def beep_recognition():
    send_buzzer(RECOGNITION_BEEP_TIMEOUT)
    time.sleep(RECOGNITION_BEEP_GAP_S)
    send_buzzer(RECOGNITION_BEEP_TIMEOUT)


def detect_pink_ball(frame_rgb) -> tuple:
    """13 sep 2026: vervangt de oude witte-V-detectie (verward met het witte
    deurkozijn, zie muto_wander_frontcheck... nee, zie de sessie van vandaag --
    een groot statisch wit vlak op afstand werd foutief als V herkend). Een
    roze bal is op kleur (i.p.v. helderheid) te vinden EN vrijwel perfect
    convex (solidity ~1.0), wat het kozijn (niet roze, niet rond) vanzelf
    uitsluit. Kleurdrempels gekalibreerd op een echte foto van de bal (zie
    /home/pi/current_camera_frame.jpg, 13 sep 2026), niet gegokt.
    Geeft (gevonden, solidity, area, center_x_px) terug -- zelfde vorm als de
    oude functie, solidity is hier een rondheids-/convexiteitscheck i.p.v.
    een V-inkeping-check. frame_rgb: numpy-array, rgb8."""
    hsv = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2HSV)
    # Hue-range dekt de gemeten cluster (172-178) plus het rood-wraparound-stuk
    # (0-8) voor het geval de kleur onder ander licht iets anders uitleest.
    mask_high = cv2.inRange(hsv, (PINK_HUE_LOW, PINK_SAT_MIN, PINK_VAL_MIN), (179, 255, 255))
    mask_low = cv2.inRange(hsv, (0, PINK_SAT_MIN, PINK_VAL_MIN), (PINK_HUE_WRAP_HIGH, 255, 255))
    mask = cv2.bitwise_or(mask_high, mask_low)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best = None
    for c in contours:
        area = cv2.contourArea(c)
        if area < PINK_BALL_MIN_AREA_PX:
            continue
        hull = cv2.convexHull(c)
        hull_area = cv2.contourArea(hull)
        if hull_area <= 0:
            continue
        solidity = area / hull_area
        if solidity >= PINK_BALL_MIN_SOLIDITY:  # rond/bol i.p.v. de V's inkeping
            if best is None or area > best[1]:
                x, y, w, h = cv2.boundingRect(c)
                best = (solidity, area, x + w / 2.0)

    if best is None:
        return False, None, None, None
    return True, best[0], best[1], best[2]


WANDER_CYCLE_S = 2.0
CANDIDATE_HEADINGS_DEG = [-90, -60, -30, 0, 30, 60, 90]
STEP_M = 0.3
MIN_CLEAR_M = 0.45
FORWARD_ALIGN_DEG = 25.0
FORWARD_SAFETY_MIN_M = 0.45   # extra check vlak voor het FORWARD-commando zelf
# TEKEN-FIX 13 sep 2026 (na de LiDAR-hoek-fix, zie muto_lidar_heading_fix_
# 2026-09-13-memory): de vooruit-check gebruikte tot nu toe het GLOBALE
# minimum over ALLE richtingen rondom (bewust zo gebouwd na het kast-
# incident, toen de hoek-toewijzing zelf nog onbetrouwbaar was). Nu die
# hoek-fix live bevestigd is (rechtdoor lopen is rotsvast), is dat globale
# vangnet te breed: iets opzij (een deurpost, een bank naast het pad) kon
# vooruitlopen blokkeren ook als het PAD zelf vrij was -- gevonden doordat
# de robot bleef vastlopen op hetzelfde, op zichzelf prima vrije doel
# (0 graden), geblokkeerd door een heel ANDERE richting z'n clearance.
# Zelfde reden waarom hij anders nooit door een deuropening zou durven
# lopen (posten aan weerszijden dichtbij, pad zelf vrij). Nieuwe aanpak:
# alleen de richtingen binnen een corridor rond de HUIDIGE (robot-eigen)
# voorkant tellen mee voor de vooruit-veiligheidscheck -- ruwweg de breedte
# van de robot geprojecteerd op de te lopen afstand. De bredere, 360-graden
# "KRITIEK dichtbij"-noodstop (hieronder, in cycle()) blijft ongewijzigd
# als laatste vangnet.
FORWARD_CORRIDOR_HALF_DEG = 20.0
TURN_VYAW = 0.09              # zelfde ordegrootte als de bevestigde Fase 3 TURN-test
FORWARD_VX = 0.03             # bewust klein, eerste live wander-test
MAX_SESSION_S = 60.0          # harde stop, ongeacht mode -- eerste test, conservatief
# 13 sep 2026 (gebruikersidee): een doel dat ongeveer OPZIJ ligt hoeft niet
# via draaien+lopen bereikt te worden -- SHIFT_LEFT/RIGHT (vy) is al fysiek
# bevestigd (Fase 3/6) en laat de robot recht zijwaarts stappen zonder te
# draaien, handig in een nauwe ruimte (bv. langs een bank). Vanaf dit aantal
# graden resterend wordt gestrafet i.p.v. gedraaid.
STRAFE_MIN_DEG = 70.0
STRAFE_VY = 0.03               # zelfde ordegrootte als FORWARD_VX/de bevestigde SHIFT-test
STRAFE_SAFETY_MIN_M = 0.45     # zelfde drempel als FORWARD_SAFETY_MIN_M, nu voor de zijwaartse richting

# 13 sep 2026 (gebruikersidee): in een smalle doorgang (bv. een deuropening)
# is de kans op een succesvolle doorgang het grootst als de robot ongeveer
# centraal loopt -- evenveel ruimte links als rechts. move_to_gait() in
# daemon.py kan maar 1 as tegelijk aansturen (geen diagonaal), dus dit wordt
# een losse correctiecyclus (SHIFT i.p.v. FORWARD) zodra de corridor smal is
# EN er een merkbare links/rechts-onbalans is, i.p.v. gecombineerd met vx.
CENTERING_NARROW_M = 1.2        # onder deze corridor_clear actief proberen te centreren
CENTERING_SIDE_DEG = 30.0       # bucket net buiten de vooruit-corridor, representatief voor ruimte opzij
CENTERING_MIN_IMBALANCE_M = 0.15  # kleiner verschil dan dit is ruis, dan niet corrigeren
CENTERING_VY = 0.02             # kleiner dan STRAFE_VY -- dit is fijnbijsturen, geen bewuste zijstap

# -- witte-V-detectie (samengevoegd vanuit white_v_detector.py, 13 sep 2026) --
# Gekalibreerd op een echte foto van de roze bal (13 sep 2026), niet gegokt --
# gemeten HSV-cluster op de bal was H=172-178, S=111-148, V=63-128.
PINK_HUE_LOW = 165             # ondergrens van het hoofdcluster (172-178)
PINK_HUE_WRAP_HIGH = 8         # rood-wraparound-stuk (hue 0-8), voor andere lichtval
PINK_SAT_MIN = 70              # ruim onder de gemeten p10 van 111, voor marge
PINK_VAL_MIN = 30              # ruim onder de gemeten p10 van 63, voor marge
PINK_BALL_MIN_AREA_PX = 150    # kleiner dan de bal maar groter dan de vloerreflectie-vlek
PINK_BALL_MIN_SOLIDITY = 0.85  # een bal is vrijwel perfect convex, i.t.t. een deurkozijnrand
V_BEARING_DEADBAND_DEG = 8.0   # zelfde als face_follow_controller.py
V_MAX_TURN_VYAW = 0.10         # zelfde ordegrootte als de bevestigde Fase 3 TURN-test
V_TURN_GAIN = 0.01             # vyaw = clamp(-V_TURN_GAIN*bearing, ...)
V_CONSECUTIVE_FRAMES_REQUIRED = 3  # cycle() draait al maar om de 2s, dus minder frames nodig dan white_v_detector.py's losse 0.3s-timer
V_COOLDOWN_S = 4.0             # geen nieuwe stap binnen dit venster na een trigger
V_FORWARD_VX = 0.03
V_LOST_GRACE_S = 2.0           # V-voorrang blijft dit lang staan na een gemist frame,
                                # zodat het stuurprogramma niet meteen overneemt bij flikkering


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

    def notify_interaction(self):
        self._ensure()
        return rpc_call(self._sock, self._rfile, "robot.notify_interaction")


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
        self.color_frame = None  # rgb8 numpy-array, voor de YOLO-check bij een veiligheidsstop
        self.color_fx = None    # voor de witte-V-bearing-berekening
        self.color_cx = None


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--live", action="store_true",
                    help="stuur ECHTE robot.move-commando's (default: dry-run, alleen loggen)")
    p.add_argument("--max-session-s", type=float, default=MAX_SESSION_S,
                    help=f"harde sessie-watchdog in seconden (default {MAX_SESSION_S:.0f}, "
                         f"13 sep 2026: instelbaar gemaakt voor langere begeleide tests, bv. een deuropening)")
    args = p.parse_args()
    max_session_s = args.max_session_s

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

    def color_image_cb(msg: Image):
        cache.color_frame = bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")

    def color_info_cb(msg: CameraInfo):
        if cache.color_fx is None and msg.k[0] > 0:
            cache.color_fx, cache.color_cx = msg.k[0], msg.k[2]

    node.create_subscription(LaserScan, "/scan_fixed", scan_cb, qos_profile_sensor_data)
    node.create_subscription(Odometry, "/odom_fused", odom_cb, 10)
    node.create_subscription(CameraInfo, "/camera/depth/camera_info", depth_info_cb, 10)
    node.create_subscription(Image, "/camera/depth/image_raw", depth_image_cb, qos_profile_sensor_data)
    node.create_subscription(Image, "/camera/color/image_raw", color_image_cb, qos_profile_sensor_data)
    node.create_subscription(CameraInfo, "/camera/color/camera_info", color_info_cb, 10)

    grid = NoveltyGrid(cell_m=0.20)
    session_start = None
    watchdog_tripped = False
    was_wandering = False
    target_world_heading_deg = None  # vastgehouden doel, i.p.v. elke cyclus opnieuw kiezen (zie fix 12 sep 2026)
    v_state = {"consecutive_centered": 0, "last_trigger_ts": 0.0, "last_turn_ts": 0.0, "last_seen_ts": 0.0}

    def normalize_deg(d):
        return ((d + 180.0) % 360.0) - 180.0

    def send_move(vx=0.0, vy=0.0, vyaw=0.0, why=""):
        if not args.live:
            node.get_logger().info(f"[DRY-RUN] zou sturen: robot.move vx={vx:+.3f} vy={vy:+.3f} vyaw={vyaw:+.3f} ({why})")
            return
        resp = mutod.move(vx=vx, vy=vy, vyaw=vyaw)
        node.get_logger().info(f"[LIVE] robot.move vx={vx:+.3f} vy={vy:+.3f} vyaw={vyaw:+.3f} ({why}) -> {resp.get('result') or resp.get('error')}")

    def send_stop(why=""):
        node.get_logger().info(f"[STOP] {why}")
        if not args.live:
            node.get_logger().info("[DRY-RUN] zou sturen: robot.stop")
            return
        resp = mutod.stop()
        node.get_logger().info(f"[LIVE] robot.stop -> {resp.get('result') or resp.get('error')}")

    def handle_obstacle_stop(why: str) -> bool:
        """Wordt aangeroepen bij elke veiligheidsstop i.p.v. blind een nieuwe
        uitwijkrichting te kiezen. Stopt altijd eerst, kijkt dan (als er een
        kleurbeeld beschikbaar is) met YOLO wat er voor staat. Bij een
        gedetecteerde persoon: meldt interactie aan mutod (robot.notify_
        interaction) en bewaart een foto (INTERACTION_SNAPSHOT_PATH) -- geeft
        True terug zodat de aanroeper GEEN nieuwe uitwijkrichting kiest (de
        eigen behavior.py-prioriteit regelt vanzelf dat WANDER de volgende
        cyclus verlaten wordt). Bij iets anders (of geen beeld/detectie-
        fout): logt wat herkend is (of dat er niets herkend kon worden) en
        geeft False terug -- aanroeper gaat door met de bestaande uitwijklogica."""
        send_stop(why)
        if cache.color_frame is None:
            node.get_logger().info("geen kleurbeeld beschikbaar voor YOLO-check -- ga door met uitwijken")
            return False
        try:
            buf = io.BytesIO()
            PILImage.fromarray(cache.color_frame).save(buf, format="JPEG", quality=85)
            buf.seek(0)
            resp = requests.post(YOLO_JETSON_URL, files={"color": ("color.jpg", buf, "image/jpeg")}, timeout=8)
            resp.raise_for_status()
            detections = resp.json().get("detections", [])
        except requests.RequestException as exc:
            node.get_logger().warn(f"YOLO-check mislukt ({exc}) -- ga door met uitwijken")
            return False

        labels = sorted({d["label"] for d in detections})
        if "person" in labels:
            node.get_logger().info(f"YOLO herkent PERSOON in de weg ({labels}) -- interactie melden aan mutod, geen uitwijk")
            beep_recognition()
            try:
                PILImage.fromarray(cache.color_frame).save(INTERACTION_SNAPSHOT_PATH)
                node.get_logger().info(f"foto bewaard: {INTERACTION_SNAPSHOT_PATH}")
            except OSError as exc:
                node.get_logger().warn(f"kon interactie-foto niet bewaren: {exc}")
            if args.live:
                resp = mutod.notify_interaction()
                node.get_logger().info(f"[LIVE] robot.notify_interaction -> {resp.get('result') or resp.get('error')}")
            else:
                node.get_logger().info("[DRY-RUN] zou sturen: robot.notify_interaction")
            return True

        node.get_logger().info(f"YOLO herkent geen persoon (gezien: {labels or 'niets'}) -- ga door met uitwijken")
        return False

    def check_white_v() -> bool:
        """13 sep 2026: doel omgezet van witte-V-papier naar roze bal (zie
        detect_pink_ball) -- functienaam en v_state/V_*-namen intern ongewijzigd
        gelaten om de diff klein te houden, maar dit reageert nu op de bal.
        Kijkt of het doel-object zichtbaar is en reageert daarop, ONGEACHT de
        huidige mutod-modus (dit is een direct stimulus-antwoord, geen
        wander-gedrag). Geeft True terug als er een actie (draaien of stapje)
        is ondernomen, OF als het object recent genoeg gezien is (V_LOST_GRACE_S)
        om voorrang te houden zonder deze cyclus zelf iets te doen -- in beide
        gevallen slaat de aanroeper de rest van deze cyclus (het gewone
        stuurprogramma) over, zodat een enkel gemist frame niet meteen tot
        tegenstrijdige aansturing leidt."""
        if cache.color_frame is None:
            return False
        found, solidity, area, center_x = detect_pink_ball(cache.color_frame)
        now = time.monotonic()
        if not found:
            if v_state["consecutive_centered"] > 0:
                node.get_logger().info("roze bal niet meer zichtbaar (telling -1 i.p.v. reset, kan nog herstellen)")
                v_state["consecutive_centered"] -= 1
            if (now - v_state["last_seen_ts"]) < V_LOST_GRACE_S:
                return True  # recent genoeg gezien -- voorrang blijft, stuurprogramma wacht
            return False
        if cache.color_fx is None:
            return False  # nog geen camera-intrinsics, kan geen bearing berekenen

        v_state["last_seen_ts"] = now
        img_w = cache.color_frame.shape[1]
        offset_px = center_x - img_w / 2.0
        bearing_deg = math.degrees(math.atan2(offset_px, cache.color_fx))

        if abs(bearing_deg) > V_BEARING_DEADBAND_DEG:
            v_state["consecutive_centered"] = max(0, v_state["consecutive_centered"] - 1)
            node.get_logger().info(f"roze bal gezien maar niet gecentreerd (bearing={bearing_deg:+.1f} graden, solidity={solidity:.2f})")
            if (now - v_state["last_turn_ts"]) < WANDER_CYCLE_S:
                return True  # net gedraaid, deze cyclus verder niets doen
            v_state["last_turn_ts"] = now
            vyaw = max(-V_MAX_TURN_VYAW, min(V_MAX_TURN_VYAW, -V_TURN_GAIN * bearing_deg))
            send_move(vyaw=vyaw, why=f"draaien naar roze bal (bearing={bearing_deg:+.1f} graden)")
            return True

        v_state["consecutive_centered"] += 1
        node.get_logger().info(
            f"roze bal GECENTREERD (bearing={bearing_deg:+.1f}, solidity={solidity:.2f}, oppervlak={area:.0f}px, "
            f"{v_state['consecutive_centered']}/{V_CONSECUTIVE_FRAMES_REQUIRED} op rij)"
        )
        if v_state["consecutive_centered"] < V_CONSECUTIVE_FRAMES_REQUIRED:
            return True
        if (now - v_state["last_trigger_ts"]) < V_COOLDOWN_S:
            node.get_logger().info(f"V bevestigd maar nog in cooldown ({V_COOLDOWN_S:.0f}s) -- geen nieuwe stap")
            return True
        v_state["last_trigger_ts"] = now
        v_state["consecutive_centered"] = 0
        send_move(vx=V_FORWARD_VX, why="roze bal gecentreerd en bevestigd, stapje vooruit")
        return True

    def cycle():
        nonlocal session_start, watchdog_tripped, was_wandering, target_world_heading_deg

        # watchdog-stop is hard en geldt ook voor de witte-V-reactie (anders kon de V
        # 'm omzeilen) -- maar get_mode() hieronder moet wel blijven lopen, anders
        # merkt dit cycle() nooit meer dat de modus (en dus watchdog_tripped) is
        # gewisseld en blijft het voor altijd genegeerd i.p.v. alleen deze sessie.
        if not watchdog_tripped and check_white_v():
            return  # witte-V-reactie heeft voorrang boven het gewone wander-gedrag

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
        elif time.monotonic() - session_start > max_session_s:
            send_stop(f"WATCHDOG: {max_session_s:.0f}s wander-sessie voorbij, hard stoppen (herstart dit script om door te gaan)")
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

        # TEKEN-FIX 13 sep 2026 (vervolg): deze "iets ergens heel dichtbij"-
        # check (12 sep 2026, originele kast-incident-fix) stond hier
        # BOVENAAN de hele cyclus en blokkeerde daardoor ook DRAAIEN, niet
        # alleen vooruitlopen -- gevonden tijdens een live deur-test: de
        # robot bevroor volledig zodra de deurposten aan weerszijden
        # dichtbij kwamen, kon zelfs niet meer proberen een andere richting
        # te vinden ("geen uitwijkende actie ondernomen"). Ter plekke draaien
        # brengt de robot niet dichter bij iets, dus hoeft niet door dezelfde
        # grens geraakt te worden. Verplaatst naar de FORWARD-tak hieronder
        # (samen met de corridor-check) -- blokkeert nog steeds vooruitlopen
        # bij iets kritiek dichtbij, maar draaien blijft altijd toegestaan.
        global_min_clear = min(merged.values()) if merged else None

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
            # TEKEN-FIX 13 sep 2026: gebruikte hier tot nu toe het GLOBALE
            # minimum over ALLE richtingen rondom (nodig zolang de LiDAR-
            # hoek-toewijzing zelf onbetrouwbaar was, zie
            # muto_wander_frontcheck_bug_2026-09-12-memory). Die hoek-fix is
            # inmiddels live bevestigd correct -- het globale vangnet bleek
            # daarna zelf een nieuw probleem: de robot liep vast op eenzelfde,
            # op zichzelf prima vrij doel omdat iets OPZIJ (buiten het pad)
            # de score verpestte, en zou om dezelfde reden nooit door een
            # deuropening durven lopen (posten aan weerszijden dichtbij, pad
            # zelf vrij). Nu: alleen de corridor recht vooruit (robot-eigen
            # richting, +-FORWARD_CORRIDOR_HALF_DEG -- ruwweg de robotbreedte
            # geprojecteerd op de loopafstand) telt mee voor DEZE check. De
            # bredere 360-graden "KRITIEK dichtbij"-noodstop hierboven blijft
            # ongewijzigd als laatste vangnet voor iets dat overal rondom
            # echt te dichtbij komt.
            corridor_clear = min(
                (v for h, v in merged.items() if abs(h) <= FORWARD_CORRIDOR_HALF_DEG),
                default=None,
            )
            # Twee checks moeten allebei slagen voor een FORWARD-commando:
            # (1) het pad recht vooruit zelf vrij genoeg (corridor_clear), en
            # (2) niets ergens rondom kritiek dichtbij (global_min_clear,
            # de vroegere top-level gate) -- een laatste vangnet specifiek
            # tegen vooruitlopen, ongeacht of de corridor-berekening zelf
            # ergens een fout zou hebben. Draaien (else-tak) heeft geen van
            # beide checks nodig, zie de uitleg hierboven.
            if global_min_clear is not None and global_min_clear < FORWARD_SAFETY_MIN_M * 0.7:
                is_interaction = handle_obstacle_stop(f"KRITIEK dichtbij iets rondom (min={global_min_clear:.2f}m), niet vooruit")
                target_world_heading_deg = None
                if not is_interaction:
                    node.get_logger().info("nieuw doel volgende cyclus")
                return
            if corridor_clear is None or corridor_clear < FORWARD_SAFETY_MIN_M:
                handle_obstacle_stop(f"veiligheidscheck faalt (laagste clearance in het pad vooruit={corridor_clear}), niet vooruit")
                target_world_heading_deg = None  # opnieuw kiezen volgende cyclus (tenzij interactie)
                return
            if corridor_clear < CENTERING_NARROW_M:
                left_clear = merged.get(CENTERING_SIDE_DEG)
                right_clear = merged.get(-CENTERING_SIDE_DEG)
                if left_clear is not None and right_clear is not None:
                    imbalance = left_clear - right_clear
                    target_side_clear = max(left_clear, right_clear)
                    if abs(imbalance) > CENTERING_MIN_IMBALANCE_M and target_side_clear >= STRAFE_SAFETY_MIN_M:
                        vy = math.copysign(CENTERING_VY, imbalance)
                        send_move(vy=vy, why=(
                            f"smalle doorgang (corridor={corridor_clear:.2f}m), centreren "
                            f"(links={left_clear:.2f} rechts={right_clear:.2f})"
                        ))
                        return
            send_move(vx=FORWARD_VX, why=f"doel {offset_to_target:+.0f} graden ~voorwaarts, lopen")
        elif abs(offset_to_target) >= STRAFE_MIN_DEG:
            # 13 sep 2026: doel ligt ongeveer opzij -- direct zijwaarts
            # stappen (SHIFT_LEFT/RIGHT, al fysiek bevestigd) i.p.v. eerst
            # draaien en dan pas lopen. Nuttig in een nauwe ruimte. Eigen
            # veiligheidscheck op de clearance-bucket het dichtst bij de
            # richting van het doel zelf (niet de brede vooruit-corridor,
            # want de robot beweegt hier opzij, niet naar voren).
            nearest_bucket = min(merged.keys(), key=lambda h: abs(h - offset_to_target)) if merged else None
            strafe_clear = merged.get(nearest_bucket) if nearest_bucket is not None else None
            if strafe_clear is None or strafe_clear < STRAFE_SAFETY_MIN_M:
                handle_obstacle_stop(f"veiligheidscheck faalt (clearance richting zijwaarts doel={strafe_clear}), niet strafen")
                target_world_heading_deg = None
                return
            vy = math.copysign(STRAFE_VY, offset_to_target)
            send_move(vy=vy, why=f"doel {offset_to_target:+.0f} graden ~opzij, strafen i.p.v. draaien")
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
