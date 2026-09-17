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
import subprocess
import sys
import time

import cv2
import numpy as np
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


def speak(text: str, logger=None):
    """Piper (lokale neurale TTS, zie muto_piper_tts_2026-09-13-memory) ->
    aplay via een pipe, fire-and-forget zodat een trage/falende TTS-aanroep
    de wander-cyclus nooit blokkeert."""
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
        if logger:
            logger.warn(f"kon niet spreken (piper/aplay niet gevonden?): {exc}")


def detect_pink_ball(frame_rgb) -> tuple:
    """13 sep 2026: vervangt de oude witte-V-detectie (verward met het witte
    deurkozijn, zie muto_wander_frontcheck... nee, zie de sessie van vandaag --
    een groot statisch wit vlak op afstand werd foutief als V herkend). Een
    roze bal is op kleur (i.p.v. helderheid) te vinden EN vrijwel perfect
    convex (solidity ~1.0), wat het kozijn (niet roze, niet rond) vanzelf
    uitsluit. Kleurdrempels gekalibreerd op een echte foto van de bal (zie
    /home/pi/current_camera_frame.jpg, 13 sep 2026), niet gegokt.
    Geeft (gevonden, solidity, area, center_x_px, center_y_px) terug -- zelfde
    vorm als de oude functie plus center_y (17 sep 2026, nodig om de bijpassende
    pixel in het dieptebeeld op te zoeken), solidity is hier een rondheids-/
    convexiteitscheck i.p.v. een V-inkeping-check. frame_rgb: numpy-array, rgb8."""
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
                best = (solidity, area, x + w / 2.0, y + h / 2.0)

    if best is None:
        return False, None, None, None, None
    return True, best[0], best[1], best[2], best[3]


def estimate_ball_depth_m(depth_frame, color_w, color_h, center_x, center_y, window=5, min_valid_fraction=0.5):
    """17 sep 2026 (gebruikersvraag: afstand meten via de dieptecamera i.p.v.
    alleen de pixel-oppervlak-proxy). De astra_camera-launch draait met
    depth_registration:=true, dus depth_frame is al uitgelijnd op het
    kleurbeeld -- schaalt center_x/center_y alleen naar depth_frame's eigen
    resolutie als die toch afwijkt van het kleurbeeld. Neemt de mediaan van
    geldige (niet-nul, INVALID_DEPTH_MM=0 is de Astra/OpenNI2-conventie voor
    "geen meting") metingen in een klein venster rond het middelpunt i.p.v.
    1 pixel.

    17 sep 2026 (bugfix, live gevonden): de eerste versie gaf een mediaan
    terug zodra er ook maar EEN enkele geldige pixel in het venster zat --
    live bleek dat op de vloer vlak voor de robot (waar de bal altijd staat)
    bijna het hele venster ongeldig is en een enkele losse ruis-pixel dan een
    overtuigend uitziend maar nep getal opleverde (een eerste "0.79m, geldig"
    resultaat bleek bij nader onderzoek zo'n toevalstreffer -- de bal stond
    toen zelfs nog dieper in de dode zone dan een latere meting die wel
    terecht None gaf). Nu pas een resultaat als minstens min_valid_fraction
    van het venster geldig is.

    Live bevestigd (17 sep 2026, volledige framekaart): deze Astra-camera
    geeft alleen geldige diepte in ongeveer de bovenste 40-45% van het beeld
    (rijen ~20-200 van 480) -- de vloer vlak voor de robot (waar de bal
    altijd staat) is een structurele dode zone door de montagehoek, GEEN
    kwestie van te dichtbij. Diepte is dus niet bruikbaar voor de bal-
    afstand in het traject waar het aantikken triggert; blijft wel bruikbaar
    voor obstakels verder weg (bestaand gebruik in depth_obstacle.py)."""
    if depth_frame is None or center_x is None or center_y is None:
        return None
    dh, dw = depth_frame.shape[:2]
    dx = int(center_x * dw / color_w)
    dy = int(center_y * dh / color_h)
    x0, x1 = max(0, dx - window), min(dw, dx + window + 1)
    y0, y1 = max(0, dy - window), min(dh, dy + window + 1)
    patch = depth_frame[y0:y1, x0:x1]
    valid = patch[patch > 0]
    if patch.size == 0 or (valid.size / patch.size) < min_valid_fraction:
        return None
    return float(np.median(valid)) / 1000.0  # mm -> m


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
# 17 sep 2026 (bugfix ronde 1): V_MAX_TURN_VYAW/V_TURN_GAIN stonden op dezelfde
# waarde als face_follow_controller.py (0.10 / 0.01), maar die controller
# corrigeert elke 0.6s (mutod's deadman-venster, CORRECTION_INTERVAL_S) terwijl
# cycle() hier maar om de WANDER_CYCLE_S=2.0s draait -- 3.3x minder vaak, dus
# dezelfde vyaw-waarde per stap draait fysiek 3.3x verder door. Live bevestigd:
# bij verzadigde vyaw=0.10 sprong de bearing herhaaldelijk van bv. +10 naar
# -10 graden na EEN correctie (~20 graden/stap) -- ruim voorbij het doel,
# aanhoudende oscillatie i.p.v. centreren. V_MAX_TURN_VYAW is daarom verkleind
# naar 0.05 (~10 graden/stap bij verzadiging, schatting op basis van die
# 20deg/0.10-verhouding).
#
# 17 sep 2026 (bugfix ronde 2, gevonden direct na ronde 1 live te testen):
# V_TURN_GAIN simpelweg verkleinen (naar 0.0015) gaf een NIEUW, erger probleem
# -- bij bearingen net voorbij de deadband (~9 graden) werd vyaw zo klein
# (0.0015*9=0.0135) dat mutod's EIGEN interne deadband (MOVE_DEADBAND_RADPS=
# 0.02, mutod/daemon.py) het commando stil negeerde ({'direction': None,
# 'note': 'binnen deadband, geen gait verstuurd'}) -- de robot bleef dan
# volledig stilstaan i.p.v. draaien, meerdere cycli achtereen live bevestigd.
# Fix: V_MIN_TURN_VYAW garandeert dat een eenmaal BESLOTEN correctie altijd
# ruim boven mutod's 0.02-drempel blijft (0.03, 50% marge), ongeacht hoe klein
# V_TURN_GAIN*bearing zelf uitkomt vlak na de deadband-rand.
V_MAX_TURN_VYAW = 0.05
V_MIN_TURN_VYAW = 0.03         # moet ruim boven mutod's MOVE_DEADBAND_RADPS=0.02 blijven
V_TURN_GAIN = 0.003            # vyaw = clamp(V_MIN_TURN_VYAW, V_MAX_TURN_VYAW, V_TURN_GAIN*|bearing|)
V_CONSECUTIVE_FRAMES_REQUIRED = 3  # cycle() draait al maar om de 2s, dus minder frames nodig dan white_v_detector.py's losse 0.3s-timer
V_COOLDOWN_S = 4.0             # geen nieuwe stap binnen dit venster na een trigger
V_FORWARD_VX = 0.03
V_LOST_GRACE_S = 2.0           # V-voorrang blijft dit lang staan na een gemist frame,
                                # zodat het stuurprogramma niet meteen overneemt bij flikkering

# 17 sep 2026 (gebruikersidee): de bal werd in live tests herhaaldelijk "te
# dichtbij" uit beeld verloren (laatst gemeten oppervlak vlak voor verlies:
# 4700-11000px), duidelijk anders dan een "gewoon nooit goed gezien"-verlies
# (een paar honderd px). CLOSE_LOST_AREA_PX scheidt die twee gevallen. Live
# bevestigd dat FOUND_AREA_PX (15000px) daarbij structureel onhaalbaar is --
# de bal valt telkens al eerder uit beeld. Daarom (tweede gebruikersidee,
# zelfde sessie): een "te dichtbij"-verlies IS zelf het gevonden-signaal --
# stap terug + aantikken, i.p.v. te blijven proberen dichterbij te komen.
CLOSE_LOST_AREA_PX = 2000
STEP_BACK_VX = -V_FORWARD_VX

# 17 sep 2026 (gebruikersidee): --search maakt van het bestaande volg-gedrag
# een echt "zoek de bal"-commando -- eenmalige TTS-melding zodra hij dichtbij
# genoeg is (ipv oneindig door te blijven naderen/stapjes nemen). Drempel is
# een RUWE SCHATTING op basis van eerder gemeten oppervlaktes tijdens nadering
# (groeide van ~3000px naar ~30000px terwijl hij echt dichterbij liep, zie
# muto_wander_door_crossing_milestone-sessie) -- geen exacte "raakt de bal"-
# meting, startwaarde om te tunen.
FOUND_AREA_PX = 15000
PIPER_BIN = "/root/piper_venv/bin/piper"
PIPER_MODEL = "/root/piper_voices/en_US-amy-medium.onnx"
PIPER_SAMPLE_RATE = 22050
TTS_ALSA_DEVICE = "plughw:2,0"  # kaartnummer kan per boot wisselen, zie muto_audio_module_fix_2026-09-13-memory


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


# 17 sep 2026: los-poot-besturing (raw "op"-kanaal, zelfde patroon als
# send_buzzer -- eigen korte verbinding per aanroep, geen JSON-RPC-framing).
def send_move_leg(leg: str, angles, runtime_ms: int = 600):
    try:
        s = socket.create_connection((MUTOD_HOST, MUTOD_PORT), timeout=2)
        s.sendall((json.dumps({"op": "move_leg", "leg": leg, "angles": list(angles), "runtime_ms": runtime_ms}) + "\n").encode())
        resp = json.loads(s.recv(4096).decode())
        s.close()
        return resp
    except OSError:
        return None


# Handmatig gekalibreerde "kijk-naar-beneden"-houding (17 sep 2026, zie
# muto_manual_pose_calibration_2026-09-17-memory voor de methode): torque per
# poot uitgezet, met de hand in de gewenste kijkhoek gezet, hoeken afgelezen.
# RR/LR (achter) blijven op hun normale sta-hoek -- alleen voor+midden gekanteld.
LOOK_DOWN_POSE = {
    "RF": [20, -37, 15],
    "RM": [0, -52, -19],
    "LM": [4, -38, -14],
    "LF": [-5, -50, -8],
}


# Normale sta-houding, 17 sep 2026 gemeten (niet gegokt) via twee
# onafhankelijke robot.state-uitlezingen na aparte sessies -- dit is mutod's
# eigen deadman/stay_put-idle-stand, per poot afgelezen en gemiddeld. Gebruikt
# om (a) elke run met dezelfde, bekende startpositie te beginnen, ongeacht
# welke houding de vorige sessie achterliet, en (b) zelf soepel terug te gaan
# vanuit een aangepaste houding (bv. LOOK_DOWN_POSE) voordat er een echt
# gait-commando volgt -- anders doet de STM32-firmware die reset zelf,
# stilzwijgend en abrupt (zie OVERDRACHT_2026-09-17.txt sectie 3).
NEUTRAL_POSE = {
    "RF": [0, -36, -20],
    "RM": [0, -36, -20],
    "RR": [0, -36, -20],
    "LR": [0, -35, -20],
    "LM": [0, -35, -20],
    "LF": [0, -37, -21],
}


def move_to_pose(pose: dict, logger=None, runtime_ms: int = 700, settle_s: float = 0.8, label: str = "houding"):
    for leg, angles in pose.items():
        resp = send_move_leg(leg, angles, runtime_ms=runtime_ms)
        if logger:
            logger.info(f"{label}: {leg} -> {resp}")
    time.sleep(settle_s)  # geef de servo's tijd om de houding daadwerkelijk te bereiken


def move_to_look_down_pose(logger=None):
    move_to_pose(LOOK_DOWN_POSE, logger=logger, runtime_ms=600, settle_s=0.8, label="kijk-naar-beneden-houding")


def move_to_neutral_pose(logger=None):
    move_to_pose(NEUTRAL_POSE, logger=logger, runtime_ms=700, settle_s=0.8, label="neutrale sta-houding")


def get_servo_angles():
    try:
        s = socket.create_connection((MUTOD_HOST, MUTOD_PORT), timeout=2)
        s.sendall((json.dumps({"jsonrpc": "2.0", "id": 1, "method": "robot.state"}) + "\n").encode())
        resp = json.loads(s.recv(4096).decode())
        s.close()
        return resp.get("result", {}).get("servos")
    except OSError:
        return None


# Handmatig gekalibreerde aanraak-houdingen voor RF, meerdere afstanden
# (17 sep 2026, zie muto_targeted_leg_tap_2026-09-17-memory voor de eerste
# poging en muto_search_turn_and_close_range_2026-09-17-memory voor deze
# tabel) -- de IK-route liep vast op een reken-grensprobleem (zelfs het
# neutrale punt zat al op de rekengrens van acos()), dus net als
# LOOK_DOWN_POSE met de hand gevonden: torque uit, met de hand tegen een
# echt neergelegde bal aan gezet, hoeken afgelezen. Deze keer twee punten,
# beide vanuit de normale NEUTRAL_POSE-nadering (niet LOOK_DOWN_POSE zoals de
# oude eenmalige TAP_POSE_RF), gesleuteld op pixel-oppervlak -- NIET diepte:
# zie muto_ball_depth_measurement_2026-09-17-memory, de dieptecamera is
# structureel blind op de vloer vlak voor de robot (Astra Pro Plus'
# officiele 0.6m-minimumbereik + montagehoek), dus oppervlak is de enige
# bruikbare afstandsmaat hier.
TAP_POSE_TABLE = [
    (1900, [53, -39, 29]),   # verder weg -- poot verder uitgestrekt
    (11789, [56, -31, 35]),  # dichterbij
]


def interpolate_tap_pose(area_px):
    """Kiest/interpoleert de gekalibreerde RF-aanraakhouding op basis van het
    laatst gemeten pixel-oppervlak van de bal. Buiten het gekalibreerde
    bereik: klemt op het dichtstbijzijnde kalibratiepunt i.p.v. te
    extrapoleren (een geëxtrapoleerde hoek is nooit fysiek geverifieerd)."""
    table = TAP_POSE_TABLE
    if area_px <= table[0][0]:
        return table[0][1]
    if area_px >= table[-1][0]:
        return table[-1][1]
    for (a0, p0), (a1, p1) in zip(table, table[1:]):
        if a0 <= area_px <= a1:
            t = (area_px - a0) / (a1 - a0)
            return [p0[i] + t * (p1[i] - p0[i]) for i in range(3)]
    return table[-1][1]  # zou niet moeten gebeuren, veilige terugval


TAP_SNAPSHOT_VOOR_PATH = "/root/tap_snapshot_voor.jpg"
TAP_SNAPSHOT_NA_PATH = "/root/tap_snapshot_na.jpg"


def _save_tap_snapshot(color_frame, path, label, logger=None):
    if color_frame is None:
        if logger:
            logger.warn(f"geen kleurbeeld beschikbaar voor tik-foto ({label})")
        return
    try:
        PILImage.fromarray(color_frame).save(path)
        if logger:
            logger.info(f"tik-foto ({label}) bewaard: {path}")
    except OSError as exc:
        if logger:
            logger.warn(f"kon tik-foto niet bewaren ({label}): {exc}")


def start_tap(area_px, cache, v_state, logger=None):
    """17 sep 2026 (gebruikersvraag: foto voor/na de tik om te verifieren of
    'ie echt raak is). NIET blokkerend (geen time.sleep tijdens het vasthouden)
    -- rclpy.spin_once() opnieuw aanroepen vanuit een timer-callback die al
    door de lopende rclpy.spin()-executor wordt afgehandeld, zou een
    onveilige/niet-gedocumenteerde herintrede zijn (cache.color_frame wordt
    toch niet ververst tijdens een blokkerende sleep, want de image-
    subscription-callback kan dan niet vuren). In plaats daarvan: hier alleen
    de voor-foto + de beweging naar de aanraakhouding, RF blijft daar staan.
    cycle() ziet op de VOLGENDE reguliere cyclus (WANDER_CYCLE_S later, dus
    met een echt ververst cache.color_frame) dat er een tik openstaat, maakt
    dan de na-foto en trekt RF terug -- puur cyclus-gedreven, geen sleep."""
    servos = get_servo_angles()
    if servos is None:
        if logger:
            logger.warn("kon RF-hoeken niet uitlezen, sla aantikken over")
        return
    rf_before = [servos["1"], servos["2"], servos["3"]]
    target = interpolate_tap_pose(area_px)
    if logger:
        logger.info(f"RF aantikken (oppervlak={area_px:.0f}px -> geinterpoleerde houding): {rf_before} -> {target}")
    _save_tap_snapshot(cache.color_frame, TAP_SNAPSHOT_VOOR_PATH, "voor", logger)
    send_move_leg("RF", target, runtime_ms=500)
    v_state["tap_rf_before"] = rf_before


def finish_tap_if_pending(cache, v_state, logger=None) -> bool:
    """Wordt elke cyclus als eerste aangeroepen. Geeft True terug als er een
    tik openstond (en dus deze cyclus is afgehandeld, verder niets anders
    doen) -- anders False."""
    if v_state["tap_rf_before"] is None:
        return False
    _save_tap_snapshot(cache.color_frame, TAP_SNAPSHOT_NA_PATH, "na", logger)
    rf_before = v_state["tap_rf_before"]
    v_state["tap_rf_before"] = None
    if logger:
        logger.info(f"RF terug naar uitgangshoek {rf_before}")
    send_move_leg("RF", rf_before, runtime_ms=500)
    return True


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
        self.depth_fy = None
        self.depth_cy = None
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
    p.add_argument("--search", action="store_true",
                    help="17 sep 2026: maakt van het bestaande volg-gedrag een 'zoek de roze bal'-opdracht -- "
                         "eenmalige TTS-melding zodra hij dichtbij genoeg is (i.p.v. oneindig door te naderen)")
    args = p.parse_args()
    max_session_s = args.max_session_s
    search_mode = args.search

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

    # 17 sep 2026: elke run begint met dezelfde, bekende neutrale sta-houding,
    # ongeacht welke houding een vorige sessie achterliet (bv. LOOK_DOWN_POSE
    # of een half afgemaakte SHIFT) -- voorkomt dat een run start vanuit een
    # onbekende/onvoorspelbare pose.
    if args.live:
        node.get_logger().info("startpositie: eerst naar de normale neutrale sta-houding")
        move_to_neutral_pose(node.get_logger())

    def scan_cb(msg: LaserScan):
        cache.scan = (msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max)

    def odom_cb(msg: Odometry):
        q = msg.pose.pose.orientation
        cache.odom = (msg.pose.pose.position.x, msg.pose.pose.position.y, quat_to_yaw_deg(q.z, q.w))

    def depth_info_cb(msg: CameraInfo):
        if cache.depth_fx is None and msg.k[0] > 0:
            cache.depth_fx, cache.depth_cx = msg.k[0], msg.k[2]
            cache.depth_fy, cache.depth_cy = msg.k[4], msg.k[5]

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
    v_state = {
        "consecutive_centered": 0, "last_trigger_ts": 0.0, "last_turn_ts": 0.0,
        "last_seen_ts": 0.0, "found": False,
        "last_area": 0.0,          # laatst gemeten oppervlak, ook als niet gecentreerd
        "tap_rf_before": None,     # niet None zolang RF in de aanraakhouding staat te wachten op de na-foto
    }
    in_custom_pose = False  # True zolang de robot in bv. LOOK_DOWN_POSE staat i.p.v. de normale sta-houding

    if search_mode:
        node.get_logger().info("--search actief: opdracht 'zoek de roze bal' gestart")
        speak("Searching for the pink ball.", node.get_logger())
        # 17 sep 2026: eerst de handmatig gekalibreerde kijk-naar-beneden-houding
        # aannemen -- vangt het geval dat de bal al vlak voor de robot ligt,
        # buiten het zicht van de normale sta-houding. Overleeft geen echte
        # loop-stap (de firmware-gait neemt alle 18 servo's over) -- send_move()
        # verlaat deze houding daarom voortaan zelf eerst soepel (in_custom_pose)
        # voordat een echt gait-commando volgt, i.p.v. de firmware dat abrupt en
        # stilzwijgend te laten doen.
        if args.live:
            move_to_look_down_pose(node.get_logger())
            in_custom_pose = True

    def normalize_deg(d):
        return ((d + 180.0) % 360.0) - 180.0

    def send_move(vx=0.0, vy=0.0, vyaw=0.0, why=""):
        nonlocal in_custom_pose
        if not args.live:
            node.get_logger().info(f"[DRY-RUN] zou sturen: robot.move vx={vx:+.3f} vy={vy:+.3f} vyaw={vyaw:+.3f} ({why})")
            return
        if in_custom_pose:
            # 17 sep 2026: voorkomt dat de STM32-gait-firmware dit zelf abrupt en
            # stilzwijgend doet zodra dit robot.move-commando binnenkomt -- eerst
            # zelf soepel (met runtime_ms) terug naar de neutrale sta-houding.
            node.get_logger().info("verlaat aangepaste houding voor een echt bewegingscommando -- eerst soepel terug naar neutraal")
            move_to_neutral_pose(node.get_logger())
            in_custom_pose = False
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
        found, solidity, area, center_x, center_y = detect_pink_ball(cache.color_frame)
        now = time.monotonic()
        if not found:
            if v_state["consecutive_centered"] > 0:
                node.get_logger().info("roze bal niet meer zichtbaar (telling -1 i.p.v. reset, kan nog herstellen)")
                v_state["consecutive_centered"] -= 1
            if (now - v_state["last_seen_ts"]) < V_LOST_GRACE_S:
                return True  # recent genoeg gezien -- voorrang blijft, stuurprogramma wacht

            # 17 sep 2026 (gebruikersidee, na live bevestiging dat FOUND_AREA_PX
            # structureel onhaalbaar is -- de bal valt herhaaldelijk rond
            # 9000-11000px al buiten beeld, ver onder de 15000px-drempel, en
            # het eerdere stap-terug/bukken-fallback-ontwerp bracht 'm telkens
            # weer terug naar dezelfde afstand i.p.v. vooruitgang te boeken):
            # bal buiten beeld op korte afstand IS zelf al het "dichtbij
            # genoeg"-signaal -- geen reden meer om FOUND_AREA_PX te proberen
            # halen. Dus: eenmalig een stap terug (ruimte/zicht), dan GEVONDEN
            # + aantikken, i.p.v. te blijven proberen dichterbij te komen.
            if search_mode and not v_state["found"] and v_state["last_area"] >= CLOSE_LOST_AREA_PX:
                v_state["found"] = True
                node.get_logger().info(
                    f"roze bal buiten beeld op korte afstand (laatst oppervlak={v_state['last_area']:.0f}px, "
                    f">= CLOSE_LOST_AREA_PX={CLOSE_LOST_AREA_PX}px) -- behandeld als gevonden, stap terug en aantikken"
                )
                speak("Found the pink ball!", node.get_logger())
                send_move(vx=STEP_BACK_VX, why="stap terug voor het aantikken (bal was te dichtbij uit beeld)")
                if args.live:
                    start_tap(v_state["last_area"], cache, v_state, node.get_logger())
                    # 17 sep 2026: start_tap() is niet-blokkerend (RF staat nog te
                    # bewegen/uitgestrekt) -- True houdt voorrang vast zodat cycle()
                    # deze cyclus GEEN los wander-gait-commando erbovenop stuurt;
                    # finish_tap_if_pending() rondt het volgende cyclus zelf af.
                    return True
                return False
            return False
        if cache.color_fx is None:
            return False  # nog geen camera-intrinsics, kan geen bearing berekenen

        v_state["last_seen_ts"] = now
        v_state["last_area"] = area
        img_w = cache.color_frame.shape[1]
        offset_px = center_x - img_w / 2.0
        bearing_deg = math.degrees(math.atan2(offset_px, cache.color_fx))

        if abs(bearing_deg) > V_BEARING_DEADBAND_DEG:
            v_state["consecutive_centered"] = max(0, v_state["consecutive_centered"] - 1)
            node.get_logger().info(f"roze bal gezien maar niet gecentreerd (bearing={bearing_deg:+.1f} graden, solidity={solidity:.2f})")
            if (now - v_state["last_turn_ts"]) < WANDER_CYCLE_S:
                return True  # net gedraaid, deze cyclus verder niets doen
            v_state["last_turn_ts"] = now
            turn_mag = max(V_MIN_TURN_VYAW, min(V_MAX_TURN_VYAW, V_TURN_GAIN * abs(bearing_deg)))
            vyaw = -turn_mag if bearing_deg > 0 else turn_mag
            send_move(vyaw=vyaw, why=f"draaien naar roze bal (bearing={bearing_deg:+.1f} graden)")
            return True

        v_state["consecutive_centered"] += 1
        img_h = cache.color_frame.shape[0]
        depth_m = estimate_ball_depth_m(cache.depth_frame, img_w, img_h, center_x, center_y)
        depth_txt = f"{depth_m:.2f}m" if depth_m is not None else "geen geldige meting"
        node.get_logger().info(
            f"roze bal GECENTREERD (bearing={bearing_deg:+.1f}, solidity={solidity:.2f}, oppervlak={area:.0f}px, "
            f"diepte={depth_txt}, {v_state['consecutive_centered']}/{V_CONSECUTIVE_FRAMES_REQUIRED} op rij)"
        )

        if search_mode and not v_state["found"] and area >= FOUND_AREA_PX:
            v_state["found"] = True
            node.get_logger().info(f"roze bal GEVONDEN (oppervlak={area:.0f}px >= {FOUND_AREA_PX}px) -- zoekopdracht voltooid")
            speak("Found the pink ball!", node.get_logger())
            if args.live:
                start_tap(area, cache, v_state, node.get_logger())
                return True  # niet-blokkerend, RF staat nog te bewegen -- zie andere start_tap-aanroep
            return False  # geeft voorrang terug aan het gewone stuurprogramma, geen verdere nadering

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

        # 17 sep 2026 (bugfix): de sessiewatchdog telde niet mee zolang
        # check_white_v() (bal-tracking) de cyclus onderschepte -- de tijd-check
        # stond er ACHTER en werd dus nooit bereikt tijdens actief volgen. Live
        # bevestigd (--search-tests): een sessie liep 215s+ door tegen een
        # ingestelde 180s-limiet. Fix: mode ophalen en de tijd-check doen VOORDAT
        # check_white_v() wordt aangeroepen, zodat de klok elke cyclus meetelt,
        # ongeacht wat check_white_v() doet. Bij een verbindingsfout met mutod
        # slaat dit deel over (mode=None) -- check_white_v() blijft daardoor
        # "ongeacht modus" werken zoals bedoeld, ook als mutod even niet
        # bereikbaar is.
        try:
            mode = mutod.get_mode()
        except (ConnectionError, OSError) as exc:
            node.get_logger().warn(f"kan mutod niet bereiken: {exc}")
            mode = None

        if mode is not None:
            if mode != "wander":
                if was_wandering:
                    send_stop(f"mode is nu {mode!r}, niet meer wander")
                was_wandering = False
                session_start = None
                watchdog_tripped = False
                target_world_heading_deg = None
            elif session_start is None:
                session_start = time.monotonic()
                node.get_logger().info("wander-sessie gestart")
            elif not watchdog_tripped and time.monotonic() - session_start > max_session_s:
                send_stop(f"WATCHDOG: {max_session_s:.0f}s wander-sessie voorbij, hard stoppen (herstart dit script om door te gaan)")
                watchdog_tripped = True

        # 17 sep 2026: als RF nog in de aanraakhouding staat te wachten op de
        # na-foto (zie start_tap()), dan NU afhandelen en verder niets anders
        # deze cyclus -- voorkomt dat het gewone stuurprogramma een gait-
        # commando stuurt terwijl RF nog in een niet-standaard houding staat.
        # Loopt ALTIJD, ook als de watchdog al getript is, anders zou RF voor
        # altijd uitgestrekt kunnen blijven staan.
        if finish_tap_if_pending(cache, v_state, node.get_logger()):
            return

        # watchdog-stop is hard en geldt ook voor de bal-tracking-reactie
        # (anders kon de bal-tracking 'm omzeilen).
        if not watchdog_tripped and check_white_v():
            return  # bal-tracking-reactie heeft voorrang boven het gewone wander-gedrag

        if watchdog_tripped or mode != "wander":
            return  # getript: blijft genegeerd tot het script herstart wordt (bewuste, harde grens)

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
        #
        # 17 sep 2026: ECHTE VASTLOOP GEVONDEN (live "zoek de bal"-sessie,
        # diag_clearance_snapshot.py bevestigde het) -- deze check gebruikte
        # tot nu toe het minimum over de VOLLEDIGE 360 graden, inclusief recht
        # ACHTER de robot. choose_heading()'s kandidatenlijst (-90..+90, om de
        # 30 graden) beslaat nooit die achterkant, dus koos steevast een
        # prima-vrije richting (bv. 0 graden, 1.89m) -- maar deze bredere
        # vangnet-check zag 0.24m pal achter de robot en blokkeerde vooruit-
        # lopen sowieso, elke cyclus opnieuw, zonder ooit te draaien (draaien
        # gebeurt alleen als het GEKOZEN doel zelf niet-uitgelijnd is -- hier
        # was het doel steeds al uitgelijnd). Resultaat: een volledige 180s-
        # sessie zonder ook maar 1 beweging. Iets recht achter de robot kan
        # onmogelijk geraakt worden tijdens vooruitlopen, dus telt niet meer
        # mee -- alleen de voorste hemisfeer (+-90 graden) nog, ruimer dan de
        # smalle rijstrook-check maar niet meer rondom.
        GLOBAL_SAFETY_ARC_HALF_DEG = 90.0
        global_min_clear = min(
            (v for h, v in merged.items() if abs(h) <= GLOBAL_SAFETY_ARC_HALF_DEG),
            default=None,
        )

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
                is_interaction = handle_obstacle_stop(f"KRITIEK dichtbij iets in voorste hemisfeer (min={global_min_clear:.2f}m), niet vooruit")
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
