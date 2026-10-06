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
import base64
import io
import json
import socket
import subprocess
import sys
import threading
import time

import requests
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
from PIL import Image as PILImage

sys.path.insert(0, "/root")
from depth_obstacle import estimate_clearance_by_heading  # zelfde, al-bewezen functie als wander_executor.py gebruikt

JETSON_URL = "http://192.168.68.86:8600/detect"
SNAPSHOT_INTERVAL_S = 5.0
MUTOD_HOST, MUTOD_PORT = "127.0.0.1", 8420
BEEP_TIMEOUT = 3  # 300ms -- kort "ik zag iets nieuws"-piepje
# 20 sep 2026 (AI-executive-architectuur, stap 3, zie
# muto_ai_executive_architecture_decision_2026-09-20-memory): zelfde
# qwen3-vl-via-Ollama-endpoint als robot_bridge.py's /camera/describe (juli
# 2026, zie muto_legacy_prellm_artifacts_2026-09-20-memory) -- die bridge
# zelf wordt niet hergebruikt (botst met mutod's exclusieve seriele-poort-
# eigendom), alleen dit bewezen HTTP-patroon.
OLLAMA_VISION_URL = "http://192.168.68.77:11434/api/generate"
VISION_MODEL = "qwen3-vl"
# 13 sep 2026: espeak-ng (formant-synthese) was slecht verstaanbaar, ook in het
# Engels en met een lagere spreeksnelheid -- vervangen door Piper (neurale TTS,
# lokaal/offline, zoals Reachy's cloud-TTS maar zonder de cloud-afhankelijkheid,
# zie muto_reachy_tts_research_2026-09-13-memory), user-confirmed "veel beter!!".
PIPER_BIN = "/root/piper_venv/bin/piper"
PIPER_MODEL = "/root/piper_voices/en_US-amy-medium.onnx"
PIPER_SAMPLE_RATE = 22050
# USB-speaker kaartnummer kan per boot wisselen (zie muto_audio_module_fix_2026-09-13-memory)
# -- vandaar hier als losse, makkelijk te updaten constante i.p.v. hardcoded verderop.
# 20 sep 2026: na de Pi-herstart deze sessie verschoven van card 2 -> card 3
# (bevestigd via `aplay -l` in humble_run, "USB Audio Device") -- live
# gevonden doordat de greet-skill wel piepte (STM32-buzzer, ander pad) maar
# niet sprak ("Cannot get card index for 2" / "audio open error").
# 29 sep 2026 (gebruikersidee): het KAARTNUMMER wisselt aantoonbaar per boot
# (2 -> 3 -> weer 2, alle drie live waargenomen in dezelfde week), maar de
# ALSA-kaart-ID blijft altijd "Device" (zie `aplay -l`: "card N: Device
# [USB Audio Device]", N wisselt, "Device" niet). Adresseren op NAAM i.p.v.
# nummer is dus stabiel over reboots heen, live bevestigd -- geen losse
# per-sessie check meer nodig.
TTS_ALSA_DEVICE = "plughw:CARD=Device,DEV=0"
# 20 sep 2026 (AI-executive-architectuur, eerste ECHTE skill execution --
# zie muto_skill_execution_greet_2026-09-20-memory): zelfde 2-piepjes-
# patroon als wander_executor.py's beep_recognition() (RECOGNITION_BEEP_
# TIMEOUT/GAP_S daar), hier los gedupliceerd omdat dit een ander proces is
# (geen IPC tussen wander_executor.py en yolo_snapshot_sender.py) -- bewust
# dezelfde getallen aanhouden zodat het geluid herkenbaar hetzelfde blijft.
GREET_BEEP_TIMEOUT = 3    # 300ms per piepje
GREET_BEEP_GAP_S = 0.5    # stilte tussen de twee piepjes
# Eigen bestandsnaam (niet wander_executor.py's INTERACTION_SNAPSHOT_PATH) --
# andere proces, voorkomt dat twee schrijvers elkaars snapshot overschrijven.
GREET_SNAPSHOT_PATH = "/root/llm_greet_snapshot.jpg"


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

    def beep_greet(self):
        """Zelfde 2-piepjes-patroon als wander_executor.py's beep_recognition()."""
        self.beep(GREET_BEEP_TIMEOUT)
        time.sleep(GREET_BEEP_GAP_S)
        self.beep(GREET_BEEP_TIMEOUT)


class MutodNotify:
    """20 sep 2026: aparte, kleine JSON-RPC-client naar robot.notify_
    interaction -- zelfde newline-JSON-kanaal als MutodBuzzer/MutodWorld
    Update hierboven, eigen korte verbinding per aanroep. Geen params, geen
    beweging -- werkt alleen behavior.py's last_interaction_ts bij (zie
    mutod/behavior.py). Faalt stil, zelfde reden als de andere IPC-clients
    hier: een gemiste melding mag de detectie-pijplijn niet breken."""

    def __init__(self, logger):
        self._logger = logger

    def notify_interaction(self):
        try:
            sock = socket.create_connection((MUTOD_HOST, MUTOD_PORT), timeout=2)
            req = {"jsonrpc": "2.0", "id": 1, "method": "robot.notify_interaction", "params": {}}
            sock.sendall((json.dumps(req) + "\n").encode())
            resp = sock.recv(256)
            sock.close()
            self._logger.info(f"robot.notify_interaction -> {resp.decode(errors='replace').strip()}")
        except OSError as exc:
            self._logger.warn(f"kon notify_interaction niet sturen (mutod niet bereikbaar?): {exc}")


class MutodWorldUpdate:
    """20 sep 2026 (AI-executive-architectuur, stap 2, zie
    muto_ai_executive_architecture_decision_2026-09-20-memory): pusht de
    laatste YOLO-detecties naar mutod's robot.world_update (stap 1, zelfde
    sessie) zodat robot.world_state ze kan teruggeven. Zelfde newline-JSON-
    RPC-kanaal als de rest van mutod se IPC (127.0.0.1:8420), eigen korte
    verbinding per aanroep zoals MutodBuzzer hierboven. Faalt stil (met een
    warning) als mutod niet bereikbaar is -- een gemiste push mag nooit de
    detectie-pijplijn zelf breken, zelfde reden als MutodBuzzer.beep()."""

    def __init__(self, logger):
        self._logger = logger

    def push(self, key: str, data):
        try:
            sock = socket.create_connection((MUTOD_HOST, MUTOD_PORT), timeout=2)
            req = {"jsonrpc": "2.0", "id": 1, "method": "robot.world_update",
                   "params": {"key": key, "data": data}}
            sock.sendall((json.dumps(req) + "\n").encode())
            sock.recv(256)
            sock.close()
        except OSError as exc:
            self._logger.warn(f"kon world_update niet pushen (mutod niet bereikbaar?): {exc}")


class VisionDescriber:
    """20 sep 2026 (AI-executive-architectuur, stap 3): event-driven scene-
    beschrijving via qwen3-vl op Ollama -- NIET elke cyclus (dat was
    expliciet de eis uit de doel-analyse, zie
    muto_ai_executive_architecture_decision_2026-09-20-memory), alleen als
    er iets NIEUWS gezien wordt (zelfde new_labels-trigger als de piep/
    spraak hieronder). Draait in een achtergrondthread -- een VL-aanroep
    kan meerdere seconden duren en mag de detectie-cyclus (die zelf al een
    netwerkaanroep naar de Jetson doet) nooit blokkeren, zelfde reden als
    Tts.say()'s fire-and-forget-aanpak. Een simpele lock voorkomt
    overlappende aanroepen als er kort na elkaar weer iets nieuws
    binnenkomt -- een tweede trigger wordt dan gewoon overgeslagen (niet in
    een wachtrij gezet), Ollama is hier geen concurrente-verzoeken-server."""

    def __init__(self, logger, world_update, goal_selector=None):
        self._logger = logger
        self._world_update = world_update
        self._goal_selector = goal_selector
        self._busy = threading.Lock()

    def describe_async(self, color_frame, detections, trigger_labels):
        if not self._busy.acquire(blocking=False):
            self._logger.info("vorige scene-beschrijving nog bezig, deze trigger overgeslagen")
            return
        threading.Thread(target=self._run, args=(color_frame, detections, trigger_labels), daemon=True).start()

    def _run(self, color_frame, detections, trigger_labels):
        try:
            buf = io.BytesIO()
            PILImage.fromarray(color_frame).save(buf, format="JPEG", quality=85)
            image_b64 = base64.b64encode(buf.getvalue()).decode("utf-8")
            prompt = (
                "Beschrijf kort en concreet wat je ziet in dit beeld, vanuit het "
                "perspectief van een hexapod-robot op grondniveau. Noem obstakels, "
                "vrije ruimte, en opvallende objecten. Antwoord in het Nederlands."
            )
            t0 = time.time()
            resp = requests.post(
                OLLAMA_VISION_URL,
                json={"model": VISION_MODEL, "prompt": prompt, "images": [image_b64], "stream": False},
                # 20 sep 2026: 30s was te kort -- een losse, offline meting gaf
                # 59.9s voor 1 echte aanroep (qwen3-vl:8.8B, Q4_K_M) op deze
                # machine. 120s geeft ruime marge zonder een falende aanroep
                # eerst als mislukking te loggen.
                timeout=120,
            )
            resp.raise_for_status()
            payload = resp.json()
            # 20 sep 2026 (bugfix, live gevonden): 'response' kwam soms leeg
            # terug terwijl het model wel degelijk iets zinnigs produceerde --
            # een losse diagnostische aanroep liet zien dat qwen3-vl (heeft
            # "thinking" als capability) de hele inhoud dan in het aparte
            # 'thinking'-veld zet i.p.v. 'response'. Val terug op 'thinking'
            # als 'response' leeg is, i.p.v. een lege beschrijving te loggen.
            description = (payload.get("response") or payload.get("thinking") or "").strip()
            elapsed_ms = round((time.time() - t0) * 1000)
            self._logger.info(f"[{elapsed_ms}ms] scene-beschrijving: {description!r}")
            self._world_update.push("scene_description", {
                "text": description,
                "trigger_labels": sorted(trigger_labels),
            })
            # 20 sep 2026 (stap 4): pas een doel/skill kiezen NA een geslaagde
            # beschrijving -- de beslissing gebruikt die tekst als context,
            # dus geketend i.p.v. los getriggerd (voorkomt een race waarbij
            # de doel-selector een oude/lege beschrijving zou gebruiken).
            if self._goal_selector is not None and description:
                self._goal_selector.decide(detections, description, color_frame=color_frame)
        except requests.RequestException as exc:
            self._logger.warn(f"vision-LLM-aanroep mislukt (Ollama niet bereikbaar?): {exc}")
        finally:
            self._busy.release()


class GoalSelector:
    """20 sep 2026 (AI-executive-architectuur, stap 4): kiest een doel/skill
    op basis van de laatste waarneming (YOLO-labels + scene-beschrijving),
    via een TEKST-aanroep naar hetzelfde al-geladen qwen3-vl-model op Ollama
    -- geen apart tekstmodel (bv. qwen3:8b) gepulld, eerst meten of dat
    uberhaupt nodig is (zie de inference-backend-afweging in
    muto_ai_executive_architecture_decision_2026-09-20-memory: klein
    beginnen, alleen uitbreiden bij een gemeten probleem).

    Sindsdien (nog steeds 20 sep 2026, zie
    muto_skill_execution_greet_2026-09-20-memory): de EERSTE echte skill-
    executie, bewust minimaal gescoped op expliciete keuze van de gebruiker
    ("klein beginnen: observeren/negeren vs 1 veilige skill", geen
    beweging). De LLM mag ALLEEN kiezen uit een vaste, in de prompt
    afgedwongen vocabulaire (GEEN vrije tekst meer als skill-veld) --
    "observe" (niets doen, alleen loggen/pushen, zoals voorheen) of "greet"
    (het al-bestaande, elders al live-bevestigde interactie-aankondigings-
    patroon: 2 piepjes + snapshot + robot.notify_interaction, zie
    wander_executor.py's beep_recognition()/notify_interaction() voor het
    origineel -- hier zelfstandig gedupliceerd, ander proces, geen IPC
    ertussen). Nog steeds GEEN robot.move/wander-aansturing -- dat blijft
    een aparte, latere stap met zijn eigen expliciete aankondiging vooraf,
    zie muto_announce_before_movement-memory.

    Draait al binnen VisionDescriber's achtergrondthread (geketend na een
    geslaagde beschrijving), dus geen eigen thread/lock nodig hier."""

    ALLOWED_SKILLS = ("observe", "greet")

    def __init__(self, logger, world_update, buzzer=None, notify=None, tts=None):
        self._logger = logger
        self._world_update = world_update
        self._buzzer = buzzer
        self._notify = notify
        self._tts = tts

    def decide(self, detections, description, color_frame=None):
        labels = sorted({d["label"] for d in detections})
        # 29 sep 2026 (gebruikersidee, na onderzoek van een extern Ollama+
        # Kokoro+"Speech Manager"-architectuurvoorstel, zie
        # muto_speech_intent_2026-09-29-memory): eerste stap daaruit --
        # laat de LLM zelf de gesproken TEKST bedenken (i.p.v. een vaste
        # Python-template) plus een paar spreekparameters, i.p.v. alleen
        # een skill-label. Bewust GEEN "emotion" die al iets doet -- Piper
        # heeft geen emotie-sturing, dat veld wordt puur gelogd/opgeslagen
        # als voorbereiding op een latere TTS-engine (zie het onderzoek:
        # dat was juist het hele punt van een los "speech"-object, de
        # engine moet later te vervangen zijn zonder dit stuk te wijzigen).
        prompt = (
            "Je bent de AI-executive van een hexapod-robot (Muto). Op basis van "
            "de waarneming hieronder, kies een doel en een vaardigheid.\n"
            'Het "skill"-veld MOET exact een van deze twee waarden zijn, niets '
            'anders: "observe" (alleen waarnemen, geen actie -- de standaardkeuze '
            'bij twijfel) of "greet" (een korte, veilige interactie-aankondiging: '
            '2 piepjes + een gesproken zin -- kies dit ALLEEN als er duidelijk '
            "een persoon aanwezig is die aandacht verdient). Er is nog GEEN "
            "beweeg-vaardigheid beschikbaar, kies daar dus nooit voor.\n"
            'Als skill "greet" is, voeg ook een "speech"-object toe met wat de '
            "robot moet zeggen. Antwoord ALLEEN met JSON in dit exacte formaat, "
            'niets anders: {"goal": "<kort label>", "target": "<label of null>", '
            '"skill": "observe of greet", "priority": <getal 0.0-1.0>, '
            '"reasoning": "<korte Nederlandse uitleg>", "speech": {"text": '
            '"<korte natuurlijke Engelse zin, alleen als skill=greet, anders null>", '
            '"emotion": "<kort label zoals curious/friendly/surprised>", '
            '"speed": <getal 0.8-1.2, 1.0=normaal>, "pause_before": '
            "<getal 0.0-1.0 seconden>}}\n\n"
            f"Gedetecteerde objecten: {labels}\n"
            f"Scene-beschrijving: {description}"
        )
        t0 = time.time()
        try:
            resp = requests.post(
                OLLAMA_VISION_URL,
                json={"model": VISION_MODEL, "prompt": prompt, "format": "json", "stream": False},
                timeout=90,
            )
            resp.raise_for_status()
            payload = resp.json()
            # 20 sep 2026 (zelfde bugfix als VisionDescriber, hier live
            # gevonden bij deze format="json"-aanroep): val terug op
            # 'thinking' als 'response' leeg is.
            raw = payload.get("response") or payload.get("thinking") or ""
            elapsed_ms = round((time.time() - t0) * 1000)
            try:
                decision = json.loads(raw)
            except json.JSONDecodeError:
                self._logger.warn(f"[{elapsed_ms}ms] LLM-beslissing was geen geldige JSON: {raw!r}")
                return
            self._logger.info(f"[{elapsed_ms}ms] LLM-beslissing: {decision}")
            self._world_update.push("llm_decision", decision)

            skill = decision.get("skill")
            if skill not in self.ALLOWED_SKILLS:
                self._logger.warn(f"onbekende/niet-toegestane skill {skill!r} genegeerd (observe-only)")
            elif skill == "greet":
                self._execute_greet(decision, color_frame)
        except requests.RequestException as exc:
            self._logger.warn(f"doel-selectie-aanroep mislukt (Ollama niet bereikbaar?): {exc}")

    def _execute_greet(self, decision, color_frame):
        """De ENIGE skill die hier daadwerkelijk iets fysieks doet -- 2
        piepjes + gesproken herkenning + snapshot + notify_interaction,
        zelfde als wander_executor.py's al live-bevestigde patroon (dat
        stopte bij de piepjes; hier komt spraak erbij op uitdrukkelijk
        verzoek van de gebruiker, 20 sep 2026, na het live horen van de
        piepjes). Elke stap faalt stil op zichzelf (zelfde reden als de
        IPC-clients hierboven), zodat een mislukt onderdeel de andere niet
        blokkeert."""
        self._logger.info("SKILL UITGEVOERD: greet (LLM-beslissing)")
        if self._buzzer is not None:
            self._buzzer.beep_greet()
        if self._tts is not None:
            # 29 sep 2026: gebruik de door de LLM bedachte tekst+spreek-
            # parameters ("speech"-object, zie decide()) als die er geldig
            # is -- val terug op de oude vaste template bij ontbreken/
            # onzin (LLM-output is nooit vertrouwd zonder validatie, zelfde
            # reden als de skill-whitelist hierboven). Alle getallen
            # geclamped tegen een absurde waarde (bv. een dubbelzinnige
            # LLM-output met speed=5.0).
            speech = decision.get("speech") if isinstance(decision.get("speech"), dict) else {}
            text = speech.get("text")
            if not isinstance(text, str) or not text.strip():
                target = decision.get("target")
                text = f"Hello! I recognize a {target}." if target else "Hello!"
            text = text.strip()[:200]  # redelijke bovengrens, geen paginalange LLM-uitweiding voorlezen
            speed = speech.get("speed", 1.0)
            speed = max(0.7, min(1.4, speed)) if isinstance(speed, (int, float)) else 1.0
            pause_before = speech.get("pause_before", 0.0)
            pause_before = max(0.0, min(1.5, pause_before)) if isinstance(pause_before, (int, float)) else 0.0
            emotion = speech.get("emotion") if isinstance(speech.get("emotion"), str) else None
            self._logger.info(
                f"spreek (greet): {text!r} (speed={speed:.2f}, pause_before={pause_before:.2f}s, "
                f"emotion={emotion!r} -- nog niet gebruikt voor audio, Piper heeft geen emotiesturing)"
            )
            self._tts.say(text, length_scale=speed, pause_before_s=pause_before)
        if color_frame is not None:
            try:
                PILImage.fromarray(color_frame).save(GREET_SNAPSHOT_PATH)
            except OSError as exc:
                self._logger.warn(f"kon greet-snapshot niet opslaan: {exc}")
        if self._notify is not None:
            self._notify.notify_interaction()


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

    def say(self, text: str, length_scale: float = 1.0, pause_before_s: float = 0.0):
        # 29 sep 2026 (gebruikersidee, "spraakintentie" i.p.v. platte tekst,
        # zie het Ollama+Kokoro-architectuurvoorstel dat diezelfde dag
        # onderzocht is): length_scale stuurt Piper's eigen --length-scale
        # aan (1.0=normaal, hoger=langzamer -- Piper's eigen semantiek,
        # bewust NIET omgedraaid om verwarring te voorkomen). pause_before_s
        # is gewoon een stilte ervoor. Deze functie draait al in een eigen
        # achtergrondthread (VisionDescriber's thread, niet de hoofdcyclus),
        # dus een korte sleep() hier blokkeert niets belangrijks.
        if pause_before_s > 0:
            time.sleep(pause_before_s)
        try:
            piper = subprocess.Popen(
                [PIPER_BIN, "--model", PIPER_MODEL, "--length-scale", str(length_scale), "--output-raw"],
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
        self.depth_fx = None
        self.depth_cx = None
        self.buzzer = MutodBuzzer(self.get_logger())
        self.world_update = MutodWorldUpdate(self.get_logger())
        self.notify = MutodNotify(self.get_logger())
        self.tts = Tts(self.get_logger())
        self.goal_selector = GoalSelector(self.get_logger(), self.world_update, buzzer=self.buzzer, notify=self.notify, tts=self.tts)
        self.vision = VisionDescriber(self.get_logger(), self.world_update, self.goal_selector)
        self.previous_labels = set()

        self.create_subscription(Image, "/camera/color/image_raw", self._color_cb, qos_profile_sensor_data)
        self.create_subscription(Image, "/camera/depth/image_raw", self._depth_cb, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "/camera/depth/camera_info", self._depth_info_cb, 10)
        self.create_timer(self.interval_s, self._tick)
        self.get_logger().info(f"yolo_snapshot_sender gestart -- elke {self.interval_s:.0f}s een snapshot naar {JETSON_URL}")

    def _color_cb(self, msg):
        self.latest_color = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")

    def _depth_cb(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def _depth_info_cb(self, msg):
        if self.depth_fx is None and msg.k[0] > 0:
            self.depth_fx, self.depth_cx = msg.k[0], msg.k[2]

    def _push_depth_clearance(self):
        """20 sep 2026 (AI-executive-architectuur, "echte diepte aansluiten"):
        vervangt de nooit-bijgewerkte synthetische "depth_front_m"-testwaarde
        van stap 1. Hergebruikt depth_obstacle.py's estimate_clearance_by_
        heading() -- dezelfde, al live-bevestigde functie die
        wander_executor.py voor echte obstakelvermijding gebruikt -- i.p.v.
        een naieve center-pixel-meting: die bleek op 17 sep structureel
        onbruikbaar vlak voor de robot (Astra Pro Plus' 0.6m-minimumbereik +
        montagehoek = permanente dode zone op de vloer, zie
        muto_ball_depth_measurement_2026-09-17-memory). Deze heading-per-
        richting-vorm laat dat eerlijk zien: headings zonder geldige meting
        ontbreken gewoon in het resultaat i.p.v. een vals getal te tonen.
        Puur lokale berekening (geen netwerkaanroep behalve de push zelf),
        dus elke cyclus (niet event-driven zoals de LLM-aanroepen -- die
        beperking was specifiek voor dure Ollama-aanroepen, niet hiervoor)."""
        if self.latest_depth is None or self.depth_fx is None:
            return
        clearance = estimate_clearance_by_heading(
            self.latest_depth, self.depth_fx, self.depth_cx, heading_step_deg=15.0
        )
        self.world_update.push("depth_clearance", {
            str(heading): round(clear_m, 2) for heading, clear_m in clearance.items()
        })

    def _tick(self):
        if self.latest_color is None:
            self.get_logger().warn("nog geen kleurbeeld ontvangen, sla deze snapshot over")
            return

        self._push_depth_clearance()

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
        self.world_update.push("yolo", {"detections": detections, "elapsed_ms": elapsed_ms})

        if detections:
            summary = ", ".join(f"{d['label']}({d['distance_m']}m)" if d["distance_m"] else d["label"] for d in detections)
            self.get_logger().info(f"[{elapsed_ms}ms round-trip] gezien: {summary}")
        else:
            self.get_logger().info(f"[{elapsed_ms}ms round-trip] niets gedetecteerd")

        new_labels = current_labels - self.previous_labels
        if new_labels:
            self.vision.describe_async(self.latest_color, detections, new_labels)
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
