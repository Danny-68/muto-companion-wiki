#!/usr/bin/env python3
"""voice_stop_listener.py -- 29 sep 2026 (gebruikersidee: "kan je de
microfoons laten luisten zodat muto een gesproken reactie kan geven of
zelf iets kan doen, bv stoppen"). Herschreven 6 okt 2026 (v2).

Luistert CONTINU op een attentiewoord ("muto") -- reageert daarop met een
gesproken "Yes.", en voert daarna een herkend commando ("stop") echt uit
via robot.stop op mutod's IPC.

Draait op de host (geen ROS/rclpy nodig, net als mutod zelf) -- microfoon
(arecord) + lokale Whisper-inferentie (pywhispercpp, zie
/home/pi/whisper_venv) + een TCP-aanroep naar mutod. Geen Jetson nodig.

BEWUST GEEN vervanging voor de fysieke e-stop (zie
muto_estop_stm32_latching-memory: de software-laag stoppen garandeert
niet dat de fysieke beweging stopt) -- dit is een extra, redundante
stem-commando bovenop de bestaande veiligheidslagen, niet de enige
stopmethode.

Waarom v2: v1 nam vaste blokken van 3s op en transcribeerde daarna ~5s
(de microfoon was die tijd DOOF) -- een "stop" in die dove 63% van de
cyclus werd nooit gehoord, ook al klopte de logica (6 okt 2026 end-to-end
bevestigd). v2 neemt onafgebroken op, knipt uitspraken af met Silero
VAD (pysilero-vad, geinstalleerd in whisper_venv; eerste v2-test op 6 okt
slaagde met een eenvoudige energiedetector, daarna vervangen) en
transcribeert in een aparte thread terwijl de opname doorloopt.

Eigen spraak ("Yes.") wordt NIET opnieuw gehoord: tijdens het afspelen is
de microfoon gedempt (de echo clipt de mic op volle schaal, gemeten 6 okt
2026 -- Muto's speaker zit vlak naast de mic).
"""
import argparse
import fcntl
import json
import os
import queue
import re
import socket
import subprocess
import sys
import threading
import time
import wave
from collections import deque

import numpy as np
from pysilero_vad import SileroVoiceActivityDetector
from pywhispercpp.model import Model

MUTOD_HOST, MUTOD_PORT = "127.0.0.1", 8420
MIC_DEVICE = "plughw:CARD=Device,DEV=0"  # stabiele kaart-NAAM, niet -nummer, zie muto_investigate_pause_and_stable_audio-memory
SAMPLE_RATE = 16000
FRAME_SAMPLES = 512                      # 32 ms
FRAME_BYTES = FRAME_SAMPLES * 2
LOCK_PATH = "/tmp/voice_stop_listener.lock"
DEBUG_SEG_DIR = "/tmp/voice_segments"      # alleen met --debug: elke uitspraak als wav bewaard om achteraf te kunnen beluisteren

ATTENTION_WORD = "muto"                  # alleen voor logregels; de echte check is ATTENTION_RE
ATTENTION_RE = re.compile(r"\b(muto|mudo)\b")   # strikt: een losse fuzzy-match gaf 5-6/21 vals-alarmen ("mute", "mutual", "motor")
STOP_RE = re.compile(r"\bstop\b")

# --- Whisper-versnelling (gemeten 6 okt 2026 op 23 opnames: loop via speaker + echte stem) ---
# Whisper.cpp verwerkt standaard ALTIJD een venster van 30 s (~5 s rekentijd, ook voor een uitspraak van 1 s).
# Een gevuld venster van 15 s (stilte erachter + audio_ctx afgestemd) kost ~2,3 s en bleef 10/10 "muto", 11/11 "stop", 0/21 vals-alarm.
# Kortere vensters (6/10 s) en tiny.en waren sneller maar onbetrouwbaar (muto 1-4/10, "stop" -> "suck"/"sot"); audio_ctx zonder
# opvullen liet base.en hallucineren. De beginprompt tilt "muto" (geen Engels woord: Whisper hoort "mito"/"nuto") van 4/10 naar 10/10.
WHISPER_MODEL = "base.en"
WHISPER_PAD_S = 15.0
WHISPER_PROMPT = "Muto. Muto, stop. Stop."
WHISPER_MAX_TOKENS = 16                  # begrenst hallucinatielussen ("you know. you know. ...")
ARM_WINDOW_S = 20.0                      # gemeten vanaf het EINDE van de "muto"-uitspraak, niet vanaf de transcriptie

# --- stemdetectie: Silero VAD v6.2 (pysilero-vad, ggml-model zit in het pakket, ~0.2 ms per 32 ms-frame) ---
# Gemeten 6 okt 2026: kamerruis max kans 0.06; echo van Muto's eigen "Yes." 0.69-0.97. De mic-ruisvloer
# is ~180 stdev, dus een vaste energiedrempel was onbetrouwbaar -- Silero kijkt naar spraak, niet naar volume.
VAD_START_PROB = 0.5                     # frame telt als spraak boven deze kans (om te starten)
VAD_CONTINUE_PROB = 0.35                 # binnen een uitspraak: lagere drempel (hysterese, geen tussentijds afkappen)
START_VOICED_FRAMES = 3                  # ~100 ms aaneengesloten om een uitspraak te starten
END_SILENCE_FRAMES = 19                  # ~600 ms stilte beeindigt de uitspraak
MAX_SEG_FRAMES = 250                     # ~8 s: langer wordt afgekapt
PRE_ROLL_FRAMES = 7                      # ~220 ms voor de start meenemen (eerste klank niet missen)
MIN_VOICED_FRAMES = 8                    # ~250 ms spraak minimaal, anders was het een klik/tik

# --- eigen spraak (Piper) ---
PIPER_BIN = "/home/pi/piper_venv/bin/piper"
PIPER_MODEL = "/home/pi/piper_voices/en_US-amy-medium.onnx"
PIPER_SAMPLE_RATE = 22050
POST_SPEECH_MUTE_S = 0.4                 # nagalm na het afspelen

NON_SPEECH_TAG = re.compile(r"^\s*[\(\[\*].*[\)\]\*]\s*$")  # whisper-hallucinaties op ruis: "(applause)", "[music]"


def log(msg: str):
    print(msg, flush=True)


class Listener:
    def __init__(self, debug: bool):
        self.debug = debug
        self.mute_until = 0.0            # tot dan worden mic-frames genegeerd (eigen spraak)
        self.speak_lock = threading.Lock()
        self.segments: "queue.Queue[tuple[float, bytes]]" = queue.Queue()
        self.armed_until = 0.0

    # ---------------- eigen spraak ----------------
    def speak(self, text: str):
        threading.Thread(target=self._speak_blocking, args=(text,), daemon=True).start()

    def _speak_blocking(self, text: str):
        with self.speak_lock:
            self.mute_until = float("inf")
            try:
                piper = subprocess.Popen(
                    [PIPER_BIN, "--model", PIPER_MODEL, "--output-raw"],
                    stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                )
                aplay = subprocess.Popen(
                    ["aplay", "-q", "-r", str(PIPER_SAMPLE_RATE), "-f", "S16_LE", "-t", "raw", "-D", MIC_DEVICE],
                    stdin=piper.stdout, stderr=subprocess.DEVNULL,
                )
                piper.stdout.close()
                piper.stdin.write(text.encode())
                piper.stdin.close()
                aplay.wait()
                piper.wait()
            except OSError as exc:
                log(f"kon niet spreken (piper/aplay niet gevonden?): {exc}")
            finally:
                self.mute_until = time.monotonic() + POST_SPEECH_MUTE_S
                if self.debug:
                    log(f"  [debug] spreken klaar: {text!r}")

    # ---------------- opname + stemdetectie (thread 1) ----------------
    def capture_loop(self):
        while True:
            try:
                self._capture_once()
            except Exception as exc:  # arecord weg (USB-glitch, apparaat bezet): opnieuw proberen
                log(f"opname-fout: {exc} -- opnieuw proberen over 2s")
            time.sleep(2.0)

    def _capture_once(self):
        proc = subprocess.Popen(
            ["arecord", "-D", MIC_DEVICE, "-f", "S16_LE", "-r", str(SAMPLE_RATE), "-c", "1", "-t", "raw", "-q"],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        )
        vad = SileroVoiceActivityDetector()
        pre_roll: deque = deque(maxlen=PRE_ROLL_FRAMES)
        in_speech = False
        voiced_run = 0
        seg: list = []
        voiced_count = 0
        silence_run = 0
        try:
            while True:
                buf = proc.stdout.read(FRAME_BYTES)
                if len(buf) < FRAME_BYTES:
                    err = proc.stderr.read().decode(errors="replace").strip()
                    raise RuntimeError(f"arecord gestopt ({err or 'geen foutmelding'})")
                if time.monotonic() < self.mute_until:  # eigen spraak: niets verwerken, lopende uitspraak weggooien
                    in_speech, voiced_run, seg, voiced_count, silence_run = False, 0, [], 0, 0
                    pre_roll.clear()
                    vad.reset()
                    continue
                prob = vad.process_chunk(buf)
                is_voice = prob > (VAD_CONTINUE_PROB if in_speech else VAD_START_PROB)

                if not in_speech:
                    pre_roll.append(buf)
                    voiced_run = voiced_run + 1 if is_voice else 0
                    if voiced_run >= START_VOICED_FRAMES:
                        in_speech = True
                        seg = list(pre_roll)
                        voiced_count = voiced_run
                        silence_run = 0
                else:
                    seg.append(buf)
                    if is_voice:
                        voiced_count += 1
                        silence_run = 0
                    else:
                        silence_run += 1
                    if silence_run >= END_SILENCE_FRAMES or len(seg) >= MAX_SEG_FRAMES:
                        if voiced_count >= MIN_VOICED_FRAMES:
                            self.segments.put((time.monotonic(), b"".join(seg)))
                        elif self.debug:
                            log(f"  [debug] te korte uitspraak genegeerd ({voiced_count} stemframes)")
                        in_speech, voiced_run, seg, voiced_count, silence_run = False, 0, [], 0, 0
                        pre_roll.clear()
                        vad.reset()
        finally:
            proc.kill()

    # ---------------- transcriptie + beslissing (thread 2 = hoofdthread) ----------------
    def transcribe(self, model, pcm: bytes) -> str:
        x = np.frombuffer(pcm, dtype=np.int16).astype(np.float32) / 32768.0
        pad_s = max(WHISPER_PAD_S, len(x) / SAMPLE_RATE + 0.5)
        x = np.concatenate([x, np.zeros(int(pad_s * SAMPLE_RATE) - len(x), dtype=np.float32)])
        segs = model.transcribe(
            x, no_context=True, single_segment=True, max_tokens=WHISPER_MAX_TOKENS,
            initial_prompt=WHISPER_PROMPT, audio_ctx=min(1500, int(pad_s * 50)),
        )
        return " ".join(sg.text for sg in segs).strip().lower()

    def process_loop(self, model):
        model.transcribe(np.zeros(SAMPLE_RATE, dtype=np.float32), audio_ctx=int(WHISPER_PAD_S * 50))  # warm-up, eerste aanroep is trager
        while True:
            end_ts, pcm = self.segments.get()
            if self.debug:
                os.makedirs(DEBUG_SEG_DIR, exist_ok=True)
                with wave.open(f"{DEBUG_SEG_DIR}/seg_{time.strftime('%H%M%S')}_{int(end_ts*1000) % 100000}.wav", "wb") as w:
                    w.setnchannels(1)
                    w.setsampwidth(2)
                    w.setframerate(SAMPLE_RATE)
                    w.writeframes(pcm)
            t0 = time.monotonic()
            text = self.transcribe(model, pcm)
            took = time.monotonic() - t0
            if not text or NON_SPEECH_TAG.match(text):
                if self.debug:
                    log(f"  [debug] genegeerd ({len(pcm)/2/SAMPLE_RATE:.1f}s audio, {took:.1f}s whisper): {text!r}")
                continue
            log(f"[{len(pcm)/2/SAMPLE_RATE:.1f}s audio, {took:.1f}s whisper, backlog {self.segments.qsize()}] gehoord: {text!r}")
            self.decide(end_ts, text)

    def decide(self, end_ts: float, text: str):
        heard_attention = bool(ATTENTION_RE.search(text))
        has_command = bool(STOP_RE.search(text))
        armed = heard_attention or end_ts < self.armed_until
        if self.debug:
            log(f"  [debug] end_ts={end_ts:.1f} armed_until={self.armed_until:.1f} "
                f"heard_attention={heard_attention} armed={armed} has_command={has_command}")

        if heard_attention:
            self.armed_until = end_ts + ARM_WINDOW_S
            log(f'"{ATTENTION_WORD}" gehoord -- antwoord en luistert nu {ARM_WINDOW_S:.0f}s naar een commando')
            self.speak("Yes.")

        if armed and has_command:
            self.armed_until = 0.0  # 1 commando per keer "muto"
            log(f"STOP-COMMANDO HERKEND ({text!r}) -> robot.stop")
            ok, detail = call_robot_stop()
            log(f"robot.stop -> {detail}")
            if not ok:
                # stille mislukking is gevaarlijk: de gebruiker denkt dat de robot stopt
                log("WAARSCHUWING: stop NIET bevestigd door mutod")
                self.speak("I could not stop. My controller is not responding.")


def call_robot_stop():
    """(ok, ruwe_tekst). ok is alleen True als mutod expliciet accepted:true teruggeeft."""
    try:
        s = socket.create_connection((MUTOD_HOST, MUTOD_PORT), timeout=2)
        req = {"jsonrpc": "2.0", "id": 1, "method": "robot.stop", "params": {}}
        s.sendall((json.dumps(req) + "\n").encode())
        resp = s.recv(512).decode(errors="replace").strip()
        s.close()
        try:
            ok = bool(json.loads(resp).get("result", {}).get("accepted"))
        except ValueError:
            ok = False
        return ok, resp
    except OSError as exc:
        return False, f"FOUT (mutod niet bereikbaar?): {exc}"


def acquire_single_instance_lock():
    # 6 okt 2026: twee listeners tegelijk kunnen niet van dezelfde mic opnemen (tweede crasht) en
    # vervuilen elkaars tests -- een tweede instantie weigert te starten.
    f = open(LOCK_PATH, "w")
    try:
        fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        sys.exit("voice_stop_listener draait al (lock %s) -- tweede instantie geweigerd" % LOCK_PATH)
    return f


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--debug", action="store_true", help="toon ook genegeerde uitspraken en beslis-details")
    args = ap.parse_args()
    _lock = acquire_single_instance_lock()  # noqa: F841 (moet leven tot afsluiten)

    log(f"voice_stop_listener v2: model laden ({WHISPER_MODEL})...")
    model = Model(WHISPER_MODEL)
    listener = Listener(args.debug)
    threading.Thread(target=listener.capture_loop, daemon=True).start()
    log(f'voice_stop_listener v2 gestart -- luistert continu, wacht op "{ATTENTION_WORD}" gevolgd door een commando')
    listener.process_loop(model)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
