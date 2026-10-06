#!/usr/bin/env python3
"""voice_stop_listener.py -- 29 sep 2026 (gebruikersidee: "kan je de
microfoons laten luisten zodat muto een gesproken reactie kan geven of
zelf iets kan doen, bv stoppen"). Luistert continu in korte stukken op
een attentiewoord ("muto") -- reageert daarop met een gesproken "Yes.",
en voert daarna een herkend commando ("stop") echt uit via robot.stop
op mutod's IPC.

Draait op de host (geen ROS/rclpy nodig, net als mutod zelf) -- pure
microfoon-opname (arecord) + lokale Whisper-inferentie (pywhispercpp,
zie /home/pi/whisper_venv) + een TCP-aanroep naar mutod. Geen Jetson
nodig, dat is een apart, ongerelateerd pad (YOLO-detectie).

BEWUST GEEN vervanging voor de fysieke e-stop (zie
muto_estop_stm32_latching-memory: de software-laag stoppen garandeert
niet dat de fysieke beweging stopt) -- dit is een extra, redundante
stem-commando bovenop de bestaande veiligheidslagen (deadman, robot.stop
IPC), niet de enige stopmethode.

Live getest (29 sep 2026): microfoon + Whisper (base.en) herkennen
herhaald "stop" correct in een rustige ruimte -- achtergrondgeluid
(bv. een tv) kan valse/onzin-transcripties geven, vandaar de stilte-
check hieronder VOOR whisper wordt aangeroepen (bespaart CPU, en
whisper hallucineert soms op pure ruis/stilte).
"""
import json
import socket
import statistics
import struct
import subprocess
import sys
import time
import wave

from pywhispercpp.model import Model

MUTOD_HOST, MUTOD_PORT = "127.0.0.1", 8420
MIC_DEVICE = "plughw:CARD=Device,DEV=0"  # stabiele kaart-NAAM, niet -nummer, zie muto_investigate_pause_and_stable_audio-memory
CHUNK_S = 3.0
SAMPLE_RATE = 16000
CHUNK_PATH = "/tmp/voice_stop_chunk.wav"
STOP_KEYWORDS = ("stop",)
MIN_SIGNAL_STDEV = 150.0  # onder dit niveau: waarschijnlijk stilte, whisper overslaan
# 29 sep 2026: Piper draait ook op de host (niet alleen in humble_run,
# bevestigd gecheckt) -- zelfde stemmodel/patroon als wander_executor.py/
# yolo_snapshot_sender.py, hier los gedupliceerd (eigen, ander proces).
PIPER_BIN = "/home/pi/piper_venv/bin/piper"
PIPER_MODEL = "/home/pi/piper_voices/en_US-amy-medium.onnx"
PIPER_SAMPLE_RATE = 22050
# 29 sep 2026 (gebruikersidee): een attentiewoord voorkomt dat een
# toevallig genoemd "stop" (gesprek, tv, radio) het commando triggert --
# moet eerst "muto" gehoord worden, dan pas telt een commandowoord. Twee
# paden: (1) allebei in DEZELFDE 3s-opname ("Muto, stop"), of (2)
# "muto" in de ene opname, commando in een van de eerstvolgende
# ARM_WINDOW_S seconden (voor als er een korte stilte tussen zit, wat bij
# 3s-brokken makkelijk over twee opnames heen kan vallen).
ATTENTION_WORD = "muto"
# 29 sep 2026 (bugfix, live gevonden): een volledige luistercyclus (3s
# opname + whisper-verwerking) duurt in de praktijk ~7-8s op deze Pi 5 --
# de oorspronkelijke 6s was dus KORTER dan 1 cyclus, waardoor "muto" in
# de ene opname en het commando in de volgende bijna altijd te laat kwam
# (bevestigd: "yes!" -- mutod's eigen antwoord-echo -- kwam 7.8s na
# "muto" binnen, het venster was toen al dicht). Ruim verhoogd zodat er
# minstens 2 volledige cycli in passen.
ARM_WINDOW_S = 20.0


def record_chunk(path: str, duration_s: float):
    # 29 sep 2026 (bugfix, live gevonden): arecord's -d verwacht een
    # GEHEEL aantal seconden -- "3.0" werd afgewezen (exit 1), vandaar
    # de int()-afronding hier i.p.v. de rauwe CHUNK_S-float doorgeven.
    subprocess.run(
        ["arecord", "-D", MIC_DEVICE, "-f", "S16_LE", "-r", str(SAMPLE_RATE),
         "-d", str(int(round(duration_s))), "-q", path],
        check=True,
    )


def has_signal(path: str, min_stdev: float = MIN_SIGNAL_STDEV) -> bool:
    with wave.open(path, "rb") as w:
        frames = w.readframes(w.getnframes())
    if not frames:
        return False
    samples = struct.unpack(f"<{len(frames)//2}h", frames)
    return statistics.pstdev(samples) >= min_stdev


def speak(text: str):
    # fire-and-forget, zelfde patroon als elders -- mag de luister-lus
    # nooit blokkeren.
    try:
        piper = subprocess.Popen(
            [PIPER_BIN, "--model", PIPER_MODEL, "--output-raw"],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
        )
        subprocess.Popen(
            ["aplay", "-r", str(PIPER_SAMPLE_RATE), "-f", "S16_LE", "-t", "raw", "-D", MIC_DEVICE],
            stdin=piper.stdout,
        )
        piper.stdout.close()
        piper.stdin.write(text.encode())
        piper.stdin.close()
    except OSError as exc:
        print(f"kon niet spreken (piper/aplay niet gevonden?): {exc}", flush=True)


def call_robot_stop() -> str:
    try:
        s = socket.create_connection((MUTOD_HOST, MUTOD_PORT), timeout=2)
        req = {"jsonrpc": "2.0", "id": 1, "method": "robot.stop", "params": {}}
        s.sendall((json.dumps(req) + "\n").encode())
        resp = s.recv(512)
        s.close()
        return resp.decode(errors="replace").strip()
    except OSError as exc:
        return f"FOUT (mutod niet bereikbaar?): {exc}"


def main():
    print("voice_stop_listener: model laden (base.en)...", flush=True)
    model = Model("base.en")
    print(
        f"voice_stop_listener gestart -- luistert in stukken van {CHUNK_S:.0f}s, "
        f"wacht op \"{ATTENTION_WORD}\" gevolgd door een commando",
        flush=True,
    )

    armed_until = 0.0  # 0.0 = niet gewapend (attentiewoord nog niet recent gehoord)

    while True:
        t0 = time.monotonic()
        record_chunk(CHUNK_PATH, CHUNK_S)
        if not has_signal(CHUNK_PATH):
            continue
        segments = model.transcribe(CHUNK_PATH)
        text = " ".join(s.text for s in segments).strip().lower()
        elapsed = time.monotonic() - t0
        if not text:
            continue
        print(f"[{elapsed:.1f}s] gehoord: {text!r}", flush=True)

        heard_attention = ATTENTION_WORD in text
        now = time.monotonic()
        is_armed = heard_attention or now < armed_until
        has_command = any(kw in text for kw in STOP_KEYWORDS)
        print(f"  [debug] now={now:.1f} armed_until={armed_until:.1f} heard_attention={heard_attention} is_armed={is_armed} has_command={has_command}", flush=True)

        if heard_attention:
            armed_until = time.monotonic() + ARM_WINDOW_S
            print(f"\"{ATTENTION_WORD}\" gehoord -- antwoord en luistert nu {ARM_WINDOW_S:.0f}s naar een commando", flush=True)
            speak("Yes.")

        if is_armed and has_command:
            armed_until = 0.0  # meteen weer ontwapenen -- 1 commando per keer "muto" zeggen
            print(f"STOP-COMMANDO HERKEND ({text!r}) -> robot.stop", flush=True)
            result = call_robot_stop()
            print(f"robot.stop -> {result}", flush=True)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
