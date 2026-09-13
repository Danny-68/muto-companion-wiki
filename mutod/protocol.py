"""STM32-baseboard serieel protocol (0x55 .. 0xAA framing).

Adressen en frame-opbouw zijn geverifieerd tegen de echte robot en tegen
MutoLibCore.py (muto_hexapod_lib, in de humble_run container). Zie Fase 1
briefing (11 sep 2026), sectie 1.1-1.3, voor de meetgegevens.

LET OP runtime-byte-volgorde bij MOTOR (0x40): MutoLibCore verstuurt de
runtime als [hi, lo] (struct.pack('<h', runtime), dan value[1] voor value[0]),
dus HOGE byte eerst op de draad. Dit wijkt af van de kortere beschrijving in
de Fase-1-briefing zelf ("runtime_lo, runtime_hi") -- de library-code hier is
de geverifieerde bron, niet de briefing-tekst.
"""
import struct

HEAD = 0x55
DEVICE_ID = 0x00
WRITE_CMD = 0x01
READ_CMD = 0x02
DATA_RETURN = 0x12
TAIL = (0x00, 0xAA)

# Adrestabel (definitief, Fase 1 §1.2)
RESET = 0x06
BUZZER = 0x18  # timeout: 0=uit, 1-254=timeout*100ms auto-uit, 255=continu (bevestigd via MutoLibCore + Yahboom's "Control buzzer"-cursus, 13 sep 2026)
STAY_PUT = 0x11  # stopt de lopende gait, benen zetten neer -- gebruikt door de deadman
FORWARD = 0x12
BACKWARD = 0x13
SHIFT_LEFT = 0x14
SHIFT_RIGHT = 0x15
TURN_LEFT = 0x16
TURN_RIGHT = 0x17
TORQUE_ON = 0x26
TORQUE_OFF = 0x27
ACTION = 0x3E
MOTOR = 0x40
LEG = 0x41  # NOOIT gebruiken -- veroorzaakte ooit OSError: No such device. Altijd 3x MOTOR.
MOTOR_ANGLE = 0x50
ATTITUDE_ANGLE = 0x60
IMU_RAW = 0x61
EXEC_TIME = 0x2C  # servo-executietijd-register, gebruikt door phoenix_gait.py

# ACTION (0x3E)-preset-mapping, geverifieerd tegen MutoLibCore.py's action()-
# docstring (humble_run container, Fase 6, 12 sep 2026) -- ground truth, niet
# afgeleid uit de Fase-1-briefingtekst ("reset/stretch/greet/wave/spin/...").
ACTION_NAMES = {
    0: "reset",
    1: "stretch",
    2: "greet",
    3: "retreat",
    4: "warmup",
    5: "spin",
    6: "wave_no",
    7: "curl_up",
    8: "stride",
}

NUM_SERVOS = 18

# Adressen die via de raw_frames-doorgeefluik (mutod als relay voor
# phoenix_driver.py's eigen IK, zie mutod_client.py) toegestaan zijn. Bewust
# een korte allowlist -- dit kanaal geeft een extern proces geen vrije
# doorgang naar willekeurige adressen zoals RESET of TORQUE_OFF.
#
# LEG (0x41) staat er sinds 11 sep 2026 ook in: phoenix_gait.py's Leg.move_tip()
# (via de container-eigen MutoLib 1.2.2-egg, MutoLib/servo.py's
# set_angle_leg()) heeft ALTIJD al zo gewerkt -- 1 gecombineerd 3-servo-frame
# per poot, runtime=20ms, geen losse 0x40-writes (die weg is er in die
# library zelfs uitgecommentarieerd). Dit is dus geen nieuw gebruik dat mutod
# introduceert, alleen een reeds bestaand, eerder gevalideerd patroon dat nu
# via mutod loopt i.p.v. via een eigen seriele verbinding.
#
# LET OP: dit is een ANDER pad dan de OSError: No such device die Fase 1
# §1.2 aan 0x41 toeschrijft -- die kwam van MutoLibCore/muto_hexapod_lib's
# leg_motor() (een andere, oudere library-kopie), niet van dit pad. Blijf
# 0x41 dus wel vermijden bij nieuwe code die MutoLibCore/muto_hexapod_lib
# gebruikt; alleen phoenix_gait.py's eigen MutoLib-pad is hier expliciet
# toegestaan.
RAW_FRAME_ALLOWED_ADDRESSES = {MOTOR, EXEC_TIME, LEG}

GAIT_ADDRESSES = {
    "FORWARD": FORWARD,
    "BACKWARD": BACKWARD,
    "SHIFT_LEFT": SHIFT_LEFT,
    "SHIFT_RIGHT": SHIFT_RIGHT,
    "TURN_LEFT": TURN_LEFT,
    "TURN_RIGHT": TURN_RIGHT,
}

# Poot-volgorde (Fase 1 §1.3): drie servo's per poot, coxa/femur/tibia.
LEG_SERVO_IDS = {
    "RF": (1, 2, 3),
    "RM": (4, 5, 6),
    "RR": (7, 8, 9),
    "LR": (10, 11, 12),
    "LM": (13, 14, 15),
    "LF": (16, 17, 18),
}


def _checksum(length: int, mode: int, addr: int, data_sum: int) -> int:
    return (255 - ((length + mode + addr + data_sum) % 256)) & 0xFF


def build_write_frame(addr: int, data: list) -> bytes:
    """Bouwt een write-commando (Pi -> STM32)."""
    length = len(data) + 0x08
    checksum = _checksum(length, WRITE_CMD, addr, sum(data))
    frame = [HEAD, DEVICE_ID, length, WRITE_CMD, addr, *data, checksum, *TAIL]
    return bytes(frame)


def build_read_frame(addr: int, count: int) -> bytes:
    """Bouwt een read-verzoek (Pi -> STM32); count = aantal databytes dat terug moet komen."""
    length = 0x09
    checksum = _checksum(length, READ_CMD, addr, count)
    frame = [HEAD, DEVICE_ID, length, READ_CMD, addr, count, checksum, *TAIL]
    return bytes(frame)


def response_size(count: int) -> int:
    """Totale framegrootte (bytes) van een read-antwoord met `count` databytes."""
    return count + 8  # HEAD DEVICE_ID LEN TYPE ADDR + count*data + CHECKSUM 0x00 0xAA


def parse_response(raw: bytes, expect_addr: int, expect_count: int):
    """Valideert en ontleedt een compleet read-antwoord.

    Geeft de lijst met ruwe databytes terug, of None als het antwoord
    incompleet, verminkt of voor een ander adres is.
    """
    if len(raw) != response_size(expect_count):
        return None
    if raw[0] != HEAD or raw[1] != DEVICE_ID:
        return None
    length = raw[2]
    frame_type = raw[3]
    addr = raw[4]
    if frame_type != DATA_RETURN or addr != expect_addr:
        return None
    if length != expect_count + 8:
        return None
    data = raw[5:5 + expect_count]
    checksum = raw[5 + expect_count]
    tail = raw[5 + expect_count + 1:5 + expect_count + 3]
    if tail != bytes(TAIL):
        return None
    if checksum != _checksum(length, frame_type, addr, sum(data)):
        return None
    return list(data)


def parse_write_frame(raw: bytes):
    """Valideert een write-frame zoals build_write_frame() produceert (en zoals
    MutoLib.Servo.motor()/phoenix_gait.set_exec_time() ze zelf bouwen).

    Geeft (addr, data) terug, of None als het frame verminkt is. Gebruikt
    door de raw_frames-doorgeefluik om binnenkomende, elders al opgebouwde
    frames te controleren voor ze naar de STM32 gaan.
    """
    if len(raw) < 9:
        return None
    if raw[0] != HEAD or raw[1] != DEVICE_ID:
        return None
    length = raw[2]
    mode = raw[3]
    addr = raw[4]
    if mode != WRITE_CMD:
        return None
    data_count = length - 8
    if data_count < 0 or len(raw) != data_count + 8:
        return None
    data = raw[5:5 + data_count]
    checksum = raw[5 + data_count]
    tail = raw[5 + data_count + 1:5 + data_count + 3]
    if tail != bytes(TAIL):
        return None
    if checksum != _checksum(length, mode, addr, sum(data)):
        return None
    return addr, list(data)


def motor_runtime_bytes(runtime_ms: int) -> tuple:
    """Runtime als [hi, lo] zoals MutoLibCore die op de draad zet."""
    packed = struct.pack('<h', int(runtime_ms))
    lo, hi = packed[0], packed[1]
    return hi, lo
