"""MutoHAL -- dunne, thread-safe laag boven het STM32-serieel protocol.

Enige plek die daadwerkelijk naar de seriele poort schrijft/leest. Alle
hogere lagen (daemon, ipc) gaan hierdoorheen.
"""
import fcntl
import struct
import termios
import threading
import time

import serial

from . import protocol, safety


class MutoHALError(Exception):
    pass


class MutoHAL:
    def __init__(self, port: str = "/dev/myserial", baud: int = 115200, timeout: float = 0.2):
        self._ser = serial.Serial(port, baud, timeout=timeout)
        # TIOCEXCL: maakt "exclusief" ook waar na het openen -- andere
        # processen die deze tty proberen te openen (bv. yahboom_oled.py's
        # serial-fallback als /tmp/battery_pct ontbreekt) krijgen vanaf nu
        # een schone EBUSY in plaats van dat ze frames kunnen laten
        # interleaven met mutod's eigen verkeer.
        try:
            fcntl.ioctl(self._ser.fileno(), termios.TIOCEXCL)
        except OSError:
            pass
        self._lock = threading.Lock()

    def close(self):
        self._ser.close()

    def drain_stale(self, settle_s: float = 0.3):
        """Leegt bytes die de STM32 nog naar de vorige eigenaar van de poort
        onderweg had (bv. een BATTERY-antwoord op een request van app_muto.py
        vlak voor het afsluiten). Roep dit eenmalig aan bij het opstarten,
        voordat de control-loop begint te lezen -- anders kan de eerste
        read_all_angles() een oud, niet-passend antwoord oppikken."""
        with self._lock:
            deadline = time.monotonic() + settle_s
            while time.monotonic() < deadline:
                if self._ser.in_waiting:
                    self._ser.read(self._ser.in_waiting)
                    deadline = time.monotonic() + 0.1
                else:
                    time.sleep(0.02)
            self._ser.reset_input_buffer()

    # -- laag-niveau ----------------------------------------------------

    def _write_frame(self, addr: int, data: list):
        with self._lock:
            self._ser.write(protocol.build_write_frame(addr, data))

    def _read_frame(self, addr: int, count: int):
        with self._lock:
            self._ser.reset_input_buffer()
            self._ser.write(protocol.build_read_frame(addr, count))
            raw = self._ser.read(protocol.response_size(count))
        return protocol.parse_response(raw, addr, count)

    # -- servo's ----------------------------------------------------------

    def read_all_angles(self) -> dict:
        """Bulk-read (0x50): alle 18 servo-hoeken in graden, signed. ~27ms."""
        data = self._read_frame(protocol.MOTOR_ANGLE, protocol.NUM_SERVOS)
        if data is None:
            raise MutoHALError("read_all_angles: geen of ongeldig antwoord van STM32")
        return {
            servo_id: struct.unpack('b', bytes([raw_byte]))[0]
            for servo_id, raw_byte in enumerate(data, start=1)
        }

    def write_servo(self, servo_id: int, angle_deg: int, runtime_ms: int = 100):
        """Een servo naar een hoek (0x40). Enige manier om joints aan te sturen --
        0x41 (LEG) wordt hier bewust nooit gebruikt, zie protocol.LEG."""
        servo_id = safety.validate_servo_id(servo_id)
        angle_deg = safety.validate_angle_deg(angle_deg)
        runtime_ms = safety.validate_runtime_ms(runtime_ms)
        hi, lo = protocol.motor_runtime_bytes(runtime_ms)
        self._write_frame(protocol.MOTOR, [servo_id, angle_deg & 0xFF, hi, lo])

    def write_leg(self, leg_name: str, angles, runtime_ms: int = 100):
        """Drie servo's van een poot, als 3x write_servo (nooit 0x41)."""
        if leg_name not in protocol.LEG_SERVO_IDS:
            raise ValueError(f"onbekende poot: {leg_name!r}, verwacht een van {list(protocol.LEG_SERVO_IDS)}")
        if len(angles) != 3:
            raise ValueError("angles moet [coxa, femur, tibia] zijn (3 waarden)")
        for servo_id, angle in zip(protocol.LEG_SERVO_IDS[leg_name], angles):
            self.write_servo(servo_id, angle, runtime_ms)

    def torque(self, on: bool, servo_id: int = 0):
        """servo_id=0 -> alle servo's."""
        if servo_id != 0:
            servo_id = safety.validate_servo_id(servo_id)
        addr = protocol.TORQUE_ON if on else protocol.TORQUE_OFF
        wire_id = 0xFE if servo_id == 0 else servo_id
        self._write_frame(addr, [wire_id])

    def reset_posture(self):
        self._write_frame(protocol.RESET, [0])

    def buzzer(self, timeout: int):
        """0x18 (BUZZER): 0=uit, 1-254=timeout*100ms auto-uit, 255=continu.
        Bevestigd via MutoLibCore.buzzer() + Yahboom's "Control buzzer"-cursus
        (13 sep 2026) -- omzeilt de kapotte USB-C-Media-speaker volledig,
        dit gaat rechtstreeks via het STM32-board."""
        timeout = int(timeout)
        if not (0 <= timeout <= 255):
            raise ValueError(f"buzzer-timeout buiten bereik 0-255: {timeout}")
        self._write_frame(protocol.BUZZER, [timeout])

    def action(self, action_id: int):
        """0x3E (ACTION): een van 9 fabrieks-preset-routines, 0-8. Mapping
        geverifieerd tegen MutoLibCore.py's action()-docstring (Fase 6, 12 sep
        2026): 0=reset, 1=stretch, 2=greet/hello, 3=fear/retreat, 4=warm-up,
        5=spin-in-place, 6=wave/"no", 7=curl-up, 8=stride-forward. Fysieke
        uitvoering per ID is tot dusver NIET live getest -- zie Fase 6."""
        action_id = int(action_id)
        if not (0 <= action_id <= 8):
            raise ValueError(f"action_id buiten bereik 0-8: {action_id}")
        self._write_frame(protocol.ACTION, [action_id])

    def stay_put(self):
        """Stopt de lopende gait, benen zetten neer. Gebruikt door de deadman."""
        self._write_frame(protocol.STAY_PUT, [0])

    def gait(self, direction: str, step: int = 10):
        """direction: FORWARD/BACKWARD/SHIFT_LEFT/SHIFT_RIGHT/TURN_LEFT/TURN_RIGHT."""
        if direction not in protocol.GAIT_ADDRESSES:
            raise ValueError(f"onbekende gait-richting: {direction!r}")
        step = max(10, min(25, int(step)))
        self._write_frame(protocol.GAIT_ADDRESSES[direction], [step])

    def read_attitude(self) -> dict:
        """IMU-orientatie (0x60): roll/pitch/yaw in graden + temp. Negeert de
        ID-parameter van de STM32 (niet servo-gerelateerd), zie Fase 1 §1.2."""
        data = self._read_frame(protocol.ATTITUDE_ANGLE, 7)
        if data is None:
            raise MutoHALError("read_attitude: geen of ongeldig antwoord van STM32")
        raw = bytes(data)
        roll = struct.unpack('>h', raw[0:2])[0] / 100.0
        pitch = struct.unpack('>h', raw[2:4])[0] / 100.0
        yaw = struct.unpack('>h', raw[4:6])[0] / 100.0
        temp = raw[6]
        return {"roll_deg": roll, "pitch_deg": pitch, "yaw_deg": yaw, "temp_c": temp}

    def write_raw_frames(self, frames: list, inter_frame_delay_s: float = 0.001):
        """Schrijft vooraf opgebouwde, al gevalideerde write-frames rechtstreeks
        naar de STM32, met dezelfde 1ms-tussenpauze als MutoLib.Servo.motor().
        Gebruikt door de raw_frames-socket-op (phoenix_driver.py's eigen IK,
        via mutod_client.py) -- de aanroeper (daemon._handle_command) heeft elk
        frame al gevalideerd tegen protocol.RAW_FRAME_ALLOWED_ADDRESSES."""
        with self._lock:
            for frame in frames:
                self._ser.write(frame)
                if inter_frame_delay_s > 0:
                    time.sleep(inter_frame_delay_s)
