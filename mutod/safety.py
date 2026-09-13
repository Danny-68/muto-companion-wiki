"""Deadman-timer en invoer-validatie voor mutod.

Torque wordt door de deadman nooit uitgeschakeld (dat zou de robot laten
instorten) -- alleen het versturen van nieuwe bewegingscommando's stopt.
Zie Fase 1 briefing §2 (architectuurbeslissingen) en Fase 2 deliverables.
"""
import math
import threading
import time


class Deadman:
    """touch() wordt aangeroepen vanuit socket-handler-threads (een per
    verbinding), trip_once() vanuit de control-loop-thread. Zonder lock kan
    een touch() net tussen trip_once()'s expired()-check en het zetten van
    _tripped in glippen: trip_once() zou dan nog True teruggeven vlak na een
    geldige touch() -- een overbodige extra stay_put() op een net actief
    commando. Onschuldig (stay_put() op een net bewegende robot is geen
    veiligheidsprobleem), maar wel een echt races-gat; hier dichtgezet."""

    def __init__(self, timeout_s: float = 0.6):
        self.timeout_s = timeout_s
        self._lock = threading.Lock()
        self._last_intent = time.monotonic()
        self._tripped = False

    def touch(self):
        """Aanroepen bij elk ontvangen bewegingscommando."""
        with self._lock:
            self._last_intent = time.monotonic()
            self._tripped = False

    def expired(self) -> bool:
        with self._lock:
            return (time.monotonic() - self._last_intent) > self.timeout_s

    def trip_once(self) -> bool:
        """True de eerste keer dat expired() waar wordt na de laatste touch();
        daarna False tot de volgende touch(). Voorkomt dat de stop-actie elke
        tick opnieuw wordt afgevuurd."""
        with self._lock:
            expired = (time.monotonic() - self._last_intent) > self.timeout_s
            if expired and not self._tripped:
                self._tripped = True
                return True
            return False


def validate_angle_deg(angle_deg) -> int:
    """Wijst NaN/inf/niet-numerieke/out-of-range hoeken af voor ze naar de
    servo gaan. Geeft een afgeronde int terug, roept ValueError op bij afwijzing."""
    try:
        angle = float(angle_deg)
    except (TypeError, ValueError):
        raise ValueError(f"ongeldige hoek: {angle_deg!r}")
    if math.isnan(angle) or math.isinf(angle):
        raise ValueError(f"NaN/inf hoek geweigerd: {angle_deg!r}")
    if not (-90 <= angle <= 90):
        raise ValueError(f"hoek buiten bereik -90..90: {angle}")
    return int(round(angle))


def validate_servo_id(servo_id) -> int:
    try:
        sid = int(servo_id)
    except (TypeError, ValueError):
        raise ValueError(f"ongeldig servo_id: {servo_id!r}")
    if not (1 <= sid <= 18):
        raise ValueError(f"servo_id buiten bereik 1-18: {sid}")
    return sid


def validate_runtime_ms(runtime_ms) -> int:
    try:
        rt = int(runtime_ms)
    except (TypeError, ValueError):
        raise ValueError(f"ongeldige runtime_ms: {runtime_ms!r}")
    return max(0, min(2000, rt))
