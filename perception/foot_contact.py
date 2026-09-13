#!/usr/bin/env python3
"""
Optie A — Voetcontact detectie via servo hoekfout.
v4 (12 sep 2026, Fase 6): herbouwd op 0x50 (MOTOR_ANGLE bulk-read), via mutod
-- v3 gebruikte 0x60 (ATTITUDE_ANGLE, IMU) alsof het een per-servo-hoek was;
dat adres negeert de ID-parameter volledig (zie Fase 1 briefing §1.2), dus de
v3-metingen waren zinloos. mutod is nu bovendien de enige eigenaar van
/dev/myserial (TIOCEXCL) -- een eigen serial.Serial() hier zou EBUSY geven.
Deze v4 leest via mutod's TCP-kanaal (127.0.0.1:8420, {"op": "get_state"}),
dezelfde 0x50-bulk-read die mutod's control-loop toch al continu doet, i.p.v.
zelf los te lezen. Puur lezen -- deze detector stuurt zelf NOOIT een
bewegingscommando (ook `standing_posture()` niet meer, zie onderaan).
"""
import json
import socket
import threading
import time

TIBIA_IDS         = [3, 6, 9, 12, 15, 18]
LEG_NAMES         = {3: "RF", 6: "RM", 9: "RR", 12: "LR", 15: "LM", 18: "LF"}
CONTACT_THRESHOLD = 12    # graden -- ongewijzigd t.o.v. v3


class MutodStateClient:
    """Kleine, alleen-lezende TCP-client voor mutod's Fase 2 {"op": ...}-kanaal.
    Geen afhankelijkheid van mutod_client.py (die is voor binnen de humble_run
    container bedoeld) -- dit draait op de host, waar mutod zelf ook draait."""

    def __init__(self, host="127.0.0.1", port=8420, timeout=2.0):
        self._host, self._port, self._timeout = host, port, timeout
        self._sock = None
        self._rfile = None
        self._lock = threading.Lock()

    def _ensure_connected(self):
        if self._sock is not None:
            return
        self._sock = socket.create_connection((self._host, self._port), timeout=self._timeout)
        self._rfile = self._sock.makefile("rb")

    def get_all_angles(self) -> dict:
        """Geeft {servo_id: hoek_graden} voor alle 18 servo's, uit mutod's
        laatst gelezen 0x50-snapshot (zelfde data als de control-loop gebruikt,
        typisch <=1s oud -- geen extra los serieel verkeer)."""
        with self._lock:
            try:
                self._ensure_connected()
                self._sock.sendall((json.dumps({"op": "get_state"}) + "\n").encode())
                line = self._rfile.readline()
                if not line:
                    raise ConnectionError("mutod sloot de verbinding")
                resp = json.loads(line)
            except (OSError, ConnectionError, json.JSONDecodeError) as exc:
                self._sock = None
                self._rfile = None
                raise ConnectionError(f"mutod-verbinding mislukt: {exc}") from exc
        if not resp.get("ok"):
            raise RuntimeError(f"get_state mislukt: {resp.get('error')}")
        return {int(sid): deg for sid, deg in resp["angles"].items()}


class FootContactDetector:
    def __init__(self, client: MutodStateClient):
        self.client    = client
        self.lock      = threading.Lock()
        self.commanded = {sid: None  for sid in TIBIA_IDS}
        self.contact   = {sid: False for sid in TIBIA_IDS}
        self.actual    = {sid: None  for sid in TIBIA_IDS}
        self.error_deg = {sid: 0.0   for sid in TIBIA_IDS}

    def calibrate_baseline(self):
        """Legt de HUIDIGE houding vast als baseline -- verstuurt zelf GEEN
        houding-commando (v3 deed dat nog via een eigen standing_posture()-
        write). Roep zelf eerst mutod's al-geteste reset_posture-op aan als je
        wilt beginnen vanuit een bekende stand, dan pas deze functie."""
        print("  Baseline uitlezen (via mutod, 0x50 bulk-read)...")
        angles = self.client.get_all_angles()
        for sid in TIBIA_IDS:
            leg = LEG_NAMES[sid]
            angle = angles.get(sid)
            if angle is not None:
                self.commanded[sid] = angle
                print(f"    {leg} tibia (servo {sid:2d}): {angle}°")
            else:
                print(f"    {leg} tibia (servo {sid:2d}): ontbreekt in mutod's snapshot")

    def update(self):
        angles = self.client.get_all_angles()
        with self.lock:
            for sid in TIBIA_IDS:
                actual = angles.get(sid)
                if actual is None:
                    continue
                self.actual[sid] = actual

                cmd = self.commanded[sid]
                if cmd is not None:
                    err = abs(cmd - actual)
                    self.error_deg[sid] = err
                    self.contact[sid]   = err > CONTACT_THRESHOLD
                else:
                    self.commanded[sid] = actual
                    self.error_deg[sid] = 0.0
                    self.contact[sid]   = False

    def get_contact(self) -> dict:
        with self.lock:
            return {LEG_NAMES[sid]: self.contact[sid] for sid in TIBIA_IDS}

    def get_status(self) -> dict:
        with self.lock:
            return {
                LEG_NAMES[sid]: {
                    "contact":   self.contact[sid],
                    "commanded": self.commanded[sid],
                    "actual":    self.actual[sid],
                    "error_deg": self.error_deg[sid],
                }
                for sid in TIBIA_IDS
            }


# ── Standalone test (alleen-lezend, geen bewegingscommando) ────────────
if __name__ == "__main__":
    print("=" * 50)
    print(" FootContact detector v4 (via mutod, 0x50 bulk-read)")
    print(f" Drempel: {CONTACT_THRESHOLD}°")
    print("=" * 50)

    client = MutodStateClient()
    fc = FootContactDetector(client)

    print("\n[1/2] Baseline vastleggen (huidige houding, geen commando verstuurd):")
    fc.calibrate_baseline()
    print("      Baseline opgeslagen")

    print(f"\n[2/2] Live contact detectie (drempel={CONTACT_THRESHOLD}°)")
    print("      Til robot op -> alle poten 'vrij'")
    print("      Zet neer    -> alle poten 'GROND'")
    print("      Ctrl+C = stoppen\n")
    print(f"  {'RF':>7} {'RM':>7} {'RR':>7} {'LR':>7} {'LM':>7} {'LF':>7}")
    print("  " + "-" * 44)

    try:
        while True:
            fc.update()
            s = fc.get_status()
            line = "  "
            for leg in ["RF", "RM", "RR", "LR", "LM", "LF"]:
                st = s[leg]
                if st["commanded"] is None:
                    marker = "  ???"
                elif st["contact"]:
                    marker = "GROND"
                else:
                    marker = f" {st['error_deg']:3.0f}°"
                line += f"{marker:>7}"
            print(line, end="\r")
            time.sleep(0.15)

    except KeyboardInterrupt:
        print("\n\nEindstatus:")
        for leg, st in fc.get_status().items():
            status = "GROND" if st["contact"] else "vrij"
            print(f"  {leg}: actual={st['actual']}°  "
                  f"cmd={st['commanded']}°  "
                  f"err={st['error_deg']:.0f}°  "
                  f"-> {status}")
