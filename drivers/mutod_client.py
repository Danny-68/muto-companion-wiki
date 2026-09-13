"""mutod_client.py -- dunne TCP-client voor mutod, voor gebruik binnen de
humble_run ROS2-container (die geen toegang heeft tot /home/pi/mutod op de
host -- alleen /dev en /home/pi/yahboomcar_ros2_ws zijn gemount).

Verbindt via TCP naar 127.0.0.1:8420 (werkt vanuit de container omdat deze
met NetworkMode=host draait, dus dezelfde netwerk-namespace als de host
deelt -- zie muto_companion_overdracht_2026-09-11.md).

Twee onderdelen:
- MutodClient: newline-delimited-JSON request/response over TCP.
- MutodSerialShim: implementeert het kleine stukje van de serial.Serial-
  interface dat MutoLib.Servo/phoenix_gait.set_exec_time gebruiken, zodat
  phoenix_driver.py's HardwareInterface ongewijzigde IK-code kan blijven
  draaien terwijl de daadwerkelijke bytes via mutod naar de STM32 gaan i.p.v.
  via een eigen, tweede serial.Serial-verbinding (die zou botsen met mutod's
  TIOCEXCL-exclusieve poort).
"""
import json
import socket
import threading


class MutodClient:
    def __init__(self, host="127.0.0.1", port=8420, timeout=2.0):
        self._host = host
        self._port = port
        self._timeout = timeout
        self._sock = None
        self._rfile = None
        self._lock = threading.Lock()

    def _ensure_connected(self):
        if self._sock is not None:
            return
        sock = socket.create_connection((self._host, self._port), timeout=self._timeout)
        self._sock = sock
        self._rfile = sock.makefile("rb")

    def _close(self):
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass
        self._sock = None
        self._rfile = None

    def send_command(self, cmd: dict) -> dict:
        with self._lock:
            try:
                self._ensure_connected()
                self._sock.sendall((json.dumps(cmd) + "\n").encode())
                line = self._rfile.readline()
                if not line:
                    raise ConnectionError("mutod sloot de verbinding")
                return json.loads(line)
            except (OSError, ConnectionError, json.JSONDecodeError) as exc:
                self._close()
                raise ConnectionError(f"mutod-verbinding mislukt: {exc}") from exc

    def read_attitude(self) -> dict:
        resp = self.send_command({"op": "read_attitude"})
        if not resp.get("ok"):
            raise RuntimeError(f"read_attitude mislukt: {resp.get('error')}")
        return resp

    def stop(self):
        resp = self.send_command({"op": "stop"})
        if not resp.get("ok"):
            raise RuntimeError(f"stop mislukt: {resp.get('error')}")


class MutodSerialShim:
    """Vervangt een echte serial.Serial binnen MutoLib.Servo. Buffert write()-
    aanroepen (elke aanroep is al een compleet, valide protocol-frame,
    gebouwd door Servo.motor()/set_exec_time()) en stuurt ze pas als batch
    naar mutod via flush_batch() -- normaal 1x per gait-tick, zodat er een
    socket-round-trip per tick is i.p.v. een per servo-write."""

    def __init__(self, client: MutodClient):
        self._client = client
        self._buffer = []

    def write(self, data) -> int:
        self._buffer.append(bytes(data))
        return len(data)

    def flush_batch(self):
        if not self._buffer:
            return
        frames_hex = [f.hex() for f in self._buffer]
        self._buffer = []
        resp = self._client.send_command({"op": "raw_frames", "frames_hex": frames_hex})
        if not resp.get("ok"):
            raise RuntimeError(f"mutod raw_frames mislukt: {resp.get('error')}")

    def read(self, size=1):
        raise NotImplementedError(
            "MutodSerialShim ondersteunt geen read() -- gebruik MutodClient.read_attitude()"
        )

    def reset_input_buffer(self):
        pass  # geen-op: er wordt nooit gelezen via deze shim

    @property
    def is_open(self):
        return True
