"""Fase 4 -- UDP-broadcast discovery & coordinatie.

Architectuurbeslissing 3 (Fase 1 briefing): discovery via UDP-broadcast (niet
BLE, Muto en een toekomstige Microduck delen een LAN), coordinatiepatroon
(settle-window/stale-timeout/deterministisch leiderschap) "naar analogie van
chorale" overgenomen, eigen payload.

Noot bij "chorale": een publieke referentie-implementatie met die naam is bij
het bouwen van dit bestand (12 sep 2026) niet vindbaar gebleken (GitHub-zoek-
opdrachten leverden alleen muziek/Bach-chorale-datasets op, geen swarm-
coordinatieproject) -- vermoedelijk een niet-publieke of anders geheten bron
uit een eerdere sessie. De architectuurbeslissing zelf specificeert de
parameters al concreet genoeg (settle 1.5s, stale 3s, laagste-ID-leiderschap)
om zonder die bron te bouwen; als de originele "chorale" ooit teruggevonden
wordt, dit bestand ertegen afzetten.

Bewust opt-in, default UIT: deze service doet helemaal niets (geen socket
geopend, geen broadcast, geen luisteren) totdat start() expliciet aangeroepen
wordt. Zolang er geen tweede Muto/Microduck op het LAN is, voegt ongevraagd
UDP-verkeer alleen ruis en aanvalsoppervlak toe -- zie ook mutod's TCP-kanaal
(127.0.0.1:8420) dat om dezelfde soort reden geen authenticatie heeft zolang
het binnen localhost blijft.
"""
import json
import socket
import threading
import time
import uuid

from . import ipc_types

DISCOVERY_PORT = 8421  # apart van mutod's eigen TCP-commandokanaal (8420)
BEACON_INTERVAL_S = 1.5
SETTLE_S = 1.5
STALE_S = 3.0
MAGIC = "muto-discovery-v1"  # onderscheidt eigen beacons van willekeurige UDP-ruis op deze poort


def default_capabilities():
    return sorted(ipc_types.SUPPORTED_METHODS)


def default_robot_id():
    """Stabiel per machine (MAC-gebaseerd), zodat een robot na een herstart
    hetzelfde ID houdt -- belangrijk voor deterministisch leiderschap."""
    return f"muto-{uuid.getnode():012x}"


class PeerRegistry:
    """Thread-safe. first_seen/last_seen/host/tcp_port/capabilities per robot_id.

    Gebruikt time.monotonic() intern (niet time.time()) -- ongevoelig voor
    klokverschuivingen tussen robots, wat voor settle/stale-logica belangrijker
    is dan een absoluut tijdstip."""

    def __init__(self, settle_s: float = SETTLE_S, stale_s: float = STALE_S):
        self._lock = threading.Lock()
        self._peers = {}
        self.settle_s = settle_s
        self.stale_s = stale_s

    def update(self, robot_id: str, host: str, tcp_port, capabilities, now: float = None):
        now = time.monotonic() if now is None else now
        with self._lock:
            p = self._peers.get(robot_id)
            if p is None:
                self._peers[robot_id] = {
                    "host": host, "tcp_port": tcp_port, "capabilities": capabilities,
                    "first_seen": now, "last_seen": now,
                }
            else:
                p["host"] = host
                p["tcp_port"] = tcp_port
                p["capabilities"] = capabilities
                p["last_seen"] = now

    def prune(self, now: float = None) -> list:
        """Verwijdert stale peers, geeft hun ID's terug."""
        now = time.monotonic() if now is None else now
        with self._lock:
            stale = [rid for rid, p in self._peers.items() if now - p["last_seen"] > self.stale_s]
            for rid in stale:
                del self._peers[rid]
            return stale

    def settled_peer_ids(self, now: float = None) -> set:
        """ID's die minstens settle_s geleden voor het eerst gezien zijn EN
        niet stale zijn -- voorkomt dat een enkel, net binnengekomen beacon
        meteen meetelt voor leiderschap (flapping)."""
        now = time.monotonic() if now is None else now
        with self._lock:
            return {
                rid for rid, p in self._peers.items()
                if (now - p["first_seen"]) >= self.settle_s and (now - p["last_seen"]) <= self.stale_s
            }

    def snapshot(self) -> dict:
        with self._lock:
            return {rid: dict(p) for rid, p in self._peers.items()}


def elect_leader(self_id: str, settled_peer_ids) -> str:
    """Deterministisch leiderschap: laagste ID wint (lexicografische string-
    vergelijking), self altijd meegeteld zodat een robot zonder peers zichzelf
    als leider beschouwt."""
    return min(set(settled_peer_ids) | {self_id})


class DiscoveryService:
    def __init__(self, robot_id: str = None, port: int = DISCOVERY_PORT,
                 capabilities=None, interval_s: float = BEACON_INTERVAL_S,
                 settle_s: float = SETTLE_S, stale_s: float = STALE_S,
                 tcp_port: int = 8420):
        self.robot_id = robot_id or default_robot_id()
        self.port = port
        self.capabilities = default_capabilities() if capabilities is None else capabilities
        self.interval_s = interval_s
        self.tcp_port = tcp_port
        self.registry = PeerRegistry(settle_s=settle_s, stale_s=stale_s)
        self._stop = threading.Event()
        self._recv_sock = None
        self._threads = []

    # -- socket-opbouw ----------------------------------------------------

    def _make_recv_socket(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if hasattr(socket, "SO_REUSEPORT"):
            # Laat meerdere lokale processen dezelfde poort delen -- nodig om
            # Fase 4 lokaal met 2 processen te testen zonder een tweede robot
            # (zie module-docstring/roadmap-acceptatiecriterium).
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        sock.bind(("", self.port))
        sock.settimeout(0.5)
        return sock

    def _beacon_payload(self) -> bytes:
        return json.dumps({
            "magic": MAGIC,
            "robot_id": self.robot_id,
            "tcp_port": self.tcp_port,
            "capabilities": self.capabilities,
            "ts": time.time(),
        }).encode()

    # -- achtergrond-loops --------------------------------------------------

    def _broadcast_loop(self):
        send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        send_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        try:
            while not self._stop.is_set():
                try:
                    send_sock.sendto(self._beacon_payload(), ("255.255.255.255", self.port))
                except OSError:
                    pass
                self._stop.wait(self.interval_s)
        finally:
            send_sock.close()

    def _listen_loop(self):
        while not self._stop.is_set():
            try:
                data, addr = self._recv_sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                msg = json.loads(data.decode())
            except (ValueError, UnicodeDecodeError):
                continue
            if not isinstance(msg, dict) or msg.get("magic") != MAGIC:
                continue  # geen mutod-beacon, negeren (kan willekeurige UDP-ruis op deze poort zijn)
            rid = msg.get("robot_id")
            if not rid or rid == self.robot_id:
                continue  # eigen beacon (of ontbrekend ID) -- niet als peer registreren
            self.registry.update(rid, addr[0], msg.get("tcp_port"), msg.get("capabilities", []))

    def _prune_loop(self):
        while not self._stop.is_set():
            self.registry.prune()
            self._stop.wait(1.0)

    # -- publiek -------------------------------------------------------------

    def leader(self) -> str:
        return elect_leader(self.robot_id, self.registry.settled_peer_ids())

    def is_leader(self) -> bool:
        return self.leader() == self.robot_id

    def start(self):
        """Bewust de enige plek waar een socket geopend wordt -- zie de
        opt-in-uitleg in de module-docstring."""
        if self._recv_sock is not None:
            return
        self._recv_sock = self._make_recv_socket()
        self._stop.clear()
        self._threads = [
            threading.Thread(target=self._broadcast_loop, daemon=True),
            threading.Thread(target=self._listen_loop, daemon=True),
            threading.Thread(target=self._prune_loop, daemon=True),
        ]
        for t in self._threads:
            t.start()

    def stop(self):
        self._stop.set()
        for t in self._threads:
            t.join(timeout=1.0)
        self._threads = []
        if self._recv_sock is not None:
            self._recv_sock.close()
            self._recv_sock = None


def main():
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-id", default=None)
    parser.add_argument("--port", type=int, default=DISCOVERY_PORT)
    parser.add_argument("--seconds", type=float, default=15.0, help="hoe lang draaien voor afsluiten")
    args = parser.parse_args()

    svc = DiscoveryService(robot_id=args.robot_id, port=args.port)
    print(f"discovery: robot_id={svc.robot_id} poort={svc.port} (opt-in, nu expliciet gestart)")
    svc.start()
    try:
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            time.sleep(1.0)
            peers = svc.registry.snapshot()
            print(f"peers={list(peers.keys())} settled={sorted(svc.registry.settled_peer_ids())} leider={svc.leader()} (ik={svc.is_leader()})")
    except KeyboardInterrupt:
        pass
    finally:
        svc.stop()


if __name__ == "__main__":
    main()
