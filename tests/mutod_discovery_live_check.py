#!/usr/bin/env python3
"""Live acceptatietest: 2 echte DiscoveryService-instanties (in dit proces,
elk op zijn eigen echte UDP-socket via SO_REUSEPORT op dezelfde broadcast-
poort) wisselen daadwerkelijk beacons uit over de loopback/LAN-broadcast --
geen robot-hardware nodig, wel echte sockets/threads/timing, geen mocks."""
import sys
import time

sys.path.insert(0, "/home/pi")

from mutod.discovery import DiscoveryService

failures = 0


def check(label, cond):
    global failures
    status = "OK  " if cond else "FOUT"
    print(f"[{status}] {label}")
    if not cond:
        failures += 1


a = DiscoveryService(robot_id="muto-test-a", port=18421, interval_s=0.3, settle_s=1.0, stale_s=3.0, tcp_port=9001)
b = DiscoveryService(robot_id="muto-test-b", port=18421, interval_s=0.3, settle_s=1.0, stale_s=3.0, tcp_port=9002)

a.start()
b.start()
try:
    time.sleep(0.6)
    check("A heeft B nog niet als settled peer (binnen settle-window)",
          "muto-test-b" not in a.registry.settled_peer_ids())

    time.sleep(1.2)
    peers_a = a.registry.snapshot()
    peers_b = b.registry.snapshot()
    print("A ziet:", list(peers_a.keys()))
    print("B ziet:", list(peers_b.keys()))
    check("A heeft B ontdekt", "muto-test-b" in peers_a)
    check("B heeft A ontdekt", "muto-test-a" in peers_b)
    check("A's beeld van B's tcp_port klopt", peers_a.get("muto-test-b", {}).get("tcp_port") == 9002)

    check("A heeft B nu settled", "muto-test-b" in a.registry.settled_peer_ids())
    check("B heeft A nu settled", "muto-test-a" in b.registry.settled_peer_ids())

    leader_a = a.leader()
    leader_b = b.leader()
    check("A en B kiezen dezelfde leider", leader_a == leader_b)
    check("de gekozen leider is de laagste ID ('muto-test-a')", leader_a == "muto-test-a")
    check("A weet dat hij leider is", a.is_leader() is True)
    check("B weet dat hij GEEN leider is", b.is_leader() is False)

    # -- stale-timeout: B stopt, A moet B na stale_s kwijtraken --
    b.stop()
    time.sleep(3.5)
    settled_after = a.registry.settled_peer_ids()
    check("na B's stop + stale-timeout: A heeft B losgelaten", "muto-test-b" not in settled_after)
    check("A is weer zijn eigen leider", a.is_leader() is True)

finally:
    a.stop()
    if b._recv_sock is not None:
        b.stop()

print(f"\n{'ALLES OK' if failures == 0 else f'{failures} FOUT(EN)'}")
sys.exit(1 if failures else 0)
