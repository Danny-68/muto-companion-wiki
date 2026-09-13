#!/usr/bin/env python3
"""Offline zelftest voor mutod/discovery.py's PeerRegistry/elect_leader-logica
-- puur met synthetische tijdstempels, geen sockets. Zie
mutod_discovery_live_check.py voor de echte UDP-test met 2 processen."""
import sys

sys.path.insert(0, "/home/pi")

from mutod.discovery import PeerRegistry, elect_leader, SETTLE_S, STALE_S

failures = 0


def check(label, cond):
    global failures
    status = "OK  " if cond else "FOUT"
    print(f"[{status}] {label}")
    if not cond:
        failures += 1


# -- settle-window: een net binnengekomen peer telt nog niet mee --
reg = PeerRegistry(settle_s=1.5, stale_s=3.0)
reg.update("robot-b", "10.0.0.2", 8420, [], now=100.0)
check("net gezien -> nog niet settled", reg.settled_peer_ids(now=100.5) == set())
check("na settle_s -> wel settled", reg.settled_peer_ids(now=101.6) == {"robot-b"})

# -- stale-timeout: een aparte, ondubbelzinnige tijdlijn --
reg2 = PeerRegistry(settle_s=1.5, stale_s=3.0)
reg2.update("robot-b", "10.0.0.2", 8420, [], now=0.0)
check("t=0: nog niet settled (net gezien)", reg2.settled_peer_ids(now=0.0) == set())
check("t=2.0: settled (>1.5s oud, <3s stale)", reg2.settled_peer_ids(now=2.0) == {"robot-b"})
check("t=2.9: nog settled (net onder stale-grens)", reg2.settled_peer_ids(now=2.9) == {"robot-b"})
check("t=3.6: stale (>3s zonder nieuwe beacon) -> weg", reg2.settled_peer_ids(now=3.6) == set())
pruned = reg2.prune(now=3.6)
check("prune() ruimt de stale peer daadwerkelijk op", pruned == ["robot-b"])
check("na prune: registry leeg", reg2.snapshot() == {})

# -- deterministisch leiderschap: laagste ID wint --
check("alleen zelf -> zelf leider", elect_leader("muto-b", set()) == "muto-b")
check("zelf + hogere ID -> zelf blijft leider", elect_leader("muto-b", {"muto-c"}) == "muto-b")
check("zelf + lagere ID -> de ander wordt leider", elect_leader("muto-b", {"muto-a"}) == "muto-a")
check("meerdere peers -> laagste wint", elect_leader("muto-z", {"muto-m", "muto-a", "muto-q"}) == "muto-a")

# -- update() ververst last_seen zonder first_seen te resetten --
reg3 = PeerRegistry(settle_s=1.5, stale_s=3.0)
reg3.update("robot-x", "10.0.0.3", 8420, [], now=0.0)
reg3.update("robot-x", "10.0.0.3", 8420, [], now=1.0)  # zelfde peer, nieuwe beacon
check("herhaalde update -> nog steeds settled o.b.v. ORIGINELE first_seen", reg3.settled_peer_ids(now=1.6) == {"robot-x"})

print(f"\n{'ALLES OK' if failures == 0 else f'{failures} FOUT(EN)'}")
sys.exit(1 if failures else 0)
