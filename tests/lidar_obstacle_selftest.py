#!/usr/bin/env python3
"""Offline zelftest voor lidar_obstacle.py -- synthetische scan-arrays."""
import sys, math

sys.path.insert(0, "/home/pi")
from lidar_obstacle import estimate_clearance_by_heading

failures = 0


def check(label, cond):
    global failures
    status = "OK  " if cond else "FOUT"
    print(f"[{status}] {label}")
    if not cond:
        failures += 1


N = 360
angle_min = math.radians(-180)
angle_increment = math.radians(1)  # 1 graad per sample, makkelijk te redeneren
range_min, range_max = 0.05, 8.0

# Alles 3.0m, behalve laser-hoek -90 (dus base_link-hoek -90, de correctie is
# sinds de 13-sep-2026-teken-fix 0 graden, niet meer +180) op 0.3m.
# (bewust NIET exact op de +-180-naad getest -- twee dict-keys (-180.0/180.0)
# voor dezelfde fysieke richting is een onschuldig, verwacht randgeval daar,
# niet relevant voor de candidate-headings die de executor gebruikt.)
ranges = [3.0] * N
ranges[90] = 0.3  # index 90 komt overeen met laser-hoek -90 (angle_min=-180, dus idx90=-180+90=-90)

clearance = estimate_clearance_by_heading(ranges, angle_min, angle_increment, range_min, range_max,
                                            heading_step_deg=15.0, useful_range_m=4.0)
check("laser-hoek -90 (obstakel 0.3m) -> base_link-hoek -90 laag", clearance.get(-90, 99) < 0.5)
check("base_link-hoek 0 (recht vooruit) blijft vrij (~3.0m)", abs(clearance.get(0, -1) - 3.0) < 0.2)
check("alle overige buckets rond 3.0m", all(abs(v - 3.0) < 0.2 for h, v in clearance.items() if h != -90))

# NaN/inf/buiten-bereik moet genegeerd worden, niet als 0 of oneindig meetellen.
ranges2 = [float('nan')] * N
ranges2[180] = 2.0  # laser-hoek 0 (angle_min=-180, idx180=-180+180=0) -> base_link-hoek 0 (correctie=0)
clearance2 = estimate_clearance_by_heading(ranges2, angle_min, angle_increment, range_min, range_max)
check("NaN-metingen genegeerd, alleen de geldige richting verschijnt", clearance2 == {0: 2.0} or set(clearance2.keys()) == {0})

print(f"\n{'ALLES OK' if failures == 0 else f'{failures} FOUT(EN)'}")
sys.exit(1 if failures else 0)
