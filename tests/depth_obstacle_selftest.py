#!/usr/bin/env python3
"""Offline zelftest voor depth_obstacle.py -- synthetische numpy depth-frames,
geen camera/hardware nodig."""
import sys

sys.path.insert(0, "/home/pi")
import numpy as np
from depth_obstacle import estimate_clearance_by_heading, merge_lidar_and_depth_clearance

failures = 0


def check(label, cond):
    global failures
    status = "OK  " if cond else "FOUT"
    print(f"[{status}] {label}")
    if not cond:
        failures += 1


WIDTH, HEIGHT = 640, 480
FX, CX = 574.45, 311.73

# -- uniforme diepte: alle richtingen zelfde clearance --
frame = np.full((HEIGHT, WIDTH), 2000, dtype=np.uint16)  # 2.0m overal
clearance = estimate_clearance_by_heading(frame, FX, CX, heading_step_deg=15.0)
check("uniforme 2m-diepte -> center-heading (0) rond 2.0m", 0 in clearance and abs(clearance[0] - 2.0) < 0.05)
check("uniforme diepte -> meerdere richtingen gevonden", len(clearance) >= 3)
check("alle waarden rond 2.0m", all(abs(v - 2.0) < 0.05 for v in clearance.values()))

# -- obstakel recht vooruit (kolom rond cx), ver weg elders --
frame2 = np.full((HEIGHT, WIDTH), 3000, dtype=np.uint16)
frame2[:, int(CX) - 30:int(CX) + 30] = 300  # obstakel op 0.3m, recht vooruit
clearance2 = estimate_clearance_by_heading(frame2, FX, CX, heading_step_deg=15.0)
check("obstakel vooruit -> heading 0 laag (~0.3m)", clearance2.get(0, 99) < 0.5)
check("obstakel vooruit -> zijwaartse headings blijven ver (~3.0m)",
      all(abs(v - 3.0) < 0.1 for h, v in clearance2.items() if h != 0))

# -- (nagenoeg) volledig ongeldige data -> geen richtingen gerapporteerd --
frame3 = np.zeros((HEIGHT, WIDTH), dtype=np.uint16)
clearance3 = estimate_clearance_by_heading(frame3, FX, CX, heading_step_deg=15.0)
check("volledig ongeldige diepte -> lege clearance-dict (niet gokken)", clearance3 == {})

# -- merge: minimum waar beide bestaan, unie waar niet --
lidar = {-30: 5.0, 0: 5.0, 30: 5.0, 90: 2.0, 180: 1.0, -90: 2.0}
depth = {-30: 4.5, 0: 0.4, 30: 5.5}  # dieptecamera ziet alleen het voorwaartse segment
merged = merge_lidar_and_depth_clearance(lidar, depth)
check("merge: overlappende heading neemt het MINIMUM", merged[0] == 0.4 and merged[-30] == 4.5)
check("merge: depth's hogere waarde wint niet als lidar lager is", merged[30] == 5.0)
check("merge: headings alleen in LiDAR blijven behouden", merged[180] == 1.0 and merged[90] == 2.0)
check("merge: geen extra headings verzonnen", set(merged.keys()) == set(lidar.keys()))

print(f"\n{'ALLES OK' if failures == 0 else f'{failures} FOUT(EN)'}")
sys.exit(1 if failures else 0)
