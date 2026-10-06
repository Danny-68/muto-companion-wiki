# Draait in de ROS-container: cd /root && source /opt/ros/humble/setup.bash && python3 boredom_test.py (importeert wander_executor)
import re, wander_executor as w
TH = w.BOREDOM_PAUSE_THRESHOLD
print(f"constanten: groei={w.BOREDOM_GROWTH_PER_S}/s relief={w.BOREDOM_NOVELTY_RELIEF_PER_S}/s maxdt={w.BOREDOM_MAX_DT_S}s drempel={TH}")

def run(name, dt, nov, T=300):
    b = 0.0; t = 0.0; first = None; n = 0
    while t <= T:
        b = w.update_boredom(b, dt, nov); t += dt
        if b >= TH:
            n += 1; first = first or t; b = 0.0
    fs = f"{first:.0f}s" if first else "nooit"
    print(f"  {name:38s} eerste pauze: {fs:>6s} | pauzes in {T}s: {n}")

print("S1-S4 kunstmatig (300 s):")
run("krap: keuze elke 2 s, novelty 0.05", 2.0, 0.05)
run("krap: keuze elke 2 s, novelty 0.25", 2.0, 0.25)
run("open: keuze elke 10 s, novelty 0.10", 10.0, 0.10)
run("open: keuze elke 10 s, novelty 0.30", 10.0, 0.30)
run("vers gebied: novelty 1.0", 10.0, 1.0)
G, R, M = w.BOREDOM_GROWTH_PER_S, w.BOREDOM_NOVELTY_RELIEF_PER_S, w.BOREDOM_MAX_DT_S
fails = []
def check(name, got, want):
    ok = abs(got - want) < 1e-6
    print(f"  {'PASS' if ok else 'FAIL'} {name}: {got:.3f} (verwacht {want:.3f})")
    if not ok: fails.append(name)
print("eenheidstests (verwachting uit de constanten berekend):")
check("S5 ontdekking 0.40, dt 10, novelty 0.9", w.update_boredom(0.4, 10, 0.9), 0.4 + 10 * (G - R * 0.9))
check("S6 dt-cap dt 60, novelty 0", w.update_boredom(0.0, 60, 0.0), M * G)
check("S7a clamp boven", w.update_boredom(0.99, 20, 0.0), 1.0)
check("S7b clamp onder", w.update_boredom(0.05, 20, 1.0), 0.0)
check("S8 negatieve dt wordt 0", w.update_boredom(0.3, -5, 0.0), 0.3)
check("S9 dt 0 verandert niets", w.update_boredom(0.42, 0, 0.1), 0.42)
check("S10 neutraal bij novelty G/R", w.update_boredom(0.3, 10, G / R), 0.3)
print("EENHEIDSTESTS:", "ALLE GESLAAGD" if not fails else f"MISLUKT: {fails}")

import os
LOG = "/root/wander_live_replay.log"  # kopie van een echt wander-log (niet in de repo); zonder dit bestand wordt de replay overgeslagen
ev = []
for l in (open(LOG, errors="replace") if os.path.exists(LOG) else []):
    m = re.search(r"\[(\d+\.\d+)\].*verveling: novelty hier=([0-9.]+)", l)
    if m: ev.append((float(m.group(1)), "pick", float(m.group(2)))); continue
    m = re.search(r"\[(\d+\.\d+)\].*onderzoek klaar", l)
    if m: ev.append((float(m.group(1)), "pauze_einde", 0.0))
if not ev:
    print("REPLAY overgeslagen: geen wander-log gevonden"); raise SystemExit(0 if not fails else 1)
b = 0.0; last = None; trig = []; t0 = ev[0][0]; picks = sum(1 for e in ev if e[1] == "pick")
for ts, k, nv in ev:
    if k == "pauze_einde": last = None; continue
    dt = 0.0 if last is None else ts - last; last = ts
    b = w.update_boredom(b, dt, nv)
    if b >= TH: trig.append(round(ts - t0)); b = 0.0; last = None
print(f"REPLAY echte sessie ({picks} keuzemomenten): oude formule 5 pauzes; nieuwe formule pauzeert op t={trig} s -> {len(trig)} pauzes")
