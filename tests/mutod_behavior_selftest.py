#!/usr/bin/env python3
"""Offline zelftest voor mutod/behavior.py -- puur beslislogica, geen
hardware/sockets. Gebruikt een injecteerbare fake klok zodat tijdsverloop
deterministisch getest kan worden zonder echt te wachten."""
import sys

sys.path.insert(0, "/home/pi")

from mutod.behavior import (
    BehaviorMode, BehaviorStateMachine,
    REST_TO_WANDER_IDLE_S, INTERACTION_TIMEOUT_S,
    LOW_BATTERY_PCT, LOW_BATTERY_RECOVER_PCT,
    NoveltyGrid, choose_heading,
)

failures = 0


def check(label, cond):
    global failures
    status = "OK  " if cond else "FOUT"
    print(f"[{status}] {label}")
    if not cond:
        failures += 1


class FakeClock:
    def __init__(self, t=0.0):
        self.t = t

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


# -- basis state machine gedrag --
clock = FakeClock(1000.0)
sm = BehaviorStateMachine(now_fn=clock)
check("start in REST", sm.state == BehaviorMode.REST)

check("blijft REST vóór idle-drempel", sm.tick() == BehaviorMode.REST)
clock.advance(REST_TO_WANDER_IDLE_S - 1)
check("net onder de drempel -> nog REST", sm.tick() == BehaviorMode.REST)
clock.advance(2.0)
check("over de drempel -> WANDER", sm.tick() == BehaviorMode.WANDER)
check("blijft WANDER zonder verstoring", sm.tick() == BehaviorMode.WANDER)

# -- interactie overschrijft Rondlopen --
sm.notify_interaction()
check("interactie -> meteen INTERACTION", sm.tick() == BehaviorMode.INTERACTION)
clock.advance(INTERACTION_TIMEOUT_S - 1)
check("nog binnen interactie-timeout -> INTERACTION", sm.tick() == BehaviorMode.INTERACTION)
clock.advance(2.0)
check("na interactie-timeout -> REST (niet meteen WANDER)", sm.tick() == BehaviorMode.REST)
clock.advance(REST_TO_WANDER_IDLE_S + 1)
check("na nieuwe idle-periode -> weer WANDER", sm.tick() == BehaviorMode.WANDER)

# -- batterij heeft absolute prioriteit, met hysterese --
check("lage batterij tijdens WANDER -> LOW_BATTERY", sm.tick(battery_pct=LOW_BATTERY_PCT - 1) == BehaviorMode.LOW_BATTERY)
sm.notify_interaction()
check("interactie tijdens LOW_BATTERY -> blijft LOW_BATTERY (batterij wint)", sm.tick(battery_pct=5.0) == BehaviorMode.LOW_BATTERY)
mid = (LOW_BATTERY_PCT + LOW_BATTERY_RECOVER_PCT) / 2
check("batterij tussen drempel en herstel -> nog LOW_BATTERY (hysterese)", sm.tick(battery_pct=mid) == BehaviorMode.LOW_BATTERY)
clock.advance(INTERACTION_TIMEOUT_S + 1)  # laat de eerdere notify_interaction() verlopen, isoleert het batterij-effect
check("batterij boven hersteldrempel (en interactie inmiddels verlopen) -> REST", sm.tick(battery_pct=LOW_BATTERY_RECOVER_PCT + 1) == BehaviorMode.REST)

# -- aanwezigheid van andere robots onderdrukt Rondlopen --
clock2 = FakeClock(0.0)
sm2 = BehaviorStateMachine(now_fn=clock2)
clock2.advance(REST_TO_WANDER_IDLE_S + 1)
check("geen peers -> WANDER start normaal", sm2.tick(peers_present=False) == BehaviorMode.WANDER)
check("peer verschijnt tijdens WANDER -> terug naar REST", sm2.tick(peers_present=True) == BehaviorMode.REST)
clock2.advance(REST_TO_WANDER_IDLE_S + 1)
check("peer blijft aanwezig -> blijft REST, GEEN WANDER ondanks lange idle-tijd", sm2.tick(peers_present=True) == BehaviorMode.REST)
check("peer weg -> mag weer WANDER worden", sm2.tick(peers_present=False) == BehaviorMode.WANDER)

# -- NoveltyGrid --
grid = NoveltyGrid(cell_m=0.20)
check("onbezochte cel -> score 1.0", grid.novelty_score(1.0, 1.0) == 1.0)
grid.visit(1.0, 1.0)
check("1x bezocht -> score 0.5", grid.novelty_score(1.0, 1.0) == 0.5)
grid.visit(1.0, 1.0)
grid.visit(1.0, 1.0)
check("3x bezocht -> score 0.25", grid.novelty_score(1.0, 1.0) == 0.25)
check("nabijgelegen andere cel blijft onbezocht", grid.novelty_score(5.0, 5.0) == 1.0)

# -- choose_heading --
grid2 = NoveltyGrid(cell_m=0.20)
headings = [0, 90, 180, 270]
# 0 graden: dichtbij (geblokkeerd), 90: ruim vrij maar al vaak bezocht,
# 180: ruim vrij en onbezocht -> moet winnen, 270: net te weinig clearance
clearance = {0: 0.1, 90: 2.0, 180: 2.0, 270: 0.3}
# markeer de cel die je bij 90 graden zou bereiken als druk bezocht
import math
step_m = 0.4
x90 = 0 + step_m * math.cos(math.radians(0 + 90))
y90 = 0 + step_m * math.sin(math.radians(0 + 90))
for _ in range(10):
    grid2.visit(x90, y90)

pick = choose_heading(headings, clearance, current_x=0.0, current_y=0.0, current_yaw_deg=0.0,
                       grid=grid2, step_m=step_m, min_clear_m=0.4)
check("kiest 180 (ruim vrij + onbezocht), niet 0/270 (te weinig clearance) of 90 (druk bezocht)", pick == 180)

pick_none = choose_heading([0, 90], {0: 0.1, 90: 0.2}, 0.0, 0.0, 0.0, grid2, min_clear_m=0.4)
check("alles geblokkeerd -> None (nooit een default-richting forceren)", pick_none is None)

print(f"\n{'ALLES OK' if failures == 0 else f'{failures} FOUT(EN)'}")
sys.exit(1 if failures else 0)
