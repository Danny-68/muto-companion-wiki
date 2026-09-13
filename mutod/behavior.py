"""Fase 5 -- gedragslaag: centrale state machine + novelty-grid-wandelroutine.

Belangrijke scope-afbakening (12 sep 2026): dit bestand levert de
BESLISLOGICA (state machine + richtingskeuze) en de koppeling van `mode` in
`robot.state` (Fase 3's IPC). Het laat de robot in WANDER-modus nog NIET
automatisch daadwerkelijk lopen -- dat is een aparte, veel groter-impact stap
(onbeheerde, voortdurende, zelf-gekozen beweging i.p.v. een enkel, deadman-
begrensd commando) die een eigen, expliciete aankondiging/toestemming
verdient, zie muto_announce_before_movement-memory. `Daemon` in daemon.py
ticked de state machine alleen om `mode` te kunnen RAPPORTEREN; er wordt
vanuit hier geen enkele hal.gait()-aanroep gedaan.

Eén centrale beslisser (BehaviorStateMachine.tick()), geen losse if-else-
ketens verspreid over de codebase -- zie roadmap Fase 5 deliverables.
"""
import math
import time


class BehaviorMode:
    REST = "rest"
    WANDER = "wander"
    INTERACTION = "interaction"
    LOW_BATTERY = "low_battery"


# -- afstelbare drempels (bewust hier, niet verspreid) --------------------
REST_TO_WANDER_IDLE_S = 30.0    # zo lang niets gebeurt voor Rondlopen start
INTERACTION_TIMEOUT_S = 20.0    # zo lang blijft Interactie actief na de laatste trigger
LOW_BATTERY_PCT = 20.0          # onder dit percentage -> Laag batterij (hoogste prioriteit)
LOW_BATTERY_RECOVER_PCT = 30.0  # moet hierboven komen om Laag batterij weer te verlaten (hysterese)


class BehaviorStateMachine:
    """Eén beslisser: tick() neemt batterijniveau/tijd-sinds-interactie/
    aanwezigheid-van-andere-robots als input en levert de huidige modus.
    Geen state-mutatie buiten notify_interaction()/tick() om."""

    def __init__(self, now_fn=time.monotonic):
        self._now = now_fn
        self.state = BehaviorMode.REST
        self.last_interaction_ts = self._now() - INTERACTION_TIMEOUT_S - 1.0  # start "lang geleden"
        self._rest_entered_ts = self._now()

    def notify_interaction(self):
        """Aan te roepen door welke interactie-detectie dan ook (Fase 6/8's
        camera-gebaseerde detectie later, of handmatig/IPC nu) -- dit bestand
        detecteert zelf geen interacties, het reageert er alleen op."""
        self.last_interaction_ts = self._now()

    def tick(self, battery_pct=None, peers_present: bool = False) -> str:
        now = self._now()
        since_interaction = now - self.last_interaction_ts

        # Prioriteit 1: batterij, met hysterese zodat het niet flippert rond de drempel.
        if battery_pct is not None and battery_pct < LOW_BATTERY_PCT:
            self.state = BehaviorMode.LOW_BATTERY
            return self.state
        if self.state == BehaviorMode.LOW_BATTERY:
            if battery_pct is not None and battery_pct < LOW_BATTERY_RECOVER_PCT:
                return self.state  # nog niet genoeg hersteld
            self.state = BehaviorMode.REST
            self._rest_entered_ts = now

        # Prioriteit 2: interactie overschrijft Rondlopen/Rust.
        if since_interaction < INTERACTION_TIMEOUT_S:
            if self.state != BehaviorMode.INTERACTION:
                self.state = BehaviorMode.INTERACTION
            return self.state
        if self.state == BehaviorMode.INTERACTION:
            self.state = BehaviorMode.REST
            self._rest_entered_ts = now

        # Prioriteit 3: andere robots op het LAN -> conservatief niet rondlopen
        # (voorkomt onbeheerd risico op twee autonoom bewegende robots).
        if peers_present:
            if self.state == BehaviorMode.WANDER:
                self.state = BehaviorMode.REST
                self._rest_entered_ts = now
            return self.state

        # Prioriteit 4: lang genoeg niets gebeurd -> Rondlopen; eenmaal
        # Rondlopen, blijft dat totdat iets van bovenstaande het onderbreekt.
        if self.state == BehaviorMode.REST and (now - self._rest_entered_ts) >= REST_TO_WANDER_IDLE_S:
            self.state = BehaviorMode.WANDER
        return self.state


# -- novelty-grid-wandelroutine (richtingskeuze) ---------------------------

class NoveltyGrid:
    """Visit-count-raster op basis van odometrie, cel ~20cm (roadmap Fase 5)."""

    def __init__(self, cell_m: float = 0.20):
        self.cell_m = cell_m
        self._visits = {}

    def _cell(self, x: float, y: float):
        return (math.floor(x / self.cell_m), math.floor(y / self.cell_m))

    def visit(self, x: float, y: float):
        c = self._cell(x, y)
        self._visits[c] = self._visits.get(c, 0) + 1

    def visit_count(self, x: float, y: float) -> int:
        return self._visits.get(self._cell(x, y), 0)

    def novelty_score(self, x: float, y: float) -> float:
        """1.0 voor nooit bezocht, dalend naar 0 naarmate een cel vaker bezocht is."""
        return 1.0 / (1.0 + self.visit_count(x, y))


def choose_heading(candidate_headings_deg, clearance_by_heading_deg: dict,
                    current_x: float, current_y: float, current_yaw_deg: float,
                    grid: NoveltyGrid, step_m: float = 0.4, min_clear_m: float = 0.4):
    """Kiest de kandidaat-richting (graden, robot-relatief) met de beste
    score = min(clearance, 4.0) * (0.3 + 0.7 * onbezocht-score) van de cel
    die je zou bereiken als je step_m in die richting zou lopen. Clearance
    domineert bewust harder dan novelty (cap opgehoogd van 2.0m naar 4.0m,
    en novelty heeft een vloer van 0.3 zodat een veel opener richting niet
    wordt weggestemd puur omdat hij al vaker bezocht is) -- novelty blijft
    alleen de tiebreaker tussen ongeveer even open richtingen. Richtingen
    met minder dan min_clear_m vrije ruimte vallen af. Geeft None terug als
    niets voldoende vrij is (bv. omsingeld) -- de aanroeper moet dat als
    "sta stil, geen veilige richting" behandelen, nooit als "0 graden
    kiezen"."""
    best_heading = None
    best_score = -1.0
    for heading_deg in candidate_headings_deg:
        clearance = clearance_by_heading_deg.get(heading_deg)
        if clearance is None or clearance < min_clear_m:
            continue
        world_deg = current_yaw_deg + heading_deg
        rad = math.radians(world_deg)
        next_x = current_x + step_m * math.cos(rad)
        next_y = current_y + step_m * math.sin(rad)
        novelty = 0.3 + 0.7 * grid.novelty_score(next_x, next_y)
        score = min(clearance, 4.0) * novelty
        if score > best_score:
            best_score = score
            best_heading = heading_deg
    return best_heading
