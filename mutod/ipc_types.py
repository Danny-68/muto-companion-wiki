"""duck-ipc-proto-compatibele IPC-laag (Fase 3).

Methode-namen en param-vormen komen rechtstreeks van upstream's
`pollen-robotics/microduck` `duck-ipc-proto` protocol, zoals gepubliceerd in
`joeynyc/microduck-mcp`'s `src/transport/protocol.ts` (referentie-implementatie,
commit 590b986 t.o.v. bovenstroom -- gecheckt 12 sep 2026). Alleen de subset
die Muto's eigen roadmap nodig heeft is hier geimplementeerd: robot.move,
robot.state, robot.health, robot.do, robot.stop, robot.subscribe. Methodes
als robot.head/robot.sound/robot.init/robot.policies horen bij Microduck's
eigen (tweevoeter/camera/policy-channel) functionaliteit en zijn bewust
buiten scope voor deze fase.

Transport: zelfde TCP-newline-delimited-JSON-kanaal als Fase 2 (127.0.0.1:8420)
-- niet upstream's Unix-sockets, om dezelfde reden als de rest van mutod (zie
daemon.py's module-docstring). Het enige verschil met upstream is dus het
transport, niet de berichtvorm: hier staat wel een echte JSON-RPC 2.0-envelope
(jsonrpc/id/method/params, result/error) op de draad, in tegenstelling tot
Fase 2's kortere interne {"op": ...}-kanaal (dat blijft ongewijzigd bestaan
voor mutod_client.py/phoenix_driver.py's raw_frames-relay).
"""

# -- methode-namen (upstream duck-ipc-proto spelling, ongewijzigd overgenomen) --
ROBOT_HEALTH = "robot.health"
ROBOT_MOVE = "robot.move"          # {vx, vy, vyaw} m/s, m/s, rad/s
ROBOT_STOP = "robot.stop"
ROBOT_DO = "robot.do"              # {skill}
ROBOT_SUBSCRIBE = "robot.subscribe"  # {hz?} -> SubscribeResult, dan robot.state-notificaties
ROBOT_STATE = "robot.state"        # notificatie (server->client); ook one-shot beantwoordbaar
# 13 sep 2026 (Fase 5-vervolg): externe detectoren (bv. wander_executor.py's
# YOLO-check bij een veiligheidsstop) kunnen hiermee behavior.py's al
# bestaande, tot nu toe nooit aangeroepen notify_interaction() triggeren --
# geen nieuwe gedragslogica, alleen de al ontworpen hook eindelijk aansluiten.
ROBOT_NOTIFY_INTERACTION = "robot.notify_interaction"  # geen params -> {"accepted": True}

# 20 sep 2026 (AI-executive-architectuur, stap 1 van het plan, zie
# muto_ai_executive_architecture_decision_2026-09-20-memory): een Muto-eigen
# toevoeging, GEEN upstream duck-ipc-proto-methode -- gecheckt bij
# joeynyc/microduck-mcp's protocol.ts en upstream heeft niets vergelijkbaars
# (robot.policies gaat daar over RL-policy-slots wisselen, niet over
# perceptie-aggregatie). mutod draait op de kale host, zonder rclpy/ROS-
# toegang, dus kan zelf geen YOLO/diepte/positie uitlezen -- externe
# processen (wander_executor.py, yolo_snapshot_sender.py, straks een relay)
# PUSHEN hun laatste waarneming hierheen; robot.world_state leest het
# geheel terug, samen met wat mutod toch al weet (mode/servos/deadman).
# Bewust generiek (willekeurige key -> willekeurige JSON-blob + tijdstempel)
# -- geen schema-afdwinging hier, dat is aan de pushers zelf.
ROBOT_WORLD_UPDATE = "robot.world_update"  # {key, data} -> {"accepted": True}
ROBOT_WORLD_STATE = "robot.world_state"    # geen params -> {t, mode, world: {key: {data, age_s}}}

SUPPORTED_METHODS = {
    ROBOT_HEALTH, ROBOT_MOVE, ROBOT_STOP, ROBOT_DO, ROBOT_SUBSCRIBE, ROBOT_STATE,
    ROBOT_NOTIFY_INTERACTION, ROBOT_WORLD_UPDATE, ROBOT_WORLD_STATE,
}

# JSON-RPC 2.0 standaard foutcodes (gebruikt zoals upstream ze gebruikt).
PARSE_ERROR = -32700
INVALID_REQUEST = -32600
METHOD_NOT_FOUND = -32601
INVALID_PARAMS = -32602
INTERNAL_ERROR = -32603

# Muto's eigen skills (Fase 6) zijn nog niet gebouwd/getest -- 0x3E (ACTION)
# is expliciet "niet getest" in Fase 1 §1.2 en Fase 6 zegt uitdrukkelijk
# "test dit eerst voor je iets nieuws bouwt". robot.do bestaat hier dus al
# als plumbing/schema, maar wijst elke skill af tot Fase 6 dat daadwerkelijk
# valideert -- geen ongeteste hardware-aanroepen via een achterdeur.
KNOWN_SKILLS: set = set()

# Bandbreedte voor robot.subscribe's hz-parameter -- lager dan de 25Hz control-
# loop zelf (geen zin om sneller te pushen dan de data zelf ververst), en met
# een bodem zodat een client geen 0Hz/oneindige-interval-thread kan starten.
SUBSCRIBE_MIN_HZ = 0.5
SUBSCRIBE_MAX_HZ = 25.0
SUBSCRIBE_DEFAULT_HZ = 5.0


def is_jsonrpc_request(cmd: dict) -> bool:
    """Onderscheidt een duck-ipc-proto JSON-RPC-bericht van Fase 2's interne
    {"op": ...}-berichten op hetzelfde kanaal -- ze blijven naast elkaar
    bestaan, zie module-docstring."""
    return isinstance(cmd, dict) and cmd.get("jsonrpc") == "2.0" and "method" in cmd


def make_result(req_id, result: dict) -> dict:
    return {"jsonrpc": "2.0", "id": req_id, "result": result}


def make_error(req_id, code: int, message: str, data=None) -> dict:
    err = {"code": code, "message": message}
    if data is not None:
        err["data"] = data
    return {"jsonrpc": "2.0", "id": req_id, "error": err}


def make_notification(method: str, params: dict) -> dict:
    return {"jsonrpc": "2.0", "method": method, "params": params}


def clamp_hz(hz) -> float:
    try:
        hz = float(hz)
    except (TypeError, ValueError):
        hz = SUBSCRIBE_DEFAULT_HZ
    return max(SUBSCRIBE_MIN_HZ, min(SUBSCRIBE_MAX_HZ, hz))
