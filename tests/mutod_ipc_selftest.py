#!/usr/bin/env python3
"""Offline zelftest voor mutod's Fase 3 duck-ipc-proto-laag (ipc_types.py +
Daemon._handle_rpc in daemon.py). Raakt GEEN hardware -- de echte MutoHAL
wordt vervangen door een fake die alleen aanroepen registreert, dus dit is
puur een test van de berichten-plumbing (JSON-RPC-vorm, methode-dispatch,
vx/vy/vyaw -> gait-richting, deadband, validatie). Een echte fysieke
robot.move-test (die de robot laat lopen) is bewust een aparte, expliciet
aangekondigde stap -- zie muto_announce_before_movement-memory."""
import sys

sys.path.insert(0, "/home/pi")

from mutod import ipc_types
from mutod.daemon import Daemon


class FakeHAL:
    def __init__(self):
        self.calls = []

    def gait(self, direction, step):
        self.calls.append(("gait", direction, step))

    def stay_put(self):
        self.calls.append(("stay_put",))


def rpc(daemon, method, params=None, req_id=1):
    req = {"jsonrpc": "2.0", "id": req_id, "method": method}
    if params is not None:
        req["params"] = params
    return daemon._handle_rpc(req)


def check(label, cond):
    status = "OK  " if cond else "FOUT"
    print(f"[{status}] {label}")
    if not cond:
        global failures
        failures += 1


failures = 0

d = Daemon(port="/dev/null", listen_host="127.0.0.1", listen_port=0)
d.hal = FakeHAL()

# -- robot.health: geen reads gedaan -> unhealthy --
resp = rpc(d, ipc_types.ROBOT_HEALTH)
check("health zonder reads -> ok", resp.get("result", {}).get("healthy") is False)

# -- na een verse read -> healthy --
d.state.set_angles({i: 0 for i in range(1, 19)})
resp = rpc(d, ipc_types.ROBOT_HEALTH)
check("health na verse read -> healthy", resp["result"]["healthy"] is True)

# -- robot.state vorm --
resp = rpc(d, ipc_types.ROBOT_STATE)
r = resp["result"]
check("state heeft 18 servo's", len(r["servos"]) == 18)
check("state heeft move/loop/safety", all(k in r for k in ("move", "loop", "safety")))
check("state heeft mode (Fase 5)", "mode" in r and isinstance(r["mode"], str))

# -- robot.move: dominante as + richting + deadband --
resp = rpc(d, ipc_types.ROBOT_MOVE, {"vx": 0.05, "vy": 0.0, "vyaw": 0.0})
check("move vx>0 -> FORWARD", resp["result"]["direction"] == "FORWARD")
check("move riep hal.gait aan", d.hal.calls[-1][0] == "gait" and d.hal.calls[-1][1] == "FORWARD")

resp = rpc(d, ipc_types.ROBOT_MOVE, {"vx": -0.05, "vy": 0.0, "vyaw": 0.0})
check("move vx<0 -> BACKWARD", resp["result"]["direction"] == "BACKWARD")

resp = rpc(d, ipc_types.ROBOT_MOVE, {"vx": 0.0, "vy": 0.0, "vyaw": 0.1})
check("move vyaw>0 -> TURN_LEFT", resp["result"]["direction"] == "TURN_LEFT")

resp = rpc(d, ipc_types.ROBOT_MOVE, {"vx": 0.0, "vy": 0.0, "vyaw": -0.1})
check("move vyaw<0 -> TURN_RIGHT", resp["result"]["direction"] == "TURN_RIGHT")

n_calls_before = len(d.hal.calls)
resp = rpc(d, ipc_types.ROBOT_MOVE, {"vx": 0.001, "vy": 0.001, "vyaw": 0.001})
check("move binnen deadband -> geen gait-call", len(d.hal.calls) == n_calls_before)
check("move binnen deadband -> direction None", resp["result"]["direction"] is None)

resp = rpc(d, ipc_types.ROBOT_MOVE, {"vx": "geen getal"})
check("move met ongeldig type -> INVALID_PARAMS", resp.get("error", {}).get("code") == ipc_types.INVALID_PARAMS)

resp = rpc(d, ipc_types.ROBOT_MOVE, {"vx": float("nan")})
check("move met NaN -> INVALID_PARAMS", resp.get("error", {}).get("code") == ipc_types.INVALID_PARAMS)

# -- robot.stop --
resp = rpc(d, ipc_types.ROBOT_STOP)
check("stop -> accepted", resp["result"]["accepted"] is True)
check("stop riep hal.stay_put aan", d.hal.calls[-1] == ("stay_put",))
move, _ = d.state.get_last_move()
check("stop -> last_move gereset naar 0", move == {"vx": 0.0, "vy": 0.0, "vyaw": 0.0})

# -- robot.do: geen skills geimplementeerd (Fase 6) --
resp = rpc(d, ipc_types.ROBOT_DO, {"skill": "wave"})
check("do -> INVALID_PARAMS (nog geen skills)", resp.get("error", {}).get("code") == ipc_types.INVALID_PARAMS)

# -- robot.subscribe: geeft alleen de geklemde hz terug; het starten van de
# achtergrond-pushthread gebeurt in Handler.handle() (na het schrijven van dit
# antwoord, zie daemon.py) en wordt dus hier niet getest -- dat vereist een
# echte socketverbinding, zie mutod_ipc_live_check.py.
resp = rpc(d, ipc_types.ROBOT_SUBSCRIBE, {"hz": 999})
check("subscribe -> hz geklemd op max", resp["result"]["hz"] == ipc_types.SUBSCRIBE_MAX_HZ)
check("subscribe -> accepted", resp["result"]["accepted"] is True)

# -- onbekende methode --
resp = rpc(d, "robot.headbutt")
check("onbekende methode -> METHOD_NOT_FOUND", resp.get("error", {}).get("code") == ipc_types.METHOD_NOT_FOUND)

# -- Fase 2's oude {"op": ...}-kanaal blijft apart werken --
old_resp = d._handle_command({"op": "get_state"})
check("Fase 2 op-kanaal ongewijzigd (get_state)", old_resp.get("ok") is True and "angles" in old_resp)

print(f"\n{'ALLES OK' if failures == 0 else f'{failures} FOUT(EN)'}")
sys.exit(1 if failures else 0)
