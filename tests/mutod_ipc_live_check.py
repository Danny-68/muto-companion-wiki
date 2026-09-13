#!/usr/bin/env python3
"""Live acceptatietest tegen de ECHTE, draaiende mutod-daemon over TCP, met
exacte duck-ipc-proto JSON-RPC 2.0-berichten (methode-namen/vorm uit
joeynyc/microduck-mcp's protocol.ts). Test bewust alleen niet-bewegende
methodes (robot.health, robot.state, robot.subscribe, robot.stop) -- een
live robot.move-test (echte gait-beweging via een tot nu toe ongeteste STM32-
firmware-gait-weg) is een aparte, expliciet aan te kondigen stap, geen
onderdeel van deze build-verificatie."""
import json
import socket
import sys
import time

HOST, PORT = "127.0.0.1", 8420


def call(sock, rfile, method, params=None, req_id=1):
    req = {"jsonrpc": "2.0", "id": req_id, "method": method}
    if params is not None:
        req["params"] = params
    sock.sendall((json.dumps(req) + "\n").encode())
    line = rfile.readline()
    return json.loads(line)


def main():
    sock = socket.create_connection((HOST, PORT), timeout=5)
    rfile = sock.makefile("rb")

    resp = call(sock, rfile, "robot.health", req_id=1)
    print("robot.health ->", resp)
    ok = "result" in resp and "healthy" in resp["result"]
    print("[OK  ]" if ok else "[FOUT]", "robot.health geeft geldig resultaat")
    failures = 0 if ok else 1

    resp = call(sock, rfile, "robot.state", req_id=2)
    print("robot.state ->", {k: resp["result"][k] for k in ("t", "loop")} if "result" in resp else resp)
    ok = "result" in resp and len(resp["result"].get("servos", {})) == 18
    print("[OK  ]" if ok else "[FOUT]", "robot.state geeft 18 servo-hoeken")
    failures += 0 if ok else 1

    resp = call(sock, rfile, "robot.subscribe", {"hz": 5}, req_id=3)
    print("robot.subscribe ->", resp)
    ok = resp.get("result", {}).get("accepted") is True
    print("[OK  ]" if ok else "[FOUT]", "robot.subscribe geaccepteerd")
    failures += 0 if ok else 1

    notifications = []
    sock.settimeout(2.0)
    deadline = time.time() + 1.5
    while time.time() < deadline:
        try:
            line = rfile.readline()
        except socket.timeout:
            break
        if not line:
            break
        msg = json.loads(line)
        if msg.get("method") == "robot.state":
            notifications.append(msg)

    ok = len(notifications) >= 2
    print(f"[OK  ]" if ok else "[FOUT]", f"robot.state-notificaties ontvangen ({len(notifications)}, verwacht >=2 bij 5Hz/1.5s)")
    failures += 0 if ok else 1

    resp = call(sock, rfile, "robot.stop", req_id=4)
    print("robot.stop ->", resp)
    ok = resp.get("result", {}).get("accepted") is True
    print("[OK  ]" if ok else "[FOUT]", "robot.stop geaccepteerd")
    failures += 0 if ok else 1

    sock.close()
    print(f"\n{'ALLES OK' if failures == 0 else f'{failures} FOUT(EN)'} -- robot.move bewust NIET live getest, zie docstring")
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
