# Muto Companion

Software companion-laag voor de Yahboom Muto RS hexapod (Raspberry Pi 5 + STM32F103RCT6-baseboard, ROS2 Humble), geinspireerd op — niet gekopieerd van — Pollen Robotics' Microduck-architectuur.

Volledige status/geschiedenis: [docs/muto_companion_overdracht.md](docs/muto_companion_overdracht.md).

## Structuur

- `mutod/` — kern-daemon, enige eigenaar van `/dev/myserial`. Protocol, HAL, veiligheid (deadman), IPC (duck-ipc-proto JSON-RPC), UDP-discovery, gedrags-state-machine.
- `drivers/` — bewegingsaansturing: `phoenix_driver.py` (Nav2/cmd_vel, PhoenixGait tripod-engine), `phoenix_gait.py` (gait-wiskunde), `mutod_client.py` (TCP-client + serial-shim voor gebruik binnen de ROS2-container).
- `perception/` — waarneming: voetcontact, gezichtsdetectie+volgen, LiDAR/dieptecamera-obstakelschatting, front-conventie.
- `navigation/` — `wander_executor.py`, de novelty-grid-wandelroutine die perceptie+gedrag+beweging combineert.
- `skills/` — YOLO-objectdetectie-pijplijn (Pi-camera -> Jetson Orin Nano -> afstand+buzzer).
- `rl/` — Fase 7: MJCF-modelbouw voor MuJoCo-simulatie, op basis van Yahboom's eigen kinematica-broncode (niet CAD).
- `tests/` — offline en live zelftests per laag.
- `docs/` — de volledige, doorlopend bijgewerkte roadmap/statusbriefing.

Buiten scope van deze push: eenmalige kalibratie-/diagnose-/patch-scripts uit juni-augustus 2026 (blijven lokaal op de Pi/WSL2-machine staan, niet onderdeel van de actieve architectuur).
