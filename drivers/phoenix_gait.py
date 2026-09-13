#!/usr/bin/env python3
"""
phoenix_gait.py — Phoenix-style gait engine voor Yahboom Muto RS
Reconstructie: continu fasemodel (50Hz) + sinusoïdale easing +
tripod/ripple/wave/centipede + biologische verbeteringen + HW-interpolatie.
"""

import sys
import math
import time
import argparse
import serial
from dataclasses import dataclass, field
from typing import List

# ---------------------------------------------------------------------------
# Geometrie
# ---------------------------------------------------------------------------
COXA_A    = 27.5
COXA_B    = 50.59
COXA_LEN  = COXA_A + COXA_B
FEMUR_LEN = 72.60
TIBIA_LEN = 134.5

MOUNT_DEG = [-45.0, 0.0, 45.0, 135.0, 180.0, 225.0]
MOUNT_RAD = [math.radians(d) for d in MOUNT_DEG]

# Leg-index: 0=RF, 1=RM, 2=RR, 3=LR, 4=LM, 5=LF
LEG_NAMES = ["RF", "RM", "RR", "LR", "LM", "LF"]

from MutoLib import (p1_x, p1_y, p1_z, p2_x, p2_y, p2_z,
                      p3_x, p3_y, p3_z, p4_x, p4_y, p4_z,
                      p5_x, p5_y, p5_z, p6_x, p6_y, p6_z)
NEUTRAL_POS = [(p1_x,p1_y,p1_z),(p2_x,p2_y,p2_z),(p3_x,p3_y,p3_z),
               (p4_x,p4_y,p4_z),(p5_x,p5_y,p5_z),(p6_x,p6_y,p6_z)]
print("[INFO] Neutrale posities geladen uit MutoLib")


def ease(t: float) -> float:
    """Sinusoidale easing: langzame start/stop, snel in het midden."""
    t = max(0.0, min(1.0, t))
    return 0.5 - 0.5 * math.cos(math.pi * t)


# ---------------------------------------------------------------------------
# Gait-definities (continu fasemodel: elke poot heeft een offset in [0,1))
# ---------------------------------------------------------------------------
@dataclass
class GaitParams:
    name: str
    cycle_time_s: float      # tijd voor één volledige gaitcyclus
    duty_factor: float       # fractie van cyclus in stance (0-1)
    leg_offsets: List[float] # fase-offset per poot, in [0,1)
    lift_height: float       # mm
    step_length: float       # mm

    @property
    def swing_frac(self) -> float:
        return 1.0 - self.duty_factor


# Tripod: Groep A (LF+RM+LR = 5,1,3) offset 0.0, Groep B (RF+LM+RR = 0,4,2) offset 0.5
TRIPOD   = GaitParams("tripod",   1.6, 0.5,
                       [0.5, 0.0, 0.5, 0.0, 0.5, 0.0], 40.0, 60.0)

# Ripple: 3 groepen van 2 poten, 1/3 fase uit elkaar
RIPPLE   = GaitParams("ripple",   2.4, 2/3,
                       [0.0, 2/3, 1/3, 0.0, 2/3, 1/3], 35.0, 50.0)

# Wave: metachronaal, elke poot een eigen 1/6e fase-slot, 5 poten altijd aan de grond
WAVE     = GaitParams("wave",     3.6, 5/6,
                       [0.0, 1/6, 2/6, 3/6, 4/6, 5/6], 30.0, 40.0)

# Centipede: metachronale golf, RR->LR->RM->LM->RF->LF
# LET OP: offsets hieronder zijn NIET herbevestigd op hardware sinds reconstructie — valideren.
CENTIPEDE = GaitParams("centipede", 3.6, 5/6,
                        [4/6, 2/6, 0/6, 1/6, 5/6, 3/6], 30.0, 30.0)

GAITS = {"tripod": TRIPOD, "ripple": RIPPLE, "wave": WAVE, "centipede": CENTIPEDE}


# ---------------------------------------------------------------------------
# Gait-engine
# ---------------------------------------------------------------------------
class PhoenixGait:
    def __init__(self):
        self._speed_ramp = 0.0   # huidige effectieve travel-schaal (accel/decel)

    def foot_targets(self, global_phase: float, gait: GaitParams,
                      travel_x: float = 1.0, travel_z: float = 0.0,
                      rotate: float = 0.0, target_speed: float = 1.0,
                      body_sway: bool = True, body_dip: bool = True):
        # --- Acceleratie/deceleratie: ramp de effectieve travel-schaal ---
        if target_speed > self._speed_ramp:
            self._speed_ramp = min(target_speed, self._speed_ramp + 0.05)   # lineaire ramp
        else:
            self._speed_ramp += (target_speed - self._speed_ramp) * 0.15   # exp. decay, alpha=0.85

        eff_travel = self._speed_ramp

        positions = []
        swing_ts = []
        for leg in range(6):
            phase = (global_phase + gait.leg_offsets[leg]) % 1.0
            if phase < gait.swing_frac:
                t = ease(phase / gait.swing_frac)
                pos = self._swing(leg, t, gait, travel_x * eff_travel,
                                   travel_z * eff_travel, rotate * eff_travel)
                swing_ts.append(t)
            else:
                t = ease((phase - gait.swing_frac) / gait.duty_factor)
                pos = self._stance(leg, t, gait, travel_x * eff_travel,
                                    travel_z * eff_travel, rotate * eff_travel)
            positions.append(pos)

        # --- Snelheidsafhankelijke lift-hoogte ---
        f_min = 0.4
        eff_lift_scale = max(f_min, eff_travel)

        # --- Body dip tijdens swing ---
        dip_z = 0.0
        if body_dip and swing_ts:
            avg_t = sum(swing_ts) / len(swing_ts)
            dip_z = -3.0 * math.sin(math.pi * avg_t)   # D=3mm, aanpasbaar

        positions = [(x, y, z + dip_z) for x, y, z in positions]

        if body_sway:
            positions = self._body_sway(global_phase, gait, positions)

        return positions, eff_lift_scale

    def _foot_delta(self, leg, gait, tx, tz, rot):
        nx, ny, _ = NEUTRAL_POS[leg]
        dx = gait.step_length * tz
        dy = gait.step_length * tx
        rot_scale = gait.step_length * 0.6
        rx = -ny * rot * rot_scale / max(math.hypot(nx, ny), 1)
        ry =  nx * rot * rot_scale / max(math.hypot(nx, ny), 1)
        return (dx + rx, dy + ry)

    def _swing(self, leg, t, gait, tx, tz, rot):
        nx, ny, nz = NEUTRAL_POS[leg]
        ddx, ddy = self._foot_delta(leg, gait, tx, tz, rot)
        return (nx + (t - 0.5) * ddx, ny + (t - 0.5) * ddy,
                nz + math.sin(math.pi * t) * gait.lift_height)

    def _stance(self, leg, t, gait, tx, tz, rot):
        nx, ny, nz = NEUTRAL_POS[leg]
        ddx, ddy = self._foot_delta(leg, gait, tx, tz, rot)
        return (nx + (0.5 - t) * ddx, ny + (0.5 - t) * ddy, nz)

    def _body_sway(self, global_phase, gait, positions):
        stance_idx = [leg for leg in range(6)
                      if (global_phase + gait.leg_offsets[leg]) % 1.0 >= gait.swing_frac]
        if not stance_idx:
            return positions
        cx = sum(positions[i][0] for i in stance_idx) / len(stance_idx)
        cy = sum(positions[i][1] for i in stance_idx) / len(stance_idx)
        SWAY = 0.12
        return [(x - cx * SWAY, y - cy * SWAY, z) for x, y, z in positions]


# ---------------------------------------------------------------------------
# Hardware-interface
# ---------------------------------------------------------------------------
def set_exec_time(ser: serial.Serial, ms: int):
    """Zet STM32-servo-executietijd (register 0x2C, 2-byte, big-endian ms)."""
    if ms <= 0:
        return
    hi = (ms >> 8) & 0xFF
    lo = ms & 0xFF
    length = 0x0A
    wr = 0x01
    addr = 0x2C
    checksum = (0xFF - (length + wr + addr + hi + lo)) & 0xFF
    packet = bytes([0x55, 0x00, length, wr, addr, hi, lo, checksum, 0x00, 0xAA])
    ser.write(packet)


def _flush_if_batched(ser):
    """Als ser een gebufferde shim is (bv. MutodSerialShim, zie
    mutod_client.py) i.p.v. een echte serial.Serial, stuurt dit de
    geaccumuleerde writes als een batch. Geen-op voor een echte seriele poort
    (die schrijft direct, per write())."""
    flush = getattr(ser, "flush_batch", None)
    if flush is not None:
        flush()


class HardwareInterface:
    """Stuurt echte servo's via MutoLib Leg.move_tip().

    Standaard opent dit zijn eigen serial.Serial (voor losse CLI-tests, zie
    main() onderaan dit bestand). Geef `ser` mee om i.p.v. daarvan een reeds
    bestaande verbinding te hergebruiken -- phoenix_driver.py doet dit met een
    MutodSerialShim (mutod_client.py), zodat mutod de enige eigenaar van
    /dev/myserial blijft (Fase 1 briefing §2, architectuurbeslissing 1)."""

    def __init__(self, port='/dev/myserial', exec_time_ms=18, ser=None):
        from MutoLib import Servo, Leg, point3d
        self._point3d = point3d
        self._ser = ser if ser is not None else serial.Serial(port, 115200, timeout=0.1)
        srv = Servo(self._ser)
        self._legs = [Leg(i, srv) for i in range(6)]

        if exec_time_ms > 0:
            set_exec_time(self._ser, exec_time_ms)
            _flush_if_batched(self._ser)
            print(f"[HW] Servo-executietijd ingesteld op {exec_time_ms}ms")

        print("[HW] Naar neutrale stand...")
        for i, leg in enumerate(self._legs):
            leg.move_tip(point3d(*NEUTRAL_POS[i]))
        _flush_if_batched(self._ser)
        time.sleep(1.0)
        print(f"[HW] MutoLib geladen, 6 poten op {port}")

    def send(self, positions):
        for i, (x, y, z) in enumerate(positions):
            self._legs[i].move_tip(self._point3d(x, y, z))
        _flush_if_batched(self._ser)

    def destroy(self):
        if hasattr(self, "_ser") and self._ser and self._ser.is_open:
            set_exec_time(self._ser, 0)   # interpolatie uit bij stoppen
            _flush_if_batched(self._ser)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def test_neutral(iface):
    print("[TEST] neutral — Ctrl-C om te stoppen")
    while True:
        iface.send(list(NEUTRAL_POS))
        time.sleep(0.05)


def test_one_leg(iface, gait, cycles=4):
    print(f"[TEST] one_leg — {cycles} cycli")
    dt = 0.05
    for _ in range(cycles):
        for step in range(20):
            t = step / 20
            positions = list(NEUTRAL_POS)
            nx, ny, nz = NEUTRAL_POS[0]
            positions[0] = (nx, ny, nz + math.sin(math.pi * t) * gait.lift_height)
            iface.send(positions)
            time.sleep(dt)
        time.sleep(0.3)


def test_gait(iface, gait, engine, sway=False, dip=True, hz=50):
    print(f"[TEST] {gait.name} — cycle={gait.cycle_time_s:.1f}s "
          f"lift={gait.lift_height:.0f}mm sway={'aan' if sway else 'uit'} — Ctrl-C om te stoppen")
    dt = 1.0 / hz
    global_phase = 0.0
    while True:
        positions, _ = engine.foot_targets(global_phase, gait,
                                            travel_x=1.0, target_speed=1.0,
                                            body_sway=sway, body_dip=dip)
        iface.send(positions)
        time.sleep(dt)
        global_phase = (global_phase + dt / gait.cycle_time_s) % 1.0


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    p = argparse.ArgumentParser()
    p.add_argument('--test', choices=['neutral', 'one_leg', 'gait'], default='gait')
    p.add_argument('--gait', choices=list(GAITS.keys()), default='tripod')
    p.add_argument('--lift', type=float, default=None)
    p.add_argument('--step-length', type=float, default=None)
    p.add_argument('--cycle-time', type=float, default=None)
    p.add_argument('--sway', action='store_true')
    p.add_argument('--dip', action='store_true')
    p.add_argument('--cycles', type=int, default=4)
    p.add_argument('--exec-time', type=int, default=18,
                    help='STM32 servo-executietijd in ms, 0=uitgeschakeld')
    p.add_argument('--port', default='/dev/myserial')
    args = p.parse_args()

    iface = HardwareInterface(port=args.port, exec_time_ms=args.exec_time)
    gait = GAITS[args.gait]
    if args.lift:        gait.lift_height = args.lift
    if args.step_length: gait.step_length = args.step_length
    if args.cycle_time:  gait.cycle_time_s = args.cycle_time
    engine = PhoenixGait()

    print("=" * 50)
    print(f"  Phoenix Gait | {gait.name} | lift={gait.lift_height:.0f}mm | "
          f"cycle={gait.cycle_time_s:.1f}s | exec_time={args.exec_time}ms")
    print("=" * 50)

    try:
        if args.test == 'neutral':
            test_neutral(iface)
        elif args.test == 'one_leg':
            test_one_leg(iface, gait, args.cycles)
        else:
            test_gait(iface, gait, engine, args.sway, args.dip)
    except KeyboardInterrupt:
        print("\n[INFO] Gestopt.")
    finally:
        iface.destroy()


if __name__ == '__main__':
    main()