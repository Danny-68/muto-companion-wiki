#!/usr/bin/env python3
"""
phoenix_driver.py — Nav2-bewegingsbackend op basis van PhoenixGait (tripod-only)
i.p.v. de STM32-firmware-gait (0x12-0x17) die muto_driver_fixed.py aanstuurt.

Vervangt muto_driver_fixed.py NIET — dat bestand blijft intact als terugval-
optie. Dit is een apart te starten/stoppen ROS2-node die op hetzelfde topic
('cmd_vel', gevoed via de bestaande `ros2 run topic_tools relay /cmd_vel_nav
/cmd_vel`) abonneert, zodat de rest van de Nav2-stack ongewijzigd blijft.

Nog NIET in de Nav2-launch-keten gehaakt — dit bestand is voor stap 1
(zelfstandig bouwen en los testen), per expliciete instructie.

--------------------------------------------------------------------------
KALIBRATIE-STATUS — zie ook PROBLEMS.md:

  MAX_LINEAR_SPEED_MPS    — geschat uit gemeten ~9.5cm/cyclus bij
                             cycle_time_s=1.6s (phoenix_yaw_drift_test.py,
                             10 aug 2026). Nooit apart gevalideerd bij
                             gedeeltelijke (niet-volle) snelheid.
  MAX_ANGULAR_SPEED_RADPS — HERKALIBREERD 12 sep 2026 via /imu_stm32
                             (onboard STM32-IMU, niet de externe ICM20948 --
                             zie muto_amcl_localization_procedure-memory over
                             onbetrouwbare externe IMU) met phoenix_driver.py
                             zelf als bewegingsbron (n=5, vaste duren 5/5/8/8/8s):
                             0.1290, 0.1275, 0.1297, 0.1240, 0.1273 rad/s --
                             zeer consistent. Gemiddelde 0.1275 rad/s.
                             De vorige 10-aug-waarde (0.055 rad/s, via de
                             externe IMU) bleek ~2.3x te laag: ontdekt doordat
                             een 2-cirkel-localisatie-spin (op basis van de
                             oude waarde becijferd op ~256s) in de praktijk
                             beduidend meer dan 4 volle rotaties bleek te
                             maken. Les: de externe IMU faalde hier niet
                             alleen bij linksom-draaien (eerdere bevinding),
                             maar leverde blijkbaar ook een verkeerde snelheid
                             op in deze eerdere kalibratie -- de onboard
                             STM32-IMU is nu de vertrouwde bron voor dit soort
                             metingen.
--------------------------------------------------------------------------
"""
import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool

sys.path.insert(0, '/root')
from phoenix_gait import PhoenixGait, HardwareInterface, GAITS, NEUTRAL_POS, ease
from mutod_client import MutodClient, MutodSerialShim

SERIAL_PORT = '/dev/myserial'  # nog gebruikt in commentaar/logs; de daadwerkelijke
                                # verbinding loopt nu via mutod (MUTOD_HOST/MUTOD_PORT)
MUTOD_HOST = '127.0.0.1'
MUTOD_PORT = 8420

# STM32-onboard-yaw (zelfde protocol als robot_bridge.py, "stabiel binnen
# ~0.1 graden in rust", bewezen betrouwbaar voor rotatie -- i.t.t. rf2o, dat
# op 11 aug 2026 een consistente ~2.4-2.8x overschatting bleek te geven
# tijdens draaien, zie PROBLEMS.md). Loopt sinds de mutod-koppeling (11 sep
# 2026) via mutod's read_attitude-op i.p.v. een eigen raw serial-read -- mutod
# is nu de enige eigenaar van /dev/myserial, dus geen eigen verbinding meer.
# LET OP (11 aug 2026, nog steeds relevant): op 10Hz interfereerde dit
# merkbaar met de servo-aansturing op dezelfde seriele poort -- gebruiker zag
# nog maar "heel weinig" beweging. Verlaagd naar 2Hz. mutod's eigen
# achtergrond-telemetrie (read_all_angles) valt om dezelfde reden terug naar
# 2Hz zodra er bewegingscommando's binnenkomen, zie mutod/daemon.py.
STM32_IMU_HZ = 2
HZ = 50
DT = 1.0 / HZ

# -- zie "KALIBRATIE-STATUS" hierboven --
MAX_LINEAR_SPEED_MPS = 0.0594
MAX_ANGULAR_SPEED_RADPS = 0.1275  # herkalibreerd 12 sep 2026, n=5, zie boven

CMD_TIMEOUT_S = 5.0  # zelfde als de huidige live waarde in muto_driver_fixed.py
CMD_DEADBAND = 0.02  # onder deze genormaliseerde waarde tellen we als "stil"
DECEL_DURATION_S = 1.0
NEUTRAL_DURATION_S = 1.0  # TEST 11 aug 2026: 0.3s (3x sneller) geprobeerd om de
                          # ~15-25s fysieke naslinger te verkorten -- geen
                          # aantoonbaar effect (settle_curve_test.py, zelfde
                          # convergentietijd), teruggezet naar 1.0s.
# Live Nav2-test (10 aug 2026) toonde dat de controller vaak kort onder de
# deadband duikt tijdens normale bijsturing (~elke 2.5-3s een dip) -- zonder
# debounce triggerde dat elke keer de volledige (2s, blokkerende) stop-
# sequentie, waardoor de robot nooit vooruitkwam. Pas als "stil" langer dan
# deze tijd aanhoudt, behandelen we het als een echte stop.
STOP_DEBOUNCE_S = 0.5

# Na een stop blijft het lichaam ~15-25s fysiek naslingeren (dempende
# oscillatie, bevestigd via settle_curve_test.py, zie PROBLEMS.md) -- twee
# pogingen om de stopsequentie zelf te versnellen/verzachten gaven geen
# aantoonbaar effect. In plaats daarvan: een "pose_settling"-vlag publiceren
# zodat AMCL/Nav2 de pose in dit venster als verhoogd-onzeker kan behandelen
# i.p.v. te wachten op een kortere naslinger die niet blijkt te bestaan.
# 24s is empirisch bepaald: bij alle drie geteste settle-curves was de
# afwijking vanaf 24s consistent binnen 2-6%, terwijl 18-21s nog een
# aangetoonde terugval liet zien.
SETTLING_DURATION_S = 24.0


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _require_mutod():
    """mutod is sinds 11 sep 2026 de enige eigenaar van /dev/myserial (Fase 1
    briefing §2, architectuurbeslissing 1) -- phoenix_driver.py opent de
    seriele poort niet meer zelf en stopt ook app_muto.py niet meer zelf
    (dat was voor de mutod-koppeling nodig, zie git-historie van dit bestand).
    In plaats daarvan: fail loud als mutod niet bereikbaar is, i.p.v. een
    tweede proces de poort te laten grijpen."""
    import socket as _socket
    try:
        with _socket.create_connection((MUTOD_HOST, MUTOD_PORT), timeout=2.0):
            pass
    except OSError as exc:
        print(
            f"[FOUT] mutod niet bereikbaar op {MUTOD_HOST}:{MUTOD_PORT} ({exc}). "
            f"Start eerst mutod (python3 -m mutod.daemon op de Pi-host) -- "
            f"phoenix_driver.py opent /dev/myserial niet meer zelf."
        )
        sys.exit(1)


class PhoenixDriver(Node):
    def __init__(self):
        super().__init__('phoenix_driver')
        self._lock = threading.Lock()

        self.mutod = MutodClient(MUTOD_HOST, MUTOD_PORT)
        shim = MutodSerialShim(self.mutod)
        self.iface = HardwareInterface(exec_time_ms=18, ser=shim)
        self.engine = PhoenixGait()
        self.gait = GAITS['tripod']  # UITSLUITEND tripod, per expliciete eis

        self.travel_x = 0.0
        self.travel_y = 0.0  # Fase 6 (12 sep 2026): zijwaarts, zie cb()/PROBLEMS.md-noot hieronder
        self.rotate = 0.0
        self.target_speed = 0.0
        self.state = 'idle'  # 'idle' | 'moving'
        self.global_phase = 0.0
        self.last_cmd = self.get_clock().now()
        self.zero_since = None
        self.settling_until = None  # None = niet aan het naslingeren; anders rclpy.Time

        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cb, 10)
        self.create_timer(0.3, self.timeout_check)
        self.create_timer(DT, self._step)

        self.imu_pub = self.create_publisher(Imu, 'imu_stm32', 10)
        self.create_timer(1.0 / STM32_IMU_HZ, self._publish_stm32_yaw)

        self.settling_pub = self.create_publisher(Bool, 'pose_settling', 10)
        self.create_timer(0.5, self._publish_settling)
        self.get_logger().info(
            'Phoenix driver gestart (tripod-only, PhoenixGait i.p.v. STM32-firmware-gait)')

    def cb(self, msg):
        self.last_cmd = self.get_clock().now()
        with self._lock:
            travel_x = clamp(msg.linear.x / MAX_LINEAR_SPEED_MPS, -1.0, 1.0)
            # Fase 6 (12 sep 2026): PhoenixGait.foot_targets() had al een losse
            # travel_z-as (zie de engine's eigen _foot_delta()) die tot nu toe
            # altijd hardcoded op 0.0 stond -- linear.y werd nooit gelezen. Geen
            # eigen laterale snelheidskalibratie beschikbaar; hergebruikt
            # voorlopig dezelfde MAX_LINEAR_SPEED_MPS als vooruit/achteruit,
            # tot een echte meting anders uitwijst.
            # Teken-conventie FYSIEK GEVERIFIEERD (12 sep 2026, phoenix_sideways_
            # test.py, direction=pos/travel_z=+1 -> robot-eigen RECHTS). Dat is
            # het omgekeerde van de standaard ROS-conventie (linear.y>0 = links)
            # -- daarom hier een expliciete min, zodat cmd_vel.linear.y wél de
            # standaard conventie volgt: linear.y>0 -> travel_z<0 -> robot-links.
            travel_y = clamp(-msg.linear.y / MAX_LINEAR_SPEED_MPS, -1.0, 1.0)
            rotate = clamp(msg.angular.z / MAX_ANGULAR_SPEED_RADPS, -1.0, 1.0)
            want_moving = (
                abs(travel_x) > CMD_DEADBAND or abs(travel_y) > CMD_DEADBAND or abs(rotate) > CMD_DEADBAND
            )
            self.travel_x = travel_x
            self.travel_y = travel_y
            self.rotate = rotate
            if want_moving:
                self.zero_since = None
                if self.state == 'idle':
                    self.state = 'moving'
                    self.target_speed = 1.0
            elif self.state == 'moving' and self.zero_since is None:
                # Zie STOP_DEBOUNCE_S -- niet meteen stoppen, eerst afwachten
                # of dit een kortstondige dip is (normaal Nav2-controllergedrag)
                # of een echte stop.
                self.zero_since = self.get_clock().now()

    def _check_stop_debounce(self):
        with self._lock:
            if self.state != 'moving' or self.zero_since is None:
                return
            elapsed = (self.get_clock().now() - self.zero_since).nanoseconds / 1e9
            should_stop = elapsed > STOP_DEBOUNCE_S
        if should_stop:
            self._do_stable_stop()

    def _step(self):
        if self.state != 'moving':
            return
        with self._lock:
            travel_x, travel_y, rotate, target_speed = self.travel_x, self.travel_y, self.rotate, self.target_speed
        # TEST 11 aug 2026 (sway uit tijdens pure rotatie) WEERLEGD -- ratio
        # bleef ~2.4x, vrijwel gelijk aan met sway aan. Teruggezet naar altijd
        # aan. Zie PROBLEMS.md voor de volledige rf2o-overschatting-bevinding.
        positions, _ = self.engine.foot_targets(
            self.global_phase, self.gait,
            travel_x=travel_x, travel_z=travel_y, rotate=rotate,
            target_speed=target_speed, body_sway=True, body_dip=True)
        self.iface.send(positions)
        self.global_phase = (self.global_phase + DT / self.gait.cycle_time_s) % 1.0

    def _publish_stm32_yaw(self):
        try:
            attitude = self.mutod.read_attitude()
        except (ConnectionError, RuntimeError) as exc:
            self.get_logger().warn(f'read_attitude mislukt: {exc}', throttle_duration_sec=5.0)
            return
        yaw_rad = math.radians(attitude["yaw_deg"])
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        msg.orientation.z = math.sin(yaw_rad / 2.0)
        msg.orientation.w = math.cos(yaw_rad / 2.0)
        # Alleen yaw vertrouwd (roll/pitch niet uitgelezen door dit commando);
        # covariantie klein voor yaw, groot voor de rest zodat EKF alleen yaw fuseert.
        msg.orientation_covariance[0] = 99999.0
        msg.orientation_covariance[4] = 99999.0
        msg.orientation_covariance[8] = 0.02
        msg.angular_velocity_covariance[0] = -1.0
        msg.linear_acceleration_covariance[0] = -1.0
        self.imu_pub.publish(msg)

    def _do_stable_stop(self):
        """Vloeiende stop: decel binnen de gait (target_speed -> 0) gevolgd
        door sinusoidale interpolatie naar NEUTRAL_POS. Zelfde patroon als
        stable_stop() in phoenix_yaw_drift_test.py, gegeneraliseerd naar
        gecombineerde travel_x+rotate (de referentie-implementatie test alleen
        pure vooruit/achteruit, geen gecombineerde beweging+draai)."""
        self.get_logger().info('Vloeiende stop-sequentie (decel + neutraal)')
        with self._lock:
            last_travel_x, last_travel_y, last_rotate = self.travel_x, self.travel_y, self.rotate

        # TEST 11 aug 2026 (sway tijdens decel uitfaseren) GEEN AANTOONBAAR
        # EFFECT op de naslinger -- teruggezet naar de oorspronkelijke,
        # normale sway/dip tijdens decel. Zie PROBLEMS.md voor de
        # settle_curve_test.py-resultaten van beide varianten.
        decel_steps = int(DECEL_DURATION_S * HZ)
        last_pos = None
        for _ in range(decel_steps):
            positions, _ = self.engine.foot_targets(
                self.global_phase, self.gait,
                travel_x=last_travel_x, travel_z=last_travel_y, rotate=last_rotate,
                target_speed=0.0, body_sway=True, body_dip=True)
            self.iface.send(positions)
            last_pos = positions
            time.sleep(DT)
            self.global_phase = (self.global_phase + DT / self.gait.cycle_time_s) % 1.0

        neutral_steps = int(NEUTRAL_DURATION_S * HZ)
        for s in range(1, neutral_steps + 1):
            t = ease(s / neutral_steps)
            interp = [(fx + (nx - fx) * t, fy + (ny - fy) * t, fz + (nz - fz) * t)
                      for (fx, fy, fz), (nx, ny, nz) in zip(last_pos, NEUTRAL_POS)]
            self.iface.send(interp)
            time.sleep(DT)

        with self._lock:
            self.travel_x = 0.0
            self.travel_y = 0.0
            self.rotate = 0.0
            self.target_speed = 0.0
            self.state = 'idle'
            self.zero_since = None
            self.settling_until = self.get_clock().now() + Duration(seconds=SETTLING_DURATION_S)

    def _publish_settling(self):
        settling = False
        with self._lock:
            if self.settling_until is not None:
                if self.get_clock().now() < self.settling_until:
                    settling = True
                else:
                    self.settling_until = None
        self.settling_pub.publish(Bool(data=settling))

    def timeout_check(self):
        elapsed = (self.get_clock().now() - self.last_cmd).nanoseconds / 1e9
        if elapsed > CMD_TIMEOUT_S and self.state == 'moving':
            self.get_logger().info(f'Timeout ({CMD_TIMEOUT_S}s zonder nieuw cmd_vel)')
            self._do_stable_stop()
            return
        self._check_stop_debounce()

    def destroy_node(self):
        if self.state == 'moving':
            self._do_stable_stop()
        self.iface.destroy()
        super().destroy_node()


def main():
    _require_mutod()
    rclpy.init()
    node = PhoenixDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
