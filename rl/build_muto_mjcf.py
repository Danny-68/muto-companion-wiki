#!/usr/bin/env python3
"""build_muto_mjcf.py -- genereert Muto's MJCF-model voor Fase 7 RL-training.

Alle geometrie komt rechtstreeks uit Yahboom's eigen, echte kinematica-code
(muto_hexapod_lib/core/config.py + movement/kinematicLib.py, gevonden op de
WSL2-machine, 13 sep 2026) -- GEEN CAD-decompositie, GEEN geschatte metingen.
Zie muto_fase7_step_findings_2026-09-13-memory voor waarom de CAD-route is
losgelaten.

Wat WEL een aanname/benadering is (Yahboom's config geeft dit niet):
- Chassis-afmetingen/massa (ruwe schatting, geen echte meting).
- Capsule-diameter voor elk pootsegment (structurele benadering, geen exacte
  Yahboom-maat -- de servo's zelf zijn wel exact: 20x40x40.5mm, 61.5g elk,
  uit de "35KG steering gear specifications"-datasheet).
- Massaverdeling: elke servo (61.5g) wordt toegekend aan het segment dat hij
  aandrijft, een gangbare vereenvoudiging voor RL-sim (microduck_rl doet dit
  ook, zie muto_microduck_watchlist_check_2026-09-11-memory).
"""
import math

# -- Yahboom's eigen, echte kinematica-constanten (mm) --------------------
LEG_MOUNT_LR_X = 53.4      # midden-poten (RM/LM)
LEG_MOUNT_OTHER_X = 31.12  # voor/achter-poten
LEG_MOUNT_OTHER_Y = 83.07

LEG_ROOT2JOINT1 = 27.5    # mount -> coxa-as offset
LEG_COXA = 50.59          # joint1 -> joint2
LEG_FEMUR = 72.60         # joint2 -> joint3
LEG_TIBIA = 134.5         # joint3 -> voet

# Volgorde/mounthoeken exact zoals config.py's mount_position/default_angle-
# tuples (index0..5) -- komt overeen met RF/RM/RR/LR/LM/LF per de al-
# bevestigde protocol.py LEG_SERVO_IDS-volgorde (Fase 1).
LEGS = [
    {"name": "RF", "servo_ids": (1, 2, 3), "mount": (LEG_MOUNT_OTHER_X, LEG_MOUNT_OTHER_Y), "angle_deg": -45},
    {"name": "RM", "servo_ids": (4, 5, 6), "mount": (LEG_MOUNT_LR_X, 0), "angle_deg": 0},
    {"name": "RR", "servo_ids": (7, 8, 9), "mount": (LEG_MOUNT_OTHER_X, -LEG_MOUNT_OTHER_Y), "angle_deg": 45},
    {"name": "LR", "servo_ids": (10, 11, 12), "mount": (-LEG_MOUNT_OTHER_X, -LEG_MOUNT_OTHER_Y), "angle_deg": 135},
    {"name": "LM", "servo_ids": (13, 14, 15), "mount": (-LEG_MOUNT_LR_X, 0), "angle_deg": 180},
    {"name": "LF", "servo_ids": (16, 17, 18), "mount": (-LEG_MOUNT_OTHER_X, LEG_MOUNT_OTHER_Y), "angle_deg": 225},
]

# Gewrichtslimieten (graden), exact uit config.py's angleLimitation
COXA_RANGE = (-45, 45)
FEMUR_RANGE = (-45, 75)
TIBIA_RANGE = (-60, 60)

# -- Benaderingen (GEEN Yahboom-brongegevens, hier expliciet gemarkeerd) ---
SERVO_MASS_KG = 0.0615  # 61.5g, WEL uit de echte servo-datasheet
SEGMENT_RADIUS_M = 0.007  # structurele benadering
# LET OP (gevonden 13 sep 2026, via visuele controle door de gebruiker):
# de lange as van het chassis moet in Y liggen, niet X -- de echte mount-
# posities (LEG_MOUNT_OTHER_Y=83.07mm voor/achter vs LEG_MOUNT_LR_X=53.4mm
# midden) laten zien dat de poten verder uit elkaar staan in Y (voor/achter)
# dan in X (links/rechts). Eerdere versie had dit per ongeluk omgedraaid.
CHASSIS_LENGTH_Y_M = 0.19   # voor-achter, ruwe benadering, nog niet fysiek geverifieerd
CHASSIS_WIDTH_X_M = 0.12    # links-rechts
CHASSIS_HEIGHT_M = 0.045
CHASSIS_MASS_KG = 0.9     # ruwe schatting (frame+elektronica+accu), NIET gemeten

MM = 0.001  # mm -> m


def leg_xml(leg):
    name = leg["name"]
    mx, my = leg["mount"][0] * MM, leg["mount"][1] * MM
    angle = leg["angle_deg"]

    return f"""
    <body name="{name}_coxa_mount" pos="{mx:.5f} {my:.5f} 0" euler="0 0 {angle}">
      <body name="{name}_coxa" pos="{LEG_ROOT2JOINT1*MM:.5f} 0 0">
        <joint name="{name}_coxa_joint" type="hinge" axis="0 0 1" range="{COXA_RANGE[0]} {COXA_RANGE[1]}" pos="0 0 0"/>
        <inertial pos="0 0 0" mass="{SERVO_MASS_KG}" diaginertia="0.00002 0.00002 0.00002"/>
        <geom name="{name}_coxa_geom" type="capsule" fromto="0 0 0 {LEG_COXA*MM:.5f} 0 0" size="{SEGMENT_RADIUS_M}" rgba="0.7 0.7 0.75 1"/>
        <body name="{name}_femur" pos="{LEG_COXA*MM:.5f} 0 0">
          <joint name="{name}_femur_joint" type="hinge" axis="0 1 0" range="{FEMUR_RANGE[0]} {FEMUR_RANGE[1]}" pos="0 0 0"/>
          <inertial pos="0 0 0" mass="{SERVO_MASS_KG}" diaginertia="0.00003 0.00003 0.00003"/>
          <geom name="{name}_femur_geom" type="capsule" fromto="0 0 0 {LEG_FEMUR*MM:.5f} 0 0" size="{SEGMENT_RADIUS_M}" rgba="0.75 0.75 0.8 1"/>
          <body name="{name}_tibia" pos="{LEG_FEMUR*MM:.5f} 0 0">
            <joint name="{name}_tibia_joint" type="hinge" axis="0 1 0" range="{TIBIA_RANGE[0]} {TIBIA_RANGE[1]}" pos="0 0 0"/>
            <inertial pos="0 0 0" mass="{SERVO_MASS_KG}" diaginertia="0.00006 0.00006 0.00006"/>
            <geom name="{name}_tibia_geom" type="capsule" fromto="0 0 0 {LEG_TIBIA*MM:.5f} 0 0" size="{SEGMENT_RADIUS_M*0.8:.4f}" rgba="0.3 0.3 0.35 1"/>
            <site name="{name}_foot" pos="{LEG_TIBIA*MM:.5f} 0 0" size="0.005"/>
          </body>
        </body>
      </body>
    </body>"""


# Yahboom's eigen berekende sta-houding (zie sessie: k_standby + hun ik(),
# identiek voor alle 6 poten dankzij de radiale symmetrie van de neutrale stand).
STANDING_COXA_DEG, STANDING_FEMUR_DEG, STANDING_TIBIA_DEG = 0.0, 29.9, -14.8
STANDING_TORSO_Z_M = 0.10


def build_mjcf():
    legs_xml = "\n".join(leg_xml(leg) for leg in LEGS)

    qpos_vals = [0, 0, STANDING_TORSO_Z_M, 1, 0, 0, 0]  # freejoint: x,y,z,qw,qx,qy,qz
    ctrl_vals = []
    for _ in LEGS:
        for deg in (STANDING_COXA_DEG, STANDING_FEMUR_DEG, STANDING_TIBIA_DEG):
            qpos_vals.append(math.radians(deg))
            ctrl_vals.append(deg)
    standing_qpos_xml = " ".join(f"{v:.6f}" for v in qpos_vals)
    standing_ctrl_xml = " ".join(f"{v:.2f}" for v in ctrl_vals)

    actuators = []
    for leg in LEGS:
        for joint in ("coxa", "femur", "tibia"):
            actuators.append(
                f'    <position name="{leg["name"]}_{joint}_act" joint="{leg["name"]}_{joint}_joint" '
                f'kp="20" ctrlrange="-90 90"/>'
            )
    actuators_xml = "\n".join(actuators)

    return f"""<mujoco model="muto_rs">
  <compiler angle="degree" autolimits="true"/>
  <option timestep="0.002" gravity="0 0 -9.81"/>

  <default>
    <joint damping="0.2" armature="0.001"/>
    <!-- condim=4 (i.p.v. 3): telt torsiewrijving mee, niet alleen de
         tangentiale glijwrijving -- zonder dit hebben de bijna-punt-vormige
         capsule-voettippen geen enkele weerstand tegen ronddraaien op de
         grond, wat de robot tijdens het settelen liet ronddraaien om zijn
         eigen as (gevonden 13 sep 2026, zie muto_fase7-memory). -->
    <geom friction="1.0 0.3 0.01" condim="4"/>
  </default>

  <worldbody>
    <light directional="true" pos="0 0 3" dir="0 0 -1"/>
    <geom name="floor" type="plane" size="5 5 0.1" rgba="0.5 0.5 0.5 1"/>

    <body name="torso" pos="0 0 0.15">
      <freejoint name="root"/>
      <inertial pos="0 0 0" mass="{CHASSIS_MASS_KG}" diaginertia="0.004 0.006 0.008"/>
      <geom name="chassis_geom" type="box" size="{CHASSIS_WIDTH_X_M/2:.4f} {CHASSIS_LENGTH_Y_M/2:.4f} {CHASSIS_HEIGHT_M/2:.4f}" rgba="0.2 0.5 0.8 1"/>
      <site name="imu_site" pos="0 0 0"/>
{legs_xml}
    </body>
  </worldbody>

  <actuator>
{actuators_xml}
  </actuator>

  <keyframe>
    <key name="standing" qpos="{standing_qpos_xml}" ctrl="{standing_ctrl_xml}"/>
  </keyframe>
</mujoco>
"""


if __name__ == "__main__":
    xml = build_mjcf()
    with open("muto_rs.xml", "w") as f:
        f.write(xml)
    print("Geschreven: muto_rs.xml")
    print(f"{len(LEGS)} poten x 3 gewrichten = {len(LEGS)*3} actuators")
