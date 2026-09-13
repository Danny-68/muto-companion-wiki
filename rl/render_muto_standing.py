#!/usr/bin/env python3
"""render_muto_standing.py -- laadt muto_rs.xml, stuurt de actuators naar
Yahboom's eigen sta-houding (coxa=0, femur=+30, tibia=-15 graden, berekend
via hun eigen ik()-functie + k_standby-doelposities), simuleert het settelen,
en rendert een video zodat de gebruiker kan meekijken (geen live GUI nodig,
werkt ongeacht WSLg-scherminstelling)."""
import mujoco
import numpy as np
import imageio.v2 as imageio

model = mujoco.MjModel.from_xml_path("/home/meinds/muto_rs.xml")
data = mujoco.MjData(model)

# Yahboom's eigen berekende sta-houding (zie sessie-uitleg): identiek voor
# alle 6 poten dankzij de radiale symmetrie van de neutrale stand.
COXA_DEG, FEMUR_DEG, TIBIA_DEG = 0.0, 29.9, -14.8

for leg in ["RF", "RM", "RR", "LR", "LM", "LF"]:
    for joint, target in [("coxa", COXA_DEG), ("femur", FEMUR_DEG), ("tibia", TIBIA_DEG)]:
        act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{leg}_{joint}_act")
        data.ctrl[act_id] = target

# Start DIRECT in de sta-houding (geen snelle overgang vanuit "poten recht
# opzij") -- isoleert of de houding zelf stabiel is, los van een eventuele
# schok tijdens een snelle overgang.
for leg in ["RF", "RM", "RR", "LR", "LM", "LF"]:
    for joint, target in [("coxa", COXA_DEG), ("femur", FEMUR_DEG), ("tibia", TIBIA_DEG)]:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{leg}_{joint}_joint")
        data.qpos[model.jnt_qposadr[jid]] = target * 3.14159265 / 180

data.qpos[2] = 0.10  # torso vlak boven de al-berekende sta-hoogte starten

renderer = mujoco.Renderer(model, height=480, width=640)
frames = []
cam = mujoco.MjvCamera()
cam.distance = 0.8
cam.azimuth = 120
cam.elevation = -25
cam.lookat = np.array([0, 0, 0.05])

n_steps = 1500  # 3s bij timestep 0.002
for i in range(n_steps):
    mujoco.mj_step(model, data)
    if i % 15 == 0:  # ~25 fps in de video (elke 30ms simtijd een frame)
        renderer.update_scene(data, camera=cam)
        frames.append(renderer.render())

imageio.mimsave("/home/meinds/muto_standing.mp4", frames, fps=25)
print(f"Video opgeslagen: muto_standing.mp4 ({len(frames)} frames)")
print(f"Eindpositie torso: z={data.qpos[2]:.3f}m")
print(f"Eind-quaternion (moet dicht bij [1,0,0,0] zijn = rechtop): {data.qpos[3:7]}")
