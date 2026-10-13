import mujoco
import mujoco.viewer
import time
import os

HERE = os.path.dirname(os.path.abspath(__file__))
# URDF = os.path.join(HERE, '..', '..', 'src', 'p4_common', 'urdf', 'so101_new_calib.urdf')
URDF = os.path.join(HERE, "..", "..", "src", "p4_common", "mjcf", "so101.xml")

model = mujoco.MjModel.from_xml_path(URDF)
data = mujoco.MjData(model)
model.opt.gravity[:] = 0
model = mujoco.MjModel.from_xml_path(URDF)
mujoco.mj_saveLastXML("so101.xml", model)

JOINT = "wrist_roll"  # change per run
jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, JOINT)
adr = model.jnt_qposadr[jid]

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        for angle in (0.0, 0.5, 0.0, -0.5):
            data.qpos[:] = 0
            data.qpos[adr] = angle
            mujoco.mj_forward(model, data)  # NOT mj_step
            viewer.sync()
            print(f"{JOINT} = {angle:+.1f}")
            time.sleep(1.5)
