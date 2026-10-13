import sys
import mujoco
import numpy as np

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]

model = mujoco.MjModel.from_xml_path("src/p4_common/mjcf/so101.xml")
data = mujoco.MjData(model)

angles = [float(a) for a in sys.argv[1:6]]
for name, a in zip(JOINTS, angles):
    data.qpos[model.joint(name).qposadr[0]] = a

mujoco.mj_forward(model, data)

body = data.body("gripper_link")
print("pos:", np.round(body.xpos, 6))
print(
    "rpy:",
    np.round(
        [
            np.arctan2(body.xmat.reshape(3, 3)[2, 1], body.xmat.reshape(3, 3)[2, 2]),
            np.arcsin(-body.xmat.reshape(3, 3)[2, 0]),
            np.arctan2(body.xmat.reshape(3, 3)[1, 0], body.xmat.reshape(3, 3)[0, 0]),
        ],
        6,
    ),
)
