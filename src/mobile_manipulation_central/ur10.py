import numpy as np

# TODO put in config file eventually
UR10_HOME = np.pi * np.array([0.5, -0.25, 0.5, -0.25, 0.5, 0.417])

UR10_JOINT_NAMES = [
    "ur10_arm_shoulder_pan_joint",
    "ur10_arm_shoulder_lift_joint",
    "ur10_arm_elbow_joint",
    "ur10_arm_wrist_1_joint",
    "ur10_arm_wrist_2_joint",
    "ur10_arm_wrist_3_joint",
]

# maps joint names to the indices they are expected to have used to re-order
# feedback messages, which don't guarantee order
UR10_JOINT_INDEX_MAP = {name: index for index, name in enumerate(UR10_JOINT_NAMES)}
