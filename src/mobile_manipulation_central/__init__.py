import os
from pathlib import Path

import numpy as np
import rospkg
import yaml

from mobile_manipulation_central.ur10 import UR10_JOINT_NAMES, UR10_JOINT_INDEX_MAP
from mobile_manipulation_central.trajectory_client import TrajectoryClient
from mobile_manipulation_central.ros_utils import (
    msg_time,
    parse_time,
    parse_ur10_joint_state_msgs,
    trim_msgs,
)
from mobile_manipulation_central.mobile_manipulator_ros_interface import (
    RidgebackROSInterface,
    UR10ROSInterface,
    MobileManipulatorROSInterface,
)

BAG_DIR = os.environ["MOBILE_MANIPULATION_CENTRAL_BAG_DIR"]


def bound_array(a, lb=None, ub=None):
    """Elementwise bound array above and below."""
    if lb is not None:
        a = np.maximum(a, lb)
    if ub is not None:
        a = np.minimum(a, ub)
    return a


def load_home_position(name):
    """Load robot home position from config file."""
    rospack = rospkg.RosPack()
    path = (
        Path(rospack.get_path("mobile_manipulation_central")) / "config" / "home.yaml"
    )
    with open(path) as f:
        data = yaml.safe_load(f)
    return np.array(data[name])
