import os
from pathlib import Path

import numpy as np
import rospkg
import yaml

from mobile_manipulation_central.trajectory_client import TrajectoryClient
from mobile_manipulation_central.ros_utils import (
    msg_time,
    parse_time,
    parse_ur10_joint_state_msgs,
    trim_msgs,
    compile_xacro,
)
from mobile_manipulation_central.ros_interface import (
    ViconObjectInterface,
    RidgebackROSInterface,
    UR10ROSInterface,
    MobileManipulatorROSInterface,
    RobotSignalHandler,
)
from mobile_manipulation_central.simulation_ros_interface import (
    SimulatedRidgebackROSInterface,
    SimulatedUR10ROSInterface,
    SimulatedMobileManipulatorROSInterface,
    SimulatedViconObjectInterface,
)
from mobile_manipulation_central.kinematics import MobileManipulatorKinematics
from mobile_manipulation_central.trajectory_generation import (
    PointToPointTrajectory,
    QuinticTimeScaling,
)
from mobile_manipulation_central.smoothing import ExponentialSmoother
from mobile_manipulation_central import ros_utils, simulation
from mobile_manipulation_central.simulation import (
    BulletSimulation,
    BulletSimulatedRobot,
)

BAG_DIR_ENV_VAR = "MOBILE_MANIPULATION_CENTRAL_BAG_DIR"
BAG_DIR = os.environ[BAG_DIR_ENV_VAR]


def bound_array(a, lb=None, ub=None):
    """Elementwise bound array above and below."""
    if lb is not None:
        a = np.maximum(a, lb)
    if ub is not None:
        a = np.minimum(a, ub)
    return a


def load_home_position(name="default", path=None):
    """Load robot home position from YAML config file located at `path`.

    If `path` is None, then the default is to load the config file shipped with
    this repo.
    """
    if path is None:
        rospack = rospkg.RosPack()
        pkg_path = Path(rospack.get_path("mobile_manipulation_central"))
        path = pkg_path / "config" / "home.yaml"

    with open(path) as f:
        data = yaml.safe_load(f)
    return np.array(data[name])
