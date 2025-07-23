import math
import numpy as np
import rospkg
import yaml
from pathlib import Path


def wrap_to_pi(x):
    """Wrap a value to [-pi, pi]"""
    return math.remainder(x, 2 * np.pi)


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


def load_pkg_config(pkg_name, relpath):
    """Load a YAML config file from a ROS package.

    Parameters
    ----------
    pkg_name : str
        The name of the ROS package.
    relpath : str or Path
        Path of the config file relative to the package root.

    Returns
    -------
    : dict
        The configuration.
    """
    rospack = rospkg.RosPack()
    pkg_path = Path(rospack.get_path(pkg_name))
    path = pkg_path / relpath
    with open(path) as f:
        return yaml.safe_load(f)
