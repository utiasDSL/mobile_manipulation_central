import math
import numpy as np
import rospkg
import yaml
from pathlib import Path


def bound_array(a, lb=None, ub=None):
    """Elementwise bound array above and below."""
    if lb is not None:
        a = np.maximum(a, lb)
    if ub is not None:
        a = np.minimum(a, ub)
    return a


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
