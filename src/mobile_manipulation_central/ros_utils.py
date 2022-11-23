"""General ROS parsing utilities."""
import subprocess
import tempfile
import numpy as np
from spatialmath import UnitQuaternion
import xacro


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


def msg_time(msg):
    """Extract message timestamp as float in seconds."""
    return msg.header.stamp.to_sec()


def parse_time(msgs, normalize_time=True, t0=None):
    """Parse time in seconds from a list of messages.

    If normalize_time is True (the default), the array of time values will be
    normalized such that t[0] = t0. If t0 is not provided, it defaults to t[0].
    """
    t = np.array([msg_time(msg) for msg in msgs])
    if normalize_time:
        if t0:
            t -= t0
        else:
            t -= t[0]
    return t


def parse_ur10_joint_state_msg(msg):
    """Return a tuple (t, q, v) of time, configuration, velocity parsed from the
    JointState message of the UR10.

    In particular, this correctly orders the joints.
    """
    t = msg_time(msg)

    # re-order joint names so the names correspond to indices given in
    # UR10_JOINT_INDEX_MAP
    q = np.zeros(6)
    v = np.zeros(6)
    for i in range(len(msg.position)):
        j = UR10_JOINT_INDEX_MAP[msg.name[i]]
        q[j] = msg.position[i]
        v[j] = msg.velocity[i]

    return t, q, v


def parse_ur10_joint_state_msgs(msgs, normalize_time=True):
    """Parse a list of UR10 JointState messages.

    If normalize_time=True, the time array is shifted so that t[0] = 0."""
    ts = []
    qs = []
    vs = []

    for msg in msgs:
        t, q, v = parse_ur10_joint_state_msg(msg)
        ts.append(t)
        qs.append(q)
        vs.append(v)

    ts = np.array(ts)
    if normalize_time:
        ts -= ts[0]

    return ts, np.array(qs), np.array(vs)


def trim_msgs(msgs, t0=None, t1=None):
    """Trim messages that so only those in the time interval [t0, t1] are included."""
    ts = parse_time(msgs, normalize_time=False)
    start = 0
    if t0 is not None:
        for i in range(ts.shape[0]):
            if ts[i] >= t0:
                start = i
                break

    end = ts.shape[0]
    if t1 is not None:
        for i in range(start, ts.shape[0]):
            if ts[i] > t1:
                end = i
                break

    return msgs[start:end]


def quaternion_from_msg(msg):
    """Parse a spatialmath quaternion from a geometry_msgs/Quaternion ROS message."""
    return UnitQuaternion(s=msg.w, v=[msg.x, msg.y, msg.z])


def yaw_from_quaternion_msg(msg):
    """Return the yaw component of a geometry_msgs/Quaternion ROS message."""
    Q = quaternion_from_msg(msg)
    return Q.rpy()[2]


def parse_ridgeback_vicon_msg(msg):
    """Get the base's (x, y, yaw) 2D pose from a geometry_msgs/TransformStamped ROS message from Vicon."""
    t = msg_time(msg)
    x = msg.transform.translation.x
    y = msg.transform.translation.y
    θ = yaw_from_quaternion_msg(msg.transform.rotation)
    q = np.array([x, y, θ])
    return t, q


def parse_ridgeback_vicon_msgs(msgs):
    """Parse a list of Vicon messages representing the base's pose."""
    ts = []
    qs = []
    for msg in msgs:
        t, q = parse_ridgeback_vicon_msg(msg)
        ts.append(t)
        qs.append(q)
    return np.array(ts), np.array(qs)


def parse_ridgeback_joint_state_msgs(msgs, normalize_time=True):
    """Parse a list of Ridgeback JointState messages.

    If normalize_time=True, the time array is shifted so that t[0] = 0."""
    ts = []
    qs = []
    vs = []

    for msg in msgs:
        ts.append(msg_time(msg))
        qs.append(msg.position)
        vs.append(msg.velocity)

    ts = np.array(ts)
    if normalize_time:
        ts -= ts[0]

    return ts, np.array(qs), np.array(vs)


# NOTE: this is much worse than the linear interpolation approach below
# def align_lists(times1, times2, values):
#     """Align values in `values2` with the times `times11."""
#     aligned_values = []
#     idx2 = 0
#     for idx1 in range(len(times1)):
#         t = times1[idx1]
#         # iterate through times2 until a more recent value found
#         while idx2 + 1 < len(times2) and times2[idx2 + 1] < t:
#             idx2 += 1
#         aligned_values.append(values[idx2])
#     return aligned_values


def align_lists_linear_interpolate(times1, times2, values):
    """Align values in `values2` with the times `times11 using linear interpolation.

    Each value in `values` should be a scalar or numpy array (i.e. something
    that can be scaled and added).

    Returns a new list of values corresponding to `times1`.
    """
    aligned_values = []
    idx2 = 0
    n1 = times1.shape[0]
    n2 = times2.shape[0]
    for idx1 in range(n1):
        t = times1[idx1]

        if t <= times2[0]:
            aligned_values.append(values[0])
            continue
        if t >= times2[-1]:
            aligned_values.append(values[-1])
            continue

        # iterate through times2 until a more recent value found
        while idx2 + 1 < n2 and times2[idx2 + 1] < t:
            idx2 += 1

        assert times2[idx2] <= t <= times2[idx2 + 1]

        # linear interpolate between values[idx2] and values[idx2 + 1]
        Δt = times2[idx2 + 1] - times2[idx2]
        a = times2[idx2 + 1] - t
        b = t - times2[idx2]
        value = (a * values[idx2] + b * values[idx2 + 1]) / Δt
        aligned_values.append(value)

        # aligned_values.append(values[idx2])
    return aligned_values


def compile_xacro(xacro_path):
    """Compile a xacro file into raw URDF."""
    return xacro.process_file(xacro_path).toxml()
