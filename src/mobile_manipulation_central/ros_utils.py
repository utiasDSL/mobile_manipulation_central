"""General ROS parsing utilities."""
import numpy as np
from spatialmath import UnitQuaternion

from mobile_manipulation_central.ur10 import UR10_JOINT_INDEX_MAP


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


def parse_ur10_joint_state_msgs(msgs):
    """Parse a list of UR10 JointState messages."""
    ts = []
    qs = []
    vs = []

    for msg in msgs:
        t, q, v = parse_ur10_joint_state_msg(msg)
        ts.append(t)
        qs.append(q)
        vs.append(v)

    return np.array(ts), np.array(qs), np.array(vs)


# TODO deprecated in favour of the above
# def parse_feedback_msgs(feedback_msgs):
#     """Parse JointState feedback messages.
#
#     This involves reordering the joints in the correct order.
#
#     Returns:
#         ts  times
#         qs  joint positions
#         vs  joint velocities
#     """
#     ts = parse_time(feedback_msgs, normalize_time=False)
#     qs_unordered = np.array([msg.position for msg in feedback_msgs])
#     vs_unordered = np.array([msg.velocity for msg in feedback_msgs])
#
#     qs = np.zeros_like(qs_unordered)
#     vs = np.zeros_like(vs_unordered)
#
#     joint_names = feedback_msgs[0].name
#
#     # re-order joint names so the names correspond to indices given in
#     # JOINT_INDEX_MAP
#     for i in range(qs.shape[1]):
#         j = UR10_JOINT_INDEX_MAP[joint_names[i]]
#         qs[:, j] = qs_unordered[:, i]
#         vs[:, j] = vs_unordered[:, i]
#
#     return ts, qs, vs


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
