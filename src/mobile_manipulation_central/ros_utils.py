"""General ROS parsing utilities."""
import numpy as np
from spatialmath import UnitQuaternion
from spatialmath.base import qslerp
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


def parse_transform_stamped_msg(msg):
    """Parse time and pose from a TransformStamped message.

    Pose is represented as a length-7 vector with position followed by a
    quaternion representing orientation, with the scalar part of the quaternion
    at the end.
    """
    t = msg_time(msg)
    r = msg.transform.translation
    q = msg.transform.rotation
    pose = np.array([r.x, r.y, r.z, q.x, q.y, q.z, q.w])
    return t, pose


def parse_transform_stamped_msgs(msgs, normalize_time=True):
    """Parse a list of TransformStamped messages."""
    ts = parse_time(msgs, normalize_time=normalize_time)
    poses = np.array([parse_transform_stamped_msg(msg)[1] for msg in msgs])
    return ts, poses


def parse_wrench_stamped_msg(msg):
    """Parse time and wrench from a WrenchStamped message."""
    t = msg_time(msg)
    f = msg.wrench.force
    τ = msg.wrench.torque
    wrench = np.array([f.x, f.y, f.z, τ.x, τ.y, τ.z])
    return t, wrench


def parse_wrench_stamped_msgs(msgs, normalize_time=True):
    """Parse a list of WrenchStamped messages."""
    ts = parse_time(msgs, normalize_time=normalize_time)
    wrenches = np.array([parse_wrench_stamped_msg(msg)[1] for msg in msgs])
    return ts, wrenches


def lerp(x, y, s):
    """Linearly interpolate between values x and y with parameter s in [0, 1]."""
    assert 0 <= s <= 1
    return (1 - s) * x + s * y


def slerp(q0, q1, s):
    """Spherical linear interpolation between quaternions q0 and q1 with parameter s in [0, 1].

    Quaternions have order [x, y, z, w]; i.e., the scalar part comes at the end.
    """
    assert 0 <= s <= 1
    assert q0.shape == q1.shape == (4,)

    # we need to roll to convert between spatialmath's [w, x, y, z] convention
    # and our own
    q0 = np.roll(q0, 1)
    q1 = np.roll(q1, 1)
    q = qslerp(q0, q1, s)
    return np.roll(q, -1)


# TODO not ROS specific, so consider moving elsewhere
def interpolate_list(new_times, old_times, values, method="lerp"):
    """Align `values` (corresponding to `old_times`) with the `new_times` using
    interpolation.

    Each value in `values` should be a scalar or numpy array (i.e. something
    that can be scaled and added).

    `method` is the interpolation method. Linear interpolation `lerp` is the
    default. Alternative is `slerp` for spherical linear interpolation when the
    data to be interpolated is quaternions (in xyzw order).

    Returns a new list of values corresponding to `new_times`.
    """
    if method == "slerp":
        interp_func = slerp
    else:
        interp_func = lerp

    aligned_values = []
    idx2 = 0
    n1 = len(new_times)
    n2 = len(old_times)
    for idx1 in range(n1):
        t = new_times[idx1]
        if idx1 > 0:
            assert t >= new_times[idx1 - 1], "Time moved backward!"

        # time is before the values start: pad with the first value
        if t <= old_times[0]:
            aligned_values.append(values[0])
            continue

        # time is after values end: pad with the last value
        if t >= old_times[-1]:
            aligned_values.append(values[-1])
            continue

        # iterate through old_times until a more recent value found
        while idx2 + 1 < n2 and old_times[idx2 + 1] < t:
            idx2 += 1

        assert old_times[idx2] <= t <= old_times[idx2 + 1]

        # interpolate between values[idx2] and values[idx2 + 1]
        Δt = old_times[idx2 + 1] - old_times[idx2]
        s = (t - old_times[idx2]) / Δt
        value = interp_func(values[idx2], values[idx2 + 1], s)
        aligned_values.append(value)

    return aligned_values


def compile_xacro(xacro_path):
    """Compile a xacro file into raw URDF."""
    return xacro.process_file(xacro_path).toxml()


def vicon_topic_name(name):
    return "/".join(["/vicon", name, name])
