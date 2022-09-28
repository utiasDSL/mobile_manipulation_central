import rospy
import numpy as np

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist, TransformStamped
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState

from mobile_manipulation_central.ros_utils import UR10_JOINT_NAMES


class SimulatedViconObjectInterface:
    """Simulation of the Vicon ROS end point for a detected object.

    This is intended to be instantiated from a simulation environment to
    publish data in the same manner as Vicon would do in the real world.
    """

    def __init__(self, name):
        topic = "/vicon/" + name + "/" + name
        self.pub = rospy.Publisher(topic, TransformStamped, queue_size=1)

        self.ground_truth_pub = rospy.Publisher(name + "/true_joint_states", JointState, queue_size=1)


    def publish_pose(self, t, r, Q):
        """Publish the object's pose at time t, consisting of position r and
        orientation (represented as a quaternion) Q.

        Note that the order of Q is [x, y, z, w]."""
        msg = TransformStamped()
        msg.header.stamp = rospy.Time(t)

        msg.transform.translation.x = r[0]
        msg.transform.translation.y = r[1]
        msg.transform.translation.z = r[2]

        msg.transform.rotation.x = Q[0]
        msg.transform.rotation.y = Q[1]
        msg.transform.rotation.z = Q[2]
        msg.transform.rotation.w = Q[3]

        self.pub.publish(msg)

    def publish_ground_truth(self, t, r, v):
        """Publish ground-truth pose and twist.

        This is useful for debugging purposes: compare estimated state to this
        ground truth."""
        msg = JointState()
        msg.header.stamp = rospy.Time(t)
        msg.name = ["x", "y", "z"]
        msg.position = list(r)
        msg.velocity = list(v)
        self.ground_truth_pub.publish(msg)


class SimulatedRobotROSInterface:
    """Interface between the MPC node and a simulated robot.

    This can be used as a generic ROS end point to simulate a robot. The idea
    is that a simulator should instantiate this class and update it at the
    desired frequency as the simulation runs.
    """

    def __init__(self, nq, nv, robot_name, joint_names):
        self.cmd_vel = None
        self.nq = nq
        self.nv = nv
        self.joint_names = joint_names

        self.clock_pub = rospy.Publisher("/clock", Clock, queue_size=1)
        self.feedback_pub = rospy.Publisher(
            robot_name + "/joint_states", JointState, queue_size=1
        )

    def ready(self):
        return self.cmd_vel is not None

    def publish_feedback(self, t, q, v):
        assert q.shape == (self.nq,)
        assert v.shape == (self.nv,)

        msg = JointState()
        msg.header.stamp = rospy.Time(t)
        msg.name = self.joint_names
        msg.position = q
        msg.velocity = v
        self.feedback_pub.publish(msg)

    def publish_time(self, t):
        """Publish (simulation) time."""
        msg = Clock()
        msg.clock = rospy.Time(t)
        self.clock_pub.publish(msg)


class SimulatedRidgebackROSInterface(SimulatedRobotROSInterface):
    """Simulated Ridgeback interface."""

    def __init__(self):
        robot_name = "ridgeback"
        super().__init__(
            nq=3, nv=3, robot_name=robot_name, joint_names=["x", "y", "yaw"]
        )

        self.cmd_sub = rospy.Subscriber(robot_name + "/cmd_vel", Twist, self._cmd_cb)

    def _cmd_cb(self, msg):
        self.cmd_vel = np.array([msg.linear.x, msg.linear.y, msg.angular.z])


class SimulatedUR10ROSInterface(SimulatedRobotROSInterface):
    """Simulated UR10 interface."""

    def __init__(self):
        robot_name = "ur10"
        super().__init__(
            nq=6, nv=6, robot_name=robot_name, joint_names=UR10_JOINT_NAMES
        )

        self.cmd_sub = rospy.Subscriber(
            robot_name + "/cmd_vel", Float64MultiArray, self._cmd_cb
        )

    def _cmd_cb(self, msg):
        self.cmd_vel = np.array(msg.data)
        assert self.cmd_vel.shape == (self.nv,)


class SimulatedMobileManipulatorROSInterface:
    def __init__(self):
        self.arm = SimulatedUR10ROSInterface()
        self.base = SimulatedRidgebackROSInterface()

        self.nq = self.arm.nq + self.base.nq
        self.nv = self.arm.nv + self.base.nv

    @property
    def cmd_vel(self):
        return np.concatenate((self.base.cmd_vel, self.arm.cmd_vel))

    def ready(self):
        return self.base.ready() and self.arm.ready()

    def publish_feedback(self, t, q, v):
        assert q.shape == (self.nq,)
        assert v.shape == (self.nv,)

        self.base.publish_feedback(t=t, q=q[: self.base.nq], v=v[: self.base.nv])
        self.arm.publish_feedback(t=t, q=q[self.base.nq :], v=v[self.base.nq :])

    def publish_time(self, t):
        """Publish (simulation) time."""
        # arbitrary: we could also use the arm component
        self.base.publish_time(t)
