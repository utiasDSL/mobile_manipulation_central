import numpy as np
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

from mobile_manipulation_central import ros_utils


# TODO add protections if time since last message is too large


class RobotROSInterface:
    """Base class for defining ROS interfaces for robots."""
    def __init__(self, nq, nv):
        self.nq = nq
        self.nv = nv

        self.q = np.zeros(self.nq)
        self.v = np.zeros(self.nv)

        self.joint_states_received = False
        self.last_msg_time = None

    def brake(self):
        """Brake (stop) the robot."""
        self.publish_cmd_vel(np.zeros(self.nv))

    def ready(self):
        """True if joint state messages have been received for both arm and base."""
        return self.joint_states_received


class RidgebackROSInterface(RobotROSInterface):
    """ROS interface for the Ridgeback mobile base."""
    def __init__(self):
        super().__init__(nq=3, nv=3)

        self.cmd_pub = rospy.Publisher("/ridgeback/cmd_vel", Twist, queue_size=1)
        self.joint_state_sub = rospy.Subscriber(
            "/ridgeback/joint_states", JointState, self._joint_state_cb
        )

    def _joint_state_cb(self, msg):
        """Callback for Ridgeback joint feedback."""
        self.q = np.array(msg.position)
        self.v = np.array(msg.velocity)

        self.joint_states_received = True
        # self.last_msg_time = rospy.Time.now().to_sec()

    def publish_cmd_vel(self, cmd_vel):
        """Command the velocity of the robot's joints."""
        assert cmd_vel.shape == (self.nv,)

        msg = Twist()
        msg.linear.x = cmd_vel[0]
        msg.linear.y = cmd_vel[1]
        msg.angular.z = cmd_vel[2]
        self.cmd_pub.publish(msg)


class UR10ROSInterface(RobotROSInterface):
    """ROS interface for the UR10 arm."""
    def __init__(self):
        super().__init__(nq=6, nv=6)

        self.cmd_pub = rospy.Publisher("/ur10/cmd_vel", Float64MultiArray, queue_size=1)
        self.joint_state_sub = rospy.Subscriber(
            "/ur10/joint_states", JointState, self._joint_state_cb
        )

    def _joint_state_cb(self, msg):
        """Callback for arm joint feedback."""
        _, self.q, self.v = ros_utils.parse_ur10_joint_state_msg(msg)
        self.joint_states_received = True

    def publish_cmd_vel(self, cmd_vel):
        """Command the velocity of the robot's joints."""
        assert cmd_vel.shape == (self.nv,)

        msg = Float64MultiArray()
        msg.data = list(cmd_vel)
        self.cmd_pub.publish(msg)


class MobileManipulatorROSInterface:
    """ROS interface to the real mobile manipulator."""

    def __init__(self):
        self.arm = UR10ROSInterface()
        self.base = RidgebackROSInterface()

        self.nq = self.arm.nq + self.base.nq
        self.nv = self.arm.nv + self.base.nv

    def brake(self):
        """Brake (stop) the robot."""
        self.base.brake()
        self.arm.brake()

    def ready(self):
        """True if joint state messages have been received for both arm and base."""
        return self.base.ready() and self.arm.ready()

    def publish_cmd_vel(self, cmd_vel):
        """Command the velocity of the robot's joints."""
        assert cmd_vel.shape == (self.nv,)

        self.base.publish_cmd_vel(cmd_vel[:self.base.nv])
        self.arm.publish_cmd_vel(cmd_vel[self.base.nv:])

    @property
    def q(self):
        """Latest joint configuration measurement."""
        return np.concatenate((self.base.q, self.arm.q))

    @property
    def v(self):
        """Latest joint velocity measurement.

        Note that the base velocity is in the world frame.
        """
        return np.concatenate((self.base.v, self.arm.v))
