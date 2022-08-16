import numpy as np
import rospy

from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

from mobile_manipulation_central import ros_utils


class MobileManipulatorROSInterface:
    """ROS interface to the real mobile manipulator."""

    def __init__(self):
        # dimensions
        self.nq = 9
        self.nv = 9

        # feedback
        self.q_base = np.zeros(3)
        self.v_base = np.zeros(3)
        self.q_arm = np.zeros(6)
        self.v_arm = np.zeros(6)

        # set to true once feedback has been received
        self.base_msg_received = False
        self.arm_msg_received = False

        # publishers for velocity commands
        self.base_cmd_pub = rospy.Publisher("/ridgeback/cmd_vel", Twist, queue_size=1)
        self.arm_cmd_pub = rospy.Publisher("/ur10/cmd_vel", JointTrajectory, queue_size=1)

        # subscribers for joint states
        self.base_joint_sub = rospy.Subscriber(
            "/ridgeback/joint_states", JointState, self._base_joint_cb
        )
        self.arm_joint_sub = rospy.Subscriber(
            "/ur10/joint_states", JointState, self._arm_joint_cb
        )

    def ready(self):
        """True if joint state messages have been received for both arm and base."""
        return self.base_msg_received and self.arm_msg_received

    def publish_base_cmd_vel(self, cmd_vel_base):
        """Publish base velocity commands."""
        assert cmd_vel_base.shape == (3,)

        msg = Twist()
        msg.linear.x = cmd_vel_base[0]
        msg.linear.y = cmd_vel_base[1]
        msg.angular.z = cmd_vel_base[2]
        self.base_cmd_pub.publish(msg)

    def publish_arm_cmd_vel(self, cmd_vel_arm):
        """Publish arm velocity commands."""
        assert cmd_vel_arm.shape == (6,)

        msg = Float64MultiArray()
        msg.data = list(cmd_vel_arm)
        self.arm_cmd_pub.publish(msg)

    def publish_cmd_vel(self, cmd_vel):
        """Command the velocity of the robot's joints."""
        assert cmd_vel.shape == (9,)

        self.publish_base_cmd_vel(cmd_vel[:3])
        self.publish_arm_cmd_vel(cmd_vel[3:])

    def _base_joint_cb(self, msg):
        """Callback for base joint feedback."""
        self.q_base = np.array(msg.position)
        self.v_base = np.array(msg.velocity)
        self.base_msg_received = True

    def _arm_joint_cb(self, msg):
        """Callback for arm joint feedback."""
        _, self.q_arm, self.v_arm = ros_utils.parse_ur10_joint_state_msg(msg)
        self.arm_msg_received = True

    @property
    def q(self):
        """Latest joint configuration measurement."""
        return np.concatenate((self.q_base, self.q_arm))

    @property
    def v(self):
        """Latest joint velocity measurement.

        Note that the base velocity is in the world frame.
        """
        return np.concatenate((self.v_base, self.v_arm))
