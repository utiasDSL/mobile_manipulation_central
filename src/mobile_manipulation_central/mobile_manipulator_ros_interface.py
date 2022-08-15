import numpy as np
import rospy
from spatialmath.base import rotz, UnitQuaternion

from geometry_msgs.msg import Twist, TransformStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

import mobile_manipulation_central.ros_utils as util


class MobileManipulatorROSInterface:
    """ROS interface to the real mobile manipulator."""

    def __init__(self, config):
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
        self.base_cmd_pub = rospy.Publisher(
            "/ridgeback_velocity_controller/cmd_vel", Twist, queue_size=1
        )
        self.arm_cmd_pub = rospy.Publisher(
            "/ur10/cmd_vel", JointTrajectory, queue_size=1
        )

        # subscribers for joint states
        self.base_joint_sub = rospy.Subscriber(
            "/vicon/ThingBase/ThingBase", TransformStamped, self._base_joint_cb
        )
        self.arm_joint_sub = rospy.Subscriber(
            "/ur10/joint_states", JointState, self._arm_joint_cb
        )

    def ready(self):
        return self.base_msg_received and self.arm_msg_received

    def _base_rotation_matrix(self):
        """Get rotation matrix for the base.

        This is just the rotation about the z-axis by the yaw angle.
        """
        yaw = self.q_base[2]
        C_wb = rotz(yaw)
        return C_wb

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

        msg = JointTrajectory()
        msg.header.stamp = rospy.Time.now()

        point = JointTrajectoryPoint()
        point.velocities = list(cmd_vel_arm)
        msg.points.append(point)

        self.arm_cmd_pub.publish(msg)

    def publish_cmd_vel(self, cmd_vel):
        """Command the velocity of the robot's joints."""
        assert cmd_vel.shape == (9,)

        self.publish_base_cmd_vel(cmd_vel[:3])
        self.publish_arm_cmd_vel(cmd_vel[3:])

    def _base_joint_cb(self, msg):
        """Callback for base joint feedback."""
        x = msg.transform.translation.x
        y = msg.transform.translation.y
        yaw = util.quaternion_from_msg(msg.transform.rotation).rpy()[2]
        self.q_base = np.array([x, y, yaw])

        self.v_base = np.zeros(3)  # TODO

        self.base_msg_received = True

    def _arm_joint_cb(self, msg):
        """Callback for arm joint feedback."""
        _, self.q_arm, self.v_arm = util.parse_ur10_joint_state_msg(msg)
        self.arm_msg_received = True

    @property
    def q(self):
        """Latest joint configuration measurement."""
        return np.concatenate((self.q_base, self.q_arm))

    @property
    def v(self):
        """Latest joint velocity measurement."""
        # Velocity representation is in the body frame. Vicon gives feedback in
        # the world frame, so we have to rotate the linear base velocity.
        C_wb = self._base_rotation_matrix()
        v_base_body = np.append((C_wb.T @ self.v_base[:2], self.v_base[2]))
        return np.concatenate((v_base_body, self.v_arm))
