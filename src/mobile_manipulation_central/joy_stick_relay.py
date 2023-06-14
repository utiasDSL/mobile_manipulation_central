import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Joy


class ControlCommandRelayInterface:
    """Base class for control command topic relay interface"""

    def __init__(self):
        # True to relay controller commands, False otherwise
        self.enable_relay = False


class RidgebackControlCommandRelayInterface(ControlCommandRelayInterface):
    def __init__(self):
        super().__init__()

        self.cmd_sub = rospy.Subscriber("/ridgeback/cmd_vel", Twist, self._cmd_vel_cb)
        self.relay_pub = rospy.Publisher(
            "/ridgeback_velocity_controller/cmd_vel", Twist, queue_size=1
        )

    def _cmd_vel_cb(self, msg):
        if self.enable_relay:
            relay_msg = msg
        else:
            relay_msg = Twist()
        self.relay_pub.publish(relay_msg)


class UR10ControlCommandRelayInterface(ControlCommandRelayInterface):
    def __init__(self):
        super().__init__()

        self.cmd_sub = rospy.Subscriber(
            "/ur10/cmd_vel", Float64MultiArray, self._cmd_vel_cb
        )
        self.relay_pub = rospy.Publisher(
            "/ur10/ur10_velocity_controller/cmd_vel", Float64MultiArray, queue_size=1
        )

    def _cmd_vel_cb(self, msg):
        if self.enable_relay:
            relay_msg = msg
        else:
            relay_msg = Float64MultiArray()
            relay_msg.data = [0] * 6
        self.relay_pub.publish(relay_msg)


class MobileManipulatorControlCommandRelayInterface:
    def __init__(self):
        self.arm = UR10ControlCommandRelayInterface()
        self.base = RidgebackControlCommandRelayInterface()

        self.joy_sub = rospy.Subscriber("/bluetooth_teleop/joy", Joy, self._joy_cb)

    def _joy_cb(self, msg):
        # R1 button is the deadman's switch
        r1_pressed = msg.buttons[5]
        enable_relay = r1_pressed == 1

        self.arm.enable_relay = enable_relay
        self.base.enable_relay = enable_relay
