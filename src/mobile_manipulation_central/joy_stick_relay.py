import rospy
import threading
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Joy

class ControlCommandRelayInterface:
    """Base class for control command topic relay interface"""

    def __init__(self):
        self.switch = 0     # 0 - send empty message, 1 - send controller messages
        self.switch_lock = threading.Lock()

    def update_switch(self, val):
        if self.switch != val:
            self.switch_lock.acquire()
            self.switch = val
            self.switch_lock.release()

class RidgebackControlCommandRelayInterface(ControlCommandRelayInterface):

    def __init__(self):
        super().__init__()

        self.cmd_sub = rospy.Subscriber("/ridgeback/cmd_vel", Twist, self._cmd_vel_cb)
        self.relay_pub = rospy.Publisher("/ridgeback_velocity_controller/cmd_vel", Twist, queue_size=1)
        self.empty_msg = Twist()

    def _cmd_vel_cb(self, msg):
        self.switch_lock.acquire()
        switch = self.switch
        self.switch_lock.release()

        if switch == 1:
            self.relay_pub.publish(msg)
        else:
            self.relay_pub.publish(self.empty_msg)

class UR10ControlCommandRelayInterface(ControlCommandRelayInterface):

    def __init__(self):
        super().__init__()

        self.cmd_sub = rospy.Subscriber("/ur10/cmd_vel", Float64MultiArray, self._cmd_vel_cb)
        self.relay_pub = rospy.Publisher("/ur10/ur10_velocity_controller/cmd_vel", Float64MultiArray, queue_size=1)
        self.empty_msg = Float64MultiArray()

    def _cmd_vel_cb(self, msg):
        self.switch_lock.acquire()
        switch = self.switch
        self.switch_lock.release()

        if switch == 1:
            self.relay_pub.publish(msg)
        else:
            self.relay_pub.publish(self.empty_msg)

class MobileManipulatorControlCommandRelayInterface:

    def __init__(self):
        self.arm = UR10ControlCommandRelayInterface()
        self.base = RidgebackControlCommandRelayInterface()

        self.joy_sub = rospy.Subscriber("/bluetooth_teleop/joy", Joy, self._joy_cb)

    def _joy_cb(self, msg):
        button = msg.buttons[5]         # 1 when pressed

        self.arm.update_switch(button)
        self.base.update_switch(button)