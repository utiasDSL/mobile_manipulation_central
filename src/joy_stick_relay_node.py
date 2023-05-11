import rospy
from mobile_manipulation_central.joy_stick_relay import MobileManipulatorControlCommandRelayInterface

if __name__ == "__main__":
    rospy.init_node("joy_stick_relay")

    node = MobileManipulatorControlCommandRelayInterface()

    rospy.spin()
