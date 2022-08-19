import rospy
import numpy as np
from geometry_msgs.msg import TransformStamped

import mobile_manipulation_central as mm

import IPython


class ViconObject:
    def __init__(self, name):
        topic = "/vicon/" + name + "/" + name
        self.msg_received = False
        self.sub = rospy.Subscriber(topic, TransformStamped, self._transform_cb)

    def ready(self):
        return self.msg_received

    def _transform_cb(self, msg):
        L = msg.translation.translation
        R = msg.translation.rotation

        self.position = np.array([L.x, L.y, L.z])
        self.orientation = np.array([R.x, R.y, R.z, R.w])

        self.msg_received = True


def main():
    model = mm.MobileManipulatorKinematics()
    link_idx = model.model.getBodyId("base_link")

    q = np.array([-2.0, -1.0, 0, 1.5708, -0.7854,  1.5708, -0.7854,  1.5708, 1.3100])
    model.forward(q)

    r, Q = model.link_pose(link_idx)

    IPython.embed()
    return

    rospy.init_node("calibration")

    robot = mm.MobileManipulatorROSInterface()
    obj = ViconObject("BoxTray")

    # wait until robot feedback has been received
    while not rospy.is_shutdown() and not (robot.ready() and obj.ready()):
        rate.sleep()

    q = robot.q.copy()
    model.forward(q)
    r_ee, Q_ee = robot.link_pose()
    r_obj, Q_obj = obj.position, obj.orientation

    IPython.embed()


if __name__ == "__main__":
    main()
