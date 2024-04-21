import datetime
import os
from pathlib import Path
import subprocess
import signal

import rospy
from geometry_msgs.msg import TransformStamped

from mobile_manipulation_central.ros_utils import vicon_topic_name


BAG_DIR_ENV_VAR = "MOBILE_MANIPULATION_CENTRAL_BAG_DIR"
BAG_DIR = os.environ.get(BAG_DIR_ENV_VAR, None)

ROSBAG_CMD_ROOT = ["rosbag", "record"]


class DataRecorder:
    def __init__(self, topics, name=None, root=None, notes=None):
        if root is None:
            if BAG_DIR is None:
                raise ValueError(
                    f"No root directory given and {BAG_DIR_ENV_VAR} environment variable not set."
                )
            root = BAG_DIR

        stamp = datetime.datetime.now()
        ymd = stamp.strftime("%Y-%m-%d")
        hms = stamp.strftime("%H-%M-%S")
        if name is not None:
            dir_name = Path(ymd) / (name + "_" + hms)
        else:
            dir_name = Path(ymd) / hms

        self.log_dir = root / dir_name
        self.topics = topics
        self.notes = notes

    def _mkdir(self):
        self.log_dir.mkdir(parents=True)

    def _record_notes(self):
        # write any notes
        if self.notes is not None:
            notes_out_path = self.log_dir / "notes.txt"
            with open(notes_out_path, "w") as f:
                f.write(self.notes)

    def _record_bag(self):
        # start the logging with rosbag
        rosbag_out_path = self.log_dir / "bag"
        rosbag_cmd = ROSBAG_CMD_ROOT + ["-o", rosbag_out_path] + self.topics
        self.proc = subprocess.Popen(rosbag_cmd)

    def record(self):
        # NOTE: you may want to sleep briefly after this (~3 seconds) to make
        # sure the bag is setup and recording before you do other things!
        self._mkdir()
        self._record_notes()
        self._record_bag()

    def close(self):
        self.proc.send_signal(signal.SIGINT)


class ViconRateChecker:
    """Check that Vicon rate is what you expect it to be.

    It is a good idea to put this before your control loops to make sure the
    data you record is what you want.

    Parameters
    ----------
    vicon_object_name : str
        The name of the Vicon object of which to count the messages.
    duration : float
        Duration over which to record the messages, in seconds.
    """

    def __init__(self, vicon_object_name, duration=5):
        assert duration > 0

        self.duration = duration
        self.msg_count = 0
        self.start_time = None
        self.started = False
        self.done = False

        self.topic_name = vicon_topic_name(vicon_object_name)
        self.vicon_sub = rospy.Subscriber(
            self.topic_name, TransformStamped, self._vicon_cb
        )

    def _vicon_cb(self, msg):
        """Vicon subscriber callback."""
        if not self.started:
            return

        # record first time a message is received
        now = rospy.Time.now().to_sec()
        if self.start_time is None:
            self.start_time = now

        # stop counting messages once ``self.duration`` seconds has elapsed
        if now - self.start_time > self.duration:
            self.vicon_sub.unregister()
            self.done = True
            return

        self.msg_count += 1

    def check_rate(self, expected_rate, bound=1, verbose=True):
        """Check the Vicon rate.

        Parameters
        ----------
        expected_rate : float, positive
            Expected rate of Vicon message publishing, in Hz.
        bound : float
            The actual rate is considered acceptable if it lies within
            ``bound`` Hz of the expected rate.

        Returns
        -------
        : bool
            ``True`` if the measured Vicon rate is within the bounds, ``False``
            otherwise.
        """

        self.started = True

        # let the user know if we aren't receiving messages
        rate = rospy.Rate(1)
        while not self.done and not rospy.is_shutdown():
            rate.sleep()
            if self.msg_count == 0:
                print(f"I haven't received any messages on {self.topic_name}")

        rate = self.msg_count / self.duration
        lower = expected_rate - bound
        upper = expected_rate + bound

        if verbose:
            print(
                f"Received {self.msg_count} Vicon messages over {self.duration} seconds."
            )
            print(f"Expected Vicon rate = {expected_rate} Hz")
            print(f"Average Vicon rate = {rate} Hz")

        return lower <= rate <= upper
