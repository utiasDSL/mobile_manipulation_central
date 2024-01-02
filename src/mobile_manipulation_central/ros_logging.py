import datetime
import os
from pathlib import Path
import subprocess
import signal

from geometry_msgs.msg import TransformStamped

from mobile_manipulation_central.ros_utils import vicon_topic_name


ROSBAG_CMD_ROOT = ["rosbag", "record"]


class DataRecorder:
    def __init__(self, topics, name=None, root=None, notes=None):
        if root is None:
            root = os.environ["MOBILE_MANIPULATION_CENTRAL_BAG_DIR"]

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
    expected_rate : float
        Expected rate of Vicon message publishing, in Hz.
    duration : float
        Duration over which to record the messages, in seconds.
    bound : float
        The actual rate is considered acceptable if it lies within ``bound`` Hz
        of the expected rate.
    """
    def __init__(self, vicon_object_name, expected_rate=100, duration=5, bound=1):
        assert expected_rate > 0
        assert duration > 0
        assert bound >= 0

        self.expected_rate = expected_rate
        self.duration = duration
        self.bound = bound
        self.msg_count = 0
        self.start_time = None

        topic_name = vicon_topic_name(vicon_object_name)
        self.vicon_sub = rospy.Subscriber(topic_name, TransformStamped, self._vicon_cb)

    def _vicon_cb(self, msg):
        now = rospy.Time.now().to_sec()

        # record first time a message is received
        if self.start_time is None:
            self.start_time = now

        # stop counting messages once ``self.duration`` seconds has elapsed
        if self.start_time + 5 > self.duration:
            self.vicon_sub.unregister()
            self._check_rate()
            return

        self.msg_count += 1

    def _check_rate(self):
        rate = self.msg_count / self.duration
        lower = self.expected_rate - self.bound
        upper = self.expected_rate + self.bound
        assert (
            lower <= rate <= upper
        ), "Expected Vicon rate is {self.expected_rate} Hz, but actual rate is {rate} Hz."
