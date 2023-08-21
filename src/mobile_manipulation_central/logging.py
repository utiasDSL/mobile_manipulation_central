import datetime
import os
from pathlib import Path
import subprocess
import signal

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
        self._mkdir()
        self._record_notes()
        self._record_bag()

    def close(self):
        self.proc.send_signal(signal.SIGINT)
