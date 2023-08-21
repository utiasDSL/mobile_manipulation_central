#!/usr/bin/env python3
import argparse
import time

import mobile_manipulation_central as mm


# fmt: off
ROSBAG_TOPICS = [
        "/clock",
        "--regex", "/ridgeback/(.*)",
        "--regex", "/ridgeback_velocity_controller/(.*)",
        "--regex", "/ur10/(.*)",
        "--regex", "/vicon/(.*)",
        "--regex", "/projectile/(.*)",
        "--regex", "/mobile_manipulator_(.*)"
]
# fmt: on


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("config_path", help="Path to config file")
    parser.add_argument("--name", help="Name to be prepended to directory.")
    parser.add_argument(
        "--notes",
        help="Additional information written to notes.txt inside the directory.",
    )
    args = parser.parse_args()

    recorder = mm.DataRecorder(topics=ROSBAG_TOPICS, name=args.name, notes=args.notes)
    recorder.record()

    # spin until SIGINT (Ctrl-C) is received
    try:
        while True:
            time.sleep(0.01)
    except KeyboardInterrupt:
        recorder.close()


if __name__ == "__main__":
    main()
