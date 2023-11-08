"""Plot a point-to-point trajectory with a quintic polynominal time-scaling."""
import numpy as np
import matplotlib.pyplot as plt

import mobile_manipulation_central as mm


START = np.array([0])
DELTA = np.array([1])
MAX_VEL = 1.0
MAX_ACC = 1.0
N = 100


def main():
    traj = mm.PointToPointTrajectory.quintic(
        start=START, delta=DELTA, max_vel=MAX_VEL, max_acc=MAX_ACC, t0=0
    )
    ts = np.linspace(0, traj.duration, N)
    pos = np.zeros_like(ts)
    vel = np.zeros_like(ts)
    acc = np.zeros_like(ts)
    for i, t in enumerate(ts):
        p, v, a = traj.sample(t)
        pos[i] = p
        vel[i] = v
        acc[i] = a

    print(f"v_max = {traj.max_velocity}")
    print(f"a_max = {traj.max_acceleration}")

    plt.plot(ts, pos, label="Position")
    plt.plot(ts, vel, label="Velocity")
    plt.plot(ts, acc, label="Acceleration")
    plt.grid()
    plt.legend()
    plt.xlabel("Time [s]")
    plt.show()


if __name__ == "__main__":
    main()
