import numpy as np
import mobile_manipulation_central as mm
import IPython


HOME_NAME = "projectile2"


def main():
    home = mm.load_home_position(HOME_NAME)
    kinematics = mm.MobileManipulatorKinematics()

    J = kinematics.jacobian(home)
    U, S, VT = np.linalg.svd(J[:3, :])

    # the direction which we can instantaneously move fastest in joint space
    # (ignoring constraints) corresponds to the largest singular value of J (or
    # largest eigenvalue of JJ^T)
    n = U[:, 0]
    dqdt = VT[0, :]

    IPython.embed()


if __name__ == "__main__":
    main()
