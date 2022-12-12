#!/usr/bin/env python3
"""Calibrate the arm using data collected with `collect_arm_calibration_data.py`.

We formulate a nonlinear optimization problem over the product of two SE(3) manifolds.
"""
import argparse
import datetime

import numpy as np
import pymanopt
import yaml
import jax
import jaxlie

from mobile_manipulation_central import MobileManipulatorKinematics

import IPython


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "data_file_name", help="NPZ file containing the calibration data."
    )
    parser.add_argument(
        "-o",
        "--output_file_name",
        help="YAML file to output with optimized transforms.",
    )
    args = parser.parse_args()

    # load calibration data
    data = np.load(args.data_file_name)
    qs = data["qs"]
    # qds = data["qds"]
    r_tw_ws_meas = data["rs"]
    Q_wts_meas = data["Qs"]
    num_configs = qs.shape[0]

    # measured base pose
    T_wbs = []
    for i in range(num_configs):
        r_bw_w = np.array([qs[i, 0], qs[i, 1], 0])
        C_wb = jaxlie.SO3.from_z_radians(qs[i, 2])
        T_wb = jaxlie.SE3.from_rotation_and_translation(C_wb, r_bw_w)
        T_wbs.append(T_wb)

    # measured transform from base to tool
    # T_tbs_meas = []
    T_tws_meas = []
    for i in range(num_configs):
        C_wt_meas = jaxlie.SO3.from_quaternion_xyzw(Q_wts_meas[i, :])
        T_wt_meas = jaxlie.SE3.from_rotation_and_translation(
            C_wt_meas, r_tw_ws_meas[i, :]
        )
        T_tws_meas.append(T_wt_meas.inverse())
        # T_tbs_meas.append(T_wt_meas.inverse() @ T_wbs[i])

    # modelled transform from base to tool
    kinematics = MobileManipulatorKinematics()
    T_bts_model = []
    for i in range(num_configs):
        kinematics.forward(qs[i, :])
        r_tw_w_model, Q_wt_model = kinematics.link_pose()
        C_wt_model = jaxlie.SO3.from_quaternion_xyzw(Q_wt_model)
        T_wt_model = jaxlie.SE3.from_rotation_and_translation(C_wt_model, r_tw_w_model)
        T_bts_model.append(T_wbs[i].inverse() @ T_wt_model)

    # we're optimizing over two SE(3) matrices
    manifold = pymanopt.manifolds.Product(
        (
            pymanopt.manifolds.SpecialOrthogonalGroup(3),
            pymanopt.manifolds.Euclidean(3),
            pymanopt.manifolds.SpecialOrthogonalGroup(3),
            pymanopt.manifolds.Euclidean(3),
        )
    )

    def transform(C, r):
        return jaxlie.SE3.from_rotation_and_translation(jaxlie.SO3.from_matrix(C), r)

    def transform_dict(T):
        return {
            "xyz": list([float(x) for x in T.translation()]),
            "rpy": list([float(x) for x in T.rotation().as_rpy_radians()]),
        }

    # jax-version of cost for autodiff
    def jcost(C1, r1, C2, r2):
        T1 = transform(C1, r1)
        T2 = transform(C2, r2)

        # T1 = transform(C1, r1)
        # T2 = transform(np.eye(3), r2) @ transform(C2, np.zeros(3))

        # T1 = transform(np.eye(3), np.zeros(3))
        # T2 = transform(C2, r2)

        cost = 0
        for i in range(num_configs):
            ΔT = T_wbs[i] @ T1 @ T_bts_model[i] @ T2 @ T_tws_meas[i]
            e = ΔT.log()
            cost = cost + 0.5 * e @ e
        return cost

    # gradient of the cost
    jgrad = jax.grad(jcost, argnums=(0, 1, 2, 3))

    @pymanopt.function.numpy(manifold)
    def cost(C1, r1, C2, r2):
        return jcost(C1, r1, C2, r2)

    @pymanopt.function.numpy(manifold)
    def gradient(C1, r1, C2, r2):
        return jgrad(C1, r1, C2, r2)

    # initial guess
    C20 = np.array([[0., 0, 1], [0, -1, 0], [1, 0, 0]])
    r20 = np.array([0, 0, 0.3])
    x0 = (np.eye(3), np.zeros(3), C20, r20)

    # setup and solve the optimization problem
    problem = pymanopt.Problem(manifold, cost, euclidean_gradient=gradient)
    line_searcher = pymanopt.optimizers.line_search.BackTrackingLineSearcher()
    optimizer = pymanopt.optimizers.SteepestDescent(line_searcher=line_searcher)
    result = optimizer.run(problem, initial_point=x0)

    # optimal transforms
    T1_opt = transform(result.point[0], result.point[1])
    T2_opt = transform(result.point[2], result.point[3])

    yaml_dict = {
        "T1": transform_dict(T1_opt),
        "T2": transform_dict(T2_opt),
    }

    # IPython.embed()

    # save parameters to a file for use in control
    if args.output_file_name is not None:
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        output_file_name = args.output_file_name + "_" + timestamp + ".yaml"
        with open(output_file_name, "w") as f:
            yaml.dump(yaml_dict, f)
        print(f"Saved transform data to {output_file_name}.")
    else:
        print(yaml_dict)


if __name__ == "__main__":
    main()
