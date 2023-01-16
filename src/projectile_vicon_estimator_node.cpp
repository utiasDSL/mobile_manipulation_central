#include <ros/ros.h>

#include <mobile_manipulation_central/projectile.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "projectile_vicon_estimator_node");
    ros::NodeHandle nh;

    ros::Rate rate(100);

    double proc_var, meas_var;
    nh.param<double>("proc_var", proc_var, 1.0);
    nh.param<double>("meas_var", meas_var, 1e-4);

    std::cout << "proc_var = " << proc_var << std::endl;
    std::cout << "meas_var = " << meas_var << std::endl;

    mm::ProjectileViconEstimatorNode node;
    node.init(nh, "ThingProjectile", proc_var, meas_var);
    node.spin(rate);

    return 0;
}
