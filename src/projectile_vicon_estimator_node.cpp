#include <ros/ros.h>

#include <mobile_manipulation_central/projectile.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "projectile_vicon_estimator_node");
    ros::NodeHandle nh;

    ros::Rate rate(100);

    double pos_proc_var, vel_proc_var, pos_meas_var;
    nh.param<double>("pos_proc_var", pos_proc_var, 1.0);
    nh.param<double>("vel_proc_var", vel_proc_var, 1.0);
    nh.param<double>("pos_meas_var", pos_meas_var, 1.0);

    std::cout << "pos_proc_var = " << pos_proc_var << std::endl;
    std::cout << "vel_proc_var = " << vel_proc_var << std::endl;
    std::cout << "pos_meas_var = " << pos_meas_var << std::endl;

    mm::ProjectileViconEstimatorNode node;
    node.init(nh, "Projectile", pos_proc_var, vel_proc_var, pos_meas_var);
    node.spin(rate);

    return 0;
}
