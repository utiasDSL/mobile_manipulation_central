#include <ros/ros.h>

#include <mobile_manipulation_central/projectile.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "projectile_vicon_estimator_node");
    ros::NodeHandle nh;

    ros::Rate rate(100);
    mm::ProjectileViconEstimatorNode node;
    node.init(nh);
    node.spin(rate);

    return 0;
}
