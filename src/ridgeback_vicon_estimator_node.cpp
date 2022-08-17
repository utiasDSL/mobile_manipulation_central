#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Eigen>

#include <mobile_manipulation_central/exponential_smoothing.h>
#include <mobile_manipulation_central/wrap.h>

// The node listens to raw Vicon transform messages for the Ridgeback base and
// converts it into a JointState message which includes a numerically
// differentiated and filtered velocity estimate on (x, y, yaw).
class RidgebackViconEstimatorNode {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    RidgebackViconEstimatorNode() {}

    // Start the node.
    bool start(ros::NodeHandle& nh) {
        std::string base_vicon_topic;
        nh.param<std::string>("base_vicon_topic", base_vicon_topic,
                              "/vicon/ThingBase/ThingBase");
        ridgeback_vicon_sub = nh.subscribe(
            base_vicon_topic, 1,
            &RidgebackViconEstimatorNode::ridgeback_vicon_cb, this);

        ridgeback_joint_states_pub =
            nh.advertise<sensor_msgs::JointState>("/ridgeback/joint_states", 1);

        // Velocity is assumed to be 0 initially. Values for tau taken from
        // dsl__estimation__vicon package.
        linear_velocity_filter.init(0.045, Eigen::Vector2d::Zero());
        angular_velocity_filter.init(0.025, 0);

        return true;
    }

   private:
    /* FUNCTIONS */

    void publish_ridgeback_joint_states(const Eigen::Vector3d& q,
                                        const Eigen::Vector3d& v) {
        sensor_msgs::JointState msg;
        msg.header.stamp = ros::Time::now();
        msg.name = {"x", "y", "yaw"};

        for (int i = 0; i < 3; ++i) {
            msg.position.push_back(q(i));
            msg.velocity.push_back(v(i));
        }

        ridgeback_joint_states_pub.publish(msg);
    }

    void ridgeback_vicon_cb(const geometry_msgs::TransformStamped& msg) {
        ++msg_count;

        // Get the current joint configuration
        double t = msg.header.stamp.toSec();
        Eigen::Vector3d q;
        q << msg.transform.translation.x, msg.transform.translation.y,
            tf::getYaw(msg.transform.rotation);

        // Wait until we have at least two messages so we can numerically
        // differentiate.
        if (msg_count >= 2) {
            double dt = t - t_prev;

            // Compute velocity via numerical differentiation
            Eigen::Vector3d delta = q - q_prev;
            delta(2) = mm::wrap_to_pi(delta(2));
            Eigen::Vector3d v_measured = delta / dt;

            // Filter velocity
            Eigen::Vector3d v_filtered;
            v_filtered << linear_velocity_filter.next(v_measured.head(2), dt),
                angular_velocity_filter.next(v_measured(2), dt);

            // Publish the joint states
            publish_ridgeback_joint_states(q, v_filtered);
        }

        t_prev = t;
        q_prev = q;
    }

    /* VARIABLES */

    // Subscriber to Vicon pose.
    ros::Subscriber ridgeback_vicon_sub;

    // Publisher for position and velocity of the base (x, y, theta).
    ros::Publisher ridgeback_joint_states_pub;

    // Store last received time and configuration for numerical differentiation
    double t_prev;
    Eigen::Vector3d q_prev;

    // Exponential smoothing filters to remove noise from numerically
    // differentiated velocity.
    mm::ExponentialSmoother<double> angular_velocity_filter;
    mm::ExponentialSmoother<Eigen::Vector2d> linear_velocity_filter;

    // Number of messages received.
    uint32_t msg_count = 0;

};  // class RidgebackViconEstimatorNode

int main(int argc, char** argv) {
    ros::init(argc, argv, "ridgeback_vicon_estimator_node");
    ros::NodeHandle nh;

    RidgebackViconEstimatorNode node;
    node.start(nh);
    ros::spin();

    return 0;
}
