#pragma once

#include <geometry_msgs/TransformStamped.h>
#include <mobile_manipulation_central/kalman_filter.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Empty.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Eigen>

namespace mm {

// The node listens to raw Vicon transform messages for a projectile object and
// converts them to a JointState message for the translational component. For
// the moment we assume we don't care about the projectile orientation and so
// do not do anything with it.
class ProjectileViconEstimator {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ProjectileViconEstimator() {}

    bool init(ros::NodeHandle& nh, double proc_var, double meas_var,
              double nis_bound, double activation_height,
              double deactivation_height, const Eigen::Vector3d& gravity) {
        proc_var_ = proc_var;
        meas_var_ = meas_var;
        nis_bound_ = nis_bound;
        activation_height_ = activation_height;
        deactivation_height_ = deactivation_height;
        gravity_ = gravity;
        return true;
    }

    // True if enough messages have been received so all data is initialized.
    bool ready() const { return msg_count_ > 0; }

    // Get number of Vicon messages received
    size_t get_num_msgs() const { return msg_count_; }

    // Get most recent position
    Eigen::Vector3d q() const { return estimate_.x.head(3); }

    // Get most recent velocity
    Eigen::Vector3d v() const { return estimate_.x.tail(3); }

    // Reset the estimator (estimate goes back to initial conditions)
    void reset() {
        msg_count_ = 0;
        active_ = false;
    }

    // Update the estimator at time t with measurement y of the projectile's
    // position.
    void update(const double t, const Eigen::Vector3d& y) {
        // We assume the projectile is in flight (i.e. subject to gravitational
        // acceleration) if it is above a certain height and has not yet
        // reached a certain minimum height. Otherwise, we assume acceleration
        // is zero.
        // TODO probably better to use the estimate for this
        if (active_ && y(2) <= deactivation_height_) {
            active_ = false;
        }
        if (!active_ && y(2) >= activation_height_) {
            active_ = true;
        }

        if (msg_count_ == 0) {
            // Initialize the estimate
            estimate_.x = Eigen::VectorXd::Zero(6);
            estimate_.x.head(3) = y;
            estimate_.P = Eigen::MatrixXd::Identity(6, 6);  // TODO P0
        } else if (msg_count_ >= 1) {
            double dt = t - t_prev_;

            // Compute system matrices
            Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

            Eigen::MatrixXd A = Eigen::MatrixXd::Identity(6, 6);
            A.topRightCorner(3, 3) = dt * I;

            Eigen::MatrixXd B(6, 3);
            B << 0.5 * dt * dt * I, dt * I;

            Eigen::MatrixXd C = Eigen::MatrixXd::Zero(3, 6);
            C.leftCols(3) = I;

            // Compute process and measurement noise covariance
            Eigen::MatrixXd Q = proc_var_ * B * B.transpose();
            Eigen::MatrixXd R = meas_var_ * I;

            Eigen::Vector3d u = Eigen::Vector3d::Zero();
            if (active_) {
                u = gravity_;
            }

            // Predict new state
            // We know that the ball cannot penetrate the floor, so we don't
            // let it
            kf::GaussianEstimate prediction =
                kf::predict(estimate_, A, Q, B * u);
            if (prediction.x(2) <= 0) {
                prediction.x(2) = 0;
            }

            // Update the estimate if measurement falls within likely range
            const double nis = kf::nis(prediction, C, R, y);
            if (nis >= nis_bound_) {
                ROS_WARN_STREAM("Rejected position = " << y.transpose());
                estimate_ = prediction;
                return;
            } else {
                estimate_ = kf::correct(prediction, C, R, y);
            }
        }

        t_prev_ = t;
        ++msg_count_;
    }

   private:
    // Store last received time and configuration for numerical differentiation
    double t_prev_;

    // Process and noise variance
    double proc_var_;
    double meas_var_;

    Eigen::Vector3d gravity_;

    // State estimate and Kalman filter
    kf::GaussianEstimate estimate_;

    bool active_ = false;

    // Height above which the object is "activated" and considered to be
    // undergoing projectile motion
    double activation_height_;

    // Height below which the object is "deactivated" and considered to be done
    // with or about to be done with projectile motion (i.e. it is about to fit
    // the ground)
    double deactivation_height_;

    // Measurement is rejected if NIS is above this bound
    double nis_bound_;

    // Number of messages received
    size_t msg_count_ = 0;

};  // class ProjectileViconEstimator

class ProjectileViconEstimatorNode {
   public:
    ProjectileViconEstimatorNode() {}

    void init(ros::NodeHandle& nh) {
        Eigen::Vector3d gravity(0, 0, -9.81);

        // Load parameters
        std::string vicon_object_name;
        double proc_var, meas_var, nis_bound, activation_height, deactivation_height;
        nh.param<std::string>("/projectile/vicon_object_name",
                              vicon_object_name, "ThingProjectile");
        nh.param<double>("/projectile/proc_var", proc_var, 1.0);
        nh.param<double>("/projectile/meas_var", meas_var, 1e-4);
        nh.param<double>("/projectile/activation_height", activation_height,
                         1.0);
        nh.param<double>("/projectile/deactivation_height", deactivation_height,
                         0.2);

        ROS_INFO_STREAM("projectile proc var = " << proc_var);
        ROS_INFO_STREAM("projectile meas var = " << meas_var);

        // 14.156 corresponds to 3-sigma bound for 3-dim Gaussian variable
        nh.param<double>("/projectile/nis_bound", nis_bound, 14.156);

        const std::string vicon_topic =
            "/vicon/" + vicon_object_name + "/" + vicon_object_name;
        vicon_sub_ = nh.subscribe(
            vicon_topic, 1, &ProjectileViconEstimatorNode::vicon_cb, this);

        // This is a topic rather than a service, because we want to use this
        // from simulation while use_sim_time=True
        reset_sub_ =
            nh.subscribe("/projectile/reset_estimate", 1,
                         &ProjectileViconEstimatorNode::reset_cb, this);

        joint_states_pub_ = nh.advertise<sensor_msgs::JointState>(
            "/projectile/joint_states", 1);

        estimator_.init(nh, proc_var, meas_var, nis_bound, activation_height,
                        deactivation_height, gravity);
    }

    // Spin, publishing the most recent estimate at the specified rate.
    void spin(ros::Rate& rate) {
        // Wait until the estimator has enough information
        while (ros::ok() && !estimator_.ready()) {
            ros::spinOnce();
            rate.sleep();
        }

        while (ros::ok()) {
            ros::spinOnce();
            publish_joint_states(estimator_.q(), estimator_.v());
            rate.sleep();
        }
    }

   private:
    void reset_cb(const std_msgs::Empty&) {
        estimator_.reset();
        ROS_INFO("Projectile estimate reset.");
    }

    void vicon_cb(const geometry_msgs::TransformStamped& msg) {
        // Get the current joint configuration
        double t = msg.header.stamp.toSec();
        Eigen::Vector3d y;
        y << msg.transform.translation.x, msg.transform.translation.y,
            msg.transform.translation.z;

        estimator_.update(t, y);
    }

    void publish_joint_states(const Eigen::Vector3d& q,
                              const Eigen::Vector3d& v) {
        sensor_msgs::JointState msg;
        msg.header.stamp = ros::Time::now();
        msg.name = {"x", "y", "z"};

        for (int i = 0; i < 3; ++i) {
            msg.position.push_back(q(i));
            msg.velocity.push_back(v(i));
        }

        joint_states_pub_.publish(msg);
    }

    // Subscribers to Vicon pose and reset hook.
    ros::Subscriber vicon_sub_;
    ros::Subscriber reset_sub_;

    // Publisher for position and velocity.
    ros::Publisher joint_states_pub_;

    // Underlying estimator
    ProjectileViconEstimator estimator_;
};

class ProjectileROSInterface {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ProjectileROSInterface() {}

    void init(ros::NodeHandle& nh) {
        joint_state_sub_ =
            nh.subscribe("/projectile/joint_states", 1,
                         &ProjectileROSInterface::joint_state_cb, this);
    }

    bool ready() const { return joint_states_received_; }

    Eigen::Vector3d q() const { return q_; }

    Eigen::Vector3d v() const { return v_; }

   private:
    void joint_state_cb(const sensor_msgs::JointState& msg) {
        for (int i = 0; i < 3; ++i) {
            q_[i] = msg.position[i];
            v_[i] = msg.velocity[i];
        }
        joint_states_received_ = true;
    }

    bool joint_states_received_ = false;

    ros::Subscriber joint_state_sub_;

    Eigen::Vector3d q_;
    Eigen::Vector3d v_;
};

}  // namespace mm
