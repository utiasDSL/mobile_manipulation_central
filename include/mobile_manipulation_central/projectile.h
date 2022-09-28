#pragma once

#include <ros/console.h>
#include <ros/ros.h>
#include <Eigen/Eigen>

#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_datatypes.h>

#include <mobile_manipulation_central/exponential_smoothing.h>

namespace mm {

struct Estimate {
    Eigen::VectorXd x;
    Eigen::MatrixXd P;
};

// Kalman Filter class, which assumes a continuous-time linear model but
// (potentially) time-varying process and measurement noise.
class KalmanFilter {
   public:
    KalmanFilter() {}

    void init(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
              const Eigen::MatrixXd& C) {
        // \dot{x} = Ax + Bu + Gaussian noise
        //       y = Cx + Gaussian noise
        A_ = A;
        B_ = B;
        C_ = C;
    }

    Estimate predict(const Estimate& e, const Eigen::VectorXd& u,
                     const Eigen::MatrixXd& Q, double dt) {
        // Discretize the system for current timestep
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(e.x.rows(), e.x.rows());
        Eigen::MatrixXd Ad = I + dt * A_;
        Eigen::MatrixXd Bd = dt * B_;

        // Predict using motion model
        Estimate prediction;
        prediction.x = Ad * e.x + Bd * u;
        prediction.P = Ad * e.P * Ad.transpose() + Q;
        return prediction;
    }

    Estimate correct(const Estimate& e, const Eigen::VectorXd& y,
                     const Eigen::MatrixXd& R, double dt) {
        // Use Cholesky decomposition instead of computing the matrix inverse
        // in Kalman gain directly
        Eigen::MatrixXd G = C_ * e.P * C_.transpose() + R;
        Eigen::LLT<Eigen::MatrixXd> LLT = G.llt();

        // Correct using measurement model
        Estimate correction;
        correction.P = e.P - e.P * C_.transpose() * LLT.solve(C_ * e.P);
        correction.x = e.x + e.P * C_.transpose() * LLT.solve(y - C_ * e.x);
        return correction;
    }

   private:
    Eigen::MatrixXd A_;
    Eigen::MatrixXd B_;
    Eigen::MatrixXd C_;
};

// The node listens to raw Vicon transform messages for a projectile object and
// converts them to a JointState message for the translational component. For
// the moment we assume we don't care about the projectile orientation and so
// do not do anything with it.
class ProjectileViconEstimator {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ProjectileViconEstimator() {}

    // Start the node.
    bool init(ros::NodeHandle& nh, double pos_meas_var, double pos_proc_var,
              double vel_proc_var, const Eigen::Vector3d& gravity) {

        pos_meas_var_ = pos_meas_var;
        pos_proc_var_ = pos_proc_var;
        vel_proc_var_ = vel_proc_var;
        g_ = gravity;

        // Initialize the Kalman filter
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(6, 6);
        A.topRightCorner(3, 3) = Eigen::MatrixXd::Identity(3, 3);
        Eigen::MatrixXd B = Eigen::MatrixXd::Zero(6, 3);
        B.bottomRows(3) = Eigen::MatrixXd::Identity(3, 3);
        Eigen::MatrixXd C = Eigen::MatrixXd::Identity(6, 6);
        kf_.init(A, B, C);

        std::string vicon_topic;
        nh.param<std::string>("vicon_topic", vicon_topic,
                              "/vicon/Projectile/Projectile");
        vicon_sub_ = nh.subscribe(vicon_topic, 1,
                                  &ProjectileViconEstimator::vicon_cb, this);

        return true;
    }

    // True if enough messages have been received so all data is initialized.
    bool ready() const { return msg_count_ > 1; }

    // Get number of Vicon messages received
    size_t get_num_msgs() const { return msg_count_; }

    // Get most recent position
    Eigen::Vector3d q() const { return estimate_.x.head(3); }

    // Get most recent velocity
    Eigen::Vector3d v() const { return estimate_.x.tail(3); }

   private:
    /* FUNCTIONS */

    void vicon_cb(const geometry_msgs::TransformStamped& msg) {
        // Get the current joint configuration
        double t = msg.header.stamp.toSec();
        Eigen::Vector3d q_meas;
        q_meas << msg.transform.translation.x, msg.transform.translation.y,
            msg.transform.translation.z;

        // Wait until we have at least two messages so we can numerically
        // differentiate.
        if (msg_count_ >= 1) {
            double dt = t - t_prev_;

            // Skip this measurement if not enough time has elapsed, to avoid
            // numerical problems.
            if (dt < 1e-6) {
                ROS_WARN("Time between Vicon messages is very small.");
                return;
            }

            // Compute velocity via numerical differentiation
            Eigen::Vector3d v_meas = (q_meas - q_prev_) / dt;

            // Measurement of the state
            Eigen::VectorXd y(6);
            y << q_meas, v_meas;

            // Compute process and measurement noise covariance (time-dependent)
            Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

            Eigen::MatrixXd R(6, 6);
            R << pos_meas_var_ * I, 0 * I, 0 * I,
                2 * pos_meas_var_ * I / (dt * dt);

            // TODO shouldn't this be squared?
            Eigen::MatrixXd Q(6, 6);
            Q << dt * pos_proc_var_ * I, 0 * I, 0 * I, dt * vel_proc_var_ * I;

            // Apply Kalman filter to estimate state
            if (msg_count_ == 1) {
                estimate_.x = y;
                estimate_.P = R;
            } else {
                estimate_ = kf_.predict(estimate_, g_, Q, dt);
                estimate_ = kf_.correct(estimate_, y, R, dt);
            }
        }

        t_prev_ = t;
        q_prev_ = q_meas;
        ++msg_count_;
    }

    /* VARIABLES */

    // Subscriber to Vicon pose.
    ros::Subscriber vicon_sub_;

    // Store last received time and configuration for numerical differentiation
    double t_prev_;
    Eigen::Vector3d q_prev_;

    // state estimate
    Estimate estimate_;

    // process and noise variance
    double pos_meas_var_;
    double pos_proc_var_;
    double vel_proc_var_;

    Eigen::Vector3d g_;  // gravity
    KalmanFilter kf_;

    // Number of messages received.
    size_t msg_count_ = 0;

};  // class ProjectileViconEstimator

class ProjectileViconEstimatorNode {
   public:
    ProjectileViconEstimatorNode() {}

    bool init(ros::NodeHandle& nh, const std::string& name) {
        double pos_meas_var = 1e-2;
        double pos_proc_var = 1.0;
        double vel_proc_var = 1.0;
        Eigen::Vector3d gravity(0, 0, -9.81);

        estimator_.init(nh, pos_meas_var, pos_proc_var, vel_proc_var, gravity);

        joint_states_pub_ =
            nh.advertise<sensor_msgs::JointState>(name + "/joint_states", 1);

        return true;
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

    // Publisher for position and velocity.
    ros::Publisher joint_states_pub_;

    ProjectileViconEstimator estimator_;
};

class ProjectileROSInterface {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ProjectileROSInterface(ros::NodeHandle& nh, const std::string& name) {
        joint_state_sub_ =
            nh.subscribe(name + "/joint_states", 1,
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
