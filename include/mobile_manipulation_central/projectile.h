#pragma once

#include <ros/console.h>
#include <ros/ros.h>
#include <Eigen/Eigen>

#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Empty.h>
#include <tf/transform_datatypes.h>

#include <mobile_manipulation_central/kalman_filter.h>

namespace mm {

// The node listens to raw Vicon transform messages for a projectile object and
// converts them to a JointState message for the translational component. For
// the moment we assume we don't care about the projectile orientation and so
// do not do anything with it.
class ProjectileViconEstimator {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ProjectileViconEstimator() {}

    bool init(ros::NodeHandle& nh, double pos_proc_var, double vel_proc_var,
              double pos_meas_var, const Eigen::Vector3d& gravity) {
        pos_proc_var_ = pos_proc_var;
        vel_proc_var_ = vel_proc_var;
        pos_meas_var_ = pos_meas_var;
        gravity_ = gravity;

        // Initialize the Kalman filter
        // A = |0 I|  B = |0|  C = I
        //     |0 0|      |I|
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(6, 6);
        A.topRightCorner(3, 3) = Eigen::MatrixXd::Identity(3, 3);
        Eigen::MatrixXd B = Eigen::MatrixXd::Zero(6, 3);
        B.bottomRows(3) = Eigen::MatrixXd::Identity(3, 3);
        Eigen::MatrixXd C = Eigen::MatrixXd::Identity(6, 6);
        kf_.init(A, B, C);

        std::string vicon_topic;
        nh.param<std::string>("vicon_topic", vicon_topic,
                              "/vicon/Projectile/Projectile");
        nh.param<double>("activation_height", activation_height_, 1.0);
        nh.param<double>("deactivation_height", deactivation_height_, 0.2);

        // 30 m/s is approximately the velocity of an object dropped from rest
        // from a height of 3 meters once it reaches the floor. We should never
        // go beyond that here.
        nh.param<double>("max_projectile_velocity", max_velocity_, 30);

        vicon_sub_ = nh.subscribe(vicon_topic, 1,
                                  &ProjectileViconEstimator::vicon_cb, this);

        // This is a topic rather than a service, because we want to use this
        // from simulation while use_sim_time=True
        reset_sub_ = nh.subscribe("reset_projectile_estimate", 1,
                                  &ProjectileViconEstimator::reset_cb, this);

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

    void reset_cb(const std_msgs::Empty&) {
        std::cout << "Projectile estimate reset." << std::endl;
        msg_count_ = 0;
        active_ = false;
    }

    void vicon_cb(const geometry_msgs::TransformStamped& msg) {
        // Get the current joint configuration
        double t = msg.header.stamp.toSec();
        Eigen::Vector3d q_meas;
        q_meas << msg.transform.translation.x, msg.transform.translation.y,
            msg.transform.translation.z;

        // bool switched = false;

        // We assume the projectile is in flight (i.e. subject to gravitational
        // acceleration) if it is above a certain height and has not yet
        // reached a certain minimum height. Otherwise, we assume acceleration
        // is zero.
        if (active_ && q_meas(2) <= deactivation_height_) {
            active_ = false;
            // switched = true;
        }
        if (!active_ && q_meas(2) >= activation_height_) {
            active_ = true;
            // reset the estimate here?
            // switched = true;
        }

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

            // clang-format off
            Eigen::MatrixXd R(6, 6);
            R << pos_meas_var_ * I, 0 * I,
                             0 * I, 2 * pos_meas_var_ * I / (dt * dt);

            Eigen::MatrixXd Q(6, 6);
            Q << dt * dt * pos_proc_var_ * I, 0 * I,
                                       0 * I, dt * dt * vel_proc_var_ * I;
            // clang-format on

            // Reset estimated covariance if we switch in or out of projectile
            // flight
            // if (switched) {
            //     estimate_.P = R;
            // }

            // Apply Kalman filter to estimate state
            // if (v_meas.norm() > max_velocity_) {
            //     // Reset the estimate when we have a large velocity change
            //     // XXX This is a bit of a hack to deal with resetting
            //     obstacles
            //     // in simulation, which results in large state
            //     discontinuities estimate_.x << q_meas, 0, 0, 0; estimate_.P =
            //     R;
            // } else if (msg_count_ == 1 || !active_) {

            // Reject discontinuous jumps in the measurement
            if ((q_meas - q_prev_).norm() > 0.3) {
                std::cout << "rejected q_meas = " << q_meas.transpose()
                          << std::endl;
                t_prev_ = t;
                return;
            } else if (msg_count_ == 1 || !active_) {
                estimate_.x = y;
                estimate_.P = R;
            } else {
                const GaussianEstimate prediction =
                    kf_.predict(estimate_, gravity_, Q, dt);
                estimate_ = kf_.correct(prediction, y, R, dt);
            }

            // TODO clean up
            // if (msg_count_ == 1) {
            //     estimate_.x = y;
            //     estimate_.P = R;
            // } else {
            //     if (!active_) {
            //         // TODO this is actually just the previous estimate
            //         if ((q_meas - q_prev_).norm() > 0.2) {
            //             return;
            //         }
            //         estimate_.x = y;
            //         estimate_.P = R;
            //     } else {
            //         const GaussianEstimate prediction =
            //             kf_.predict(estimate_, gravity_, Q, dt);
            //         const Eigen::Vector3d q_pred = prediction.x.head(3);
            //         // If measurement disagrees too much with prediction,
            //         then
            //         // we reject it
            //         if ((q_meas - q_pred).norm() > 0.2) {
            //             estimate_ = prediction;
            //             return;
            //         } else {
            //             estimate_ = kf_.correct(prediction, y, R, dt);
            //         }
            //     }
            // }
            //     Eigen::Vector3d u = Eigen::Vector3d::Zero();
            //     if (active_) {
            //         u = gravity_;
            //     }
            //     const GaussianEstimate prediction = kf_.predict(estimate_, u,
            //     Q, dt); const double nis = kf_.nis(prediction, y, R);
            //
            //     if (nis >= 16.812) {
            //         // do nothing
            //         std::cout << "rejected: q_meas = " << q_meas.transpose()
            //         << std::endl;
            //     // } else if (!active_) {
            //     //     // If not active we don't do any estimation, we just
            //     directly
            //     //     // take the measured value (we don't care about super
            //     accurate
            //     //     // estimates when the object is not undergoing
            //     projectile
            //     //     // motion)
            //     //     // TODO this is pretty questionable given the NIS test
            //     //     estimate_.x = y;
            //     //     estimate_.P = R;
            //     } else {
            //         // Eigen::Vector3d u = Eigen::Vector3d::Zero();
            //         // if (active_) {
            //         //     u = gravity_;
            //         // }
            //         // const GaussianEstimate prediction =
            //         kf_.predict(estimate_, gravity_, Q, dt);
            //
            //         // Only update the prediction if we pass chi-squared test
            //         // Pr(X >= 16.812) = 0.01 => 99% chance of being an
            //         outlier
            //         // const double nis = kf_.nis(prediction, y, R);
            //         // if (nis < 16.812) {
            //             estimate_ = kf_.correct(prediction, y, R, dt);
            //         // }
            //
            //         // estimate_ = kf_.predict(estimate_, gravity_, Q, dt);
            //         // estimate_ = kf_.correct(estimate_, y, R, dt);
            //     }
            // }
        }

        t_prev_ = t;
        q_prev_ = q_meas;
        ++msg_count_;
    }

    /* VARIABLES */

    // Subscriber to Vicon pose.
    ros::Subscriber vicon_sub_;
    ros::Subscriber reset_sub_;

    // Store last received time and configuration for numerical differentiation
    double t_prev_;
    Eigen::Vector3d q_prev_;

    // Process and noise variance
    double pos_proc_var_;
    double vel_proc_var_;
    double pos_meas_var_;

    Eigen::Vector3d gravity_;

    // State estimate and Kalman filter
    GaussianEstimate estimate_;
    KalmanFilter kf_;

    bool active_ = false;

    // Height above which the object is "activated" and considered to be
    // undergoing projectile motion
    double activation_height_;

    // Height below which the object is "deactivated" and considered to be done
    // with or about to be done with projectile motion (i.e. it is about to fit
    // the ground)
    double deactivation_height_;

    // Maximum allowed measured velocity. The estimate is not updated if the
    // measured velocity if above this value.
    double max_velocity_;

    // Number of messages received
    size_t msg_count_ = 0;

};  // class ProjectileViconEstimator

class ProjectileViconEstimatorNode {
   public:
    ProjectileViconEstimatorNode() {}

    void init(ros::NodeHandle& nh, const std::string& name, double pos_proc_var,
              double vel_proc_var, double pos_meas_var) {
        Eigen::Vector3d gravity(0, 0, -9.81);
        estimator_.init(nh, pos_proc_var, vel_proc_var, pos_meas_var, gravity);

        joint_states_pub_ =
            nh.advertise<sensor_msgs::JointState>(name + "/joint_states", 1);
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
