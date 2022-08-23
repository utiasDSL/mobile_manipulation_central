#pragma once

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Eigen>

namespace mm {

std::map<std::string, size_t> JOINT_INDEX_MAP = {
    {"ur10_arm_shoulder_pan_joint", 0}, {"ur10_arm_shoulder_lift_joint", 1},
    {"ur10_arm_elbow_joint", 2},        {"ur10_arm_wrist_1_joint", 3},
    {"ur10_arm_wrist_2_joint", 4},      {"ur10_arm_wrist_3_joint", 5}};

// TODO make a corresponding Vicon object class

class RobotROSInterface {
   public:
    RobotROSInterface(size_t nq, size_t nv) : nq_(nq), nv_(nv) {
        q_ = Eigen::VectorXd::Zero(nq);
        v_ = Eigen::VectorXd::Zero(nv);
    }

    size_t nq() { return nq_; };
    size_t nv() { return nv_; };

    virtual Eigen::VectorXd q() { return q_; }

    virtual Eigen::VectorXd v() { return v_; }

    virtual void brake() { publish_cmd_vel(Eigen::VectorXd::Zero(nv_)); }

    virtual bool ready() { return joint_states_received_; }

    virtual void publish_cmd_vel(const Eigen::VectorXd& cmd_vel) = 0;

   protected:
    size_t nq_;
    size_t nv_;

    Eigen::VectorXd q_;
    Eigen::VectorXd v_;

    bool joint_states_received_ = false;

    ros::Subscriber joint_state_sub_;
    ros::Publisher cmd_pub_;
};

class RidgebackROSInterface : public RobotROSInterface {
   public:
    RidgebackROSInterface(ros::NodeHandle& nh) : RobotROSInterface(3, 3) {
        joint_state_sub_ =
            nh.subscribe("ridgeback/joint_states", 1,
                         &RidgebackROSInterface::joint_state_cb, this);

        cmd_pub_ =
            nh.advertise<geometry_msgs::Twist>("ridgeback/cmd_vel", 1, true);
    }

    void publish_cmd_vel(const Eigen::VectorXd& cmd_vel) override {
        if (cmd_vel.rows() != nv_) {
            throw std::runtime_error("Ridgeback given cmd_vel of wrong shape.");
        }

        geometry_msgs::Twist msg;
        msg.linear.x = cmd_vel(0);
        msg.linear.y = cmd_vel(1);
        msg.angular.z = cmd_vel(2);
        cmd_pub_.publish(msg);
    }

   private:
    void joint_state_cb(const sensor_msgs::JointState& msg) {
        for (int i = 0; i < msg.name.size(); ++i) {
            q_[i] = msg.position[i];
            v_[i] = msg.velocity[i];
        }
        joint_states_received_ = true;
    }
};

class UR10ROSInterface : public RobotROSInterface {
   public:
    UR10ROSInterface(ros::NodeHandle& nh) : RobotROSInterface(6, 6) {
        joint_state_sub_ = nh.subscribe(
            "ur10/joint_states", 1, &UR10ROSInterface::joint_state_cb, this);

        cmd_pub_ =
            nh.advertise<std_msgs::Float64MultiArray>("ur10/cmd_vel", 1, true);
    }

    void publish_cmd_vel(const Eigen::VectorXd& cmd_vel) override {
        if (cmd_vel.rows() != nv_) {
            throw std::runtime_error("UR10 given cmd_vel of wrong shape.");
        }

        std_msgs::Float64MultiArray msg;
        msg.data = std::vector<double>(cmd_vel.data(),
                                       cmd_vel.data() + cmd_vel.rows());
        cmd_pub_.publish(msg);
    }

   private:
    void joint_state_cb(const sensor_msgs::JointState& msg) {
        for (int i = 0; i < msg.name.size(); ++i) {
            size_t j = JOINT_INDEX_MAP.at(msg.name[i]);
            q_[j] = msg.position[i];
            v_[j] = msg.velocity[i];
        }
        joint_states_received_ = true;
    }
};

class MobileManipulatorROSInterface : public RobotROSInterface {
   public:
    MobileManipulatorROSInterface(ros::NodeHandle& nh)
        : base_(nh),
          arm_(nh),
          RobotROSInterface(base_.nq() + arm_.nq(), base_.nv() + arm_.nv()) {}

    Eigen::VectorXd q() override {
        Eigen::VectorXd q(nq_);
        q << base_.q(), arm_.q();
        return q;
    }

    Eigen::VectorXd v() override {
        Eigen::VectorXd v(nv_);
        v << base_.v(), arm_.v();
        return v;
    }

    void brake() override {
        base_.brake();
        arm_.brake();
    }

    bool ready() override { return base_.ready() && arm_.ready(); }

    void publish_cmd_vel(const Eigen::VectorXd& cmd_vel) override {
        base_.publish_cmd_vel(cmd_vel.head(base_.nv()));
        arm_.publish_cmd_vel(cmd_vel.tail(arm_.nv()));
    }

   private:
    RidgebackROSInterface base_;
    UR10ROSInterface arm_;
};

}  // namespace mm
