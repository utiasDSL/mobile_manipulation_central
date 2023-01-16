#pragma once

#include <Eigen/Eigen>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace mm {

// Wrapper around Pinocchio for robot kinematics.
template <typename Scalar>
class RobotKinematics {
   public:
    using Model =
        pinocchio::ModelTpl<Scalar, 0, pinocchio::JointCollectionDefaultTpl>;
    using Data =
        pinocchio::DataTpl<Scalar, 0, pinocchio::JointCollectionDefaultTpl>;

    using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
    using Mat3 = Eigen::Matrix<Scalar, 3, 3>;
    using VecX = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
    using MatX = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;

    RobotKinematics(Model& model, const std::string& tool_link_name)
        : model_(model), data_(model) {
        tool_idx_ = model.getBodyId(tool_link_name);
    }

    void forward(const VecX& q) {
        pinocchio::forwardKinematics(model_, data_, q);
        pinocchio::updateFramePlacements(model_, data_);
    }

    Vec3 position() const { return data_.oMf[tool_idx_].translation(); }

    Mat3 rotation() const { return data_.oMf[tool_idx_].rotation(); }

    MatX jacobian(const VecX& q) {
        MatX J = MatX::Zero(6, model_.nq);
        pinocchio::computeFrameJacobian(
            model_, data_, q, tool_idx_,
            pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J);
        return J;
    }

   private:
    Model model_;
    Data data_;
    pinocchio::JointIndex tool_idx_;
};

}  // namespace mm
