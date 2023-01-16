#include <iostream>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <urdf_parser/urdf_parser.h>

#include <ros/package.h>

#include <mobile_manipulation_central/kinematics.h>


int main() {
    const std::string package_path = ros::package::getPath("mobile_manipulation_central");
    const std::string urdf_path = package_path + "/urdf/compiled/thing_no_wheels.urdf";
    const std::string tool_link_name = "gripper";

    // Joint configuration
    mm::RobotKinematics<double>::VecX q(9);
    q << 0, 0, 0, 1.5708, -0.7854, 1.5708, -0.7854, 1.5708, 1.3100;

    // Parse URDF of the model
    ::urdf::ModelInterfaceSharedPtr urdf_tree =
        ::urdf::parseURDFFile(urdf_path);
    if (urdf_tree == nullptr) {
        throw std::invalid_argument("The file " + urdf_path +
                                    " does not contain a valid URDF model.");
    }

    // Add mobile base DOFs to the model
    mm::RobotKinematics<double>::Model model;
    pinocchio::JointModelComposite root_joint(3);
    root_joint.addJoint(pinocchio::JointModelPX());
    root_joint.addJoint(pinocchio::JointModelPY());
    root_joint.addJoint(pinocchio::JointModelRZ());
    pinocchio::urdf::buildModel(urdf_tree, root_joint, model);

    // Compute EE pose and Jacobian
    mm::RobotKinematics<double> kinematics(model, tool_link_name);
    kinematics.forward(q);
    std::cout << "Position = " << kinematics.position().transpose() << std::endl;
    std::cout << "Rotation = " << std::endl << kinematics.rotation() << std::endl;
    std::cout << "Jacobian = " << std::endl << kinematics.jacobian(q) << std::endl;

    return 0;
}
