#include <ros/ros.h>

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "mobile_manipulation_central/robot_interfaces.h"

using namespace pybind11::literals;
using namespace mm;

// Initialize the ROS node and return the node handle
// This is used to interact with roscpp from Python
ros::NodeHandle init_node(const std::string& name,
                          std::vector<std::string> args) {
    int argc = args.size();
    std::vector<char*> c_args;
    c_args.reserve(argc);
    for (auto& s : args) {
        c_args.push_back(const_cast<char*>(s.c_str()));
    }
    ros::init(argc, c_args.data(), name);
    return ros::NodeHandle();
}

ros::NodeHandle init_node(const std::string& name) {
    std::vector<std::string> args;
    return init_node(name, args);
}

PYBIND11_MODULE(bindings, m) {
    pybind11::class_<ros::NodeHandle>(m, "NodeHandle").def(pybind11::init<>());

    m.def("init_node", pybind11::overload_cast<const std::string&>(&init_node),
          "name"_a);
    m.def("init_node",
          pybind11::overload_cast<const std::string&, std::vector<std::string>>(
              &init_node),
          "name"_a, "args"_a);

    pybind11::class_<RidgebackROSInterface>(m, "RidgebackROSInterface")
        .def(pybind11::init<ros::NodeHandle&>(), "nh"_a)
        .def("nq", &RidgebackROSInterface::nq)
        .def("nv", &RidgebackROSInterface::nv)
        .def("q", &RidgebackROSInterface::q)
        .def("v", &RidgebackROSInterface::v)
        .def("brake", &RidgebackROSInterface::brake)
        .def("ready", &RidgebackROSInterface::ready)
        .def("publish_cmd_vel", &RidgebackROSInterface::publish_cmd_vel,
             "cmd_vel"_a);

    pybind11::class_<UR10ROSInterface>(m, "UR10ROSInterface")
        .def(pybind11::init<ros::NodeHandle&>(), "nh"_a)
        .def("nq", &UR10ROSInterface::nq)
        .def("nv", &UR10ROSInterface::nv)
        .def("q", &UR10ROSInterface::q)
        .def("v", &UR10ROSInterface::v)
        .def("brake", &UR10ROSInterface::brake)
        .def("ready", &UR10ROSInterface::ready)
        .def("publish_cmd_vel", &UR10ROSInterface::publish_cmd_vel,
             "cmd_vel"_a);

    pybind11::class_<MobileManipulatorROSInterface>(
        m, "MobileManipulatorROSInterface")
        .def(pybind11::init<ros::NodeHandle&>(), "nh"_a)
        .def("nq", &MobileManipulatorROSInterface::nq)
        .def("nv", &MobileManipulatorROSInterface::nv)
        .def("q", &MobileManipulatorROSInterface::q)
        .def("v", &MobileManipulatorROSInterface::v)
        .def("brake", &MobileManipulatorROSInterface::brake)
        .def("ready", &MobileManipulatorROSInterface::ready)
        .def("publish_cmd_vel", &MobileManipulatorROSInterface::publish_cmd_vel,
             "cmd_vel"_a);
}
