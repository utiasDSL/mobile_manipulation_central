#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include "mobile_manipulation_central/kalman_filter.h"

using namespace pybind11::literals;
using namespace mm;

PYBIND11_MODULE(bindings, m) {
    pybind11::class_<kf::GaussianEstimate>(m, "GaussianEstimate")
        .def(pybind11::init<const Eigen::VectorXd&, const Eigen::MatrixXd&>(), "x"_a, "P"_a)
        .def_readwrite("x", &kf::GaussianEstimate::x)
        .def_readwrite("P", &kf::GaussianEstimate::P);

    m.def("kf_predict", &kf::predict);
    m.def("kf_correct", &kf::correct);
    m.def("kf_nis", &kf::nis);
    m.def("kf_predict_and_correct", &kf::predict_and_correct);
}
