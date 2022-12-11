#pragma once

#include <Eigen/Eigen>

namespace mm {

struct GaussianEstimate {
    Eigen::VectorXd x;
    Eigen::MatrixXd P;
};

// Kalman Filter class, which assumes a constant continuous-time linear model
// but (potentially) time-varying process and measurement noise.
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

    // Prediction step: predict next state given a apriori model and input
    GaussianEstimate predict(const GaussianEstimate& e,
                             const Eigen::VectorXd& u, const Eigen::MatrixXd& Q,
                             double dt) {
        // Discretize the system for current timestep
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(e.x.rows(), e.x.rows());
        Eigen::MatrixXd Ad = I + dt * A_;
        Eigen::MatrixXd Bd = dt * B_;

        // Predict using motion model
        GaussianEstimate prediction;
        prediction.x = Ad * e.x + Bd * u;
        prediction.P = Ad * e.P * Ad.transpose() + Q;
        return prediction;
    }

    // Compute normalized innovation squared, which can be used for chi-squared
    // hypothesis testing (i.e. reject bad measurements)
    double nis(const GaussianEstimate& e, const Eigen::VectorXd& y, const Eigen::MatrixXd& R) {
        // Innovation
        Eigen::VectorXd v = y - C_ * e.x;

        // Innovation covariance
        Eigen::MatrixXd S = C_ * e.P * C_.transpose() + R;
        Eigen::LLT<Eigen::MatrixXd> LLT = S.llt();

        return v.dot(LLT.solve(v));
    }

    // Correction step: fuse measured output with predicted state
    GaussianEstimate correct(const GaussianEstimate& e,
                             const Eigen::VectorXd& y, const Eigen::MatrixXd& R,
                             double dt) {
        // Innovation covariance
        // Use Cholesky decomposition instead of computing the matrix inverse
        // in Kalman gain directly
        Eigen::MatrixXd S = C_ * e.P * C_.transpose() + R;
        Eigen::LLT<Eigen::MatrixXd> LLT = S.llt();

        // Correct using measurement model
        GaussianEstimate correction;
        correction.P = e.P - e.P * C_.transpose() * LLT.solve(C_ * e.P);
        correction.x = e.x + e.P * C_.transpose() * LLT.solve(y - C_ * e.x);
        return correction;
    }

   private:
    Eigen::MatrixXd A_;
    Eigen::MatrixXd B_;
    Eigen::MatrixXd C_;
};

}  // namespace mm
