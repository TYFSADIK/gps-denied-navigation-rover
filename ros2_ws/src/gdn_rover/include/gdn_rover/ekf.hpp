// Planar EKF, independent cross-check of robot_localization.
// State: [x, y, yaw, v, w]. Inputs: wheel twist (v, w), IMU yaw rate (w) + absolute yaw,
// LiDAR odometry pose (x, y, yaw) from rf2o. Mahalanobis gating rejects slip spikes.
#pragma once
#include <Eigen/Dense>
#include <cmath>

namespace gdn {

class PlanarEKF {
 public:
  using Vec5 = Eigen::Matrix<double, 5, 1>;
  using Mat5 = Eigen::Matrix<double, 5, 5>;

  PlanarEKF() {
    x_.setZero();
    P_ = Mat5::Identity() * 1e-3;
    q_ << 0.01, 0.01, 0.005, 0.5, 0.8;  // process noise spectral densities
  }

  void predict(double dt) {
    const double c = std::cos(x_(2)), s = std::sin(x_(2)), v = x_(3);
    Mat5 F = Mat5::Identity();
    F(0, 2) = -v * s * dt; F(0, 3) = c * dt;
    F(1, 2) = v * c * dt;  F(1, 3) = s * dt;
    F(2, 4) = dt;
    x_(0) += v * c * dt;
    x_(1) += v * s * dt;
    x_(2) = wrap(x_(2) + x_(4) * dt);
    Mat5 Q = (q_ * dt).asDiagonal();
    P_ = F * P_ * F.transpose() + Q;
  }

  // Generic linear update on selected state indices; returns false if gated out.
  template <int M>
  bool update(const Eigen::Matrix<double, M, 1>& z, const Eigen::Matrix<double, M, 5>& H,
              const Eigen::Matrix<double, M, M>& R, int yaw_row = -1, double gate_chi2 = 16.0) {
    Eigen::Matrix<double, M, 1> y = z - H * x_;
    if (yaw_row >= 0) y(yaw_row) = wrap(y(yaw_row));
    const Eigen::Matrix<double, M, M> S = H * P_ * H.transpose() + R;
    const double d2 = y.transpose() * S.ldlt().solve(y);
    last_nis_ = d2;
    if (d2 > gate_chi2) { ++rejected_; return false; }
    const Eigen::Matrix<double, 5, M> K = P_ * H.transpose() * S.inverse();
    x_ += K * y;
    x_(2) = wrap(x_(2));
    const Mat5 IKH = Mat5::Identity() - K * H;
    P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();  // Joseph form
    return true;
  }

  bool update_wheel_twist(double v, double w, double var_v, double var_w) {
    Eigen::Matrix<double, 2, 5> H = Eigen::Matrix<double, 2, 5>::Zero();
    H(0, 3) = 1; H(1, 4) = 1;
    return update<2>(Eigen::Vector2d(v, w), H, Eigen::Vector2d(var_v, var_w).asDiagonal());
  }

  bool update_imu(double yaw, double w, double var_yaw, double var_w) {
    Eigen::Matrix<double, 2, 5> H = Eigen::Matrix<double, 2, 5>::Zero();
    H(0, 2) = 1; H(1, 4) = 1;
    return update<2>(Eigen::Vector2d(yaw, w), H, Eigen::Vector2d(var_yaw, var_w).asDiagonal(), 0);
  }

  bool update_lidar_pose(double x, double y, double yaw, double var_xy, double var_yaw) {
    Eigen::Matrix<double, 3, 5> H = Eigen::Matrix<double, 3, 5>::Zero();
    H(0, 0) = 1; H(1, 1) = 1; H(2, 2) = 1;
    return update<3>(Eigen::Vector3d(x, y, yaw), H, Eigen::Vector3d(var_xy, var_xy, var_yaw).asDiagonal(), 2);
  }

  const Vec5& x() const { return x_; }
  const Mat5& P() const { return P_; }
  int rejected() const { return rejected_; }
  double last_nis() const { return last_nis_; }

  static double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }

 private:
  Vec5 x_, q_;
  Mat5 P_;
  int rejected_ = 0;
  double last_nis_ = 0;
};

}  // namespace gdn
