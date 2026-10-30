// Skid-steer odometry with an EFFECTIVE wheel separation and slip-aware covariance.
// Header-only, no ROS dependency (host-testable).
#pragma once
#include <array>
#include <cmath>
#include <cstdint>

namespace gdn {

struct SkidSteerParams {
  double wheel_radius_m = 0.06;
  int ticks_per_rev = 1440;          // encoder CPR x gearbox x quadrature
  double track_m = 0.28;             // tape-measure separation
  double effective_track_m = 0.37;   // calibrated (Run 2): what actually predicts yaw. Usually > track_m
  // Covariance model: odometry gets less trustworthy the harder it turns (scrub = slip).
  double sigma_v_frac = 0.02;        // 2% of speed
  double sigma_w_base = 0.02;        // rad/s
  double sigma_w_per_turn = 0.25;    // extra yaw-rate sigma per rad/s of commanded turn
};

struct OdomState {
  double x = 0, y = 0, yaw = 0;
  double v = 0, w = 0;
  std::array<double, 2> twist_var{0, 0};  // var(v), var(w)
};

class SkidSteerOdometry {
 public:
  explicit SkidSteerOdometry(SkidSteerParams p = {}) : p_(p) {}

  // Feed cumulative encoder counts (handles int32 wraparound).
  const OdomState& update(int32_t left_ticks, int32_t right_ticks, double dt) {
    if (!init_) { last_l_ = left_ticks; last_r_ = right_ticks; init_ = true; return s_; }
    const double m_per_tick = 2.0 * M_PI * p_.wheel_radius_m / p_.ticks_per_rev;
    const double dl = static_cast<int32_t>(left_ticks - last_l_) * m_per_tick;
    const double dr = static_cast<int32_t>(right_ticks - last_r_) * m_per_tick;
    last_l_ = left_ticks; last_r_ = right_ticks;
    const double ds = 0.5 * (dl + dr);
    const double dyaw = (dr - dl) / p_.effective_track_m;
    // Midpoint (second-order) integration: better than Euler on turns.
    s_.x += ds * std::cos(s_.yaw + 0.5 * dyaw);
    s_.y += ds * std::sin(s_.yaw + 0.5 * dyaw);
    s_.yaw = std::atan2(std::sin(s_.yaw + dyaw), std::cos(s_.yaw + dyaw));
    if (dt > 0) { s_.v = ds / dt; s_.w = dyaw / dt; }
    const double sv = p_.sigma_v_frac * std::fabs(s_.v) + 0.005;
    const double sw = p_.sigma_w_base + p_.sigma_w_per_turn * std::fabs(s_.w);
    s_.twist_var = {sv * sv, sw * sw};
    return s_;
  }

  // Run 2 calibration: drive in place N full turns (truth from IMU/LiDAR yaw), solve for effective track.
  static double calibrate_effective_track(double left_dist_m, double right_dist_m, double true_yaw_rad) {
    return (right_dist_m - left_dist_m) / true_yaw_rad;
  }

  const OdomState& state() const { return s_; }

 private:
  SkidSteerParams p_;
  OdomState s_;
  int32_t last_l_ = 0, last_r_ = 0;
  bool init_ = false;
};

}  // namespace gdn
