// Plain-assert tests (also build under ament_cmake_gtest as a main()-style test).
#include <cassert>
#include <cmath>
#include <cstdio>
#include <random>
#include "gdn_rover/ekf.hpp"
#include "gdn_rover/skid_steer.hpp"

int main() {
  // 1. Straight line: 1 m per side -> 1 m forward, no yaw
  {
    gdn::SkidSteerParams p; gdn::SkidSteerOdometry o(p);
    const int t1m = int(p.ticks_per_rev / (2 * M_PI * p.wheel_radius_m));
    o.update(0, 0, 0.1); auto s = o.update(t1m, t1m, 1.0);
    assert(std::fabs(s.x - 1.0) < 0.01 && std::fabs(s.yaw) < 1e-9);
  }
  // 2. Encoder wraparound
  {
    gdn::SkidSteerOdometry o; o.update(INT32_MAX - 10, INT32_MAX - 10, 0.1);
    auto s = o.update(INT32_MIN + 9, INT32_MIN + 9, 0.1);
    assert(s.x > 0 && s.x < 0.01);
  }
  // 3. Effective-track calibration
  assert(std::fabs(gdn::SkidSteerOdometry::calibrate_effective_track(-1.0, 1.0, 2 * M_PI * 0.86) - 0.37) < 0.01);
  // 4. EKF: biased wheel yaw-rate drifts; IMU + LiDAR pull it back; slip spike gated
  {
    std::mt19937 rng(1); std::normal_distribution<double> n(0, 1);
    gdn::PlanarEKF prop, full;
    double tx = 0, ty = 0, tyaw = 0;
    for (int k = 0; k < 3000; ++k) {  // 60 s @ 50 Hz, 0.5 m/s, gentle turn
      const double dt = 0.02, v = 0.5, w = 0.05;
      tyaw += w * dt; tx += v * std::cos(tyaw) * dt; ty += v * std::sin(tyaw) * dt;
      for (auto* f : {&prop, &full}) {
        f->predict(dt);
        f->update_wheel_twist(v * 1.03 + 0.01 * n(rng), w + 0.02 + 0.01 * n(rng), 1e-4, 4e-4);  // slip bias
        f->update_imu(gdn::PlanarEKF::wrap(tyaw + 0.01 * n(rng)), w + 0.005 * n(rng), 1e-4, 2.5e-5);
      }
      if (k % 5 == 0) full.update_lidar_pose(tx + 0.02 * n(rng), ty + 0.02 * n(rng), tyaw + 0.01 * n(rng), 4e-4, 1e-4);
    }
    const double e_prop = std::hypot(prop.x()(0) - tx, prop.x()(1) - ty);
    const double e_full = std::hypot(full.x()(0) - tx, full.x()(1) - ty);
    std::printf("60 s drift: encoders+IMU %.2f m, +LiDAR %.3f m\n", e_prop, e_full);
    assert(e_full < e_prop && e_full < 0.2);
    const int before = full.rejected();
    full.update_wheel_twist(3.0, 0.0, 1e-4, 1e-4);  // wheel spin in mud
    assert(full.rejected() == before + 1);
  }
  std::puts("gdn_rover core tests passed");
  return 0;
}
