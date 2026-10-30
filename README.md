# GPS-Denied Navigation Rover

Private research repo. A ROS 2 rover that keeps navigating after GNSS is cut. It fuses wheel odometry, a BNO085 IMU, and 2D LiDAR (rf2o) in an Extended Kalman Filter, and measures drift against u-blox truth.

Full write-up: [`docs/project-page.md`](docs/project-page.md)

## What's here

| Path | Language | Purpose |
|---|---|---|
| `ros2_ws/src/gdn_rover/include/gdn_rover/skid_steer.hpp` | C++17 | Skid-steer odometry: **effective track width**, midpoint integration, encoder wraparound, slip-aware twist covariance (yaw sigma grows with turn rate) |
| `.../ekf.hpp` | C++17 / Eigen | Independent planar EKF (x, y, yaw, v, ω) with Mahalanobis gating and Joseph update, used to cross-check robot_localization |
| `.../src/wheel_odom_node.cpp` | C++ / rclcpp | `wheel/ticks` → `wheel/odom` (twist + covariance) |
| `.../config/ekf_run3_*.yaml`, `ekf_run4_*.yaml` | YAML | robot_localization configs for Run 3 (encoders + IMU) and Run 4 (+ LiDAR witness). GNSS is recorded but never fused |
| `.../launch/rover.launch.py` | Python | LD19 + rf2o + EKF + wheel odometry, selected with `run:=3` or `run:=4` |
| `gdn_eval/trajectory.py` | Python | ATE/RPE with **Umeyama alignment on the pre-cut segment only**, error at +30/60/120 s, drift % of distance |
| `gdn_eval/export.py` | Python | ROS 2 bag → CSV without ROS (`rosbags`) |

## Quickstart

```bash
# Host tests (no ROS needed; brew install eigen)
c++ -std=c++17 -O2 -Iros2_ws/src/gdn_rover/include -I$(brew --prefix eigen)/include/eigen3     ros2_ws/src/gdn_rover/test/test_core.cpp -o /tmp/gdn_test && /tmp/gdn_test
python -m unittest -v

# On the rover (ROS 2 Jazzy)
cd ros2_ws && colcon build --symlink-install && source install/setup.bash
ros2 launch gdn_rover rover.launch.py run:=4
```

## Next steps
- [ ] Run 2: calibrate `effective_track_m` with in-place turns (`calibrate_effective_track`) on grass, gravel, and snow, and keep one value per surface
- [ ] Publish encoder ticks from the motor controller as `std_msgs/Int32MultiArray` on `wheel/ticks`
- [ ] Run 5 GNSS cut: bag everything, then `export` → `trajectory --cut <t>`
- [ ] Add scan-match health (rf2o fails on featureless snow) and inflate LiDAR covariance when few points are returned
- [ ] Move to 3D: KISS-ICP or FAST-LIO on a 3D LiDAR (see `gps-denied-cargo-ugv`)
