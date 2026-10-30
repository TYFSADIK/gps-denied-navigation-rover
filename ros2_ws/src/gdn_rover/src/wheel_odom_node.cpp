// ROS 2 node: encoder ticks -> nav_msgs/Odometry twist with slip-aware covariance.
// Publishes TWIST only for robot_localization (two_d_mode); pose integration is done by the EKF.
#include <chrono>
#include <memory>

#include "gdn_rover/skid_steer.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

class WheelOdomNode : public rclcpp::Node {
 public:
  WheelOdomNode() : Node("wheel_odom") {
    gdn::SkidSteerParams p;
    p.wheel_radius_m = declare_parameter("wheel_radius_m", p.wheel_radius_m);
    p.ticks_per_rev = declare_parameter("ticks_per_rev", p.ticks_per_rev);
    p.effective_track_m = declare_parameter("effective_track_m", p.effective_track_m);
    odom_ = std::make_unique<gdn::SkidSteerOdometry>(p);
    pub_ = create_publisher<nav_msgs::msg::Odometry>("wheel/odom", rclcpp::SensorDataQoS());
    sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
        "wheel/ticks", rclcpp::SensorDataQoS(),
        [this](std_msgs::msg::Int32MultiArray::SharedPtr m) { on_ticks(*m); });
  }

 private:
  void on_ticks(const std_msgs::msg::Int32MultiArray& m) {
    if (m.data.size() < 2) return;
    const auto now = get_clock()->now();
    const double dt = last_.nanoseconds() ? (now - last_).seconds() : 0.0;
    last_ = now;
    const auto& s = odom_->update(m.data[0], m.data[1], dt);
    nav_msgs::msg::Odometry o;
    o.header.stamp = now;
    o.header.frame_id = "odom";
    o.child_frame_id = "base_link";
    o.twist.twist.linear.x = s.v;
    o.twist.twist.angular.z = s.w;
    o.twist.covariance.fill(0.0);
    o.twist.covariance[0] = s.twist_var[0];
    o.twist.covariance[7] = 1e-4;     // vy ~ 0 (non-holonomic), still allow a little
    o.twist.covariance[35] = s.twist_var[1];
    for (int i : {14, 21, 28}) o.twist.covariance[i] = 1e6;  // unused dims
    pub_->publish(o);
  }

  std::unique_ptr<gdn::SkidSteerOdometry> odom_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_;
  rclcpp::Time last_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelOdomNode>());
  rclcpp::shutdown();
}
