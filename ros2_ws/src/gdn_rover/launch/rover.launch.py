"""ros2 launch gdn_rover rover.launch.py run:=4"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    run = LaunchConfiguration("run")
    cfg = PathJoinSubstitution([FindPackageShare("gdn_rover"), "config",
                                PythonExpression(["'ekf_run3_encoders_imu.yaml' if '", run,
                                                  "' == '3' else 'ekf_run4_plus_lidar.yaml'"])])
    return LaunchDescription([
        DeclareLaunchArgument("run", default_value="4"),
        Node(package="gdn_rover", executable="wheel_odom_node", parameters=[{"effective_track_m": 0.37}]),
        Node(package="ldlidar_stl_ros2", executable="ldlidar_stl_ros2_node", name="ld19",
             parameters=[{"product_name": "LDLiDAR_LD19", "topic_name": "scan", "port_name": "/dev/ttyUSB0",
                          "frame_id": "laser", "laser_scan_dir": True}]),
        Node(package="rf2o_laser_odometry", executable="rf2o_laser_odometry_node",
             parameters=[{"laser_scan_topic": "/scan", "odom_topic": "/odom_rf2o", "publish_tf": False,
                          "base_frame_id": "base_link", "odom_frame_id": "odom", "freq": 10.0}]),
        Node(package="robot_localization", executable="ekf_node", name="ekf_filter_node", parameters=[cfg]),
        Node(package="tf2_ros", executable="static_transform_publisher",
             arguments=["0.1", "0", "0.15", "0", "0", "0", "base_link", "laser"]),
    ])
