# ros2overlays

Misc projects and tutorials created/used during the ROS2 learning process.

## Notes regarding current work-in-progress

### Using Gazebo

https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Ignition.html

ign gazebo -v 4 -r visualize_lidar.sdf

ros2 run ros_ign_bridge parameter_bridge /model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist
ros2 run ros_ign_bridge parameter_bridge /lidar2@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan --ros-args -r /lidar2:=/laser_scan

cd src/ros2overlays
source install/setup.bash
ros2 run lidar_monitor subscriber

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/model/vehicle_blue/cmd_vel

### Other worlds and robots to explore

https://gazebosim.org/docs/garden/ros2_integration

