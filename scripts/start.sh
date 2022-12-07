#!/bin/bash

#ign gazebo -v 3 -r ~/src/gazebo/worlds/sensor_tutorial/sensor_tutorial_fortress.sdf &
ign gazebo -v 3 -r ~/src/ros2overlays/gazeboworlds/two_walls.sdf &
sleep 5
ros2 run ros_ign_bridge parameter_bridge /lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan --ros-args -r /lidar:=/laser_scan &
ros2 run ros_ign_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist &
echo "source ~/src/ros2overlays/setup.bash and run: ros2 run lidar_monitor subscriber"
ros2 run teleop_twist_keyboard teleop_twist_keyboard
