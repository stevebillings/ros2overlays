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

Racetrack: https://app.gazebosim.org/OpenRobotics/fuel/worlds/Prius%20on%20Sonoma%20Raceway
Download sonoma.sdf and reference it by file path.

### Current

https://gazebosim.org/docs/garden/install_ubuntu

https://raw.githubusercontent.com/gazebosim/docs/master/garden/tutorials/sensors/sensor_tutorial.sdf

ign topic -l

gazebo6 (Fortress; previous release):
ign gazebo -v 4 -r ~/src/gazebo/worlds/sensor_tutorial/sensor_tutorial.sdf
==> Errors ("[Err]") loading several libraries, including imu
==> This sdf is for gazebo7.

gazebo7 (Garden; latest release):
Gazebo 7 (which is what gz sim runs) seems to be Garden (which is the orig sdf using gz.msgs.Twist). 
gz sim -v 4 -r ~/src/gazebo/worlds/sensor_tutorial/sensor_tutorial.sdf
==> Warnings but no errors.
	TriggeredPublisher subscribed on /keyboard/keypress and publishing on /cmd_vel
	Does create /imu topic (and others)
==> No idea why I can't get this to work.

This works:
	
Switched to Fortress (earlier) version of tutorial, which uses ignition.msgs.Twist:
sensor_tutorial_fortress.sdf

gazebo 6 (ign gazebo) with fortress.sdf:
ign topic --topic /cmd_vel --msgtype ignition.msgs.Twist --pub "linear: {x: 0.1, y: 0.0, z: 9.0}"
ign topic --topic /imu --echo


