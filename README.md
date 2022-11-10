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

Reading Lidar

Create bridge between ros and simulated robot lidar:
Gazebo topic: /lidar
ROS topic: /laser_scan
ros2 run ros_ign_bridge parameter_bridge /lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan --ros-args -r /lidar:=/laser_scan

Moving robot using ign topic

ign topic --topic /cmd_vel --msgtype ignition.msgs.Twist --pub "linear: {x: -0.0, y: 0.0, z: 9.0}, angular: {x: 0.0, y: 0.0, z: 0.1}"

Moving robot using teleop_twist_keyboard (ROS)

ros2 run ros_ign_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist
# (Both use the same topic name: /cmd_vel; no remap required)
ros2 run teleop_twist_keyboard teleop_twist_keyboard


Algorithm:

while (true)
	Find the wall
	Find parallel path
	Drive for a bit

Find the wall:
	until found:
		spin right

Find parallel path:
	until clear again
		slowly spin left

Drive for a bit:
	Straight ahead for a bit

