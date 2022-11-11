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

### Simulated robot car: attempts to get it working

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

### Simulated robot car startup w/ notes and intermediate steps

This works: Gazebo 6 (Fortress) with corresponding fortress sdf file:
	
Fortress (earlier) version of tutorial uses ignition.msgs.Twist:
sensor_tutorial_fortress.sdf

ign gazebo -v 3 -r ~/src/gazebo/worlds/sensor_tutorial/sensor_tutorial_fortress.sdf

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

# Monitor lidar for obstacles
cd src/ros2overlays/
source install/setup.bash 
ros2 run lidar_monitor subscriber

## Simulated robot car startup

For notes and intermediate steps, see above.

Gazebo 6 (Fortress) with corresponding fortress sdf file:

ign gazebo -v 3 -r ~/src/gazebo/worlds/sensor_tutorial/sensor_tutorial_fortress.sdf
ros2 run ros_ign_bridge parameter_bridge /lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan --ros-args -r /lidar:=/laser_scan
ros2 run ros_ign_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist
ros2 run teleop_twist_keyboard teleop_twist_keyboard
cd src/ros2overlays/
source install/setup.bash 
ros2 run lidar_monitor subscriber

## Tasks

### Task 1: Drive and spin

#### Task 1a: Drive

#### Task 1b: Drive and spin

### Task 2: Bump and go

from state / input / to state:

stopped / clear / driving
driving / obstacle any dir within 3m / spinning
spinning / clear ahead / driving

### Task 3: Mill around the object

Min distance from wall: about 1.5 meters
Lidar can see fine at 6 meters (likely more)
Get within 4 meters
Then stay 2 - 4 m from object
Lidar can see back to about 4:00/8:00.

Figure out states/transitions for this:

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




