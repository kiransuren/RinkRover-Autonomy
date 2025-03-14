# RinkRover-Autonomy
ROS2 Autonomy Stack for RinkRover

## RPI5 Jazzy Specifc Instructions
### Mapping Mode
MAPPING MODE:
VSCODE Terminal:
```
colcon build --packages-ignore rinkrover_gazebo
```

Open 5 Terminal Tabs:
```
[1] 	rviz2
[2 SSH] source install/setup.bash && ros2 launch rinkrover_hardware rr.launch.py
[3 SSH] source install/setup.bash && ros2 run rr_teleop_twist_keyboard rr_teleop_twist_keyboard
[4 SSH] source install/setup.bash && ros2 launch rplidar_ros rplidar_c1_launch.py
[5 SSH] source install/setup.bash && ros2 launch rinkrover_hardware online_async_launch.py
[6 SSH] source install/setup.bash && ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/home/pi/ros2_ws/RinkRover-Autonomy/MAP_NAME'}"
```
### MAKE SURE TO UPDATE THE MAP NAME IN THE YAML FILE! ###
### LOCALIZATION MODE:
```
[1] 	rviz2
[2 SSH] source install/setup.bash && ros2 launch rinkrover_hardware rr.launch.py
[3 SSH] source install/setup.bash &&  ros2 run twist_to_twist_stamped twist_to_twist_stamped 
[4 SSH] source install/setup.bash && ros2 launch rplidar_ros rplidar_c1_launch.py
[5 SSH] source install/setup.bash && ros2 launch rinkrover_hardware localization_launch.py
[6 SSH] source install/setup.bash && ros2 launch rinkrover_hardware navigation.launch.py
```

## Important First-Time Installations
Install ROS2 Humble:
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
```
sudo rosdep init
rosdep update
rosdep install --from-paths . --ignore-src -r -y

sudo apt-get install gz-fortress ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-twist-mux
```

Install ROS2 jazzy (for RPI5):
```
sudo apt-get install ros-jazzy-slam-toolbox ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-twist-mux
```

## Running Gazebo Simulation
1. Build the rinkrover_gazebo project to ensure you have the latest version to run
```
colcon build --packages-select rinkrover_gazebo rinkrover_description

source install/setup.bash
```

2. Launch the gazebo simulation for the rink rectangle map
```
ros2 launch rinkrover_gazebo rinkrover_rectangle.launch.py
```

3. Teleoperate rinkrover via keyboard
```
source install/setup.bash && ros2 run rr_teleop_twist_keyboard rr_teleop_twist_keyboard
```

## Running SLAM

1. Start SLAM Online Asynchronous Mapping
```
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=./mapping_online_async.yaml use_sim_time:=true
```
2. Add map and SlamToolboxPlugin to RVIZ2
3. Drive around and map data
4. Use "serialize map" to save map for future use

5. Start SLAM Localization Mode (update yaml file with map file name and start pose)
```
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=./localization_online_async.yaml use_sim_time:=true
```

## Running RinkRover Hardware System

Launch RinkRover Hardware and Control System
```
ros2 launch rinkrover_hardware rr.launch.py
```
Remember to turn on/off use_sim_time=True/False in tricycle_drive_controller.yaml!