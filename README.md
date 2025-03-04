# RinkRover-Autonomy
ROS2 Autonomy Stack for RinkRover

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
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --param stamped:=true --remap cmd_vel:=/tricycle_controller/cmd_vel
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

Run twist mux to mux teleop and nav2 commands
```
ros2 run twist_mux twist_mux --ros-args --params-file twist_mux.yaml -r cmd_vel_out:=tricycle_controller/cmd_vel
```

Remember to turn on/off use_sim_time=True/False in tricycle_drive_controller.yaml!



## Deployment Commands Summary
```
ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true

source install/setup.bash && ros2 launch rinkrover_description online_async_launch.py

source install/setup.bash && ros2 launch rinkrover_gazebo rinkrover_rectangle.launch.py

source install/setup.bash && ros2 launch rinkrover_description navigation_humble.launch.py
```

## RPI5 Jazzy Specifc Instructions
```
colcon build --packages-ignore rinkrover_gazebo

ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true

source install/setup.bash && ros2 launch rinkrover_hardware rr.launch.py

source install/setup.bash && ros2 launch rinkrover_hardware online_async_launch.py

source install/setup.bash && ros2 launch rinkrover_hardware navigation.launch.py


```