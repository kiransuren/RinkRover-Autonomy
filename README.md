# RinkRover-Autonomy
ROS2 Autonomy Stack for RinkRover

## Important First-Time Installations
Install ROS2 Humble:
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
```
sudo rosdep init
rosdep update
rosdep install --from-paths . --ignore-src -r -y
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

Start SLAM Online Asynchronous Mapping
```
ros2 launch slam_toolbox online_async_launch.py params_file:=./mapper_params_online_async.yaml use_sim_time:=true
```

## Running RinkRover Hardware System

Launch RinkRover Hardware and Control System
```
ros2 launch rinkrover_hardware rr.launch.py
```

Remember to turn on/off use_sim_time=True/False in tricycle_drive_controller!