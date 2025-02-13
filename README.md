# RinkRover-Autonomy
ROS2 Autonomy Stack for RinkRover

## Running Gazebo Simulation
1. Build the rinkrover_gazebo project to ensure you have the latest version to run
```
colcon build --packages-select rinkrover_gazebo

source install/setup.bash
```

2. Launch the gazebo simulation for the rink rectangle map
```
ros2 launch rinkrover_gazebo rinkrover_rectangle.launch.py
```

3. Teleoperate rinkrover via keyboard
'''
ros2 run turtlebot3_teleop teleop_keyboard
'''

4. Start SLAM Online Asynchronous Mapping
'''
ros2 launch slam_toolbox online_async_launch.py params_file:=./mapper_params_online_async.yaml use_sim_time:=true
'''