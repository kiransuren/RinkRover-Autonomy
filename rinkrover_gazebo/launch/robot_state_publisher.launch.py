import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf_file_name = 'robot.urdf.xacro'

    print('urdf_file_name : {}'.format(urdf_file_name))

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('rinkrover_description'))
    xacro_file = os.path.join(pkg_path,'description',urdf_file_name)
    robot_description_config = xacro.process_file(xacro_file)
    
    # Create a robot_state_publisher node (transform tree)
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        node_robot_state_publisher
    ])
