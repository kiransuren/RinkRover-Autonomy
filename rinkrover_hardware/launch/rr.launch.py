from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os
import xacro

def generate_launch_description():

    # Get URDF via xacro
    urdf_file_name = 'robot.urdf.xacro'

    print('urdf_file_name : {}'.format(urdf_file_name))

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('rinkrover_description'))
    xacro_file = os.path.join(pkg_path,'description',urdf_file_name)
    robot_description_config = xacro.process_file(xacro_file)
    
    # Create a robot_state_publisher node (transform tree)
    robot_description = {'robot_description': robot_description_config.toxml()}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("rinkrover_description"),
            "config",
            "tricycle_drive_controller.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ]
    )
    robot_state_pub_tricycle_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", '--param-file', robot_controllers],
    )

    robot_tricycle_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["tricycle_controller",'--param-file', robot_controllers],
    )
    
    
    # add twist_to_twist_stamped
    twist_to_twist_stamped = Node(
        package='twist_to_twist_stamped',
        executable='twist_to_twist_stamped'
    )

    ld = LaunchDescription()
    ld.add_action(control_node)
    ld.add_action(twist_to_twist_stamped)
    ld.add_action(robot_state_pub_tricycle_node)
    ld.add_action(robot_tricycle_controller_spawner)
    ld.add_action(joint_state_broadcaster_spawner)

    return ld
