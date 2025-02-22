from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Launch configuration variables specific to simulation
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    # Declare the launch arguments
    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Specify namespace of the robot')

    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Specify namespace of the robot')
    
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('rinkrover_description'),
            'config',
            'tricycle_drive_controller.yaml',
        ]
    )

    start_gazebo_ros_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'rinkrover_robot',
            '-topic', 'robot_description',
            '-allow_renaming', 'true',
            '-x', x_pose,
            '-y', y_pose,
            '-z', '3'
        ],
        output='screen',
    )
    
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--param-file',
            robot_controllers,
            ],
    )
    tricycle_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'tricycle_controller',
            '--param-file',
            robot_controllers,
            ],
    )

    # joint_state_broadcaster_spawner = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    #          'joint_state_broadcaster'],
    #     output='screen'
    # )

    # tricycle_controller_spawner = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    #          'tricycle_controller'],
    #     output='screen'
    # )
    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)

    ld.add_action(bridge)

    # Add any conditioned actions
    ld.add_action(start_gazebo_ros_spawner_cmd)
    
    ld.add_action(        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=start_gazebo_ros_spawner_cmd,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ))
    ld.add_action(        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[tricycle_controller_spawner],
            )
        ))

    return ld
