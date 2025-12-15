import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Package directories
    mobile_manipulator_gazebo_pkg_dir = get_package_share_directory('mobile_manipulator_pltf_gazebo')

    # Launch configurations
    world_path = LaunchConfiguration(
        'world_path', default=os.path.join(mobile_manipulator_gazebo_pkg_dir, 'worlds', 'empty_world.world')
    )
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    roll = LaunchConfiguration('roll', default='0.0')
    pitch = LaunchConfiguration('pitch', default='0.0')
    yaw = LaunchConfiguration('yaw', default='1.45')
    use_rviz = LaunchConfiguration('use_rviz', default='false')
    use_gazebo = LaunchConfiguration('use_gazebo', default='true')

    # Gazebo parameters
    gazebo_params_file = os.path.join(mobile_manipulator_gazebo_pkg_dir, 'config', 'gazebo_params.yaml')

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("mobile_manipulator_pltf_description"), "description" ,"mobile_manipulator_pltf.urdf.xacro"]
            ),
            " ",
            "is_sim:=",
             use_sim_time,
             " ",
            "prefix:=''",
            " ",
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }
    # Create a robot_state_publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={
            'world': world_path,
            'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file
        }.items(),
        condition=IfCondition(use_gazebo)
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'hunter_with_arm_gazebo',
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01',
            '-R', roll,
            '-P', pitch,
            '-Y', yaw,
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_tricycle_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'ackermann_like_controller'],
        output='screen'
    )
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d',
            os.path.join(os.path.join(get_package_share_directory('hunter_with_arm_description')), 'rviz/robot_view.rviz'),
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz)
    )

   # Launch description
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock'),
        DeclareLaunchArgument('x_pose', default_value='20.0', description='Start x position'),
        DeclareLaunchArgument('y_pose', default_value='-4.0', description='Start y position'),
        DeclareLaunchArgument('roll', default_value='0.0', description='Start roll angle'),
        DeclareLaunchArgument('pitch', default_value='0.0', description='Start pitch angle'),
        DeclareLaunchArgument('yaw', default_value='1.45', description='Start yaw angle'),
        DeclareLaunchArgument('world_path', default_value=world_path, description='Gazebo world file path'),
        DeclareLaunchArgument('use_rviz', default_value='false', description='Whether to start RViZ'),
        DeclareLaunchArgument('use_gazebo', default_value='true', description='Whether to start Gazebo'),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_tricycle_controller],
            )
        ),
        gazebo,
        rviz,
        node_robot_state_publisher,
        spawn_entity,
    ])
