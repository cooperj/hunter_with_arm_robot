from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    start_joint_controller = LaunchConfiguration("start_joint_controller")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    launch_rviz = LaunchConfiguration("launch_rviz")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    world_file = LaunchConfiguration("world_file")
    launch_gazebo = LaunchConfiguration("launch_gazebo")

    model_path = os.path.join(
        get_package_share_directory(runtime_config_package.perform(context)), "models"
    )
    if "GAZEBO_MODEL_PATH" in os.environ:
        model_path = os.environ["GAZEBO_MODEL_PATH"] + ":" + model_path

    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controllers_file]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "view_robot.rviz"]
    )

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare(description_package), "urdf", description_file
        ]),
        " ",
        "safety_limits:=", safety_limits,
        " ",
        "safety_pos_margin:=", safety_pos_margin,
        " ",
        "safety_k_position:=", safety_k_position,
        " ",
        "name:=ur",
        " ",
        "ur_type:=", ur_type,
        " ",
        "tf_prefix:=", prefix,
        " ",
        "sim_gazebo:=true",
        " ",
        "simulation_controllers:=", initial_joint_controllers,
    ])
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        ),
        condition=IfCondition(launch_rviz),
    )

    initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/controller_manager"],
        condition=IfCondition(start_joint_controller),
    )
    initial_joint_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/controller_manager", "--stopped"],
        condition=UnlessCondition(start_joint_controller),
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]
        ),
        launch_arguments={
            "world": world_file,
            "gui": gazebo_gui,
            "verbose": "true",
        }.items(),
        condition=IfCondition(launch_gazebo)
    )

    gz_spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-entity", "ur",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.0",
        ],
        condition=IfCondition(launch_gazebo)
    )

    return [
        SetEnvironmentVariable(name="GAZEBO_MODEL_PATH", value=model_path),
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        initial_joint_controller_spawner_stopped,
        initial_joint_controller_spawner_started,
        gazebo,
        gz_spawn_entity,
    ]

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("ur_type", default_value="ur5e"),
        DeclareLaunchArgument("safety_limits", default_value="true"),
        DeclareLaunchArgument("safety_pos_margin", default_value="0.15"),
        DeclareLaunchArgument("safety_k_position", default_value="20"),
        DeclareLaunchArgument("runtime_config_package", default_value="ur_simulation_gz"),
        DeclareLaunchArgument("controllers_file", default_value="ur_controllers.yaml"),
        DeclareLaunchArgument("description_package", default_value="ur_description"),
        DeclareLaunchArgument("description_file", default_value="ur.urdf.xacro"),
        DeclareLaunchArgument("prefix", default_value='""'),
        DeclareLaunchArgument("start_joint_controller", default_value="true"),
        DeclareLaunchArgument("initial_joint_controller", default_value="joint_trajectory_controller"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument("gazebo_gui", default_value="true"),
        DeclareLaunchArgument("world_file", default_value="empty.world"),
        DeclareLaunchArgument("launch_gazebo", default_value="true"),
    ]
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
