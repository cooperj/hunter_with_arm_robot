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
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    FindExecutable,
    PathJoinSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory("mobile_manipulator_pltf_description")

    # Launch configurations
    world_path = LaunchConfiguration(
        "world_path", default=os.path.join(pkg_dir, "worlds", "empty_world.world")
    )
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    x_pose = LaunchConfiguration("x_pose", default="0.0")
    y_pose = LaunchConfiguration("y_pose", default="0.0")
    roll = LaunchConfiguration("roll", default="0.0")
    pitch = LaunchConfiguration("pitch", default="0.0")
    yaw = LaunchConfiguration("yaw", default="1.45")
    use_rviz = LaunchConfiguration("use_rviz", default="true")
    use_gazebo = LaunchConfiguration("use_gazebo", default="true")
    use_base = LaunchConfiguration("use_base", default="false")
    use_arm = LaunchConfiguration("use_arm", default="true")
    sim_gazebo = LaunchConfiguration("sim_gazebo", default="true")
    tf_prefix = LaunchConfiguration("tf_prefix", default="")
    safety_limits = LaunchConfiguration("safety_limits", default="true")
    ur_type = LaunchConfiguration("ur_type", default="ur5e")
    name = LaunchConfiguration("name", default="ur")

    gazebo_params_file = os.path.join(pkg_dir, "config", "gazebo_params.yaml")

    # Unified robot_description
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("mobile_manipulator_pltf_description"),
                    "description",
                    "mobile_manipulator_pltf.urdf.xacro",
                ]
            ),
            " ",
            "is_sim:=",
            use_sim_time,
            " ",
            "use_base:=",
            use_base,
            " ",
            "use_arm:=",
            use_arm,
            " ",
            "name:=",
            name,
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "safety_limits:=",
            safety_limits,
            " ",
            "tf_prefix:=",
            tf_prefix,
            " ",
            "sim_gazebo:=",
            sim_gazebo,
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # Nodes
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("gazebo_ros"),
                    "launch",
                    "gazebo.launch.py",
                )
            ]
        ),
        launch_arguments={
            "world": world_path,
            "extra_gazebo_args": "--ros-args --params-file " + gazebo_params_file,
        }.items(),
        condition=IfCondition(use_gazebo),
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "mobile_manipulator",
            "-x",
            x_pose,
            "-y",
            y_pose,
            "-z",
            "0.01",
            "-R",
            roll,
            "-P",
            pitch,
            "-Y",
            yaw,
        ],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_gazebo),
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
        ],
        output="screen",
    )

    load_tricycle_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "ackermann_like_controller",
        ],
        output="screen",
        condition=IfCondition(use_base),
    )

    load_arm_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "arm_joint_trajectory_controller",
        ],
        output="screen",
        condition=IfCondition(use_arm),
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("mobile_manipulator_pltf_description"),
                        "launch",
                        "ur_classic_moveit.launch.py",
                    ]
                )
            ]
        ),
        condition=IfCondition(use_arm),
        launch_arguments={
            "ur_type": ur_type,
            "safety_limits": safety_limits,
            "runtime_config_package": "ur_simulation_gz",
            "controllers_file": "ur_controllers.yaml",
            "description_package": "ur_description",
            "description_file": "ur.urdf.xacro",
            "moveit_config_package": "ur_moveit_config",
            "moveit_config_file": "ur.srdf.xacro",
            "prefix": "",
            "launch_rviz": "false",
            "launch_gazebo": "false",
        }.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("hunter_with_arm_description"),
                "rviz/robot_view.rviz",
            ),
        ],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("x_pose", default_value="20.0"),
            DeclareLaunchArgument("y_pose", default_value="-4.0"),
            DeclareLaunchArgument("roll", default_value="0.0"),
            DeclareLaunchArgument("pitch", default_value="0.0"),
            DeclareLaunchArgument("yaw", default_value="1.45"),
            DeclareLaunchArgument("world_path", default_value=world_path),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("use_gazebo", default_value="true"),
            DeclareLaunchArgument("use_base", default_value="false"),
            DeclareLaunchArgument("use_arm", default_value="true"),
            DeclareLaunchArgument("ur_type", default_value="ur5e"),
            DeclareLaunchArgument("safety_limits", default_value="true"),
            DeclareLaunchArgument("name", default_value="ur"),
            DeclareLaunchArgument("tf_prefix", default_value=""),
            DeclareLaunchArgument("sim_gazebo", default_value="true"),
            spawn_entity,
            node_robot_state_publisher,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn_entity, on_exit=[load_joint_state_broadcaster]
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=load_joint_state_broadcaster,
                    on_exit=[load_tricycle_controller],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=load_joint_state_broadcaster,
                    on_exit=[load_arm_controller],
                )
            ),
            moveit_launch,
            gazebo,
            rviz,
        ]
    )
