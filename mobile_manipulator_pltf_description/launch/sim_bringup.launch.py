import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    FindExecutable,
    PathJoinSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
import yaml


def generate_launch_description():
    """
    Basic simulator launch file for mobile manipulator platform with MoveIt support.
    Uses empty_world.world from mobile_manipulator_pltf_description package.
    
    This launch file depends on mobile_manipulator_pltf_bringup package for MoveIt
    configuration (SRDF, controllers, kinematics, etc.). It provides a complete
    simulation environment suitable for use by external packages.
    """
    ld = LaunchDescription()

    pkg_dir = get_package_share_directory("mobile_manipulator_pltf_description")

    # Declare arguments
    x_arg = DeclareLaunchArgument(
        "x_pose", default_value="0.0", description="X position of the robot"
    )
    y_arg = DeclareLaunchArgument(
        "y_pose", default_value="0.0", description="Y position of the robot"
    )
    z_arg = DeclareLaunchArgument(
        "z_pose", default_value="0.01", description="Z position of the robot"
    )
    R_arg = DeclareLaunchArgument(
        "roll", default_value="0.0", description="Roll orientation of the robot"
    )
    P_arg = DeclareLaunchArgument(
        "pitch", default_value="0.0", description="Pitch orientation of the robot"
    )
    Y_arg = DeclareLaunchArgument(
        "yaw", default_value="1.45", description="Yaw orientation of the robot"
    )
    arm_name_arg = DeclareLaunchArgument(
        "name", default_value="ur", description="Name of the arm"
    )
    arm_type_arg = DeclareLaunchArgument(
        "ur_type", default_value="ur5", description="Type of UR arm"
    )
    safety_lim_arg = DeclareLaunchArgument(
        "safety_limits", default_value="true", description="Enable safety limits"
    )
    arm_tf_prefix_arg = DeclareLaunchArgument(
        "tf_prefix", default_value="", description="TF prefix for the arm"
    )
    mobile_base_prefix_arg = DeclareLaunchArgument(
        "prefix", default_value="", description="prefix for the mobile base"
    )
    use_gazebo_arg = DeclareLaunchArgument(
        "use_gazebo", default_value="true", description="Use Gazebo simulation"
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz", default_value="true", description="Use RViz tool"
    )
    use_base_arg = DeclareLaunchArgument(
        "use_base",
        default_value="true",
        description="Include the base in the simulation",
    )
    use_arm_arg = DeclareLaunchArgument(
        "use_arm",
        default_value="true",
        description="Include the arm in the simulation",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    # Launch configurations
    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")
    z_pose = LaunchConfiguration("z_pose")
    roll = LaunchConfiguration("roll")
    pitch = LaunchConfiguration("pitch")
    yaw = LaunchConfiguration("yaw")
    use_rviz = LaunchConfiguration("use_rviz")
    use_gazebo = LaunchConfiguration("use_gazebo")
    use_base = LaunchConfiguration("use_base")
    use_arm = LaunchConfiguration("use_arm")
    tf_prefix = LaunchConfiguration("tf_prefix")
    prefix = LaunchConfiguration("prefix")
    safety_limits = LaunchConfiguration("safety_limits")
    ur_type = LaunchConfiguration("ur_type")
    name = LaunchConfiguration("name")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # World file from mobile_manipulator_pltf_description package
    world_file = os.path.join(pkg_dir, "worlds", "empty_world.world")

    # Robot description
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
            "prefix:=",
            prefix,
            " ",
            "sim_gazebo:=",
            use_gazebo,
        ]
    )

    # Build MoveIt config (from mobile_manipulator_pltf_bringup package)
    # This provides SRDF, trajectory execution, kinematics, and planning pipelines
    moveit_config = (
        MoveItConfigsBuilder(
            "mobile_manipulator", package_name="mobile_manipulator_pltf_bringup"
        )
        .robot_description_semantic(file_path="config/ur.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
            publish_planning_scene=True,
        )
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    # Inject robot_description manually
    moveit_config.robot_description["robot_description"] = robot_description_content

    # Include Gazebo launch file
    gazebo_launch_file = os.path.join(
        get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py"
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={
            "use_sim_time": "true",
            "debug": "false",
            "gui": "true",
            "paused": "true",
            "world": world_file,
        }.items(),
        condition=IfCondition(use_gazebo),
    )

    # RViz configuration
    rviz_config_path = os.path.join(
        get_package_share_directory("mobile_manipulator_pltf_bringup"),
        "config",
        "moveit.rviz",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(use_rviz),
    )

    # Spawn the robot in Gazebo
    spawn_the_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            "mobile_manipulator",
            "-topic",
            "robot_description",
            "-x",
            x_pose,
            "-y",
            y_pose,
            "-z",
            z_pose,
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

    # Controller configuration
    joint_controllers_file = os.path.join(
        pkg_dir,
        "config",
        "ur5_controllers_gripper.yaml",
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, joint_controllers_file],
        output="screen",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description, {"use_sim_time": use_sim_time}],
        output="screen",
    )

    # Controller spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    ackermann_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "ackermann_like_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
        condition=IfCondition(use_base),
    )

    arm_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
        condition=IfCondition(use_arm),
    )

    gripper_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_position_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
        condition=IfCondition(use_arm),
    )

    # Sensor configuration for MoveIt
    sensor_yaml_path = os.path.join(
        get_package_share_directory("mobile_manipulator_pltf_bringup"),
        "config",
        "sensors_3d.yaml",
    )

    with open(sensor_yaml_path, "r") as f:
        sensor_config = yaml.safe_load(f)

    config_dict = moveit_config.to_dict()
    config_dict.update({"use_sim_time": True})

    config_dict["move_group"] = {
        **config_dict.get("move_group", {}),
        "sensors": sensor_config["sensors"],
        "default_sensor": sensor_config["default_sensor"],
        "octomap_frame": "base_link",
        "octomap_resolution": 0.02,
        "max_range": 5.0,
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[config_dict],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # Event handlers to sequence node startup
    delay_joint_state_broadcaster = RegisterEventHandler(
        OnProcessStart(
            target_action=controller_manager_node,
            on_start=[joint_state_broadcaster_spawner],
        )
    )

    delay_base_controller = RegisterEventHandler(
        OnProcessStart(
            target_action=joint_state_broadcaster_spawner,
            on_start=[ackermann_controller_spawner],
        )
    )

    delay_arm_controller = RegisterEventHandler(
        OnProcessStart(
            target_action=joint_state_broadcaster_spawner,
            on_start=[arm_trajectory_controller_spawner],
        )
    )

    delay_gripper_controller = RegisterEventHandler(
        OnProcessStart(
            target_action=joint_state_broadcaster_spawner,
            on_start=[gripper_position_controller_spawner],
        )
    )

    delay_rviz_node = RegisterEventHandler(
        OnProcessStart(
            target_action=robot_state_publisher,
            on_start=[rviz_node],
        )
    )

    delay_move_group = RegisterEventHandler(
        OnProcessStart(
            target_action=robot_state_publisher,
            on_start=[move_group_node],
        )
    )

    # Add all actions to launch description
    ld.add_action(x_arg)
    ld.add_action(y_arg)
    ld.add_action(z_arg)
    ld.add_action(R_arg)
    ld.add_action(P_arg)
    ld.add_action(Y_arg)
    ld.add_action(use_arm_arg)
    ld.add_action(use_base_arg)
    ld.add_action(use_gazebo_arg)
    ld.add_action(use_rviz_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(arm_tf_prefix_arg)
    ld.add_action(mobile_base_prefix_arg)
    ld.add_action(safety_lim_arg)
    ld.add_action(arm_type_arg)
    ld.add_action(arm_name_arg)
    ld.add_action(gazebo)
    ld.add_action(controller_manager_node)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_the_robot)
    ld.add_action(delay_joint_state_broadcaster)
    ld.add_action(delay_arm_controller)
    ld.add_action(delay_gripper_controller)
    ld.add_action(delay_base_controller)
    ld.add_action(delay_rviz_node)
    ld.add_action(delay_move_group)

    return ld
