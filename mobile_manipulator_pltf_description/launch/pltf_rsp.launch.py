from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")

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
            "prefix:=''",
            " ",
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("hunter_with_arm_description"), "rviz", "robot_view.rviz"]
    )

    # Create a robot_state_publisher node
    params = {
        "robot_description": robot_description_content,
        "use_sim_time": use_sim_time,
    }
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[params],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(gui),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    # Launch!
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use sim time if true",
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Start Rviz2 and Joint State Publisher gui automatically \
        with this launch file.",
            ),
            node_robot_state_publisher,
            joint_state_publisher_node,
            rviz_node,
        ]
    )
