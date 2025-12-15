# Copyright 2020 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
 
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
 
def generate_launch_description():
    
    # Initialize Arguments
    gui = LaunchConfiguration("gui", default="true")
    kp_v = LaunchConfiguration('kp_v', default='40.0')
    kd_v = LaunchConfiguration('kd_v', default='0.1') 
    kp_w = LaunchConfiguration('kp_w', default='35.0')
    kd_w = LaunchConfiguration("kd_w", default="0.1")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware", default="true")
    is_sim = LaunchConfiguration('is_sim' , default='True')
    enable_pd_regulator = LaunchConfiguration('enable_pd_regulator', default='False')
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
  
    gui_declare = DeclareLaunchArgument(
            "gui", default_value=gui, description="Start RViz2 automatically with this launch file.")
   
    # IH: THIS PARAM IS NOT USED
    use_mock_hardware_declare = DeclareLaunchArgument(
            "use_mock_hardware", default_value=use_mock_hardware,description="Start robot with mock hardware mirroring command to its states.")
    
    use_sim_time_declare = DeclareLaunchArgument('use_sim_time', default_value=use_sim_time,
                                                                    description='Use simulation clock if true')
    kp_v_val_declare = DeclareLaunchArgument('kp_v', default_value=kp_v, description='Proportional gain for linear velocity')
    kd_v_val_declare = DeclareLaunchArgument('kd_v', default_value=kd_v, description='Derivative gain for linear velocity')
    kp_w_val_declare = DeclareLaunchArgument('kp_w', default_value=kp_w, description='Proportional gain for angular velocity')
    kd_w_val_declare = DeclareLaunchArgument('kd_w', default_value=kd_w, description='Derivative gain for angular velocity')
    enable_pd_regulator_declare = DeclareLaunchArgument('enable_pd_regulator', default_value=enable_pd_regulator
        , description='Use PD regulator estimate residual control to the robot')
    
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("mobile_manipulator_pltf_description"), "description" ,"mobile_manipulator_pltf.urdf.xacro"]
            ),
            " ",
            "is_sim:=",
             is_sim,
             " ",
            "prefix:=''",
            " ",
        ]
    )
    
    robot_description = {"robot_description": robot_description_content}
 
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("hunter_base"),
            "config",
            "hardware_controllers.yaml",
        ]
    )

    base_launch = os.path.join(get_package_share_directory("hunter_base"), "launch", "hunter_base.launch.py")
 
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )
    
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    
    hunter_base_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base_launch),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'kp_v': kp_v,
                'kd_v': kd_v,
                'kp_w': kp_w,
                'kd_w': kd_w,
                'enable_pd_regulator': enable_pd_regulator
                }.items(),
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(gui_declare)
    ld.add_action(use_mock_hardware_declare)
    ld.add_action(kp_v_val_declare)
    ld.add_action(kd_v_val_declare)
    ld.add_action(kp_w_val_declare)
    ld.add_action(kd_w_val_declare)
    ld.add_action(enable_pd_regulator_declare)
    ld.add_action(use_sim_time_declare)
    
    ld.add_action(robot_state_pub_node)
    ld.add_action(hunter_base_node)
   
    return ld