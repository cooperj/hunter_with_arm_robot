import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    RegisterEventHandler,
    Shutdown,
    SetEnvironmentVariable
)
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import yaml

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():
    # Package directories
    mobile_manipulator_gazebo_pkg_dir = get_package_share_directory('mobile_manipulator_pltf_gazebo')

    # Launch configurations
    world_path = LaunchConfiguration(
        'world_path', default=os.path.join(mobile_manipulator_gazebo_pkg_dir, 'worlds', 'empty_world.world')
    )
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='20.0')
    y_pose = LaunchConfiguration('y_pose', default='-4.0')
    roll = LaunchConfiguration('roll', default='0.0')
    pitch = LaunchConfiguration('pitch', default='0.0')
    yaw = LaunchConfiguration('yaw', default='1.45')
    use_rviz = LaunchConfiguration('use_rviz', default='false')
    use_gazebo = LaunchConfiguration('use_gazebo', default='true')
    robot_ip = LaunchConfiguration('robot_ip')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    load_gripper = LaunchConfiguration('load_gripper')
    fake_sensor_commands = LaunchConfiguration('fake_sensor_commands')
    load_camera = LaunchConfiguration('load_camera')
    camera_id = LaunchConfiguration('serial')
    camera_model = LaunchConfiguration('camera_type')
    planner_ = LaunchConfiguration('planner')
    db = LaunchConfiguration('db')

    # Gazebo parameters
    gazebo_params_file = os.path.join(mobile_manipulator_gazebo_pkg_dir, 'config', 'gazebo_params.yaml')

    # Robot description for the combined mobile manipulator (Hunter + Franka)
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("mobile_manipulator_pltf_description"), "description", "mobile_manipulator_pltf.urdf.xacro"]
            ),
            " ",
            "is_sim:=", use_sim_time,
            " ",
            "prefix:=''",
            " ",
            "hand:=", load_gripper,
            " ",
            "camera:=", load_camera,
            " ",
            "camera_model:=", camera_model,
            " ",
            "robot_ip:=", robot_ip,
            " ",
            "use_fake_hardware:=", use_fake_hardware,
            " ",
            "fake_sensor_commands:=", fake_sensor_commands
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # Franka semantic description
    franka_semantic_xacro_file = os.path.join(
        get_package_share_directory('franka_moveit_config'), 'srdf', 'panda_arm_platform.srdf.xacro'
    )
    robot_description_semantic_config = Command(
        [FindExecutable(name='xacro'), ' ', franka_semantic_xacro_file, ' hand:=', load_gripper, ' camera:=', load_camera]
    )
    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_config
    }

    # Load kinematics and planning configurations
    kinematics_yaml = load_yaml('franka_moveit_config', 'config/kinematics.yaml')
    ompl_planning_yaml = load_yaml('franka_moveit_config', 'config/ompl_planning.yaml')
    combined_planning_pipelines_config = {
        'move_group': {
            'planning_plugin': planner_,
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
                                'default_planner_request_adapters/ResolveConstraintFrames '
                                'default_planner_request_adapters/FixWorkspaceBounds '
                                'default_planner_request_adapters/FixStartStateBounds '
                                'default_planner_request_adapters/FixStartStateCollision '
                                'default_planner_request_adapters/FixStartStatePathConstraints '
                                'default_planning_request_adapters/CheckStartStateBounds '
                                'default_planning_request_adapters/CheckStartStateCollision '
                                'default_planning_request_adapters/ValidateWorkspaceBounds',
            'start_state_max_bounds_error': 0.1,
            'default_planner_config': 'PTP',
        },
        'robot_description_planning': {
            'cartesian_limits': {
                'max_trans_vel': 0.2,
                'max_trans_acc': 1.0,
                'max_trans_dec': -1.0,
                'max_rot_vel': 0.5
            }
        }
    }
    combined_planning_pipelines_config['move_group'].update(ompl_planning_yaml)

    # Trajectory execution configuration
    moveit_simple_controllers_yaml = load_yaml('franka_moveit_config', 'config/panda_controllers.yaml')
    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_simple_controllers_yaml,
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }
    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }
    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # Robot state publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
        remappings=[('joint_states', 'franka/joint_states')]
    )

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={
            'world': world_path,
            'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file
        }.items(),
        condition=IfCondition(use_gazebo)
    )

    # Spawn the combined robot in Gazebo
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

    # Controllers for Hunter robot
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )
    load_tricycle_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'ackermann_like_controller'],
        output='screen'
    )

    # Controllers for Franka arm
    ros2_controllers_path = os.path.join(
        get_package_share_directory('franka_moveit_config'),
        'config',
        'panda_mock_ros_controllers.yaml' if use_fake_hardware else 'panda_ros_controllers.yaml'
    )
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, ros2_controllers_path],
        remappings=[('joint_states', 'franka/joint_states')],
        output={'stdout': 'screen', 'stderr': 'screen'},
        on_exit=Shutdown(),
    )
    load_controllers = []
    for controller in ['panda_arm_controller', 'joint_state_broadcaster']:
        load_controllers += [
            ExecuteProcess(
                cmd=['ros2 run controller_manager spawner {}'.format(controller)],
                shell=True,
                output='screen',
            )
        ]

    # MoveIt node for Franka arm
    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            combined_planning_pipelines_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"publish_robot_description_semantic": True},
        ],
    )

    # RViz configuration
    rviz_base = os.path.join(get_package_share_directory('franka_moveit_config'), 'rviz')
    rviz_full_config = os.path.join(rviz_base, 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_full_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            combined_planning_pipelines_config,
            kinematics_yaml,
            {'use_sim_time': use_sim_time}
        ],
        condition=IfCondition(use_rviz)
    )

    # Joint state publisher for Franka
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[
            {'source_list': ['franka/joint_states', 'panda_gripper/joint_states'], 'rate': 30}
        ],
    )

    # Gripper and camera launch files
    gripper_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
            [FindPackageShare('franka_gripper'), 'launch', 'gripper.launch.py'])]),
        launch_arguments={'robot_ip': robot_ip, 'use_fake_hardware': use_fake_hardware}.items(),
        condition=IfCondition(load_gripper)
    )
    camera_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
            [FindPackageShare('spinnaker_camera_driver'), 'launch', 'driver_node.launch.py'])]),
        launch_arguments={'camera_type': camera_model, 'serial': camera_id}.items(),
        condition=IfCondition(load_camera)
    )

    # Warehouse MongoDB server
    mongodb_server_node = Node(
        package='warehouse_ros_mongo',
        executable='mongo_wrapper_ros.py',
        parameters=[
            {'warehouse_port': 33829},
            {'warehouse_host': 'localhost'},
            {'warehouse_plugin': 'warehouse_ros_mongo::MongoDatabaseConnection'},
        ],
        output='screen',
        condition=IfCondition(db)
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
        DeclareLaunchArgument('use_rviz', default_value='false', description='Whether to start RViz'),
        DeclareLaunchArgument('use_gazebo', default_value='true', description='Whether to start Gazebo'),
        DeclareLaunchArgument('robot_ip', description='Hostname or IP address of the robot'),
        DeclareLaunchArgument('use_fake_hardware', default_value='false', description='Use fake hardware'),
        DeclareLaunchArgument('fake_sensor_commands', default_value='false', description='Fake sensor commands'),
        DeclareLaunchArgument('load_gripper', default_value='false', description='Use Franka Gripper as end-effector'),
        DeclareLaunchArgument('load_camera', default_value='true', description='Use Flir camera as end-effector'),
        DeclareLaunchArgument('serial', default_value="'22141921'", description='Camera id, serial number'),
        DeclareLaunchArgument('camera_type', default_value='blackfly_s', description='Camera model, e.g., blackfly_s'),
        DeclareLaunchArgument('planner', default_value='ompl_interface/OMPLPlanner', description='Planner for arm control'),
        DeclareLaunchArgument('db', default_value='False', description='Database flag'),
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
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
        ros2_control_node,
        run_move_group_node,
        rviz_node,
        joint_state_publisher,
        gripper_launch_file,
        camera_launch_file,
        mongodb_server_node,
        *load_controllers
    ])