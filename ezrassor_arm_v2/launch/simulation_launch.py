# Node for starting the ezrassor_arm_v2 package through a gazebo simulation

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import (IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler)
from launch.substitutions import (LaunchConfiguration)
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit
from launch.conditions import (LaunchConfigurationEquals, LaunchConfigurationNotEquals)
import xacro
import yaml

# Functions for handling/loading yaml files for moveit configuration
def load_yaml(pkg_path, file):
    file_uri = os.path.join(pkg_path, 'config', file)
    
    try:
        with open(file_uri) as file:
            return yaml.safe_load(file)
    except OSError:
        return None

#To be finished -- Testing function for multiple rovers controllers ---
def generate_controllers(pkg_ezrassor_arm_v2, robot_description, robot_description_semantic, robot_description_kinematics, robot_description_planning, 
ompl_planning_pipeline_config, trajectory_execution, moveit_controllers, planning_scene_monitor_parameters, rover_count):
    move_group_nodes = []
    for i in range(rover_count):
        # Controllers configurations for moveit
        controllers_yaml = load_yaml(pkg_ezrassor_arm_v2, 'controllers.yaml')
        new_controller_name = '/ezrassor' + str(i) + '/' + 'joint_trajectory_controller'
        controllers_yaml['controller_names'] = [new_controller_name]
        controllers_yaml[new_controller_name] = controllers_yaml.pop('joint_trajectory_controller')
        moveit_controllers = {
            "moveit_simple_controller_manager": controllers_yaml,
            "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        }

def generate_launch_description():

    # Package path for gazebo_ros to spawn the simulation enviornment
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    # Package path for the ezrassor_arm_v2 package
    pkg_ezrassor_arm_v2 = get_package_share_directory('ezrassor_arm_v2')
    
    # Model content for the paver arm rover model to be used with the robot state publisher
    model_uri_arm = os.path.join(pkg_ezrassor_arm_v2, 'resource/ezrassor_arm_v2.xacro')
    ezrassor_arm_description_config = xacro.process_file(model_uri_arm)
    arm_content = ezrassor_arm_description_config.toxml()

    # Model content for the standard rover model to be used with the robot state publisher
    model_uri_standard = os.path.join(pkg_ezrassor_arm_v2, 'resource/ezrassor.xacro')
    ezrassor_standard_description_config = xacro.process_file(model_uri_standard)
    standard_content = ezrassor_standard_description_config.toxml()

    # Model content for the paver arm rover model to be used with moveit
    moveit_model_uri_arm = os.path.join(pkg_ezrassor_arm_v2, 'resource/arm_model.urdf')
    moveit_ezrassor_arm_description_config = xacro.process_file(moveit_model_uri_arm)
    moveit_arm_content = moveit_ezrassor_arm_description_config.toxml()
    
    # Semantic model content for the paver arm rover model to be used with moveit
    semantic_arm_uri = os.path.join(pkg_ezrassor_arm_v2, 'config/ezrassor.srdf' )
    semantic_ezrassor_arm_description_config = xacro.process_file(semantic_arm_uri)
    semantic_arm_content = semantic_ezrassor_arm_description_config.toxml()

    # Setting up robot descriptions for moveit model and semantic moveit model
    robot_description = {'robot_description': moveit_arm_content}
    robot_description_semantic = {"robot_description_semantic": semantic_arm_content}

    # Kinematics configuration for moveit
    kinematics_yaml = load_yaml(pkg_ezrassor_arm_v2, 'kinematics.yaml')    
    robot_description_kinematics = {'robot_description_kinematics': kinematics_yaml}
    
    # Joint limit configurations for moveit
    joint_limit_yaml = load_yaml(pkg_ezrassor_arm_v2, 'joint_limits.yaml')
    robot_description_planning = {'robot_description_planning': joint_limit_yaml}
    
    # Planning scene configurations for moveit
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_yaml = load_yaml(pkg_ezrassor_arm_v2, 'ompl_planning.yaml')
    ompl_planning_pipeline_config['move_group'].update(ompl_yaml)
    
    # Controllers configurations for moveit
    controllers_yaml = load_yaml(pkg_ezrassor_arm_v2, 'controllers.yaml')
    new_controller_name = '/ezrassor/' + 'joint_trajectory_controller'
    controllers_yaml['controller_names'] = [new_controller_name]
    controllers_yaml[new_controller_name] = controllers_yaml.pop('joint_trajectory_controller')
    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }
    
    # Trajectory execution configuration for moveit
    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    # Planning scene monitoring configuration for moveit
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "planning_scene_monitor_options": {
            "name": "planning_scene_monitor",
            "robot_description": "robot_description",
            "joint_state_topic": "ezrassor/joint_states",
            "attached_collision_object_topic": "/move_group/planning_scene_monitor",
            "publish_planning_scene_topic": "/move_group/publish_planning_scene",
            "monitored_planning_scene_topic": "/move_group/monitored_planning_scene",
            "wait_for_initial_state_timeout": 10.0,
        },
    }
    
    # Launch the moveit node
    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters
        ],
        condition=LaunchConfigurationEquals('rover_model', 'arm')
    )

    # Launch move group interface node from the ezrassor_arm_v2 package
    move_group_interface = Node(
        package='ezrassor_arm_v2',
        executable='move_group_interface',
        output='screen',
        condition=LaunchConfigurationEquals('rover_model', 'arm')
    )

    # Launch the static transform publisher
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "base_link", "link1"],
        condition=LaunchConfigurationEquals('rover_model', 'arm')
    )

    # Include the gazebo sim launch file, to be called during launch to launch the simulation
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )

    # Arguments for the spawn_entity node in the ezrassor_arm_v2 package
    spawn_entity_args = [
        LaunchConfiguration('rover_model'), 
        LaunchConfiguration('robot_count'), 
        LaunchConfiguration('spawn_x_coords'), 
        LaunchConfiguration('spawn_y_coords'), 
        LaunchConfiguration('spawn_z_coords'), 
        LaunchConfiguration('spawn_roll'), 
        LaunchConfiguration('spawn_pitch'), 
        LaunchConfiguration('spawn_yaw')
    ]

    # Launch the spawn entity node 
    spawn_entity = Node(
        package='ezrassor_arm_v2',
        executable ='spawn_rover',
        arguments=spawn_entity_args,
        output='screen'
    )

    # Launch the robot state publisher for the paver arm rover model
    # Only launches if the rover_model launch argument is set to 'arm'
    robot_state_publisher_arm = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace='ezrassor',
            output="screen",
            parameters=[{"robot_description": arm_content}],
            condition = LaunchConfigurationEquals('rover_model', 'arm')
        )

    # Launch the robot state publisher for the standard rover model
    # Only launches if the rover_model launch argument is set to 'standard' (which is the default setting)
    robot_state_publisher_standard = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace='ezrassor',
            output="screen",
            parameters=[{"robot_description": standard_content}],
            condition = LaunchConfigurationNotEquals('rover_model', 'arm')
        )

    # Launch the keyboard_controls node of the ezrassor_arm_v2 package
    keyboard_controls = Node(
        package='ezrassor_arm_v2',
        executable='keyboard_controls',
        arguments=[LaunchConfiguration('rover_model')],
        output='screen',
        condition = LaunchConfigurationEquals('control_methods', 'keyboard')
    )

    # Arguments for the autonomous_controls node in the ezrassor_arm_v2 package
    autonomous_args = [
        LaunchConfiguration('rover_model'),
        LaunchConfiguration('target_x_coords'),
        LaunchConfiguration('target_y_coords'),
        LaunchConfiguration('spawn_x_coords'),
        LaunchConfiguration('spawn_y_coords'),
        LaunchConfiguration('enable_real_odometry'),
        LaunchConfiguration('enable_park_ranger'),
        LaunchConfiguration('enable_swarm_control'),
        LaunchConfiguration('world'),   
    ]

    # Launch the autonomous_controls node of the ezrassor_arm_v2 package
    autonomous_controls = Node(
        package='ezrassor_arm_v2',
        executable='autonomous_controls',
        arguments=autonomous_args,
        output='screen',
        condition = LaunchConfigurationEquals('control_methods', 'autonomous')
    )

    # Launch the gamepad_controls node of the ezrassor_arm_v2 package
    gamepad_controls = Node(
        package='ezrassor_arm_v2',
        executable='gamepad_controls',
        arguments=[LaunchConfiguration('rover_model'), LaunchConfiguration('port')],
        output='screen',
        condition = LaunchConfigurationEquals('control_methods', 'gamepad')
    )

    # Array of launch commands with arguments/conditions to load the drivers needed for the desired rover model
    drivers = [
            Node(
                package='ezrassor_arm_v2',
                executable='arms_driver',
                arguments=[LaunchConfiguration('rover_model')],
                output='screen'
            ),
            Node(
                package='ezrassor_arm_v2',
                executable='drums_driver',
                arguments=[LaunchConfiguration('rover_model')],
                output='screen'
            ),
            Node(
                package='ezrassor_arm_v2',
                executable='wheels_driver',
                output='screen'
            ),
            Node(
                package='ezrassor_arm_v2',
                executable='paver_arm_driver',
                output='screen',
                condition = LaunchConfigurationEquals('rover_model', 'arm')
            )     
        ]

    # Launch the joint_state_controller
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',"-c", "/ezrassor/controller_manager", '--set-state', 'start', 'joint_state_broadcaster'],
        output='screen'
    )

    # Launch the paver_arm_trajectory_controller from the default positions controller configuration file
    load_paver_arm_trajectory_controller = ExecuteProcess(
        condition = LaunchConfigurationEquals('rover_model', 'arm'),
        cmd=['ros2', 'control', 'load_controller',"-c", "/ezrassor/controller_manager", '--set-state', 'start', 'joint_trajectory_controller'],
        output='screen'
    )

    # Launch the gripper_effort_controller from the default positions controller configuration file
    load_gripper_effort_controller = ExecuteProcess(
        condition = LaunchConfigurationEquals('rover_model', 'arm'),
        cmd=['ros2', 'control', 'load_controller',"-c", "/ezrassor/controller_manager", '--set-state', 'start', 'gripper_effort_controller'],
        output='screen'
    )

    # Launch the diff_drive_controller from the default positions controller configuration file
    load_diff_drive_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',"-c", "/ezrassor/controller_manager", '--set-state', 'start', 'diff_drive_controller'],
        output='screen'
    )

    # Launch the arm_back_velocity_controller from the default positions controller configuration file
    load_arm_back_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',"-c", "/ezrassor/controller_manager", '--set-state', 'start', 'arm_back_velocity_controller'],
        output='screen'
    )
    # Launch the drum_back_velocity_controller from the default positions controller configuration file
    load_drum_back_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',"-c", "/ezrassor/controller_manager", '--set-state', 'start', 'drum_back_velocity_controller'],
        output='screen'
    )

    # Launch the arm_front_velocity_controller from the default positions controller configuration file
    load_arm_front_velocity_controller = ExecuteProcess(
        condition = LaunchConfigurationEquals('rover_model', 'standard'),
        cmd=['ros2', 'control', 'load_controller',"-c", "/ezrassor/controller_manager", '--set-state', 'start', 'arm_front_velocity_controller'],
        output='screen'
    )
    # Launch the drum_front_velocity_controller from the default positions controller configuration file
    load_drum_front_velocity_controller = ExecuteProcess(
        condition = LaunchConfigurationEquals('rover_model', 'standard'),
        cmd=['ros2', 'control', 'load_controller',"-c", "/ezrassor/controller_manager", '--set-state', 'start', 'drum_front_velocity_controller'],
        output='screen'
    )

    return LaunchDescription([

        # All startup launch argumnets for managing models/controls/simulation
        DeclareLaunchArgument(
            'world',
            default_value=[os.path.join(pkg_ezrassor_arm_v2, 'worlds', 'base.world')],
            description='SDF world file'
        ),
        DeclareLaunchArgument(
            'rover_model',
            default_value=['standard', ''],
            description='Xacro rover model file'
        ),
        DeclareLaunchArgument(
            'robot_count',
            default_value=['1', ''],
            description='The number of rovers to spawn'
        ),
        DeclareLaunchArgument(
            'ports',
            default_value=['default', ''],
            description='Ports for gamepad connection'
        ),
        DeclareLaunchArgument(
            'joysticks',
            default_value=['default', ''],
            description='Joysticks for operating rover'
        ),
        DeclareLaunchArgument(
            'spawn_x_coords',
            default_value=['0', ''],
            description='The x coordinate to spawn the rover in the simulation'
        ),
        DeclareLaunchArgument(
            'spawn_y_coords',
            default_value=['0', ''],
            description='The y coordinate to spawn the rover in the simulation'
        ),
        DeclareLaunchArgument(
            'spawn_z_coords',
            default_value=['0.5', ''],
            description='The z coordinate to spawn the rover in the simulation'
        ),
        DeclareLaunchArgument(
            'spawn_roll',
            default_value=['0', ''],
            description='The roll value to spawn the rover in the simulation'
        ),
        DeclareLaunchArgument(
            'spawn_pitch',
            default_value=['0', ''],
            description='The pitch value to spawn the rover in the simualtion'
        ),
        DeclareLaunchArgument(
            'spawn_yaw',
            default_value=['0', ''],
            description='The yaw value to spawn the rover in the simulation'
        ),
        DeclareLaunchArgument(
            'target_x_coords',
            default_value=['default', ''],
            description='The x coordinate of the target location'
        ),
        DeclareLaunchArgument(
            'target_y_coords',
            default_value=['default', ''],
            description='the y coordinate of the target location'
        ),
        DeclareLaunchArgument(
            'control_methods',
            default_value=['keyboard', ''],
            description='The control methods for operating the rover'
        ),
        DeclareLaunchArgument(
            'debug',
            default_value=['false', ''],
            description='Enable the debug systems in the gazebo simulation'
        ),
        DeclareLaunchArgument(
            'paused',
            default_value=['false', ''],
            description='Start the gazebo simulation in its paused state'
        ),
        DeclareLaunchArgument(
            'verbose',
            default_value=['false', ''],
            description='Enable the verbose mode of the gazebo'
        ),
        DeclareLaunchArgument(
            'show_gui',
            default_value=['true', ''],
            description='Show the gazebo gui'
        ),
        DeclareLaunchArgument(
            'recording',
            default_value=['false', ''],
            description='Enable recording of simulation during startup'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=['false', ''],
            description='Run the simulation on simulated time'
        ),
        DeclareLaunchArgument(
            'enable_real_odometry',
            default_value=['false', ''],
            description='Enable the real odometry package for the rover'
        ),
        DeclareLaunchArgument(
            'enable_park_ranger',
            default_value=['false', ''],
            description='Enable the park ranger package for the rover'
        ),
        DeclareLaunchArgument(
            'enable_swarm_control',
            default_value=['false', ''],
            description='Enable swarm controls package for the rover'
        ),

        # Spawn defined nodes above to process in the neccesary order
        gazebo,
        spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[robot_state_publisher_arm,robot_state_publisher_standard],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=drivers
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[
                    load_diff_drive_controller,
                    load_arm_back_velocity_controller,
                    load_arm_front_velocity_controller,
                    load_drum_back_velocity_controller,
                    load_drum_front_velocity_controller,
                    load_gripper_effort_controller,
                    load_paver_arm_trajectory_controller,
                ],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_paver_arm_trajectory_controller,
                on_exit=[move_group, static_tf, move_group_interface]
            )
        ), 
    ])