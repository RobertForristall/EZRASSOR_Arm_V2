from ast import arguments
from http.server import executable
from multiprocessing import Condition
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import (IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler)
from launch.substitutions import (LaunchConfiguration, PythonExpression)
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit
from launch.conditions import (LaunchConfigurationEquals, LaunchConfigurationNotEquals, IfCondition, UnlessCondition)
import xacro

def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    pkg_ezrassor_arm_v2 = get_package_share_directory('ezrassor_arm_v2')
    
    
    model_uri_arm = os.path.join(pkg_ezrassor_arm_v2, 'resource/ezrassor_arm_v2.xacro')
    ezrassor_arm_description_config = xacro.process_file(model_uri_arm)
    arm_content = ezrassor_arm_description_config.toxml()

    model_uri_standard = os.path.join(pkg_ezrassor_arm_v2, 'resource/ezrassor.xacro')
    ezrassor_standard_description_config = xacro.process_file(model_uri_standard)
    standard_content = ezrassor_standard_description_config.toxml()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )

    spawn_entity = Node(
        package='ezrassor_arm_v2',
        executable ='spawn_rover',
        arguments=[LaunchConfiguration('rover_model'), LaunchConfiguration('robot_count'), LaunchConfiguration('spawn_x_coords'), LaunchConfiguration('spawn_y_coords'), LaunchConfiguration('spawn_z_coords'), LaunchConfiguration('spawn_roll'), LaunchConfiguration('spawn_pitch'), LaunchConfiguration('spawn_yaw')],
        output='screen'
    )

    robot_state_publisher_arm = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace='ezrassor',
            output="screen",
            parameters=[{"robot_description": arm_content}],
            condition = LaunchConfigurationEquals('rover_model', 'arm')
        )

    robot_state_publisher_standard = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace='ezrassor',
            output="screen",
            parameters=[{"robot_description": standard_content}],
            condition = LaunchConfigurationNotEquals('rover_model', 'arm')
        )

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


    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',"-c", "/ezrassor/controller_manager", '--set-state', 'start', 'joint_state_broadcaster'],
        output='screen'
    )

    load_paver_arm_trajectory_controller = ExecuteProcess(
        condition = LaunchConfigurationEquals('rover_model', 'arm'),
        cmd=['ros2', 'control', 'load_controller',"-c", "/ezrassor/controller_manager", '--set-state', 'start', 'paver_arm_trajectory_controller'],
        output='screen'
    )

    load_gripper_effort_controller = ExecuteProcess(
        condition = LaunchConfigurationEquals('rover_model', 'arm'),
        cmd=['ros2', 'control', 'load_controller',"-c", "/ezrassor/controller_manager", '--set-state', 'start', 'gripper_effort_controller'],
        output='screen'
    )

    load_diff_drive_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',"-c", "/ezrassor/controller_manager", '--set-state', 'start', 'diff_drive_controller'],
        output='screen'
    )

    load_arm_back_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',"-c", "/ezrassor/controller_manager", '--set-state', 'start', 'arm_back_velocity_controller'],
        output='screen'
    )

    load_drum_back_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',"-c", "/ezrassor/controller_manager", '--set-state', 'start', 'drum_back_velocity_controller'],
        output='screen'
    )

    return LaunchDescription([
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
            default_value=['default', ''],
            description='Enable the real odometry package for the rover'
        ),
        DeclareLaunchArgument(
            'enable_park_ranger',
            default_value=['default', ''],
            description='Enable the park ranger package for the rover'
        ),
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
                on_exit=[load_diff_drive_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_arm_back_velocity_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_drum_back_velocity_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_paver_arm_trajectory_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_gripper_effort_controller],
            )
        ),
            
    ])