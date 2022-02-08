import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    pkg_ezrassor_arm_v2 = get_package_share_directory('ezrassor_arm_v2')



    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
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
            default_value=['0', ''],
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
        Node(
            package='ezrassor_arm_v2',
            executable ='spawn_rover',
            arguments=[LaunchConfiguration('rover_model'), LaunchConfiguration('robot_count'), LaunchConfiguration('spawn_x_coords'), LaunchConfiguration('spawn_y_coords'), LaunchConfiguration('spawn_z_coords'), LaunchConfiguration('spawn_roll'), LaunchConfiguration('spawn_pitch'), LaunchConfiguration('spawn_yaw')],
            output='screen'
        )
    ])