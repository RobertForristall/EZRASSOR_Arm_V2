import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import xacro

def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    pkg_ezrassor_arm_v2 = get_package_share_directory('ezrassor_arm_v2')
    model_uri = os.path.join(pkg_ezrassor_arm_v2, 'resource/ezrassor.xacro')

    ezrassor_description_config = xacro.process_file(model_uri)
    robot_desc = ezrassor_description_config.toxml()


    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        )
    )

    model = Node(
        package='ezrassor_arm_v2',
        executable ='spawn_rover',
        arguments=[robot_desc],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=[os.path.join(pkg_ezrassor_arm_v2, 'worlds', 'base.world'), ''],
            description='SDF world file'
        ),
        gazebo,
        model
    ])