import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    pkg_ezrassor_arm_v2 = get_package_share_directory('ezrassor_arm_v2')
    model_uri = os.path.join(pkg_ezrassor_arm_v2, 'resource/ezrassor.xacro')


    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        )
    )

    model = Node(
        package='gazebo_ros',
        executable ='spawn_entity.py',
        arguments=['-entity', 'model_name', '-file', model_uri],
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