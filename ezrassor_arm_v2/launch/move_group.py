# Node for starting the moveit group for the paver arm

from ntpath import join
import os
from webbrowser import get
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
import math

def construct_angle_radians(loader, node):

    value = loader.construct_scaler(node)
    try:
        return float(value)
    except SyntaxError:
        raise Exception('invalid expression: f{value}')

def construct_angle_degrees(loader, node):
    return math.radians(construct_angle_radians(loader, node))

def load_yaml(pkg_path, file):
    file_uri = os.path.join(pkg_path, 'config', file)

    try:
        yaml.SafeLoader.add_constructor('!radians', construct_angle_radians)
        yaml.SafeLoader.add_constructor('!degrees', construct_angle_degrees)
    except Exception:
        raise Exception('yaml support unavialable; install python-yaml')
    
    try:
        with open(file_uri) as file:
            return yaml.safe_load(file)
    except OSError:
        return None


def generate_launch_description():

    pkg_ezrassor_arm_v2 = get_package_share_directory('ezrassor_arm_v2')

    model_uri_arm = os.path.join(pkg_ezrassor_arm_v2, 'resource/arm_model.urdf')
    ezrassor_arm_description_config = xacro.process_file(model_uri_arm)
    arm_content = ezrassor_arm_description_config.toxml()

    semantic_arm_uri = os.path.join(pkg_ezrassor_arm_v2, 'config/ezrassor.srdf' )
    semantic_ezrassor_arm_description_config = xacro.process_file(semantic_arm_uri)
    semantic_arm_content = semantic_ezrassor_arm_description_config.toxml()

    robot_description = {'robot_description': arm_content}
    robot_description_semantic = {"robot_description_semantic": semantic_arm_content}

    kinematics_yaml = load_yaml(pkg_ezrassor_arm_v2, 'kinematics.yaml')    
    robot_description_kinematics = {'robot_description_kinematics': kinematics_yaml}

    joint_limit_yaml = load_yaml(pkg_ezrassor_arm_v2, 'joint_limits.yaml')
    robot_description_planning = {'robot_description_planning': joint_limit_yaml}

    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }

    ompl_yaml = load_yaml(pkg_ezrassor_arm_v2, 'ompl_planning.yaml')
    ompl_planning_pipeline_config['move_group'].update(ompl_yaml)

    controllers_yaml = load_yaml(pkg_ezrassor_arm_v2, 'ros_controllers.yaml')
    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

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
        ]
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'debug',
            default_value=["false"],
            description='GDB Debug Option'
        ),
        DeclareLaunchArgument(
            'info',
            default_value=['false'],
            description='Verbose Mode Option'
        ),
        DeclareLaunchArgument(
            'pipeline',
            default_value=['ompl'],
            description='The moveit pipeline to launch'
        ),
        DeclareLaunchArgument(
            'allow_trajectory_execution',
            default_value=['true'],
            description='Allow for the moveit controller to execute trajectories'
        ),
        DeclareLaunchArgument(
            'fake_execution',
            default_value=['false'],
            description=''
        ),
        DeclareLaunchArgument(
            'execution_type',
            default_value=['interpolate'],
            description=''
        ),
        DeclareLaunchArgument(
            'max_safe_path_cost',
            default_value=['1'],
            description=''
        ),
        DeclareLaunchArgument(
            'jiggle_fraction',
            default_value=['0.05'],
            description=''
        ),
        DeclareLaunchArgument(
            'publish_monitored_planning_scene',
            default_value=['true'],
            description=''
        ),
        DeclareLaunchArgument(
            'capabilities',
            default_value=[''],
            description=''
        ),
        DeclareLaunchArgument(
            'disabled_capabilities',
            default_value=[''],
            description=''
        ),
        DeclareLaunchArgument(
            'load_robot_description',
            default_value=['true'],
            description=''
        ),
        move_group

    ])