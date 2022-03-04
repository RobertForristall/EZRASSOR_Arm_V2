from setuptools import setup

package_name = 'ezrassor_arm_v2'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/simulation_launch.py', 'launch/move_group.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/base.world']))
data_files.append(('share/' + package_name + '/resource', ['resource/arm_model.urdf','resource/ezrassor.gazebo', 'resource/ezrassor.xacro', 'resource/macros.xacro', 'resource/materials.xacro', 'resource/model.sdf', 'resource/tutorial_model.sdf', 'resource/ezrassor_v2.xacro', 'resource/ezrassor_arm.gazebo', 'resource/ezrassor_arm_v2.xacro']))
data_files.append(('share/' + package_name + '/meshes', ['meshes/base_station.dae', 'meshes/base_unit.dae', 'meshes/drum_arm.dae', 'meshes/drum.dae', 'meshes/wheel.dae', 'meshes/arm_camera.dae', 'meshes/grabber1.dae', 'meshes/grabber2.dae', 'meshes/link1.dae', 'meshes/link2.dae', 'meshes/link3.dae', 'meshes/link4.dae', 'meshes/link5.dae', 'meshes/link6.dae', 'meshes/link6.1.dae', 'meshes/platform.dae', 'meshes/rover_arm_back.dae', 'meshes/processor_housing.dae']))
data_files.append(('share/' + package_name + '/config/paver_arm_rover', ['config/paver_arm_rover/default_position_controllers.yaml']))
data_files.append(('share/' + package_name + '/config/standard_rover', ['config/standard_rover/default_position_controllers.yaml']))
data_files.append(('share/' + package_name + '/config', ['config/ezrassor.srdf', 'config/joint_limits.yaml', 'config/kinematics.yaml', 'config/ompl_planning.yaml', 'config/ros_controllers.yaml', 'config/controllers.yaml', 'config/test_controllers.yaml']))
#data_files.append(('share/' + package_name + '/source/arms_driver', ['source/arms_driver/__init__.py', 'source/arms_driver/__main__.py']))
#data_files.append(('share/' + package_name + '/source/drums_driver', ['source/drums_driver/__init__.py', 'source/drums_driver/__main__.py']))
#data_files.append(('share/' + package_name + '/source/wheels_driver', ['source/wheels_driver/__init__.py', 'source/wheels_driver/__main__.py']))
#data_files.append(('share/' + package_name + '/source/paver_arm_driver', ['source/paver_arm_driver/__init__.py', 'source/paver_arm_driver/__main__.py']))
data_files.append(('share/' + package_name, ['package.xml']))
 
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robert',
    maintainer_email='robert.forristall@knights.ucf.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spawn_rover = ezrassor_arm_v2.spawn_rover:main',
            "arms_driver = ezrassor_arm_v2.arms_driver:main",
            "wheels_driver = ezrassor_arm_v2.wheels_driver:main",
            "drums_driver = ezrassor_arm_v2.drums_driver:main",
            "paver_arm_driver = ezrassor_arm_v2.paver_arm_driver:main",
            "move_group_interface = ezrassor_arm_v2.move_group_interface:main",
            'obstacle_detection = ezrassor_arm_v2.obstacle_detection:main',
            'autonomous_controls = ezrassor_arm_v2.autonomous_controls:main'
        ],
    },
)
