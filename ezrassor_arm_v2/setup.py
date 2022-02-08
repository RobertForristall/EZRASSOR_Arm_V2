from setuptools import setup

package_name = 'ezrassor_arm_v2'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/arm_launch.py', 'launch/test_arm_launch.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/base.world']))
data_files.append(('share/' + package_name + '/resource', ['resource/ezrassor.gazebo', 'resource/ezrassor.xacro', 'resource/macros.xacro', 'resource/materials.xacro', 'resource/model.sdf', 'resource/tutorial_model.sdf', 'resource/ezrassor_v2.xacro', 'resource/ezrassor_arm.gazebo', 'resource/ezrassor_arm_v2.xacro']))
data_files.append(('share/' + package_name + '/meshes', ['meshes/base_station.dae', 'meshes/base_unit.dae', 'meshes/drum_arm.dae', 'meshes/drum.dae', 'meshes/wheel.dae', 'meshes/arm_camera.dae', 'meshes/grabber1.dae', 'meshes/grabber2.dae', 'meshes/link1.dae', 'meshes/link2.dae', 'meshes/link3.dae', 'meshes/link4.dae', 'meshes/link5.dae', 'meshes/link6.dae', 'meshes/link6.1.dae', 'meshes/platform.dae', 'meshes/rover_arm_back.dae', 'meshes/processor_housing.dae']))
data_files.append(('share/' + package_name + '/config/paver_arm_rover', ['config/paver_arm_rover/default_position_controllers.yaml']))
data_files.append(('share/' + package_name + '/config/standard_rover', ['config/standard_rover/default_position_controllers.yaml']))
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
            'arm_driver = ezrassor_arm_v2.arm_driver:main',
            'spawn_rover = ezrassor_arm_v2.spawn_rover:main'
        ],
    },
)
