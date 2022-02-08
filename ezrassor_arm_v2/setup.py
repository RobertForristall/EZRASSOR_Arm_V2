from setuptools import setup

package_name = 'ezrassor_arm_v2'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/arm_launch.py', 'launch/test_arm_launch.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/base.world']))
data_files.append(('share/' + package_name + '/resource', ['resource/ezrassor.gazebo', 'resource/ezrassor.xacro', 'resource/macros.xacro', 'resource/materials.xacro', 'resource/model.sdf', 'resource/tutorial_model.sdf', 'resource/ezrassor_v2.xacro']))
data_files.append(('share/' + package_name + '/meshes', ['meshes/base_station.dae', 'meshes/base_unit.dae', 'meshes/drum_arm.dae', 'meshes/drum.dae', 'meshes/wheel.dae']))
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
