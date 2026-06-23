from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'robot_sim'


def package_files(directory):
    paths = []
    for root, _, filenames in os.walk(directory):
        files = [os.path.join(root, filename) for filename in filenames]
        if files:
            paths.append((os.path.join('share', package_name, root), files))
    return paths

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'usd'), [
            path for path in glob('usd/*') if os.path.isfile(path)
        ]),
    ] + package_files('usd/assets') + package_files('usd/scenes'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ruan-x',
    maintainer_email='ruan-x@example.com',
    description='Isaac Sim integration package for the robot control stack.',
    license='TODO',
    entry_points={
        'console_scripts': [
            'sim_omni_drive_command_node = robot_sim.sim_omni_drive_command_node:main',
            'pointcloud2_to_livox_custom_msg_node = robot_sim.pointcloud2_to_livox_custom_msg_node:main',
            'sim_sensor_check_node = robot_sim.sim_sensor_check_node:main',
            'sim_livox_imu_node = robot_sim.sim_livox_imu_node:main',
            'sim_lift_command_node = robot_sim.sim_lift_command_node:main',
        ],
    },
)
