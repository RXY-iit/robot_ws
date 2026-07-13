from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'robot_agent'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ruan-x',
    maintainer_email='ruan-x@example.com',
    description='Mission agent layer for the Robot Harness System.',
    license='TODO',
    entry_points={
        'console_scripts': [
            'agent_node = robot_agent.nodes.agent_node:main',
            'dashboard_node = robot_agent.nodes.dashboard_node:main',
            'agent_dry_run = robot_agent.nodes.agent_dry_run:main',
            'parser_dry_run_matrix = robot_agent.tools.parser_dry_run_matrix:main',
        ],
    },
)
