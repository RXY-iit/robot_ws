from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'robot_skills'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ruan-x',
    maintainer_email='ruan-x@example.com',
    description='Typed reusable robot Skills for the Robot Harness System.',
    license='TODO',
    entry_points={
        'console_scripts': [
            'semantic_map_marker_node = robot_skills.nodes.semantic_map_marker_node:main',
            'segmentation_tool = robot_skills.nodes.segmentation_tool:main',
            'navigation_annotation_tool = robot_skills.nodes.navigation_annotation_tool:main',
            'perception_dry_run_matrix = robot_skills.tools.perception_dry_run_matrix:main',
        ],
    },
)
