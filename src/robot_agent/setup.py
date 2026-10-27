from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'robot_agent'


def _web_dist_data_files():
    # ament_python's data_files needs one (dest_dir, [files]) tuple per source
    # directory -- it does not recurse. web/dist is a Vite build (index.html +
    # assets/*), built separately via tools/build_dashboard_web.sh.
    dist_root = os.path.join('web', 'dist')
    entries = []
    for dirpath, _dirnames, filenames in os.walk(dist_root):
        if not filenames:
            continue
        rel_dir = os.path.relpath(dirpath, dist_root)
        dest = os.path.join('share', package_name, 'web') if rel_dir == '.' else os.path.join(
            'share', package_name, 'web', rel_dir
        )
        # Symlink builds retain old hashed filenames after Vite replaces dist.
        files = [os.path.join(dirpath, name) for name in filenames
                 if os.path.isfile(os.path.join(dirpath, name))]
        if files:
            entries.append((dest, files))
    return entries


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'recovery'), glob('config/recovery/*.yaml')),
        (os.path.join('share', package_name, 'config', 'recovery', 'envs'), glob('config/recovery/envs/*.yaml')),
        (
            os.path.join('share', package_name, 'config', 'recovery', 'playbooks'),
            glob('config/recovery/playbooks/*.yaml'),
        ),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'foxglove'), glob('foxglove/*.json')),
        *_web_dist_data_files(),
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
            'real_robot_monitor = robot_agent.nodes.real_robot_monitor:main',
            'health_monitor = robot_agent.nodes.health_monitor:main',
            'recovery_agent = robot_agent.nodes.recovery_agent:main',
            'agent_dry_run = robot_agent.nodes.agent_dry_run:main',
            'parser_dry_run_matrix = robot_agent.tools.parser_dry_run_matrix:main',
        ],
    },
)
