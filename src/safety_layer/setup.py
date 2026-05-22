from setuptools import find_packages, setup

package_name = 'safety_layer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='matsunaga-h',
    maintainer_email='hide.matsuhide0312@gmail.com',
    description='Safety layer: cmd_vel watchdog, speed limits, emergency stop.',
    license='TODO',
    entry_points={
        'console_scripts': [
            'cmd_vel_safety_node = safety_layer.cmd_vel_safety_node:main',
        ],
    },
)
