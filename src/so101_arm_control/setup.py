from glob import glob
import os

from setuptools import find_packages, setup


package_name = "so101_arm_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, ["README.md"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.xacro")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="matsunaga-h",
    maintainer_email="matsunaga-h@example.com",
    description="Safety-gated SO-ARM101 ROS 2 control gateway.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "arm_gateway = so101_arm_control.arm_gateway:main",
            "mock_trajectory_server = so101_arm_control.mock_trajectory_server:main",
            "mock_smoke_test = so101_arm_control.mock_smoke_test:main",
            "dashboard_ws_smoke_test = so101_arm_control.dashboard_ws_smoke_test:main",
            "read_only_probe = so101_arm_control.read_only_probe:main",
            "real_read_only_joint_state_publisher = so101_arm_control.real_read_only_joint_state_publisher:main",
            "supervised_micro_move = so101_arm_control.supervised_micro_move:main",
            "real_trajectory_server = so101_arm_control.real_trajectory_server:main",
            "real_dashboard_sequence_test = so101_arm_control.real_dashboard_sequence_test:main",
            "arm_mount_tf_publisher = so101_arm_control.arm_mount_tf_publisher:main",
            "arm_visual_joint_state_calibrator = so101_arm_control.arm_visual_joint_state_calibrator:main",
            "leader_state_publisher = so101_arm_control.leader_state_publisher:main",
        ],
    },
)
