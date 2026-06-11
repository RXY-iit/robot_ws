
```bash
unset LD_LIBRARY_PATH
unset PYTHONPATH
unset AMENT_PREFIX_PATH
unset COLCON_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset RMW_IMPLEMENTATION
unset CYCLONEDDS_URI

source /opt/ros/humble/setup.bash
source /home/ruan-x/robot_ws/install/setup.bash
export ROS_DOMAIN_ID=20
export ROS_DISTRO=humble

ros2 topic list

```