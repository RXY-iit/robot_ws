Motor Systems Summary (Excluding Soft Arm)
1. Drive Motors (3x: Right/Left/Back)
Hardware: BLV-R motors with Modbus communication
Key Files:

src/movement/om_modbus_master_V201/om_modbus_master/sample/BLV_R/drive_motor.py - Main control node drive_motor.py:1-40
src/movement/om_modbus_master_V201/om_modbus_master/sample/BLV_R/idshare1.py - ID Share mode example idshare1.py:1-88
src/movement/om_modbus_master_V201/om_modbus_master/sample/BLV_R/ros2_motor_v2.py - Alternative implementation ros2_motor_v2.py:497-527
Topics:

/drive_vel - Subscribes to velocity commands
/drive_odom - Publishes odometry data
/om_response0, /om_state0 - Modbus communication
2. Linear Motor (Chokudo)
Hardware: Linear actuator for hose extension
Key Files:

src/serial_transciever/serial_transciever/chokudo_cameraswing_air_serial_node.py - Combined control with camera swing chokudo_cameraswing_air_serial_node.py:1-91
src/serial_transciever/serial_transciever/manipulator_control/motor_manual_chokudo_node.py - Manual control interface motor_manual_chokudo_node.py:1-169
Topics:

/chokudomotor/target_angle - Target angle commands
/chokudomotor/angle - Current angle feedback
3. Camera Swing Motor
Hardware: Motor for camera pitch control
Key Files:

src/serial_transciever/serial_transciever/chokudo_cameraswing_air_serial_node.py - Combined with Chokudo control chokudo_cameraswing_air_serial_node.py:1-91
src/serial_transciever/serial_transciever/manipulator_control/motor_manual_chokudo_node.py - Manual control motor_manual_chokudo_node.py:1-169
src/object_chaser/object_chaser/object_chaser_node.py - Automatic control during object chasing object_chaser_node.py:1-74
Topics:

/cameraswingmotor/target_angle - Target angle commands
/cameraswingmotor/angle - Current angle feedback
4. Steer Motors
Hardware: Dynamixel motors for wheel steering
Key Files:

src/movement/robot_motor2/src/robot_odom_node.cpp - Odometry calculation using steer data
src/movement/DynamixelSDK/dynamixel_sdk_examples/include/picking_robot_matrix.hpp - Kinematics calculations
Topics:

/steer_odom - Steering angle feedback
/wheel_odom - Combined odometry output
5. Integrated Control Nodes
Key Files:

src/serial_transciever/serial_transciever/manipulator_control/integrated_control_node.py - Unified control for Chokudo and Camera Swing integrated_control_node.py:70-135
src/serial_transciever/serial_transciever/manipulator_control/integrated_control_node.py - Keyboard control interface integrated_control_node.py:300-373
Notes
The drive motors use Oriental Motor BLV-R drivers with Modbus RTU communication over USB drive_motor.py:5-18 . The Chokudo and Camera Swing motors are controlled via a single serial connection to a Robotis OpenRB-150 board chokudo_cameraswing_air_serial_node.py:44-50 . The steer motors use Dynamixel SDK for communication dynamixel_sdk.h:26-31 .