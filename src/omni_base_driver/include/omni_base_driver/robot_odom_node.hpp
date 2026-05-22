#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <my_messages/msg/steer_motor.hpp>
#include <my_messages/msg/drive_motor.hpp>
#include <message_filters/subscriber.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "omni_base_driver/picking_robot_matrix.hpp"
#include <mutex>

class PickingRobotOdom : public rclcpp::Node
{
public:
    PickingRobotOdom();
    ~PickingRobotOdom();
    void calcOdometry(const rclcpp::Time& current_time);

    // Odometry data
    double x, y, th;
    double vx, vy, vth;
    rclcpp::Time current_time, last_time;
    std::string frame_id, child_frame_id;
    bool is_first_callback;

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

    // Subscribers
    rclcpp::Subscription<my_messages::msg::SteerMotor>::SharedPtr steer_sub;
    rclcpp::Subscription<my_messages::msg::DriveMotor>::SharedPtr drive_sub;
    
    // Timer for periodic odom calculation
    rclcpp::TimerBase::SharedPtr timer;

private:
    // Callbacks
    void steerCallback(const my_messages::msg::SteerMotor::ConstSharedPtr& msg);
    void driveCallback(const my_messages::msg::DriveMotor::ConstSharedPtr& msg);
    void timerCallback();
    
    // Latest messages
    my_messages::msg::SteerMotor latest_steer_msg;
    my_messages::msg::DriveMotor latest_drive_msg;
    bool has_steer_msg, has_drive_msg;
    std::mutex msg_mutex;
    
    const int size;
    PickingRobotMatrix KinemaMatrix;
};

