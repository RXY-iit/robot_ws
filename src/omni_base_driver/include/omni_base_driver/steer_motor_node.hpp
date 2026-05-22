#pragma once

#include <rclcpp/rclcpp.hpp>
#include <my_messages/msg/steer_motor.hpp>
#include <cmath>
#include <dynamixel_sdk_custom_interfaces/msg/set_position.hpp>
#include <dynamixel_sdk_custom_interfaces/srv/get_position.hpp>
#include "motor_param.hpp"

class SteerMotorNode : public rclcpp::Node
{
public:
    SteerMotorNode();  // Constructor
    ~SteerMotorNode(); // Destructor

    // Setting position
    float set_pos[size];
    // Current position
    float current_pos[size];

    // Method
    void convertPositionRadian(std::shared_ptr<dynamixel_sdk_custom_interfaces::srv::GetPosition::Response> get_pos);

    // Callback
    void callback(const my_messages::msg::SteerMotor::SharedPtr ang_msg);

    // Publisher
    rclcpp::Publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>::SharedPtr steer_set_pub;
    rclcpp::Publisher<my_messages::msg::SteerMotor>::SharedPtr steer_odom_pub;

    // Subscriber
    rclcpp::Subscription<my_messages::msg::SteerMotor>::SharedPtr steer_ang_sub;

    // Service Client
    rclcpp::Client<dynamixel_sdk_custom_interfaces::srv::GetPosition>::SharedPtr steer_client;
};

SteerMotorNode::SteerMotorNode()
    : Node("steer_motor_node")
{
    for (int i = 0; i < size; i++)
        set_pos[i] = home_pos[i];

    // Subscriber
    steer_ang_sub = this->create_subscription<my_messages::msg::SteerMotor>(
        "steer_angle", 10, std::bind(&SteerMotorNode::callback, this, std::placeholders::_1));

    // Publisher
    steer_set_pub = this->create_publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>("set_position", 10);
    steer_odom_pub = this->create_publisher<my_messages::msg::SteerMotor>("steer_odom", 10);

    // Service Client
    steer_client = this->create_client<dynamixel_sdk_custom_interfaces::srv::GetPosition>("get_position");
}

SteerMotorNode::~SteerMotorNode()
{
    RCLCPP_INFO(this->get_logger(), "Shutting down SteerMotorNode");
}

void SteerMotorNode::callback(const my_messages::msg::SteerMotor::SharedPtr ang_msg)
{
    // Convert Radian to Position
    set_pos[0] = home_pos[0] - int(ang_msg->phi1 / M_PI * 2048);
    set_pos[1] = home_pos[1] - int(ang_msg->phi2 / M_PI * 2048);
    set_pos[2] = home_pos[2] - int(ang_msg->phi3 / M_PI * 2048);

    RCLCPP_INFO(this->get_logger(), "Updated Set Position: [%d, %d, %d]",
                static_cast<int>(set_pos[0]),
                static_cast<int>(set_pos[1]),
                static_cast<int>(set_pos[2]));
}

void SteerMotorNode::convertPositionRadian(std::shared_ptr<dynamixel_sdk_custom_interfaces::srv::GetPosition::Response> get_pos)
{
    // Convert current position to radians
    current_pos[0] = M_PI * (get_pos->position1 - home_pos[0]) / 2048;
    current_pos[1] = M_PI * (get_pos->position2 - home_pos[1]) / 2048;
    current_pos[2] = M_PI * (get_pos->position3 - home_pos[2]) / 2048;

    RCLCPP_INFO(this->get_logger(), "Current Positions (radians): [%f, %f, %f]",
                current_pos[0], current_pos[1], current_pos[2]);
}
