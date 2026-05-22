#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <my_messages/msg/steer_motor.hpp>
#include <my_messages/msg/drive_motor.hpp>
#include <dynamixel_sdk_custom_interfaces/srv/get_position.hpp>
#include "omni_base_driver/picking_robot_matrix.hpp"
#include <cmath>

class CmdvelToMotorNode : public rclcpp::Node
{
public:
    CmdvelToMotorNode();
    ~CmdvelToMotorNode();

    // Publisher
    rclcpp::Publisher<my_messages::msg::DriveMotor>::SharedPtr drive_vel_pub;
    rclcpp::Publisher<my_messages::msg::SteerMotor>::SharedPtr steer_ang_pub;

    // Service Client
    rclcpp::Client<dynamixel_sdk_custom_interfaces::srv::GetPosition>::SharedPtr steer_client;

    // Variables
    double drive_vel[size];
    float steer_phi[size];

    void calcDiffPos(dynamixel_sdk_custom_interfaces::srv::GetPosition::Response::SharedPtr get_pos);
    void enforceCmdVelTimeout();

private:
    // Callback
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    // Subscriber
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;

    PickingRobotMatrix InvKinemaMatrix;

    rclcpp::Time last_cmd_vel_time;
    bool has_cmd_vel;
    const double cmd_vel_timeout_sec;
};

CmdvelToMotorNode::CmdvelToMotorNode() 
: Node("cmd_vel_to_motor_node"),
    drive_vel{0.0, 0.0, 0.0},
    steer_phi{0.0, 0.0, 0.0},
    last_cmd_vel_time(0, 0, this->get_clock()->get_clock_type()),
    has_cmd_vel(false),
    cmd_vel_timeout_sec(0.3)
{
    // Init subscriber
    cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&CmdvelToMotorNode::cmdVelCallback, this, std::placeholders::_1));

    // Init publishers
    drive_vel_pub = this->create_publisher<my_messages::msg::DriveMotor>(
        "drive_vel", 10);

    steer_ang_pub = this->create_publisher<my_messages::msg::SteerMotor>(
        "steer_angle", 10);

    // Init service client
    steer_client = this->create_client<dynamixel_sdk_custom_interfaces::srv::GetPosition>(
        "get_position");
    RCLCPP_INFO(this->get_logger(), "CmdvelToMotorNode initialized.");

}

CmdvelToMotorNode::~CmdvelToMotorNode()
{
    RCLCPP_INFO(this->get_logger(), "Closing CmdvelToMotor");
}

void CmdvelToMotorNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    last_cmd_vel_time = this->now();
    has_cmd_vel = true;

    // cmd_velの値をメンバ変数に代入
    double vx = msg->linear.x;
    double vy = msg->linear.y;
    double vth = msg->angular.z;
    
    // デバッグログ
    RCLCPP_INFO(this->get_logger(), "Received cmd_vel: vx=%f, vy=%f, vth=%f", vx, vy, vth);

    // 逆運動学計算
    InvKinemaMatrix.calcWheelVelAng(vx, vy, vth, drive_vel, steer_phi);
    
    auto drive = my_messages::msg::DriveMotor();
    drive.vel1 = drive_vel[0];
    drive.vel2 = drive_vel[1];
    drive.vel3 = drive_vel[2];

    drive_vel_pub->publish(drive);
    RCLCPP_INFO(this->get_logger(), "Published drive_vel: [%f, %f, %f]", drive.vel1, drive.vel2, drive.vel3);
    
    auto steer = my_messages::msg::SteerMotor();
    steer.phi1 = steer_phi[0];
    steer.phi2 = steer_phi[1];
    steer.phi3 = steer_phi[2];
    steer_ang_pub->publish(steer);
    RCLCPP_INFO(this->get_logger(), "Published steer_phi: [%f, %f, %f]", steer_phi[0], steer_phi[1], steer_phi[2]);
        
}

void CmdvelToMotorNode::enforceCmdVelTimeout()
{
    if (!has_cmd_vel)
    {
        for (int i = 0; i < size; i++)
        {
            drive_vel[i] = 0.0;
        }
        return;
    }

    if ((this->now() - last_cmd_vel_time).seconds() <= cmd_vel_timeout_sec)
    {
        return;
    }

    for (int i = 0; i < size; i++)
    {
        drive_vel[i] = 0.0;
    }
}

void CmdvelToMotorNode::calcDiffPos(dynamixel_sdk_custom_interfaces::srv::GetPosition::Response::SharedPtr get_pos)
{
    int set_pos[size], diff_pos[size], threshold;

    for (int i = 0; i < size; i++)
        set_pos[i] = home_pos[i] - int(steer_phi[i] / M_PI * 2048);

    diff_pos[0] = abs(set_pos[0] - get_pos->position1);
    diff_pos[1] = abs(set_pos[1] - get_pos->position2);
    diff_pos[2] = abs(set_pos[2] - get_pos->position3);

    threshold = 200;
    RCLCPP_INFO(this->get_logger(), "Calculated diff_pos: [%d, %d, %d]", diff_pos[0], diff_pos[1], diff_pos[2]);

    for (int i = 0; i < size; i++)
    {
        if (diff_pos[i] > threshold)
        {
            RCLCPP_WARN(this->get_logger(), "diff_pos[%d] exceeds threshold. Resetting drive_vel.", i);

            for (int j = 0; j < size; j++)
            {
                drive_vel[j] = 0.0;
            }
            break;
        }
    }
}
