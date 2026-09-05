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
    int steer_error_threshold_ticks;
    int steer_error_report_period_ms;
};

CmdvelToMotorNode::CmdvelToMotorNode() 
: Node("cmd_vel_to_motor_node"),
    drive_vel{0.0, 0.0, 0.0},
    steer_phi{0.0, 0.0, 0.0},
    last_cmd_vel_time(0, 0, this->get_clock()->get_clock_type()),
    has_cmd_vel(false),
    cmd_vel_timeout_sec(0.08),
    steer_error_threshold_ticks(200),
    steer_error_report_period_ms(1000)
{
    steer_error_threshold_ticks = this->declare_parameter<int>(
        "steer_error_threshold_ticks", steer_error_threshold_ticks);
    steer_error_report_period_ms = this->declare_parameter<int>(
        "steer_error_report_period_ms", steer_error_report_period_ms);
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
    RCLCPP_INFO(
        this->get_logger(),
        "CmdvelToMotorNode initialized. steer_error_threshold=%d ticks (%.1f deg)",
        steer_error_threshold_ticks,
        steer_error_threshold_ticks * 360.0 / 4096.0);

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
    
    RCLCPP_DEBUG(this->get_logger(), "Received cmd_vel: vx=%f, vy=%f, vth=%f", vx, vy, vth);

    // 逆運動学計算
    InvKinemaMatrix.calcWheelVelAng(vx, vy, vth, drive_vel, steer_phi);
    
    auto drive = my_messages::msg::DriveMotor();
    drive.vel1 = drive_vel[0];
    drive.vel2 = drive_vel[1];
    drive.vel3 = drive_vel[2];

    auto steer = my_messages::msg::SteerMotor();
    steer.phi1 = steer_phi[0];
    steer.phi2 = steer_phi[1];
    steer.phi3 = steer_phi[2];
    RCLCPP_DEBUG(this->get_logger(), "Calculated drive/steer command");
        
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
    int set_pos[size], diff_pos[size];

    for (int i = 0; i < size; i++)
        set_pos[i] = home_pos[i] - int(steer_phi[i] / M_PI * 2048);

    diff_pos[0] = abs(set_pos[0] - get_pos->position1);
    diff_pos[1] = abs(set_pos[1] - get_pos->position2);
    diff_pos[2] = abs(set_pos[2] - get_pos->position3);

    // Keep one periodic measurement in the normal INFO log so the real tick
    // distribution can be evaluated before changing the safety threshold.
    RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), steer_error_report_period_ms,
        "steer_error ticks=[%d,%d,%d] deg=[%.1f,%.1f,%.1f] target=[%d,%d,%d] actual=[%d,%d,%d] threshold=%d",
        diff_pos[0], diff_pos[1], diff_pos[2],
        diff_pos[0] * 360.0 / 4096.0,
        diff_pos[1] * 360.0 / 4096.0,
        diff_pos[2] * 360.0 / 4096.0,
        set_pos[0], set_pos[1], set_pos[2],
        get_pos->position1, get_pos->position2, get_pos->position3,
        steer_error_threshold_ticks);

    for (int i = 0; i < size; i++)
    {
        if (diff_pos[i] > steer_error_threshold_ticks)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 500,
                "steer_error_exceeded axis=%d error=%d ticks (%.1f deg) target=%d actual=%d threshold=%d; drive_vel reset",
                i, diff_pos[i], diff_pos[i] * 360.0 / 4096.0,
                set_pos[i],
                i == 0 ? get_pos->position1 : (i == 1 ? get_pos->position2 : get_pos->position3),
                steer_error_threshold_ticks);

            for (int j = 0; j < size; j++)
            {
                drive_vel[j] = 0.0;
            }
            break;
        }
    }
}
