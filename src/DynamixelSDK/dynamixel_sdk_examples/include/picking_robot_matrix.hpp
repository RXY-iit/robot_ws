#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"
#include "motor_param.hpp"

class PickingRobotMatrix : public rclcpp::Node
{
public:
    PickingRobotMatrix();  // コンストラクタ
    ~PickingRobotMatrix(); // デストラクタ
    void calcRobotVelocity(double &vx, double &vy, double &vth, const double *drive_vel, const float *steer_phi); // ロボットの順運動学
    void calcWheelVelAng(const double &vx, const double &vy, const double &vth, double *drive_vel, float *steer_phi); // ロボットの逆運動学

private:
    rclcpp::Client<dynamixel_sdk_custom_interfaces::srv::GetPosition>::SharedPtr steer_client_; // steer_motorの現在値取得のためのクライアント
    int get_pos[size]; // 操舵モータの現在値

    const float wheel_dist[size] = {0.30810, 0.30810, 0.38500}; // ロボット中心から各車輪までの距離
    const float wheel_phi[size] = {-0.842803, 0.842803, M_PI};  // ロボット中心から見た各車輪の位置(極座標)
    const double r_x[size] = {wheel_dist[0] * cos(wheel_phi[0]), wheel_dist[1] * cos(wheel_phi[1]), wheel_dist[2] * cos(wheel_phi[2])}; // 各車輪のx座標
    const double r_y[size] = {wheel_dist[0] * sin(wheel_phi[0]), wheel_dist[1] * sin(wheel_phi[1]), wheel_dist[2] * sin(wheel_phi[2])}; // 各車輪のy座標

    Eigen::Matrix<double, 6, 3> R;       // ロボットの表現行列
    Eigen::Matrix<double, 3, 6> R_inv;   // 表現行列Rの逆行列
    Eigen::Matrix<double, 6, 1> V_wheel; // 各車輪のvx, vy成分を格納
    Eigen::Matrix<double, 3, 1> V;       // vx, vy, vth
};

PickingRobotMatrix::PickngRobotMatrix()
    : Node("Picking_robot_matrix"),
      get_pos{home_pos[0], home_pos[1], home_pos[2]}
{
    // 初期化
    R << 1.0, 0.0, -r_y[0],
        0.0, 1.0, r_x[0],
        1.0, 0.0, -r_y[1],
        0.0, 1.0, r_x[1],
        1.0, 0.0, -r_y[2],
        0.0, 1.0, r_x[2];

    V = Eigen::MatrixXd::Zero(3, 1);
    V_wheel = Eigen::MatrixXd::Zero(6, 1);
    R_inv = (R.transpose() * R).inverse() * R.transpose();

    // 操舵モータの現在値取得のためのクライアントの初期化
    steer_client_ = this->create_client<dynamixel_sdk_custom_interfaces::srv::GetPosition>("get_position");
    RCLCPP_INFO(this->get_logger(), "BlowerRobotMatrix initialized.");
}

PickingRobotMatrix::~PickingRobotMatrix()
{
    RCLCPP_INFO(this->get_logger(), "Closing PickingRobotMatrix.");
}

void PickingRobotMatrix::calcRobotVelocity(double &vx, double &vy, double &vth, const double *drive_vel, const float *steer_phi)
{
    // 各車輪のvx, vyの格納
    for (int i = 0; i < size; i++)
    {
        V_wheel(i * 2, 0) = drive_vel[i] * cos(steer_phi[i]);
        V_wheel(i * 2 + 1, 0) = drive_vel[i] * sin(steer_phi[i]);
    }
    V = R_inv * V_wheel; // 運動学の計算

    vx = V(0, 0);
    vy = V(1, 0);
    vth = -V(2, 0);
}

void PickingRobotMatrix::calcWheelVelAng(const double &vx, const double &vy, const double &vth, double *drive_vel, float *steer_phi)
{
    V(0, 0) = vx;
    V(1, 0) = vy;
    V(2, 0) = vth;
    V_wheel = R * V;

    // 逆運動学計算
    for (int i = 0; i < size; i++)
    {
        drive_vel[i] = sqrt(pow(V_wheel(i * 2, 0), 2) + pow(V_wheel(i * 2 + 1, 0), 2));
        steer_phi[i] = std::atan2(V_wheel(i * 2 + 1, 0), V_wheel(i * 2, 0));

        if (steer_phi[i] < -0.71 * M_PI)
        {
            steer_phi[i] += M_PI;
            drive_vel[i] *= -1;
        }
        else if (steer_phi[i] > 0.71 * M_PI)
        {
            steer_phi[i] -= M_PI;
            drive_vel[i] *= -1;
        }
    }
}
