#ifndef CMD_VEL_TO_SET_POSITION_HPP
#define CMD_VEL_TO_SET_POSITION_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"
#include <vector>
#include <cmath>
#include "motor_param.hpp"

class CmdVelToSetPosition : public rclcpp::Node
{
public:
  CmdVelToSetPosition();
  ~CmdVelToSetPosition() = default;

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  // Subscriber
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  // Publisher
  rclcpp::Publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>::SharedPtr set_position_pub_;

  // Service Client
  rclcpp::Client<dynamixel_sdk_custom_interfaces::srv::GetPosition>::SharedPtr get_position_client_;

  // Parameters
  std::vector<int> motor_ids_;
  const int max_position_;
  const int min_position_;
  std::vector<int> home_pos_;
  int positions_1_;
  int positions_2_;
  int positions_3_;
};

#endif  // CMD_VEL_TO_SET_POSITION_HPP
