#include "cmd_vel_to_set_position.hpp"
#include "motor_param.hpp"

#include <algorithm> // std::clamp

CmdVelToSetPosition::CmdVelToSetPosition()
: Node("cmd_vel_to_setposition"),
  max_position_(4095),
  min_position_(0)
{
  // cmd_velの購読
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10,
    std::bind(&CmdVelToSetPosition::cmdVelCallback, this, std::placeholders::_1));

  // set_positionのパブリッシャー
  set_position_pub_ = this->create_publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>(
    "set_position", 5);

  // get_positionサービスクライアント
  get_position_client_ = this->create_client<dynamixel_sdk_custom_interfaces::srv::GetPosition>(
    "get_position");

  // モーターIDの設定
  motor_ids_ = {11, 12, 13};
  positions_1_ = home_pos[0]; // モータ1
  positions_2_ = home_pos[1]; // モータ2
  positions_3_ = home_pos[2]; // モータ3

  RCLCPP_INFO(this->get_logger(), "CmdVelToSetPosition node initialized.");
}

void CmdVelToSetPosition::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  double angular_velocity = msg->angular.z;

  RCLCPP_INFO(this->get_logger(), "Received angular velocity: %f", angular_velocity);

  dynamixel_sdk_custom_interfaces::msg::SetPosition position_msg;
  position_msg.id1 = motor_ids_[0];
  position_msg.id2 = motor_ids_[1];
  position_msg.id3 = motor_ids_[2];

  position_msg.position1 = std::clamp(positions_1_ - static_cast<int>(angular_velocity / M_PI * 2048), min_position_, max_position_);
  position_msg.position2 = std::clamp(positions_2_ - static_cast<int>(angular_velocity / M_PI * 2048), min_position_, max_position_);
  position_msg.position3 = std::clamp(positions_3_ - static_cast<int>(angular_velocity / M_PI * 2048), min_position_, max_position_);

  set_position_pub_->publish(position_msg);

  RCLCPP_INFO(this->get_logger(), "Published positions: [%d, %d, %d]", position_msg.position1, position_msg.position2, position_msg.position3);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CmdVelToSetPosition>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}