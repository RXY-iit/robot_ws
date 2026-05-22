#ifndef READ_WRITE_NODE_HPP_
#define READ_WRITE_NODE_HPP_

#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"

class ReadWriteNode : public rclcpp::Node
{
public:
  using SetPosition = dynamixel_sdk_custom_interfaces::msg::SetPosition;
  using GetPosition = dynamixel_sdk_custom_interfaces::srv::GetPosition;

  ReadWriteNode();
  virtual ~ReadWriteNode();

private:
  rclcpp::Subscription<SetPosition>::SharedPtr set_position_subscriber_;
  rclcpp::Service<GetPosition>::SharedPtr get_position_server_;

  void handle_set_position(const SetPosition::SharedPtr msg);
  void handle_get_position(
    const std::shared_ptr<GetPosition::Request> request,
    std::shared_ptr<GetPosition::Response> response);

  void setupDynamixel(uint8_t dxl_id);

  int32_t read_present_position(uint8_t dxl_id);

  static constexpr uint8_t motor_ids_[3] = {11, 12, 13};
};

#endif  // READ_WRITE_NODE_HPP_
