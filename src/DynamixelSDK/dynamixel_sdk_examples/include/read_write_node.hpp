#ifndef READ_WRITE_NODE_HPP_
#define READ_WRITE_NODE_HPP_

#include <cstdio>
#include <algorithm>
#include <array>
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"
#include "std_msgs/msg/string.hpp"

class ReadWriteNode : public rclcpp::Node
{
public:
  using SetPosition = dynamixel_sdk_custom_interfaces::msg::SetPosition;
  using GetPosition = dynamixel_sdk_custom_interfaces::srv::GetPosition;

  ReadWriteNode();
  virtual ~ReadWriteNode();

private:
  struct MotorReadStatus
  {
    bool identity_observed{false};
    uint16_t model_number{0};
    int ping_comm_result{0};
    uint8_t ping_device_error{0};
    bool observed{false};
    bool comm_success{false};
    int comm_result{0};
    uint8_t device_error{0};
    uint64_t attempts{0};
    uint64_t successes{0};
    std::chrono::steady_clock::time_point last_success{};
    bool hardware_status_supported{false};
    bool hardware_observed{false};
    bool hardware_comm_success{false};
    int hardware_comm_result{0};
    uint8_t hardware_device_error{0};
    uint8_t hardware_error_status{0};
    int32_t present_current_raw{0};
    double present_current_a{0.0};
    double input_voltage_v{0.0};
    uint8_t temperature_c{0};
    uint64_t hardware_attempts{0};
    uint64_t hardware_successes{0};
    std::chrono::steady_clock::time_point last_hardware_success{};
  };

  rclcpp::Subscription<SetPosition>::SharedPtr set_position_subscriber_;
  rclcpp::Service<GetPosition>::SharedPtr get_position_server_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr health_publisher_;
  rclcpp::TimerBase::SharedPtr health_timer_;
  rclcpp::TimerBase::SharedPtr hardware_timer_;
  std::array<MotorReadStatus, 3> motor_read_status_{};

  void handle_set_position(const SetPosition::SharedPtr msg);
  void handle_get_position(
    const std::shared_ptr<GetPosition::Request> request,
    std::shared_ptr<GetPosition::Response> response);

  void setupDynamixel(uint8_t dxl_id);

  int32_t read_present_position(uint8_t dxl_id);
  void sample_hardware_status();
  void publish_health_summary();

  static constexpr uint8_t motor_ids_[3] = {11, 12, 13};
};

#endif  // READ_WRITE_NODE_HPP_
