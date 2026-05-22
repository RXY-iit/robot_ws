#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"

class GetMotorPositionNode : public rclcpp::Node
{
public:
  GetMotorPositionNode()
  : Node("get_motor_position_node")
  {
    get_position_client_ = this->create_client<dynamixel_sdk_custom_interfaces::srv::GetPosition>("get_position");
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&GetMotorPositionNode::requestMotorPositions, this));

    RCLCPP_INFO(this->get_logger(), "GetMotorPositionNode initialized.");
  }

private:
  void requestMotorPositions()
  {
    if (!get_position_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(this->get_logger(), "get_position service not available");
      return;
    }

    auto request = std::make_shared<dynamixel_sdk_custom_interfaces::srv::GetPosition::Request>();
    request->id1 = 11;
    request->id2 = 12;
    request->id3 = 13;

    using ServiceResponseFuture =
      rclcpp::Client<dynamixel_sdk_custom_interfaces::srv::GetPosition>::SharedFuture;

    auto future_result = get_position_client_->async_send_request(request,
      [this](ServiceResponseFuture future)
      {
        try {
          auto response = future.get();
          RCLCPP_INFO(this->get_logger(), "Motor positions - ID11: %d, ID12: %d, ID13: %d",
                      response->position1, response->position2, response->position3);
        } catch (const std::exception &e) {
          RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
        }
      });
  }

  rclcpp::Client<dynamixel_sdk_custom_interfaces::srv::GetPosition>::SharedPtr get_position_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GetMotorPositionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
