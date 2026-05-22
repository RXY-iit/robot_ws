#include "omni_base_driver/steer_motor_node.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<SteerMotorNode>();

    // for set steer motor position to dynamixel Publish
    auto set_pos = dynamixel_sdk_custom_interfaces::msg::SetPosition();
    // for get steer motor position Service
    auto get_pos_request = std::make_shared<dynamixel_sdk_custom_interfaces::srv::GetPosition::Request>();
    // for give steer motor position to odom node Publish
    auto steer_odom = my_messages::msg::SteerMotor();

    set_pos.id1 = 11;
    set_pos.id2 = 12;
    set_pos.id3 = 13;

    set_pos.position1 = home_pos[0];
    set_pos.position2 = home_pos[1];
    set_pos.position3 = home_pos[2];
            
    steer_odom.phi1 = 0.0;
    steer_odom.phi2 = 0.0;
    steer_odom.phi3 = 0.0;

    get_pos_request->id1 = 11;
    get_pos_request->id2 = 12;
    get_pos_request->id3 = 13;

    // avoid service client call error
    rclcpp::sleep_for(std::chrono::milliseconds(200));

    rclcpp::Rate loop(60);

    while (rclcpp::ok())
    {
        if (node->steer_client->wait_for_service(std::chrono::seconds(1)))
        {
            auto future = node->steer_client->async_send_request(get_pos_request);
            if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS)
            {
                node->convertPositionRadian(future.get());
            }
            else
            {
                RCLCPP_ERROR(node->get_logger(), "Failed to call service get_position");
            }
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "Service get_position not available");
        }

        steer_odom.header.stamp = node->now();
        steer_odom.phi1 = node->current_pos[0];
        steer_odom.phi2 = node->current_pos[1];
        steer_odom.phi3 = node->current_pos[2];

        set_pos.position1 = node->set_pos[0];
        set_pos.position2 = node->set_pos[1];
        set_pos.position3 = node->set_pos[2];

        node->steer_set_pub->publish(set_pos);
        node->steer_odom_pub->publish(steer_odom);

        rclcpp::spin_some(node);
        loop.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
