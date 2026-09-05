// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/joy.hpp>
// #include "robot_motor/msg/steer_motor.hpp"
// #include <my_messages/msg/drive_vel.hpp>
#include "omni_base_driver/cmd_vel_to_motor_node.hpp"
// #include <dynamixel_sdk_custom_interfaces/srv/get_position.hpp>
// #include <my_messages/msg/steermotor.hpp>

int main(int argc, char **argv)
{
    // ROS2ノードの初期化
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CmdvelToMotorNode>();

    // for steer motor position publish
    auto steer = my_messages::msg::SteerMotor();
    // for right rear drive motor velocity publish
    auto drive = my_messages::msg::DriveMotor();

    // サービスリクエストの準備
    auto get_pos_request = std::make_shared<dynamixel_sdk_custom_interfaces::srv::GetPosition::Request>();
    get_pos_request->id1 = 11;
    get_pos_request->id2 = 12;
    get_pos_request->id3 = 13;

    // 0.2秒の待機（スリープ）
    rclcpp::sleep_for(std::chrono::milliseconds(200));

    // Keep motor command publication independent from the comparatively slow
    // Dynamixel position service.  The previous synchronous service call
    // blocked this loop and produced visible gaps in /drive_vel.
    rclcpp::Rate loop(100);
    bool steer_request_pending = false;
    auto last_steer_request = node->now();

    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        node->enforceCmdVelTimeout();

        // set steer motor position
        steer.phi1 = node->steer_phi[0];
        steer.phi2 = node->steer_phi[1];
        steer.phi3 = node->steer_phi[2];

        // set drive motor velocity
        drive.vel1 = node->drive_vel[0];
        drive.vel2 = node->drive_vel[1];
        drive.vel3 = node->drive_vel[2];

        node->steer_ang_pub->publish(steer);
        node->drive_vel_pub->publish(drive);

        // At most one asynchronous steer-position request every 50 ms.  Never
        // wait here: stopping /drive_vel must not depend on this service.
        if (!steer_request_pending &&
            (node->now() - last_steer_request).seconds() >= 0.05 &&
            node->steer_client->service_is_ready())
        {
            steer_request_pending = true;
            last_steer_request = node->now();
            node->steer_client->async_send_request(
                get_pos_request,
                [&node, &steer_request_pending](
                    rclcpp::Client<dynamixel_sdk_custom_interfaces::srv::GetPosition>::SharedFuture future)
                {
                    steer_request_pending = false;
                    try {
                        node->calcDiffPos(future.get());
                    } catch (const std::exception & exc) {
                        RCLCPP_ERROR(node->get_logger(),
                            "get_position response failed: %s", exc.what());
                    }
                });
        }

        // Publish
        // NavToMotor.right_rear_vel_pub->publish(right_rear_drive);
        // NavToMotor.left_vel_pub->publish(left_drive);

        loop.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
