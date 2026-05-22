// #include <cstdio>
// #include <memory>
// #include <string>

// #include "dynamixel_sdk/dynamixel_sdk.h"
// #include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
// #include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"
// #include "rclcpp/rclcpp.hpp"
// #include "rcutils/cmdline_parser.h"

// #include "read_write_node.hpp"

// using namespace dynamixel;

// // Control table address
// #define ADDR_TORQUE_ENABLE    64
// #define ADDR_PRESENT_POSITION 132
// #define ADDR_GOAL_POSITION    116

// // Protocol version
// #define PROTOCOL_VERSION      2.0

// // Default setting
// #define DXL1_ID               11
// #define DXL2_ID               12
// #define DXL3_ID               13
// #define BAUDRATE              57600
// #define DEVICE_NAME           "/dev/ttyUSB1"

// PortHandler * portHandler = PortHandler::getPortHandler(DEVICE_NAME);
// PacketHandler * packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

// GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, 4);
// GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, 4);

// class TriReadWriteNode : public rclcpp::Node
// {
// public:
//   TriReadWriteNode()
//   : Node("tri_read_write_node")
//   {
//     // サービスとサブスクリプションの作成
//     steer_get_pos_srv_ = this->create_service<dynamixel_sdk_examples::srv::SyncGetPosition>(
//       "/steer_node/sync_get_position",
//       std::bind(
//         &TriReadWriteNode::syncGetPresentPositionCallback,
//         this,
//         std::placeholders::_1,
//         // std::placeholders::_2));

//     cmd_get_pos_srv_ = this->create_service<dynamixel_sdk_examples::srv::SyncGetPosition>(
//       "/cmd_steer/sync_get_position",
//       std::bind(
//         &TriReadWriteNode::syncGetPresentPositionCallback,
//         this,
//         std::placeholders::_1,
//         std::placeholders::_2));

//     sync_set_position_sub_ = this->create_subscription<dynamixel_sdk_examples::msg::SyncSetPosition>(
//       "/sync_set_position",
//       10,
//       std::bind(
//         &TriReadWriteNode::syncSetPositionCallback,
//         this,
//         std::placeholders::_1));

//     if (!portHandler->openPort()) {
//       RCLCPP_ERROR(this->get_logger(), "Failed to open the port!");
//       rclcpp::shutdown();
//     }

//     if (!portHandler->setBaudRate(BAUDRATE)) {
//       RCLCPP_ERROR(this->get_logger(), "Failed to set the baudrate!");
//       rclcpp::shutdown();
//     }

//     enableTorque(DXL1_ID);
//     enableTorque(DXL2_ID);
//     enableTorque(DXL3_ID);
//   }

//   ~TriReadWriteNode()
//   {
//     portHandler->closePort();
//   }

// private:
//   void enableTorque(uint8_t id)
//   {
//     uint8_t dxl_error = 0;
//     int dxl_comm_result = packetHandler->write1ByteTxRx(
//       portHandler, id, ADDR_TORQUE_ENABLE, 1, &dxl_error);

//     if (dxl_comm_result != COMM_SUCCESS) {
//       RCLCPP_ERROR(this->get_logger(), "Failed to enable torque for Dynamixel ID %d", id);
//       rclcpp::shutdown();
//     }
//   }

//   bool syncGetPresentPositionCallback(
//     const std::shared_ptr<dynamixel_sdk_examples::srv::SyncGetPosition::Request> req,
//     std::shared_ptr<dynamixel_sdk_examples::srv::SyncGetPosition::Response> res)
//   {
//     uint8_t dxl_error = 0;
//     int dxl_comm_result = COMM_TX_FAIL;

//     dxl_comm_result = groupSyncRead.txRxPacket();
//     if (dxl_comm_result == COMM_SUCCESS) {
//       res->position1 = groupSyncRead.getData(req->id1, ADDR_PRESENT_POSITION, 4);
//       return true;
//     } else {
//       RCLCPP_ERROR(this->get_logger(), "Failed to get position!");
//       return false;
//     }
//   }

//   void syncSetPositionCallback(
//     const dynamixel_sdk_examples::msg::SyncSetPosition::SharedPtr msg)
//   {
//     // サブスクライブされたデータでポジションを設定する処理
//   }

//   rclcpp::Service<dynamixel_sdk_examples::srv::SyncGetPosition>::SharedPtr steer_get_pos_srv_;
//   rclcpp::Service<dynamixel_sdk_examples::srv::SyncGetPosition>::SharedPtr cmd_get_pos_srv_;
//   rclcpp::Subscription<dynamixel_sdk_examples::msg::SyncSetPosition>::SharedPtr sync_set_position_sub_;
// };

// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<TriReadWriteNode>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }
