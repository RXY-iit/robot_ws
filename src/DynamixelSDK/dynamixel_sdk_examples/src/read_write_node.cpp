#include "read_write_node.hpp"

/* Define Control Table Addresses and Constants */
#define ADDR_OPERATING_MODE       11
#define ADDR_TORQUE_ENABLE        64
#define ADDR_GOAL_POSITION        116
#define ADDR_PRESENT_POSITION     132
#define PROTOCOL_VERSION          2.0
#define BAUDRATE                  57600
#define DEVICE_NAME               "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT4TCWV6-if00-port0"
#define TORQUE_ENABLE             1
#define TORQUE_DISABLE            0

/* Global variables for Port and Packet Handlers */
dynamixel::PortHandler * portHandler;
dynamixel::PacketHandler * packetHandler;

/******************************************************************************/
/* Constructor                                                                */
/******************************************************************************/
ReadWriteNode::ReadWriteNode()
: Node("read_write_node")
{
  RCLCPP_INFO(this->get_logger(), "Initializing ReadWriteNode");

  portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open the port
  if (!portHandler->openPort()) {
    RCLCPP_FATAL(this->get_logger(), "Failed to open the port!");
    throw std::runtime_error("Failed to open the port!");
  }

  // Set baudrate
  if (!portHandler->setBaudRate(BAUDRATE)) {
    RCLCPP_FATAL(this->get_logger(), "Failed to set the baudrate!");
    throw std::runtime_error("Failed to set the baudrate!");
  }

  // Initialize Dynamixel motors
  for (const auto &id : motor_ids_) {
    setupDynamixel(id);
  }

  // Subscriber for set_position
  set_position_subscriber_ = this->create_subscription<SetPosition>(
    "set_position", 10,
    [this](const SetPosition::SharedPtr msg) { handle_set_position(msg); });

  // Service for get_position
  get_position_server_ = this->create_service<GetPosition>(
    "get_position",
    [this](
      const std::shared_ptr<GetPosition::Request> request,
      std::shared_ptr<GetPosition::Response> response) { handle_get_position(request, response); });

  RCLCPP_INFO(this->get_logger(), "Node Initialized");
}

/******************************************************************************/
/* Destructor                                                                 */
/******************************************************************************/
ReadWriteNode::~ReadWriteNode()
{
  for (const auto &id : motor_ids_) {
    packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, nullptr);
  }
  portHandler->closePort();
}

/******************************************************************************/
/* Private Methods                                                            */
/******************************************************************************/
void ReadWriteNode::setupDynamixel(uint8_t dxl_id)
{
  uint8_t dxl_error = 0;

  // Set to position control mode
  if (packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_OPERATING_MODE, 4, &dxl_error) != COMM_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set Position Control Mode for ID: %d", dxl_id);
  }

  // Enable torque
  if (packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error) != COMM_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to enable torque for ID: %d", dxl_id);
  }
}

void ReadWriteNode::handle_set_position(const SetPosition::SharedPtr msg)
{
  uint8_t dxl_error = 0;

    // Set goal positions for each motor
  uint32_t positions[3] = {
    static_cast<uint32_t>(msg->position1),
    static_cast<uint32_t>(msg->position2),
    static_cast<uint32_t>(msg->position3)
  };
  for (size_t i = 0; i < 3; ++i) {
    if (packetHandler->write4ByteTxRx(portHandler, motor_ids_[i], ADDR_GOAL_POSITION, positions[i], &dxl_error) != COMM_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set position for ID: %d", motor_ids_[i]);
    } else {
      RCLCPP_INFO(this->get_logger(), "Set Position for ID: %d -> %d", motor_ids_[i], positions[i]);
    }
  }
}

void ReadWriteNode::handle_get_position(
  const std::shared_ptr<GetPosition::Request> request,
  std::shared_ptr<GetPosition::Response> response)
{
  int32_t positions[3];
  for (size_t i = 0; i < 3; ++i) {
    positions[i] = read_present_position(motor_ids_[i]);
  }
  response->position1 = positions[0];
  response->position2 = positions[1];
  response->position3 = positions[2];

  RCLCPP_INFO(this->get_logger(), "Returned Positions: [%d, %d, %d]", positions[0], positions[1], positions[2]);
}

int32_t ReadWriteNode::read_present_position(uint8_t dxl_id)
{
  uint8_t dxl_error = 0;
  uint32_t present_position = 0;

  if (packetHandler->read4ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_POSITION, &present_position, &dxl_error) != COMM_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to read position for ID: %d", dxl_id);
    return -1;
  }
  return static_cast<int32_t>(present_position);
}

/******************************************************************************/
/* Main Function                                                              */
/******************************************************************************/
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<ReadWriteNode>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("read_write_node"), "Exception: %s", e.what());
  }

  rclcpp::shutdown();
  return 0;
}