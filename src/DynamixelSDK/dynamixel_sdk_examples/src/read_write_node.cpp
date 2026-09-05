#include "read_write_node.hpp"

#include <sstream>

/* Define Control Table Addresses and Constants */
#define ADDR_OPERATING_MODE       11
#define ADDR_TORQUE_ENABLE        64
#define ADDR_HARDWARE_ERROR_STATUS 70
#define ADDR_GOAL_POSITION        116
#define ADDR_PRESENT_CURRENT      126
#define ADDR_PRESENT_POSITION     132
#define ADDR_PRESENT_INPUT_VOLTAGE 144
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

  // Publish only results from the reads already performed by get_position.
  // No unverified control-table register is accessed by this health path.
  health_publisher_ = this->create_publisher<std_msgs::msg::String>(
    "/dynamixel/health_summary", 10);
  health_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&ReadWriteNode::publish_health_summary, this));
  // Model 1100 was observed by direct Protocol 2.0 ping and maps to the
  // official ROBOTIS XH540-W270 control table.  Sample its slow-changing
  // hardware registers at 1 Hz; position feedback remains on its existing
  // request path.
  hardware_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&ReadWriteNode::sample_hardware_status, this));

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
  uint16_t model_number = 0;
  const auto status_it = std::find(std::begin(motor_ids_), std::end(motor_ids_), dxl_id);
  const size_t status_index = static_cast<size_t>(std::distance(std::begin(motor_ids_), status_it));
  auto & status = motor_read_status_.at(status_index);

  // Protocol 2.0 ping returns the device's own model number.  This provides
  // identity evidence without assuming a control table from compiled
  // register addresses.
  const int ping_result = packetHandler->ping(
    portHandler, dxl_id, &model_number, &dxl_error);
  status.identity_observed = ping_result == COMM_SUCCESS;
  status.model_number = model_number;
  status.hardware_status_supported = ping_result == COMM_SUCCESS && model_number == 1100;
  status.ping_comm_result = ping_result;
  status.ping_device_error = dxl_error;
  if (ping_result == COMM_SUCCESS) {
    RCLCPP_INFO(
      this->get_logger(), "DYNAMIXEL ID %d reports model number %u",
      dxl_id, static_cast<unsigned int>(model_number));
  } else {
    RCLCPP_ERROR(
      this->get_logger(), "DYNAMIXEL ping failed for ID %d (comm_result=%d)",
      dxl_id, ping_result);
  }

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
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Failed to set position for ID: %d", motor_ids_[i]);
    } else {
      RCLCPP_DEBUG(this->get_logger(), "Set Position for ID: %d -> %d", motor_ids_[i], positions[i]);
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

  RCLCPP_DEBUG(this->get_logger(), "Returned Positions: [%d, %d, %d]", positions[0], positions[1], positions[2]);
}

int32_t ReadWriteNode::read_present_position(uint8_t dxl_id)
{
  uint8_t dxl_error = 0;
  uint32_t present_position = 0;
  const auto status_it = std::find(std::begin(motor_ids_), std::end(motor_ids_), dxl_id);
  const size_t status_index = static_cast<size_t>(std::distance(std::begin(motor_ids_), status_it));
  auto & status = motor_read_status_.at(status_index);
  status.observed = true;
  status.attempts += 1;

  const int comm_result = packetHandler->read4ByteTxRx(
    portHandler, dxl_id, ADDR_PRESENT_POSITION, &present_position, &dxl_error);
  status.comm_result = comm_result;
  status.comm_success = comm_result == COMM_SUCCESS;
  status.device_error = dxl_error;
  if (comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Failed to read position for ID: %d (comm_result=%d)", dxl_id, comm_result);
    return -1;
  }
  status.successes += 1;
  status.last_success = std::chrono::steady_clock::now();
  if (dxl_error != 0) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "DYNAMIXEL ID %d returned device error byte 0x%02x", dxl_id, dxl_error);
  }
  return static_cast<int32_t>(present_position);
}

void ReadWriteNode::sample_hardware_status()
{
  for (size_t index = 0; index < motor_read_status_.size(); ++index) {
    auto & status = motor_read_status_[index];
    if (!status.hardware_status_supported) {
      continue;
    }

    status.hardware_observed = true;
    status.hardware_attempts += 1;
    uint8_t hardware_error_status = 0;
    uint16_t present_current_raw = 0;
    uint8_t voltage_temperature[3] = {0, 0, 0};
    uint8_t register_error = 0;
    uint8_t combined_device_error = 0;
    int first_failure = COMM_SUCCESS;

    int result = packetHandler->read1ByteTxRx(
      portHandler, motor_ids_[index], ADDR_HARDWARE_ERROR_STATUS,
      &hardware_error_status, &register_error);
    combined_device_error |= register_error;
    if (result != COMM_SUCCESS) {
      first_failure = result;
    }

    register_error = 0;
    result = packetHandler->read2ByteTxRx(
      portHandler, motor_ids_[index], ADDR_PRESENT_CURRENT,
      &present_current_raw, &register_error);
    combined_device_error |= register_error;
    if (first_failure == COMM_SUCCESS && result != COMM_SUCCESS) {
      first_failure = result;
    }

    register_error = 0;
    result = packetHandler->readTxRx(
      portHandler, motor_ids_[index], ADDR_PRESENT_INPUT_VOLTAGE, 3,
      voltage_temperature, &register_error);
    combined_device_error |= register_error;
    if (first_failure == COMM_SUCCESS && result != COMM_SUCCESS) {
      first_failure = result;
    }

    status.hardware_comm_result = first_failure;
    status.hardware_device_error = combined_device_error;
    status.hardware_comm_success = first_failure == COMM_SUCCESS;
    if (!status.hardware_comm_success) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Failed to read XH540-W270 hardware status for ID: %d (comm_result=%d)",
        motor_ids_[index], first_failure);
      continue;
    }

    const int32_t signed_current = present_current_raw >= 0x8000
      ? static_cast<int32_t>(present_current_raw) - 0x10000
      : static_cast<int32_t>(present_current_raw);
    const uint16_t input_voltage_raw = static_cast<uint16_t>(voltage_temperature[0]) |
      (static_cast<uint16_t>(voltage_temperature[1]) << 8);
    status.hardware_error_status = hardware_error_status;
    status.present_current_raw = signed_current;
    status.present_current_a = static_cast<double>(signed_current) * 0.00269;
    status.input_voltage_v = static_cast<double>(input_voltage_raw) * 0.1;
    status.temperature_c = voltage_temperature[2];
    status.hardware_successes += 1;
    status.last_hardware_success = std::chrono::steady_clock::now();
  }
}

void ReadWriteNode::publish_health_summary()
{
  const auto now = std::chrono::steady_clock::now();
  std::ostringstream json;
  json << "{\"schema_version\":1,\"source\":\"direct DYNAMIXEL transactions\",";
  json << "\"publish_rate_hz\":2.0,\"motors\":[";
  for (size_t index = 0; index < motor_read_status_.size(); ++index) {
    const auto & status = motor_read_status_[index];
    if (index > 0) {
      json << ',';
    }
    json << "{\"id\":" << static_cast<int>(motor_ids_[index]);
    json << ",\"identity_observed\":" << (status.identity_observed ? "true" : "false");
    json << ",\"model_number\":" << status.model_number;
    json << ",\"model_name\":";
    if (status.model_number == 1100) {
      json << "\"XH540-W270\"";
    } else {
      json << "null";
    }
    json << ",\"ping_comm_result\":" << status.ping_comm_result;
    json << ",\"ping_device_error\":" << static_cast<int>(status.ping_device_error);
    json << ",\"observed\":" << (status.observed ? "true" : "false");
    json << ",\"comm_success\":" << (status.comm_success ? "true" : "false");
    json << ",\"comm_result\":" << status.comm_result;
    json << ",\"device_error\":" << static_cast<int>(status.device_error);
    json << ",\"attempts\":" << status.attempts;
    json << ",\"successes\":" << status.successes;
    json << ",\"last_success_age_sec\":";
    if (status.successes == 0) {
      json << "null";
    } else {
      const std::chrono::duration<double> age = now - status.last_success;
      json << age.count();
    }
    json << ",\"hardware_status_supported\":" <<
      (status.hardware_status_supported ? "true" : "false");
    json << ",\"hardware_observed\":" << (status.hardware_observed ? "true" : "false");
    json << ",\"hardware_comm_success\":" <<
      (status.hardware_comm_success ? "true" : "false");
    json << ",\"hardware_comm_result\":" << status.hardware_comm_result;
    json << ",\"hardware_device_error\":" << static_cast<int>(status.hardware_device_error);
    json << ",\"hardware_error_status\":" << static_cast<int>(status.hardware_error_status);
    json << ",\"present_current_raw\":" << status.present_current_raw;
    json << ",\"present_current_a\":" << status.present_current_a;
    json << ",\"input_voltage_v\":" << status.input_voltage_v;
    json << ",\"temperature_c\":" << static_cast<int>(status.temperature_c);
    json << ",\"hardware_attempts\":" << status.hardware_attempts;
    json << ",\"hardware_successes\":" << status.hardware_successes;
    json << ",\"last_hardware_success_age_sec\":";
    if (status.hardware_successes == 0) {
      json << "null";
    } else {
      const std::chrono::duration<double> age = now - status.last_hardware_success;
      json << age.count();
    }
    json << '}';
  }
  json << "]}";
  std_msgs::msg::String message;
  message.data = json.str();
  health_publisher_->publish(message);
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
