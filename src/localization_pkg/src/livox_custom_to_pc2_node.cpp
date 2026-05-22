#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

namespace
{
constexpr uint32_t kPointStep = 16;  // x, y, z, intensity as float32

void write_float(std::vector<uint8_t> & data, const size_t offset, const float value)
{
  std::memcpy(data.data() + offset, &value, sizeof(float));
}
}  // namespace

class LivoxCustomToPc2Node : public rclcpp::Node
{
public:
  LivoxCustomToPc2Node()
  : Node("livox_custom_to_pc2")
  {
    auto qos = rclcpp::SensorDataQoS();
    sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
      "/livox/lidar", qos,
      std::bind(&LivoxCustomToPc2Node::callback, this, std::placeholders::_1));
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/livox/lidar_pc2", qos);

    RCLCPP_INFO(
      get_logger(),
      "livox_custom_to_pc2: C++ relay ready  /livox/lidar -> /livox/lidar_pc2");
  }

private:
  void callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
  {
    if (msg->points.empty()) {
      return;
    }

    sensor_msgs::msg::PointCloud2 out;
    out.header = msg->header;
    out.height = 1;
    out.is_bigendian = false;
    out.is_dense = true;
    out.point_step = kPointStep;
    out.fields.resize(4);

    out.fields[0].name = "x";
    out.fields[0].offset = 0;
    out.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    out.fields[0].count = 1;

    out.fields[1].name = "y";
    out.fields[1].offset = 4;
    out.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    out.fields[1].count = 1;

    out.fields[2].name = "z";
    out.fields[2].offset = 8;
    out.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    out.fields[2].count = 1;

    out.fields[3].name = "intensity";
    out.fields[3].offset = 12;
    out.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    out.fields[3].count = 1;

    out.data.reserve(msg->points.size() * kPointStep);

    for (const auto & point : msg->points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }

      const size_t offset = out.data.size();
      out.data.resize(offset + kPointStep);
      write_float(out.data, offset + 0, point.x);
      write_float(out.data, offset + 4, point.y);
      write_float(out.data, offset + 8, point.z);
      write_float(out.data, offset + 12, static_cast<float>(point.reflectivity));
    }

    out.width = static_cast<uint32_t>(out.data.size() / kPointStep);
    if (out.width == 0) {
      return;
    }
    out.row_step = out.point_step * out.width;
    pub_->publish(out);
  }

  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LivoxCustomToPc2Node>());
  rclcpp::shutdown();
  return 0;
}
