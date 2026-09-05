#ifndef CAMERA_PAN_TILT_CALIBRATION__CAPTURE_PANEL_HPP_
#define CAMERA_PAN_TILT_CALIBRATION__CAPTURE_PANEL_HPP_

#include <memory>

#include <QLabel>
#include <QLineEdit>
#include <QPushButton>

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

namespace camera_pan_tilt_calibration
{

class CapturePanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit CapturePanel(QWidget * parent = nullptr);
  void onInitialize() override;

private Q_SLOTS:
  void requestCapture();

private:
  void updateStatus(const std::string & text);
  void updateAxisValue(QLabel * label, const char * axis, float value);

  QLineEdit * label_edit_;
  QPushButton * capture_button_;
  QLabel * pan_value_label_;
  QLabel * tilt_value_label_;
  QLabel * status_label_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr request_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr pan_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr tilt_sub_;
};

}  // namespace camera_pan_tilt_calibration

#endif  // CAMERA_PAN_TILT_CALIBRATION__CAPTURE_PANEL_HPP_
