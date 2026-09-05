#ifndef SO101_KINEMATIC_HANDEYE_CALIBRATION__CAPTURE_PANEL_HPP_
#define SO101_KINEMATIC_HANDEYE_CALIBRATION__CAPTURE_PANEL_HPP_
#include <memory>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "std_msgs/msg/string.hpp"
namespace so101_kinematic_handeye_calibration {
class CapturePanel : public rviz_common::Panel {
  Q_OBJECT
public:
  explicit CapturePanel(QWidget * parent = nullptr);
  void onInitialize() override;
private Q_SLOTS:
  void requestCapture();
private:
  void updateStatus(const std::string & text);
  QLineEdit * setup_edit_;
  QLineEdit * label_edit_;
  QPushButton * capture_button_;
  QLabel * status_label_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr request_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
};
}
#endif
