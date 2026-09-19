#ifndef TOUCH_ANYTHING__TOUCH_PANEL_HPP_
#define TOUCH_ANYTHING__TOUCH_PANEL_HPP_

#include <memory>
#include <string>

#include <QImage>
#include <QLabel>
#include <QPushButton>
#include <QRadioButton>

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

namespace touch_anything
{

class ClickableImage : public QLabel
{
  Q_OBJECT

public:
  explicit ClickableImage(QWidget * parent = nullptr);
  void setSourceImage(const QImage & image);

Q_SIGNALS:
  void imageClicked(int u, int v, bool positive);

protected:
  void mousePressEvent(QMouseEvent * event) override;
  void resizeEvent(QResizeEvent * event) override;

private:
  void refreshPixmap();
  QImage source_;
  QRect image_rect_;
};

class TouchPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit TouchPanel(QWidget * parent = nullptr);
  void onInitialize() override;

private Q_SLOTS:
  void requestFreeze();
  void clearPrompts();
  void confirmPreview();
  void planPreTouchFar();
  void executePreTouchFar();
  void acquireArmTarget();
  void confirmArmTarget();
  void confirmPress();
  void planApproachStep();
  void executeApproachStep();
  void exitTouchMode();
  void sendPoint(int u, int v, bool mouse_positive);

private:
  void publishCommand(const std::string & command, const std::string & extra = "");
  void updateImage(sensor_msgs::msg::Image::ConstSharedPtr msg);
  void updateState(const std::string & text);
  void updateArmState(const std::string & text);

  ClickableImage * image_;
  QLabel * mode_label_;
  QLabel * phase_label_;
  QLabel * progress_label_;
  QLabel * state_label_;
  QLabel * help_label_;
  QPushButton * freeze_button_;
  QPushButton * clear_button_;
  QPushButton * confirm_button_;
  QPushButton * plan_button_;
  QPushButton * execute_button_;
  QPushButton * acquire_arm_button_;
  QPushButton * confirm_arm_button_;
  QPushButton * confirm_press_button_;
  QPushButton * plan_approach_button_;
  QPushButton * execute_approach_button_;
  QPushButton * exit_button_;
  QRadioButton * positive_button_;
  QRadioButton * negative_button_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr arm_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  bool active_{false};
  bool arm_code_mode_{false};
};

}  // namespace touch_anything

#endif  // TOUCH_ANYTHING__TOUCH_PANEL_HPP_
