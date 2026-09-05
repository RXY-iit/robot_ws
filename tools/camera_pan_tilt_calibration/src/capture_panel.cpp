#include "camera_pan_tilt_calibration/capture_panel.hpp"

#include <QMetaObject>
#include <QVBoxLayout>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"

namespace camera_pan_tilt_calibration
{

CapturePanel::CapturePanel(QWidget * parent)
: rviz_common::Panel(parent),
  label_edit_(new QLineEdit(this)),
  capture_button_(new QPushButton("Capture stable sample / 安定姿勢を保存", this)),
  pan_value_label_(new QLabel("Pan raw feedback / パン: --", this)),
  tilt_value_label_(new QLabel("Tilt raw feedback / チルト: --", this)),
  status_label_(new QLabel("Waiting for ROS initialization...", this))
{
  label_edit_->setPlaceholderText("Optional label, e.g. middle_center");
  status_label_->setWordWrap(true);
  capture_button_->setEnabled(false);

  auto * layout = new QVBoxLayout;
  layout->addWidget(new QLabel("Dual-camera pan/tilt calibration", this));
  layout->addWidget(pan_value_label_);
  layout->addWidget(tilt_value_label_);
  layout->addWidget(label_edit_);
  layout->addWidget(capture_button_);
  layout->addWidget(status_label_);
  setLayout(layout);

  connect(capture_button_, &QPushButton::clicked, this, &CapturePanel::requestCapture);
}

void CapturePanel::onInitialize()
{
  auto node_abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!node_abstraction) {
    status_label_->setText("ERROR: RViz ROS node is unavailable");
    return;
  }
  auto node = node_abstraction->get_raw_node();
  request_pub_ = node->create_publisher<std_msgs::msg::String>(
    "/camera_calibration/capture_request", 10);
  status_sub_ = node->create_subscription<std_msgs::msg::String>(
    "/camera_calibration/status", 10,
    [this](std_msgs::msg::String::ConstSharedPtr msg) {updateStatus(msg->data);});
  pan_sub_ = node->create_subscription<std_msgs::msg::Float32>(
    "/chokudomotor/angle", 10,
    [this](std_msgs::msg::Float32::ConstSharedPtr msg) {
      updateAxisValue(pan_value_label_, "Pan / パン", msg->data);
    });
  tilt_sub_ = node->create_subscription<std_msgs::msg::Float32>(
    "/cameraswingmotor/angle", 10,
    [this](std_msgs::msg::Float32::ConstSharedPtr msg) {
      updateAxisValue(tilt_value_label_, "Tilt / チルト", msg->data);
    });
  capture_button_->setEnabled(true);
  status_label_->setText("Waiting for camera, motor feedback and ChArUco capture node...");
}

void CapturePanel::requestCapture()
{
  if (!request_pub_) {
    status_label_->setText("ERROR: capture publisher is not initialized");
    return;
  }
  const auto subscriber_count = request_pub_->get_subscription_count();
  if (subscriber_count == 0) {
    const QString error =
      "ERROR: capture node is not connected (capture_request subscribers=0)";
    status_label_->setText(error);
    RCLCPP_ERROR(
      rclcpp::get_logger("camera_pan_tilt_calibration.capture_panel"), "%s",
      error.toStdString().c_str());
    return;
  }
  std_msgs::msg::String request;
  request.data = label_edit_->text().trimmed().toStdString();
  request_pub_->publish(request);
  const QString status = QString(
    "Capture request published (subscribers=%1); validating evidence...")
    .arg(subscriber_count);
  status_label_->setText(status);
}

void CapturePanel::updateStatus(const std::string & text)
{
  const QString status = QString::fromStdString(text);
  QMetaObject::invokeMethod(
    status_label_, [this, status]() {status_label_->setText(status);}, Qt::QueuedConnection);
}

void CapturePanel::updateAxisValue(QLabel * label, const char * axis, float value)
{
  const QString text = QString("%1 raw feedback: %2")
    .arg(axis)
    .arg(static_cast<double>(value), 0, 'f', 2);
  QMetaObject::invokeMethod(
    label, [label, text]() {label->setText(text);}, Qt::QueuedConnection);
}

}  // namespace camera_pan_tilt_calibration

PLUGINLIB_EXPORT_CLASS(camera_pan_tilt_calibration::CapturePanel, rviz_common::Panel)
