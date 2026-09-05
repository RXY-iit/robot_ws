#include "so101_kinematic_handeye_calibration/capture_panel.hpp"
#include <QMetaObject>
#include <QVBoxLayout>
#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"
namespace so101_kinematic_handeye_calibration {
CapturePanel::CapturePanel(QWidget * parent)
: rviz_common::Panel(parent), setup_edit_(new QLineEdit(this)),
  label_edit_(new QLineEdit(this)),
  capture_button_(new QPushButton("Capture stable sample / 安定姿勢を保存", this)),
  status_label_(new QLabel("Waiting for ROS initialization...", this)) {
  setup_edit_->setPlaceholderText("Board setup ID (required; change if board moves)");
  label_edit_->setPlaceholderText("Pose label, e.g. front_roll_plus");
  status_label_->setWordWrap(true); capture_button_->setEnabled(false);
  auto * layout = new QVBoxLayout;
  layout->addWidget(new QLabel("SO101 kinematics + eye-in-hand calibration", this));
  layout->addWidget(new QLabel("Recorder is read-only; move only with teleoperate mode.", this));
  layout->addWidget(setup_edit_); layout->addWidget(label_edit_);
  layout->addWidget(capture_button_); layout->addWidget(status_label_); setLayout(layout);
  connect(capture_button_, &QPushButton::clicked, this, &CapturePanel::requestCapture);
}
void CapturePanel::onInitialize() {
  auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) { status_label_->setText("ERROR: RViz ROS node unavailable"); return; }
  auto node = abstraction->get_raw_node();
  request_pub_ = node->create_publisher<std_msgs::msg::String>("/so101_calibration/capture_request", 10);
  status_sub_ = node->create_subscription<std_msgs::msg::String>(
    "/so101_calibration/status", 10,
    [this](std_msgs::msg::String::ConstSharedPtr msg) { updateStatus(msg->data); });
  capture_button_->setEnabled(true); status_label_->setText("Waiting for recorder evidence...");
}
void CapturePanel::requestCapture() {
  const auto setup = setup_edit_->text().trimmed();
  if (setup.isEmpty()) { status_label_->setText("REJECTED: Board setup ID is required"); return; }
  if (!request_pub_ || request_pub_->get_subscription_count() == 0) {
    status_label_->setText("ERROR: capture node is not connected"); return;
  }
  std_msgs::msg::String msg;
  msg.data = (setup + "\t" + label_edit_->text().trimmed()).toStdString();
  request_pub_->publish(msg); status_label_->setText("Validating stable evidence...");
}
void CapturePanel::updateStatus(const std::string & text) {
  const auto q = QString::fromStdString(text);
  QMetaObject::invokeMethod(status_label_, [this, q]() { status_label_->setText(q); }, Qt::QueuedConnection);
}
}
PLUGINLIB_EXPORT_CLASS(so101_kinematic_handeye_calibration::CapturePanel, rviz_common::Panel)
