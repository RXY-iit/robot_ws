#include "touch_anything/touch_panel.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>

#include <QButtonGroup>
#include <QHBoxLayout>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMetaObject>
#include <QMouseEvent>
#include <QPixmap>
#include <QResizeEvent>
#include <QSizePolicy>
#include <QVBoxLayout>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"

namespace touch_anything
{

ClickableImage::ClickableImage(QWidget * parent) : QLabel(parent)
{
  setAlignment(Qt::AlignCenter);
  // Do not impose a large dock minimum size.  With a saved narrow RViz window,
  // that can squeeze Ogre's render window to (nearly) zero during dock layout
  // restoration and trigger a crash in RenderSystem_GL::resize().
  setMinimumSize(160, 120);
  setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Expanding);
  setStyleSheet("background:#20242a; border:1px solid #667085;");
  setText("No frozen D435 frame / D435静止画像なし");
}

void ClickableImage::setSourceImage(const QImage & image)
{
  source_ = image.copy();
  refreshPixmap();
}

void ClickableImage::refreshPixmap()
{
  if (source_.isNull()) {
    return;
  }
  const QPixmap scaled = QPixmap::fromImage(source_).scaled(
    size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
  image_rect_ = QRect(
    (width() - scaled.width()) / 2, (height() - scaled.height()) / 2,
    scaled.width(), scaled.height());
  setPixmap(scaled);
}

void ClickableImage::resizeEvent(QResizeEvent * event)
{
  QLabel::resizeEvent(event);
  refreshPixmap();
}

void ClickableImage::mousePressEvent(QMouseEvent * event)
{
  if (source_.isNull() || !image_rect_.contains(event->pos())) {
    return;
  }
  const double nx = static_cast<double>(event->pos().x() - image_rect_.x()) /
    static_cast<double>(image_rect_.width());
  const double ny = static_cast<double>(event->pos().y() - image_rect_.y()) /
    static_cast<double>(image_rect_.height());
  const int u = std::min(source_.width() - 1, std::max(0, static_cast<int>(nx * source_.width())));
  const int v = std::min(source_.height() - 1, std::max(0, static_cast<int>(ny * source_.height())));
  Q_EMIT imageClicked(u, v, event->button() != Qt::RightButton);
}

TouchPanel::TouchPanel(QWidget * parent)
: rviz_common::Panel(parent),
  image_(new ClickableImage(this)),
  mode_label_(new QLabel("Dashboard mode: waiting", this)),
  phase_label_(new QLabel("Phase / フェーズ: WAITING", this)),
  progress_label_(new QLabel("Mask / マスク: --  |  Too close / 過近: 15%  |  FAR advance: --", this)),
  state_label_(new QLabel("Touch pipeline: waiting for ROS", this)),
  help_label_(new QLabel(
      "Dashboardで code2sim&real → Touch Anything を選択してください。\n"
      "左クリック: 正例、右クリック: 負例。画像はFreeze時点で固定されます。", this)),
  freeze_button_(new QPushButton("Freeze D435 frame / D435画像を固定", this)),
  clear_button_(new QPushButton("Clear prompts / 点を消去", this)),
  confirm_button_(new QPushButton("Confirm 3D preview / 3D候補を確認", this)),
  plan_button_(new QPushButton("Plan pre-touch far / 遠方事前接触を計画", this)),
  execute_button_(new QPushButton("Execute FAR only / FAR位置だけへ移動", this)),
  acquire_arm_button_(new QPushButton("Acquire arm-camera target / 腕カメラ対象を取得", this)),
  confirm_arm_button_(new QPushButton(
      "Confirm & start auto approach / 確認して自動接近開始", this)),
  plan_approach_button_(new QPushButton("Plan next visual step / 次の視覚ステップを計画", this)),
  execute_approach_button_(new QPushButton("Execute one step / 1ステップ実行", this)),
  exit_button_(new QPushButton("Exit Touch Mode / Touch Mode終了", this)),
  positive_button_(new QRadioButton("Positive / 対象", this)),
  negative_button_(new QRadioButton("Negative / 除外", this))
{
  help_label_->setWordWrap(true);
  state_label_->setWordWrap(true);
  phase_label_->setStyleSheet(
    "font-weight:600; padding:3px 6px; background:#eef4ff; border:1px solid #9bb8e8;");
  progress_label_->setStyleSheet(
    "font-weight:600; padding:3px 6px; background:#f7f9fc; border:1px solid #cbd5e1;");
  positive_button_->setChecked(true);
  freeze_button_->setEnabled(false);
  clear_button_->setEnabled(false);
  confirm_button_->setEnabled(false);
  plan_button_->setEnabled(false);
  execute_button_->setEnabled(false);
  acquire_arm_button_->setEnabled(false);
  confirm_arm_button_->setEnabled(false);
  plan_approach_button_->setEnabled(false);
  execute_approach_button_->setEnabled(false);
  exit_button_->setEnabled(false);

  auto * prompt_row = new QHBoxLayout;
  prompt_row->addWidget(positive_button_);
  prompt_row->addWidget(negative_button_);
  auto * action_row = new QHBoxLayout;
  action_row->addWidget(clear_button_);
  action_row->addWidget(confirm_button_);
  auto * reach_row = new QHBoxLayout;
  reach_row->addWidget(plan_button_);
  reach_row->addWidget(execute_button_);
  auto * handoff_row = new QHBoxLayout;
  handoff_row->addWidget(acquire_arm_button_);
  handoff_row->addWidget(confirm_arm_button_);
  // Manual one-step commands remain implemented for diagnostics, but the V1
  // operator workflow is one explicit object confirmation followed by the
  // bounded automatic stop-observe loop.
  plan_approach_button_->setVisible(false);
  execute_approach_button_->setVisible(false);
  auto * auto_approach_label = new QLabel(
    "After FAR: centre the SAM mask while advancing 40 mm to NEAR. At 15% mask "
    "occupancy retreat 5 mm. In NEAR, lock X, align image centre then the fixed tip; PRESS advances "
    "+X by 25 mm only after alignment. / "
    "FAR後はSAMマスクを中央へ寄せながら40 mm進んでNEARへ入り、占有率15%以上では5 mm後退します。"
    "NEARではXを固定し、画面中央と爪先をY/Z整列後のみPRESSで+Xへ25 mm進みます。", this);
  auto_approach_label->setWordWrap(true);
  auto * layout = new QVBoxLayout;
  layout->addWidget(new QLabel("Touch Anything — human point prompt", this));
  layout->addWidget(mode_label_);
  layout->addWidget(phase_label_);
  layout->addWidget(progress_label_);
  layout->addWidget(state_label_);
  layout->addWidget(image_, 1);
  layout->addLayout(prompt_row);
  layout->addWidget(freeze_button_);
  layout->addLayout(action_row);
  layout->addLayout(reach_row);
  layout->addLayout(handoff_row);
  layout->addWidget(auto_approach_label);
  layout->addWidget(exit_button_);
  layout->addWidget(help_label_);
  setLayout(layout);

  connect(freeze_button_, &QPushButton::clicked, this, &TouchPanel::requestFreeze);
  connect(clear_button_, &QPushButton::clicked, this, &TouchPanel::clearPrompts);
  connect(confirm_button_, &QPushButton::clicked, this, &TouchPanel::confirmPreview);
  connect(plan_button_, &QPushButton::clicked, this, &TouchPanel::planPreTouchFar);
  connect(execute_button_, &QPushButton::clicked, this, &TouchPanel::executePreTouchFar);
  connect(acquire_arm_button_, &QPushButton::clicked, this, &TouchPanel::acquireArmTarget);
  connect(confirm_arm_button_, &QPushButton::clicked, this, &TouchPanel::confirmArmTarget);
  connect(plan_approach_button_, &QPushButton::clicked, this, &TouchPanel::planApproachStep);
  connect(execute_approach_button_, &QPushButton::clicked, this, &TouchPanel::executeApproachStep);
  connect(exit_button_, &QPushButton::clicked, this, &TouchPanel::exitTouchMode);
  connect(image_, &ClickableImage::imageClicked, this, &TouchPanel::sendPoint);
}

void TouchPanel::onInitialize()
{
  auto node_abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!node_abstraction) {
    state_label_->setText("ERROR: RViz ROS node unavailable");
    return;
  }
  auto node = node_abstraction->get_raw_node();
  command_pub_ = node->create_publisher<std_msgs::msg::String>("/touch_anything/command", 10);
  state_sub_ = node->create_subscription<std_msgs::msg::String>(
    "/touch_anything/state", 10,
    [this](std_msgs::msg::String::ConstSharedPtr msg) {updateState(msg->data);});
  arm_state_sub_ = node->create_subscription<std_msgs::msg::String>(
    "/arm/dashboard_state", 10,
    [this](std_msgs::msg::String::ConstSharedPtr msg) {updateArmState(msg->data);});
  image_sub_ = node->create_subscription<sensor_msgs::msg::Image>(
    "/touch_anything/overlay_image", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {updateImage(msg);});
  state_label_->setText("Waiting for /touch_anything/state ...");
}

void TouchPanel::publishCommand(const std::string & command, const std::string & extra)
{
  if (!command_pub_) {
    return;
  }
  QJsonObject object;
  object["command"] = QString::fromStdString(command);
  object["source"] = "rviz";
  object["request_id"] = QString("rviz-%1").arg(
    std::chrono::steady_clock::now().time_since_epoch().count());
  if (!extra.empty()) {
    const auto parsed = QJsonDocument::fromJson(QByteArray::fromStdString(extra));
    if (parsed.isObject()) {
      // Keep the object alive while iterating. Calling parsed.object() in both
      // iterator expressions creates different temporaries (undefined behavior).
      const QJsonObject extra_object = parsed.object();
      for (auto it = extra_object.constBegin(); it != extra_object.constEnd(); ++it) {
        object[it.key()] = it.value();
      }
    }
  }
  std_msgs::msg::String msg;
  msg.data = QJsonDocument(object).toJson(QJsonDocument::Compact).toStdString();
  command_pub_->publish(msg);
}

void TouchPanel::requestFreeze() {publishCommand("freeze");}
void TouchPanel::clearPrompts() {publishCommand("clear_prompts");}
void TouchPanel::confirmPreview() {publishCommand("confirm_preview");}
void TouchPanel::planPreTouchFar() {publishCommand("plan_pre_touch_far");}
void TouchPanel::executePreTouchFar() {publishCommand("execute_pre_touch_far");}
void TouchPanel::acquireArmTarget() {publishCommand("acquire_arm_target");}
void TouchPanel::confirmArmTarget() {publishCommand("confirm_arm_target");}
void TouchPanel::planApproachStep() {publishCommand("plan_approach_step");}
void TouchPanel::executeApproachStep() {publishCommand("execute_approach_step");}
void TouchPanel::exitTouchMode() {publishCommand("exit");}

void TouchPanel::sendPoint(int u, int v, bool mouse_positive)
{
  if (!active_) {
    return;
  }
  const bool positive = mouse_positive ? positive_button_->isChecked() : false;
  QJsonObject extra;
  extra["u"] = u;
  extra["v"] = v;
  extra["label"] = positive ? 1 : 0;
  publishCommand("add_prompt", QJsonDocument(extra).toJson(QJsonDocument::Compact).toStdString());
}

void TouchPanel::updateImage(sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  QImage converted;
  const int width = static_cast<int>(msg->width);
  const int height = static_cast<int>(msg->height);
  if (msg->encoding == "rgb8" && msg->step >= msg->width * 3) {
    converted = QImage(msg->data.data(), width, height, static_cast<int>(msg->step), QImage::Format_RGB888).copy();
  } else if (msg->encoding == "bgr8" && msg->step >= msg->width * 3) {
    converted = QImage(msg->data.data(), width, height, static_cast<int>(msg->step), QImage::Format_BGR888).copy();
  } else if (msg->encoding == "rgba8" && msg->step >= msg->width * 4) {
    converted = QImage(msg->data.data(), width, height, static_cast<int>(msg->step), QImage::Format_RGBA8888).copy();
  }
  if (!converted.isNull()) {
    QMetaObject::invokeMethod(image_, [this, converted]() {image_->setSourceImage(converted);}, Qt::QueuedConnection);
  }
}

void TouchPanel::updateState(const std::string & text)
{
  const auto document = QJsonDocument::fromJson(QByteArray::fromStdString(text));
  if (!document.isObject()) {
    return;
  }
  const auto object = document.object();
  const bool active = object.value("active").toBool(false);
  const QString state = object.value("state").toString("UNKNOWN");
  const QString message = object.value("message").toString();
  const auto reach_plan = object.value("reach_plan").toObject();
  const bool execution_enabled = reach_plan.value("execution_enabled").toBool(false);
  const auto approach_plan = object.value("approach_plan").toObject();
  const QString step_kind = approach_plan.value("step_kind").toString();
  const bool approach_execution_enabled =
    approach_plan.value("execution_enabled").toBool(false);
  QMetaObject::invokeMethod(
    this,
    [this, active, state, message, step_kind, approach_plan,
      execution_enabled, approach_execution_enabled]() {
    active_ = active;
    QString phase = "TARGET / 対象選択";
    if (!active_) {
      phase = "INACTIVE / 停止";
    } else if (state == "AUTO_APPROACH_CARTESIAN_CORRECTING") {
      phase = "CARTESIAN CORRECTION / 直交残差修正";
    } else if (step_kind == "ALIGN_FIXED_TIP_YZ") {
      phase = "TIP ALIGN / 爪先整列";
    } else if (step_kind == "MASK_TOO_CLOSE_RETREAT_SMALL") {
      phase = "TOO CLOSE: RETREAT / 過近後退";
    } else if (step_kind == "ALIGNED_PRESS_FORWARD_WITH_FIXED_TIP_YZ") {
      phase = "PRESS / 押下";
    } else if (step_kind == "COARSE_CENTER_AND_FORWARD_TO_NEAR") {
      phase = "FAR → NEAR 35 mm / 中央寄せ接近";
    } else if (state.contains("PRETOUCH") || state.contains("D435_TARGET_REACQUIRE")) {
      phase = "FAR PREPARE / 遠方姿勢準備";
    } else if (state.contains("ARM_TARGET") || state.contains("ARM_SAM")) {
      phase = "ARM TARGET / 腕カメラ対象";
    } else if (state.contains("TOUCH_SUCCEEDED")) {
      phase = "COMPLETE / 完了";
    }
    phase_label_->setText(QString("Phase / フェーズ: %1").arg(phase));
    if (!approach_plan.isEmpty()) {
      const double mask = approach_plan.value("arm_mask_fraction").toDouble(-1.0);
      const double threshold =
        approach_plan.value("too_close_mask_fraction_threshold").toDouble(0.15);
      const double remaining = approach_plan.value("remaining_before_m").toDouble(0.0);
      const double advanced = approach_plan.value("advanced_from_far_m").toDouble(0.0);
      const double near_distance = approach_plan.value("near_advance_distance_m").toDouble(0.035);
      const bool too_close = approach_plan.value("too_close").toBool(false);
      const bool near_reached = approach_plan.value("near_reached").toBool(false);
      const QString gate = too_close ? "TOO CLOSE" : (near_reached ? "NEAR" : "FAR");
      progress_label_->setText(
        QString("Mask / マスク: %1%  |  Too close / 過近: %2%  |  FAR advance: %3/%4 mm  |  X to D435: %5 mm  |  %6")
          .arg(mask >= 0.0 ? QString::number(mask * 100.0, 'f', 1) : "--")
          .arg(threshold >= 0.0 ? QString::number(threshold * 100.0, 'f', 1) : "--")
          .arg(QString::number(advanced * 1000.0, 'f', 1))
          .arg(QString::number(near_distance * 1000.0, 'f', 1))
          .arg(QString::number(remaining * 1000.0, 'f', 1))
          .arg(gate));
    } else {
      progress_label_->setText("Mask / マスク: --  |  Too close / 過近: 15%  |  FAR advance: --");
    }
    state_label_->setText(QString("%1 — %2").arg(state, message));
    freeze_button_->setEnabled(active_ && arm_code_mode_);
    clear_button_->setEnabled(active_);
    confirm_button_->setEnabled(active_ && state == "TARGET_READY");
    plan_button_->setEnabled(
      active_ && arm_code_mode_ &&
      (state == "PREVIEW_CONFIRMED" || state == "PRETOUCH_GATEWAY_REJECTED" ||
      state == "PRETOUCH_UNREACHABLE"));
    execute_button_->setEnabled(
      active_ && arm_code_mode_ && state == "D435_TARGET_REACQUIRED" && execution_enabled);
    acquire_arm_button_->setEnabled(
      active_ && arm_code_mode_ &&
      (state == "PRETOUCH_FAR_REACHED" || state == "ARM_REACQUIRE_REQUIRED" ||
      state == "ARM_FRAME_READY" || state == "ARM_TARGET_REJECTED" ||
      state == "APPROACH_GATEWAY_REJECTED" || state == "APPROACH_BLOCKED" ||
      state == "APPROACH_UNREACHABLE" || state == "APPROACH_EXECUTION_FAILED" ||
      state == "AUTO_TARGET_LOST" || state == "AUTO_APPROACH_TIMEOUT" ||
      state == "AUTO_APPROACH_STEP_LIMIT"));
    confirm_arm_button_->setEnabled(
      active_ && arm_code_mode_ && state == "ARM_TARGET_READY");
    plan_approach_button_->setEnabled(
      active_ && arm_code_mode_ &&
      (state == "ARM_TARGET_CONFIRMED" || state == "APPROACH_GATEWAY_REJECTED"));
    execute_approach_button_->setEnabled(
      active_ && arm_code_mode_ && state == "APPROACH_STEP_PLANNED" &&
      approach_execution_enabled);
    exit_button_->setEnabled(active_);
    }, Qt::QueuedConnection);
}

void TouchPanel::updateArmState(const std::string & text)
{
  const auto document = QJsonDocument::fromJson(QByteArray::fromStdString(text));
  if (!document.isObject()) {
    return;
  }
  const QString mode = document.object().value("control_mode").toString("UNKNOWN");
  const bool code_mode = mode == "code2real&sim";
  QMetaObject::invokeMethod(this, [this, mode, code_mode]() {
    arm_code_mode_ = code_mode;
    mode_label_->setText(QString("Arm control mode: %1").arg(mode));
    freeze_button_->setEnabled(active_ && arm_code_mode_);
  }, Qt::QueuedConnection);
}

}  // namespace touch_anything

PLUGINLIB_EXPORT_CLASS(touch_anything::TouchPanel, rviz_common::Panel)
