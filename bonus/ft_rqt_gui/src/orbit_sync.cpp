#include "orbit_sync.hpp"

#include <cmath>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.h>
#include <rviz/config.h>
#include <rviz/properties/property.h>
#include <rviz/view_controller.h>
#include <rviz/view_manager.h>
#include <rviz/visualization_manager.h>

namespace ft_rqt_gui {

// Qt 定时器使用实际时间，bag 暂停后也持续采样视角变化。
OrbitSync::OrbitSync(QWidget* parent) : rviz::Panel(parent) {
  auto* layout = new QVBoxLayout(this);
  label_ = new QLabel;
  label_->setStyleSheet("font-weight: bold;");
  layout->addWidget(label_);
  status_ = new QLabel("Waiting for a comparison session.");
  status_->setWordWrap(true);
  layout->addWidget(status_);
  connect(&timer_, &QTimer::timeout, this, &OrbitSync::tick);
  connect(this, &OrbitSync::incoming, this, &OrbitSync::receive, Qt::QueuedConnection);
  timer_.start(33);
}

// 先注销 ROS 回调，再由 Qt 回收面板和排队信号。
OrbitSync::~OrbitSync() {
  timer_.stop();
  subscriber_.shutdown();
  publisher_.shutdown();
}

// 每次会话分配独立话题；ROS 回调只送信号，不跨线程访问 RViz 属性。
void OrbitSync::load(const rviz::Config& config) {
  rviz::Panel::load(config);
  config.mapGetString("Channel", &channel_);
  config.mapGetString("Side", &side_);
  config.mapGetString("Label", &label_text_);
  label_->setText(label_text_);
  subscriber_.shutdown();
  publisher_.shutdown();
  sampled_ = false;
  have_remote_ = false;
  if (!channel_.startsWith('/') || (side_ != "left" && side_ != "right")) {
    status_->setText("Invalid Orbit sync configuration.");
    return;
  }
  publisher_ = node_.advertise<std_msgs::String>(channel_.toStdString(), 1, true);
  subscriber_ = node_.subscribe<std_msgs::String>(channel_.toStdString(), 1,
      [this](const std_msgs::String::ConstPtr& message) {
        Q_EMIT incoming(QString::fromStdString(message->data));
      });
}

// 配置文件只保存通道和左右身份，不将动态业务数据写入磁盘。
void OrbitSync::save(rviz::Config config) const {
  rviz::Panel::save(config);
  config.mapSetValue("Channel", channel_);
  config.mapSetValue("Side", side_);
  config.mapSetValue("Label", label_text_);
}

// Orbit 的角度、距离和焦点足以复现当前项目使用的旋转、缩放和平移。
bool OrbitSync::read(std::array<double, 6>& values) const {
  if (!vis_manager_) return false;
  auto* view = vis_manager_->getViewManager()->getCurrent();
  if (!view || view->getClassId() != "rviz/Orbit") return false;
  const char* scalar[] = {"Yaw", "Pitch", "Distance"};
  const char* axis[] = {"X", "Y", "Z"};
  for (int i = 0; i < 6; ++i) {
    const QVariant value = i < 3 ? view->subProp(scalar[i])->getValue() :
        view->subProp("Focal Point")->subProp(axis[i - 3])->getValue();
    bool valid = false;
    values[i] = value.toDouble(&valid);
    if (!valid || !std::isfinite(values[i])) return false;
  }
  return true;
}

// 接收更新后立即重采样 previous_，防止下一次 tick 将同一更新回发给对端。
void OrbitSync::write(const std::array<double, 6>& values) {
  auto* view = vis_manager_->getViewManager()->getCurrent();
  const char* scalar[] = {"Yaw", "Pitch", "Distance"};
  const char* axis[] = {"X", "Y", "Z"};
  for (int i = 0; i < 6; ++i) {
    auto* property = i < 3 ? view->subProp(scalar[i]) : view->subProp("Focal Point")->subProp(axis[i - 3]);
    property->setValue(values[i]);
  }
  sampled_ = read(previous_);
  vis_manager_->queueRender();
}

// 只广播本地变化；左窗提供初始视角，右窗避免用启动默认值覆盖对端。
void OrbitSync::tick() {
  if (!publisher_) return;
  if (vis_manager_) {
    auto* view = vis_manager_->getViewManager()->getCurrent();
    // 两侧必须围绕共同相机系运动，不能分别追随各自的 armor_0。
    if (view && view->getClassId() == "rviz/Orbit")
      view->subProp("Target Frame")->setValue("<Fixed Frame>");
  }
  std::array<double, 6> values;
  if (!read(values)) {
    sampled_ = false;
    status_->setText("Orbit sync paused: select the Orbit view to resume.");
    return;
  }
  // 从不支持的视图切回 Orbit 时，恢复暂停期间收到的最新对端视角。
  if (!sampled_ && have_remote_) {
    write(remote_);
    read(values);
  }
  bool changed = !sampled_ && side_ == "left";
  if (sampled_) {
    for (int i = 0; i < 6; ++i) changed = changed || std::abs(values[i] - previous_[i]) > 1e-6;
  }
  previous_ = values;
  sampled_ = true;
  status_->setText(publisher_.getNumSubscribers() > 1 ?
      "Linked Orbit: rotate, pan or zoom either window." : "Waiting for the other Orbit panel...");
  if (!changed) return;
  QJsonArray array;
  for (double value : values) array.append(value);
  QJsonObject object;
  object["source"] = side_;
  object["values"] = array;
  std_msgs::String message;
  message.data = QJsonDocument(object).toJson(QJsonDocument::Compact).toStdString();
  publisher_.publish(message);
}

// 过滤自身消息和不合法数据；全部属性操作均发生在 Qt 主线程。
void OrbitSync::receive(const QString& message) {
  const auto object = QJsonDocument::fromJson(message.toUtf8()).object();
  const QString peer = side_ == "left" ? "right" : "left";
  if (object["source"].toString() != peer) return;
  const auto array = object["values"].toArray();
  if (array.size() != 6) return;
  std::array<double, 6> values, current;
  for (int i = 0; i < 6; ++i) {
    if (!array[i].isDouble()) return;
    values[i] = array[i].toDouble();
    if (!std::isfinite(values[i])) return;
  }
  if (values[2] <= 0.001) return;
  remote_ = values;
  have_remote_ = true;
  if (!read(current)) return;
  write(values);
}

}  // namespace ft_rqt_gui

PLUGINLIB_EXPORT_CLASS(ft_rqt_gui::OrbitSync, rviz::Panel)
