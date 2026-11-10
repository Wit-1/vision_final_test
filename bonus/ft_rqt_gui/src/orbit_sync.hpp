#pragma once

#include <array>
#include <QLabel>
#include <QString>
#include <QTimer>
#include <ros/ros.h>
#include <rviz/panel.h>
#include <std_msgs/String.h>

namespace ft_rqt_gui {

// 仅同步可视化相机，不读取或修改任何检测、跟踪和选板状态。
class OrbitSync : public rviz::Panel {
  Q_OBJECT
 public:
  explicit OrbitSync(QWidget* parent = nullptr);
  ~OrbitSync() override;
  void load(const rviz::Config& config) override;
  void save(rviz::Config config) const override;

 Q_SIGNALS:
  void incoming(QString message);

 private Q_SLOTS:
  void tick();
  void receive(const QString& message);

 private:
  bool read(std::array<double, 6>& values) const;
  void write(const std::array<double, 6>& values);
  QLabel* label_;
  QLabel* status_;
  QTimer timer_;
  ros::NodeHandle node_;
  ros::Publisher publisher_;
  ros::Subscriber subscriber_;
  QString channel_, side_, label_text_;
  std::array<double, 6> previous_{};
  std::array<double, 6> remote_{};
  bool sampled_ = false;
  bool have_remote_ = false;
};

}  // namespace ft_rqt_gui
