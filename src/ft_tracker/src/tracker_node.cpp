#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <dynamic_reconfigure/server.h>
#include <ft_detection/DetectionFrame.h>
#include <ft_msgs/Track.h>
#include <ft_tracker/TrackerConfig.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "ft_tracker/linear_tracker.hpp"

// ROS 适配层负责读取配置和转换消息，匀速滤波交给 LinearTracker。
class TrackerNode {
 public:
  // 建立跟踪器，检查话题、队列和显示参数，再连接输入输出。
  TrackerNode() : private_nh_("~") {
    ft_tracker::TrackerParams params;
    params.process_noise_density = required<double>("process_noise_density");
    params.measurement_noise_variance = required<double>("measurement_noise_variance");
    params.initial_position_variance = required<double>("initial_position_variance");
    params.initial_velocity_variance = required<double>("initial_velocity_variance");
    params.association_distance_m = required<double>("association_distance_m");
    params.lost_timeout_s = required<double>("lost_timeout_s");
    params.prediction_time_s = required<double>("prediction_time_s");
    params.match_color = required<bool>("match_color");
    params.match_size = required<bool>("match_size");
    tracker_.reset(new ft_tracker::LinearTracker(params));
    // 算法配置与 ROS 通信配置分开读取，话题都来自 YAML。
    const auto input = required<std::string>("detection_topic");
    const auto output = required<std::string>("track_topic");
    const auto marker = required<std::string>("marker_topic");
    const int input_queue = required<int>("detection_queue_size");
    const int output_queue = required<int>("track_queue_size");
    const int marker_queue = required<int>("marker_queue_size");
    marker_diameter_m_ = required<double>("marker_diameter_m");
    marker_lifetime_s_ = required<double>("marker_lifetime_s");
    if (input.empty() || output.empty() || marker.empty() || input_queue <= 0 || output_queue <= 0 ||
        marker_queue < 3 || !std::isfinite(marker_diameter_m_) || marker_diameter_m_ <= 0.0 ||
        !std::isfinite(marker_lifetime_s_) || marker_lifetime_s_ <= 0.0)
      throw std::invalid_argument("Tracker topics/queues and marker dimensions/lifetime are invalid");
    // 一份检测帧触发一次跟踪更新，并发布状态与 marker。
    track_publisher_ = nh_.advertise<ft_msgs::Track>(output, output_queue);
    marker_publisher_ = nh_.advertise<visualization_msgs::Marker>(marker, marker_queue);
    subscriber_ = nh_.subscribe(input, input_queue, &TrackerNode::receive, this);
    initializeTuning(params);
    ROS_INFO_STREAM("Tracker: " << input << " -> " << output << ", marker=" << marker);
  }

 private:
  // 必需参数没有隐藏默认值，错误配置在启动时报告。
  template <typename T> T required(const std::string& name) const {
    T value;
    if (!private_nh_.getParam(name, value))
      throw std::invalid_argument("Missing or wrongly typed parameter: " + private_nh_.resolveName(name));
    return value;
  }

  // 从启动配置设置动态默认值；在线更新只替换四项字段，不重新创建滤波器。
  void initializeTuning(const ft_tracker::TrackerParams& params) {
    active_tuning_.process_noise_density = params.process_noise_density;
    active_tuning_.measurement_noise_variance = params.measurement_noise_variance;
    active_tuning_.association_distance_m = params.association_distance_m;
    active_tuning_.prediction_time_s = params.prediction_time_s;
    auto clamped = active_tuning_;
    clamped.__clamp__();
    if (active_tuning_.__level__(clamped) != 0)
      throw std::invalid_argument("Tracker YAML tuning is outside the UI range");
    const ros::NodeHandle tuning_nh(private_nh_, "tuning");
    tuning_server_.reset(new TuningServer(tuning_mutex_, tuning_nh));
    tuning_server_->updateConfig(active_tuning_);
    tuning_server_->setConfigDefault(active_tuning_);
    tuning_server_->setCallback([this](ft_tracker::TrackerConfig& config, std::uint32_t) {
      try {
        tracker_->setTuning({config.process_noise_density, config.measurement_noise_variance,
                             config.association_distance_m, config.prediction_time_s});
        active_tuning_ = config;
      } catch (const std::invalid_argument& error) {
        config = active_tuning_;
        ROS_WARN_STREAM("Linear tuning rejected; previous configuration retained: " << error.what());
      }
    });
    ROS_INFO_STREAM("Dynamic linear tuning: " << tuning_nh.getNamespace());
  }

  // 显示当前点、未来点和连线；无轨迹时发送 DELETE 清除旧标记。
  void publishMarkers(const ft_msgs::Track& track) {
    if (track.header.frame_id.empty()) return;
    for (int id = 0; id < 3; ++id) {
      visualization_msgs::Marker marker;
      marker.header = track.header;
      marker.ns = "ft_tracker";
      // marker ID 区分三种显示对象，Track.id 才是跨帧轨迹身份。
      marker.id = id;
      marker.action = track.id == 0 ? visualization_msgs::Marker::DELETE : visualization_msgs::Marker::ADD;
      marker.type = id == 2 ? visualization_msgs::Marker::LINE_STRIP : visualization_msgs::Marker::SPHERE;
      marker.pose.orientation.w = 1.0;
      marker.lifetime = ros::Duration(marker_lifetime_s_);
      marker.color.a = track.tracking ? 1.0 : 0.45;
      marker.color.r = id == 0 ? 0.1 : 1.0;
      marker.color.g = id == 0 ? 1.0 : 0.5;
      marker.color.b = 0.1;
      // 连线使用 points，球使用 pose.position；纯预测时统一变淡。
      if (id == 2) {
        marker.scale.x = marker_diameter_m_ * 0.15;
        marker.points = {track.position, track.position_predict};
      } else {
        marker.scale.x = marker.scale.y = marker.scale.z = marker_diameter_m_;
        marker.pose.position = id == 0 ? track.position : track.position_predict;
      }
      marker_publisher_.publish(marker);
    }
  }

  // 接收检测帧，将观测送入滤波器，并沿用图像 header 发布 Track。
  void receive(const ft_detection::DetectionFrameConstPtr& message) {
    // 换坐标系时重置滤波；帧名丢失时还需清除旧系中的 marker。
    if (message->header.frame_id != frame_) {
      if (message->header.frame_id.empty() && !frame_.empty()) {
        ft_msgs::Track cleared;
        cleared.header = message->header;
        cleared.header.frame_id = frame_;
        publishMarkers(cleared);
      }
      tracker_->reset();
      frame_ = message->header.frame_id;
    }
    ft_msgs::Track output;
    output.header = message->header;
    try {
      if (frame_.empty()) {
        tracker_->reset();
        ROS_WARN_THROTTLE(1.0, "Detection frame has no camera frame_id; track cleared");
      } else {
        // 将 ROS 检测转换为位置与属性观测，帧内 ID 仅用于稳定排序。
        std::vector<ft_tracker::Observation> observations;
        observations.reserve(message->detections.size());
        for (const auto& detection : message->detections) {
          ft_tracker::Observation observation;
          observation.detection_id = detection.id;
          observation.is_red = detection.is_red;
          observation.is_large = detection.is_large_armor;
          const auto& p = detection.pose.position;
          observation.position = {p.x, p.y, p.z};
          observations.push_back(observation);
        }
        // 即使观测为空也按图像时间更新，推进预测与超时删除。
        const auto state = tracker_->update(message->header.stamp.toSec(), observations);
        output.id = state.id;
        output.tracking = state.tracking;
        output.position.x = state.position[0]; output.position.y = state.position[1]; output.position.z = state.position[2];
        output.velocity.x = state.velocity[0]; output.velocity.y = state.velocity[1]; output.velocity.z = state.velocity[2];
        output.position_predict.x = state.position_predict[0];
        output.position_predict.y = state.position_predict[1];
        output.position_predict.z = state.position_predict[2];
      }
    } catch (const std::exception& error) {
      // 滤波异常清理内部轨迹，本帧仍发布默认的无轨迹状态。
      tracker_->reset();
      ROS_ERROR_STREAM("Tracking failed; track cleared: " << error.what());
    }
    track_publisher_.publish(output);
    publishMarkers(output);
  }

  // 回调之间持续保存通信对象、滤波器和当前相机坐标系名称。
  ros::NodeHandle nh_, private_nh_;
  ros::Subscriber subscriber_;
  ros::Publisher track_publisher_, marker_publisher_;
  std::unique_ptr<ft_tracker::LinearTracker> tracker_;
  using TuningServer = dynamic_reconfigure::Server<ft_tracker::TrackerConfig>;
  ft_tracker::TrackerConfig active_tuning_;
  boost::recursive_mutex tuning_mutex_;
  std::unique_ptr<TuningServer> tuning_server_;
  std::string frame_;
  double marker_diameter_m_, marker_lifetime_s_;
};

// 注册跟踪节点并进入 ROS 回调循环。
int main(int argc, char** argv) {
  ros::init(argc, argv, "ft_tracker");
  try {
    TrackerNode tracker;
    ros::spin();
  } catch (const std::exception& error) {
    std::cerr << "ft_tracker startup failed: " << error.what() << std::endl;
    return 1;
  }
  return 0;
}
