#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <dynamic_reconfigure/server.h>
#include <ft_detection/DetectionFrame.h>
#include <ft_msgs/Target.h>
#include <ft_msgs/PredictionDebug.h>
#include <ft_selector/SelectorConfig.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/MarkerArray.h>

#include "ft_selector/target_selector.hpp"
#include "ft_selector/observation_noise.hpp"

namespace {

using visualization_msgs::Marker;

// 将算法使用的 Eigen 向量转换为 ROS 点消息。
geometry_msgs::Point toPoint(const Eigen::Vector3d& position) {
  geometry_msgs::Point point;
  point.x = position.x();
  point.y = position.y();
  point.z = position.z();
  return point;
}

// 同时间戳且同坐标系才视为重复输入帧。
bool sameFrame(const std_msgs::Header& a, const std_msgs::Header& b) {
  return a.stamp == b.stamp && a.frame_id == b.frame_id;
}

// 选板节点直接使用检测位姿，独立于单板匀速跟踪链路。
class SelectorNode {
 public:
  // 加载旋转模型、与线性链路相同的 T、显示参数和话题配置。
  SelectorNode() : private_nh_("~"), params_(readTrackerParams()), tracker_(params_) {
    horizon_s_ = required<double>("linear/prediction_time_s");
    min_facing_ = required<double>("min_facing_cosine");
    min_horizontal_normal_ = required<double>("min_horizontal_normal");
    marker_size_m_ = required<double>("marker_size_m");
    marker_lifetime_s_ = required<double>("marker_lifetime_s");
    const auto detection = required<std::string>("detection_topic");
    const auto target = required<std::string>("target_topic");
    const auto markers = required<std::string>("marker_topic");
    const auto prediction = required<std::string>("prediction_topic");
    const int queue = required<int>("queue_size");
    if (detection.empty() || target.empty() || markers.empty() || prediction.empty() || queue <= 0 ||
        !std::isfinite(horizon_s_) || horizon_s_ < 0.0 ||
        !(min_facing_ >= 0.0 && min_facing_ < 1.0) ||
        !(min_horizontal_normal_ > 0.0 && min_horizontal_normal_ <= 1.0) ||
        !std::isfinite(marker_size_m_) || marker_size_m_ <= 0.0 ||
        !std::isfinite(marker_lifetime_s_) || marker_lifetime_s_ <= 0.0)
      throw std::invalid_argument("Invalid selector topics, horizon, visibility or marker parameters");

    // 检测仍驱动每帧更新；CameraInfo 只补充观测质量所需的标定。
    target_pub_ = nh_.advertise<ft_msgs::Target>(target, queue);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(markers, queue);
    prediction_pub_ = nh_.advertise<ft_msgs::PredictionDebug>(prediction, queue);
    readDimensions();
    initializeObservationNoise();
    detection_sub_ = nh_.subscribe(detection, queue, &SelectorNode::receiveDetections, this);
    initializeTuning();
    ROS_INFO_STREAM("Selector: " << detection << " -> " << target << ", radius="
                    << params_.circle.radius_m << " m, plates=" << params_.circle.plate_count
                    << ", T=" << horizon_s_ << " s");
  }

 private:
  // 强制读取显式配置，缺字段或类型错误时终止启动。
  template <typename T>
  T required(const std::string& name) const {
    T value;
    if (!private_nh_.getParam(name, value))
      throw std::invalid_argument("Missing or wrongly typed parameter: " + private_nh_.resolveName(name));
    return value;
  }

  // 将半径、槽位、关联门限和 EKF 噪声组装为算法参数。
  ft_selector::RotatingParams readTrackerParams() const {
    ft_selector::RotatingParams params;
    params.circle.radius_m = required<double>("radius_m");
    params.circle.plate_count = required<int>("plate_count");
    params.position_gate_m = required<double>("association_position_m");
    params.yaw_gate_rad = required<double>("association_yaw_rad");
    params.lost_timeout_s = required<double>("lost_timeout_s");
    params.linear_noise_density = required<double>("linear_noise_density");
    params.angular_noise_density = required<double>("angular_noise_density");
    params.position_variance = required<double>("position_variance");
    params.yaw_variance = required<double>("yaw_variance");
    params.initial_velocity_variance = required<double>("initial_velocity_variance");
    params.initial_omega_variance = required<double>("initial_omega_variance");
    params.radius_min_m = required<double>("geometry/radius_min_m");
    params.radius_max_m = required<double>("geometry/radius_max_m");
    params.max_height_difference_m = required<double>("geometry/max_height_difference_m");
    params.initial_geometry_variance = required<double>("geometry/initial_variance");
    params.geometry_noise_density = required<double>("geometry/noise_density");
    params.geometry_min_phase_span_rad = required<double>("geometry/min_phase_span_rad");
    params.geometry_min_observations = required<int>("geometry/min_observations");
    params.estimate_pair_offsets = required<bool>("geometry/estimate_pair_offsets");
    return params;
  }

  // 尺寸直接读取同侧检测节点已加载的 PnP 配置，避免维护第二套物理尺寸。
  void readDimensions() {
    const ros::NodeHandle detector(ros::names::parentNamespace(private_nh_.getNamespace()) + "/ft_detector");
    for (int large = 0; large < 2; ++large) {
      const auto prefix = std::string("pnp/") + (large ? "large/" : "small/");
      if (!detector.getParam(prefix + "width_m", widths_[large]) ||
          !detector.getParam(prefix + "height_m", heights_[large]) ||
          !std::isfinite(widths_[large]) || !std::isfinite(heights_[large]) ||
          widths_[large] <= 0.0 || heights_[large] <= 0.0) {
        widths_[large] = heights_[large] = 0.0;
        ROS_WARN("PnP dimensions unavailable; prediction centers remain available without armor corners");
      }
    }
  }

  // 从同侧检测配置获取模式与相机入口；旧 Baseline 无需增加新接口。
  void initializeObservationNoise() {
    const auto prefix = std::string("observation_noise/");
    noise_params_.error_reference_px = required<double>(prefix + "error_reference_px");
    noise_params_.distance_reference_m = required<double>(prefix + "distance_reference_m");
    noise_params_.max_multiplier = required<double>(prefix + "max_multiplier");
    const auto strength = [&](const std::string& mode) {
      return ft_selector::NoiseStrength{required<double>(prefix + mode + "/confidence_reference"),
          required<double>(prefix + mode + "/position_strength"), required<double>(prefix + mode + "/yaw_strength")};
    };
    noise_params_.nn = strength("nn");
    noise_params_.tradition = strength("tradition");
    ft_selector::validateObservationNoise(noise_params_);
    const ros::NodeHandle detector(ros::names::parentNamespace(private_nh_.getNamespace()) + "/ft_detector");
    if (!private_nh_.getParam("mode", detector_mode_)) detector.getParam("mode", detector_mode_);
    std::string topic;
    private_nh_.getParam("camera_info_topic", topic);
    if (topic.empty()) detector.getParam("camera_info_topic", topic);
    if (!topic.empty()) camera_sub_ = nh_.subscribe(topic, 1, &SelectorNode::receiveCameraInfo, this);
    if (topic.empty() || (detector_mode_ != "nn" && detector_mode_ != "tradition"))
      ROS_WARN("Observation quality input is incomplete; fixed R remains available");
    ROS_INFO_STREAM("Observation noise: quality weighted, detector mode=" << detector_mode_ << ", CameraInfo=" << topic);
  }

  // 无效标定主动清空缓存，后续检测继续按固定 R 跟踪，不丢弃原有观测。
  void receiveCameraInfo(const sensor_msgs::CameraInfoConstPtr& info) {
    camera_info_.reset();
    if (info->header.frame_id.empty() || info->width == 0 || info->height == 0 ||
        info->width > static_cast<unsigned>(std::numeric_limits<int>::max()) ||
        info->height > static_cast<unsigned>(std::numeric_limits<int>::max()) ||
        info->binning_x > 1 || info->binning_y > 1 || info->roi.do_rectify ||
        info->roi.x_offset || info->roi.y_offset ||
        (info->roi.width && info->roi.width != info->width) ||
        (info->roi.height && info->roi.height != info->height)) return;
    ft_selector::QualityCalibration camera;
    std::copy(info->K.begin(), info->K.end(), camera.matrix.val);
    camera.distortion.assign(info->D.begin(), info->D.end());
    camera.image_size = cv::Size(info->width, info->height);
    camera.distortion_model = info->distortion_model;
    if (!ft_selector::validQualityCalibration(camera)) return;
    camera_info_ = info;
    quality_camera_ = std::move(camera);
  }

  // 旋转调参使用启动 YAML 的默认值，T 仍从原有 linear 配置读取。
  void initializeTuning() {
    active_tuning_.linear_noise_density = params_.linear_noise_density;
    active_tuning_.angular_noise_density = params_.angular_noise_density;
    active_tuning_.position_variance = params_.position_variance;
    active_tuning_.yaw_variance = params_.yaw_variance;
    active_tuning_.association_position_m = params_.position_gate_m;
    active_tuning_.association_yaw_rad = params_.yaw_gate_rad;
    active_tuning_.prediction_time_s = horizon_s_;
    auto clamped = active_tuning_;
    clamped.__clamp__();
    if (active_tuning_.__level__(clamped) != 0)
      throw std::invalid_argument("Selector YAML tuning is outside the UI range");
    const ros::NodeHandle tuning_nh(private_nh_, "tuning");
    tuning_server_.reset(new TuningServer(tuning_mutex_, tuning_nh));
    tuning_server_->updateConfig(active_tuning_);
    tuning_server_->setConfigDefault(active_tuning_);
    tuning_server_->setCallback([this](ft_selector::SelectorConfig& config, std::uint32_t) {
      try {
        if (!std::isfinite(config.prediction_time_s) || config.prediction_time_s < 0.0)
          throw std::invalid_argument("Prediction horizon must be finite and nonnegative");
        tracker_.setTuning({config.linear_noise_density, config.angular_noise_density,
                           config.position_variance, config.yaw_variance,
                           config.association_position_m, config.association_yaw_rad});
        horizon_s_ = config.prediction_time_s;
        active_tuning_ = config;
      } catch (const std::invalid_argument& error) {
        config = active_tuning_;
        ROS_WARN_STREAM("Rotating tuning rejected; previous configuration retained: " << error.what());
      }
    });
    ROS_INFO_STREAM("Dynamic rotating tuning: " << tuning_nh.getNamespace());
  }

  // 从检测 Pose 提取板中心和水平法向角，形成旋转滤波的观测。
  std::vector<ft_selector::ArmorObservation> observations(const ft_detection::DetectionFrame& frame) const {
    std::vector<ft_selector::ArmorObservation> result;
    result.reserve(frame.detections.size());
    for (std::size_t index = 0; index < frame.detections.size(); ++index) {
      const auto& detection = frame.detections[index];
      const auto& p = detection.pose.position;
      const Eigen::Vector3d position(p.x, p.y, p.z);
      const auto& q = detection.pose.orientation;
      Eigen::Quaterniond rotation(q.w, q.x, q.y, q.z);
      // 拒绝非法位置与退化四元数，使用前归一化旋转。
      const double norm = rotation.norm();
      if (!position.allFinite() || position.z() <= 0.0 || !std::isfinite(norm) || norm < 1e-6) continue;
      rotation.coeffs() /= norm;
      // The PnP plate's local +Z points inward. Project its outward normal onto camera X-Z.
      // 局部 +Z 指向板背后，取反得到朝外法向，再检查水平分量和朝向。
      const Eigen::Vector3d outward = -(rotation * Eigen::Vector3d::UnitZ());
      if (std::hypot(outward.x(), outward.z()) < min_horizontal_normal_ ||
          outward.x() * p.x + outward.z() * p.z >= 0.0) continue;
      ft_selector::ArmorObservation observation{position, std::atan2(outward.x(), outward.z()), detection.is_red != 0,
                                               detection.is_large_armor != 0, static_cast<int>(index)};
      // 质量信息完整才加权；不改变前面的位姿、法向门控或每帧一次校正。
      if (camera_info_ && camera_info_->header.frame_id == frame.header.frame_id) {
        std::array<cv::Point2d,4> corners;
        for (std::size_t i = 0; i < corners.size(); ++i)
          corners[i] = {detection.corners[i].x, detection.corners[i].y};
        const int large = detection.is_large_armor != 0;
        const auto quality = ft_selector::measureObservationQuality(position, rotation, corners,
            widths_[large], heights_[large], detection.confidence, quality_camera_);
        observation.variance_multipliers = ft_selector::observationVarianceMultipliers(quality, detector_mode_, noise_params_);
      }
      result.push_back(observation);
    }
    return result;
  }

  // 把同一次预测的时间、姿态与几何打包；调参后也不会把旧预测误配到新的 T。
  void publishPrediction(const ft_detection::DetectionFrame& frame,
                         const ft_selector::VehicleEstimate& vehicle,
                         const ft_selector::SelectedTarget& target) {
    if (vehicle.id != orientation_id_) {
      orientation_id_ = vehicle.id;
      have_orientation_ = false;
    }
    if (vehicle.observed && vehicle.detection_index >= 0 &&
        static_cast<std::size_t>(vehicle.detection_index) < frame.detections.size()) {
      const auto& pose = frame.detections[vehicle.detection_index].pose;
      Eigen::Quaterniond rotation(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
      rotation.normalize();
      const Eigen::Vector3d outward = -(rotation * Eigen::Vector3d::UnitZ());
      const double observed_phase = std::atan2(outward.x(), outward.z());
      // 先去掉观测水平相位，保留当前有效 PnP 的倾角和滚转作为共同安装姿态。
      mounting_rotation_ = Eigen::AngleAxisd(3.14159265358979323846 - observed_phase,
                                            Eigen::Vector3d::UnitY()) * rotation.toRotationMatrix();
      have_orientation_ = true;
    }
    ft_msgs::PredictionDebug output;
    output.header = frame.header;
    output.prediction_time_s = horizon_s_;
    output.prediction_stamp = frame.header.stamp + ros::Duration(horizon_s_);
    output.target_id = target.id;
    output.geometry_ready = vehicle.geometry_ready;
    output.plate_count = params_.circle.plate_count;
    output.current_slot = target.current_slot; output.predicted_slot = target.predicted_slot;
    output.detection_index = vehicle.detection_index;
    output.is_large = vehicle.is_large;
    output.width_m = widths_[vehicle.is_large]; output.height_m = heights_[vehicle.is_large];
    output.valid = target.valid && have_orientation_ && output.width_m > 0.0 && output.height_m > 0.0;
    output.current.position = toPoint(target.current);
    output.predicted.position = toPoint(target.predicted);
    // 局部 +Z 指向板背面；正面朝相机时 phase=pi，水平旋转为单位阵。
    const auto set_orientation = [this](double phase, geometry_msgs::Quaternion& orientation) {
      Eigen::Quaterniond rotation(Eigen::AngleAxisd(phase - 3.14159265358979323846,
                                                   Eigen::Vector3d::UnitY()) * mounting_rotation_);
      rotation.normalize();
      orientation.x = rotation.x(); orientation.y = rotation.y();
      orientation.z = rotation.z(); orientation.w = rotation.w();
    };
    if (have_orientation_) {
      set_orientation(target.current_phase, output.current.orientation);
      set_orientation(target.predicted_phase, output.predicted.orientation);
    }
    output.center = toPoint(vehicle.center);
    output.velocity.x = vehicle.velocity.x(); output.velocity.y = vehicle.velocity.y(); output.velocity.z = vehicle.velocity.z();
    output.phase = vehicle.phase; output.angular_velocity = vehicle.omega;
    output.radius_m = vehicle.geometry.radius_m;
    output.delta_radius_m = vehicle.geometry.delta_radius_m;
    output.delta_height_m = vehicle.geometry.delta_height_m;
    output.geometry_stddev.x = vehicle.geometry_stddev.x();
    output.geometry_stddev.y = vehicle.geometry_stddev.y();
    output.geometry_stddev.z = vehicle.geometry_stddev.z();
    prediction_pub_.publish(output);
  }

  // 设置所有基础标记共用的 header、命名空间、生命周期和增删动作。
  Marker marker(const std_msgs::Header& header, int id, int type, bool show) const {
    Marker item;
    item.header = header;
    item.ns = "ft_selector";
    item.id = id;
    item.type = type;
    item.action = show ? Marker::ADD : Marker::DELETE;
    item.pose.orientation.w = 1.0;
    item.lifetime = ros::Duration(marker_lifetime_s_);
    return item;
  }

  // 发布车体中心、当前板和未来板，短时纯预测降低透明度。
  void publishMarkers(const std_msgs::Header& header, const ft_selector::SelectedTarget& target) {
    if (header.frame_id.empty()) return;
    visualization_msgs::MarkerArray display;
    // ID 0/1/2 是显示对象编号，与车体 target_id 的身份含义不同。
    auto center = marker(header, 0, Marker::CUBE, target.id != 0);
    auto current = marker(header, 1, Marker::SPHERE, target.id != 0);
    auto future = marker(header, 2, Marker::SPHERE, target.has_prediction);
    center.pose.position = toPoint(target.center);
    current.pose.position = toPoint(target.current);
    future.pose.position = toPoint(target.predicted);
    center.color.g = center.color.b = 0.9;
    current.color.r = current.color.b = 0.9;
    future.color.r = 0.2;
    future.color.g = 0.4;
    future.color.b = 1.0;
    for (auto* item : {&center, &current, &future}) {
      item->scale.x = item->scale.y = item->scale.z = marker_size_m_;
      item->color.a = target.valid ? 1.0 : 0.4;
      display.markers.push_back(*item);
    }
    marker_pub_.publish(display);
  }

  // 每帧先维护时间与坐标系，再更新旋转状态、选板并发布。
  void receiveDetections(const ft_detection::DetectionFrameConstPtr& message) {
    const auto& header = message->header;
    const auto previous = last_header_;
    // 重复帧跳过，时间回退或换坐标系则结束旧车体轨迹。
    if (!previous.frame_id.empty() && sameFrame(previous, header)) return;
    if (header.frame_id != previous.frame_id || header.stamp < previous.stamp) {
      tracker_.reset();
      if (!previous.frame_id.empty()) camera_info_.reset();
    }

    // 空检测仍进入 update 推进失联状态；异常时输出无效目标。
    ft_selector::SelectedTarget target;
    ft_selector::VehicleEstimate vehicle;
    try {
      if (!header.frame_id.empty()) {
        vehicle = tracker_.update(header.stamp.toSec(), observations(*message));
        target = ft_selector::selectTarget(vehicle, params_.circle, horizon_s_, min_facing_);
      }
    } catch (const std::exception& error) {
      tracker_.reset();
      ROS_ERROR_STREAM_THROTTLE(1.0, "Rotating tracker cleared: " << error.what());
    }

    // 消息始终保留输入 header；缺帧名时用旧帧名删除旧显示对象。
    auto marker_header = header;
    if (marker_header.frame_id.empty()) marker_header.frame_id = previous.frame_id;
    ft_msgs::Target output;
    output.header = header;
    output.valid = target.valid;
    output.target_id = target.id;
    output.position = toPoint(target.current);
    output.position_predict = toPoint(target.predicted);
    target_pub_.publish(output);
    publishPrediction(*message, vehicle, target);
    last_header_ = header;
    publishMarkers(marker_header, target);
  }

  // 保存通信对象、旋转滤波器，以及用于重复/回退检查的上一帧 header。
  ros::NodeHandle nh_, private_nh_;
  ft_selector::RotatingParams params_;
  ft_selector::RotatingTracker tracker_;
  using TuningServer = dynamic_reconfigure::Server<ft_selector::SelectorConfig>;
  ft_selector::SelectorConfig active_tuning_;
  boost::recursive_mutex tuning_mutex_;
  std::unique_ptr<TuningServer> tuning_server_;
  ros::Subscriber detection_sub_, camera_sub_;
  sensor_msgs::CameraInfoConstPtr camera_info_;
  ft_selector::QualityCalibration quality_camera_;
  ft_selector::ObservationNoiseParams noise_params_;
  std::string detector_mode_;
  ros::Publisher target_pub_, marker_pub_, prediction_pub_;
  std_msgs::Header last_header_;
  double horizon_s_, min_facing_, min_horizontal_normal_, marker_size_m_, marker_lifetime_s_;
  double widths_[2]{0.0, 0.0}, heights_[2]{0.0, 0.0};
  std::uint32_t orientation_id_ = 0;
  bool have_orientation_ = false;
  Eigen::Matrix3d mounting_rotation_ = Eigen::Matrix3d::Identity();
};

}  // namespace

// 注册选板节点并处理检测回调，启动错误通过标准错误输出。
int main(int argc, char** argv) {
  ros::init(argc, argv, "ft_selector");
  try {
    SelectorNode selector;
    ros::spin();
  } catch (const std::exception& error) {
    std::cerr << "ft_selector startup failed: " << error.what() << std::endl;
    return 1;
  }
  return 0;
}
