#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <ft_detection/DetectionFrame.h>
#include <ft_msgs/Target.h>
#include <ft_msgs/Track.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include "ft_selector/target_selector.hpp"

namespace {

using visualization_msgs::Marker;

geometry_msgs::Point toPoint(const Eigen::Vector3d& position) {
  geometry_msgs::Point point;
  point.x = position.x();
  point.y = position.y();
  point.z = position.z();
  return point;
}

bool sameFrame(const std_msgs::Header& a, const std_msgs::Header& b) {
  return a.stamp == b.stamp && a.frame_id == b.frame_id;
}

class SelectorNode {
 public:
  SelectorNode() : private_nh_("~"), params_(readTrackerParams()), tracker_(params_) {
    horizon_s_ = required<double>("linear/prediction_time_s");
    min_facing_ = required<double>("min_facing_cosine");
    min_horizontal_normal_ = required<double>("min_horizontal_normal");
    marker_size_m_ = required<double>("marker_size_m");
    marker_lifetime_s_ = required<double>("marker_lifetime_s");
    const auto detection = required<std::string>("detection_topic");
    const auto track = required<std::string>("linear/track_topic");
    const auto target = required<std::string>("target_topic");
    const auto markers = required<std::string>("marker_topic");
    const int queue = required<int>("queue_size");
    if (detection.empty() || track.empty() || target.empty() || markers.empty() || queue <= 0 ||
        !std::isfinite(horizon_s_) || horizon_s_ < 0.0 ||
        !(min_facing_ >= 0.0 && min_facing_ < 1.0) ||
        !(min_horizontal_normal_ > 0.0 && min_horizontal_normal_ <= 1.0) ||
        !std::isfinite(marker_size_m_) || marker_size_m_ <= 0.0 ||
        !std::isfinite(marker_lifetime_s_) || marker_lifetime_s_ <= 0.0)
      throw std::invalid_argument("Invalid selector topics, horizon, visibility or marker parameters");

    target_pub_ = nh_.advertise<ft_msgs::Target>(target, queue);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(markers, queue);
    detection_sub_ = nh_.subscribe(detection, queue, &SelectorNode::receiveDetections, this);
    track_sub_ = nh_.subscribe(track, queue, &SelectorNode::receiveTrack, this);
    ROS_INFO_STREAM("Selector: " << detection << " -> " << target << ", radius="
                    << params_.circle.radius_m << " m, plates=" << params_.circle.plate_count
                    << ", T=" << horizon_s_ << " s");
  }

 private:
  template <typename T>
  T required(const std::string& name) const {
    T value;
    if (!private_nh_.getParam(name, value))
      throw std::invalid_argument("Missing or wrongly typed parameter: " + private_nh_.resolveName(name));
    return value;
  }

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
    return params;
  }

  std::vector<ft_selector::ArmorObservation> observations(const ft_detection::DetectionFrame& frame) const {
    std::vector<ft_selector::ArmorObservation> result;
    result.reserve(frame.detections.size());
    for (const auto& detection : frame.detections) {
      const auto& p = detection.pose.position;
      const Eigen::Vector3d position(p.x, p.y, p.z);
      const auto& q = detection.pose.orientation;
      Eigen::Quaterniond rotation(q.w, q.x, q.y, q.z);
      const double norm = rotation.norm();
      if (!position.allFinite() || position.z() <= 0.0 || !std::isfinite(norm) || norm < 1e-6) continue;
      rotation.coeffs() /= norm;
      // The PnP plate's local +Z points inward. Project its outward normal onto camera X-Z.
      const Eigen::Vector3d outward = -(rotation * Eigen::Vector3d::UnitZ());
      if (std::hypot(outward.x(), outward.z()) < min_horizontal_normal_ ||
          outward.x() * p.x + outward.z() * p.z >= 0.0) continue;
      result.push_back({position, std::atan2(outward.x(), outward.z()), detection.is_red != 0});
    }
    return result;
  }

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

  Marker comparisonMarker(const std_msgs::Header& header) const {
    auto item = marker(header, 3, Marker::LINE_LIST, false);
    if (!latest_target_.valid || !latest_track_ || !latest_track_->tracking || latest_track_->id == 0 ||
        !sameFrame(latest_target_.header, latest_track_->header)) return item;

    const auto& linear = latest_track_->position_predict;
    if (!std::isfinite(linear.x) || !std::isfinite(linear.y) || !std::isfinite(linear.z)) return item;
    item.action = Marker::ADD;
    item.scale.x = marker_size_m_ * 0.12;
    item.color.r = item.color.g = item.color.b = 0.8;
    item.color.a = 0.8;
    item.points = {linear, latest_target_.position_predict};
    return item;
  }

  void publishMarkers(const std_msgs::Header& header, const ft_selector::SelectedTarget& target) {
    if (header.frame_id.empty()) return;
    visualization_msgs::MarkerArray display;
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
    display.markers.push_back(comparisonMarker(header));
    marker_pub_.publish(display);
  }

  void receiveTrack(const ft_msgs::TrackConstPtr& message) {
    latest_track_ = message;
    if (sameFrame(latest_target_.header, message->header) && !message->header.frame_id.empty()) {
      visualization_msgs::MarkerArray display;
      display.markers.push_back(comparisonMarker(message->header));
      marker_pub_.publish(display);
    }
  }

  void receiveDetections(const ft_detection::DetectionFrameConstPtr& message) {
    const auto& header = message->header;
    const auto& previous = latest_target_.header;
    if (!previous.frame_id.empty() && sameFrame(previous, header)) return;
    if (header.frame_id != previous.frame_id || header.stamp < previous.stamp) {
      tracker_.reset();
      if (latest_track_ && !sameFrame(header, latest_track_->header)) latest_track_.reset();
    }

    ft_selector::SelectedTarget target;
    try {
      if (!header.frame_id.empty()) {
        const auto vehicle = tracker_.update(header.stamp.toSec(), observations(*message));
        target = ft_selector::selectTarget(vehicle, params_.circle, horizon_s_, min_facing_);
      }
    } catch (const std::exception& error) {
      tracker_.reset();
      ROS_ERROR_STREAM_THROTTLE(1.0, "Rotating tracker cleared: " << error.what());
    }

    auto marker_header = header;
    if (marker_header.frame_id.empty()) marker_header.frame_id = previous.frame_id;
    latest_target_.header = header;
    latest_target_.valid = target.valid;
    latest_target_.target_id = target.id;
    latest_target_.position = toPoint(target.current);
    latest_target_.position_predict = toPoint(target.predicted);
    target_pub_.publish(latest_target_);
    publishMarkers(marker_header, target);
  }

  ros::NodeHandle nh_, private_nh_;
  ft_selector::RotatingParams params_;
  ft_selector::RotatingTracker tracker_;
  ros::Subscriber detection_sub_, track_sub_;
  ros::Publisher target_pub_, marker_pub_;
  ft_msgs::TrackConstPtr latest_track_;
  ft_msgs::Target latest_target_;
  double horizon_s_, min_facing_, min_horizontal_normal_, marker_size_m_, marker_lifetime_s_;
};

}  // namespace

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
