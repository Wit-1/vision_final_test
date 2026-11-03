#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <ft_detection/DetectionFrame.h>
#include <ft_msgs/Track.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "ft_tracker/linear_tracker.hpp"

class TrackerNode {
 public:
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
    track_publisher_ = nh_.advertise<ft_msgs::Track>(output, output_queue);
    marker_publisher_ = nh_.advertise<visualization_msgs::Marker>(marker, marker_queue);
    subscriber_ = nh_.subscribe(input, input_queue, &TrackerNode::receive, this);
    ROS_INFO_STREAM("Tracker: " << input << " -> " << output << ", marker=" << marker);
  }

 private:
  template <typename T> T required(const std::string& name) const {
    T value;
    if (!private_nh_.getParam(name, value))
      throw std::invalid_argument("Missing or wrongly typed parameter: " + private_nh_.resolveName(name));
    return value;
  }

  void publishMarkers(const ft_msgs::Track& track) {
    if (track.header.frame_id.empty()) return;
    for (int id = 0; id < 3; ++id) {
      visualization_msgs::Marker marker;
      marker.header = track.header;
      marker.ns = "ft_tracker";
      marker.id = id;
      marker.action = track.id == 0 ? visualization_msgs::Marker::DELETE : visualization_msgs::Marker::ADD;
      marker.type = id == 2 ? visualization_msgs::Marker::LINE_STRIP : visualization_msgs::Marker::SPHERE;
      marker.pose.orientation.w = 1.0;
      marker.lifetime = ros::Duration(marker_lifetime_s_);
      marker.color.a = track.tracking ? 1.0 : 0.45;
      marker.color.r = id == 0 ? 0.1 : 1.0;
      marker.color.g = id == 0 ? 1.0 : 0.5;
      marker.color.b = 0.1;
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

  void receive(const ft_detection::DetectionFrameConstPtr& message) {
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
      tracker_->reset();
      ROS_ERROR_STREAM("Tracking failed; track cleared: " << error.what());
    }
    track_publisher_.publish(output);
    publishMarkers(output);
  }

  ros::NodeHandle nh_, private_nh_;
  ros::Subscriber subscriber_;
  ros::Publisher track_publisher_, marker_publisher_;
  std::unique_ptr<ft_tracker::LinearTracker> tracker_;
  std::string frame_;
  double marker_diameter_m_, marker_lifetime_s_;
};

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
