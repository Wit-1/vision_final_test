#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include "ft_detector/debug_draw.hpp"
#include "ft_detector/traditional_detector.hpp"

class DetectorNode {
 public:
  DetectorNode() : private_nh_("~"), image_transport_(nh_) {
    const auto image_topic = requiredParam<std::string>("image_topic");
    const auto camera_info_topic = requiredParam<std::string>("camera_info_topic");
    const auto debug_image_topic = requiredParam<std::string>("debug_image_topic");
    const auto blue_mask_topic = requiredParam<std::string>("blue_mask_topic");
    const auto red_mask_topic = requiredParam<std::string>("red_mask_topic");
    debug_details_ = requiredParam<bool>("debug_details");
    detector_.reset(new ft_detector::TraditionalDetector(loadTraditionalParams()));
    const auto transport = requiredParam<std::string>("image_transport");
    const int image_queue = requiredParam<int>("image_queue_size");
    const int camera_info_queue = requiredParam<int>("camera_info_queue_size");
    const int debug_queue = requiredParam<int>("debug_queue_size");

    if (image_topic.empty() || camera_info_topic.empty() || debug_image_topic.empty() || blue_mask_topic.empty() ||
        red_mask_topic.empty() || transport.empty()) {
      throw std::invalid_argument("Image topics and transport must not be empty");
    }
    if (image_queue <= 0 || camera_info_queue <= 0 || debug_queue <= 0) {
      throw std::invalid_argument("Image, CameraInfo and debug queue sizes must be positive");
    }

    debug_publisher_ = image_transport_.advertise(debug_image_topic, debug_queue);
    blue_mask_publisher_ = image_transport_.advertise(blue_mask_topic, debug_queue);
    red_mask_publisher_ = image_transport_.advertise(red_mask_topic, debug_queue);
    camera_info_subscriber_ = nh_.subscribe(camera_info_topic, camera_info_queue,
                                           &DetectorNode::cameraInfoCallback, this);
    // compressed 插件先解码，imageCallback 接收到的是 sensor_msgs::Image。
    image_subscriber_ = image_transport_.subscribe(
        image_topic, image_queue, &DetectorNode::imageCallback, this,
        image_transport::TransportHints(transport, ros::TransportHints(), private_nh_));

    ROS_INFO_STREAM("Image input: " << image_topic << " transport=" << transport
                                   << ", CameraInfo: " << camera_info_topic
                                   << ", debug: " << debug_image_topic);
  }

 private:
  template <typename T>
  T requiredParam(const std::string& name) const {
    T value;
    if (!private_nh_.getParam(name, value)) {
      throw std::invalid_argument("Missing or wrongly typed parameter: " + private_nh_.resolveName(name));
    }
    return value;
  }

  ft_detector::TraditionalParams loadTraditionalParams() const {
    const auto hsv = [this](const std::string& name) {
      return ft_detector::HsvRange{
          requiredParam<int>(name + "/h_min"), requiredParam<int>(name + "/h_max"),
          requiredParam<int>(name + "/s_min"), requiredParam<int>(name + "/s_max"),
          requiredParam<int>(name + "/v_min"), requiredParam<int>(name + "/v_max")};
    };
    ft_detector::TraditionalParams params;
    params.blue = hsv("traditional/blue");
    params.red_low = hsv("traditional/red_low");
    params.red_high = hsv("traditional/red_high");
    params.min_contour_area = requiredParam<double>("traditional/min_contour_area");
    params.min_light_length = requiredParam<double>("traditional/min_light_length");
    params.min_light_aspect = requiredParam<double>("traditional/min_light_aspect");
    params.max_light_aspect = requiredParam<double>("traditional/max_light_aspect");
    params.min_light_fill = requiredParam<double>("traditional/min_light_fill");
    params.max_light_tilt_deg = requiredParam<double>("traditional/max_light_tilt_deg");
    params.min_pair_length_ratio = requiredParam<double>("traditional/min_pair_length_ratio");
    params.max_pair_angle_deg = requiredParam<double>("traditional/max_pair_angle_deg");
    params.max_pair_center_dy_ratio = requiredParam<double>("traditional/max_pair_center_dy_ratio");
    params.min_pair_spacing_ratio = requiredParam<double>("traditional/min_pair_spacing_ratio");
    params.max_pair_spacing_ratio = requiredParam<double>("traditional/max_pair_spacing_ratio");
    params.large_armor_min_spacing_ratio = requiredParam<double>("traditional/large_armor_min_spacing_ratio");
    params.intervening_light_max_offset_ratio = requiredParam<double>("traditional/intervening_light_max_offset_ratio");
    params.pair_ambiguity_margin = requiredParam<double>("traditional/pair_ambiguity_margin");
    params.border_margin = requiredParam<int>("traditional/border_margin");
    return params;
  }

  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& message) {
    const auto finite = [](double value) { return std::isfinite(value); };
    if (message->width == 0 || message->height == 0 || message->header.frame_id.empty() ||
        message->K[0] <= 0.0 || message->K[4] <= 0.0 ||
        !std::all_of(message->K.begin(), message->K.end(), finite) ||
        !std::all_of(message->D.begin(), message->D.end(), finite)) {
      camera_info_.reset();
      ROS_WARN_THROTTLE(1.0, "Invalid or uncalibrated CameraInfo; calibration cache cleared");
      return;
    }

    const bool changed = !camera_info_ || message->width != camera_info_->width ||
                         message->height != camera_info_->height ||
                         message->header.frame_id != camera_info_->header.frame_id ||
                         message->distortion_model != camera_info_->distortion_model ||
                         message->K != camera_info_->K || message->D != camera_info_->D;
    // 保留完整 CameraInfo；K、D、尺寸和 frame_id 供后续 PnP 使用。
    camera_info_ = message;
    if (changed) {
      ROS_INFO_STREAM("CameraInfo cached: " << message->width << "x" << message->height
                                            << " frame=" << message->header.frame_id
                                            << " fx=" << message->K[0] << " fy=" << message->K[4]
                                            << " cx=" << message->K[2] << " cy=" << message->K[5]
                                            << " distortion=" << message->distortion_model
                                            << " D_count=" << message->D.size());
    }
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& message) {
    try {
      // 编码相同时共享解码后的像素；压缩解码和发布输出仍有各自的数据分配。
      const auto shared_image = cv_bridge::toCvShare(message, sensor_msgs::image_encodings::BGR8);
      if (shared_image->image.empty()) {
        ROS_WARN_THROTTLE(1.0, "Empty image received; frame skipped");
        return;
      }

      const bool calibration_ready = camera_info_ && camera_info_->width == message->width &&
                                     camera_info_->height == message->height &&
                                     camera_info_->header.frame_id == message->header.frame_id;
      if (!calibration_ready) {
        ROS_WARN_THROTTLE(1.0, "No matching CameraInfo; 2D detection continues while calibration is unavailable");
      }

      const auto result = detector_->detect(shared_image->image);
      const auto annotated = ft_detector::drawDetections(shared_image->image, result, debug_details_);
      debug_publisher_.publish(cv_bridge::CvImage(
          message->header, sensor_msgs::image_encodings::BGR8, annotated).toImageMsg());
      blue_mask_publisher_.publish(cv_bridge::CvImage(
          message->header, sensor_msgs::image_encodings::MONO8, result.blue_mask).toImageMsg());
      red_mask_publisher_.publish(cv_bridge::CvImage(
          message->header, sensor_msgs::image_encodings::MONO8, result.red_mask).toImageMsg());
      ++frame_count_;
      ROS_INFO_STREAM_THROTTLE(1.0, "Processed image " << frame_count_ << ": " << message->width
                                                     << "x" << message->height
                                                     << " lights=" << result.lights.size()
                                                     << " armors=" << result.armors.size()
                                                     << " frame=" << message->header.frame_id
                                                     << " calibration="
                                                     << (calibration_ready ? "ready" : "unavailable"));
    } catch (const cv_bridge::Exception& error) {
      ROS_ERROR_STREAM("Image conversion failed: " << error.what());
    } catch (const cv::Exception& error) {
      ROS_ERROR_STREAM("OpenCV image processing failed: " << error.what());
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  image_transport::ImageTransport image_transport_;
  image_transport::Publisher debug_publisher_;
  image_transport::Publisher blue_mask_publisher_, red_mask_publisher_;
  std::unique_ptr<ft_detector::TraditionalDetector> detector_;
  bool debug_details_ = false;
  image_transport::Subscriber image_subscriber_;
  ros::Subscriber camera_info_subscriber_;
  sensor_msgs::CameraInfoConstPtr camera_info_;
  std::uint64_t frame_count_ = 0;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "ft_detector");
  try {
    DetectorNode detector;
    ros::spin();
  } catch (const std::exception& error) {
    // 构造失败后 NodeHandle 可能已关闭 ROS 日志，标准错误仍可显示启动原因。
    std::cerr << "ft_detector startup failed: " << error.what() << std::endl;
    return 1;
  }
  return 0;
}
