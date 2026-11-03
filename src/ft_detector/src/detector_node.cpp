#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>
#include <tf2_ros/transform_broadcaster.h>

#include "ft_detector/debug_draw.hpp"
#include "ft_detector/detection_output.hpp"
#include "ft_detector/pnp_solver.hpp"
#include "ft_detector/pose_history.hpp"
#include "ft_detector/traditional_detector.hpp"
#include "ft_detector/tuning_parameters.hpp"

class DetectorNode {
 public:
  DetectorNode() : private_nh_("~"), image_transport_(nh_) {
    const auto image_topic = requiredParam<std::string>("image_topic");
    const auto camera_info_topic = requiredParam<std::string>("camera_info_topic");
    const auto debug_image_topic = requiredParam<std::string>("debug_image_topic");
    const auto blue_mask_topic = requiredParam<std::string>("blue_mask_topic");
    const auto red_mask_topic = requiredParam<std::string>("red_mask_topic");
    const auto detection_topic = requiredParam<std::string>("detection_topic");
    debug_details_ = requiredParam<bool>("debug_details");
    traditional_params_ = loadTraditionalParams();
    detector_.reset(new ft_detector::TraditionalDetector(traditional_params_));
    pnp_enabled_ = requiredParam<bool>("pnp/enabled");
    publish_tf_ = requiredParam<bool>("pnp/publish_tf");
    child_frame_prefix_ = requiredParam<std::string>("pnp/child_frame_prefix");
    if (child_frame_prefix_.empty() ||
        !std::all_of(child_frame_prefix_.begin(), child_frame_prefix_.end(),
                     [](unsigned char c) { return std::isalnum(c) || c == '_'; })) {
      throw std::invalid_argument("PnP child frame prefix must contain only letters, digits or underscores");
    }
    const auto pnp_params = loadPnpParams();
    pnp_solver_.reset(new ft_detector::PnpSolver(pnp_params));
    pose_continuity_enabled_ = requiredParam<bool>("pnp/continuity/enabled");
    pose_history_.reset(new ft_detector::PoseHistory(
        requiredParam<double>("pnp/continuity/max_gap_s"),
        requiredParam<double>("pnp/continuity/max_corner_shift_ratio")));
    if (pnp_enabled_ && (!pnp_params.small.confirmed || !pnp_params.large.confirmed)) {
      if (pnp_params.allow_provisional_dimensions)
        ROS_WARN("Provisional endpoint dimensions are enabled; pose/TF metric scale remains unverified");
      else
        ROS_WARN("Unconfirmed endpoint dimensions: affected armor classes will not publish a pose or TF");
    }
    const auto transport = requiredParam<std::string>("image_transport");
    const int image_queue = requiredParam<int>("image_queue_size");
    const int camera_info_queue = requiredParam<int>("camera_info_queue_size");
    const int debug_queue = requiredParam<int>("debug_queue_size");
    const int detection_queue = requiredParam<int>("detection_queue_size");

    if (image_topic.empty() || camera_info_topic.empty() || debug_image_topic.empty() || blue_mask_topic.empty() ||
        red_mask_topic.empty() || detection_topic.empty() || transport.empty()) {
      throw std::invalid_argument("Input/output topics and image transport must not be empty");
    }
    if (image_queue <= 0 || camera_info_queue <= 0 || debug_queue <= 0 || detection_queue <= 0) {
      throw std::invalid_argument("Image, CameraInfo, debug and detection queue sizes must be positive");
    }

    debug_publisher_ = image_transport_.advertise(debug_image_topic, debug_queue);
    blue_mask_publisher_ = image_transport_.advertise(blue_mask_topic, debug_queue);
    red_mask_publisher_ = image_transport_.advertise(red_mask_topic, debug_queue);
    detection_publisher_ = nh_.advertise<ft_detection::DetectionFrame>(detection_topic, detection_queue);
    camera_info_subscriber_ = nh_.subscribe(camera_info_topic, camera_info_queue,
                                           &DetectorNode::cameraInfoCallback, this);
    // compressed 插件先解码，imageCallback 接收到的是 sensor_msgs::Image。
    image_subscriber_ = image_transport_.subscribe(
        image_topic, image_queue, &DetectorNode::imageCallback, this,
        image_transport::TransportHints(transport, ros::TransportHints(), private_nh_));

    initializeTuning();

    ROS_INFO_STREAM("Image input: " << image_topic << " transport=" << transport
                                   << ", CameraInfo: " << camera_info_topic
                                   << ", debug: " << debug_image_topic << ", detections: " << detection_topic);
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
          requiredParam<int>("tuning/" + name + "_h_min"), requiredParam<int>("tuning/" + name + "_h_max"),
          requiredParam<int>("tuning/" + name + "_s_min"), requiredParam<int>("tuning/" + name + "_s_max"),
          requiredParam<int>("tuning/" + name + "_v_min"), requiredParam<int>("tuning/" + name + "_v_max")};
    };
    ft_detector::TraditionalParams params;
    params.blue = hsv("blue");
    params.red_low = hsv("red_low");
    params.red_high = hsv("red_high");
    params.min_contour_area = requiredParam<double>("tuning/min_contour_area");
    params.min_light_length = requiredParam<double>("traditional/min_light_length");
    params.min_light_aspect = requiredParam<double>("traditional/min_light_aspect");
    params.max_light_aspect = requiredParam<double>("traditional/max_light_aspect");
    params.min_light_fill = requiredParam<double>("traditional/min_light_fill");
    params.max_light_tilt_deg = requiredParam<double>("traditional/max_light_tilt_deg");
    params.min_pair_length_ratio = requiredParam<double>("traditional/min_pair_length_ratio");
    params.max_pair_angle_deg = requiredParam<double>("tuning/max_pair_angle_deg");
    params.max_pair_center_dy_ratio = requiredParam<double>("traditional/max_pair_center_dy_ratio");
    params.min_pair_spacing_ratio = requiredParam<double>("traditional/min_pair_spacing_ratio");
    params.max_pair_spacing_ratio = requiredParam<double>("tuning/max_pair_spacing_ratio");
    params.large_armor_min_spacing_ratio = requiredParam<double>("traditional/large_armor_min_spacing_ratio");
    params.intervening_light_max_offset_ratio = requiredParam<double>("traditional/intervening_light_max_offset_ratio");
    params.pair_ambiguity_margin = requiredParam<double>("traditional/pair_ambiguity_margin");
    params.border_margin = requiredParam<int>("traditional/border_margin");
    return params;
  }

  ft_detector::PnpParams loadPnpParams() const {
    const auto dimensions = [this](const std::string& name) {
      return ft_detector::ArmorDimensions{requiredParam<double>(name + "/width_m"),
          requiredParam<double>(name + "/height_m"), requiredParam<bool>(name + "/confirmed")};
    };
    ft_detector::PnpParams params;
    params.small = dimensions("pnp/small");
    params.large = dimensions("pnp/large");
    params.allow_provisional_dimensions = requiredParam<bool>("pnp/allow_provisional_dimensions");
    params.ambiguity_error_margin_px = requiredParam<double>("pnp/continuity/error_margin_px");
    params.min_depth_m = requiredParam<double>("pnp/min_depth_m");
    params.max_depth_m = requiredParam<double>("pnp/max_depth_m");
    params.max_reprojection_error_px = requiredParam<double>("pnp/max_reprojection_error_px");
    params.min_quad_area_px2 = requiredParam<double>("pnp/min_quad_area_px2");
    params.min_edge_length_px = requiredParam<double>("pnp/min_edge_length_px");
    params.axis_length_m = requiredParam<double>("pnp/axis_length_m");
    return params;
  }

  void initializeTuning() {
    active_tuning_ = ft_detector::tuningFromParams(traditional_params_);
    auto clamped = active_tuning_;
    clamped.__clamp__();
    if (active_tuning_.__level__(clamped) != 0)
      throw std::invalid_argument("YAML tuning value is outside the Detector.cfg UI range");

    const ros::NodeHandle tuning_nh(private_nh_, "tuning");
    tuning_server_.reset(new TuningServer(tuning_mutex_, tuning_nh));
    // Seed before setCallback: its initial call must use the loaded YAML, including dataset overrides.
    tuning_server_->updateConfig(active_tuning_);
    tuning_server_->setConfigDefault(active_tuning_);
    tuning_server_->setCallback([this](ft_detector::DetectorConfig& config, std::uint32_t) {
      try {
        const auto next = ft_detector::paramsWithTuning(traditional_params_, config);
        const bool changed = active_tuning_.__level__(config) != 0;
        detector_->setParams(next);  // Validates the complete combination before changing the algorithm.
        traditional_params_ = next;
        active_tuning_ = config;
        if (changed) {
          pose_history_->reset();
          ROS_INFO("Detector tuning applied; previous PnP references cleared");
        }
      } catch (const std::invalid_argument& error) {
        config = active_tuning_;  // The server publishes and stores this valid configuration back to the client.
        ROS_WARN_STREAM("Detector tuning rejected; previous configuration retained: " << error.what());
      }
    });
    ROS_INFO_STREAM("Dynamic detector tuning: " << tuning_nh.getNamespace());
  }

  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& message) {
    const auto finite = [](double value) { return std::isfinite(value); };
    if (message->width == 0 || message->height == 0 || message->header.frame_id.empty() ||
        message->K[0] <= 0.0 || message->K[4] <= 0.0 ||
        message->binning_x > 1 || message->binning_y > 1 ||
        message->roi.x_offset != 0 || message->roi.y_offset != 0 ||
        (message->roi.width != 0 && message->roi.width != message->width) ||
        (message->roi.height != 0 && message->roi.height != message->height) ||
        !std::all_of(message->K.begin(), message->K.end(), finite) ||
        !std::all_of(message->D.begin(), message->D.end(), finite)) {
      camera_info_.reset();
      pose_history_->reset();
      ROS_WARN_THROTTLE(1.0, "Invalid or uncalibrated CameraInfo; calibration cache cleared");
      return;
    }

    ft_detector::CameraCalibration calibration;
    std::copy(message->K.begin(), message->K.end(), calibration.camera_matrix.val);
    calibration.distortion.assign(message->D.begin(), message->D.end());
    calibration.image_size = cv::Size(message->width, message->height);
    calibration.distortion_model = message->distortion_model;
    if (!ft_detector::PnpSolver::validCalibration(calibration)) {
      camera_info_.reset();
      pose_history_->reset();
      ROS_WARN_THROTTLE(1.0, "Unsupported camera model or invalid K/D; calibration cache cleared");
      return;
    }
    const bool changed = !camera_info_ || message->width != camera_info_->width ||
                         message->height != camera_info_->height ||
                         message->header.frame_id != camera_info_->header.frame_id ||
                         message->distortion_model != camera_info_->distortion_model ||
                         message->K != camera_info_->K || message->D != camera_info_->D;
    // 保留完整 CameraInfo；K、D、尺寸和 frame_id 供后续 PnP 使用。
    camera_info_ = message;
    calibration_ = calibration;
    if (changed) {
      pose_history_->reset();
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
        pose_history_->reset();
        ROS_WARN_THROTTLE(1.0, "Empty image received; frame skipped");
        return;
      }

      const bool calibration_ready = camera_info_ && camera_info_->width == message->width &&
                                     camera_info_->height == message->height &&
                                     camera_info_->header.frame_id == message->header.frame_id;
      if (!calibration_ready) {
        pose_history_->reset();
        ROS_WARN_THROTTLE(1.0, "No matching CameraInfo; 2D detection continues while calibration is unavailable");
      }

      const auto result = detector_->detect(shared_image->image);
      std::vector<ft_detector::PnpResult> poses(result.armors.size());
      for (std::size_t i = 0; i < result.armors.size(); ++i) {
        poses[i].pose.id = result.armors[i].id;
        poses[i].reason = pnp_enabled_ ? "awaiting_camera_info" : "disabled";
      }
      std::string pnp_status = "disabled";
      if (pnp_enabled_) {
        pnp_status = "awaiting_camera_info";
        if (calibration_ready) {
          std::vector<const ft_detector::ArmorPose*> references(result.armors.size(), nullptr);
          if (pose_continuity_enabled_)
            references = pose_history_->references(result.armors, message->header.stamp.toSec());
          for (std::size_t i = 0; i < result.armors.size(); ++i)
            poses[i] = pnp_solver_->solve(result.armors[i], calibration_, references[i]);
          // 求完整帧后再更新历史，避免前面返回的参考指针提前失效。
          if (pose_continuity_enabled_)
            pose_history_->store(result.armors, poses, message->header.stamp.toSec());
          pnp_status = poses.empty() ? "no_candidates" : poses.front().reason;
          if (std::any_of(poses.begin(), poses.end(), [](const ft_detector::PnpResult& pose) { return pose.valid; })) {
            const bool provisional = std::any_of(poses.begin(), poses.end(),
                [](const ft_detector::PnpResult& pose) { return pose.valid && pose.provisional; });
            pnp_status = provisional ? "provisional" : "ready";
          }
        }
      }
      if (!pnp_enabled_ || !pose_continuity_enabled_) pose_history_->reset();
      const auto output = ft_detector::buildDetectionOutput(message->header, result.armors, poses, child_frame_prefix_);
      detection_publisher_.publish(output.frame);  // 包括 detections 为空的成功处理帧。
      if (publish_tf_ && !output.transforms.empty()) tf_broadcaster_.sendTransform(output.transforms);
      auto annotated = ft_detector::drawDetections(shared_image->image, result, debug_details_);
      ft_detector::drawPoses(annotated, poses, pnp_status, debug_details_);
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
                                                     << " pnp=" << pnp_status
                                                     << " published=" << output.frame.detections.size()
                                                     << " frame=" << message->header.frame_id
                                                     << " calibration="
                                                     << (calibration_ready ? "ready" : "unavailable"));
    } catch (const cv_bridge::Exception& error) {
      pose_history_->reset();
      ROS_ERROR_STREAM("Image conversion failed: " << error.what());
    } catch (const cv::Exception& error) {
      pose_history_->reset();
      ROS_ERROR_STREAM("OpenCV image processing failed: " << error.what());
    } catch (const std::exception& error) {
      pose_history_->reset();
      ROS_ERROR_STREAM("Detection output failed: " << error.what());
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  image_transport::ImageTransport image_transport_;
  image_transport::Publisher debug_publisher_;
  image_transport::Publisher blue_mask_publisher_, red_mask_publisher_;
  ros::Publisher detection_publisher_;
  std::unique_ptr<ft_detector::TraditionalDetector> detector_;
  ft_detector::TraditionalParams traditional_params_;
  using TuningServer = dynamic_reconfigure::Server<ft_detector::DetectorConfig>;
  ft_detector::DetectorConfig active_tuning_;
  boost::recursive_mutex tuning_mutex_;
  std::unique_ptr<TuningServer> tuning_server_;
  std::unique_ptr<ft_detector::PnpSolver> pnp_solver_;
  std::unique_ptr<ft_detector::PoseHistory> pose_history_;
  ft_detector::CameraCalibration calibration_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  bool pnp_enabled_ = false, publish_tf_ = false;
  bool pose_continuity_enabled_ = false;
  std::string child_frame_prefix_;
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
