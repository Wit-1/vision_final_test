#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <iomanip>
#include <memory>
#include <stdexcept>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>
#include <opencv2/imgproc.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "ft_detector/debug_draw.hpp"
#include "ft_detector/detection_output.hpp"
#include "ft_detector/nn_detector.hpp"
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
    initializeModels();
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
    if (compare_modes_) comparison_publisher_ = image_transport_.advertise(comparison_topic_, debug_queue);
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

  ~DetectorNode() {
    if (frame_count_ > 0) std::clog << "FINAL " << timingSummary() << std::endl;
  }

 private:
  using Clock = std::chrono::steady_clock;

  static double elapsedMs(Clock::time_point start) {
    return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
  }

  std::string timingSummary() const {
    const auto traditional_count = std::max<std::uint64_t>(1, traditional_frames_);
    const auto nn_count = std::max<std::uint64_t>(1, nn_frames_);
    std::ostringstream summary;
    summary << std::fixed << std::setprecision(3)
            << "FT_TIMING mode=" << mode_ << " compare=" << compare_modes_ << " frames=" << frame_count_
            << " traditional_frames=" << traditional_frames_ << " nn_frames=" << nn_frames_
            << " mean_ms traditional=" << traditional_sum_ms_ / traditional_count
            << " nn_pre=" << nn_pre_sum_ms_ / nn_count << " nn_infer=" << nn_infer_sum_ms_ / nn_count
            << " nn_post=" << nn_post_sum_ms_ / nn_count
            << " callback=" << frame_sum_ms_ / std::max<std::uint64_t>(1, frame_count_);
    return summary.str();
  }

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

  void initializeModels() {
    mode_ = requiredParam<std::string>("mode");
    compare_modes_ = requiredParam<bool>("compare_modes");
    if (mode_ != "tradition" && mode_ != "nn")
      throw std::invalid_argument("mode must be tradition or nn");
    comparison_topic_ = requiredParam<std::string>("comparison_topic");
    comparison_scale_ = requiredParam<double>("comparison_scale");
    const int threads = requiredParam<int>("processing_threads");
    if (threads <= 0 || comparison_topic_.empty() || !std::isfinite(comparison_scale_) ||
        comparison_scale_ <= 0.0 || comparison_scale_ > 1.0)
      throw std::invalid_argument("Invalid processing threads, comparison topic or scale");
    cv::setNumThreads(threads);
    nn_params_.model_path = requiredParam<std::string>("nn/model_path");
    nn_params_.input_width = requiredParam<int>("nn/input_width");
    nn_params_.input_height = requiredParam<int>("nn/input_height");
    nn_params_.pad_value = requiredParam<int>("nn/pad_value");
    nn_params_.cpu_threads = requiredParam<int>("nn/cpu_threads");
    nn_params_.max_candidates = requiredParam<int>("nn/max_candidates");
    nn_params_.large_armor_min_spacing_ratio = traditional_params_.large_armor_min_spacing_ratio;
    nn_params_.min_quad_area_px2 = requiredParam<double>("pnp/min_quad_area_px2");
    nn_params_.min_edge_length_px = requiredParam<double>("pnp/min_edge_length_px");
    nn_params_.thresholds = {requiredParam<double>("tuning/nn_min_color_score"),
        requiredParam<double>("tuning/nn_min_number_score"), requiredParam<double>("tuning/nn_min_confidence"),
        requiredParam<double>("tuning/nn_nms_iou")};
    ft_detector::NnDetector::validateParams(nn_params_);
    if (mode_ == "nn" || compare_modes_) {
      nn_detector_.reset(new ft_detector::NnDetector(nn_params_));
      ROS_INFO_STREAM("Neural backend: ONNX Runtime " << ft_detector::NnDetector::backendVersion()
                      << " CPU, threads=" << nn_params_.cpu_threads << ", model=" << nn_params_.model_path);
    }
    ROS_INFO_STREAM("Detection mode=" << mode_ << " compare_modes=" << compare_modes_
                    << " OpenCV_threads=" << threads);
  }

  void initializeTuning() {
    active_tuning_ = ft_detector::tuningFromParams(traditional_params_);
    active_tuning_.nn_min_color_score = nn_params_.thresholds.color_score;
    active_tuning_.nn_min_number_score = nn_params_.thresholds.number_score;
    active_tuning_.nn_min_confidence = nn_params_.thresholds.confidence;
    active_tuning_.nn_nms_iou = nn_params_.thresholds.nms_iou;
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
        auto next_nn = nn_params_;
        next_nn.thresholds = {config.nn_min_color_score, config.nn_min_number_score,
                              config.nn_min_confidence, config.nn_nms_iou};
        ft_detector::NnDetector::validateParams(next_nn);
        const bool changed = active_tuning_.__level__(config) != 0;
        detector_->setParams(next);  // Validates the complete combination before changing the algorithm.
        if (nn_detector_) nn_detector_->setThresholds(next_nn.thresholds);
        nn_params_ = next_nn;
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

  struct PoseEvaluation {
    std::vector<ft_detector::PnpResult> poses;
    std::string status;
  };

  PoseEvaluation estimatePoses(const std::vector<ft_detector::ArmorCandidate>& armors,
                               double stamp, bool calibration_ready, bool update_history) {
    PoseEvaluation evaluation;
    evaluation.poses.resize(armors.size());
    evaluation.status = pnp_enabled_ ? "awaiting_camera_info" : "disabled";
    for (std::size_t i = 0; i < armors.size(); ++i) {
      evaluation.poses[i].pose.id = armors[i].id;
      evaluation.poses[i].reason = evaluation.status;
    }
    if (pnp_enabled_ && calibration_ready) {
      std::vector<const ft_detector::ArmorPose*> references(armors.size(), nullptr);
      if (update_history && pose_continuity_enabled_)
        references = pose_history_->references(armors, stamp);
      for (std::size_t i = 0; i < armors.size(); ++i)
        evaluation.poses[i] = pnp_solver_->solve(armors[i], calibration_, references[i]);
      if (update_history && pose_continuity_enabled_) pose_history_->store(armors, evaluation.poses, stamp);
      evaluation.status = evaluation.poses.empty() ? "no_candidates" : evaluation.poses.front().reason;
      if (std::any_of(evaluation.poses.begin(), evaluation.poses.end(), [](const auto& pose) { return pose.valid; })) {
        const bool provisional = std::any_of(evaluation.poses.begin(), evaluation.poses.end(),
            [](const auto& pose) { return pose.valid && pose.provisional; });
        evaluation.status = provisional ? "provisional" : "ready";
      }
    }
    if (update_history && (!pnp_enabled_ || !pose_continuity_enabled_)) pose_history_->reset();
    return evaluation;
  }

  static void labelMode(cv::Mat& image, const std::string& mode) {
    cv::putText(image, "Mode: " + mode, cv::Point(12, std::max(18, image.rows - 12)),
                cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& message) {
    try {
      const auto frame_start = Clock::now();
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

      ft_detector::DetectionResult traditional, neural;
      if (mode_ == "tradition" || compare_modes_) {
        const auto start = Clock::now();
        traditional = detector_->detect(shared_image->image);
        traditional_sum_ms_ += elapsedMs(start);
        ++traditional_frames_;
      }
      if (nn_detector_) {
        auto detected = nn_detector_->detect(shared_image->image);
        neural.armors = std::move(detected.decoded.armors);
        nn_pre_sum_ms_ += detected.preprocess_ms;
        nn_infer_sum_ms_ += detected.inference_ms;
        nn_post_sum_ms_ += detected.decode_ms;
        if (nn_frames_ == 0) {
          ROS_INFO_STREAM("NN output float32 [1,21,6300], raw score range=[" << detected.decoded.min_score
                          << "," << detected.decoded.max_score << "] candidates="
                          << detected.decoded.threshold_pass << " kept=" << neural.armors.size());
        }
        ++nn_frames_;
      }
      const auto& result = mode_ == "tradition" ? traditional : neural;
      const auto evaluation = estimatePoses(result.armors, message->header.stamp.toSec(), calibration_ready, true);
      const auto& poses = evaluation.poses;
      const auto& pnp_status = evaluation.status;
      const auto output = ft_detector::buildDetectionOutput(message->header, result.armors, poses, child_frame_prefix_);
      detection_publisher_.publish(output.frame);  // 包括 detections 为空的成功处理帧。
      if (publish_tf_ && !output.transforms.empty()) tf_broadcaster_.sendTransform(output.transforms);
      auto annotated = ft_detector::drawDetections(shared_image->image, result, debug_details_);
      ft_detector::drawPoses(annotated, poses, pnp_status, debug_details_);
      labelMode(annotated, mode_);
      debug_publisher_.publish(cv_bridge::CvImage(
          message->header, sensor_msgs::image_encodings::BGR8, annotated).toImageMsg());
      if (mode_ == "tradition" || compare_modes_) {
        blue_mask_publisher_.publish(cv_bridge::CvImage(
            message->header, sensor_msgs::image_encodings::MONO8, traditional.blue_mask).toImageMsg());
        red_mask_publisher_.publish(cv_bridge::CvImage(
            message->header, sensor_msgs::image_encodings::MONO8, traditional.red_mask).toImageMsg());
      }
      if (compare_modes_) {
        const auto& alternative = mode_ == "tradition" ? neural : traditional;
        const auto other_poses = estimatePoses(alternative.armors, message->header.stamp.toSec(), calibration_ready, false);
        auto other = ft_detector::drawDetections(shared_image->image, alternative, debug_details_);
        ft_detector::drawPoses(other, other_poses.poses, other_poses.status, debug_details_);
        labelMode(other, mode_ == "tradition" ? "nn" : "tradition");
        cv::Mat left, right, comparison;
        const cv::Size size(std::max(1, static_cast<int>(std::lround(message->width * comparison_scale_))),
                            std::max(1, static_cast<int>(std::lround(message->height * comparison_scale_))));
        cv::resize(mode_ == "tradition" ? annotated : other, left, size);
        cv::resize(mode_ == "tradition" ? other : annotated, right, size);
        cv::hconcat(left, right, comparison);
        comparison_publisher_.publish(cv_bridge::CvImage(
            message->header, sensor_msgs::image_encodings::BGR8, comparison).toImageMsg());
      }
      ++frame_count_;
      frame_sum_ms_ += elapsedMs(frame_start);
      ROS_INFO_STREAM_THROTTLE(1.0, "Processed image " << frame_count_ << ": " << message->width
                                                     << "x" << message->height
                                                     << " lights=" << result.lights.size()
                                                     << " armors=" << result.armors.size()
                                                     << " pnp=" << pnp_status
                                                     << " published=" << output.frame.detections.size()
                                                     << " frame=" << message->header.frame_id
                                                     << " calibration="
                                                     << (calibration_ready ? "ready" : "unavailable")
                                                     << " " << timingSummary());
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
  image_transport::Publisher comparison_publisher_;
  ros::Publisher detection_publisher_;
  std::unique_ptr<ft_detector::TraditionalDetector> detector_;
  ft_detector::TraditionalParams traditional_params_;
  std::unique_ptr<ft_detector::NnDetector> nn_detector_;
  ft_detector::NnParams nn_params_;
  std::string mode_, comparison_topic_;
  bool compare_modes_ = false;
  double comparison_scale_ = 0.5;
  double traditional_sum_ms_ = 0.0, nn_pre_sum_ms_ = 0.0, nn_infer_sum_ms_ = 0.0,
         nn_post_sum_ms_ = 0.0, frame_sum_ms_ = 0.0;
  std::uint64_t traditional_frames_ = 0, nn_frames_ = 0;
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
