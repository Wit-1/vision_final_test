#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <stdexcept>
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
#include "ft_detector/traditional_roi.hpp"
#include "ft_detector/tuning_parameters.hpp"

// 检测节点负责 ROS 接入和组织调用，算法由独立类实现。
class DetectorNode {
 public:
  // 启动时加载配置、建立算法对象，再连接输入、输出和调参服务。
  DetectorNode() : private_nh_("~"), image_transport_(nh_) {
    // 读取话题和检测配置，两种模式都通过同一候选接口输出。
    const auto image_topic = requiredParam<std::string>("image_topic");
    const auto camera_info_topic = requiredParam<std::string>("camera_info_topic");
    const auto debug_image_topic = requiredParam<std::string>("debug_image_topic");
    const auto blue_mask_topic = requiredParam<std::string>("blue_mask_topic");
    const auto red_mask_topic = requiredParam<std::string>("red_mask_topic");
    const auto detection_topic = requiredParam<std::string>("detection_topic");
    debug_details_ = requiredParam<bool>("debug_details");
    traditional_params_ = loadTraditionalParams();
    detector_.reset(new ft_detector::TraditionalRoiDetector(traditional_params_, loadRoiParams("traditional/roi")));
    initializeModels();
    // 设置 PnP、TF 子系名称和跨帧选解历史。
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
    // 队列和传输方式由 YAML 控制，启动时拒绝无效配置。
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

    // 注册输出与输入回调；CameraInfo 独立缓存，不要求每帧同步到达。
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
  // 参数必须存在且类型正确，避免隐藏的默认值掩盖 YAML 配置错误。
  template <typename T>
  T requiredParam(const std::string& name) const {
    T value;
    if (!private_nh_.getParam(name, value)) {
      throw std::invalid_argument("Missing or wrongly typed parameter: " + private_nh_.resolveName(name));
    }
    return value;
  }

  // 将参数服务器中的颜色区间和几何阈值组装成传统检测配置。
  ft_detector::TraditionalParams loadTraditionalParams() const {
    // 三组 HSV 共用同一套字段读取方式。
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

  // 两种检测方式共用搜索时序；留白和最小窗口分别由自己的 YAML 决定。
  ft_detector::RoiParams loadRoiParams(const std::string& prefix) const {
    ft_detector::RoiParams params;
    params.enabled = requiredParam<bool>(prefix + "/enabled");
    params.search_interval_s = requiredParam<double>(prefix + "/search_interval_s");
    params.max_history_gap_s = requiredParam<double>(prefix + "/max_history_gap_s");
    params.horizontal_padding = requiredParam<double>(prefix + "/horizontal_padding");
    params.vertical_padding = requiredParam<double>(prefix + "/vertical_padding");
    params.min_width = requiredParam<int>(prefix + "/min_width");
    params.min_height = requiredParam<int>(prefix + "/min_height");
    params.confirmation_searches = requiredParam<int>(prefix + "/confirmation_searches");
    params.max_misses = requiredParam<int>(prefix + "/max_misses");
    return params;
  }

  // 读取大小板端点尺寸以及深度、重投影和绘轴门限。
  ft_detector::PnpParams loadPnpParams() const {
    const auto dimensions = [this](const std::string& name) {
      return ft_detector::ArmorDimensions{requiredParam<double>(name + "/width_m"),
          requiredParam<double>(name + "/height_m")};
    };
    ft_detector::PnpParams params;
    params.small = dimensions("pnp/small");
    params.large = dimensions("pnp/large");
    params.ambiguity_error_margin_px = requiredParam<double>("pnp/continuity/error_margin_px");
    params.min_depth_m = requiredParam<double>("pnp/min_depth_m");
    params.max_depth_m = requiredParam<double>("pnp/max_depth_m");
    params.max_reprojection_error_px = requiredParam<double>("pnp/max_reprojection_error_px");
    params.min_quad_area_px2 = requiredParam<double>("pnp/min_quad_area_px2");
    params.min_edge_length_px = requiredParam<double>("pnp/min_edge_length_px");
    params.axis_length_m = requiredParam<double>("pnp/axis_length_m");
    return params;
  }

  // 检查模式并配置 OpenCV/NN，仅在 NN 模式创建推理 Session。
  void initializeModels() {
    mode_ = requiredParam<std::string>("mode");
    if (mode_ != "tradition" && mode_ != "nn")
      throw std::invalid_argument("mode must be tradition or nn");
    int threads = requiredParam<int>("processing_threads");
    if (mode_=="nn" && private_nh_.hasParam("nn/processing_threads"))
      threads=requiredParam<int>("nn/processing_threads");
    if (threads <= 0) throw std::invalid_argument("Invalid processing thread count");
    cv::setNumThreads(threads);
    // 模型路径、协议和筛选参数与传统模式共用的几何参数在此对齐。
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
    // 编号辅助仍只在 NN 内部使用，不参与跟踪关联。
    auto& policy=nn_params_.size_policy;
    policy.enabled=requiredParam<bool>("nn/number_size/enabled");
    policy.min_score=requiredParam<double>("nn/number_size/min_score");
    policy.min_margin=requiredParam<double>("nn/number_size/min_margin");
    const auto sizes=requiredParam<std::vector<int>>("nn/number_size/class_sizes");
    if (sizes.size()!=policy.class_sizes.size()) throw std::invalid_argument("Expected nine number-size rules");
    std::copy(sizes.begin(),sizes.end(),policy.class_sizes.begin());
    auto& roi=nn_params_.roi;
    static_cast<ft_detector::RoiParams&>(roi)=loadRoiParams("nn/roi");
    roi.seed_number_score=requiredParam<double>("nn/roi/seed_number_score");
    roi.seed_number_margin=requiredParam<double>("nn/roi/seed_number_margin");
    ft_detector::NnDetector::validateParams(nn_params_);
    if (mode_ == "nn") {
      nn_detector_.reset(new ft_detector::NnDetector(nn_params_));
      ROS_INFO_STREAM("Neural backend: ONNX Runtime " << ft_detector::NnDetector::backendVersion()
                      << " CPU, threads=" << nn_params_.cpu_threads << ", model=" << nn_params_.model_path);
    }
    ROS_INFO_STREAM("Detection mode=" << mode_ << " OpenCV_threads=" << threads);
  }

  // 用启动 YAML 设置动态初值与默认值，再注册整组校验的更新回调。
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

    // 实例默认值采用数据集覆盖后的配置，不直接使用 .cfg 的静态默认。
    const ros::NodeHandle tuning_nh(private_nh_, "tuning");
    tuning_server_.reset(new TuningServer(tuning_mutex_, tuning_nh));
    // Seed before setCallback: its initial call must use the loaded YAML, including dataset overrides.
    tuning_server_->updateConfig(active_tuning_);
    tuning_server_->setConfigDefault(active_tuning_);
    tuning_server_->setCallback([this](ft_detector::DetectorConfig& config, std::uint32_t) {
      // 先组装并校验两种检测配置，合法后统一应用。
      try {
        const auto next = ft_detector::paramsWithTuning(traditional_params_, config);
        auto next_nn = nn_params_;
        next_nn.thresholds = {config.nn_min_color_score, config.nn_min_number_score,
                              config.nn_min_confidence, config.nn_nms_iou};
        next_nn.large_armor_min_spacing_ratio = config.large_armor_min_spacing_ratio;
        ft_detector::NnDetector::validateParams(next_nn);
        const bool changed = active_tuning_.__level__(config) != 0;
        detector_->setParams(next);  // Validates the complete combination before changing the algorithm.
        if (nn_detector_) nn_detector_->setThresholds(next_nn.thresholds, next_nn.large_armor_min_spacing_ratio);
        nn_params_ = next_nn;
        traditional_params_ = next;
        active_tuning_ = config;
        // 检测条件改变后，旧像素对应的 PnP 历史不再继续使用。
        if (changed) {
          detector_->resetHistory();
          pose_history_->reset();
          ROS_INFO("Detector tuning applied; previous ROI and PnP references cleared");
        }
      } catch (const std::invalid_argument& error) {
        // 将整个请求恢复为上一组有效值，同时反馈给调参界面。
        config = active_tuning_;  // The server publishes and stores this valid configuration back to the client.
        ROS_WARN_STREAM("Detector tuning rejected; previous configuration retained: " << error.what());
      }
    });
    ROS_INFO_STREAM("Dynamic detector tuning: " << tuning_nh.getNamespace());
  }

  // 校验并缓存最新标定，无效消息会主动清空缓存和位姿历史。
  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& message) {
    // 拒绝未标定、裁剪或分箱与当前支持方式不一致的相机信息。
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

    // 将 ROS 的 K/D 转为算法相机模型，并校验畸变协议。
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
    // 只有标定内容变化时才重置历史；共享指针使缓存跨回调保持有效。
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

  // 同时返回逐候选 PnP 结果和本帧调试状态，保持候选下标对应。
  struct PoseEvaluation {
    std::vector<ft_detector::PnpResult> poses;
    std::string status;
  };

  // 统一解算选中模式的所有候选，缺标定时保留带原因的失败结果。
  PoseEvaluation estimatePoses(const std::vector<ft_detector::ArmorCandidate>& armors,
                               double stamp, bool calibration_ready) {
    PoseEvaluation evaluation;
    evaluation.poses.resize(armors.size());
    evaluation.status = pnp_enabled_ ? "awaiting_camera_info" : "disabled";
    for (std::size_t i = 0; i < armors.size(); ++i) {
      evaluation.poses[i].pose.id = armors[i].id;
      evaluation.poses[i].reason = evaluation.status;
    }
    // 先取得唯一匹配的历史参考，整帧解算完成后再替换历史。
    if (pnp_enabled_ && calibration_ready) {
      std::vector<const ft_detector::ArmorPose*> references(armors.size(), nullptr);
      if (pose_continuity_enabled_)
        references = pose_history_->references(armors, stamp);
      for (std::size_t i = 0; i < armors.size(); ++i)
        evaluation.poses[i] = pnp_solver_->solve(armors[i], calibration_, references[i]);
      if (pose_continuity_enabled_) pose_history_->store(armors, evaluation.poses, stamp);
      evaluation.status = evaluation.poses.empty() ? "no_candidates" : evaluation.poses.front().reason;
      // 存在有效位姿时显示 ready，否则保留失败原因。
      if (std::any_of(evaluation.poses.begin(), evaluation.poses.end(), [](const auto& pose) { return pose.valid; })) {
        evaluation.status = "ready";
      }
    }
    if (!pnp_enabled_ || !pose_continuity_enabled_) pose_history_->reset();
    return evaluation;
  }

  // 标注本帧实际运行的检测模式。
  static void labelMode(cv::Mat& image, const std::string& mode) {
    cv::putText(image, "Mode: " + mode, cv::Point(12, std::max(18, image.rows - 12)),
                cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
  }

  // 图像回调串起检测、PnP、消息/TF 和基础调试图，沿用输入 header。
  void imageCallback(const sensor_msgs::ImageConstPtr& message) {
    try {
      // 编码相同时共享解码后的像素；压缩解码和发布输出仍有各自的数据分配。
      const auto shared_image = cv_bridge::toCvShare(message, sensor_msgs::image_encodings::BGR8);
      if (shared_image->image.empty()) {
        detector_->resetHistory();
        if (nn_detector_) nn_detector_->resetHistory();
        pose_history_->reset();
        ROS_WARN_THROTTLE(1.0, "Empty image received; frame skipped");
        return;
      }

      // 标定尺寸和坐标系必须与图像一致，缺标定仍可继续二维检测。
      const bool calibration_ready = camera_info_ && camera_info_->width == message->width &&
                                     camera_info_->height == message->height &&
                                     camera_info_->header.frame_id == message->header.frame_id;
      if (!calibration_ready) {
        pose_history_->reset();
        ROS_WARN_THROTTLE(1.0, "No matching CameraInfo; 2D detection continues while calibration is unavailable");
      }

      // 仅运行选中模式，随后接入共用的位姿和发布流程。
      ft_detector::DetectionResult result;
      if (mode_ == "tradition") result = detector_->detect(
          shared_image->image, message->header.stamp.toSec(), message->header.frame_id);
      else result.armors = nn_detector_->detect(shared_image->image,message->header.stamp.toSec(),message->header.frame_id);
      const auto evaluation = estimatePoses(result.armors, message->header.stamp.toSec(), calibration_ready);
      const auto& poses = evaluation.poses;
      const auto& pnp_status = evaluation.status;
      // 过滤无效位姿后发布整帧；空数组也用于推进下游丢失状态。
      const auto output = ft_detector::buildDetectionOutput(message->header, result.armors, poses, child_frame_prefix_);
      detection_publisher_.publish(output.frame);  // 包括 detections 为空的成功处理帧。
      if (publish_tf_ && !output.transforms.empty()) tf_broadcaster_.sendTransform(output.transforms);
      // 在副本绘制候选与坐标轴，发布图像和传统模式的颜色掩膜。
      auto annotated = ft_detector::drawDetections(shared_image->image, result, debug_details_);
      ft_detector::drawPoses(annotated, poses, pnp_status, debug_details_);
      labelMode(annotated, mode_);
      ft_detector::drawRoiDebug(annotated, nn_detector_ ? nn_detector_->debugState() : detector_->debugState());
      debug_publisher_.publish(cv_bridge::CvImage(
          message->header, sensor_msgs::image_encodings::BGR8, annotated).toImageMsg());
      if (mode_ == "tradition") {
        blue_mask_publisher_.publish(cv_bridge::CvImage(
            message->header, sensor_msgs::image_encodings::MONO8, result.blue_mask).toImageMsg());
        red_mask_publisher_.publish(cv_bridge::CvImage(
            message->header, sensor_msgs::image_encodings::MONO8, result.red_mask).toImageMsg());
      }
      // 限频输出运行状态，累计帧数只用于观察节点是否持续处理。
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
      // 本帧处理失败时清理参考，避免下一帧使用失效的姿态历史。
      detector_->resetHistory();
      if (nn_detector_) nn_detector_->resetHistory();
      pose_history_->reset();
      ROS_ERROR_STREAM("Image conversion failed: " << error.what());
    } catch (const cv::Exception& error) {
      detector_->resetHistory();
      if (nn_detector_) nn_detector_->resetHistory();
      pose_history_->reset();
      ROS_ERROR_STREAM("OpenCV image processing failed: " << error.what());
    } catch (const std::exception& error) {
      detector_->resetHistory();
      if (nn_detector_) nn_detector_->resetHistory();
      pose_history_->reset();
      ROS_ERROR_STREAM("Detection output failed: " << error.what());
    }
  }

  // ROS 通信句柄与发布者，以及跨图像回调复用的算法和配置对象。
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  image_transport::ImageTransport image_transport_;
  image_transport::Publisher debug_publisher_;
  image_transport::Publisher blue_mask_publisher_, red_mask_publisher_;
  ros::Publisher detection_publisher_;
  std::unique_ptr<ft_detector::TraditionalRoiDetector> detector_;
  ft_detector::TraditionalParams traditional_params_;
  std::unique_ptr<ft_detector::NnDetector> nn_detector_;
  ft_detector::NnParams nn_params_;
  std::string mode_;
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

// 注册检测节点并进入 ROS 回调循环，启动失败时返回非零状态。
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
