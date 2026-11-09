#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <opencv2/imgcodecs.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/CompressedImage.h>
#include <yaml-cpp/yaml.h>

#include <ft_detector/debug_draw.hpp>
#include <ft_detector/detection_output.hpp>
#include <ft_detector/nn_detector.hpp>
#include <ft_detector/pose_history.hpp>
#include <ft_selector/target_selector.hpp>
#include <ft_tracker/linear_tracker.hpp>

namespace {
// 单调时钟测量真实调用耗时，ROS 时间仅用于状态更新。
using Clock = std::chrono::steady_clock;
namespace fs = std::filesystem;

// 命令行只控制项目位置、素材、模式和处理片段。
struct Options {
  fs::path project;
  std::string dataset = "blue_1", mode = "tradition";
  double start_s = 0.0, duration_s = 1.0;
};

// 解析成对命令行参数，校验模式、数据集与时间范围。
Options parseOptions(int argc, char** argv) {
  Options options;
  for (int index = 1; index < argc; index += 2) {
    const std::string name = argv[index];
    if (name == "--help") {
      std::cout << "benchmark_pipeline --project ROOT [--dataset blue_1] [--mode tradition|nn] "
                   "[--start 0] [--duration 1]\nCSV goes to stdout; diagnostics go to stderr.\n";
      std::exit(0);
    }
    if (index + 1 == argc) throw std::invalid_argument("Missing value for " + name);
    const std::string value = argv[index + 1];
    if (name == "--project") options.project = value;
    else if (name == "--dataset") options.dataset = value;
    else if (name == "--mode") options.mode = value;
    else if (name == "--start" || name == "--duration") {
      std::size_t parsed = 0;
      const double seconds = std::stod(value, &parsed);
      if (parsed != value.size()) throw std::invalid_argument("Invalid seconds: " + value);
      if (name == "--start") options.start_s = seconds;
      else options.duration_s = seconds;
    } else throw std::invalid_argument("Unknown option " + name);
  }
  const std::vector<std::string> datasets{"blue_1", "blue_guard", "red_1_3", "red_4"};
  if (options.project.empty() || std::find(datasets.begin(), datasets.end(), options.dataset) == datasets.end() ||
      (options.mode != "tradition" && options.mode != "nn") || !std::isfinite(options.start_s) ||
      options.start_s < 0.0 || !std::isfinite(options.duration_s) || options.duration_s <= 0.0)
    throw std::invalid_argument("Invalid project, dataset, mode or time range");
  options.project = fs::canonical(options.project);
  return options;
}

// 支持以斜杠访问嵌套 YAML，字段缺失或类型错误时直接报错。
template <typename T> T required(const YAML::Node& node, const std::string& path) {
  const auto slash = path.find('/');
  if (slash == std::string::npos) return node[path].as<T>();
  return required<T>(node[path.substr(0, slash)], path.substr(slash + 1));
}

// 递归覆盖同名字段，复现 launch 的公共、模型、数据集加载顺序。
void merge(YAML::Node destination, const YAML::Node& source) {
  for (const auto& entry : source) {
    const auto name = entry.first.as<std::string>();
    if (entry.second.IsMap() && destination[name].IsMap()) merge(destination[name], entry.second);
    else destination[name] = YAML::Clone(entry.second);
  }
}

// 将 YAML 适配为传统检测参数，复用主工程的算法实现。
ft_detector::TraditionalParams traditionalParams(const YAML::Node& config) {
  const auto hsv = [&](const std::string& color) {
    const auto prefix = "tuning/" + color;
    return ft_detector::HsvRange{required<int>(config, prefix + "_h_min"), required<int>(config, prefix + "_h_max"),
        required<int>(config, prefix + "_s_min"), required<int>(config, prefix + "_s_max"),
        required<int>(config, prefix + "_v_min"), required<int>(config, prefix + "_v_max")};
  };
  ft_detector::TraditionalParams params;
  params.blue = hsv("blue");
  params.red_low = hsv("red_low");
  params.red_high = hsv("red_high");
  params.min_contour_area = required<double>(config, "tuning/min_contour_area");
  params.min_light_length = required<double>(config, "traditional/min_light_length");
  params.min_light_aspect = required<double>(config, "traditional/min_light_aspect");
  params.max_light_aspect = required<double>(config, "traditional/max_light_aspect");
  params.min_light_fill = required<double>(config, "traditional/min_light_fill");
  params.max_light_tilt_deg = required<double>(config, "traditional/max_light_tilt_deg");
  params.border_margin = required<int>(config, "traditional/border_margin");
  params.min_pair_length_ratio = required<double>(config, "traditional/min_pair_length_ratio");
  params.max_pair_angle_deg = required<double>(config, "tuning/max_pair_angle_deg");
  params.max_pair_center_dy_ratio = required<double>(config, "traditional/max_pair_center_dy_ratio");
  params.min_pair_spacing_ratio = required<double>(config, "traditional/min_pair_spacing_ratio");
  params.max_pair_spacing_ratio = required<double>(config, "tuning/max_pair_spacing_ratio");
  params.large_armor_min_spacing_ratio = required<double>(config, "traditional/large_armor_min_spacing_ratio");
  params.intervening_light_max_offset_ratio = required<double>(config, "traditional/intervening_light_max_offset_ratio");
  params.pair_ambiguity_margin = required<double>(config, "traditional/pair_ambiguity_margin");
  return params;
}

// 读取大小板尺寸、临时尺寸开关与 PnP 筛选条件。
ft_detector::PnpParams poseParams(const YAML::Node& config) {
  const auto dimensions = [&](const std::string& size) {
    const auto prefix = "pnp/" + size;
    return ft_detector::ArmorDimensions{required<double>(config, prefix + "/width_m"),
        required<double>(config, prefix + "/height_m"), required<bool>(config, prefix + "/confirmed")};
  };
  ft_detector::PnpParams params;
  params.small = dimensions("small");
  params.large = dimensions("large");
  params.allow_provisional_dimensions = required<bool>(config, "pnp/allow_provisional_dimensions");
  params.ambiguity_error_margin_px = required<double>(config, "pnp/continuity/error_margin_px");
  params.min_depth_m = required<double>(config, "pnp/min_depth_m");
  params.max_depth_m = required<double>(config, "pnp/max_depth_m");
  params.max_reprojection_error_px = required<double>(config, "pnp/max_reprojection_error_px");
  params.min_quad_area_px2 = required<double>(config, "pnp/min_quad_area_px2");
  params.min_edge_length_px = required<double>(config, "pnp/min_edge_length_px");
  params.axis_length_m = required<double>(config, "pnp/axis_length_m");
  return params;
}

// 读取 NN 协议及门限，并替换本项目使用的包路径占位符。
ft_detector::NnParams neuralParams(const YAML::Node& config, const fs::path& package) {
  ft_detector::NnParams params;
  params.model_path = required<std::string>(config, "nn/model_path");
  const std::string token = "$(find ft_bringup)";
  const auto location = params.model_path.find(token);
  if (location != std::string::npos) params.model_path.replace(location, token.size(), package.string());
  params.input_width = required<int>(config, "nn/input_width");
  params.input_height = required<int>(config, "nn/input_height");
  params.pad_value = required<int>(config, "nn/pad_value");
  params.cpu_threads = required<int>(config, "nn/cpu_threads");
  params.max_candidates = required<int>(config, "nn/max_candidates");
  params.large_armor_min_spacing_ratio = required<double>(config, "traditional/large_armor_min_spacing_ratio");
  params.min_quad_area_px2 = required<double>(config, "pnp/min_quad_area_px2");
  params.min_edge_length_px = required<double>(config, "pnp/min_edge_length_px");
  params.thresholds = {required<double>(config, "tuning/nn_min_color_score"),
      required<double>(config, "tuning/nn_min_number_score"), required<double>(config, "tuning/nn_min_confidence"),
      required<double>(config, "tuning/nn_nms_iou")};
  return params;
}

// 读取平移滤波、关联、生命周期和提前量配置。
ft_tracker::TrackerParams linearParams(const YAML::Node& config) {
  return {required<double>(config, "process_noise_density"), required<double>(config, "measurement_noise_variance"),
      required<double>(config, "initial_position_variance"), required<double>(config, "initial_velocity_variance"),
      required<double>(config, "association_distance_m"), required<double>(config, "lost_timeout_s"),
      required<double>(config, "prediction_time_s"), required<bool>(config, "match_color"), required<bool>(config, "match_size")};
}

// 读取圆周模型及旋转 EKF 配置，保持与在线节点相同的参数含义。
ft_selector::RotatingParams rotatingParams(const YAML::Node& config) {
  ft_selector::RotatingParams params;
  params.circle = {required<double>(config, "radius_m"), required<int>(config, "plate_count")};
  params.position_gate_m = required<double>(config, "association_position_m");
  params.yaw_gate_rad = required<double>(config, "association_yaw_rad");
  params.lost_timeout_s = required<double>(config, "lost_timeout_s");
  params.linear_noise_density = required<double>(config, "linear_noise_density");
  params.angular_noise_density = required<double>(config, "angular_noise_density");
  params.position_variance = required<double>(config, "position_variance");
  params.yaw_variance = required<double>(config, "yaw_variance");
  params.initial_velocity_variance = required<double>(config, "initial_velocity_variance");
  params.initial_omega_variance = required<double>(config, "initial_omega_variance");
  return params;
}

// 将 bag 中的 CameraInfo 转为共用相机模型，内参不在工具中另设。
ft_detector::CameraCalibration calibrationFrom(const sensor_msgs::CameraInfo& info) {
  ft_detector::CameraCalibration camera;
  std::copy(info.K.begin(), info.K.end(), camera.camera_matrix.val);
  camera.distortion.assign(info.D.begin(), info.D.end());
  camera.image_size = {static_cast<int>(info.width), static_cast<int>(info.height)};
  camera.distortion_model = info.distortion_model;
  return camera;
}

// 缓存前检查相机帧名、ROI、分箱和针孔畸变协议。
bool validInfo(const sensor_msgs::CameraInfo& info) {
  const auto& roi = info.roi;
  return !info.header.frame_id.empty() && info.binning_x <= 1 && info.binning_y <= 1 &&
      roi.x_offset == 0 && roi.y_offset == 0 &&
      (roi.width == 0 || roi.width == info.width) && (roi.height == 0 || roi.height == info.height) &&
      ft_detector::PnpSolver::validCalibration(calibrationFrom(info));
}

// 统一把两个计时点之间的差值转换为毫秒。
double elapsed(Clock::time_point first, Clock::time_point last) {
  return std::chrono::duration<double, std::milli>(last - first).count();
}

// 离线基准入口：准备资源，再顺序调用现有算法库并输出逐帧 CSV。
void run(const Options& options) {
  // 配置与模型初始化在逐帧计时之外，不依赖 ROS master。
  const auto package = options.project / "src/ft_bringup";
  const auto config_dir = package / "config";
  auto config = YAML::LoadFile((config_dir / "detector.yaml").string());
  merge(config, YAML::LoadFile((config_dir / "model.yaml").string()));
  merge(config, YAML::LoadFile((config_dir / "datasets" / (options.dataset + ".yaml")).string()));
  const auto linear_config = YAML::LoadFile((config_dir / "tracker.yaml").string());
  const auto rotating_config = YAML::LoadFile((config_dir / "selector.yaml").string());
  // 建立两种检测、PnP 历史及两种跟踪器，状态在相邻图像间持续保存。
  const int threads = required<int>(config, "processing_threads");
  if (threads <= 0) throw std::invalid_argument("Invalid OpenCV processing thread count");
  cv::setNumThreads(threads);
  ft_detector::TraditionalDetector traditional(traditionalParams(config));
  ft_detector::PnpSolver solver(poseParams(config));
  ft_detector::PoseHistory history(required<double>(config, "pnp/continuity/max_gap_s"),
                                   required<double>(config, "pnp/continuity/max_corner_shift_ratio"));
  std::unique_ptr<ft_detector::NnDetector> neural;
  if (options.mode == "nn") neural = std::make_unique<ft_detector::NnDetector>(neuralParams(config, package));
  const auto linear_params = linearParams(linear_config);
  const auto rotating_params = rotatingParams(rotating_config);
  ft_tracker::LinearTracker linear(linear_params);
  ft_selector::RotatingTracker rotating(rotating_params);
  const double min_horizontal = required<double>(rotating_config, "min_horizontal_normal");
  const double min_facing = required<double>(rotating_config, "min_facing_cosine");
  if (!(min_horizontal > 0.0 && min_horizontal <= 1.0) || !(min_facing >= 0.0 && min_facing < 1.0))
    throw std::invalid_argument("Invalid selector normal or facing threshold");
  const bool pnp_enabled = required<bool>(config, "pnp/enabled");
  const bool continuity = required<bool>(config, "pnp/continuity/enabled");
  const bool details = required<bool>(config, "debug_details");
  const auto child_prefix = required<std::string>(config, "pnp/child_frame_prefix");
  if (required<std::string>(config, "image_transport") != "compressed")
    throw std::invalid_argument("This bag benchmark requires compressed image transport");
  // 片段范围按 bag 记录时间指定，只读取图像和 CameraInfo。
  const auto image_topic = required<std::string>(config, "image_topic") + "/compressed";
  const auto info_topic = required<std::string>(config, "camera_info_topic");
  const auto bag_path = options.project / "task" / required<std::string>(config, "bag_file");
  rosbag::Bag bag(bag_path.string(), rosbag::bagmode::Read);
  rosbag::View all_messages(bag);
  if (all_messages.size() == 0) throw std::runtime_error("The bag is empty");
  const auto begin_time = all_messages.getBeginTime() + ros::Duration(options.start_s);
  const auto end_time = begin_time + ros::Duration(options.duration_s);
  rosbag::View view(bag, rosbag::TopicQuery({image_topic, info_topic}));
  sensor_msgs::CameraInfoConstPtr info;
  ft_detector::CameraCalibration camera;
  std_msgs::Header previous;
  std::size_t count = 0;
  // 诊断写标准错误，标准输出只放可供汇总工具读取的 CSV。
  std::cerr << "Offline serial benchmark; OpenCV " << CV_VERSION << ", ORT "
            << ft_detector::NnDetector::backendVersion() << "; first frame included; model/bag loading excluded.\n";
  std::cout << "dataset,mode,stamp_ns,frame_id,decode_ms,detect_ms,pose_ms,track_ms,selector_ms,draw_ms,total_ms,"
               "candidates,detections,track_id,tracking,target_id,target_valid\n" << std::fixed << std::setprecision(6);
  for (const auto& record : view) {
    if (record.getTime() > end_time) break;
    // 即使标定位于片段起点之前也先缓存；缓存处理不计入帧耗时。
    if (const auto next = record.instantiate<sensor_msgs::CameraInfo>()) {
      const bool changed = !info || next->header.frame_id != info->header.frame_id || next->width != info->width ||
          next->height != info->height || next->K != info->K || next->D != info->D ||
          next->distortion_model != info->distortion_model;
      const bool valid = validInfo(*next);
      if (changed || !valid) history.reset();
      info = valid ? next : nullptr;
      if (info) camera = calibrationFrom(*info);
      continue;
    }
    if (record.getTime() < begin_time) continue;
    const auto image = record.instantiate<sensor_msgs::CompressedImage>();
    if (!image) continue;
    // 跳过重复帧，时间回退或换系时清除算法状态。
    const auto& header = image->header;
    if (!previous.frame_id.empty() && header.frame_id == previous.frame_id && header.stamp == previous.stamp) continue;
    if (header.frame_id != previous.frame_id || header.stamp < previous.stamp) {
      history.reset();
      linear.reset();
      rotating.reset();
    }
    previous = header;
    // 计时从 JPEG 解码开始，每个阶段边界记录一次单调时钟。
    const auto start = Clock::now();
    const cv::Mat bgr = cv::imdecode(image->data, cv::IMREAD_COLOR);
    if (bgr.empty()) throw std::runtime_error("Image decoding failed");
    const auto decoded = Clock::now();
    // 只测选中的检测方式，NN 阶段包含预处理、推理和后处理。
    ft_detector::DetectionResult result;
    if (neural) result.armors = neural->detect(bgr);
    else result = traditional.detect(bgr);
    const auto detected = Clock::now();
    // 共用 PnP 与历史选解逻辑，缺标定时保留空的有效检测输出。
    const bool calibrated = info && camera.image_size == bgr.size() && info->header.frame_id == header.frame_id;
    std::vector<ft_detector::PnpResult> poses(result.armors.size());
    if (pnp_enabled && calibrated) {
      std::vector<const ft_detector::ArmorPose*> references(result.armors.size(), nullptr);
      if (continuity) references = history.references(result.armors, header.stamp.toSec());
      for (std::size_t index = 0; index < result.armors.size(); ++index)
        poses[index] = solver.solve(result.armors[index], camera, references[index]);
      if (continuity) history.store(result.armors, poses, header.stamp.toSec());
    }
    if (!pnp_enabled || !calibrated || !continuity) history.reset();
    const auto output = ft_detector::buildDetectionOutput(header, result.armors, poses, child_prefix);
    // 位姿阶段同时包含消息构造和两路观测转换，不做 ROS 发布。
    std::vector<ft_tracker::Observation> linear_observations;
    std::vector<ft_selector::ArmorObservation> rotating_observations;
    for (const auto& detection : output.frame.detections) {
      const auto& p = detection.pose.position;
      linear_observations.push_back({detection.id, detection.is_red != 0, detection.is_large_armor != 0, {p.x,p.y,p.z}});
      const auto& q = detection.pose.orientation;
      Eigen::Quaterniond rotation(q.w,q.x,q.y,q.z);
      const Eigen::Vector3d normal = -(rotation.normalized() * Eigen::Vector3d::UnitZ());
      if (std::hypot(normal.x(), normal.z()) >= min_horizontal && normal.x()*p.x + normal.z()*p.z < 0.0)
        rotating_observations.push_back({{p.x,p.y,p.z}, std::atan2(normal.x(),normal.z()), detection.is_red != 0});
    }
    const auto posed = Clock::now();
    // 用图像时间分别更新平移跟踪与旋转选板，两者沿用同一 T。
    ft_tracker::TrackState track;
    if (!header.frame_id.empty()) track = linear.update(header.stamp.toSec(), linear_observations);
    const auto tracked = Clock::now();
    ft_selector::SelectedTarget target;
    if (!header.frame_id.empty()) {
      const auto vehicle = rotating.update(header.stamp.toSec(), rotating_observations);
      target = ft_selector::selectTarget(vehicle, rotating_params.circle, linear_params.prediction_time_s, min_facing);
    }
    const auto selected = Clock::now();
    // 绘图纳入串行总用时，CSV 写入位于最后一个计时点之后。
    auto debug = ft_detector::drawDetections(bgr, result, details);
    ft_detector::drawPoses(debug, poses, calibrated ? "ready" : "awaiting_camera_info", details);
    const auto drawn = Clock::now();
    std::cout << options.dataset << ',' << options.mode << ',' << header.stamp.toNSec() << ','
              << std::quoted(header.frame_id, '"', '"') << ',' << elapsed(start,decoded) << ','
              << elapsed(decoded,detected) << ',' << elapsed(detected,posed) << ',' << elapsed(posed,tracked) << ','
              << elapsed(tracked,selected) << ',' << elapsed(selected,drawn) << ',' << elapsed(start,drawn) << ','
              << result.armors.size() << ',' << output.frame.detections.size() << ',' << track.id << ','
              << track.tracking << ',' << target.id << ',' << target.valid << '\n';
    ++count;
  }
  // 空片段和写入失败返回错误，避免把无效文件当成性能结果。
  if (count == 0) throw std::runtime_error("No image frames in the requested range");
  if (!std::cout) throw std::runtime_error("Failed to write timing CSV");
  std::cerr << "Measured " << count << " frames; no ROS master or playback-rate scaling used.\n";
}
}  // namespace

// 初始化独立使用的 ROS 时间类型，捕获参数或运行异常并返回非零状态。
int main(int argc, char** argv) {
  try {
    ros::Time::init();
    run(parseOptions(argc, argv));
  } catch (const std::exception& error) {
    std::cerr << "Benchmark failed: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
