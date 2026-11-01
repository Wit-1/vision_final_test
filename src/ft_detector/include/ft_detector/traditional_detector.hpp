#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "ft_detector/armor_candidate.hpp"

namespace ft_detector {

struct HsvRange {
  int h_min, h_max, s_min, s_max, v_min, v_max;
};

// 不在算法里藏一套默认阈值；节点从 YAML 读取，测试自己提供参数。
struct TraditionalParams {
  HsvRange blue, red_low, red_high;
  double min_contour_area;
  double min_light_length;
  double min_light_aspect, max_light_aspect;
  double min_light_fill;
  double max_light_tilt_deg;
  int border_margin;
  double min_pair_length_ratio;
  double max_pair_angle_deg;
  double max_pair_center_dy_ratio;
  double min_pair_spacing_ratio, max_pair_spacing_ratio;
  double large_armor_min_spacing_ratio;
  double intervening_light_max_offset_ratio;
  double pair_ambiguity_margin;
};

struct LightBar {
  cv::RotatedRect rectangle;
  cv::Point2f center, top, bottom;
  double length = 0.0, width = 0.0, angle_deg = 0.0;
  bool is_red = false;
};

struct RejectedLight {
  cv::RotatedRect rectangle;
  std::string reason;
};

struct PairDecision {
  std::size_t left, right;  // DetectionResult::lights 的下标。
  double spacing_ratio = 0.0;
  double cost = 0.0;  // 越小越好：长度差 + 相对高度差 + 归一化角度差。
  std::string reason;
};

struct DetectionResult {
  cv::Mat blue_mask, red_mask;
  std::vector<LightBar> lights;
  std::vector<RejectedLight> rejected_lights;
  std::vector<PairDecision> pairs;
  std::vector<ArmorCandidate> armors;
};

// 角度是无向长轴的方向，周期为 180 度。
double normalizeLightAngle(double angle_deg);
double lightAngleDifference(double first_deg, double second_deg);

class TraditionalDetector {
 public:
  explicit TraditionalDetector(const TraditionalParams& params);
  void setParams(const TraditionalParams& params);
  DetectionResult detect(const cv::Mat& bgr) const;

 private:
  void findLights(const cv::Mat& mask, bool is_red, DetectionResult& result) const;
  void pairLights(DetectionResult& result) const;
  TraditionalParams params_;
};

}  // namespace ft_detector
