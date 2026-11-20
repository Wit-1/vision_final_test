#pragma once

#include <string>
#include <vector>
#include <opencv2/core.hpp>

namespace ft_detector {

// 两种检测器共用搜索时序与留白，模型相关的确认条件由各自适配。
struct RoiParams {
  bool enabled = false;
  double search_interval_s = 0.2, max_history_gap_s = 0.5;
  double horizontal_padding = 2.0, vertical_padding = 0.5;
  int min_width = 64, min_height = 32, confirmation_searches = 2, max_misses = 2;
};

struct RoiDebugState {
  bool enabled = false, full_search = true;
  cv::Rect crop;
  std::vector<cv::Rect> regions;
  std::string status;
};

void validateRoiParams(const RoiParams& params);
void drawRoiDebug(cv::Mat& image, const RoiDebugState& state);

}  // namespace ft_detector
