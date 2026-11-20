#pragma once

#include <functional>
#include <string>
#include <vector>
#include "ft_detector/armor_candidate.hpp"
#include "ft_detector/roi.hpp"

namespace ft_detector {

// 搜索管理不区分检测方式，模型上下文与新区域资格由策略提供。
struct RoiPolicy {
  bool keep_model_aspect = false; // NN 保持原 4:3 裁剪，传统不强制比例。
  bool trust_union = false;       // 传统只信任各独立区域，不信任区域间空白。
  std::function<bool(const ArmorCandidate&)> seed_filter;
};

// 仅管理图像搜索区域，独立于 ROS、三维跟踪和编号身份关联。
class RoiPlanner {
 public:
  explicit RoiPlanner(const RoiParams& params, const RoiPolicy& policy = {});
  void reset();
  RoiDebugState begin(cv::Size size, double stamp, const std::string& frame);
  std::vector<ArmorCandidate> finish(const std::vector<ArmorCandidate>& candidates);

 private:
  struct Region {
    cv::Rect2f box;
    cv::Point2f velocity;
    double stamp = 0.0;
    bool red = false;
    int misses = 0, confirmations = 1;
  };
  cv::Rect expanded(const Region& region) const;
  bool compatible(const Region& region, const ArmorCandidate& armor) const;
  RoiParams params_;
  RoiPolicy policy_;
  RoiDebugState state_;
  cv::Size size_;
  double stamp_ = 0.0, last_stamp_ = -1.0, last_search_ = -1.0;
  std::string frame_;
  bool force_search_ = true;
  std::vector<Region> regions_, pending_;
};

}  // namespace ft_detector
