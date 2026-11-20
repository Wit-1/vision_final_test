#pragma once

#include <memory>
#include "ft_detector/roi.hpp"
#include "ft_detector/traditional_detector.hpp"

namespace ft_detector {

// 为原有无状态传统检测增加搜索历史，输出始终使用完整原图的坐标。
class TraditionalRoiDetector {
 public:
  TraditionalRoiDetector(const TraditionalParams& detector, const RoiParams& roi);
  ~TraditionalRoiDetector();
  TraditionalRoiDetector(const TraditionalRoiDetector&) = delete;
  TraditionalRoiDetector& operator=(const TraditionalRoiDetector&) = delete;
  void setParams(const TraditionalParams& params);
  DetectionResult detect(const cv::Mat& bgr, double stamp_s, const std::string& frame_id);
  void resetHistory();
  const RoiDebugState& debugState() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace ft_detector
