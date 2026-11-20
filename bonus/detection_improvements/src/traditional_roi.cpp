#include "ft_detector/traditional_roi.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>
#include "detection_improvements/roi_planner.hpp"

namespace ft_detector {
namespace {

// 平移所有二维调试几何；配对下标和几何尺度不受裁剪偏移影响。
void restoreCoordinates(DetectionResult& result, const cv::Point& origin, cv::Size size) {
  const cv::Point2f offset(origin);
  for (auto& light : result.lights) {
    light.rectangle.center += offset;
    light.center += offset;
    light.top += offset;
    light.bottom += offset;
  }
  for (auto& light : result.rejected_lights) light.rectangle.center += offset;
  for (auto& armor : result.armors) {
    armor.center += offset;
    for (auto& point : armor.corners) point += offset;
  }
  // 未搜索区域保持黑色，掩膜尺寸与 CameraInfo、调试图和原输入一致。
  for (auto* mask : {&result.blue_mask, &result.red_mask}) {
    cv::Mat full = cv::Mat::zeros(size, CV_8UC1);
    mask->copyTo(full(cv::Rect(origin, mask->size())));
    *mask = std::move(full);
  }
}

}  // namespace

// 传统 ROI 不创建推理 Session，不依赖编号分数或下游三维跟踪。
struct TraditionalRoiDetector::Impl {
  TraditionalDetector detector;
  RoiPlanner roi;
  RoiDebugState debug;
  Impl(const TraditionalParams& detector_params, const RoiParams& roi_params)
      : detector(detector_params), roi(roi_params) {}
};

TraditionalRoiDetector::TraditionalRoiDetector(const TraditionalParams& detector, const RoiParams& roi)
    : impl_(new Impl(detector, roi)) {}
TraditionalRoiDetector::~TraditionalRoiDetector() = default;

// 原算法负责整组校验；调用方只在配置实际变化后清理历史。
void TraditionalRoiDetector::setParams(const TraditionalParams& params) { impl_->detector.setParams(params); }

// 先裁剪检测，再恢复原图坐标和掩膜，最后执行区域确认与候选过滤。
DetectionResult TraditionalRoiDetector::detect(const cv::Mat& bgr, double stamp_s, const std::string& frame_id) {
  try {
    if (bgr.empty() || bgr.type() != CV_8UC3)
      throw std::invalid_argument("Traditional ROI expects a non-empty CV_8UC3 BGR image");
    impl_->debug = impl_->roi.begin(bgr.size(), stamp_s, frame_id);
    auto result = impl_->detector.detect(bgr(impl_->debug.crop));
    if (!impl_->debug.enabled) return result;
    restoreCoordinates(result, impl_->debug.crop.tl(), bgr.size());
    result.armors = impl_->roi.finish(result.armors);
    // 详细调试图标记已通过几何筛选、但还未通过 ROI 确认的配对。
    for (auto& pair : result.pairs) {
      if (pair.reason != "accepted") continue;
      const auto center = (result.lights[pair.left].center + result.lights[pair.right].center) * 0.5F;
      const bool accepted = std::any_of(result.armors.begin(), result.armors.end(), [&](const ArmorCandidate& armor) {
        return cv::norm(armor.center - center) < 0.001 && armor.is_red == result.lights[pair.left].is_red;
      });
      if (!accepted) pair.reason = impl_->debug.full_search ? "roi_unconfirmed" : "roi_outside";
    }
    return result;
  } catch (...) {
    resetHistory();
    throw;
  }
}

// 空输入或处理失败后也不沿用上一帧的搜索区域。
void TraditionalRoiDetector::resetHistory() { impl_->roi.reset(); impl_->debug = {}; }
const RoiDebugState& TraditionalRoiDetector::debugState() const { return impl_->debug; }

}  // namespace ft_detector
