#pragma once

#include <opencv2/core.hpp>

#include "ft_detector/traditional_detector.hpp"

namespace ft_detector {

// 返回独立的标注图，绝不在 toCvShare 的输入像素上直接绘制。
cv::Mat drawDetections(const cv::Mat& bgr, const DetectionResult& result, bool details);

}  // namespace ft_detector
