#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "ft_detector/pnp_solver.hpp"
#include "ft_detector/traditional_detector.hpp"

namespace ft_detector {

// 返回独立的标注图，绝不在 toCvShare 的输入像素上直接绘制。
cv::Mat drawDetections(const cv::Mat& bgr, const DetectionResult& result, bool details);

// 在 drawDetections 返回的独立副本上叠加，不再复制整张图。
void drawPoses(cv::Mat& debug, const std::vector<PnpResult>& poses,
               const std::string& status, bool details);

}  // namespace ft_detector
