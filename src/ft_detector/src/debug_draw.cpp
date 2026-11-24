#include "ft_detector/debug_draw.hpp"

#include <algorithm>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace ft_detector {

// 在原图副本上绘制检测结果，避免改动 toCvShare 共享的输入像素。
cv::Mat drawDetections(const cv::Mat& bgr, const DetectionResult& result, bool details) {
  cv::Mat debug = bgr.clone();
  if (details) {
    // 详细模式先展示灯条、配对和拒绝原因，便于解释筛选过程。
    for (const auto& light : result.rejected_lights) {
      const auto box = light.rectangle.boundingRect();
      cv::rectangle(debug, box, cv::Scalar(80, 80, 80), 1);
      cv::putText(debug, light.reason, box.tl(), cv::FONT_HERSHEY_SIMPLEX, 0.35,
                  cv::Scalar(150, 150, 150), 1);
    }
    for (std::size_t i = 0; i < result.lights.size(); ++i) {
      const auto& light = result.lights[i];
      cv::line(debug, light.top, light.bottom, cv::Scalar(0, 255, 255), 2);
      cv::putText(debug, "L" + std::to_string(i), light.top, cv::FONT_HERSHEY_SIMPLEX,
                  0.4, cv::Scalar(0, 255, 255), 1);
    }
    for (const auto& pair : result.pairs) {
      if (pair.reason == "accepted" || pair.reason == "color") continue;
      const auto first = result.lights[pair.left].center;
      const auto second = result.lights[pair.right].center;
      cv::line(debug, first, second, cv::Scalar(100, 100, 100), 1);
      cv::putText(debug, pair.reason, (first + second) * 0.5F, cv::FONT_HERSHEY_SIMPLEX,
                  0.35, cv::Scalar(180, 180, 180), 1);
    }
  }
  // 绘制所有候选的框、真实角点与角点序号，文字偏移只影响显示。
  for (const auto& armor : result.armors) {
    const cv::Scalar color = armor.is_red ? cv::Scalar(0, 80, 255) : cv::Scalar(255, 180, 0);
    const cv::Point2f offsets[] = {{-10, -3}, {4, -3}, {4, 13}, {-10, 13}};
    for (std::size_t i = 0; i < armor.corners.size(); ++i) {
      cv::line(debug, armor.corners[i], armor.corners[(i + 1) % 4], color, 2, cv::LINE_AA);
      cv::circle(debug, armor.corners[i], 3, cv::Scalar(0, 255, 0), -1);
      cv::putText(debug, std::to_string(i), armor.corners[i] + offsets[i],
                  cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(230, 230, 230), 1, cv::LINE_AA);
    }
    // 标签区分帧内 ID、颜色、大小和 NN 识别出的编号类别。
    std::string label = "ID " + std::to_string(armor.id) +
                              (armor.is_red ? " RED " : " BLUE ") +
                              (armor.is_large_armor ? "LARGE" : "SMALL");
    if (armor.number_class >= 0 && armor.number_class < 9) {
      const char* names[] = {"SENTRY", "1", "2", "3", "4", "5", "OUTPOST", "BASE", "BASE"};
      std::ostringstream score;
      score << std::fixed << std::setprecision(2) << armor.confidence;
      label += " #" + std::string(names[armor.number_class]) + " c=" + score.str();
      label += armor.size_from_number ? " size:number" : " size:geometry";
    }
    const auto box = cv::boundingRect(std::vector<cv::Point2f>(armor.corners.begin(), armor.corners.end()));
    cv::putText(debug, label, cv::Point(box.x, std::max(18, box.y - 8)),
                cv::FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv::LINE_AA);
  }
  // 显示检测候选总数，此时尚未按 PnP 成功与否过滤。
  cv::putText(debug, "Armors: " + std::to_string(result.armors.size()), cv::Point(12, 25),
              cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(0, 255, 0), 2);
  return debug;
}

// 叠加位姿成功数、投影坐标轴、深度和重投影误差。
void drawPoses(cv::Mat& debug, const std::vector<PnpResult>& poses,
               const std::string& status, bool details) {
  const auto valid = std::count_if(poses.begin(), poses.end(), [](const PnpResult& pose) { return pose.valid; });
  cv::putText(debug, "PnP: " + status + " " + std::to_string(valid) + "/" + std::to_string(poses.size()),
              cv::Point(12, 50), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 255), 1);
  int reason_y = 70;
  for (const auto& result : poses) {
    // 无效位姿只显示原因，不绘制可能误导的坐标轴。
    if (!result.valid) {
      if (details) {
        cv::putText(debug, "ID " + std::to_string(result.pose.id) + ": " + result.reason,
                    cv::Point(12, reason_y), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 200, 255), 1);
        reason_y += 18;
      }
      continue;
    }
    // 三条轴来自同一 PnP 解，颜色依次为 X 红、Y 绿、Z 蓝。
    const auto& pose = result.pose;
    const cv::Point origin = pose.axis_pixels[0];
    const cv::Scalar colors[] = {cv::Scalar(0, 0, 255), cv::Scalar(0, 255, 0), cv::Scalar(255, 0, 0)};
    for (std::size_t i = 0; i < 3; ++i) {
      cv::line(debug, origin, pose.axis_pixels[i + 1], colors[i], 2, cv::LINE_AA);
    }
    // 显示深度、重投影误差和历史选解状态。
    std::ostringstream label;
    label << std::fixed << std::setprecision(2) << "Z=" << pose.tvec[2]
          << "m e=" << pose.reprojection_error_px << "px";
    if (result.continuity_selected) label << " (prior)";
    cv::putText(debug, label.str(), origin + cv::Point(0, 22), cv::FONT_HERSHEY_SIMPLEX,
                0.45, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
  }
}

}  // namespace ft_detector
