#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "ft_detector/armor_candidate.hpp"

namespace ft_detector {

struct ArmorDimensions {
  double width_m, height_m;  // 灯条中心距 W、发光长度 L；不是板本体宽高。
};

struct PnpParams {
  ArmorDimensions small, large;
  double min_depth_m, max_depth_m;
  double max_reprojection_error_px;
  double min_quad_area_px2, min_edge_length_px;
  double axis_length_m;
  double ambiguity_error_margin_px = 0.0;  // 有参考时允许比当前最优 RMS 多出的像素误差。
};

// 原始畸变图像的标定；不使用投影矩阵 P 代替 K。
struct CameraCalibration {
  cv::Matx33d camera_matrix;
  std::vector<double> distortion;
  cv::Size image_size;
  std::string distortion_model;
};

struct ArmorPose {
  std::uint32_t id = 0;  // 与该帧候选的 id 相同。
  cv::Vec3d rvec{0.0, 0.0, 0.0}, tvec{0.0, 0.0, 0.0};
  double reprojection_error_px = 0.0;
  std::array<cv::Point2d, 4> axis_pixels{};  // 原点、+X、+Y、+Z 的投影。
};

struct PnpResult {
  bool valid = false;
  bool continuity_selected = false;  // 参考历史姿态，选中了误差略大的当前帧合格分支。
  std::string reason;
  ArmorPose pose;
};

class PnpSolver {
 public:
  explicit PnpSolver(const PnpParams& params);
  static bool validCalibration(const CameraCalibration& camera);
  std::vector<cv::Point3d> objectPoints(bool is_large) const;
  PnpResult solve(const ArmorCandidate& armor, const CameraCalibration& camera,
                   const ArmorPose* reference = nullptr) const;

 private:
  PnpParams params_;
};

}  // namespace ft_detector
