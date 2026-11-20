#include "ft_selector/observation_noise.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <opencv2/calib3d.hpp>

namespace ft_selector {
namespace {
bool positive(double value) { return std::isfinite(value) && value > 0.0; }
bool validStrength(const NoiseStrength& strength) {
  return positive(strength.confidence_reference) && strength.confidence_reference <= 1.0 &&
      std::isfinite(strength.position) && strength.position >= 0.0 &&
      std::isfinite(strength.yaw) && strength.yaw >= 0.0;
}
}  // namespace

// 错误的启动配置直接报错；单帧质量信息缺失则由计算入口回退固定 R。
void validateObservationNoise(const ObservationNoiseParams& params) {
  if (!positive(params.error_reference_px) || !positive(params.distance_reference_m) ||
      !std::isfinite(params.max_multiplier) || params.max_multiplier < 1.0 || params.max_multiplier > 25.0 ||
      !validStrength(params.nn) || !validStrength(params.tradition))
    throw std::invalid_argument("Invalid adaptive observation noise references, strengths or bound");
}

// 使用与检测 PnP 相同的针孔模型、K/D 条件，不把畸变坐标当成去畸变坐标。
bool validQualityCalibration(const QualityCalibration& camera) {
  const auto count = camera.distortion.size();
  if (camera.image_size.empty() ||
      !((camera.distortion_model == "plumb_bob" && (count == 0 || count == 4 || count == 5)) ||
        (camera.distortion_model == "rational_polynomial" && count == 8))) return false;
  for (double value : camera.matrix.val) if (!std::isfinite(value)) return false;
  for (double value : camera.distortion) if (!std::isfinite(value)) return false;
  return camera.matrix(0,0) > 0.0 && camera.matrix(1,1) > 0.0 &&
      std::abs(camera.matrix(0,1)) < 1e-9 && std::abs(camera.matrix(1,0)) < 1e-9 &&
      std::abs(camera.matrix(2,0)) < 1e-9 && std::abs(camera.matrix(2,1)) < 1e-9 &&
      std::abs(camera.matrix(2,2) - 1.0) < 1e-9;
}

// 直接重投影已发布的 Pose，四点采用定义一；这里不再次执行 PnP。
ObservationQuality measureObservationQuality(const Eigen::Vector3d& position,
    const Eigen::Quaterniond& orientation, const std::array<cv::Point2d,4>& corners,
    double width_m, double height_m, double confidence, const QualityCalibration& camera) {
  ObservationQuality quality;
  if (!validQualityCalibration(camera) || !position.allFinite() || position.z() <= 0.0 ||
      !orientation.coeffs().allFinite() ||
      !positive(width_m) || !positive(height_m) || !std::isfinite(confidence) || confidence < 0.0 || confidence > 1.0)
    return quality;
  // 长度只计算一次，校验后同时用于归一化和距离质量项。
  const double orientation_norm = orientation.norm();
  const double distance_m = position.norm();
  if (!std::isfinite(orientation_norm) || orientation_norm < 1e-6 || !positive(distance_m)) return quality;
  for (const auto& point : corners) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || point.x < 0.0 || point.y < 0.0 ||
        point.x >= camera.image_size.width || point.y >= camera.image_size.height) return quality;
  }
  try {
    const Eigen::Quaterniond normalized_orientation(orientation.coeffs() / orientation_norm);
    const Eigen::Matrix3d rotation = normalized_orientation.toRotationMatrix();
    cv::Matx33d matrix;
    for (int row = 0; row < 3; ++row)
      for (int col = 0; col < 3; ++col) matrix(row,col) = rotation(row,col);
    const std::vector<cv::Point3d> objects{{-width_m/2,-height_m/2,0}, {width_m/2,-height_m/2,0},
                                         {width_m/2,height_m/2,0}, {-width_m/2,height_m/2,0}};
    // 任何端点在相机后方，都不能作为有效的重投影质量依据。
    for (const auto& point : objects)
      if ((rotation * Eigen::Vector3d(point.x,point.y,point.z) + position).z() <= 0.0) return quality;
    cv::Vec3d rvec;
    cv::Rodrigues(matrix, rvec);
    std::vector<cv::Point2d> projected;
    cv::projectPoints(objects, rvec, cv::Vec3d(position.x(),position.y(),position.z()),
                      camera.matrix, camera.distortion, projected);
    double squared_error = 0.0;
    for (std::size_t index = 0; index < corners.size(); ++index) {
      const auto delta = projected[index] - corners[index];
      squared_error += delta.dot(delta);
    }
    quality.error_px = std::sqrt(squared_error / corners.size());
    quality.distance_m = distance_m;
    quality.confidence = confidence;
    const Eigen::Vector3d outward = -rotation.col(2);
    quality.facing = std::clamp(outward.dot(-position / distance_m), -1.0, 1.0);
    quality.valid = std::isfinite(quality.error_px) && std::isfinite(quality.facing);
  } catch (const cv::Exception&) {
    return {};
  }
  return quality;
}

// 位置的侧向不确定度与角度的正视歧义分别加权，并限制为基础方差的 1～25 倍。
Eigen::Vector4d observationVarianceMultipliers(const ObservationQuality& q,
    const std::string& mode, const ObservationNoiseParams& params) {
  if (!q.valid || (mode != "nn" && mode != "tradition") ||
      !std::isfinite(q.error_px) || q.error_px < 0.0 || !std::isfinite(q.confidence) ||
      q.confidence < 0.0 || q.confidence > 1.0 || !positive(q.distance_m) ||
      !std::isfinite(q.facing) || std::abs(q.facing) > 1.0) return Eigen::Vector4d::Ones();
  const auto& strength = mode == "nn" ? params.nn : params.tradition;
  const double error = q.error_px / params.error_reference_px;
  const double confidence = std::max(1.0, strength.confidence_reference / std::max(q.confidence, 0.2));
  const double base = (1.0 + error * error) * confidence * confidence;
  const double distance = std::max(1.0, q.distance_m / params.distance_reference_m);
  const double u2 = distance * distance, f2 = q.facing * q.facing;
  const double side = 1.0 + 2.0 * (1.0 - f2), front = 1.0 + f2;
  const auto scale = [&](double raw, double lambda) {
    if (lambda == 0.0) return 1.0;  // 也避免极端输入下出现 0×∞。
    return std::clamp(1.0 + lambda * (raw - 1.0), 1.0, params.max_multiplier);
  };
  const double xy = scale(base * u2 * side, strength.position);
  return {xy, xy, scale(base * u2 * u2 * side, strength.position), scale(base * u2 * front, strength.yaw)};
}

}  // namespace ft_selector
