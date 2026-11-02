#include "ft_detector/pnp_solver.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <stdexcept>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace ft_detector {
namespace {

bool finiteVector(const cv::Vec3d& vector) {
  return std::isfinite(vector[0]) && std::isfinite(vector[1]) && std::isfinite(vector[2]);
}

bool validDimensions(const ArmorDimensions& dimensions) {
  return std::isfinite(dimensions.width_m) && std::isfinite(dimensions.height_m) &&
         dimensions.width_m >= 0.0 && dimensions.height_m >= 0.0 &&
         (!dimensions.confirmed || (dimensions.width_m > 0.0 && dimensions.height_m > 0.0));
}

}  // namespace

PnpSolver::PnpSolver(const PnpParams& params) : params_(params) {
  const double numbers[] = {params.min_depth_m, params.max_depth_m,
      params.max_reprojection_error_px, params.min_quad_area_px2,
      params.min_edge_length_px, params.axis_length_m};
  if (!validDimensions(params.small) || !validDimensions(params.large) ||
      !std::all_of(std::begin(numbers), std::end(numbers),
                   [](double value) { return std::isfinite(value) && value > 0.0; }) ||
      params.max_depth_m <= params.min_depth_m ||
      !std::isfinite(params.ambiguity_error_margin_px) || params.ambiguity_error_margin_px < 0.0) {
    throw std::invalid_argument("Invalid PnP dimensions or validation parameters");
  }
}

bool PnpSolver::validCalibration(const CameraCalibration& camera) {
  const auto& k = camera.camera_matrix;
  if (camera.image_size.width <= 0 || camera.image_size.height <= 0 ||
      !std::all_of(std::begin(k.val), std::end(k.val),
                   [](double value) { return std::isfinite(value); }) ||
      k(0, 0) <= 0.0 || k(1, 1) <= 0.0 ||
      std::abs(k(0, 1)) > 1e-9 || std::abs(k(1, 0)) > 1e-9 ||
      std::abs(k(2, 0)) > 1e-9 || std::abs(k(2, 1)) > 1e-9 || std::abs(k(2, 2) - 1.0) > 1e-9 ||
      !std::all_of(camera.distortion.begin(), camera.distortion.end(),
                   [](double value) { return std::isfinite(value); })) return false;
  // 不把 equidistant 的四个系数当成普通针孔畸变来使用。
  const auto size = camera.distortion.size();
  return (camera.distortion_model == "plumb_bob" && (size == 0 || size == 4 || size == 5)) ||
         (camera.distortion_model == "rational_polynomial" && size == 8);
}

std::vector<cv::Point3d> PnpSolver::objectPoints(bool is_large) const {
  const auto& dimensions = is_large ? params_.large : params_.small;
  if ((!dimensions.confirmed && !params_.allow_provisional_dimensions) ||
      dimensions.width_m <= 0.0 || dimensions.height_m <= 0.0)
    throw std::invalid_argument("Positive endpoint dimensions must be confirmed or explicitly allowed as provisional");
  const double half_width = dimensions.width_m / 2.0;
  const double half_height = dimensions.height_m / 2.0;
  // 原点在两灯中心连线中点，X 向右、Y 向下、Z=X×Y 指向板背后，单位米。
  return {{-half_width, -half_height, 0.0}, {half_width, -half_height, 0.0},
          {half_width, half_height, 0.0}, {-half_width, half_height, 0.0}};
}

PnpResult PnpSolver::solve(const ArmorCandidate& armor, const CameraCalibration& camera,
                          const ArmorPose* reference) const {
  PnpResult result;
  result.pose.id = armor.id;
  const auto& dimensions = armor.is_large_armor ? params_.large : params_.small;
  if (!dimensions.confirmed && (!params_.allow_provisional_dimensions ||
      dimensions.width_m <= 0.0 || dimensions.height_m <= 0.0)) {
    result.reason = "dimensions_unconfirmed"; return result;
  }
  result.provisional = !dimensions.confirmed;
  if (!validCalibration(camera)) { result.reason = "camera_model"; return result; }

  const std::vector<cv::Point2f> pixels(armor.corners.begin(), armor.corners.end());
  for (const auto& pixel : pixels) {
    if (!std::isfinite(pixel.x) || !std::isfinite(pixel.y) || pixel.x < 0.0F || pixel.y < 0.0F ||
        pixel.x >= camera.image_size.width || pixel.y >= camera.image_size.height) {
      result.reason = "corner_bounds"; return result;
    }
  }
  // TL、TR、BR、BL：不重排错误输入，避免悄悄改变像素点的物理对应。
  if (!cv::isContourConvex(pixels) || cv::contourArea(pixels, true) < params_.min_quad_area_px2 ||
      pixels[0].y >= pixels[3].y || pixels[1].y >= pixels[2].y ||
      pixels[0].x + pixels[3].x >= pixels[1].x + pixels[2].x) {
    result.reason = "corner_order"; return result;
  }
  for (std::size_t i = 0; i < pixels.size(); ++i) {
    if (cv::norm(pixels[i] - pixels[(i + 1) % 4]) < params_.min_edge_length_px) {
      result.reason = "short_edge"; return result;
    }
  }

  const auto objects = objectPoints(armor.is_large_armor);
  const std::vector<cv::Point2d> image_points(pixels.begin(), pixels.end());
  std::vector<cv::Mat> rotations, translations;
  try {
    // 平面目标可能有两个解；不用仅适用于特定正方形点序的 IPPE_SQUARE。
    const int count = cv::solvePnPGeneric(objects, image_points, camera.camera_matrix, camera.distortion,
                                         rotations, translations, false, cv::SOLVEPNP_IPPE);
    if (count <= 0) { result.reason = "solver_failed"; return result; }
    double best_error = std::numeric_limits<double>::infinity();
    std::vector<ArmorPose> candidates;
    for (std::size_t i = 0; i < rotations.size(); ++i) {
      const cv::Vec3d rvec(rotations[i].at<double>(0), rotations[i].at<double>(1), rotations[i].at<double>(2));
      const cv::Vec3d tvec(translations[i].at<double>(0), translations[i].at<double>(1), translations[i].at<double>(2));
      if (!finiteVector(rvec) || !finiteVector(tvec)) continue;
      cv::Matx33d rotation;
      cv::Rodrigues(rvec, rotation);
      bool depth_valid = tvec[2] >= params_.min_depth_m && tvec[2] <= params_.max_depth_m;
      for (const auto& point : objects) {
        const cv::Vec3d transformed = rotation * cv::Vec3d(point.x, point.y, point.z) + tvec;
        depth_valid = depth_valid && finiteVector(transformed) &&
                      transformed[2] >= params_.min_depth_m && transformed[2] <= params_.max_depth_m;
      }
      const cv::Vec3d normal(rotation(0, 2), rotation(1, 2), rotation(2, 2));
      if (!depth_valid || normal.dot(tvec) <= 0.0) continue;
      std::vector<cv::Point2d> reprojection;
      cv::projectPoints(objects, rvec, tvec, camera.camera_matrix, camera.distortion, reprojection);
      double squared_error = 0.0;
      for (std::size_t j = 0; j < image_points.size(); ++j) {
        const auto difference = reprojection[j] - image_points[j];
        squared_error += difference.dot(difference);
      }
      const double rms = std::sqrt(squared_error / image_points.size());
      if (!std::isfinite(rms)) continue;
      ArmorPose pose;
      pose.id = armor.id;
      pose.rvec = rvec;
      pose.tvec = tvec;
      pose.reprojection_error_px = rms;
      candidates.push_back(pose);
      if (rms < best_error) {
        best_error = rms;
        result.pose = pose;
      }
    }
    if (!std::isfinite(best_error)) { result.reason = "no_positive_solution"; return result; }
    if (best_error > params_.max_reprojection_error_px) { result.reason = "reprojection"; return result; }

    // 历史只帮助当前合格且误差接近的候选消歧，不能绕过绝对误差门限。
    if (reference && finiteVector(reference->rvec) && finiteVector(reference->tvec)) {
      cv::Matx33d previous_rotation;
      cv::Rodrigues(reference->rvec, previous_rotation);
      double closest_angle = std::numeric_limits<double>::infinity();
      for (const auto& pose : candidates) {
        if (pose.reprojection_error_px > params_.max_reprojection_error_px ||
            pose.reprojection_error_px > best_error + params_.ambiguity_error_margin_px) continue;
        cv::Matx33d rotation;
        cv::Rodrigues(pose.rvec, rotation);
        const auto relative = previous_rotation.t() * rotation;
        const double cosine = (relative(0, 0) + relative(1, 1) + relative(2, 2) - 1.0) / 2.0;
        const double angle = std::acos(std::max(-1.0, std::min(1.0, cosine)));
        if (angle < closest_angle) {
          closest_angle = angle;
          result.pose = pose;
        }
      }
      result.continuity_selected = result.pose.reprojection_error_px > best_error;
    }

    const double axis = params_.axis_length_m;
    const std::vector<cv::Point3d> endpoints{{0, 0, 0}, {axis, 0, 0}, {0, axis, 0}, {0, 0, axis}};
    cv::Matx33d rotation;
    cv::Rodrigues(result.pose.rvec, rotation);
    for (const auto& point : endpoints) {
      const auto transformed = rotation * cv::Vec3d(point.x, point.y, point.z) + result.pose.tvec;
      if (!finiteVector(transformed) || transformed[2] <= 0.0) {
        result.reason = "axis_depth"; return result;
      }
    }
    std::vector<cv::Point2d> axis_pixels;
    cv::projectPoints(endpoints, result.pose.rvec, result.pose.tvec,
                      camera.camera_matrix, camera.distortion, axis_pixels);
    const double drawing_limit = std::numeric_limits<int>::max() / 4.0;
    for (std::size_t i = 0; i < axis_pixels.size(); ++i) {
      if (!std::isfinite(axis_pixels[i].x) || !std::isfinite(axis_pixels[i].y) ||
          std::abs(axis_pixels[i].x) > drawing_limit || std::abs(axis_pixels[i].y) > drawing_limit) {
        result.reason = "axis_projection"; return result;
      }
      result.pose.axis_pixels[i] = axis_pixels[i];
    }
    result.valid = true;
    result.reason = "accepted";
  } catch (const cv::Exception&) {
    result.reason = "solver_error";
  }
  return result;
}

}  // namespace ft_detector
