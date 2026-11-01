#include "ft_detector/traditional_detector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <iterator>
#include <stdexcept>
#include <tuple>
#include <utility>

#include <opencv2/imgproc.hpp>

namespace ft_detector {
namespace {

void validateHsv(const HsvRange& range) {
  if (range.h_min < 0 || range.h_max > 179 || range.h_min > range.h_max ||
      range.s_min < 0 || range.s_max > 255 || range.s_min > range.s_max ||
      range.v_min < 0 || range.v_max > 255 || range.v_min > range.v_max) {
    throw std::invalid_argument("Invalid HSV interval (H: 0..179, S/V: 0..255)");
  }
}

cv::Mat segment(const cv::Mat& hsv, const HsvRange& range) {
  cv::Mat mask;
  cv::inRange(hsv, cv::Scalar(range.h_min, range.s_min, range.v_min),
              cv::Scalar(range.h_max, range.s_max, range.v_max), mask);
  return mask;
}

bool sharesLight(const PairDecision& first, const PairDecision& second) {
  return first.left == second.left || first.left == second.right ||
         first.right == second.left || first.right == second.right;
}

}  // namespace

double normalizeLightAngle(double angle_deg) {
  double value = std::fmod(angle_deg + 90.0, 180.0);
  if (value < 0.0) value += 180.0;
  return value - 90.0;
}

double lightAngleDifference(double first_deg, double second_deg) {
  return std::abs(normalizeLightAngle(first_deg - second_deg));
}

TraditionalDetector::TraditionalDetector(const TraditionalParams& params) { setParams(params); }

void TraditionalDetector::setParams(const TraditionalParams& params) {
  validateHsv(params.blue);
  validateHsv(params.red_low);
  validateHsv(params.red_high);
  const double numbers[] = {
      params.min_contour_area, params.min_light_length, params.min_light_aspect,
      params.max_light_aspect, params.min_light_fill, params.max_light_tilt_deg,
      params.min_pair_length_ratio, params.max_pair_angle_deg, params.max_pair_center_dy_ratio,
      params.min_pair_spacing_ratio, params.max_pair_spacing_ratio,
      params.large_armor_min_spacing_ratio, params.intervening_light_max_offset_ratio,
      params.pair_ambiguity_margin};
  if (!std::all_of(std::begin(numbers), std::end(numbers),
                   [](double number) { return std::isfinite(number); }) ||
      params.min_contour_area <= 0.0 || params.min_light_length <= 0.0 ||
      params.min_light_aspect <= 1.0 || params.max_light_aspect < params.min_light_aspect ||
      params.min_light_fill <= 0.0 || params.min_light_fill > 1.0 ||
      params.max_light_tilt_deg <= 0.0 || params.max_light_tilt_deg >= 90.0 ||
      params.border_margin < 0 || params.min_pair_length_ratio <= 0.0 ||
      params.min_pair_length_ratio > 1.0 || params.max_pair_angle_deg <= 0.0 ||
      params.max_pair_angle_deg > 90.0 || params.max_pair_center_dy_ratio <= 0.0 ||
      params.min_pair_spacing_ratio <= 0.0 ||
      params.large_armor_min_spacing_ratio <= params.min_pair_spacing_ratio ||
      params.max_pair_spacing_ratio <= params.large_armor_min_spacing_ratio ||
      params.intervening_light_max_offset_ratio <= 0.0 || params.pair_ambiguity_margin < 0.0) {
    throw std::invalid_argument("Invalid traditional detector geometry parameters");
  }
  params_ = params;
}

DetectionResult TraditionalDetector::detect(const cv::Mat& bgr) const {
  if (bgr.empty() || bgr.type() != CV_8UC3) {
    throw std::invalid_argument("TraditionalDetector expects a non-empty CV_8UC3 BGR image");
  }
  cv::Mat hsv;
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
  DetectionResult result;
  result.blue_mask = segment(hsv, params_.blue);
  cv::bitwise_or(segment(hsv, params_.red_low), segment(hsv, params_.red_high), result.red_mask);
  // 不做闭运算补断条：不把遮挡后的两段强行接成完整灯条。
  findLights(result.blue_mask, false, result);
  findLights(result.red_mask, true, result);
  std::sort(result.lights.begin(), result.lights.end(), [](const LightBar& a, const LightBar& b) {
    return std::tie(a.center.x, a.center.y, a.is_red) < std::tie(b.center.x, b.center.y, b.is_red);
  });
  pairLights(result);
  return result;
}

void TraditionalDetector::findLights(const cv::Mat& mask, bool is_red, DetectionResult& result) const {
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  for (const auto& contour : contours) {
    const auto rectangle = cv::minAreaRect(contour);
    const double area = cv::contourArea(contour);
    cv::Point2f vertices[4];
    rectangle.points(vertices);
    cv::Point2f direction = vertices[1] - vertices[0];
    const cv::Point2f adjacent = vertices[2] - vertices[1];
    if (cv::norm(adjacent) > cv::norm(direction)) direction = adjacent;
    const double length = cv::norm(direction);
    const double width = std::min(rectangle.size.width, rectangle.size.height);
    std::string reason;
    if (area < params_.min_contour_area) reason = "area";
    else if (length < params_.min_light_length || width <= 0.0) reason = "length";
    else if (length / width < params_.min_light_aspect || length / width > params_.max_light_aspect)
      reason = "aspect";
    else if (area / (length * width) < params_.min_light_fill) reason = "fill";
    if (!reason.empty()) {
      result.rejected_lights.push_back({rectangle, reason});
      continue;
    }

    direction *= static_cast<float>(1.0 / length);
    if (direction.y < 0.0F || (direction.y == 0.0F && direction.x < 0.0F)) direction *= -1.0F;
    // 从长边向量计算角度，不依赖 RotatedRect::angle 的版本约定。
    const double angle = normalizeLightAngle(std::atan2(direction.x, direction.y) * 180.0 / CV_PI);
    if (std::abs(angle) > params_.max_light_tilt_deg) reason = "tilt";
    for (const auto& point : vertices) {
      if (point.x <= params_.border_margin || point.y <= params_.border_margin ||
          point.x >= mask.cols - 1 - params_.border_margin ||
          point.y >= mask.rows - 1 - params_.border_margin) reason = "border";
    }
    if (!reason.empty()) {
      result.rejected_lights.push_back({rectangle, reason});
      continue;
    }
    LightBar light;
    light.rectangle = rectangle;
    light.center = rectangle.center;
    light.top = light.center - direction * static_cast<float>(length / 2.0);
    light.bottom = light.center + direction * static_cast<float>(length / 2.0);
    light.length = length;
    light.width = width;
    light.angle_deg = angle;
    light.is_red = is_red;
    result.lights.push_back(light);
  }
}

void TraditionalDetector::pairLights(DetectionResult& result) const {
  for (std::size_t i = 0; i < result.lights.size(); ++i) {
    for (std::size_t j = i + 1; j < result.lights.size(); ++j) {
      const auto& left = result.lights[i];
      const auto& right = result.lights[j];
      PairDecision pair{i, j, 0.0, 0.0, ""};
      const double mean_length = (left.length + right.length) / 2.0;
      const double length_ratio = std::min(left.length, right.length) / std::max(left.length, right.length);
      const double angle_diff = lightAngleDifference(left.angle_deg, right.angle_deg);
      const double dy_ratio = std::abs(left.center.y - right.center.y) / mean_length;
      pair.spacing_ratio = cv::norm(right.center - left.center) / mean_length;
      pair.cost = (1.0 - length_ratio) + dy_ratio + angle_diff / params_.max_pair_angle_deg;
      if (left.is_red != right.is_red) pair.reason = "color";
      else if (right.center.x <= left.center.x) pair.reason = "order";
      else if (length_ratio < params_.min_pair_length_ratio) pair.reason = "length_ratio";
      else if (angle_diff > params_.max_pair_angle_deg) pair.reason = "angle";
      else if (dy_ratio > params_.max_pair_center_dy_ratio) pair.reason = "center_dy";
      else if (pair.spacing_ratio < params_.min_pair_spacing_ratio ||
               pair.spacing_ratio > params_.max_pair_spacing_ratio) pair.reason = "spacing";
      else {
        // 有第三根灯条挡在中间时，不跨过它去拼一个更大的框。
        const cv::Point2f span = right.center - left.center;
        const double span_squared = span.dot(span);
        for (std::size_t k = 0; k < result.lights.size(); ++k) {
          if (k == i || k == j || result.lights[k].is_red != left.is_red) continue;
          const cv::Point2f offset = result.lights[k].center - left.center;
          const double t = offset.dot(span) / span_squared;
          const double distance = cv::norm(offset - span * static_cast<float>(t));
          if (t > 0.0 && t < 1.0 &&
              distance / mean_length < params_.intervening_light_max_offset_ratio) {
            pair.reason = "intervening_light";
            break;
          }
        }
      }
      result.pairs.push_back(std::move(pair));
    }
  }

  // 每根灯条只能属于一块板。竞争评分过近时，放弃歧义配对。
  std::vector<double> best(result.lights.size(), std::numeric_limits<double>::infinity());
  std::vector<std::size_t> eligible;
  for (std::size_t i = 0; i < result.pairs.size(); ++i) {
    const auto& pair = result.pairs[i];
    if (!pair.reason.empty()) continue;
    eligible.push_back(i);
    best[pair.left] = std::min(best[pair.left], pair.cost);
    best[pair.right] = std::min(best[pair.right], pair.cost);
  }
  for (const auto index : eligible) {
    auto& pair = result.pairs[index];
    if (pair.cost > best[pair.left] + params_.pair_ambiguity_margin ||
        pair.cost > best[pair.right] + params_.pair_ambiguity_margin) {
      pair.reason = "shared_light";
      continue;
    }
    const bool ambiguous = std::any_of(eligible.begin(), eligible.end(), [&](std::size_t other) {
      return other != index && sharesLight(pair, result.pairs[other]) &&
             std::abs(pair.cost - result.pairs[other].cost) <= params_.pair_ambiguity_margin;
    });
    if (ambiguous) {
      pair.reason = "ambiguous";
      continue;
    }
    pair.reason = "accepted";
    const auto& left = result.lights[pair.left];
    const auto& right = result.lights[pair.right];
    ArmorCandidate armor;
    armor.is_red = left.is_red;
    armor.is_large_armor = pair.spacing_ratio >= params_.large_armor_min_spacing_ratio;
    armor.corners = {{left.top, right.top, right.bottom, left.bottom}};
    armor.center = (left.center + right.center) * 0.5F;
    armor.confidence = static_cast<float>(1.0 / (1.0 + pair.cost));
    result.armors.push_back(armor);
  }
  std::sort(result.armors.begin(), result.armors.end(), [](const ArmorCandidate& a, const ArmorCandidate& b) {
    return std::tie(a.center.x, a.center.y, a.is_red) < std::tie(b.center.x, b.center.y, b.is_red);
  });
  for (std::size_t i = 0; i < result.armors.size(); ++i) result.armors[i].id = i;
}

}  // namespace ft_detector
