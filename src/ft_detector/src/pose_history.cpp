#include "ft_detector/pose_history.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ft_detector {
namespace {

double meanLightLength(const ArmorCandidate& armor) {
  return (cv::norm(armor.corners[0] - armor.corners[3]) +
          cv::norm(armor.corners[1] - armor.corners[2])) / 2.0;
}

bool compatible(const ArmorCandidate& current, const ArmorCandidate& previous, double max_ratio) {
  if (current.is_red != previous.is_red || current.is_large_armor != previous.is_large_armor) return false;
  const double scale = std::min(meanLightLength(current), meanLightLength(previous));
  if (!std::isfinite(scale) || scale <= 0.0) return false;
  double squared_distance = 0.0;
  for (std::size_t i = 0; i < current.corners.size(); ++i) {
    const auto delta = current.corners[i] - previous.corners[i];
    squared_distance += delta.dot(delta);
  }
  const double rms = std::sqrt(squared_distance / current.corners.size());
  return std::isfinite(rms) && rms / scale <= max_ratio;
}

}  // namespace

PoseHistory::PoseHistory(double max_gap_s, double max_corner_shift_ratio)
    : max_gap_s_(max_gap_s), max_corner_shift_ratio_(max_corner_shift_ratio) {
  if (!std::isfinite(max_gap_s) || max_gap_s <= 0.0 ||
      !std::isfinite(max_corner_shift_ratio) || max_corner_shift_ratio <= 0.0) {
    throw std::invalid_argument("Pose history time and corner gates must be finite and positive");
  }
}

void PoseHistory::reset() {
  entries_.clear();
  stamp_s_ = std::numeric_limits<double>::quiet_NaN();
}

std::vector<const ArmorPose*> PoseHistory::references(const std::vector<ArmorCandidate>& armors,
                                                     double stamp_s) {
  std::vector<const ArmorPose*> result(armors.size(), nullptr);
  const double dt = stamp_s - stamp_s_;
  if (!std::isfinite(stamp_s) || !std::isfinite(dt) || dt <= 0.0 || dt > max_gap_s_) {
    reset();
    return result;
  }
  std::vector<std::size_t> previous_counts(entries_.size(), 0);
  std::vector<std::size_t> current_counts(armors.size(), 0);
  // 保存历史数组下标，不使用逐帧重新编号的 armor.id。
  std::vector<std::size_t> previous_indices(armors.size(), 0);
  for (std::size_t i = 0; i < armors.size(); ++i) {
    for (std::size_t j = 0; j < entries_.size(); ++j) {
      if (!compatible(armors[i], entries_[j].armor, max_corner_shift_ratio_)) continue;
      ++current_counts[i];
      ++previous_counts[j];
      previous_indices[i] = j;
    }
  }
  for (std::size_t i = 0; i < armors.size(); ++i) {
    if (current_counts[i] != 1) continue;
    const auto previous_index = previous_indices[i];
    if (previous_counts[previous_index] == 1) result[i] = &entries_[previous_index].pose;
  }
  return result;
}

void PoseHistory::store(const std::vector<ArmorCandidate>& armors,
                         const std::vector<PnpResult>& poses, double stamp_s) {
  reset();
  if (armors.size() != poses.size()) throw std::invalid_argument("Pose history requires one result per candidate");
  if (!std::isfinite(stamp_s)) return;
  stamp_s_ = stamp_s;
  for (std::size_t i = 0; i < armors.size(); ++i) {
    if (poses[i].valid) entries_.push_back({armors[i], poses[i].pose});
  }
}

}  // namespace ft_detector
