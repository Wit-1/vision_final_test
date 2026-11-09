#include "ft_detector/pose_history.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ft_detector {
namespace {

// 用两侧灯条的平均长度作为像素位移的归一化尺度。
double meanLightLength(const ArmorCandidate& armor) {
  return (cv::norm(armor.corners[0] - armor.corners[3]) +
          cv::norm(armor.corners[1] - armor.corners[2])) / 2.0;
}

// 同色、同大小且四点位移足够小，才允许当前候选借用历史位姿。
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

// 保存时间与几何门限，并拒绝无效配置。
PoseHistory::PoseHistory(double max_gap_s, double max_corner_shift_ratio)
    : max_gap_s_(max_gap_s), max_corner_shift_ratio_(max_corner_shift_ratio) {
  if (!std::isfinite(max_gap_s) || max_gap_s <= 0.0 ||
      !std::isfinite(max_corner_shift_ratio) || max_corner_shift_ratio <= 0.0) {
    throw std::invalid_argument("Pose history time and corner gates must be finite and positive");
  }
}

// 清空上一帧参考，使下一次 PnP 回到无历史的选解方式。
void PoseHistory::reset() {
  entries_.clear();
  stamp_s_ = std::numeric_limits<double>::quiet_NaN();
}

// 为当前候选寻找双方唯一的历史匹配；歧义项返回空指针。
std::vector<const ArmorPose*> PoseHistory::references(const std::vector<ArmorCandidate>& armors,
                                                     double stamp_s) {
  std::vector<const ArmorPose*> result(armors.size(), nullptr);
  const double dt = stamp_s - stamp_s_;
  // 重复、回退或相隔过久的时间不能继续使用上一帧姿态。
  if (!std::isfinite(stamp_s) || !std::isfinite(dt) || dt <= 0.0 || dt > max_gap_s_) {
    reset();
    return result;
  }
  std::vector<std::size_t> previous_counts(entries_.size(), 0);
  std::vector<std::size_t> current_counts(armors.size(), 0);
  // 保存历史数组下标，不使用逐帧重新编号的 armor.id。
  std::vector<std::size_t> previous_indices(armors.size(), 0);
  // 统计两侧各有几个可匹配对象，而非仅取距离最近的一项。
  for (std::size_t i = 0; i < armors.size(); ++i) {
    for (std::size_t j = 0; j < entries_.size(); ++j) {
      if (!compatible(armors[i], entries_[j].armor, max_corner_shift_ratio_)) continue;
      ++current_counts[i];
      ++previous_counts[j];
      previous_indices[i] = j;
    }
  }
  for (std::size_t i = 0; i < armors.size(); ++i) {
    // 只有当前项与历史项都恰好匹配一次，才提供参考。
    if (current_counts[i] != 1) continue;
    const auto previous_index = previous_indices[i];
    if (previous_counts[previous_index] == 1) result[i] = &entries_[previous_index].pose;
  }
  return result;
}

// 整帧解算结束后统一替换历史，仅保存当前帧的有效位姿。
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
