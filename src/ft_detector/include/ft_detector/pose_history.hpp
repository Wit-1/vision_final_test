#pragma once

#include <limits>
#include <vector>

#include "ft_detector/pnp_solver.hpp"

namespace ft_detector {

// 仅保存上一帧合格位姿，供平面双解消歧使用；不产生跨帧 ID 或运动预测。
class PoseHistory {
 public:
  PoseHistory(double max_gap_s, double max_corner_shift_ratio);
  // 颜色、大小、四点距离门控内必须形成唯一的一对一关系。
  // 返回指针在 store/reset 前有效，调用方须先完成整帧求解，再更新历史。
  std::vector<const ArmorPose*> references(const std::vector<ArmorCandidate>& armors, double stamp_s);
  void store(const std::vector<ArmorCandidate>& armors,
             const std::vector<PnpResult>& poses, double stamp_s);
  void reset();

 private:
  struct Entry {
    ArmorCandidate armor;
    ArmorPose pose;
  };
  double max_gap_s_, max_corner_shift_ratio_;
  double stamp_s_ = std::numeric_limits<double>::quiet_NaN();
  std::vector<Entry> entries_;
};

}  // namespace ft_detector
