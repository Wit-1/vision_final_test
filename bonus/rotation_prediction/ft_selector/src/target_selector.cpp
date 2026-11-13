#include "ft_selector/target_selector.hpp"

#include <cmath>

namespace ft_selector {
namespace {

// 在模型槽位中选择正深度且最朝向相机的一块；不要求当前图像已检测到它。
int mostFacingSlot(const Eigen::Vector3d& center, double phase, const CircleModel& circle, double minimum) {
  int best_slot = -1;
  double best_facing = minimum;
  for (int slot = 0; slot < circle.plate_count; ++slot) {
    const Eigen::Vector3d position = circle.platePosition(center, phase, slot);
    if (!position.allFinite() || position.z() <= 0.0) continue;
    // 法向与指向相机方向的点积越大，板越正对相机。
    const Eigen::Vector3d to_camera(-position.x(), 0.0, -position.z());
    const double facing = outwardNormal(circle.platePhase(phase, slot)).dot(to_camera.normalized());
    if (facing > best_facing) {
      best_facing = facing;
      best_slot = slot;
    }
  }
  return best_slot;
}

}  // namespace

// 根据当前车体状态构造当前板和 T 秒后的选板结果。
SelectedTarget selectTarget(const VehicleEstimate& vehicle, const CircleModel& circle,
                            double horizon_s, double min_facing_cosine) {
  SelectedTarget target;
  if (!vehicle.id) return target;
  // 兼容原调用接口，但运行时优先使用滤波器输出的估计几何。
  const CircleModel& estimated = vehicle.geometry.plate_count ? vehicle.geometry : circle;

  target.id = vehicle.id;
  target.center = vehicle.center;
  // 有观测时当前板取匹配槽位，短漏检时可用几何朝向选择显示槽位。
  int current_slot = vehicle.observed_slot;
  if (!vehicle.observed) {
    const int visible = mostFacingSlot(vehicle.center, vehicle.phase, estimated, min_facing_cosine);
    if (visible >= 0) current_slot = visible;
  }
  target.current_slot = current_slot;
  target.current_phase = estimated.platePhase(vehicle.phase, current_slot);
  target.current = estimated.platePosition(vehicle.center, vehicle.phase, current_slot);
  // 同时外推车体平移和旋转，再重新选未来槽位，允许切换到另一块板。
  const Eigen::Vector3d future_center = vehicle.center + horizon_s * vehicle.velocity;
  const double future_phase = wrapAngle(vehicle.phase + horizon_s * vehicle.omega);
  const int future_slot = mostFacingSlot(future_center, future_phase, estimated, min_facing_cosine);
  if (future_slot >= 0) {
    // 预测存在不等于本帧有效：valid 还要求当前观测命中且当前板深度为正。
    target.predicted_slot = future_slot;
    target.predicted_phase = estimated.platePhase(future_phase, future_slot);
    target.predicted = estimated.platePosition(future_center, future_phase, future_slot);
    target.has_prediction = true;
    target.valid = vehicle.observed && target.current.z() > 0.0;
  }
  return target;
}

}  // namespace ft_selector
