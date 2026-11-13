#include "ft_selector/rotating_tracker.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

#include <Eigen/Cholesky>

namespace ft_selector {
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

using StateMatrix = Eigen::Matrix<double, 8, 8>;

// 噪声、半径和门限都要求是有限正值。
bool positive(double value) { return std::isfinite(value) && value > 0.0; }

// 旋转观测同时需要有效板位置与法向角。
bool usable(const ArmorObservation& observation) {
  return observation.position.allFinite() && observation.position.z() > 0.0 &&
         std::isfinite(observation.yaw);
}

// Integrate continuous white acceleration over dt for one position/velocity pair.
// 为位置/速度或相位/角速度这一对状态填入随 dt 积分的过程噪声。
void addProcessNoise(StateMatrix& noise, int position, int velocity, double dt, double density) {
  noise(position, position) = density * dt * dt * dt / 3.0;
  noise(position, velocity) = noise(velocity, position) = density * dt * dt / 2.0;
  noise(velocity, velocity) = density * dt;
}
}  // namespace

// 将相位和角度残差环绕到 ±π 附近，避免跨边界时出现整圈跳变。
double wrapAngle(double angle) { return std::remainder(angle, kTwoPi); }

// 在相机 X-Z 水平面中，由相位计算板的朝外方向。
Eigen::Vector3d outwardNormal(double phase) { return {std::sin(phase), 0.0, std::cos(phase)}; }

// 各槽位相对参考板等角分布，加入偏移后再次环绕角度。
double CircleModel::platePhase(double phase, int slot) const {
  return wrapAngle(phase + kTwoPi * slot / plate_count);
}

// 板中心等于车体中心加上半径乘以该槽位的朝外方向。
Eigen::Vector3d CircleModel::platePosition(const Eigen::Vector3d& center, double phase, int slot) const {
  return center + radius_m * outwardNormal(platePhase(phase, slot));
}

// 检查模型与滤波参数，角度门限不能让相邻槽位的范围重叠。
RotatingTracker::RotatingTracker(RotatingParams params) : params_(params) {
  if (!positive(params.circle.radius_m) || params.circle.plate_count < 2 ||
      params.circle.plate_count > 4 || !positive(params.position_gate_m) ||
      !positive(params.yaw_gate_rad) || params.yaw_gate_rad >= kPi / params.circle.plate_count ||
      !positive(params.lost_timeout_s) || !positive(params.linear_noise_density) ||
      !positive(params.angular_noise_density) || !positive(params.position_variance) ||
      !positive(params.yaw_variance) || !positive(params.initial_velocity_variance) ||
      !positive(params.initial_omega_variance))
    throw std::invalid_argument("Invalid rotating tracker parameters or overlapping yaw gates");
}

// 校验六项在线字段后统一应用，当前车体估计与协方差保持不变。
void RotatingTracker::setTuning(const RotatingTuning& tuning) {
  if (!positive(tuning.linear_noise_density) || !positive(tuning.angular_noise_density) ||
      !positive(tuning.position_variance) || !positive(tuning.yaw_variance) ||
      !positive(tuning.position_gate_m) || !positive(tuning.yaw_gate_rad) ||
      tuning.yaw_gate_rad >= kPi / params_.circle.plate_count)
    throw std::invalid_argument("Invalid rotating tuning or overlapping yaw gates");
  params_.linear_noise_density = tuning.linear_noise_density;
  params_.angular_noise_density = tuning.angular_noise_density;
  params_.position_variance = tuning.position_variance;
  params_.yaw_variance = tuning.yaw_variance;
  params_.position_gate_m = tuning.position_gate_m;
  params_.yaw_gate_rad = tuning.yaw_gate_rad;
}

// 结束当前车体轨迹，下一次观测建立新 ID。
void RotatingTracker::reset() {
  id_ = 0;
  last_stamp_ = last_observed_stamp_ = 0.0;
}

// 把首次观测视为槽位 0，用板位置和配置半径反推车体中心。
void RotatingTracker::initialize(const ArmorObservation& observation, double stamp) {
  if (next_id_ > std::numeric_limits<std::uint32_t>::max()) throw std::overflow_error("Vehicle ID exhausted");
  id_ = static_cast<std::uint32_t>(next_id_++);
  is_red_ = observation.is_red;
  observed_slot_ = 0;
  state_.setZero();
  state_[6] = wrapAngle(observation.yaw);
  state_.head<3>() = observation.position - params_.circle.radius_m * outwardNormal(state_[6]);
  // 初始化位置、速度、相位和角速度的不确定度。
  covariance_.setZero();
  covariance_.diagonal() << params_.position_variance, params_.position_variance,
      params_.position_variance, params_.initial_velocity_variance, params_.initial_velocity_variance,
      params_.initial_velocity_variance, params_.yaw_variance, params_.initial_omega_variance;
  last_stamp_ = last_observed_stamp_ = stamp;
}

// 按匀速平移和匀角速度推进八维状态，并预测协方差。
void RotatingTracker::predict(double dt) {
  StateMatrix transition = StateMatrix::Identity();
  transition.block<3, 3>(0, 3) = dt * Eigen::Matrix3d::Identity();
  transition(6, 7) = dt;
  state_ = transition * state_;
  state_[6] = wrapAngle(state_[6]);

  // 平移和旋转分别加入过程噪声，容纳短时运动变化。
  StateMatrix noise = StateMatrix::Zero();
  for (int axis = 0; axis < 3; ++axis)
    addProcessNoise(noise, axis, axis + 3, dt, params_.linear_noise_density);
  addProcessNoise(noise, 6, 7, dt, params_.angular_noise_density);
  covariance_ = transition * covariance_ * transition.transpose() + noise;
}

// 用匹配槽位的板位置与法向角执行 EKF 校正。
void RotatingTracker::correct(const ArmorObservation& observation, int slot) {
  // 残差比较同一块模型板的观测和预测，角残差必须环绕。
  const double yaw = params_.circle.platePhase(state_[6], slot);
  Eigen::Vector4d residual;
  residual.head<3>() = observation.position - params_.circle.platePosition(state_.head<3>(), state_[6], slot);
  residual[3] = wrapAngle(observation.yaw - yaw);

  // 位置观测对圆心和相位均敏感，雅可比包含圆周位置对相位的导数。
  Eigen::Matrix<double, 4, 8> jacobian = Eigen::Matrix<double, 4, 8>::Zero();
  jacobian.block<3, 3>(0, 0).setIdentity();
  jacobian.block<3, 1>(0, 6) =
      params_.circle.radius_m * Eigen::Vector3d(std::cos(yaw), 0.0, -std::sin(yaw));
  jacobian(3, 6) = 1.0;

  // 分别设置位置与角度观测噪声，通过 Cholesky 求解增益。
  Eigen::Matrix4d noise = Eigen::Matrix4d::Identity() * params_.position_variance;
  noise(3, 3) = params_.yaw_variance;
  const Eigen::Matrix<double, 4, 8> projected_covariance = jacobian * covariance_;
  const Eigen::Matrix4d innovation = projected_covariance * jacobian.transpose() + noise;
  const Eigen::LLT<Eigen::Matrix4d> factor(innovation);
  if (factor.info() != Eigen::Success) throw std::runtime_error("Invalid EKF innovation covariance");
  const Eigen::Matrix<double, 8, 4> gain = factor.solve(projected_covariance).transpose();
  state_ += gain * residual;
  state_[6] = wrapAngle(state_[6]);
  // Joseph 形式更新协方差，减少数值误差破坏其对称与半正定性。
  const StateMatrix remaining = StateMatrix::Identity() - gain * jacobian;
  covariance_ = remaining * covariance_ * remaining.transpose() + gain * noise * gain.transpose();
}

// 输出车体状态与当前匹配槽位，车体 ID 不等于帧内检测 ID。
VehicleEstimate RotatingTracker::estimate(bool observed) const {
  return {id_, observed, observed_slot_, state_.head<3>(), state_.segment<3>(3), state_[6], state_[7]};
}

// 逐帧管理车体生命周期，并在全部槽位和观测之间选择最佳关联。
VehicleEstimate RotatingTracker::update(double stamp, const std::vector<ArmorObservation>& observations) {
  // 时间回退重置；超时首帧先输出无轨迹，避免立即接上另一辆车。
  if (!std::isfinite(stamp)) throw std::invalid_argument("Invalid observation timestamp");
  if (id_ && stamp < last_stamp_) reset();
  if (id_ && stamp - last_observed_stamp_ > params_.lost_timeout_s) {
    reset();
    return {};
  }

  if (!id_) {
    // 无车体轨迹时用最近有效板初始化，尚不估计初始速度。
    const ArmorObservation* nearest = nullptr;
    for (const auto& observation : observations) {
      if (usable(observation) && (!nearest || observation.position.norm() < nearest->position.norm()))
        nearest = &observation;
    }
    if (!nearest) return {};
    initialize(*nearest, stamp);
    return estimate(true);
  }

  // 用本帧时间预测所有槽位，再按同色、位置门限和角度门限筛选。
  predict(stamp - last_stamp_);
  last_stamp_ = stamp;
  const ArmorObservation* best = nullptr;
  int best_slot = 0;
  double best_cost = std::numeric_limits<double>::infinity();
  for (int slot = 0; slot < params_.circle.plate_count; ++slot) {
    const double yaw = params_.circle.platePhase(state_[6], slot);
    const Eigen::Vector3d position = params_.circle.platePosition(state_.head<3>(), state_[6], slot);
    for (const auto& observation : observations) {
      if (!usable(observation) || observation.is_red != is_red_) continue;
      const double position_error = (observation.position - position).norm() / params_.position_gate_m;
      const double yaw_error = std::abs(wrapAngle(observation.yaw - yaw)) / params_.yaw_gate_rad;
      if (position_error > 1.0 || yaw_error > 1.0) continue;
      // 比较归一化位置和角度误差的平方和，每帧只校正一组。
      const double cost = position_error * position_error + yaw_error * yaw_error;
      if (cost < best_cost) {
        best = &observation;
        best_slot = slot;
        best_cost = cost;
      }
    }
  }
  // 换到同车另一槽位仍沿用车体 ID；漏匹配时仅保留预测。
  if (best) {
    correct(*best, best_slot);
    last_observed_stamp_ = stamp;
    observed_slot_ = best_slot;
  }
  // 数值异常立即清理，避免无效状态成为后续选板依据。
  if (!state_.allFinite() || !covariance_.allFinite()) {
    reset();
    return {};
  }
  return estimate(best != nullptr);
}

}  // namespace ft_selector
