#pragma once

#include <cstdint>
#include <vector>

#include <Eigen/Core>

namespace ft_selector {

double wrapAngle(double angle);
Eigen::Vector3d outwardNormal(double phase);

struct CircleModel {
  double radius_m = 0.0;
  int plate_count = 0;

  double platePhase(double phase, int slot) const;
  Eigen::Vector3d platePosition(const Eigen::Vector3d& center, double phase, int slot) const;
};

struct RotatingParams {
  CircleModel circle;
  double position_gate_m = 0.0;
  double yaw_gate_rad = 0.0;
  double lost_timeout_s = 0.0;
  double linear_noise_density = 0.0;
  double angular_noise_density = 0.0;
  double position_variance = 0.0;
  double yaw_variance = 0.0;
  double initial_velocity_variance = 0.0;
  double initial_omega_variance = 0.0;
};

// 在线字段独立声明，避免改参数时意外替换半径、板数或初始协方差。
struct RotatingTuning {
  double linear_noise_density, angular_noise_density;
  double position_variance, yaw_variance;
  double position_gate_m, yaw_gate_rad;
};

struct ArmorObservation {
  Eigen::Vector3d position;
  double yaw;
  bool is_red;
};

struct VehicleEstimate {
  std::uint32_t id = 0;
  bool observed = false;
  int observed_slot = 0;
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  double phase = 0.0;
  double omega = 0.0;
};

class RotatingTracker {
 public:
  explicit RotatingTracker(RotatingParams params);
  void setTuning(const RotatingTuning& tuning);
  VehicleEstimate update(double stamp, const std::vector<ArmorObservation>& observations);
  void reset();

 private:
  void initialize(const ArmorObservation& observation, double stamp);
  void predict(double dt);
  void correct(const ArmorObservation& observation, int slot);
  VehicleEstimate estimate(bool observed) const;

  RotatingParams params_;
  // [cx, cy, cz, vx, vy, vz, phase, omega] in metres, seconds and radians.
  Eigen::Matrix<double, 8, 1> state_ = Eigen::Matrix<double, 8, 1>::Zero();
  Eigen::Matrix<double, 8, 8> covariance_ = Eigen::Matrix<double, 8, 8>::Zero();
  std::uint32_t id_ = 0;
  std::uint64_t next_id_ = 1;
  bool is_red_ = false;
  int observed_slot_ = 0;
  double last_stamp_ = 0.0, last_observed_stamp_ = 0.0;
};

}  // namespace ft_selector
