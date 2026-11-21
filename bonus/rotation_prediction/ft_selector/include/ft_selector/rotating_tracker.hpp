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
  double delta_radius_m = 0.0;
  double delta_height_m = 0.0;  // Positive means the odd pair is higher (camera Y decreases).

  double plateRadius(int slot) const;
  double platePhase(double phase, int slot) const;
  Eigen::Vector3d platePosition(const Eigen::Vector3d& center, double phase, int slot) const;
};

struct RotatingParams {
  CircleModel circle;
  double position_gate_m = 0.0;
  double yaw_gate_rad = 0.0;
  double switch_cost_margin = 0.01;  // Absolute improvement in the normalized association cost.
  double switch_cooldown_s = 0.02;  // Image time; an unmatched current slot bypasses the cooldown.
  double lost_timeout_s = 0.0;
  double linear_noise_density = 0.0;
  double angular_noise_density = 0.0;
  double position_variance = 0.0;
  double yaw_variance = 0.0;
  double initial_velocity_variance = 0.0;
  double initial_omega_variance = 0.0;
  double radius_min_m = 0.05, radius_max_m = 0.50, max_height_difference_m = 0.20;
  double initial_geometry_variance = 0.0025;
  double geometry_noise_density = 0.00001;  // Random walk, m^2/s.
  double geometry_min_phase_span_rad = 0.35;
  int geometry_min_observations = 10;
  bool estimate_pair_offsets = true;  // Four plates only; false is the shared-radius stage.
};

// 在线字段独立声明，避免改参数时意外替换半径、板数或初始协方差。
struct RotatingTuning {
  double linear_noise_density, angular_noise_density;
  double position_variance, yaw_variance;
  double position_gate_m, yaw_gate_rad;
  double switch_cost_margin, switch_cooldown_s;
};

struct ArmorObservation {
  Eigen::Vector3d position;
  double yaw;
  bool is_red;
  bool is_large = false;
  int detection_index = -1;  // Array index in this input frame, not a cross-frame identity.
};

struct VehicleEstimate {
  std::uint32_t id = 0;
  bool observed = false;
  int observed_slot = 0;
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  double phase = 0.0;
  double omega = 0.0;
  CircleModel geometry;
  Eigen::Vector3d geometry_stddev = Eigen::Vector3d::Zero();
  bool geometry_ready = false;
  bool is_large = false;
  int detection_index = -1;
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
  bool correct(const ArmorObservation& observation, int slot);
  VehicleEstimate estimate(bool observed) const;
  CircleModel geometry() const;

  RotatingParams params_;
  // [cx, cy, cz, vx, vy, vz, phase, omega, r, delta_r, delta_h].
  Eigen::Matrix<double, 11, 1> state_ = Eigen::Matrix<double, 11, 1>::Zero();
  Eigen::Matrix<double, 11, 11> covariance_ = Eigen::Matrix<double, 11, 11>::Zero();
  std::uint32_t id_ = 0;
  std::uint64_t next_id_ = 1;
  bool is_red_ = false;
  int observed_slot_ = 0;
  int detection_index_ = -1;
  bool is_large_ = false;
  unsigned int groups_seen_ = 0;
  int observation_count_ = 0;
  double unwrapped_phase_ = 0.0, min_phase_ = 0.0, max_phase_ = 0.0;
  double last_stamp_ = 0.0, last_observed_stamp_ = 0.0;
  double last_switch_stamp_ = 0.0;
};

}  // namespace ft_selector
