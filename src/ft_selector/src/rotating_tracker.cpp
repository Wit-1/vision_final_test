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

bool positive(double value) { return std::isfinite(value) && value > 0.0; }

bool usable(const ArmorObservation& observation) {
  return observation.position.allFinite() && observation.position.z() > 0.0 &&
         std::isfinite(observation.yaw);
}

// Integrate continuous white acceleration over dt for one position/velocity pair.
void addProcessNoise(StateMatrix& noise, int position, int velocity, double dt, double density) {
  noise(position, position) = density * dt * dt * dt / 3.0;
  noise(position, velocity) = noise(velocity, position) = density * dt * dt / 2.0;
  noise(velocity, velocity) = density * dt;
}
}  // namespace

double wrapAngle(double angle) { return std::remainder(angle, kTwoPi); }

Eigen::Vector3d outwardNormal(double phase) { return {std::sin(phase), 0.0, std::cos(phase)}; }

double CircleModel::platePhase(double phase, int slot) const {
  return wrapAngle(phase + kTwoPi * slot / plate_count);
}

Eigen::Vector3d CircleModel::platePosition(const Eigen::Vector3d& center, double phase, int slot) const {
  return center + radius_m * outwardNormal(platePhase(phase, slot));
}

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

void RotatingTracker::reset() {
  id_ = 0;
  last_stamp_ = last_observed_stamp_ = 0.0;
}

void RotatingTracker::initialize(const ArmorObservation& observation, double stamp) {
  if (next_id_ > std::numeric_limits<std::uint32_t>::max()) throw std::overflow_error("Vehicle ID exhausted");
  id_ = static_cast<std::uint32_t>(next_id_++);
  is_red_ = observation.is_red;
  observed_slot_ = 0;
  state_.setZero();
  state_[6] = wrapAngle(observation.yaw);
  state_.head<3>() = observation.position - params_.circle.radius_m * outwardNormal(state_[6]);
  covariance_.setZero();
  covariance_.diagonal() << params_.position_variance, params_.position_variance,
      params_.position_variance, params_.initial_velocity_variance, params_.initial_velocity_variance,
      params_.initial_velocity_variance, params_.yaw_variance, params_.initial_omega_variance;
  last_stamp_ = last_observed_stamp_ = stamp;
}

void RotatingTracker::predict(double dt) {
  StateMatrix transition = StateMatrix::Identity();
  transition.block<3, 3>(0, 3) = dt * Eigen::Matrix3d::Identity();
  transition(6, 7) = dt;
  state_ = transition * state_;
  state_[6] = wrapAngle(state_[6]);

  StateMatrix noise = StateMatrix::Zero();
  for (int axis = 0; axis < 3; ++axis)
    addProcessNoise(noise, axis, axis + 3, dt, params_.linear_noise_density);
  addProcessNoise(noise, 6, 7, dt, params_.angular_noise_density);
  covariance_ = transition * covariance_ * transition.transpose() + noise;
}

void RotatingTracker::correct(const ArmorObservation& observation, int slot) {
  const double yaw = params_.circle.platePhase(state_[6], slot);
  Eigen::Vector4d residual;
  residual.head<3>() = observation.position - params_.circle.platePosition(state_.head<3>(), state_[6], slot);
  residual[3] = wrapAngle(observation.yaw - yaw);

  Eigen::Matrix<double, 4, 8> jacobian = Eigen::Matrix<double, 4, 8>::Zero();
  jacobian.block<3, 3>(0, 0).setIdentity();
  jacobian.block<3, 1>(0, 6) =
      params_.circle.radius_m * Eigen::Vector3d(std::cos(yaw), 0.0, -std::sin(yaw));
  jacobian(3, 6) = 1.0;

  Eigen::Matrix4d noise = Eigen::Matrix4d::Identity() * params_.position_variance;
  noise(3, 3) = params_.yaw_variance;
  const Eigen::Matrix<double, 4, 8> projected_covariance = jacobian * covariance_;
  const Eigen::Matrix4d innovation = projected_covariance * jacobian.transpose() + noise;
  const Eigen::LLT<Eigen::Matrix4d> factor(innovation);
  if (factor.info() != Eigen::Success) throw std::runtime_error("Invalid EKF innovation covariance");
  const Eigen::Matrix<double, 8, 4> gain = factor.solve(projected_covariance).transpose();
  state_ += gain * residual;
  state_[6] = wrapAngle(state_[6]);
  const StateMatrix remaining = StateMatrix::Identity() - gain * jacobian;
  covariance_ = remaining * covariance_ * remaining.transpose() + gain * noise * gain.transpose();
}

VehicleEstimate RotatingTracker::estimate(bool observed) const {
  return {id_, observed, observed_slot_, state_.head<3>(), state_.segment<3>(3), state_[6], state_[7]};
}

VehicleEstimate RotatingTracker::update(double stamp, const std::vector<ArmorObservation>& observations) {
  if (!std::isfinite(stamp)) throw std::invalid_argument("Invalid observation timestamp");
  if (id_ && stamp < last_stamp_) reset();
  if (id_ && stamp - last_observed_stamp_ > params_.lost_timeout_s) {
    reset();
    return {};
  }

  if (!id_) {
    const ArmorObservation* nearest = nullptr;
    for (const auto& observation : observations) {
      if (usable(observation) && (!nearest || observation.position.norm() < nearest->position.norm()))
        nearest = &observation;
    }
    if (!nearest) return {};
    initialize(*nearest, stamp);
    return estimate(true);
  }

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
      const double cost = position_error * position_error + yaw_error * yaw_error;
      if (cost < best_cost) {
        best = &observation;
        best_slot = slot;
        best_cost = cost;
      }
    }
  }
  if (best) {
    correct(*best, best_slot);
    last_observed_stamp_ = stamp;
    observed_slot_ = best_slot;
  }
  if (!state_.allFinite() || !covariance_.allFinite()) {
    reset();
    return {};
  }
  return estimate(best != nullptr);
}

}  // namespace ft_selector
