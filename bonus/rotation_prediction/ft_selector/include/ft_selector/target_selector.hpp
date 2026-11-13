#pragma once

#include "ft_selector/rotating_tracker.hpp"

namespace ft_selector {

struct SelectedTarget {
  std::uint32_t id = 0;
  bool valid = false;
  bool has_prediction = false;
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Vector3d current = Eigen::Vector3d::Zero();
  Eigen::Vector3d predicted = Eigen::Vector3d::Zero();
  int current_slot = -1, predicted_slot = -1;
  double current_phase = 0.0, predicted_phase = 0.0;
};

SelectedTarget selectTarget(const VehicleEstimate& vehicle, const CircleModel& circle,
                            double horizon_s, double min_facing_cosine);

}  // namespace ft_selector
