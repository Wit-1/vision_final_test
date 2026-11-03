#pragma once

#include <cstdint>
#include <limits>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/video/tracking.hpp>

namespace ft_tracker {

struct TrackerParams {
  double process_noise_density;       // Continuous white acceleration noise, m^2/s^3.
  double measurement_noise_variance;  // Each position observation axis, m^2.
  double initial_position_variance;   // m^2.
  double initial_velocity_variance;   // (m/s)^2.
  double association_distance_m;
  double lost_timeout_s;
  double prediction_time_s;
  bool match_color, match_size;
};

struct Observation {
  std::uint32_t detection_id = 0;  // Frame-local; never used as a track identity.
  bool is_red = false, is_large = false;
  cv::Vec3d position{0.0, 0.0, 0.0};
};

struct TrackState {
  bool exists = false, tracking = false;
  std::uint32_t id = 0;
  cv::Vec3d position{0.0, 0.0, 0.0}, velocity{0.0, 0.0, 0.0};
  cv::Vec3d position_predict{0.0, 0.0, 0.0};
};

// ROS-independent state machine. Call once per processed detection frame, including empty frames.
class LinearTracker {
 public:
  explicit LinearTracker(const TrackerParams& params);
  TrackState update(double image_stamp_s, const std::vector<Observation>& observations);
  void reset();  // Preserve the ID counter, including after a time/frame change.

 private:
  void initialize(const Observation& observation, double stamp_s);
  void predict(double dt);
  TrackState state(bool observed);
  TrackerParams params_;
  cv::KalmanFilter filter_;
  bool active_ = false, is_red_ = false, is_large_ = false;
  std::uint32_t id_ = 0;
  std::uint64_t next_id_ = 1;
  double last_stamp_s_ = std::numeric_limits<double>::quiet_NaN();
  double last_observed_s_ = std::numeric_limits<double>::quiet_NaN();
};

}  // namespace ft_tracker
