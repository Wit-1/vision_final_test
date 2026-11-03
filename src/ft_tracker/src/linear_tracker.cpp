#include "ft_tracker/linear_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <stdexcept>

namespace ft_tracker {
namespace {

bool validObservation(const Observation& observation) {
  const auto& p = observation.position;
  return std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]) && p[2] > 0.0;
}

cv::Vec3d positionOf(const cv::Mat& state) {
  return {state.at<double>(0), state.at<double>(1), state.at<double>(2)};
}

}  // namespace

LinearTracker::LinearTracker(const TrackerParams& params) : params_(params), filter_(6, 3, 0, CV_64F) {
  const double positive[] = {params.process_noise_density, params.measurement_noise_variance,
      params.initial_position_variance, params.initial_velocity_variance,
      params.association_distance_m, params.lost_timeout_s};
  if (!std::all_of(std::begin(positive), std::end(positive),
                   [](double value) { return std::isfinite(value) && value > 0.0; }) ||
      !std::isfinite(params.prediction_time_s) || params.prediction_time_s < 0.0)
    throw std::invalid_argument("Tracker noise, variance, gate and timeout must be positive; T must be nonnegative");
  cv::setIdentity(filter_.measurementNoiseCov, cv::Scalar(params.measurement_noise_variance));
  filter_.measurementMatrix = cv::Mat::zeros(3, 6, CV_64F);
  for (int axis = 0; axis < 3; ++axis) filter_.measurementMatrix.at<double>(axis, axis) = 1.0;
}

void LinearTracker::reset() {
  active_ = false;
  id_ = 0;
  last_stamp_s_ = last_observed_s_ = std::numeric_limits<double>::quiet_NaN();
}

void LinearTracker::initialize(const Observation& observation, double stamp_s) {
  if (next_id_ > std::numeric_limits<std::uint32_t>::max()) throw std::overflow_error("Track ID exhausted");
  id_ = static_cast<std::uint32_t>(next_id_++);
  is_red_ = observation.is_red;
  is_large_ = observation.is_large;
  filter_.statePost = cv::Mat::zeros(6, 1, CV_64F);
  filter_.errorCovPost = cv::Mat::zeros(6, 6, CV_64F);
  for (int axis = 0; axis < 3; ++axis) {
    filter_.statePost.at<double>(axis) = observation.position[axis];
    filter_.errorCovPost.at<double>(axis, axis) = params_.initial_position_variance;
    filter_.errorCovPost.at<double>(axis + 3, axis + 3) = params_.initial_velocity_variance;
  }
  last_observed_s_ = stamp_s;
  active_ = true;
}

void LinearTracker::predict(double dt) {
  cv::setIdentity(filter_.transitionMatrix);
  filter_.processNoiseCov = cv::Mat::zeros(6, 6, CV_64F);
  const double q = params_.process_noise_density;
  // Integrated continuous white acceleration: Q_axis = q * [[dt^3/3,dt^2/2],[dt^2/2,dt]].
  for (int axis = 0; axis < 3; ++axis) {
    filter_.transitionMatrix.at<double>(axis, axis + 3) = dt;
    filter_.processNoiseCov.at<double>(axis, axis) = q * dt * dt * dt / 3.0;
    filter_.processNoiseCov.at<double>(axis, axis + 3) = q * dt * dt / 2.0;
    filter_.processNoiseCov.at<double>(axis + 3, axis) = q * dt * dt / 2.0;
    filter_.processNoiseCov.at<double>(axis + 3, axis + 3) = q * dt;
  }
  filter_.predict();
  // A missed observation must advance the posterior as well, so consecutive misses keep moving.
  filter_.statePre.copyTo(filter_.statePost);
  filter_.errorCovPre.copyTo(filter_.errorCovPost);
}

TrackState LinearTracker::state(bool observed) {
  TrackState result;
  result.exists = active_;
  if (!active_) return result;
  result.id = id_;
  result.tracking = observed;
  result.position = positionOf(filter_.statePost);
  for (int axis = 0; axis < 3; ++axis) result.velocity[axis] = filter_.statePost.at<double>(axis + 3);
  result.position_predict = result.position + params_.prediction_time_s * result.velocity;
  for (int axis = 0; axis < 3; ++axis) {
    if (!std::isfinite(result.position[axis]) || !std::isfinite(result.velocity[axis]) ||
        !std::isfinite(result.position_predict[axis])) {
      reset();
      return {};
    }
  }
  return result;
}

TrackState LinearTracker::update(double image_stamp_s, const std::vector<Observation>& observations) {
  if (!std::isfinite(image_stamp_s)) {
    reset();
    return {};
  }
  if (std::isfinite(last_stamp_s_) && image_stamp_s <= last_stamp_s_) reset();
  if (active_ && image_stamp_s - last_observed_s_ > params_.lost_timeout_s) reset();

  if (!active_) {
    const Observation* closest = nullptr;
    for (const auto& observation : observations) {
      if (!validObservation(observation)) continue;
      if (!closest || cv::norm(observation.position) < cv::norm(closest->position) ||
          (cv::norm(observation.position) == cv::norm(closest->position) &&
           observation.detection_id < closest->detection_id)) closest = &observation;
    }
    if (closest) initialize(*closest, image_stamp_s);
    last_stamp_s_ = image_stamp_s;
    return state(closest != nullptr);
  }

  predict(image_stamp_s - last_stamp_s_);
  last_stamp_s_ = image_stamp_s;
  const auto predicted = positionOf(filter_.statePre);
  const Observation* matched = nullptr;
  double best_distance = params_.association_distance_m;
  for (const auto& observation : observations) {
    if (!validObservation(observation) || (params_.match_color && observation.is_red != is_red_) ||
        (params_.match_size && observation.is_large != is_large_)) continue;
    const double distance = cv::norm(observation.position - predicted);
    if (distance <= best_distance && (!matched || distance < best_distance ||
                                     observation.detection_id < matched->detection_id)) {
      best_distance = distance;
      matched = &observation;
    }
  }
  if (matched) {
    cv::Mat measurement(3, 1, CV_64F);
    for (int axis = 0; axis < 3; ++axis) measurement.at<double>(axis) = matched->position[axis];
    filter_.correct(measurement);
    last_observed_s_ = image_stamp_s;
  }
  if (!cv::checkRange(filter_.statePost) || !cv::checkRange(filter_.errorCovPost) ||
      filter_.statePost.at<double>(2) <= 0.0) {
    reset();
    return {};
  }
  return state(matched != nullptr);
}

}  // namespace ft_tracker
