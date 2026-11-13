#include "ft_tracker/linear_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <stdexcept>

namespace ft_tracker {
namespace {

// 关联前只接受有限坐标且位于相机前方的观测。
bool validObservation(const Observation& observation) {
  const auto& p = observation.position;
  return std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]) && p[2] > 0.0;
}

// 六维状态的前三项为位置，后三项为速度。
cv::Vec3d positionOf(const cv::Mat& state) {
  return {state.at<double>(0), state.at<double>(1), state.at<double>(2)};
}

}  // namespace

// 初始化双精度匀速滤波器：六维状态、三维位置观测。
LinearTracker::LinearTracker(const TrackerParams& params) : params_(params), filter_(6, 3, 0, CV_64F) {
  const double positive[] = {params.process_noise_density, params.measurement_noise_variance,
      params.initial_position_variance, params.initial_velocity_variance,
      params.association_distance_m, params.lost_timeout_s};
  if (!std::all_of(std::begin(positive), std::end(positive),
                   [](double value) { return std::isfinite(value) && value > 0.0; }) ||
      !std::isfinite(params.prediction_time_s) || params.prediction_time_s < 0.0)
    throw std::invalid_argument("Tracker noise, variance, gate and timeout must be positive; T must be nonnegative");
  // R 表达位置观测噪声，H 只取位置分量，速度由连续观测估计。
  cv::setIdentity(filter_.measurementNoiseCov, cv::Scalar(params.measurement_noise_variance));
  filter_.measurementMatrix = cv::Mat::zeros(3, 6, CV_64F);
  for (int axis = 0; axis < 3; ++axis) filter_.measurementMatrix.at<double>(axis, axis) = 1.0;
}

// 在线替换 Q/R、关联门限和 T，保留状态、协方差以及轨迹身份。
void LinearTracker::setTuning(const LinearTuning& tuning) {
  const double positive[] = {tuning.process_noise_density, tuning.measurement_noise_variance,
                            tuning.association_distance_m};
  if (!std::all_of(std::begin(positive), std::end(positive),
                   [](double value) { return std::isfinite(value) && value > 0.0; }) ||
      !std::isfinite(tuning.prediction_time_s) || tuning.prediction_time_s < 0.0)
    throw std::invalid_argument("Linear tuning requires positive noise/gate and nonnegative T");
  params_.process_noise_density = tuning.process_noise_density;
  params_.measurement_noise_variance = tuning.measurement_noise_variance;
  params_.association_distance_m = tuning.association_distance_m;
  params_.prediction_time_s = tuning.prediction_time_s;
  cv::setIdentity(filter_.measurementNoiseCov, cv::Scalar(tuning.measurement_noise_variance));
}

// 删除当前轨迹，保留 ID 计数器以避免重新捕获时复用旧 ID。
void LinearTracker::reset() {
  active_ = false;
  id_ = 0;
  last_stamp_s_ = last_observed_s_ = std::numeric_limits<double>::quiet_NaN();
}

// 首次锁定时分配新 ID，以观测位置和零速度建立初始状态。
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

// 用真实图像时间差更新 F 和 Q，将状态推进到本帧时刻。
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

// 整理当前估计与 p+vT，并区分本帧命中和纯预测。
TrackState LinearTracker::state(bool observed) {
  TrackState result;
  result.exists = active_;
  if (!active_) return result;
  result.id = id_;
  result.tracking = observed;
  result.position = positionOf(filter_.statePost);
  for (int axis = 0; axis < 3; ++axis) result.velocity[axis] = filter_.statePost.at<double>(axis + 3);
  result.position_predict = result.position + params_.prediction_time_s * result.velocity;
  // 发布前检查估计数值，异常时清空轨迹而非输出非法坐标。
  for (int axis = 0; axis < 3; ++axis) {
    if (!std::isfinite(result.position[axis]) || !std::isfinite(result.velocity[axis]) ||
        !std::isfinite(result.position_predict[axis])) {
      reset();
      return {};
    }
  }
  return result;
}

// 每帧推进生命周期，再执行首次选择或预测、关联与校正。
TrackState LinearTracker::update(double image_stamp_s, const std::vector<Observation>& observations) {
  // 时间非法、重复、回退或观测超时，均结束旧轨迹。
  if (!std::isfinite(image_stamp_s)) {
    reset();
    return {};
  }
  if (std::isfinite(last_stamp_s_) && image_stamp_s <= last_stamp_s_) reset();
  if (active_ && image_stamp_s - last_observed_s_ > params_.lost_timeout_s) reset();

  if (!active_) {
    // 无活跃轨迹时选最近有效板，同距时用帧内 ID 确定稳定顺序。
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

  // 先预测到当前图像时刻，再在预测位置附近关联同属性观测。
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
  // 有匹配才校正；无匹配保留预测，tracking=false。
  if (matched) {
    cv::Mat measurement(3, 1, CV_64F);
    for (int axis = 0; axis < 3; ++axis) measurement.at<double>(axis) = matched->position[axis];
    filter_.correct(measurement);
    last_observed_s_ = image_stamp_s;
  }
  // 检查滤波后的状态、协方差和正深度，防止异常状态继续传播。
  if (!cv::checkRange(filter_.statePost) || !cv::checkRange(filter_.errorCovPost) ||
      filter_.statePost.at<double>(2) <= 0.0) {
    reset();
    return {};
  }
  return state(matched != nullptr);
}

}  // namespace ft_tracker
