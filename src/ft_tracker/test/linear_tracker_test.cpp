#include <cmath>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "ft_tracker/linear_tracker.hpp"

namespace ft_tracker {
namespace {

TrackerParams params() { return {2.0, 0.0025, 0.01, 4.0, 0.5, 0.5, 0.1, true, true}; }

Observation observation(double x = 0.0, std::uint32_t id = 0, double z = 3.0) {
  Observation result;
  result.position = {x, 0.0, z};
  result.detection_id = id;
  return result;
}

TrackState warmup(LinearTracker& tracker) {
  TrackState result;
  for (int i = 0; i <= 50; ++i) result = tracker.update(i * 0.02, {observation(i * 0.02)});
  return result;
}

TEST(LinearTracker, RejectsInvalidParameters) {
  auto p = params(); p.process_noise_density = 0.0; EXPECT_THROW(LinearTracker{p}, std::invalid_argument);
  p = params(); p.measurement_noise_variance = -1.0; EXPECT_THROW(LinearTracker{p}, std::invalid_argument);
  p = params(); p.initial_position_variance = 0.0; EXPECT_THROW(LinearTracker{p}, std::invalid_argument);
  p = params(); p.initial_velocity_variance = 0.0; EXPECT_THROW(LinearTracker{p}, std::invalid_argument);
  p = params(); p.association_distance_m = 0.0; EXPECT_THROW(LinearTracker{p}, std::invalid_argument);
  p = params(); p.lost_timeout_s = 0.0; EXPECT_THROW(LinearTracker{p}, std::invalid_argument);
  p = params(); p.prediction_time_s = -0.1; EXPECT_THROW(LinearTracker{p}, std::invalid_argument);
  p = params(); p.prediction_time_s = std::numeric_limits<double>::infinity();
  EXPECT_THROW(LinearTracker{p}, std::invalid_argument);
  p = params(); p.process_noise_density = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(LinearTracker{p}, std::invalid_argument);
}

TEST(LinearTracker, EmptyInputBeforeAcquisitionHasNoTrack) {
  LinearTracker tracker(params());
  const auto state = tracker.update(1.0, {});
  EXPECT_FALSE(state.exists); EXPECT_FALSE(state.tracking); EXPECT_EQ(state.id, 0U);
  EXPECT_EQ(cv::norm(state.position), 0.0); EXPECT_EQ(cv::norm(state.velocity), 0.0);
}

TEST(LinearTracker, InitializesFromPositionWithZeroVelocity) {
  LinearTracker tracker(params());
  const auto state = tracker.update(0.0, {observation(0.3, 47)});
  EXPECT_TRUE(state.exists); EXPECT_TRUE(state.tracking); EXPECT_EQ(state.id, 1U);
  EXPECT_DOUBLE_EQ(state.position[0], 0.3); EXPECT_DOUBLE_EQ(state.position[2], 3.0);
  EXPECT_EQ(cv::norm(state.velocity), 0.0); EXPECT_EQ(cv::norm(state.position_predict - state.position), 0.0);
}

TEST(LinearTracker, InitialChoiceUsesRangeRatherThanDepthAlone) {
  LinearTracker tracker(params());
  const auto state = tracker.update(0.0, {observation(10.0, 0, 1.0), observation(0.0, 1, 2.0)});
  EXPECT_DOUBLE_EQ(state.position[0], 0.0); EXPECT_DOUBLE_EQ(state.position[2], 2.0);
}

TEST(LinearTracker, FrameLocalDetectionIdsDoNotChangeTrackIdentity) {
  LinearTracker tracker(params());
  const auto first = tracker.update(1.0, {observation(0.0, 9)});
  const auto next = tracker.update(1.02, {observation(0.01, 74)});
  EXPECT_EQ(first.id, next.id); EXPECT_TRUE(next.tracking);
}

TEST(LinearTracker, KeepsLockedTargetWhenAnotherTargetBecomesCloserToCamera) {
  LinearTracker tracker(params());
  const auto first = tracker.update(1.0, {observation(0.0)});
  const auto next = tracker.update(1.02, {observation(0.0, 0, 1.0), observation(0.02, 1)});
  EXPECT_EQ(next.id, first.id); EXPECT_TRUE(next.tracking); EXPECT_NEAR(next.position[2], 3.0, 1e-10);
}

TEST(LinearTracker, RejectsWrongColorWithinSpatialGate) {
  LinearTracker tracker(params()); tracker.update(1.0, {observation()});
  auto red = observation(); red.is_red = true;
  const auto state = tracker.update(1.02, {red});
  EXPECT_TRUE(state.exists); EXPECT_FALSE(state.tracking);
}

TEST(LinearTracker, RejectsWrongSizeWithinSpatialGate) {
  LinearTracker tracker(params()); tracker.update(1.0, {observation()});
  auto large = observation(); large.is_large = true;
  EXPECT_FALSE(tracker.update(1.02, {large}).tracking);
}

TEST(LinearTracker, AttributeGatesCanBeDisabled) {
  auto p = params(); p.match_color = p.match_size = false;
  LinearTracker tracker(p); tracker.update(1.0, {observation()});
  auto other = observation(); other.is_red = other.is_large = true;
  EXPECT_TRUE(tracker.update(1.02, {other}).tracking);
}

TEST(LinearTracker, EstimatesThreeDimensionalConstantVelocity) {
  LinearTracker tracker(params()); TrackState state;
  const cv::Vec3d velocity(0.4, -0.2, 0.1), start(0.0, 0.0, 3.0);
  for (int i = 0; i <= 80; ++i) {
    auto item = observation(); item.position = start + (i * 0.02) * velocity;
    state = tracker.update(i * 0.02, {item});
  }
  EXPECT_TRUE(state.tracking); EXPECT_LT(cv::norm(state.velocity - velocity), 0.002);
  EXPECT_LT(cv::norm(state.position - (start + 1.6 * velocity)), 0.001);
  EXPECT_LT(cv::norm(state.position_predict - (state.position + 0.1 * state.velocity)), 1e-12);
}

TEST(LinearTracker, UsesActualIrregularImageIntervals) {
  LinearTracker tracker(params()); TrackState state;
  double stamp = 0.0;
  for (int i = 0; i < 100; ++i) {
    stamp += i % 2 == 0 ? 0.01 : 0.07;
    state = tracker.update(stamp, {observation(stamp * 0.6)});
  }
  EXPECT_NEAR(state.velocity[0], 0.6, 0.002); EXPECT_NEAR(state.position[0], stamp * 0.6, 0.001);
}

TEST(LinearTracker, ConsecutiveMissesAdvancePredictionAndKeepId) {
  LinearTracker tracker(params()); const auto seen = warmup(tracker);
  const auto first = tracker.update(1.1, {}); const auto second = tracker.update(1.2, {});
  EXPECT_EQ(first.id, seen.id); EXPECT_EQ(second.id, seen.id); EXPECT_FALSE(second.tracking);
  EXPECT_NEAR(first.position[0], seen.position[0] + seen.velocity[0] * 0.1, 1e-10);
  EXPECT_NEAR(second.position[0], seen.position[0] + seen.velocity[0] * 0.2, 1e-10);
}

TEST(LinearTracker, ObservationAfterShortMissCorrectsExistingTrack) {
  LinearTracker tracker(params()); const auto first = warmup(tracker);
  tracker.update(1.1, {});
  const auto state = tracker.update(1.2, {observation(1.2, 93)});
  EXPECT_EQ(state.id, first.id); EXPECT_TRUE(state.tracking);
}

TEST(LinearTracker, TimeoutIsStrictlyGreaterThanHalfASecond) {
  LinearTracker tracker(params()); tracker.update(0.0, {observation()});
  EXPECT_TRUE(tracker.update(0.5, {}).exists);
  const auto expired = tracker.update(0.5001, {});
  EXPECT_FALSE(expired.exists); EXPECT_EQ(expired.id, 0U); EXPECT_FALSE(expired.tracking);
}

TEST(LinearTracker, ReacquisitionAfterTimeoutUsesANewId) {
  LinearTracker tracker(params()); const auto first = tracker.update(1.0, {observation()});
  tracker.update(1.51, {});
  const auto second = tracker.update(1.52, {observation()});
  EXPECT_GT(second.id, first.id); EXPECT_TRUE(second.tracking);
}

TEST(LinearTracker, LongGapReinitializesEvenWhenNewMeasurementIsNearby) {
  LinearTracker tracker(params()); const auto first = tracker.update(1.0, {observation()});
  const auto next = tracker.update(2.0, {observation()});
  EXPECT_GT(next.id, first.id); EXPECT_EQ(cv::norm(next.velocity), 0.0);
}

TEST(LinearTracker, OutOfGateObservationDoesNotRefreshLastObservedTime) {
  LinearTracker tracker(params()); tracker.update(0.0, {observation()});
  EXPECT_FALSE(tracker.update(0.4, {observation(2.0)}).tracking);
  const auto state = tracker.update(0.51, {});
  EXPECT_FALSE(state.exists);
}

TEST(LinearTracker, RewindClearsVelocityAndAllocatesNewIdentity) {
  LinearTracker tracker(params()); const auto moving = warmup(tracker);
  const auto state = tracker.update(0.1, {observation(0.1)});
  EXPECT_GT(state.id, moving.id); EXPECT_EQ(cv::norm(state.velocity), 0.0);
}

TEST(LinearTracker, RepeatedImageTimestampStartsFreshTrack) {
  LinearTracker tracker(params()); const auto first = tracker.update(1.0, {observation()});
  const auto next = tracker.update(1.0, {observation()});
  EXPECT_GT(next.id, first.id); EXPECT_TRUE(next.tracking);
}

TEST(LinearTracker, ExplicitResetPreservesMonotonicIdCounter) {
  LinearTracker tracker(params()); const auto first = tracker.update(1.0, {observation()});
  tracker.reset(); const auto next = tracker.update(1.1, {observation()});
  EXPECT_GT(next.id, first.id);
}

TEST(LinearTracker, InvalidObservationsCannotInitializeOrCorrect) {
  LinearTracker tracker(params());
  auto nan = observation(); nan.position[0] = std::numeric_limits<double>::quiet_NaN();
  auto inf = observation(); inf.position[1] = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(tracker.update(0.0, {nan, inf, observation(0.0, 0, 0.0), observation(0.0, 0, -1.0)}).exists);
  tracker.update(0.1, {observation()});
  EXPECT_FALSE(tracker.update(0.2, {nan, inf}).tracking);
}

TEST(LinearTracker, ZeroLeadTimeMakesFuturePositionEqualCurrentPosition) {
  auto p = params(); p.prediction_time_s = 0.0;
  LinearTracker tracker(p); const auto state = warmup(tracker);
  EXPECT_LT(cv::norm(state.position_predict - state.position), 1e-12);
}

TEST(LinearTracker, HigherMeasurementVarianceTrustsPositionJumpLess) {
  auto low = params(), high = params(); high.measurement_noise_variance = 1.0;
  LinearTracker first(low), second(high);
  first.update(0.0, {observation()}); second.update(0.0, {observation()});
  const auto a = first.update(0.02, {observation(0.2)});
  const auto b = second.update(0.02, {observation(0.2)});
  EXPECT_GT(a.position[0], b.position[0]);
}

TEST(LinearTracker, InvalidTimestampClearsTrackWithoutNonFiniteOutput) {
  LinearTracker tracker(params()); tracker.update(0.0, {observation()});
  const auto state = tracker.update(std::numeric_limits<double>::quiet_NaN(), {observation()});
  EXPECT_FALSE(state.exists); EXPECT_EQ(state.id, 0U); EXPECT_EQ(cv::norm(state.position), 0.0);
}

}  // namespace
}  // namespace ft_tracker

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
