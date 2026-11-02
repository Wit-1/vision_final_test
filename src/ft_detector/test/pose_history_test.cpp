#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "ft_detector/pose_history.hpp"

namespace ft_detector {
namespace {

ArmorCandidate candidate(float x, std::uint32_t id = 0) {
  ArmorCandidate armor;
  armor.id = id;
  armor.is_red = true;
  armor.corners = {{{x,100},{x+80,100},{x+80,140},{x,140}}};
  armor.center = {x+40,120};
  return armor;
}

PnpResult accepted(std::uint32_t id) {
  PnpResult pose;
  pose.valid = true;
  pose.pose.id = id;
  pose.pose.tvec = {0,0,3};
  return pose;
}

TEST(PoseHistory, NoReferenceOnFirstFrame) {
  PoseHistory history(0.1,0.5);
  EXPECT_EQ(history.references({candidate(100)},1.0).at(0), nullptr);
}

TEST(PoseHistory, MatchesGeometryDespiteChangedIdsAndArrayOrder) {
  PoseHistory history(0.1,0.5);
  history.store({candidate(100,0),candidate(400,1)}, {accepted(0),accepted(1)},1.0);
  const auto refs = history.references({candidate(402,0),candidate(102,99)},1.01);
  ASSERT_NE(refs[0], nullptr); ASSERT_NE(refs[1], nullptr);
  EXPECT_EQ(refs[0]->id, 1U); EXPECT_EQ(refs[1]->id, 0U);
}

TEST(PoseHistory, MatchingRequiresSameColourSizeAndNearbyCorners) {
  PoseHistory history(0.1,0.5);
  history.store({candidate(100)}, {accepted(0)},1.0);
  auto red = candidate(100); red.is_red = false;
  EXPECT_EQ(history.references({red},1.01)[0], nullptr);
  auto large = candidate(100); large.is_large_armor = true;
  EXPECT_EQ(history.references({large},1.01)[0], nullptr);
  EXPECT_EQ(history.references({candidate(121)},1.01)[0], nullptr);
  auto malformed = candidate(100); malformed.corners[0].x = std::numeric_limits<float>::quiet_NaN();
  EXPECT_EQ(history.references({malformed},1.01)[0], nullptr);
}

TEST(PoseHistory, AmbiguousPreviousOrCurrentMatchesProvideNoReference) {
  PoseHistory history(0.1,0.5);
  history.store({candidate(100),candidate(104)}, {accepted(0),accepted(1)},1.0);
  EXPECT_EQ(history.references({candidate(102)},1.01)[0], nullptr);
  history.store({candidate(100)}, {accepted(0)},1.0);
  const auto refs = history.references({candidate(101),candidate(102)},1.01);
  EXPECT_EQ(refs[0], nullptr); EXPECT_EQ(refs[1], nullptr);
}

TEST(PoseHistory, MissingOrInvalidResultsCannotLeaveStaleReference) {
  PoseHistory history(0.1,0.5);
  history.store({candidate(100)}, {accepted(0)},1.0);
  history.store({}, {},1.01);
  EXPECT_EQ(history.references({candidate(100)},1.02)[0], nullptr);
  history.store({candidate(100)}, {PnpResult{}},1.03);
  EXPECT_EQ(history.references({candidate(100)},1.04)[0], nullptr);
}

TEST(PoseHistory, OneLostBoardDoesNotEraseOtherBoardsReference) {
  PoseHistory history(0.1,0.5);
  history.store({candidate(100),candidate(400)}, {accepted(0),accepted(1)},1.0);
  history.store({candidate(402)}, {accepted(7)},1.01);
  const auto refs = history.references({candidate(101),candidate(403)},1.02);
  EXPECT_EQ(refs[0], nullptr);
  ASSERT_NE(refs[1], nullptr); EXPECT_EQ(refs[1]->id, 7U);
}

TEST(PoseHistory, RepeatedReversedLargeOrInvalidTimeClearsReference) {
  for (double next : {1.0,0.9,1.2,std::numeric_limits<double>::quiet_NaN()}) {
    PoseHistory history(0.1,0.5);
    history.store({candidate(100)}, {accepted(0)},1.0);
    EXPECT_EQ(history.references({candidate(100)},next)[0], nullptr);
    EXPECT_EQ(history.references({candidate(100)},1.01)[0], nullptr);
  }
}

TEST(PoseHistory, ExplicitResetClearsReferenceAndAllowsFreshInitialization) {
  PoseHistory history(0.1,0.5);
  history.store({candidate(100)}, {accepted(0)},1.0);
  history.reset();
  EXPECT_EQ(history.references({candidate(100)},1.01)[0], nullptr);
  history.store({candidate(100)}, {accepted(5)},1.02);
  const auto refs = history.references({candidate(100)},1.03);
  ASSERT_NE(refs[0], nullptr); EXPECT_EQ(refs[0]->id, 5U);
}

TEST(PoseHistory, RejectsInvalidParametersAndMismatchedResults) {
  EXPECT_THROW(PoseHistory(0.0,0.5), std::invalid_argument);
  EXPECT_THROW(PoseHistory(0.1,-1.0), std::invalid_argument);
  EXPECT_THROW(PoseHistory(std::numeric_limits<double>::infinity(),0.5), std::invalid_argument);
  PoseHistory history(0.1,0.5);
  EXPECT_THROW(history.store({candidate(100)}, {},1.0), std::invalid_argument);
}

}  // namespace
}  // namespace ft_detector

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
