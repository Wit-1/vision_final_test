#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <limits>
#include <string>

#include <gtest/gtest.h>
#include <opencv2/imgproc.hpp>

#include "ft_detector/debug_draw.hpp"
#include "ft_detector/traditional_detector.hpp"

namespace ft_detector {
namespace {

// 可控合成场景的参数，不是运行时的第二套默认值。
TraditionalParams testParams() {
  return {{85, 135, 65, 255, 130, 255}, {0, 35, 80, 255, 210, 255},
          {160, 179, 80, 255, 210, 255}, 20.0, 10.0, 2.5, 15.0, 0.45, 30.0, 1,
          0.75, 12.0, 0.35, 1.4, 5.8, 3.4, 0.5, 0.1};
}

cv::Mat scene() { return cv::Mat::zeros(500, 1000, CV_8UC3); }

void light(cv::Mat& image, cv::Point2f center, double length = 40.0, double width = 8.0,
           int hue = 110, double angle_deg = 0.0, int value = 250) {
  cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(hue, 230, value)), bgr;
  cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
  const auto pixel = bgr.at<cv::Vec3b>(0, 0);
  const cv::Scalar color(pixel[0], pixel[1], pixel[2]);
  const double angle = angle_deg * CV_PI / 180.0;
  const cv::Point2f axis(std::sin(angle), std::cos(angle));
  const cv::Point2f perpendicular(std::cos(angle), -std::sin(angle));
  std::vector<cv::Point> vertices;
  for (const auto& sign : std::vector<cv::Point2f>{{-1, -1}, {1, -1}, {1, 1}, {-1, 1}}) {
    vertices.emplace_back(center + axis * static_cast<float>(sign.y * length / 2.0) +
                          perpendicular * static_cast<float>(sign.x * width / 2.0));
  }
  cv::fillConvexPoly(image, vertices, color);
}

bool reasonExists(const DetectionResult& result, const std::string& reason) {
  return std::any_of(result.pairs.begin(), result.pairs.end(), [&](const PairDecision& pair) {
    return pair.reason == reason;
  });
}

TEST(Angles, UnorientedAxisWrapsAtNinety) {
  EXPECT_DOUBLE_EQ(normalizeLightAngle(90.0), -90.0);
  EXPECT_DOUBLE_EQ(normalizeLightAngle(-91.0), 89.0);
  EXPECT_DOUBLE_EQ(normalizeLightAngle(360.0), 0.0);
  EXPECT_DOUBLE_EQ(lightAngleDifference(89.0, -89.0), 2.0);
}

TEST(TraditionalDetector, NoLightsMeansNoArmor) {
  const auto result = TraditionalDetector(testParams()).detect(scene());
  EXPECT_TRUE(result.armors.empty());
  EXPECT_EQ(cv::countNonZero(result.blue_mask), 0);
  EXPECT_EQ(cv::countNonZero(result.red_mask), 0);
}

TEST(TraditionalDetector, FindsSmallAndLargeWithFrameLocalIds) {
  auto image = scene();
  light(image, {100, 150}); light(image, {200, 150});
  light(image, {650, 200}); light(image, {810, 200});
  const auto result = TraditionalDetector(testParams()).detect(image);
  ASSERT_EQ(result.armors.size(), 2U);
  EXPECT_EQ(result.armors[0].id, 0U);
  EXPECT_EQ(result.armors[1].id, 1U);
  EXPECT_FALSE(result.armors[0].is_large_armor);
  EXPECT_TRUE(result.armors[1].is_large_armor);
  EXPECT_FALSE(result.armors[0].is_red);
  const auto& corners = result.armors[0].corners;
  EXPECT_LT(corners[0].x, corners[1].x);
  EXPECT_LT(corners[0].y, corners[3].y);
  EXPECT_LT(corners[1].y, corners[2].y);
  EXPECT_NEAR(corners[0].y, 130.0, 1.0);
  EXPECT_NEAR(corners[2].y, 170.0, 1.0);
}

TEST(TraditionalDetector, RedHueIntervalsJoinIntoOneMask) {
  auto image = scene();
  light(image, {100, 200}, 40, 8, 5);
  light(image, {200, 200}, 40, 8, 175);
  const auto result = TraditionalDetector(testParams()).detect(image);
  ASSERT_EQ(result.armors.size(), 1U);
  EXPECT_TRUE(result.armors[0].is_red);
  EXPECT_GT(cv::countNonZero(result.red_mask), 0);
  EXPECT_EQ(cv::countNonZero(result.blue_mask), 0);
}

TEST(TraditionalDetector, RetainsBrightYellowishRedCore) {
  auto image = scene();
  light(image, {100, 200}, 40, 8, 30);
  light(image, {200, 200}, 40, 8, 30);
  const auto result = TraditionalDetector(testParams()).detect(image);
  ASSERT_EQ(result.armors.size(), 1U);
  EXPECT_TRUE(result.armors[0].is_red);
}

TEST(TraditionalDetector, RejectsDimRedReflectionInColorStage) {
  auto image = scene();
  light(image, {100, 200}, 40, 8, 5, 0, 200);
  light(image, {200, 200}, 40, 8, 5, 0, 200);
  const auto result = TraditionalDetector(testParams()).detect(image);
  EXPECT_EQ(cv::countNonZero(result.red_mask), 0);
  EXPECT_TRUE(result.armors.empty());
}

TEST(TraditionalDetector, DoesNotPairDifferentColors) {
  auto image = scene();
  light(image, {100, 200}, 40, 8, 5); light(image, {200, 200});
  const auto result = TraditionalDetector(testParams()).detect(image);
  EXPECT_TRUE(result.armors.empty());
  EXPECT_TRUE(reasonExists(result, "color"));
}

TEST(TraditionalDetector, KeepsRedAndBlueTargetsInSameFrame) {
  auto image = scene();
  light(image, {100, 150}); light(image, {200, 150});
  light(image, {650, 200}, 40, 8, 5); light(image, {750, 200}, 40, 8, 175);
  const auto result = TraditionalDetector(testParams()).detect(image);
  ASSERT_EQ(result.armors.size(), 2U);
  EXPECT_FALSE(result.armors[0].is_red);
  EXPECT_TRUE(result.armors[1].is_red);
}

TEST(TraditionalDetector, RejectsHalfLightRatherThanExtendingIt) {
  auto image = scene();
  light(image, {100, 200}); light(image, {200, 200}, 22, 4);
  const auto result = TraditionalDetector(testParams()).detect(image);
  ASSERT_EQ(result.lights.size(), 2U);
  EXPECT_TRUE(result.armors.empty());
  EXPECT_TRUE(reasonExists(result, "length_ratio"));
}

TEST(TraditionalDetector, RejectsHeightMismatch) {
  auto image = scene();
  light(image, {100, 200}); light(image, {200, 240});
  const auto result = TraditionalDetector(testParams()).detect(image);
  EXPECT_TRUE(result.armors.empty());
  EXPECT_TRUE(reasonExists(result, "center_dy"));
}

TEST(TraditionalDetector, RejectsAngleMismatch) {
  auto image = scene();
  light(image, {100, 200}, 40, 8, 110, -15);
  light(image, {200, 200}, 40, 8, 110, 15);
  const auto result = TraditionalDetector(testParams()).detect(image);
  EXPECT_TRUE(result.armors.empty());
  EXPECT_TRUE(reasonExists(result, "angle"));
}

TEST(TraditionalDetector, SlantedLightsKeepTopBottomOrder) {
  auto image = scene();
  light(image, {100, 200}, 40, 8, 110, 20);
  light(image, {200, 200}, 40, 8, 110, 20);
  const auto result = TraditionalDetector(testParams()).detect(image);
  ASSERT_EQ(result.armors.size(), 1U);
  for (const auto& item : result.lights) {
    EXPECT_GE(item.angle_deg, -90.0);
    EXPECT_LT(item.angle_deg, 90.0);
    EXPECT_LT(item.top.y, item.bottom.y);
    EXPECT_NEAR(item.angle_deg, 20.0, 2.0);
  }
}

TEST(TraditionalDetector, RejectsSeparatedAndTooCloseLights) {
  for (const float second_x : {130.0F, 400.0F}) {
    auto image = scene();
    light(image, {100, 200}); light(image, {second_x, 200});
    const auto result = TraditionalDetector(testParams()).detect(image);
    EXPECT_TRUE(result.armors.empty());
    EXPECT_TRUE(reasonExists(result, "spacing"));
  }
}

TEST(TraditionalDetector, RejectsHorizontalStripAndBroadReflection) {
  auto image = scene();
  light(image, {100, 100}, 100, 8, 110, 90);
  light(image, {200, 100}, 28, 16, 5);
  const auto result = TraditionalDetector(testParams()).detect(image);
  EXPECT_TRUE(result.lights.empty());
  EXPECT_TRUE(result.armors.empty());
}

TEST(TraditionalDetector, RejectsImageBorderTruncation) {
  auto image = scene();
  light(image, {4, 200}); light(image, {104, 200});
  const auto result = TraditionalDetector(testParams()).detect(image);
  EXPECT_TRUE(result.armors.empty());
  ASSERT_EQ(result.lights.size(), 1U);
  EXPECT_TRUE(std::any_of(result.rejected_lights.begin(), result.rejected_lights.end(),
                          [](const RejectedLight& item) { return item.reason == "border"; }));
}

TEST(TraditionalDetector, AvoidsSpanningAndAmbiguousThreeLightPairs) {
  auto image = scene();
  light(image, {100, 200}); light(image, {200, 200}); light(image, {300, 200});
  const auto result = TraditionalDetector(testParams()).detect(image);
  EXPECT_TRUE(result.armors.empty());
  EXPECT_TRUE(reasonExists(result, "intervening_light"));
  EXPECT_TRUE(reasonExists(result, "ambiguous"));
}

TEST(TraditionalDetector, SharedLightBelongsOnlyToBetterPair) {
  auto image = scene();
  light(image, {100, 200}); light(image, {200, 200}); light(image, {300, 210});
  const auto result = TraditionalDetector(testParams()).detect(image);
  ASSERT_EQ(result.armors.size(), 1U);
  EXPECT_NEAR(result.armors[0].center.x, 150.0, 1.0);
  EXPECT_TRUE(reasonExists(result, "shared_light"));
}

TEST(TraditionalDetector, ThresholdChangeActuallyChangesResult) {
  auto image = scene();
  light(image, {100, 200}); light(image, {200, 200});
  auto params = testParams();
  TraditionalDetector detector(params);
  ASSERT_EQ(detector.detect(image).armors.size(), 1U);
  params.min_contour_area = 1000;
  detector.setParams(params);
  EXPECT_TRUE(detector.detect(image).armors.empty());
}

TEST(TraditionalDetector, InvalidParametersAndImageAreRejected) {
  auto params = testParams();
  params.max_pair_angle_deg = 0.0;
  EXPECT_THROW(TraditionalDetector detector(params), std::invalid_argument);
  params = testParams(); params.min_light_fill = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(TraditionalDetector detector(params), std::invalid_argument);
  params = testParams(); params.red_low.h_max = 180;
  EXPECT_THROW(TraditionalDetector detector(params), std::invalid_argument);
  TraditionalDetector detector(testParams());
  EXPECT_THROW(detector.detect(cv::Mat()), std::invalid_argument);
  EXPECT_THROW(detector.detect(cv::Mat::zeros(100, 100, CV_8UC1)), std::invalid_argument);
}

TEST(DebugDraw, KeepsSharedInputUnchanged) {
  auto image = scene();
  light(image, {100, 200}); light(image, {200, 200});
  const auto original = image.clone();
  const auto result = TraditionalDetector(testParams()).detect(image);
  const auto debug = drawDetections(image, result, true);
  EXPECT_EQ(cv::norm(image, original, cv::NORM_INF), 0.0);
  EXPECT_GT(cv::norm(debug, image, cv::NORM_INF), 0.0);
  EXPECT_EQ(debug.size(), image.size());
  EXPECT_EQ(debug.type(), CV_8UC3);
}

}  // namespace
}  // namespace ft_detector

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
