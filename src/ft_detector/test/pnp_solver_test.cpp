#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/calib3d.hpp>

#include "ft_detector/debug_draw.hpp"
#include "ft_detector/pnp_solver.hpp"

namespace ft_detector {
namespace {

// 合成场景的已知尺寸，仅用于测试，不是素材的实测值。
PnpParams testParams() {
  return {{0.135, 0.055, true}, {0.230, 0.055, true},
          0.05, 30.0, 3.0, 20.0, 2.0, 0.03};
}

CameraCalibration testCamera() {
  return {cv::Matx33d(1000, 0, 720, 0, 1000, 540, 0, 0, 1),
          {-0.08, 0.10, -0.001, 0.001, 0.0}, cv::Size(1440, 1080), "plumb_bob"};
}

ArmorCandidate projectCandidate(const PnpSolver& solver, const CameraCalibration& camera,
                                const cv::Vec3d& rvec, const cv::Vec3d& tvec, bool large = false) {
  ArmorCandidate armor;
  armor.id = 7;
  armor.is_large_armor = large;
  std::vector<cv::Point2d> pixels;
  cv::projectPoints(solver.objectPoints(large), rvec, tvec, camera.camera_matrix, camera.distortion, pixels);
  for (std::size_t i = 0; i < pixels.size(); ++i) armor.corners[i] = pixels[i];
  armor.center = (armor.corners[0] + armor.corners[1] + armor.corners[2] + armor.corners[3]) * 0.25F;
  return armor;
}

void expectPose(const PnpResult& result, const cv::Vec3d& rvec, const cv::Vec3d& tvec) {
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_LT(cv::norm(result.pose.tvec - tvec), 0.002);
  cv::Matx33d actual, expected;
  cv::Rodrigues(result.pose.rvec, actual);
  cv::Rodrigues(rvec, expected);
  EXPECT_LT(cv::norm(actual - expected), 0.002);
  EXPECT_LT(result.pose.reprojection_error_px, 0.001);
}

TEST(PnpGeometry, ObjectPointsMatchEndpointOrderAndMetres) {
  const auto points = PnpSolver(testParams()).objectPoints(false);
  ASSERT_EQ(points.size(), 4U);
  EXPECT_DOUBLE_EQ(points[0].x, -0.0675);
  EXPECT_DOUBLE_EQ(points[0].y, -0.0275);
  EXPECT_DOUBLE_EQ(points[1].x, 0.0675);
  EXPECT_DOUBLE_EQ(points[1].y, -0.0275);
  EXPECT_DOUBLE_EQ(points[2].y, 0.0275);
  EXPECT_DOUBLE_EQ(points[3].x, -0.0675);
  for (const auto& point : points) EXPECT_DOUBLE_EQ(point.z, 0.0);
}

TEST(PnpGeometry, MissingMeasurementsDoNotProducePose) {
  auto params = testParams();
  const auto camera = testCamera();
  const auto armor = projectCandidate(PnpSolver(params), camera, {0.1, 0.2, 0.0}, {0, 0, 3});
  params.small = {0.0, 0.0, false};
  const PnpSolver solver(params);
  const auto result = solver.solve(armor, camera);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "dimensions_unconfirmed");
  EXPECT_THROW(solver.objectPoints(false), std::invalid_argument);
}

TEST(PnpGeometry, PositiveUnconfirmedDimensionsStayDisabledWithoutExplicitPermission) {
  auto params = testParams();
  const auto camera = testCamera();
  const auto armor = projectCandidate(PnpSolver(params), camera, {0.1,0.2,0}, {0,0,3});
  params.small.confirmed = false;
  const auto result = PnpSolver(params).solve(armor, camera);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "dimensions_unconfirmed");
}

TEST(PnpGeometry, ProvisionalDimensionsRequireOptInAndMarkTheResult) {
  auto params = testParams();
  const auto camera = testCamera();
  const cv::Vec3d rvec(0.1,0.2,0), tvec(0,0,3);
  const auto armor = projectCandidate(PnpSolver(params), camera, rvec, tvec);
  params.small.confirmed = false;
  params.allow_provisional_dimensions = true;
  const auto result = PnpSolver(params).solve(armor, camera);
  expectPose(result, rvec, tvec);
  EXPECT_TRUE(result.provisional);
}

TEST(PnpGeometry, ZeroDimensionsAreRejectedEvenWhenProvisionalIsAllowed) {
  auto params = testParams();
  const auto camera = testCamera();
  const auto armor = projectCandidate(PnpSolver(params), camera, {0.1,0.2,0}, {0,0,3});
  params.small = {0.0,0.0,false};
  params.allow_provisional_dimensions = true;
  const auto result = PnpSolver(params).solve(armor, camera);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "dimensions_unconfirmed");
}

TEST(PnpGeometry, ConfirmedGeometryIsNotMarkedProvisionalWhenOptInIsEnabled) {
  auto params = testParams();
  params.allow_provisional_dimensions = true;
  const auto camera = testCamera();
  const PnpSolver solver(params);
  const auto result = solver.solve(projectCandidate(solver, camera, {0.1,0.2,0}, {0,0,3}), camera);
  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.provisional);
}

TEST(PnpGeometry, ConfirmationIsIndependentForSmallAndLarge) {
  auto params = testParams();
  const auto camera = testCamera();
  const PnpSolver known(params);
  const auto small = projectCandidate(known, camera, {0.1, 0.2, 0.0}, {0, 0, 3});
  const auto large = projectCandidate(known, camera, {0.1, 0.2, 0.0}, {0, 0, 3}, true);
  params.large.confirmed = false;
  const PnpSolver solver(params);
  EXPECT_TRUE(solver.solve(small, camera).valid);
  EXPECT_FALSE(solver.solve(large, camera).valid);
}

TEST(PnpGeometry, WrongScaleCanStillReprojectCorrectly) {
  auto params = testParams();
  const auto camera = testCamera();
  const cv::Vec3d rvec(0.1, -0.2, 0.03), tvec(0.1, -0.04, 3.0);
  const auto armor = projectCandidate(PnpSolver(params), camera, rvec, tvec);
  params.small.width_m *= 2.0;
  params.small.height_m *= 2.0;
  const auto result = PnpSolver(params).solve(armor, camera);
  expectPose(result, rvec, tvec * 2.0);
}

TEST(PnpSolver, RecoversFrontalPose) {
  const PnpSolver solver(testParams());
  auto camera = testCamera(); camera.distortion.clear();
  const cv::Vec3d rvec(0, 0, 0), tvec(0, 0, 2.5);
  const auto result = solver.solve(projectCandidate(solver, camera, rvec, tvec), camera);
  expectPose(result, rvec, tvec);
  EXPECT_EQ(result.pose.id, 7U);
}

TEST(PnpSolver, FrontalFloatRoundingHasBoundedOrientationError) {
  const PnpSolver solver(testParams());
  auto camera = testCamera(); camera.distortion.clear();
  const cv::Vec3d tvec(0, 0, 3);
  const auto result = solver.solve(projectCandidate(solver, camera, {0,0,0}, tvec), camera);
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_LT(cv::norm(result.pose.tvec - tvec), 0.002);
  // 3 m 的角点转为 Point2f 后存在舍入；近正视倾角不要求数学上精确为零。
  cv::Matx33d actual;
  cv::Rodrigues(result.pose.rvec, actual);
  EXPECT_LT(cv::norm(actual - cv::Matx33d::eye()), 0.01);
  EXPECT_LT(result.pose.reprojection_error_px, 0.001);
}

TEST(PnpSolver, RecoversTiltAndDistortionForBothSizes) {
  const PnpSolver solver(testParams());
  const auto camera = testCamera();
  for (bool large : {false, true}) {
    for (double yaw : {-0.45, 0.15, 0.5}) {
      const cv::Vec3d rvec(0.12, yaw, -0.04), tvec(0.2, -0.1, 2.5);
      expectPose(solver.solve(projectCandidate(solver, camera, rvec, tvec, large), camera), rvec, tvec);
    }
  }
}

TEST(PnpSolver, RecoversRationalCameraModel) {
  const PnpSolver solver(testParams());
  auto camera = testCamera();
  camera.distortion_model = "rational_polynomial";
  camera.distortion = {-0.08, 0.10, -0.001, 0.001, 0.0, 0.01, -0.01, 0.0};
  const cv::Vec3d rvec(0.1, 0.2, 0.05), tvec(0.1, 0.05, 2);
  expectPose(solver.solve(projectCandidate(solver, camera, rvec, tvec), camera), rvec, tvec);
}

TEST(PnpSolver, AxisPixelsUseSamePoseAndCalibration) {
  const PnpSolver solver(testParams());
  const auto camera = testCamera();
  const cv::Vec3d rvec(0.1, 0.3, 0.02), tvec(0.1, -0.1, 3);
  const auto result = solver.solve(projectCandidate(solver, camera, rvec, tvec), camera);
  ASSERT_TRUE(result.valid) << result.reason;
  std::vector<cv::Point2d> expected;
  cv::projectPoints(std::vector<cv::Point3d>{{0,0,0}, {.03,0,0}, {0,.03,0}, {0,0,.03}},
                    rvec, tvec, camera.camera_matrix, camera.distortion, expected);
  for (std::size_t i = 0; i < expected.size(); ++i)
    EXPECT_LT(cv::norm(expected[i] - result.pose.axis_pixels[i]), 0.01);
}

TEST(PnpSolver, SmallPixelChangesKeepNonFrontalPoseContinuous) {
  const PnpSolver solver(testParams());
  const auto camera = testCamera();
  const cv::Vec3d rvec(0.15,0.4,0.05), tvec(0.1,-0.05,3);
  const auto original = projectCandidate(solver, camera, rvec, tvec);
  for (float offset : {-0.03F, 0.0F, 0.03F}) {
    auto armor = original;
    armor.corners[1].x += offset;
    const auto result = solver.solve(armor, camera);
    ASSERT_TRUE(result.valid) << result.reason;
    EXPECT_LT(cv::norm(result.pose.tvec - tvec), 0.02);
    cv::Matx33d actual, expected;
    cv::Rodrigues(result.pose.rvec, actual); cv::Rodrigues(rvec, expected);
    EXPECT_LT(cv::norm(actual - expected), 0.03);
  }
}

TEST(PnpSolver, RejectsNonFiniteOrOutOfImagePoints) {
  const PnpSolver solver(testParams());
  const auto camera = testCamera();
  auto armor = projectCandidate(solver, camera, {0.1,0.2,0}, {0,0,3});
  armor.corners[0].x = std::numeric_limits<float>::quiet_NaN();
  EXPECT_EQ(solver.solve(armor, camera).reason, "corner_bounds");
  armor.corners[0].x = -1;
  EXPECT_EQ(solver.solve(armor, camera).reason, "corner_bounds");
}

TEST(PnpSolver, RejectsCrossedReversedOrRepeatedCorners) {
  const PnpSolver solver(testParams());
  const auto camera = testCamera();
  const auto original = projectCandidate(solver, camera, {0.1,0.2,0}, {0,0,3});
  auto armor = original;
  std::swap(armor.corners[0], armor.corners[1]);
  EXPECT_EQ(solver.solve(armor, camera).reason, "corner_order");
  armor = original; std::swap(armor.corners[1], armor.corners[3]);
  EXPECT_EQ(solver.solve(armor, camera).reason, "corner_order");
  armor = original; armor.corners[1] = armor.corners[0];
  EXPECT_EQ(solver.solve(armor, camera).reason, "corner_order");
}

TEST(PnpSolver, RejectsVeryShortEdge) {
  const PnpSolver solver(testParams());
  ArmorCandidate armor;
  armor.corners = {{{100,100}, {140,100}, {140,101}, {100,101}}};
  EXPECT_EQ(solver.solve(armor, testCamera()).reason, "short_edge");
}

TEST(PnpSolver, RejectsTooSmallQuadrilateral) {
  const PnpSolver solver(testParams());
  ArmorCandidate armor;
  armor.corners = {{{100,100}, {103,100}, {103,103}, {100,103}}};
  EXPECT_EQ(solver.solve(armor, testCamera()).reason, "corner_order");
}

TEST(PnpSolver, RejectsDepthOutsideConfiguredInterval) {
  auto params = testParams();
  const auto camera = testCamera();
  const auto armor = projectCandidate(PnpSolver(params), camera, {0.1,0.2,0}, {0,0,3});
  params.max_depth_m = 1.0;
  EXPECT_EQ(PnpSolver(params).solve(armor, camera).reason, "no_positive_solution");
  params = testParams(); params.min_depth_m = 5.0;
  EXPECT_EQ(PnpSolver(params).solve(armor, camera).reason, "no_positive_solution");
}

TEST(PnpSolver, RejectsInconsistentReprojection) {
  auto params = testParams();
  const auto camera = testCamera();
  auto armor = projectCandidate(PnpSolver(params), camera, {0.1,0.2,0}, {0,0,3});
  armor.corners[1].x += 12;
  params.max_reprojection_error_px = 0.001;
  const auto result = PnpSolver(params).solve(armor, camera);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "reprojection");
}

TEST(PnpCalibration, RejectsUnsupportedDistortionModelAndLengths) {
  auto camera = testCamera();
  camera.distortion_model = "equidistant"; camera.distortion.resize(4);
  EXPECT_FALSE(PnpSolver::validCalibration(camera));
  camera = testCamera(); camera.distortion.resize(6);
  EXPECT_FALSE(PnpSolver::validCalibration(camera));
  camera = testCamera(); camera.distortion[0] = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(PnpSolver::validCalibration(camera));
}

// 取得两个候选只用于构造“上一帧靠近次优分支”的受控输入。
std::vector<ArmorPose> ippePoses(const PnpSolver& solver, const ArmorCandidate& armor,
                                const CameraCalibration& camera) {
  std::vector<cv::Mat> rotations, translations;
  const std::vector<cv::Point2d> pixels(armor.corners.begin(), armor.corners.end());
  cv::solvePnPGeneric(solver.objectPoints(armor.is_large_armor), pixels, camera.camera_matrix,
                     camera.distortion, rotations, translations, false, cv::SOLVEPNP_IPPE);
  std::vector<ArmorPose> poses;
  for (std::size_t i = 0; i < rotations.size(); ++i) {
    ArmorPose pose;
    pose.rvec = {rotations[i].at<double>(0), rotations[i].at<double>(1), rotations[i].at<double>(2)};
    pose.tvec = {translations[i].at<double>(0), translations[i].at<double>(1), translations[i].at<double>(2)};
    std::vector<cv::Point2d> projected;
    cv::projectPoints(solver.objectPoints(armor.is_large_armor), pose.rvec, pose.tvec,
                      camera.camera_matrix, camera.distortion, projected);
    double squared = 0.0;
    for (std::size_t j = 0; j < pixels.size(); ++j) {
      const auto delta = projected[j] - pixels[j];
      squared += delta.dot(delta);
    }
    pose.reprojection_error_px = std::sqrt(squared / pixels.size());
    poses.push_back(pose);
  }
  return poses;
}

TEST(PnpContinuity, SelectsNearbyValidBranchAndProjectsItsOwnAxes) {
  auto params = testParams(); params.ambiguity_error_margin_px = 1.0;
  const PnpSolver solver(params);
  const auto camera = testCamera();
  const auto armor = projectCandidate(solver, camera, {0.25,0.4,0.05}, {0.1,-0.1,3});
  auto candidates = ippePoses(solver, armor, camera);
  ASSERT_EQ(candidates.size(), 2U);
  ASSERT_GT(candidates[1].reprojection_error_px, candidates[0].reprojection_error_px);
  ASSERT_LT(candidates[1].reprojection_error_px - candidates[0].reprojection_error_px, 1.0);
  candidates[1].id = 999;  // 参考 ID 不得覆盖当前帧 ID。
  const auto result = solver.solve(armor, camera, &candidates[1]);
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_TRUE(result.continuity_selected);
  EXPECT_EQ(result.pose.id, armor.id);
  EXPECT_LT(cv::norm(result.pose.rvec - candidates[1].rvec), 1e-7);
  EXPECT_LT(cv::norm(result.pose.tvec - candidates[1].tvec), 1e-7);
  std::vector<cv::Point2d> expected;
  const double a = params.axis_length_m;
  cv::projectPoints(std::vector<cv::Point3d>{{0,0,0},{a,0,0},{0,a,0},{0,0,a}},
                    result.pose.rvec, result.pose.tvec, camera.camera_matrix, camera.distortion, expected);
  for (std::size_t i = 0; i < expected.size(); ++i) EXPECT_LT(cv::norm(expected[i] - result.pose.axis_pixels[i]), 1e-7);
}

TEST(PnpContinuity, ErrorMarginPreventsHistoryOverridingClearImageEvidence) {
  auto params = testParams(); params.ambiguity_error_margin_px = 0.0;
  const PnpSolver solver(params);
  const auto camera = testCamera();
  const auto armor = projectCandidate(solver, camera, {0.25,0.4,0.05}, {0.1,-0.1,3});
  const auto candidates = ippePoses(solver, armor, camera);
  ASSERT_EQ(candidates.size(), 2U);
  const auto result = solver.solve(armor, camera, &candidates[1]);
  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.continuity_selected);
  EXPECT_LT(cv::norm(result.pose.rvec - candidates[0].rvec), 1e-7);
}

TEST(PnpContinuity, AbsoluteErrorThresholdStillRejectsThePriorBranch) {
  auto params = testParams(); params.ambiguity_error_margin_px = 10.0;
  const auto camera = testCamera();
  const auto armor = projectCandidate(PnpSolver(params), camera, {0.25,0.4,0.05}, {0.1,-0.1,3});
  const auto candidates = ippePoses(PnpSolver(params), armor, camera);
  ASSERT_EQ(candidates.size(), 2U);
  params.max_reprojection_error_px = (candidates[0].reprojection_error_px + candidates[1].reprojection_error_px) / 2.0;
  const auto result = PnpSolver(params).solve(armor, camera, &candidates[1]);
  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.continuity_selected);
  EXPECT_LE(result.pose.reprojection_error_px, params.max_reprojection_error_px);
  EXPECT_LT(cv::norm(result.pose.rvec - candidates[0].rvec), 1e-7);
}

TEST(PnpContinuity, InvalidCurrentObservationCannotReuseAnOldPose) {
  auto params = testParams(); params.ambiguity_error_margin_px = 1.0;
  const auto camera = testCamera();
  auto armor = projectCandidate(PnpSolver(params), camera, {0.1,0.2,0}, {0,0,3});
  const auto reference = PnpSolver(params).solve(armor, camera).pose;
  armor.corners[1].x += 12;
  params.max_reprojection_error_px = 0.001;
  const auto result = PnpSolver(params).solve(armor, camera, &reference);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "reprojection");
}

TEST(PnpContinuity, InvalidReferenceFallsBackToTheCurrentFrameMinimum) {
  auto params = testParams(); params.ambiguity_error_margin_px = 1.0;
  const PnpSolver solver(params);
  const auto camera = testCamera();
  const auto armor = projectCandidate(solver, camera, {0.25,0.4,0.05}, {0.1,-0.1,3});
  const auto baseline = solver.solve(armor, camera);
  auto reference = baseline.pose; reference.rvec[0] = std::numeric_limits<double>::quiet_NaN();
  const auto result = solver.solve(armor, camera, &reference);
  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.continuity_selected);
  EXPECT_LT(cv::norm(result.pose.rvec - baseline.pose.rvec), 1e-7);
}

TEST(PnpContinuity, RejectsNegativeOrNonFiniteErrorMargin) {
  for (double margin : {-1.0, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
    auto params = testParams(); params.ambiguity_error_margin_px = margin;
    EXPECT_THROW(PnpSolver solver(params), std::invalid_argument);
  }
}

TEST(PnpCalibration, RejectsInvalidIntrinsicsAndImageSize) {
  auto camera = testCamera(); camera.camera_matrix(0,0) = 0;
  EXPECT_FALSE(PnpSolver::validCalibration(camera));
  camera = testCamera(); camera.camera_matrix(2,2) = 2;
  EXPECT_FALSE(PnpSolver::validCalibration(camera));
  camera = testCamera(); camera.camera_matrix(0,1) = 1;
  EXPECT_FALSE(PnpSolver::validCalibration(camera));
  camera = testCamera(); camera.image_size.width = 0;
  EXPECT_FALSE(PnpSolver::validCalibration(camera));
}

TEST(PnpParameters, RejectsInvalidDimensionsAndThresholds) {
  auto params = testParams(); params.small.width_m = 0;
  EXPECT_THROW(PnpSolver solver(params), std::invalid_argument);
  params = testParams(); params.axis_length_m = 0;
  EXPECT_THROW(PnpSolver solver(params), std::invalid_argument);
  params = testParams(); params.max_depth_m = params.min_depth_m;
  EXPECT_THROW(PnpSolver solver(params), std::invalid_argument);
  params = testParams(); params.large.height_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(PnpSolver solver(params), std::invalid_argument);
}

TEST(PnpDrawing, AxesOverlayAndStatusDoNotTouchOriginalPixels) {
  cv::Mat original = cv::Mat::zeros(1080, 1440, CV_8UC3);
  cv::Mat debug = original.clone();
  const PnpSolver solver(testParams());
  const auto camera = testCamera();
  const auto pose = solver.solve(projectCandidate(solver, camera, {0.1,0.2,0}, {0,0,3}), camera);
  ASSERT_TRUE(pose.valid);
  drawPoses(debug, {pose}, "ready", true);
  EXPECT_EQ(cv::norm(original, cv::NORM_INF), 0.0);
  EXPECT_GT(cv::norm(debug, cv::NORM_INF), 0.0);
}

}  // namespace
}  // namespace ft_detector

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
