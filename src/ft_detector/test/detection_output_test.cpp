#include <cmath>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>
#include <opencv2/calib3d.hpp>
#include <tf2/LinearMath/Matrix3x3.h>

#include "ft_detector/detection_output.hpp"

namespace ft_detector {
namespace {

std_msgs::Header header() {
  std_msgs::Header value;
  value.seq = 47;
  value.stamp = ros::Time(123,456789);
  value.frame_id = "test_camera";
  return value;
}

ArmorCandidate armor(std::uint32_t id = 7) {
  ArmorCandidate value;
  value.id = id;
  value.is_red = true;
  value.confidence = 0.72F;
  value.corners = {{{100,200},{180,202},{178,240},{102,238}}};
  return value;
}

PnpResult pose(std::uint32_t id = 7) {
  PnpResult value;
  value.valid = true;
  value.pose.id = id;
  value.pose.rvec = {0.1,0.2,0.3};
  value.pose.tvec = {0.2,-0.1,3.0};
  return value;
}

TEST(DetectionOutput, CopiesAllFieldsAndKeepsImageHeaderAndPixelOrder) {
  auto first = armor(); auto second = armor(11);
  second.is_red = false; second.is_large_armor = true; second.confidence = 0.91F;
  const auto h = header();
  const auto output = buildDetectionOutput(h,{first,second},{pose(),pose(11)},"armor_");
  EXPECT_EQ(output.frame.header.seq,h.seq);
  EXPECT_EQ(output.frame.header.stamp,h.stamp);
  EXPECT_EQ(output.frame.header.frame_id,h.frame_id);
  ASSERT_EQ(output.frame.detections.size(),2U); ASSERT_EQ(output.transforms.size(),2U);
  EXPECT_EQ(output.frame.detections[0].id,7U); EXPECT_EQ(output.frame.detections[1].id,11U);
  EXPECT_TRUE(output.frame.detections[0].is_red); EXPECT_FALSE(output.frame.detections[1].is_red);
  EXPECT_FALSE(output.frame.detections[0].is_large_armor); EXPECT_TRUE(output.frame.detections[1].is_large_armor);
  EXPECT_FLOAT_EQ(output.frame.detections[0].confidence,0.72F);
  EXPECT_FLOAT_EQ(output.frame.detections[1].confidence,0.91F);
  for (std::size_t i=0;i<4;++i) {
    EXPECT_FLOAT_EQ(output.frame.detections[0].corners[i].x,first.corners[i].x);
    EXPECT_FLOAT_EQ(output.frame.detections[0].corners[i].y,first.corners[i].y);
    EXPECT_FLOAT_EQ(output.frame.detections[0].corners[i].z,0.0F);
  }
}

TEST(DetectionOutput, EmptyFrameStillCarriesImageTimestampAndFrame) {
  const auto output=buildDetectionOutput(header(),{},{},"armor_");
  EXPECT_EQ(output.frame.header.stamp,header().stamp);
  EXPECT_EQ(output.frame.header.frame_id,header().frame_id);
  EXPECT_TRUE(output.frame.detections.empty()); EXPECT_TRUE(output.transforms.empty());
}

TEST(DetectionOutput, FiltersFailedPnPWithoutLosingOtherValidDetections) {
  auto failed=pose(); failed.valid=false;
  const auto output=buildDetectionOutput(header(),{armor(),armor(11)},{failed,pose(11)},"armor_");
  ASSERT_EQ(output.frame.detections.size(),1U); ASSERT_EQ(output.transforms.size(),1U);
  EXPECT_EQ(output.frame.detections[0].id,11U); EXPECT_EQ(output.transforms[0].child_frame_id,"armor_11");
}

TEST(DetectionOutput, MessageAndTransformShareTheSamePoseInMetres) {
  const auto output=buildDetectionOutput(header(),{armor()},{pose()},"armor_");
  ASSERT_EQ(output.frame.detections.size(),1U);
  const auto& p=output.frame.detections[0].pose;
  const auto& tf=output.transforms[0];
  EXPECT_DOUBLE_EQ(p.position.x,0.2); EXPECT_DOUBLE_EQ(p.position.y,-0.1); EXPECT_DOUBLE_EQ(p.position.z,3.0);
  EXPECT_DOUBLE_EQ(tf.transform.translation.x,p.position.x);
  EXPECT_DOUBLE_EQ(tf.transform.translation.y,p.position.y);
  EXPECT_DOUBLE_EQ(tf.transform.translation.z,p.position.z);
  EXPECT_EQ(tf.transform.rotation,p.orientation);
  EXPECT_EQ(tf.header.stamp,output.frame.header.stamp);
  EXPECT_EQ(tf.header.frame_id,output.frame.header.frame_id);
  const auto& q=p.orientation;
  EXPECT_NEAR(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w,1.0,1e-12);
  const tf2::Matrix3x3 actual(tf2::Quaternion(q.x,q.y,q.z,q.w));
  cv::Matx33d expected; cv::Rodrigues(pose().pose.rvec,expected);
  for (int row=0;row<3;++row) for (int col=0;col<3;++col) EXPECT_NEAR(actual[row][col],expected(row,col),1e-12);
}

TEST(DetectionOutput, CandidateAndPoseArrayLengthsMustAgree) {
  EXPECT_THROW(buildDetectionOutput(header(),{armor()},{},"armor_"),std::invalid_argument);
}

TEST(DetectionOutput, DoesNotGuessWhenCandidateAndPoseIdsDisagree) {
  const auto output=buildDetectionOutput(header(),{armor()},{pose(99)},"armor_");
  EXPECT_TRUE(output.frame.detections.empty()); EXPECT_TRUE(output.transforms.empty());
}

TEST(DetectionOutput, FrameLocalIdsRemainUniqueWithoutRenumbering) {
  const auto output=buildDetectionOutput(header(),{armor(),armor(),armor(11)},{pose(),pose(),pose(11)},"armor_");
  ASSERT_EQ(output.frame.detections.size(),2U);
  EXPECT_EQ(output.frame.detections[0].id,7U); EXPECT_EQ(output.frame.detections[1].id,11U);
  EXPECT_EQ(output.transforms.size(),2U);
}

TEST(DetectionOutput, RejectsNonFinitePublishedFields) {
  for (int field=0;field<6;++field) {
    auto a=armor(); auto p=pose();
    if (field==0) a.confidence=std::numeric_limits<float>::quiet_NaN();
    if (field==1) a.corners[0].x=std::numeric_limits<float>::infinity();
    if (field==2) a.corners[1].y=std::numeric_limits<float>::quiet_NaN();
    if (field==3) p.pose.rvec[0]=std::numeric_limits<double>::quiet_NaN();
    if (field==4) p.pose.tvec[1]=std::numeric_limits<double>::infinity();
    if (field==5) a.confidence=std::numeric_limits<float>::infinity();
    const auto output=buildDetectionOutput(header(),{a},{p},"armor_");
    EXPECT_TRUE(output.frame.detections.empty()); EXPECT_TRUE(output.transforms.empty());
  }
}

TEST(DetectionOutput, RejectsOutOfRangeConfidence) {
  for (float confidence : {-0.1F,1.1F}) {
    auto a=armor(); a.confidence=confidence;
    EXPECT_TRUE(buildDetectionOutput(header(),{a},{pose()},"armor_").frame.detections.empty());
  }
}

TEST(DetectionOutput, RejectsNonPositiveDepthEvenIfMarkedValid) {
  for (double z : {0.0,-1.0}) {
    auto p=pose(); p.pose.tvec[2]=z;
    EXPECT_TRUE(buildDetectionOutput(header(),{armor()},{p},"armor_").frame.detections.empty());
  }
}

TEST(DetectionOutput, NoPoseWithoutAParentFrameOrChildPrefix) {
  auto h=header(); h.frame_id.clear();
  EXPECT_TRUE(buildDetectionOutput(h,{armor()},{pose()},"armor_").frame.detections.empty());
  EXPECT_TRUE(buildDetectionOutput(header(),{armor()},{pose()},"").frame.detections.empty());
}

TEST(DetectionOutput, RejectsSelfReferencingTransformForBothOutputs) {
  auto h=header(); h.frame_id="armor_7";
  const auto output=buildDetectionOutput(h,{armor()},{pose()},"armor_");
  EXPECT_TRUE(output.frame.detections.empty()); EXPECT_TRUE(output.transforms.empty());
}

}  // namespace
}  // namespace ft_detector

int main(int argc,char** argv) {
  testing::InitGoogleTest(&argc,argv);
  return RUN_ALL_TESTS();
}
