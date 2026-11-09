#include "ft_detector/detection_output.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <set>
#include <stdexcept>

#include <opencv2/calib3d.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace ft_detector {
namespace {

// 检查三维向量，避免 NaN 或无穷大进入消息和 TF。
bool finiteVector(const cv::Vec3d& value) {
  return std::isfinite(value[0]) && std::isfinite(value[1]) && std::isfinite(value[2]);
}

// 将 PnP 的旋转向量和平移转换为 ROS Pose，失败时交给上层过滤。
bool convertPose(const ArmorPose& input, geometry_msgs::Pose& output) {
  if (!finiteVector(input.rvec) || !finiteVector(input.tvec) || input.tvec[2] <= 0.0) return false;
  try {
    // Rodrigues 得到旋转矩阵，再转换为归一化四元数。
    cv::Matx33d rotation;
    cv::Rodrigues(input.rvec, rotation);
    if (!std::all_of(std::begin(rotation.val), std::end(rotation.val),
                     [](double value) { return std::isfinite(value); })) return false;
    const tf2::Matrix3x3 basis(rotation(0, 0), rotation(0, 1), rotation(0, 2),
                               rotation(1, 0), rotation(1, 1), rotation(1, 2),
                               rotation(2, 0), rotation(2, 1), rotation(2, 2));
    tf2::Quaternion quaternion;
    basis.getRotation(quaternion);
    const double norm_squared = quaternion.length2();
    if (!std::isfinite(norm_squared) || norm_squared <= 0.0) return false;
    quaternion.normalize();
    // 平移直接采用相机系中的米制坐标。
    output.position.x = input.tvec[0];
    output.position.y = input.tvec[1];
    output.position.z = input.tvec[2];
    output.orientation.x = quaternion.x();
    output.orientation.y = quaternion.y();
    output.orientation.z = quaternion.z();
    output.orientation.w = quaternion.w();
    return true;
  } catch (const cv::Exception&) {
    return false;
  }
}

}  // namespace

// 从同一批候选和位姿构造检测消息与 TF，保证两路输出一致。
DetectionOutput buildDetectionOutput(const std_msgs::Header& image_header,
                                     const std::vector<ArmorCandidate>& armors,
                                     const std::vector<PnpResult>& poses,
                                     const std::string& child_frame_prefix) {
  if (armors.size() != poses.size()) throw std::invalid_argument("One PnP result is required per candidate");
  DetectionOutput output;
  output.frame.header = image_header;
  if (image_header.frame_id.empty() || child_frame_prefix.empty()) return output;
  std::set<std::uint32_t> used_ids;
  // 逐个检查有效位姿、候选 ID、评分和角点；不合格项不发布。
  for (std::size_t i = 0; i < armors.size(); ++i) {
    const auto& armor = armors[i];
    const auto& result = poses[i];
    if (!result.valid || result.pose.id != armor.id || used_ids.count(armor.id) != 0 ||
        !std::isfinite(armor.confidence) || armor.confidence < 0.0F || armor.confidence > 1.0F ||
        !std::all_of(armor.corners.begin(), armor.corners.end(), [](const cv::Point2f& point) {
          return std::isfinite(point.x) && std::isfinite(point.y);
        })) continue;
    const std::string child = child_frame_prefix + std::to_string(armor.id);
    if (child == image_header.frame_id) continue;
    ft_detection::Detection detection;
    if (!convertPose(result.pose, detection.pose)) continue;
    // 保留帧内 ID 和像素角点，PnP 过滤后不重新编号。
    detection.id = armor.id;
    detection.is_red = armor.is_red;
    detection.is_large_armor = armor.is_large_armor;
    detection.confidence = armor.confidence;
    for (std::size_t j = 0; j < armor.corners.size(); ++j) {
      detection.corners[j].x = armor.corners[j].x;
      detection.corners[j].y = armor.corners[j].y;
      detection.corners[j].z = 0.0F;  // Point32 承载二维像素；不是第三个空间坐标。
    }

    // TF 复用已经转换的 Pose，使用输入图像的时间戳与父坐标系。
    geometry_msgs::TransformStamped transform;
    transform.header = output.frame.header;
    transform.child_frame_id = child;
    transform.transform.translation.x = detection.pose.position.x;
    transform.transform.translation.y = detection.pose.position.y;
    transform.transform.translation.z = detection.pose.position.z;
    transform.transform.rotation = detection.pose.orientation;
    output.frame.detections.push_back(detection);
    output.transforms.push_back(transform);
    used_ids.insert(armor.id);
  }
  return output;
}

}  // namespace ft_detector
