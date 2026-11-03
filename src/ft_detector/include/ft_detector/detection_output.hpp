#pragma once

#include <string>
#include <vector>

#include <ft_detection/DetectionFrame.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Header.h>

#include "ft_detector/pnp_solver.hpp"

namespace ft_detector {

// ROS 输出适配层；检测和 PnP 算法库本身不依赖生成的 ROS 消息。
struct DetectionOutput {
  ft_detection::DetectionFrame frame;
  std::vector<geometry_msgs::TransformStamped> transforms;
};

// armors[i] 与 poses[i] 对应。仅输出当前帧有效、有限且 ID 唯一的观测。
// 图像 header 原样传入；无有效观测时仍返回含该 header 的空检测帧。
DetectionOutput buildDetectionOutput(const std_msgs::Header& image_header,
                                     const std::vector<ArmorCandidate>& armors,
                                     const std::vector<PnpResult>& poses,
                                     const std::string& child_frame_prefix);

}  // namespace ft_detector
