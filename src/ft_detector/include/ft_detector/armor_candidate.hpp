#pragma once

#include <array>
#include <cstdint>

#include <opencv2/core.hpp>

namespace ft_detector {

// 两种检测方式共用的像素候选；第 3 点才把它送入 PnP。
struct ArmorCandidate {
  std::uint32_t id = 0;  // 每帧从 0 开始，按中心 x/y 排序，不是跨帧 ID。
  bool is_red = false;
  bool is_large_armor = false;
  std::array<cv::Point2f, 4> corners;  // 左上、右上、右下、左下；灯条长轴端点。
  cv::Point2f center;
  float confidence = 0.0F;  // 传统为几何评分；NN 为颜色分数×编号分数，均非标定概率。
  int number_class = -1;  // 神经网络编号类别 0..8；-1 为传统模式，独立于帧内 id。
};

}  // namespace ft_detector
