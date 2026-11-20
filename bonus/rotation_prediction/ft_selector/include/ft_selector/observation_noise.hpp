#pragma once

#include <array>
#include <string>
#include <vector>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>

namespace ft_selector {

struct NoiseStrength {
  double confidence_reference = 0.8;
  double position = 1.0, yaw = 1.0;
};

// 这些参数描述可解释的倍率规则，不将分类分数当作标定概率。
struct ObservationNoiseParams {
  double error_reference_px = 1.0, distance_reference_m = 3.0, max_multiplier = 25.0;
  NoiseStrength nn, tradition{0.85, 1.0, 1.0};
};

struct QualityCalibration {
  cv::Matx33d matrix = cv::Matx33d::eye();
  std::vector<double> distortion;
  cv::Size image_size;
  std::string distortion_model;
};

struct ObservationQuality {
  bool valid = false;
  double error_px = 0.0, confidence = 0.0, distance_m = 0.0, facing = 0.0;
};

void validateObservationNoise(const ObservationNoiseParams& params);
bool validQualityCalibration(const QualityCalibration& camera);
ObservationQuality measureObservationQuality(const Eigen::Vector3d& position,
    const Eigen::Quaterniond& rotation, const std::array<cv::Point2d, 4>& corners,
    double width_m, double height_m, double confidence, const QualityCalibration& camera);
Eigen::Vector4d observationVarianceMultipliers(const ObservationQuality& quality,
    const std::string& mode, const ObservationNoiseParams& params);

}  // namespace ft_selector
