#pragma once

#include <cstddef>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "ft_detector/armor_candidate.hpp"
#include "ft_detector/roi.hpp"

namespace ft_detector {

struct NnThresholds {
  double color_score, number_score, confidence, nms_iou;
};

struct NnSizePolicy {
  bool enabled = false;
  double min_score = 0.8, min_margin = 0.1;
  // -1: geometric fallback; 0: small; 1: large. Preserve ambiguous infantry/base classes.
  std::array<int, 9> class_sizes{{0, 1, 0, -1, -1, -1, 0, -1, -1}};
};

struct NnRoiParams : RoiParams {
  NnRoiParams() { min_width = 160; min_height = 96; }
  double seed_number_score = 0.8, seed_number_margin = 0.1;
};

using NnDebugState = RoiDebugState;

void drawNnDebug(cv::Mat& image, const NnDebugState& state);

struct NnParams {
  std::string model_path;
  int input_width, input_height, pad_value, cpu_threads, max_candidates;
  double large_armor_min_spacing_ratio, min_quad_area_px2, min_edge_length_px;
  NnThresholds thresholds;
  NnSizePolicy size_policy;
  NnRoiParams roi;
};

struct LetterboxTransform {
  cv::Size original_size, resized_size;
  int left = 0, top = 0;
  cv::Point roi_origin;
  double scale_x = 1.0, scale_y = 1.0;
  cv::Point2f toOriginal(const cv::Point2f& point) const;
};

struct NnInput {
  cv::Mat blob;  // RGB float32 NCHW，持有送入 Session::Run 的内存。
  LetterboxTransform transform;
};

NnInput prepareNnInput(const cv::Mat& bgr, const NnParams& params);
std::vector<ArmorCandidate> decodeNnOutput(const float* values, std::size_t count,
                        const LetterboxTransform& transform, const NnParams& params);

class NnDetector {
 public:
  explicit NnDetector(const NnParams& params);
  ~NnDetector();
  NnDetector(const NnDetector&) = delete;
  NnDetector& operator=(const NnDetector&) = delete;
  static void validateParams(const NnParams& params);
  void setThresholds(const NnThresholds& thresholds, double large_armor_min_spacing_ratio);
  std::vector<ArmorCandidate> detect(const cv::Mat& bgr);
  std::vector<ArmorCandidate> detect(const cv::Mat& bgr, double stamp_s, const std::string& frame_id);
  void resetHistory();
  const NnDebugState& debugState() const;
  static std::string backendVersion();

 private:
  struct Impl;
  NnParams params_;
  std::unique_ptr<Impl> impl_;
};

}  // namespace ft_detector
