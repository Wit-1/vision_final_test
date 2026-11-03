#include "ft_detector/tuning_parameters.hpp"

namespace ft_detector {

DetectorConfig tuningFromParams(const TraditionalParams& params) {
  auto config = DetectorConfig::__getDefault__();
  config.blue_h_min = params.blue.h_min; config.blue_h_max = params.blue.h_max;
  config.blue_s_min = params.blue.s_min; config.blue_s_max = params.blue.s_max;
  config.blue_v_min = params.blue.v_min; config.blue_v_max = params.blue.v_max;
  config.red_low_h_min = params.red_low.h_min; config.red_low_h_max = params.red_low.h_max;
  config.red_low_s_min = params.red_low.s_min; config.red_low_s_max = params.red_low.s_max;
  config.red_low_v_min = params.red_low.v_min; config.red_low_v_max = params.red_low.v_max;
  config.red_high_h_min = params.red_high.h_min; config.red_high_h_max = params.red_high.h_max;
  config.red_high_s_min = params.red_high.s_min; config.red_high_s_max = params.red_high.s_max;
  config.red_high_v_min = params.red_high.v_min; config.red_high_v_max = params.red_high.v_max;
  config.min_contour_area = params.min_contour_area;
  config.max_pair_spacing_ratio = params.max_pair_spacing_ratio;
  config.max_pair_angle_deg = params.max_pair_angle_deg;
  return config;
}

TraditionalParams paramsWithTuning(const TraditionalParams& current, const DetectorConfig& config) {
  auto params = current;
  params.blue = {config.blue_h_min, config.blue_h_max, config.blue_s_min,
                 config.blue_s_max, config.blue_v_min, config.blue_v_max};
  params.red_low = {config.red_low_h_min, config.red_low_h_max, config.red_low_s_min,
                    config.red_low_s_max, config.red_low_v_min, config.red_low_v_max};
  params.red_high = {config.red_high_h_min, config.red_high_h_max, config.red_high_s_min,
                     config.red_high_s_max, config.red_high_v_min, config.red_high_v_max};
  params.min_contour_area = config.min_contour_area;
  params.max_pair_spacing_ratio = config.max_pair_spacing_ratio;
  params.max_pair_angle_deg = config.max_pair_angle_deg;
  return params;
}

}  // namespace ft_detector
