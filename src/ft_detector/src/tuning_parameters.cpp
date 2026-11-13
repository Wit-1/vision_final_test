#include "ft_detector/tuning_parameters.hpp"

namespace ft_detector {

// 将算法参数中的传统在线字段转换为 dynamic_reconfigure 配置。
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
  config.min_light_length = params.min_light_length;
  config.min_light_aspect = params.min_light_aspect;
  config.max_light_aspect = params.max_light_aspect;
  config.min_light_fill = params.min_light_fill;
  config.max_light_tilt_deg = params.max_light_tilt_deg;
  config.border_margin = params.border_margin;
  config.min_pair_length_ratio = params.min_pair_length_ratio;
  config.max_pair_center_dy_ratio = params.max_pair_center_dy_ratio;
  config.min_pair_spacing_ratio = params.min_pair_spacing_ratio;
  config.large_armor_min_spacing_ratio = params.large_armor_min_spacing_ratio;
  config.intervening_light_max_offset_ratio = params.intervening_light_max_offset_ratio;
  config.pair_ambiguity_margin = params.pair_ambiguity_margin;
  return config;
}

// 复制当前参数后覆盖在线字段，节点启动仍沿用原有 YAML 层级。
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
  params.min_light_length = config.min_light_length;
  params.min_light_aspect = config.min_light_aspect;
  params.max_light_aspect = config.max_light_aspect;
  params.min_light_fill = config.min_light_fill;
  params.max_light_tilt_deg = config.max_light_tilt_deg;
  params.border_margin = config.border_margin;
  params.min_pair_length_ratio = config.min_pair_length_ratio;
  params.max_pair_center_dy_ratio = config.max_pair_center_dy_ratio;
  params.min_pair_spacing_ratio = config.min_pair_spacing_ratio;
  params.large_armor_min_spacing_ratio = config.large_armor_min_spacing_ratio;
  params.intervening_light_max_offset_ratio = config.intervening_light_max_offset_ratio;
  params.pair_ambiguity_margin = config.pair_ambiguity_margin;
  return params;
}

}  // namespace ft_detector
