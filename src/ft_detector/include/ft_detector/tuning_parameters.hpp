#pragma once

#include <ft_detector/DetectorConfig.h>

#include "ft_detector/traditional_detector.hpp"

namespace ft_detector {

// Convert the complete traditional runtime configuration without changing YAML layout.
DetectorConfig tuningFromParams(const TraditionalParams& params);
TraditionalParams paramsWithTuning(const TraditionalParams& current, const DetectorConfig& tuning);

}  // namespace ft_detector
