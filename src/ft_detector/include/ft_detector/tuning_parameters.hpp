#pragma once

#include <ft_detector/DetectorConfig.h>

#include "ft_detector/traditional_detector.hpp"

namespace ft_detector {

// Convert runtime fields; all unexposed geometry fields stay in the startup parameter set.
DetectorConfig tuningFromParams(const TraditionalParams& params);
TraditionalParams paramsWithTuning(const TraditionalParams& current, const DetectorConfig& tuning);

}  // namespace ft_detector
