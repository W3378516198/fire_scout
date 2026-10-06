#pragma once

#include <cmath>

namespace fire_scout {

enum class FusionState { Unknown, Free, Occupied };

struct ConfidenceDecisionConfig {
  double minimum_occupied_confidence{0.20};
  double occupied_conflict_bias{1.05};
  bool self_observation_priority{true};

  bool valid() const {
    return std::isfinite(minimum_occupied_confidence) &&
           minimum_occupied_confidence > 0 &&
           std::isfinite(occupied_conflict_bias) && occupied_conflict_bias > 0;
  }
};

// Decide one voxel after confidence weights have been accumulated.  An
// uncertain peer-body return contributes a small occupied score but does not
// set self_occupied; it therefore becomes Unknown unless corroborated.
inline FusionState decideFusionState(double occupied, double free,
                                     bool self_occupied, bool self_free,
                                     const ConfidenceDecisionConfig &config) {
  if (!config.valid() || !std::isfinite(occupied) || !std::isfinite(free) ||
      occupied < 0 || free < 0)
    return FusionState::Unknown;
  if (config.self_observation_priority && self_occupied)
    return FusionState::Occupied;
  if (config.self_observation_priority && self_free)
    return FusionState::Free;
  if (occupied >= config.minimum_occupied_confidence &&
      occupied * config.occupied_conflict_bias >= free)
    return FusionState::Occupied;
  if (free > 0) return FusionState::Free;
  return FusionState::Unknown;
}

}  // namespace fire_scout
