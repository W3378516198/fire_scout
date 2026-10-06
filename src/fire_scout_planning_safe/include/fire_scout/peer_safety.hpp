#pragma once

#include "geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace fire_scout {

// This layer is deliberately independent of radar occupancy.  A peer UAV is a
// moving vehicle with its own envelope, not a point obstacle which happens to
// appear in a local point cloud.
struct PeerSafetyConfig {
  bool enabled{true};
  double hard_separation{.94};
  double vertical_separation{.55};
  double reaction_time{.45};
  double brake_accel{1.0};
  double prediction_horizon{2.0};
  double passage_width{1.35};
  double passage_lookahead{2.5};
  double passage_reservation_radius{1.60};
  double passage_priority_hysteresis{.20};
  double separation_escape_speed{.12};
  // Queue order is geometric, not just numeric.  A vehicle already ahead on
  // the requested passage axis must never stop to yield to a faster vehicle
  // approaching from behind; the rear vehicle owns the braking obligation.
  double rear_ignore_distance{.15};
  // A yielded vehicle may still execute a map-certified retreat which does
  // not close on any peer.  This is deliberately slower than normal flight.
  double yield_retreat_speed{.18};

  bool valid() const {
    const double values[]{hard_separation, vertical_separation, reaction_time,
                          brake_accel, prediction_horizon, passage_width,
                          passage_lookahead, passage_reservation_radius,
                          passage_priority_hysteresis,
                          separation_escape_speed, rear_ignore_distance,
                          yield_retreat_speed};
    for (double value : values)
      if (!std::isfinite(value) || value < 0) return false;
    return hard_separation > 0 && vertical_separation > 0 && brake_accel > 0 &&
           prediction_horizon > 0 && passage_width > hard_separation &&
           passage_lookahead >= .5 && passage_reservation_radius >= .5 &&
           separation_escape_speed > 0 && separation_escape_speed <= .30 &&
           rear_ignore_distance >= 0 && rear_ignore_distance <= hard_separation &&
           yield_retreat_speed > 0 && yield_retreat_speed <= .30;
  }
};

struct PeerKinematicState {
  Vec3 position{};
  Vec3 velocity{};
  int priority{0};  // Smaller value owns a deterministic tie.
  size_t index{0};
  bool fresh{false};
};

struct PassageReservation {
  bool valid{false};
  bool own_inside{false};
  Vec3 anchor{};
};

enum class PeerSafetyReason {
  CLEAR,
  SPEED_LIMIT,
  PREDICTED_CONFLICT_YIELD,
  PASSAGE_YIELD,
  HARD_SEPARATION
};

inline const char *peerSafetyReasonName(PeerSafetyReason reason) {
  switch (reason) {
    case PeerSafetyReason::CLEAR: return "CLEAR";
    case PeerSafetyReason::SPEED_LIMIT: return "SPEED_LIMIT";
    case PeerSafetyReason::PREDICTED_CONFLICT_YIELD:
      return "PREDICTED_CONFLICT_YIELD";
    case PeerSafetyReason::PASSAGE_YIELD: return "PASSAGE_YIELD";
    case PeerSafetyReason::HARD_SEPARATION: return "HARD_SEPARATION";
  }
  return "UNKNOWN";
}

struct PeerSafetyDecision {
  Vec3 velocity{};
  PeerSafetyReason reason{PeerSafetyReason::CLEAR};
  double nearest_distance{std::numeric_limits<double>::infinity()};
  double nearest_clearance{std::numeric_limits<double>::infinity()};
  double predicted_miss{std::numeric_limits<double>::infinity()};
  double predicted_time{std::numeric_limits<double>::infinity()};
  int blocking_priority{-1};
  size_t blocking_index{std::numeric_limits<size_t>::max()};
  size_t fresh_peers{0};
  size_t rear_yields_ignored{0};
  bool limited{false};
  bool yielding{false};
};

struct PeerYieldWatchdogConfig {
  double no_progress_timeout{3.0};
  double blocker_progress_distance{.18};

  bool valid() const {
    return std::isfinite(no_progress_timeout) && no_progress_timeout > 0 &&
           std::isfinite(blocker_progress_distance) &&
           blocker_progress_distance > 0 && blocker_progress_distance <= 1.0;
  }
};

// Tracks progress of the *blocking* vehicle, not just time spent waiting.  A
// legitimate queue can therefore wait indefinitely while its owner advances,
// whereas a stopped owner eventually releases the follower's recovery path.
class PeerYieldWatchdog {
 public:
  PeerYieldWatchdogConfig cfg;

  void reset() {
    active_ = false;
    blocker_priority_ = -1;
    started_ = last_progress_ = 0;
    have_reference_ = false;
    reference_ = {};
  }

  bool update(double time, bool active, int blocker_priority,
              Vec3 blocker_position, bool have_position) {
    if (!cfg.valid() || !std::isfinite(time) || !active) {
      reset();
      return false;
    }
    if (!active_ || blocker_priority != blocker_priority_ ||
        time < last_progress_) {
      active_ = true;
      blocker_priority_ = blocker_priority;
      started_ = last_progress_ = time;
      have_reference_ = have_position && finite(blocker_position);
      if (have_reference_) reference_ = blocker_position;
      return false;
    }
    if (have_position && finite(blocker_position)) {
      if (!have_reference_) {
        reference_ = blocker_position;
        have_reference_ = true;
        last_progress_ = time;
      } else if (time - last_progress_ >= cfg.no_progress_timeout) {
        // Judge net displacement over a complete lease window.  Updating the
        // reference on every short excursion would let a blocker oscillate
        // back and forth forever without actually clearing the passage.
        const Vec3 displacement = blocker_position - reference_;
        if (std::hypot(displacement.x, displacement.y) <
            cfg.blocker_progress_distance)
          return true;
        reference_ = blocker_position;
        last_progress_ = time;
      }
    }
    return time - last_progress_ >= cfg.no_progress_timeout;
  }

  double activeSeconds(double time) const {
    return active_ && std::isfinite(time) && time >= started_
               ? time - started_
               : 0;
  }
  double stagnantSeconds(double time) const {
    return active_ && std::isfinite(time) && time >= last_progress_
               ? time - last_progress_
               : 0;
  }
  int blockerPriority() const { return blocker_priority_; }
  bool active() const { return active_; }

 private:
  bool active_{false};
  int blocker_priority_{-1};
  double started_{0}, last_progress_{0};
  bool have_reference_{false};
  Vec3 reference_{};
};

inline double horizontalNorm(Vec3 value) {
  return std::hypot(value.x, value.y);
}

inline double peerStoppingSpeed(double clearance, double acceleration,
                                double reaction) {
  if (clearance <= 0 || acceleration <= 0) return 0;
  const double ar = acceleration * std::max(0.0, reaction);
  return std::max(0.0, std::sqrt(ar * ar + 2 * acceleration * clearance) - ar);
}

inline void selectPeerReason(PeerSafetyDecision &decision,
                             PeerSafetyReason reason,
                             const PeerKinematicState &peer) {
  // Larger enum values are intentionally the more restrictive states.
  if (static_cast<int>(reason) >= static_cast<int>(decision.reason)) {
    decision.reason = reason;
    decision.blocking_priority = peer.priority;
    decision.blocking_index = peer.index;
  }
}

inline PeerSafetyDecision constrainPeerMotion(
    Vec3 current, Vec3 requested, int own_priority,
    const std::vector<PeerKinematicState> &peers,
    const PeerSafetyConfig &config,
    const PassageReservation &passage = {}) {
  PeerSafetyDecision decision;
  decision.velocity = requested;
  if (!config.enabled || !config.valid() || !finite(current) ||
      !finite(requested))
    return decision;

  const double own_anchor_distance = passage.valid
      ? horizontalNorm(current - passage.anchor)
      : std::numeric_limits<double>::infinity();

  struct RadialConstraint {
    Vec3 direction{};
    double allowed{0};
    const PeerKinematicState *peer{nullptr};
  };
  std::vector<RadialConstraint> constraints;
  std::vector<RadialConstraint> hard_contacts;
  std::vector<RadialConstraint> yield_constraints;
  bool coincident_contact = false;
  bool priority_yield = false;

  const Vec3 requested_xy{requested.x, requested.y, 0};
  const double requested_speed = horizontalNorm(requested_xy);
  const Vec3 requested_axis = requested_speed > 1e-6
      ? requested_xy * (1.0 / requested_speed) : Vec3{};
  const Vec3 passage_delta = passage.anchor - current;
  const double passage_distance = horizontalNorm(passage_delta);
  const Vec3 passage_axis = passage.valid && passage_distance > 1e-6
      ? passage_delta * (1.0 / passage_distance) : Vec3{};

  for (const auto &peer : peers) {
    if (!peer.fresh || !finite(peer.position) || !finite(peer.velocity)) continue;
    ++decision.fresh_peers;
    const Vec3 offset = peer.position - current;
    if (std::abs(offset.z) > config.vertical_separation) continue;
    const double distance_xy = horizontalNorm(offset);
    if (distance_xy < decision.nearest_distance) {
      decision.nearest_distance = distance_xy;
      decision.nearest_clearance = distance_xy - config.hard_separation;
    }
    if (distance_xy < 1e-6) {
      coincident_contact = true;
      selectPeerReason(decision, PeerSafetyReason::HARD_SEPARATION, peer);
      continue;
    }

    // A vehicle already occupying the narrow section owns it until it leaves.
    // Otherwise the vehicle closer to the common bottleneck owns it; only a
    // near tie falls back to the stable launch priority.
    if (passage.valid && !passage.own_inside) {
      const double peer_anchor_distance =
          horizontalNorm(peer.position - passage.anchor);
      const bool both_near =
          own_anchor_distance <= config.passage_reservation_radius &&
          peer_anchor_distance <= config.passage_reservation_radius;
      const bool peer_closer =
          peer_anchor_distance + config.passage_priority_hysteresis <
          own_anchor_distance;
      const bool tied = std::abs(peer_anchor_distance - own_anchor_distance) <=
                        config.passage_priority_hysteresis;
      const bool would_yield =
          both_near && (peer_closer || (tied && own_priority > peer.priority));
      const bool peer_behind = passage_distance > 1e-6 &&
          dot(Vec3{offset.x, offset.y, 0}, passage_axis) <
              -config.rear_ignore_distance;
      if (would_yield) {
        if (peer_behind) {
          ++decision.rear_yields_ignored;
        } else {
          priority_yield = true;
          const Vec3 direction{offset.x / distance_xy,
                               offset.y / distance_xy, 0};
          yield_constraints.push_back({direction, 0, &peer});
          selectPeerReason(decision, PeerSafetyReason::PASSAGE_YIELD, peer);
        }
      }
    }

    const Vec3 direction{offset.x / distance_xy, offset.y / distance_xy, 0};
    Vec3 relative_velocity{decision.velocity.x - peer.velocity.x,
                           decision.velocity.y - peer.velocity.y, 0};
    const double relative_speed_sq = dot(relative_velocity, relative_velocity);
    if (relative_speed_sq > 1e-8) {
      const double time = std::clamp(
          dot(Vec3{offset.x, offset.y, 0}, relative_velocity) /
              relative_speed_sq,
          0.0, config.prediction_horizon);
      const Vec3 predicted =
          Vec3{offset.x, offset.y, 0} - relative_velocity * time;
      const double miss = horizontalNorm(predicted);
      if (miss < decision.predicted_miss) {
        decision.predicted_miss = miss;
        decision.predicted_time = time;
      }
      if (time > .05 && miss < config.hard_separation &&
          own_priority > peer.priority) {
        const bool peer_behind = requested_speed > 1e-6 &&
            dot(Vec3{offset.x, offset.y, 0}, requested_axis) <
                -config.rear_ignore_distance;
        if (peer_behind) {
          ++decision.rear_yields_ignored;
        } else {
          priority_yield = true;
          yield_constraints.push_back({direction, 0, &peer});
          selectPeerReason(decision,
                           PeerSafetyReason::PREDICTED_CONFLICT_YIELD, peer);
        }
      }
    }

    const double clearance = distance_xy - config.hard_separation;
    const double peer_radial =
        peer.velocity.x * direction.x + peer.velocity.y * direction.y;
    if (clearance <= 0) {
      hard_contacts.push_back({direction, 0, &peer});
      selectPeerReason(decision, PeerSafetyReason::HARD_SEPARATION, peer);
      continue;
    }

    // Limit only the component which closes the pairwise envelope.  A peer
    // moving toward us consumes the same braking budget; a peer moving away
    // safely increases it.  Side and separating motion remain available.
    const double allowed_closing = peerStoppingSpeed(
        clearance, config.brake_accel, config.reaction_time);
    const double allowed_approach = std::max(0.0, allowed_closing + peer_radial);
    constraints.push_back({direction, allowed_approach, &peer});
  }

  // Once envelopes overlap, resolving that overlap takes precedence over
  // passage priority.  Preserve a requested escape only when it is a small,
  // purely radial move away from one peer.  With coincident or multiple
  // non-collinear contacts there is no unambiguous safe radial direction.
  if (coincident_contact || !hard_contacts.empty()) {
    decision.limited = true;
    double escape = 0;
    if (!coincident_contact && hard_contacts.size() == 1) {
      const Vec3 direction = hard_contacts.front().direction;
      const double requested_away = std::max(
          0.0, -(requested.x * direction.x + requested.y * direction.y));
      escape = std::min(requested_away, config.separation_escape_speed);
      // A separating move from the overlapping peer must still obey every
      // non-overlapping peer's braking half-space.
      const Vec3 escape_direction{-direction.x, -direction.y, 0};
      for (const auto &constraint : constraints) {
        const double projection = dot(escape_direction, constraint.direction);
        if (projection > 1e-10)
          escape = std::min(escape, constraint.allowed / projection);
      }
      escape = std::max(0.0, escape);
      decision.velocity.x = escape_direction.x * escape;
      decision.velocity.y = escape_direction.y * escape;
    } else {
      decision.velocity.x = decision.velocity.y = 0;
    }
    decision.yielding = escape <= 1e-8;
    // A prior passage/conflict reason must not hide the hard-envelope state.
    decision.reason = PeerSafetyReason::HARD_SEPARATION;
    if (!hard_contacts.empty()) {
      decision.blocking_priority = hard_contacts.front().peer->priority;
      decision.blocking_index = hard_contacts.front().peer->index;
    }
    return decision;
  }

  if (priority_yield) {
    // Do not let a normal closing command enter the reserved passage, but do
    // retain a bounded recovery command which is non-closing for every peer
    // that requested the yield.  Cyclic projection handles two peers on
    // different sides without re-opening an earlier half-space.
    for (int pass = 0; pass < 16; ++pass) {
      bool changed = false;
      for (const auto &constraint : yield_constraints) {
        const double approach =
            decision.velocity.x * constraint.direction.x +
            decision.velocity.y * constraint.direction.y;
        if (approach <= 1e-9) continue;
        decision.velocity.x -= constraint.direction.x * approach;
        decision.velocity.y -= constraint.direction.y * approach;
        changed = true;
      }
      if (!changed) break;
    }
    double retreat_speed = horizontalNorm(decision.velocity);
    if (retreat_speed > config.yield_retreat_speed) {
      const double scale = config.yield_retreat_speed / retreat_speed;
      decision.velocity.x *= scale;
      decision.velocity.y *= scale;
    }
    // The non-closing retreat must also respect every ordinary braking plane.
    for (int pass = 0; pass < 16; ++pass) {
      bool changed = false;
      for (const auto &constraint : constraints) {
        const double approach =
            decision.velocity.x * constraint.direction.x +
            decision.velocity.y * constraint.direction.y;
        if (approach <= constraint.allowed + 1e-9) continue;
        const double remove = approach - constraint.allowed;
        decision.velocity.x -= constraint.direction.x * remove;
        decision.velocity.y -= constraint.direction.y * remove;
        changed = true;
      }
      if (!changed) break;
    }
    for (const auto &constraint : yield_constraints) {
      if (decision.velocity.x * constraint.direction.x +
              decision.velocity.y * constraint.direction.y > 1e-6) {
        decision.velocity.x = decision.velocity.y = 0;
        break;
      }
    }
    for (const auto &constraint : constraints) {
      if (decision.velocity.x * constraint.direction.x +
              decision.velocity.y * constraint.direction.y >
          constraint.allowed + 1e-6) {
        decision.velocity.x = decision.velocity.y = 0;
        break;
      }
    }
    decision.limited = true;
    decision.yielding = horizontalNorm(decision.velocity) <= 1e-8;
    return decision;
  }

  // Project onto all pairwise braking half-spaces.  A single ordered pass can
  // make an earlier constraint unsafe again when peers sit on different sides;
  // cyclic projection plus a final fail-safe verification avoids that leak
  // while retaining every tangential/separating component that is feasible.
  for (int pass = 0; pass < 16; ++pass) {
    bool changed = false;
    for (const auto &constraint : constraints) {
      const double approach = decision.velocity.x * constraint.direction.x +
                              decision.velocity.y * constraint.direction.y;
      if (approach <= constraint.allowed + 1e-9) continue;
      const double remove = approach - constraint.allowed;
      decision.velocity.x -= constraint.direction.x * remove;
      decision.velocity.y -= constraint.direction.y * remove;
      decision.limited = changed = true;
      selectPeerReason(decision, PeerSafetyReason::SPEED_LIMIT,
                       *constraint.peer);
    }
    if (!changed) break;
  }
  for (const auto &constraint : constraints) {
    const double approach = decision.velocity.x * constraint.direction.x +
                            decision.velocity.y * constraint.direction.y;
    if (approach > constraint.allowed + 1e-6) {
      decision.velocity.x = decision.velocity.y = 0;
      decision.limited = true;
      selectPeerReason(decision, PeerSafetyReason::SPEED_LIMIT,
                       *constraint.peer);
      break;
    }
  }
  return decision;
}

}  // namespace fire_scout
