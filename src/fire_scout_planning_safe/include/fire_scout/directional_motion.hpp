#pragma once
#include "fire_scout/geometry.hpp"
#include <deque>

namespace fire_scout {

struct DirectionalConfig {
  double max_lateral_speed{.35};
  double max_reverse_speed{.25};
  double forward_half_angle{.35};
  double sonar_half_angle{.26};
  double prediction_horizon{.6};
  // While yaw converges, a requested path vector may sit outside the frontal
  // sensor cone. A small body-forward vector may replace it only after the
  // replacement corridor is observed and collision checked.
  double alignment_forward_speed_cap{.25};
  bool valid() const {
    return std::isfinite(max_lateral_speed) && max_lateral_speed >= 0 &&
           std::isfinite(max_reverse_speed) && max_reverse_speed >= 0 &&
           std::isfinite(forward_half_angle) && forward_half_angle > 0 &&
           forward_half_angle < pi / 2 && std::isfinite(sonar_half_angle) &&
           sonar_half_angle >= 0 && sonar_half_angle < pi / 2 &&
           std::isfinite(prediction_horizon) && prediction_horizon > 0 &&
           std::isfinite(alignment_forward_speed_cap) && alignment_forward_speed_cap > 0;
  }
};

struct DirectionalRequest {
  Vec3 position{}, velocity{};
  double body_yaw{0}, sonar_yaw{0}; // Both angles are in the world frame.
  double front_speed_cap{std::numeric_limits<double>::infinity()};
  bool sonar_valid{true}, sonar_required{true}, relaxed{false};
  // Parent must prove the entire requested retreat follows the recent actual
  // flight corridor. This explicit exception is not enabled by map free cells.
  bool allow_verified_retreat{false};
  // Parent separately certifies a bounded departure from frozen initial
  // envelope contacts, full observed new swept space, and all sonar contacts.
  bool allow_separating_retreat{false};
  // Parent enables this only for a fresh, executable path below the configured
  // hard heading-stop threshold. It never bypasses observation or collision.
  bool allow_alignment_forward{false};
};

enum class DirectionalReason {
  ALLOWED, ALIGNMENT_FORWARD, INVALID_INPUT, SONAR_MISSING, ALIGNMENT_REQUIRED,
  UNOBSERVED_CORRIDOR, COLLISION, SENSOR_LIMIT
};
inline const char *directionalReasonName(DirectionalReason r) {
  switch (r) {
    case DirectionalReason::ALLOWED: return "ALLOWED";
    case DirectionalReason::ALIGNMENT_FORWARD: return "ALIGNMENT_FORWARD";
    case DirectionalReason::INVALID_INPUT: return "INVALID_INPUT";
    case DirectionalReason::SONAR_MISSING: return "SONAR_MISSING";
    case DirectionalReason::ALIGNMENT_REQUIRED: return "ALIGNMENT_REQUIRED";
    case DirectionalReason::UNOBSERVED_CORRIDOR: return "UNOBSERVED_CORRIDOR";
    case DirectionalReason::COLLISION: return "COLLISION";
    case DirectionalReason::SENSOR_LIMIT: return "SENSOR_LIMIT";
  }
  return "INVALID_INPUT";
}
struct DirectionalResult {
  Vec3 velocity{};
  bool non_forward{false}, limited{false}, requires_alignment{false}, blocked{false};
  bool alignment_forward{false};
  DirectionalReason reason{DirectionalReason::ALLOWED};
};

inline Vec3 velocityInBody(Vec3 v, double yaw) {
  const double c = std::cos(yaw), s = std::sin(yaw);
  return {c * v.x + s * v.y, -s * v.x + c * v.y, v.z};
}
inline bool outsideForwardCone(Vec3 velocity, double yaw, double half_angle) {
  const Vec3 body = velocityInBody(velocity, yaw);
  return std::hypot(body.x, body.y) > 1e-7 &&
         std::abs(std::atan2(body.y, body.x)) > half_angle;
}
inline double maximumConeClosing(Vec3 velocity, double sonar_yaw, double half_angle) {
  const double speed = std::hypot(velocity.x, velocity.y);
  if (speed < 1e-12) return 0;
  const double angle = std::max(0.0, std::abs(wrap(std::atan2(velocity.y, velocity.x) -
                                                sonar_yaw)) - half_angle);
  return std::max(0.0, speed * std::cos(angle));
}

// This is a direction guard, not a planner, map, glass detector or dynamics
// controller. Call after command generation and before publishing. The two
// callbacks must validate the *complete* final segment with the vehicle envelope:
// observedCorridor requires actual observation (not allow_unknown), while
// collisionClear must use the newest occupancy, including sonar obstacles.
// A parent may OR observed free space with RecentCorridor::covers(), but must
// still run collisionClear. Speed caps uniformly scale the requested velocity;
// an acoustic return is not a wall normal and cannot justify inventing a slide.
template<class ObservedCorridor, class CollisionClear>
DirectionalResult constrainDirectional(const DirectionalRequest &r, const DirectionalConfig &c,
                                       ObservedCorridor observedCorridor,
                                       CollisionClear collisionClear) {
  DirectionalResult out;
  if (!c.valid() || !finite(r.position) || !finite(r.velocity) ||
      !std::isfinite(r.body_yaw) || !std::isfinite(r.sonar_yaw) ||
      std::isnan(r.front_speed_cap) || r.front_speed_cap < 0) {
    out.blocked = true; out.reason = DirectionalReason::INVALID_INPUT; return out;
  }
  const bool verified_retreat = (r.allow_verified_retreat||r.allow_separating_retreat) &&
      std::hypot(r.velocity.x, r.velocity.y) > 1e-7 &&
      maximumConeClosing(r.velocity, r.sonar_yaw, c.sonar_half_angle) <= 1e-9;
  if (r.sonar_required && !r.sonar_valid && !verified_retreat) {
    out.blocked = true; out.reason = DirectionalReason::SONAR_MISSING; return out;
  }
  const Vec3 body = velocityInBody(r.velocity, r.body_yaw);
  double scale = 1;
  if (std::abs(body.y) > c.max_lateral_speed)
    scale = std::min(scale, c.max_lateral_speed / std::abs(body.y));
  if (-body.x > c.max_reverse_speed)
    scale = std::min(scale, c.max_reverse_speed / -body.x);
  if (r.sonar_valid) {
    // A range belongs somewhere inside the acoustic cone. Check the largest
    // closing speed over that cone, not just the center ray: at a zero cap a
    // pure lateral command remains constrained, while a genuine retreat does not.
    const double closing = maximumConeClosing(r.velocity, r.sonar_yaw, c.sonar_half_angle);
    if (closing > r.front_speed_cap) scale = std::min(scale, r.front_speed_cap / closing);
  }
  const Vec3 v = r.velocity * scale;
  out.limited = distance(v, r.velocity) > 1e-8;
  out.non_forward = outsideForwardCone(v, r.body_yaw, c.forward_half_angle);
  if (out.non_forward && !r.relaxed) {
    if(r.allow_alignment_forward){
      const double horizontal=std::hypot(v.x,v.y);
      if(horizontal>1e-7){
        const double requested=std::atan2(v.y,v.x);
        const double relative=wrap(requested-r.body_yaw);
        const double interior=.90*c.forward_half_angle;
        const double steered=r.body_yaw+std::clamp(relative,-interior,interior);
        const double aligned_speed=std::min(horizontal,c.alignment_forward_speed_cap);
        Vec3 candidate{aligned_speed*std::cos(steered),
                       aligned_speed*std::sin(steered),v.z};
        // Rotating toward the sonar axis can increase acoustic closing speed;
        // reapply its cap to the replacement, rather than trusting the scale
        // computed for the rejected path direction.
        const double closing=maximumConeClosing(candidate,r.sonar_yaw,c.sonar_half_angle);
        if(closing>r.front_speed_cap)
          candidate=candidate*(r.front_speed_cap/closing);
        const Vec3 candidate_body=velocityInBody(candidate,r.body_yaw);
        double candidate_scale=1.;
        if(std::abs(candidate_body.y)>c.max_lateral_speed)
          candidate_scale=std::min(candidate_scale,c.max_lateral_speed/std::abs(candidate_body.y));
        if(-candidate_body.x>c.max_reverse_speed)
          candidate_scale=std::min(candidate_scale,c.max_reverse_speed/-candidate_body.x);
        candidate=candidate*candidate_scale;
        const Vec3 end=r.position+candidate*c.prediction_horizon;
        // Positive projection guarantees this bounded steering motion still
        // advances the requested route instead of walking away from it.
        const bool advances=candidate.x*v.x+candidate.y*v.y>1e-9;
        if(advances && norm(candidate)>1e-7 &&
           observedCorridor(r.position,end) && collisionClear(r.position,end)){
          out.velocity=candidate;out.non_forward=false;out.limited=true;
          out.alignment_forward=true;out.reason=DirectionalReason::ALIGNMENT_FORWARD;
          return out;
        }
      }
    }
    out.requires_alignment = true; out.blocked = true;
    out.reason = DirectionalReason::ALIGNMENT_REQUIRED; return out;
  }
  const Vec3 end = r.position + v * c.prediction_horizon;
  if ((out.non_forward || (r.sonar_required && !r.sonar_valid)) &&
      !observedCorridor(r.position, end)) {
    out.requires_alignment = true; out.blocked = true;
    out.reason = DirectionalReason::UNOBSERVED_CORRIDOR; return out;
  }
  if (!collisionClear(r.position, end)) {
    out.blocked = true; out.reason = DirectionalReason::COLLISION; return out;
  }
  out.velocity = v;
  if (norm(v) < 1e-7 && norm(r.velocity) > 1e-7) {
    out.blocked = true; out.reason = DirectionalReason::SENSOR_LIMIT;
  }
  return out;
}

struct NonForwardBudgetConfig {
  double max_distance{.75}, max_seconds{4}, reset_forward_distance{.30};
  double component_deadband{.025}, alignment_half_angle{.35};
  bool valid() const {
    return std::isfinite(max_distance) && max_distance > 0 &&
           std::isfinite(max_seconds) && max_seconds > 0 &&
           std::isfinite(reset_forward_distance) && reset_forward_distance > 0 &&
           std::isfinite(component_deadband) && component_deadband >= 0 &&
           std::isfinite(alignment_half_angle) && alignment_half_angle > 0 &&
           alignment_half_angle < pi / 2;
  }
};

class NonForwardBudget {
 public:
  NonForwardBudgetConfig cfg;
  explicit NonForwardBudget(NonForwardBudgetConfig c = {}) : cfg(c) {}
  // Call once for each control interval, with measured velocity AND the command
  // actually sent after all guards. A route heartbeat/replacement must not clear
  // this budget. Counting max(actual, sent) covers both actuator lag and drift.
  bool update(Vec3 actual, Vec3 sent, double yaw, double dt, bool aligned) {
    if (!cfg.valid() || !finite(actual) || !finite(sent) || !std::isfinite(yaw) ||
        !std::isfinite(dt) || dt < 0) { exhausted_ = true; return false; }
    if (dt == 0) return !exhausted_;
    const Vec3 a = velocityInBody(actual, yaw), s = velocityInBody(sent, yaw);
    auto side_reverse = [](Vec3 b, double deadband) {
      return std::hypot(std::abs(b.y) > deadband ? b.y : 0,
                        b.x < -deadband ? b.x : 0);
    };
    // The noise deadband belongs only to measured velocity. Deliberately small
    // sent commands must still consume time/distance instead of enabling an
    // unbounded slow sideways or reverse crawl below a measurement threshold.
    // Normal forward tracking can contain a small lateral correction while
    // remaining inside the same frontal cone that the final guard certifies.
    // Charging that correction spent the 4 s side/reverse allowance before a
    // slow aircraft could travel the .30 m needed to reset it. Charge only
    // motion outside that cone; actual drift and tiny deliberate side/reverse
    // commands remain accounted for independently.
    const double actual_nonforward=outsideForwardCone(actual,yaw,cfg.alignment_half_angle)?
      side_reverse(a,cfg.component_deadband):0.;
    const double sent_nonforward=outsideForwardCone(sent,yaw,cfg.alignment_half_angle)?
      side_reverse(s,1e-7):0.;
    const double lateral = std::max(actual_nonforward,sent_nonforward);
    if (lateral > 0) {
      used_distance_ += lateral * dt; used_seconds_ += dt;
      exhausted_ = exhausted_ || used_distance_ >= cfg.max_distance - 1e-9 ||
                   used_seconds_ >= cfg.max_seconds - 1e-9;
    }
    const bool actually_forward = a.x > cfg.component_deadband &&
        !outsideForwardCone(actual, yaw, cfg.alignment_half_angle);
    const bool commanded_forward = s.x > cfg.component_deadband &&
        !outsideForwardCone(sent, yaw, cfg.alignment_half_angle);
    if (aligned && actually_forward && commanded_forward) {
      reset_progress_ += a.x * dt;
      if (reset_progress_ >= cfg.reset_forward_distance) clear();
    } else reset_progress_ = 0;
    return !exhausted_;
  }
  bool available() const { return cfg.valid() && !exhausted_; }
  bool exhausted() const { return !available(); }
  double usedDistance() const { return used_distance_; }
  double usedSeconds() const { return used_seconds_; }
  double resetProgress() const { return reset_progress_; }
  // Use for a deliberate mission/reset transition only, not path updates.
  void clear() { used_distance_ = used_seconds_ = reset_progress_ = 0; exhausted_ = false; }
 private:
  double used_distance_{0}, used_seconds_{0}, reset_progress_{0};
  bool exhausted_{false};
};

struct RecentCorridorConfig {
  double max_age{10}, tube_radius{.12}, max_sample_gap{.25}, max_sample_step{.25};
  size_t max_points{1000};
  bool valid() const {
    return std::isfinite(max_age) && max_age > 0 && std::isfinite(tube_radius) &&
           tube_radius > 0 && std::isfinite(max_sample_gap) && max_sample_gap > 0 &&
           std::isfinite(max_sample_step) && max_sample_step > 0 && max_points >= 2;
  }
};

class RecentCorridor {
 public:
  RecentCorridorConfig cfg;
  explicit RecentCorridor(RecentCorridorConfig c = {}) : cfg(c) {}
  void clear() { points_.clear(); }
  void add(Vec3 position, double stamp) {
    if (!cfg.valid() || !finite(position) || !std::isfinite(stamp)) { clear(); return; }
    if (!points_.empty() && stamp < points_.back().stamp) clear();
    if (!points_.empty() && stamp == points_.back().stamp) return;
    bool connected = !points_.empty() && stamp - points_.back().stamp <= cfg.max_sample_gap &&
                     distance(position, points_.back().p) <= cfg.max_sample_step;
    points_.push_back({position, stamp, connected});
    prune(stamp);
    while (points_.size() > cfg.max_points) points_.pop_front();
  }
  // A conservative continuous certificate, not endpoint-only sampling:
  // distance-to-trail is 1-Lipschitz, so midpoint distance + half subsegment
  // length <= tube_radius proves every point in that subsegment is covered.
  // An otherwise valid candidate on the tube boundary may be rejected; false
  // positive coverage through a corner, a recording gap or expired trail is not
  // accepted. This only constrains centerline deviation. It does not certify a
  // whole drone's free envelope or transparent-object clearance after offsetting
  // the body: tube_radius must fit inside the parent's retained tracking reserve.
  bool covers(Vec3 a, Vec3 b, double now) {
    if (!cfg.valid() || !finite(a) || !finite(b) || !std::isfinite(now) || points_.empty())
      return false;
    if (now < points_.back().stamp) { clear(); return false; }
    prune(now);
    if (points_.empty()) return false;
    const double len = distance(a, b);
    const double steps = std::ceil(len / (.5 * cfg.tube_radius));
    if (!std::isfinite(steps) || steps > 10000) return false;
    const size_t n = std::max<size_t>(1, static_cast<size_t>(steps));
    const double reserve = len / (2 * n);
    for (size_t i = 0; i < n; ++i) {
      const Vec3 mid = a + (b - a) * ((i + .5) / n);
      if (nearest(mid) + reserve > cfg.tube_radius) return false;
    }
    return true;
  }
  // For fallback retreat authorization, use a frozen COPY of this history until
  // ordinary observed forward progress restores the non-forward budget. Merely
  // adding each sideways command's new odometry to its own authorization trail
  // would let a hovering aircraft ratchet an unexplored tube outwards.
  bool retraces(Vec3 a, Vec3 b, double now, double min_age = .30,
                double endpoint_tolerance = .04, double min_history_span = .20) {
    if (!std::isfinite(min_age) || min_age <= 0 || !std::isfinite(endpoint_tolerance) ||
        endpoint_tolerance <= 0 || !std::isfinite(min_history_span) || min_history_span <= 0 ||
        !covers(a, b, now)) return false;
    const double tolerance = std::min(cfg.tube_radius, endpoint_tolerance);
    const double cutoff = now - min_age;
    double end_distance = std::numeric_limits<double>::infinity();
    // Displacement span, not accumulated path length: jitter cannot manufacture
    // a flown corridor by accumulating many tiny samples. Requiring displacement
    // from the first retained sample is conservative for winding trajectories.
    for (size_t first = 0; first < points_.size();) {
      size_t last = first + 1;
      while (last < points_.size() && points_[last].connected) ++last;
      bool span = false;
      for (size_t i = first; i < last; ++i)
        span = span || distance(points_[i].p, points_[first].p) >= min_history_span;
      // Separate recording runs cannot contribute apparent displacement via a
      // teleport or a long odometry gap. The endpoint must belong to a run that
      // actually contains enough continuous recorded travel.
      if (span) for (size_t i = first; i < last; ++i) {
        if (points_[i].stamp <= cutoff)
          end_distance = std::min(end_distance, distance(b, points_[i].p));
        if (i > first && points_[i - 1].stamp <= cutoff) {
          const Sample &lo = points_[i - 1], &hi = points_[i];
          const double fraction = std::clamp((cutoff - lo.stamp) / (hi.stamp - lo.stamp), 0.0, 1.0);
          const Vec3 d = (hi.p - lo.p) * fraction;
          const double length2 = dot(d, d);
          if (length2 > 1e-12)
            end_distance = std::min(end_distance, distance(b, lo.p + d *
                std::clamp(dot(b - lo.p, d) / length2, 0.0, 1.0)));
        }
      }
      first = last;
    }
    return end_distance <= tolerance;
  }
  size_t size() const { return points_.size(); }
 private:
  struct Sample { Vec3 p; double stamp; bool connected; };
  std::deque<Sample> points_;
  void prune(double now) {
    while (!points_.empty() && now - points_.front().stamp > cfg.max_age) points_.pop_front();
  }
  double nearest(Vec3 p) const {
    double best = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < points_.size(); ++i) {
      best = std::min(best, distance(p, points_[i].p));
      if (i && points_[i].connected) {
        const Vec3 d = points_[i].p - points_[i - 1].p;
        const double length2 = dot(d, d);
        if (length2 > 1e-12)
          best = std::min(best, distance(p, points_[i - 1].p + d *
              std::clamp(dot(p - points_[i - 1].p, d) / length2, 0.0, 1.0)));
      }
    }
    return best;
  }
};
} // namespace fire_scout
