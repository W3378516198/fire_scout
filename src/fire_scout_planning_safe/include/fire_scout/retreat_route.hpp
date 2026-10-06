#pragma once
#include "geometry.hpp"
#include <deque>
#include <utility>

namespace fire_scout {

struct RetreatRouteConfig {
  double history_seconds{30.0};
  double tube_radius{.04};
  double max_sample_gap{.25}, max_sample_step{.35};
  double min_length{.4}, max_length{1.0}, preferred_length{.7};
  double min_displacement{.35}, max_path_ratio{1.8}, min_endpoint_age{.3};
  size_t max_points{3000}, max_certificate_checks{512}, max_waypoints{64};
  bool valid() const {
    return std::isfinite(history_seconds) && history_seconds > 0 && history_seconds <= 120 &&
      std::isfinite(tube_radius) && tube_radius > 0 && tube_radius <= .04 &&
      std::isfinite(max_sample_gap) && max_sample_gap > 0 && max_sample_gap <= 1 &&
      std::isfinite(max_sample_step) && max_sample_step > 0 && max_sample_step <= .5 &&
      std::isfinite(min_length) && min_length >= .2 &&
      std::isfinite(max_length) && max_length >= min_length && max_length <= 2 &&
      std::isfinite(preferred_length) && preferred_length >= min_length && preferred_length <= max_length &&
      std::isfinite(min_displacement) && min_displacement >= .2 && min_displacement <= min_length &&
      std::isfinite(max_path_ratio) && max_path_ratio >= 1 && max_path_ratio <= 2 &&
      std::isfinite(min_endpoint_age) && min_endpoint_age > 0 && min_endpoint_age < history_seconds &&
      max_points >= 2 && max_points <= 10000 && max_certificate_checks >= 1 &&
      max_certificate_checks <= 4096 && max_waypoints >= 2 && max_waypoints <= 256;
  }
};

class RetreatRouteHistory;

// A frozen retreat with the original odometry evidence attached. This is NOT a
// free-space map: covers() only bounds the CENTERLINE deviation. Its caller must
// independently check the complete current vehicle envelope against occupancy,
// directional/sonar restrictions, and reserve at least tube_radius for tracking.
// Never append escape odometry to this evidence or authorize a wider glass-free
// corridor merely because the centre previously passed through it.
class RetreatRoute {
 public:
  std::vector<Vec3> points;
  double length{0}, displacement{0};
  bool valid() const {
    return cfg_.valid() && points.size() >= 2 && finite(points.front()) && finite(points.back()) &&
      std::isfinite(length) && std::isfinite(displacement) &&
      length >= cfg_.min_length - 1e-8 && length <= cfg_.max_length + 1e-8 &&
      displacement >= cfg_.min_displacement - 1e-8 &&
      length <= cfg_.max_path_ratio * displacement + 1e-8 && !evidence_.empty();
  }
  bool validAt(double now) const { return valid() && evidenceValidAt(now); }
  double oldestStamp() const { return evidence_.empty() ? -1 : evidence_.front().stamp; }
  double newestStamp() const { return evidence_.empty() ? -1 : evidence_.back().stamp; }
  size_t evidenceSize() const { return evidence_.size(); }
  double tubeRadius() const { return cfg_.tube_radius; }

  // Midpoint distance + half subsegment length proves continuous coverage by
  // the 1-Lipschitz distance-to-trail function. Endpoint tests alone would permit
  // cutting the inside of a corner or bridging the interior of a loop.
  bool covers(Vec3 a, Vec3 b, double now) const {
    if (!finite(a) || !finite(b) || !evidenceValidAt(now)) return false;
    const double len = distance(a, b);
    const double count = std::ceil(len / (.5 * cfg_.tube_radius));
    if (!std::isfinite(count) || count > 10000) return false;
    const size_t n = std::max<size_t>(1, static_cast<size_t>(count));
    const double reserve = len / (2 * n);
    for (size_t i = 0; i < n; ++i) {
      const Vec3 mid = a + (b - a) * ((i + .5) / n);
      if (nearest(mid) + reserve > cfg_.tube_radius + 1e-10) return false;
    }
    return true;
  }

  // Call before executing and on every control cycle. next_waypoint is the next
  // unreached points[] index, starting at 1. Heartbeats/new proposed plans do not
  // modify this object. A changed wall, expired history or an unsafe corner
  // connector aborts the retreat immediately.
  template<class Certificate>
  bool certifiedFrom(Vec3 current, size_t next_waypoint, double now, Certificate certified) const {
    if (!validAt(now) || next_waypoint == 0 || next_waypoint >= points.size()) return false;
    Vec3 previous = current;
    for (size_t i = next_waypoint; i < points.size(); ++i) {
      if (!covers(previous, points[i], now) || !certified(previous, points[i], *this)) return false;
      previous = points[i];
    }
    return true;
  }

 private:
  struct Sample { Vec3 p; double stamp; };
  RetreatRouteConfig cfg_;
  std::vector<Sample> evidence_; // Oldest first; one continuous odometry run.
  friend class RetreatRouteHistory;
  bool evidenceValidAt(double now) const {
    return cfg_.valid() && std::isfinite(now) && !evidence_.empty() &&
      now >= evidence_.back().stamp && now - evidence_.front().stamp <= cfg_.history_seconds;
  }
  double nearest(Vec3 p) const {
    double best = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < evidence_.size(); ++i) {
      best = std::min(best, distance(p, evidence_[i].p));
      if (i) {
        const Vec3 d = evidence_[i].p - evidence_[i - 1].p;
        const double len2 = dot(d, d);
        if (len2 > 1e-12)
          best = std::min(best, distance(p, evidence_[i - 1].p + d *
            std::clamp(dot(p - evidence_[i - 1].p, d) / len2, 0., 1.)));
      }
    }
    return best;
  }
  bool endpointOldEnough(Vec3 p, double now) const {
    const double cutoff = now - cfg_.min_endpoint_age;
    double best = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < evidence_.size(); ++i) {
      if (evidence_[i].stamp <= cutoff) best = std::min(best, distance(p, evidence_[i].p));
      if (i && evidence_[i - 1].stamp < cutoff) {
        const auto &lo = evidence_[i - 1], &hi = evidence_[i];
        const double fraction = std::clamp((cutoff - lo.stamp) / (hi.stamp - lo.stamp), 0., 1.);
        const Vec3 d = (hi.p - lo.p) * fraction;
        const double len2 = dot(d, d);
        if (len2 > 1e-12) best = std::min(best, distance(p, lo.p + d *
          std::clamp(dot(p - lo.p, d) / len2, 0., 1.)));
      }
    }
    return best <= cfg_.tube_radius;
  }
};

class RetreatRouteHistory {
 public:
  RetreatRouteConfig cfg;
  explicit RetreatRouteHistory(RetreatRouteConfig c = {}) : cfg(c) {}
  void clear() { samples_.clear(); }
  size_t size() const { return samples_.size(); }

  // Supply real odometry with its measurement timestamp, only during ordinary
  // navigation. Planned points, held setpoints and recovery samples are not
  // evidence of a previously flown corridor. Missing samples, clock resets and
  // teleports discard the old connected run instead of fabricating a bridge.
  void add(Vec3 p, double stamp) {
    if (!cfg.valid() || !finite(p) || !std::isfinite(stamp)) { clear(); return; }
    if (!samples_.empty()) {
      const double dt = stamp - samples_.back().stamp;
      if (dt == 0) return;
      if (dt < 0 || dt > cfg.max_sample_gap || distance(p, samples_.back().p) > cfg.max_sample_step)
        clear();
    }
    samples_.push_back({p, stamp});
    while (!samples_.empty() && stamp - samples_.front().stamp > cfg.history_seconds) samples_.pop_front();
    while (samples_.size() > cfg.max_points) samples_.pop_front();
  }

  // Certificate signature: bool(Vec3 a, Vec3 b, const RetreatRoute &frozen).
  // The callback can use frozen.covers(a,b,now) as tightly bounded traversed
  // evidence, alongside current full-body occupancy and glass restrictions.
  // Clearance signature: double(Vec3); only ranks ALREADY certified routes.
  template<class Certificate, class Clearance>
  RetreatRoute select(Vec3 current, double now, Certificate certified, Clearance clearance,bool planar=false) const {
    RetreatRoute empty;
    if (!cfg.valid() || !finite(current) || !std::isfinite(now) || samples_.size() < 2 ||
        now < samples_.back().stamp || now - samples_.back().stamp > cfg.max_sample_gap ||
        distance(current, samples_.back().p) > cfg.tube_radius) return empty;
    RetreatRoute frozen;
    frozen.cfg_ = cfg;
    for (const auto &s : samples_) if (now - s.stamp <= cfg.history_seconds) frozen.evidence_.push_back(s);
    if (frozen.evidence_.size() < 2) return empty;

    std::vector<Vec3> route{current};
    RetreatRoute best;
    double used = 0, best_score = -std::numeric_limits<double>::infinity();
    size_t cursor = frozen.evidence_.size() - 1, checks = 0;
    // Farthest-in-time certified shortcut erases hover jitter and completed
    // loops, but can never cut an unflown corner. Keep ordered waypoints when a
    // direct connection fails. Work is bounded even on prolonged noisy hover.
    while (cursor > 0 && route.size() < cfg.max_waypoints && used < cfg.max_length - 1e-8 &&
           checks < cfg.max_certificate_checks) {
      bool found = false;
      size_t chosen = cursor;
      Vec3 next{};
      for (size_t j = 0; j < cursor; ++j) {
        Vec3 q = frozen.evidence_[j].p;
        // The projected leg must still fit the original 4 cm measured tube.
        if(planar)q.z=current.z;
        const double leg = distance(route.back(), q);
        if (leg > cfg.max_length - used + 1e-8) continue;
        if (++checks > cfg.max_certificate_checks) break;
        if (!frozen.covers(route.back(), q, now) || !certified(route.back(), q, frozen)) continue;
        chosen = j; next = q; found = true; break;
      }
      if (!found) break;
      const Vec3 previous = route.back();
      const double leg = distance(previous, next);
      // Rank useful endpoints along the certified leg, rather than always
      // choosing the full distance or stopping beside another nearby wall.
      const size_t divisions = std::max<size_t>(1, static_cast<size_t>(std::ceil(leg / .05)));
      for (size_t k = 1; k <= divisions; ++k) {
        const double fraction = double(k) / divisions;
        const Vec3 q = previous + (next - previous) * fraction;
        const double length = used + leg * fraction, displacement = distance(current, q);
        if (length < cfg.min_length - 1e-8 || displacement < cfg.min_displacement - 1e-8 ||
            length > cfg.max_path_ratio * displacement + 1e-8 ||
            !frozen.endpointOldEnough(q, now)) continue;
        const double room = clearance(q);
        if (!std::isfinite(room)) continue;
        const double score = 3 * room + .25 * displacement - .1 * length -
          .25 * std::abs(length - cfg.preferred_length);
        if (score <= best_score) continue;
        RetreatRoute candidate = frozen;
        candidate.points = route;
        if (distance(candidate.points.back(), q) > 1e-8) candidate.points.push_back(q);
        candidate.length = length; candidate.displacement = displacement;
        // Keep the original connected evidence through the whole certified
        // source edge. Do not stamp an old return position with today's time.
        candidate.evidence_.erase(candidate.evidence_.begin(), candidate.evidence_.begin() + chosen);
        // The callback may constrain an endpoint (for example sonar geometry),
        // so a valid full edge must not silently authorize every shorter edge.
        auto budgeted_certificate = [&](Vec3 a, Vec3 b, const RetreatRoute &proof) {
          return ++checks <= cfg.max_certificate_checks && certified(a, b, proof);
        };
        if (!candidate.validAt(now) || !candidate.certifiedFrom(current, 1, now, budgeted_certificate)) continue;
        best_score = score; best = std::move(candidate);
      }
      if (leg > 1e-8) route.push_back(next);
      used += leg;
      cursor = chosen;
    }
    return best;
  }

 private:
  std::deque<RetreatRoute::Sample> samples_;
};

} // namespace fire_scout
