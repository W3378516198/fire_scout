#pragma once

#include "route_topology.hpp"

namespace fire_scout {

// A review is advisory: collision checks and peer separation must bypass it.
// Probe a few arclength samples per call instead of starting a whole search
// merely because another aircraft has a route. An unfinished sweep survives
// new map frames, otherwise a long route's far entrance may never be reached.
class SharedEntranceReview {
public:
  void clear() { *this = SharedEntranceReview{}; }

  bool shouldReview(const Grid &grid, const std::vector<Vec3> &route,
      double progress, Vec3 current, const std::vector<std::vector<Vec3>> &peers,
      const TopologyConfig &cfg, double wall_seconds, uint64_t peer_revision,
      size_t sample_budget = 8, double budget_ms = 3.0) {
    last_samples_ = 0;
    if (!cfg.enabled || route.size() < 2 || peers.empty() ||
        !std::isfinite(wall_seconds) || !std::isfinite(progress) ||
        sample_budget == 0 || !(budget_ms > 0)) return false;
    const bool route_changed = !sameGeometry(route_, route);
    const bool peers_changed = !have_peer_revision_ || peer_revision_ != peer_revision;
    if (route_changed) {
      route_ = route; arc_ = arcLengths(route_); active_ = false;
      next_scan_ = wall_seconds;
    }
    if (peers_changed) {
      peer_revision_ = peer_revision; have_peer_revision_ = true;
      // A newcomer is not hidden behind a previous no-improvement cooldown.
      if (!active_) next_scan_ = wall_seconds;
    }
    if (arc_.empty() || arc_.back() < 1.6) return false;
    if (wall_seconds < last_wall_) { active_ = false; next_scan_ = wall_seconds; }
    last_wall_ = wall_seconds;
    if (!active_) {
      if (wall_seconds < next_scan_) return false;
      scan_ = std::max(.8, progress + .8); active_ = true;
    }
    scan_ = std::max(scan_, progress + .8);
    const auto begin = std::chrono::steady_clock::now();
    const auto stop = [&] {
      return std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - begin).count() >= budget_ms;
    };
    const double step = std::max(.35, grid.cfg.resolution * 2);
    while (scan_ < arc_.back() - .8 && last_samples_ < sample_budget && !stop()) {
      // A 1.6002m window makes routePassages inspect exactly its central
      // sample, using the SAME body envelope and wall-side evidence as the
      // full entrance selector. No alternative geometry is constructed here.
      const double lo = scan_ - .8, hi = scan_ + .8002;
      std::vector<Vec3> window{atArc(route_, arc_, lo)};
      auto first = std::upper_bound(arc_.begin(), arc_.end(), lo);
      for (size_t i = size_t(first - arc_.begin()); i < arc_.size() && arc_[i] < hi; ++i)
        window.push_back(route_[i]);
      window.push_back(atArc(route_, arc_, hi));
      bool sample_aborted = false;
      const auto gates = routePassages(grid, window, cfg, [&] {
        sample_aborted = stop(); return sample_aborted;
      });
      if (sample_aborted) break; // Resume this sample on the next callback.
      ++last_samples_; scan_ += step;
      for (const auto &gate : reservablePassages(gates, current, route_.back(), cfg)) {
        if (!routeCrossesPassage(route_, gate)) continue;
        for (const auto &peer : peers) if (routeCrossesPassage(peer, gate)) {
          active_ = false; next_scan_ = wall_seconds + cfg.refresh_period;
          return true;
        }
      }
    }
    if (scan_ >= arc_.back() - .8) {
      active_ = false; next_scan_ = wall_seconds + cfg.refresh_period;
    }
    return false;
  }

  // Re-evaluating the same single usable doorway has no navigation benefit.
  // Retry it later; a changed route/peer, safety repair or mission still wins.
  void noImprovement(double wall_seconds, double refresh_period) {
    active_ = false;
    next_scan_ = std::max(next_scan_, wall_seconds + std::max(1., 3 * refresh_period));
  }
  size_t lastSamples() const { return last_samples_; }
  bool scanning() const { return active_; }
  double scanArc() const { return scan_; }

private:
  static bool sameGeometry(const std::vector<Vec3> &a, const std::vector<Vec3> &b) {
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); ++i)
      if (distance(a[i], b[i]) > 1e-6) return false;
    return true;
  }
  std::vector<Vec3> route_;
  std::vector<double> arc_;
  uint64_t peer_revision_{0};
  bool have_peer_revision_{false};
  double scan_{.8}, next_scan_{0}, last_wall_{0};
  size_t last_samples_{0};
  bool active_{false};
};

// Only reject/no-handoff retries of a VALID partial route are throttled.
// Do not use this gate for missing routes, confirmed hazards or a new goal.
class PartialRouteRetryGate {
public:
  void clear() { failures_ = 0; until_ = 0; }
  bool ready(double wall_seconds, Vec3 endpoint, Vec3 goal, double remaining) const {
    return failures_ == 0 || !std::isfinite(wall_seconds) || wall_seconds < last_ ||
      distance(endpoint, endpoint_) > .1 || distance(goal, goal_) > .1 ||
      std::abs(remaining - remaining_) >= .25 || wall_seconds >= until_;
  }
  void rejected(double wall_seconds, Vec3 endpoint, Vec3 goal, double remaining) {
    if (distance(endpoint, endpoint_) > .1 || distance(goal, goal_) > .1 ||
        std::abs(remaining - remaining_) >= .25 || wall_seconds < last_) failures_ = 0;
    failures_ = std::min(size_t(3), failures_ + 1);
    last_ = wall_seconds; endpoint_ = endpoint; goal_ = goal; remaining_ = remaining;
    until_ = wall_seconds + .25 * double(size_t(1) << (failures_ - 1));
  }
private:
  size_t failures_{0};
  double until_{0}, last_{0}, remaining_{0};
  Vec3 endpoint_{}, goal_{};
};
} // namespace fire_scout
