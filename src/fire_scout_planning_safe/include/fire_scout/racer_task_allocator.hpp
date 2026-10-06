#pragma once

#include "grid.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace fire_scout {

// The coordinator no longer performs frontier exploration. It only creates
// collision-free arrival slots near a shared mission goal. These slots exist
// solely because several vehicles cannot occupy the same 3-D point.
struct ArrivalSlotConfig {
  double resolution{0.15};
  double radius{1.0};
  double max_radius{2.0};
  double radius_step{0.35};
  double min_separation{0.90};
  double clearance_xy{0.48};
  double clearance_z{0.25};
  double z_min{0.65};
  double z_max{3.0};
  size_t angular_samples{16};
  bool include_exact_goal{true};

  bool valid() const {
    return std::isfinite(resolution) && resolution >= 0.05 && resolution <= 0.50 &&
           std::isfinite(radius) && radius >= 0 &&
           std::isfinite(max_radius) && max_radius >= radius &&
           std::isfinite(radius_step) && radius_step > 0 &&
           std::isfinite(min_separation) && min_separation >= 0 &&
           std::isfinite(clearance_xy) && clearance_xy >= 0 &&
           std::isfinite(clearance_z) && clearance_z >= 0 &&
           std::isfinite(z_min) && std::isfinite(z_max) && z_max > z_min &&
           angular_samples >= 4 && angular_samples <= 128;
  }
};

struct ArrivalSlot {
  int64_t id{0};
  Vec3 target{};
};

namespace route_detail {

inline int64_t slotId(Vec3 point, double resolution) {
  const Key k = key(point, resolution);
  return static_cast<int64_t>(KeyHash::mix(uint32_t(k.x)) ^
      KeyHash::mix(uint64_t(uint32_t(k.y)) + 0x9e3779b97f4a7c15ULL) ^
      KeyHash::mix(uint64_t(uint32_t(k.z)) + 0x243f6a8885a308d3ULL));
}

inline bool endpointClear(Vec3 point, const KeySet &occupied,
                          const ArrivalSlotConfig &cfg) {
  if (!finite(point) || point.z < cfg.z_min || point.z > cfg.z_max) return false;
  const int xy = int(std::ceil((cfg.clearance_xy + cfg.resolution) /
                               cfg.resolution));
  const int zz = int(std::ceil((cfg.clearance_z + cfg.resolution) /
                               cfg.resolution));
  const Key centre = key(point, cfg.resolution);
  for (int dx = -xy; dx <= xy; ++dx)
    for (int dy = -xy; dy <= xy; ++dy)
      for (int dz = -zz; dz <= zz; ++dz) {
        const Key voxel{centre.x + dx, centre.y + dy, centre.z + dz};
        if (!occupied.count(voxel)) continue;
        const Vec3 obstacle = center(voxel, cfg.resolution);
        if (std::hypot(obstacle.x - point.x, obstacle.y - point.y) <=
                cfg.clearance_xy + 0.87 * cfg.resolution &&
            std::abs(obstacle.z - point.z) <=
                cfg.clearance_z + 0.5 * cfg.resolution)
          return false;
      }
  return true;
}

inline double pointSegmentDistance(Vec3 point, Vec3 a, Vec3 b) {
  const Vec3 ab = b - a;
  const double length2 = dot(ab, ab);
  if (length2 < 1e-12) return distance(point, a);
  const double t = std::clamp(dot(point - a, ab) / length2, 0.0, 1.0);
  return distance(point, a + ab * t);
}

inline double nearestRouteDistance(Vec3 point, const std::vector<Vec3> &route) {
  if (route.empty()) return std::numeric_limits<double>::infinity();
  if (route.size() == 1) return distance(point, route.front());
  double best = std::numeric_limits<double>::infinity();
  for (size_t i = 1; i < route.size(); ++i)
    best = std::min(best, pointSegmentDistance(point, route[i - 1], route[i]));
  return best;
}

}  // namespace route_detail

inline std::vector<ArrivalSlot> makeArrivalSlots(
    const KeySet &occupied, Vec3 goal, size_t required,
    const ArrivalSlotConfig &cfg) {
  std::vector<ArrivalSlot> slots;
  if (!cfg.valid() || !finite(goal) || required == 0) return slots;

  auto append = [&](Vec3 candidate) {
    if (!route_detail::endpointClear(candidate, occupied, cfg)) return;
    for (const auto &slot : slots)
      if (distance(slot.target, candidate) < cfg.min_separation) return;
    slots.push_back({route_detail::slotId(candidate, cfg.resolution), candidate});
  };

  if (cfg.include_exact_goal) append(goal);
  if (slots.size() >= required) return slots;

  // A half-sample phase offset prevents every mission from favouring world +X.
  for (double radius = std::max(cfg.radius, cfg.radius_step);
       radius <= cfg.max_radius + 1e-9 && slots.size() < required;
       radius += cfg.radius_step) {
    for (size_t i = 0; i < cfg.angular_samples && slots.size() < required; ++i) {
      const double angle = 2.0 * pi * (double(i) + 0.5) /
                           double(cfg.angular_samples);
      append({goal.x + radius * std::cos(angle),
              goal.y + radius * std::sin(angle), goal.z});
    }
  }
  return slots;
}

inline bool conflictsWithReservedSlot(
    Vec3 candidate, const std::vector<Vec3> &reserved_targets,
    double minimum_separation) {
  for (Vec3 target : reserved_targets)
    if (distance(candidate, target) < minimum_separation) return true;
  return false;
}

inline bool slotMissingConfirmed(int64_t missing_since_ns, int64_t now_ns,
                                 double confirmation_time) {
  return missing_since_ns > 0 && now_ns >= missing_since_ns &&
         double(now_ns - missing_since_ns) * 1e-9 >= confirmation_time;
}

inline double routeTurnLoss(const std::vector<Vec3> &route) {
  double loss = 0;
  for (size_t i = 2; i < route.size(); ++i) {
    const Vec3 a = route[i - 1] - route[i - 2];
    const Vec3 b = route[i] - route[i - 1];
    const double scale = norm(a) * norm(b);
    if (scale < 1e-9) continue;
    loss += 1.0 - std::clamp(dot(a, b) / scale, -1.0, 1.0);
  }
  return loss;
}

inline double routeTimeConflictLoss(
    const std::vector<Vec3> &route,
    const std::vector<std::vector<Vec3>> &peer_routes,
    double separation, double speed, double dt) {
  if (route.size() < 2 || peer_routes.empty() || separation <= 0 || speed <= 0)
    return 0;
  const auto arc = arcLengths(route);
  if (arc.back() <= 0) return 0;
  double loss = 0;
  for (const auto &peer : peer_routes) {
    if (peer.size() < 2) continue;
    const auto peer_arc = arcLengths(peer);
    const double duration = std::min(arc.back(), peer_arc.back()) / speed;
    for (double t = 0; t <= duration + 1e-9; t += dt) {
      const Vec3 a = atArc(route, arc, std::min(arc.back(), t * speed));
      const Vec3 b = atArc(peer, peer_arc,
                           std::min(peer_arc.back(), t * speed));
      const double d = distance(a, b);
      if (d < separation) {
        const double q = 1.0 - d / separation;
        loss += q * q * dt;
      }
    }
  }
  return loss;
}

inline bool routesDistinct(const std::vector<Vec3> &a,
                           const std::vector<Vec3> &b,
                           double minimum_difference,
                           double spacing = 0.30) {
  if (a.size() < 2 || b.size() < 2) return true;
  const auto arc = arcLengths(a);
  double sum = 0;
  size_t count = 0;
  for (double s = 0; s <= arc.back() + 1e-9; s += spacing) {
    sum += route_detail::nearestRouteDistance(
        atArc(a, arc, std::min(s, arc.back())), b);
    ++count;
  }
  return count == 0 || sum / double(count) >= minimum_difference;
}

}  // namespace fire_scout
