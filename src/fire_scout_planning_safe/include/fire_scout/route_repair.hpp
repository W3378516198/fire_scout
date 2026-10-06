#pragma once

#include "grid.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace fire_scout {

// Geometry retained by the V19 planner is inspected independently from route
// search.  The first blocked edge determines whether an immediate stop is
// necessary; the last blocked edge determines where a small repair may rejoin
// the exact old tail.
struct RouteInspection {
  bool has_route{false};
  bool projection_valid{false};
  bool valid{false};
  double progress{0};
  double length{0};
  double first_invalid_distance{std::numeric_limits<double>::infinity()};
  double last_invalid_distance{0};
  Vec3 first_invalid_point{};
  std::vector<Vec3> suffix;
  std::vector<bool> edge_valid;
};

inline bool routeInvalidationConfirmed(
    double invalid_distance, double near_distance, bool matched_snapshot,
    size_t matched_updates, double age, size_t near_updates,
    double near_time, size_t ordinary_updates, double ordinary_time) {
  if (!std::isfinite(invalid_distance) || !std::isfinite(near_distance) ||
      !std::isfinite(age) || !std::isfinite(near_time) ||
      !std::isfinite(ordinary_time) || near_distance < 0 || age < 0 ||
      near_updates == 0 || ordinary_updates == 0)
    return false;
  const bool near = invalid_distance <= near_distance;
  const bool near_persistent = near && matched_snapshot &&
      matched_updates >= near_updates && age >= near_time;
  const bool ordinary_persistent = matched_updates >= ordinary_updates &&
      age >= ordinary_time;
  return near_persistent || ordinary_persistent;
}

inline Vec3 forwardRouteTangent(const std::vector<Vec3> &route,
                                const std::vector<double> &arc, double s) {
  if (route.size() < 2 || arc.empty() || arc.back() < 1e-9) return {};
  const double ahead = std::min(arc.back(), s + .15);
  Vec3 tangent = atArc(route, arc, ahead) - atArc(route, arc, s);
  if (norm(tangent) < 1e-8) {
    const double behind = std::max(0.0, s - .15);
    tangent = atArc(route, arc, s) - atArc(route, arc, behind);
  }
  const double length = norm(tangent);
  return length < 1e-8 ? Vec3{} : tangent * (1.0 / length);
}

// Find a forward, tangent-continuous point at which the live vehicle can join
// the retained route.  A perpendicular projection can be a few centimetres
// behind a moving vehicle; prepending that projection creates a 180 degree
// micro-hairpin which the follower quite correctly treats as a mandatory stop.
inline double forwardRouteJoinArc(const std::vector<Vec3> &route,
                                  const std::vector<double> &arc, Vec3 current,
                                  double start) {
  if (route.size() < 2 || arc.empty() || !finite(current) ||
      !std::isfinite(start))
    return std::numeric_limits<double>::quiet_NaN();
  start = std::clamp(start, 0.0, arc.back());
  if (arc.back() - start < .03) return start;
  const double end = std::min(arc.back(), start + 1.50);
  constexpr double minimum_cosine = .25881904510252074;  // cos(75 deg)
  for (size_t step = 0;; ++step) {
    const double s = std::min(end, start + .04 * step);
    const Vec3 point = atArc(route, arc, s);
    const Vec3 connector = point - current;
    const double connector_length = norm(connector);
    if (connector_length <= .02) return s;
    const Vec3 tangent = forwardRouteTangent(route, arc, s);
    if (norm(tangent) < .5) continue;
    const double cosine = dot(connector, tangent) / connector_length;
    if (cosine >= minimum_cosine) return s;
    if (s >= end - 1e-9) break;
  }
  return std::numeric_limits<double>::quiet_NaN();
}

inline std::vector<Vec3> routeSuffixAt(const std::vector<Vec3> &route,
                                       Vec3 current,
                                       double progress_hint = 0) {
  if (route.empty() || !finite(current)) return {};
  if (route.size() == 1) return {current, route.front()};
  const auto arc = arcLengths(route);
  const auto projection = project(
      route, arc, current, std::max(0.0, progress_hint - .10),
      std::min(arc.back(), std::max(progress_hint + 2.0, 2.0)));
  if (!std::isfinite(projection.error) || projection.error > 1.2) return {};
  const double projected_start = std::max(progress_hint, projection.s);
  const double start =
      forwardRouteJoinArc(route, arc, current, projected_start);
  if (!std::isfinite(start)) return {};
  std::vector<Vec3> suffix{current};
  const Vec3 on_route = atArc(route, arc, start);
  if (distance(current, on_route) > .02) suffix.push_back(on_route);
  for (size_t i = 1; i < route.size(); ++i)
    if (arc[i] > start + 1e-8 &&
        distance(suffix.back(), route[i]) > 1e-6)
      suffix.push_back(route[i]);
  if (suffix.size() == 1 && distance(current, route.back()) > 1e-6)
    suffix.push_back(route.back());
  return suffix;
}

inline RouteInspection inspectRoute(const Grid &grid,
                                    const std::vector<Vec3> &route,
                                    Vec3 current, bool allow_unknown = true,
                                    double progress_hint = 0) {
  RouteInspection result;
  result.has_route = !route.empty();
  result.suffix = routeSuffixAt(route, current, progress_hint);
  result.projection_valid = result.suffix.size() >= 2;
  if (!result.projection_valid) {
    result.first_invalid_distance = 0;
    result.first_invalid_point = current;
    return result;
  }
  const auto arc = arcLengths(result.suffix);
  result.length = arc.back();
  result.edge_valid.reserve(result.suffix.size() - 1);
  for (size_t i = 1; i < result.suffix.size(); ++i) {
    const Vec3 a = result.suffix[i - 1], b = result.suffix[i];
    const bool clear = grid.segmentFrom(a, b, allow_unknown);
    result.edge_valid.push_back(clear);
    if (clear) continue;
    if (!std::isfinite(result.first_invalid_distance)) {
      // Find the last certified point of a sparse edge.  This makes the near
      // obstacle gate reflect physical distance, not the preceding waypoint.
      double lo = 0, hi = 1;
      const Vec3 delta = b - a;
      for (int n = 0; n < 14; ++n) {
        const double mid = .5 * (lo + hi);
        if (grid.segmentFrom(a, a + delta * mid, allow_unknown))
          lo = mid;
        else
          hi = mid;
      }
      result.first_invalid_distance = arc[i - 1] + lo * distance(a, b);
      result.first_invalid_point = a + delta * hi;
    }
    result.last_invalid_distance = arc[i];
  }
  result.valid = std::isinf(result.first_invalid_distance);
  if (result.valid) result.first_invalid_point = result.suffix.back();
  return result;
}

inline std::vector<Vec3> routeSlice(const std::vector<Vec3> &route,
                                    double begin, double end) {
  if (route.empty()) return {};
  const auto arc = arcLengths(route);
  begin = std::clamp(begin, 0.0, arc.back());
  end = std::clamp(end, begin, arc.back());
  std::vector<Vec3> out{atArc(route, arc, begin)};
  for (size_t i = 1; i + 1 < route.size(); ++i)
    if (arc[i] > begin + 1e-8 && arc[i] < end - 1e-8)
      out.push_back(route[i]);
  const Vec3 finish = atArc(route, arc, end);
  if (distance(out.back(), finish) > 1e-6) out.push_back(finish);
  return out;
}

struct LocalRepairWindow {
  bool valid{false};
  double invalid_distance{0};
  double anchor_distance{0};
  double rejoin_distance{0};
  std::vector<Vec3> prefix;
  std::vector<Vec3> tail;
};

inline LocalRepairWindow localRepairWindow(const RouteInspection &inspection,
                                           double pre_margin,
                                           double post_margin,
                                           double max_patch_span) {
  LocalRepairWindow window;
  if (inspection.valid || !inspection.projection_valid ||
      !std::isfinite(inspection.first_invalid_distance) ||
      inspection.edge_valid.empty() || pre_margin < 0 || post_margin < 0 ||
      max_patch_span <= 0)
    return window;
  window.invalid_distance = inspection.first_invalid_distance;
  window.anchor_distance =
      std::max(0.0, inspection.first_invalid_distance - pre_margin);
  window.rejoin_distance =
      std::min(inspection.length,
               inspection.last_invalid_distance + post_margin);
  if (window.rejoin_distance - window.anchor_distance > max_patch_span ||
      window.rejoin_distance >= inspection.length - .20)
    return window;
  window.prefix = routeSlice(inspection.suffix, 0, window.anchor_distance);
  window.tail = routeSlice(inspection.suffix, window.rejoin_distance,
                           inspection.length);
  window.valid = !window.prefix.empty() && window.tail.size() >= 2 &&
                 distance(window.prefix.back(), window.tail.front()) > .20;
  return window;
}

inline std::vector<Vec3> spliceLocalRepair(
    const LocalRepairWindow &window, const std::vector<Vec3> &patch,
    size_t *patch_begin = nullptr, size_t *patch_end = nullptr) {
  if (!window.valid || patch.size() < 2 ||
      distance(window.prefix.back(), patch.front()) > .08 ||
      distance(window.tail.front(), patch.back()) > .08)
    return {};
  std::vector<Vec3> out = window.prefix;
  const size_t begin = out.size() - 1;
  for (const Vec3 point : patch)
    if (distance(out.back(), point) > 1e-6) out.push_back(point);
  const size_t end = out.size() - 1;
  for (const Vec3 point : window.tail)
    if (distance(out.back(), point) > 1e-6) out.push_back(point);
  if (patch_begin) *patch_begin = begin;
  if (patch_end) *patch_end = end;
  return out;
}

inline bool routeGeometryValid(const Grid &grid,
                               const std::vector<Vec3> &route,
                               bool allow_unknown = true) {
  if (route.size() < 2) return false;
  for (size_t i = 1; i < route.size(); ++i)
    if (!grid.segmentFrom(route[i - 1], route[i], allow_unknown)) return false;
  return true;
}

}  // namespace fire_scout
