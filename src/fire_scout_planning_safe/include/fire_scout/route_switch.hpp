#pragma once

#include "route_repair.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace fire_scout {

// Parameters for comparing a newly planned route with the route that the
// vehicle has already committed to. Angles are stored in radians internally;
// the ROS node exposes the corresponding parameters in degrees.
struct RouteSwitchConfig {
  bool enabled{true};
  double lookahead{.90};
  double backtrack_horizon{1.50};
  double soft_angle{20.0 * pi / 180.0};
  double medium_angle{45.0 * pi / 180.0};
  double large_angle{90.0 * pi / 180.0};
  double reverse_angle{120.0 * pi / 180.0};
  double angle_weight{2.50};
  double backtrack_weight{4.00};
  double minimum_absolute_gain{.15};
  double minimum_gain_ratio{.05};
  double medium_gain_ratio{.12};
  double large_gain_ratio{.25};
  double reverse_gain_ratio{.40};
  double confirmation_time{.75};
  double cooldown{2.0};
  double candidate_heading_tolerance{18.0 * pi / 180.0};
  double candidate_endpoint_tolerance{.40};

  bool valid() const {
    if (!std::isfinite(lookahead) || lookahead < .20 || lookahead > 3.0 ||
        !std::isfinite(backtrack_horizon) || backtrack_horizon < .20 ||
        backtrack_horizon > 5.0 || !std::isfinite(soft_angle) ||
        !std::isfinite(medium_angle) || !std::isfinite(large_angle) ||
        !std::isfinite(reverse_angle) || soft_angle < 0 ||
        medium_angle <= soft_angle || large_angle <= medium_angle ||
        reverse_angle <= large_angle || reverse_angle > pi ||
        !std::isfinite(angle_weight) || angle_weight < 0 ||
        !std::isfinite(backtrack_weight) || backtrack_weight < 0 ||
        !std::isfinite(minimum_absolute_gain) || minimum_absolute_gain < 0 ||
        !std::isfinite(confirmation_time) || confirmation_time < 0 ||
        confirmation_time > 5.0 || !std::isfinite(cooldown) || cooldown < 0 ||
        cooldown > 15.0 || !std::isfinite(candidate_heading_tolerance) ||
        candidate_heading_tolerance <= 0 || candidate_heading_tolerance > pi ||
        !std::isfinite(candidate_endpoint_tolerance) ||
        candidate_endpoint_tolerance <= 0 || candidate_endpoint_tolerance > 3.0)
      return false;
    const double ratios[] = {minimum_gain_ratio, medium_gain_ratio,
                             large_gain_ratio, reverse_gain_ratio};
    for (double value : ratios)
      if (!std::isfinite(value) || value < 0 || value >= 1.0) return false;
    return minimum_gain_ratio <= medium_gain_ratio &&
           medium_gain_ratio <= large_gain_ratio &&
           large_gain_ratio <= reverse_gain_ratio;
  }
};

struct RouteTransition {
  bool valid{false};
  double angle{0};
  double backtrack{0};
  double heading{0};
  Vec3 endpoint{};
};

struct RouteSwitchAssessment {
  RouteTransition transition;
  double old_total{std::numeric_limits<double>::infinity()};
  double new_total{std::numeric_limits<double>::infinity()};
  double angle_penalty{0};
  double backtrack_penalty{0};
  double effective_new_total{std::numeric_limits<double>::infinity()};
  double gain{0};
  double gain_ratio{0};
  double required_gain_ratio{0};
  bool cost_allows{false};
  bool needs_confirmation{false};
};

inline std::vector<Vec3> routeSuffixFrom(const std::vector<Vec3> &route,
                                         Vec3 current,
                                         double progress_hint =
                                             std::numeric_limits<double>::quiet_NaN()) {
  if (route.empty()) return {};
  if (route.size() == 1) return {current, route.front()};
  const auto arc = arcLengths(route);
  const auto projection = std::isfinite(progress_hint)
      ? project(route, arc, current, std::max(0.0, progress_hint - .10),
                std::min(arc.back(), progress_hint + 2.0))
      : project(route, arc, current);
  if (!std::isfinite(projection.error) || projection.error > 2.0) return route;
  std::vector<Vec3> suffix{current};
  const double hinted = std::isfinite(progress_hint)
      ? std::max(progress_hint, projection.s)
      : projection.s;
  const double start = forwardRouteJoinArc(route, arc, current, hinted);
  if (!std::isfinite(start)) return {};
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

inline Vec3 horizontalRouteDirection(const std::vector<Vec3> &route,
                                     Vec3 current, double lookahead) {
  const auto suffix = routeSuffixFrom(route, current);
  if (suffix.size() < 2) return {};
  const auto arc = arcLengths(suffix);
  if (arc.back() < 1e-6) return {};
  const Vec3 target = atArc(suffix, arc, std::min(arc.back(), lookahead));
  Vec3 direction{target.x - current.x, target.y - current.y, 0};
  double length = std::hypot(direction.x, direction.y);
  if (length < .02) {
    for (size_t i = 1; i < suffix.size(); ++i) {
      direction = {suffix[i].x - current.x, suffix[i].y - current.y, 0};
      length = std::hypot(direction.x, direction.y);
      if (length >= .02) break;
    }
  }
  return length < .02 ? Vec3{} : direction * (1.0 / length);
}

inline double routeBacktrackDistance(const std::vector<Vec3> &candidate,
                                     Vec3 current, Vec3 reference,
                                     double horizon) {
  if (std::hypot(reference.x, reference.y) < .5 || horizon <= 0) return 0;
  const auto suffix = routeSuffixFrom(candidate, current);
  if (suffix.size() < 2) return 0;
  const auto arc = arcLengths(suffix);
  const double end = std::min(horizon, arc.back());
  double reverse = 0;
  Vec3 previous = current;
  for (double s = std::min(.10, end); s < end + 1e-9; s += .10) {
    const Vec3 point = atArc(suffix, arc, std::min(s, end));
    const Vec3 step{point.x - previous.x, point.y - previous.y, 0};
    reverse += std::max(0.0, -dot(step, reference));
    previous = point;
  }
  if (end > 0 && distance(previous, atArc(suffix, arc, end)) > 1e-8) {
    const Vec3 point = atArc(suffix, arc, end);
    const Vec3 step{point.x - previous.x, point.y - previous.y, 0};
    reverse += std::max(0.0, -dot(step, reference));
  }
  return reverse;
}

inline RouteTransition routeTransition(const std::vector<Vec3> &incumbent,
                                       const std::vector<Vec3> &candidate,
                                       Vec3 current,
                                       const RouteSwitchConfig &config) {
  RouteTransition result;
  if (candidate.empty()) return result;
  const Vec3 old_direction =
      horizontalRouteDirection(incumbent, current, config.lookahead);
  const Vec3 new_direction =
      horizontalRouteDirection(candidate, current, config.lookahead);
  const double old_length = std::hypot(old_direction.x, old_direction.y);
  const double new_length = std::hypot(new_direction.x, new_direction.y);
  result.endpoint = candidate.back();
  if (new_length >= .5) result.heading = std::atan2(new_direction.y, new_direction.x);
  if (old_length < .5 || new_length < .5) return result;
  result.valid = true;
  result.angle = std::acos(std::clamp(dot(old_direction, new_direction), -1.0, 1.0));
  result.backtrack = routeBacktrackDistance(
      candidate, current, old_direction, config.backtrack_horizon);
  return result;
}

inline double interpolateSwitchRequirement(double angle,
                                           const RouteSwitchConfig &config) {
  const auto interpolate = [](double x, double x0, double x1,
                              double y0, double y1) {
    if (x1 <= x0) return y1;
    const double t = std::clamp((x - x0) / (x1 - x0), 0.0, 1.0);
    return y0 + t * (y1 - y0);
  };
  if (angle <= config.soft_angle) return config.minimum_gain_ratio;
  if (angle <= config.medium_angle)
    return interpolate(angle, config.soft_angle, config.medium_angle,
                       config.minimum_gain_ratio, config.medium_gain_ratio);
  if (angle <= config.large_angle)
    return interpolate(angle, config.medium_angle, config.large_angle,
                       config.medium_gain_ratio, config.large_gain_ratio);
  if (angle <= config.reverse_angle)
    return interpolate(angle, config.large_angle, config.reverse_angle,
                       config.large_gain_ratio, config.reverse_gain_ratio);
  return config.reverse_gain_ratio;
}

inline RouteSwitchAssessment assessRouteSwitch(
    const std::vector<Vec3> &incumbent,
    const std::vector<Vec3> &candidate, Vec3 current,
    double old_total, double new_total, const RouteSwitchConfig &config) {
  RouteSwitchAssessment result;
  result.transition = routeTransition(incumbent, candidate, current, config);
  result.old_total = old_total;
  result.new_total = new_total;
  result.required_gain_ratio = result.transition.valid
      ? interpolateSwitchRequirement(result.transition.angle, config)
      : config.minimum_gain_ratio;
  if (!std::isfinite(old_total) || !std::isfinite(new_total)) return result;
  if (result.transition.valid) {
    result.angle_penalty = config.angle_weight *
        (1.0 - std::cos(result.transition.angle));
    result.backtrack_penalty =
        config.backtrack_weight * result.transition.backtrack;
  }
  result.effective_new_total = new_total + result.angle_penalty +
                               result.backtrack_penalty;
  result.gain = old_total - result.effective_new_total;
  result.gain_ratio = result.gain / std::max(1.0, std::abs(old_total));
  result.cost_allows = result.gain >= config.minimum_absolute_gain &&
                       result.gain_ratio >= result.required_gain_ratio;
  result.needs_confirmation = result.transition.valid &&
      (result.transition.angle > config.soft_angle ||
       result.transition.backtrack > .10);
  return result;
}

inline bool samePendingSwitch(const RouteTransition &a,
                              const RouteTransition &b,
                              const RouteSwitchConfig &config) {
  if (!a.valid || !b.valid) return false;
  return distance(a.endpoint, b.endpoint) <=
             config.candidate_endpoint_tolerance &&
         std::abs(wrap(a.heading - b.heading)) <=
             config.candidate_heading_tolerance;
}

}  // namespace fire_scout
