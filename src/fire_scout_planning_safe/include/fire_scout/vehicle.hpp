#pragma once
#include "geometry.hpp"

namespace fire_scout {
// wheelbase is the distance BETWEEN OPPOSITE motor axes, not the rotor-tip span.
struct VehicleConfig {
  double wheelbase{.50}, propeller_diameter{.28};
  double body_length{.30}, body_width{.22}, height{.25};
  double max_tilt_deg{15.0}, margin_xy{.08}, tracking_margin{.04}, margin_z{.05};
  double physicalRadius() const {
    return std::max(.5 * (wheelbase + propeller_diameter),
                    .5 * std::hypot(body_length, body_width));
  }
  double tiltedRadius() const {
    const double r = physicalRadius(), h = height * .5;
    const double a = std::min(max_tilt_deg * pi / 180, std::atan2(h, r));
    return r * std::cos(a) + h * std::sin(a);
  }
  double radius() const { return tiltedRadius() + margin_xy + tracking_margin; }
  // A narrow passage may spend COMFORT clearance, never rotor/tilt geometry or
  // the tracking reserve. The remaining fixed margin is independently bounded.
  double passageRadius(double minimum_margin=.02) const {
    if(!std::isfinite(minimum_margin)||minimum_margin<.02||minimum_margin>margin_xy)
      throw std::runtime_error("minimum safety margin must be in [0.02,safety_margin_xy]");
    return tiltedRadius()+tracking_margin+minimum_margin;
  }
  double halfHeight() const {
    const double a = std::min(max_tilt_deg * pi / 180, std::atan2(physicalRadius(), height * .5));
    return height * .5 * std::cos(a) + physicalRadius() * std::sin(a) + margin_z;
  }
  bool valid() const {
    const double p[]{wheelbase, propeller_diameter, body_length, body_width, height,
                     max_tilt_deg, margin_xy, tracking_margin, margin_z};
    for (double x : p) if (!std::isfinite(x) || x < 0) return false;
    return wheelbase > 0 && propeller_diameter > 0 && height > 0 && max_tilt_deg < 60;
  }
};
} // namespace fire_scout
