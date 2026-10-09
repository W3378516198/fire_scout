#pragma once
#include "geometry.hpp"
#include <cmath>
#include <vector>

namespace fire_scout {
// Geometry repair targets, not per-sample stop thresholds. A continuously
// sampled curve can exceed the preferred curvature yet remain dynamically
// feasible at a lower speed. Only a localized direction discontinuity is a
// stop/rotate maneuver; speed limits never become minimum commands.
struct TurnPolicyConfig {
  bool enabled{true};
  double max_curvature{1.6},min_curve_speed{.35};
  double capture_distance{.10},stop_speed{.10},yaw_tolerance{8*pi/180};
  double repair_budget_ms{16.0};
  bool valid()const {
    return std::isfinite(max_curvature)&&max_curvature>=.2&&max_curvature<=10 &&
      std::isfinite(min_curve_speed)&&min_curve_speed>=.1&&min_curve_speed<=1 &&
      std::isfinite(capture_distance)&&capture_distance>=.03&&capture_distance<=.20 &&
      std::isfinite(stop_speed)&&stop_speed>=.02&&stop_speed<=.15 &&
      std::isfinite(yaw_tolerance)&&yaw_tolerance>=2*pi/180&&yaw_tolerance<=15*pi/180 &&
      std::isfinite(repair_budget_ms)&&repair_budget_ms>=1&&repair_budget_ms<=40;
  }
};
struct TurnSample {double curvature{0},yaw_curvature{0},angle{0},speed{1e9};};
inline TurnSample turnSample(Vec3 a,Vec3 b,double lateral_accel,double yaw_rate){
  TurnSample out;const double la=norm(a),lb=norm(b);
  if(la<1e-7||lb<1e-7)return out;
  out.angle=std::acos(std::clamp(dot(a,b)/(la*lb),-1.,1.));
  const double ds=std::max(.001,.5*(la+lb));
  out.curvature=2*std::sin(out.angle*.5)/ds;
  if(out.curvature>1e-5)out.speed=std::sqrt(lateral_accel/out.curvature);
  const bool steep=std::abs(a.z)>3*std::hypot(a.x,a.y)&&std::abs(b.z)>3*std::hypot(b.x,b.y);
  if(!steep&&std::hypot(a.x,a.y)>1e-6&&std::hypot(b.x,b.y)>1e-6){
    out.yaw_curvature=std::abs(wrap(std::atan2(b.y,b.x)-std::atan2(a.y,a.x)))/ds;
    if(out.yaw_curvature>1e-5)out.speed=std::min(out.speed,.75*yaw_rate/out.yaw_curvature);
  }
  return out;
}
inline bool requiresTurnRepair(const TurnSample&s,const TurnPolicyConfig&c,double sharp){
  return s.angle>sharp || (c.enabled&&(s.curvature>c.max_curvature+1e-8||
    s.speed<c.min_curve_speed-1e-8));
}
// The geometry worker uses the repair predicate above. Execution uses a
// separate discontinuity test.
// Dense smooth bends (e.g. 3 cm samples on a 0.55 m radius) must not create
// dozens of zero-speed anchors. A localized >=20 degree discontinuity still
// stops if the curvature/speed targets cannot be met, as does any sharp turn.
inline bool requiresExecutionStop(const TurnSample&s,const TurnPolicyConfig&c,double sharp){
  return s.angle>sharp || (c.enabled&&s.angle>=20*pi/180&&requiresTurnRepair(s,c,sharp));
}
struct TurnPathAssessment {size_t stops{0},repair_samples{0};double max_curvature{0};};
inline TurnPathAssessment assessTurnPath(const std::vector<Vec3>&p,const TurnPolicyConfig&c,
    double lateral_accel,double yaw_rate,double sharp){
  TurnPathAssessment out;
  for(size_t i=1;i+1<p.size();++i){const auto s=turnSample(p[i]-p[i-1],p[i+1]-p[i],lateral_accel,yaw_rate);
    out.max_curvature=std::max(out.max_curvature,s.curvature);
    if(requiresTurnRepair(s,c,sharp))++out.repair_samples;
    if(requiresExecutionStop(s,c,sharp))++out.stops;}
  return out;
}
} // namespace fire_scout
