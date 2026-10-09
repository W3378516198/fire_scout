#pragma once
#include "geometry.hpp"
#include <algorithm>
#include <cmath>
namespace fire_scout {
struct MeasuredBrakeConfig {
  double response_time{.80},deceleration{.65},reserve{.25},comfort_speed_cap{.20};
  bool valid()const {
    return std::isfinite(response_time)&&response_time>=.1&&response_time<=3&&
      std::isfinite(deceleration)&&deceleration>=.1&&deceleration<=3&&
      std::isfinite(reserve)&&reserve>=0&&reserve<=1&&
      std::isfinite(comfort_speed_cap)&&comfort_speed_cap>=.05&&comfort_speed_cap<=.5;
  }
};
struct MeasuredBrakeDecision {
  Vec3 velocity{};bool momentum_risk{false},limited{false},hard_limited{false},comfort_limited{false};
  double measured_stop_distance{0};
};
// A speed setpoint does not erase measured momentum. Test that momentum and
// the command after response delay, in addition to the command's own sweep.
template<class Collision>
MeasuredBrakeDecision measuredStoppingVelocity(Vec3 p,Vec3 requested,Vec3 measured,
    const MeasuredBrakeConfig& cfg,Collision clear) {
  MeasuredBrakeDecision out;out.velocity=requested;
  if(!cfg.valid()||!finite(p)||!finite(requested)||!finite(measured)){
    out.velocity={};out.hard_limited=out.limited=out.momentum_risk=true;return out;
  }
  const double speed=norm(measured),command=norm(requested);
  out.measured_stop_distance=speed*cfg.response_time+speed*speed/(2*cfg.deceleration);
  if(speed>1e-6)out.momentum_risk=!clear(p,p+measured*(cfg.response_time+speed/(2*cfg.deceleration)));
  if(command<1e-8)return out;
  if(out.momentum_risk&&dot(requested,measured)>1e-8){
    out.velocity={};out.hard_limited=out.limited=true;return out;
  }
  const Vec3 delayed=p+measured*cfg.response_time;
  const bool delayed_clear=distance(p,delayed)<1e-8||clear(p,delayed);
  auto feasible=[&](double scale){
    const Vec3 v=requested*scale;const double s=command*scale;
    if(!clear(p,p+v*(cfg.response_time+s/(2*cfg.deceleration))))return false;
    // Retain an independently certified opposite command when momentum is
    // already unsafe: refusing braking/escape cannot undo the initial state.
    if(out.momentum_risk&&dot(requested,measured)<=0)return true;
    return delayed_clear&&clear(delayed,delayed+v*(s/(2*cfg.deceleration)));
  };
  if(feasible(1))return out;
  double lo=0,hi=1;
  for(int n=0;n<12;++n){const double m=(lo+hi)/2;if(feasible(m))lo=m;else hi=m;}
  out.velocity=requested*lo;out.hard_limited=out.limited=true;return out;
}
// Hard safety is evaluated with measured motion; preferred clearance controls
// speed only. A small odometry drift into the comfort band is not a collision
// and must not veto every candidate, including a certified away command.
template<class HardCollision,class PreferredClearance>
MeasuredBrakeDecision measuredStoppingVelocity(Vec3 p,Vec3 requested,Vec3 measured,
    const MeasuredBrakeConfig& cfg,HardCollision hard,PreferredClearance preferred) {
  auto out=measuredStoppingVelocity(p,requested,measured,cfg,hard);
  const double speed=norm(out.velocity);
  if(speed<1e-8)return out;
  auto preferredCommand=[&](double s){const Vec3 v=out.velocity*(s/speed);
    return preferred(p,p+v*(cfg.response_time+s/(2*cfg.deceleration)));};
  if(preferredCommand(speed))return out;
  double lo=0,hi=speed;
  for(int n=0;n<12;++n){const double m=(lo+hi)/2;
    if(preferredCommand(m))lo=m;else hi=m;}
  // A cap floor is not a minimum command: do not increase a small request or
  // undo the hard brake, directional, sonar or peer constraints already applied.
  const double allowed=std::min(speed,std::max(lo,cfg.comfort_speed_cap));
  if(allowed<speed-1e-8){out.velocity=out.velocity*(allowed/speed);
    out.limited=out.comfort_limited=true;}
  return out;
}
} // namespace fire_scout
