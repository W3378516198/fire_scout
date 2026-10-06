#include "fire_scout/directional_motion.hpp"
#include "fire_scout/tracker.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

using namespace fire_scout;

namespace {
bool near(double a,double b,double tolerance=1e-9){
  return std::abs(a-b)<=tolerance;
}
}

int main(){
  TrackerConfig cfg;
  cfg.heading_speed_scaling_enabled=true;
  cfg.allow_forward_during_alignment=true;
  cfg.heading_full_speed=10*pi/180;
  cfg.heading_slow=25*pi/180;
  cfg.heading_min_speed=60*pi/180;
  cfg.heading_stop=85*pi/180;
  cfg.heading_slow_speed_scale=.60;
  cfg.heading_min_speed_scale=.25;
  cfg.min_forward_speed=.12;

  assert(near(headingSpeedScale(cfg,0),1));
  assert(near(headingSpeedScale(cfg,cfg.heading_full_speed),1));
  assert(near(headingSpeedScale(cfg,cfg.heading_slow),.60));
  assert(near(headingSpeedScale(cfg,cfg.heading_min_speed),.25));
  assert(near(headingSpeedScale(cfg,80*pi/180),.25));
  assert(near(headingSpeedScale(cfg,cfg.heading_stop),0));

  DirectionalConfig directional_cfg;
  directional_cfg.max_lateral_speed=.35;
  directional_cfg.max_reverse_speed=.25;
  directional_cfg.forward_half_angle=8*pi/180;
  directional_cfg.sonar_half_angle=10*pi/180;
  directional_cfg.prediction_horizon=.8;
  directional_cfg.alignment_forward_speed_cap=.25;

  DirectionalRequest request;
  request.position={0,0,1};
  request.velocity={.20*std::cos(60*pi/180),.20*std::sin(60*pi/180),0};
  request.body_yaw=0;
  request.sonar_yaw=0;
  request.sonar_valid=true;
  request.sonar_required=true;
  request.allow_alignment_forward=true;
  auto clear=[](Vec3,Vec3){return true;};
  auto result=constrainDirectional(request,directional_cfg,clear,clear);
  assert(!result.blocked);
  assert(result.alignment_forward);
  assert(result.reason==DirectionalReason::ALIGNMENT_FORWARD);
  assert(!outsideForwardCone(result.velocity,0,directional_cfg.forward_half_angle));
  assert(result.velocity.x*request.velocity.x+result.velocity.y*request.velocity.y>0);
  assert(std::hypot(result.velocity.x,result.velocity.y)<=
         directional_cfg.alignment_forward_speed_cap+1e-9);

  auto unobserved=constrainDirectional(request,directional_cfg,
    [](Vec3,Vec3){return false;},clear);
  assert(unobserved.blocked);
  assert(unobserved.requires_alignment);
  request.allow_alignment_forward=false;
  auto disabled=constrainDirectional(request,directional_cfg,clear,clear);
  assert(disabled.blocked);
  assert(disabled.reason==DirectionalReason::ALIGNMENT_REQUIRED);

  cfg.max_speed_xy=1.2;
  cfg.max_accel_xy=3.0;
  cfg.max_jerk_xy=12.0;
  cfg.sharp_turn=80*pi/180;
  cfg.strict_forward_half_angle=8*pi/180;
  Tracker tracker(cfg);
  const Vec3 current{0,0,1};
  const double route_angle=70*pi/180;
  tracker.initializeYaw(0);
  tracker.setPath({current,{5*std::cos(route_angle),5*std::sin(route_angle),1}},current);
  Control control;
  for(int i=0;i<120;++i)
    control=tracker.update(current,{},.025,clear,0,.18,{},HeadingMode::Strict,clear);
  assert(control.alignment_active);
  assert(control.alignment_forward_enabled);
  assert(control.alignment_min_speed_active);
  assert(!control.heading_wait);
  assert(near(control.heading_speed_scale,.25,1e-6));
  const double command_xy=std::hypot(control.velocity.x,control.velocity.y);
  assert(command_xy>=.10);
  assert(command_xy<=.18+1e-6);

  DirectionalRequest integrated_request=request;
  integrated_request.velocity=control.velocity;
  integrated_request.allow_alignment_forward=control.alignment_forward_enabled;
  auto integrated=constrainDirectional(integrated_request,directional_cfg,clear,clear);
  assert(!integrated.blocked);
  assert(integrated.alignment_forward);
  assert(std::hypot(integrated.velocity.x,integrated.velocity.y)>=.10);
  assert(!outsideForwardCone(integrated.velocity,0,directional_cfg.forward_half_angle));

  // A path replacement can arrive while the previous command is still
  // decelerating.  Once the vehicle is spatially on the outgoing leg, an old
  // centimetre-scale hairpin behind it must be skipped rather than revisited.
  Tracker repaired(cfg);
  repaired.initializeYaw(0);
  repaired.setPath({{0,0,1},{-.03,0,1},{2,0,1}},{0,0,1});
  auto repaired_control=repaired.update({.15,0,1},{.20,0,0},.025,clear,0,
    1.0,{},HeadingMode::Strict,clear);
  assert(!repaired_control.heading_wait);
  assert(std::abs(repaired_control.course_error)<10*pi/180);
  assert(repaired_control.velocity.x>=-1e-9);

  // Startup spreading uses a direct point target without installing a mission
  // path. It must wait for a 90-degree turn, then exceed the legacy 0.25 m/s
  // station-keeping cap while respecting its dedicated speed limit.
  Tracker staging(cfg);
  staging.initializeYaw(0);
  auto turning=staging.directMove(current,{0,9,1},{},.025,.80,0);
  assert(turning.heading_wait);
  assert(std::hypot(turning.velocity.x,turning.velocity.y)<1e-9);
  Control staging_control;
  for(int i=0;i<120;++i)
    staging_control=staging.directMove(current,{0,9,1},{},.025,.80,pi/2);
  assert(!staging_control.heading_wait);
  assert(staging_control.velocity.y>.25);
  assert(std::hypot(staging_control.velocity.x,staging_control.velocity.y)<=.80+1e-9);
  assert(near(staging_control.target.y,9));

  std::cout<<"follower_alignment_test: PASS\n";
  return 0;
}
