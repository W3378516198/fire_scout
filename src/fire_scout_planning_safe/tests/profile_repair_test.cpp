#include "fire_scout/profile_repair.hpp"
#include <cassert>
#include <iostream>
using namespace fire_scout;

int main(){
  GridConfig gc;gc.resolution=.05;gc.inflation_xy=.15;gc.inflation_z=.15;
  Grid clear(gc);RouteHandoffConfig handoff;handoff.budget_ms=50;
  RouteSwitchConfig policy;TrackerConfig tracker;tracker.turn.enabled=false; // Regression of the optional legacy low-profile repair; global turn policy has its own tests.

  tracker.max_speed_xy=1.8;tracker.max_accel_xy=1.5;tracker.max_jerk_xy=4.5;
  tracker.yaw_rate=1.2;tracker.lateral_accel=1.;tracker.sharp_turn=80*pi/180;
  // A 1 cm dogleg in a densely sampled, otherwise straight route creates the
  // real follower's low curvature/yaw ceiling. No speed floor is changed.
  auto path=densify({{0,0,1.5},{.32,0,1.5},{.34,.01,1.5},{.36,0,1.5},{6,0,1.5}},.02);
  const Vec3 current{0,0,1.5};
  auto fix=repairProfileLimitedRoute(clear,path,current,0,{},0,tracker,policy,handoff);
  std::cout<<"before="<<fix.before_limit<<" after="<<fix.after_limit<<" "<<fix.reason<<"\n";
  assert(fix.applied&&fix.before_limit<.10&&fix.after_limit>.8);
  assert(fix.protected_length>=.30-1e-8&&fix.modified_length<=3.0);
  const auto old_arc=arcLengths(path),new_arc=arcLengths(fix.points);
  // The entire protected prefix is exact, including its original vertices.
  for(double s=0;s<=.30;s+=.01)
    assert(distance(atArc(path,old_arc,s),atArc(fix.points,new_arc,s))<1e-9);
  // Every original point after the local repair is still present, in order.
  size_t index=0;
  for(Vec3 p:path)if(p.x>=3.0){
    while(index<fix.points.size()&&distance(fix.points[index],p)>1e-9)++index;
    assert(index<fix.points.size());++index;
  }
  assert(distance(fix.points.back(),path.back())<1e-9);
  assert(routeGeometryValid(clear,fix.points,true));
  const auto transition=routeTransition(path,fix.points,current,policy);
  assert(transition.valid&&transition.angle<=policy.soft_angle&&transition.backtrack<=.02);
  // A healthy path gets no geometry churn; goal approach, nonfinite input,
  // disabled handoff and cancellation never create an executable replacement.
  assert(!repairProfileLimitedRoute(clear,densify({{0,0,1.5},{6,0,1.5}},.02),
    current,0,{},0,tracker,policy,handoff).applied);
  assert(!repairProfileLimitedRoute(clear,path,{5.5,0,1.5},5.5,{},0,tracker,policy,handoff).applied);
  assert(!repairProfileLimitedRoute(clear,path,current,0,{},0,tracker,policy,handoff,true,[]{return true;}).applied);
  auto disabled=handoff;disabled.enabled=false;
  assert(!repairProfileLimitedRoute(clear,path,current,0,{},0,tracker,policy,disabled).applied);
  auto invalid=path;invalid[4].x=std::numeric_limits<double>::quiet_NaN();
  assert(!repairProfileLimitedRoute(clear,invalid,current,0,{},0,tracker,policy,handoff).applied);
  // A live obstacle in the protected prefix cannot be repaired away.
  Grid blocked(gc);blocked.update({blocked.toKey({.2,0,1.5})},{});
  assert(!repairProfileLimitedRoute(blocked,path,current,0,{},0,tracker,policy,handoff).applied);
  // With only the narrow dogleg observed, smoothing must not cut into unknown
  // cells or relax the caller's unknown-space policy.
  GridConfig tight_config=gc;tight_config.resolution=.005;tight_config.inflation_xy=.001;
  Grid narrow(tight_config);KeySet known;auto narrow_path=path;
  const Vec3 offset{.002731,.002731,.002731};
  for(Vec3&p:narrow_path)p=p+offset;
  for(size_t i=1;i<narrow_path.size();++i)
    trace(narrow_path[i-1],narrow_path[i],tight_config.resolution,[&](Key k){known.insert(k);return true;});
  narrow.update({},std::move(known));
  assert(routeGeometryValid(narrow,narrow_path,false));
  auto constrained=repairProfileLimitedRoute(narrow,narrow_path,current+offset,0,{},0,tracker,policy,handoff,false);
  assert(!constrained.applied);
  // A required 90 degree corner inside the protected prefix stays a normal
  // stop/turn. A profile-limited signal alone never licenses cutting it.
  const auto turn=densify({{0,0,1.5},{.35,0,1.5},{.35,6,1.5}},.02);
  auto mandatory=repairProfileLimitedRoute(clear,turn,{.22,0,1.5},.22,{},0,
                                           tracker,policy,handoff);
  assert(!mandatory.applied&&mandatory.before_limit<.25);
  // A newly occupied voxel lies on the otherwise ideal straight replacement,
  // while the old small detour remains collision-free. It must be retained.
  GridConfig wall_config=gc;wall_config.resolution=.002;wall_config.inflation_xy=.001;
  Grid wall(wall_config);auto tiny_path=narrow_path;
  wall.update({wall.toKey({.342731,.002731,1.502731})},{});
  assert(routeGeometryValid(wall,tiny_path,true));
  auto wall_limited=repairProfileLimitedRoute(wall,tiny_path,current+offset,0,{},0,
                                             tracker,policy,handoff);
  assert(!wall_limited.applied);
  // Far progress is projected locally, so the helper neither jumps at route
  // crossings nor resurrects already-flown parts of a long route.
  for(Vec3&p:path)p.x+=8;
  path.insert(path.begin(),{0,0,1.5});
  auto late=repairProfileLimitedRoute(clear,path,{8,0,1.5},8,{},0,tracker,policy,handoff);
  assert(late.applied&&late.points.front().x>=7.95-1e-8&&late.after_limit>.8);
  std::cout<<"profile_repair_test: PASS\n";
}
