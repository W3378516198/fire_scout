#include "fire_scout/tracker.hpp"
#include "fire_scout/turn_geometry.hpp"
#include <cassert>
#include <iostream>
#include <string>
using namespace fire_scout;
int main(){
  TrackerConfig c;auto clear=[](Vec3,Vec3){return true;};
  std::vector<Vec3> circle;
  for(int i=0;i<=60;++i){double t=1.5*i/60;circle.push_back({2*std::sin(t),2*(1-std::cos(t)),1.5});}
  auto regular=assessTurnPath(circle,c.turn,c.lateral_accel,c.yaw_rate,c.sharp_turn);
  assert(regular.stops==0&&regular.max_curvature<.51);
  Tracker continuous(c);continuous.initializeYaw(0);continuous.setPath(circle,circle.front());
  Control u;for(int i=0;i<80;++i)u=continuous.update(circle.front(),{},.025,clear,0);
  assert(!u.turn_stop&&norm(u.velocity)>.2);
  for(double angle:{pi/2,pi}){
    Tracker t(c);std::vector<Vec3> route{{0,0,1.5},{1,0,1.5},{1+std::cos(angle),std::sin(angle),1.5}};
    t.initializeYaw(0);t.setPath(route,route.front());
    u=t.update(route[1],{.5,0,0},.025,clear,0);
    assert(u.turn_stop&&std::string(u.turn_phase)=="TURN_BRAKE"&&norm(u.velocity)==0&&std::abs(u.yaw)<1e-9);
    double yaw=0;int rotates=0;
    for(int i=0;i<500;++i){
      assert(!t.setPath(route,route[1])); // A path heartbeat cannot restart yaw acceleration.
      u=t.update(route[1],{},.025,clear,yaw);
      if(!u.turn_stop)break;
      assert(std::string(u.turn_phase)=="TURN_IN_PLACE"&&norm(u.velocity)==0);
      yaw=u.yaw;++rotates;
    }
    assert(rotates>5&&rotates<400&&std::abs(wrap(angle-yaw))<=c.turn.yaw_tolerance);
    for(int i=0;i<60;++i)u=t.update(route[1],{},.025,clear,yaw);
    assert(!u.turn_stop&&dot(u.velocity,route.back()-route[1])>.05);
  }
  // A new reversed two-point route has no internal curvature sample, but
  // must still brake before rotating and must not depart at the old 85deg gate.
  Tracker reversed(c);reversed.initializeYaw(0);reversed.setPath({{0,0,1.5},{-4,0,1.5}},{0,0,1.5});
  u=reversed.update({0,0,1.5},{.3,0,0},.025,clear,0,1e9,{},HeadingMode::Relaxed);
  assert(u.turn_stop&&std::string(u.turn_phase)=="TURN_BRAKE"&&norm(u.velocity)==0);
  u=reversed.update({0,0,1.5},{},.025,clear,120*pi/180,1e9,{},HeadingMode::Relaxed);
  assert(u.turn_stop&&norm(u.velocity)==0);
  auto blocked=[](Vec3,Vec3){return false;};
  u=reversed.update({0,0,1.5},{},.025,blocked,120*pi/180);
  assert(u.blocked&&norm(u.velocity)==0);
  // Global inspection protects the moving prefix and retains certified segments.
  GridConfig gc;gc.resolution=.05;gc.inflation_xy=.15;gc.inflation_z=.15;Grid free(gc);
  RouteHandoffConfig h;h.budget_ms=50;
  auto wide=prepareTurnGeometry(free,circle,c.turn,h,c.lateral_accel,c.yaw_rate,c.sharp_turn,0,.25);
  assert(wide.remaining_stops==0&&wide.points.size()==circle.size());
  std::vector<Vec3> sharp{{0,0,1.5},{2,0,1.5},{2,.1,1.5},{2.1,.1,1.5},{2.1,3,1.5}};
  auto rounded=prepareTurnGeometry(free,sharp,c.turn,h,c.lateral_accel,c.yaw_rate,c.sharp_turn,1,.25);
  assert(rounded.smoothed+rounded.polygonized>0);
  assert(distance(rounded.points.front(),sharp.front())<1e-9&&distance(rounded.points.back(),sharp.back())<1e-9);
  assert(project(rounded.points,arcLengths(rounded.points),{.8,0,1.5}).error<1e-9);
  assert(routeGeometryValid(free,rounded.points,true));
  auto cancel=prepareTurnGeometry(free,sharp,c.turn,h,c.lateral_accel,c.yaw_rate,c.sharp_turn,0,.25,true,[]{return true;});
  assert(cancel.budget_exhausted&&cancel.points.size()==sharp.size());
  std::cout<<"global_turn_policy_test: PASS continuous curve, 90/180deg brake-rotate-depart, heartbeat, hard hold, protected geometry\n";
}
