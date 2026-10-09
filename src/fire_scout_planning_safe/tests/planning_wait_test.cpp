#include "fire_scout/planning_wait.hpp"
#include "fire_scout/execution_permit.hpp"
#include "fire_scout/route_memory.hpp"
#include "fire_scout/autonomous_recovery.hpp"
#include "fire_scout/navigation_contract.hpp"
#include <cassert>
#include <iostream>
using namespace fire_scout;
int main(){
  PlanningWait wait;
  assert(wait.deferRecovery(true,true,true,false,10,100));
  // Repeated searches/route revisions cannot make a close-obstacle grace last
  // forever; a comfortable stationary anchor doesn't need sideways escape.
  assert(wait.deferRecovery(true,true,true,false,30,120));assert(wait.seconds==20);
  assert(!wait.deferRecovery(true,true,false,false,30,120));
  assert(wait.reason=="CLEARANCE_ESCAPE");
  assert(!wait.deferRecovery(true,true,true,true,31,121));
  assert(wait.seconds==0); // explicit tracking/peer stall bypasses the wait
  assert(wait.deferRecovery(true,true,false,false,40,130));
  assert(!wait.deferRecovery(true,true,false,false,40.1,136.1)); // slow /clock
  assert(wait.deferRecovery(true,true,false,false,1,140)); // rewind
  assert(!wait.deferRecovery(false,true,true,false,2,141));

  RoutePermit advisory{int64_t(1e9),int64_t(2e9),false,"NO_ROUTE",{10,0,3},9,true};
  RoutePermit decoded;assert(RoutePermit::decode(advisory.encode(),decoded)&&decoded.searching);
  RouteExecutionGate gate;assert(gate.observe(advisory.encode(),1.,1.));
  assert(!gate.allowed(int64_t(2e9),1.,1.,.8));
  advisory.searching=false;assert(RoutePermit::decode(advisory.encode(),decoded)&&!decoded.searching);
  assert(!RoutePermit::decode(advisory.encode()+" searching=1 searching=1",decoded));

  GridConfig gc;gc.resolution=.18;gc.inflation_xy=.469063;gc.inflation_z=.272;
  gc.z_min=.65;gc.z_max=6;Grid map(gc);
  map.update({map.toKey({5,0,3})},{});
  RouteMemory route;route.set(densify({{0,0,3},{10,0,3}},.05));
  auto hard=route.safePrefix(map,{0,0,3});auto comfortable=route.safePrefix(map,{0,0,3},true,.25);
  assert(!hard.empty()&&!comfortable.empty()&&comfortable.back().x<hard.back().x-.15);
  assert(map.clearanceAt(comfortable.back())>=.25&&routeGeometryValid(map,comfortable));

  Vec3 join{};std::vector<Vec3> fresh{{0,0,3},{2,0,3}};Vec3 actual{0,.55,3};
  auto physical=[&](Vec3 a,Vec3 b){return map.segment(a,b);};
  assert(!certifiedRouteHandoff(fresh,actual,physical,join));
  assert(certifiedRouteHandoff(fresh,actual,physical,join,0.,.80));
  auto join_obstacles=map.occupied;join_obstacles.insert(map.toKey({.2,.3,3}));map.update(std::move(join_obstacles),{});
  assert(!certifiedRouteHandoff(fresh,actual,physical,join,0.,.80));

  // No-route escape increases room instead of sliding toward the distant goal
  // along a wall. In an empty room it yields no needless escape at all.
  GridConfig ec;ec.resolution=.10;ec.inflation_xy=.30;ec.inflation_z=.15;
  ec.z_min=.5;ec.z_max=2;Grid wall(ec);KeySet occupied;
  for(int y=-100;y<=100;++y)occupied.insert(wall.toKey({1.,(y+.5)*.1,1.}));
  wall.update(std::move(occupied),{});Vec3 p{.2,0,1};
  ObservedEscapeConfig escape;escape.prefer_clearance=true;escape.max_height_change=0;
  auto away=observedEscapeRoute(p,{10,4,1},escape,
    [&](Vec3 a,Vec3 b){return wall.segment(a,b);},[&](Vec3 q){return wall.clearanceAt(q);},[](Vec3){return true;});
  assert(!away.empty()&&wall.clearanceAt(away.back())>=wall.clearanceAt(p)+.10-1e-9);
  for(size_t i=1;i<away.size();++i)assert(wall.segment(away[i-1],away[i]));
  Grid empty(ec);empty.update({},{});
  auto needless=observedEscapeRoute(p,{10,4,1},escape,[](Vec3,Vec3){return true;},
    [&](Vec3 q){return empty.clearanceAt(q);},[](Vec3){return true;});
  assert(needless.empty());
  std::cout<<"planning_wait_test: PASS advisory permit, bounded near-wall grace, safe terminal reserve, clearance-first escape\n";
}
