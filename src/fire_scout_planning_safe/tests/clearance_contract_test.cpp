#include "fire_scout/clearance_contract.hpp"
#include <cassert>
#include <iostream>
using namespace fire_scout;
int main(){
  GridConfig gc;gc.resolution=.1;gc.inflation_xy=.469063;gc.inflation_z=.27;gc.z_max=2;
  Grid g(gc);KeySet occupied;
  for(int x=30;x<40;++x)for(int y=6;y<14;++y)for(int z=10;z<21;++z)occupied.insert({x,y,z});
  g.update(occupied,{});
  const std::vector<Vec3> route{{0,0,1.5},{8,0,1.5}};
  assert(routeGeometryValid(g,route)); // Previous lock-valid gate would retain it.
  assert(!routeHasMargin(g,route,.25));
  assert(routeHasMargin(g,route,.12));
  // A collision halfway along a long edge cannot escape the reserve audit.
  assert(g.segmentWithMargin(route.front(),route.front(),.25));
  assert(g.segmentWithMargin(route.back(),route.back(),.25));
  assert(!g.segmentWithMargin(route.front(),route.back(),.25));
  assert(!g.segmentWithMargin(route.front(),route.back(),-.01));
  ClearanceReview review;ClearanceContract c;
  assert(!review.due(g,route,0,10,1,c));
  assert(!review.due(g,route,0,10.4,1,c)); // Duplicate snapshot is not evidence.
  assert(review.due(g,route,0,10.8,2,c));
  assert(!review.due(g,route,0,11.2,3,c)); // Retry budget is retained.
  assert(!review.due(g,route,0,13,4,c));
  assert(review.due(g,route,0,13.4,5,c));
  PlannerConfig pc;pc.repair_search_ms=200;pc.clearance_weight=7;
  auto patched=repairRouteClearance(g,route,0,1,c,pc);
  assert(!patched.points.empty()&&patched.locally_patched);
  assert(patched.reason=="CLEARANCE_LOCAL_REPAIR");
  assert(routeHasMargin(g,patched.points,.245));
  assert(distance(patched.points.back(),route.back())<1e-9);
  const auto a=arcLengths(patched.points);
  for(double s=0;s<=1;s+=.025)assert(distance(atArc(patched.points,a,s),Vec3{s,0,1.5})<1e-9);
  assert(g.cfg.inflation_xy==gc.inflation_xy); // Search-only inflation never leaks.
  auto cancelled=repairRouteClearance(g,route,0,1,c,pc,[]{return true;});
  assert(cancelled.points.empty());
  // A continuous narrow corridor cannot magically acquire an extra 25 cm.
  KeySet walls;for(int x=-10;x<100;++x)for(int z=10;z<21;++z){walls.insert({x,6,z});walls.insert({x,-7,z});}
  Grid narrow(gc);narrow.update(walls,{});
  assert(routeGeometryValid(narrow,route));
  auto impossible=repairRouteClearance(narrow,route,0,1,c,pc);
  assert(impossible.points.empty());assert(routeGeometryValid(narrow,route));
  Grid empty(gc);empty.update({},{});review.clear();
  for(int n=0;n<30;++n)assert(!review.due(empty,route,0,n*.4,n,c));
  std::cout<<"clearance contract: continuous sweep, distinct maps, cooldown, local detour, exact prefix/tail, narrow fallback passed\n";
}
