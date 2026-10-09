#include "fire_scout/planner.hpp"
#include <cassert>
#include <iostream>
using namespace fire_scout;

Grid barrierMap(bool door=false) {
  GridConfig c;c.resolution=.18;c.inflation_xy=.469063;c.inflation_z=.272;
  c.preferred_clearance=.75;c.z_min=.65;c.z_max=6;
  Grid g(c);KeySet wall,free;
  for(int y=-56;y<=56;++y){
    if(door&&std::abs((y+.5)*c.resolution)<1.2)continue;
    wall.insert(g.toKey({5.,(y+.5)*c.resolution,3.}));
  }
  for(int x=-20;x<=235;++x)for(int y=-80;y<=80;++y)free.insert({x,y,16});
  g.update(std::move(wall),std::move(free));return g;
}
PlannerConfig config() {
  PlannerConfig c;c.cruise_altitude=3;c.enable_3d_search=false;c.multi_altitude_slices=false;
  c.local_repair_enabled=false;c.clearance_weight=7;c.max_plan_distance=48;
  c.search_margin=c.max_search_margin=12;c.max_expansions=350;
  c.max_search_ms=c.max_retry_search_ms=500;c.coarse_detour_ms=65;
  return c;
}
std::pair<PlanResult,int> finish(Planner&p,const Grid&g,Vec3 a,Vec3 b,int limit=1000) {
  int work=0;PlanResult r;
  for(int n=0;n<limit;++n){r=p.plan(g,a,b);work+=r.expansions;
    if(!r.points.empty())return {r,work};}
  return {r,work};
}
int main(){
  auto g=barrierMap();auto c=config();Vec3 start{0,0,3},goal{40,0,3};
  Planner guided(c);auto fast=finish(guided,g,start,goal);
  assert(!fast.first.points.empty()&&fast.first.search_mode=="COARSE_DETOUR");
  assert(routeGeometryValid(g,fast.first.points));
  c.coarse_detour_enabled=false;Planner fine(c);auto slow=finish(fine,g,start,goal);
  assert(!slow.first.points.empty()&&routeGeometryValid(g,slow.first.points));
  assert(fast.second<slow.second);
  std::cout<<"Long-wall expansions: guided="<<fast.second<<" fine="<<slow.second<<"\n";

  // The new guide must retain the parent's transient peer-route soft cost.
  // Penalizing the originally chosen wall end should select the other end.
  Planner peer_aware(config());
  peer_aware.addRoutePenalty(g,{fast.first.points},1.2,20.,.4);
  auto alternative=finish(peer_aware,g,start,goal);
  assert(!alternative.first.points.empty()&&routeGeometryValid(g,alternative.first.points));
  auto end_side=[](const std::vector<Vec3>&p){double side=0;
    for(Vec3 q:p){if(std::abs(q.y)>std::abs(side))side=q.y;}
    return side;};
  assert(end_side(fast.first.points)*end_side(alternative.first.points)<0);

  // Change an explored local branch while the fine frontier is unfinished.
  c.max_expansions=100;Planner retained(c);
  for(int n=0;n<8;++n)assert(retained.plan(g,start,goal).points.empty());
  const size_t before=retained.retainedSearchNodes();assert(before>100);
  auto changed=g;auto occupied=g.occupied;occupied.insert(g.toKey({2.,.7,3.}));
  changed.update(std::move(occupied),g.free);
  assert(retained.refreshChangedSearches(changed)>0);
  assert(retained.retainedSearchNodes()==before); // schedule a repair, no reset
  auto repaired=finish(retained,changed,start,goal,2000);
  assert(!repaired.first.points.empty()&&routeGeometryValid(changed,repaired.first.points));
  assert(repaired.first.search_repair_events>0&&repaired.first.search_repair_removed>0);
  assert(repaired.first.search_repair_preserved>50);

  // Reopen a previously disconnected slice when a real opening appears.
  auto closed=barrierMap();c.search_margin=c.max_search_margin=2;
  c.max_expansions=250;Planner opening(c);Vec3 nearby{10,0,3};
  PlanResult no_path;
  for(int n=0;n<200;++n){no_path=opening.plan(closed,start,nearby);
    if(no_path.reason=="NO_PATH_RETRY")break;}
  assert(no_path.points.empty()&&no_path.reason=="NO_PATH_RETRY");
  const size_t old_nodes=opening.retainedSearchNodes();assert(old_nodes>100);
  auto opened=barrierMap(true);assert(opening.refreshChangedSearches(opened)>0);
  assert(opening.retainedSearchNodes()==old_nodes);
  auto through=finish(opening,opened,start,nearby);
  assert(!through.first.points.empty()&&routeGeometryValid(opened,through.first.points));
  assert(through.first.search_repair_preserved>0);

  // A sparse lattice can miss a narrow off-lattice entrance. Fine search is
  // still serviced every call and must find it with the SAME physical radius.
  Grid narrow(g.cfg);KeySet partition;
  for(int y=-80;y<=80;++y){const double py=(y+.5)*.18;
    if(py>.72&&py<1.98)continue;
    partition.insert(narrow.toKey({5.,py,3.}));}
  narrow.update(std::move(partition),g.free);
  auto fallback_config=config();fallback_config.coarse_detour_resolution=.90;
  fallback_config.search_margin=fallback_config.max_search_margin=3;
  Planner fallback(fallback_config);auto passed=finish(fallback,narrow,start,nearby);
  assert(!passed.first.points.empty()&&routeGeometryValid(narrow,passed.first.points));
  assert(passed.first.search_mode=="FLAT");
  assert(narrow.cfg.inflation_xy==g.cfg.inflation_xy);
  std::cout<<"incremental_detour_test: PASS retained changed branches, reopened doorway and fine fallback\n";
}
