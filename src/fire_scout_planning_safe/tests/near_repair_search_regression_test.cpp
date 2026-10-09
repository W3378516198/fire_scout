#include "fire_scout/clearance_contract.hpp"
#include <cassert>
#include <iostream>
using namespace fire_scout;
int main(){
 GridConfig gc;gc.resolution=.1;gc.inflation_xy=.469063;gc.inflation_z=.27;gc.z_max=2;
 Grid g(gc);KeySet wall;
 for(int x=-10;x<40;++x)for(int z=10;z<21;++z)wall.insert({x,6,z});
 g.update(wall,{});
 const auto route=densify({{0,0,1.5},{8,0,1.5}},.05);ClearanceContract c;PlannerConfig pc;
 pc.repair_search_ms=140;pc.clearance_weight=7;
 auto d=repairRouteClearance(g,route,0,.30,c,pc);
 assert(d.clearance_near_repair&&d.locally_patched&&!d.points.empty());
 assert(routeGeometryValid(g,d.points));
 assert(clearanceDeficit(g,d.points,6,.25)<.8*clearanceDeficit(g,route,6,.25));
 const auto arc=arcLengths(d.points);
 for(double s=0;s<=.30;s+=.01)assert(distance(atArc(d.points,arc,s),Vec3{s,0,1.5})<1e-8);
 assert(distance(d.points.back(),route.back())<1e-8);
 // First local obstruction can be repaired without searching around a remote
 // one. Only the certified tail before that second obstacle may be published.
 GridConfig sc;sc.resolution=.1;sc.inflation_xy=.15;sc.inflation_z=.15;
 sc.z_min=1.49;sc.z_max=1.51;Grid search(sc);
 PlannerConfig tiny;tiny.enable_3d_search=false;tiny.multi_altitude_slices=false;
 tiny.cruise_altitude=1.5;tiny.max_plan_distance=20;tiny.max_expansions=8;
 tiny.max_search_ms=tiny.max_retry_search_ms=tiny.repair_search_ms=1000;
 tiny.repair_search_margin=1.0;tiny.repair_max_span=4.;
 Planner planner(tiny);const std::vector<Vec3>mission{{0,0,1.5},{10,0,1.5}};
 // Prime the stable mission goal and exact committed route.
 planner.plan(search,mission.front(),mission.back());planner.setCommittedRoute(densify(mission,.1));
 KeySet blocks;for(int x:{20,70})for(int y=-4;y<=4;++y)blocks.insert({x,y,15});
 search.update(blocks,{});bool resumed=false;size_t retained=0;PlanResult result;
 for(int n=0;n<500;++n){
  planner.requestRepair();result=planner.plan(search,mission.front(),mission.back());
  resumed=resumed||result.patch_search_resumed;retained=std::max(retained,result.retained_search_nodes);
  if(!result.points.empty())break;
 }
 assert(resumed&&retained>8&&result.locally_patched&&!result.points.empty());
 assert(result.points.back().x<7&&result.points.back().x>3);
 assert(routeGeometryValid(search,result.points));
 std::cout<<"Near-wall offset, protected prefix, resumed local frontier and certified temporary tail passed\n";
}
