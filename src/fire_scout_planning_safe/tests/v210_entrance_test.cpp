#include "fire_scout/entrance_routes.hpp"
#include "fire_scout/bspline.hpp"
#include <cassert>
#include <iostream>
using namespace fire_scout;
Grid wallMap(bool three){
  GridConfig c;c.resolution=.20;c.inflation_xy=.25;c.inflation_z=.15;
  c.z_min=.6;c.z_max=2.5;Grid g(c);KeySet occ;
  for(int y=-150;y<=150;++y){double v=(y+.5)*c.resolution;
    if(std::abs(v)<.9 || (three&&(std::abs(v-3)<.9||std::abs(v+3)<.9)))continue;
    for(int z=0;z<=15;++z)occ.insert({25,y,z});
  }g.update(std::move(occ),{});return g;
}
bool same(const std::vector<Vec3>&a,const std::vector<Vec3>&b){
  if(a.size()!=b.size())return false;
  for(size_t i=0;i<a.size();++i)if(distance(a[i],b[i])>1e-10)return false;
  return true;
}
int main(){
  auto g=wallMap(true);const auto original=g.occupied;
  PlannerConfig pc;pc.allow_unknown=true;pc.cruise_altitude=1.5;
  pc.max_search_ms=600;pc.max_retry_search_ms=600;pc.search_margin=6;
  pc.max_expansions=180000;pc.multi_altitude_slices=false;pc.enable_3d_search=false;
  pc.max_plan_distance=40;pc.reverse_weight=0;
  EntranceConfig cfg;Vec3 start{0,0,1.5},goal{10,0,1.5};Planner p(pc);
  const auto primary=p.plan(g,start,goal);assert(primary.points.size()>1);
  auto second=selectEntranceRoute(g,start,primary,pc,cfg,{primary.points});
  assert(second.changed && second.shared==0 && second.evaluated<=size_t(cfg.count));
  auto third=selectEntranceRoute(g,start,primary,pc,cfg,{primary.points,second.route.points});
  assert(third.changed && third.shared==0);
  assert(g.occupied==original); // Virtual cuts do not edit the actual map.
  auto gates=peerPassages(g,{primary.points,second.route.points},cfg.topology);
  assert(gates.size()>=2 && sharedPassageCount(third.route.points,gates)==0);
  for(const auto &r:{primary.points,second.route.points,third.route.points})
    for(size_t i=1;i<r.size();++i)assert(g.segment(r[i-1],r[i],true));
  auto retained=selectEntranceRoute(g,start,second.route,pc,cfg,{primary.points});
  assert(!retained.changed && same(retained.route.points,second.route.points));
  auto one=wallMap(false);Planner q(pc);auto single=q.plan(one,start,goal);
  auto fallback=selectEntranceRoute(one,start,single,pc,cfg,{single.points});
  assert(!fallback.changed && same(fallback.route.points,single.points));
  Grid empty(g.cfg);PlanResult straight;straight.points={start,goal};
  auto no_detour=selectEntranceRoute(empty,start,straight,pc,cfg,{{start,goal}});
  assert(!no_detour.changed && same(no_detour.route.points,straight.points));
  cfg.count=1;
  assert(same(selectEntranceRoute(g,start,primary,pc,cfg,{primary.points}).route.points,primary.points));
  cfg.count=3;cfg.topology.max_detour_ratio=1.01;cfg.topology.max_extra_length=.05;
  auto limited=selectEntranceRoute(g,start,primary,pc,cfg,{primary.points});
  assert(!limited.changed && same(limited.route.points,primary.points));
  // A planning horizon remains the endpoint even when the mission is far away.
  assert(distance(second.route.points.back(),primary.points.back())<.2);
  cfg=EntranceConfig{};
  auto near=reservablePassages(gates,gates.front().center,goal,cfg.topology);
  assert(near.size()<gates.size()); // No late reversal inside a committed doorway.
  std::cout<<"v210_entrance_test: PASS three doors, unchanged corridor, one-door fallback, budget/count, detour bound, committed approach\n";
}
