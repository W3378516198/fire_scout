#pragma once
#include "grid.hpp"
namespace fire_scout {
struct GoalApproachConfig {
  bool enabled{true};double max_offset{.35},clearance{.04};
  bool valid()const{return std::isfinite(max_offset)&&max_offset>=.05&&max_offset<=.60&&
    std::isfinite(clearance)&&clearance>=0&&clearance<=.15;}
};
struct GoalApproach {Vec3 requested{},effective{};bool valid{false},adjusted{false};};
// Adjust only an empty goal centre whose body envelope lacks clearance. The
// centreline to the requested point must remain in the same unoccupied region:
// never project a goal out of an occupied cell or through a wall. This line is
// NOT a flyable route; the planner must certify the full body to effective.
inline GoalApproach resolveGoalApproach(const Grid&g,Vec3 requested,Vec3 start,
    GoalApproachConfig cfg,bool unknown=true,const GoalApproach*retained=nullptr){
  GoalApproach out{requested,requested,false,false};
  if(!cfg.valid()||!g.insideAltitude(requested)||!finite(start))return out;
  auto same_region=[&](Vec3 p){return trace(p,requested,g.cfg.resolution,
      [&](Key k){return !g.occupied.count(k)&&(unknown||g.known(k));});};
  if(retained&&retained->valid&&retained->adjusted&&cfg.enabled&&distance(retained->requested,requested)<1e-6&&
     distance(retained->effective,requested)<=cfg.max_offset+1e-8&&
     g.segment(retained->effective,retained->effective,unknown)&&same_region(retained->effective))return *retained;
  // An unknown but geometrically clear mission remains a valid intention.
  // With allow_unknown=false the planner may return only a certified known
  // frontier; this selector must not disable that existing exploration policy.
  if(g.segment(requested,requested,true)){out.valid=true;return out;}
  if(!cfg.enabled||g.occupied.count(g.toKey(requested)))return out;
  double best=1e100;
  const int rings=std::max(1,int(std::ceil(cfg.max_offset/.025)));
  for(int i=1;i<=rings;++i){double r=cfg.max_offset*i/rings;
    for(int j=0;j<32;++j){double a=2*pi*j/32;Vec3 p=requested+Vec3{r*std::cos(a),r*std::sin(a),0};
      if(!g.segment(p,p,unknown)||g.clearanceAt(p)<cfg.clearance||!same_region(p))continue;
      double cost=r+.001*distance(start,p);
      if(cost<best){best=cost;out.effective=p;out.valid=out.adjusted=true;}
    }
    if(out.valid)break;
  }
  return out;
}
} // namespace fire_scout
