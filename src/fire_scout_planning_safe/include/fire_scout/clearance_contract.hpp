#pragma once
#include "planner.hpp"
#include <cstdint>

namespace fire_scout {
struct ClearanceContract {
  bool enabled{true};
  double reserve{.25}, horizon{6.0}, review_period{.30}, retry_period{2.0};
  bool valid()const {
    return std::isfinite(reserve)&&reserve>=.05&&reserve<=1.0&&
      std::isfinite(horizon)&&horizon>=2&&horizon<=12&&
      std::isfinite(review_period)&&review_period>=.1&&review_period<=2&&
      std::isfinite(retry_period)&&retry_period>=.1&&retry_period<=10;
  }
};

inline bool routeHasMargin(const Grid &g,const std::vector<Vec3>& route,
                           double margin,bool unknown=true) {
  if(route.size()<2)return false;
  for(size_t i=1;i<route.size();++i)
    if(!g.segmentWithMargin(route[i-1],route[i],margin,unknown))return false;
  return true;
}

// Largest continuously certified reserve, not a waypoint-only distance.
inline double routeMargin(const Grid &g,const std::vector<Vec3>& route,
                          double ceiling,bool unknown=true) {
  if(!routeHasMargin(g,route,0,unknown))return 0;
  if(routeHasMargin(g,route,ceiling,unknown))return ceiling;
  double lo=0,hi=ceiling;
  for(int n=0;n<7;++n){const double m=(lo+hi)/2;
    if(routeHasMargin(g,route,m,unknown))lo=m;else hi=m;}
  return lo;
}

class ClearanceReview {
 public:
  void clear(){*this=ClearanceReview{};}
  bool due(const Grid& g,const std::vector<Vec3>& route,double progress,
           double time,uint64_t generation,const ClearanceContract& c,bool unknown=true) {
    if(!c.enabled||!c.valid()||route.size()<2)return false;
    if(time<last_time_)clear();
    if(time-last_time_<c.review_period||time<retry_after_)return false;
    last_time_=time;
    const auto ahead=routeSlice(route,progress,progress+c.horizon);
    if(ahead.size()<2||routeHasMargin(g,ahead,c.reserve,unknown)){
      evidence_=0;last_generation_=generation;return false;
    }
    // Distinct installed maps, not repeated timer ticks on the same snapshot.
    if(evidence_==0||generation!=last_generation_)++evidence_;
    last_generation_=generation;
    if(evidence_<2)return false;
    evidence_=0;retry_after_=time+c.retry_period;return true;
  }
 private:
  double last_time_{-1e30},retry_after_{-1e30};
  uint64_t last_generation_{0};unsigned evidence_{0};
};

// A bounded local replacement: preserve the original forward prefix and tail,
// search only around the first low-clearance interval. If the actual doorway
// cannot supply this reserve, leave the existing hard-valid route available.
inline PlanResult repairDistantRouteClearance(const Grid& g,const std::vector<Vec3>& route,
    double progress,double protected_length,const ClearanceContract& c,
    PlannerConfig pc,const std::function<bool()>& cancelled={}) {
  PlanResult out;out.reason="CLEARANCE_NO_LOCAL_ALTERNATIVE";
  if(!c.enabled||!c.valid()||route.size()<2||(cancelled&&cancelled()))return out;
  const auto arc=arcLengths(route);const double length=arc.back();
  const double begin=std::clamp(progress,0.,length),end=std::min(length,begin+c.horizon);
  double first=-1,last=-1;
  // Split long edges for locating the interval, but certify every subsegment.
  for(double s=begin;s<end-1e-6;s+=.20){
    if(cancelled&&cancelled())return out;
    const double next=std::min(end,s+.20);
    const bool clear=g.segmentWithMargin(atArc(route,arc,s),atArc(route,arc,next),c.reserve,pc.allow_unknown);
    if(!clear){if(first<0)first=s;last=next;}
    else if(first>=0)break;
  }
  if(first<0){out.reason="CLEARANCE_ALREADY_RECOVERED";return out;}
  const double anchor=std::max(begin+protected_length,first-.80);
  const double rejoin=std::min(length,last+.80);
  if(anchor>=first-.05||rejoin-anchor>pc.repair_max_span||rejoin<=last+.05)return out;
  const Vec3 a=atArc(route,arc,anchor),b=atArc(route,arc,rejoin);
  if(!g.segmentWithMargin(a,a,c.reserve,pc.allow_unknown)||
     !g.segmentWithMargin(b,b,c.reserve,pc.allow_unknown))return out;
  auto expanded=g.cfg;expanded.inflation_xy+=c.reserve;
  // This temporary search map is worker-owned. Physical/recovery radii of the
  // live map remain unchanged. Endpoints must already meet the full reserve.
  expanded.recovery_xy=0;
  Grid search(expanded);search.update(g.occupied,g.free);
  pc.local_repair_enabled=false;pc.lock_valid_route=false;
  pc.max_search_ms=pc.max_retry_search_ms=pc.repair_search_ms;
  pc.search_margin=pc.max_search_margin=pc.repair_search_margin;
  pc.max_plan_distance=pc.repair_max_span*2;
  Planner planner(pc);planner.setCancellation(cancelled);
  auto patch=planner.plan(search,a,b,forwardRouteTangent(route,arc,anchor));
  out=patch;out.points.clear();out.reason="CLEARANCE_SEARCH_"+patch.reason;
  if(patch.points.size()<2||distance(patch.points.back(),b)>.05||
     !routeHasMargin(g,patch.points,c.reserve,pc.allow_unknown)||
     pathLength(patch.points)>(rejoin-anchor)*pc.repair_max_detour_ratio)return out;
  LocalRepairWindow w;w.valid=true;
  w.prefix=routeSlice(route,std::max(0.,begin-.25),anchor);
  w.tail=routeSlice(route,rejoin,length);
  out=patch;out.points=spliceLocalRepair(w,patch.points,&out.patch_begin,&out.patch_end);
  out.locally_patched=true;out.reused=false;out.reason="CLEARANCE_LOCAL_REPAIR";
  if(!routeGeometryValid(g,out.points,pc.allow_unknown))out.points.clear();
  return out;
}
// Soft ranking only. Every accepted candidate also passes the continuous
// hard sweep; these samples never serve as a collision certificate.
inline double clearanceDeficit(const Grid& g,const std::vector<Vec3>& route,
    double horizon,double reserve) {
  if(route.size()<2)return 0;
  const auto arc=arcLengths(route);const double end=std::min(horizon,arc.back());
  double loss=0;
  for(double s=0;s<end;s+=.15){const double ds=std::min(.15,end-s);
    loss+=std::max(0.,reserve-g.clearanceAt(atArc(route,arc,s+.5*ds),reserve))*ds;}
  return loss;
}

inline PlanResult repairNearRouteClearance(const Grid& g,const std::vector<Vec3>& route,
    double progress,double protect,const ClearanceContract& c,const PlannerConfig& pc,
    const std::function<bool()>& cancelled={}) {
  PlanResult out;out.reason="CLEARANCE_NEAR_NO_CERTIFIED_OFFSET";
  if(route.size()<2||!c.valid())return out;
  const auto started=std::chrono::steady_clock::now();
  const auto arc=arcLengths(route);const double length=arc.back();
  const double begin=std::clamp(progress,0.,length);
  const double anchor=std::min(length,begin+std::max(.30,protect));
  const double rejoin=std::min(length,begin+std::min(c.horizon,pc.repair_max_span));
  if(rejoin-anchor<1.2)return out;
  const double ramp=std::min(1.2,.4*(rejoin-anchor));
  const Vec3 tangent=forwardRouteTangent(route,arc,anchor);
  const double xy=std::hypot(tangent.x,tangent.y);
  if(xy<.3)return out;
  const Vec3 normal{-tangent.y/xy,tangent.x/xy,0};
  const auto reference=routeSlice(route,begin,rejoin);
  const double before=clearanceDeficit(g,reference,c.horizon,c.reserve);
  double best=before;out.clearance_deficit_before=before;
  auto ease=[](double t){t=std::clamp(t,0.,1.);return t*t*t*(10+t*(-15+6*t));};
  for(double offset:{.15,-.15,.25,-.25,.40,-.40,.60,-.60}){
    if((cancelled&&cancelled())||std::chrono::duration<double,std::milli>(
        std::chrono::steady_clock::now()-started).count()>pc.repair_search_ms)break;
    auto candidate=routeSlice(route,std::max(0.,begin-.25),anchor);
    const size_t patch_begin=candidate.size()-1;bool safe=true;
    for(double s=anchor+.05;;s+=.05){
      s=std::min(s,rejoin);const Vec3 base=atArc(route,arc,s);
      const double weight=ease((s-anchor)/ramp)*ease((rejoin-s)/ramp);
      const Vec3 point=base+normal*(offset*weight);
      if((cancelled&&cancelled())||!g.segment(candidate.back(),point,pc.allow_unknown)||
         g.clearanceAt(point,c.reserve)+.005<g.clearanceAt(base,c.reserve)){
        safe=false;break;
      }
      candidate.push_back(point);if(s>=rejoin-1e-8)break;
    }
    if(!safe)continue;
    const size_t patch_end=candidate.size()-1;
    const auto tail=routeSlice(route,rejoin,length);
    for(size_t i=1;i<tail.size();++i)candidate.push_back(tail[i]);
    if(!routeGeometryValid(g,candidate,pc.allow_unknown))continue;
    const double overlap=begin-std::max(0.,begin-.25);
    const auto ahead=routeSlice(candidate,overlap,pathLength(candidate));
    const double loss=clearanceDeficit(g,ahead,c.horizon,c.reserve);
    if(loss>=best-std::max(.002,.10*before))continue;
    best=loss;out.points=std::move(candidate);out.locally_patched=true;
    out.clearance_near_repair=true;out.patch_begin=patch_begin;out.patch_end=patch_end;
    out.clearance_deficit_after=loss;out.reason="NEAR_CLEARANCE_OFFSET";
  }
  out.elapsed_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();
  return out;
}

inline PlanResult repairRouteClearance(const Grid& g,const std::vector<Vec3>& route,
    double progress,double protect,const ClearanceContract& c,PlannerConfig pc,
    const std::function<bool()>& cancelled={}) {
  auto out=repairDistantRouteClearance(g,route,progress,protect,c,pc,cancelled);
  if(!out.points.empty()||!c.enabled||!c.valid()||(cancelled&&cancelled()))return out;
  // A full-clearance start/rejoin may not exist once already beside a wall.
  // Keep the hard envelope and bend a bounded part of the actual old curve.
  pc.repair_search_ms=std::max(0.,pc.repair_search_ms-out.elapsed_ms);
  auto near=repairNearRouteClearance(g,route,progress,protect,c,pc,cancelled);
  near.elapsed_ms+=out.elapsed_ms;near.expansions+=out.expansions;
  near.flat_expansions+=out.flat_expansions;near.spatial_expansions+=out.spatial_expansions;
  if(near.points.empty())near.reason=out.reason+"_"+near.reason;
  return near;
}
} // namespace fire_scout
