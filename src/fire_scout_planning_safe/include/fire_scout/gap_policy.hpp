#pragma once
#include "grid.hpp"

namespace fire_scout {
inline bool snapshotContainsSweep(Vec3 anchor,double half_extent,double radius,double resolution,Vec3 a,Vec3 b){
  const double inner=half_extent-radius-resolution;
  if(!finite(anchor)||!finite(a)||!finite(b)||!std::isfinite(inner)||inner<=0)return false;
  return std::max(std::abs(a.x-anchor.x),std::abs(b.x-anchor.x))<inner &&
         std::max(std::abs(a.y-anchor.y),std::abs(b.y-anchor.y))<inner;
}
// Hard body envelope is NEVER changed by this policy. The gap describes
// lateral room for that complete cylinder, not a centerline ray opening.
struct GapPolicyConfig {
  bool enabled{true};
  double margin_weight{1.0},speed_weight{1.0},scale{.60};
  double max_extra_margin{.45},probe_distance{1.0},min_speed{.20};
  bool valid()const{
    for(double x:{margin_weight,speed_weight,scale,max_extra_margin,probe_distance,min_speed})
      if(!std::isfinite(x)||x<0)return false;
    return scale>=.05 && probe_distance>=.1 && probe_distance<=3 &&
      margin_weight<=10 && speed_weight<=10 && min_speed>0 && max_extra_margin<=1.5;
  }
};
struct GapState {
  double width{0},left{0},right{0},extra_margin{0},speed{0};
};
inline GapState gapFromRoom(double hard_radius,double left,double right,
                            double maximum_speed,const GapPolicyConfig&c){
  GapState out;
  if(!c.valid() || !std::isfinite(hard_radius)||hard_radius<=0 ||
     !std::isfinite(left)||!std::isfinite(right)||left<0||right<0||
     !std::isfinite(maximum_speed)||maximum_speed<=0)return out;
  out.left=left;out.right=right;
  const double spare=left+right;
  out.width=2*hard_radius+spare;
  // Soft extra margin rises smoothly with AVAILABLE room. An inverse-gap hard
  // inflation would enlarge the vehicle most exactly where it cannot fit.
  out.extra_margin=std::min(.45*spare,c.max_extra_margin*
    (1-std::exp(-c.margin_weight*spare/c.scale)));
  const double low=std::min(c.min_speed,maximum_speed);
  out.speed=low+(maximum_speed-low)*(1-std::exp(-c.speed_weight*spare/c.scale));
  // A wide passage is not permission to fly fast along one of its walls.
  const double centering=out.extra_margin>1e-6?
    std::clamp(std::min(left,right)/out.extra_margin,0.,1.):1.;
  out.speed=std::min(out.speed,low+(maximum_speed-low)*centering);
  if(!c.enabled){out.extra_margin=0;out.speed=maximum_speed;}
  return out;
}
inline GapState localGap(const Grid&g,Vec3 p,Vec3 tangent,double maximum_speed,
                          const GapPolicyConfig&c={}){
  const double xy=std::hypot(tangent.x,tangent.y);
  if(!c.valid()||!finite(tangent)||xy<1e-8||!g.segment(p,p))return {};
  Vec3 side{-tangent.y/xy,tangent.x/xy,0};
  auto room=[&](double sign){
    double lo=0,hi=c.probe_distance;
    if(g.segment(p,p+side*(hi*sign)))return hi;
    for(int i=0;i<9;++i){double mid=(lo+hi)*.5;
      if(g.segment(p,p+side*(mid*sign)))lo=mid;else hi=mid;}
    return lo;
  };
  return gapFromRoom(g.cfg.inflation_xy,room(1),room(-1),maximum_speed,c);
}
} // namespace fire_scout
