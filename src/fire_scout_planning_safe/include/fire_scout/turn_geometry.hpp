#pragma once
#include "turn_policy.hpp"
#include "route_handoff.hpp"

namespace fire_scout {
inline std::vector<Vec3> refreshLocalShortcut(const Grid&g,const std::vector<Vec3>&path,
    Vec3 current,double progress,double keep,double reserve,bool unknown,
    const std::function<bool()>&cancelled={}){
  if(path.size()<3)return {};
  const auto arc=arcLengths(path);const auto pr=project(path,arc,current,
    std::max(0.,progress-.10),std::min(arc.back(),progress+2.));
  if(!std::isfinite(pr.error)||pr.error>.30)return {};
  const double at=std::max(progress,pr.s),begin=std::max(0.,at-.05);
  const double lo=std::min(arc.back(),at+std::max(.30,keep));
  const auto began=std::chrono::steady_clock::now();
  for(double span:{5.0,3.0,1.5}){
    if((cancelled&&cancelled())||std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count()>5.)break;
    const double hi=std::min(arc.back(),lo+span);const Vec3 a=atArc(path,arc,lo),b=atArc(path,arc,hi);
    if(hi-lo-distance(a,b)<.12||!g.segmentWithMargin(a,b,reserve,unknown))continue;
    const auto reference=routeSlice(path,lo,hi);const auto ra=arcLengths(reference);
    bool nearby=true;for(double u=0;u<=1.;u+=.05)
      if(project(reference,ra,a+(b-a)*u).error>.60){nearby=false;break;}
    if(!nearby)continue;
    auto candidate=routeSlice(path,begin,lo);appendRoute(candidate,{a,b});
    appendRoute(candidate,routeSlice(path,hi,arc.back()));return candidate;
  }
  return {};
}
struct TurnGeometryResult {
  std::vector<Vec3> points;size_t smoothed{0},polygonized{0},remaining_stops{0},remaining_repair_samples{0};
  double max_curvature{0};bool budget_exhausted{false};
};
// Full-route inspection, bounded edits. A smooth replacement must be entirely
// collision certified and satisfy the same policy including its two seams.
// Otherwise a certified polygon retains a real corner for stop/rotate/depart.
inline TurnGeometryResult prepareTurnGeometry(const Grid&g,const std::vector<Vec3>&path,
    const TurnPolicyConfig&policy,const RouteHandoffConfig&handoff,double lateral_accel,
    double yaw_rate,double sharp,double protected_arc,double reserve,bool unknown=true,
    const std::function<bool()>&cancelled={}){
  TurnGeometryResult out;out.points=path;
  if(!policy.enabled||!policy.valid()||!handoff.valid()||path.size()<3)return out;
  const auto began=std::chrono::steady_clock::now();
  const auto stop=[&]{return (cancelled&&cancelled())||
    std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count()>=policy.repair_budget_ms;};
  double cursor=std::max(0.,protected_arc);size_t trials=0;
  while(!stop()&&trials++<16){
    const auto arc=arcLengths(out.points);size_t first=out.points.size(),last=first,peak=first;
    double severity=-1;
    for(size_t i=1;i+1<out.points.size();++i){
      if(arc[i]<cursor+.02)continue;
      const auto s=turnSample(out.points[i]-out.points[i-1],out.points[i+1]-out.points[i],lateral_accel,yaw_rate);
      if(!requiresTurnRepair(s,policy,sharp))continue;
      if(first==out.points.size()){first=last=peak=i;severity=s.curvature;}
      else if(arc[i]-arc[last]>.25||arc[i]-arc[first]>1.2)break;
      else {last=i;if(s.curvature>severity){peak=i;severity=s.curvature;}}
    }
    if(first==out.points.size())break;
    bool replaced=false;
    for(double half:{.45,.75,1.10}){
      if(stop())break;
      const double lo=std::max(cursor,arc[first]-half),hi=std::min(arc.back(),arc[last]+half);
      if(lo>=arc[first]-.025||hi<=arc[last]+.025)continue;
      auto reference=routeSlice(out.points,lo,hi);
      auto span=certifiedHandoffSpan(g,reference.front(),segmentTangent(out.points,arc,lo,true),
        reference.back(),segmentTangent(out.points,arc,hi),reference,handoff,unknown,stop);
      if(span.empty())continue;
      auto joined=routeSlice(out.points,0,lo);appendRoute(joined,span);
      appendRoute(joined,routeSlice(out.points,hi,arc.back()));
      // Certify new span's reserve only: distant narrow original portions are
      // neither changed nor used to lower the new span's safety requirement.
      bool safe=true;
      for(size_t j=1;j<span.size();++j)if(stop()||!g.segmentWithMargin(span[j-1],span[j],reserve,unknown)){safe=false;break;}
      const auto ja=arcLengths(joined);
      for(size_t j=1;safe&&j+1<joined.size();++j)if(ja[j]>=lo-.03&&ja[j]<=lo+pathLength(span)+.03)
        if(requiresTurnRepair(turnSample(joined[j]-joined[j-1],joined[j+1]-joined[j],lateral_accel,yaw_rate),policy,sharp))safe=false;
      if(!safe)continue;
      out.points=std::move(joined);++out.smoothed;cursor=lo+pathLength(span)+.02;replaced=true;break;
    }
    if(replaced)continue;
    // Collapse only the small limiting region, with bounded deviation from
    // its original corridor. A tight curved region may require several real
    // corners; no uncertified chord is allowed. Continuous bends instead
    // retain their dynamically limited speed profile.
    const double lo=std::max(cursor,arc[first]-.10),hi=std::min(arc.back(),arc[last]+.10);
    // Never polygonize a sampled continuous bend merely for exceeding the
    // preferred radius: that manufactured extra seams and repeated stops.
    const bool discontinuity=requiresExecutionStop(turnSample(out.points[peak]-out.points[peak-1],
      out.points[peak+1]-out.points[peak],lateral_accel,yaw_rate),policy,sharp);
    if(!stop()&&discontinuity&&lo<arc[peak]-.01&&hi>arc[peak]+.01){
      auto reference=routeSlice(out.points,lo,hi);const auto ra=arcLengths(reference);
      std::vector<Vec3> polygon{reference.front(),out.points[peak],reference.back()};bool safe=true;
      for(size_t j=1;j<polygon.size()&&safe;++j){
        if(!g.segmentWithMargin(polygon[j-1],polygon[j],reserve,unknown)){safe=false;break;}
        const int n=std::max(1,int(std::ceil(distance(polygon[j-1],polygon[j])/.03)));
        for(int k=0;k<=n;++k)if(stop()||project(reference,ra,polygon[j-1]+(polygon[j]-polygon[j-1])*(double(k)/n)).error>handoff.max_deviation){safe=false;break;}
      }
      if(safe){auto joined=routeSlice(out.points,0,lo);appendRoute(joined,polygon);
        appendRoute(joined,routeSlice(out.points,hi,arc.back()));out.points=std::move(joined);
        ++out.polygonized;cursor=lo+pathLength(polygon)+.03;continue;}
    }
    cursor=arc[last]+.03;
  }
  out.budget_exhausted=stop();const auto assessment=assessTurnPath(out.points,policy,lateral_accel,yaw_rate,sharp);
  out.remaining_stops=assessment.stops;out.remaining_repair_samples=assessment.repair_samples;out.max_curvature=assessment.max_curvature;
  return out;
}
} // namespace fire_scout
