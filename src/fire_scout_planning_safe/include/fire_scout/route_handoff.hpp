#pragma once
#include "route_repair.hpp"
#include <array>
#include <chrono>
#include <functional>
#include <string>

namespace fire_scout {
// A short geometric commitment, never permission to execute an unsafe prefix.
// The follower retains its ordinary time/velocity, observation and peer guards.
struct RouteHandoffConfig {
  bool enabled{true};
  double min_prefix{1.0}, max_prefix{2.5}, time{1.2};
  double blend_min{.60}, blend_max{2.40}, max_deviation{.30};
  double sample_spacing{.05}, budget_ms{12.0};
  bool valid() const {
    for(double x:{min_prefix,max_prefix,time,blend_min,blend_max,max_deviation,sample_spacing,budget_ms})
      if(!std::isfinite(x)||x<=0)return false;
    return min_prefix>=.30&&max_prefix>=min_prefix&&max_prefix<=5.0&&time<=3.0&&
      blend_min>=.20&&blend_max>=blend_min&&blend_max<=5.0&&max_deviation<=1.0&&
      sample_spacing>=.02&&sample_spacing<=.10&&budget_ms<=50.0;
  }
};
struct RouteHandoffResult {
  std::vector<Vec3> points;
  std::string reason{"NO_PREFIX"};
  double prefix_length{0}, blend_length{0};
  bool applied{false};
};
inline Vec3 segmentTangent(const std::vector<Vec3>&p,const std::vector<double>&s,
                          double at,bool incoming=false){
  if(p.size()<2)return {};
  const double x=std::clamp(at+(incoming?-1e-7:1e-7),0.,s.back());
  for(size_t i=1;i<p.size();++i)if(s[i]>=x&&s[i]-s[i-1]>1e-8)
    return (p[i]-p[i-1])*(1./(s[i]-s[i-1]));
  return {};
}
// This is an exact slice of the incumbent, including its original vertices.
// No new straight chord is substituted for the already flown curve.
inline std::vector<Vec3> retainedForwardPrefix(const Grid&g,
    const std::vector<Vec3>&old,Vec3 current,double progress,double speed,
    const RouteHandoffConfig&c,bool unknown=true){
  if(!c.enabled||!c.valid()||old.size()<2||!finite(current)||!std::isfinite(speed))return {};
  const auto arc=arcLengths(old);
  const auto pr=project(old,arc,current,std::max(0.,progress-.10),std::min(arc.back(),progress+2.));
  if(!std::isfinite(pr.error)||pr.error>.40)return {};
  const double first=std::max(progress,pr.s);
  const double requested=std::clamp(std::max(0.,speed)*c.time,c.min_prefix,c.max_prefix);
  const double length=std::min(requested,arc.back()-first);
  // Shorten only for safety/route end. No obstacle or unknown-policy relaxation.
  for(double keep=length;keep>=.30-1e-8;keep-=.20){
    auto prefix=routeSlice(old,first,first+keep);
    if(prefix.size()<2||!routeGeometryValid(g,prefix,unknown))continue;
    const auto sa=arcLengths(prefix);
    if(g.segmentFrom(current,atArc(prefix,sa,std::min(.25,sa.back())),unknown))return prefix;
  }
  return {};
}

// A quintic Bezier span is a one-span clamped degree-five B-spline. Matching
// endpoint tangents and zero endpoint second derivatives joins the local line
// segments without an angular corner. This is geometric continuity, not a
// claim that nav_msgs/Path carries a time-parameterized C2 flight trajectory.
using HandoffBezier=std::array<Vec3,6>;
inline HandoffBezier handoffBezier(Vec3 a,Vec3 ta,Vec3 b,Vec3 tb,double handle){
  return {a,a+ta*handle,a+ta*(2*handle),b-tb*(2*handle),b-tb*handle,b};
}
inline Vec3 handoffPoint(HandoffBezier p,double u){
  for(size_t n=p.size()-1;n>0;--n)for(size_t i=0;i<n;++i)p[i]=p[i]*(1-u)+p[i+1]*u;
  return p[0];
}
inline bool certifyHandoff(const Grid&g,const HandoffBezier&p,bool unknown,
    const std::function<bool()>&stop,int depth=0){
  if(stop&&stop())return false;
  Vec3 lo=p[0],hi=p[0];
  for(Vec3 v:p){
    if(!finite(v))return false;
    lo={std::min(lo.x,v.x),std::min(lo.y,v.y),std::min(lo.z,v.z)};
    hi={std::max(hi.x,v.x),std::max(hi.y,v.y),std::max(hi.z,v.z)};
  }
  if(g.boxClear(lo,hi,unknown))return true;
  if(depth>=12||!g.segment(p.front(),p.front(),unknown)||!g.segment(p.back(),p.back(),unknown))return false;
  HandoffBezier work=p,left{},right{};left[0]=p[0];right[5]=p[5];
  for(size_t k=1;k<6;++k){
    for(size_t i=0;i<6-k;++i)work[i]=(work[i]+work[i+1])*.5;
    left[k]=work[0];right[5-k]=work[5-k];
  }
  return certifyHandoff(g,left,unknown,stop,depth+1)&&certifyHandoff(g,right,unknown,stop,depth+1);
}
inline void appendRoute(std::vector<Vec3>&out,const std::vector<Vec3>&p){
  for(Vec3 q:p)if(out.empty()||distance(out.back(),q)>1e-7)out.push_back(q);
}
inline std::vector<Vec3> certifiedHandoffSpan(const Grid&g,Vec3 a,Vec3 ta,Vec3 b,Vec3 tb,
    const std::vector<Vec3>&reference,const RouteHandoffConfig&c,bool unknown,
    const std::function<bool()>&stop){
  const double chord=distance(a,b);
  if(chord<.15||chord>2*c.blend_max||norm(ta)<.9||norm(tb)<.9||reference.size()<2)return {};
  const auto ref_arc=arcLengths(reference);
  for(double ratio:{.20,.30,.12}){
    if(stop&&stop())return {};
    const auto controls=handoffBezier(a,ta,b,tb,chord*ratio);
    // ||B'(u)|| <= degree * max(control-edge length). This bounds every
    // uniform-u sample chord, unlike the total control-polygon length.
    // Certify the emitted polyline AND the complete analytic curve.
    double edge=0;for(size_t i=1;i<controls.size();++i)edge=std::max(edge,distance(controls[i-1],controls[i]));
    const int count=std::max(12,int(std::ceil(5*edge/c.sample_spacing)));
    if(count>512)continue;
    std::vector<Vec3> samples;bool good=true;Vec3 previous_direction=ta;
    for(int i=0;i<=count;++i){
      if(stop&&stop())return {};
      Vec3 q=handoffPoint(controls,double(i)/count);
      if(project(reference,ref_arc,q).error>c.max_deviation){good=false;break;}
      if(!samples.empty()){
        Vec3 d=q-samples.back();const double n=norm(d);
        if(n<1e-7||dot(d*(1./n),previous_direction)<std::cos(20*pi/180)||
            !g.segment(samples.back(),q,unknown)){good=false;break;}
        previous_direction=d*(1./n);
      }
      samples.push_back(q);
    }
    if(good&&dot(previous_direction,tb)>=std::cos(20*pi/180)&&
       certifyHandoff(g,controls,unknown,stop))return samples;
  }
  return {};
}

inline RouteHandoffResult connectRetainedPrefix(const Grid&g,
    const std::vector<Vec3>&prefix,const std::vector<Vec3>&candidate,
    const RouteHandoffConfig&c,bool unknown=true,const std::function<bool()>&cancelled={}){
  RouteHandoffResult out;out.points=candidate;
  if(!c.enabled||!c.valid()||prefix.size()<2||candidate.size()<2)return out;
  const auto began=std::chrono::steady_clock::now();
  const auto stop=[&]{return (cancelled&&cancelled())||
    std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count()>=c.budget_ms;};
  if(!routeGeometryValid(g,prefix,unknown)){out.reason="PREFIX_UNSAFE";return out;}
  const auto pa=arcLengths(prefix),ca=arcLengths(candidate);
  const Vec3 a=prefix.back(),ta=segmentTangent(prefix,pa,pa.back(),true);
  const auto near=project(candidate,ca,a,0,std::min(ca.back(),pa.back()+2*c.blend_max));
  bool shared=near.error<1e-6&&dot(ta,segmentTangent(candidate,ca,near.s))>.999;
  double cursor=0;
  for(double s=std::min(.20,pa.back());shared&&s<pa.back();s+=.10){
    if(stop()){out.reason="HANDOFF_BUDGET";return out;}
    auto pr=project(candidate,ca,atArc(prefix,pa,s),cursor,near.s);
    shared=pr.error<1e-6;cursor=pr.s;
  }
  if(shared){
    // Finding the prefix somewhere in the candidate is not enough: its
    // leading live-position connector can still be a centimetre-scale hook.
    // Emit the exact retained prefix, then only the candidate's future tail.
    auto tail=routeSlice(candidate,near.s,ca.back());
    if(!tail.empty())tail.front()=prefix.back(); // remove a numerical sub-micrometre seam
    out.points=prefix;appendRoute(out.points,tail);
    if(stop()||!routeGeometryValid(g,out.points,unknown)){
      out.points=candidate;out.reason=stop()?"HANDOFF_BUDGET":"PREFIX_TAIL_UNSAFE";return out;
    }
    out.applied=true;out.prefix_length=pa.back();out.reason="PREFIX_ALREADY_SHARED";return out;
  }
  if(!std::isfinite(near.error)){out.reason="NO_FORWARD_JOIN";return out;}
  const double last=std::min(ca.back(),near.s+c.blend_max);
  for(double s=std::min(last,near.s+c.blend_min);s<=last+1e-8;s+=.30){
    if(stop())break;
    const Vec3 b=atArc(candidate,ca,s),tb=segmentTangent(candidate,ca,s);
    auto reference=prefix;appendRoute(reference,routeSlice(candidate,0,s));
    // Do not use the artificial anchor-to-candidate-start connector as a
    // deviation reference: independent distances to each original route win.
    auto span=certifiedHandoffSpan(g,a,ta,b,tb,reference,c,unknown,stop);
    if(span.empty())continue;
    bool deviation_ok=true;
    for(Vec3 q:span)if(std::min(project(prefix,pa,q).error,project(candidate,ca,q).error)>c.max_deviation){deviation_ok=false;break;}
    if(!deviation_ok)continue;
    std::vector<Vec3> joined=prefix;appendRoute(joined,span);
    appendRoute(joined,routeSlice(candidate,s,ca.back()));
    if(stop()||!routeGeometryValid(g,joined,unknown))continue;
    out.points=std::move(joined);out.applied=true;out.prefix_length=pa.back();
    out.blend_length=pathLength(span);out.reason="PREFIX_BEZIER_JOIN";return out;
  }
  out.reason=stop()?"HANDOFF_BUDGET":"NO_SAFE_BLEND";return out;
}

// Round only a repair seam. The exact curve outside the short window remains
// untouched, and every shortcut is recertified with the full vehicle envelope.
inline std::vector<Vec3> smoothRepairSeam(const Grid&g,const std::vector<Vec3>&path,
    Vec3 seam,double minimum_begin,const RouteHandoffConfig&c,bool unknown,
    const std::function<bool()>&cancelled={}){
  if(!c.enabled||path.size()<3)return path;
  const auto arc=arcLengths(path);const auto at=project(path,arc,seam);
  if(at.error>.01)return path;
  const auto began=std::chrono::steady_clock::now();
  const auto stop=[&]{return (cancelled&&cancelled())||
    std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count()>=c.budget_ms;};
  for(double half:{.45,.70,1.0}){
    const double lo=std::max(minimum_begin,at.s-half),hi=std::min(arc.back(),at.s+half);
    if(lo>=at.s-.10||hi<=at.s+.10)continue;
    auto reference=routeSlice(path,lo,hi);
    auto span=certifiedHandoffSpan(g,reference.front(),segmentTangent(path,arc,lo,true),
      reference.back(),segmentTangent(path,arc,hi),reference,c,unknown,stop);
    if(span.empty())continue;
    auto out=routeSlice(path,0,lo);appendRoute(out,span);appendRoute(out,routeSlice(path,hi,arc.back()));
    return out;
  }
  return path;
}
} // namespace fire_scout
