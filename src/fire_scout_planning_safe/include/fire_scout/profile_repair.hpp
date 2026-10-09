#pragma once
#include "route_handoff.hpp"
#include "route_switch.hpp"
#include "tracker.hpp"
#include "turn_geometry.hpp"

namespace fire_scout {
// Geometry-only response to PROFILE_LIMITED_PROGRESS. This never raises a
// follower speed limit: it proposes a small, recertified change to the same
// corridor, then measures the existing Tracker's unchanged profile on it.
// The caller still validates the latest map, peer reservations and route epoch.
struct ProfileRepairResult {
  std::vector<Vec3> points;
  std::string reason{"PROFILE_REPAIR_NOT_NEEDED"};
  bool applied{false};
  double before_limit{0},after_limit{0},protected_length{0},modified_length{0};
};

inline double alignedRouteProfileLimit(const std::vector<Vec3>&path,Vec3 current,
    Vec3 measured_velocity,double measured_yaw,const TrackerConfig&config){
  if(path.size()<2)return 0;
  Tracker probe(config);probe.initializeYaw(measured_yaw);probe.setPath(path,current);
  // Only the route profile is compared here. Live obstacle, observation,
  // braking, peer and heading guards remain authoritative in the follower.
  return probe.update(current,measured_velocity,.025,[](Vec3,Vec3){return true;},
    measured_yaw,config.max_speed_xy,{},HeadingMode::Strict).speed_limit;
}

// Only a measured stopped/crawling aircraft may replace its entry instead of
// preserving the very hook that prevents motion. Build a complete certified
// forward span to the unchanged tail, never current->old first waypoint.
inline ProfileRepairResult repairStoppedRouteEntry(const Grid&grid,
    const std::vector<Vec3>&committed,Vec3 current,double progress,double speed,
    const TrackerConfig&tracker,const RouteHandoffConfig&handoff,bool unknown,
    const std::function<bool()>&cancelled={}){
  ProfileRepairResult out;
  if(!finite(current)||!std::isfinite(progress)||!std::isfinite(speed)||speed<0||
     !tracker.turn.enabled||speed>tracker.turn.stop_speed||committed.size()<3||!handoff.enabled||!handoff.valid())return out;
  const auto arc=arcLengths(committed);
  const auto pr=project(committed,arc,current,std::max(0.,progress-.10),std::min(arc.back(),progress+2.));
  if(!std::isfinite(pr.error)||pr.error>.30||!grid.segment(current,current,unknown))return out;
  const double at=std::max(progress,pr.s);
  const auto reference=geometricRouteSuffixAt(committed,current,progress);
  const Vec3 tangent=forwardRouteTangent(committed,arc,at);
  out.before_limit=alignedRouteProfileLimit(reference,current,{},std::atan2(tangent.y,tangent.x),tracker);
  if(out.before_limit>.25)return out;
  const auto began=std::chrono::steady_clock::now();
  const auto stop=[&]{return (cancelled&&cancelled())||
    std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count()>=handoff.budget_ms;};
  double reserve=.25;
  for(Vec3 p:routeSlice(committed,at,std::min(arc.back(),at+1.5)))
    reserve=std::min(reserve,grid.clearanceAt(p,.25));
  reserve=std::min(reserve,grid.clearanceAt(current,.25));
  for(double span:{.65,1.0,1.4}){
    if(stop())break;
    const double join=std::min(arc.back(),at+span);const Vec3 b=atArc(committed,arc,join);
    const Vec3 d=b-current;const double length=norm(d);
    if(length<.35)continue;
    const Vec3 ta=d*(1./length),tb=segmentTangent(committed,arc,join);
    if(dot(ta,tb)<std::cos(45*pi/180))continue;
    auto blended=certifiedHandoffSpan(grid,current,ta,b,tb,reference,handoff,unknown,stop);
    if(blended.empty())continue;
    const auto geometry=assessTurnPath(blended,tracker.turn,tracker.lateral_accel,tracker.yaw_rate,tracker.sharp_turn);
    if(geometry.repair_samples||geometry.stops)continue;
    bool safe=true;
    for(size_t i=1;i<blended.size();++i)
      if(stop()||!grid.segmentWithMargin(blended[i-1],blended[i],std::max(0.,reserve-.005),unknown)){safe=false;break;}
    if(!safe)continue;
    appendRoute(blended,routeSlice(committed,join,arc.back()));
    if(!routeGeometryValid(grid,blended,unknown)||stop())continue;
    const double yaw=std::atan2(ta.y,ta.x);
    const double improved=alignedRouteProfileLimit(blended,current,{},yaw,tracker);
    if(improved<std::max(out.before_limit+.06,1.5*out.before_limit))continue;
    out.points=std::move(blended);out.applied=true;out.after_limit=improved;
    out.modified_length=span;out.reason="STOPPED_FORWARD_ENTRY_REPAIR";return out;
  }
  return out;
}

// Execution stalls are different from an aligned low-speed profile: a nearly
// stationary vehicle may need its NEAR bend repaired too. The former 30 cm
// mandatory prefix preserved exactly the corner that was preventing motion.
inline ProfileRepairResult repairTurnExecutionRoute(const Grid&grid,
    const std::vector<Vec3>&committed,Vec3 current,double progress,double speed,
    const TrackerConfig&tracker,const RouteHandoffConfig&handoff,bool unknown=true,
    const std::function<bool()>&cancelled={}){
  ProfileRepairResult out;out.reason="TURN_REPAIR_NO_CERTIFIED_GAIN";
  if(committed.size()<3||!finite(current)||!std::isfinite(progress)||
     !std::isfinite(speed)||speed<0)return out;
  const auto entry=repairStoppedRouteEntry(grid,committed,current,progress,speed,tracker,handoff,unknown,cancelled);
  if(entry.applied)return entry;
  const auto arc=arcLengths(committed);
  const auto pr=project(committed,arc,current,std::max(0.,progress-.10),std::min(arc.back(),progress+2.));
  if(pr.error>.30)return out;
  const double at=std::max(progress,pr.s),start=std::max(0.,at-.05);
  const auto reference=routeSlice(committed,start,arc.back());
  const double keep=speed<=tracker.turn.stop_speed?0.:
    std::max(.30,speed*(.30+tracker.max_accel_xy/tracker.max_jerk_xy)+
      speed*speed/(2*.7*tracker.max_accel_xy));
  // Preserve the existing corridor's reserve; final completion also rechecks
  // the latest local obstacle overlay and the common clearance contract.
  double reserve=.25;
  for(const auto &p:routeSlice(reference,0,3.))reserve=std::min(reserve,grid.clearanceAt(p,.25));
  const auto local=routeSlice(reference,0,std::min(3.,pathLength(reference)));
  auto repaired=prepareTurnGeometry(grid,local,tracker.turn,handoff,
    tracker.lateral_accel,tracker.yaw_rate,tracker.sharp_turn,at-start+keep,
    std::max(0.,reserve-.005),unknown,cancelled);
  if(!repaired.smoothed||repaired.points.empty()||(cancelled&&cancelled()))return out;
  appendRoute(repaired.points,routeSlice(reference,pathLength(local),pathLength(reference)));
  for(size_t i=1;i<repaired.points.size();++i)
    if(!grid.segment(repaired.points[i-1],repaired.points[i],unknown))return out;
  const Vec3 tangent=segmentTangent(reference,arcLengths(reference),at-start);
  const double yaw=std::atan2(tangent.y,tangent.x);
  out.before_limit=alignedRouteProfileLimit(reference,current,tangent*speed,yaw,tracker);
  out.after_limit=alignedRouteProfileLimit(repaired.points,current,tangent*speed,yaw,tracker);
  if(out.after_limit<out.before_limit+.03)return out;
  out.points=std::move(repaired.points);out.applied=true;
  out.protected_length=at-start+keep;out.modified_length=pathLength(local);
  out.reason="TURN_EXECUTION_LOCAL_REPAIR";return out;
}

inline ProfileRepairResult repairProfileLimitedRoute(const Grid&grid,
    const std::vector<Vec3>&committed,Vec3 current,double progress,
    Vec3 measured_velocity,double measured_yaw,const TrackerConfig&tracker,
    const RouteSwitchConfig&switch_config,const RouteHandoffConfig&handoff,
    bool unknown=true,const std::function<bool()>&cancelled={}){
  ProfileRepairResult out;
  // A bounded local attempt, not another global optimizer or a fallback that
  // can turn a failed safety certificate into permission to move.
  if(!handoff.enabled||!handoff.valid()||!switch_config.valid()||committed.size()<3||
     committed.size()>8192||!finite(current)||!finite(measured_velocity)||
     !std::isfinite(progress)||!std::isfinite(measured_yaw))return out;
  for(Vec3 p:committed)if(!finite(p))return out;
  for(double v:{tracker.lookahead,tracker.max_speed_xy,tracker.max_accel_xy,
      tracker.max_jerk_xy,tracker.yaw_rate,tracker.lateral_accel})
    if(!std::isfinite(v)||v<=0)return out;
  const auto began=std::chrono::steady_clock::now();
  const auto stop=[&]{return (cancelled&&cancelled())||
    std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count()>=handoff.budget_ms;};
  if(stop()){out.reason="PROFILE_REPAIR_BUDGET";return out;}
  const auto arc=arcLengths(committed);
  const auto pr=project(committed,arc,current,std::max(0.,progress-.10),
                        std::min(arc.back(),progress+2.));
  if(!std::isfinite(pr.error)||pr.error>.30){out.reason="PROFILE_REPAIR_OFF_ROUTE";return out;}
  const double at=std::max(progress,pr.s);
  if(arc.back()-at<1.5)return out; // Keep ordinary goal braking unchanged.
  // Exact geometry, including a short piece behind projection; never prepend
  // noisy current odometry, which was the source of centimetre-scale corners.
  const double start=std::max(0.,at-.05);
  const auto incumbent=routeSlice(committed,start,arc.back());
  const auto local_arc=arcLengths(incumbent);
  const double local_at=at-start;
  out.before_limit=out.after_limit=alignedRouteProfileLimit(
    incumbent,current,measured_velocity,measured_yaw,tracker);
  if(!std::isfinite(out.before_limit)||out.before_limit>.25)return out;
  const auto entry=repairStoppedRouteEntry(grid,committed,current,progress,norm(measured_velocity),
    tracker,handoff,unknown,stop);
  if(entry.applied)return entry;
  if(!grid.segmentFrom(current,atArc(incumbent,local_arc,std::min(local_arc.back(),local_at+.25)),unknown)){
    out.reason="PROFILE_REPAIR_PREFIX_UNSAFE";return out;
  }
  // At measured stop speed a bad entry can be replaced. During motion keep
  // at least 30 cm and a speed-dependent prefix before any edit.
  const double keep=tracker.turn.enabled&&norm(measured_velocity)<=tracker.turn.stop_speed?0.:
    std::clamp(norm(measured_velocity)*handoff.time,.30,1.0);
  const double first=local_at+keep,last=std::min(local_arc.back(),local_at+3.0);
  const Vec3 old_direction=horizontalRouteDirection(incumbent,current,switch_config.lookahead);
  const double old_backtrack=routeBacktrackDistance(incumbent,current,old_direction,
                                                   switch_config.backtrack_horizon);
  // Limit trials even when a densely sampled curve contains thousands of
  // small bends. Work on the earliest limiting bend that can be edited.
  size_t trials=0;
  for(size_t i=1;i+1<incumbent.size()&&trials<12;++i){
    if(stop())break;
    if(local_arc[i]<=first+.015||local_arc[i]>=last-.10)continue;
    const Vec3 a=incumbent[i]-incumbent[i-1],b=incumbent[i+1]-incumbent[i];
    if(norm(a)<1e-6||norm(b)<1e-6)continue;
    const double angle=std::acos(std::clamp(dot(a,b)/(norm(a)*norm(b)),-1.,1.));
    if(angle<2*pi/180)continue;
    for(double half:{.45,.70,1.05}){
      if(stop()||trials++>=12)break;
      const double lo=std::max(first,local_arc[i]-half);
      const double hi=std::min(last,local_arc[i]+half);
      if(lo>=local_arc[i]-.015||hi<=local_arc[i]+.05)continue;
      const auto reference=routeSlice(incumbent,lo,hi);
      const Vec3 ta=segmentTangent(incumbent,local_arc,lo,true),
                 tb=segmentTangent(incumbent,local_arc,hi);
      // A reversal/U-turn needs the ordinary route-switch policy and planning
      // machinery; this local profile repair cannot authorize one.
      if(dot(ta,tb)<0)continue;
      auto span=certifiedHandoffSpan(grid,reference.front(),ta,reference.back(),tb,
                                    reference,handoff,unknown,stop);
      if(span.empty())continue;
      auto candidate=routeSlice(incumbent,0,lo);appendRoute(candidate,span);
      appendRoute(candidate,routeSlice(incumbent,hi,local_arc.back()));
      const auto transition=routeTransition(incumbent,candidate,current,switch_config);
      if(!transition.valid||transition.angle>switch_config.soft_angle+1e-8||
         transition.backtrack>old_backtrack+.02)continue;
      // The certified span is new, but exact prefix/tail may have become
      // invalid in the snapshot too. Check all emitted segments, with a budget.
      bool safe=true;
      for(size_t j=1;j<candidate.size();++j){
        if(stop()||!grid.segment(candidate[j-1],candidate[j],unknown)){safe=false;break;}
      }
      if(!safe)continue;
      const double improved=alignedRouteProfileLimit(candidate,current,measured_velocity,
                                                     measured_yaw,tracker);
      if(stop()||!std::isfinite(improved)||
         improved<std::max(out.before_limit+.06,1.5*out.before_limit))continue;
      out.points=std::move(candidate);out.applied=true;
      out.after_limit=improved;out.protected_length=lo-local_at;
      out.modified_length=hi-lo;out.reason="PROFILE_LOCAL_BEZIER_REPAIR";return out;
    }
  }
  out.reason=stop()?"PROFILE_REPAIR_BUDGET":"PROFILE_REPAIR_NO_CERTIFIED_GAIN";
  return out;
}
} // namespace fire_scout
