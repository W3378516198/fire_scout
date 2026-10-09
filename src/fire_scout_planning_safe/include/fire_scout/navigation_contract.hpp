#pragma once
#include "geometry.hpp"
namespace fire_scout {
// A failed sideways preview is a short alignment fallback, not a permanent
// veto tied to an obsolete velocity vector. All retries still need fresh
// full-body observation and the final directional/collision guards.
class ObservationFallback {
 public:
  void reject(double now,Vec3 position,Vec3 motion){time_=now;origin_=position;motion_=motion;active_=true;}
  void clear(){active_=false;}
  bool blocks(double now,Vec3 position,Vec3 course){
    if(!active_)return false;
    const double product=norm(motion_)*norm(course);
    if(!std::isfinite(now)||now<time_||now-time_>=.6||distance(position,origin_)>.25||
       (product>1e-8&&dot(motion_,course)<.866*product))active_=false;
    return active_;
  }
 private:
  bool active_{false};double time_{0};Vec3 origin_{},motion_{};
};
// The path heartbeat alone cannot interrupt recovery. Verify a near join and
// a useful observed forward prefix from the ACTUAL pose; never certify the
// unrecorded remainder just because the planner published a line.
template<class Certificate>
bool certifiedRouteHandoff(const std::vector<Vec3>&path,Vec3 current,Certificate certified,Vec3&join,
    double progress=0,double maximum_error=.25){
  if(path.size()<2||!finite(current)||!std::isfinite(progress)||!std::isfinite(maximum_error)||
     maximum_error<.05||maximum_error>1.0)return false;
  for(auto p:path)if(!finite(p))return false;
  const auto arc=arcLengths(path);
  const auto pr=project(path,arc,current,std::max(0.,progress-.75),std::min(progress+2.,arc.back()));
  if(!std::isfinite(pr.error)||pr.error>maximum_error||arc.back()-pr.s<.40)return false;
  const double first=std::min(arc.back(),pr.s+.12),last=std::min(arc.back(),pr.s+.60);
  Vec3 previous=current;join=atArc(path,arc,last);
  Vec3 p=atArc(path,arc,first);if(!certified(previous,p))return false;previous=p;
  for(size_t i=1;i<path.size();++i)if(arc[i]>first&&arc[i]<last){
    if(!certified(previous,path[i]))return false;
    previous=path[i];
  }
  return certified(previous,join);
}
class StableRouteHandoff {
 public:
  void clear(){since_=-1;}
  bool update(bool certified,double now,Vec3 join,double settle_time=.30){
    if(!certified||!std::isfinite(now)||!finite(join)||!std::isfinite(settle_time)||
       settle_time<.10||settle_time>1.0){clear();return false;}
    if(since_<0||now<since_||distance(join,join_)>.25){since_=now;join_=join;return false;}
    return now-since_>=settle_time;
  }
 private:
  double since_{-1};Vec3 join_{};
};
// Fixed-anchor station keeping is spatially bounded independently of the
// side/back navigation allowance. It still requires observed free space,
// current collision checks, fresh sonar and no detected glass risk.
inline bool boundedStationKeeping(Vec3 current,Vec3 anchor,Vec3 command){
  return finite(current)&&finite(anchor)&&finite(command)&&distance(current,anchor)<=.15&&norm(command)<=.08+1e-9;
}
} // namespace fire_scout
