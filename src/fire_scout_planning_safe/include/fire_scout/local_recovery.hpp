#pragma once
#include "geometry.hpp"
#include <string>

namespace fire_scout {
inline bool recoveryNeedsBraking(Vec3 actual_velocity){
  return !finite(actual_velocity)||norm(actual_velocity)>.05;
}

struct LocalRecoveryConfig {
  bool enabled{true};
  double wait{.9},duration{3.5},distance{.45},speed{.18};
  int attempts{2};
  bool valid()const{
    return std::isfinite(wait)&&wait>=.3&&std::isfinite(duration)&&duration>=1&&duration<=8&&
      std::isfinite(distance)&&distance>=.2&&distance<=.75&&
      std::isfinite(speed)&&speed>0&&speed<=.25&&attempts>=1&&attempts<=3;
  }
};
// Bounded, deterministic escape episode. Candidates require a caller-provided
// continuous FULL-BODY observation + collision + directional certificate.
// No wall deletion, radius relaxation or unknown-space probing is performed.
class LocalRecovery {
 public:
  LocalRecoveryConfig cfg;
  std::string reason{"IDLE"};
  bool active()const{return active_;}
  Vec3 target()const{return target_;}
  int attempts()const{return attempts_;}
  // Parent has certified EVERY segment of a frozen historical corridor. This
  // starts the same bounded episode as a radial escape, without asserting that
  // the chord to its final endpoint is clear (it may go around a corner).
  bool beginCertifiedRoute(double now,Vec3 target,double duration){
    if(!cfg.valid()||!cfg.enabled||!finite(target)||!std::isfinite(now)||!std::isfinite(duration)||
       duration<1||duration>8||active_||attempts_>=cfg.attempts)return false;
    ++attempts_;last_attempt_=now;began_=now;active_duration_=duration;
    target_=last_target_=target;have_last_target_=true;active_=true;have_progress_=false;
    reason="CERTIFIED_HISTORY_RETREAT";return true;
  }
  void reset(){active_=false;waiting_=-1;last_attempt_=-1e9;attempts_=0;reason="IDLE";have_progress_=false;have_last_target_=false;}
  void cancel(){active_=false;waiting_=-1;reason="ROUTE_AVAILABLE";}
  // Only normal route progress rearms the episode, not its own escape motion.
  void normalProgress(Vec3 p){
    if(active_)return;
    if(!have_progress_){progress_origin_=p;have_progress_=true;}
    else if(distance(p,progress_origin_)>.75){reset();progress_origin_=p;have_progress_=true;}
  }
  bool due(double now,bool needed){
    if(!cfg.valid()||!cfg.enabled||!std::isfinite(now))return false;
    if(!needed){waiting_=-1;return false;}
    if(waiting_<0||now<waiting_)waiting_=now;
    return !active_ && now-waiting_>=cfg.wait && now-last_attempt_>=cfg.duration && attempts_<cfg.attempts;
  }
  template<class Certificate,class Clearance>
  bool start(double now,Vec3 p,Vec3 goal,Certificate certified,Clearance clearance){
    if(!cfg.valid()||!cfg.enabled||active_||!std::isfinite(now)||attempts_>=cfg.attempts||!finite(p)||!finite(goal))return false;
    last_attempt_=now;
    double best=-1e9;Vec3 candidate{};const double initial=clearance(p);
    // Short actions keep the selected exit stable and never become an endless
    // random walk. Prefer increased clearance; goal progress is secondary.
    for(double length:{.65*cfg.distance,cfg.distance})for(int i=0;i<16;++i){
      const double angle=2*pi*i/16;
      Vec3 q=p+Vec3{length*std::cos(angle),length*std::sin(angle),0};
      if(!certified(p,q))continue;
      const double gain=clearance(q)-initial;
      const double progress=distance(p,goal)-distance(q,goal);
      if(gain<.05 && progress<.15)continue;
      if(have_last_target_&&distance(q,last_target_)<.15)continue;
      const double score=3*gain+.3*progress-.05*length;
      if(score>best){best=score;candidate=q;}
    }
    if(best<-1e8){reason="NO_CERTIFIED_ESCAPE";return false;}
    ++attempts_;
    target_=candidate;last_target_=candidate;have_last_target_=true;active_=true;began_=now;
    active_duration_=cfg.duration;
    // Set the normal-progress origin AFTER this maneuver, so two escape
    // displacements cannot themselves rearm an unlimited sequence of escapes.
    have_progress_=false;
    reason="CERTIFIED_LOCAL_ESCAPE";return true;
  }
  bool step(double now,Vec3 p,bool still_safe){
    if(!active_)return false;
    if(!still_safe){reason="ESCAPE_INVALIDATED";active_=false;}
    else if(distance(p,target_)<.07){reason="ESCAPE_COMPLETE";active_=false;}
    else if(now-began_>=active_duration_||now<began_){reason="ESCAPE_TIME_LIMIT";active_=false;}
    return active_;
  }
 private:
  bool active_{false},have_progress_{false},have_last_target_{false};
  double waiting_{-1},last_attempt_{-1e9},began_{0},active_duration_{3.5};int attempts_{0};
  Vec3 target_{},last_target_{},progress_origin_{};
};
// Uniform scaling preserves direction. Apply BEFORE the binary direction
// guard, otherwise a distant braking endpoint causes stop/start chattering.
template<class Collision>
Vec3 stoppingLimitedVelocity(Vec3 p,Vec3 velocity,double reaction,double acceleration,Collision clear){
  const double speed=norm(velocity);
  if(!finite(p)||!finite(velocity)||!std::isfinite(reaction)||reaction<0||
     !std::isfinite(acceleration)||acceleration<=0)return {};
  if(speed<1e-8)return velocity;
  const Vec3 direction=velocity*(1./speed);
  double lo=0,hi=speed*reaction+speed*speed/(2*acceleration);
  if(clear(p,p+direction*hi))return velocity;
  for(int i=0;i<12;++i){const double mid=(lo+hi)*.5;if(clear(p,p+direction*mid))lo=mid;else hi=mid;}
  const double ar=acceleration*reaction;
  const double allowed=std::sqrt(ar*ar+2*acceleration*lo)-ar;
  return velocity*(std::clamp(allowed,0.,speed)/speed);
}
} // namespace fire_scout
