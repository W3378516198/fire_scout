#pragma once
#include "geometry.hpp"
#include "turn_policy.hpp"
#include <deque>
#include <functional>
namespace fire_scout {
struct TrackerConfig {
  TurnPolicyConfig turn;
  double lookahead{.65}, max_speed_xy{1.8}, max_speed_z{.50}, max_accel_xy{1.5}, max_accel_z{.70};
  double max_jerk_xy{3.0}, max_jerk_z{2.0}, lateral_accel{.7};
  double kp_xy{1.2}, kp_z{1.0}, kd_xy{.10}, kd_z{.20};
  double goal_tolerance{.18}, yaw_rate{.8}, yaw_accel{1.2}, yaw_deadband{.025};
  double yaw_preview_time{.30};
  // Velocity-mode tracking: geometry feedforward plus bounded cross-track PD.
  double cross_track_gain{.9}, cross_track_damping{.65}, cross_track_speed{.45};
  bool face_motion{true};
  // Used only by explicit Relaxed updates. These constrain the desired motion
  // before the acceleration/jerk filter; a policy must still check the actual
  // swept stopping corridor and bound cumulative motion outside sensor view.
  double relaxed_lateral_speed{.35}, relaxed_reverse_speed{.25};
  // Heading alignment is continuous for a holonomic multirotor. A valid path
  // is slowed in stages, then stopped only at an extreme error. The minimum
  // speed is a floor below the already safety-limited route speed; it can never
  // raise a command above the planner, clearance or external speed ceiling.
  bool heading_speed_scaling_enabled{true};
  bool allow_forward_during_alignment{true};
  double heading_full_speed{10*pi/180}, heading_slow{25*pi/180};
  double heading_min_speed{60*pi/180}, heading_stop{85*pi/180};
  double heading_slow_speed_scale{.60}, heading_min_speed_scale{.25};
  double min_forward_speed{.12};
  double sharp_turn{35*pi/180};
  // Match the final directional guard's frontal cone. A low-speed cross-track
  // correction must not ask that guard to stop an otherwise aligned route.
  double strict_forward_half_angle{.35};
  double narrow_speed{.50}, clearance_slow_distance{.40};
};

inline double headingSpeedScale(const TrackerConfig &cfg,double absolute_error){
  const double e=std::abs(absolute_error);
  if(!cfg.heading_speed_scaling_enabled)
    return std::clamp((cfg.heading_stop-e)/
      std::max(1e-9,cfg.heading_stop-cfg.heading_slow),0.,1.);
  if(e<=cfg.heading_full_speed)return 1.;
  if(e>=cfg.heading_stop)return 0.;
  if(e<cfg.heading_slow){
    const double u=(e-cfg.heading_full_speed)/
      std::max(1e-9,cfg.heading_slow-cfg.heading_full_speed);
    return 1.+u*(cfg.heading_slow_speed_scale-1.);
  }
  if(e<cfg.heading_min_speed){
    const double u=(e-cfg.heading_slow)/
      std::max(1e-9,cfg.heading_min_speed-cfg.heading_slow);
    return cfg.heading_slow_speed_scale+
      u*(cfg.heading_min_speed_scale-cfg.heading_slow_speed_scale);
  }
  return cfg.heading_min_speed_scale;
}

struct Control {
  Vec3 velocity{}, target{};
  double yaw{0}, progress{0};
  bool reached{false}, blocked{false};
  bool heading_wait{false};
  double speed_limit{0}, course_error{0};
  bool alignment_active{false}, alignment_forward_enabled{false};
  bool alignment_min_speed_active{false};
  double heading_speed_scale{1};
  bool turn_stop{false};
  const char *turn_phase{"NONE"};
  double turn_curvature{0},turn_anchor_distance{-1};
};
// Configured preserves the legacy cfg.face_motion behavior. A sensor policy can
// select Strict around glass-risk observations and Relaxed for small smooth
// sidesteps / reverse escapes without reinitializing the trajectory tracker.
enum class HeadingMode { Configured, Strict, Relaxed };
// Replan when neither translation nor yaw alignment makes measurable progress.
// Path heartbeats do not reset this watchdog; safety holds and goal hold do.
class MotionProgressWatchdog {
public:
  double timeout{2.0}, translation{.10}, rotation{.10}, repeat{1.0};
  // Yaw alignment is real progress, but repeated rotation/oscillation must not
  // hide an indefinitely stationary vehicle. This is deliberately longer than
  // a 180-degree alignment at the configured default yaw rate.
  double max_alignment_time{8.0};
  double useful_translation{.35};
  bool update(Vec3 position,double yaw,double dt,bool active){
    if(!active){
      ready_=false;stagnant_=translation_stagnant_=0;
      net_samples_.clear();elapsed_=0;next_net_report_=max_alignment_time;
      next_report_=timeout;next_translation_report_=max_alignment_time;return false;
    }
    if(!ready_){
      anchor_=translation_anchor_=useful_anchor_=position;anchor_yaw_=yaw;ready_=true;
      stagnant_=translation_stagnant_=0;next_report_=timeout;
      next_translation_report_=max_alignment_time;
      net_samples_.clear();net_samples_.push_back({position,0});
      useful_stagnant_=0;elapsed_=0;next_net_report_=max_alignment_time;return false;
    }
    const double elapsed=std::max(0.,dt);
    elapsed_+=elapsed;
    useful_stagnant_+=elapsed;
    if(distance(position,useful_anchor_)>=useful_translation){useful_anchor_=position;useful_stagnant_=0;}
    if(net_samples_.empty() || elapsed_-net_samples_.back().second>=.05)
      net_samples_.push_back({position,elapsed_});
    while(net_samples_.size()>1 && net_samples_[1].second<=elapsed_-max_alignment_time)
      net_samples_.pop_front();
    translation_stagnant_+=elapsed;
    if(distance(position,translation_anchor_)+1e-9>=translation){
      translation_anchor_=position;translation_stagnant_=0;
      next_translation_report_=max_alignment_time;
    }
    if(distance(position,anchor_)+1e-9>=translation || std::abs(wrap(yaw-anchor_yaw_))>=rotation){
      anchor_=position;anchor_yaw_=yaw;stagnant_=0;next_report_=timeout;
    }else stagnant_+=elapsed;
    const bool no_progress=stagnant_+1e-9>=next_report_;
    const bool rotation_only=translation_stagnant_+1e-9>=next_translation_report_;
    // Repeated side/back oscillation can cross every .10 m anchor without
    // making useful net progress. Retain a bounded time window so neither
    // those anchor resets nor yaw rotations hide an indefinitely stuck drone.
    const bool cycling=!net_samples_.empty() &&
      elapsed_-net_samples_.front().second+1e-9>=max_alignment_time &&
      elapsed_+1e-9>=next_net_report_ &&
      distance(position,net_samples_.front().first)<translation;
    const bool local_activity=useful_stagnant_>=max_alignment_time && elapsed_>=next_net_report_;
    if(!no_progress && !rotation_only && !cycling && !local_activity)return false;
    if(no_progress)next_report_=stagnant_+repeat;
    if(rotation_only)next_translation_report_=translation_stagnant_+repeat;
    if(cycling||local_activity)next_net_report_=elapsed_+repeat;
    return true;
  }
  double stagnantSeconds()const{return stagnant_;}
  double translationStagnantSeconds()const{return translation_stagnant_;}
  double usefulStagnantSeconds()const{return ready_?useful_stagnant_:0.;}
private:
  Vec3 anchor_{},translation_anchor_{},useful_anchor_{};
  double useful_stagnant_{0};
  double anchor_yaw_{0},stagnant_{0},translation_stagnant_{0},next_report_{2},next_translation_report_{8};
  bool ready_{false};
  std::deque<std::pair<Vec3,double>> net_samples_;
  double elapsed_{0},next_net_report_{8};
};
class Tracker {
public:
  explicit Tracker(TrackerConfig c = {}) : cfg(c), profile_strict_(c.face_motion) {}
  TrackerConfig cfg;
  void initializeYaw(double yaw) { yaw_=yaw; yaw_rate_=0; yaw_ready_=true; course_ready_=false; course_rate_=0; }
  void resetMotion() { last_={}; acceleration_={}; yaw_rate_=0; }
  void resumeAfterRecovery(Vec3 current){
    resetMotion();pivot_active_=course_turn_active_=rejoin_active_=false;
    if(path_.empty())return;
    // Keep the local arc neighborhood, including a bounded retreat. Reusing
    // setPath from scratch on a long mission would jump back to its first 2 m.
    progress_=project(path_,arc_,current,std::max(0.,progress_-.75),
      std::min(arc_.back(),progress_+2.)).s;
    passed_corner_s_=std::min(passed_corner_s_,progress_-.01);
  }
  // Feed back the command actually sent after an emergency/sensor speed override.
  void appliedVelocity(Vec3 v) {
    if(distance(v,last_)>1e-6){last_=v; acceleration_={};}
  }
  // Return whether the stored geometry changed. Heartbeats (including repeated
  // empty paths) return false and must not cancel a recovery or reset motion.
  // Non-finite input is rejected without replacing the current valid route.
  bool setPath(const std::vector<Vec3>&points,Vec3 current) {
    if(points.empty()){
      const bool changed=!path_.empty();
      path_.clear();arc_.clear();profile_.clear();corner_.clear();progress_=0;pivot_active_=false;course_turn_active_=false;rejoin_active_=false;
      return changed;
    }
    for(Vec3 p:points)if(!finite(p))return false;
    std::vector<Vec3> candidate;
    for(Vec3 p:points)if(candidate.empty()||distance(p,candidate.back())>1e-6)candidate.push_back(p);
    bool same=path_.size()==candidate.size();
    if(same)for(size_t i=0;i<candidate.size();++i)if(distance(path_[i],candidate[i])>1e-5){same=false;break;}
    if(same)return false;
    const bool had_rejoin=rejoin_active_&&!path_.empty();
    const Vec3 old_rejoin=had_rejoin?atArc(path_,arc_,rejoin_s_):Vec3{};
    Vec3 old_course=previewTarget(current)-current;
    rejoin_active_=false;
    // Prefix repairs often only trim already-flown samples. Keep continuous
    // yaw-rate tracking when the immediate geometric direction is unchanged.
    const bool had_passed_corner=passed_corner_s_>=0&&!path_.empty();
    const Vec3 passed_corner=had_passed_corner?atArc(path_,arc_,passed_corner_s_):Vec3{};
    path_=std::move(candidate);
    if(pivot_active_){
      bool retained=false;
      for(size_t i=1;i+1<path_.size();++i)if(distance(path_[i],pivot_route_point_)<.015){
        const auto outgoing=path_[i+1]-path_[i];
        if(std::hypot(outgoing.x,outgoing.y)>1e-6&&
           std::abs(wrap(std::atan2(outgoing.y,outgoing.x)-pivot_yaw_))<=.35)retained=true;
        break;
      }
      if(!retained)pivot_active_=false;
    }
    // The planner validates a new route from its first two metres. Use the
    // same local projection, otherwise a crossing in the new tail can jump
    // past the retained prefix and instantly reverse the tracking direction.
    arc_=arcLengths(path_); progress_=project(path_,arc_,current,0,std::min(arc_.back(),2.0)).s;
    passed_corner_s_=-1;
    buildProfile();
    if(had_rejoin){
      const auto retained=project(path_,arc_,old_rejoin,progress_,std::min(arc_.back(),progress_+1.5));
      if(retained.error<.03&&retained.s>progress_+.05){rejoin_active_=true;rejoin_s_=retained.s;}
    }
    if(had_passed_corner){
      const double end=rejoin_active_?rejoin_s_:progress_+.10;
      const auto old_corner=project(path_,arc_,passed_corner,0,std::min(arc_.back(),end));
      if(old_corner.error<.03)passed_corner_s_=old_corner.s;
    }
    const Vec3 new_course=previewTarget(current)-current;
    if(std::hypot(old_course.x,old_course.y)<.02||std::hypot(new_course.x,new_course.y)<.02||
       std::abs(wrap(std::atan2(old_course.y,old_course.x)-std::atan2(new_course.y,new_course.x)))>.35){
      course_ready_=false;course_rate_=0;course_turn_active_=false;
    }
    return true;
  }
  Control update(Vec3 current,Vec3 measured_velocity,double dt,
                 const std::function<bool(Vec3,Vec3)>&clear,
                 double measured_yaw=std::numeric_limits<double>::quiet_NaN(),
                 double external_limit=1e9,
                 const std::function<double(Vec3)>&clearance={},
                 HeadingMode heading_mode=HeadingMode::Configured,
                 const std::function<bool(Vec3,Vec3)>&vertical_clear={}) {
    const bool strict=heading_mode==HeadingMode::Strict ||
      (heading_mode==HeadingMode::Configured && cfg.face_motion);
    if(strict!=profile_strict_){profile_strict_=strict;buildProfile();}
    if(path_.empty())return hold(current,current,measured_velocity,dt);
    if(path_.size()==1)return terminalHold(current,path_.front(),measured_velocity,dt,measured_yaw,strict);
    auto pr=project(path_,arc_,current,std::max(0.,progress_-.10),
                    std::min(arc_.back(),progress_+2.0));
    const bool rejoin_passed=rejoin_active_&&pr.s>rejoin_s_+.05&&pr.s<rejoin_s_+.35&&
      pr.error<.15&&clear(current,atArc(path_,arc_,std::min(arc_.back(),pr.s+.15)));
    if(rejoin_active_&&(distance(current,atArc(path_,arc_,rejoin_s_))<=cfg.turn.capture_distance||rejoin_passed)){
      progress_=std::max(progress_,rejoin_s_);rejoin_active_=false;course_turn_active_=false;
    }
    // Do not project past an unpassed fallback corner: with cross-track error,
    // nearest-point projection can already lie on its outgoing segment.
    double next_corner=arc_.back()+1;size_t next_corner_index=path_.size();
    for(size_t i=1;i+1<path_.size();++i){
      if(!corner_[i] || arc_[i]<=passed_corner_s_)continue;
      if(cfg.turn.enabled){
        const bool captured=distance(current,path_[i])<=cfg.turn.capture_distance;
        const bool past=spatiallyPastCorner(i,current,pr.s);
        const bool active=pivot_active_&&distance(path_[i],pivot_route_point_)<.015;
        const Vec3 incoming=path_[i]-path_[i-1];
        const double overrun=dot(current-path_[i],incoming)/std::max(1e-9,norm(incoming));
        const bool overshot=overrun>cfg.turn.capture_distance+.03&&
          (active||(distance(current,path_[i])<=1.2&&arc_[i]<=progress_+1.2));
        const Vec3 out_direction=path_[i+1]-path_[i];
        const double observed_yaw=std::isfinite(measured_yaw)?measured_yaw:yaw_;
        const Vec3 to_corner=path_[i]-current;
        const bool stopped=norm(measured_velocity)<=cfg.turn.stop_speed;
        // A short entry stitch can lie behind/sideways of a stopped aircraft.
        // Turning toward it and then toward its outgoing leg creates two
        // contradictory yaw owners. Join the future leg only through a checked
        // chord, before the next mandatory corner. This is never bookkeeping
        // permission to cross an obstacle or a later corner.
        const bool backward_entry=arc_[i]<=.35&&norm(to_corner)<=.40&&
          norm(to_corner)>cfg.turn.capture_distance&&
          (course_turn_active_||std::abs(wrap(std::atan2(to_corner.y,to_corner.x)-observed_yaw))>=cfg.heading_stop);
        if(stopped&&backward_entry){
          double end=std::min(arc_.back(),arc_[i]+1.2);
          for(size_t j=i+1;j+1<path_.size();++j)if(corner_[j]){end=std::min(end,arc_[j]);break;}
          const auto local=project(path_,arc_,current,arc_[i],end);
          const double join_s=std::min(end,std::max(arc_[i],local.s)+cfg.lookahead);
          const Vec3 join=atArc(path_,arc_,join_s);
          if(join_s>arc_[i]+.05&&distance(current,join)<=1.5&&clear(current,join)){
            passed_corner_s_=arc_[i];progress_=std::max(progress_,local.s);
            pivot_active_=course_turn_active_=false;rejoin_active_=true;rejoin_s_=join_s;
            resetMotion();initializeYaw(observed_yaw);continue;
          }
          last_={};acceleration_={};Control u;u.target=current;u.yaw=observed_yaw;u.progress=progress_;
          u.turn_stop=true;u.turn_phase="TURN_REJOIN_BLOCKED";
          u.turn_anchor_distance=norm(to_corner);u.blocked=!clear(current,current);return u;
        }
        // An obsolete stitch already behind an aligned, departing vehicle is
        // not a new turn. Retain the existing no-return spatial certificate.
        if(past&&!active&&std::hypot(out_direction.x,out_direction.y)>1e-6&&
           std::abs(wrap(std::atan2(out_direction.y,out_direction.x)-observed_yaw))<=cfg.turn.yaw_tolerance){
          passed_corner_s_=arc_[i];progress_=std::max(progress_,arc_[i]);continue;
        }
        if(captured||past||overshot||active){
          const double actual_yaw=std::isfinite(measured_yaw)?measured_yaw:yaw_;
          if(!active){
            course_turn_active_=false;pivot_active_=true;pivot_anchor_=current;pivot_route_point_=path_[i];
            pivot_yaw_=std::hypot(out_direction.x,out_direction.y)>1e-6?
              std::atan2(out_direction.y,out_direction.x):actual_yaw;
            initializeYaw(actual_yaw);
          }
          // The first outgoing edge is the commanded geometry. Looking across
          // the next corner could align yaw with a chord that is never flown.
          const Vec3 outgoing=path_[i+1]-path_[i];
          const double direction=pivot_yaw_;
          const double error=wrap(direction-actual_yaw);
          // Braking drift can carry the vehicle beyond a newly introduced
          // corner. Never servo back to that old point by bookkeeping alone.
          // After measured braking, join only its immediate outgoing region,
          // before the next mandatory corner and through a checked chord.
          if(stopped&&overshot){
            double end=std::min(arc_.back(),arc_[i]+1.2);
            for(size_t j=i+1;j+1<path_.size();++j)if(corner_[j]){end=std::min(end,arc_[j]);break;}
            const auto local=project(path_,arc_,current,arc_[i],end);
            const double join_s=std::min(end,std::max(arc_[i],local.s)+cfg.lookahead);
            const Vec3 join=atArc(path_,arc_,join_s);
            if(join_s>arc_[i]+.05&&distance(current,join)<=1.5&&clear(current,join)){
              passed_corner_s_=arc_[i];progress_=std::max(progress_,local.s);
              pivot_active_=false;rejoin_active_=true;rejoin_s_=join_s;
              resetMotion();initializeYaw(actual_yaw);continue;
            }
            // No certified connector: hold and let the execution-repair
            // watchdog request new geometry. Do not turn toward a stale point.
            last_={};acceleration_={};Control u;u.target=current;u.yaw=actual_yaw;u.progress=progress_;
            u.turn_stop=true;u.turn_phase="TURN_REJOIN_BLOCKED";
            u.turn_anchor_distance=distance(current,path_[i]);return u;
          }
          if(stopped&&std::abs(error)<=cfg.turn.yaw_tolerance){
            passed_corner_s_=arc_[i];progress_=std::max(progress_,arc_[i]);
            pivot_active_=false;continue;
          }
          // Brake before rotating. A zero translational setpoint is still
          // passed through the node's map/sonar/peer safeguards; it is not a
          // certificate that the measured vehicle has already stopped.
          if(!stopped)initializeYaw(actual_yaw);else stepYaw(direction,dt);
          last_={};acceleration_={};Control u;u.target=pivot_anchor_;u.yaw=yaw_;u.progress=progress_;
          u.heading_wait=true;u.course_error=error;u.alignment_active=true;
          u.turn_stop=true;u.turn_phase=stopped?"TURN_IN_PLACE":"TURN_BRAKE";
          u.turn_curvature=turnSample(path_[i]-path_[i-1],outgoing,cfg.lateral_accel,cfg.yaw_rate).curvature;
          u.turn_anchor_distance=distance(current,pivot_route_point_);
          u.blocked=!clear(current,current);return u;
        }
      }
      if(!cfg.turn.enabled&&distance(current,path_[i])<.08 && norm(measured_velocity)<.10){
        passed_corner_s_=arc_[i];
        progress_=std::max(progress_,arc_[i]+.01);
        continue;
      }
      // A freshly repaired path can place a centimetre-scale stitch behind a
      // vehicle which is still decelerating.  If the live projection and the
      // spatial position are already on the outgoing leg, never command a
      // return to that obsolete corner.
      if(!cfg.turn.enabled&&spatiallyPastCorner(i,current,pr.s)){
        passed_corner_s_=arc_[i];
        progress_=std::max(progress_,arc_[i]+.01);
        continue;
      }
      // A new path may legitimately begin behind the current position.
      if(arc_[i]+.10<progress_){passed_corner_s_=arc_[i];continue;}
      next_corner=arc_[i];next_corner_index=i;break;
    }
    progress_=std::min(next_corner,std::max(progress_,pr.s));
    const double remaining=std::max(0.,arc_.back()-progress_);
    const double goal_distance=distance(current,path_.back());
    if(remaining<.08 && goal_distance<.3){
      return terminalHold(current,path_.back(),measured_velocity,dt,measured_yaw,strict);
    }
    double ahead=std::min(arc_.back(),progress_+cfg.lookahead);
    if(rejoin_active_)ahead=rejoin_s_;
    for(size_t i=1;i+1<path_.size();++i)
      if(corner_[i] && arc_[i]>passed_corner_s_ && arc_[i]>=progress_-1e-6)
        ahead=std::min(ahead,arc_[i]);
    Vec3 target=atArc(path_,arc_,ahead);
    while(!clear(current,target) && ahead>progress_+.025){
      ahead=progress_+(ahead-progress_)*.5; target=atArc(path_,arc_,ahead);
    }
    if(!clear(current,target))return {smooth({},dt),current,yaw_,progress_,false,true};
    const double approach_remaining=std::max(remaining,goal_distance);
    double speed=std::min({cfg.max_speed_xy,external_limit,
                          std::sqrt(2*.7*cfg.max_accel_xy*approach_remaining),1.5*approach_remaining});
    // Look ahead by a stopping horizon, rather than discovering a sharp bend at speed.
    const double vxy=std::hypot(measured_velocity.x,measured_velocity.y);
    const double reaction=.30+cfg.max_accel_xy/cfg.max_jerk_xy;
    // The lookahead chord starts steering before the vehicle reaches a bend.
    // Align curvature braking with its midpoint; leave mandatory stop anchors
    // at their actual location so preview cannot create a premature zero speed.
    const double steering_preview=.5*(ahead-progress_);
    const double horizon=std::max(.45,vxy*vxy/(2*.7*cfg.max_accel_xy)+vxy*reaction)+steering_preview;
    for(size_t i=0;i<path_.size();++i){
      if(arc_[i]<progress_-1e-6 || arc_[i]>progress_+horizon)continue;
      double distance_ahead=std::max(0.,arc_[i]-progress_-
        ((corner_[i] || i+1==path_.size())?0:steering_preview));
      // A zero-speed corner is a spatial target, not just an arc coordinate.
      // Using only projected arc distance deadlocked 8+ cm off the corner.
      if((corner_[i] && arc_[i]>passed_corner_s_) || i+1==path_.size())
        distance_ahead=std::max(distance_ahead,distance(current,path_[i]));
      double target_speed=(corner_[i] && arc_[i]<=passed_corner_s_)?cfg.max_speed_xy:profile_[i];
      const double ar=.7*cfg.max_accel_xy*reaction;
      speed=std::min(speed,std::sqrt((target_speed+ar)*(target_speed+ar)+
                                    2*.7*cfg.max_accel_xy*distance_ahead)-ar);
    }
    if(clearance){
      double reserve=std::min(clearance(current),clearance(target));
      double k=std::clamp(reserve/std::max(.01,cfg.clearance_slow_distance),0.,1.);
      speed=std::min(speed,cfg.narrow_speed+(cfg.max_speed_xy-cfg.narrow_speed)*k);
      // Brake BEFORE a low-clearance section, including controller/jerk lag.
      // Looking only at the .45 m carrot discovered narrow space too late.
      for(double ds=.15;ds<=horizon;ds+=.15){
        const Vec3 p=atArc(path_,arc_,std::min(arc_.back(),progress_+ds));
        const double q=std::clamp(clearance(p)/std::max(.01,cfg.clearance_slow_distance),0.,1.);
        const double local=cfg.narrow_speed+(cfg.max_speed_xy-cfg.narrow_speed)*q;
        const double ar=.7*cfg.max_accel_xy*reaction;
        speed=std::min(speed,std::sqrt((local+ar)*(local+ar)+2*.7*cfg.max_accel_xy*ds)-ar);
      }
    }
    // Feedforward comes from the planned curve, not a normalized vector from
    // noisy odometry to a moving carrot. V7's effective lateral gain was v/L
    // (2.67 /s at 1.2 m/s and L=.45 m), with no velocity damping; the command
    // filter and flight-controller lag then produced repeated overshoot.
    const Vec3 reference=atArc(path_,arc_,progress_);
    Vec3 geometry=target-reference;
    const Vec3 course=target-current;
    const bool anchor=rejoin_active_ || std::abs(ahead-next_corner)<1e-6 || remaining<cfg.lookahead;
    Vec3 tangent=geometry*(1./std::max(1e-9,norm(geometry)));
    Vec3 desired{};
    if(anchor || norm(geometry)<1e-6){
      // A sharp fallback corner/end must be reached spatially before advancing.
      // A bounded position servo avoids constant-speed orbiting at tiny error.
      desired=course*cfg.cross_track_gain-measured_velocity*cfg.cross_track_damping;
      desired=limitNorm(desired,speed);
      geometry=course;
    }else{
      Vec3 error=reference-current;
      error=error-tangent*dot(error,tangent);
      Vec3 cross_velocity=measured_velocity-tangent*dot(measured_velocity,tangent);
      Vec3 correction=error*cfg.cross_track_gain-cross_velocity*cfg.cross_track_damping;
      correction=limitNorm(correction,std::min(cfg.cross_track_speed,.45*speed));
      if(strict && std::hypot(tangent.x,tangent.y)>1e-6){
        const double actual_yaw=std::isfinite(measured_yaw)?measured_yaw:yaw_;
        const double c=std::cos(actual_yaw),s=std::sin(actual_yaw);
        // Leave a small interior reserve for roundoff and the final guard. Only
        // reduce correction when the reference itself is already inside the
        // frontal cone; a sideways route still needs the normal yaw alignment.
        const double slope=std::tan(.95*std::clamp(cfg.strict_forward_half_angle,.001,pi/2-.001));
        const auto frontal=[&](Vec3 v){
          const double x=c*v.x+s*v.y,y=-s*v.x+c*v.y;
          return x>1e-9 && std::abs(y)<=slope*x;
        };
        const Vec3 base=tangent*speed;
        if(frontal(base) && !frontal(base+correction)){
          double lo=0,hi=1;
          for(int i=0;i<16;++i){const double mid=.5*(lo+hi);
            if(frontal(base+correction*mid))lo=mid;else hi=mid;}
          correction=correction*lo;
        }
      }
      desired=tangent*speed+correction;
      desired=limitNorm(desired,speed);
    }
    // Feedforward + correction may point outside the certified lookahead chord.
    // Revert to that chord if its short steering preview is not collision-free;
    // the node additionally checks the complete swept stopping corridor.
    if(norm(desired)>1e-6 && !clear(current,current+desired*.35)){
      // Only the shortened carrot chord is certified here. Normalizing a tiny
      // chord to the full route speed can overshoot its end while steering/yaw
      // changes, even though both the path and the carrot point are clear.
      const double ar=.7*cfg.max_accel_xy*reaction;
      const double chord_speed=std::sqrt(ar*ar+2*.7*cfg.max_accel_xy*norm(course))-ar;
      speed=std::min(speed,chord_speed);
      desired=course*(speed/std::max(1e-9,norm(course)));
      // Steering reverted to the safe carrot chord. Yaw must follow that same
      // chord, not the rejected curve tangent (a second alignment deadlock).
      geometry=course;
    }
    if(std::abs(desired.z)>cfg.max_speed_z)
      desired=desired*(cfg.max_speed_z/std::abs(desired.z));
    desired=limitXY(desired,cfg.max_speed_xy);
    const double actual_yaw=std::isfinite(measured_yaw)?measured_yaw:yaw_;
    // A new route/rejoin may require a large initial heading change without
    // containing any internal corner. Latch it until stopped AND aligned,
    // rather than releasing translation as soon as error falls below 85 deg.
    if(cfg.turn.enabled&&std::hypot(geometry.x,geometry.y)>1e-5){
      const double direction=std::atan2(geometry.y,geometry.x);
      if(!course_turn_active_&&std::abs(wrap(direction-actual_yaw))>=cfg.heading_stop){
        course_turn_active_=true;course_turn_yaw_=direction;initializeYaw(actual_yaw);
      }
      if(course_turn_active_){
        const double error=wrap(course_turn_yaw_-actual_yaw);
        const bool stopped=norm(measured_velocity)<=cfg.turn.stop_speed;
        if(stopped&&std::abs(error)<=cfg.turn.yaw_tolerance){course_turn_active_=false;resetMotion();}
        else {
          if(!stopped)initializeYaw(actual_yaw);else stepYaw(course_turn_yaw_,dt);
          last_={};acceleration_={};Control u;u.target=current;u.yaw=yaw_;u.progress=progress_;
          u.heading_wait=true;u.course_error=error;u.alignment_active=true;u.turn_stop=true;
          u.turn_phase=stopped?"TURN_IN_PLACE":"TURN_BRAKE";u.turn_anchor_distance=0;
          u.blocked=!clear(current,current);return u;
        }
      }
    }
    bool waiting=false,alignment_active=false,alignment_forward=false;
    bool alignment_min_speed=false;
    double heading_speed_scale=1.;
    double course_error=0;
    // Heading depends on path geometry, never on a sonar/curvature speed cap.
    // A tiny (or zero) cap must still allow turning toward a clear escape route.
    if(std::hypot(geometry.x,geometry.y)>1e-5){
      // Yaw follows the reference geometry. Do not differentiate lateral
      // odometry error into a yaw feedforward rate on an otherwise straight path.
      double direction=std::atan2(geometry.y,geometry.x);
      course_error=wrap(direction-actual_yaw);
      // An abrupt opposite goal still requires braking along the observed
      // forward corridor. Ordinary bends can align yaw during deceleration.
      if(strict && std::abs(course_error)>pi/2 && vxy>.15)
        initializeYaw(actual_yaw);
      else stepYaw(direction,dt);
      if(strict){
        const double vertical_speed=desired.z;
        double e=std::abs(course_error);
        alignment_active=e>(cfg.heading_speed_scaling_enabled?
          cfg.heading_full_speed:cfg.heading_slow);
        heading_speed_scale=headingSpeedScale(cfg,e);
        waiting=e>=cfg.heading_stop;
        const double original_xy=std::hypot(desired.x,desired.y);
        desired.x*=heading_speed_scale;desired.y*=heading_speed_scale;
        if(alignment_active && cfg.heading_speed_scaling_enabled &&
           cfg.allow_forward_during_alignment && !waiting &&
           original_xy>1e-7){
          // Never exceed the unscaled command. This preserves every upstream
          // speed/clearance/goal-approach limit while preventing a zero crawl.
          const double floor=std::min(cfg.min_forward_speed,original_xy);
          const double scaled=std::hypot(desired.x,desired.y);
          if(scaled+1e-9<floor){
            desired.x*=floor/std::max(1e-9,scaled);
            desired.y*=floor/std::max(1e-9,scaled);
            alignment_min_speed=true;
          }
          alignment_forward=true;
        }
        // Do not climb diagonally through a turn while horizontal translation waits.
        if(waiting)desired.z=0;
        // Near-vertical geometry has an ill-conditioned XY heading: a tiny
        // horizontal stitch or odometry error can demand a large yaw change.
        // A separately observed, collision-checked vertical corridor may make
        // altitude progress while XY translation waits for frontal alignment.
        // This does not authorize an unchecked diagonal or bypass the ordinary
        // acceleration/jerk filter. Without the extra certificate, retain the
        // former heading wait behavior.
        const double horizontal=std::hypot(course.x,course.y);
        const Vec3 vertical_target{current.x,current.y,target.z};
        if(vertical_clear && std::abs(course.z)>.05 && std::abs(course.z)>3*horizontal &&
           e>.90*cfg.strict_forward_half_angle && vertical_speed*course.z>0 &&
           clear(current,vertical_target) && vertical_clear(current,vertical_target)){
          desired={0,0,vertical_speed};waiting=true;
          alignment_forward=false;alignment_min_speed=false;
        }
      }
    }
    if(heading_mode==HeadingMode::Relaxed){
      // Scale rather than independently clamp body axes: this keeps the motion
      // on its collision-checked geometric direction. Yaw continues aligning,
      // so the reduced speed naturally opens up as the vehicle turns forward.
      const double c=std::cos(actual_yaw),s=std::sin(actual_yaw);
      const double forward=c*desired.x+s*desired.y;
      const double lateral=-s*desired.x+c*desired.y;
      double factor=1.;
      if(std::abs(lateral)>1e-9)
        factor=std::min(factor,std::max(0.,cfg.relaxed_lateral_speed)/std::abs(lateral));
      if(forward< -1e-9)
        factor=std::min(factor,std::max(0.,cfg.relaxed_reverse_speed)/(-forward));
      desired=desired*factor;
      // Keep the veto authoritative even if later desired-motion constraints
      // evolve to rotate the vector. The node additionally checks braking.
      if(norm(desired)>1e-6 && !clear(current,current+desired*.35))desired={};
    }
    Vec3 command=smooth(desired,dt);
    // A filter with the previous route's velocity must not emit 1.3 m/s when
    // a new stop profile allows 0.2 m/s. This command cap requests braking;
    // measured inertia is still checked by the downstream stopping sweep.
    if(cfg.turn.enabled&&(rejoin_active_||next_corner<=progress_+horizon)){
      command=limitNorm(command,speed);appliedVelocity(command);
    }
    if(heading_mode==HeadingMode::Strict && !waiting && !alignment_forward &&
       std::abs(course_error)<cfg.heading_stop){
      // The jerk filter can retain an OLD lateral direction after the desired
      // correction has been constrained. Certify the actually emitted vector.
      // Remove only the excessive lateral component; never turn a reverse
      // request into forward motion. The complete braking sweep is checked by
      // the node after this local continuous collision check.
      const double c=std::cos(actual_yaw),s=std::sin(actual_yaw);
      const double forward=c*command.x+s*command.y;
      if(forward>0){
        const double side=-s*command.x+c*command.y;
        const double bound=forward*std::tan(.90*cfg.strict_forward_half_angle);
        const double limited=std::clamp(side,-bound,bound);
        Vec3 candidate{c*forward-s*limited,s*forward+c*limited,command.z};
        if(clear(current,current+candidate*.35))command=candidate;
      }
    }
    Control result{command,target,yaw_,progress_,false,false,waiting,speed,course_error,
      alignment_active,alignment_forward,alignment_min_speed,heading_speed_scale};
    for(size_t i=1;i+1<path_.size();++i)if(arc_[i]>=progress_&&arc_[i]<=progress_+horizon)
      result.turn_curvature=std::max(result.turn_curvature,
        turnSample(path_[i]-path_[i-1],path_[i+1]-path_[i],cfg.lateral_accel,cfg.yaw_rate).curvature);
    if(cfg.turn.enabled&&anchor&&next_corner_index<path_.size()){
      result.turn_stop=true;result.turn_phase="TURN_APPROACH";
      const size_t i=next_corner_index;
      result.turn_curvature=turnSample(path_[i]-path_[i-1],path_[i+1]-path_[i],cfg.lateral_accel,cfg.yaw_rate).curvature;
      result.turn_anchor_distance=distance(current,path_[i]);
    }
    if(rejoin_active_){result.turn_stop=true;result.turn_phase="TURN_REJOIN";
      result.turn_anchor_distance=distance(current,atArc(path_,arc_,rejoin_s_));}
    return result;
  }
  Control hold(Vec3 current,Vec3 anchor,Vec3 velocity,double dt){
    return positionHold(current,anchor,velocity,dt,1.,false);
  }
  // A bounded point-to-point command for short, deliberately simple staging
  // motions.  Unlike setPath()/update(), this does not replace or consume the
  // mission path.  It turns the vehicle toward the target, then flies straight
  // to it with the normal acceleration/jerk filters.
  Control directMove(Vec3 current,Vec3 anchor,Vec3 velocity,double dt,
                     double max_speed_xy,double measured_yaw){
    const Vec3 error=anchor-current;
    double factor=1.,course_error=0.;bool waiting=false;
    bool alignment_active=false;
    double heading_speed_scale=1.;
    if(std::hypot(error.x,error.y)>std::min(.025,.25*cfg.goal_tolerance)){
      const double desired_yaw=std::atan2(error.y,error.x);
      const double actual_yaw=std::isfinite(measured_yaw)?measured_yaw:yaw_;
      course_error=wrap(desired_yaw-actual_yaw);
      stepYaw(desired_yaw,dt);
      alignment_active=std::abs(course_error)>(cfg.heading_speed_scaling_enabled?
        cfg.heading_full_speed:cfg.heading_slow);
      heading_speed_scale=headingSpeedScale(cfg,std::abs(course_error));
      factor=heading_speed_scale;
      waiting=std::abs(course_error)>=cfg.heading_stop;
    }
    auto control=positionHold(current,anchor,velocity,dt,factor,waiting,
      std::clamp(max_speed_xy,0.,cfg.max_speed_xy));
    control.reached=distance(current,anchor)<cfg.goal_tolerance && norm(velocity)<.10;
    control.heading_wait=waiting;control.course_error=course_error;
    control.alignment_active=alignment_active;
    control.alignment_forward_enabled=false;
    control.heading_speed_scale=heading_speed_scale;
    return control;
  }
  double yaw()const{return yaw_;}
  double yawRate()const{return yaw_rate_;}
  Vec3 acceleration()const{return acceleration_;}
  double progress()const{return progress_;}
  Vec3 endpoint()const{return path_.empty()?Vec3{}:path_.back();}
  // Read-only preview for the direction/sensor policy, using the same progress
  // window and unpassed-corner rule as update. No path or motion state changes.
  Vec3 previewTarget(Vec3 current,Vec3 measured_velocity={})const{
    if(path_.empty())return current;
    if(path_.size()==1)return path_.front();
    if(rejoin_active_)return atArc(path_,arc_,rejoin_s_);
    auto pr=project(path_,arc_,current,std::max(0.,progress_-.10),
                    std::min(arc_.back(),progress_+2.0));
    double progress=progress_,next_corner=arc_.back()+1;
    for(size_t i=1;i+1<path_.size();++i){
      if(!corner_[i] || arc_[i]<=passed_corner_s_)continue;
      if(distance(current,path_[i])<.08 && norm(measured_velocity)<.10){
        progress=std::max(progress,arc_[i]+.01);continue;
      }
      if(spatiallyPastCorner(i,current,pr.s)){
        progress=std::max(progress,arc_[i]+.01);continue;
      }
      if(arc_[i]+.10<progress)continue;
      next_corner=arc_[i];break;
    }
    progress=std::min(next_corner,std::max(progress,pr.s));
    return atArc(path_,arc_,std::min({arc_.back(),progress+cfg.lookahead,next_corner}));
  }
  const std::vector<double>&speedProfile()const{return profile_;}
  Vec3 futurePoint(Vec3 current,double ahead)const {
    if(path_.empty())return current;
    auto p=project(path_,arc_,current,std::max(0.,progress_-.10),std::min(arc_.back(),progress_+2.));
    return atArc(path_,arc_,std::min(arc_.back(),std::max(progress_,p.s)+std::max(0.,ahead)));
  }
private:
  std::vector<Vec3>path_;
  std::vector<double>arc_,profile_;
  std::vector<bool>corner_;
  double progress_{0},yaw_{0},yaw_rate_{0},passed_corner_s_{-1};
  bool yaw_ready_{false},course_ready_{false},profile_strict_{true};
  double previous_course_{0},course_rate_{0};
  Vec3 last_{},acceleration_{};
  bool pivot_active_{false};Vec3 pivot_anchor_{},pivot_route_point_{};double pivot_yaw_{0};
  bool rejoin_active_{false};double rejoin_s_{0};
  bool course_turn_active_{false};double course_turn_yaw_{0};
  bool spatiallyPastCorner(size_t index,Vec3 current,double projected_s)const{
    if(index==0 || index+1>=path_.size() ||
       projected_s<=arc_[index]+.05)return false;
    Vec3 outgoing=path_[index+1]-path_[index];
    const double length=norm(outgoing);
    if(length<1e-6)return false;
    outgoing=outgoing*(1./length);
    const Vec3 offset=current-path_[index];
    const double along=dot(offset,outgoing);
    if(along<=.04)return false;
    const double lateral=norm(offset-outgoing*along);
    const double tolerance=std::max(.12,std::min(.30,.35*cfg.lookahead));
    return lateral<=tolerance;
  }
  Control positionHold(Vec3 current,Vec3 anchor,Vec3 velocity,double dt,double factor,bool waiting,
                       double max_speed_xy=.25){
    Vec3 e=anchor-current;
    Vec3 desired{cfg.kp_xy*e.x-cfg.kd_xy*velocity.x,
                 cfg.kp_xy*e.y-cfg.kd_xy*velocity.y,cfg.kp_z*e.z-cfg.kd_z*velocity.z};
    desired=limitXY(desired,std::min(max_speed_xy,cfg.max_speed_xy));
    desired.x*=factor;desired.y*=factor;
    desired.z=waiting?0:std::clamp(desired.z,-cfg.max_speed_z,cfg.max_speed_z);
    return {smooth(desired,dt),anchor,yaw_,progress_,distance(current,anchor)<cfg.goal_tolerance,false};
  }
  Control terminalHold(Vec3 current,Vec3 anchor,Vec3 velocity,double dt,double measured_yaw,bool strict){
    const Vec3 e=anchor-current;
    double factor=1.,error=0;bool waiting=false;
    // The direction guard may reject a lateral endpoint servo after a switch
    // to Strict. Keep aligning toward a still-unreached endpoint, otherwise the
    // final 18--30 cm can remain blocked forever with a fixed stale yaw. Once
    // inside the goal tolerance, position noise must not cause a turn-in-place.
    bool alignment_active=false;
    double heading_speed_scale=1.;
    if(strict && norm(e)>=cfg.goal_tolerance &&
       std::hypot(e.x,e.y)>std::min(.025,.25*cfg.goal_tolerance)){
      const double direction=std::atan2(e.y,e.x);
      const double actual_yaw=std::isfinite(measured_yaw)?measured_yaw:yaw_;
      error=wrap(direction-actual_yaw);stepYaw(direction,dt);
      alignment_active=std::abs(error)>(cfg.heading_speed_scaling_enabled?
        cfg.heading_full_speed:cfg.heading_slow);
      heading_speed_scale=headingSpeedScale(cfg,std::abs(error));
      factor=heading_speed_scale;
      waiting=std::abs(error)>=cfg.heading_stop;
    }
    auto u=positionHold(current,anchor,velocity,dt,factor,waiting);
    u.reached=distance(current,anchor)<cfg.goal_tolerance && norm(velocity)<.10;
    u.heading_wait=waiting;u.course_error=error;u.alignment_active=alignment_active;
    u.alignment_forward_enabled=alignment_active&&cfg.heading_speed_scaling_enabled&&
      cfg.allow_forward_during_alignment&&!waiting;
    u.heading_speed_scale=heading_speed_scale;
    return u;
  }
  void buildProfile(){
    profile_.assign(path_.size(),cfg.max_speed_xy);corner_.assign(path_.size(),false);
    if(path_.empty())return;
    profile_.back()=0;
    for(size_t i=1;i+1<path_.size();++i){
      Vec3 a=path_[i]-path_[i-1],b=path_[i+1]-path_[i];
      double la=norm(a),lb=norm(b);
      double angle=std::acos(std::clamp(dot(a,b)/(la*lb),-1.,1.));
      double curvature=2*std::sin(angle*.5)/std::max(.001,.5*(la+lb));
      if(curvature>1e-5)profile_[i]=std::min(profile_[i],std::sqrt(cfg.lateral_accel/curvature));
      // At steep climb/descent, yaw is not a useful speed parameter: horizontal
      // derivatives can approach zero while the 3D tangent remains smooth.
      // Keep spatial-curvature braking and z limits; the execution guard still
      // checks the actual XY heading or certifies a vertical-only alignment step.
      const bool steep=std::abs(a.z)>3*std::hypot(a.x,a.y) &&
                       std::abs(b.z)>3*std::hypot(b.x,b.y);
      if(profile_strict_ && !steep && std::hypot(a.x,a.y)>1e-6 && std::hypot(b.x,b.y)>1e-6){
        double k=std::abs(wrap(std::atan2(b.y,b.x)-std::atan2(a.y,a.x)))/std::max(.001,.5*(la+lb));
        if(k>1e-5)profile_[i]=std::min(profile_[i],.75*cfg.yaw_rate/k);
      }
      if(requiresExecutionStop(turnSample(a,b,cfg.lateral_accel,cfg.yaw_rate),cfg.turn,cfg.sharp_turn)){
        profile_[i]=0;corner_[i]=true;
      }
    }
    for(size_t i=path_.size()-1;i>0;--i)
      profile_[i-1]=std::min(profile_[i-1],std::sqrt(profile_[i]*profile_[i]+2*.7*cfg.max_accel_xy*(arc_[i]-arc_[i-1])));
  }
  Vec3 smooth(Vec3 desired,double dt){
    dt=std::clamp(dt,.001,.15);
    Vec3 want=(desired-last_)*4.0;
    want=limitXY(want,cfg.max_accel_xy);
    want.z=std::clamp(want.z,-cfg.max_accel_z,cfg.max_accel_z);
    Vec3 da=want-acceleration_;
    da=limitXY(da,cfg.max_jerk_xy*dt);
    da.z=std::clamp(da.z,-cfg.max_jerk_z*dt,cfg.max_jerk_z*dt);
    acceleration_=acceleration_+da;
    last_=last_+acceleration_*dt;
    return last_;
  }
  void stepYaw(double desired,double dt){
    if(!yaw_ready_)initializeYaw(desired);
    if(course_ready_){
      double raw=std::clamp(wrap(desired-previous_course_)/std::max(.001,dt),-cfg.yaw_rate,cfg.yaw_rate);
      double alpha=dt/(.12+dt);course_rate_+=alpha*(raw-course_rate_);
    }
    previous_course_=desired;course_ready_=true;
    // Small course-rate preview compensates yaw response lag on a continuous
    // bend. Bound the lead below the translation slowdown angle; a route change
    // still uses the normal heading gate and yaw acceleration/rate limits.
    double lead=std::clamp(course_rate_*cfg.yaw_preview_time,
                           -.75*cfg.heading_slow,.75*cfg.heading_slow);
    double e=wrap(desired+lead-yaw_);
    double rate=std::clamp((std::abs(e)<cfg.yaw_deadband?0:2.5*e)+course_rate_,-cfg.yaw_rate,cfg.yaw_rate);
    yaw_rate_+=std::clamp(rate-yaw_rate_,-cfg.yaw_accel*dt,cfg.yaw_accel*dt);
    yaw_=wrap(yaw_+yaw_rate_*dt);
  }
};
} // namespace fire_scout
