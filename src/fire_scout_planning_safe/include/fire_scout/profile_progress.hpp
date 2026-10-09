#pragma once
#include "geometry.hpp"

namespace fire_scout {

// A geometric speed cap is not evidence of a blocked flight controller.  Give
// the planner a bounded opportunity to repair it while retaining independent
// command-response and zero-motion checks.  This policy never raises a speed.
struct ProfileProgressConfig {
  double speed_threshold{.12}, repair_delay{2.0}, repair_repeat{2.0};
  double recovery_grace{8.0};
  bool valid() const {
    return std::isfinite(speed_threshold) && speed_threshold>0 && speed_threshold<=.30 &&
      std::isfinite(repair_delay) && repair_delay>0 &&
      std::isfinite(repair_repeat) && repair_repeat>0 &&
      std::isfinite(recovery_grace) && recovery_grace>=repair_delay+repair_repeat;
  }
};

struct ProfileProgressDecision {
  bool limited{false}, request_repair{false}, request_recovery{false};
  const char *reason{"NORMAL_PROGRESS"};
};

class ProfileProgressMonitor {
public:
  ProfileProgressConfig cfg;
  void reset() {
    active_=false;elapsed_=window_time_=expected_=displacement_=0;
    next_report_=cfg.repair_delay;
  }
  ProfileProgressDecision update(bool limited,Vec3 position,Vec3 sent_velocity,double dt) {
    if(!limited || !finite(position) || !finite(sent_velocity) ||
       !std::isfinite(dt) || dt<0) {reset();return {};}
    ProfileProgressDecision result;result.limited=true;result.reason="PROFILE_LIMITED";
    if(!active_) {
      active_=true;anchor_=position;next_report_=cfg.repair_delay;
      return result;
    }
    elapsed_+=dt;window_time_+=dt;expected_+=norm(sent_velocity)*dt;
    displacement_=distance(position,anchor_);
    if(elapsed_+1e-9>=next_report_) {
      result.request_repair=true;next_report_=elapsed_+cfg.repair_repeat;
    }
    if(window_time_+1e-9>=cfg.recovery_grace) {
      // A .04 m/s crawl normally makes .32 m in this window: it requests a
      // profile repair, not a retreat. A frozen vehicle under that same command
      // must still recover. Net displacement also exposes repeated oscillation.
      const bool no_response=expected_>=.10 &&
        displacement_<std::max(.04,.25*expected_);
      const bool no_command=expected_<.10 && displacement_<.03;
      if(no_response || no_command) {
        result.request_recovery=true;
        result.reason=no_response?"PROFILE_COMMAND_NOT_FOLLOWED":"PROFILE_REPAIR_TIMEOUT";
      }else {
        anchor_=position;window_time_=expected_=0;
      }
    }
    return result;
  }
  double seconds() const {return elapsed_;}
  double expectedDistance() const {return expected_;}
  double displacement() const {return displacement_;}
private:
  bool active_{false};Vec3 anchor_{};
  double elapsed_{0},window_time_{0},expected_{0},displacement_{0},next_report_{2};
};

// One campaign across FOLLOW/ALIGN/TURN and path revisions. State-specific
// monitors below may pause or restart; they must not erase this net-motion
// deadline. External safety/peer queues pause it, never authorize translation.
struct ExecutionProgressConfig {
  double useful_distance{.25},repair_delay{2.0},repair_repeat{2.0},recovery_grace{8.0};
  bool valid()const{
    return std::isfinite(useful_distance)&&useful_distance>=.10&&useful_distance<=1.&&
      std::isfinite(repair_delay)&&repair_delay>0&&std::isfinite(repair_repeat)&&repair_repeat>0&&
      std::isfinite(recovery_grace)&&recovery_grace>=repair_delay+repair_repeat;
  }
};
class ExecutionProgressMonitor {
public:
  ExecutionProgressConfig cfg;
  void reset(){active_=repair_seen_=departure_granted_=false;elapsed_=displacement_=extension_=0;next_report_=cfg.repair_delay;}
  ProfileProgressDecision update(bool eligible,Vec3 position,double dt,bool departure=false){
    if(!finite(position)||!std::isfinite(dt)||dt<0){reset();return {};}
    if(active_){
      displacement_=distance(position,anchor_);
      if(displacement_>=cfg.useful_distance)reset();
    }
    if(!eligible)return {}; // A real safety queue pauses the campaign.
    ProfileProgressDecision out;out.reason="EXECUTION_PROGRESS_MONITORING";
    if(!active_){active_=true;anchor_=position;next_report_=cfg.repair_delay;return out;}
    elapsed_+=dt;
    // A departure after repair gets ONE response interval, not a fresh
    // campaign whenever FOLLOW briefly replaces TURN or ALIGN.
    if(departure&&repair_seen_&&!departure_granted_){extension_=cfg.repair_repeat;departure_granted_=true;}
    if(elapsed_+1e-9>=next_report_){
      out.request_repair=true;repair_seen_=true;next_report_=elapsed_+cfg.repair_repeat;
      out.reason="EXECUTION_LOCAL_STAGNATION";
    }
    if(elapsed_+1e-9>=cfg.recovery_grace+extension_){
      out.request_recovery=true;out.reason="EXECUTION_PROGRESS_TIMEOUT";
    }
    return out;
  }
  double seconds()const{return elapsed_;}
  double displacement()const{return displacement_;}
private:
  bool active_{false},repair_seen_{false},departure_granted_{false};Vec3 anchor_{};
  double elapsed_{0},displacement_{0},extension_{0},next_report_{2.};
};

}  // namespace fire_scout
