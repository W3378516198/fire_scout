#!/usr/bin/env python3
"""Compile the real follower progress arbitration with transport-only stubs."""
import pathlib
import subprocess
import tempfile

ROOT = pathlib.Path(__file__).resolve().parents[1]
source = (ROOT / 'src/safe_airfar_path_follower.cpp').read_text()
start = source.index('    const bool profile_limited=')
end = source.index('    const double direction_dt=', start)
arbitration = source[start:end]
code = r'''
#include "fire_scout/tracker.hpp"
#include "fire_scout/profile_progress.hpp"
#include <cassert>
#include <memory>
#include <string>
#include <iostream>
#define RCLCPP_WARN_THROTTLE(...) do{}while(0)
using namespace fire_scout;
std::string textMessage(const std::string &s){return s;}
struct Publisher{std::vector<std::string>events;void publish(std::string s){events.push_back(s);}};
struct Grid{bool clear{true};bool segment(Vec3,Vec3){return clear;}};
struct Fixture{
  ProfileProgressMonitor profile_progress_,turn_progress_;MotionProgressWatchdog progress_watchdog_;
  ExecutionProgressMonitor execution_progress_;
  Vec3 current_{0,0,3};double yaw_current_{0},map_speed_factor_{1};
  std::vector<Vec3>pending_path_{{0,0,3},{10,0,3}};
  struct GapConfig{bool enabled{true};}gap_config_;
  struct GapState{double speed{1.8};}gap_state_;
  struct Sonar{double speed_limit{1.8};}sonar;
  struct PeerDecision{bool limited{false};}last_peer_decision_;
  bool glass_risk_active_{false},measured_braking_risk_{false},have_map_{true};
  bool recovery_requested_{false};std::string progress_cause_,direction_reason_{"ALLOWED"};
  std::shared_ptr<Grid>grid_{std::make_shared<Grid>()};
  std::shared_ptr<Publisher>replan_pub_{std::make_shared<Publisher>()};
  void step(Control u,std::string state="FOLLOW_BSPLINE",std::string effective="FOLLOW_BSPLINE",
            bool can_request=true,double watch_dt=.025){
    const bool navigation_motion=!u.blocked&&!u.reached;
''' + arbitration + r'''
  }
};
int main(){
  Control u;u.speed_limit=.04;u.velocity={.04,0,0};
  Fixture crawl;
  for(int i=0;i<=1200;++i){crawl.current_.x=.04*.025*i;crawl.step(u);assert(!crawl.recovery_requested_);}
  assert(crawl.replan_pub_->events.size()==15);
  for(const auto &event:crawl.replan_pub_->events)assert(event=="PROFILE_LIMITED_PROGRESS");
  assert(crawl.progress_cause_=="PROFILE_LIMITED");

  Fixture frozen;
  for(int i=0;i<=320;++i){frozen.step(u);if(i<320)assert(!frozen.recovery_requested_);}
  assert(frozen.recovery_requested_&&frozen.progress_cause_=="PROFILE_COMMAND_NOT_FOLLOWED");

  // A repair just before the profile grace expires must receive a fresh
  // ordinary response deadline, not inherit its suppressed 8-second timeout.
  Fixture repaired;Control zero;zero.speed_limit=0;
  for(int i=0;i<320;++i){repaired.step(zero);assert(!repaired.recovery_requested_);}
  zero.speed_limit=1.8;
  repaired.step(zero);assert(!repaired.recovery_requested_);
  for(int i=0;i<70;++i){repaired.step(zero);assert(!repaired.recovery_requested_);}
  for(int i=0;i<12;++i)repaired.step(zero);
  assert(repaired.recovery_requested_); // True non-response still expires.

  // An external guard has the same low final speed but requires the existing
  // ordinary blocked-response policy. This must not acquire the profile grace.
  Fixture guard;guard.direction_reason_="COLLISION";u.velocity={};
  for(int i=0;i<=81;++i)guard.step(u,"FOLLOW_BSPLINE","DIRECTION_COLLISION");
  assert(guard.recovery_requested_&&guard.progress_cause_=="SAFETY_GUARD");
  for(const auto &event:guard.replan_pub_->events)assert(event!="PROFILE_LIMITED_PROGRESS");

  Fixture turn;u.heading_wait=true;u.alignment_active=true;
  for(int i=0;i<240;++i){turn.yaw_current_=.4*.025*i;turn.step(u,"ROTATE_TO_PATH","ROTATE_TO_PATH");}
  assert(!turn.recovery_requested_&&turn.progress_cause_=="HEADING_ALIGNMENT");
  for(int i=240;i<=322;++i){turn.yaw_current_=.4*.025*i;turn.step(u,"ROTATE_TO_PATH","ROTATE_TO_PATH");}
  assert(turn.recovery_requested_); // Endless rotation remains bounded.

  Fixture endpoint;u.heading_wait=u.alignment_active=false;
  endpoint.current_={9.9,0,3};
  for(int i=0;i<=81;++i)endpoint.step(u);
  assert(endpoint.recovery_requested_&&endpoint.profile_progress_.seconds()==0);


  Fixture pivot;Control pivot_u;pivot_u.turn_stop=true;pivot_u.turn_phase="TURN_REJOIN_BLOCKED";
  for(int i=0;i<320;++i){pivot.step(pivot_u,"TURN_REJOIN_BLOCKED","TURN_REJOIN_BLOCKED");
    assert(!pivot.recovery_requested_);}
  assert(std::find(pivot.replan_pub_->events.begin(),pivot.replan_pub_->events.end(),"TURN_EXECUTION_REPAIR")!=pivot.replan_pub_->events.end());
  pivot.step(pivot_u,"TURN_REJOIN_BLOCKED","TURN_REJOIN_BLOCKED");assert(pivot.recovery_requested_);
  Fixture normal_pivot;
  for(int i=0;i<60;++i)normal_pivot.step(pivot_u,"TURN_IN_PLACE","TURN_IN_PLACE");
  Control depart;depart.velocity={.4,0,0};depart.speed_limit=1.;
  normal_pivot.step(depart);assert(normal_pivot.turn_progress_.seconds()==0&&!normal_pivot.recovery_requested_);
  Fixture queued;
  for(int i=0;i<=1000;++i)queued.step(u,"FOLLOW_BSPLINE","PEER_PASSAGE_YIELD",false);
  assert(!queued.recovery_requested_&&queued.replan_pub_->events.empty());
  std::cout<<"Actual follower progress arbitration regressions passed\n";
}
'''
with tempfile.TemporaryDirectory(prefix='profile-progress-node-') as temp:
    cpp = pathlib.Path(temp) / 'test.cpp'
    binary = pathlib.Path(temp) / 'test'
    cpp.write_text(code)
    subprocess.run(['g++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-Wpedantic',
                    '-UNDEBUG', '-I' + str(ROOT / 'include'), str(cpp), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
