#!/usr/bin/env python3
"""Compile the actual recovery-to-route arbitration with transport stubs.
The path certificate, tracker and stable-handoff timer are production code.
"""
import pathlib
import subprocess
import tempfile
root=pathlib.Path(__file__).resolve().parents[1]
source=(root/'src/safe_airfar_path_follower.cpp').read_text()
start=source.index('    if(recovery_.active() && !separating_active_)',source.index('  bool runRecovery('))
end=source.index('    if(!recovery_.active() && (!have_mission_goal_',start)
branch=source[start:end]
code=r'''
#include "fire_scout/tracker.hpp"
#include "fire_scout/navigation_contract.hpp"
#include "fire_scout/local_recovery.hpp"
#include <cassert>
#include <memory>
#include <string>
#include <iostream>
using namespace fire_scout;
struct Time{double t;double seconds()const{return t;}friend Time operator-(Time a,Time b){return {a.t-b.t};}};
struct Fixture{
  double t{100};Time now()const{return {t};}Time path_stamp_{100};double path_timeout_{5};
  bool separating_active_{false},have_path_{true},permit{true},clear{true},hard_clear{true};
  double yaw_current_{0};
  bool ended{false},handoff_ready_{false};double last_handoff_check_{-1};
  StableRouteHandoff handoff_;std::vector<Vec3>pending_path_{{0,0,3},{20,0,3}};
  struct Channel{size_t generation{1};}route_channel_;
  size_t recovery_entry_generation_{1};Vec3 recovery_entry_position_{0,0,3},current_{0,0,3},velocity_{};
  std::unique_ptr<Tracker>tracker_{std::make_unique<Tracker>()};
  struct Recovery{bool running{true};bool active()const{return running;}
    bool step(double,Vec3,bool){return running;}void cancel(){running=false;}}recovery_;
  std::string state;Control emitted{};
  Fixture(){tracker_->setPath(pending_path_,current_);tracker_->initializeYaw(0);}
  bool plannerAllowed()const{return permit;}
  bool observedMapMotion(Vec3,Vec3)const{return clear&&hard_clear;}
  bool collisionClear(Vec3,Vec3)const{return hard_clear;}
  bool detectedGlassRisk(Vec3,Vec3)const{return false;}
  void endRecovery(){ended=true;}
  void publish(Control u,std::string s){emitted=u;state=s;}
  bool check(double dt=.025){
''' + branch + r'''
    return false;
  }
  void ticks(int n){for(int i=0;i<n;++i){t+=.1;path_stamp_={t};check();}}
};
int main(){
  Fixture same;same.ticks(20);assert(!same.ended&&same.recovery_.active());
  // Identical path heartbeats cannot immediately cancel an unmoved escape.
  same.current_.x=.36;same.ticks(8);assert(same.ended&&!same.recovery_.active());
  Fixture revised;++revised.route_channel_.generation;
  revised.velocity_={.1,0,0};revised.ticks(8);
  assert(!revised.ended&&revised.state=="RECOVERY_HANDOFF_BRAKE");
  revised.velocity_={};revised.ticks(3);assert(revised.ended);
  Fixture blocked;++blocked.route_channel_.generation;blocked.clear=false;
  blocked.ticks(20);assert(!blocked.ended&&!blocked.handoff_ready_);
  Fixture stale;++stale.route_channel_.generation;stale.permit=false;
  stale.ticks(20);assert(!stale.ended);
  // A changed route freezes an ongoing escape BEFORE observation settles,
  // then can certify a wider actual-pose rejoin. No translation is emitted
  // just because the new planner line is geometrically clear.
  Fixture delayed;++delayed.route_channel_.generation;delayed.current_.y=.55;
  delayed.velocity_={.2,0,0};delayed.clear=false;delayed.ticks(2);
  assert(delayed.state=="RECOVERY_HANDOFF_BRAKE"&&!delayed.ended);
  delayed.velocity_={};delayed.ticks(5);
  assert(delayed.state=="RECOVERY_HANDOFF_OBSERVE"&&!delayed.ended);
  assert(norm(delayed.emitted.velocity)==0&&!delayed.emitted.reached);
  delayed.clear=true;delayed.ticks(5);assert(delayed.ended);
  Fixture physical_block;++physical_block.route_channel_.generation;
  physical_block.hard_clear=false;physical_block.ticks(20);assert(!physical_block.ended);
  // The same committed route may be 12 metres past its original start.
  Fixture long_route;
  for(double x=0;x<=12;x+=.1)long_route.tracker_->update({x,0,3},{.5,0,0},.025,
    [](Vec3,Vec3){return true;},0);
  long_route.recovery_entry_position_={12,0,3};long_route.current_={11.6,0,3};
  long_route.ticks(8);assert(long_route.ended);
  std::cout<<"turn_recovery_node_test: PASS actual recovery handoff, no immediate same-route cancellation, measured braking, collision/permit veto, bounded long-route projection\n";
}
'''
with tempfile.TemporaryDirectory(prefix='turn-recovery-node-') as temp:
    p=pathlib.Path(temp);(p/'test.cpp').write_text(code)
    subprocess.run(['g++','-std=c++17','-O2','-Wall','-Wextra','-Wpedantic','-UNDEBUG',
        '-I'+str(root/'include'),str(p/'test.cpp'),'-o',str(p/'test')],check=True)
    subprocess.run([str(p/'test')],check=True)
