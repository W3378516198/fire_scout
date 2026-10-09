#include "fire_scout/profile_progress.hpp"
#include "fire_scout/tracker.hpp"
#include <cassert>
#include <iostream>

using namespace fire_scout;

int main() {
  ProfileProgressMonitor monitor;
  assert(monitor.cfg.valid());
  monitor.cfg.recovery_grace=3;
  assert(!monitor.cfg.valid());
  monitor.cfg.recovery_grace=8;

  // Reproduce the actual command-vs-watchdog contradiction: .04 m/s needs
  // 2.5 seconds for 10 cm, but the ordinary watchdog expires after 2 seconds.
  MotionProgressWatchdog ordinary;
  bool ordinary_fired=false;int repairs=0;
  for(int i=0;i<=1200;++i) {
    const double t=i*.025;
    Vec3 position{.04*t,0,3};
    ordinary_fired|=ordinary.update(position,0,.025,true);
    const auto result=monitor.update(true,position,{.04,0,0},.025);
    repairs+=result.request_repair;
    assert(result.limited && !result.request_recovery);
    if(i<80)assert(!result.request_repair);
  }
  assert(ordinary_fired && repairs==15);

  // If the same nonzero command is not executed, profile classification must
  // not permanently suppress certified recovery.
  monitor.reset();bool failed=false;
  for(int i=0;i<=320;++i) {
    auto result=monitor.update(true,{0,0,3},{.04,0,0},.025);
    if(i<320)assert(!result.request_recovery);
    if(result.request_recovery) {
      failed=true;assert(std::string(result.reason)=="PROFILE_COMMAND_NOT_FOLLOWED");
    }
  }
  assert(failed);

  // A zero command also receives a bounded geometry-repair grace, then the
  // usual fully certified recovery. No velocity floor is introduced.
  monitor.reset();failed=false;
  for(int i=0;i<=320;++i) {
    auto result=monitor.update(true,{0,0,3},{},.025);
    if(result.request_recovery) {
      failed=true;assert(std::string(result.reason)=="PROFILE_REPAIR_TIMEOUT");
    }
  }
  assert(failed);

  // Position oscillations cannot masquerade as useful command response.
  monitor.reset();failed=false;
  for(int i=0;i<=320;++i) {
    auto result=monitor.update(true,{.005*std::sin(i*.1),0,3},{.04,0,0},.025);
    failed|=result.request_recovery;
  }
  assert(failed);

  // Episode end and clock rewind discard stale deadlines; a paused sim clock
  // does not consume either repair or recovery grace.
  auto stopped=monitor.update(false,{},{},.025);
  assert(!stopped.limited && monitor.seconds()==0);
  monitor.update(true,{},{},.025);
  for(int i=0;i<1000;++i) {
    auto paused=monitor.update(true,{},{},0);
    assert(!paused.request_repair && !paused.request_recovery);
  }
  monitor.update(true,{},{},-1);
  assert(monitor.seconds()==0);
  std::cout<<"Profile-progress monitoring regressions passed\n";
}
