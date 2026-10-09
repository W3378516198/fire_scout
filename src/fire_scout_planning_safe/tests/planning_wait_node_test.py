#!/usr/bin/env python3
"""Exercise the production NAV wait/recovery arbitration with transport stubs."""
import pathlib
import subprocess
import tempfile

root = pathlib.Path(__file__).resolve().parents[1]
source = (root / 'src/safe_airfar_path_follower.cpp').read_text()
start = source.index('    const double local_clearance=', source.index('    const bool missing_path='))
end = source.index('    holding_ = false;', start)
branch = source[start:end]
code = r'''
#include "fire_scout/planning_wait.hpp"
#include <cassert>
#include <memory>
#include <string>
#include <iostream>
using namespace fire_scout;
struct Clock{double t;double seconds()const{return t;}};
struct Map{double room{.6};double clearanceAt(Vec3)const{return room;}};
struct Fixture{
  PlanningWait planning_wait_;double ros{10},wall{100};
  Clock now()const{return {ros};}double steadySeconds()const{return wall;}
  Vec3 current_{};std::shared_ptr<Map>grid_{std::make_shared<Map>()},sonar_collision_{std::make_shared<Map>()};
  struct Brake{double reserve{.25};}measured_brake_;
  struct Channel{struct Gate{struct Permit{bool searching{true};}permit;}gate;}route_channel_;
  struct Campaign{bool exhausted()const{return false;}}campaign_;
  bool missing_path{true},recovery_requested_{false},peer_deadlock_recovery_{false},autonomous_recovery_{true};
  bool active{false},needed{false},called{false},tracking{false};std::string state;
  bool runRecovery(double,bool n){needed=n;called=true;return active;}
  void hold(double,const std::string&s){state=s;}
  void tick(double dt=.025){state.clear();tracking=called=false;
''' + branch + r'''
    tracking=true;
  }
};
int main(){
  Fixture safe;safe.tick();assert(safe.state=="PLANNING_WAIT_HOLD"&&!safe.needed&&safe.called);
  safe.ros+=20;safe.wall+=20;safe.tick();
  assert(safe.state=="PLANNING_WAIT_HOLD"&&!safe.needed);
  safe.grid_->room=.20;safe.tick();assert(safe.needed&&safe.state=="WAIT_PATH_HOLD");
  Fixture active;active.active=true;active.tick();
  assert(active.called&&!active.needed&&active.state.empty()); // existing recovery must check its new-route handoff
  Fixture explicit_stall;explicit_stall.recovery_requested_=true;explicit_stall.tick();assert(explicit_stall.needed);
  Fixture missing_search;missing_search.route_channel_.gate.permit.searching=false;missing_search.tick();
  missing_search.ros+=2.1;missing_search.wall+=2.1;missing_search.tick();assert(missing_search.needed);
  Fixture route;route.tick();route.missing_path=false;route.tick();
  assert(route.tracking&&!route.needed&&route.planning_wait_.reason=="IDLE");
  std::cout<<"planning_wait_node_test: PASS actual NAV wait, recovery handoff service, explicit stall and route resumption\n";
}
'''
with tempfile.TemporaryDirectory(prefix='planning-wait-node-') as temp:
    p = pathlib.Path(temp)
    (p / 'test.cpp').write_text(code)
    subprocess.run(['g++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-Wpedantic', '-UNDEBUG',
                    '-I' + str(root / 'include'), str(p / 'test.cpp'), '-o', str(p / 'test')], check=True)
    subprocess.run([str(p / 'test')], check=True)
