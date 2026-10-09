#!/usr/bin/env python3
"""Run actual follower map braking, measured peer call and final projection gate."""
import pathlib,subprocess,tempfile
ROOT=pathlib.Path(__file__).resolve().parents[1]
s=(ROOT/'src/safe_airfar_path_follower.cpp').read_text()
a=s.index('      auto brake_config=measured_brake_;');b=s.index('      // Test the swept braking corridor',a)
map_block=s[a:b]
a=s.index('    const Vec3 before_peer=u.velocity;');b=s.index('    if(state=="PLANNER_INPUT_PAUSED"',a)
final_block=s[a:b]
a=s.index('    last_peer_decision_=constrainPeerMotion(');b=s.index('    if(last_peer_decision_.limited)',a)
peer_call=s[a:b]
code=r'''
#include "fire_scout/measured_braking.hpp"
#include "fire_scout/peer_safety.hpp"
#include "fire_scout/grid.hpp"
#include "fire_scout/tracker.hpp"
#include <cassert>
#include <memory>
#include <string>
#include <iostream>
using namespace fire_scout;
struct Time{double t;double seconds()const{return t;}Time operator-(Time b)const{return {t-b.t};}};
struct Fixture{
  Vec3 current_{},velocity_{};std::shared_ptr<Grid>grid_;
  MeasuredBrakeConfig measured_brake_;PeerSafetyConfig peer_safety_config_;
  PeerSafetyDecision last_peer_decision_;PassageReservation last_passage_reservation_;
  int own_priority_{1};std::vector<PeerKinematicState>peers;
  bool measured_braking_risk_{false},altitude_return_active_{false},comfort_limited_{false};double measured_stop_distance_{0};
  Time now()const{return {10};}Time map_stamp_{9.9};
  enum class State{NAV};State state_{State::NAV};
  bool collisionClear(Vec3 a,Vec3 b)const{return altitude_return_active_?grid_->boundaryReturn(a,b):grid_->segmentFrom(a,b);}
  void applyPeerSafety(Control &u,std::string &){auto &control=u;
'''+peer_call+r'''
    if(last_peer_decision_.limited)u.velocity=last_peer_decision_.velocity;
  }
  void step(Control &u,std::string &effective){
'''+map_block+final_block+r'''
  }
};
int main(){
  GridConfig cfg;cfg.resolution=.1;cfg.inflation_xy=.469063;cfg.inflation_z=.27;
  Fixture f;f.grid_=std::make_shared<Grid>(cfg);KeySet wall;
  for(int y=-40;y<200;++y)for(int z=10;z<20;++z)wall.insert({50,y,z});
  f.grid_->update(wall,{});f.current_={5-.469063-.406235,0,1.5};
  f.velocity_={.15,1.9767,0};Control u;u.velocity={.14,1.77,0};std::string state="FOLLOW_BSPLINE";
  f.step(u,state);assert(!f.measured_braking_risk_&&norm(u.velocity)>.01&&norm(u.velocity)<1.77);
  assert(state=="COMFORT_SPEED_LIMIT");
  f.current_.x=5-.469063-.20;u.velocity={.14,1.77,0};
  f.step(u,state);assert(f.measured_braking_risk_&&norm(u.velocity)<1e-8);
  assert(state=="MEASURED_BRAKING_RISK");
  // Regression from 071201: 2 cm/s drift must not zero safe tangential/away commands.
  f.current_.x=5-.469063-.18;
  for(double drift:{.0,.016,.02,.049,.051,.06}){
    f.velocity_={drift,0,0};u.velocity={0,.1,0};f.step(u,state);assert(u.velocity.y>.099);
    u.velocity={-.1,0,0};f.step(u,state);assert(u.velocity.x<-.099);
  }
  // A narrow but straight corridor keeps useful tangential motion available.
  f.current_.x=5-.469063-.12;f.velocity_={0,1.8,0};u.velocity=f.velocity_;
  f.step(u,state);assert(!f.measured_braking_risk_&&u.velocity.y>1.79);
  // Actual source call forwards the odometry velocity even for a small command.
  f.grid_->update({},{});f.current_={0,0,1.5};f.velocity_={1.861,0,0};
  f.peer_safety_config_.brake_accel=.65;f.peer_safety_config_.reaction_time=.8;
  f.peers={{{2.726,0,1.5},{},4,0,true}};u.velocity={.3,0,0};
  f.step(u,state);assert(f.last_peer_decision_.measured_braking_risk&&u.velocity.x==0);
  // The actual production call also protects a pair before entering the same
  // altitude band; this must reach the published control, including z.
  f.current_={0,0,2};f.velocity_={1,0,.25};f.peers={{{1,.3,2.8},{0,0,-.15},2,0,true}};
  u.velocity={1,0,.25};f.step(u,state);
  assert(f.last_peer_decision_.predicted_vertical_peers==1&&u.velocity.z==0);
  f.current_={0,0,1.5};
  // Projection away from a diagonal peer creates sideways motion into a wall.
  // The final production map gate must recertify it and brake.
  wall.clear();for(int x=-10;x<100;++x)for(int z=10;z<20;++z)wall.insert({x,-8,z});
  f.grid_->update(wall,{});f.velocity_={};f.peers={{{1,1,1.5},{},4,0,true}};
  u.velocity={1.8,0,0};f.step(u,state);
  assert(f.last_peer_decision_.velocity.y<-.01);
  assert(norm(u.velocity)<norm(f.last_peer_decision_.velocity));
  assert(state=="PEER_PROJECTED_MAP_BRAKE");
  f.grid_->update({},{});f.peers.clear();f.current_={0,0,.60};f.velocity_={};
  f.altitude_return_active_=true;u.velocity={0,0,.1};f.step(u,state);
  assert(u.velocity.z>.099); // Bounded altitude return keeps its original proof.
  std::cout<<"Actual follower measured map/peer braking and projection map recertification passed\n";
}
'''
with tempfile.TemporaryDirectory(prefix='measured-braking-node-') as temp:
 p=pathlib.Path(temp);(p/'test.cpp').write_text(code)
 subprocess.run(['g++','-std=c++17','-O2','-Wall','-Wextra','-Wpedantic','-UNDEBUG','-I'+str(ROOT/'include'),str(p/'test.cpp'),'-o',str(p/'test')],check=True)
 subprocess.run([str(p/'test')],check=True)
