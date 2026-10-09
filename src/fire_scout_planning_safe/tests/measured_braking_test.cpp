#include "fire_scout/measured_braking.hpp"
#include "fire_scout/peer_safety.hpp"
#include "fire_scout/grid.hpp"
#include <cassert>
#include <iostream>
using namespace fire_scout;
int main(){
  MeasuredBrakeConfig c;Vec3 p{0,0,1.5};
  auto wall=[](Vec3 a,Vec3 b){return a.x<3&&b.x<3;};
  // New command nearly zero, but measured UAV still flies at 1.9 m/s.
  auto lag=measuredStoppingVelocity(p,{.2,0,0},{1.9,0,0},c,wall);
  assert(lag.momentum_risk&&lag.velocity.x==0);
  auto clear=measuredStoppingVelocity(p,{1.8,0,0},{1.8,0,0},c,[](Vec3,Vec3){return true;});
  assert(!clear.limited&&clear.velocity.x==1.8);
  auto away=measuredStoppingVelocity(p,{-.15,0,0},{1.9,0,0},c,wall);
  assert(away.momentum_risk&&away.velocity.x<0); // Braking escape stays available.
  auto turn=measuredStoppingVelocity(p,{0,1,0},{1.9,0,0},c,wall);
  assert(turn.momentum_risk); // A perpendicular command is not evidence of stopping.
  // Closed-loop delayed first-order velocity response toward a fixed wall.
  // The wall margin is 25 cm beyond the .469063 m hard body envelope.
  double x=0,v=1.8,min_clearance=1e9;int stopped=0;
  for(int i=0;i<1600;++i){
    auto d=measuredStoppingVelocity({x,0,1.5},{1.8,0,0},{v,0,0},c,
      [](Vec3 a,Vec3 b){return a.x<8-.469063-.25&&b.x<8-.469063-.25;});
    v+=std::clamp((d.velocity.x-v)/.65,-.65,1.5)*.02;x+=v*.02;
    min_clearance=std::min(min_clearance,8-.469063-x);
    if(v<.02)++stopped;
  }
  assert(min_clearance>.20&&stopped>100);
  PeerSafetyConfig pc;pc.reaction_time=.8;pc.brake_accel=.65;pc.braking_reserve=.2;
  std::vector<PeerKinematicState> peers{{{2.726,0,1.5},{},4,0,true}};
  Vec3 measured{1.861,0,0};
  auto peer=constrainPeerMotion(p,{1.385,0,0},3,peers,pc,{},&measured);
  assert(peer.measured_braking_risk&&peer.limited&&peer.velocity.x<1e-6);
  assert(peer.required_braking_clearance>peers[0].position.x-pc.hard_separation);
  // Rearward peer does not brake a UAV moving away in clear space.
  peers[0].position={-4,0,1.5};measured={1.8,0,0};
  peer=constrainPeerMotion(p,measured,3,peers,pc,{},&measured);
  assert(!peer.limited&&peer.velocity.x==1.8);
  // Approach a stopped leader from 10 m, with delayed measured response.
  peers[0].position={10,0,1.5};x=0;v=1.8;double nearest=10;int hard=0;
  for(int i=0;i<1800;++i){
    measured={v,0,0};auto d=constrainPeerMotion({x,0,1.5},{1.8,0,0},3,peers,pc,{},&measured);
    v+=std::clamp((d.velocity.x-v)/.65,-.65,1.5)*.02;x+=v*.02;
    nearest=std::min(nearest,10-x);hard+=d.reason==PeerSafetyReason::HARD_SEPARATION;
  }
  assert(hard==0&&nearest>pc.hard_separation+.10);
  std::cout<<"measured braking: delayed wall and stopped leader; minimum wall clearance="<<min_clearance<<" peer separation="<<nearest<<" passed\n";
}
