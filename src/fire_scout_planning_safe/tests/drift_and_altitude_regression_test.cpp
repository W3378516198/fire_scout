#include "fire_scout/measured_braking.hpp"
#include "fire_scout/peer_safety.hpp"
#include "fire_scout/grid.hpp"
#include <cassert>
#include <iostream>
using namespace fire_scout;
int main(){
 GridConfig gc;gc.resolution=.1;gc.inflation_xy=.469063;gc.recovery_xy=.429063;
 Grid g(gc);KeySet wall;for(int y=-60;y<=60;++y)for(int z=10;z<20;++z)wall.insert({10,y,z});g.update(wall,{});
 Vec3 p{1-gc.inflation_xy-.18,0,1.5};MeasuredBrakeConfig c;c.response_time=.918;
 auto hard=[&](Vec3 a,Vec3 b){return g.segmentFrom(a,b);};
 auto preferred=[&](Vec3 a,Vec3 b){return g.segmentWithMargin(a,b,.175);};
 // Both sides of the former 0.05 m/s discontinuity, including log velocities.
 for(double v:{0.,.016,.02,.0337,.047,.0499,.0501,.06}){
  for(Vec3 requested:{Vec3{0,.1,0},Vec3{-.1,0,0}}){
   auto d=measuredStoppingVelocity(p,requested,{v,0,0},c,hard,preferred);
   assert(distance(d.velocity,requested)<1e-6&&!d.hard_limited);
  }
 }
 auto soft=measuredStoppingVelocity(p,{.4,0,0},{.01,0,0},c,hard,preferred);
 assert(soft.velocity.x>0&&soft.velocity.x<=.20001); // Soft deficit cannot force a stop.
 auto collision=measuredStoppingVelocity(p,{1.8,0,0},{1.8,0,0},c,hard,preferred);
 assert(collision.momentum_risk&&norm(collision.velocity)==0);
 // A stationary certified envelope escape must not be rejected because the
 // zero-length delayed leg itself cannot demonstrate strictly improving clearance.
 p.x=1-.45;
 assert(!g.segment(p,p)&&g.segmentFrom(p,p+Vec3{-.1,0,0}));
 auto escape=measuredStoppingVelocity(p,{-.1,0,0},{},c,hard);
 assert(escape.velocity.x<-.099);
 // Vertical convergence: old instantaneous z filter ignored this near pair.
 PeerSafetyConfig pc;pc.hard_separation=.938126;pc.vertical_separation=.544;
 pc.reaction_time=.8;pc.brake_accel=.65;
 p={0,0,2.0};Vec3 measured{1.,0,.25};
 std::vector<PeerKinematicState> peers{{{1.,.3,2.8},{0,0,-.15},2,0,true}};
 auto d=constrainPeerMotion(p,{1.,0,.25},1,peers,pc,{},&measured);
 assert(d.predicted_vertical_peers==1&&d.vertical_limited&&d.velocity.z==0);
 assert(d.nearest_distance<1.1&&d.limited);
 // A genuinely separated, non-converging altitude layer remains free.
 peers[0].position.z=4.;peers[0].velocity={};measured={1.,0,0};
 d=constrainPeerMotion(p,measured,1,peers,pc,{},&measured);
 assert(!d.limited&&std::isinf(d.nearest_distance));
 // Delayed vertical response of two opposing UAVs, already overlapping in XY.
 Vec3 a{0,0,2.0},b{.6,0,3.1},va{0,0,.25},vb{0,0,-.25};double min_z=b.z-a.z;
 for(int i=0;i<1000;++i){
  auto da=constrainPeerMotion(a,{0,0,.25},0,{{b,vb,1,0,true}},pc,{},&va);
  auto db=constrainPeerMotion(b,{0,0,-.25},1,{{a,va,0,0,true}},pc,{},&vb);
  va.z+=std::clamp((da.velocity.z-va.z)/.5,-.5,.5)*.02;
  vb.z+=std::clamp((db.velocity.z-vb.z)/.5,-.5,.5)*.02;
  a=a+va*.02;b=b+vb*.02;min_z=std::min(min_z,b.z-a.z);
 }
 assert(min_z>pc.vertical_separation+.05);
 std::cout<<"Drift, certified stationary escape, predicted vertical convergence passed; minimum dz="<<min_z<<"\n";
}
