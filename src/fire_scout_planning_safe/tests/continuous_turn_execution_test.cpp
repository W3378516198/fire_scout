#include "fire_scout/tracker.hpp"
#include "fire_scout/profile_repair.hpp"
#include <cassert>
#include <iostream>
#include <string>
using namespace fire_scout;
static auto clear=[](Vec3,Vec3){return true;};
struct Aircraft {
  Vec3 p{},v{};double yaw{0};
  void step(const Control &u,double dt){
    v=v+limitNorm((u.velocity-v)*(dt/.25),1.5*dt);
    p=p+v*dt;
    yaw=wrap(yaw+std::clamp(wrap(u.yaw-yaw)*(dt/.12),-1.2*dt,1.2*dt));
  }
};
static TrackerConfig config(){TrackerConfig c;c.sharp_turn=80*pi/180;
  c.yaw_rate=1.2;c.lateral_accel=1.;c.max_jerk_xy=4.5;return c;}
int main(){
  const auto c=config();constexpr double dt=.025;
  // Log-like dense curves: 3 cm samples, curvature 1.82..3.0 /m, with
  // first-order autopilot response and acceleration-limited physical inertia.
  for(double radius:{.55,1./3.}){
    std::vector<Vec3> path{{0,0,3},{2,0,3}};
    const int count=int(std::ceil((pi/2)*radius/.03));
    for(int i=1;i<=count;++i){double t=(pi/2)*i/count;
      path.push_back({2+radius*std::sin(t),radius*(1-std::cos(t)),3});}
    path.push_back({2+radius,4,3});
    Tracker tracker(c);tracker.initializeYaw(0);tracker.setPath(path,path.front());
    Aircraft a;a.p=path.front();bool reached=false;int stops=0;double finished=0;
    for(int i=0;i<1600;++i){
      if(i%4==0)assert(!tracker.setPath(path,a.p)); // 10 Hz heartbeats
      const auto u=tracker.update(a.p,a.v,dt,clear,a.yaw);
      if(u.turn_stop)++stops;
      a.step(u,dt);
      if(u.reached){reached=true;finished=i*dt;break;}
    }
    std::cout<<"radius="<<radius<<" finish="<<finished<<" stops="<<stops<<" pos="<<a.p.x<<","<<a.p.y<<"\n";
    assert(reached&&finished<30&&stops==0);
    // None of the interior dense samples is a zero-speed anchor.
    auto speeds=tracker.speedProfile();for(size_t i=1;i+1<speeds.size();++i)assert(speeds[i]>.05);
  }
  // New short stop profile must cap an old fast filter immediately. Actual
  // measured inertia is not claimed to disappear with that command.
  Tracker switched(c);switched.initializeYaw(0);switched.setPath({{0,0,3},{10,0,3}},{0,0,3});
  for(int i=0;i<120;++i)switched.update({0,0,3},{1.3,0,0},dt,clear,0);
  switched.setPath({{1.5,0,3},{2,0,3},{2,4,3}},{1.78,0,3});
  auto u=switched.update({1.78,0,3},{1.3,0,0},dt,clear,0);
  assert(u.turn_stop&&norm(u.velocity)<=u.speed_limit+1e-9&&norm(u.velocity)<.4);

  // 70 cm braking overshoot: rejoin the outgoing leg through a checked local
  // chord, not the already passed corner. Do not cross the following corner.
  const std::vector<Vec3> corner{{0,0,3},{2,0,3},{2,4,3}};
  Tracker over(c);Aircraft a;a.p={2.7,-.02,3};over.initializeYaw(0);over.setPath(corner,a.p);
  bool rejoined=false;double min_y=0;
  for(int i=0;i<600;++i){u=over.update(a.p,a.v,dt,clear,a.yaw);a.step(u,dt);
    min_y=std::min(min_y,a.p.y);if(a.p.y>.8){rejoined=true;break;}}
  std::cout<<"overrun rejoin="<<rejoined<<" pos="<<a.p.x<<","<<a.p.y<<"\n";
  assert(rejoined&&min_y>-.03&&over.progress()>2.5);
  Tracker obstructed(c);obstructed.initializeYaw(0);obstructed.setPath(corner,{2.7,0,3});
  auto wall=[](Vec3 a,Vec3 b){return !(a.x>2.1&&b.y>.05);};
  for(int i=0;i<200;++i){u=obstructed.update({2.7,0,3},{},dt,wall,0);
    assert(norm(u.velocity)==0&&std::string(u.turn_phase)=="TURN_REJOIN_BLOCKED");}
  Tracker looping(c);looping.initializeYaw(0);
  looping.setPath({{0,0,3},{2,0,3},{2,.3,3},{1,.3,3},{1,-1,3},{4,-1,3}},{2.7,0,3});
  looping.update({2.7,0,3},{},dt,clear,0);
  assert(distance(looping.previewTarget({2.7,0,3}),{2,.3,3})<1e-8);

  // Recovery on a long mission preserves its local arc neighborhood.
  Tracker long_route(c);long_route.initializeYaw(0);
  long_route.setPath({{0,0,3},{20,0,3}},{0,0,3});
  for(double x=0;x<=12;x+=.1)long_route.update({x,0,3},{.5,0,0},dt,clear,0);
  long_route.resumeAfterRecovery({11.7,0,3});
  assert(long_route.progress()>11&&long_route.previewTarget({11.7,0,3}).x>12);

  // A true abrupt 90-degree discontinuity still brakes, then rotates; a wall
  // certificate continues to veto every translation through it.
  Tracker hard(c);hard.initializeYaw(0);hard.setPath(corner,{1.9,0,3});
  u=hard.update({2,0,3},{.5,0,0},dt,clear,0);
  assert(std::string(u.turn_phase)=="TURN_BRAKE"&&norm(u.velocity)==0);
  u=hard.update({2,0,3},{},dt,[](Vec3,Vec3){return false;},0);
  assert(u.blocked&&norm(u.velocity)==0);
  std::cout<<"continuous_turn_execution_test: PASS dense bends, inertia, command cap, certified rejoin, blocked rejoin, long-route resume\n";
}
