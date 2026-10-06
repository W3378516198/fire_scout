#include "fire_scout/passage_scheduler.hpp"
#include <cassert>
#include <iostream>
using namespace fire_scout;

void loggedExit(int robot,Vec3 anchor,Vec3 normal,const std::vector<Vec3>&positions){
  PassageScheduler s;PassageProgress client;
  const int other=robot+10;const int64_t epoch=1000+robot;
  double t=1.;
  s.request({robot,anchor,positions.front(),t,t,false,2.2,epoch});s.update(t);
  assert(s.admitted(robot));
  s.request({other,anchor,anchor+Vec3{-3,0,0},t,t,false,2.2,2000});
  for(Vec3 p:positions){
    t+=.1;
    s.request({other,anchor,anchor+Vec3{-3,0,0},t,t,false,2.2,2000});
    s.request({robot,anchor,p,t,t,false,2.2,epoch});s.update(t);
    // These are the logged positions at which 2.0.3 revoked the grant. The
    // owner must keep it so it can reach the common exit boundary at 2.4 m.
    assert(s.admitted(robot)&&!s.admitted(other));
    assert(!client.exited(p,anchor,normal,2.2,true,false,false));
  }
  const Vec3 direction=(positions.back()-anchor)*(1./distance(positions.back(),anchor));
  const Vec3 outside=anchor+direction*2.45;
  t+=.1;s.request({robot,anchor,outside,t,t,false,2.2,epoch});s.update(t);
  assert(s.finished(robot,anchor,epoch)&&!s.admitted(robot)&&s.admitted(other));
  // The coordinator can finish first and drop a reply. Repeated requests
  // must retain the completion so the next heartbeat can acknowledge it.
  for(int k=0;k<5;++k){t+=.1;s.request({robot,anchor,outside,t,t,false,2.2,epoch});s.update(t);
    assert(s.finished(robot,anchor,epoch)&&!s.admitted(robot));}
  assert(client.exited(outside,anchor,normal,2.2,false,false,false));
  assert(!s.finished(robot,anchor,epoch+1));
  std::cout<<"scout"<<robot+1<<" logged exit: lease retained through old stop, completion recoverable\n";
}

int main(){
  loggedExit(0,{8.743,6.698,2.934},{1,0,0},{{9.889,7.278,3.0},
    {10.398,7.789,2.819},{10.430,7.809,2.812},{10.549,7.843,2.801},{10.613,7.714,2.848}});
  loggedExit(1,{29.524,3.240,2.166},{0,1,0},{{27.620,3.037,2.210},
    {27.872,3.040,2.207},{31.420,3.129,2.629},{31.466,3.127,2.633},{31.693,2.909,2.617}});

  PassageScheduler s;Vec3 a{0,0,1.5};
  s.request({0,a,a,1,1,true,2.2,10});s.update(1);
  s.request({1,a,a+Vec3{3,0,0},1.1,1.1,false,2.2,11});
  s.release(0,a+Vec3{2.1,0,0});s.update(1.1);
  assert(!s.admitted(1)); // A release inside the common exit bound is refused.
  s.update(2);assert(s.leases.front().quarantined);
  s.request({1,a,a+Vec3{3,0,0},2.1,2.1,false,2.2,11});s.update(2.1);
  assert(!s.admitted(1)); // Never convert owner loss or timeout into permission.
  s.release(0,a+Vec3{2.5,0,0});s.update(2.1);assert(s.admitted(1));

  // Cluster centers must not replace the individual owner's exit anchor.
  const Vec3 shifted{.8,0,1.5};
  s.request({2,shifted,shifted+Vec3{-3,0,0},2.2,2.2,false,2.2,12});
  s.release(1,a+Vec3{3,0,0});s.update(2.2);assert(s.admitted(2));
  s.request({2,shifted,shifted,2.3,2.3,true,2.2,12});s.update(2.3);
  s.request({2,shifted,shifted+Vec3{2.1,0,0},2.4,2.4,false,2.2,12});s.update(2.4);
  assert(s.admitted(2)); // r=2.9 to cluster center, only 2.1 to actual gate.
  s.request({2,shifted,shifted+Vec3{2.5,0,0},2.5,2.5,false,2.2,12});s.update(2.5);
  assert(s.finished(2,shifted,12));
  s.request({2,shifted,shifted+Vec3{2.5,0,0},2.6,2.6,false,2.2,13});s.update(2.6);
  assert(s.admitted(2)&&!s.finished(2,shifted,13)); // New traversal has its own id.

  PassageWaitMonitor wait;
  assert(!wait.update(100,true,8,4));assert(!wait.update(107.9,true,8,4));
  assert(wait.update(108,true,8,4)&&wait.reports==1);
  assert(!wait.update(109,true,8,4));assert(wait.update(112,true,8,4));
  assert(!wait.update(113,false,8,4)&&wait.seconds==0);
  assert(!wait.update(120,true,8,4));assert(!wait.update(1,true,8,4));
  assert(wait.update(9,true,8,4));
  PassageDepartureMemory memory;memory.remember(a,{2.5,0,1.5},2.2);
  assert(memory.suppress(a,{2.5,0,1.5})&&memory.suppress(a,{3,0,1.5}));
  assert(!memory.suppress(a,{2.1,0,1.5}));
  assert(!memory.suppress({6,0,1.5},{3,0,1.5}));
  memory.clear();assert(!memory.suppress(a,{3,0,1.5}));
  std::cout<<"v2_passage_handshake_test: PASS log replay, completion, FIFO, owner loss, clustered anchors, wait monitor, reentry\n";
}
