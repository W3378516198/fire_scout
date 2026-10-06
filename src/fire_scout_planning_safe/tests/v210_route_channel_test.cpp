#include "fire_scout/execution_permit.hpp"
#include "fire_scout/tracker.hpp"
#include <cassert>
#include <iostream>
using namespace fire_scout;
int main(){
  GeometricRouteChannel c;Vec3 goal{5,0,1.5};
  auto wire=[&](int64_t rev,double ros,bool ready=true,std::string reason="READY",int64_t inst=1){
    return RoutePermit{int64_t(ros*1e9),rev,ready,reason,goal,inst}.encode();};
  std::vector<Vec3> a{{0,0,1.5},goal},b{{0,0,1.5},{2,.1,1.5},goal};
  assert(c.path(10,a));assert(!c.synchronize(10,100,.8,goal));
  assert(c.permit(wire(10,10),10,100));assert(c.synchronize(10,100,.8,goal));
  Tracker t;t.setPath(c.active_path,{0,0,1.5});
  assert(c.allowed(10,100,.8,goal));auto n=c.generation;
  assert(c.permit(wire(10,10.1),10.1,100.1));
  assert(!c.synchronize(10.1,100.1,.8,goal)&&c.generation==n);
  assert(!t.setPath(c.active_path,{.1,0,1.5})); // Heartbeat preserves tracker profile.
  assert(c.path(11,b));assert(c.allowed(10.1,100.1,.8,goal));
  assert(c.permit(wire(11,10.2),10.2,100.2));assert(c.synchronize(10.2,100.2,.8,goal));
  assert(c.active_revision==11 && c.generation==n+1);
  // Reverse arrival order: no unbounded use of the old route while waiting.
  assert(c.permit(wire(12,10.3),10.3,100.3));
  assert(c.allowed(10.3,100.3,.8,goal));
  assert(!c.allowed(10.51,100.51,.8,goal));
  assert(c.path(12,a));assert(c.synchronize(10.51,100.51,.8,goal));
  assert(c.permit(wire(12,10.6,false,"STALE_MAP"),10.6,100.6));
  assert(c.inputPaused(10.6,100.6,.8) && !c.allowed(10.6,100.6,.8,goal));
  assert(c.active_path.size()==2); // Display geometry survives an input pause.
  assert(!c.permit(wire(12,10.5),10.6,100.6)); // Late positive cannot undo a denial.
  assert(c.permit(wire(12,10.7),10.7,100.7));
  assert(!c.synchronize(10.7,100.7,.8,goal));assert(c.allowed(10.7,100.7,.8,goal));
  assert(!c.allowed(10.71,102,.8,goal)); // Also reject a stalled ROS clock by wall age.
  assert(!c.allowed(10.7,100.7,.8,goal+Vec3{1,0,0}));
  assert(c.permit(wire(13,10.8,false,"NO_ROUTE"),10.8,100.8));
  assert(!c.inputPaused(10.8,100.8,.8));assert(!c.allowed(10.8,100.8,.8,goal));
  assert(c.permit(wire(14,10.9,true,"READY",2),10.9,100.9));
  assert(!c.allowed(10.9,100.9,.8,goal));assert(c.path(14,b));
  assert(c.synchronize(10.9,100.9,.8,goal));
  assert(!c.permit(wire(14,11,true,"READY",1),11,101)); // Retired publisher.
  assert(!c.permit("v=1 ready=1",11,101));
  c.reset();assert(c.inputPaused(0,0,.8)&&c.active_path.empty());
  std::cout<<"v210_route_channel_test: PASS revisions, DDS arrival skew, no heartbeat reset, stale input, recovery eligibility, publisher restart\n";
}
