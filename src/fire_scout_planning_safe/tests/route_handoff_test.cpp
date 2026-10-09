#include "fire_scout/route_handoff.hpp"
#include "fire_scout/tracker.hpp"
#include "fire_scout/route_memory.hpp"
#include <cassert>
#include <iostream>
using namespace fire_scout;

int main(){
  GridConfig gc;gc.resolution=.15;gc.inflation_xy=.25;gc.inflation_z=.20;
  Grid free(gc);RouteHandoffConfig c;c.budget_ms=50;
  const std::vector<Vec3>old{{0,0,1.5},{1,0,1.5},{3,0,1.5},{6,0,1.5}};
  auto prefix=retainedForwardPrefix(free,old,{.4,0,1.5},.4,1.0,c);
  assert(prefix.size()>=2&&std::abs(pathLength(prefix)-1.2)<1e-8);
  assert(distance(prefix.front(),{.4,0,1.5})<1e-8);
  const std::vector<Vec3>changed{{.4,0,1.5},{2,.6,1.5},{4,1,1.5},{6,1,1.5}};
  auto joined=connectRetainedPrefix(free,prefix,changed,c);
  assert(joined.applied&&joined.reason=="PREFIX_BEZIER_JOIN");
  assert(joined.points.size()>prefix.size());
  for(size_t i=0;i<prefix.size();++i)assert(distance(joined.points[i],prefix[i])<1e-9);
  assert(distance(joined.points.back(),changed.back())<1e-9);
  assert(routeGeometryValid(free,joined.points,true));
  Vec3 incoming=prefix.back()-prefix[prefix.size()-2];
  Vec3 outgoing=joined.points[prefix.size()]-prefix.back();
  assert(dot(incoming,outgoing)/(norm(incoming)*norm(outgoing))>.999);
  const auto joined_arc=arcLengths(joined.points);
  for(size_t i=1;i<joined.points.size();++i)
    if(joined_arc[i-1]>=joined.prefix_length-1e-8 &&
       joined_arc[i]<=joined.prefix_length+joined.blend_length+1e-8)
      assert(distance(joined.points[i-1],joined.points[i])<=c.sample_spacing+1e-8);
  auto shared=connectRetainedPrefix(free,prefix,routeSlice(old,.4,6),c);
  assert(shared.applied&&shared.reason=="PREFIX_ALREADY_SHARED");
  assert(distance(shared.points.front(),prefix.front())<1e-9);
  assert(distance(shared.points.back(),old.back())<1e-9);
  assert(routeGeometryValid(free,shared.points,true));
  // Prefix must follow a bent incumbent, not cut its corner with a chord.
  std::vector<Vec3>bent{{0,0,1.5},{.5,0,1.5},{.5,.5,1.5},{.5,3,1.5}};
  auto bent_prefix=retainedForwardPrefix(free,bent,{.1,0,1.5},.1,1,c);
  assert(bent_prefix.size()>=3&&distance(bent_prefix[1],{.5,0,1.5})<1e-9);
  // Close obstacle: the commitment must shorten/disappear, never overrule it.
  Grid blocked(gc);blocked.update({blocked.toKey({.85,0,1.5})},{});
  auto unsafe=retainedForwardPrefix(blocked,old,{.4,0,1.5},.4,2,c);
  assert(unsafe.empty());
  auto rejected=connectRetainedPrefix(blocked,prefix,changed,c);
  assert(!rejected.applied&&rejected.reason=="PREFIX_UNSAFE");
  // Unknown-space policy and cancellation apply to the connector too.
  assert(!connectRetainedPrefix(free,prefix,changed,c,false).applied);
  auto cancelled=connectRetainedPrefix(free,prefix,changed,c,true,[]{return true;});
  assert(!cancelled.applied);
  RouteHandoffConfig disabled=c;disabled.enabled=false;
  assert(!connectRetainedPrefix(free,prefix,changed,disabled).applied);
  const std::vector<Vec3>climb{{.4,0,1.5},{2,.4,1.8},{4,.8,2.1},{6,1,2.3}};
  auto vertical=connectRetainedPrefix(free,prefix,climb,c);
  assert(vertical.applied&&routeGeometryValid(free,vertical.points,true));
  assert(distance(vertical.points.back(),climb.back())<1e-9);
  // Full-volume analytic certification detects an obstacle between endpoints.
  const auto curve=handoffBezier({0,0,1.5},{1,0,0},{2,1,1.5},{1,0,0},.4);
  Grid curve_blocked(gc);curve_blocked.update({curve_blocked.toKey(handoffPoint(curve,.5))},{});
  assert(!certifyHandoff(curve_blocked,curve,true,{}));
  // Both endpoint derivatives of the blend have the requested tangents;
  // second differences at both ends vanish exactly.
  assert(norm(curve[2]-curve[1]*2+curve[0])<1e-10);
  assert(norm(curve[5]-curve[4]*2+curve[3])<1e-10);
  // A local repair's entry/exit window can be rounded while keeping the
  // protected front and the distant tail exact.
  std::vector<Vec3>patch{{0,0,1.5},{2,0,1.5},{3,1,1.5},{4,1,1.5},{5,0,1.5},{8,0,1.5}};
  auto smooth=smoothRepairSeam(free,patch,{2,0,1.5},1,c,true);
  assert(smooth.size()>patch.size());
  assert(distance(smooth.front(),patch.front())<1e-9&&distance(smooth.back(),patch.back())<1e-9);
  assert(project(smooth,arcLengths(smooth),{1,0,1.5}).error<1e-9);
  assert(routeGeometryValid(free,smooth,true));
  auto protect=smoothRepairSeam(free,patch,{2,0,1.5},2,c,true);
  assert(protect.size()==patch.size());
  // Updated geometry must not reset a moving tracker, nor jump to a later
  // self-crossing tail. This exercises real state/filter transitions.
  TrackerConfig tc;tc.max_speed_xy=1.;tc.max_accel_xy=1.5;tc.max_jerk_xy=3;
  Tracker track(tc);track.initializeYaw(0);track.setPath(old,{.4,0,1.5});
  auto clear=[](Vec3,Vec3){return true;};Control before;
  for(int i=0;i<100;++i)before=track.update({.4,0,1.5},{.8,0,0},.025,clear,0);
  assert(norm(before.velocity)>.3);
  assert(track.setPath(joined.points,{.4,0,1.5}));
  auto after=track.update({.4,0,1.5},{.8,0,0},.025,clear,0);
  assert(norm(after.velocity)>.3&&distance(before.velocity,after.velocity)<=tc.max_accel_xy*.025+1e-6);
  assert(!after.heading_wait);
  std::vector<Vec3>cross{{0,0,1.5},{2,0,1.5},{2,2,1.5},{.5,.01,1.5},{-3,.01,1.5}};
  track.setPath(cross,{.5,.01,1.5});
  auto crossing=track.update({.5,.01,1.5},{.8,0,0},.025,clear,0);
  assert(crossing.progress<1&&crossing.target.x>.5);
  // A moving drone beyond two metres on the incumbent can still select a
  // local prefix using its measured arc progress.
  auto far=retainedForwardPrefix(free,old,{4,0,1.5},4,1,c);
  assert(!far.empty()&&distance(far.front(),{4,0,1.5})<1e-9);
  // A clear .25 m shortcut is not a certificate for a bent initial curve.
  GridConfig short_gc;short_gc.resolution=.02;short_gc.inflation_xy=.469;short_gc.inflation_z=.272;
  Grid short_obstacle(short_gc);short_obstacle.update({short_obstacle.toKey({.05,.50,1.5})},{});
  std::vector<Vec3> short_bend{{0,0,1.5},{.05,.05,1.5},{.12,0,1.5},{2,0,1.5}};
  RouteMemory memory;memory.set(short_bend);
  assert(!memory.valid(short_obstacle,short_bend.front()));
  auto safe=memory.safePrefix(short_obstacle,short_bend.front());
  for(size_t i=1;i<safe.size();++i)assert(short_obstacle.segmentFrom(safe[i-1],safe[i]));
  std::cout<<"route_handoff_test: PASS\n";
}
