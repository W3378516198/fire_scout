#include "fire_scout/planner.hpp"
#include "fire_scout/profile_repair.hpp"
#include "fire_scout/profile_progress.hpp"
#include "fire_scout/diagnostic_snapshot.hpp"
#include <cassert>
#include <iostream>
#include <string>
using namespace fire_scout;

int main(){
  GridConfig gc;gc.resolution=.10;gc.inflation_xy=.16;gc.inflation_z=.12;
  gc.z_min=.5;gc.z_max=4.;Grid free(gc);
  RouteHandoffConfig handoff;handoff.budget_ms=50;
  TrackerConfig tc;tc.sharp_turn=80*pi/180;tc.lateral_accel=1.;tc.yaw_rate=1.2;
  tc.max_jerk_xy=4.5;

  // The old shared-prefix fast path retained a 5 cm lateral connector and a
  // 68 degree entry corner, despite claiming the straight prefix was retained.
  const std::vector<Vec3> prefix{{0,0,2},{1,0,2},{2,0,2}};
  const std::vector<Vec3> hook{{0,.05,2},{.02,0,2},{.04,0,2},{1,0,2},{2,0,2},{4,0,2}};
  const auto joined=connectRetainedPrefix(free,prefix,hook,handoff);
  assert(joined.applied&&joined.reason=="PREFIX_ALREADY_SHARED");
  for(size_t i=0;i<prefix.size();++i)assert(distance(joined.points[i],prefix[i])<1e-9);
  for(Vec3 p:joined.points)assert(std::abs(p.y)<1e-9);
  assert(assessTurnPath(joined.points,tc.turn,tc.lateral_accel,tc.yaw_rate,tc.sharp_turn).stops==0);

  // A distant new obstacle must not turn a 10 cm cross-track error into the
  // first edge of a repaired path. Exercise the production incremental search.
  std::vector<Vec3> old;for(int i=0;i<=30;++i)old.push_back({.2*i,0,2});
  Grid blocked(gc);blocked.update({blocked.toKey({3,0,2})},{});
  const Vec3 current{.55,.10,2};
  const auto inspection=inspectRoute(blocked,old,current,true,.55);
  assert(inspection.projection_valid&&!inspection.valid&&inspection.first_invalid_distance>1.5);
  for(size_t i=0;i<5;++i)assert(std::abs(inspection.suffix[i].y)<1e-9);
  PlannerConfig pc;pc.cruise_altitude=2.;pc.enable_3d_search=false;
  pc.max_search_ms=pc.max_retry_search_ms=500;pc.max_expansions=100000;
  pc.repair_search_ms=500;pc.repair_pre_margin=.45;pc.repair_post_margin=.6;pc.repair_max_span=4.;
  Planner planner(pc);assert(!planner.plan(free,old.front(),old.back()).points.empty());
  planner.setCommittedRoute(old);planner.requestRepair();
  const auto repaired=planner.plan(blocked,current,old.back());
  assert(repaired.locally_patched&&routeGeometryValid(blocked,repaired.points,true));
  for(size_t i=0;i<repaired.patch_begin;++i)assert(std::abs(repaired.points[i].y)<1e-9);
  assert(std::abs(repaired.points.front().x-current.x)<1e-8);
  assert(!inspectRoute(free,old,current,false,.55).valid); // unknown policy is retained
  PlannerConfig resumed_config=pc;resumed_config.max_expansions=4;
  resumed_config.local_repair_enabled=false;
  Planner resumed(resumed_config);auto resumed_result=resumed.plan(free,{0,0,2},{3,0,2});
  assert(resumed_result.points.empty());
  // Finish the retained search with an ample budget. This case tests its
  // geometry after the root moves, rather than the number of budget retries.
  resumed.cfg.max_expansions=100000;
  resumed_result=resumed.plan(free,{.03,.05,2},{3,0,2});
  assert(!resumed_result.points.empty()&&std::abs(resumed_result.points.front().y)<1e-9);

  // Log 2: a 21 cm entry leg points behind the vehicle; the next leg goes
  // forward. The old tracker first commanded about -115 deg, then -35 deg.
  std::vector<Vec3> entry{{0,.21,2},{.013,0,2},{.076,-.05,2},{3,-.4,2},{5,-.4,2}};
  Tracker tracker(tc);Vec3 pos{.10,.21,2},velocity{};double yaw=0,min_yaw=0;
  tracker.initializeYaw(yaw);tracker.setPath(entry,pos);
  auto clear=[](Vec3,Vec3){return true;};constexpr double dt=.025;
  int pivots=0;bool moved=false;
  for(int i=0;i<400;++i){
    if(i==8){entry[0].x-=.01;tracker.setPath(entry,pos);} // a small prefix revision
    const auto u=tracker.update(pos,velocity,dt,clear,yaw);
    if(std::string(u.turn_phase)=="TURN_IN_PLACE"||std::string(u.turn_phase)=="TURN_BRAKE")++pivots;
    velocity=velocity+limitNorm((u.velocity-velocity)*(dt/.25),1.5*dt);
    pos=pos+velocity*dt;
    yaw=wrap(yaw+std::clamp(wrap(u.yaw-yaw)*(dt/.12),-1.2*dt,1.2*dt));
    min_yaw=std::min(min_yaw,yaw);
    if(pos.x>1.0){moved=true;break;}
  }
  std::cout<<"entry moved="<<moved<<" pivots="<<pivots<<" min_yaw_deg="<<min_yaw*180/pi<<"\n";
  assert(moved&&pivots==0&&min_yaw>-80*pi/180);
  Tracker veto(tc);veto.initializeYaw(0);veto.setPath(entry,{.10,.21,2});
  const auto stopped=veto.update({.10,.21,2},{},dt,
    [](Vec3 a,Vec3 b){return distance(a,b)<.01;},0);
  assert(norm(stopped.velocity)==0&&std::string(stopped.turn_phase)=="TURN_REJOIN_BLOCKED");

  // Log 1: a centimetre-scale S entry generated a ~0.03 m/s speed ceiling.
  // A stopped repair may replace that entry, but must preserve the far tail.
  const std::vector<Vec3> micro{{29.669,2.664,2.023},{29.685,2.627,2.036},
    {29.697,2.608,2.047},{29.707,2.598,2.055},{29.717,2.592,2.062},
    {29.736,2.590,2.069},{29.759,2.597,2.070},{30.,2.66,2.075},
    {32.,2.7,2.075},{36.,3.,2.1}};
  const Vec3 at{29.705,2.61,2.05};
  const auto fixed=repairStoppedRouteEntry(free,micro,at,0,.02,tc,handoff,true);
  std::cout<<"micro repair="<<fixed.applied<<" before="<<fixed.before_limit<<" after="<<fixed.after_limit<<"\n";
  assert(fixed.applied&&fixed.after_limit>.25&&routeGeometryValid(free,fixed.points,true));
  assert(distance(fixed.points.back(),micro.back())<1e-9);
  assert(distance(fixed.points[fixed.points.size()-2],micro[micro.size()-2])<1e-9);
  assert(!repairStoppedRouteEntry(free,micro,at,0,.4,tc,handoff,true).applied);
  Grid entry_wall(gc);entry_wall.update({entry_wall.toKey(at)},{});
  assert(!repairStoppedRouteEntry(entry_wall,micro,at,0,.02,tc,handoff,true).applied);
  const auto snapshot=mapConflictSnapshot(entry_wall,at,"scout2/odom");
  assert(snapshot.find("samples=1")!=std::string::npos&&snapshot.size()<1024);

  // Small oscillations and brief FOLLOW phases cannot erase a campaign. A
  // safety queue pauses it; meaningful net motion clears it. Revisions have
  // deliberately no reset operation here.
  ExecutionProgressMonitor progress;bool requested=false,timed_out=false;
  for(int i=0;i<=420;++i){
    const Vec3 p{.06*std::sin(i*.05),.04*std::sin(i*.09),2};
    const auto d=progress.update(true,p,dt,i%40<3);
    requested|=d.request_repair;timed_out|=d.request_recovery;
  }
  assert(requested&&timed_out);
  const double elapsed=progress.seconds();
  for(int i=0;i<800;++i)assert(!progress.update(false,{0,0,2},dt).request_recovery);
  assert(progress.seconds()==elapsed);
  progress.update(true,{1,0,2},dt);assert(progress.seconds()==0);
  std::cout<<"entry_continuity_test: PASS\n";
}
