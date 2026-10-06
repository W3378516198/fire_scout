#include "fire_scout/confidence_fusion.hpp"
#include "fire_scout/envelope_escape.hpp"
#include "fire_scout/peer_uav_filter.hpp"
#include "fire_scout/peer_safety.hpp"
#include "fire_scout/pose_history.hpp"
#include "fire_scout/racer_task_allocator.hpp"
#include "fire_scout/ultrasonic.hpp"
#include "fire_scout/vehicle.hpp"

#include <cassert>
#include <iostream>
#include <limits>
#include <random>

using namespace fire_scout;

int main() {
  VehicleConfig x500;
  assert(std::abs(x500.physicalRadius()-.39)<1e-9);
  assert(x500.passageRadius(.02)>.46 && x500.passageRadius(.02)<.48);

  const double resolution = 0.1;
  ArrivalSlotConfig slots_config;
  slots_config.resolution = resolution;
  slots_config.radius = 1.0;
  slots_config.max_radius = 2.0;
  slots_config.radius_step = .35;
  slots_config.min_separation = .9;
  slots_config.clearance_xy = .25;
  slots_config.clearance_z = .20;
  slots_config.angular_samples = 24;

  KeySet occupied;
  auto slots = makeArrivalSlots(occupied, {10, 0, 1.5}, 4, slots_config);
  assert(slots.size() >= 4);
  assert(distance(slots.front().target, {10, 0, 1.5}) < 1e-9);
  for (size_t i = 0; i < slots.size(); ++i)
    for (size_t j = i + 1; j < slots.size(); ++j)
      assert(distance(slots[i].target, slots[j].target) + 1e-9 >=
             slots_config.min_separation);

  // Occupying the exact common goal removes only that endpoint; route-only
  // coordination still supplies distinct nearby arrival slots.
  occupied.insert(key({10, 0, 1.5}, resolution));
  slots = makeArrivalSlots(occupied, {10, 0, 1.5}, 4, slots_config);
  assert(slots.size() >= 4);
  for (const auto &slot : slots)
    assert(distance(slot.target, {10, 0, 1.5}) > .5);

  const std::vector<Vec3> straight{{0, 0, 1}, {5, 0, 1}, {10, 0, 1}};
  const std::vector<Vec3> same_corridor{{0, .2, 1}, {10, .2, 1}};
  const std::vector<Vec3> separate{{0, 3, 1}, {10, 3, 1}};
  // Shared corridors are allowed when arrival times differ; only an actual
  // near-horizon time conflict may ask the stable-route planner to reconsider.
  assert(routeTimeConflictLoss(straight, {same_corridor}, 1.0, 1.0, .2) > 0);
  assert(routeTimeConflictLoss(straight, {separate}, 1.0, 1.0, .2) == 0);
  assert(!routesDistinct(straight, same_corridor, .25, .25));
  assert(routesDistinct(straight, separate, .25, .25));
  assert(routeTurnLoss(straight) < 1e-9);
  assert(routeTurnLoss({{0, 0, 1}, {1, 0, 1}, {1, 1, 1}}) > .9);

  const Key body = key({0, 0, 1}, resolution);
  const Key wall = key({2, 0, 1}, resolution);
  occupied.clear();
  occupied.insert(body);
  occupied.insert(wall);
  KeySet uncertain;
  PeerUavBox box{{0, 0, 1}, 0, 1.0, 1.0, 0.8};
  assert(extractPeerUavBoxes(occupied, uncertain, resolution, {box}) == 1);
  assert(!occupied.count(body) && uncertain.count(body) && occupied.count(wall));

  PeerUavBoxHistoryConfig history_config;
  history_config.duration = 0.8;
  history_config.position_step = 0.15;
  history_config.max_boxes = 4;
  PeerUavBoxHistory history(history_config);
  history.add(box, 0.0);
  auto nearby_box = box;
  nearby_box.center.x = 0.10;
  history.add(nearby_box, 0.10);
  assert(history.size() == 1);
  nearby_box.center.x = 0.20;
  history.add(nearby_box, 0.20);
  assert(history.size() == 2);
  assert(history.boxes(0.50).size() == 2);
  assert(history.boxes(1.10).empty());

  PoseHistory poses;
  poses.add({1.00, {0, 0, 1}, 1, 0, 0, 0});
  poses.add({1.10, {0.1, 0, 1}, 1, 0, 0, 0});
  PoseSample synchronized_pose;
  assert(poses.at(1.05, synchronized_pose, 0.08));
  assert(std::abs(synchronized_pose.position.x - 0.05) < 1e-9);
  assert(!poses.at(1.20, synchronized_pose, 0.08));

  const std::vector<float> clear_sonar(
      9, std::numeric_limits<float>::infinity());
  const auto clear_reading = parseSonar(
      clear_sonar, -10 * pi / 180.0, 2.5 * pi / 180.0, 0.2, 3.0,
      10 * pi / 180.0, true);
  assert(clear_reading.valid && !clear_reading.has_hit &&
         std::abs(clear_reading.distance - 3.0) < 1e-9);

  ConfidenceDecisionConfig confidence;
  assert(decideFusionState(0.90, 1.0, false, true, confidence) ==
         FusionState::Free);
  assert(decideFusionState(0.45, 0.0, false, false, confidence) ==
         FusionState::Occupied);
  assert(decideFusionState(0.05, 0.0, false, false, confidence) ==
         FusionState::Unknown);

  PeerSafetyConfig peer_safety;
  peer_safety.hard_separation=1.0;
  peer_safety.vertical_separation=.6;
  peer_safety.reaction_time=.5;
  peer_safety.brake_accel=1.0;
  peer_safety.prediction_horizon=2.5;
  peer_safety.passage_width=1.4;
  PeerKinematicState peer;
  peer.position={1.2,0,1};peer.velocity={};peer.priority=0;
  peer.index=0;peer.fresh=true;
  auto limited=constrainPeerMotion({0,0,1},{1,0,0},1,{peer},peer_safety);
  assert(limited.limited);
  assert(limited.velocity.x<1.0);
  assert(limited.nearest_clearance<.21);

  PassageReservation passage;
  passage.valid=true;passage.anchor={1,0,1};passage.own_inside=false;
  peer.position={1.2,0,1};
  auto yielded=constrainPeerMotion({0,0,1},{.4,0,0},0,{peer},peer_safety,passage);
  assert(yielded.yielding);
  assert(yielded.reason==PeerSafetyReason::PASSAGE_YIELD);
  assert(std::hypot(yielded.velocity.x,yielded.velocity.y)<1e-9);

  // Queue order is geometric.  A lower-numbered peer just behind the current
  // vehicle must brake itself; it cannot make the front vehicle stop and form
  // a circular entrance deadlock merely because the anchor distances tie.
  PeerKinematicState rear_peer=peer;
  rear_peer.position={-.4,0,1};rear_peer.velocity={};rear_peer.priority=0;
  auto queue_safety=peer_safety;
  queue_safety.hard_separation=.30;
  queue_safety.passage_priority_hysteresis=.50;
  PassageReservation near_passage;
  near_passage.valid=true;near_passage.anchor={.1,0,1};
  auto rear_ignored=constrainPeerMotion(
      {0,0,1},{.4,0,0},1,{rear_peer},queue_safety,near_passage);
  assert(!rear_ignored.yielding);
  assert(rear_ignored.rear_yields_ignored>=1);
  assert(rear_ignored.velocity.x>0);

  // The same rule applies to a faster rear vehicle predicted to catch up.
  // Pairwise braking remains active on that rear vehicle's own controller.
  rear_peer.position={-1.1,0,1};rear_peer.velocity={2,0,0};
  auto rear_conflict_ignored=constrainPeerMotion(
      {0,0,1},{1,0,0},1,{rear_peer},peer_safety);
  assert(!rear_conflict_ignored.yielding);
  assert(rear_conflict_ignored.rear_yields_ignored>=1);

  // A stalled passage owner still blocks forward motion, but a certified
  // command away from it is retained at the dedicated low retreat speed.
  peer.position={1.2,0,1};peer.velocity={};peer.priority=0;
  auto yielded_retreat=constrainPeerMotion(
      {0,0,1},{-.4,0,0},1,{peer},peer_safety,passage);
  assert(yielded_retreat.reason==PeerSafetyReason::PASSAGE_YIELD);
  assert(!yielded_retreat.yielding);
  assert(yielded_retreat.velocity.x<0);
  assert(std::abs(yielded_retreat.velocity.x)<=
         peer_safety.yield_retreat_speed+1e-9);

  PeerYieldWatchdog yield_watchdog;
  yield_watchdog.cfg.no_progress_timeout=3.0;
  yield_watchdog.cfg.blocker_progress_distance=.12;
  assert(!yield_watchdog.update(0.0,true,0,{1,0,1},true));
  assert(!yield_watchdog.update(2.9,true,0,{1.05,0,1},true));
  assert(yield_watchdog.update(3.1,true,0,{1.05,0,1},true));
  // Real blocker progress renews the lease; a new blocker and a clock rewind
  // also start fresh epochs instead of inheriting a stale timeout.
  assert(!yield_watchdog.update(3.2,true,0,{1.13,0,1},true));
  assert(!yield_watchdog.update(6.0,true,1,{2,0,1},true));
  assert(!yield_watchdog.update(1.0,true,1,{2,0,1},true));
  assert(!yield_watchdog.update(2.0,false,-1,{},false));
  assert(!yield_watchdog.active());

  // A short back-and-forth excursion is not useful queue progress.  Net
  // displacement over the complete lease remains zero, so it must expire.
  PeerYieldWatchdog oscillating_watchdog;
  oscillating_watchdog.cfg=yield_watchdog.cfg;
  assert(!oscillating_watchdog.update(0.0,true,0,{1,0,1},true));
  assert(!oscillating_watchdog.update(1.0,true,0,{1.13,0,1},true));
  assert(!oscillating_watchdog.update(2.0,true,0,{1,0,1},true));
  assert(oscillating_watchdog.update(3.1,true,0,{1,0,1},true));

  // Inside the hard pairwise envelope only a bounded separating command is
  // retained; a closing or tangential command is never forwarded.
  peer.position={.8,0,1};
  auto separating=constrainPeerMotion({0,0,1},{-.3,.2,0},0,{peer},peer_safety);
  assert(separating.reason==PeerSafetyReason::HARD_SEPARATION);
  assert(separating.velocity.x<0);
  assert(std::abs(separating.velocity.x)<=
         peer_safety.separation_escape_speed+1e-9);
  assert(std::abs(separating.velocity.y)<1e-9);

  // Multiple peers impose simultaneous radial half-spaces.  Applying them
  // once in order can re-open the first constraint, so verify the final
  // command satisfies both regardless of that coupling.
  PeerKinematicState right_peer=peer;
  right_peer.position={1.1,0,1};right_peer.velocity={};
  const Vec3 diagonal_direction{-std::sqrt(.75),.5,0};
  PeerKinematicState diagonal_peer=peer;
  diagonal_peer.position=diagonal_direction*1.1;
  diagonal_peer.position.z=1;diagonal_peer.velocity={};
  auto multi_limited=constrainPeerMotion(
      {0,0,1},{0,1,0},0,{right_peer,diagonal_peer},peer_safety);
  const double simultaneous_limit=peerStoppingSpeed(
      .1,peer_safety.brake_accel,peer_safety.reaction_time);
  assert(multi_limited.limited);
  assert(multi_limited.velocity.x<=simultaneous_limit+1e-6);
  assert(dot(Vec3{multi_limited.velocity.x,multi_limited.velocity.y,0},
             diagonal_direction)<=simultaneous_limit+1e-6);

  // Deterministic stress coverage for simultaneous braking constraints.
  std::mt19937 generator(20);
  std::uniform_real_distribution<double> unit(-1.0,1.0);
  std::uniform_real_distribution<double> range(1.01,3.0);
  for(int sample=0;sample<1000;++sample){
    std::vector<PeerKinematicState> random_peers;
    for(size_t i=0;i<4;++i){
      const double angle=pi*(unit(generator)+1.0);
      const double radius=range(generator);
      PeerKinematicState random_peer;
      random_peer.position={radius*std::cos(angle),radius*std::sin(angle),1};
      random_peer.velocity={.5*unit(generator),.5*unit(generator),0};
      random_peer.priority=1;random_peer.index=i;random_peer.fresh=true;
      random_peers.push_back(random_peer);
    }
    const Vec3 request{1.8*unit(generator),1.8*unit(generator),0};
    const auto constrained=constrainPeerMotion(
        {0,0,1},request,0,random_peers,peer_safety);
    for(const auto &random_peer:random_peers){
      const Vec3 offset=random_peer.position-Vec3{0,0,1};
      const double radius=horizontalNorm(offset);
      const Vec3 direction{offset.x/radius,offset.y/radius,0};
      const double peer_radial=dot(random_peer.velocity,direction);
      const double allowed=std::max(0.0,peerStoppingSpeed(
          radius-peer_safety.hard_separation,peer_safety.brake_accel,
          peer_safety.reaction_time)+peer_radial);
      assert(dot(constrained.velocity,direction)<=allowed+1e-6);
    }
  }

  // A conservative map voxel may overlap the normal tracking envelope while
  // remaining outside the tilted-rotor recovery envelope minus the bounded
  // initial allowance.  Only monotonic motion away from the frozen contact is
  // accepted; tangential/closing motion and new contacts remain forbidden.
  GridConfig escape_cfg;
  escape_cfg.resolution=.17;escape_cfg.inflation_xy=.469;
  escape_cfg.recovery_xy=.429;escape_cfg.inflation_z=.28;
  escape_cfg.z_min=.5;escape_cfg.z_max=2.5;
  Grid escape_grid(escape_cfg);
  escape_grid.update({Key{0,0,5}},{});
  const Vec3 overlapped{.45,.085,1.0};
  auto proof=separationProof(escape_grid,overlapped,.15);
  assert(proof.valid);
  assert(std::abs(proof.radius-.429)<1e-9);
  assert(separatingSweep(escape_grid,proof,overlapped,{.80,.085,1.0}));
  assert(!separatingSweep(escape_grid,proof,overlapped,{.35,.085,1.0}));
  escape_grid.occupied.insert(Key{3,0,5});
  assert(!separatingSweep(escape_grid,proof,overlapped,{.80,.085,1.0}));

  // Preserve the legacy/unscaled SDF's 1.2 m doorway as a conservative
  // regression. With a 0.469 m envelope, 0.17 m quantization leaves no
  // collision-free A* voxel centre; 0.15 m retains a centred representable
  // route. The paired V21 world scales this opening to 2.4 m.
  const auto doorway_has_grid_center=[](double cell){
    GridConfig doorway_cfg;
    doorway_cfg.resolution=cell;doorway_cfg.inflation_xy=.469;
    doorway_cfg.inflation_z=.28;doorway_cfg.z_min=.5;doorway_cfg.z_max=2.5;
    Grid doorway(doorway_cfg);
    doorway.update({key({-5.6,1.5,1.5},cell),
                    key({-4.4,1.5,1.5},cell)},{});
    const int first=key({-5.6,0,0},cell).x;
    const int last=key({-4.4,0,0},cell).x;
    for(int x=first+1;x<last;++x){
      const double center_x=doorway.point({x,0,0}).x;
      if(doorway.segment({center_x,1.0,1.5},{center_x,2.0,1.5}))return true;
    }
    return false;
  };
  assert(!doorway_has_grid_center(.17));
  assert(doorway_has_grid_center(.15));

  std::cout << "coordination_components_test: PASS\n";
  return 0;
}
