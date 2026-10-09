#include "fire_scout/replan_schedule.hpp"
#include <cassert>
#include <iostream>
using namespace fire_scout;

Grid doorwayMap(double x = 20) {
  GridConfig c; c.resolution = .2; c.inflation_xy = .25; c.inflation_z = .15;
  c.z_min = .6; c.z_max = 2.5;
  Grid g(c); KeySet occupied;
  for (int y = -150; y <= 150; ++y) {
    if (std::abs((y + .5) * c.resolution) < .9) continue;
    for (int z = 0; z <= 15; ++z) occupied.insert({int(x / c.resolution), y, z});
  }
  g.update(std::move(occupied), {}); return g;
}

int main() {
  TopologyConfig cfg;
  auto wall = doorwayMap(); Grid open(wall.cfg);
  const std::vector<Vec3> route{{0, 0, 1.5}, {40, 0, 1.5}};
  SharedEntranceReview probe; bool requested = false; double previous = 0;
  for (int k = 0; k < 100; ++k) {
    // Fresh evidence every iteration must not reset a long sweep to its start.
    wall.update(wall.occupied, {});
    requested = probe.shouldReview(wall, route, 0, route.front(), {route}, cfg, k * .1, 1, 1, 1000);
    assert(probe.lastSamples() <= 1);
    if (requested) break;
    assert(probe.scanArc() > previous); previous = probe.scanArc();
  }
  assert(requested); // The distant, physically witnessed shared doorway matters.

  SharedEntranceReview empty;
  for (int k = 0; k < 250; ++k)
    assert(!empty.shouldReview(open, route, 0, route.front(), {route}, cfg, k * .1, 1, 4, 1000));
  SharedEntranceReview separate;
  const std::vector<Vec3> different{{0, 8, 1.5}, {40, 8, 1.5}};
  for (int k = 0; k < 100; ++k)
    assert(!separate.shouldReview(wall, route, 0, route.front(), {different}, cfg, k * .1, 1, 4, 1000));

  SharedEntranceReview committed;
  for (int k = 0; k < 100; ++k)
    assert(!committed.shouldReview(wall, route, 19.5, {19.5, 0, 1.5}, {route}, cfg, k * .1, 1, 4, 1000));
  // Cooldown suppresses the same unhelpful review, but a newcomer wakes it.
  probe.noImprovement(100, cfg.refresh_period);
  assert(!probe.shouldReview(wall, route, 0, route.front(), {route}, cfg, 100.1, 1, 1000, 1000));
  assert(probe.lastSamples() == 0);
  auto advancing_peer = route; advancing_peer.front().x = .5;
  assert(!probe.shouldReview(wall, route, 0, route.front(), {advancing_peer}, cfg, 100.15, 1, 1000, 1000));
  assert(probe.lastSamples() == 0); // Odom trimming is not a newcomer.
  assert(probe.shouldReview(wall, route, 0, route.front(), {route, different}, cfg, 100.2, 2, 1000, 1000));

  PartialRouteRetryGate retry; const Vec3 end{10, 0, 1.5}, goal{30, 0, 1.5};
  assert(retry.ready(0, end, goal, 1.5));
  retry.rejected(1, end, goal, 1.5);
  assert(!retry.ready(1.1, end, goal, 1.5));
  assert(retry.ready(1.25, end, goal, 1.5));
  retry.rejected(1.25, end, goal, 1.5);
  assert(!retry.ready(1.74, end, goal, 1.5));
  assert(retry.ready(1.75, end, goal, 1.5));
  retry.rejected(1.75, end, goal, 1.5);
  assert(!retry.ready(2.7, end, goal, 1.5));
  assert(retry.ready(2.75, end, goal, 1.5));
  assert(retry.ready(1.8, end, goal, 1.2));
  assert(retry.ready(1.8, end + Vec3{1, 0, 0}, goal, 1.5));
  assert(retry.ready(1.8, end, goal + Vec3{1, 0, 0}, 1.5));
  assert(retry.ready(.5, end, goal, 1.5));
  retry.clear(); assert(retry.ready(1.8, end, goal, 1.5));
  std::cout << "replan_schedule_test: PASS bounded entrance review and partial retry wakeups\n";
}
