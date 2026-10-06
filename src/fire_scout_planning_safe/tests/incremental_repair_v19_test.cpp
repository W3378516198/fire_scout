#include "fire_scout/glass_risk.hpp"
#include "fire_scout/planner.hpp"
#include "fire_scout/ultrasonic.hpp"

#include <cassert>
#include <iostream>

using namespace fire_scout;

int main() {
  GridConfig grid_config;
  grid_config.resolution = .20;
  grid_config.inflation_xy = .16;
  grid_config.inflation_z = .12;
  grid_config.preferred_clearance = .10;
  grid_config.z_min = .50;
  grid_config.z_max = 2.50;

  Grid clear_grid(grid_config);
  clear_grid.update({}, {});
  const Vec3 start{0, 0, 1}, goal{6, 0, 1};
  const std::vector<Vec3> committed{{0, 0, 1}, {1, 0, 1}, {2, 0, 1},
                                    {3, 0, 1}, {4, 0, 1}, {5, 0, 1},
                                    {6, 0, 1}};

  // A retained route is joined ahead of the live vehicle.  The old
  // current->perpendicular-projection stitch was a 90--180 degree micro turn
  // and made the follower stop and return to a point already behind it.
  const auto forward_suffix=routeSuffixAt(committed,{.10,.03,1},0);
  assert(forward_suffix.size()>=3);
  const Vec3 connector=forward_suffix[1]-forward_suffix[0];
  const Vec3 outgoing=forward_suffix[2]-forward_suffix[1];
  const double join_angle=std::acos(std::clamp(
    dot(connector,outgoing)/(norm(connector)*norm(outgoing)),-1.,1.));
  assert(join_angle<80*pi/180);
  assert(dot(connector,outgoing)>0);
  assert(!routeInvalidationConfirmed(.4,1.5,false,2,.3,2,.2,2,.6));
  assert(!routeInvalidationConfirmed(.4,1.5,true,1,.3,2,.2,2,.6));
  assert(routeInvalidationConfirmed(.4,1.5,true,2,.3,2,.2,2,.6));
  assert(!routeInvalidationConfirmed(3.0,1.5,true,2,.3,2,.2,2,.6));
  assert(routeInvalidationConfirmed(3.0,1.5,false,2,.7,2,.2,2,.6));

  PlannerConfig config;
  config.allow_unknown = true;
  config.cruise_altitude = 1.0;
  config.search_margin = 2.0;
  config.max_search_margin = 4.0;
  config.max_plan_distance = 10.0;
  config.max_expansions = 80000;
  config.max_search_ms = 300.0;
  config.max_retry_search_ms = 300.0;
  config.enable_3d_search = false;
  config.local_repair_enabled = true;
  config.repair_pre_margin = .45;
  config.repair_post_margin = .75;
  config.repair_max_span = 4.0;
  config.repair_search_margin = 1.5;
  config.repair_search_ms = 250.0;

  Planner planner(config);
  assert(!planner.plan(clear_grid, start, goal).points.empty());
  planner.setCommittedRoute(committed);

  KeySet occupied;
  occupied.insert(key({3, 0, 1}, grid_config.resolution));
  Grid changed_grid(grid_config);
  changed_grid.update(std::move(occupied), {});
  const auto inspection = inspectRoute(changed_grid, committed, start, true);
  assert(!inspection.valid);
  const auto window = localRepairWindow(inspection, .45, .75, 4.0);
  assert(window.valid);
  assert(window.prefix.back().x < 3.0);
  assert(window.tail.front().x > 3.0);

  planner.requestRepair();
  const auto repaired = planner.plan(changed_grid, start, goal);
  assert(repaired.locally_patched);
  assert(repaired.reason == "PATCHED_ROUTE_REJOIN");
  assert(routeGeometryValid(changed_grid, repaired.points, true));
  assert(distance(repaired.points.front(), start) < 1e-6);
  assert(distance(repaired.points.back(), goal) < 1e-6);
  assert(repaired.patch_end > repaired.patch_begin);
  assert(repaired.patch_begin >= 2);
  assert(distance(repaired.points[1], committed[1]) < 1e-6);
  assert(repaired.patch_end + 2 < repaired.points.size());
  assert(distance(repaired.points[repaired.patch_end + 1], committed[5]) < 1e-6);
  assert(distance(repaired.points[repaired.patch_end + 2], committed[6]) < 1e-6);

  // Finite sonar observations need repeat confirmation and then expire.
  SonarLayer layer;
  layer.resolution = .20;
  layer.half_height = .20;
  layer.confirm_frames = 2;
  layer.confirmation_window = .80;
  layer.ttl = 4.0;
  layer.integrate({{1, 0, 1}}, 1.0);
  assert(layer.occupied(1.0).empty());
  layer.integrate({{1, 0, 1}}, 1.2);
  assert(!layer.occupied(1.2).empty());
  assert(layer.occupied(5.3).empty());

  // Unlocalizable too-close echoes are conservative briefly, never permanent.
  UnlocalizedSonarRisk risk;
  risk.add({0, 0, 1}, 0, 10 * pi / 180.0, 1.0);
  assert(risk.near({-.2, 0, 1}, {.2, 0, 1}, .2, 2.0, 2.0));
  assert(!risk.near({-.2, 0, 1}, {.2, 0, 1}, .2, 3.1, 2.0));
  risk.prune(3.1, 2.0);
  assert(risk.size() == 0);

  std::cout << "incremental_repair_v19_test: PASS\n";
  return 0;
}
