#include "fire_scout/racer_task_allocator.hpp"
#include "fire_scout/route_switch.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

using namespace fire_scout;

int main() {
  RouteSwitchConfig config;
  assert(config.valid());

  const Vec3 current{0, 0, 1};
  const std::vector<Vec3> east{{0, 0, 1}, {5, 0, 1}, {10, 0, 1}};
  const std::vector<Vec3> west{{0, 0, 1}, {-5, 0, 1}, {-10, 0, 1}};
  const std::vector<Vec3> north{{0, 0, 1}, {0, 5, 1}, {0, 10, 1}};
  const double ten_degrees = 10.0 * pi / 180.0;
  const std::vector<Vec3> slight{{0, 0, 1},
      {5 * std::cos(ten_degrees), 5 * std::sin(ten_degrees), 1},
      {10 * std::cos(ten_degrees), 10 * std::sin(ten_degrees), 1}};

  const auto reversal = routeTransition(east, west, current, config);
  assert(reversal.valid);
  assert(std::abs(reversal.angle - pi) < 1e-8);
  assert(reversal.backtrack > 1.40);

  const auto right_angle = routeTransition(east, north, current, config);
  assert(right_angle.valid);
  assert(std::abs(right_angle.angle - .5 * pi) < 1e-8);
  assert(right_angle.backtrack < 1e-8);

  const auto continuous = routeTransition(east, slight, current, config);
  assert(continuous.valid);
  assert(continuous.angle < config.soft_angle);

  // A slightly shorter reverse route must not replace a valid forward route.
  const auto weak_reverse = assessRouteSwitch(east, west, current, 10.0, 1.0,
                                               config);
  assert(!weak_reverse.cost_allows);
  assert(weak_reverse.required_gain_ratio >= config.reverse_gain_ratio - 1e-9);

  // Reversal remains possible when the incumbent has a genuinely large cost
  // (for example an unavoidable peer conflict); this is deliberately not a ban.
  const auto necessary_reverse = assessRouteSwitch(east, west, current, 30.0,
                                                    4.0, config);
  assert(necessary_reverse.cost_allows);
  assert(necessary_reverse.needs_confirmation);

  const auto smooth_improvement = assessRouteSwitch(east, slight, current,
                                                    10.0, 8.0, config);
  assert(smooth_improvement.cost_allows);
  assert(!smooth_improvement.needs_confirmation);

  auto repeated = reversal;
  repeated.heading += 5.0 * pi / 180.0;
  repeated.endpoint.x += .10;
  assert(samePendingSwitch(reversal, repeated, config));
  repeated.heading += 30.0 * pi / 180.0;
  assert(!samePendingSwitch(reversal, repeated, config));

  const std::vector<Vec3> reserved{{1, 0, 1}};
  assert(conflictsWithReservedSlot({1.5, 0, 1}, reserved, .90));
  assert(!conflictsWithReservedSlot({2.0, 0, 1}, reserved, .90));
  assert(!slotMissingConfirmed(1000000000LL, 2100000000LL, 1.20));
  assert(slotMissingConfirmed(1000000000LL, 2200000000LL, 1.20));

  std::cout << "route_switch_test: PASS\n";
  return 0;
}
