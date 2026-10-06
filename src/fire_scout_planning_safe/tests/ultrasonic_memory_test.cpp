#include "fire_scout/ultrasonic.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

using namespace fire_scout;

namespace {
SonarLayer layer(){
  SonarLayer value;
  value.resolution=.10;
  value.half_height=0;
  value.ttl=4.0;
  value.confirmation_window=.8;
  value.confirm_frames=2;
  value.clear_confirm_frames=2;
  value.clear_endpoint_guard=.12;
  return value;
}
}

int main(){
  const float inf=std::numeric_limits<float>::infinity();
  const auto clear=parseSonar({inf},0,0,.05,3.0,10*pi/180,true);
  assert(clear.valid);
  assert(!clear.has_hit);
  assert(clear.endpoints.empty());
  assert(clear.free_endpoints.size()==1);
  assert(std::abs(clear.distance-3.0)<1e-9);

  const auto hit=parseSonar({1.0f},0,0,.05,3.0,10*pi/180,true);
  assert(hit.valid&&hit.has_hit);
  assert(hit.endpoints.size()==1&&hit.free_endpoints.size()==1);

  auto expiring=layer();
  expiring.integrate({{.95,0,1.05}},10.0);
  expiring.integrate({{.95,0,1.05}},10.1);
  assert(!expiring.occupied(10.1).empty());
  assert(expiring.occupied(14.2).empty());
  assert(expiring.expiredTotal()==1);

  auto actively_cleared=layer();
  actively_cleared.integrate({{.95,0,1.05}},20.0);
  actively_cleared.integrate({{.95,0,1.05}},20.1);
  assert(!actively_cleared.occupied(20.1).empty());
  const Vec3 origin{0,0,1.05};
  const std::vector<Vec3>free_ray{{2.0,0,1.05}};
  assert(actively_cleared.clearObserved(origin,free_ray,20.2)==0);
  assert(actively_cleared.clearObserved(origin,free_ray,20.3)==1);
  assert(actively_cleared.occupied(20.3).empty());
  assert(actively_cleared.clearedTotal()==1);

  // A current hit at the measured endpoint is protected by the endpoint guard.
  auto endpoint_guard=layer();
  endpoint_guard.integrate({{1.95,0,1.05}},30.0);
  endpoint_guard.integrate({{1.95,0,1.05}},30.1);
  endpoint_guard.clearObserved(origin,free_ray,30.2);
  endpoint_guard.clearObserved(origin,free_ray,30.3);
  assert(!endpoint_guard.occupied(30.3).empty());

  // Even if a caller supplies a resettable clock, a backwards epoch cannot
  // leave future-dated cells immortal.
  auto rewound=layer();
  rewound.integrate({{.95,0,1.05}},100.0);
  rewound.integrate({{.95,0,1.05}},100.1);
  assert(!rewound.occupied(100.1).empty());
  assert(rewound.occupied(1.0).empty());
  assert(rewound.timeResetTotal()==1);

  SonarGuard guard;
  SonarReading close;close.valid=close.has_hit=true;close.distance=.1;
  assert(guard.evaluate(close,true,0,0).speed_limit==0);
  guard.reset();
  SonarReading far;far.valid=true;far.distance=3.0;
  assert(guard.evaluate(far,true,0,0).speed_limit>0);

  std::cout<<"ultrasonic_memory_test: PASS\n";
  return 0;
}
