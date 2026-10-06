#pragma once
#include <algorithm>
#include <cmath>
namespace fire_scout {
// A soft slowdown absorbs normal snapshot/transport jitter. The hard deadline
// still forbids navigation; increasing it never makes an old stamp fresh.
inline double mapAgeSpeedFactor(double age,double slow_start,double timeout){
  if(!std::isfinite(age)||!std::isfinite(slow_start)||!std::isfinite(timeout)||
     age<-.05||slow_start<0||timeout<=slow_start||age>=timeout)return 0;
  if(age<=slow_start)return 1;
  const double t=std::clamp((age-slow_start)/(timeout-slow_start),0.,1.);
  return 1-t*t*(3-2*t);
}
} // namespace fire_scout
