#pragma once
#include "grid.hpp"
namespace fire_scout {
// Refresh a stale hold anchor after a map change or an outward margin drift.
// The command publisher still certifies the swept braking corridor.
inline Vec3 safeHoldAnchor(const Grid &grid,Vec3 current,Vec3 anchor){
  return !grid.segment(current,current) || !grid.segment(current,anchor) ? current : anchor;
}
} // namespace fire_scout
