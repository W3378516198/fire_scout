#pragma once
#include "grid.hpp"
#include <iomanip>
#include <sstream>
#include <string>

namespace fire_scout {
// A bounded diagnostic only. Never changes occupancy or motion permission.
inline std::string mapConflictSnapshot(const Grid&grid,Vec3 current,const std::string&frame){
  std::vector<std::pair<double,Vec3>> closest;size_t nearby=0;
  for(Key k:grid.occupied){
    const Vec3 p=grid.point(k);const double d=distance(p,current);
    if(d>2.)continue;
    ++nearby;
    auto at=std::lower_bound(closest.begin(),closest.end(),d,
      [](const auto&a,double b){return a.first<b;});
    closest.insert(at,{d,p});if(closest.size()>12)closest.pop_back();
  }
  std::ostringstream out;out<<std::fixed<<std::setprecision(3);
  out<<"MAP_CONFLICT_SNAPSHOT frame="<<frame<<" position="<<current.x<<','<<current.y<<','<<current.z
    <<" resolution="<<grid.cfg.resolution<<" radius="<<grid.cfg.inflation_xy
    <<" half_height="<<grid.cfg.inflation_z<<" nearby_occupied="<<nearby<<" samples="<<closest.size()<<" voxels=";
  for(size_t i=0;i<closest.size();++i){
    if(i)out<<'|';
    const Vec3 p=closest[i].second;out<<p.x<<','<<p.y<<','<<p.z;
  }
  return out.str();
}
} // namespace fire_scout
