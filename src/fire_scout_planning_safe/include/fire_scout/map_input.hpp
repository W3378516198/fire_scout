#pragma once
#include "grid.hpp"
#include "map_snapshot.hpp"
#include <cstdint>
#include <memory>

namespace fire_scout {
enum class MapInputSource {None,Primary,LocalFallback};
inline const char *mapInputName(MapInputSource s){
  return s==MapInputSource::Primary?"PRIMARY":s==MapInputSource::LocalFallback?"LOCAL_FALLBACK":"NONE";
}
inline bool freshMapStamp(int64_t source,int64_t now,double timeout){
  const double age=double(now-source)*1e-9;
  return source>0 && age>=-.05 && age<=timeout;
}
inline MapInputSource selectMapInput(int64_t primary,int64_t local,int64_t now,
                                     double timeout,bool fallback){
  if(freshMapStamp(primary,now,timeout))return MapInputSource::Primary;
  if(fallback&&freshMapStamp(local,now,timeout))return MapInputSource::LocalFallback;
  return MapInputSource::None;
}
// Near-field snapshots supplement a still-fresh global map. They cannot make
// that global map fresh, and an older local packet must never supersede it.
inline bool localOverlayEligible(int64_t primary,int64_t local,int64_t now,
                                 double timeout){
  return local>primary && freshMapStamp(primary,now,timeout) &&
    freshMapStamp(local,now,timeout);
}
inline map_snapshot::Region localOverlayRegion(const GridConfig &config,
                                               Vec3 position,double radius){
  if(!finite(position)||!std::isfinite(radius)||radius<=0||radius>20 ||
     !std::isfinite(config.resolution)||config.resolution<=0)
    throw std::runtime_error("Invalid local overlay region");
  // Include the entire vehicle cylinder at the boundary and every permitted
  // flight altitude, so clipping cannot hide a low/high obstacle on a repair.
  const double reach=radius+config.inflation_xy;
  return {key({position.x-reach,position.y-reach,config.z_min-config.inflation_z},config.resolution),
          key({position.x+reach,position.y+reach,config.z_max+config.inflation_z},config.resolution),true};
}
// Source timestamps describe complete packets, not the observation age of
// each persistent free voxel. Therefore local FREE must not erase a PRIMARY
// obstacle merely because the local packet is newer. Add fresh local occupied
// cells conservatively; disappearance is resolved by the primary fusion's
// evidence policy. Unrelated primary geometry/free space remains unchanged.
//
// Most local packets add no new occupied cells or repeat the same additions.
// Cache those cases: no full-map copy, bucket rebuild, or evidence-revision
// churn. Every distinct local packet must still be considered separately by
// the caller's invalidation gate, using its original source timestamp.
class LocalOccupiedOverlay {
public:
  std::shared_ptr<Grid> apply(const std::shared_ptr<Grid> &primary,
                              const KeySet &local_occupied){
    if(!primary){clear();return {};}
    KeySet additions;additions.reserve(local_occupied.size());
    for(Key k:local_occupied)if(!primary->occupied.count(k))additions.insert(k);
    if(primary==primary_ && additions==additions_)return combined_;
    primary_=primary;additions_=std::move(additions);
    if(additions_.empty()){combined_=primary;return combined_;}
    KeySet occupied=primary->occupied,free=primary->free;
    occupied.reserve(occupied.size()+additions_.size());
    for(Key k:additions_){occupied.insert(k);free.erase(k);}
    auto next=std::make_shared<Grid>(primary->cfg);
    next->update(std::move(occupied),std::move(free));
    combined_=std::move(next);return combined_;
  }
  void clear(){primary_.reset();combined_.reset();additions_.clear();}
  size_t addedOccupied()const{return additions_.size();}
private:
  std::shared_ptr<Grid> primary_,combined_;
  KeySet additions_;
};
// Old remote OCCUPIED evidence remains a conservative obstacle layer. Remote
// FREE evidence is deliberately excluded: only the complete fresh local pair
// can provide free evidence during a fusion outage. Own observed free clears
// historical peer/body returns, consistently with fusion's own-map priority.
inline std::shared_ptr<Grid> localFallbackGrid(const GridConfig &config,
                                              KeySet occupied,KeySet free,
                                              const Grid *remote){
  if(remote)for(Key k:remote->occupied)if(!free.count(k))occupied.insert(k);
  auto next=std::make_shared<Grid>(config);
  next->update(std::move(occupied),std::move(free));return next;
}
} // namespace fire_scout
