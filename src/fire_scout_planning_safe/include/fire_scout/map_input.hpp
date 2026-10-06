#pragma once
#include "grid.hpp"
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
