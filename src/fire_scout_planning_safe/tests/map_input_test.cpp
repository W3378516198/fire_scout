#include "fire_scout/map_input.hpp"
#include "fire_scout/route_memory.hpp"
#include <cassert>
#include <iostream>
using namespace fire_scout;
int main(){
  const int64_t now=136000000000LL,stale=111000000000LL,local=135850000000LL;
  assert(selectMapInput(stale,local,now,5,true)==MapInputSource::LocalFallback);
  assert(selectMapInput(stale,local,now,5,false)==MapInputSource::None);
  assert(selectMapInput(stale,stale,now,5,true)==MapInputSource::None);
  assert(selectMapInput(now,local,now,5,true)==MapInputSource::Primary);
  assert(selectMapInput(now+1000000000LL,local,now,5,true)==MapInputSource::LocalFallback);
  assert(selectMapInput(now+1000000000LL,now+1000000000LL,now,5,true)==MapInputSource::None);
  assert(selectMapInput(0,0,now,5,true)==MapInputSource::None);
  assert(localOverlayEligible(now-1000000000LL,local,now,5));
  assert(!localOverlayEligible(now,local,now,5));
  assert(!localOverlayEligible(stale,local,now,5));
  assert(!localOverlayEligible(now-1000000000LL,now+1000000000LL,now,5));
  GridConfig cfg;cfg.resolution=.1;cfg.inflation_xy=.3;cfg.inflation_z=.2;
  Grid remote(cfg);Key wall{40,20,15},peer_body{0,0,15},remote_free{50,0,15};
  remote.update({wall,peer_body},{remote_free});
  auto result=localFallbackGrid(cfg,{{-30,-30,15}},{{0,0,15},{1,0,15}},&remote);
  assert(result->occupied.count(wall));assert(!result->occupied.count(peer_body));
  assert(!result->free.count(remote_free));assert(result->free.count({0,0,15}));
  assert(remote.occupied.count(peer_body)&&remote.free.count(remote_free));
  auto independent=localFallbackGrid(cfg,{{40,20,15}},{{0,0,15}},nullptr);
  assert(independent->occupied.count(wall));assert(independent->free.count({0,0,15}));
  RouteMemory route;route.set({{0,0,1.5},{12,0,1.5}});
  for(int x=1;x<=8;++x)assert(route.updateProgress({double(x),0,1.5}));
  const double progress=route.progress();assert(route.valid(*result,{8,0,1.5},true));
  assert(route.progress()==progress); // Map selection does not reset route arc progress.
  // Fresh local occupied evidence blocks the same route before slow fusion
  // catches up. Neither a local free return nor a disappearing local obstacle
  // is allowed to clear a primary obstacle. Primary memory stays immutable.
  auto primary=std::make_shared<Grid>(cfg);
  const Key new_obstacle{40,0,15},far_obstacle{500,0,15},far_free{500,20,15};
  primary->update({far_obstacle},{new_obstacle,far_free});
  const auto primary_revision=primary->evidenceRevision();
  LocalOccupiedOverlay overlay;
  auto supplemented=overlay.apply(primary,{new_obstacle,far_obstacle});
  assert(supplemented!=primary && overlay.addedOccupied()==1);
  assert(supplemented->occupied.count(new_obstacle));
  assert(supplemented->occupied.count(far_obstacle));
  assert(!supplemented->free.count(new_obstacle)&&supplemented->free.count(far_free));
  assert(primary->free.count(new_obstacle)&&!primary->occupied.count(new_obstacle));
  assert(primary->evidenceRevision()==primary_revision);
  assert(primary->segment({3,0,1.5},{5,0,1.5},true));
  assert(!supplemented->segment({3,0,1.5},{5,0,1.5},true));
  const auto revision=supplemented->evidenceRevision();
  for(int i=0;i<100;++i){
    assert(overlay.apply(primary,{far_obstacle,new_obstacle})==supplemented);
    assert(supplemented->evidenceRevision()==revision);
  }
  assert(overlay.apply(primary,{far_obstacle})==primary && overlay.addedOccupied()==0);
  // A primary update removes an old remote obstacle; the cache must not carry
  // the old primary or old local additions into this newer map generation.
  auto newer=std::make_shared<Grid>(cfg);newer->update({new_obstacle},{far_free});
  assert(overlay.apply(newer,{new_obstacle})==newer);
  assert(!overlay.apply(newer,{new_obstacle})->occupied.count(far_obstacle));
  overlay.clear();assert(overlay.addedOccupied()==0);
  assert(!overlay.apply({},{}));
  // Decode/hash only near-field voxels, while still validating all run data.
  auto region=localOverlayRegion(cfg,{0,0,1.5},6);
  auto packet=map_snapshot::encode({new_obstacle,far_obstacle},{far_free},cfg.resolution);
  auto bounded=map_snapshot::decode(packet,cfg.resolution,map_snapshot::default_max_cells,region);
  assert(bounded.occupied.count(new_obstacle)&&!bounded.occupied.count(far_obstacle));
  assert(bounded.free.empty());
  std::cout<<"map_input_test: PASS fallback freshness, immutable cached near-field overlay, occupied precedence, bounded decoding and stable arc progress\n";
}
