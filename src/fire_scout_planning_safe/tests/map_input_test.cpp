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
  std::cout<<"map_input_test: PASS stale-fusion/fresh-local, both-stale rejection, clock skew, conservative occupied and no stale remote free\n";
}
