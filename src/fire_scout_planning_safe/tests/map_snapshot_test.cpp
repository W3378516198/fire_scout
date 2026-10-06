#include "fire_scout/map_snapshot.hpp"
#include "fire_scout/map_pair_buffer.hpp"
#include <cassert>
#include <chrono>
#include <iostream>
#include <random>
using namespace fire_scout;
template<class F>void rejects(F f){bool rejected=false;try{f();}catch(const std::runtime_error&){rejected=true;}assert(rejected);}
int main(){
  using namespace map_snapshot;
  KeySet occupied{{-9,-2,3},{0,0,0},{1,0,0},{INT32_MIN,7,0},{INT32_MAX,7,0}};
  KeySet free{{-8,-2,3},{-7,-2,3},{0,0,0},{2,0,0},{3,0,0}};
  auto packet=encode(occupied,free,.18);auto decoded=decode(packet,.18);
  free.erase({0,0,0});assert(decoded.occupied==occupied&&decoded.free==free);
  assert(decoded.counts.occupied==occupied.size()&&decoded.counts.free==free.size());
  assert(decode(encode({}, {}, .1),.1).occupied.empty());
  Region region{{-8,-2,3},{-7,-2,3},true};
  auto clipped=decode(packet,.18,100,region);
  assert(clipped.occupied.empty()&&clipped.free==KeySet({{-8,-2,3},{-7,-2,3}}));
  rejects([&]{decode(packet,.1);});rejects([&]{decode(packet,.18,2);});
  auto truncated=packet;truncated.pop_back();rejects([&]{decode(truncated,.18);});
  auto corrupt=packet;write32(corrupt.data()+4,99);rejects([&]{decode(corrupt,.18);});
  corrupt=packet;write32(corrupt.data()+record_bytes+12,UINT32_MAX);rejects([&]{decode(corrupt,.18);});
  corrupt=packet;write32(corrupt.data()+record_bytes+16,9);rejects([&]{decode(corrupt,.18);});
  corrupt=packet;write32(corrupt.data()+8,0);rejects([&]{decode(corrupt,.18);});
  corrupt=packet;corrupt.insert(corrupt.end(),packet.begin()+record_bytes,packet.begin()+2*record_bytes);
  rejects([&]{decode(corrupt,.18);});
  std::mt19937 rng(19);
  for(int trial=0;trial<30;++trial){
    KeySet o,f;
    for(int i=0;i<400;++i){Key k{int(rng()%40)-20,int(rng()%20)-10,int(rng()%8)-4};
      (rng()%3?f:o).insert(k);}
    auto result=decode(encode(o,f,.18),.18);
    for(Key k:o)f.erase(k);
    assert(result.occupied==o&&result.free==f);
  }
  // Reproduce legacy starvation: occupied keeps advancing while the matching
  // large free samples lag by more than the four-entry pairing window.
  MapPairBuffer<int> pairs(4);
  for(int i=1;i<=20;++i){pairs.putOccupied(i,i);if(i>6)pairs.putFree(i-6,i-6);}
  assert(!pairs.takeNewestPair());
  // Each new compact sample is independently decodable after arbitrary loss.
  auto next=decode(encode({{99,2,3}},{{98,2,3}},.18),.18);
  assert(next.occupied.count({99,2,3})&&next.free.count({98,2,3}));
  // A dense 800k free-space map verifies lossless compression at map scale.
  KeySet big;for(int z=0;z<40;++z)for(int y=0;y<100;++y)for(int x=-100;x<100;++x)big.insert({x,y,z});
  const auto start=std::chrono::steady_clock::now();auto compressed=encode({},big,.18);
  auto roundtrip=decode(compressed,.18);assert(roundtrip.free==big);
  const auto ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
  assert(compressed.size()<big.size()*16/10);
  std::cout<<"map_snapshot_test: PASS; synthetic_free_cells="<<big.size()
    <<" xyz_bytes="<<big.size()*16<<" snapshot_bytes="<<compressed.size()
    <<" codec_roundtrip_ms="<<ms<<'\n';
}
