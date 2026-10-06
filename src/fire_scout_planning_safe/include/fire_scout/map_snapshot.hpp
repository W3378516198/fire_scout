#pragma once
#include "grid.hpp"
#include <cstring>
#include <limits>
#include <vector>

namespace fire_scout {
// Lossless, self-contained occupied+free snapshot. Runs extend along x; an
// interrupted/missing DDS sample cannot leave half a map committed. No deltas
// depend on a previous packet. The ROS envelope supplies frame and source time.
namespace map_snapshot {
constexpr uint32_t magic = 0x46534d31U;
constexpr uint32_t version = 1;
constexpr size_t record_bytes = 24;
constexpr size_t default_max_cells = 4000000;
inline uint32_t read32(const uint8_t *p) {
  return uint32_t(p[0]) | uint32_t(p[1])<<8 | uint32_t(p[2])<<16 | uint32_t(p[3])<<24;
}
inline void write32(uint8_t *p,uint32_t v) {
  for(size_t i=0;i<4;++i)p[i]=uint8_t(v>>(8*i));
}
inline int32_t signed32(uint32_t v) {
  // Avoid implementation-defined out-of-range unsigned->signed conversion.
  return v<=uint32_t(INT32_MAX)?int32_t(v):int32_t(int64_t(v)-4294967296LL);
}
inline uint32_t floatBits(float v){uint32_t b;std::memcpy(&b,&v,4);return b;}
inline float bitsFloat(uint32_t b){float v;std::memcpy(&v,&b,4);return v;}
struct Counts {size_t occupied{0},free{0},runs{0};};
struct Region {Key lo{},hi{};bool enabled{false};};
struct Decoded {KeySet occupied,free;Counts counts;};
inline std::vector<uint8_t> encode(const KeySet &occupied,const KeySet &free,double resolution) {
  if(!std::isfinite(resolution)||resolution<.05||resolution>.5 ||
     occupied.size()>UINT32_MAX || free.size()>UINT32_MAX)
    throw std::runtime_error("Invalid snapshot resolution/count");
  std::vector<uint8_t> out(record_bytes,0);
  write32(out.data(),magic);write32(out.data()+4,version);
  write32(out.data()+8,uint32_t(occupied.size()));
  write32(out.data()+20,floatBits(float(resolution)));
  size_t free_count=0;
  const auto append=[&](const KeySet &cells,uint32_t state){
    std::vector<Key> sorted;sorted.reserve(cells.size());
    for(Key k:cells)if(state==1 || !occupied.count(k))sorted.push_back(k);
    if(state==2)free_count=sorted.size();
    std::sort(sorted.begin(),sorted.end(),[](Key a,Key b){
      return a.z!=b.z?a.z<b.z:a.y!=b.y?a.y<b.y:a.x<b.x;
    });
    for(size_t i=0;i<sorted.size();){
      const Key start=sorted[i];size_t j=i+1;
      while(j<sorted.size() && sorted[j].z==start.z && sorted[j].y==start.y &&
            int64_t(sorted[j].x)==int64_t(start.x)+int64_t(j-i))++j;
      const size_t offset=out.size();out.resize(offset+record_bytes,0);auto *p=out.data()+offset;
      write32(p,uint32_t(start.x));write32(p+4,uint32_t(start.y));write32(p+8,uint32_t(start.z));
      write32(p+12,uint32_t(j-i));write32(p+16,state);
      write32(p+20,floatBits(float(resolution)));i=j;
    }
  };
  append(occupied,1);append(free,2);write32(out.data()+12,uint32_t(free_count));return out;
}
inline Counts validate(const std::vector<uint8_t> &data,double resolution,
                       size_t max_cells=default_max_cells) {
  if(data.size()<record_bytes || data.size()%record_bytes ||
     read32(data.data())!=magic || read32(data.data()+4)!=version ||
     read32(data.data()+16)!=0)
    throw std::runtime_error("Invalid compact map header");
  const float packet_resolution=bitsFloat(read32(data.data()+20));
  if(!std::isfinite(resolution)||resolution<.05||resolution>.5 ||
     !std::isfinite(packet_resolution)||std::abs(packet_resolution-resolution)>1e-6)
    throw std::runtime_error("Snapshot voxel_size mismatch");
  Counts c{read32(data.data()+8),read32(data.data()+12),data.size()/record_bytes-1};
  if(!max_cells || c.occupied>max_cells || c.free>max_cells-c.occupied)
    throw std::runtime_error("Snapshot exceeds voxel limit");
  size_t occupied=0,free=0;
  Key previous{};uint32_t previous_count=0,previous_state=0;
  for(size_t offset=record_bytes;offset<data.size();offset+=record_bytes){
    const auto *p=data.data()+offset;
    Key k{signed32(read32(p)),signed32(read32(p+4)),signed32(read32(p+8))};
    const uint32_t n=read32(p+12),state=read32(p+16);
    if(!n || (state!=1&&state!=2) || n>max_cells ||
       int64_t(k.x)+int64_t(n)-1>INT32_MAX || read32(p+20)!=read32(data.data()+20))
      throw std::runtime_error("Invalid compact map run");
    if(state<previous_state || (state==previous_state &&
       (k.z<previous.z || (k.z==previous.z && (k.y<previous.y ||
        (k.y==previous.y && int64_t(k.x)<int64_t(previous.x)+previous_count))))))
      throw std::runtime_error("Unordered/overlapping compact map runs");
    auto &count=state==1?occupied:free;
    if(count>max_cells-n)throw std::runtime_error("Snapshot run count overflow");
    count+=n;previous=k;previous_count=n;previous_state=state;
  }
  if(occupied!=c.occupied || free!=c.free)
    throw std::runtime_error("Snapshot voxel count mismatch");
  return c;
}
inline Decoded decode(const std::vector<uint8_t> &data,double resolution,
                      size_t max_cells=default_max_cells,Region region={}) {
  Decoded result;result.counts=validate(data,resolution,max_cells);
  size_t reserve_limit=max_cells;
  if(region.enabled){
    if(region.lo.x>region.hi.x || region.lo.y>region.hi.y || region.lo.z>region.hi.z)
      throw std::runtime_error("Invalid compact map region");
    reserve_limit=std::min<size_t>(max_cells,100000);
  }
  result.occupied.reserve(std::min(reserve_limit,result.counts.occupied));
  result.free.reserve(std::min(reserve_limit,result.counts.free));
  for(size_t offset=record_bytes;offset<data.size();offset+=record_bytes){
    const auto *p=data.data()+offset;
    Key k{signed32(read32(p)),signed32(read32(p+4)),signed32(read32(p+8))};
    int64_t lo=k.x,hi=int64_t(k.x)+read32(p+12)-1;
    if(region.enabled){
      if(k.y<region.lo.y||k.y>region.hi.y||k.z<region.lo.z||k.z>region.hi.z)continue;
      lo=std::max(lo,int64_t(region.lo.x));hi=std::min(hi,int64_t(region.hi.x));
    }
    auto &cells=read32(p+16)==1?result.occupied:result.free;
    for(int64_t x=lo;x<=hi;++x){k.x=int(x);cells.insert(k);}
  }
  // Occupied always wins a contradictory packet. Never derive free cells from
  // a missing or malformed packet or by mixing different source times.
  for(Key k:result.occupied)result.free.erase(k);
  return result;
}
} // namespace map_snapshot
} // namespace fire_scout
