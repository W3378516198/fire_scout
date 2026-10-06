#pragma once
#include <cstdint>
#include <map>
#include <optional>
#include <utility>
#include <algorithm>

namespace fire_scout {
// Keep a few samples from each map topic. Independent latest-only slots lose
// complete pairs when the larger free cloud arrives after a newer occupied
// cloud. Matching never mixes timestamps and never regresses the committed map.
template<class Value> class MapPairBuffer {
public:
  struct Pair { int64_t stamp; Value occupied,free; };
  explicit MapPairBuffer(size_t capacity=4):capacity_(std::max(size_t(1),capacity)){}
  void putOccupied(int64_t stamp,Value value){put(occupied_,stamp,std::move(value));}
  void putFree(int64_t stamp,Value value){put(free_,stamp,std::move(value));}
  std::optional<Value> occupiedAt(int64_t stamp)const{return at(occupied_,stamp);}
  std::optional<Value> freeAt(int64_t stamp)const{return at(free_,stamp);}
  std::optional<Pair> takeNewestPair(){
    for(auto it=occupied_.rbegin();it!=occupied_.rend();++it){
      const auto f=free_.find(it->first);
      if(f==free_.end())continue;
      Pair pair{it->first,it->second,f->second};committed_=it->first;
      occupied_.erase(occupied_.begin(),occupied_.upper_bound(committed_));
      free_.erase(free_.begin(),free_.upper_bound(committed_));
      return pair;
    }
    return {};
  }
  size_t occupiedSize()const{return occupied_.size();}
  size_t freeSize()const{return free_.size();}
  void clear(){occupied_.clear();free_.clear();committed_=-1;}
private:
  using Samples=std::map<int64_t,Value>;
  size_t capacity_;
  int64_t committed_{-1};
  Samples occupied_,free_;
  void put(Samples&samples,int64_t stamp,Value value){
    if(stamp<0||stamp<=committed_)return;
    samples.insert_or_assign(stamp,std::move(value));
    while(samples.size()>capacity_)samples.erase(samples.begin());
  }
  static std::optional<Value> at(const Samples&samples,int64_t stamp){
    const auto it=samples.find(stamp);
    if(it==samples.end())return {};
    return it->second;
  }
};
}
