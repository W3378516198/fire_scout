#pragma once
#include "geometry.hpp"
#include <map>
#include <string>

namespace fire_scout {
// Both endpoints use this same measured-position exit test. The requested
// radius is carried in the third request pose, not inferred independently.
inline bool passageOutside(Vec3 position,Vec3 anchor,double radius){
  return finite(position)&&finite(anchor)&&std::isfinite(radius)&&radius>0&&
    distance(position,anchor)>radius+.20;
}
struct PassageBid {
  int robot{0};Vec3 anchor{},position{};double received{0},waiting_since{0};
  bool inside{false};
  double release_radius{0};int64_t epoch{0};
};
struct PassageLease {
  Vec3 anchor{};int owner{-1};double last_owner_request{0};
  bool quarantined{false};
  bool entered{false};
  Vec3 owner_anchor{};double release_radius{0};int64_t epoch{0};
};
struct PassageCompletion {
  Vec3 anchor{},position{};double release_radius{0};int64_t epoch{0};
};
// Avoid immediately reserving the just-completed opening again. A real return
// toward it rearms detection; a fixed-duration blind exclusion is unnecessary.
struct PassageDepartureMemory {
  bool have{false};Vec3 anchor{},position{};double radius{0};
  void clear(){have=false;}
  void remember(Vec3 a,Vec3 p,double r){have=true;anchor=a;position=p;radius=r;}
  bool suppress(Vec3 a,Vec3 p)const{
    return have&&distance(a,anchor)<=1.4&&distance(p,a)>radius&&
      distance(p,a)>distance(position,a)-.35;
  }
};
struct PassageWaitMonitor {
  double since{-1},last_report{-1},seconds{0};size_t reports{0};
  void reset(){since=last_report=-1;seconds=0;}
  bool update(double now,bool waiting,double timeout,double repeat){
    if(!waiting){reset();return false;}
    if(since<0||now<since){since=now;last_report=-1;}
    seconds=now-since;
    if(seconds<timeout||(last_report>=0&&now-last_report<repeat))return false;
    last_report=now;++reports;return true;
  }
};
// Local admission progress is independent of a frozen doorway normal. An
// owner that turns after passing the opening can release in any direction.
struct PassageProgress {
  bool entered{false};
  void reset(){entered=false;}
  bool exited(Vec3 position,Vec3 anchor,Vec3 direction,double radius,
      bool granted,bool terminal,bool abandoning){
    const double r=distance(position,anchor);
    if(granted&&r<=radius)entered=true;
    return passageOutside(position,anchor,radius)&&(entered||
      dot(position-anchor,direction)>radius||terminal||abandoning);
  }
};
// FIFO with inside-first admission. A lost owner quarantines the resource:
// timeout is NOT evidence that a doorway is physically empty. No auto regrant.
class PassageScheduler {
public:
  double merge_radius{1.4},request_timeout{.8},exit_radius{1.8};
  size_t capacity{128};
  std::map<int,PassageBid>bids;std::vector<PassageLease>leases;
  std::map<int,PassageCompletion>completed;
  void reset(){bids.clear();leases.clear();completed.clear();}
  void request(PassageBid bid){
    if(!finite(bid.anchor)||!finite(bid.position)||!std::isfinite(bid.received)||
       !std::isfinite(bid.release_radius)||bid.release_radius<0||bid.release_radius>6||bid.epoch<0)return;
    auto previous=bids.find(bid.robot);
    if(previous!=bids.end()&&(bid.received<previous->second.received||bid.epoch<previous->second.epoch))return;
    auto done=completed.find(bid.robot);
    if(done!=completed.end()){
      if(done->second.epoch==bid.epoch&&distance(done->second.anchor,bid.anchor)<=merge_radius&&
         distance(bid.position,bid.anchor)>done->second.release_radius&&
         distance(bid.position,bid.anchor)>
           distance(done->second.position,bid.anchor)-.35)return;
      completed.erase(done);
    }
    auto old=bids.find(bid.robot);
    if(old!=bids.end()&&distance(old->second.anchor,bid.anchor)<=merge_radius)
      bid.waiting_since=old->second.waiting_since;
    else bid.waiting_since=bid.received;
    bids[bid.robot]=bid;
  }
  void release(int robot,Vec3 measured_position){
    bids.erase(robot);
    for(auto &l:leases)if(l.owner==robot&&passageOutside(measured_position,l.owner_anchor,l.release_radius)){
      completed[robot]={l.owner_anchor,measured_position,l.release_radius,l.epoch};
      l.owner=-1;l.quarantined=false;l.entered=false;}
  }
  bool finished(int robot,Vec3 anchor,int64_t epoch)const{
    auto it=completed.find(robot);
    return it!=completed.end()&&it->second.epoch==epoch&&
      distance(it->second.anchor,anchor)<=merge_radius;
  }
  bool admitted(int robot)const{
    auto b=bids.find(robot);if(b==bids.end())return false;
    for(const auto&l:leases)if(l.owner==robot&&!l.quarantined&&
      l.epoch==b->second.epoch&&distance(l.owner_anchor,b->second.anchor)<=merge_radius)return true;
    return false;
  }
  void update(double time){
    // Recycle only ownerless, unused resources. A quarantined owner is never
    // evicted to make room for another door.
    leases.erase(std::remove_if(leases.begin(),leases.end(),[&](const PassageLease&l){
      if(l.owner>=0)return false;
      for(const auto &[robot,b]:bids){(void)robot;
        if(time>=b.received&&time-b.received<=request_timeout&&
           distance(l.anchor,b.anchor)<=merge_radius)return false;}
      return true;
    }),leases.end());
    for(auto &l:leases){
      if(l.owner<0)continue;
      auto b=bids.find(l.owner);
      // Fresh owner position is evidence independently of WHICH opening the
      // owner now requests. In particular, a rejected migration while still
      // inside must be re-evaluated on every later heartbeat. Do not erase the
      // new bid when completing the old lease; timeout alone never frees it.
      const bool fresh=b!=bids.end()&&time>=b->second.received&&
        time-b->second.received<=request_timeout;
      const bool migrating=fresh&&(distance(b->second.anchor,l.owner_anchor)>merge_radius||
        distance(b->second.anchor,l.anchor)>merge_radius);
      if(migrating&&passageOutside(b->second.position,l.owner_anchor,l.release_radius)){
        completed[l.owner]={l.owner_anchor,b->second.position,l.release_radius,l.epoch};
        l.owner=-1;l.quarantined=false;l.entered=false;++migrations_completed;
        continue;
      }
      if(b!=bids.end()&&time>=b->second.received&&
         time-b->second.received<=request_timeout&&
         !migrating){
        l.last_owner_request=time;l.quarantined=false;
        // A fresh same-owner request may resume after a node/clock restart.
        // Keep the original physical resource reserved while adopting its id.
        l.epoch=b->second.epoch;
        l.release_radius=std::max({l.release_radius,exit_radius,b->second.release_radius});
        if(distance(l.owner_anchor,b->second.position)<=l.release_radius)l.entered=true;
        if(l.entered&&passageOutside(b->second.position,l.owner_anchor,l.release_radius)&&
           passageOutside(b->second.position,b->second.anchor,l.release_radius)){
          // Fresh measured exit, not a heartbeat timeout. Suppress stale
          // renewals until this vehicle actually starts approaching again.
          completed[l.owner]={b->second.anchor,b->second.position,l.release_radius,l.epoch};bids.erase(b);
          l.owner=-1;l.entered=false;
        }
      }else l.quarantined=true;
    }
    for(const auto &[robot,b]:bids){
      (void)robot;
      if(time<b.received||time-b.received>request_timeout)continue;
      auto found=std::find_if(leases.begin(),leases.end(),[&](const PassageLease&l){
        return distance(l.anchor,b.anchor)<=merge_radius;});
      if(found==leases.end()&&leases.size()<capacity)leases.push_back({b.anchor,-1,time,false});
    }
    for(auto &l:leases){
      if(l.owner>=0)continue;
      const PassageBid*best=nullptr;
      for(const auto &[robot,b]:bids){(void)robot;
        if(time<b.received||time-b.received>request_timeout||
           distance(l.anchor,b.anchor)>merge_radius)continue;
        // Do not give a robot a second lease until its first is released.
        bool owns=false;for(const auto &other:leases)if(other.owner==b.robot)owns=true;
        if(owns)continue;
        if(!best||(b.inside&&!best->inside)||(b.inside==best->inside&&
            (b.waiting_since<best->waiting_since||
             (b.waiting_since==best->waiting_since&&b.robot<best->robot))))best=&b;
      }
      if(best){l.owner=best->robot;l.last_owner_request=time;l.quarantined=false;
        l.owner_anchor=best->anchor;l.epoch=best->epoch;
        l.release_radius=std::max(exit_radius,best->release_radius);
        l.entered=distance(l.owner_anchor,best->position)<=l.release_radius;}
    }
  }
  size_t migrations_completed{0};
};
} // namespace fire_scout
