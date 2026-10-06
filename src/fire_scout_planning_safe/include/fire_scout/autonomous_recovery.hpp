#pragma once
#include "grid.hpp"
#include <chrono>
#include <functional>
#include <queue>
#include <unordered_map>
#include <vector>
namespace fire_scout {
struct RecoveryCampaignConfig {
  double max_distance{4.0},cooldown{2.5},novelty_distance{.30};int max_actions{6};
  bool valid()const{return std::isfinite(max_distance)&&max_distance>=.8&&max_distance<=6&&
    std::isfinite(cooldown)&&cooldown>=1&&cooldown<=15&&std::isfinite(novelty_distance)&&
    novelty_distance>=.2&&novelty_distance<=.6&&max_actions>=2&&max_actions<=8;}
};
// One stall episode, independent of mission messages. A short LocalRecovery
// burst may be renewed only after a new route has passed this total budget and
// visited-endpoint gate. Escape motion cannot replenish its own allowance.
class RecoveryCampaign {
 public:
  RecoveryCampaignConfig cfg;
  bool retryReady(double now)const{return cfg.valid()&&std::isfinite(now)&&!active_&&now>=next_retry_&&
      actions_<cfg.max_actions&&spent_<cfg.max_distance-.2;}
  bool novel(Vec3 target)const{for(Vec3 p:visited_)if(distance(p,target)<cfg.novelty_distance)return false;return true;}
  bool permits(double now,const std::vector<Vec3>&route)const{
    if(!retryReady(now)||route.size()<2||!finite(route.front())||!finite(route.back()))return false;
    const double length=pathLength(route);
    return std::isfinite(length)&&length>=.2&&distance(route.front(),route.back())>=.30&&
      spent_+length<=cfg.max_distance+1e-8&&novel(route.back());
  }
  bool begin(double now,const std::vector<Vec3>&route){
    if(!permits(now,route))return false;
    if(visited_.empty())visited_.push_back(route.front());
    visited_.push_back(route.back());spent_+=pathLength(route);++actions_;active_=true;progress_ready_=false;return true;
  }
  void finish(double now){if(active_){active_=false;next_retry_=now+cfg.cooldown;progress_ready_=false;}}
  void searched(double now){next_retry_=std::max(next_retry_,now+1.0);}
  void normalProgress(Vec3 p){
    if(active_||!finite(p))return;
    if(!progress_ready_){progress_anchor_=p;progress_ready_=true;}
    else if(distance(p,progress_anchor_)>.75){reset();progress_anchor_=p;progress_ready_=true;}
  }
  void reset(){spent_=0;actions_=0;next_retry_=-1e9;active_=progress_ready_=false;visited_.clear();}
  int actions()const{return actions_;}double distanceSpent()const{return spent_;}
  bool exhausted()const{return actions_>=cfg.max_actions||spent_>=cfg.max_distance-.2;}
 private:
  double spent_{0},next_retry_{-1e9};int actions_{0};bool active_{false},progress_ready_{false};
  Vec3 progress_anchor_{};std::vector<Vec3>visited_;
};
struct ObservedEscapeConfig {
  double step{.25},max_length{1.20},max_height_change{.50},max_ms{25};int max_checks{256};
};
// Small 3-D graph over observed free space. It can take a side leg before
// advancing, unlike the old single radial move. Every edge must pass the
// caller's complete body/observation/glass certificate. No global SDF is used.
template<class Certificate,class Clearance,class Novel>
std::vector<Vec3> observedEscapeRoute(Vec3 start,Vec3 goal,const ObservedEscapeConfig&cfg,
    Certificate certified,Clearance clearance,Novel novel){
  if(!finite(start)||!finite(goal)||!std::isfinite(cfg.step)||!std::isfinite(cfg.max_length)||!std::isfinite(cfg.max_height_change)||cfg.step<.15||cfg.max_length<.4||cfg.max_length>2||
     cfg.max_height_change<0||cfg.max_height_change>.75||cfg.max_checks<1||cfg.max_checks>2048||
     !std::isfinite(cfg.max_ms)||cfg.max_ms<=0||cfg.max_ms>100)return {};
  const auto began=std::chrono::steady_clock::now();
  struct Node{Vec3 p;double length;int parent;};struct Item{double score;int id;bool operator<(const Item&o)const{return score<o.score;}};
  std::vector<Node>nodes{{start,0,-1}};std::priority_queue<Item>open;open.push({0,0});
  std::unordered_map<Key,double,KeyHash>cost;cost[key(start,cfg.step*.5)]=0;
  int checks=0,best_id=-1;double best=-1e100;const double initial=clearance(start);
  while(!open.empty()&&checks<cfg.max_checks){
    if(std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count()>=cfg.max_ms)break;
    const int id=open.top().id;open.pop();const Node n=nodes[id];
    for(int i=0;i<10&&checks<cfg.max_checks;++i){
      if(std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count()>=cfg.max_ms)break;
      Vec3 d=i<8?Vec3{std::cos(2*pi*i/8),std::sin(2*pi*i/8),0}:Vec3{0,0,i==8?1.:-1.};
      Vec3 p=n.p+d*cfg.step;double length=n.length+cfg.step;
      if(length>cfg.max_length+1e-8||std::abs(p.z-start.z)>cfg.max_height_change+1e-8)continue;
      Key k=key(p,cfg.step*.5);auto old=cost.find(k);if(old!=cost.end()&&old->second<=length+1e-8)continue;
      ++checks;if(!certified(n.p,p))continue;cost[k]=length;
      const double progress=distance(start,goal)-distance(p,goal),gain=clearance(p)-initial;
      const double score=2*progress+1.5*gain-.12*length;
      int added=int(nodes.size());nodes.push_back({p,length,id});open.push({score,added});
      if(distance(start,p)>=.35&&(progress>=.15||gain>=.08)&&novel(p)&&score>best){best=score;best_id=added;}
    }
  }
  if(best_id<0)return {};
  std::vector<Vec3>route;for(int id=best_id;id>=0;id=nodes[id].parent)route.push_back(nodes[id].p);
  std::reverse(route.begin(),route.end());
  // Certified shortcuts preserve the same endpoint; no spline is allowed to
  // cut into unknown cells. Active execution recertifies all remaining legs.
  std::vector<Vec3>shortened{route.front()};size_t at=0;
  while(at+1<route.size()){size_t next=at+1;for(size_t j=route.size()-1;j>at+1;--j)if(certified(route[at],route[j])){next=j;break;}
    shortened.push_back(route[next]);at=next;}
  return shortened;
}
} // namespace fire_scout
