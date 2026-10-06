#pragma once
#include "grid.hpp"
#include <chrono>
namespace fire_scout {
// This is NOT a collision-free trajectory: the initial conservative envelope
// is already overlapping recorded obstacles. Only continuous separation from
// the frozen initial contact set is authorized. Normal planning radius is intact.
struct SeparationProof {
  Vec3 origin{};KeySet contacts;GridConfig geometry;double radius{0},max_overlap{.15};bool valid{false};
};
inline double cellDistanceXY(Vec3 p,Key k,double r){
  return std::hypot(p.x-std::clamp(p.x,k.x*r,(k.x+1)*r),p.y-std::clamp(p.y,k.y*r,(k.y+1)*r));
}
template<class Visit>bool visitSweepCells(const Grid&g,Vec3 a,Vec3 b,double radius,Visit visit){
  if(!finite(a)||!finite(b)||!std::isfinite(g.cfg.resolution)||g.cfg.resolution<.05||
     !std::isfinite(radius)||radius<=0||radius>g.cfg.inflation_xy+1e-9||radius>2||
     !std::isfinite(g.cfg.inflation_z)||g.cfg.inflation_z<0||g.cfg.inflation_z>2)return false;
  const Vec3 pad{radius+1e-8,radius+1e-8,g.cfg.inflation_z+1e-8};
  Vec3 lo{std::min(a.x,b.x),std::min(a.y,b.y),std::min(a.z,b.z)},hi{std::max(a.x,b.x),std::max(a.y,b.y),std::max(a.z,b.z)};
  lo=lo-pad;hi=hi+pad;
  if(std::max({std::abs(lo.x),std::abs(lo.y),std::abs(lo.z),std::abs(hi.x),std::abs(hi.y),std::abs(hi.z)})>1e6)return false;
  const Key l=g.toKey(lo),h=g.toKey(hi);
  const int64_t count=int64_t(h.x-l.x+1)*(h.y-l.y+1)*(h.z-l.z+1);
  if(count<1||count>20000)return false;
  for(int x=l.x;x<=h.x;++x)for(int y=l.y;y<=h.y;++y)for(int z=l.z;z<=h.z;++z){
    Key k{x,y,z};if(g.sweptCellContact(a,b,k,radius)&&!visit(k))return false;
  }
  return true;
}
inline SeparationProof separationProof(const Grid&g,Vec3 p,double max_overlap=.15){
  SeparationProof proof;proof.origin=p;proof.geometry=g.cfg;proof.max_overlap=max_overlap;
  // Normal motion retains inflation_xy.  An already-overlapping vehicle may
  // consume only the tracking reserve while it moves monotonically away; the
  // configured recovery radius still encloses the tilted rotor disk plus a
  // physical margin.  The bounded overlap also covers one conservative 0.15 m
  // voxel face, without authorizing motion toward that voxel.
  proof.radius=g.cfg.recovery_xy>0?std::min(g.cfg.inflation_xy,g.cfg.recovery_xy):g.cfg.inflation_xy;
  if(!g.insideAltitude(p)||!std::isfinite(max_overlap)||max_overlap<=0||max_overlap>.18||
     max_overlap>=proof.radius)return proof;
  proof.valid=visitSweepCells(g,p,p,proof.radius,[&](Key k){
    if(!g.occupied.count(k))return true;
    const double d=cellDistanceXY(p,k,g.cfg.resolution);
    if(d<proof.radius-max_overlap||d<1e-5)return false;
    proof.contacts.insert(k);return true;
  });return proof;
}
inline bool separatingSweep(const Grid&g,const SeparationProof&p,Vec3 a,Vec3 b,bool observed=false){
  if(!p.valid||!g.insideAltitude(a)||!g.insideAltitude(b)||std::abs(a.z-b.z)>1e-7||
     std::abs(a.z-p.origin.z)>.08||distance(a,p.origin)>.80||distance(b,p.origin)>.80||
     std::abs(g.cfg.resolution-p.geometry.resolution)>1e-9||
     std::abs(g.cfg.inflation_xy-p.geometry.inflation_xy)>1e-9||
     std::abs(g.cfg.inflation_z-p.geometry.inflation_z)>1e-9||
     std::abs(g.cfg.recovery_xy-p.geometry.recovery_xy)>1e-9||
     !std::isfinite(p.radius)||p.radius<=0||p.radius>g.cfg.inflation_xy+1e-9)return false;
  return visitSweepCells(g,a,b,p.radius,[&](Key k){
    if(!g.occupied.count(k))return !observed||g.known(k);
    // New obstacles and re-entry into an already cleared contact are rejected.
    if(!p.contacts.count(k)||!g.sweptCellContact(a,a,k,p.radius)||
       !g.sweptCellContact(p.origin,p.origin,k,p.radius))return false;
    const double r=g.cfg.resolution;
    Vec3 normal{a.x-std::clamp(a.x,k.x*r,(k.x+1)*r),a.y-std::clamp(a.y,k.y*r,(k.y+1)*r),0};
    // Squared distance to a convex voxel is convex along this straight leg.
    // Nonnegative initial derivative proves it never approaches that voxel.
    return norm(normal)>=p.radius-p.max_overlap&&dot(normal,b-a)>=-1e-10;
  });
}
template<class Certificate,class Score>
std::vector<Vec3> separationRoute(Vec3 start,Certificate certified,Score score){
  Vec3 best{};double value=-1e100;
  const auto began=std::chrono::steady_clock::now();
  for(double length:{.35,.50,.65})for(int i=0;i<48;++i){
    if(std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count()>25)
      return value>-1e99?std::vector<Vec3>{start,best}:std::vector<Vec3>{};
    const double theta=2*pi*i/48;Vec3 p=start+Vec3{length*std::cos(theta),length*std::sin(theta),0};
    if(!certified(start,p))continue;
    const double s=score(p)-.15*length;if(s>value){value=s;best=p;}
  }
  return value>-1e99?std::vector<Vec3>{start,best}:std::vector<Vec3>{};
}
// One purposeful observation turn per stagnation episode, independent of goal
// messages. It never commands translation and never treats the turn as free-space
// evidence: the ordinary observation certificate must pass afterwards.
class SeparationObservationTurn {
 public:
  bool begin(double now,Vec3 p,double current_yaw,Vec3 candidate){
    if(used_||!std::isfinite(now)||!finite(p)||!finite(candidate)||!std::isfinite(current_yaw)||distance(p,candidate)<.2)return false;
    origin_=p;target_=std::atan2(candidate.y-p.y,candidate.x-p.x);command_=current_yaw;
    began_=now;settled_=-1;used_=active_=true;rate_=0;return true;
  }
  double step(double now,double dt,double measured_yaw){
    if(!active_)return command_;
    if(!std::isfinite(now)||!std::isfinite(dt)||!std::isfinite(measured_yaw)||dt<=0||dt>.2||now<began_||now-began_>9.){active_=false;return command_;}
    const double desired=std::clamp(2*wrap(target_-command_),-.55,.55);
    rate_+=std::clamp(desired-rate_,-dt,dt);command_=wrap(command_+rate_*dt);
    if(std::abs(wrap(target_-measured_yaw))<.10){if(settled_<0)settled_=now;}
    else settled_=-1;
    if(settled_>=0&&now-settled_>.8)active_=false;
    return command_;
  }
  void normalProgress(Vec3 p){if(used_&&!active_&&distance(p,origin_)>.75)reset();}
  void reset(){active_=used_=false;rate_=0;}
  bool active()const{return active_;}bool used()const{return used_;}
 private:
  Vec3 origin_{};double target_{0},command_{0},rate_{0},began_{0},settled_{-1};bool active_{false},used_{false};
};
} // namespace fire_scout
