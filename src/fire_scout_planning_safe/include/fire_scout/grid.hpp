#pragma once
#include "geometry.hpp"
#include <array>
#include <atomic>
#include <functional>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

namespace fire_scout {
struct Key {
  int x{0}, y{0}, z{0};
  bool operator==(const Key &b) const { return x == b.x && y == b.y && z == b.z; }
  bool operator!=(const Key &b) const { return !(*this == b); }
  bool operator<(const Key &b) const { return x != b.x ? x < b.x : y != b.y ? y < b.y : z < b.z; }
};
struct KeyHash {
  static uint64_t mix(uint64_t v) {
    v ^= v >> 30;
    v *= 0xbf58476d1ce4e5b9ULL;
    v ^= v >> 27;
    v *= 0x94d049bb133111ebULL;
    return v ^ (v >> 31);
  }
  size_t operator()(const Key &k) const noexcept {
    return mix(uint32_t(k.x) + 0x9e3779b97f4a7c15ULL) ^ mix(uint32_t(k.y) + 0x243f6a8885a308d3ULL) ^
           mix(uint32_t(k.z) + 0x13198a2e03707344ULL);
  }
};
using KeySet = std::unordered_set<Key, KeyHash>;
inline Key key(Vec3 p, double r) {
  return {int(std::floor(p.x / r)), int(std::floor(p.y / r)), int(std::floor(p.z / r))};
}
inline Vec3 center(Key k, double r) { return {(k.x + .5) * r, (k.y + .5) * r, (k.z + .5) * r}; }
// Voxel traversal; at an edge/corner also checks all touched side voxels.
// visit returns false to stop traversal. Endpoint cells are included.
template <class F> bool trace(Vec3 a, Vec3 b, double r, F visit, bool supercover = true) {
  Key cur = key(a, r), end = key(b, r);
  if (!visit(cur))
    return false;
  if (cur == end)
    return true;
  double aa[3] = {a.x, a.y, a.z}, dd[3] = {b.x - a.x, b.y - a.y, b.z - a.z};
  int c[3] = {cur.x, cur.y, cur.z}, e[3] = {end.x, end.y, end.z}, sg[3];
  double tm[3], td[3];
  for (int i = 0; i < 3; ++i) {
    sg[i] = (dd[i] > 0) - (dd[i] < 0);
    td[i] = sg[i] ? r / std::abs(dd[i]) : 1e100;
    tm[i] = sg[i] ? ((c[i] + (sg[i] > 0)) * r - aa[i]) / dd[i] : 1e100;
  }
  const int bound = std::abs(e[0] - c[0]) + std::abs(e[1] - c[1]) + std::abs(e[2] - c[2]) + 4;
  for (int n = 0; n < bound; ++n) {
    double next = std::min({tm[0], tm[1], tm[2]});
    int mask = 0;
    for (int i = 0; i < 3; ++i)
      if (tm[i] <= next + 1e-10)
        mask |= 1 << i;
    if (supercover)
      for (int s = 1; s < 8; ++s)
        if ((s & mask) == s && s != mask) {
          Key k{c[0] + ((s & 1) ? sg[0] : 0), c[1] + ((s & 2) ? sg[1] : 0),
                c[2] + ((s & 4) ? sg[2] : 0)};
          if (!visit(k))
            return false;
        }
    for (int i = 0; i < 3; ++i)
      if (mask & (1 << i)) {
        c[i] += sg[i];
        tm[i] += td[i];
      }
    cur = {c[0], c[1], c[2]};
    if (!visit(cur))
      return false;
    if (cur == end)
      return true;
  }
  return false;
}
// A vertical cylinder encloses the rotating propellers, body tilt and tracking reserve.
struct GridConfig {
  double resolution{.25}, inflation_xy{.30}, inflation_z{.25}, z_min{.65}, z_max{3.0};
  double preferred_clearance{.30};
  double recovery_xy{0}; // 0 disables margin-only escape; never smaller than physical body in ROS config.
  double comfort_margin{0}; // Optional clearance beyond the hard body/tracking envelope.
};
class Grid {
public:
  explicit Grid(GridConfig c = {}) : cfg(c) {}
  GridConfig cfg;
  KeySet occupied, free;
  uint64_t evidenceRevision()const{return evidence_revision_;}
  void update(KeySet occ, KeySet fr) {
    static std::atomic<uint64_t> next_revision{0};
    evidence_revision_=++next_revision;
    free=std::move(fr);
    if(initialized_ && occ==occupied)return;
    initialized_=true;occupied=std::move(occ);buckets_.clear();clearance_.clear();
    bucket_size_=std::max(.5,cfg.inflation_xy+cfg.preferred_clearance);
    for(Key k:occupied)buckets_[key(point(k),bucket_size_)].push_back(k);
    // O(number of occupied voxels), not O(occupied * a full inflation kernel).
    // In particular, map callbacks cannot spend seconds expanding every wall.
  }
  Key toKey(Vec3 p) const { return key(p, cfg.resolution); }
  Vec3 point(Key k) const { return center(k, cfg.resolution); }
  bool inside(Key k) const {
    double z = point(k).z;
    // Boundary cells are retained; exact endpoints are checked in segment().
    return z + .5 * cfg.resolution >= cfg.z_min && z - .5 * cfg.resolution <= cfg.z_max;
  }
  bool known(Key k) const { return free.count(k) != 0; }
  // Continuous position query. Unlike clearance(Key), this never substitutes
  // the center of a neighbouring voxel for an actual flight/spline position.
  double clearanceAt(Vec3 p,double limit=.6) const {
    if(!finite(p)||!std::isfinite(limit)||limit<0)return -1;
    double best=limit;
    const double reach=cfg.inflation_xy+limit,r=cfg.resolution;
    nearby(p-Vec3{reach,reach,cfg.inflation_z},p+Vec3{reach,reach,cfg.inflation_z},[&](Key k){
      if(k.z*r>p.z+cfg.inflation_z || (k.z+1)*r<p.z-cfg.inflation_z)return true;
      const double dx=std::max({0.,k.x*r-p.x,p.x-(k.x+1)*r});
      const double dy=std::max({0.,k.y*r-p.y,p.y-(k.y+1)*r});
      best=std::min(best,std::hypot(dx,dy)-cfg.inflation_xy);return true;
    });
    return best;
  }
  bool walkable(Key k, bool unknown = true) const {
    const double z = point(k).z;
    return z >= cfg.z_min && z <= cfg.z_max && clearance(k)>0 && (unknown || known(k));
  }
  double clearance(Key k) const {
    auto found=clearance_.find(k);
    if(found!=clearance_.end())return found->second;
    Vec3 p=point(k);
    double best=std::max(.001,cfg.preferred_clearance);
    double reach=cfg.inflation_xy+best;
    Vec3 lo{p.x-reach,p.y-reach,p.z-cfg.inflation_z};
    Vec3 hi{p.x+reach,p.y+reach,p.z+cfg.inflation_z};
    nearby(lo,hi,[&](Key k){
      const double r=cfg.resolution;
      if(k.z*r>hi.z || (k.z+1)*r<lo.z)return true;
      double dx=std::max({0.,k.x*r-p.x,p.x-(k.x+1)*r});
      double dy=std::max({0.,k.y*r-p.y,p.y-(k.y+1)*r});
      best=std::min(best,std::hypot(dx,dy)-cfg.inflation_xy);
      return best>0;
    });
    clearance_[k]=best;return best;
  }
  double proximity(Key k) const {
    if (cfg.preferred_clearance <= 0) return 0;
    double q = 1. - std::clamp(clearance(k) / cfg.preferred_clearance, 0., 1.);
    return q*q;
  }
  bool segment(Vec3 a, Vec3 b, bool unknown = true) const {
    if (!finite(a) || !finite(b) || a.z < cfg.z_min || a.z > cfg.z_max ||
        b.z < cfg.z_min || b.z > cfg.z_max) return false;
    return geometrySegment(a,b,unknown);
  }
  // Collision geometry is distinct from a mission altitude bound. Exposing
  // this query does NOT authorize motion; boundaryReturn() is the only bounded
  // exception used by the controller after it has already entered NAV.
  bool geometrySegment(Vec3 a,Vec3 b,bool unknown=true)const {
    if(!finite(a)||!finite(b))return false;
    if(!unknown && !trace(a,b,cfg.resolution,[&](Key k){return known(k);}))return false;
    Vec3 lo{std::min(a.x,b.x)-cfg.inflation_xy,std::min(a.y,b.y)-cfg.inflation_xy,
            std::min(a.z,b.z)-cfg.inflation_z};
    Vec3 hi{std::max(a.x,b.x)+cfg.inflation_xy,std::max(a.y,b.y)+cfg.inflation_xy,
            std::max(a.z,b.z)+cfg.inflation_z};
    return nearby(lo,hi,[&](Key k){return !hits(a,b,k,cfg.inflation_xy);});
  }
  bool insideAltitude(Vec3 p)const {return finite(p)&&p.z>=cfg.z_min&&p.z<=cfg.z_max;}
  // An additional operational clearance query, never a smaller body envelope.
  // Continuous cylinder/voxel sweeps avoid missing a wall between samples.
  bool segmentWithMargin(Vec3 a,Vec3 b,double margin,bool unknown=true)const {
    if(!std::isfinite(margin)||margin<0||!insideAltitude(a)||!insideAltitude(b))return false;
    if(!unknown&&!trace(a,b,cfg.resolution,[&](Key k){return known(k);}))return false;
    const double radius=cfg.inflation_xy+margin;
    return nearby({std::min(a.x,b.x)-radius,std::min(a.y,b.y)-radius,std::min(a.z,b.z)-cfg.inflation_z},
      {std::max(a.x,b.x)+radius,std::max(a.y,b.y)+radius,std::max(a.z,b.z)+cfg.inflation_z},
      [&](Key k){return !hits(a,b,k,radius);});
  }
  // Shared continuous primitive for the explicit, bounded separation witness.
  // The radius overload is used only by the slow, monotonic envelope-escape
  // proof.  Ordinary planning and tracking continue to use inflation_xy.
  bool sweptCellContact(Vec3 a,Vec3 b,Key k)const{return hits(a,b,k,cfg.inflation_xy);}
  bool sweptCellContact(Vec3 a,Vec3 b,Key k,double radius)const{
    return std::isfinite(radius)&&radius>=0&&radius<=cfg.inflation_xy+1e-9&&
      hits(a,b,k,radius);
  }
  bool boundaryReturn(Vec3 a,Vec3 b,double max_violation=.20)const {
    if(!finite(a)||!finite(b)||!std::isfinite(max_violation)||max_violation<=0||max_violation>.30||
       std::hypot(b.x-a.x,b.y-a.y)>1e-7||distance(a,b)>.50)return false;
    if(a.z<cfg.z_min){
      if(a.z<cfg.z_min-max_violation||b.z<a.z||b.z>std::min(cfg.z_max,cfg.z_min+.20))return false;
    }else if(a.z>cfg.z_max){
      if(a.z>cfg.z_max+max_violation||b.z>a.z||b.z<std::max(cfg.z_min,cfg.z_max-.20))return false;
    }else return segment(a,b,true);
    return geometrySegment(a,b,true);
  }
  // Recovery may consume the configured horizontal safety margin only.  Every
  // obstacle initially overlapping the normal envelope must be approached no
  // further; all other obstacles keep the full envelope. Cylinder/voxel sweep
  // checks remain continuous. Vertical recovery is deliberately not relaxed.
  bool segmentFrom(Vec3 a, Vec3 b, bool unknown=true) const {
    if(segment(a,b,unknown))return true;
    if(cfg.recovery_xy<=0 || cfg.recovery_xy>=cfg.inflation_xy ||
       !finite(a) || !finite(b) || a.z<cfg.z_min || a.z>cfg.z_max ||
       b.z<cfg.z_min || b.z>cfg.z_max ||
       distance(a,b)>1.0 || segment(a,a,unknown))return false;
    if(!unknown && !trace(a,b,cfg.resolution,[&](Key k){return known(k);}))return false;
    const double r=cfg.resolution, radius=cfg.inflation_xy;
    Vec3 lo{std::min(a.x,b.x)-radius,std::min(a.y,b.y)-radius,std::min(a.z,b.z)-cfg.inflation_z};
    Vec3 hi{std::max(a.x,b.x)+radius,std::max(a.y,b.y)+radius,std::max(a.z,b.z)+cfg.inflation_z};
    bool improves=false;
    const bool safe=nearby(lo,hi,[&](Key k){
      if(hits(a,b,k,cfg.recovery_xy))return false;
      if(!hits(a,b,k,radius))return true;
      if(!hits(a,a,k,radius))return false;
      Vec3 nearest{std::clamp(a.x,k.x*r,(k.x+1)*r),std::clamp(a.y,k.y*r,(k.y+1)*r),a.z};
      Vec3 normal=a-nearest, motion=b-a;
      if(dot(normal,motion)<-1e-10)return false;
      double end_dist=std::hypot(b.x-std::clamp(b.x,k.x*r,(k.x+1)*r),
                                 b.y-std::clamp(b.y,k.y*r,(k.y+1)*r));
      improves=improves || end_dist>norm(normal)+1e-7;
      return true;
    });
    return safe && improves;
  }
  bool recoveryClear(Vec3 p,bool unknown=true)const {
    if(cfg.recovery_xy<=0)return segment(p,p,unknown);
    if(!finite(p)||p.z<cfg.z_min||p.z>cfg.z_max||(!unknown&&!known(toKey(p))))return false;
    double r=std::min(cfg.inflation_xy,cfg.recovery_xy);
    return nearby(p-Vec3{r,r,cfg.inflation_z},p+Vec3{r,r,cfg.inflation_z},
                  [&](Key k){return !hits(p,p,k,r);});
  }
  // Full-body observation certificate for limited side/back motion.  Unlike
  // segment(..., false), this requires free evidence throughout the swept
  // cylinder, not just on its centerline. Radar free evidence alone still
  // cannot establish the absence of transparent glass.
  bool observedSegment(Vec3 a,Vec3 b,size_t max_cells=20000)const {
    if(!finite(a)||!finite(b)||max_cells==0||!std::isfinite(cfg.resolution)||
       cfg.resolution<=0||!std::isfinite(cfg.inflation_xy)||cfg.inflation_xy<0||
       !std::isfinite(cfg.inflation_z)||cfg.inflation_z<0)return false;
    // Include cells touching the lower AABB face too. hits() regards tangency
    // as contact, including its 1e-9 numerical tolerance.
    constexpr double epsilon=1e-9;
    Vec3 lo{std::min(a.x,b.x)-cfg.inflation_xy-epsilon,
            std::min(a.y,b.y)-cfg.inflation_xy-epsilon,
            std::min(a.z,b.z)-cfg.inflation_z-epsilon};
    Vec3 hi{std::max(a.x,b.x)+cfg.inflation_xy+epsilon,
            std::max(a.y,b.y)+cfg.inflation_xy+epsilon,
            std::max(a.z,b.z)+cfg.inflation_z+epsilon};
    const double lower[3]={lo.x,lo.y,lo.z},upper[3]={hi.x,hi.y,hi.z};
    int begin[3],end[3];
    size_t count=1;
    for(int axis=0;axis<3;++axis){
      const double l=std::floor(lower[axis]/cfg.resolution);
      const double h=std::floor(upper[axis]/cfg.resolution);
      if(!std::isfinite(l)||!std::isfinite(h)||
         l<=std::numeric_limits<int>::min()||h>=std::numeric_limits<int>::max()||h<l)
        return false;
      begin[axis]=int(l);end[axis]=int(h);
      const uint64_t width=uint64_t(int64_t(end[axis])-int64_t(begin[axis])+1);
      // Bound work before collision or voxel traversal; avoid size overflow.
      if(width>max_cells || count>max_cells/size_t(width))return false;
      count*=size_t(width);
    }
    // nearby() uses a separate bucket index; pathological large coordinates
    // must not overflow its signed keys even when voxel resolution is coarse.
    for(int axis=0;axis<3;++axis){
      const double l=(lower[axis]-.5*cfg.resolution)/bucket_size_;
      const double h=(upper[axis]+.5*cfg.resolution)/bucket_size_;
      if(!std::isfinite(l)||!std::isfinite(h)||l<=std::numeric_limits<int>::min()+1.0||
         h>=std::numeric_limits<int>::max()-1.0)return false;
    }
    if(!segmentFrom(a,b,true))return false;
    // During a certified margin-only escape, requiring the *normal safety
    // margin* itself to be free is contradictory: its existing overlap is why
    // recovery is needed. Still require observation of the complete recovery
    // body+reserve, after segmentFrom proved monotonic separation from every
    // initial obstacle and the full normal envelope for all new obstacles.
    const double observed_radius=segment(a,a,true)?cfg.inflation_xy:
      std::min(cfg.inflation_xy,cfg.recovery_xy);
    for(int64_t x=begin[0];x<=end[0];++x)
      for(int64_t y=begin[1];y<=end[1];++y)
        for(int64_t z=begin[2];z<=end[2];++z){
          Key k{int(x),int(y),int(z)};
          if(hits(a,b,k,observed_radius) && (!known(k)||occupied.count(k)))return false;
        }
    return true;
  }
  // Certifies an entire axis-aligned hull, used for recursive Bezier subdivision.
  // It is conservative and never falls back to accepting a few collision-free samples.
  bool boxClear(Vec3 lo, Vec3 hi, bool unknown = true) const {
    if (lo.z < cfg.z_min || hi.z > cfg.z_max) return false;
    const double r=cfg.resolution;
    Vec3 lower{lo.x-cfg.inflation_xy,lo.y-cfg.inflation_xy,lo.z-cfg.inflation_z};
    Vec3 upper{hi.x+cfg.inflation_xy,hi.y+cfg.inflation_xy,hi.z+cfg.inflation_z};
    if(!nearby(lower,upper,[&](Key k){
      double dx=std::max({0., k.x*r-hi.x, lo.x-(k.x+1)*r});
      double dy=std::max({0., k.y*r-hi.y, lo.y-(k.y+1)*r});
      return !(std::hypot(dx,dy)<=cfg.inflation_xy+1e-9 &&
               k.z*r<=upper.z && (k.z+1)*r>=lower.z);
    }))return false;
    if (!unknown) {
      Key a=toKey(lo), b=toKey(hi);
      for(int x=a.x;x<=b.x;++x) for(int y=a.y;y<=b.y;++y) for(int z=a.z;z<=b.z;++z)
        if (!known({x,y,z})) return false;
    }
    return true;
  }
private:
  uint64_t evidence_revision_{0};
  bool initialized_{false};
  double bucket_size_{1};
  std::unordered_map<Key,std::vector<Key>,KeyHash>buckets_;
  mutable std::unordered_map<Key,double,KeyHash> clearance_;
  template<class F>bool nearby(Vec3 lo,Vec3 hi,F visit)const{
    // Buckets index voxel centers; include half a voxel on every query side.
    double h=cfg.resolution*.5+1e-8;
    Key a=key(lo-Vec3{h,h,h},bucket_size_),b=key(hi+Vec3{h,h,h},bucket_size_);
    for(int x=a.x;x<=b.x;++x)for(int y=a.y;y<=b.y;++y)for(int z=a.z;z<=b.z;++z){
      auto it=buckets_.find({x,y,z});if(it==buckets_.end())continue;
      for(Key k:it->second){
        const double r=cfg.resolution;
        if(k.x*r>hi.x || (k.x+1)*r<lo.x || k.y*r>hi.y || (k.y+1)*r<lo.y ||
           k.z*r>hi.z || (k.z+1)*r<lo.z)continue;
        if(!visit(k))return false;
      }
    }
    return true;
  }
  static bool clip(double p,double d,double lo,double hi,double &t0,double &t1) {
    if(std::abs(d)<1e-12) return p>=lo && p<=hi;
    double a=(lo-p)/d,b=(hi-p)/d;
    if(a>b)std::swap(a,b);
    t0=std::max(t0,a); t1=std::min(t1,b);
    return t0<=t1;
  }
  bool hits(Vec3 a,Vec3 b,Key k,double radius) const {
    const double r=cfg.resolution;
    double t0=0,t1=1;
    Vec3 d=b-a;
    if(!clip(a.z,d.z,k.z*r-cfg.inflation_z,(k.z+1)*r+cfg.inflation_z,t0,t1))return false;
    Vec3 u=a+d*t0,v=a+d*t1;
    double s0=0,s1=1;
    if(clip(u.x,v.x-u.x,k.x*r,(k.x+1)*r,s0,s1) &&
       clip(u.y,v.y-u.y,k.y*r,(k.y+1)*r,s0,s1))return true;
    auto pointBox=[&](Vec3 p){return std::hypot(std::max({0.,k.x*r-p.x,p.x-(k.x+1)*r}),
                                              std::max({0.,k.y*r-p.y,p.y-(k.y+1)*r}));};
    double best=std::min(pointBox(u),pointBox(v));
    Vec3 e{v.x-u.x,v.y-u.y,0}; const double l2=dot(e,e);
    for(int x=0;x<2;++x)for(int y=0;y<2;++y){
      Vec3 p{(k.x+x)*r,(k.y+y)*r,u.z};
      double s=l2<1e-18?0:std::clamp(((p.x-u.x)*e.x+(p.y-u.y)*e.y)/l2,0.,1.);
      best=std::min(best,std::hypot(p.x-u.x-e.x*s,p.y-u.y-e.y*s));
    }
    return best<=radius+1e-9;
  }
};
struct Evidence {
  int score{0};
};
class EvidenceMap {
public:
  explicit EvidenceMap(double resolution = .25) : r(resolution) {}
  double r;
  double z_min{-.3}, z_max{4.5};
  int hit_score{3}, miss_score{1}, occupied_threshold{2};
  int min_score{-4}, max_score{6};
  std::unordered_map<Key, Evidence, KeyHash> cells;
  bool validEvidenceConfig() const {
    return hit_score > 0 && miss_score > 0 && min_score <= 0 &&
           occupied_threshold > 0 && max_score >= occupied_threshold;
  }
  void integrate(Vec3 origin, const std::vector<Vec3> &endpoints, double min_range,
                 double max_range) {
    if (!validEvidenceConfig())
      throw std::invalid_argument("Invalid evidence score limits/thresholds");
    KeySet hits, misses;
    for (Vec3 endpoint : endpoints) {
      if (!finite(endpoint))
        continue;
      double d = distance(origin, endpoint);
      if (d < min_range || d > max_range)
        continue;
      Key last = key(endpoint, r);
      if (endpoint.z >= z_min && endpoint.z <= z_max)
        hits.insert(last);
      trace(
          origin, endpoint, r,
          [&](Key k) {
            if (k == last)
              return true;
            double z = center(k, r).z;
            if (z >= z_min && z <= z_max)
              misses.insert(k);
            return true;
          },
          false);
    }
    // One vote per cell per scan; a simultaneous hit always wins.
    for (auto k : misses)
      if (!hits.count(k)) {
        auto &e = cells[k];
        e.score = int(std::clamp<int64_t>(int64_t(e.score) - miss_score, min_score, max_score));
      }
    for (auto k : hits) {
      auto &e = cells[k];
      // Keep negative free-space evidence: a single noisy return must not erase
      // repeated clear observations and instantly become a persistent obstacle.
      // Positive scores below occupied_threshold are unconfirmed/unknown.
      e.score = int(std::clamp<int64_t>(int64_t(e.score) + hit_score, min_score, max_score));
    }
  }
  void exportSets(KeySet &occupied, KeySet &free) const {
    occupied.clear();
    free.clear();
    for (const auto &kv : cells) {
      if (kv.second.score >= occupied_threshold)
        occupied.insert(kv.first);
      else if (kv.second.score <= 0)
        free.insert(kv.first);
    }
  }
};
} // namespace fire_scout
