#pragma once
#include "grid.hpp"
#include <chrono>

namespace fire_scout {
// EGO-inspired distinct initial routes and Air-FAR-inspired visibility checks.
// These are geometry-based route classes, not a proof of 3D homotopy and not
// imported EGO/Air-FAR source. A portal is inferred only from two occupied sides.
struct TopologyConfig {
  bool enabled{true};
  double passage_width{3.4},probe_distance{2.5},merge_distance{1.2};
  double gate_weight{45.0},cut_thickness{.6},commit_distance{2.0};
  double max_detour_ratio{2.0},max_extra_length{24.0},refresh_period{1.5};
  double search_resolution{.30};
  bool prefer_distinct{true};
  int max_gates{8};
  bool valid()const{
    for(double x:{passage_width,probe_distance,merge_distance,gate_weight,
      cut_thickness,commit_distance,max_detour_ratio,max_extra_length,refresh_period,search_resolution})
      if(!std::isfinite(x)||x<=0)return false;
    return passage_width>=1&&passage_width<=8&&probe_distance>=passage_width*.5&&
      probe_distance<=5&&merge_distance<=2.5&&cut_thickness>=.2&&cut_thickness<=2&&
      commit_distance>=.5&&max_detour_ratio>=1&&max_detour_ratio<=4&&
      max_extra_length<=50&&refresh_period>=.3&&max_gates>=1&&max_gates<=16&&
      search_resolution>=.10&&search_resolution<=.60;
  }
};
struct RoutePassage {Vec3 center{},normal{};double width{0};};
inline bool samePassage(const RoutePassage&a,const RoutePassage&b,const TopologyConfig&c){
  return distance(a.center,b.center)<c.merge_distance &&
    std::abs(dot(a.normal,b.normal))>.5;
}
inline std::vector<RoutePassage> routePassages(const Grid&g,const std::vector<Vec3>&p,
    const TopologyConfig&c,const std::function<bool()>&stop={}){
  std::vector<RoutePassage> result;
  if(!c.enabled||p.size()<2)return result;
  const auto arc=arcLengths(p);const double step=std::max(.35,g.cfg.resolution*2);
  bool run=false;RoutePassage first,last,best;
  auto finish=[&]{if(!run)return;
    // Average the constant-width plateau so the anchor sits at the wall plane,
    // not at the first expanded-body contact before the entrance.
    best.center=(first.center+last.center)*.5;
    bool exists=false;for(const auto&q:result)if(samePassage(q,best,c))exists=true;
    if(!exists&&result.size()<size_t(c.max_gates))result.push_back(best);
    run=false;};
  for(double s=.8;s<arc.back()-.8;s+=step){
    if(stop&&stop())break;
    const Vec3 here=atArc(p,arc,s);
    Vec3 tangent=atArc(p,arc,std::min(arc.back(),s+.3))-
      atArc(p,arc,std::max(0.,s-.3));
    const double n=std::hypot(tangent.x,tangent.y);
    if(n<.01||!g.segment(here,here,true)){finish();continue;}
    tangent={tangent.x/n,tangent.y/n,0};
    RoutePassage gate;bool found=false;
    // Oblique approaches must detect the SAME wall-normal entrance. A probe
    // perpendicular only to path tangent can miss one jamb altogether.
    for(Vec3 normal:{Vec3{1,0,0},Vec3{0,1,0},tangent}){
      if(std::abs(dot(normal,tangent))<.35)continue;
      Vec3 side{-normal.y,normal.x,0};
      auto room=[&](double sign){
        if(g.segment(here,here+side*(sign*c.probe_distance),true))return c.probe_distance;
        double lo=0,hi=c.probe_distance;
        for(int k=0;k<7;++k){double m=(lo+hi)*.5;
          if(g.segment(here,here+side*(m*sign),true))lo=m;else hi=m;}
        return lo;
      };
      const double left=room(1),right=room(-1);
      const double width=2*g.cfg.inflation_xy+left+right;
      if(left>=c.probe_distance-.04||right>=c.probe_distance-.04||width>c.passage_width)continue;
      gate={here+side*((left-right)*.5),normal,width};found=true;break;
    }
    if(!found){finish();continue;}
    if(run&&(distance(gate.center,last.center)>2*step||std::abs(dot(gate.normal,last.normal))<.5)){finish();}
    if(!run){first=last=best=gate;run=true;}
    else{last=gate;if(gate.width<best.width)best=gate;}
  }
  finish();return result;
}
inline std::vector<RoutePassage> peerPassages(const Grid&g,
    const std::vector<std::vector<Vec3>>&peers,const TopologyConfig&c,
    const std::function<bool()>&stop={}){
  std::vector<RoutePassage> out;
  for(const auto&p:peers){
    for(const auto&gate:routePassages(g,p,c,stop)){
      bool found=false;for(const auto&q:out)if(samePassage(gate,q,c))found=true;
      if(!found&&out.size()<size_t(c.max_gates))out.push_back(gate);
    }
    if(stop&&stop())break;
  }
  return out;
}
inline bool routeCrossesPassage(const std::vector<Vec3>&p,const RoutePassage&gate,
    double reserve=.2){
  if(p.size()<2)return false;
  const Vec3 side{-gate.normal.y,gate.normal.x,0};
  for(size_t i=1;i<p.size();++i){
    const double a=dot(p[i-1]-gate.center,gate.normal),b=dot(p[i]-gate.center,gate.normal);
    if((a>0&&b>0)||(a<0&&b<0)||std::abs(a-b)<1e-9)continue;
    const Vec3 at=p[i-1]+(p[i]-p[i-1])*std::clamp(a/(a-b),0.,1.);
    // Same XY entrance counts as shared even when its chosen altitude differs.
    if(std::abs(dot(at-gate.center,side))<=gate.width*.5+reserve)return true;
  }
  return false;
}
inline size_t sharedPassageCount(const std::vector<Vec3>&p,
    const std::vector<RoutePassage>&gates){
  size_t n=0;for(const auto&q:gates)if(routeCrossesPassage(p,q))++n;return n;
}
inline std::vector<RoutePassage> reservablePassages(const std::vector<RoutePassage>&gates,
    Vec3 start,Vec3 goal,const TopologyConfig&c){
  std::vector<RoutePassage> out;
  for(const auto&gate:gates){
    Vec3 a=start-gate.center,b=goal-gate.center;a.z=b.z=0;
    // Do not reverse an aircraft already committed inside a passage or invent
    // exclusivity at an unavoidable shared final approach.
    if(norm(a)>c.commit_distance&&norm(b)>c.commit_distance)out.push_back(gate);
  }
  return out;
}
inline bool addPassageCuts(Grid&search,const Grid&physical,
    const std::vector<RoutePassage>&gates,const TopologyConfig&c,
    const std::function<bool()>&stop={}){
  KeySet occupied=physical.occupied;const double r=physical.cfg.resolution;
  size_t inserted=0;
  for(const auto&gate:gates){
    const Vec3 side{-gate.normal.y,gate.normal.x,0};
    const double half=gate.width*.5+physical.cfg.inflation_xy+r;
    const double extent=half+c.cut_thickness;
    Key lo=physical.toKey(gate.center-Vec3{extent,extent,0});
    Key hi=physical.toKey(gate.center+Vec3{extent,extent,0});
    const int zlo=int(std::floor(physical.cfg.z_min/r))-1;
    const int zhi=int(std::ceil(physical.cfg.z_max/r))+1;
    for(int x=lo.x;x<=hi.x;++x)for(int y=lo.y;y<=hi.y;++y){
      const Vec3 q=physical.point({x,y,0})-gate.center;
      if(std::abs(dot(q,gate.normal))>c.cut_thickness*.5+r||std::abs(dot(q,side))>half)continue;
      for(int z=zlo;z<=zhi;++z){
        if(++inserted>80000||(stop&&stop()))return false;
        occupied.insert({x,y,z});
      }
    }
  }
  // Virtual cuts belong only to this private search copy. They are never
  // published as occupancy, and final geometry is certified on physical.
  search.update(std::move(occupied),physical.free);
  return true;
}
inline bool obstacleSeparatedRoutes(const Grid&g,const std::vector<Vec3>&a,
    const std::vector<Vec3>&b){
  if(a.size()<2||b.size()<2)return false;
  const auto aa=arcLengths(a),bb=arcLengths(b);int obstructed=0;
  for(int i=1;i<20;++i){double u=i/20.;
    const Vec3 x=atArc(a,aa,aa.back()*u),y=atArc(b,bb,bb.back()*u);
    if(distance(x,y)>.5&&!g.segment(x,y,true)&&++obstructed>=2)return true;
  }
  return false;
}
// A conservative, private guide grid accelerates long XY detours. Every fine
// occupied voxel is fully covered by coarse occupied voxels. Known-only free
// cells require all constituent fine cells; final routes still use the fine
// physical map, and guide/cut occupancy is never broadcast as sensor evidence.
inline Grid topologyGuideGrid(const Grid&g,double requested,
    const std::function<bool()>&stop={}){
  const int scale=std::max(1,int(std::ceil(requested/g.cfg.resolution-1e-8)));
  if(scale==1)return g;
  GridConfig cfg=g.cfg;cfg.resolution=g.cfg.resolution*scale;
  Grid guide(cfg);KeySet occupied,free;std::unordered_map<Key,int,KeyHash> counts;
  for(const auto&k:g.occupied){
    occupied.insert(guide.toKey(g.point(k)));
    if(stop&&stop())return g;
  }
  for(const auto&k:g.free){
    ++counts[guide.toKey(g.point(k))];
    if(stop&&stop())return g;
  }
  for(const auto&entry:counts)if(entry.second==scale*scale*scale)free.insert(entry.first);
  guide.update(std::move(occupied),std::move(free));
  return guide;
}

// Two checked waypoints can pass opposite sides of a long obstacle/house;
// one midpoint cannot describe an outside-corner + entrance approach.
inline std::vector<std::vector<Vec3>> obstacleSideCorridors(const Grid&g,Vec3 start,
    Vec3 goal,const TopologyConfig&c,bool unknown,const std::function<bool()>&stop={}){
  std::vector<std::vector<Vec3>> out;Vec3 chord=goal-start;
  const double length=std::hypot(chord.x,chord.y);
  if(length<1||g.segment(start,goal,unknown))return out;
  const Vec3 forward{chord.x/length,chord.y/length,0},side{-forward.y,forward.x,0};
  double first=-1,last=-1;
  for(double s=.4;s<length;s+=.4){
    if(stop&&stop())return out;
    const Vec3 p=start+chord*(s/length);
    if(!g.segment(p,p,unknown)){if(first<0)first=s;last=s;}
  }
  if(first<0)return out;
  const double reserve=std::max(.8,g.cfg.inflation_xy+2*g.cfg.resolution);
  const Vec3 a=start+chord*(std::max(0.,first-reserve)/length);
  const Vec3 b=start+chord*(std::min(length,last+reserve)/length);
  const double maximum=std::min(20.,std::max(3.,c.max_extra_length*.5));
  for(double sign:{-1.,1.})for(double d=1.5;d<=maximum;d+=1.5){
    if(stop&&stop())return out;
    const Vec3 p=a+side*(sign*d),q=b+side*(sign*d);
    if(g.segment(start,p,unknown)&&g.segment(p,q,unknown)&&g.segment(q,goal,unknown)){
      out.push_back({start,p,q,goal});break;
    }
  }
  return out;
}
inline std::vector<std::vector<Vec3>> obstacleSideSeeds(const Grid&g,Vec3 start,Vec3 goal,
    bool unknown,const std::function<bool()>&stop={}){
  std::vector<std::vector<Vec3>> out;
  Vec3 chord=goal-start;const double n=std::hypot(chord.x,chord.y);
  if(n<1||g.segment(start,goal,unknown))return out;
  Vec3 normal{chord.x/n,chord.y/n,0},side{-normal.y,normal.x,0};
  double first=-1,last=-1;
  for(double s=.25;s<n;s+=.25){
    if(stop&&stop())return out;
    Vec3 p=start+chord*(s/n);
    if(!g.segment(p,p,unknown)){if(first<0)first=s;last=s;}
    else if(first>=0)break;
  }
  if(first<0)return out;
  const Vec3 center=start+chord*((first+last)*.5/n);
  for(double sign:{-1.,1.})for(double d:{1.5,3.,5.,8.}){
    if(stop&&stop())return out;
    Vec3 anchor=center+side*(sign*d);
    if(g.segment(start,anchor,unknown)&&g.segment(anchor,goal,unknown)){
      out.push_back({start,anchor,goal});break;
    }
  }
  return out;
}
inline std::vector<Vec3> visibilityShortcut(const Grid&g,const std::vector<Vec3>&p,
    bool unknown,const std::function<bool()>&stop={}){
  if(p.size()<3)return p;
  std::vector<Vec3> out{p.front()};size_t i=0;
  while(i+1<p.size()){
    if(stop&&stop()){out.insert(out.end(),p.begin()+i+1,p.end());break;}
    size_t next=i+1;
    for(size_t j=p.size()-1;j>i+1;--j){
      if(stop&&stop())break;
      if(g.segment(p[i],p[j],unknown)){next=j;break;}
    }
    out.push_back(p[next]);i=next;
  }
  return out;
}
} // namespace fire_scout
