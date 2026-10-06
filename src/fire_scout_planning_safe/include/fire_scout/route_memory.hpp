#pragma once
#include "grid.hpp"
namespace fire_scout {
// Own the published geometry. Map validation never rebuilds a retained spline.
class RouteMemory {
public:
  const std::vector<Vec3>& points()const{return path_;}
  double progress()const{return progress_;}
  double remaining()const{return arc_.empty()?0.:std::max(0.,arc_.back()-progress_);}
  bool updateProgress(Vec3 current){return advance(current);}
  void clear(){path_.clear();arc_.clear();progress_=0;}
  void set(std::vector<Vec3> p){path_=std::move(p);arc_=arcLengths(path_);progress_=0;}
  bool valid(const Grid&g,Vec3 current,bool unknown=true){
    if(!advance(current))return false;
    Vec3 join=atArc(path_,arc_,std::min(arc_.back(),progress_+.25));
    double join_s=std::min(arc_.back(),progress_+.25);
    if(!g.segmentFrom(current,join,unknown)){
      join_s=progress_;join=atArc(path_,arc_,join_s);
      if(!g.segmentFrom(current,join,unknown))return false;
    }
    for(size_t i=1;i<path_.size();++i)if(arc_[i]>join_s+1e-8){
      if(!g.segmentFrom(join,path_[i],unknown))return false;
      join=path_[i];
    }
    return true;
  }
  // Return only the checked future part; the follower's terminal speed profile
  // brakes before the first invalid segment. An invalid tail is never reused.
  std::vector<Vec3> safePrefix(const Grid&g,Vec3 current,bool unknown=true){
    if(!advance(current))return {};
    // Use the same forward connector as valid(). The perpendicular
    // projection can point back into the wall when a vehicle is displaced
    // outward during a margin escape. Rejecting that connector discarded a
    // safe forward suffix whenever a distant part of the route changed.
    double join_s=std::min(arc_.back(),progress_+.25);
    Vec3 join=atArc(path_,arc_,join_s);
    if(!g.segmentFrom(current,join,unknown)) {
      join_s=progress_;join=atArc(path_,arc_,join_s);
      if(!g.segmentFrom(current,join,unknown))return {};
    }
    std::vector<Vec3> out{current};
    if(distance(current,join)>.02)out.push_back(join);
    for(size_t i=1;i<path_.size();++i)if(arc_[i]>join_s+1e-8){
      if(!g.segmentFrom(out.back(),path_[i],unknown)){
        // Retain the valid part of a long sparse segment too. A failed far
        // endpoint must not discard metres of collision-checked forward path.
        Vec3 a=out.back(),d=path_[i]-a;double lo=0,hi=1;
        for(int n=0;n<14;++n){double m=(lo+hi)*.5;if(g.segmentFrom(a,a+d*m,unknown))lo=m;else hi=m;}
        lo=std::max(0.,lo-.03/std::max(.001,norm(d)));
        if(lo*norm(d)>.02)out.push_back(a+d*lo);
        break;
      }
      out.push_back(path_[i]);
    }
    return pathLength(out)>.35?out:std::vector<Vec3>{};
  }
private:
  std::vector<Vec3>path_;std::vector<double>arc_;double progress_{0};
  bool advance(Vec3 p){
    if(path_.empty())return false;
    auto pr=project(path_,arc_,p,std::max(0.,progress_-.10),std::min(arc_.back(),progress_+2.0));
    if(!std::isfinite(pr.error)||pr.error>1.2)return false;
    progress_=std::max(progress_,pr.s);return true;
  }
};
}
