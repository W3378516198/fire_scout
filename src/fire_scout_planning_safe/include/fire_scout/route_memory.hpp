#pragma once
#include "route_repair.hpp"
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
    // The controller follows the original curve, not the shortcut used only
    // to validate cross-track rejoining. Certify every future curve segment.
    Vec3 previous=atArc(path_,arc_,progress_);
    for(size_t i=1;i<path_.size();++i)if(arc_[i]>progress_+1e-8){
      if(!g.segmentFrom(previous,path_[i],unknown))return false;
      previous=path_[i];
    }
    return true;
  }
  // Return only the checked future part; the follower's terminal speed profile
  // brakes before the first invalid segment. An invalid tail is never reused.
  std::vector<Vec3> safePrefix(const Grid&g,Vec3 current,bool unknown=true,double terminal_margin=0){
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
    // Keep the original curve. The current-to-route connector is a collision
    // check, never an extra polyline edge: tiny lateral odometry errors would
    // otherwise create centimetre-scale corners and near-zero yaw speed caps.
    double end=join_s;
    Vec3 previous=join;
    for(size_t i=1;i<path_.size();++i)if(arc_[i]>join_s+1e-8){
      if(!g.segmentFrom(previous,path_[i],unknown)){
        Vec3 d=path_[i]-previous;double lo=0,hi=1;
        for(int n=0;n<14;++n){double m=(lo+hi)*.5;if(g.segmentFrom(previous,previous+d*m,unknown))lo=m;else hi=m;}
        end+=std::max(0.,lo*norm(d)-.03);
        // Stop at a comfortable point before this blocked tail. This only
        // shortens a temporary prefix; it never inflates an entire narrow
        // corridor or changes the hard body envelope.
        if(terminal_margin>0){
          const double earliest=std::max(progress_,end-2.0);
          while(end>earliest&&!g.segmentWithMargin(atArc(path_,arc_,end),
                  atArc(path_,arc_,end),terminal_margin,unknown))end=std::max(earliest,end-.05);
          if(!g.segmentWithMargin(atArc(path_,arc_,end),atArc(path_,arc_,end),
                                  terminal_margin,unknown))return {};
        }
        break;
      }
      end=arc_[i];previous=path_[i];
    }
    // A short exact backward overlap gives the follower a bounded, unambiguous
    // projection even when the original route began many metres ago.
    double begin=std::max(0.,progress_-.25);
    const auto initial=routeSlice(path_,progress_,join_s);
    for(size_t i=1;i<initial.size();++i)
      if(!g.segmentFrom(initial[i-1],initial[i],unknown)){begin=join_s;break;}
    if(end-progress_<=.35)return {};
    auto out=routeSlice(path_,begin,end);
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
