#pragma once
#include "geometry.hpp"
#include <deque>
namespace fire_scout {
struct PoseSample {
  double time{0}; Vec3 position{}; double w{1},x{0},y{0},z{0};
  Vec3 rotate(Vec3 p)const {
    return {(1-2*(y*y+z*z))*p.x+2*(x*y-w*z)*p.y+2*(x*z+w*y)*p.z,
            2*(x*y+w*z)*p.x+(1-2*(x*x+z*z))*p.y+2*(y*z-w*x)*p.z,
            2*(x*z-w*y)*p.x+2*(y*z+w*x)*p.y+(1-2*(x*x+y*y))*p.z};
  }
  double yaw()const{return std::atan2(2*(w*z+x*y),1-2*(y*y+z*z));}
  double tilt()const{return std::acos(std::clamp(1-2*(x*x+y*y),-1.,1.));}
  bool normalize(){
    double n=std::sqrt(w*w+x*x+y*y+z*z);
    if(!std::isfinite(time)||!finite(position)||!std::isfinite(n)||n<1e-6)return false;
    w/=n;x/=n;y/=n;z/=n;return true;
  }
};
class PoseHistory {
public:
  void add(PoseSample p){
    if(!p.normalize())return;
    if(!samples.empty() && p.time<samples.back().time-1) samples.clear();
    if(!samples.empty() && p.time<=samples.back().time)return;
    samples.push_back(p);
    while(samples.size()>500||(!samples.empty()&&p.time-samples.front().time>3))samples.pop_front();
  }
  bool at(double t,PoseSample&out,double tolerance=.03)const{
    if(samples.empty()||t<samples.front().time-tolerance||t>samples.back().time+tolerance)return false;
    if(t<=samples.front().time){out=samples.front();return true;}
    for(size_t i=1;i<samples.size();++i)if(t<=samples[i].time){
      const auto&a=samples[i-1];auto b=samples[i];
      if(b.time-a.time>.25)return false;
      double u=(t-a.time)/(b.time-a.time);
      if(a.w*b.w+a.x*b.x+a.y*b.y+a.z*b.z<0){b.w=-b.w;b.x=-b.x;b.y=-b.y;b.z=-b.z;}
      out={t,a.position*(1-u)+b.position*u,a.w*(1-u)+b.w*u,a.x*(1-u)+b.x*u,
           a.y*(1-u)+b.y*u,a.z*(1-u)+b.z*u};
      return out.normalize();
    }
    out=samples.back();return true;
  }
private:
  std::deque<PoseSample>samples;
};
} // namespace fire_scout
