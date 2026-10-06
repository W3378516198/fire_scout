#pragma once
#include "geometry.hpp"

namespace fire_scout {
// Manually supplied risk regions must already use the planning/odometry frame.
// They are a policy input, not inferred glass classifications or occupancy.
struct RiskBox { Vec3 lo,hi; };
inline bool validRiskBox(const RiskBox&box){
  return finite(box.lo)&&finite(box.hi)&&box.lo.x<=box.hi.x&&
         box.lo.y<=box.hi.y&&box.lo.z<=box.hi.z;
}
// Flat ROS double-array layout: xmin,ymin,zmin,xmax,ymax,zmax, repeated.
// Zero-thickness boxes intentionally support known window planes.
inline std::vector<RiskBox> parseRiskBoxes(const std::vector<double>&values){
  if(values.size()%6)throw std::invalid_argument("glass_risk_boxes must contain 6 values per box");
  std::vector<RiskBox> result;
  result.reserve(values.size()/6);
  for(size_t i=0;i<values.size();i+=6){
    RiskBox box{{values[i],values[i+1],values[i+2]},
                {values[i+3],values[i+4],values[i+5]}};
    if(!validRiskBox(box))throw std::invalid_argument("Invalid glass risk box bounds");
    result.push_back(box);
  }
  return result;
}
// Exact line-segment / padded AABB intersection. Padding is an axis-wise
// conservative distance, so corner proximity may activate policy early.
inline bool segmentNearBox(Vec3 a,Vec3 b,const RiskBox&box,double padding=0){
  if(!finite(a)||!finite(b)||!validRiskBox(box)||!std::isfinite(padding)||padding<0)
    throw std::invalid_argument("Invalid glass risk segment/padding");
  Vec3 lo=box.lo-Vec3{padding,padding,padding};
  Vec3 hi=box.hi+Vec3{padding,padding,padding};
  if(!finite(lo)||!finite(hi))throw std::invalid_argument("Overflow in glass risk padding");
  const double start[3]={a.x,a.y,a.z},finish[3]={b.x,b.y,b.z};
  const double lower[3]={lo.x,lo.y,lo.z},upper[3]={hi.x,hi.y,hi.z};
  long double t0=0,t1=1;
  for(int axis=0;axis<3;++axis){
    const long double d=(long double)finish[axis]-start[axis];
    if(d==0){
      if(start[axis]<lower[axis]||start[axis]>upper[axis])return false;
      continue;
    }
    long double l=((long double)lower[axis]-start[axis])/d;
    long double h=((long double)upper[axis]-start[axis])/d;
    if(l>h)std::swap(l,h);
    t0=std::max(t0,l);t1=std::min(t1,h);
    if(t0>t1)return false;
  }
  return true;
}
inline bool segmentNearAnyRiskBox(Vec3 a,Vec3 b,const std::vector<RiskBox>&boxes,double padding=0){
  if(!finite(a)||!finite(b)||!std::isfinite(padding)||padding<0)
    throw std::invalid_argument("Invalid glass risk segment/padding");
  for(const auto&box:boxes)if(segmentNearBox(a,b,box,padding))return true;
  return false;
}

// A too-close / negative-infinite echo has no localizable endpoint. Remember
// where it was observed and the old world-frame acoustic direction instead of
// inventing an occupied point inside the aircraft. These observations are
// intentionally short-lived: one malformed echo must not veto motion forever.
class UnlocalizedSonarRisk {
 public:
  void clear(){samples_.clear();}
  bool add(Vec3 origin_world,double yaw,double half_angle,double stamp=0){
    if(!finite(origin_world)||!std::isfinite(yaw)||!std::isfinite(half_angle)||
       !std::isfinite(stamp)||half_angle<=0||half_angle>=pi/2)
      throw std::invalid_argument("Invalid unlocalized sonar observation");
    yaw=wrap(yaw);
    for(auto&sample:samples_){
      const double shift=distance(origin_world,sample.origin);
      const double turn=std::abs(wrap(yaw-sample.yaw));
      if(shift<=.05 && turn<=.05){
        // Keep a fixed cluster anchor to prevent successive observations from
        // walking the remembered obstacle away. Expand to cover merged origins
        // and cones conservatively; no observed restriction is weakened.
        sample.position_slack=std::max(sample.position_slack,shift);
        sample.half_angle=std::max(sample.half_angle,half_angle+turn);
        sample.stamp=std::max(sample.stamp,stamp);
        return false;
      }
    }
    samples_.push_back({origin_world,yaw,half_angle,0,stamp});return true;
  }
  bool near(Vec3 a,Vec3 b,double radius,double now=0,double ttl=0)const{
    validate(a,b,radius);
    for(const auto&sample:samples_)
      if(fresh(sample,now,ttl)&&
         separation(sample.origin,a,b)<=radius+sample.position_slack)return true;
    return false;
  }
  // Restrict every segment crossing the local region, including one whose two
  // endpoints are outside. This is a horizontal closing-direction constraint;
  // the caller must still certify the full 3-D collision/retreat corridor.
  bool allows(Vec3 a,Vec3 b,double radius,double now=0,double ttl=0)const{
    validate(a,b,radius);
    const long double dx=(long double)b.x-a.x,dy=(long double)b.y-a.y;
    const long double speed=std::hypot(dx,dy);
    if(speed<=1e-12)return true;
    const double course=double(std::atan2(dy,dx));
    for(const auto&sample:samples_){
      if(!fresh(sample,now,ttl))continue;
      if(separation(sample.origin,a,b)>radius+sample.position_slack)continue;
      const double angle=std::max(0.,std::abs(wrap(course-sample.yaw))-sample.half_angle);
      if(speed*std::cos(angle)>1e-9)return false;
    }
    return true;
  }
  void prune(double now,double ttl){
    if(!std::isfinite(now)||!std::isfinite(ttl)||ttl<=0)return;
    samples_.erase(std::remove_if(samples_.begin(),samples_.end(),
      [now,ttl](const Sample&s){return now>=s.stamp&&now-s.stamp>ttl;}),samples_.end());
  }
  size_t size(double now=0,double ttl=0)const{
    size_t count=0;for(const auto&s:samples_)count+=fresh(s,now,ttl);return count;
  }
 private:
  struct Sample { Vec3 origin;double yaw,half_angle,position_slack,stamp; };
  std::vector<Sample>samples_;
  static bool fresh(const Sample&s,double now,double ttl){
    return ttl<=0||now<s.stamp||now-s.stamp<=ttl;
  }
  static void validate(Vec3 a,Vec3 b,double radius){
    if(!finite(a)||!finite(b)||!std::isfinite(radius)||radius<0)
      throw std::invalid_argument("Invalid unlocalized sonar risk segment/radius");
  }
  static double separation(Vec3 p,Vec3 a,Vec3 b){
    const long double d[3]={(long double)b.x-a.x,(long double)b.y-a.y,(long double)b.z-a.z};
    const long double q[3]={(long double)p.x-a.x,(long double)p.y-a.y,(long double)p.z-a.z};
    const long double len2=d[0]*d[0]+d[1]*d[1]+d[2]*d[2];
    const long double t=len2==0?0:std::clamp((q[0]*d[0]+q[1]*d[1]+q[2]*d[2])/len2,0.L,1.L);
    return double(std::sqrt((q[0]-t*d[0])*(q[0]-t*d[0])+
                            (q[1]-t*d[1])*(q[1]-t*d[1])+
                            (q[2]-t*d[2])*(q[2]-t*d[2])));
  }
};
} // namespace fire_scout
