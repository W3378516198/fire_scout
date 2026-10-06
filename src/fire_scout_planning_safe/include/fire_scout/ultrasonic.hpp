#pragma once
#include "grid.hpp"
#include <string>
namespace fire_scout {
struct SonarReading {
  bool valid{false}, has_hit{false}, too_close{false};
  double distance{std::numeric_limits<double>::infinity()};
  std::vector<Vec3> endpoints; // sensor FLU frame, before timestamped TF
  // Every usable finite/+Inf ray proves free space only BEFORE its measured
  // endpoint. Consumers keep an endpoint guard, so a real hit is never erased
  // by the same observation. This lets repeated clear scans remove old ghosts
  // instead of waiting for the TTL alone.
  std::vector<Vec3> free_endpoints;
};
inline SonarReading parseSonar(const std::vector<float>&ranges,double angle_min,double increment,
                              double range_min,double range_max,double half_angle,
                              bool inf_is_clear,double sensor_yaw=0){
  SonarReading out;
  if(ranges.empty()||!std::isfinite(angle_min)||!std::isfinite(increment)||
     !std::isfinite(range_min)||!std::isfinite(range_max)||range_max<=range_min||range_min<0||
     (ranges.size()>1 && increment==0))return out;
  size_t selected=0,usable=0;
  for(size_t i=0;i<ranges.size();++i){
    const double angle=angle_min+i*increment,r=ranges[i];
    if(std::abs(wrap(angle+sensor_yaw))>half_angle+1e-8)continue;
    ++selected;
    if((std::isfinite(r)&&r>=0&&r<range_min)||(std::isinf(r)&&r<0)){
      out.too_close=out.has_hit=true;out.distance=0;++usable;
    }else if(std::isfinite(r)&&r>=range_min&&r<=range_max){
      ++usable;out.has_hit=true;out.distance=std::min(out.distance,r);
      out.endpoints.push_back({r*std::cos(angle),r*std::sin(angle),0});
      out.free_endpoints.push_back({r*std::cos(angle),r*std::sin(angle),0});
    }else if(std::isinf(r)&&r>0&&inf_is_clear){
      ++usable;out.distance=std::min(out.distance,range_max);
      out.free_endpoints.push_back(
        {range_max*std::cos(angle),range_max*std::sin(angle),0});
    }
  }
  out.valid=out.too_close || (selected>0 && usable*10>=selected*6);
  // One finite close ray is sufficient to brake even when other rays are invalid.
  if(out.has_hit && std::isfinite(out.distance))out.valid=true;
  return out;
}
struct SonarGuardConfig {
  double stop_distance{.55}, resume_distance{.70}, brake_accel{1.2}, reaction_time{.45};
  double timeout{.45}, half_angle{10*pi/180}, max_tilt{20*pi/180};
  bool required{true};
};
inline double brakingSpeed(double distance,double reserve,double acceleration,double reaction){
  double available=std::max(0.,distance-reserve);
  return std::max(0.,std::sqrt(acceleration*acceleration*reaction*reaction+2*acceleration*available)
                     -acceleration*reaction);
}
struct SonarDecision { double speed_limit{0}, stop_required{0}; std::string reason; };
class SonarGuard {
public:
  explicit SonarGuard(SonarGuardConfig c={}) : cfg(c) {}
  SonarGuardConfig cfg;
  SonarDecision evaluate(const SonarReading&r,bool fresh,double closing_speed,double age){
    if(!fresh||!r.valid){
      return {cfg.required?0.:1e9,cfg.stop_distance,"ULTRASONIC_UNAVAILABLE"};
    }
    double reaction=cfg.reaction_time+std::max(0.,age);
    double stop=cfg.stop_distance+std::max(0.,closing_speed)*reaction+
                std::max(0.,closing_speed)*std::max(0.,closing_speed)/(2*cfg.brake_accel);
    if(r.distance<=cfg.stop_distance)latched_=true;
    if(latched_ && r.distance>=cfg.resume_distance)latched_=false;
    if(latched_)return {0,stop,"ULTRASONIC_STOP"};
    return {brakingSpeed(r.distance,cfg.stop_distance,cfg.brake_accel,reaction),stop,
            r.distance<stop?"ULTRASONIC_BRAKE":"ULTRASONIC_CLEAR"};
  }
  void reset(){latched_=false;}
private:
  bool latched_{false};
};
// Separate evidence: radar free-space rays can NEVER erase sonar obstacles.
class SonarLayer {
public:
  double resolution{.10}, half_height{.20}, ttl{0}, confirmation_window{.6};
  double clear_endpoint_guard{.12}, clock_rewind_tolerance{.25};
  int confirm_frames{2}, clear_confirm_frames{2};
  struct Cell { int count{0}; double time{0}; };
  std::unordered_map<Key,Cell,KeyHash> pending;
  std::unordered_map<Key,Cell,KeyHash> clearing;
  std::unordered_map<Key,double,KeyHash> confirmed;
  void integrate(const std::vector<Vec3>&hits,double stamp){
    if(!observeTime(stamp))return;
    KeySet once;
    for(Vec3 p:hits)if(finite(p))once.insert(key(p,resolution));
    for(Key k:once){
      auto &c=pending[k];
      if(stamp<=c.time && c.count>0)continue;
      c.count=stamp-c.time<=confirmation_window?c.count+1:1;c.time=stamp;
      if(c.count<confirm_frames)continue;
      const int nz=int(std::ceil(half_height/resolution));
      for(int z=-nz;z<=nz;++z){
        const Key expanded{k.x,k.y,k.z+z};
        confirmed[expanded]=stamp;clearing.erase(expanded);
      }
    }
    for(auto it=pending.begin();it!=pending.end();)
      if(stamp-it->second.time>confirmation_window)it=pending.erase(it);else ++it;
  }
  // Remove a remembered return only after repeated rays have observed free
  // space through that voxel. The endpoint guard preserves a current hit and
  // the vertical allowance covers the cells added by half-height extrusion.
  size_t clearObserved(Vec3 origin,const std::vector<Vec3>&free_endpoints,double stamp){
    if(!finite(origin)||!observeTime(stamp)||free_endpoints.empty()||confirmed.empty())return 0;
    const double corridor=.75*resolution;
    const double vertical=half_height+1.5*resolution;
    KeySet observed;
    for(const auto &entry:confirmed){
      const Key k=entry.first;const Vec3 p=center(k,resolution);
      if(std::abs(p.z-origin.z)>vertical)continue;
      for(Vec3 endpoint:free_endpoints){
        if(!finite(endpoint))continue;
        const double dx=endpoint.x-origin.x,dy=endpoint.y-origin.y;
        const double length=std::hypot(dx,dy);
        if(length<=clear_endpoint_guard+resolution*.25)continue;
        const double ux=dx/length,uy=dy/length;
        const double px=p.x-origin.x,py=p.y-origin.y;
        const double along=px*ux+py*uy;
        if(along<0||along>length-clear_endpoint_guard)continue;
        const double lateral=std::abs(px*uy-py*ux);
        if(lateral<=corridor){observed.insert(k);break;}
      }
    }
    std::vector<Key> erase;
    for(Key k:observed){
      auto &c=clearing[k];
      if(stamp<=c.time&&c.count>0)continue;
      c.count=stamp-c.time<=confirmation_window?c.count+1:1;c.time=stamp;
      if(c.count>=clear_confirm_frames)erase.push_back(k);
    }
    for(Key k:erase){confirmed.erase(k);pending.erase(k);clearing.erase(k);}
    cleared_total_+=erase.size();
    for(auto it=clearing.begin();it!=clearing.end();)
      if(!confirmed.count(it->first)||stamp-it->second.time>confirmation_window)
        it=clearing.erase(it);else ++it;
    return erase.size();
  }
  KeySet occupied(double now){
    KeySet out;
    if(!observeTime(now))return out;
    for(auto it=confirmed.begin();it!=confirmed.end();){
      if(ttl>0 && now-it->second>ttl){
        clearing.erase(it->first);it=confirmed.erase(it);++expired_total_;
      }
      else{out.insert(it->first);++it;}
    }
    return out;
  }
  double oldestAge(double now)const{
    if(confirmed.empty()||!std::isfinite(now))return -1;
    double oldest=0;for(const auto &item:confirmed)oldest=std::max(oldest,now-item.second);
    return std::max(0.,oldest);
  }
  double newestAge(double now)const{
    if(confirmed.empty()||!std::isfinite(now))return -1;
    double newest=std::numeric_limits<double>::infinity();
    for(const auto &item:confirmed)newest=std::min(newest,now-item.second);
    return std::max(0.,newest);
  }
  size_t expiredTotal()const{return expired_total_;}
  size_t clearedTotal()const{return cleared_total_;}
  size_t timeResetTotal()const{return time_reset_total_;}
  void reset(){clearEvidence();last_time_=-1;}
private:
  double last_time_{-1};
  size_t expired_total_{0},cleared_total_{0},time_reset_total_{0};
  void clearEvidence(){pending.clear();clearing.clear();confirmed.clear();}
  bool observeTime(double stamp){
    if(!std::isfinite(stamp))return false;
    if(last_time_>=0&&stamp<last_time_-clock_rewind_tolerance){
      clearEvidence();++time_reset_total_;last_time_=stamp;return true;
    }
    last_time_=std::max(last_time_,stamp);return true;
  }
};
} // namespace fire_scout
