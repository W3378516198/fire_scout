#pragma once
#include "geometry.hpp"
#include <sstream>
#include <string>
#include <set>
#include <map>
namespace fire_scout {
struct RoutePermit {
  int64_t stamp_ns{0},route_ns{0};bool ready{false};std::string reason;Vec3 goal{};
  int64_t instance{1};
  bool searching{false}; // Advisory only; never grants execution permission.
  std::string encode()const{
    std::ostringstream s;s<<"v=1 stamp_ns="<<stamp_ns<<" route_ns="<<route_ns
      <<" ready="<<ready<<" reason="<<reason<<" instance="<<instance;
    s.precision(17);s<<" goal_x="<<goal.x<<" goal_y="<<goal.y<<" goal_z="<<goal.z;
    if(searching)s<<" searching=1";
    return s.str();
  }
  static bool decode(const std::string&text,RoutePermit &out){
    std::istringstream input(text);std::string part;RoutePermit p;unsigned mask=0;
    while(input>>part){
      const auto equal=part.find('=');if(equal==std::string::npos)return false;
      const auto key=part.substr(0,equal),value=part.substr(equal+1);
      try{
        size_t used=0;
        if(key=="v"){if(value!="1"||mask&1)return false;mask|=1;}
        else if(key=="stamp_ns"||key=="route_ns"||key=="instance"){
          const auto n=std::stoll(value,&used);if(used!=value.size()||n<0)return false;
          const unsigned bit=key=="stamp_ns"?2:key=="route_ns"?4:128;
          if(mask&bit)return false;
          mask|=bit;
          if(bit==2)p.stamp_ns=n;else if(bit==4)p.route_ns=n;else p.instance=n;
        }else if(key=="ready"){
          if((value!="0"&&value!="1")||mask&8)return false;
          mask|=8;p.ready=value=="1";
        }else if(key=="searching"){
          if((value!="0"&&value!="1")||mask&256)return false;
          mask|=256;p.searching=value=="1";
        }else if(key=="reason"){p.reason=value;}
        else if(key=="goal_x"||key=="goal_y"||key=="goal_z"){
          const double n=std::stod(value,&used);if(used!=value.size()||!std::isfinite(n))return false;
          const unsigned bit=key=="goal_x"?16:key=="goal_y"?32:64;
          if(mask&bit)return false;
          mask|=bit;
          if(bit==16)p.goal.x=n;else if(bit==32)p.goal.y=n;else p.goal.z=n;
        }else return false;
      }catch(const std::exception&){return false;}
    }
    if((mask&255)!=255||p.route_ns<=0||p.instance<=0)return false;
    out=std::move(p);return true;
  }
};
struct RouteExecutionGate {
  RoutePermit permit;bool have{false};double received{-1};
  std::set<int64_t>retired_instances;
  bool observe(const std::string&wire,double ros_time,double steady_time){
    RoutePermit p;if(!RoutePermit::decode(wire,p))return false;
    const double age=ros_time-double(p.stamp_ns)*1e-9;
    if(age<-.05||age>2.)return false;
    // Reliable per-publisher ordering plus source-time ordering excludes late
    // old heartbeats. A /clock rewind is handled by the node's epoch reset.
    if(have&&p.instance==permit.instance&&p.stamp_ns<permit.stamp_ns)return false;
    if(have&&p.instance!=permit.instance){
      if(retired_instances.count(p.instance))return false;
      retired_instances.insert(permit.instance);
      if(retired_instances.size()>64)retired_instances.erase(retired_instances.begin());
    }
    permit=std::move(p);have=true;received=steady_time;return true;
  }
  bool allowed(int64_t revision,double ros_time,double steady_time,double timeout)const{
    const double age=ros_time-double(permit.stamp_ns)*1e-9;
    return have&&permit.ready&&permit.route_ns==revision&&age>=-.05&&age<=timeout&&
      steady_time>=received&&steady_time-received<=timeout;
  }
  void reset(){*this=RouteExecutionGate{};}
};
// Geometry and permission are sent on separate DDS topics. Install them only
// as a matching pair, without clearing the tracker or its velocity profile on
// ordinary heartbeats. A positive new permit may retain the old checked route
// for at most 0.20 wall seconds while its geometry arrives; a denial never can.
class GeometricRouteChannel {
public:
  RouteExecutionGate gate;
  int64_t active_revision{0}; std::vector<Vec3> active_path;
  uint64_t generation{0};
  bool path(int64_t revision,std::vector<Vec3> points) {
    if(revision<=0 || points.size()>20000)return false;
    for(Vec3 p:points)if(!finite(p))return false;
    candidates_[revision]=std::move(points);
    while(candidates_.size()>4)candidates_.erase(candidates_.begin());
    return true;
  }
  bool permit(const std::string &wire,double ros,double wall) {
    return gate.observe(wire,ros,wall);
  }
  bool synchronize(double ros,double wall,double timeout,Vec3 goal) {
    if(!gate.allowed(gate.permit.route_ns,ros,wall,timeout) ||
      distance(gate.permit.goal,goal)>.05)return false;
    auto p=candidates_.find(gate.permit.route_ns);
    if(p==candidates_.end() || p->second.empty())return false;
    const bool changed=active_revision!=p->first || active_instance_!=gate.permit.instance;
    if(changed){
      active_revision=p->first;active_instance_=gate.permit.instance;
      active_path=p->second;++generation;
    }
    active_gate_=gate;
    return changed;
  }
  bool allowed(double ros,double wall,double timeout,Vec3 goal) const {
    if(active_path.empty() || !gate.have || !gate.permit.ready ||
      distance(gate.permit.goal,goal)>.05 || gate.permit.instance!=active_instance_)return false;
    if(gate.allowed(active_revision,ros,wall,timeout))return true;
    return wall>=gate.received && wall-gate.received<=.20 &&
      active_gate_.allowed(active_revision,ros,wall,std::min(timeout,.30));
  }
  bool inputPaused(double ros,double wall,double timeout)const {
    if(!gate.have)return true;
    const double age=ros-double(gate.permit.stamp_ns)*1e-9;
    if(age<-.05 || age>timeout || wall<gate.received || wall-gate.received>timeout)return true;
    return !gate.permit.ready && gate.permit.reason!="NO_ROUTE" &&
      gate.permit.reason!="ROUTE_NOT_CERTIFIED";
  }
  void reset(){*this=GeometricRouteChannel{};}
private:
  std::map<int64_t,std::vector<Vec3>> candidates_;
  RouteExecutionGate active_gate_;int64_t active_instance_{0};
};
} // namespace fire_scout
