#pragma once
#include <cmath>
#include <cstdint>
#include <string>
namespace fire_scout {
// PX4 DDS timestamps use the agent's wall clock when bridge time sync is on,
// and the common simulation clock when it is off. ROS sensor time stays intact.
class Px4CommandClock {
 public:
  enum class Domain { Unknown, Ros, System };
  std::string policy{"auto"};
  // Gazebo /clock must also be PX4's clock (UXRCE_DDS_SYNCT=0).
  // System-time DDS sync drifts when the simulator runs below real time.
  bool require_sim_clock{false};
  bool validPolicy()const{return policy=="auto"||policy=="ros"||policy=="system";}
  void observe(uint64_t source,uint64_t ros,uint64_t system,double steady){
    const auto delta=[](uint64_t a,uint64_t b){return a>b?a-b:b-a;};
    if(!source||!std::isfinite(steady)||!validPolicy()){invalidate();return;}
    const auto dr=delta(source,ros),ds=delta(source,system);
    Domain selected=Domain::Unknown;
    if(policy!="system"&&dr<=(require_sim_clock?500000U:2000000U))selected=Domain::Ros;
    if(!require_sim_clock&&policy!="ros"&&ds<=2000000&&(selected==Domain::Unknown||ds<=dr))selected=Domain::System;
    // Without simulated ROS time, ROS and system are the SAME clock.
    // Do not flap domains when arrival jitter changes which sample is closer.
    if(!require_sim_clock&&policy=="auto"&&dr<=2000000&&ds<=2000000&&delta(ros,system)<=100000)
      selected=Domain::System;
    if(selected==Domain::Unknown){invalidate();return;}
  const double observation_timeout = require_sim_clock ? 10.0 : 1.5;
  if(selected!=candidate_||steady<last_seen_||
   steady-last_seen_>=observation_timeout||       
   (last_source_>source&&last_source_-source>1000000)){
      candidate_=selected;consistent_=0;domain_=Domain::Unknown;candidate_since_=steady;
    }
    // Repeated DDS copies do not establish a new clock domain.
    if(source!=last_source_)++consistent_;
    last_source_=source;last_seen_=steady;
    if(consistent_>=3)domain_=candidate_;
  }
  bool ready(double steady)const{
    const double observation_timeout = require_sim_clock ? 10.0 : 1.5;
    return domain_!=Domain::Unknown&&std::isfinite(steady)&&
    steady>=last_seen_&&steady-last_seen_<observation_timeout&&
      (!flightLocked()||domain_==flight_domain_)&&
      (!require_sim_clock||flightLocked()||steady-candidate_since_>=3.0);
  }
  bool latch(double steady){if(!ready(steady))return false;flight_domain_=domain_;return true;}
  void reset(){
    invalidate();flight_domain_=Domain::Unknown;last_seen_=candidate_since_=-1e9;
  }
  bool flightLocked()const{return flight_domain_!=Domain::Unknown;}
  uint64_t stamp(uint64_t ros,uint64_t system,double steady)const{
    // A transient health failure must NEVER suppress an established flight
    // stream. The caller publishes ZERO velocity while health is degraded.
    const Domain selected=flightLocked()?flight_domain_:(ready(steady)?domain_:Domain::Unknown);
    return selected==Domain::Unknown?0:(selected==Domain::Ros?ros:system);
  }
  const char*name()const{const Domain d=flightLocked()?flight_domain_:domain_;
    return d==Domain::Ros?"ROS":d==Domain::System?"SYSTEM":"UNVERIFIED";}
  uint64_t sourceStamp()const{return last_source_;}
 private:
  void invalidate(){domain_=candidate_=Domain::Unknown;consistent_=0;last_source_=0;}
  Domain domain_{Domain::Unknown},candidate_{Domain::Unknown},flight_domain_{Domain::Unknown};
  int consistent_{0};uint64_t last_source_{0};double last_seen_{-1e9},candidate_since_{-1e9};
};

class TakeoffProgress {
 public:
  void begin(double time,double z){anchor_=z;since_=time;started_=true;}
  bool stalled(double time,double z){
    if(!started_||time<since_){begin(time,z);return false;}
    if(z>anchor_+.10){anchor_=z;since_=time;}
    return time-since_>8.;
  }
 private: double anchor_{0},since_{0};bool started_{false};
};
} // namespace fire_scout
