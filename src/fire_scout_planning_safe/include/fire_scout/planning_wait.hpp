#pragma once
#include "geometry.hpp"
#include <string>
namespace fire_scout {
// Missing geometry alone is not proof that the aircraft is physically trapped.
// Timer epochs survive path revisions and search heartbeats. A finite grace
// gives the planner time; a close obstacle then permits clearance-first escape.
class PlanningWait {
 public:
  double grace{2.0},wall_grace{6.0};
  std::string reason{"IDLE"};
  double seconds{0};
  void reset(){since_=wall_since_=-1;seconds=0;reason="IDLE";}
  bool deferRecovery(bool missing,bool searching,bool comfortable,bool forced,
      double ros,double wall){
    if(!missing||forced){reset();return false;}
    if(since_<0||ros<since_||wall<wall_since_){since_=ros;wall_since_=wall;}
    seconds=std::max(0.,ros-since_);
    if(searching&&comfortable){reason="SAFE_ANCHOR_SEARCHING";return true;}
    if(seconds<grace&&wall-wall_since_<wall_grace){reason="SEARCH_GRACE";return true;}
    reason=comfortable?"NO_ACTIVE_SEARCH":"CLEARANCE_ESCAPE";return false;
  }
 private:double since_{-1},wall_since_{-1};
};
} // namespace fire_scout
