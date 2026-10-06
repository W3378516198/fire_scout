#include "fire_scout/entrance_routes.hpp"
#include "fire_scout/execution_permit.hpp"
#include "fire_scout/passage_scheduler.hpp"
#include "fire_scout/bspline.hpp"
#include "fire_scout/tracker.hpp"
#include "fire_scout/route_memory.hpp"
#include "fire_scout/route_switch.hpp"
#include "fire_scout/peer_safety.hpp"
int main(){return fire_scout::EntranceConfig{}.valid()?0:1;}
