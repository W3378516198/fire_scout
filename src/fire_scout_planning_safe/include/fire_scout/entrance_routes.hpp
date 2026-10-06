#pragma once
#include "planner.hpp"
#include "route_topology.hpp"
#include "racer_task_allocator.hpp"
#include <limits>

namespace fire_scout {
// The second archive's route objective, unchanged. No whole-route repulsion,
// failed-route blacklist, or exploratory waypoint enters this objective.
inline double straightRouteCost(const Grid &grid, const std::vector<Vec3> &route) {
  if (route.size() < 2) return std::numeric_limits<double>::infinity();
  double cost = pathLength(route) + .20 * routeTurnLoss(route);
  const auto arc = arcLengths(route);
  const double scale = std::max(.05, grid.cfg.preferred_clearance);
  for (double s = 0; s <= arc.back() + 1e-9; s += .30) {
    const double clearance = grid.clearanceAt(
        atArc(route, arc, std::min(s, arc.back())), scale);
    const double q = 1.0 - std::clamp(clearance / scale, 0.0, 1.0);
    cost += 2.0 * q * q * .30;
  }
  return cost;
}
struct EntranceConfig {
  int count{3}; double budget_ms{300};
  TopologyConfig topology;
  bool valid() const {
    return topology.valid() && count>=1 && count<=6 &&
      std::isfinite(budget_ms) && budget_ms>=20 && budget_ms<=1000;
  }
};
struct EntranceSelection {
  PlanResult route;
  std::vector<RoutePassage> reserved;
  size_t evaluated{1}, shared{0}; bool changed{false};
  double elapsed_ms{0};
};
inline EntranceSelection selectEntranceRoute(const Grid &g, Vec3 start,
    PlanResult primary, const PlannerConfig &pc, const EntranceConfig &cfg,
    const std::vector<std::vector<Vec3>> &peers,
    const std::function<bool()> &cancelled={}) {
  EntranceSelection out; out.route=std::move(primary);
  if(!cfg.topology.enabled || cfg.count<=1 || peers.empty() ||
      out.route.points.size()<2) return out;
  const auto begin=std::chrono::steady_clock::now();
  auto remaining=[&]{return cfg.budget_ms-std::chrono::duration<double,std::milli>(
    std::chrono::steady_clock::now()-begin).count();};
  auto stop=[&]{return remaining()<2 || (cancelled && cancelled());};
  // Bound alternatives to the primary route's actual horizon endpoint.
  // A partial plan is not required to reach a far-away mission in one search.
  const Vec3 endpoint=out.route.points.back();
  out.reserved=reservablePassages(peerPassages(g,peers,cfg.topology,stop),
    start,endpoint,cfg.topology);
  out.shared=sharedPassageCount(out.route.points,out.reserved);
  const double baseline=pathLength(out.route.points);
  double best=straightRouteCost(g,out.route.points)+cfg.topology.gate_weight*out.shared;
  // Open corridors, already distinct routes and committed final approaches
  // return the identical primary geometry. Do not manufacture diversity there.
  if(out.shared==0 || stop()) return out;
  std::vector<RoutePassage> excluded;
  for(const auto &gate:out.reserved)
    if(routeCrossesPassage(out.route.points,gate)) excluded.push_back(gate);
  Grid guide=topologyGuideGrid(g,cfg.topology.search_resolution,stop);
  for(int attempt=1;attempt<cfg.count && remaining()>20 && !stop();++attempt) {
    Grid search(guide.cfg);
    if(!addPassageCuts(search,guide,excluded,cfg.topology,stop)) break;
    PlannerConfig c=pc; c.lock_valid_route=false;
    c.max_plan_distance=std::max(pc.max_plan_distance,distance(start,endpoint)+1.);
    c.enable_3d_search=false; c.multi_altitude_slices=false;
    c.search_margin=std::max(pc.search_margin,
      std::min(16.,cfg.topology.max_extra_length*.5));
    c.max_search_margin=std::max(c.max_search_margin,c.search_margin);
    c.max_search_ms=std::max(1.,std::min(140.,remaining()-10));
    c.max_retry_search_ms=c.max_search_ms;
    Planner planner(c); planner.setCancellation(stop);
    PlanResult candidate;
    do { candidate=planner.plan(search,start,endpoint); }
    while(candidate.points.empty() && candidate.reason=="SEARCH_BUDGET_RETRY" &&
      remaining()>15 && !stop());
    ++out.evaluated;
    if(candidate.points.size()<2 || distance(candidate.points.back(),endpoint)>.2) break;
    // Shorten on the private cut map, preserving the different entrance.
    candidate.points=visibilityShortcut(search,candidate.points,pc.allow_unknown,stop);
    const double length=pathLength(candidate.points);
    const size_t shared=sharedPassageCount(candidate.points,out.reserved);
    bool safe=true;
    for(size_t i=1;i<candidate.points.size();++i) {
      if(stop() || !g.segment(candidate.points[i-1],candidate.points[i],pc.allow_unknown)) {
        safe=false; break;
      }
    }
    const auto new_gates=routePassages(g,candidate.points,cfg.topology,stop);
    const double score=straightRouteCost(g,candidate.points)+cfg.topology.gate_weight*shared;
    const bool better_class=cfg.topology.prefer_distinct && shared<out.shared;
    const bool same_class=!cfg.topology.prefer_distinct || shared==out.shared;
    if(safe && length<=baseline*cfg.topology.max_detour_ratio &&
       length<=baseline+cfg.topology.max_extra_length &&
       (better_class || (same_class && score<best))) {
      candidate.reason="DISTINCT_PASSAGE_ROUTE";
      candidate.reused=false; candidate.locally_patched=false;
      out.route=std::move(candidate); out.changed=true; out.shared=shared; best=score;
    }
    // A zero-shared route is sufficient; do not pay to generate useless bends.
    if(out.shared==0) break;
    bool added=false;
    for(const auto &gate:new_gates) {
      bool exists=false;
      for(const auto &q:excluded) if(samePassage(q,gate,cfg.topology)) exists=true;
      if(!exists) { excluded.push_back(gate); added=true; }
    }
    if(!added) break;
  }
  out.elapsed_ms=cfg.budget_ms-remaining();
  return out;
}
} // namespace fire_scout
