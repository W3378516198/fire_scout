#pragma once
#include "grid.hpp"
#include "route_repair.hpp"
#include <chrono>
#include <queue>
#include <memory>
#include <functional>
#include <string>

namespace fire_scout {
struct PlannerConfig {
  bool allow_unknown{true};
  double unknown_cost{2.0}, vertical_weight{1.5}, altitude_weight{.25};
  double clearance_weight{4.0};
  double cruise_altitude{1.5}, search_margin{5.0}, max_plan_distance{24.0};
  double replan_improvement_ratio{.18}, replan_improvement_absolute{.45};
  int max_expansions{100000};
  double max_search_ms{90}, max_retry_search_ms{360};
  double max_search_margin{15}, heuristic_weight{1.5};
  bool lock_valid_route{true};
  double reverse_weight{3.0}, reverse_distance{3.0};
  bool local_repair_enabled{true};
  double repair_pre_margin{.45}, repair_post_margin{.75};
  double repair_max_span{6.0}, repair_search_margin{2.5};
  double repair_search_ms{140.0}, repair_max_detour_ratio{2.5};
  bool enable_3d_search{true};
  bool multi_altitude_slices{true};
  double flat_search_fraction{.45};
  double vertical_bridge_step{.30}, vertical_bridge_max_change{1.20};
  double altitude_boundary_weight{8.0}, altitude_boundary_reserve{.15};
};
struct PlanResult {
  std::vector<Vec3> points;
  std::string reason;
  int expansions{0};
  int flat_expansions{0}, spatial_expansions{0};
  double elapsed_ms{0};
  bool reused{false};
  bool locally_patched{false};
  size_t patch_begin{0}, patch_end{0};
  std::string search_mode;
  double flat_altitude{0},flat_search_margin{0};
  bool flat_exhausted{false};
};
class Planner {
public:
  explicit Planner(PlannerConfig c = {}) : cfg(c) {}
  PlannerConfig cfg;
  void reset() {
    active_.clear();
    route_penalty_.clear();
    clearSearches();
    failures_=0;
    repair_requested_=false;
    active_progress_ = 0;
    active_goal_ = {1e50, 1e50, 1e50};
  }
  // Age alone is not evidence that a frozen search is wrong. Free-ray updates
  // in optimistic mode change cost, not connectivity. Preserve that work;
  // restart only the affected altitude/frontier when its collision geometry
  // changes (or observed-space reachability changes in known-only mode).
  size_t refreshChangedSearches(const Grid &live){
    size_t changed=0;
    for(auto &saved:searches_)if(saved){
      const auto &s=*saved;
      if(!sameSearchEvidence(live,s)){
        saved.reset();++changed;
      }
    }
    return changed;
  }
  size_t retainedSearchNodes()const{
    size_t count=0;for(const auto &s:searches_)if(s)count+=s->score.size();return count;
  }
  const std::vector<Vec3> &active() const { return active_; }
  // Feed the exact published geometry back after a successful replacement. A
  // temporary safe-prefix heartbeat must not erase the intended route tail.
  void setCommittedRoute(const std::vector<Vec3> &p) { active_=p; active_progress_=0; }
  void setCancellation(std::function<bool()> f) { cancelled_=std::move(f); }
  void requestRepair() { repair_requested_=true; }
  void clearRoutePenalties() { route_penalty_.clear(); }
  // Add a finite soft-cost tube around the short, time-aligned future of peer
  // routes. It is never treated as a wall, so a single-door room remains
  // reachable. The caller clears this transient layer before every search.
  void addRoutePenalty(const Grid &g,
                       const std::vector<std::vector<Vec3>> &routes,
                       double radius, double weight, double sample_spacing) {
    if (routes.empty() || !std::isfinite(radius) || radius <= 0 ||
        !std::isfinite(weight) || weight <= 0 ||
        !std::isfinite(sample_spacing) || sample_spacing <= 0)
      return;
    const double resolution = g.cfg.resolution;
    const int cells = int(std::ceil(radius / resolution));
    for (const auto &route : routes) {
      if (route.empty()) continue;
      const auto arc = arcLengths(route);
      const double length = arc.back();
      for (double s = 0; s <= length + 1e-9; s += sample_spacing) {
        const Vec3 sample = atArc(route, arc, std::min(s, length));
        const Key centre_key = g.toKey(sample);
        for (int dx = -cells; dx <= cells; ++dx)
          for (int dy = -cells; dy <= cells; ++dy)
            for (int dz = -cells; dz <= cells; ++dz) {
              const Key candidate{centre_key.x + dx, centre_key.y + dy,
                                  centre_key.z + dz};
              const double d = distance(g.point(candidate), sample);
              if (d > radius) continue;
              const double q = 1.0 - d / radius;
              const double value = weight * q * q;
              auto found = route_penalty_.find(candidate);
              if (found == route_penalty_.end())
                route_penalty_.emplace(candidate, value);
              else
                found->second = std::max(found->second, value);
            }
      }
    }
  }
  // Call with odometry while no search worker owns this Planner. Route locking
  // must not freeze progress at zero until a distant map update forces repair.
  void advanceCommittedProgress(Vec3 current) {
    if(active_.empty())return;
    const auto arc=arcLengths(active_);
    const auto pr=project(active_,arc,current,std::max(0.,active_progress_-.10),
                          std::min(arc.back(),active_progress_+2.0));
    if(std::isfinite(pr.error) && pr.error<=1.2)active_progress_=std::max(active_progress_,pr.s);
  }
  PlanResult plan(const Grid &g, Vec3 start, Vec3 goal,
                  Vec3 preferred_direction = {}) {
    const auto begin = std::chrono::steady_clock::now();
    PlanResult result;
    const bool force_repair=repair_requested_;
    repair_requested_=false;
    auto done = [&](std::string reason) {
      result.reason = std::move(reason);
      result.elapsed_ms =
          std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin)
              .count();
      return result;
    };
    if (cancelled()) return done("CANCELLED");
    if (!finite(start) || !finite(goal))
      return done("INVALID_POSITION");
    bool new_goal = distance(goal, active_goal_) > .05;
    if (new_goal) {
      active_.clear();
      clearSearches();
      failures_=0;
      active_progress_ = 0;
    }
    active_goal_ = goal;
    budget_ms_=std::max(cfg.max_search_ms,std::min(cfg.max_retry_search_ms,cfg.max_search_ms*(1+failures_)));
    margin_=std::max(cfg.search_margin,std::min(cfg.max_search_margin,cfg.search_margin*(1+.5*failures_)));
    bool complete=false;
    const auto inspection = inspectRoute(g, active_, start, cfg.allow_unknown,
                                         active_progress_);
    auto remaining = trim(g, start, active_, complete);
    if (norm(route_direction_) < 1e-6 && finite(preferred_direction))
      route_direction_ = preferred_direction;
    bool have_old = complete && !remaining.empty();

    if (!g.segment(start, start, cfg.allow_unknown)) {
      active_.clear();active_progress_=0;
      if(!g.recoveryClear(start,cfg.allow_unknown))return done("START_PHYSICAL_BLOCKED");
      Vec3 best{};double score=1e100;
      for(double radius : {.20,.35,.55,.80})for(int i=0;i<48;++i){
        double angle=2*pi*i/48;
        Vec3 p=start+Vec3{radius*std::cos(angle),radius*std::sin(angle),0};
        if(!g.segment(p,p,cfg.allow_unknown)||!g.segmentFrom(start,p,cfg.allow_unknown))continue;
        double c=distance(p,goal)+.3*radius;
        if(c<score){score=c;best=p;}
      }
      if(score<1e99){result.points={start,best};return done("RECOVERY_MARGIN_ESCAPE");}
      return done("START_BLOCKED_RETRY");
    }
    if (distance(start, goal) < .12 && g.segment(start, goal, cfg.allow_unknown)) {
      active_ = {start, goal};
      result.points = active_;
      return done("GOAL");
    }
    if (!g.segment(goal, goal, cfg.allow_unknown) && cfg.allow_unknown) {
      // A mission endpoint inside a wall is an input error, never snap it across a wall.
      if (have_old) {
        result.points = remaining;
        result.reused = true;
        return done("KEEP_ROUTE_GOAL_BLOCKED");
      }
      return done("GOAL_BLOCKED");
    }
    if(have_old && !force_repair && cfg.lock_valid_route && distance(remaining.back(),goal)<.15){
      result.points=remaining;result.reused=true;failures_=0;
      return done("KEEP_VALID_ROUTE");
    }
    // Freeze the current local horizon while any search is unfinished. Moving
    // 5 cm must not move a distant mission's clipped endpoint by 5 cm and erase
    // the accumulated frontier on every retry.
    if(!have_search_goal_) {
      search_goal_=goal;
      if(distance(start,goal)>cfg.max_plan_distance)
        search_goal_=start+(goal-start)*(cfg.max_plan_distance/distance(start,goal));
      have_search_goal_=true;
    }
    goal=search_goal_;
    std::vector<Vec3> candidate;
    bool repaired=false;
    // V19 repairs only the invalid interval and rejoins the exact committed
    // tail.  It no longer keeps an arbitrary 1.5 m prefix and replans everything
    // after it, which was the main source of visually dramatic route changes.
    if (cfg.local_repair_enabled && !complete && inspection.projection_valid) {
      const auto window = localRepairWindow(
          inspection, cfg.repair_pre_margin, cfg.repair_post_margin,
          cfg.repair_max_span);
      if (window.valid && !outOfTime(begin)) {
        PlannerConfig patch_cfg = cfg;
        patch_cfg.lock_valid_route = false;
        patch_cfg.local_repair_enabled = false;
        patch_cfg.search_margin = std::min(cfg.max_search_margin,
                                           cfg.repair_search_margin);
        patch_cfg.max_search_margin = patch_cfg.search_margin;
        patch_cfg.max_plan_distance = std::max(
            2.0, cfg.repair_max_span + cfg.repair_post_margin);
        patch_cfg.max_search_ms = std::min(cfg.max_retry_search_ms,
                                           cfg.repair_search_ms);
        patch_cfg.max_retry_search_ms = patch_cfg.max_search_ms;
        Planner patcher(patch_cfg);
        Vec3 tangent{};
        if (window.prefix.size() >= 2)
          tangent = window.prefix.back() -
                    window.prefix[window.prefix.size() - 2];
        else if (inspection.suffix.size() >= 2)
          tangent = inspection.suffix[1] - inspection.suffix[0];
        auto patch_result = patcher.plan(g, window.prefix.back(),
                                         window.tail.front(), tangent);
        if (!patch_result.points.empty()) {
          const double direct = distance(window.prefix.back(),
                                         window.tail.front());
          const double detour = pathLength(patch_result.points);
          size_t patch_begin = 0, patch_end = 0;
          auto patched = spliceLocalRepair(window, patch_result.points,
                                           &patch_begin, &patch_end);
          if (!patched.empty() &&
              detour <= cfg.repair_max_detour_ratio *
                            std::max(.25, direct) + .25 &&
              routeGeometryValid(g, patched, cfg.allow_unknown)) {
            candidate = std::move(patched);
            repaired = true;
            result.locally_patched = true;
            result.patch_begin = patch_begin;
            result.patch_end = patch_end;
            result.expansions += patch_result.expansions;
            result.flat_expansions += patch_result.flat_expansions;
            result.spatial_expansions += patch_result.spatial_expansions;
            result.search_mode = "LOCAL_REJOIN";
          }
        }
      }
    }
    if(candidate.empty() && !outOfTime(begin) && result.expansions<cfg.max_expansions)
      candidate=searchModes(g,start,goal,route_direction_,2,result,begin);
    if(cancelled()) { clearSearches(); return done("CANCELLED"); }
    if (candidate.empty()) {
      if (force_repair){
        repair_requested_=true; // retry must continue bypassing the lock
      }
      if (have_old && !force_repair && distance(remaining.back(),active_goal_)<.15) {
        result.points = remaining;
        result.reused = true;
        return done("KEEP_VALID_ROUTE");
      }
      ++failures_;
      return done(outOfTime(begin) || result.expansions>=cfg.max_expansions ? "SEARCH_BUDGET_RETRY" : "NO_PATH_RETRY");
    }
    if(!repaired)candidate = prune(g, candidate);
    if(cancelled()) { clearSearches(); return done("CANCELLED"); }
    if (have_old && !force_repair) {
      double old_cost = cost(g, remaining), new_cost = cost(g, candidate);
      bool same_end = distance(remaining.back(), candidate.back()) < .15;
      // Reuse only after every remaining segment has been checked against this map.
      if (same_end && new_cost >= old_cost * (1 - cfg.replan_improvement_ratio) -
                                      cfg.replan_improvement_absolute) {
        result.points = remaining;
        result.reused = true;
        return done("KEEP_VALID_ROUTE");
      }
    }
    failures_=0;
    clearSearches();
    active_ = candidate;
    active_progress_ = 0;
    result.points = candidate;
    return done(repaired?"PATCHED_ROUTE_REJOIN":"NEW_ROUTE");
  }

private:
  std::vector<Vec3> active_;
  std::unordered_map<Key,double,KeyHash> route_penalty_;
  double active_progress_{0}, budget_ms_{90}, margin_{5};
  int failures_{0};
  bool repair_requested_{false},have_search_goal_{false};
  Vec3 search_goal_{};
  Vec3 active_goal_{1e50, 1e50, 1e50}, route_direction_{};
  std::function<bool()> cancelled_;
  bool cancelled() const { return cancelled_ && cancelled_(); }
  struct Item {
    Key k; double f,g;
    bool operator<(const Item &b) const {
      return f!=b.f?f>b.f:g!=b.g?g<b.g:b.k<k;
    }
  };
  struct SearchState {
    // A frozen map makes retained costs/parents coherent. Publication is
    // independently checked on the latest map, including every old edge.
    Grid map;
    Vec3 start,goal,direction;
    Key sk,target,best;
    bool flat{false},exhausted{false};
    uint64_t evidence_revision{0};
    double margin{0},best_score{std::numeric_limits<double>::infinity()};
    std::priority_queue<Item> open;
    std::unordered_map<Key,double,KeyHash> score;
    std::unordered_map<Key,Key,KeyHash> parent;
    std::unordered_map<Key,Vec3,KeyHash> navigation_points;
    std::unordered_map<Key,bool,KeyHash> point_clear;
    std::unordered_map<Key,double,KeyHash> point_cost;
    explicit SearchState(Grid g):map(std::move(g)){}
  };
  bool sameSearchEvidence(const Grid &live,const SearchState&s)const{
    const double r=live.cfg.resolution;
    const double pad=live.cfg.inflation_xy+std::max(0.,live.cfg.preferred_clearance)+r;
    const double xmin=std::min(s.start.x,s.goal.x)-s.margin-pad,xmax=std::max(s.start.x,s.goal.x)+s.margin+pad;
    const double ymin=std::min(s.start.y,s.goal.y)-s.margin-pad,ymax=std::max(s.start.y,s.goal.y)+s.margin+pad;
    const double zmin=s.flat?std::min(s.start.z,s.goal.z):live.cfg.z_min;
    const double zmax=s.flat?std::max(s.start.z,s.goal.z):live.cfg.z_max;
    auto same=[&](const KeySet&current,const KeySet&frozen,double zpad){
      size_t count=0;
      for(Key k:current){const Vec3 p=live.point(k);
        if(p.x<xmin||p.x>xmax||p.y<ymin||p.y>ymax||p.z<zmin-zpad||p.z>zmax+zpad)continue;
        if(!frozen.count(k))return false;
        ++count;
      }
      return count==frozen.size();
    };
    return same(live.occupied,s.map.occupied,live.cfg.inflation_z+r)&&
      (cfg.allow_unknown||same(live.free,s.map.free,r));
  }
  // Retain only evidence that can influence this bounded search. The original
  // map often contains hundreds of thousands of free cells in other rooms and
  // height layers; copying all of them can consume the whole planning budget.
  // Padding includes the full collision cylinder, clearance cost and voxel
  // centers. No obstacle influencing an in-bounds edge is removed.
  Grid searchSnapshot(const Grid &g,Vec3 start,Vec3 goal,bool flat,double margin) const {
    const double r=g.cfg.resolution;
    const double pad=g.cfg.inflation_xy+std::max(0.,g.cfg.preferred_clearance)+r;
    const double xmin=std::min(start.x,goal.x)-margin-pad;
    const double xmax=std::max(start.x,goal.x)+margin+pad;
    const double ymin=std::min(start.y,goal.y)-margin-pad;
    const double ymax=std::max(start.y,goal.y)+margin+pad;
    const double zmin=flat?std::min(start.z,goal.z):g.cfg.z_min;
    const double zmax=flat?std::max(start.z,goal.z):g.cfg.z_max;
    KeySet occupied,free;
    auto xy=[&](Vec3 p){return p.x>=xmin && p.x<=xmax && p.y>=ymin && p.y<=ymax;};
    for(Key k:g.occupied){Vec3 p=g.point(k);if(xy(p) && p.z>=zmin-(g.cfg.inflation_z+r) && p.z<=zmax+(g.cfg.inflation_z+r))occupied.insert(k);}
    for(Key k:g.free){Vec3 p=g.point(k);if(xy(p) && p.z>=zmin-r && p.z<=zmax+r)free.insert(k);}
    Grid snapshot(g.cfg);snapshot.update(std::move(occupied),std::move(free));return snapshot;
  }
  std::array<std::unique_ptr<SearchState>,12> searches_;
  std::array<unsigned,2> flat_turn_{};
  void clearSearches(){for(auto &s:searches_)s.reset();flat_turn_={};route_direction_={};have_search_goal_=false;}

  bool outOfTime(std::chrono::steady_clock::time_point t) const {
    return cancelled() || std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t)
               .count() >= budget_ms_;
  }
  double edgeCost(const Grid &g, Vec3 a, Vec3 b) const {
    double d = distance(a, b);
    if (d < 1e-9)
      return 0;
    const int n = std::max(1, int(std::ceil(d / (g.cfg.resolution * .5))));
    double mult = 0;
    for (int i = 1; i <= n; ++i) {
      Vec3 p = a + (b - a) * (double(i) / n);
      mult += 1 + (g.known(g.toKey(p)) ? 0 : cfg.unknown_cost) +
              cfg.altitude_weight * std::abs(p.z - cfg.cruise_altitude) +
              cfg.clearance_weight * g.proximity(g.toKey(p)) + boundaryCost(g,p) +
              routePenalty(g.toKey(p));
    }
    return d * mult / n + cfg.vertical_weight * std::abs(b.z - a.z);
  }
  double routePenalty(Key k) const {
    const auto found = route_penalty_.find(k);
    return found == route_penalty_.end() ? 0.0 : found->second;
  }
  double boundaryCost(const Grid&g,Vec3 p)const {
    const double reserve=std::max(.01,cfg.altitude_boundary_reserve);
    const double gap=std::min(p.z-g.cfg.z_min,g.cfg.z_max-p.z);
    const double deficit=std::clamp(1-gap/reserve,0.,1.);
    return std::max(0.,cfg.altitude_boundary_weight)*deficit*deficit;
  }
  double cost(const Grid &g, const std::vector<Vec3> &p) const {
    double c = 0;
    for (size_t i = 1; i < p.size(); ++i)
      c += edgeCost(g, p[i - 1], p[i]);
    return c;
  }
  std::vector<Vec3> trim(const Grid &g,Vec3 start,const std::vector<Vec3>&p,bool &complete) {
    complete=false;route_direction_={};
    if(p.empty())return {};
    auto s=arcLengths(p);
    auto pr=project(p,s,start,std::max(0.,active_progress_-.10),std::min(s.back(),active_progress_+2.0));
    if(!std::isfinite(pr.error)||pr.error>1.2)return {};
    active_progress_=std::max(active_progress_,pr.s);
    route_direction_=atArc(p,s,std::min(s.back(),active_progress_+.8))-atArc(p,s,active_progress_);
    double join_s=std::min(s.back(),active_progress_+.35);
    Vec3 join=atArc(p,s,join_s);
    if(!g.segment(start,join,cfg.allow_unknown)) {
      join_s=active_progress_;join=atArc(p,s,join_s);
      if(!g.segment(start,join,cfg.allow_unknown))return {};
    }
    std::vector<Vec3> out{start};
    if(distance(start,join)>.02)out.push_back(join);
    for(size_t i=1;i<p.size();++i)if(s[i]>join_s+1e-6) {
      if(g.segment(out.back(),p[i],cfg.allow_unknown))out.push_back(p[i]);
      else {
        // Sparse polylines can contain a long valid prefix before the first
        // blocked segment. Keep it, with a small reserve before its boundary.
        Vec3 a=out.back(),d=p[i]-a;double lo=0,hi=1;
        for(int n=0;n<14;++n){double m=(lo+hi)*.5;if(g.segment(a,a+d*m,cfg.allow_unknown))lo=m;else hi=m;}
        lo=std::max(0.,lo-.03/std::max(.001,norm(d)));
        if(lo*norm(d)>.02)out.push_back(a+d*lo);
        return pathLength(out)>.35?out:std::vector<Vec3>{};
      }
    }
    if(out.size()==1)out.push_back(p.back());
    complete=true;return out;
  }
  std::vector<Vec3> prune(const Grid &g, const std::vector<Vec3> &p) const {
    if (p.size() < 3)
      return p;
    std::vector<double> c(p.size(), 0);
    for (size_t i = 1; i < p.size(); ++i)
      c[i] = c[i - 1] + edgeCost(g, p[i - 1], p[i]);
    std::vector<Vec3> out{p.front()};
    size_t i = 0;
    while (i + 1 < p.size()) {
      if(cancelled())return {};
      size_t best = i + 1;
      for (size_t j = p.size() - 1; j > i + 1; --j) {
        if(cancelled())return {};
        if (g.segment(p[i], p[j], cfg.allow_unknown) &&
            edgeCost(g, p[i], p[j]) <= 1.08 * (c[j] - c[i]) + .03) {
          best = j;
          break;
        }
      }
      out.push_back(p[best]);
      i = best;
    }
    return out;
  }
  std::vector<Vec3> searchModes(const Grid &g,Vec3 start,Vec3 goal,Vec3 direction,
      size_t slot,PlanResult &result,std::chrono::steady_clock::time_point begin) {
    std::vector<Vec3> candidate;
    // A goal at z=1.0 used to disable ALL flat work because cruise was 1.5.
    // Certify vertical connectors to a useful level slice. A low goal
    // must not force the whole flight below a window sill. Retain the plane
    // of an unfinished search so odometry changes cannot erase its frontier.
    const bool level=goal.z>=g.cfg.z_min&&goal.z<=g.cfg.z_max;
    // A flat search used to consume two entire calls before 3-D received any
    // work. Reserve part of EVERY call for 3-D, retaining both frontiers.
    // Both the wall-clock deadline and expansion allowance are shared, not
    // independently granted to each mode.
    if(level) {
      Vec3 flat_start=start,flat_goal=goal;bool connectors=false;
      // A cruise slice may be disconnected by a sill or conservative doorway
      // evidence. Alternate retained heights before flooding the whole floor.
      const size_t group=slot/2;
      // Give cruise the first two calls, then service all retained planes.
      // Waiting for the entire maximum XY box to be exhausted can starve the
      // nearby lower/upper opening for the whole snapshot lifetime.
      const unsigned turn=flat_turn_[group]++;
      const unsigned plane_index=cfg.enable_3d_search&&cfg.multi_altitude_slices&&turn>=2?
        (turn-2)%5:0;
      const size_t flat_slot=plane_index?4+group*4+(plane_index-1):slot;
      const double offsets[5]={0,-1,1,-2,2};
      const double nominal=cfg.cruise_altitude+offsets[plane_index]*std::max(.10,cfg.vertical_bridge_step);
      const double first_plane=searches_[flat_slot]?searches_[flat_slot]->start.z:nominal;
      for(double plane:{first_plane,goal.z,start.z}){
        flat_start.z=flat_goal.z=plane;
        // An unknown goal is still a search direction in known-space mode.
        // Requiring it to be observed here disables safe frontier routes.
        // Check physical geometry now; certify its observation before ever
        // appending the terminal connector to the returned flight path.
        if(g.segment(start,flat_start,cfg.allow_unknown)&&g.segment(flat_goal,goal,true)){
          connectors=true;break;
        }
      }
      if(connectors) {
        const double elapsed=std::chrono::duration<double,std::milli>(
          std::chrono::steady_clock::now()-begin).count();
        const double share=cfg.enable_3d_search?std::clamp(cfg.flat_search_fraction,.1,.9):1.;
        const double deadline=elapsed+std::max(0.,budget_ms_-elapsed)*share;
        const int remaining=std::max(0,cfg.max_expansions-result.expansions);
        const int quota=cfg.enable_3d_search?std::max(1,int(remaining*share)):remaining;
        result.flat_altitude=flat_start.z;result.flat_exhausted=false;
        candidate=search(g,flat_start,flat_goal,true,direction,flat_slot,result,begin,
                         deadline,result.expansions+quota);
      }
      if(!candidate.empty() && distance(candidate.front(),start)>1e-5)candidate.insert(candidate.begin(),start);
      if(!candidate.empty() && distance(candidate.back(),flat_goal)<1e-6 && distance(flat_goal,goal)>1e-6 &&
         g.segment(flat_goal,goal,cfg.allow_unknown))
        candidate.push_back(goal);
    }
    if(cfg.enable_3d_search && !outOfTime(begin)) {
      // A few checked altitude bridges expose low beams/low obstacles to the
      // planner immediately. They are only seeds: full cylinder sweeps and the
      // same clearance/unknown/altitude costs apply, then normal pruning and
      // spline certification. An occupied ceiling can never be crossed.
      auto bridge=candidate.empty()?altitudeBridge(g,start,goal,begin):std::vector<Vec3>{};
      if(!bridge.empty() && (candidate.empty() || cost(g,bridge)<.90*cost(g,candidate))) {
        candidate=std::move(bridge);result.search_mode="ALTITUDE_BRIDGE";
      }
    }
    if(candidate.empty() && cfg.enable_3d_search && !outOfTime(begin) && result.expansions<cfg.max_expansions)
      candidate=search(g,start,goal,false,direction,slot+1,result,begin,budget_ms_,cfg.max_expansions);
    return candidate;
  }
  std::vector<Vec3> altitudeBridge(const Grid&g,Vec3 start,Vec3 goal,
      std::chrono::steady_clock::time_point begin)const {
    if(cfg.vertical_bridge_step<=0 || cfg.vertical_bridge_max_change<=0 ||
       g.segment(start,goal,cfg.allow_unknown))return {};
    std::vector<Vec3> best;double best_cost=std::numeric_limits<double>::infinity();
    const double step=std::max(g.cfg.resolution,cfg.vertical_bridge_step);
    const int levels=std::min(12,int(std::floor(cfg.vertical_bridge_max_change/step+1e-9)));
    for(int n=1;n<=levels && !outOfTime(begin);++n)for(double sign:{1.,-1.}) {
      if(outOfTime(begin))break;
      const double z=sign>0?std::max(start.z,goal.z)+n*step:std::min(start.z,goal.z)-n*step;
      if(z<g.cfg.z_min || z>g.cfg.z_max)continue;
      Vec3 a{start.x,start.y,z},b{goal.x,goal.y,z};
      if(!g.segment(start,a,cfg.allow_unknown)||!g.segment(a,b,cfg.allow_unknown)||
         !g.segment(b,goal,cfg.allow_unknown))continue;
      std::vector<Vec3> route{start,a,b,goal};const double value=cost(g,route);
      if(value<best_cost){best_cost=value;best=std::move(route);}
    }
    return best;
  }
  std::vector<Vec3> search(const Grid &live,Vec3 start,Vec3 goal,bool flat,Vec3 direction,
      size_t slot,PlanResult &result,std::chrono::steady_clock::time_point begin,
      double deadline_ms,int expansion_limit) {
    auto &saved=searches_[slot];
    if(saved && (distance(saved->goal,goal)>.05 || distance(saved->start,start)>.8 ||
                 !live.segment(start,saved->start,cfg.allow_unknown)))saved.reset();
    if(saved&&saved->exhausted&&saved->evidence_revision!=live.evidenceRevision()){
      if(!sameSearchEvidence(live,*saved))saved.reset();
      else saved->evidence_revision=live.evidenceRevision();
    }
    if(saved&&saved->exhausted&&saved->margin+1e-8<margin_)saved.reset();
    if(saved&&saved->exhausted){
      if(flat){result.flat_exhausted=true;result.flat_search_margin=saved->margin;}
      return {};
    }
    if(!saved) {
      saved=std::make_unique<SearchState>(searchSnapshot(live,start,goal,flat,margin_));
      auto &s=*saved;s.start=start;s.goal=goal;s.direction=limitNorm(direction,1.);s.evidence_revision=live.evidenceRevision();
      s.sk=live.toKey(start);s.target=live.toKey(goal);s.best=s.sk;s.flat=flat;s.margin=margin_;
      if((flat && s.sk.z!=s.target.z) || !live.segment(start,start,cfg.allow_unknown)){saved.reset();return {};}
      s.score[s.sk]=0;s.open.push({s.sk,distance(start,goal),0});
    }
    auto &s=*saved;const Grid &g=s.map;
    const double xmin=std::min(s.start.x,s.goal.x)-s.margin,xmax=std::max(s.start.x,s.goal.x)+s.margin;
    const double ymin=std::min(s.start.y,s.goal.y)-s.margin,ymax=std::max(s.start.y,s.goal.y)+s.margin;
    auto point=[&](Key k){
      if(k==s.sk)return s.start;
      if(k==s.target)return s.goal;
      auto cached=s.navigation_points.find(k);if(cached!=s.navigation_points.end())return cached->second;
      Vec3 p=g.point(k);if(flat)p.z=s.start.z;
      // A 1.0 m voxel opening and a 0.938 m envelope have valid positions at
      // the opening centre, yet neither adjacent 10 cm CELL CENTRE fits.
      // Search real subcell positions without changing occupancy or radius.
      // Segment certification still checks every edge, including diagonals.
      if(p.z>=g.cfg.z_min&&p.z<=g.cfg.z_max && (cfg.allow_unknown||g.known(k)) &&
         !g.segment(p,p,cfg.allow_unknown) && g.clearanceAt(p)>-.72*g.cfg.resolution){
        Vec3 best=p;double best_room=0;
        for(int dx=-1;dx<=1;++dx)for(int dy=-1;dy<=1;++dy){
          if(dx==0&&dy==0)continue;
          Vec3 q=p+Vec3{dx*.5*g.cfg.resolution,dy*.5*g.cfg.resolution,0};
          if(!g.segment(q,q,cfg.allow_unknown))continue;
          const double room=g.clearanceAt(q);
          if(room>best_room){best=q;best_room=room;}
        }
        p=best;
      }
      s.navigation_points[k]=p;return p;
    };
    auto h=[&](Key k){return distance(point(k),s.goal)+cfg.vertical_weight*std::abs(point(k).z-s.goal.z);};
    bool found=false;
    // Check before popping: a budget boundary must not silently lose the
    // frontier item. max_expansions is the work per call, not lifetime work.
    while(!s.open.empty() && result.expansions<expansion_limit) {
      if((result.expansions%16)==0 && (cancelled() ||
          std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count()>=deadline_ms))return {};
      Item a=s.open.top();s.open.pop();
      if(a.g>s.score[a.k]+1e-8)continue;
      ++result.expansions;
      if(flat)++result.flat_expansions;else ++result.spatial_expansions;
      if(a.k==s.target){s.best=a.k;found=true;break;}
      if(!cfg.allow_unknown && distance(point(a.k),s.start)>.8) {
        double value=a.g+1.8*h(a.k);if(value<s.best_score){s.best_score=value;s.best=a.k;}
      }
      const Vec3 from=point(a.k);
      for(int dx=-1;dx<=1;++dx)for(int dy=-1;dy<=1;++dy)
        for(int dz=flat?0:-1;dz<=(flat?0:1);++dz) {
          if(dx==0 && dy==0 && dz==0)continue;
          Key k{a.k.x+dx,a.k.y+dy,a.k.z+dz};Vec3 p=point(k);
          // A flat search flies at cruise_altitude, NOT at the voxel center.
          // At .10 m resolution 1.50 m was tested at 1.55 m by walkable(),
          // closing valid passages near a sill/overhang and wasting the whole
          // flat-search budget before retrying in 3-D.
          if(p.x<xmin || p.x>xmax || p.y<ymin || p.y>ymax)continue;
          auto valid=s.point_clear.find(k);
          if(valid==s.point_clear.end())valid=s.point_clear.emplace(k,g.segment(p,p,cfg.allow_unknown)).first;
          if(!valid->second || !g.segment(from,p,cfg.allow_unknown))continue;
          auto cost=s.point_cost.find(k);
          if(cost==s.point_cost.end())cost=s.point_cost.emplace(k,
              1+(g.known(k)?0:cfg.unknown_cost)+cfg.altitude_weight*std::abs(p.z-cfg.cruise_altitude)+
              cfg.clearance_weight*g.proximity(k)+boundaryCost(g,p)+routePenalty(k)).first;
          double step=distance(from,p)*cost->second+cfg.vertical_weight*std::abs(p.z-from.z);
          // Soft, finite reversal cost applies only near the repair origin.
          // It discourages alternating entrances but permits necessary retreat.
          double fade=std::max(0.,1.-distance(from,s.start)/std::max(.01,cfg.reverse_distance));
          step+=std::max(0.,cfg.reverse_weight)*fade*std::max(0.,-dot(p-from,s.direction));
          double ng=a.g+step;auto it=s.score.find(k);
          if(it==s.score.end() || ng<it->second-1e-8){s.score[k]=ng;s.parent[k]=a.k;s.open.push({k,ng+cfg.heuristic_weight*h(k),ng});}
        }
    }
    if(!found && !s.open.empty())return {}; // resume this exact frontier next tick
    if(!found && (cfg.allow_unknown || s.best==s.sk)){
      if(flat){result.flat_exhausted=true;result.flat_search_margin=s.margin;}
      // Remember a disconnected slice; rerun only for a larger region or new
      // geometry. Restarting its identical flood fill stole every retry's CPU.
      s.exhausted=true;return {};
    }
    std::vector<Vec3> p;Key k=s.best;
    for(size_t n=0;n<=s.parent.size();++n) {
      p.push_back(point(k));if(k==s.sk)break;
      auto it=s.parent.find(k);if(it==s.parent.end()){saved.reset();return {};}
      k=it->second;
    }
    std::reverse(p.begin(),p.end());
    if(found && distance(p.back(),s.goal)>1e-8)p.push_back(s.goal);
    if(p.size()==1)p.push_back(s.goal);
    // Current vehicle/repair anchor may have moved while the search ran.
    // Join only the future route portion rather than revisiting its old root.
    if(distance(start,p.front())>1e-6) {
      auto arc=arcLengths(p);auto pr=project(p,arc,start,0,std::min(2.,arc.back()));
      double join_s=std::min(arc.back(),pr.s+.15);Vec3 join=atArc(p,arc,join_s);
      if(!live.segment(start,join,cfg.allow_unknown)){saved.reset();return {};}
      std::vector<Vec3> joined{start};if(distance(start,join)>1e-7)joined.push_back(join);
      for(size_t i=1;i<p.size();++i)if(arc[i]>join_s+1e-8)joined.push_back(p[i]);
      p=std::move(joined);
    } else p.front()=start;
    // Nothing from the frozen snapshot may bypass current collision evidence.
    for(size_t i=1;i<p.size();++i)if(!live.segment(p[i-1],p[i],cfg.allow_unknown)){saved.reset();return {};}
    result.search_mode=flat?"FLAT":"SPATIAL";
    saved.reset();return p;
  }
};
} // namespace fire_scout
