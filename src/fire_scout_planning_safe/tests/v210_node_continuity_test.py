#!/usr/bin/env python3
"""Compile actual planner plan/publisher/completion, input gate and worker without ROS.
Stubs cover message transport only, not DDS, TF or a flight simulation.
"""
import pathlib
import subprocess
import tempfile
ROOT=pathlib.Path(__file__).resolve().parents[1]
source=(ROOT/'src/safe_airfar_like_planner.cpp').read_text()
def method(signature):
    start=source.index(signature);a=source.index('{',start);depth=1;b=a+1
    while depth:
        depth+=(source[b]=='{')-(source[b]=='}');b+=1
    return source[start:b]
start=source.index('double stableRouteCost(');end=source.index('}  // namespace',start)
helpers=source[start:end]
odom_start=source.index('[this](nav_msgs::msg::Odometry::SharedPtr m)',source.index('odom_sub_ ='))
odom_body=method(source[odom_start:source.index('{',odom_start)])
odom_body=odom_body[odom_body.index('{'):]
worker=source[source.index('    work_=std::async'):source.index('\n  void publish()')]
assert worker.endswith('\n  }') or worker.endswith('\n  }\n')
worker=worker[:worker.rfind('\n  }')]
follower=(ROOT/'src/safe_airfar_path_follower.cpp').read_text()
def follower_method(signature):
    global source
    saved=source;source=follower
    result=method(signature);source=saved
    return result
code=r'''
#include "fire_scout/entrance_routes.hpp"
#include "fire_scout/execution_permit.hpp"
#include "fire_scout/bspline.hpp"
#include "fire_scout/route_memory.hpp"
#include "fire_scout/route_switch.hpp"
#include "fire_scout/route_handoff.hpp"
#include "fire_scout/goal_approach.hpp"
#include "fire_scout/tracker.hpp"
#include "fire_scout/map_input.hpp"
#include "fire_scout/replan_schedule.hpp"
#include "fire_scout/profile_repair.hpp"
#include "fire_scout/turn_geometry.hpp"
#include "fire_scout/clearance_contract.hpp"
#include <future>
#include <atomic>
#include <cassert>
#include <memory>
#include <iostream>
#define RCLCPP_INFO_THROTTLE(...) do{}while(0)
using namespace fire_scout;
namespace builtin_interfaces {namespace msg {struct Time{int sec;unsigned nanosec;};}}
namespace rclcpp {struct Time{
  int64_t ns;Time(int64_t n=0,int=0):ns(n){}
  Time(int s,unsigned n,int):ns(int64_t(s)*1000000000LL+n){}
  Time(builtin_interfaces::msg::Time t,int):ns(int64_t(t.sec)*1000000000LL+t.nanosec){}
  int64_t nanoseconds()const{return ns;}double seconds()const{return ns*1e-9;}
  friend Time operator-(Time a,Time b){return Time(a.ns-b.ns);}
  operator builtin_interfaces::msg::Time()const{return {int(ns/1000000000LL),unsigned(ns%1000000000LL)};}
};}
struct Header{builtin_interfaces::msg::Time stamp;std::string frame_id;};
namespace nav_msgs {namespace msg {
struct Path{Header header;std::vector<Vec3>poses;};
struct Odometry{using SharedPtr=std::shared_ptr<Odometry>;Header header;
  struct Pose{struct Position{Vec3 position;}pose;}pose;
  struct Twist{struct Linear{Vec3 linear;}twist;}twist;};
}}
namespace sensor_msgs {namespace msg {struct PointCloud2 {Header header;};}}
namespace std_msgs {namespace msg {struct String{std::string data;};}}
nav_msgs::msg::Path makePath(std::vector<Vec3>p,std::string frame,rclcpp::Time t){return {{t,frame},std::move(p)};}
std_msgs::msg::String textMessage(std::string s){return {s};}
int64_t stampNs(builtin_interfaces::msg::Time t){return int64_t(t.sec)*1000000000LL+t.nanosec;}
template<class T>struct Publisher{std::vector<T>messages;void publish(const T&m){messages.push_back(m);}};
struct Clock{int get_clock_type()const{return 0;}};
'''+helpers+r'''
struct Fixture {
  MapInputSource map_source_{MapInputSource::Primary};
  LocalOccupiedOverlay local_overlay_;int64_t local_overlay_stamp_ns_{-1};
  int64_t primary_stamp_ns_{-1};
  std::shared_ptr<sensor_msgs::msg::PointCloud2>local_pending_;
  size_t primary_received_{0},local_received_{0},map_commits_{0},map_rejected_{0},primary_bytes_{0},local_bytes_{0};
  double ros{100};rclcpp::Time now()const{return rclcpp::Time(int64_t(ros*1e9));}
  std::shared_ptr<Clock>get_clock()const{return std::make_shared<Clock>();}
  std::shared_ptr<Grid> grid_{std::make_shared<Grid>(GridConfig{})};
  bool have_goal_{true},have_map_{true},have_odom_{true};
  double map_timeout_{5},odom_timeout_{1.2};
  rclcpp::Time map_stamp_{100000000000LL},odom_stamp_{100000000000LL};
  Vec3 current_{0,0,1.5},goal_{5,0,1.5};
  std::string frame_{"scout1/odom"},reason_{"KEEP_VALID_SPLINE"},detail_;
  GoalApproachConfig approach_config_;GoalApproach approach_;PlannerConfig planner_config_;SplineConfig spline_;
  TrackerConfig profile_tracker_;bool profile_repair_requested_{false};
  ClearanceContract clearance_contract_;ClearanceReview clearance_review_;
  SharedEntranceReview entrance_review_;PartialRouteRetryGate partial_retry_;
  RouteHandoffConfig handoff_config_;double measured_speed_{0};
  std::vector<Vec3>path_{{0,0,1.5},{5,0,1.5}},published_geometry_;
  int64_t route_stamp_ns_{0},execution_instance_{10};
  std::shared_ptr<Publisher<nav_msgs::msg::Path>> path_pub_{std::make_shared<Publisher<nav_msgs::msg::Path>>()},approach_pub_;
  std::shared_ptr<Publisher<std_msgs::msg::String>> execution_pub_{std::make_shared<Publisher<std_msgs::msg::String>>()},status_pub_{std::make_shared<Publisher<std_msgs::msg::String>>()};
'''+method('  bool guardValid(')+r'''
  double peer_route_separation_{1.2},peer_route_penalty_weight_{0},peer_time_step_{.3};
'''+method('  struct WorkResult {')+r''';
  std::future<WorkResult> work_;
  std::unique_ptr<Planner> planner_{std::make_unique<Planner>()};
  std::shared_ptr<std::atomic<uint64_t>>goal_epoch_{std::make_shared<std::atomic<uint64_t>>(1)};
  std::shared_ptr<Grid>occupied_guard_;
  std::vector<Vec3>owned_path_{{0,0,1.5},{5,0,1.5}};RouteMemory owned_route_,route_;
  Fixture(){owned_route_.set(owned_path_);route_.set(path_);approach_.valid=true;approach_.effective=goal_;}
  int64_t last_odom_input_stamp_ns_{0};
  void observeInputStamp(int64_t stamp,const char*,int64_t &last){last=stamp;}
  void receiveOdom(nav_msgs::msg::Odometry::SharedPtr m)
'''+odom_body+r'''
  void move(Vec3 p){
    auto m=std::make_shared<nav_msgs::msg::Odometry>();m->header={now(),frame_};
    m->pose.pose.position=p;receiveOdom(m);
  }
  RouteSwitchConfig route_switch_;EntranceConfig entrance_;
  bool force_{false},retry_{false},temporary_{false},recovery_route_{false},follower_block_reported_{false},repair_authorized_{false};
'''+method('  struct InvalidationGate {')+r''';InvalidationGate invalidation_;
  std::string last_invalidation_source_{"NONE"};double last_invalidation_distance_{0};
  Vec3 last_invalidation_point_{};
  size_t follower_stall_reports_{0},search_refreshes_{0},peer_generation_{1};
  size_t generation_{0},last_refresh_generation_{0},last_planned_peer_generation_{1};
  double search_snapshot_max_age_{8},peer_replan_min_period_{3};
  double peer_conflict_horizon_{6},peer_nominal_speed_{1.2},peer_conflict_threshold_{.12};
  double immediate_repair_distance_{1.5},near_invalidation_confirm_time_{.2},invalidation_confirm_time_{.6};
  size_t near_invalidation_confirm_updates_{2},invalidation_confirm_updates_{2};
  std::chrono::steady_clock::time_point last_plan_{},last_search_refresh_{},last_peer_replan_{},last_topology_review_{};
  std::string replan_trigger_;
  std::vector<std::vector<Vec3>> peer_inputs;
  std::vector<std::vector<Vec3>> freshPeerRoutes(bool =false)const{return peer_inputs;}
  void refreshCompactMap(){} // A separate production-body test covers map installation.
'''+method('  bool conflictsWithPeers(')+r'''
  double replan_period_{.25};std::chrono::steady_clock::time_point last_route_switch_{};
  std::shared_ptr<Publisher<nav_msgs::msg::Path>>raw_path_pub_{std::make_shared<Publisher<nav_msgs::msg::Path>>()};
'''+method('  RouteInspection inspectOwnedRoute() const')+method('  void restoreOwnedRoute()')+r'''
'''+method('  void clearInvalidation(bool restore)')+r'''
'''+method('  struct PendingRouteSwitch {')+r''';PendingRouteSwitch pending_switch_;
'''+method('  void clearPendingSwitch()')+method('  bool switchConfirmationReady(')+method('  void keepSafePrefix()')+method('  void observeInvalidation(')+method('  void complete()')+method('  void plan()')+method('  std::string inputState() const')+method('  bool ready() const')+method('  void publish() {')+r'''
  WorkResult search(bool retained=false){
    auto snapshot=grid_;const auto start=current_,goal=goal_;const auto cfg=spline_;
    const bool unknown=true;auto token=std::make_shared<std::atomic<uint64_t>>(1);const uint64_t epoch=1;
    std::vector<std::vector<Vec3>> peer_routes,full_peer_routes;
    const auto incumbent=retained?path_:std::vector<Vec3>{};
    const bool switch_bypass=!retained,safety_repair=false;
    std::string plan_trigger="ENTRANCE_REVIEW";EntranceConfig entrance_config;
    const auto pc=planner_config_;const auto committed_geometry=incumbent;const size_t peer_epoch=1;
    const auto handoff_config=handoff_config_;
    const auto profile_tracker=profile_tracker_;const auto switch_config=route_switch_;
    const auto clearance_contract=clearance_contract_;
    const double committed_progress=owned_route_.progress(),speed=measured_speed_;
    const auto handoff_prefix=retainedForwardPrefix(*snapshot,path_,start,
      route_.progress(),measured_speed_,handoff_config,unknown);
    auto p=std::make_unique<Planner>(pc);if(retained){p->plan(*snapshot,start,goal);p->setCommittedRoute(path_);}
'''+worker+r'''
    return work_.get();
  }
};
'''+r'''
struct FollowerFixture {
  enum class State{NAV};State state_{State::NAV};
  bool have_mission_goal_{true},have_path_{false},handoff_ready_{false};
  Vec3 mission_goal_{5,0,1.5},current_{0,0,1.5};
  double planner_execution_timeout_{.8},last_handoff_check_{-1};
  double ros{100},wall{1000};
  rclcpp::Time now()const{return rclcpp::Time(int64_t(ros*1e9));}
  double steadySeconds()const{return wall;}
  std::shared_ptr<Clock>get_clock()const{return std::make_shared<Clock>();}
  struct HandOff{void clear(){}}handoff_;
  struct Recovery{bool active()const{return false;}}recovery_;
  GeometricRouteChannel route_channel_;std::vector<Vec3>pending_path_;
  std::unique_ptr<Tracker>tracker_{std::make_unique<Tracker>()};rclcpp::Time path_stamp_;
'''+follower_method('  void synchronizeRoute()')+follower_method('  bool plannerAllowed()')+r'''
};
int main(){
  FollowerFixture follow;
  const std::vector<Vec3>line{{0,0,1.5},{5,0,1.5}};
  assert(follow.route_channel_.path(10,line));
  auto heartbeat=[&](double ros){return RoutePermit{int64_t(ros*1e9),10,true,"READY",{5,0,1.5},1}.encode();};
  assert(follow.route_channel_.permit(heartbeat(100),100,1000));follow.synchronizeRoute();
  assert(follow.have_path_&&follow.plannerAllowed());auto generation=follow.route_channel_.generation;
  follow.ros+=.1;follow.wall+=.1;follow.route_channel_.permit(heartbeat(follow.ros),follow.ros,follow.wall);
  follow.synchronizeRoute();assert(follow.route_channel_.generation==generation);
  assert(std::abs(follow.path_stamp_.seconds()-follow.ros)<1e-6);
  assert(!follow.tracker_->setPath(line,follow.current_));
  Fixture f;f.publish();auto revision=f.route_stamp_ns_;RoutePermit p;
  assert(RoutePermit::decode(f.execution_pub_->messages.back().data,p)&&p.ready);
  f.ros+=.1;f.publish();assert(f.route_stamp_ns_==revision);
  f.ros+=6;f.publish();
  assert(f.path_pub_->messages.back().poses.size()==2&&f.route_stamp_ns_==revision);
  assert(RoutePermit::decode(f.execution_pub_->messages.back().data,p)&&!p.ready&&p.reason=="STALE_MAP");
  f.map_stamp_=f.odom_stamp_=f.now();f.publish();
  assert(f.route_stamp_ns_==revision);assert(RoutePermit::decode(f.execution_pub_->messages.back().data,p)&&p.ready);
  f.occupied_guard_=std::make_shared<Grid>();f.occupied_guard_->update({f.grid_->toKey({3,0,1.5})},{});
  f.publish();assert(RoutePermit::decode(f.execution_pub_->messages.back().data,p)&&p.ready);
  assert(f.path_.back().x<3 && f.path_.back().x>1);
  for(size_t i=1;i<f.path_.size();++i)assert(f.occupied_guard_->segmentFrom(f.path_[i-1],f.path_[i],true));
  f.occupied_guard_.reset();f.path_={{0,0,1.5},{5,.1,1.5}};f.route_.set(f.path_);
  f.publish();assert(f.route_stamp_ns_>revision);
  f.path_.clear();f.route_.clear();f.publish();assert(RoutePermit::decode(f.execution_pub_->messages.back().data,p)&&p.reason=="NO_ROUTE");
  f.path_={{0,0,1.5},{5,0,1.5}};f.route_.set(f.path_);
  auto result=f.search();assert(result.smooth.points.size()>1);
  assert(result.handoff.applied && result.handoff.reason=="PREFIX_ALREADY_SHARED");
  f.goal_={5,1,1.5};auto changed=f.search();
  assert(changed.handoff.applied&&changed.handoff.reason=="PREFIX_BEZIER_JOIN");
  assert(!changed.switch_points.empty()&&routeGeometryValid(*f.grid_,changed.smooth.points,true));
  assert(distance(changed.smooth.points.back(),f.goal_)<1e-6);f.goal_={5,0,1.5};
  auto old=f.search(true);assert(old.smooth.mode=="RETAINED_STRAIGHT_ROUTE");
  assert(old.smooth.points.size()==f.path_.size()&&distance(old.smooth.points.back(),f.path_.back())<1e-9);
  // Exercise the actual completion/confirmation path, not just selection.
  Fixture coordinator;coordinator.owned_route_.set(coordinator.owned_path_);
  auto commit=[&](std::vector<Vec3> points,std::string reason){
    Fixture::WorkResult result;result.planner=std::make_unique<Planner>();result.epoch=1;
    result.smooth.points=std::move(points);result.route.points=result.smooth.points;
    result.route.reason=reason;result.peer_epoch=1;
    result.reserved_gates={{{2.5,0,1.5},{1,0,0},1.5}};
    result.trigger="DISTINCT_PASSAGE_ROUTE";
    std::promise<Fixture::WorkResult>ready;coordinator.work_=ready.get_future();ready.set_value(std::move(result));
    coordinator.complete();
  };
  std::vector<Vec3> alternate{{0,0,1.5},{2,2,1.5},{5,0,1.5}};
  commit(alternate,"DISTINCT_PASSAGE_ROUTE");assert(coordinator.pending_switch_.active);
  coordinator.pending_switch_.since-=std::chrono::seconds(1);
  commit(alternate,"DISTINCT_PASSAGE_ROUTE");assert(coordinator.pending_switch_.active);
  commit(alternate,"DISTINCT_PASSAGE_ROUTE");
  assert(pathLength(coordinator.path_)>5 && coordinator.detail_.find("COMMIT_DISTINCT_PASSAGE")!=std::string::npos);
  commit({{0,0,1.5},{5,0,1.5}},"ROUTE_REFRESH");
  assert(pathLength(coordinator.path_)>5 && coordinator.detail_.find("KEEP_COMMITTED_ENTRANCE")!=std::string::npos);
  // Matching the old forward prefix must not hide a 90-degree change in the
  // proposal from the existing heading/cost hysteresis.
  Fixture policy;
  auto proposal=[&]{
    Fixture::WorkResult r;r.planner=std::make_unique<Planner>();r.epoch=1;r.peer_epoch=1;
    r.switch_points={{0,0,1.5},{0,2,1.5},{5,0,1.5}};
    r.smooth.points={{0,0,1.5},{1,0,1.5},{2,2,1.5},{5,0,1.5}};
    r.route.points=r.switch_points;r.route.reason="ROUTE_REFRESH";
    r.handoff.applied=true;r.handoff.reason="PREFIX_BEZIER_JOIN";r.handoff.prefix_length=1.;
    return r;
  };
  std::promise<Fixture::WorkResult> policy_result;policy.work_=policy_result.get_future();
  policy_result.set_value(proposal());policy.complete();
  assert(policy.reason_=="KEEP_ROUTE_SWITCH_COST"&&policy.detail_.find("switch_angle_deg=90")!=std::string::npos);
  assert(std::abs(pathLength(policy.path_)-5)<1e-8);
  // A result safe in the worker snapshot is recertified on the new map.
  policy.grid_->update({policy.grid_->toKey({2,2,1.5})},{});
  std::promise<Fixture::WorkResult> outdated;policy.work_=outdated.get_future();
  outdated.set_value(proposal());policy.complete();policy.publish();
  assert(policy.reason_=="KEEP_ROUTE_CANDIDATE_INVALID");
  assert(RoutePermit::decode(policy.execution_pub_->messages.back().data,p)&&p.ready);
  // Logged regression: the aircraft moves metres along one retained curve.
  // Exercise the real odometry callback and publisher together; obstacles are
  // deliberately absent, so losing the permit can only be a progress bug.
  Fixture moving;moving.goal_={12,0,1.5};moving.owned_path_=moving.path_={{0,0,1.5},{12,0,1.5}};
  moving.owned_route_.set(moving.owned_path_);moving.route_.set(moving.path_);
  moving.publish();const auto moving_revision=moving.route_stamp_ns_;
  for(int n=1;n<=20;++n){
    moving.ros+=.1;moving.map_stamp_=moving.now();moving.move({n*.4,0,1.5});moving.publish();
    assert(RoutePermit::decode(moving.execution_pub_->messages.back().data,p)&&p.ready);
    assert(moving.route_stamp_ns_==moving_revision);
  }
  assert(moving.route_.progress()>7.9&&moving.owned_route_.progress()>7.9);
  // An unchanged worker result must carry the incumbent's progress too.
  Fixture::WorkResult retained;retained.planner=std::make_unique<Planner>();retained.epoch=1;
  retained.smooth.points=moving.owned_path_;retained.smooth.mode="RETAINED_STRAIGHT_ROUTE";
  retained.route.points=retained.smooth.points;retained.route.reused=true;retained.route.reason="KEEP_VALID_ROUTE";
  retained.peer_epoch=1;
  std::promise<Fixture::WorkResult>completed;moving.work_=completed.get_future();completed.set_value(std::move(retained));
  moving.complete();
  assert(moving.detail_.find("spline_candidate_valid=1")!=std::string::npos);
  assert(!moving.raw_path_pub_->messages.back().poses.empty());
  moving.publish();assert(RoutePermit::decode(moving.execution_pub_->messages.back().data,p)&&p.ready);
  // Also cover installing exact incumbent geometry with hysteresis disabled:
  // both ownership and execution must retain the same non-zero arc origin.
  moving.owned_route_.set({{0,0,1.5},{12,0,1.5}});
  for(int n=0;n<=20;++n)moving.owned_route_.updateProgress({n*.4,0,1.5});
  moving.owned_path_=moving.path_=moving.owned_route_.points();moving.route_=moving.owned_route_;
  moving.publish();const auto retained_revision=moving.route_stamp_ns_;
  moving.route_switch_.enabled=false;
  Fixture::WorkResult recommit;recommit.planner=std::make_unique<Planner>();recommit.epoch=1;
  recommit.smooth.points=moving.owned_path_;recommit.smooth.mode="RETAINED_STRAIGHT_ROUTE";
  recommit.route.points=recommit.smooth.points;recommit.route.reason="KEEP_VALID_ROUTE";recommit.route.reused=true;
  recommit.peer_epoch=1;
  std::promise<Fixture::WorkResult>same;moving.work_=same.get_future();same.set_value(std::move(recommit));
  moving.complete();
  assert(moving.detail_.find("route_committed=1")!=std::string::npos);
  assert(moving.route_.progress()>7.9&&moving.owned_route_.progress()>7.9);
  moving.publish();assert(moving.route_stamp_ns_==retained_revision);
  assert(RoutePermit::decode(moving.execution_pub_->messages.back().data,p)&&p.ready);
  moving.ros+=.1;moving.map_stamp_=moving.now();moving.move({8.4,0,1.5});moving.publish();
  assert(RoutePermit::decode(moving.execution_pub_->messages.back().data,p)&&p.ready);
  // Past geometry is not a future collision. A new obstacle behind the
  // aircraft must not revoke its forward suffix solely because it was retained.
  moving.occupied_guard_=std::make_shared<Grid>();moving.occupied_guard_->update({moving.grid_->toKey({2,0,1.5})},{});
  moving.publish();assert(RoutePermit::decode(moving.execution_pub_->messages.back().data,p)&&p.ready);
  // Seeding progress is not permission to skip a blocked future segment.
  moving.occupied_guard_=std::make_shared<Grid>();moving.occupied_guard_->update({moving.grid_->toKey({10,0,1.5})},{});
  moving.publish();
  assert(RoutePermit::decode(moving.execution_pub_->messages.back().data,p)&&p.ready);
  assert(moving.path_.back().x<10 && moving.path_.back().x>8.4);
  const auto prefix_revision=moving.route_stamp_ns_;moving.publish();assert(moving.route_stamp_ns_==prefix_revision);
  // Recovery before invalidation confirmation restores the complete old tail.
  moving.occupied_guard_.reset();moving.clearInvalidation(true);moving.publish();
  assert(moving.path_.back().x==12 && RoutePermit::decode(moving.execution_pub_->messages.back().data,p)&&p.ready);
  // A newly occupied body/near corridor cannot be authorized by retention.
  moving.occupied_guard_=std::make_shared<Grid>();moving.occupied_guard_->update({moving.grid_->toKey(moving.current_)},{});
  moving.publish();assert(RoutePermit::decode(moving.execution_pub_->messages.back().data,p)&&!p.ready);
  moving.occupied_guard_.reset();moving.ros+=6;moving.publish();
  assert(RoutePermit::decode(moving.execution_pub_->messages.back().data,p)&&!p.ready&&p.reason=="STALE_MAP");
  // Real restore/complete + strict tracker regression: a 1 cm odometry error
  // used to turn a dense straight path into a .041 m/s yaw-limited microcorner.
  TrackerConfig tc;tc.turn.enabled=false;tc.max_speed_xy=1.8;tc.yaw_rate=1.2;tc.lateral_accel=1.;
  tc.max_jerk_xy=4.5;tc.sharp_turn=80*pi/180;
  for(double offset:{.01,.03,.05,.10}){
    Fixture dense;dense.goal_={20,0,1.5};dense.owned_path_.clear();
    for(int n=0;n<=1000;++n)dense.owned_path_.push_back({n*.02,0,1.5});
    dense.path_=dense.owned_path_;dense.owned_route_.set(dense.owned_path_);dense.route_.set(dense.path_);
    Tracker track(tc);track.initializeYaw(0);track.setPath(dense.path_,{0,offset,1.5});
    for(int n=0;n<=20;++n){
      dense.move({n*.4,offset,1.5});
      track.update(dense.current_,{},.025,[](Vec3,Vec3){return true;},0,1.8,{},HeadingMode::Strict);
    }
    dense.publish();const auto revision=dense.route_stamp_ns_;
    for(int n=0;n<4;++n){
      Fixture::WorkResult rejected;rejected.planner=std::make_unique<Planner>();rejected.epoch=1;
      rejected.smooth.points=dense.owned_path_;rejected.smooth.mode="RETAINED_STRAIGHT_ROUTE";
      rejected.route.points=rejected.smooth.points;rejected.route.reused=true;
      std::promise<Fixture::WorkResult> done;dense.work_=done.get_future();done.set_value(std::move(rejected));
      dense.complete();dense.publish();
      assert(dense.route_stamp_ns_==revision && dense.route_.progress()>7.9);
      assert(!track.setPath(dense.path_,dense.current_));
      auto control=track.update(dense.current_,{},.025,[](Vec3,Vec3){return true;},0,1.8,{},HeadingMode::Strict);
      assert(control.speed_limit>1.79);
    }
    // A real unsafe tail can be clipped and restored beyond 8 m without a
    // fake connector, lost projection or an artificial speed restriction.
    dense.occupied_guard_=std::make_shared<Grid>();
    dense.occupied_guard_->update({dense.grid_->toKey({14,0,1.5})},{});
    dense.keepSafePrefix();assert(dense.path_.back().x<14);
    track.setPath(dense.path_,dense.current_);
    auto clipped=track.update(dense.current_,{},.025,[](Vec3,Vec3){return true;},0,1.8,{},HeadingMode::Strict);
    assert(clipped.speed_limit>1.79);
    dense.occupied_guard_.reset();dense.restoreOwnedRoute();
    assert(dense.path_.back().x==20);
    track.setPath(dense.path_,dense.current_);
    auto restored=track.update(dense.current_,{},.025,[](Vec3,Vec3){return true;},0,1.8,{},HeadingMode::Strict);
    assert(restored.speed_limit>1.79);
  }
  // Compile and execute the ENTIRE production plan() control flow. A peer
  // route in open space is not evidence for a shared entrance or another A*.
  Fixture quiet;quiet.publish();const auto quiet_revision=quiet.route_stamp_ns_;
  quiet.plan();assert(!quiet.work_.valid()&&quiet.reason_=="KEEP_VALID_SPLINE");
  quiet.peer_inputs={{{0,3,1.5},{5,3,1.5}}};
  for(int n=0;n<12;++n){
    quiet.last_plan_={};quiet.plan();quiet.publish();
    assert(!quiet.work_.valid()&&quiet.reason_=="KEEP_VALID_SPLINE");
    assert(quiet.route_stamp_ns_==quiet_revision);
  }
  const auto set_owned=[](Fixture &node,std::vector<Vec3> points,Vec3 goal){
    node.owned_path_=node.path_=std::move(points);node.goal_=goal;
    node.route_.set(node.path_);node.owned_route_.set(node.owned_path_);
    node.approach_.valid=true;node.approach_.effective=goal;
    node.planner_->setCommittedRoute(node.owned_path_);
  };
  // With valid-route locking disabled, execute actual plan->worker->complete.
  // A removed obstacle can release a detour without global cooldown/gain gates.
  const std::vector<Vec3> detour{{0,0,1.5},{1.5,0,1.5},{3,.5,1.5},{4,.5,1.5},{5,0,1.5},{8,0,1.5}};
  Fixture refresh;set_owned(refresh,detour,detour.back());
  refresh.planner_config_.lock_valid_route=false;refresh.route_switch_.cooldown=100;
  refresh.profile_tracker_.turn.enabled=true;refresh.entrance_.topology.enabled=false;
  refresh.spline_.optimization_budget_ms=30;refresh.handoff_config_.budget_ms=50;
  refresh.publish();const double detour_length=pathLength(refresh.path_);
  refresh.plan();assert(refresh.work_.valid());refresh.work_.wait();refresh.complete();
  assert(refresh.detail_.find("switch_trigger=ROUTE_LOCAL_REFRESH")!=std::string::npos);
  assert(refresh.detail_.find("switch_decision=COMMIT_LOCAL_REFRESH")!=std::string::npos);
  assert(pathLength(refresh.path_)<detour_length-.10&&routeGeometryValid(*refresh.grid_,refresh.path_));
  assert(project(refresh.path_,arcLengths(refresh.path_),{.8,0,1.5}).error<1e-9);
  // A peer revision that arrives while work runs still rejects local changes.
  Fixture changed_peer;set_owned(changed_peer,detour,detour.back());
  changed_peer.planner_config_.lock_valid_route=false;changed_peer.entrance_.topology.enabled=false;
  changed_peer.plan();assert(changed_peer.work_.valid());changed_peer.work_.wait();++changed_peer.peer_generation_;
  changed_peer.complete();assert(std::abs(pathLength(changed_peer.path_)-detour_length)<1e-8);
  // The real YAML's accelerated clearance schedule is a valid configuration.
  ClearanceContract fast;fast.review_period=.10;fast.retry_period=.30;assert(fast.valid());
  // A spurious low-profile notification performs one bounded no-op repair and
  // leaves the healthy route's revision, geometry and progress untouched.
  Fixture healthy;set_owned(healthy,densify({{0,0,1.5},{6,0,1.5}},.02),{6,0,1.5});
  healthy.profile_tracker_=tc;healthy.profile_repair_requested_=true;healthy.force_=true;
  healthy.publish();const auto healthy_revision=healthy.route_stamp_ns_;
  healthy.plan();assert(healthy.work_.valid()&&!healthy.profile_repair_requested_);
  healthy.work_.wait();healthy.complete();healthy.publish();
  assert(healthy.detail_.find("switch_trigger=PROFILE_LIMITED_PROGRESS")!=std::string::npos);
  assert(healthy.detail_.find("profile_repair=PROFILE_REPAIR_NOT_NEEDED")!=std::string::npos);
  assert(healthy.route_stamp_ns_==healthy_revision&&healthy.planner_);
  healthy.last_plan_={};healthy.plan();assert(!healthy.work_.valid());
  // Reproduce the real dense dogleg restriction through plan()->worker->
  // complete(), proving the source integration produces a certified speed
  // improvement rather than merely testing the standalone helper.
  const auto dogleg=densify({{0,0,1.5},{.32,0,1.5},{.34,.01,1.5},{.36,0,1.5},{6,0,1.5}},.02);
  GridConfig small;small.resolution=.05;small.inflation_xy=small.inflation_z=.15;
  Fixture profile;profile.grid_=std::make_shared<Grid>(small);set_owned(profile,dogleg,{6,0,1.5});
  profile.profile_tracker_=tc;profile.handoff_config_.budget_ms=50;
  profile.profile_repair_requested_=true;profile.force_=true;profile.publish();
  const auto profile_revision=profile.route_stamp_ns_;
  const double before=alignedRouteProfileLimit(profile.path_,profile.current_,{},0,tc);assert(before<.10);
  profile.plan();assert(profile.work_.valid());profile.work_.wait();profile.complete();profile.publish();
  assert(profile.detail_.find("switch_decision=COMMIT_PROFILE_LOCAL_REPAIR")!=std::string::npos);
  assert(profile.detail_.find("route_committed=1")!=std::string::npos);
  assert(profile.route_stamp_ns_>profile_revision&&routeGeometryValid(*profile.grid_,profile.path_,true));
  const double after=alignedRouteProfileLimit(profile.path_,profile.current_,{},0,tc);assert(after>.8);
  const auto dogleg_arc=arcLengths(dogleg),profile_arc=arcLengths(profile.path_);
  for(double s=0;s<=.30;s+=.01)assert(distance(atArc(dogleg,dogleg_arc,s),atArc(profile.path_,profile_arc,s))<1e-9);

  // TURN_* repair reaches the actual planner worker rather than silently
  // retaining a map-valid but poorly executable near stitch.
  Fixture turning;turning.grid_=std::make_shared<Grid>(small);
  const auto near_turn=densify({{0,0,1.5},{.10,0,1.5},{.12,.015,1.5},{.14,0,1.5},{6,0,1.5}},.015);
  set_owned(turning,near_turn,{6,0,1.5});
  turning.profile_tracker_.sharp_turn=80*pi/180;turning.handoff_config_.budget_ms=50;
  turning.profile_repair_requested_=true;turning.replan_trigger_="TURN_EXECUTION_REPAIR";
  turning.force_=true;turning.plan();assert(turning.work_.valid());
  turning.work_.wait();turning.complete();
  assert(turning.detail_.find("switch_trigger=TURN_EXECUTION_REPAIR")!=std::string::npos);
  assert(turning.detail_.find("route_committed=1")!=std::string::npos);
  assert(routeGeometryValid(*turning.grid_,turning.path_));
  assert(alignedRouteProfileLimit(turning.path_,turning.current_,{},0,turning.profile_tracker_)>.8);
  // A peer-reservation revision during that same worker must invalidate its
  // authority to commit, while preserving the old certified route.
  Fixture peer_changed;peer_changed.grid_=std::make_shared<Grid>(small);
  set_owned(peer_changed,dogleg,{6,0,1.5});peer_changed.profile_tracker_=tc;
  peer_changed.handoff_config_.budget_ms=50;peer_changed.profile_repair_requested_=true;
  peer_changed.plan();assert(peer_changed.work_.valid());++peer_changed.peer_generation_;
  peer_changed.work_.wait();peer_changed.complete();
  assert(peer_changed.reason_=="KEEP_ROUTE_PROFILE_POLICY");
  assert(peer_changed.path_.size()==dogleg.size());
  for(size_t i=0;i<dogleg.size();++i)assert(distance(peer_changed.path_[i],dogleg[i])<1e-9);
  // Feed a real rejected partial-tail proposal through complete(). Its delay
  // prevents repeated searches at the same endpoint; useful progress releases
  // the gate immediately, without waiting or stopping at the prefix endpoint.
  Fixture partial;set_owned(partial,{{0,0,1.5},{1.5,0,1.5}},{6,0,1.5});partial.temporary_=true;
  Fixture::WorkResult partial_result;partial_result.epoch=1;partial_result.peer_epoch=1;
  partial_result.planner=std::make_unique<Planner>();partial_result.switch_bypass=true;
  partial_result.trigger="PARTIAL_ROUTE_FINISHED";partial_result.route.reason="ROUTE_REFRESH";
  partial_result.smooth.points=partial_result.route.points=partial_result.switch_points={{0,0,1.5},{6,0,1.5}};
  std::promise<Fixture::WorkResult> no_join;partial.work_=no_join.get_future();no_join.set_value(std::move(partial_result));
  partial.complete();assert(partial.reason_=="KEEP_ROUTE_NO_SAFE_HANDOFF");
  partial.last_plan_={};partial.plan();
  assert(!partial.work_.valid()&&partial.reason_=="KEEP_PARTIAL_ROUTE_RETRY_DELAY");
  partial.move({.3,0,1.5});partial.last_plan_={};partial.plan();assert(partial.work_.valid());
  auto resumed=partial.work_.get();assert(resumed.trigger=="PARTIAL_ROUTE_FINISHED");
  assert(resumed.handoff.applied&&!resumed.smooth.points.empty());
  // Real production plan/worker/complete: hard-valid wall-skimming route must
  // be repaired after TWO distinct maps, then retained without search churn.
  GridConfig wall_cfg;wall_cfg.resolution=.1;wall_cfg.inflation_xy=.469063;
  wall_cfg.inflation_z=.27;wall_cfg.z_max=2;
  Fixture clearance;clearance.grid_=std::make_shared<Grid>(wall_cfg);
  KeySet wall;for(int x=30;x<40;++x)for(int y=6;y<14;++y)
    for(int z=10;z<21;++z)wall.insert({x,y,z});
  clearance.grid_->update(wall,{});
  set_owned(clearance,{{0,0,1.5},{8,0,1.5}},{8,0,1.5});
  clearance.planner_config_.clearance_weight=7;
  clearance.planner_config_.repair_search_ms=200;
  clearance.handoff_config_.budget_ms=50;
  clearance.plan();assert(!clearance.work_.valid());
  clearance.ros+=.4;clearance.map_stamp_=clearance.odom_stamp_=clearance.now();
  clearance.last_plan_={};clearance.plan();assert(!clearance.work_.valid());
  clearance.ros+=.4;++clearance.generation_;
  clearance.map_stamp_=clearance.odom_stamp_=clearance.now();clearance.last_plan_={};
  clearance.plan();assert(clearance.work_.valid());
  clearance.work_.wait();clearance.complete();
  if(clearance.detail_.find("COMMIT_CLEARANCE_LOCAL_REPAIR")==std::string::npos)
    std::cerr<<clearance.detail_<<"\n";
  assert(clearance.detail_.find("COMMIT_CLEARANCE_LOCAL_REPAIR")!=std::string::npos);
  assert(routeHasMargin(*clearance.grid_,clearance.path_,.245));
  clearance.publish();const auto clearance_revision=clearance.route_stamp_ns_;
  clearance.last_plan_={};clearance.plan();clearance.publish();
  assert(!clearance.work_.valid()&&clearance.route_stamp_ns_==clearance_revision);
  // Map changes during search: both curves still satisfy the HARD body, but
  // the new smoother cuts away reserve available on its raw reference.
  Fixture late_margin;late_margin.grid_=std::make_shared<Grid>(wall_cfg);
  late_margin.grid_->update(wall,{});late_margin.path_.clear();late_margin.owned_path_.clear();
  late_margin.route_.clear();late_margin.owned_route_.clear();late_margin.goal_={8,0,1.5};
  late_margin.approach_.effective=late_margin.goal_;
  Fixture::WorkResult late;late.epoch=1;late.peer_epoch=1;late.switch_bypass=true;
  late.planner=std::make_unique<Planner>();late.trigger="ROUTE_REFRESH";
  late.route.points={{0,0,1.5},{1,-.5,1.5},{7,-.5,1.5},{8,0,1.5}};
  late.smooth.points={{0,0,1.5},{8,0,1.5}};
  assert(routeGeometryValid(*late_margin.grid_,late.smooth.points));
  std::promise<Fixture::WorkResult> late_promise;late_margin.work_=late_promise.get_future();
  late_promise.set_value(std::move(late));late_margin.complete();
  assert(late_margin.owned_path_.empty());
  assert(late_margin.detail_.find("candidate_margin_valid=0")!=std::string::npos);
  // Already close to a wall: the current point itself cannot gain clearance
  // instantaneously. Judge the future deficit, not a global minimum pinned by
  // the preserved current prefix. Execute the actual worker/commit policy.
  Fixture near_wall;near_wall.grid_=std::make_shared<Grid>(wall_cfg);
  KeySet near_obstacle;for(int x=-10;x<40;++x)for(int z=10;z<21;++z)near_obstacle.insert({x,6,z});
  near_wall.grid_->update(near_obstacle,{});
  set_owned(near_wall,densify({{0,0,1.5},{8,0,1.5}},.05),{8,0,1.5});
  near_wall.planner_config_.clearance_weight=7;near_wall.measured_speed_=.02;
  near_wall.plan();assert(!near_wall.work_.valid());
  near_wall.ros+=.4;++near_wall.generation_;
  near_wall.map_stamp_=near_wall.odom_stamp_=near_wall.now();near_wall.last_plan_={};
  near_wall.plan();assert(near_wall.work_.valid());near_wall.work_.wait();near_wall.complete();
  if(near_wall.detail_.find("COMMIT_CLEARANCE_LOCAL_REPAIR")==std::string::npos)std::cerr<<near_wall.detail_<<"\n";
  assert(near_wall.detail_.find("COMMIT_CLEARANCE_LOCAL_REPAIR")!=std::string::npos);
  assert(near_wall.detail_.find("clearance_near_repair=1")!=std::string::npos);
  assert(routeGeometryValid(*near_wall.grid_,near_wall.path_));
  std::cout<<"v210_node_continuity_test: PASS actual plan/odometry/publisher/worker/completion; retained progress beyond 8m, dense-offset speed, no blind entrance search, certified profile repair, peer revision rejection, partial retry/resume, stale-map denial\n";
}
'''
with tempfile.TemporaryDirectory(prefix='fire_scout_continuity_') as temp:
    p=pathlib.Path(temp);(p/'test.cpp').write_text(code)
    subprocess.run(['g++','-std=c++17','-O2','-Wall','-Wextra','-Wpedantic','-UNDEBUG','-I',str(ROOT/'include'),str(p/'test.cpp'),'-pthread','-o',str(p/'test')],check=True)
    subprocess.run([str(p/'test')],check=True)
