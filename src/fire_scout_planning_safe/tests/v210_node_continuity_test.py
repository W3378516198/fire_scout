#!/usr/bin/env python3
"""Compile actual planner publisher, input gate and worker body without ROS.
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
#include "fire_scout/goal_approach.hpp"
#include "fire_scout/tracker.hpp"
#include "fire_scout/map_input.hpp"
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
  struct Pose{struct Position{Vec3 position;}pose;}pose;};
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
  GoalApproach approach_;PlannerConfig planner_config_;SplineConfig spline_;
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
  Fixture(){owned_route_.set(owned_path_);route_.set(path_);}
  int64_t last_odom_input_stamp_ns_{0};
  void observeInputStamp(int64_t stamp,const char*,int64_t &last){last=stamp;}
  void receiveOdom(nav_msgs::msg::Odometry::SharedPtr m)
'''+odom_body+r'''
  void move(Vec3 p){
    auto m=std::make_shared<nav_msgs::msg::Odometry>();m->header={now(),frame_};
    m->pose.pose.position=p;receiveOdom(m);
  }
  RouteSwitchConfig route_switch_;EntranceConfig entrance_;
  bool force_{false},retry_{false},temporary_{false},recovery_route_{false},follower_block_reported_{false};
  struct Gate{bool active{false},authorized{false};double distance{0};int matched_updates{0};
    std::string source{"NONE"};Vec3 point{};}invalidation_;
  std::string last_invalidation_source_{"NONE"};double last_invalidation_distance_{0};
  Vec3 last_invalidation_point_{};
  size_t follower_stall_reports_{0},search_refreshes_{0},peer_generation_{1};
  double replan_period_{.25};std::chrono::steady_clock::time_point last_route_switch_{};
  std::shared_ptr<Publisher<nav_msgs::msg::Path>>raw_path_pub_{std::make_shared<Publisher<nav_msgs::msg::Path>>()};
'''+method('  RouteInspection inspectOwnedRoute() const')+method('  void restoreOwnedRoute()')+r'''
  void clearInvalidation(bool){invalidation_={};}
'''+method('  struct PendingRouteSwitch {')+r''';PendingRouteSwitch pending_switch_;
'''+method('  void clearPendingSwitch()')+method('  bool switchConfirmationReady(')+method('  void keepSafePrefix()')+method('  void complete()')+method('  std::string inputState() const')+method('  bool ready() const')+method('  void publish() {')+r'''
  WorkResult search(bool retained=false){
    auto snapshot=grid_;const auto start=current_,goal=goal_;const auto cfg=spline_;
    const bool unknown=true;auto token=std::make_shared<std::atomic<uint64_t>>(1);const uint64_t epoch=1;
    std::vector<std::vector<Vec3>> peer_routes,full_peer_routes;
    const auto incumbent=retained?path_:std::vector<Vec3>{};
    const bool switch_bypass=!retained,safety_repair=false;
    std::string plan_trigger="ENTRANCE_REVIEW";EntranceConfig entrance_config;
    const auto pc=planner_config_;const auto committed_geometry=incumbent;const size_t peer_epoch=1;
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
  f.publish();assert(RoutePermit::decode(f.execution_pub_->messages.back().data,p)&&!p.ready&&p.reason=="ROUTE_NOT_CERTIFIED");
  f.occupied_guard_.reset();f.path_={{0,0,1.5},{5,.1,1.5}};f.route_.set(f.path_);
  f.publish();assert(f.route_stamp_ns_>revision);
  f.path_.clear();f.route_.clear();f.publish();assert(RoutePermit::decode(f.execution_pub_->messages.back().data,p)&&p.reason=="NO_ROUTE");
  f.path_={{0,0,1.5},{5,0,1.5}};f.route_.set(f.path_);
  auto result=f.search();assert(result.smooth.points.size()>1);
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
  assert(RoutePermit::decode(moving.execution_pub_->messages.back().data,p)&&!p.ready);
  moving.occupied_guard_.reset();moving.ros+=6;moving.publish();
  assert(RoutePermit::decode(moving.execution_pub_->messages.back().data,p)&&!p.ready&&p.reason=="STALE_MAP");
  std::cout<<"v210_node_continuity_test: PASS actual odometry/publisher/worker/completion; retained progress beyond 8m, fixed revision, reused geometry, past/future obstacles, stale-map denial\n";
}
'''
with tempfile.TemporaryDirectory(prefix='fire_scout_continuity_') as temp:
    p=pathlib.Path(temp);(p/'test.cpp').write_text(code)
    subprocess.run(['g++','-std=c++17','-O2','-Wall','-Wextra','-Wpedantic','-UNDEBUG','-I',str(ROOT/'include'),str(p/'test.cpp'),'-pthread','-o',str(p/'test')],check=True)
    subprocess.run([str(p/'test')],check=True)
