#!/usr/bin/env python3
"""Exercise actual node passage methods with small transport/clock/TF stubs.

These stubs test the two endpoints together; they do not emulate ROS or DDS.
Production methods are extracted verbatim, so message count, stamps, epoch
checks and release/acknowledgment paths are covered, not just helper classes.
"""
import pathlib
import subprocess
import tempfile

ROOT = pathlib.Path(__file__).resolve().parents[1]


def method(filename, signature):
    source = (ROOT / "src" / filename).read_text()
    start = source.index(signature)
    begin = source.index("{", start)
    depth = 1
    cursor = begin + 1
    while depth:
        depth += (source[cursor] == "{") - (source[cursor] == "}")
        cursor += 1
    return source[start:cursor]


code = r'''
#include "fire_scout/passage_scheduler.hpp"
#include "fire_scout/tracker.hpp"
#include "fire_scout/route_topology.hpp"
#include <cassert>
#include <iostream>
#include <memory>
#include <sstream>
using namespace fire_scout;
namespace builtin_interfaces { namespace msg {struct Time{int32_t sec{0};uint32_t nanosec{0};};}}
namespace rclcpp {struct Time{
  int64_t value{0};Time(int64_t n=0,int=0):value(n){}
  Time(int s,unsigned n,int):value(int64_t(s)*1000000000LL+n){}
  double seconds()const{return value*1e-9;}int64_t nanoseconds()const{return value;}
  friend Time operator-(Time a,Time b){return Time(a.value-b.value);}
  operator builtin_interfaces::msg::Time()const{return {int32_t(value/1000000000LL),uint32_t(value%1000000000LL)};}
};}
struct Header{builtin_interfaces::msg::Time stamp;std::string frame_id;};
struct Quaternion{double x{0},y{0},z{0},w{1};};
struct Pose{Vec3 position{};Quaternion orientation{};};
struct PoseStamped{Header header;Pose pose;};
namespace nav_msgs {namespace msg {struct Path{Header header;std::vector<PoseStamped>poses;};}}
namespace std_msgs {namespace msg {struct String{std::string data;};}}
int64_t stampNs(builtin_interfaces::msg::Time s){return int64_t(s.sec)*1000000000LL+s.nanosec;}
std_msgs::msg::String textMessage(std::string s){return {s};}
nav_msgs::msg::Path makePath(const std::vector<Vec3>&p,std::string frame,rclcpp::Time t){
  nav_msgs::msg::Path m;m.header={t,frame};for(Vec3 v:p)m.poses.push_back({m.header,{v,{}}});return m;
}
template<class T>struct Publisher{std::vector<T>messages;void publish(const T&m){messages.push_back(m);}};
struct Clock{int get_clock_type()const{return 0;}};
struct NodeClock{double ros{100};rclcpp::Time now()const{return rclcpp::Time(int64_t(std::llround(ros*1e9)));}
  std::shared_ptr<Clock>get_clock()const{return std::make_shared<Clock>();}};
namespace tf2 {
struct TransformException:std::runtime_error{using std::runtime_error::runtime_error;};
struct Vector3{Vec3 v;Vector3(double x,double y,double z):v{x,y,z}{}
  double x()const{return v.x;}double y()const{return v.y;}double z()const{return v.z;}};
struct Quaternion{Quaternion(double,double,double,double){}};
struct Transform{Vec3 translation{};void setIdentity(){translation={};}void setRotation(Quaternion){}
  void setOrigin(Vector3 p){translation=p.v;}Vector3 operator*(Vector3 p)const{auto q=p.v+translation;return {q.x,q.y,q.z};}};
}
struct TFBuffer{struct Result{struct Content{Vec3 translation{-12.5,-3,.2};Quaternion rotation{};}transform;};
  Result lookupTransform(const std::string&,const std::string&,rclcpp::Time)const{return {};}};
struct Follower:NodeClock {
  enum class State{NAV};State state_{State::NAV};std::string frame_{"scout1/odom"};
  std::unique_ptr<Tracker>tracker_{std::make_unique<Tracker>()};
  std::unique_ptr<Grid>grid_{std::make_unique<Grid>(GridConfig{})};
  struct Recovery{bool active()const{return false;}}recovery_;
  bool have_path_{true},have_mission_goal_{true},passage_scheduler_enabled_{true};
  bool have_odom_{true};rclcpp::Time odom_stamp_{100000000000LL};double odom_timeout_{1.2};
  std::vector<Vec3>pending_path_;bool plannerAllowed()const{return true;}
  bool portal_active_{false},portal_waiting_{false},passage_granted_{false};
  Vec3 current_{},portal_anchor_{},portal_direction_{1,0,0};
  double passage_exit_distance_{2.2},passage_wait_distance_{2},passage_lease_timeout_{.8};
  double passage_grant_time_{-10},portal_epoch_{0},last_portal_request_{-10};
  double passage_width_{2.6},passage_lookahead_{5},passage_detect_period_{.25},last_portal_detect_{-1};
  int64_t portal_id_{0};double passage_reply_source_{-1};size_t passage_completions_{0};
  std::string passage_reply_state_{"IDLE"};
  PassageProgress portal_progress_;PassageDepartureMemory portal_departure_;PassageWaitMonitor passage_wait_monitor_;
  std::shared_ptr<Publisher<nav_msgs::msg::Path>>passage_request_pub_{std::make_shared<Publisher<nav_msgs::msg::Path>>()};
  Vec3 navigationGoal()const{return {100,100,1.5};}
  void start(Vec3 a,Vec3 d){portal_anchor_=a;portal_direction_=d;portal_active_=true;portal_waiting_=true;
    passage_granted_=false;portal_epoch_=now().seconds();portal_id_=std::max(portal_id_+1,now().nanoseconds());
    passage_reply_source_=-1;last_portal_request_=-10;portal_progress_.reset();}
'''
for signature in (
    "void completePassage(", "void receivePassageGrant(",
    "void updatePassageAdmission(", "void releasePassageIfExited(",
    "void sendPassageRequest(", "double passageSpeedLimit(",
):
    code += method("safe_airfar_path_follower.cpp", signature) + "\n"
code += r'''
};
struct Coordinator:NodeClock {
  PassageScheduler scheduler_;std::string world_{"world"};TFBuffer tf_;
  double last_time_{-1},last_status_{-1};
  std::vector<nav_msgs::msg::Path>requests_{2};
  std::vector<std::shared_ptr<Publisher<nav_msgs::msg::Path>>>grants_{
    std::make_shared<Publisher<nav_msgs::msg::Path>>(),std::make_shared<Publisher<nav_msgs::msg::Path>>()};
  std::shared_ptr<Publisher<std_msgs::msg::String>>status_{std::make_shared<Publisher<std_msgs::msg::String>>()};
'''
for signature in ("void receive(size_t", "void tick()"):
    code += method("passage_coordinator.cpp", signature) + "\n"
code += r'''
};
void exchange(Follower&f,Coordinator&c,bool deliver=true){
  f.odom_stamp_=f.now();f.last_portal_request_=-10;f.sendPassageRequest();c.ros=f.ros;
  c.receive(0,f.passage_request_pub_->messages.back());c.tick();
  if(deliver)f.receivePassageGrant(c.grants_[0]->messages.back());
}
void replay(Vec3 anchor,Vec3 normal,const std::vector<Vec3>&positions){
  Follower f;Coordinator c;f.start(anchor,normal);
  for(Vec3 p:positions){f.ros+=.12;f.current_=p;exchange(f,c);
    f.updatePassageAdmission();assert(f.portal_active_&&f.passage_granted_&&!f.portal_waiting_);
    assert(f.passageSpeedLimit(1.8)==1.8);
  }
  auto delta=positions.back()-anchor;f.current_=anchor+delta*(2.45/norm(delta));f.ros+=.12;
  exchange(f,c);assert(!f.portal_active_&&!f.portal_waiting_);
  assert(f.passage_reply_state_=="COMPLETED_LOCAL"&&f.passage_completions_==1);
  assert(f.passage_request_pub_->messages.back().poses.size()==1&&f.passageSpeedLimit(1.8)==1.8);
  assert(c.scheduler_.leases.empty()&&!c.scheduler_.admitted(0));
}
int main(){
  replay({8.743,6.698,2.934},{1,0,0},{{9.889,7.278,3.0},{10.398,7.789,2.819},
    {10.43,7.809,2.812},{10.613,7.714,2.848}});
  replay({29.524,3.24,2.166},{0,1,0},{{27.62,3.037,2.21},{27.872,3.04,2.207},
    {31.466,3.127,2.633},{31.693,2.909,2.617}});

  Follower f;Coordinator c;Vec3 anchor{0,0,1.5};f.start(anchor,{0,1,0});
  f.current_=anchor;exchange(f,c,false); // Lose all initial grant replies.
  assert(c.scheduler_.admitted(0)&&!f.passage_granted_);
  f.current_={2.45,0,1.5};f.ros+=.12;exchange(f,c,false);
  auto ack=c.grants_[0]->messages.back();assert(ack.poses.size()==2);
  assert(f.portal_active_); // Client missed the entry/grant; local exit alone cannot complete.
  f.ros+=.12;exchange(f,c); // Same request gets the completion again.
  assert(!f.portal_active_&&f.passage_reply_state_=="COMPLETED_ACK");
  assert(f.passage_completions_==1&&f.passage_request_pub_->messages.back().poses.size()==1);
  f.ros+=.12;exchange(f,c);assert(c.scheduler_.leases.empty());

  f.ros+=.12;f.start(anchor,{0,1,0});f.current_=anchor;
  ack.header.stamp=f.now();f.receivePassageGrant(ack);assert(f.portal_active_&&!f.passage_granted_);
  ack.poses[0].header.stamp=rclcpp::Time(f.portal_id_);
  f.receivePassageGrant(ack);assert(f.portal_active_&&!f.passage_granted_);
  assert(f.passage_reply_state_=="COMPLETION_POSITION_MISMATCH"); // Cannot unlock while physically inside.
  exchange(f,c);assert(f.passage_granted_);
  auto good=c.grants_[0]->messages.back();
  f.ros+=.12;exchange(f,c);auto stale=good;stale.poses.clear();
  f.receivePassageGrant(stale);assert(f.passage_granted_); // Old deny cannot overwrite a newer grant.
  good.header.frame_id="wrong/odom";f.passage_granted_=false;f.receivePassageGrant(good);assert(!f.passage_granted_);
  auto old_release=makePath({{6,0,1.5}},f.frame_,rclcpp::Time(int64_t((f.ros-.12)*1e9)));
  c.receive(0,old_release);c.tick();assert(c.scheduler_.admitted(0));

  // A lost owner remains quarantined. A /clock rewind clears completion replies,
  // not physical ownership; a fresh same-owner request can safely resume it.
  c.ros+=2;c.tick();assert(c.scheduler_.leases.front().quarantined&&!c.scheduler_.admitted(0));
  c.ros=1;c.tick();assert(c.scheduler_.completed.empty()&&c.scheduler_.leases.front().quarantined);
  f.ros=1;f.start(anchor,{0,1,0});exchange(f,c);assert(f.passage_granted_);
  Follower rerouted;Coordinator other;
  rerouted.start(anchor,{0,1,0});rerouted.current_={0,-3,1.5};exchange(rerouted,other);
  assert(other.scheduler_.admitted(0));
  rerouted.pending_path_={{0,-3,1.5},{3,-3,1.5},{3,3,1.5}};
  rerouted.ros+=.12;exchange(rerouted,other);
  assert(!rerouted.portal_active_ && other.scheduler_.leases.empty());
  // A stale measured position must not keep an occupied doorway lease alive.
  rerouted.start(anchor,{0,1,0});rerouted.current_=anchor;exchange(rerouted,other);
  auto messages=rerouted.passage_request_pub_->messages.size();
  rerouted.ros+=2;rerouted.last_portal_request_=-10;rerouted.sendPassageRequest();
  assert(rerouted.passage_request_pub_->messages.size()==messages);

  // The logged failure: the first migration arrives INSIDE the old lease.
  // Later fresh position belongs to the NEW request and must still release
  // the old physical opening, atomically preserving that new request.
  Coordinator migrated;
  auto bid=[&](size_t robot,Vec3 a,Vec3 p,int64_t epoch){
    auto message=makePath({a,p,a+Vec3{2.2,0,0}},"world",migrated.now());
    message.poses[0].header.stamp=rclcpp::Time(epoch);
    migrated.receive(robot,message);return message;
  };
  bid(0,{0,0,3},{0,0,3},10);migrated.tick();assert(migrated.scheduler_.admitted(0));
  migrated.ros+=.1;bid(1,{0,0,3},{-3,0,3},11);
  auto first_new=bid(0,{5,0,3},{1,0,3},20);migrated.tick();
  assert(!migrated.scheduler_.admitted(0)&&!migrated.scheduler_.admitted(1));
  assert(migrated.scheduler_.leases[0].owner==0&&migrated.scheduler_.leases[0].quarantined);
  migrated.ros+=2;migrated.tick(); // Lost heartbeat cannot clear ownership.
  assert(migrated.scheduler_.leases[0].owner==0);
  bid(1,{0,0,3},{-3,0,3},11);bid(0,{5,0,3},{2.6,0,3},20);migrated.tick();
  assert(migrated.scheduler_.admitted(0)&&migrated.scheduler_.admitted(1));
  assert(migrated.scheduler_.migrations_completed==1);
  assert(migrated.grants_[0]->messages.back().poses.size()==1);
  assert(stampNs(migrated.grants_[0]->messages.back().poses[0].header.stamp)==20);
  // Reordered request / stale odometry cannot migrate back to the old portal.
  migrated.receive(0,first_new);migrated.tick();
  assert(migrated.scheduler_.bids.at(0).position.x==2.6&&migrated.scheduler_.admitted(0));
  std::cout<<"v2_node_passage_test: PASS actual follower/coordinator methods, both logged exits, local release, dropped completion, old reply, wrong frame/id, inside rejection, rewind\n";
}
'''
with tempfile.TemporaryDirectory(prefix="fire_scout_node_passage_") as temp:
    path = pathlib.Path(temp)
    (path / "test.cpp").write_text(code)
    subprocess.run(["g++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-Wpedantic",
                    "-UNDEBUG", "-I", str(ROOT / "include"),
                    str(path / "test.cpp"), "-o", str(path / "test")], check=True)
    subprocess.run([str(path / "test")], check=True)
