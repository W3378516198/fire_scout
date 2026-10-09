#!/usr/bin/env python3
"""Compile the actual planner peer callbacks with message/TF transport stubs."""
import pathlib
import subprocess
import tempfile

ROOT = pathlib.Path(__file__).resolve().parents[1]
SOURCE = (ROOT / 'src/safe_airfar_like_planner.cpp').read_text()


def extract(signature):
    start = SOURCE.index(signature)
    a = SOURCE.index('{', start)
    depth, b = 1, a + 1
    while depth:
        depth += (SOURCE[b] == '{') - (SOURCE[b] == '}')
        b += 1
    return SOURCE[start:b]


code = r'''
#include "fire_scout/route_repair.hpp"
#include "fire_scout/racer_task_allocator.hpp"
#include <cassert>
#include <iostream>
#include <memory>
#include <string>
#define RCLCPP_WARN_THROTTLE(...) do{}while(0)
using namespace fire_scout;
namespace builtin_interfaces {namespace msg {struct Time{int sec;unsigned nanosec;};}}
namespace rclcpp {
struct Time {
  int64_t ns;Time(int64_t n=0,int=0):ns(n){}
  Time(int s,unsigned n,int):ns(int64_t(s)*1000000000LL+n){}
  Time(builtin_interfaces::msg::Time t,int):ns(int64_t(t.sec)*1000000000LL+t.nanosec){}
  int64_t nanoseconds()const{return ns;}
  operator builtin_interfaces::msg::Time()const{return {int(ns/1000000000LL),unsigned(ns%1000000000LL)};}
};
struct Duration {static Duration from_seconds(double){return {};}};
}
struct Header{builtin_interfaces::msg::Time stamp;std::string frame_id;};
struct Rotation{double x{0},y{0},z{0},w{1};};
namespace geometry_msgs {namespace msg {
struct TransformStamped{struct Content{Vec3 translation{};Rotation rotation;}transform;};
struct PoseStamped {struct Pose{Vec3 position;}pose;};
}}
namespace nav_msgs {namespace msg {
struct Path{Header header;std::vector<geometry_msgs::msg::PoseStamped>poses;};
struct Odometry{Header header;struct Outer{struct Pose{Vec3 position;}pose;}pose;};
}}
namespace tf2 {
struct TransformException:std::runtime_error{using std::runtime_error::runtime_error;};
struct Quaternion {
  Quaternion(double,double,double,double){} double length2()const{return 1;}
  void normalize(){}
};
struct Vector3{
  Vec3 p;Vector3(double x=0,double y=0,double z=0):p{x,y,z}{}
  double x()const{return p.x;} double y()const{return p.y;} double z()const{return p.z;}
};
struct Transform {
  Vec3 translation{};Transform()=default;
  Transform(Quaternion,Vector3 p):translation(p.p){}
  void setIdentity(){translation={};}
  Vector3 operator*(Vector3 v)const{Vec3 p=v.p+translation;return {p.x,p.y,p.z};}
};
}
struct TFBuffer {
  geometry_msgs::msg::TransformStamped lookupTransform(
      const std::string&,const std::string&,rclcpp::Time,rclcpp::Duration)const{return {};}
};
struct Clock{int get_clock_type()const{return 0;}};
struct Fixture {
  double ros{100};std::string frame_{"scout1/odom"};
  double peer_tf_timeout_{.1},peer_path_timeout_{5},peer_odom_timeout_{1.2},peer_conflict_horizon_{10};
  size_t peer_generation_{0};TFBuffer tf_;
  rclcpp::Time now()const{return rclcpp::Time(int64_t(ros*1e9));}
  std::shared_ptr<Clock>get_clock()const{return std::make_shared<Clock>();}
'''
code += extract('  struct PeerRoute {') + ';std::vector<PeerRoute>peer_routes_{1};\n'
for signature in ('  static tf2::Transform toTransform(', '  bool peerRouteChanged(',
                  '  void receivePeerPath(', '  void receivePeerOdom(',
                  '  std::vector<std::vector<Vec3>> freshPeerRoutes('):
    code += extract(signature) + '\n'
code += r'''
  void path(const std::vector<Vec3>&points){
    nav_msgs::msg::Path msg;msg.header={now(),frame_};
    for(auto p:points)msg.poses.push_back({{p}});
    receivePeerPath(0,msg);
  }
  void move(Vec3 p){
    ros+=.02;nav_msgs::msg::Odometry msg;msg.header={now(),frame_};
    msg.pose.pose.position=p;receivePeerOdom(0,msg);
  }
};
int main(){
  Fixture f;const std::vector<Vec3>line{{0,0,1.5},{40,0,1.5}};
  f.path(line);assert(f.freshPeerRoutes().empty());
  for(int k=0;k<=36;++k){
    f.move({k*.25,0,1.5});
    auto routes=f.freshPeerRoutes();assert(routes.size()==1);
    assert(std::abs(routes.front().front().x-k*.25)<1e-8);
    assert(f.peer_routes_[0].progress>=k*.25-1e-8);
  }
  auto generation=f.peer_generation_;auto progress=f.peer_routes_[0].progress;
  f.path(line);assert(f.peer_generation_==generation);
  assert(f.peer_routes_[0].progress==progress && f.freshPeerRoutes().size()==1);
  // Keeping the old prefix while replacing a distant tail preserves origin.
  f.path({{0,0,1.5},{12,0,1.5},{30,4,1.5}});
  assert(std::abs(f.peer_routes_[0].progress-9)<1e-8);
  assert(f.freshPeerRoutes().size()==1);
  // An actual local replacement obtains its own origin at the live position.
  f.path({{9,0,1.5},{20,2,1.5}});
  assert(f.peer_routes_[0].progress<1e-8 && f.freshPeerRoutes().size()==1);

  Fixture late;late.move({9,0,1.5});late.path(line);
  assert(late.peer_routes_[0].progress_valid && std::abs(late.peer_routes_[0].progress-9)<1e-8);
  assert(late.freshPeerRoutes().size()==1);

  const std::vector<Vec3>crossing{{0,0,1.5},{8,0,1.5},{8,4,1.5},{4,4,1.5},{4,-4,1.5},{12,-4,1.5}};
  Fixture cross;cross.path(crossing);
  for(int k=0;k<=16;++k)cross.move({k*.25,0,1.5});
  assert(std::abs(cross.peer_routes_[0].progress-4)<1e-8);
  cross.path(crossing);cross.move({4,0,1.5});
  assert(std::abs(cross.peer_routes_[0].progress-4)<1e-8); // Never jumps to arc20.
  assert(cross.freshPeerRoutes().size()==1);
  Fixture ambiguous;ambiguous.move({4,0,1.5});ambiguous.path(crossing);
  assert(!ambiguous.peer_routes_[0].progress_valid && ambiguous.freshPeerRoutes().empty());
  // A large isolated pose jump cannot reset an already established origin.
  cross.move({12,-4,1.5});assert(!cross.peer_routes_[0].progress_valid);
  assert(cross.freshPeerRoutes().empty());
  cross.move({4.25,0,1.5});assert(cross.freshPeerRoutes().size()==1);
  assert(cross.peer_routes_[0].progress<5);
  f.path({});assert(f.freshPeerRoutes().empty() && !f.peer_routes_[0].have_progress);
  std::cout<<"v214_peer_progress_test: PASS actual callbacks, long route, heartbeat, replacement, late initialization, crossings\n";
}
'''
with tempfile.TemporaryDirectory(prefix='fire_scout_peer_progress_') as directory:
    cpp = pathlib.Path(directory) / 'test.cpp'
    executable = pathlib.Path(directory) / 'test'
    cpp.write_text(code)
    subprocess.run(['g++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-Wpedantic',
                    '-UNDEBUG', '-pthread', '-I' + str(ROOT / 'include'),
                    str(cpp), '-o', str(executable)], check=True)
    subprocess.run([str(executable)], check=True)
