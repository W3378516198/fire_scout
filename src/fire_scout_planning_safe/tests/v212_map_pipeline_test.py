#!/usr/bin/env python3
"""Compile the actual new ROS-envelope, callback, drain and input-selector bodies.

Only ROS transport, TF and logging are stubs. This is not a DDS/flight test.
"""
import os
import pathlib
import subprocess
import tempfile

ROOT = pathlib.Path(__file__).resolve().parents[1]
planner = (ROOT / "src/safe_airfar_like_planner.cpp").read_text()
fusion = (ROOT / "src/multi_uav_map_fusion.cpp").read_text()
follower = (ROOT / "src/safe_airfar_path_follower.cpp").read_text()
mapper = (ROOT / "src/radar_free_space_mapper.cpp").read_text()
utils = (ROOT / "include/fire_scout/ros_utils.hpp").read_text()


def extract(source, signature, opening=None):
    start = source.index(signature)
    a = source.index(opening or "{", start)
    if opening:
        a += len(opening) - 1
    depth, end = 1, a + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


def callback(source, after, signature):
    tail = source[source.index(after):]
    method = extract(tail, signature)
    return method[method.index("{"):]


code = r'''
#include "fire_scout/map_input.hpp"
#include "fire_scout/map_snapshot.hpp"
#include "fire_scout/map_pair_buffer.hpp"
#include "fire_scout/route_memory.hpp"
#include "fire_scout/snapshot_stamp.hpp"
#include <cassert>
#include <iostream>
#include <memory>
#define RCLCPP_WARN(...) do{}while(0)
#define RCLCPP_ERROR(...) do{}while(0)
#define RCLCPP_WARN_THROTTLE(...) do{}while(0)
#define RCLCPP_ERROR_THROTTLE(...) do{}while(0)
#define RCLCPP_INFO_THROTTLE(logger,clock,period,...) logSink(__VA_ARGS__)
template<class... T>void logSink(T&&...){}
using namespace fire_scout;
namespace builtin_interfaces {namespace msg {struct Time{int32_t sec{0};uint32_t nanosec{0};};}}
namespace rclcpp {struct Time{
  int64_t ns;Time(int64_t n=0,int=0):ns(n){}
  Time(int32_t s,uint32_t n,int):ns(int64_t(s)*1000000000LL+n){}
  Time(builtin_interfaces::msg::Time t,int=0):ns(int64_t(t.sec)*1000000000LL+t.nanosec){}
  int64_t nanoseconds()const{return ns;}double seconds()const{return ns*1e-9;}
  friend Time operator-(Time a,Time b){return Time(a.ns-b.ns);}
  operator builtin_interfaces::msg::Time()const{return {int32_t(ns/1000000000LL),uint32_t(ns%1000000000LL)};}
};}
namespace sensor_msgs {namespace msg {
struct PointField {
  static constexpr uint8_t INT32=5,UINT32=6,FLOAT32=7;
  std::string name;uint32_t offset{0},count{0};uint8_t datatype{0};
};
struct PointCloud2 {
  using SharedPtr=std::shared_ptr<PointCloud2>;
  struct Header {builtin_interfaces::msg::Time stamp;std::string frame_id;}header;
  uint32_t height{0},width{0},point_step{0},row_step{0};
  bool is_dense{false},is_bigendian{false};std::vector<PointField>fields;std::vector<uint8_t>data;
};}}
using Cloud=sensor_msgs::msg::PointCloud2;
struct Clock {int get_clock_type()const{return 0;}};
int64_t stampNs(builtin_interfaces::msg::Time t){return int64_t(t.sec)*1000000000LL+t.nanosec;}
namespace tf2 {struct TransformException:std::runtime_error {using std::runtime_error::runtime_error;};}
'''
code += extract(utils, "inline sensor_msgs::msg::PointCloud2 makeMapSnapshot(")
code += extract(utils, "inline map_snapshot::Decoded parseMapSnapshot(", "){" )
code += r'''
std::shared_ptr<Cloud> packet(double time,KeySet occupied={},KeySet free={}){
  return std::make_shared<Cloud>(makeMapSnapshot(occupied,free,.18,"scout3/odom",rclcpp::Time(int64_t(time*1e9))));
}
struct PlannerFixture {
  double ros{136};std::string frame_{"scout3/odom"};
  rclcpp::Time now()const{return rclcpp::Time(int64_t(ros*1e9));}
  std::shared_ptr<Clock>get_clock()const{return std::make_shared<Clock>();}
  bool compact_maps_{true},local_fallback_enabled_{true},have_map_{false},have_goal_{true},have_odom_{true};
  std::shared_ptr<Grid>grid_{std::make_shared<Grid>()},primary_grid_;
  std::shared_ptr<Grid>occupied_guard_;int64_t occupied_guard_stamp_{-1};
  MapInputSource map_source_{MapInputSource::None};
  int64_t primary_stamp_ns_{-1},local_committed_stamp_ns_{-1};
  int64_t last_primary_input_stamp_ns_{0},last_local_input_stamp_ns_{0};
  size_t primary_received_{0},local_received_{0},map_commits_{0},map_rejected_{0},primary_bytes_{0},local_bytes_{0},generation_{0};
  Cloud::SharedPtr primary_pending_,local_pending_;
  rclcpp::Time map_stamp_,odom_stamp_{136000000000LL};double map_timeout_{5},odom_timeout_{1.2};
  Vec3 current_{0,0,1.5};std::vector<Vec3>owned_path_;RouteMemory route_;
  void observeInputStamp(int64_t stamp,const char*,int64_t &latest){latest=std::max(latest,stamp);}
  void observeInvalidation(int,bool,const char*){}
  int inspectOwnedRoute()const{return 0;}
  PlannerFixture(){grid_->cfg.resolution=.18;}
  void receivePrimary(Cloud::SharedPtr m)
'''
code += callback(planner, "snapshot_sub_=create_subscription", "[this](sensor_msgs::msg::PointCloud2::SharedPtr m)")
code += "\n void receiveLocal(Cloud::SharedPtr m)\n"
code += callback(planner, "local_snapshot_sub_=create_subscription", "[this](sensor_msgs::msg::PointCloud2::SharedPtr m)")
for signature in ("  void installSnapshotGrid(", "  void refreshCompactMap()", "  std::string inputState() const"):
    code += extract(planner, signature)
code += r'''
};
struct FusionFixture {
  using Cloud=sensor_msgs::msg::PointCloud2;using CloudPtr=Cloud::SharedPtr;using PairBuffer=MapPairBuffer<CloudPtr>;
  struct Source {
    std::string frame{"scout3/odom"},name{"scout3"};CloudPtr snapshot_pending;
    PairBuffer pairs;std::optional<PairBuffer::Pair>pending;
    KeySet occupied_world,free_world;int64_t stamp_ns{-1},latest_snapshot_input_ns{-1};
    bool ready{false};size_t received{0},coalesced{0},commits{0},rejected{0},input_bytes{0};
  };
  std::vector<std::unique_ptr<Source>>sources_;
  bool compact_maps_{true},dirty_{false},missing_tf{false};double resolution_{.18};
  size_t max_points_per_cloud_{4000000};std::string world_frame_{"world"};
  std::shared_ptr<Clock>get_clock()const{return std::make_shared<Clock>();}
  int lookup(const std::string&,const std::string&,builtin_interfaces::msg::Time){
    if(missing_tf)throw tf2::TransformException("no TF");
    return 0;
  }
  int tf2Transform(int){return 0;}
  KeySet transformKeys(const KeySet &keys,int){return keys;}
  KeySet transformCloud(const Cloud&,int){return {};}
  void observeSourceStamp(size_t,int64_t stamp,int64_t &last,const char*){last=std::max(last,stamp);}
  FusionFixture(){sources_.push_back(std::make_unique<Source>());}
  void receive(CloudPtr msg){const size_t i=0;
'''
cb = callback(fusion, "source.snapshot_sub=create_subscription", "[this,i](Cloud::SharedPtr msg)")
code += cb[1:-1] + "\n}\n" + extract(fusion, "  void drain(size_t index)") + "\n};\n"
code += r'''
struct FollowerFixture {
  std::string frame_{"scout3/odom"};bool have_odom_{true},have_map_{false};
  Vec3 current_{0,0,1.5},local_map_anchor_,observed_map_anchor_;
  std::unique_ptr<Grid>grid_{std::make_unique<Grid>()},observed_grid_;
  double local_map_extent_{4};int64_t occupied_stamp_{0},free_stamp_{-1};
  int64_t last_occupied_input_stamp_ns_{0},last_free_input_stamp_ns_{0};
  size_t snapshot_commits_{0},snapshot_rejected_{0},snapshot_bytes_{0};rclcpp::Time map_stamp_;
  std::shared_ptr<Clock>get_clock()const{return std::make_shared<Clock>();}
  void observeMapInputStamp(int64_t stamp,int64_t &last,const char*){last=std::max(last,stamp);}
  FollowerFixture(){grid_->cfg.resolution=.18;}
  void receive(Cloud::SharedPtr m)
'''
code += callback(follower, "snapshot_sub_=create_subscription", "[this](sensor_msgs::msg::PointCloud2::SharedPtr m)")
code += r'''
};
Cloud makeCloud(const KeySet &cells,double,const std::string &frame,builtin_interfaces::msg::Time stamp){
  Cloud m;m.header.frame_id=frame;m.header.stamp=stamp;m.height=1;
  m.width=uint32_t(cells.size());m.point_step=16;m.row_step=m.width*16;m.data.resize(m.row_step);return m;
}
struct FakePublisher {
  size_t subscribers{1};std::vector<Cloud>messages;
  size_t get_subscription_count()const{return subscribers;}
  void publish(Cloud m){messages.push_back(std::move(m));}
};
struct MapperFixture {
  struct Sonar{KeySet occupied(double)const{return {};}}sonar_layer_;
  KeySet last_sonar_snapshot_;bool have_stamp_{true},dirty_{true},compact_maps_{true},publish_free_clouds_{false};
  SnapshotStamp snapshot_stamps_;std::string frame_{"scout3/odom"};
  builtin_interfaces::msg::Time stamp_{136,0};
  std::unique_ptr<EvidenceMap>map_{std::make_unique<EvidenceMap>(.18)};
  std::shared_ptr<FakePublisher>snapshot_pub_{std::make_shared<FakePublisher>()};
  std::shared_ptr<FakePublisher>occ_pub_{std::make_shared<FakePublisher>()},free_pub_{std::make_shared<FakePublisher>()};
  std::shared_ptr<FakePublisher>radar_only_pub_,sonar_pub_{std::make_shared<FakePublisher>()},visualization_pub_;
  static double steadySeconds(){return 0;}
'''
code += extract(mapper, "  void publish() {")
code += r'''
};
int main(){
  PlannerFixture p;
  p.receivePrimary(packet(111,{{30,20,8}},{{31,20,8}}));
  p.receiveLocal(packet(135.85,{{5,5,8}},{{0,0,8}}));p.refreshCompactMap();
  assert(p.map_source_==MapInputSource::LocalFallback&&p.inputState()=="READY");
  assert(p.map_stamp_.nanoseconds()==135850000000LL);
  assert(p.grid_->occupied.count({30,20,8})&&!p.grid_->free.count({31,20,8}));
  p.route_.set({{0,0,1.5},{12,0,1.5}});
  for(int x=1;x<=8;++x)assert(p.route_.updateProgress({double(x),0,1.5}));
  const auto progress=p.route_.progress();
  for(int step=1;step<=30;++step){
    p.ros+=.25;p.odom_stamp_=p.now();p.receiveLocal(packet(p.ros-.10));p.refreshCompactMap();
    assert(p.inputState()=="READY"&&p.route_.progress()==progress);
  }
  p.ros+=6;p.odom_stamp_=p.now();p.refreshCompactMap();assert(p.inputState()=="STALE_MAP");
  p.receivePrimary(packet(p.ros-.1,{{6,7,8}}));p.refreshCompactMap();
  assert(p.map_source_==MapInputSource::Primary&&p.inputState()=="READY"&&p.route_.progress()==progress);
  auto bad=packet(p.ros+.1);bad->data.pop_back();p.receivePrimary(bad);p.refreshCompactMap();
  assert(p.map_rejected_==1&&p.primary_stamp_ns_==stampNs(p.map_stamp_));
  auto envelope=packet(136);envelope->is_bigendian=true;
  bool rejected=false;try{parseMapSnapshot(*envelope,.18);}catch(const std::runtime_error&){rejected=true;}assert(rejected);
  envelope=packet(136);envelope->fields[1].name="invalid";rejected=false;
  try{parseMapSnapshot(*envelope,.18);}catch(const std::runtime_error&){rejected=true;}assert(rejected);
  FusionFixture f;
  for(int i=1;i<=20;++i)f.receive(packet(100+i,{{i,0,8}},{{i,-1,8}}));
  assert(f.sources_[0]->received==20&&f.sources_[0]->coalesced==19&&f.sources_[0]->commits==0);
  f.missing_tf=true;f.drain(0);assert(f.sources_[0]->snapshot_pending);
  f.receive(packet(121,{{21,0,8}},{{21,-1,8}}));f.missing_tf=false;f.drain(0);
  assert(f.sources_[0]->stamp_ns==121000000000LL&&f.sources_[0]->commits==1);
  assert(f.sources_[0]->occupied_world.count({21,0,8})&&!f.sources_[0]->occupied_world.count({20,0,8}));
  FollowerFixture follower;
  follower.receive(packet(136,{{0,0,8},{100,0,8}},{{1,0,8},{101,0,8}}));
  assert(follower.have_map_&&follower.free_stamp_==follower.occupied_stamp_);
  assert(follower.grid_->occupied.count({0,0,8})&&!follower.grid_->occupied.count({100,0,8}));
  assert(follower.observed_grid_->free.count({1,0,8})&&!follower.observed_grid_->free.count({101,0,8}));
  auto invalid=packet(137);invalid->data.pop_back();follower.receive(invalid);
  assert(follower.snapshot_rejected_==1&&follower.free_stamp_==136000000000LL);
  MapperFixture mapper;mapper.publish();
  assert(mapper.snapshot_pub_->messages.size()==1&&mapper.free_pub_->messages.empty());
  mapper.publish_free_clouds_=true;mapper.dirty_=true;mapper.publish();
  assert(mapper.free_pub_->messages.size()==1&&mapper.snapshot_pub_->messages.size()==2);
  MapperFixture legacy;legacy.compact_maps_=false;legacy.snapshot_pub_.reset();legacy.publish();
  assert(legacy.occ_pub_->messages.size()==1&&legacy.free_pub_->messages.size()==1);
  assert(stampNs(legacy.occ_pub_->messages[0].header.stamp)==stampNs(legacy.free_pub_->messages[0].header.stamp));
  std::cout<<"v212_map_pipeline_test: PASS actual envelope/callback/drain/selector; fusion outage, recovery, both-stale hold, retained progress, coalescing, missing TF, malformed packet, bounded follower decode\n";
}
'''
with tempfile.TemporaryDirectory() as tmp:
    cpp = pathlib.Path(tmp) / "map_pipeline.cpp"
    binary = pathlib.Path(tmp) / "map_pipeline"
    cpp.write_text(code)
    flags = ["-std=c++17", "-O1", "-Wall", "-Wextra", "-Wpedantic", "-UNDEBUG"]
    if os.environ.get("FIRE_SCOUT_SANITIZE"):
        flags += ["-fsanitize=address,undefined", "-fno-omit-frame-pointer", "-g"]
    subprocess.run(["g++", *flags, "-I" + str(ROOT / "include"), str(cpp), "-o", str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
