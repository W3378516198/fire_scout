#!/usr/bin/env python3
"""Compile production fusion methods with ROS/TF transport stubs.

The baseline is the former full evidence/classification algorithm. Compare
observer decisions and canonical packet bytes, including the rotated-TF
fallback. Set FIRE_SCOUT_FUSION_BENCHMARK=1 for a four-UAV large-map benchmark.
"""
import os
import pathlib
import subprocess
import tempfile

ROOT = pathlib.Path(__file__).resolve().parents[1]
fusion = (ROOT / "src/multi_uav_map_fusion.cpp").read_text()
utils = (ROOT / "include/fire_scout/ros_utils.hpp").read_text()


def extract(source, signature):
    start = source.index(signature)
    a = source.index("{", start)
    depth, end = 1, a + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


code = r'''
#include "fire_scout/fusion_snapshot.hpp"
#include "fire_scout/confidence_fusion.hpp"
#include "fire_scout/peer_uav_filter.hpp"
#include <cassert>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <random>
#include <sstream>
using namespace fire_scout;
#define RCLCPP_WARN_THROTTLE(...) do{}while(0)
#define RCLCPP_ERROR_THROTTLE(...) do{}while(0)
#define RCLCPP_INFO_THROTTLE(...) do{}while(0)
namespace builtin_interfaces { namespace msg { struct Time { int32_t sec{0}; uint32_t nanosec{0}; }; } }
namespace sensor_msgs { namespace msg {
struct PointField {
  static constexpr uint8_t INT32=5,UINT32=6,FLOAT32=7;
  std::string name;uint32_t offset{0},count{0};uint8_t datatype{0};
};
struct PointCloud2 {
  struct Header {builtin_interfaces::msg::Time stamp;std::string frame_id;} header;
  uint32_t height{0},width{0},point_step{0},row_step{0};
  bool is_dense{false},is_bigendian{false};std::vector<PointField>fields;std::vector<uint8_t>data;
};}}
using Cloud=sensor_msgs::msg::PointCloud2;
namespace tf2 {
struct TransformException:std::runtime_error {using std::runtime_error::runtime_error;};
struct Vector3 {
  double a,b,c;Vector3(double x=0,double y=0,double z=0):a(x),b(y),c(z){}
  double x()const{return a;}double y()const{return b;}double z()const{return c;}
};
struct Quaternion {
  double yaw{0};double x()const{return 0;}double y()const{return 0;}
  double z()const{return std::sin(yaw/2);}double w()const{return std::cos(yaw/2);}
};
struct Transform {
  Vector3 position;Quaternion rotation;
  Vector3 getOrigin()const{return position;}Quaternion getRotation()const{return rotation;}
  Vector3 operator*(Vector3 p)const {
    const double c=std::cos(rotation.yaw),s=std::sin(rotation.yaw);
    return {c*p.x()-s*p.y()+position.x(),s*p.x()+c*p.y()+position.y(),p.z()+position.z()};
  }
};
}
struct FakePublisher {
  size_t subscribers{0};std::vector<Cloud>messages;
  size_t get_subscription_count()const{return subscribers;}
  void publish(Cloud m){messages.push_back(std::move(m));}
};
struct StatusPublisher {std::string message;void publish(std::string m){message=std::move(m);}};
std::string textMessage(const std::string &s){return s;}
Cloud makeCloud(const KeySet &cells,double,const std::string &frame,builtin_interfaces::msg::Time stamp){
  Cloud m;m.header.frame_id=frame;m.header.stamp=stamp;m.width=uint32_t(cells.size());return m;
}
'''
code += extract(utils, "inline sensor_msgs::msg::PointCloud2 makeMapSnapshot(")
code += r'''
struct Fixture {
  using Cloud=sensor_msgs::msg::PointCloud2;
  struct Source {
    std::string name,frame;KeySet occupied_world,free_world;
    int64_t stamp_ns{1000000000LL},last_output_stamp_ns{-1},latest_snapshot_input_ns{1000000000LL};
    int64_t latest_occupied_input_ns{-1},latest_free_input_ns{-1};
    size_t received{0},commits{0},coalesced{0},rejected{0},input_bytes{0},output_bytes{0};
    PeerUavBoxHistory peer_history;
    struct Pairs {size_t occupiedSize()const{return 0;}size_t freeSize()const{return 0;}}pairs;
    std::shared_ptr<FakePublisher>fused_snapshot_pub{std::make_shared<FakePublisher>()};
    std::shared_ptr<FakePublisher>fused_occupied_pub{std::make_shared<FakePublisher>()};
    std::shared_ptr<FakePublisher>fused_free_pub{std::make_shared<FakePublisher>()};
    tf2::Transform local_tf;
  };
  std::vector<std::unique_ptr<Source>>sources_;
  bool compact_maps_{true},publish_free_clouds_{false},peer_filter_enabled_{false},dirty_{true};
  std::string world_frame_{"world"};double resolution_{.18};
  double self_map_confidence_{1},peer_map_confidence_{.45},history_map_confidence_{.2};
  double confidence_decay_time_{1.5},dynamic_obstacle_confidence_{.05};
  double minimum_occupied_confidence_{.2},occupied_conflict_bias_{1.05};
  bool self_observation_priority_{true};
  std::shared_ptr<FakePublisher>shared_occupied_pub_{std::make_shared<FakePublisher>()};
  std::shared_ptr<FakePublisher>shared_free_pub_{std::make_shared<FakePublisher>()};
  std::shared_ptr<FakePublisher>shared_snapshot_pub_{std::make_shared<FakePublisher>()};
  std::shared_ptr<StatusPublisher>status_pub_{std::make_shared<StatusPublisher>()};
  bool freshPeerPose(const Source&,int64_t)const{return false;}
  std::vector<PeerUavBox>peerBoxes(size_t,int64_t)const{return {};}
  static tf2::Transform tf2Transform(tf2::Transform t){return t;}
  tf2::Transform lookup(const std::string &frame,const std::string&,builtin_interfaces::msg::Time)const {
    for(const auto &source:sources_)if(source->frame==frame)return source->local_tf;
    throw tf2::TransformException("Missing test transform");
  }
'''
for signature in ("  struct PreparedSource", "  struct WeightedEvidence", "  struct EvidenceTable", "  struct FusionResult"):
    code += extract(fusion, signature) + ";\n"
for signature in ("  double confidenceFreshness(", "  KeySet transformKeys(", "  static builtin_interfaces::msg::Time timeMessage(",
                  "  EvidenceTable buildEvidence(", "  FusionResult classifyEvidence(", "  std::optional<Key> snapshotTranslation(",
                  "  Cloud sparseSnapshot(", "  void publish("):
    code += extract(fusion, signature)
code += r'''
};
using FullEvidence=std::unordered_map<Key,Fixture::WeightedEvidence,KeyHash>;
FullEvidence referenceEvidence(const Fixture &fixture,const std::vector<Fixture::PreparedSource> &prepared,
                               const std::vector<bool> &active){
  FullEvidence evidence;
  for(size_t i=0;i<prepared.size();++i){
    if(!active[i]||!prepared[i].free)continue;
    const double normal=fixture.peer_map_confidence_*prepared[i].freshness;
    for(Key k:prepared[i].occupied)evidence[k].occupied+=normal;
    for(Key k:prepared[i].uncertain_occupied)evidence[k].occupied+=normal*fixture.dynamic_obstacle_confidence_;
    for(Key k:*prepared[i].free)evidence[k].free+=normal;
  }
  return evidence;
}
Fixture::FusionResult referenceClassify(const Fixture &fixture,const FullEvidence &evidence,
                                        const std::vector<Fixture::PreparedSource> &prepared,std::optional<size_t> observer){
  Fixture::FusionResult result;
  const ConfidenceDecisionConfig cfg{fixture.minimum_occupied_confidence_,fixture.occupied_conflict_bias_,fixture.self_observation_priority_};
  for(const auto &[voxel,base]:evidence){
    double occupied=base.occupied,free=base.free;bool self_occupied=false,self_free=false;
    if(observer){
      const auto &own=prepared.at(*observer);
      const double upgrade=(fixture.self_map_confidence_-fixture.peer_map_confidence_)*own.freshness;
      self_occupied=own.occupied.count(voxel)!=0;
      const bool self_uncertain=own.uncertain_occupied.count(voxel)!=0;
      self_free=own.free&&own.free->count(voxel)!=0;
      if(self_occupied)occupied+=upgrade;
      if(self_uncertain)occupied+=upgrade*fixture.dynamic_obstacle_confidence_;
      if(self_free)free+=upgrade;
    }
    if(occupied>0&&free>0)++result.conflicts;
    const auto state=decideFusionState(occupied,free,self_occupied,self_free,cfg);
    if(state==FusionState::Occupied)result.occupied.insert(voxel);
    if(state==FusionState::Free)result.free.insert(voxel);
    if(observer&&fixture.self_observation_priority_&&
      ((self_occupied&&free>0&&state==FusionState::Occupied)||
       (self_free&&occupied>=fixture.minimum_occupied_confidence_&&state==FusionState::Free)))++result.self_overrides;
  }
  return result;
}
void compare(const Fixture::FusionResult &a,const Fixture::FusionResult &b){
  assert(a.occupied==b.occupied&&a.free==b.free&&a.conflicts==b.conflicts&&a.self_overrides==b.self_overrides);
}
void randomizedEquivalence(){
  std::mt19937 rng(72104);
  for(size_t trial=0;trial<72;++trial){
    Fixture fixture;
    fixture.peer_map_confidence_=trial%5==0?0:trial%3==0?1:.45;
    fixture.self_observation_priority_=trial%2;
    fixture.dynamic_obstacle_confidence_=trial%7==0?0:.05;
    const size_t count=1+trial%9;
    std::vector<Fixture::PreparedSource>prepared(count);std::vector<KeySet>free(count);
    std::vector<bool>active(count,true);
    for(size_t i=0;i<count;++i){
      auto &s=prepared[i];s.free=&free[i];s.freshness=(trial+i)%11==0?0:.2+.1*((trial+i)%9);
      for(size_t j=0;j<240;++j){
        const Key k{int(rng()%25)-12,int(rng()%9)-4,int(rng()%4)-2};
        const auto state=rng()%7;
        if(state==0)s.occupied.insert(k);else if(state==1)s.uncertain_occupied.insert(k);else free[i].insert(k);
      }
      for(Key k:s.occupied){free[i].erase(k);s.uncertain_occupied.erase(k);}
      for(Key k:s.uncertain_occupied)free[i].erase(k);
    }
    const auto baseline=referenceEvidence(fixture,prepared,active);
    const auto fast=fixture.buildEvidence(prepared,active);
    const auto common=fusion_snapshot::sortedKeys(fast.common_free);
    compare(fixture.classifyEvidence(fast,prepared,std::nullopt),referenceClassify(fixture,baseline,prepared,std::nullopt));
    for(size_t observer=0;observer<count;++observer){
      const auto expected=referenceClassify(fixture,baseline,prepared,observer);
      compare(fixture.classifyEvidence(fast,prepared,observer),expected);
      const auto sparse=fixture.classifyEvidence(fast,prepared,observer,false);
      for(Vec3 translation:std::vector<Vec3>{{0,0,0},{12.5,9,-.2},{-12.5,-17,.2},{.09,0,0},{-3.6,1.8,-.54}}){
        tf2::Transform transform;transform.position={translation.x,translation.y,translation.z};
        const auto shift=fixture.snapshotTranslation(transform);
        if(!shift){assert(translation.x==.09);continue;}
        const auto occupied=fixture.transformKeys(expected.occupied,transform);
        const auto free_transformed=fixture.transformKeys(expected.free,transform);
        const auto packet=fixture.sparseSnapshot(sparse,common,*shift,"test",{1,0});
        assert(packet.data==map_snapshot::encode(occupied,free_transformed,.18));
        assert(packet.width*packet.point_step==packet.row_step&&packet.row_step==packet.data.size());
        map_snapshot::validate(packet.data,.18);
      }
    }
  }
  assert(!fusion_snapshot::translationShift({INFINITY,0,0},.18));
  assert(!fusion_snapshot::translationShift({0,0,0},0));
  assert(!fusion_snapshot::translationShift({.09,0,0},.18));
  bool overflow=false;
  try{fusion_snapshot::encode({{INT32_MAX,0,0}},{},{},.18,{1,0,0});}catch(const std::runtime_error&){overflow=true;}
  assert(overflow);
}
void actualPublishEquivalence(){
  for(size_t mode=0;mode<4;++mode){
    Fixture fixture;fixture.shared_snapshot_pub_->subscribers=1;
    fixture.publish_free_clouds_=mode==3;
    fixture.compact_maps_=mode!=2;
    std::vector<Fixture::PreparedSource>prepared(4);
    for(size_t i=0;i<4;++i){
      auto source=std::make_unique<Fixture::Source>();source->name="scout"+std::to_string(i+1);source->frame=source->name+"/odom";
      source->local_tf.position={12.5,double(i)*8-9,-.2};
      if(mode==1&&i==1)source->local_tf.rotation.yaw=.32;
      if(mode==1&&i==2)source->local_tf.position.a=.09;
      if(mode==3)source->fused_free_pub->subscribers=1;
      source->fused_occupied_pub->subscribers=1;
      for(int x=-20;x<30;++x)for(int y=-5;y<5;++y)source->free_world.insert({x+int(i),y,8});
      source->occupied_world={{int(i)*2,0,8},{30,2,8}};
      for(Key k:source->occupied_world)source->free_world.erase(k);
      prepared[i].occupied=source->occupied_world;prepared[i].free=&source->free_world;
      prepared[i].freshness=fixture.confidenceFreshness(*source,1200000000LL);
      fixture.sources_.push_back(std::move(source));
    }
    const std::vector<bool>active(4,true);const auto baseline=referenceEvidence(fixture,prepared,active);
    fixture.publish(active,1200000000LL);
    assert(!fixture.dirty_);
    for(size_t i=0;i<4;++i){
      const auto result=referenceClassify(fixture,baseline,prepared,i);const auto &source=*fixture.sources_[i];
      assert(source.last_output_stamp_ns==source.stamp_ns&&source.fused_snapshot_pub->messages.size()==1);
      const auto occupied=fixture.transformKeys(result.occupied,source.local_tf),free=fixture.transformKeys(result.free,source.local_tf);
      assert(source.fused_snapshot_pub->messages.back().data==map_snapshot::encode(occupied,free,.18));
      assert(source.fused_occupied_pub->messages.size()==1);
      assert(source.fused_free_pub->messages.size()==size_t(mode>=2));
    }
    const auto shared=referenceClassify(fixture,baseline,prepared,std::nullopt);
    assert(fixture.shared_snapshot_pub_->messages.back().data==map_snapshot::encode(shared.occupied,shared.free,.18));
    assert(fixture.status_pub_->message.find("fast_snapshots="+std::to_string(mode>=2?0:mode==1?2:4))!=std::string::npos);
    fixture.publish(active,1200000000LL);
    for(const auto &source:fixture.sources_)assert(source->fused_snapshot_pub->messages.size()==1);
  }
}
void benchmark(){
  Fixture fixture;std::vector<Fixture::PreparedSource>prepared(4);std::vector<KeySet>free(4);
  const size_t cells=1000000;
  for(size_t i=0;i<4;++i){
    prepared[i].free=&free[i];free[i].reserve(cells);
    for(size_t n=0;n<cells;++n){
      const Key k{int(n%400)-200+int(i)*5,int((n/400)%250)-125,int(n/100000)};
      if(n%149==0)prepared[i].occupied.insert(k);else free[i].insert(k);
    }
  }
  const std::vector<bool>active(4,true);size_t baseline_bytes=0,fast_bytes=0;
  const auto begin=std::chrono::steady_clock::now();
  {
    const auto evidence=referenceEvidence(fixture,prepared,active);
    for(size_t i=0;i<4;++i){
      auto result=referenceClassify(fixture,evidence,prepared,i);
      tf2::Transform tf;tf.position={12.5,double(i)*8-9,-.2};
      auto occupied=fixture.transformKeys(result.occupied,tf),free_local=fixture.transformKeys(result.free,tf);
      baseline_bytes+=map_snapshot::encode(occupied,free_local,.18).size();
    }
  }
  const auto middle=std::chrono::steady_clock::now();
  {
    const auto evidence=fixture.buildEvidence(prepared,active);
    const auto common=fusion_snapshot::sortedKeys(evidence.common_free);
    for(size_t i=0;i<4;++i){
      const auto result=fixture.classifyEvidence(evidence,prepared,i,false);
      const auto shift=fusion_snapshot::translationShift({12.5,double(i)*8-9,-.2},.18);assert(shift);
      fast_bytes+=fixture.sparseSnapshot(result,common,*shift,"test",{1,0}).data.size();
    }
  }
  const auto end=std::chrono::steady_clock::now();assert(baseline_bytes==fast_bytes);
  const double baseline=std::chrono::duration<double>(middle-begin).count(),fast=std::chrono::duration<double>(end-middle).count();
  std::cout<<"fusion_benchmark: sources=4 local_cells="<<cells<<" baseline_ms="<<1000*baseline<<" optimized_ms="<<1000*fast
    <<" speedup="<<baseline/fast<<" bytes="<<fast_bytes<<" (synthetic; excludes ROS/DDS/Gazebo)\n";
}
int main(){
  randomizedEquivalence();actualPublishEquivalence();
  if(std::getenv("FIRE_SCOUT_FUSION_BENCHMARK"))benchmark();
  std::cout<<"v214_fusion_fast_test: PASS real build/classify/publish; randomized confidence and byte equivalence, four-UAV translation/rotation fallback, output stamp gating, overflow\n";
}
'''
with tempfile.TemporaryDirectory() as tmp:
    cpp = pathlib.Path(tmp) / "fusion_fast.cpp"
    binary = pathlib.Path(tmp) / "fusion_fast"
    cpp.write_text(code)
    flags = ["-std=c++17", "-O2", "-Wall", "-Wextra", "-Wpedantic", "-UNDEBUG"]
    if os.environ.get("FIRE_SCOUT_SANITIZE"):
        flags += ["-fsanitize=address,undefined", "-fno-omit-frame-pointer", "-g"]
    subprocess.run(["g++", *flags, "-I" + str(ROOT / "include"), str(cpp), "-o", str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
