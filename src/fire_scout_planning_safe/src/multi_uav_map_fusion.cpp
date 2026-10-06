#include "fire_scout/map_pair_buffer.hpp"
#include "fire_scout/confidence_fusion.hpp"
#include "fire_scout/peer_uav_filter.hpp"
#include "fire_scout/ros_utils.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

using namespace fire_scout;

class MultiUavMapFusion : public rclcpp::Node {
public:
  MultiUavMapFusion()
      : Node("multi_uav_map_fusion"), tf_(get_clock()), listener_(tf_) {
    declare_parameter<std::string>("runtime_version", "2.1.2-atomic-map");
    world_frame_ = declare_parameter<std::string>("world_frame", "world");
    resolution_ = declare_parameter("voxel_size", 0.15);
    source_timeout_ = declare_parameter("source_timeout", 5.0);
    tf_timeout_ = declare_parameter("tf_wait_timeout", 0.20);
    timestamp_reset_threshold_=declare_parameter("timestamp_reset_threshold",.50);
    const double publish_rate = declare_parameter("publish_rate", 1.0);
    const int64_t max_points =
        declare_parameter<int64_t>("max_points_per_cloud", 2000000);
    max_points_per_cloud_ = max_points > 0 ? static_cast<size_t>(max_points) : 0;
    peer_filter_enabled_ = declare_parameter("peer_filter_enabled", true);
    peer_filter_length_ = declare_parameter("peer_filter_length", 0.90);
    peer_filter_width_ = declare_parameter("peer_filter_width", 0.90);
    peer_filter_height_ = declare_parameter("peer_filter_height", 0.50);
    peer_pose_timeout_ = declare_parameter("peer_filter_pose_timeout", 0.80);
    peer_history_config_.duration =
        declare_parameter("peer_filter_history_duration", 0.80);
    peer_history_config_.position_step =
        declare_parameter("peer_filter_history_step", 0.15);
    peer_history_config_.yaw_step =
        declare_parameter("peer_filter_history_yaw_step_deg", 15.0) * pi / 180.0;
    const int64_t peer_history_max_boxes =
        declare_parameter<int64_t>("peer_filter_history_max_boxes", 12);
    peer_history_config_.max_boxes =
        peer_history_max_boxes > 0 ? static_cast<size_t>(peer_history_max_boxes) : 0;
    self_map_confidence_ = declare_parameter("self_map_confidence", 1.0);
    peer_map_confidence_ = declare_parameter("peer_map_confidence", 0.45);
    history_map_confidence_ = declare_parameter("history_map_confidence", 0.20);
    confidence_decay_time_ =
        declare_parameter("peer_obstacle_decay_time", 1.50);
    dynamic_obstacle_confidence_ =
        declare_parameter("peer_body_obstacle_confidence", 0.05);
    minimum_occupied_confidence_ =
        declare_parameter("minimum_occupied_confidence", 0.20);
    occupied_conflict_bias_ = declare_parameter("occupied_conflict_bias", 1.05);
    self_observation_priority_ =
        declare_parameter("self_observation_priority", true);

    const auto names = declare_parameter<std::vector<std::string>>(
        "scout_names", {"scout1", "scout2", "scout3", "scout4"});
    const auto frames = declare_parameter<std::vector<std::string>>(
        "source_frames", {"scout1/odom", "scout2/odom", "scout3/odom", "scout4/odom"});
    const auto occupied_topics = declare_parameter<std::vector<std::string>>(
        "occupied_topics", {"/scout1/map/radar_occupied", "/scout2/map/radar_occupied",
                            "/scout3/map/radar_occupied", "/scout4/map/radar_occupied"});
    const auto free_topics = declare_parameter<std::vector<std::string>>(
        "free_topics", {"/scout1/map/radar_free", "/scout2/map/radar_free",
                        "/scout3/map/radar_free", "/scout4/map/radar_free"});
    const auto odom_topics = declare_parameter<std::vector<std::string>>(
        "odom_topics", {"/scout1/odom", "/scout2/odom", "/scout3/odom",
                        "/scout4/odom"});
    const auto fused_occupied_topics = declare_parameter<std::vector<std::string>>(
        "fused_occupied_topics",
        {"/scout1/map/fused_occupied", "/scout2/map/fused_occupied",
         "/scout3/map/fused_occupied", "/scout4/map/fused_occupied"});
    const auto fused_free_topics = declare_parameter<std::vector<std::string>>(
        "fused_free_topics", {"/scout1/map/fused_free", "/scout2/map/fused_free",
                              "/scout3/map/fused_free", "/scout4/map/fused_free"});
    compact_maps_=compactMapTransport(*this);
    publish_free_clouds_=declare_parameter("publish_free_clouds",false);
    const auto snapshot_topics=declare_parameter<std::vector<std::string>>(
      "source_snapshot_topics",{"/scout1/map/radar_snapshot","/scout2/map/radar_snapshot",
                                "/scout3/map/radar_snapshot","/scout4/map/radar_snapshot"});
    const auto fused_snapshot_topics=declare_parameter<std::vector<std::string>>(
      "fused_snapshot_topics",{"/scout1/map/fused_snapshot","/scout2/map/fused_snapshot",
                               "/scout3/map/fused_snapshot","/scout4/map/fused_snapshot"});

    if (world_frame_.empty() || !std::isfinite(resolution_) || resolution_ < 0.05 ||
        resolution_ > 0.50 || !std::isfinite(source_timeout_) || source_timeout_ <= 0 ||
        !std::isfinite(tf_timeout_) || tf_timeout_ < 0 || tf_timeout_ > 2.0 ||
        !std::isfinite(timestamp_reset_threshold_)||timestamp_reset_threshold_<.10||
        !std::isfinite(publish_rate) || publish_rate <= 0 || publish_rate > 20 ||
        max_points_per_cloud_ == 0 || !std::isfinite(peer_filter_length_) ||
        !std::isfinite(peer_filter_width_) || !std::isfinite(peer_filter_height_) ||
        !std::isfinite(peer_pose_timeout_) || peer_filter_length_ <= 0 ||
        peer_filter_width_ <= 0 || peer_filter_height_ <= 0 ||
        peer_filter_length_ > 5.0 || peer_filter_width_ > 5.0 ||
        peer_filter_height_ > 5.0 || peer_pose_timeout_ <= 0 ||
        peer_pose_timeout_ > 5.0 || !std::isfinite(self_map_confidence_) ||
        !std::isfinite(peer_map_confidence_) || self_map_confidence_ <= 0 ||
        peer_map_confidence_ < 0 || self_map_confidence_ < peer_map_confidence_ ||
        !std::isfinite(history_map_confidence_) || history_map_confidence_ < 0 ||
        history_map_confidence_ > 1 || !std::isfinite(confidence_decay_time_) ||
        confidence_decay_time_ <= 0 || !std::isfinite(dynamic_obstacle_confidence_) ||
        dynamic_obstacle_confidence_ < 0 || dynamic_obstacle_confidence_ > 1 ||
        !std::isfinite(minimum_occupied_confidence_) ||
        minimum_occupied_confidence_ <= 0 ||
        !std::isfinite(occupied_conflict_bias_) || occupied_conflict_bias_ <= 0 ||
        !peer_history_config_.valid() ||
        names.empty())
      throw std::runtime_error("Invalid multi-UAV map fusion parameters");

    const size_t count = names.size();
    if (frames.size() != count || occupied_topics.size() != count ||
        free_topics.size() != count || odom_topics.size() != count ||
        fused_occupied_topics.size() != count || fused_free_topics.size() != count ||
        (compact_maps_&&(snapshot_topics.size()!=count||fused_snapshot_topics.size()!=count)))
      throw std::runtime_error("All multi-UAV fusion string arrays must have scout_names length");

    std::unordered_set<std::string> unique_names, unique_frames, unique_inputs, unique_outputs;
    sources_.reserve(count);
    const auto input_qos = rclcpp::QoS(compact_maps_?1:4).reliable().transient_local();
    const auto output_qos = rclcpp::QoS(1).reliable().transient_local();
    for (size_t i = 0; i < count; ++i) {
      if (names[i].empty() || frames[i].empty() || occupied_topics[i].empty() ||
          free_topics[i].empty() || odom_topics[i].empty() ||
          fused_occupied_topics[i].empty() ||
          fused_free_topics[i].empty() || occupied_topics[i] == free_topics[i] ||
          fused_occupied_topics[i] == fused_free_topics[i] ||
          !unique_names.insert(names[i]).second || !unique_frames.insert(frames[i]).second ||
          !unique_inputs.insert(occupied_topics[i]).second ||
          !unique_inputs.insert(free_topics[i]).second ||
          !unique_inputs.insert(odom_topics[i]).second ||
          !unique_outputs.insert(fused_occupied_topics[i]).second ||
          !unique_outputs.insert(fused_free_topics[i]).second)
        throw std::runtime_error("Fusion names, frames and topics must be non-empty and unique");

      auto source = std::make_unique<Source>();
      source->name = names[i];
      source->frame = frames[i];
      source->occupied_topic = occupied_topics[i];
      source->free_topic = free_topics[i];
      source->odom_topic = odom_topics[i];
      source->peer_history = PeerUavBoxHistory(peer_history_config_);
      source->fused_occupied_pub = create_publisher<sensor_msgs::msg::PointCloud2>(
          fused_occupied_topics[i], output_qos);
      source->fused_free_pub =
          create_publisher<sensor_msgs::msg::PointCloud2>(fused_free_topics[i], output_qos);
      if(compact_maps_){
        if(snapshot_topics[i].empty()||fused_snapshot_topics[i].empty() ||
           !unique_inputs.insert(snapshot_topics[i]).second ||
           !unique_outputs.insert(fused_snapshot_topics[i]).second)
          throw std::runtime_error("Snapshot topics must be non-empty and unique");
        source->snapshot_topic=snapshot_topics[i];
        source->fused_snapshot_pub=create_publisher<Cloud>(fused_snapshot_topics[i],output_qos);
      }
      sources_.push_back(std::move(source));
    }
    for (const auto &topic : unique_outputs)
      if (unique_inputs.count(topic))
        throw std::runtime_error("A fused output topic must not also be a local map input topic");

    for (size_t i = 0; i < sources_.size(); ++i) {
      auto &source = *sources_[i];
      if(compact_maps_){
        source.snapshot_sub=create_subscription<Cloud>(source.snapshot_topic,input_qos,
          [this,i](Cloud::SharedPtr msg){
            auto &s=*sources_.at(i);const auto stamp=stampNs(msg->header.stamp);
            if(stamp<=0 || msg->header.frame_id!=s.frame){++s.rejected;return;}
            observeSourceStamp(i,stamp,s.latest_snapshot_input_ns,"SNAPSHOT_STAMP_REWIND");
            ++s.received;
            if(stamp<=s.stamp_ns || (s.snapshot_pending&&stamp<=stampNs(s.snapshot_pending->header.stamp))){
              ++s.coalesced;return;
            }
            if(s.snapshot_pending)++s.coalesced;
            // The callback retains only the latest complete packet. Expensive
            // decode/TF/hash work happens once per fusion tick, not at every
            // mapper rate on the single executor thread.
            s.snapshot_pending=std::move(msg);
          });
      }else{
      source.occupied_sub = create_subscription<sensor_msgs::msg::PointCloud2>(
          source.occupied_topic, input_qos,
          [this, i](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            const int64_t stamp = stampNs(msg->header.stamp);
            if (stamp <= 0) return;
            observeSourceStamp(i,stamp,sources_[i]->latest_occupied_input_ns,
                               "OCCUPIED_STAMP_REWIND");
            sources_[i]->pairs.putOccupied(stamp, std::move(msg));
          });
      source.free_sub = create_subscription<sensor_msgs::msg::PointCloud2>(
          source.free_topic, input_qos,
          [this, i](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            const int64_t stamp = stampNs(msg->header.stamp);
            if (stamp <= 0) return;
            observeSourceStamp(i,stamp,sources_[i]->latest_free_input_ns,
                               "FREE_STAMP_REWIND");
            sources_[i]->pairs.putFree(stamp, std::move(msg));
          });
      }
      source.odom_sub = create_subscription<nav_msgs::msg::Odometry>(
          source.odom_topic, rclcpp::SensorDataQoS(),
          [this, i](nav_msgs::msg::Odometry::SharedPtr msg) {
            receiveOdom(i, *msg);
          });
    }

    shared_occupied_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        declare_parameter<std::string>("shared_occupied_topic",
                                       "/fire_scout/map/fused_occupied"),
        output_qos);
    shared_free_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        declare_parameter<std::string>("shared_free_topic", "/fire_scout/map/fused_free"),
        output_qos);
    if(compact_maps_)shared_snapshot_pub_=create_publisher<Cloud>(
      declare_parameter<std::string>("shared_snapshot_topic","/fire_scout/map/fused_snapshot"),output_qos);
    status_pub_ = create_publisher<std_msgs::msg::String>(
        declare_parameter<std::string>("status_topic", "/fire_scout/map/fusion_status"),
        rclcpp::QoS(1).reliable().transient_local());

    reset_ = create_service<std_srvs::srv::Trigger>(
        "~/reset", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          for (auto &source : sources_) {
            clearSource(*source);
          }
          last_active_.assign(sources_.size(), false);
          dirty_ = true;
          response->success = true;
          response->message = "All cached multi-UAV map snapshots were cleared.";
        });

    last_active_.assign(sources_.size(), false);
    timer_ = create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate)),
                               [this] { tick(); });
    config_lock_ = lockParameters(*this);
    RCLCPP_INFO(get_logger(),
                "V2.1.2 multi-UAV map fusion ready: sources=%zu world=%s voxel=%.2f m "
                "timeout=%.2f s peer_filter=%s box=%.2fx%.2fx%.2f m "
                "trail=%.2f s/%.2f m/%zu "
                "confidence=self:%.2f peer:%.2f history:%.2f",
                sources_.size(), world_frame_.c_str(), resolution_, source_timeout_,
                peer_filter_enabled_ ? "on" : "off", peer_filter_length_,
                peer_filter_width_, peer_filter_height_, peer_history_config_.duration,
                peer_history_config_.position_step, peer_history_config_.max_boxes,
                self_map_confidence_,
                peer_map_confidence_, history_map_confidence_);
  }

private:
  using Cloud = sensor_msgs::msg::PointCloud2;
  using CloudPtr = Cloud::SharedPtr;
  using PairBuffer = MapPairBuffer<CloudPtr>;

  struct Source {
    std::string name;
    std::string frame;
    std::string occupied_topic;
    std::string free_topic;
    std::string odom_topic;
    std::string snapshot_topic;
    CloudPtr snapshot_pending;
    int64_t latest_snapshot_input_ns{-1};
    size_t received{0},coalesced{0},rejected{0},commits{0},input_bytes{0},output_bytes{0};
    PairBuffer pairs{4};
    std::optional<PairBuffer::Pair> pending;
    KeySet occupied_world;
    KeySet free_world;
    int64_t stamp_ns{-1};
    int64_t last_output_stamp_ns{-1};
    int64_t latest_occupied_input_ns{-1};
    int64_t latest_free_input_ns{-1};
    PeerUavBox peer_box;
    PeerUavBoxHistory peer_history;
    int64_t pose_ns{-1};
    bool ready{false};
    bool have_pose{false};
    rclcpp::Subscription<Cloud>::SharedPtr occupied_sub;
    rclcpp::Subscription<Cloud>::SharedPtr free_sub;
    rclcpp::Subscription<Cloud>::SharedPtr snapshot_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Publisher<Cloud>::SharedPtr fused_occupied_pub;
    rclcpp::Publisher<Cloud>::SharedPtr fused_free_pub;
    rclcpp::Publisher<Cloud>::SharedPtr fused_snapshot_pub;
  };

  void clearSource(Source &source){
    source.pairs.clear();source.pending.reset();source.occupied_world.clear();
    source.snapshot_pending.reset();source.latest_snapshot_input_ns=-1;
    source.free_world.clear();source.stamp_ns=-1;source.last_output_stamp_ns=-1;
    source.latest_occupied_input_ns=source.latest_free_input_ns=-1;
    source.ready=false;source.have_pose=false;source.pose_ns=-1;source.peer_history.clear();
  }
  void observeSourceStamp(size_t index,int64_t stamp,int64_t &latest,const char *reason){
    const int64_t threshold=int64_t(timestamp_reset_threshold_*1e9);
    if(latest>0&&stamp<latest-threshold){
      auto &source=*sources_.at(index);clearSource(source);dirty_=true;
      RCLCPP_WARN(get_logger(),"FUSION_SOURCE_TIME_RESET scout=%s reason=%s",
                  source.name.c_str(),reason);
    }
    latest=std::max(latest,stamp);
  }

  static tf2::Transform tf2Transform(const geometry_msgs::msg::TransformStamped &msg) {
    const auto &q = msg.transform.rotation;
    tf2::Quaternion rotation(q.x, q.y, q.z, q.w);
    if (rotation.length2() < 1e-12) throw std::runtime_error("Transform has invalid quaternion");
    rotation.normalize();
    const auto &t = msg.transform.translation;
    return tf2::Transform(rotation, tf2::Vector3(t.x, t.y, t.z));
  }

  geometry_msgs::msg::TransformStamped lookup(const std::string &target,
                                               const std::string &source,
                                               const builtin_interfaces::msg::Time &stamp) {
    if (target == source) {
      geometry_msgs::msg::TransformStamped identity;
      identity.header.frame_id = target;
      identity.child_frame_id = source;
      identity.header.stamp = stamp;
      identity.transform.rotation.w = 1.0;
      return identity;
    }
    return tf_.lookupTransform(target, source,
                               rclcpp::Time(stamp, get_clock()->get_clock_type()),
                               rclcpp::Duration::from_seconds(tf_timeout_));
  }

  KeySet transformCloud(const Cloud &cloud, const tf2::Transform &transform) const {
    const uint64_t count = uint64_t(cloud.width) * uint64_t(cloud.height);
    if (count > max_points_per_cloud_)
      throw std::runtime_error("Point cloud exceeds max_points_per_cloud");
    KeySet output;
    output.reserve(static_cast<size_t>(count));
    sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
    for (; x != x.end(); ++x, ++y, ++z) {
      const Vec3 point{*x, *y, *z};
      if (!finite(point)) continue;
      const auto transformed = transform * tf2::Vector3(point.x, point.y, point.z);
      const Vec3 target{transformed.x(), transformed.y(), transformed.z()};
      if (finite(target)) output.insert(key(target, resolution_));
    }
    return output;
  }

  KeySet transformKeys(const KeySet &input, const tf2::Transform &transform) const {
    KeySet output;
    output.reserve(input.size());
    for (const Key voxel : input) {
      const Vec3 point = center(voxel, resolution_);
      const auto transformed = transform * tf2::Vector3(point.x, point.y, point.z);
      const Vec3 target{transformed.x(), transformed.y(), transformed.z()};
      if (finite(target)) output.insert(key(target, resolution_));
    }
    return output;
  }

  void receiveOdom(size_t index, const nav_msgs::msg::Odometry &msg) {
    if (msg.header.frame_id.empty()) return;
    const auto &p = msg.pose.pose.position;
    const auto &q = msg.pose.pose.orientation;
    const Vec3 local_position{p.x, p.y, p.z};
    tf2::Quaternion local_rotation(q.x, q.y, q.z, q.w);
    if (!finite(local_position) || local_rotation.length2() < 1e-12) return;
    local_rotation.normalize();
    try {
      const auto world_from_frame =
          tf2Transform(lookup(world_frame_, msg.header.frame_id, msg.header.stamp));
      const tf2::Transform frame_from_body(
          local_rotation,
          tf2::Vector3(local_position.x, local_position.y, local_position.z));
      const tf2::Transform world_from_body = world_from_frame * frame_from_body;
      const auto translation = world_from_body.getOrigin();
      const auto rotation = world_from_body.getRotation();
      const double yaw = std::atan2(
          2.0 * (rotation.w() * rotation.z() + rotation.x() * rotation.y()),
          1.0 - 2.0 * (rotation.y() * rotation.y() +
                       rotation.z() * rotation.z()));
      PeerUavBox box{{translation.x(), translation.y(), translation.z()}, yaw,
                     peer_filter_length_, peer_filter_width_, peer_filter_height_};
      if (!box.valid()) return;
      auto &source = *sources_.at(index);
      source.peer_box = box;
      source.pose_ns = now().nanoseconds();
      source.peer_history.add(box, double(source.pose_ns) * 1e-9);
      source.have_pose = true;
      dirty_ = true;
    } catch (const tf2::TransformException &error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                           "Waiting for %s odometry transform into %s: %s",
                           sources_.at(index)->name.c_str(), world_frame_.c_str(),
                           error.what());
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000,
                            "Rejected %s odometry: %s",
                            sources_.at(index)->name.c_str(), error.what());
    }
  }

  void drain(size_t index) {
    auto &source = *sources_.at(index);
    if(compact_maps_){
      if(!source.snapshot_pending)return;
      const auto &m=*source.snapshot_pending;
      try{
        const auto transform=tf2Transform(lookup(world_frame_,m.header.frame_id,m.header.stamp));
        auto decoded=parseMapSnapshot(m,resolution_,max_points_per_cloud_);
        auto occupied=transformKeys(decoded.occupied,transform);
        auto free=transformKeys(decoded.free,transform);
        for(Key k:occupied)free.erase(k);
        source.occupied_world=std::move(occupied);source.free_world=std::move(free);
        source.stamp_ns=stampNs(m.header.stamp);source.ready=true;source.input_bytes=m.data.size();
        ++source.commits;source.snapshot_pending.reset();dirty_=true;
      }catch(const tf2::TransformException &e){
        RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),3000,
          "Waiting for %s compact map TF: %s",source.name.c_str(),e.what());
      }catch(const std::runtime_error &e){
        ++source.rejected;source.snapshot_pending.reset();
        RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),3000,
          "Rejected %s compact map: %s",source.name.c_str(),e.what());
      }
      return;
    }
    auto newer=source.pairs.takeNewestPair();
    if(newer&&(!source.pending||newer->stamp>source.pending->stamp))source.pending=std::move(newer);
    if (!source.pending) return;
    const auto &pair = *source.pending;
    if (!pair.occupied || !pair.free || pair.occupied->header.frame_id.empty() ||
        pair.free->header.frame_id.empty()) {
      source.pending.reset();
      return;
    }
    if (pair.occupied->header.frame_id != source.frame ||
        pair.free->header.frame_id != source.frame) {
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "Rejected %s map pair: expected frame %s, got occupied=%s free=%s",
          source.name.c_str(), source.frame.c_str(), pair.occupied->header.frame_id.c_str(),
          pair.free->header.frame_id.c_str());
      source.pending.reset();
      return;
    }
    try {
      const auto occupied_tf =
          tf2Transform(lookup(world_frame_, pair.occupied->header.frame_id,
                              pair.occupied->header.stamp));
      const auto free_tf = tf2Transform(
          lookup(world_frame_, pair.free->header.frame_id, pair.free->header.stamp));
      auto occupied = transformCloud(*pair.occupied, occupied_tf);
      auto free = transformCloud(*pair.free, free_tf);
      for (const Key voxel : occupied) free.erase(voxel);
      source.occupied_world = std::move(occupied);
      source.free_world = std::move(free);
      source.stamp_ns = pair.stamp;
      source.ready = true;
      ++source.commits;
      source.pending.reset();
      dirty_ = true;
    } catch (const tf2::TransformException &error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                           "Waiting for %s map transform into %s: %s", source.name.c_str(),
                           world_frame_.c_str(), error.what());
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(get_logger(), "Rejected %s map pair: %s", source.name.c_str(), error.what());
      source.pending.reset();
    }
  }

  bool active(const Source &source, int64_t now_ns) const {
    if (!source.ready || source.stamp_ns <= 0) return false;
    const double age = double(now_ns - source.stamp_ns) * 1e-9;
    return age >= -0.10 && age <= source_timeout_;
  }

  bool freshPeerPose(const Source &source, int64_t now_ns) const {
    if (!source.have_pose || source.pose_ns <= 0 || now_ns < source.pose_ns)
      return false;
    return double(now_ns - source.pose_ns) * 1e-9 <= peer_pose_timeout_;
  }

  std::vector<PeerUavBox> peerBoxes(size_t observer,
                                     int64_t now_ns) {
    std::vector<PeerUavBox> boxes;
    if (!peer_filter_enabled_) return boxes;
    boxes.reserve(sources_.size() > 0 ? sources_.size() - 1 : 0);
    for (size_t peer = 0; peer < sources_.size(); ++peer) {
      // Never remove data around the observer itself.  Its own map is the best
      // available source for nearby walls and other static structure.
      if (peer == observer || !freshPeerPose(*sources_[peer], now_ns)) continue;
      const auto &current = sources_[peer]->peer_box;
      boxes.push_back(current);
      for (const auto &historic :
           sources_[peer]->peer_history.boxes(double(now_ns) * 1e-9)) {
        const double dx = historic.center.x - current.center.x;
        const double dy = historic.center.y - current.center.y;
        const double dz = historic.center.z - current.center.z;
        if (std::sqrt(dx * dx + dy * dy + dz * dz) <= 0.5 * resolution_ &&
            std::abs(wrap(historic.yaw - current.yaw)) <= pi / 180.0)
          continue;
        boxes.push_back(historic);
      }
    }
    return boxes;
  }

  static builtin_interfaces::msg::Time timeMessage(int64_t stamp_ns) {
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<int32_t>(stamp_ns / 1000000000LL);
    stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1000000000LL);
    return stamp;
  }

  struct PreparedSource {
    KeySet occupied;
    KeySet uncertain_occupied;
    const KeySet *free{nullptr};
    double freshness{1.0};
  };

  struct WeightedEvidence {
    double occupied{0};
    double free{0};
  };

  struct FusionResult {
    KeySet occupied;
    KeySet free;
    size_t conflicts{0};
    size_t self_overrides{0};
  };

  double confidenceFreshness(const Source &source, int64_t now_ns) const {
    const double age = std::max(0.0, double(now_ns - source.stamp_ns) * 1e-9);
    return std::clamp(
        std::max(history_map_confidence_, std::exp(-age / confidence_decay_time_)),
        0.0, 1.0);
  }

  std::unordered_map<Key, WeightedEvidence, KeyHash> buildEvidence(
      const std::vector<PreparedSource> &prepared,
      const std::vector<bool> &active_sources) const {
    size_t reserve = 0;
    for (size_t i = 0; i < prepared.size(); ++i)
      if (active_sources[i])
        reserve = std::max(reserve, prepared[i].occupied.size() +
                                        prepared[i].uncertain_occupied.size() +
                                        (prepared[i].free ? prepared[i].free->size() : 0));
    std::unordered_map<Key, WeightedEvidence, KeyHash> evidence;
    evidence.reserve(reserve);
    for (size_t i = 0; i < prepared.size(); ++i) {
      if (!active_sources[i] || !prepared[i].free) continue;
      const double normal = peer_map_confidence_ * prepared[i].freshness;
      const double uncertain = normal * dynamic_obstacle_confidence_;
      for (const Key voxel : prepared[i].occupied)
        evidence[voxel].occupied += normal;
      for (const Key voxel : prepared[i].uncertain_occupied)
        evidence[voxel].occupied += uncertain;
      for (const Key voxel : *prepared[i].free)
        evidence[voxel].free += normal;
    }
    return evidence;
  }

  FusionResult classifyEvidence(
      const std::unordered_map<Key, WeightedEvidence, KeyHash> &evidence,
      const std::vector<PreparedSource> &prepared,
      std::optional<size_t> observer) const {
    FusionResult result;
    result.occupied.reserve(evidence.size() / 4);
    result.free.reserve(evidence.size());
    const ConfidenceDecisionConfig decision_config{
        minimum_occupied_confidence_, occupied_conflict_bias_,
        self_observation_priority_};
    for (const auto &[voxel, base] : evidence) {
      double occupied = base.occupied;
      double free = base.free;
      bool self_occupied = false, self_free = false;
      if (observer) {
        const auto &own = prepared.at(*observer);
        const double upgrade =
            (self_map_confidence_ - peer_map_confidence_) * own.freshness;
        self_occupied = own.occupied.count(voxel) != 0;
        const bool self_uncertain = own.uncertain_occupied.count(voxel) != 0;
        self_free = own.free && own.free->count(voxel) != 0;
        if (self_occupied) occupied += upgrade;
        if (self_uncertain)
          occupied += upgrade * dynamic_obstacle_confidence_;
        if (self_free) free += upgrade;
      }

      if (occupied > 0 && free > 0) ++result.conflicts;
      const auto state = decideFusionState(occupied, free, self_occupied,
                                           self_free, decision_config);
      if (state == FusionState::Occupied) result.occupied.insert(voxel);
      if (state == FusionState::Free) result.free.insert(voxel);
      if (observer && self_observation_priority_ &&
          ((self_occupied && free > 0 && state == FusionState::Occupied) ||
           (self_free && occupied >= minimum_occupied_confidence_ &&
            state == FusionState::Free)))
        ++result.self_overrides;
    }
    return result;
  }

  void tick() {
    for (size_t i = 0; i < sources_.size(); ++i) drain(i);
    const int64_t now_ns = now().nanoseconds();
    std::vector<bool> current_active(sources_.size(), false);
    for (size_t i = 0; i < sources_.size(); ++i)
      current_active[i] = active(*sources_[i], now_ns);
    if (current_active != last_active_) {
      dirty_ = true;
      last_active_ = current_active;
    }
    if (!dirty_) return;
    publish(current_active, now_ns);
  }

  void publish(const std::vector<bool> &active_sources, int64_t now_ns) {
    std::vector<PreparedSource> prepared(sources_.size());
    size_t active_count = 0, fresh_pose_count = 0;
    size_t uncertain_occupied = 0, applied_boxes = 0;
    int64_t newest_stamp = -1;
    for (size_t i = 0; i < sources_.size(); ++i) {
      fresh_pose_count += freshPeerPose(*sources_[i], now_ns);
      if (!active_sources[i]) continue;
      ++active_count;
      newest_stamp = std::max(newest_stamp, sources_[i]->stamp_ns);
      prepared[i].occupied = sources_[i]->occupied_world;
      prepared[i].free = &sources_[i]->free_world;
      prepared[i].freshness = confidenceFreshness(*sources_[i], now_ns);
      const auto boxes = peerBoxes(i, now_ns);
      applied_boxes += boxes.size();
      uncertain_occupied += extractPeerUavBoxes(
          prepared[i].occupied, prepared[i].uncertain_occupied, resolution_, boxes);
    }

    // Build the cross-source evidence table once.  Each UAV then gets a
    // personalized classification in which its own current observation has
    // higher confidence and, when configured, explicit priority over peers.
    const auto evidence = buildEvidence(prepared, active_sources);
    auto shared = classifyEvidence(evidence, prepared, std::nullopt);

    // When all sources time out, keep the last transient-local visualization
    // instead of replacing it with an empty cloud. No new message is sent, so
    // the coordinator still marks the map stale from reception time and cannot
    // mistake the retained display for current evidence.
    if (active_count > 0) {
      const auto shared_stamp = timeMessage(std::max<int64_t>(0, newest_stamp));
      shared_occupied_pub_->publish(
          makeCloud(shared.occupied, resolution_, world_frame_, shared_stamp));
      if(!compact_maps_ || (publish_free_clouds_&&shared_free_pub_->get_subscription_count()>0))
        shared_free_pub_->publish(makeCloud(shared.free,resolution_,world_frame_,shared_stamp));
      if(shared_snapshot_pub_&&shared_snapshot_pub_->get_subscription_count()>0)
        shared_snapshot_pub_->publish(makeMapSnapshot(shared.occupied,shared.free,resolution_,world_frame_,shared_stamp));
    }

    size_t local_conflicts = 0, self_overrides = 0;
    for (size_t i = 0; i < sources_.size(); ++i) {
      auto &source = *sources_[i];
      // A planner output is fresh only when that UAV's own mapper is fresh. Remote
      // traffic therefore cannot conceal a failed local radar/mapping chain.
      if (!active_sources[i] || source.stamp_ns <= source.last_output_stamp_ns) continue;
      try {
        auto personalized = classifyEvidence(evidence, prepared, i);
        local_conflicts += personalized.conflicts;
        self_overrides += personalized.self_overrides;
        const auto stamp = timeMessage(source.stamp_ns);
        const auto local_tf = tf2Transform(lookup(source.frame, world_frame_, stamp));
        auto occupied_local = transformKeys(personalized.occupied, local_tf);
        auto free_local = transformKeys(personalized.free, local_tf);
        if(source.fused_snapshot_pub){
          auto packet=makeMapSnapshot(occupied_local,free_local,resolution_,source.frame,stamp);
          source.output_bytes=packet.data.size();source.fused_snapshot_pub->publish(std::move(packet));
        }
        if(!compact_maps_||source.fused_occupied_pub->get_subscription_count()>0)
          source.fused_occupied_pub->publish(makeCloud(occupied_local,resolution_,source.frame,stamp));
        if(!compact_maps_||(publish_free_clouds_&&source.fused_free_pub->get_subscription_count()>0))
          source.fused_free_pub->publish(makeCloud(free_local,resolution_,source.frame,stamp));
        source.last_output_stamp_ns = source.stamp_ns;
      } catch (const tf2::TransformException &error) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                             "Cannot publish fused map for %s: %s", source.name.c_str(),
                             error.what());
      } catch (const std::runtime_error &error) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000,
                              "Cannot transform fused map for %s: %s", source.name.c_str(),
                              error.what());
      }
    }

    size_t peer_history_samples = 0;
    for (const auto &source : sources_)
      peer_history_samples += source->peer_history.size();
    std::ostringstream status;
    status << "active=" << active_count << "/" << sources_.size()
           << " occupied=" << shared.occupied.size() << " free=" << shared.free.size()
           << " peer_filter=" << (peer_filter_enabled_ ? "on" : "off")
           << " peer_poses=" << fresh_pose_count << '/' << sources_.size()
           << " boxes=" << applied_boxes
           << " peer_trail_samples=" << peer_history_samples
           << " uncertain_occ=" << uncertain_occupied
           << " shared_conflicts=" << shared.conflicts
           << " local_conflicts=" << local_conflicts
           << " self_overrides=" << self_overrides
           << " confidence=" << self_map_confidence_ << '/' << peer_map_confidence_
           << " transport="<<(compact_maps_?"compact":"legacy")<<" sources=";
    for (size_t i = 0; i < sources_.size(); ++i) {
      if (i) status << ',';
      status << sources_[i]->name << ':' << (active_sources[i] ? "fresh" : "stale");
    }
    status<<" pipeline=";
    for(size_t i=0;i<sources_.size();++i){
      const auto &s=*sources_[i];if(i)status<<',';
      status<<s.name<<":age="<<(s.stamp_ns>0?double(now_ns-s.stamp_ns)*1e-9:-1.)
        <<"/input_age="<<(s.latest_snapshot_input_ns>0?double(now_ns-s.latest_snapshot_input_ns)*1e-9:-1.)
        <<"/rx="<<s.received<<"/commit="<<s.commits<<"/coalesced="<<s.coalesced
        <<"/rejected="<<s.rejected<<"/bytes="<<s.input_bytes<<'/'<<s.output_bytes;
      if(!compact_maps_)status<<"/occ_age="<<(s.latest_occupied_input_ns>0?double(now_ns-s.latest_occupied_input_ns)*1e-9:-1.)
        <<"/free_age="<<(s.latest_free_input_ns>0?double(now_ns-s.latest_free_input_ns)*1e-9:-1.)
        <<"/pair_queue="<<s.pairs.occupiedSize()<<'/'<<s.pairs.freeSize();
    }
    status_pub_->publish(textMessage(status.str()));
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "%s", status.str().c_str());
    dirty_ = false;
  }

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr config_lock_;
  std::string world_frame_;
  double resolution_{0.15};
  double source_timeout_{5.0};
  double tf_timeout_{0.20};
  double timestamp_reset_threshold_{.50};
  bool peer_filter_enabled_{true};
  double peer_filter_length_{0.90};
  double peer_filter_width_{0.90};
  double peer_filter_height_{0.50};
  double peer_pose_timeout_{0.80};
  PeerUavBoxHistoryConfig peer_history_config_{};
  double self_map_confidence_{1.0};
  double peer_map_confidence_{0.45};
  double history_map_confidence_{0.20};
  double confidence_decay_time_{1.50};
  double dynamic_obstacle_confidence_{0.05};
  double minimum_occupied_confidence_{0.20};
  double occupied_conflict_bias_{1.05};
  bool self_observation_priority_{true};
  size_t max_points_per_cloud_{2000000};
  std::vector<std::unique_ptr<Source>> sources_;
  std::vector<bool> last_active_;
  bool dirty_{true};
  tf2_ros::Buffer tf_;
  tf2_ros::TransformListener listener_;
  rclcpp::Publisher<Cloud>::SharedPtr shared_occupied_pub_;
  rclcpp::Publisher<Cloud>::SharedPtr shared_free_pub_;
  bool compact_maps_{true},publish_free_clouds_{false};
  rclcpp::Publisher<Cloud>::SharedPtr shared_snapshot_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MultiUavMapFusion>());
  rclcpp::shutdown();
  return 0;
}
