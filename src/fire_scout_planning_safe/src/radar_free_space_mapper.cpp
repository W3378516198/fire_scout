#include "fire_scout/ros_utils.hpp"
#include "fire_scout/ultrasonic.hpp"
#include "fire_scout/pose_history.hpp"
#include "fire_scout/observation_gate.hpp"
#include "fire_scout/snapshot_stamp.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <deque>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
using namespace fire_scout;
class RadarFreeSpaceMapper : public rclcpp::Node {
public:
  RadarFreeSpaceMapper() : Node("radar_free_space_mapper"), tf_(get_clock()), listener_(tf_) {
    declare_parameter<std::string>("runtime_version","2.1.2-atomic-map");
    frame_ = declare_parameter<std::string>("map_frame", "scout1/odom");
    prefix_ = declare_parameter<std::string>("frame_prefix", "scout1/");
    const double r = declare_parameter("voxel_size", .15);
    map_ = std::make_unique<EvidenceMap>(r);
    map_->z_min = declare_parameter("min_z", -.3);
    map_->z_max = declare_parameter("max_z", 4.5);
    map_->hit_score = declare_parameter("hit_score", 3);
    map_->miss_score = declare_parameter("miss_score", 1);
    map_->occupied_threshold = declare_parameter("occupied_threshold", 2);
    map_->min_score = declare_parameter("min_score", -4);
    map_->max_score = declare_parameter("max_score", 6);
    min_range_ = declare_parameter("min_range", .25);
    max_range_ = declare_parameter("max_range", 12.0);
    pending_timeout_ = declare_parameter("tf_wait_timeout", .25);
    double rate = declare_parameter("publish_rate", 3.0);
    if (!std::isfinite(r) || !std::isfinite(rate) || !std::isfinite(min_range_) || !std::isfinite(max_range_) ||
        !std::isfinite(pending_timeout_) || !std::isfinite(map_->z_min) || !std::isfinite(map_->z_max) ||
        r < .05 || rate <= 0 || rate>100 || min_range_ < 0 || max_range_ <= min_range_ || pending_timeout_ <= 0 ||
        map_->z_max <= map_->z_min || !map_->validEvidenceConfig())
      throw std::runtime_error("Invalid mapper parameters");
    const auto occupied_topic = declare_parameter<std::string>("occupied_topic", "/scout1/map/radar_occupied");
    const auto free_topic = declare_parameter<std::string>("free_topic", "/scout1/map/radar_free");
    compact_maps_=compactMapTransport(*this);
    publish_free_clouds_=declare_parameter("publish_free_clouds",false);
    const auto snapshot_topic=declare_parameter<std::string>("map_snapshot_topic","/scout1/map/radar_snapshot");
    if(snapshot_topic.empty()||snapshot_topic==occupied_topic||snapshot_topic==free_topic)
      throw std::runtime_error("Invalid mapper snapshot topic");
    if(compact_maps_)snapshot_pub_=create_publisher<sensor_msgs::msg::PointCloud2>(
      snapshot_topic,rclcpp::QoS(1).reliable().transient_local());
    const auto visualization_topic = declare_parameter<std::string>("visualization_topic", "");
    const auto radar_only_topic = declare_parameter<std::string>(
        "radar_only_topic", "/scout1/map/radar_only_occupied");
    if (occupied_topic == free_topic || (!visualization_topic.empty() &&
        (visualization_topic == occupied_topic || visualization_topic == free_topic)) ||
        (!radar_only_topic.empty() && (radar_only_topic == occupied_topic ||
         radar_only_topic == free_topic || radar_only_topic == visualization_topic)))
      throw std::runtime_error("Occupied/free/visualization/radar-only map topics must be distinct");
    occ_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        occupied_topic,
        rclcpp::QoS(1).reliable().transient_local());
    free_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        free_topic,
        rclcpp::QoS(1).reliable().transient_local());
    if (!visualization_topic.empty())
      visualization_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
          visualization_topic, rclcpp::QoS(1).reliable().transient_local());
    if (!radar_only_topic.empty())
      radar_only_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
          radar_only_topic, rclcpp::QoS(1).reliable().transient_local());
    for (const auto &side : {"front", "left", "right"}) {
      auto topic = declare_parameter<std::string>(
          std::string(side) + "_topic", "/scout1/radar/" + std::string(side) + "/mmwave_points");
      subs_.push_back(create_subscription<sensor_msgs::msg::PointCloud2>(
          topic, rclcpp::SensorDataQoS(), [this, topic](sensor_msgs::msg::PointCloud2::SharedPtr m) {
            const auto stamp = stampNs(m->header.stamp);
            const auto found=last_radar_stamp_ns_.find(topic);
            const int64_t latest=found==last_radar_stamp_ns_.end()?0:found->second;
            const bool rewound=latest>0&&stamp<latest-int64_t(sonar_stamp_reset_threshold_*1e9);
            if(rewound)
              resetTemporalEpoch("RADAR_STAMP_REWIND");
            last_radar_stamp_ns_[topic]=rewound?stamp:std::max(latest,stamp);
            if (!radar_stamps_.fresh(topic, stamp)) return;
            for (const auto &pending : pending_)
              if (pending.source == topic && stampNs(pending.cloud->header.stamp) == stamp)
                return;
            if (pending_.size() >= 60)
              pending_.pop_front();
            pending_.push_back({m, std::chrono::steady_clock::now(), topic});
            drain();
          }));
    }
    sonar_layer_.resolution=r;
    sonar_enabled_=declare_parameter("map_ultrasonic",true);
    sonar_layer_.half_height=declare_parameter("ultrasonic_obstacle_half_height",.20);
    sonar_layer_.ttl=declare_parameter("ultrasonic_obstacle_ttl",5.0);
    sonar_layer_.confirm_frames=declare_parameter("ultrasonic_confirm_frames",2);
    sonar_layer_.confirmation_window=declare_parameter("ultrasonic_confirmation_window",.80);
    sonar_layer_.clear_confirm_frames=declare_parameter("ultrasonic_clear_confirm_frames",2);
    sonar_layer_.clear_endpoint_guard=declare_parameter("ultrasonic_clear_endpoint_guard",.12);
    sonar_half_angle_=declare_parameter("ultrasonic_half_angle_deg",10.0)*pi/180;
    sonar_yaw_=declare_parameter("ultrasonic_yaw_deg",0.0)*pi/180;
    sonar_offset_={declare_parameter("ultrasonic_offset_x",.15),declare_parameter("ultrasonic_offset_y",0.0),declare_parameter("ultrasonic_offset_z",0.0)};
    sonar_odom_fallback_=declare_parameter("ultrasonic_odom_fallback",true);
    sonar_inf_clear_=declare_parameter("ultrasonic_inf_is_clear",true);
    sonar_stamp_reset_threshold_=declare_parameter("ultrasonic_stamp_reset_threshold",.50);
    if(sonar_layer_.half_height<0 || sonar_layer_.ttl<=0 || sonar_layer_.confirm_frames<1 ||
       sonar_layer_.clear_confirm_frames<1 || sonar_layer_.clear_confirm_frames>5 ||
       !std::isfinite(sonar_layer_.confirmation_window)||sonar_layer_.confirmation_window<=0||
       !std::isfinite(sonar_layer_.clear_endpoint_guard)||sonar_layer_.clear_endpoint_guard<.05||
       !std::isfinite(sonar_stamp_reset_threshold_)||sonar_stamp_reset_threshold_<.10||
       sonar_half_angle_<=0 || sonar_half_angle_>pi/2)
      throw std::runtime_error("Invalid ultrasonic map parameters");
    const auto sonar_topic=declare_parameter<std::string>("ultrasonic_occupied_topic","/scout1/map/ultrasonic_occupied");
    if(sonar_topic==occupied_topic || sonar_topic==free_topic || sonar_topic==visualization_topic ||
       (!radar_only_topic.empty() && sonar_topic==radar_only_topic))
      throw std::runtime_error("Ultrasonic map topic must be distinct from radar outputs");
    sonar_pub_=create_publisher<sensor_msgs::msg::PointCloud2>(sonar_topic,rclcpp::QoS(1).reliable().transient_local());
    sonar_odom_sub_=create_subscription<nav_msgs::msg::Odometry>(declare_parameter<std::string>("odom_topic","/scout1/odom"),rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::SharedPtr m){
        if(m->header.frame_id!=frame_)return;
        const auto&p=m->pose.pose.position;const auto&q=m->pose.pose.orientation;
        const double source=stampNs(m->header.stamp)*1e-9;
        if(source<=0)return;
        if(last_odom_stamp_>0&&source<last_odom_stamp_-sonar_stamp_reset_threshold_){
          resetTemporalEpoch("ODOMETRY_STAMP_REWIND");
        }
        if(source<=last_odom_stamp_)return;
        poses_.add({source,{p.x,p.y,p.z},q.w,q.x,q.y,q.z});last_odom_stamp_=source;
      });
    sonar_sub_=create_subscription<sensor_msgs::msg::LaserScan>(declare_parameter<std::string>("ultrasonic_topic","/scout1/ultrasonic/front/scan"),rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::SharedPtr m){
        if(!sonar_enabled_)return;
        const double source=stampNs(m->header.stamp)*1e-9;
        if(source<=0)return;
        double newest=last_sonar_stamp_;
        if(!sonar_pending_.empty())newest=std::max(newest,stampNs(sonar_pending_.back().scan->header.stamp)*1e-9);
        if(newest>0&&source<newest-sonar_stamp_reset_threshold_)
          resetSonarEvidence("ULTRASONIC_STAMP_REWIND",true);
        if(source<=last_sonar_stamp_)return;
        if(sonar_pending_.size()>=15)sonar_pending_.pop_front();
        sonar_pending_.push_back({m,std::chrono::steady_clock::now()});
      });
    retry_ = create_wall_timer(std::chrono::milliseconds(10), [this] { drain(); });
    timer_ = create_wall_timer(std::chrono::milliseconds(int(1000 / rate)), [this] { publish(); });
    reset_ = create_service<std_srvs::srv::Trigger>(
        "~/reset", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          map_->cells.clear();
          resetSonarEvidence("MANUAL_MAP_RESET",true);
          pending_.clear();
          radar_stamps_.reset();
          snapshot_stamps_.reset();
          poses_ = PoseHistory{};
          last_odom_stamp_=0;last_radar_stamp_ns_.clear();
          have_stamp_ = true;
          stamp_ = now();
          dirty_ = true;
          publish();
          res->success = true;
          res->message = "Map cleared; reset the planner too if changing worlds.";
        });
    config_lock_=lockParameters(*this);
    RCLCPP_INFO(get_logger(),
                "V19.1 mapper: radar evidence hit=%d miss=%d threshold=%d score=[%d,%d]; sonar ttl=%.2fs clear_frames=%d; visualization=%s radar_only=%s",
                map_->hit_score, map_->miss_score, map_->occupied_threshold, map_->min_score,
                map_->max_score,sonar_layer_.ttl,sonar_layer_.clear_confirm_frames,
                visualization_topic.empty() ? "disabled" : visualization_topic.c_str(),
                radar_only_topic.empty() ? "disabled" : radar_only_topic.c_str());
  }

private:
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr config_lock_;
  struct Pending {
    sensor_msgs::msg::PointCloud2::SharedPtr cloud;
    std::chrono::steady_clock::time_point arrival;
    std::string source;
  };
  static double steadySeconds(){
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  void resetSonarEvidence(const char *reason,bool clear_queue){
    sonar_layer_.reset();last_sonar_snapshot_.clear();last_sonar_stamp_=0;
    if(clear_queue)sonar_pending_.clear();
    ++sonar_epoch_resets_;dirty_=have_stamp_;
    RCLCPP_WARN(get_logger(),"SONAR_MEMORY_RESET reason=%s count=%zu",reason,sonar_epoch_resets_);
  }
  void resetTemporalEpoch(const char *reason){
    map_->cells.clear();pending_.clear();radar_stamps_.reset();snapshot_stamps_.reset();
    poses_=PoseHistory{};last_odom_stamp_=0;last_radar_stamp_ns_.clear();
    have_stamp_=false;dirty_=false;stamp_=builtin_interfaces::msg::Time{};
    resetSonarEvidence(reason,true);
    RCLCPP_WARN(get_logger(),"MAPPER_TIME_EPOCH_RESET reason=%s",reason);
  }
  bool integrate(const sensor_msgs::msg::PointCloud2 &m, const std::string &source) {
    if (m.header.frame_id.empty() || !radar_stamps_.fresh(source, stampNs(m.header.stamp)))
      return true;
    geometry_msgs::msg::TransformStamped tr;
    bool found = false;
    std::vector<std::string> frames{m.header.frame_id};
    if (m.header.frame_id.rfind(prefix_, 0) != 0)
      frames.push_back(prefix_ + m.header.frame_id);
    for (const auto &frame : frames)
      try {
        tr = tf_.lookupTransform(frame_, frame,
                                 rclcpp::Time(m.header.stamp, get_clock()->get_clock_type()),
                                 rclcpp::Duration::from_seconds(0));
        found = true;
        break;
      } catch (const tf2::TransformException &) {
      }
    if (!found)
      return false;
    auto q = tr.transform.rotation;
    tf2::Quaternion quat(q.x, q.y, q.z, q.w);
    if (quat.length2() < 1e-10)
      return true;
    quat.normalize();
    auto t = tr.transform.translation;
    tf2::Transform T(quat, tf2::Vector3(t.x, t.y, t.z));
    std::vector<Vec3> endpoints;
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(m, "x"), y(m, "y"), z(m, "z");
      endpoints.reserve(size_t(m.width) * m.height);
      for (; x != x.end(); ++x, ++y, ++z) {
        Vec3 p{*x, *y, *z};
        if (!finite(p))
          continue;
        const double d = norm(p);
        if (d < min_range_ || d > max_range_)
          continue;
        auto v = T * tf2::Vector3(p.x, p.y, p.z);
        endpoints.push_back({v.x(), v.y(), v.z()});
      }
    } catch (const std::runtime_error &e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000, "Bad cloud: %s", e.what());
      return true;
    }
    map_->integrate({t.x, t.y, t.z}, endpoints, min_range_, max_range_);
    radar_stamps_.integrated(source, stampNs(m.header.stamp));
    if (!have_stamp_ || stampNs(m.header.stamp) > stampNs(stamp_))
      stamp_ = m.header.stamp;
    have_stamp_ = true;
    dirty_ = true;
    return true;
  }
  struct PendingSonar {
    sensor_msgs::msg::LaserScan::SharedPtr scan;
    std::chrono::steady_clock::time_point arrival;
  };
  bool integrateSonar(const sensor_msgs::msg::LaserScan&m){
    const double time=stampNs(m.header.stamp)*1e-9;
    if(time<=0 || time<=last_sonar_stamp_)return true;
    auto reading=parseSonar(m.ranges,m.angle_min,m.angle_increment,m.range_min,m.range_max,
                            sonar_half_angle_,sonar_inf_clear_,sonar_yaw_);
    if(!reading.valid)return true;
    bool found=false;
    tf2::Transform transform=tf2::Transform::getIdentity();
    std::vector<std::string> frames{m.header.frame_id};
    if(!m.header.frame_id.empty() && m.header.frame_id.rfind(prefix_,0)!=0)frames.push_back(prefix_+m.header.frame_id);
    for(const auto&frame:frames){
      if(frame.empty())continue;
      try{
        auto tr=tf_.lookupTransform(frame_,frame,rclcpp::Time(m.header.stamp,get_clock()->get_clock_type()),rclcpp::Duration::from_seconds(0));
        const auto&q=tr.transform.rotation;const auto&t=tr.transform.translation;
        tf2::Quaternion quat(q.x,q.y,q.z,q.w);
        if(quat.length2()<1e-10)continue;
        quat.normalize();transform=tf2::Transform(quat,tf2::Vector3(t.x,t.y,t.z));found=true;break;
      }catch(const tf2::TransformException&){}
    }
    PoseSample pose;
    if(!found && (!sonar_odom_fallback_ || !poses_.at(time,pose)))return false;
    Vec3 origin;
    if(found){auto v=transform*tf2::Vector3(0,0,0);origin={v.x(),v.y(),v.z()};}
    else origin=pose.position+pose.rotate(sonar_offset_);
    auto world=[&](Vec3 p){
      if(found){auto v=transform*tf2::Vector3(p.x,p.y,p.z);return Vec3{v.x(),v.y(),v.z()};}
      Vec3 rotated{std::cos(sonar_yaw_)*p.x-std::sin(sonar_yaw_)*p.y,
                   std::sin(sonar_yaw_)*p.x+std::cos(sonar_yaw_)*p.y,p.z};
      return pose.position+pose.rotate(sonar_offset_+rotated);
    };
    std::vector<Vec3> hits,free_endpoints;
    for(Vec3 p:reading.endpoints){Vec3 hit=world(p);if(hit.z>=map_->z_min&&hit.z<=map_->z_max)hits.push_back(hit);}
    for(Vec3 p:reading.free_endpoints){Vec3 endpoint=world(p);if(finite(endpoint))free_endpoints.push_back(endpoint);}
    const double evidence_time=steadySeconds();
    sonar_layer_.clearObserved(origin,free_endpoints,evidence_time);
    sonar_layer_.integrate(hits,evidence_time);last_sonar_stamp_=time;
    // Sonar adds obstacles but never supplies a full radar-map freshness heartbeat.
    if(have_stamp_)dirty_=true;
    return true;
  }
  void drain() {
    for(auto it=sonar_pending_.begin();it!=sonar_pending_.end();){
      if(integrateSonar(*it->scan)){it=sonar_pending_.erase(it);continue;}
      double age=std::chrono::duration<double>(std::chrono::steady_clock::now()-it->arrival).count();
      if(age>pending_timeout_){
        RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),3000,"SONAR_MAP_POSE_MISSING: check ultrasonic TF or odom/mount parameters");
        it=sonar_pending_.erase(it);
      }else ++it;
    }
    for (auto it = pending_.begin(); it != pending_.end();) {
      if (integrate(*it->cloud, it->source)) {
        it = pending_.erase(it);
        continue;
      }
      double age =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - it->arrival).count();
      if (age > pending_timeout_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                             "Timestamped TF unavailable; dropping cloud (no latest-TF fallback)");
        it = pending_.erase(it);
      } else
        ++it;
    }
  }
  void publish() {
    const auto sonar=sonar_layer_.occupied(steadySeconds());
    if(sonar!=last_sonar_snapshot_)dirty_=true;
    if (!have_stamp_ || !dirty_)
      return;
    const auto snapshot_ns = snapshot_stamps_.next(stampNs(stamp_));
    if (!snapshot_ns) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "Snapshot stamp offset exhausted; retaining pending map until radar time advances");
      return;
    }
    if (*snapshot_ns / 1000000000LL > std::numeric_limits<int32_t>::max()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "Snapshot stamp exceeds ROS Time range; retaining pending map");
      return;
    }
    builtin_interfaces::msg::Time snapshot_stamp;
    snapshot_stamp.sec = static_cast<int32_t>(*snapshot_ns / 1000000000LL);
    snapshot_stamp.nanosec = static_cast<uint32_t>(*snapshot_ns % 1000000000LL);
    KeySet occ, free;
    map_->exportSets(occ, free);
    // Preserve sensor provenance before adding sonar obstacles. A difference
    // between these layers indicates unconfirmed radar evidence, not glass.
    // Use the same snapshot, frame and stamp as the merged occupied/free pair.
    if (radar_only_pub_ && radar_only_pub_->get_subscription_count()>0)
      radar_only_pub_->publish(makeCloud(occ, map_->r, frame_, snapshot_stamp));
    for(Key k:sonar){occ.insert(k);free.erase(k);}
    if(sonar_pub_->get_subscription_count()>0)
      sonar_pub_->publish(makeCloud(sonar,map_->r,frame_,snapshot_stamp));
    size_t snapshot_bytes=0;
    if(snapshot_pub_){
      auto snapshot=makeMapSnapshot(occ,free,map_->r,frame_,snapshot_stamp);
      snapshot_bytes=snapshot.data.size();snapshot_pub_->publish(std::move(snapshot));
    }
    // Legacy XYZ topics remain available for RViz/tools, but unused large
    // free clouds are neither allocated nor transmitted on the compact path.
    if(!compact_maps_ || occ_pub_->get_subscription_count()>0 ||
       (visualization_pub_&&visualization_pub_->get_subscription_count()>0)){
      const auto occupied=makeCloud(occ,map_->r,frame_,snapshot_stamp);
      if(!compact_maps_||occ_pub_->get_subscription_count()>0)occ_pub_->publish(occupied);
      if(visualization_pub_&&visualization_pub_->get_subscription_count()>0)
        visualization_pub_->publish(occupied);
    }
    if(!compact_maps_ || (publish_free_clouds_&&free_pub_->get_subscription_count()>0))
      free_pub_->publish(makeCloud(free,map_->r,frame_,snapshot_stamp));
    last_sonar_snapshot_=sonar;
    dirty_ = false;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
      "map occupied=%zu free=%zu transport=%s snapshot_bytes=%zu xyz_pair_bytes=%zu",
      occ.size(),free.size(),compact_maps_?"compact":"legacy",snapshot_bytes,
      (occ.size()+free.size())*size_t(16));
  }
  SonarLayer sonar_layer_;
  KeySet last_sonar_snapshot_;
  PoseHistory poses_;
  Vec3 sonar_offset_;
  double sonar_half_angle_,sonar_yaw_,last_sonar_stamp_{0},last_odom_stamp_{0};
  double sonar_stamp_reset_threshold_{.50};
  bool sonar_enabled_,sonar_odom_fallback_,sonar_inf_clear_{true};
  size_t sonar_epoch_resets_{0};
  std::unordered_map<std::string,int64_t>last_radar_stamp_ns_;
  std::deque<PendingSonar>sonar_pending_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sonar_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sonar_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sonar_odom_sub_;
  std::string frame_, prefix_;
  double min_range_, max_range_, pending_timeout_;
  std::unique_ptr<EvidenceMap> map_;
  tf2_ros::Buffer tf_;
  tf2_ros::TransformListener listener_;
  std::deque<Pending> pending_;
  ObservationStampGate radar_stamps_;
  SnapshotStamp snapshot_stamps_;
  bool have_stamp_{false}, dirty_{false};
  builtin_interfaces::msg::Time stamp_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> subs_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr occ_pub_, free_pub_, visualization_pub_, radar_only_pub_;
  bool compact_maps_{true},publish_free_clouds_{false};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr snapshot_pub_;
  rclcpp::TimerBase::SharedPtr retry_, timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_;
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RadarFreeSpaceMapper>());
  rclcpp::shutdown();
  return 0;
}
