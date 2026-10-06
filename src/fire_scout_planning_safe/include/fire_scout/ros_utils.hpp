#pragma once
#include "grid.hpp"
#include "vehicle.hpp"
#include "gap_policy.hpp"
#include "map_snapshot.hpp"
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>
namespace fire_scout {
inline bool compactMapTransport(rclcpp::Node &n){
  const auto mode=n.declare_parameter<std::string>("map_transport","compact");
  if(mode!="compact"&&mode!="legacy")throw std::runtime_error("map_transport must be compact or legacy");
  return mode=="compact";
}
inline sensor_msgs::msg::PointCloud2 makeMapSnapshot(const KeySet &occupied,const KeySet &free,
       double resolution,const std::string &frame,const builtin_interfaces::msg::Time &stamp){
  sensor_msgs::msg::PointCloud2 m;m.header.frame_id=frame;m.header.stamp=stamp;
  m.height=1;m.is_dense=true;m.is_bigendian=false;m.point_step=map_snapshot::record_bytes;
  const char *names[]={"voxel_x","voxel_y","voxel_z","run_length","state","resolution"};
  for(uint32_t i=0;i<6;++i){
    sensor_msgs::msg::PointField field;field.name=names[i];field.offset=4*i;field.count=1;
    field.datatype=i<3?sensor_msgs::msg::PointField::INT32:
                   i==5?sensor_msgs::msg::PointField::FLOAT32:sensor_msgs::msg::PointField::UINT32;
    m.fields.push_back(field);
  }
  m.data=map_snapshot::encode(occupied,free,resolution);
  if(m.data.size()>UINT32_MAX)throw std::runtime_error("Compact map exceeds ROS row size");
  m.width=uint32_t(m.data.size()/m.point_step);m.row_step=uint32_t(m.data.size());return m;
}
inline map_snapshot::Decoded parseMapSnapshot(const sensor_msgs::msg::PointCloud2 &m,
        double resolution,size_t limit=map_snapshot::default_max_cells,map_snapshot::Region region={}){
  if(m.is_bigendian || m.height!=1 || m.point_step!=map_snapshot::record_bytes ||
     uint64_t(m.width)*m.point_step!=m.data.size() || m.row_step!=m.data.size())
    throw std::runtime_error("Invalid compact map envelope");
  const char *names[]={"voxel_x","voxel_y","voxel_z","run_length","state","resolution"};
  if(m.fields.size()!=6)throw std::runtime_error("Invalid compact map fields");
  for(uint32_t i=0;i<6;++i){
    const auto type=i<3?sensor_msgs::msg::PointField::INT32:
       i==5?sensor_msgs::msg::PointField::FLOAT32:sensor_msgs::msg::PointField::UINT32;
    if(m.fields[i].name!=names[i]||m.fields[i].offset!=4*i||m.fields[i].count!=1||m.fields[i].datatype!=type)
      throw std::runtime_error("Invalid compact map schema");
  }
  return map_snapshot::decode(m.data,resolution,limit,region);
}
inline GapPolicyConfig gapParameters(rclcpp::Node&n){
  GapPolicyConfig c;
  c.enabled=n.declare_parameter("adaptive_gap_enabled",true);
  c.margin_weight=n.declare_parameter("gap_margin_weight",1.0);
  c.speed_weight=n.declare_parameter("gap_speed_weight",1.0);
  c.scale=n.declare_parameter("gap_scale",.60);
  c.max_extra_margin=n.declare_parameter("gap_max_extra_margin",.45);
  c.probe_distance=n.declare_parameter("gap_probe_distance",1.0);
  c.min_speed=n.declare_parameter("gap_min_speed",.50);
  if(!c.valid())throw std::runtime_error("Invalid gap policy parameters");
  return c;
}
inline KeySet cloudSet(const sensor_msgs::msg::PointCloud2 &m, double r) {
  KeySet out;
  out.reserve(size_t(m.width) * m.height);
  sensor_msgs::PointCloud2ConstIterator<float> x(m, "x"), y(m, "y"), z(m, "z");
  for (; x != x.end(); ++x, ++y, ++z) {
    Vec3 p{*x, *y, *z};
    if (finite(p))
      out.insert(key(p, r));
  }
  return out;
}
// The follower does not need to hash an entire multi-room global map at 40 Hz.
// Keep a bounded local copy; its caller MUST reject motion outside these bounds.
inline KeySet cloudSetRegion(const sensor_msgs::msg::PointCloud2&m,double r,Vec3 lo,Vec3 hi){
  KeySet out;out.reserve(std::min<size_t>(size_t(m.width)*m.height,100000));
  sensor_msgs::PointCloud2ConstIterator<float>x(m,"x"),y(m,"y"),z(m,"z");
  for(;x!=x.end();++x,++y,++z){
    Vec3 p{*x,*y,*z};
    if(finite(p)&&p.x>=lo.x&&p.x<=hi.x&&p.y>=lo.y&&p.y<=hi.y&&p.z>=lo.z&&p.z<=hi.z)
      out.insert(key(p,r));
  }return out;
}
inline sensor_msgs::msg::PointCloud2 makeCloud(const KeySet &keys, double r,
                                               const std::string &frame,
                                               const builtin_interfaces::msg::Time &stamp) {
  sensor_msgs::msg::PointCloud2 m;
  m.header.stamp = stamp;
  m.header.frame_id = frame;
  m.height = 1;
  m.is_dense = true;
  sensor_msgs::PointCloud2Modifier mod(m);
  mod.setPointCloud2FieldsByString(1, "xyz");
  mod.resize(keys.size());
  sensor_msgs::PointCloud2Iterator<float> x(m, "x"), y(m, "y"), z(m, "z");
  for (Key k : keys) {
    auto p = center(k, r);
    *x = p.x;
    *y = p.y;
    *z = p.z;
    ++x;
    ++y;
    ++z;
  }
  return m;
}
inline nav_msgs::msg::Path makePath(const std::vector<Vec3> &p, const std::string &frame,
                                    const builtin_interfaces::msg::Time &stamp) {
  nav_msgs::msg::Path m;
  m.header.frame_id = frame;
  m.header.stamp = stamp;
  for (auto v : p) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = m.header;
    pose.pose.position.x = v.x;
    pose.pose.position.y = v.y;
    pose.pose.position.z = v.z;
    pose.pose.orientation.w = 1;
    m.poses.push_back(pose);
  }
  return m;
}
// Configuration is read at startup; never acknowledge a change that the core ignores.
inline rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr lockParameters(rclcpp::Node&n){
  return n.add_on_set_parameters_callback([](const std::vector<rclcpp::Parameter>&parameters){
    rcl_interfaces::msg::SetParametersResult result;result.successful=true;
    for(const auto&p:parameters)if(p.get_name()!="use_sim_time"){
      result.successful=false;
      result.reason="Configuration is startup-only. Edit YAML/launch arguments and restart these nodes.";
      break;
    }
    return result;
  });
}
inline VehicleConfig vehicleParameters(rclcpp::Node &n) {
  VehicleConfig v;
  v.wheelbase=n.declare_parameter("vehicle_wheelbase",.50);
  v.propeller_diameter=n.declare_parameter("propeller_diameter",.28);
  v.body_length=n.declare_parameter("vehicle_body_length",.30);
  v.body_width=n.declare_parameter("vehicle_body_width",.22);
  v.height=n.declare_parameter("vehicle_height",.25);
  v.max_tilt_deg=n.declare_parameter("vehicle_max_tilt_deg",15.0);
  v.margin_xy=n.declare_parameter("safety_margin_xy",.08);
  v.tracking_margin=n.declare_parameter("tracking_error_margin",.04);
  v.margin_z=n.declare_parameter("safety_margin_z",.05);
  if(!v.valid())throw std::runtime_error("Invalid vehicle dimensions/margins");
  return v;
}
inline GridConfig gridParameters(rclcpp::Node &n) {
  GridConfig c;
  const auto v=vehicleParameters(n);
  c.resolution = n.declare_parameter("voxel_size", .15);
  // Old inflation parameters remain accepted as LOWER BOUNDS, never shrinking the body.
  const bool passage=n.declare_parameter("separate_comfort_clearance",true);
  const double minimum=n.declare_parameter("minimum_safety_margin_xy",.02);
  c.inflation_xy = std::max(passage?v.passageRadius(minimum):v.radius(),n.declare_parameter("inflation_radius", 0.0));
  c.inflation_z = std::max(v.halfHeight(),n.declare_parameter("inflation_z", 0.0));
  // Preserve the old wide-space preferred distance. Only the last part of the
  // optional comfort margin becomes traversable, at the follower's narrow cap.
  c.comfort_margin=std::max(0.,v.radius()-c.inflation_xy);
  c.preferred_clearance=n.declare_parameter("preferred_clearance",.30)+c.comfort_margin;
  const double recovery_margin=n.declare_parameter("recovery_margin_xy",.02);
  if(!std::isfinite(recovery_margin)||recovery_margin<.02)
    throw std::runtime_error("recovery_margin_xy must be finite and >= 0.02 m");
  // Recovery is not ordinary flight: it is a slow, continuously checked move
  // strictly away from an existing conservative-map overlap.  Retain the
  // complete tilted rotor envelope and a physical margin, while allowing the
  // tracking-error reserve to be consumed only by that explicit proof.
  c.recovery_xy=std::min(c.inflation_xy,v.tiltedRadius()+recovery_margin);
  c.z_min = n.declare_parameter("z_min", .65);
  c.z_max = n.declare_parameter("z_max", 3.0);
  if (!std::isfinite(c.resolution) || c.resolution < .05 || c.resolution>.5 ||
      !std::isfinite(c.inflation_xy) || !std::isfinite(c.inflation_z) ||
      !std::isfinite(c.preferred_clearance) || c.preferred_clearance<0 ||
      !std::isfinite(c.z_min) || !std::isfinite(c.z_max) || c.z_max <= c.z_min)
    throw std::runtime_error("Invalid grid parameters");
  RCLCPP_INFO(n.get_logger(),"Vehicle body: wheelbase=%.3fm propeller=%.3fm swept radius=%.3fm planning radius=%.3fm recovery radius=%.3fm half-height=%.3fm voxel=%.2fm",
              v.wheelbase,v.propeller_diameter,v.physicalRadius(),c.inflation_xy,c.recovery_xy,c.inflation_z,c.resolution);
  return c;
}
inline int64_t stampNs(const builtin_interfaces::msg::Time &s) {
  return int64_t(s.sec) * 1000000000LL + s.nanosec;
}
inline std_msgs::msg::String textMessage(const std::string &s) {
  std_msgs::msg::String m;
  m.data = s;
  return m;
}
} // namespace fire_scout
