/*
 * Fire Scout: rectangular exterior facade inspection (simulation prototype), ROS 2 Humble / PX4.
 * Replaces the original entrance_wall_search.cpp. C++17.
 * CORNER FIX 2026-09-20: vector acceleration / continuous lead limiting;
 * settled-pose wall acquisition; rolling solid-wall fit; automatic REFIT_WALL.
 * Wall/scan ambiguity no longer latches HOLD. Doorway evidence freezes the model.
 * New parameters: wall_track_window=2.5 s, wall_track_interval=0.5 s,
 * wall_track_yaw_rate_deg=3, wall_track_plane_rate=0.10 m/s,
 * wall_refit_time=1.5 s AFTER settling. Existing launch parameters remain valid.
 * TAKEOFF FIX: vertical reference advances to home_z - takeoff_height without
 * the cruise max_setpoint_lead cap or tracking/scan HOLD gates. Keep captured
 * yaw and XY during ascent; search yaw/scan alignment begins only after arrival.
 * A slow ascent logs a warning instead of cancelling the takeoff target.
 * This does not repair the separate PX4/DDS time-base anomaly seen in recordings.
 *
 * Dependencies: rclcpp, sensor_msgs, std_msgs, px4_msgs.
 * CMake:
 *   find_package(sensor_msgs REQUIRED)
 *   find_package(std_msgs REQUIRED)
 *   add_executable(entrance_wall_search src/entrance_wall_search.cpp)
 *   target_compile_features(entrance_wall_search PRIVATE cxx_std_17)
 *   ament_target_dependencies(entrance_wall_search rclcpp sensor_msgs std_msgs px4_msgs)
 *   install(TARGETS entrance_wall_search DESTINATION lib/${PROJECT_NAME})
 * package.xml: add <depend>sensor_msgs</depend> and <depend>std_msgs</depend>.
 * Do not duplicate a pre-existing target or dependency.
 *
 * Run (automatically requests Offboard and arm after valid inputs):
 * ros2 run fire_scout_control entrance_wall_search --ros-args \
 *   -p use_sim_time:=true -p px4_prefix:=/scout1 \
 *   -p takeoff_height:=1.5 -p wall_yaw_deg:=90.0
 *
 * wall_yaw_deg is the inward wall normal in PX4 NED: 0=north, 90=east.
 * Default 90 matches the west facade in the supplied Fire Scout world.
 * search_direction=+1 searches toward body-right, -1 toward body-left.
 * Scan must point forward; scan_yaw_deg and sensor_forward_offset are extrinsics.
 * Confirm sensor_forward_offset against YOUR current model.sdf (default 0.15 m).
 * Keeps the original PX4 local coordinates: no hard-coded world/window coordinates.
 *
 * Start within wall_acquire_max_range=2.9 m of a solid patch at takeoff height.
 * Wall orientation is fitted within +/-35 deg of wall_yaw_deg; no global search.
 * Openings outside [minimum_opening_width, maximum_opening_width] are not reported.
 * A wider unbounded opening is not evidence of a corner: adjacent-plane fit is
 * required BEFORE moving across the original wall plane.
 * Flow: wait -> prestream -> confirmed Offboard -> confirmed arm -> takeoff
 * -> stationary yaw sweep / wall-line fit (start near a solid facade patch)
 * -> stand-off -> facade sweep -> wall/hole/wall evidence -> centre -> confirm
 * -> FOUND_HOLD (2 s) -> SEARCH -> convex corner -> next facade. Does NOT cross: a horizontal ultrasonic slice cannot prove height,
 * a clear aircraft-sized 3D corridor, or the absence of glass in real hardware.
 * Assumes one isolated convex rectangular building, vertical planar walls, clear
 * exterior flight space, solid wall patches at corners, approximate initial
 * wall_yaw_deg (+/-35 deg), and stable local NED. Single-height inspection only.
 * Completes only after four OBSERVED adjacent walls and overlap of the first wall.
 * Cannot certify 3D entrances, arbitrary polygons, occluded walls, or multiple heights.
 * Uses standard ROS LaserScan: +angle is left/CCW (FLU); PX4 body is FRD.
 * Sensor is level relative to body; scan_yaw_deg is ROS-positive left rotation.
 * Fit uses body quaternion to project rays into local NED. Excessive tilt suspends
 * evidence; it cannot recover vertical geometry that a horizontal slice never sees.
 * Time regressions are diagnostic only in this simulation prototype. Persistent
 * HOLD is explained by status logs; 'resume' repeats the interrupted state.
 * Algorithm: bounded consensus line fit + wall/open/wall + adjacent-wall validation.
 * Reference: https://pointclouds.org/documentation/tutorials/planar_segmentation.html
 * No return (positive infinity) counts as far only if inf_is_clear=true AND the
 * sensor range covers the required depth. NaN/zero/below-min are always invalid.
 * Wall disappearance alone is never a confirmed entrance; both wall edges required.
 *
 * Commands while running (std_msgs/msg/String):
 * ros2 topic pub --once /entrance_wall_search/command std_msgs/msg/String "{data: hold}"
 * ros2 topic pub --once /entrance_wall_search/command std_msgs/msg/String "{data: land}"
 * ros2 topic pub --once /entrance_wall_search/command std_msgs/msg/String "{data: resume}"
 * start is accepted only in WAIT_INPUT with auto_start=false. No in-air auto-rearm.
 * Results: /entrance_wall_search/state (String, transient local),
 *          /entrance_wall_search/entrance (JSON String, transient local).
 * Don't run scout_takeoff/scout_enter_window/another Offboard publisher concurrently.
 *
 * Simulation note: PX4/sensor timestamp rollbacks and estimator reset_counter
 * changes are logged but do not release Offboard. Invalid/stale odometry or
 * lost flight mode still release control to PX4. Scan loss
 * with valid odometry latches HOLD; recovery never automatically resumes motion.
 * Ctrl+C stops this node; use 'land' and wait for PX4 landing before stopping.
 * Real-time deadlines use steady clock; mission increments use ROS clock.
 * Pause of /clock freezes mission motion; backwards jumps keep streaming.
 * Timing/limits: entrance_pause=2 s, maximum_face_length=40 m,
 * mission_timeout=1800 s, mission_radius_limit=100 m. Legacy search_half_extent
 * and acquire_lateral_limit are removed; they did not cover a whole building.
 * A single front sensor does not observe the sideways swept flight volume.
 * Use this prototype in an isolated, unobstructed rectangular simulation scene.
 * Ambiguous wall geometry triggers stationary REFIT_WALL and automatic search
 * recovery. Genuine missing/invalid sensor data still has a separate HOLD reason.
 * 'resume' requires valid inputs and current Offboard.
 * Source review and standalone numerical/state tests are not SITL flight validation.
 */
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

// BEGIN PURE CORE: no ROS dependencies; used by offline algorithm regression tests.
namespace wall_search_core {
constexpr double pi = 3.14159265358979323846;
inline double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }
inline double limit(double x, double low, double high) {
  return std::max(low, std::min(high, x));
}
inline double median(std::vector<double> values) {
  if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
  std::sort(values.begin(), values.end());
  const auto n=values.size();
  return n%2 ? values[n/2] : (values[n/2-1]+values[n/2])*0.5;
}
struct ScanSummary {
  bool valid{false};
  double normal_distance{std::numeric_limits<double>::quiet_NaN()};
  double nearest{std::numeric_limits<double>::infinity()};
  double far_fraction{0.0};
  double finite_fraction{0.0};
  double usable_fraction{0.0};
};
// dx is the depth, projected onto the chosen fixed wall normal.
inline ScanSummary summarize(const std::vector<float>& ranges, double angle_min,
    double angle_increment, double range_min, double range_max,
    double scan_yaw, double yaw_error, double half_angle,
    double far_depth, bool inf_is_clear) {
  ScanSummary out;
  if (ranges.empty() || !std::isfinite(angle_min) || !std::isfinite(angle_increment) ||
      !std::isfinite(range_min) || !std::isfinite(range_max) ||
      range_min<0 || range_max<=range_min ||
      (ranges.size()>1 && angle_increment==0)) return out;
  size_t selected=0,usable=0,finite=0,far=0;
  std::vector<double> depths;
  for(size_t i=0;i<ranges.size();++i) {
    const double body_angle=-(angle_min+double(i)*angle_increment+scan_yaw); // FLU -> FRD
    const double r=ranges[i];
    // Immediate proximity check covers all actual forward-facing rays.
    if(std::abs(wrap(body_angle))<pi/2 && std::isfinite(r) && r>0 && r<=range_max)
      out.nearest=std::min(out.nearest,r); // below-min return still means danger
    if(std::abs(wrap(body_angle))>half_angle+1e-6) continue;
    ++selected;
    const double projection=std::cos(body_angle+yaw_error);
    if(projection<=0.5) continue;
    if(std::isfinite(r) && r>=range_min && r<=range_max && r>0) {
      ++usable; ++finite;
      const double d=r*projection; depths.push_back(d);
      if(d>far_depth) ++far;
    } else if(std::isinf(r) && r>0 && inf_is_clear && range_max*projection>far_depth) {
      ++usable; ++far;
    }
  }
  if(!selected) return out;
  out.usable_fraction=double(usable)/selected;
  out.finite_fraction=double(finite)/selected;
  out.far_fraction=double(far)/selected;
  out.normal_distance=median(depths);
  out.valid=out.usable_fraction>=0.6;
  return out;
}
// Hysteresis: only distinct frames go in. Edge positions use actual vehicle motion.
// First and last wall returns must be on the same previously acquired wall plane.
enum class Evidence { Unknown, Wall, Open };
struct GapResult { bool found{false}; double first{0}, second{0}, width{0}; };
class GapTracker {
public:
  int confirm_frames{3}; double min_width{1.2},max_width{3.5};
  bool in_gap{false};
  void reset() { in_gap=false; have_wall=false; have_along=false; wall_count=open_count=0; }
  GapResult update(Evidence e,double along) {
    GapResult r;
    if(have_along && std::abs(along-last_along)>0.35)reset();
    have_along=true;last_along=along;
    if(e==Evidence::Unknown) {wall_count=open_count=0;return r;}
    if(e==Evidence::Wall) {
      ++wall_count; open_count=0;
      if(in_gap) {
        if(wall_count==1) closing=(last_open+along)*0.5;
        if(wall_count>=confirm_frames) {
          r.first=opening;r.second=closing;r.width=std::abs(closing-opening);
          r.found=r.width>=min_width && r.width<=max_width;
          in_gap=false;have_wall=true;last_wall=along;
        }
      } else {
        if(wall_count>=confirm_frames) have_wall=true;
        last_wall=along;
      }
    } else {
      wall_count=0;
      if(open_count==0) first_open=along;
      ++open_count;
      if(!in_gap && have_wall && open_count>=confirm_frames) {
        opening=(last_wall+first_open)*0.5;in_gap=true;
      }
      last_open=along;
    }
    return r;
  }
  double openingPosition() const {return opening;}
  bool pending() const {return in_gap || open_count>0;}
  bool tooWide(double along) const {return in_gap && std::abs(along-opening)>max_width;}
private:
  bool have_wall{false},have_along{false};double last_along{0};int wall_count{0},open_count{0};
  double last_wall{0},first_open{0},last_open{0},opening{0},closing{0};
};
struct Point2 {double x,y;};
struct WallFit {bool valid{false}; double yaw{0},plane{0},span{0},rms{0}; size_t count{0};};
inline WallFit fitWall(const std::vector<Point2>& points,double hint,double tolerance,
    double min_span,double min_distance,double max_distance,Point2 origin) {
  WallFit best; double best_error=1e9;
  if(points.size()<8)return best;
  // Deterministic consensus search over plausible normals; then orthogonal LS.
  for(int degree=-35;degree<=35;++degree){
    const double yaw=hint+degree*pi/180, nx=std::cos(yaw),ny=std::sin(yaw);
    std::vector<double> ds;for(const auto& p:points)ds.push_back(nx*p.x+ny*p.y);
    std::sort(ds.begin(),ds.end());
    size_t left=0;
    for(size_t right=0;right<ds.size();++right){
      while(ds[right]-ds[left]>2*tolerance)++left;
      const size_t count=right-left+1;
      if(count<8 || count*10<points.size()*6 || count<best.count)continue;
      const double plane=(ds[left]+ds[right])*0.5;
      const double distance=plane-nx*origin.x-ny*origin.y;
      if(distance<min_distance || distance>max_distance)continue;
      double lo=1e9,hi=-1e9,error=0;
      for(const auto& p:points){
        const double e=nx*p.x+ny*p.y-plane;
        if(std::abs(e)<=tolerance+1e-9){lo=std::min(lo,-ny*p.x+nx*p.y);hi=std::max(hi,-ny*p.x+nx*p.y);error+=e*e;}
      }
      if(hi-lo<min_span || (count==best.count && error>=best_error))continue;
      best={true,wrap(yaw),plane,hi-lo,std::sqrt(error/count),count};best_error=error;
    }
  }
  if(!best.valid)return best;
  const double nx=std::cos(best.yaw),ny=std::sin(best.yaw);
  std::vector<Point2> inliers;double mx=0,my=0;
  for(const auto& p:points)if(std::abs(nx*p.x+ny*p.y-best.plane)<=tolerance+1e-9){inliers.push_back(p);mx+=p.x;my+=p.y;}
  mx/=inliers.size();my/=inliers.size();double xx=0,yy=0,xy=0;
  for(const auto& p:inliers){const double x=p.x-mx,y=p.y-my;xx+=x*x;yy+=y*y;xy+=x*y;}
  double yaw=0.5*std::atan2(2*xy,xx-yy)+pi/2;
  if(std::cos(yaw-hint)<0)yaw+=pi;
  const double plane=std::cos(yaw)*mx+std::sin(yaw)*my;
  const double distance=plane-std::cos(yaw)*origin.x-std::sin(yaw)*origin.y;
  if(std::abs(wrap(yaw-hint))>35*pi/180 || distance<min_distance || distance>max_distance)return {};
  // Revalidate the refined line, not the statistics of the coarse hypothesis.
  double lo=1e9,hi=-1e9,error=0;size_t count=0;
  for(const auto& p:points){
    const double e=std::cos(yaw)*p.x+std::sin(yaw)*p.y-plane;
    if(std::abs(e)>tolerance)continue;
    const double s=-std::sin(yaw)*p.x+std::cos(yaw)*p.y;
    lo=std::min(lo,s);hi=std::max(hi,s);error+=e*e;++count;
  }
  if(count<8 || count*10<points.size()*6 || hi-lo<min_span)return {};
  return {true,wrap(yaw),plane,hi-lo,std::sqrt(error/count),count};
}
} // namespace wall_search_core
// END PURE CORE

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

namespace wc=wall_search_core;
using Odom=px4_msgs::msg::VehicleOdometry;
using Status=px4_msgs::msg::VehicleStatus;
using Ack=px4_msgs::msg::VehicleCommandAck;
using Command=px4_msgs::msg::VehicleCommand;
// PX4 >=1.16 versioned output topic names follow the compiled message version.
// Older px4_msgs without MESSAGE_VERSION use the unversioned topic name.
template<class T,class=void> struct MsgVersion {static constexpr unsigned value=0;};
template<class T> struct MsgVersion<T,std::void_t<decltype(T::MESSAGE_VERSION)>> {
  static constexpr unsigned value=T::MESSAGE_VERSION;
};

class EntranceWallSearch : public rclcpp::Node {
public:
  EntranceWallSearch():Node("entrance_wall_search") {
    prefix_=declare_parameter<std::string>("px4_prefix","/scout1");
    if(!prefix_.empty() && prefix_[0]!='/') prefix_='/'+prefix_;
    while(!prefix_.empty() && prefix_.back()=='/')prefix_.pop_back();
    target_system_=declare_parameter<int>("target_system",1);
    source_system_=declare_parameter<int>("source_system",245);
    auto_start_=declare_parameter<bool>("auto_start",true);
    inf_clear_=declare_parameter<bool>("inf_is_clear",true);
    height_=param("takeoff_height",1.5);
    yaw_target_=param("wall_yaw_deg",90.0)*wc::pi/180.0;
    yaw_target_=wc::wrap(yaw_target_);
    sensor_offset_=param("sensor_forward_offset",0.15);
    scan_yaw_=param("scan_yaw_deg",0.0)*wc::pi/180.0;
    half_angle_=param("scan_half_angle_deg",10.0)*wc::pi/180.0;
    stand_off_=param("wall_distance",1.5); // vehicle centre to facade
    min_stop_=param("minimum_front_distance",0.55); // sensor to obstacle
    acquire_max_=param("wall_acquire_max_range",2.9);
    gap_depth_=param("opening_depth_margin",0.7);
    wall_tol_=param("wall_plane_tolerance",0.25);
    evidence_timeout_=param("ambiguous_evidence_timeout",2.5);
    track_window_=param("wall_track_window",2.5);
    track_interval_=param("wall_track_interval",0.5);
    track_yaw_rate_=param("wall_track_yaw_rate_deg",3.0)*wc::pi/180;
    track_plane_rate_=param("wall_track_plane_rate",0.10);
    refit_time_=param("wall_refit_time",1.5);
    search_speed_=param("search_speed",0.20);
    approach_speed_=param("approach_speed",0.20);
    vertical_speed_=param("vertical_speed",0.7);
    takeoff_accel_=param("takeoff_acceleration",0.6);
    max_accel_=param("setpoint_acceleration",0.35);
    max_lead_=param("max_setpoint_lead",0.30);
    arrive_tol_=param("position_tolerance",0.12);
    settle_speed_=param("settle_speed",0.15);
    stable_time_=param("stable_time",0.8);
    odom_timeout_=param("odom_timeout",0.6);
    status_timeout_=param("status_timeout",2.0);
    scan_timeout_=param("scan_timeout",0.8);
    mission_timeout_=param("mission_timeout",1800.0);
    face_limit_=param("maximum_face_length",40.0);
    radius_limit_=param("mission_radius_limit",100.0);
    found_pause_=param("entrance_pause",2.0);
    fit_span_=param("wall_fit_min_span",0.6);
    fit_tol_=param("wall_fit_tolerance",0.08);
    yaw_rate_=param("yaw_rate_deg",25.0)*wc::pi/180.0;
    diagnostic_period_=param("diagnostic_period",2.0);
    stage_timeout_=param("stage_timeout",35.0);
    prestream_time_=param("prestream_time",2.0);
    yaw_tolerance_=param("yaw_tolerance_deg",8.0)*wc::pi/180.0;
    tilt_limit_=param("max_tilt_deg",12.0)*wc::pi/180.0;
    direction_=declare_parameter<int>("search_direction",1);
    gap_.confirm_frames=declare_parameter<int>("confirm_frames",3);
    const double diameter=param("vehicle_width",0.8);
    const double side_margin=param("entrance_side_margin",0.25);
    gap_.min_width=param("minimum_opening_width",1.3);
    gap_.min_width=std::max(gap_.min_width,diameter+2*side_margin);
    gap_.max_width=param("maximum_opening_width",3.5);
    if(target_system_<1 || target_system_>255 || source_system_<1 || source_system_>255 ||
       std::abs(direction_)!=1 || gap_.confirm_frames<2 || gap_.confirm_frames>30 ||
       height_<=0 || stand_off_<=sensor_offset_+min_stop_ || sensor_offset_<0 ||
       acquire_max_<=min_stop_ || gap_depth_<=wall_tol_ || wall_tol_<=0 || evidence_timeout_<=0 ||
       track_window_<=0 || track_interval_<=0 || track_yaw_rate_<=0 || track_plane_rate_<=0 || refit_time_<=0 ||
       min_stop_<=0 || search_speed_<=0 || approach_speed_<=0 || vertical_speed_<=0 ||
       takeoff_accel_<=0 || max_accel_<=0 || max_lead_<=0 || arrive_tol_<=0 || settle_speed_<=0 || stable_time_<=0 ||
       odom_timeout_<=0 || status_timeout_<=0 || scan_timeout_<=0 ||
       mission_timeout_<=0 || stage_timeout_<=0 || prestream_time_<1.5 ||
       face_limit_<=gap_.max_width+2*stand_off_ || radius_limit_<=face_limit_ ||
       found_pause_<1 || found_pause_>2 || fit_span_<=0 || fit_tol_<=0 ||
       yaw_rate_<=0 || diagnostic_period_<=0 ||
       half_angle_<=0 || half_angle_>wc::pi/3 || yaw_tolerance_<=0 || tilt_limit_<=0 ||
       gap_.max_width<=gap_.min_width || diameter<=0 || side_margin<0)
      throw std::invalid_argument("Invalid search parameters; check distances, speeds and timeouts");
    normal_={std::cos(yaw_target_),std::sin(yaw_target_)};
    tangent_={-std::sin(yaw_target_),std::cos(yaw_target_)}; // body right, NED
    auto qos=rclcpp::SensorDataQoS();
    auto odom_topic=declare_parameter<std::string>("odom_topic",outTopic<Odom>("vehicle_odometry"));
    auto status_topic=declare_parameter<std::string>("status_topic",outTopic<Status>("vehicle_status"));
    auto ack_topic=declare_parameter<std::string>("ack_topic",outTopic<Ack>("vehicle_command_ack"));
    auto scan_topic=declare_parameter<std::string>("scan_topic",prefix_+"/ultrasonic/front/scan");
    odom_sub_=create_subscription<Odom>(odom_topic,qos,
      [this](const Odom::SharedPtr m){onOdom(*m);});
    status_sub_=create_subscription<Status>(status_topic,qos,
      [this](const Status::SharedPtr m){
        if(m->timestamp==0 || m->timestamp==last_status_stamp_)return;
        if(last_status_stamp_ && m->timestamp<last_status_stamp_ && active_)
          RCLCPP_WARN(get_logger(),"PX4 status timestamp moved backwards; ignored in simulation mode");
        last_status_stamp_=m->timestamp;status_=*m;status_rx_=steady();have_status_=true;
      });
    scan_sub_=create_subscription<sensor_msgs::msg::LaserScan>(scan_topic,qos,
      [this](const sensor_msgs::msg::LaserScan::SharedPtr m){
        const int64_t stamp=int64_t(m->header.stamp.sec)*1000000000LL+m->header.stamp.nanosec;
        if(stamp<=0 || stamp==last_scan_stamp_)return;
        if(last_scan_stamp_ && stamp<last_scan_stamp_ && active_)
          RCLCPP_WARN(get_logger(),"Scan timestamp moved backwards; ignored in simulation mode");
        last_scan_stamp_=stamp;scan_=*m;scan_rx_=steady();++scan_seq_;have_scan_=true;
        // Save pose at reception. Search rejects delayed scans and excessive pose skew.
        scan_position_=position_;scan_q_=q_;
        scan_pose_received_ok_=odomFresh() && std::abs(steady()-odom_rx_)<=0.15 && tilt_<=tilt_limit_;
        refreshScanFrame();
      });
    ack_sub_=create_subscription<Ack>(ack_topic,qos,[this](const Ack::SharedPtr m){
      if(m->target_system!=0 && m->target_system!=source_system_)return;
      if(m->command!=Command::VEHICLE_CMD_DO_SET_MODE &&
         m->command!=Command::VEHICLE_CMD_COMPONENT_ARM_DISARM &&
         m->command!=Command::VEHICLE_CMD_NAV_LAND)return;
      RCLCPP_INFO(get_logger(),"PX4 ACK: command=%u result=%u reason=%d",
        unsigned(m->command),unsigned(m->result),int(m->result_param2));
      // State feedback decides success. A rejection is not silently treated as success.
    });
    offboard_pub_=create_publisher<px4_msgs::msg::OffboardControlMode>(prefix_+"/fmu/in/offboard_control_mode",10);
    trajectory_pub_=create_publisher<px4_msgs::msg::TrajectorySetpoint>(prefix_+"/fmu/in/trajectory_setpoint",10);
    command_pub_=create_publisher<Command>(prefix_+"/fmu/in/vehicle_command",10);
    auto latched=rclcpp::QoS(1).reliable().transient_local();
    state_pub_=create_publisher<std_msgs::msg::String>("~/state",latched);
    result_pub_=create_publisher<std_msgs::msg::String>("~/entrance",latched);
    user_sub_=create_subscription<std_msgs::msg::String>("~/command",10,
      [this](const std_msgs::msg::String::SharedPtr m){
        if(m->data=="start" && state_==State::WAIT_INPUT)auto_start_=true;
        else if(m->data=="hold" && active_ && odomFresh())hold("Operator HOLD");
        else if(m->data=="resume" && state_==State::HOLD && active_ && odomFresh() && statusFresh() && scanFresh() && armed() && offboard()){
          setpoint_=position_;setpoint_velocity_={0,0,0};tracking_bad_time_=alignment_bad_time_=unknown_time_=0;
          if(resume_state_==State::ALIGN_WALL){yaw_target_=fit_hint_;beginAlignment();}
          else {if(resume_state_==State::SEARCH)gap_.reset();transition(resume_state_,"Operator resume; continue interrupted stage");}
        }
        else if(m->data=="land" && have_status_ && armed()){
          if(odomFresh()){setpoint_=position_;setpoint_velocity_={0,0,0};}
          land_start_=steady();last_request_=-1e9;transition(State::LANDING,"Operator LAND requested");
        }
      });
    timer_=create_wall_timer(std::chrono::milliseconds(50),[this](){tick();});
    RCLCPP_INFO(get_logger(),"Inputs odom=%s status=%s scan=%s",odom_topic.c_str(),status_topic.c_str(),scan_topic.c_str());
    RCLCPP_INFO(get_logger(),"Build: corner-refit-20260920; ambiguity -> automatic REFIT_WALL; vector trajectory enabled");
    RCLCPP_INFO(get_logger(),"yaw(NED)=%.1f deg height=%.2f wall distance=%.2f speed=%.2f",
      yaw_target_*180/wc::pi,height_,stand_off_,search_speed_);
    RCLCPP_INFO(get_logger(),"Auto-start=%s; waits for fresh PX4 status/odom and scan. Result: entrance pause then continue; four observed corners + start overlap; no automatic crossing.",auto_start_?"true":"false");
    transition(State::WAIT_INPUT,"Waiting for valid inputs; do not run another Offboard controller");
  }
private:
  enum class State {WAIT_INPUT,PRESTREAM,REQUEST_MODE,REQUEST_ARM,TAKEOFF,
    ALIGN_WALL,REFIT_WALL,STANDOFF,SEARCH,CENTER,VERIFY,FOUND_HOLD,
    CORNER_APPROACH,CORNER_TURN,CORNER_SEEK,COMPLETE_HOLD,HOLD,LANDING,RELEASED,DONE};
  static double steady(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
  double param(const std::string& name,double value){
    const double v=declare_parameter<double>(name,value);
    if(!std::isfinite(v))throw std::invalid_argument(name+" must be finite");
    return v;
  }
  template<class T> std::string outTopic(const std::string& base) const {
    return prefix_+"/fmu/out/"+base+(MsgVersion<T>::value ? "_v"+std::to_string(MsgVersion<T>::value):"");
  }
  const char* name(State s) const {
    switch(s){
      case State::WAIT_INPUT:return "WAIT_INPUT";case State::PRESTREAM:return "PRESTREAM";
      case State::REQUEST_MODE:return "REQUEST_MODE";case State::REQUEST_ARM:return "REQUEST_ARM";
      case State::TAKEOFF:return "TAKEOFF";      case State::ALIGN_WALL:return "ALIGN_WALL";
      case State::REFIT_WALL:return "REFIT_WALL";
      case State::CORNER_APPROACH:return "CORNER_APPROACH";
      case State::CORNER_TURN:return "CORNER_TURN";
      case State::CORNER_SEEK:return "CORNER_SEEK";
      case State::COMPLETE_HOLD:return "COMPLETE_HOLD";

      case State::STANDOFF:return "STANDOFF";case State::SEARCH:return "SEARCH";
      case State::CENTER:return "CENTER";case State::VERIFY:return "VERIFY";
      case State::FOUND_HOLD:return "FOUND_HOLD";case State::HOLD:return "HOLD";
      case State::LANDING:return "LANDING";case State::RELEASED:return "RELEASED";
      case State::DONE:return "DONE";
    }return "UNKNOWN";
  }
  void transition(State s,const std::string& why){
    state_=s;state_reason_=why;stage_time_=0;settle_elapsed_=0;confirm_count_=0;invalid_scan_since_=-1;
    std_msgs::msg::String m;m.data=std::string(name(s))+": "+why;state_pub_->publish(m);
    RCLCPP_INFO(get_logger(),"%s",m.data.c_str());
  }
  void onOdom(const Odom& m){
    if(m.timestamp==0 || m.timestamp==last_odom_stamp_)return;
    if(last_odom_stamp_ && m.timestamp<last_odom_stamp_ && active_)
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,
        "Odometry timestamp rollback %.3f ms; sample=%llu; diagnostic only (check ordering/time sync/multiple publishers)",
        double(last_odom_stamp_-m.timestamp)/1000.0,static_cast<unsigned long long>(m.timestamp_sample));
    last_odom_stamp_=m.timestamp;
    if(m.pose_frame!=Odom::POSE_FRAME_NED || m.velocity_frame!=Odom::VELOCITY_FRAME_NED){odom_valid_=false;return;}
    double qnorm=0;
    for(float q:m.q){if(!std::isfinite(q)){odom_valid_=false;return;}qnorm+=double(q)*q;}
    for(size_t i=0;i<3;++i)if(!std::isfinite(m.position[i]) || !std::isfinite(m.velocity[i])){odom_valid_=false;return;}
    if(qnorm<0.5 || qnorm>1.5){odom_valid_=false;return;}
    const double inv=1/std::sqrt(qnorm),w=m.q[0]*inv,x=m.q[1]*inv,y=m.q[2]*inv,z=m.q[3]*inv;
    if(have_odom_ && active_ && m.reset_counter!=odom_reset_)
      RCLCPP_WARN(get_logger(),"PX4 estimator reset_counter changed %u -> %u; ignored in simulation mode",
        unsigned(odom_reset_),unsigned(m.reset_counter));
    odom_reset_=m.reset_counter;
    q_={w,x,y,z};
    yaw_=std::atan2(2*(w*z+x*y),1-2*(y*y+z*z));
    yaw_motion_=std::isfinite(m.angular_velocity[2]) ? std::abs(m.angular_velocity[2]):1e9;
    tilt_=std::acos(wc::limit(1-2*(x*x+y*y),-1,1));
    for(size_t i=0;i<3;++i){position_[i]=m.position[i];velocity_[i]=m.velocity[i];}
    odom_rx_=steady();have_odom_=true;odom_valid_=true;
  }
  bool armed() const{return status_.arming_state==Status::ARMING_STATE_ARMED;}
  bool offboard() const{return status_.nav_state==Status::NAVIGATION_STATE_OFFBOARD;}
  bool odomFresh() const{return have_odom_ && odom_valid_ && steady()-odom_rx_<odom_timeout_;}
  bool statusFresh() const{return have_status_ && steady()-status_rx_<status_timeout_;}
  bool scanFresh() {
    if(!have_scan_ || steady()-scan_rx_>scan_timeout_)return false;
    // Gazebo scan stamps must share /clock with the node, not wall epoch time.
    const double age=(get_clock()->now().nanoseconds()-last_scan_stamp_)*1e-9;
    return age>=-0.1 && age<scan_timeout_;
  }
  double normalPosition() const{return normal_[0]*position_[0]+normal_[1]*position_[1];}
  double alongPosition() const{return tangent_[0]*position_[0]+tangent_[1]*position_[1];}
  double speed() const{return std::sqrt(velocity_[0]*velocity_[0]+velocity_[1]*velocity_[1]+velocity_[2]*velocity_[2]);}
  double trackingError() const {
    double d=0;for(size_t i=0;i<3;++i)d+=std::pow(setpoint_[i]-position_[i],2);return std::sqrt(d);
  }
  uint64_t timestamp(){return uint64_t(std::max<int64_t>(0,get_clock()->now().nanoseconds()/1000));}
  void sendCommand(uint16_t command,float p1=0,float p2=0){
    Command m{};m.timestamp=timestamp();m.command=command;m.param1=p1;m.param2=p2;
    m.target_system=uint8_t(target_system_);m.target_component=1;
    m.source_system=uint8_t(source_system_);m.source_component=191;m.from_external=true;
    command_pub_->publish(m);last_request_=steady();
  }
  void publishControl(){
    px4_msgs::msg::OffboardControlMode mode{};mode.timestamp=timestamp();mode.position=true;
    offboard_pub_->publish(mode);
    px4_msgs::msg::TrajectorySetpoint sp{};sp.timestamp=mode.timestamp;
    const float nan=std::numeric_limits<float>::quiet_NaN();
    sp.velocity.fill(nan);sp.acceleration.fill(nan);sp.jerk.fill(nan);sp.yawspeed=nan;
    for(size_t i=0;i<3;++i)sp.position[i]=float(setpoint_[i]);
    const bool ascent=state_==State::PRESTREAM || state_==State::REQUEST_MODE ||
      state_==State::REQUEST_ARM || state_==State::TAKEOFF;
    const double commanded_yaw=ascent ? takeoff_yaw_:yaw_target_;
    yaw_command_=wc::wrap(yaw_command_+wc::limit(wc::wrap(commanded_yaw-yaw_command_),-yaw_rate_*0.05,yaw_rate_*0.05));
    sp.yaw=float(yaw_command_);trajectory_pub_->publish(sp);
  }
  void release(const std::string& why){
    active_=false;transition(State::RELEASED,why+"; stop Offboard stream, PX4/operator takes over");
  }
  void hold(const std::string& why){
    if(!odomFresh()){release(why+"; no valid position");return;}
    if(state_!=State::HOLD)resume_state_=state_;
    setpoint_=position_;setpoint_velocity_={0,0,0};transition(State::HOLD,why);
  }
  // TAKEOFF has its own vertical reference generator. Ground/spool-up delay must
  // not cap the reference at 0.30 m or reset it back to ground via tracking HOLD.
  void startTakeoff(){
    home_=position_;setpoint_=position_;setpoint_velocity_={0,0,0};
    home_n_=normalPosition();home_s_=alongPosition();target_z_=home_[2]-height_;
    takeoff_yaw_=yaw_;yaw_command_=yaw_;tracking_bad_time_=0;
    transition(State::TAKEOFF,"Armed; fixed XY/yaw, independent vertical ramp (no 0.30 m cap)");
    RCLCPP_INFO(get_logger(),"TAKEOFF target_z=%.3f speed_limit=%.2fm/s accel=%.2fm/s^2",
      target_z_,vertical_speed_,takeoff_accel_);
  }
  void advanceTakeoff(double dt){
    setpoint_[0]=home_[0];setpoint_[1]=home_[1];
    setpoint_velocity_[0]=0;setpoint_velocity_[1]=0;
    const double remaining=std::max(0.0,setpoint_[2]-target_z_);
    if(remaining<1e-6){setpoint_[2]=target_z_;setpoint_velocity_[2]=0;return;}
    const double wanted=-std::min(vertical_speed_,std::sqrt(2*takeoff_accel_*remaining));
    setpoint_velocity_[2]+=wc::limit(wanted-setpoint_velocity_[2],-takeoff_accel_*dt,takeoff_accel_*dt);
    setpoint_[2]=std::max(target_z_,setpoint_[2]+setpoint_velocity_[2]*dt);
    if(setpoint_[2]<=target_z_+1e-6)setpoint_velocity_[2]=0;
  }
  bool takeoffArrived(double dt){
    const double horizontal=std::hypot(position_[0]-home_[0],position_[1]-home_[1]);
    const bool settled=horizontal<arrive_tol_ && std::abs(position_[2]-target_z_)<arrive_tol_ && speed()<settle_speed_;
    settle_elapsed_=settled ? settle_elapsed_+dt:0;
    return settle_elapsed_>=stable_time_;
  }
  // Cruise uses bounded position references. TAKEOFF never calls this method.
  void moveTo(const std::array<double,3>& goal,double horizontal,double dt){
    if(dt<=0)return;
    if(trackingError()>max_lead_+1e-6){setpoint_velocity_={0,0,0};return;}
    std::array<double,3> want{},next=setpoint_;
    const double dx=goal[0]-setpoint_[0],dy=goal[1]-setpoint_[1];
    const double dist=std::hypot(dx,dy);
    const double v=std::min(horizontal,std::sqrt(2*max_accel_*dist));
    if(dist>1e-6){want[0]=dx/dist*v;want[1]=dy/dist*v;}
    const double dz=goal[2]-setpoint_[2];
    want[2]=std::copysign(std::min(vertical_speed_,std::sqrt(2*max_accel_*std::abs(dz))),dz);
    // Scale the XY velocity change as ONE vector. Independent axis clipping
    // changes the direction every time a diagonal trajectory accelerates.
    const double dvx=want[0]-setpoint_velocity_[0],dvy=want[1]-setpoint_velocity_[1];
    const double dv=std::hypot(dvx,dvy),gain=dv>1e-9?std::min(1.0,max_accel_*dt/dv):1.0;
    setpoint_velocity_[0]+=gain*dvx;setpoint_velocity_[1]+=gain*dvy;
    setpoint_velocity_[2]+=wc::limit(want[2]-setpoint_velocity_[2],-max_accel_*dt,max_accel_*dt);
    double xy_scale=1.0;
    for(size_t i=0;i<2;++i){
      const double step=setpoint_velocity_[i]*dt,delta=goal[i]-setpoint_[i];
      if(step*delta>0 && std::abs(step)>std::abs(delta))xy_scale=std::min(xy_scale,std::abs(delta/step));
    }
    for(size_t i=0;i<2;++i)next[i]+=setpoint_velocity_[i]*dt*xy_scale;
    const double step_z=setpoint_velocity_[2]*dt;
    next[2]+=(step_z*dz>=0 && std::abs(step_z)>std::abs(dz))?dz:step_z;
    // Intersect the proposed segment with the lead sphere. Keep the allowed
    // fraction and its velocity; do not discard an entire tick and restart XY.
    double a=0,b=0,c=-max_lead_*max_lead_;
    for(size_t i=0;i<3;++i){const double p=setpoint_[i]-position_[i],d=next[i]-setpoint_[i];a+=d*d;b+=2*p*d;c+=p*p;}
    double scale=1.0;
    if(a+b+c>0 && a>1e-15)scale=wc::limit((-b+std::sqrt(std::max(0.0,b*b-4*a*c)))/(2*a),0.0,1.0);
    for(size_t i=0;i<3;++i){const double step=(next[i]-setpoint_[i])*scale;setpoint_[i]+=step;setpoint_velocity_[i]=step/dt;}
  }
  std::array<double,3> goalAt(double n,double s) const {
    return {normal_[0]*n+tangent_[0]*s,normal_[1]*n+tangent_[1]*s,target_z_};
  }
  bool arrived(const std::array<double,3>& goal,double dt){
    double err=0;for(size_t i=0;i<3;++i)err+=std::pow(goal[i]-position_[i],2);
    const bool ok=std::sqrt(err)<arrive_tol_ && speed()<settle_speed_ &&
      std::abs(wc::wrap(yaw_-yaw_target_))<yaw_tolerance_ && tilt_<tilt_limit_;
    settle_elapsed_=ok ? settle_elapsed_+dt:0;return settle_elapsed_>=stable_time_;
  }
  wc::ScanSummary observation() const {
    wc::ScanSummary out;
    if(scan_.ranges.empty() || !std::isfinite(scan_.angle_min) || !std::isfinite(scan_.angle_increment) ||
       !std::isfinite(scan_.range_min) || !std::isfinite(scan_.range_max) || scan_.range_min<0 ||
       scan_.range_max<=scan_.range_min || (scan_.ranges.size()>1 && scan_.angle_increment==0))return out;
    const double w=scan_q_[0],x=scan_q_[1],y=scan_q_[2],z=scan_q_[3];
    const double r00=1-2*(y*y+z*z),r01=2*(x*y-w*z),r10=2*(x*y+w*z),r11=1-2*(x*x+z*z);
    const double offset=(normal_[0]*r00+normal_[1]*r10)*sensor_offset_-sensor_offset_;
    const double far=have_wall_ ? wall_plane_-scan_n_-sensor_offset_+gap_depth_:acquire_max_;
    size_t selected=0,usable=0,finite=0,far_count=0;std::vector<double> depths;
    for(size_t i=0;i<scan_.ranges.size();++i){
      const double a=scan_.angle_min+i*scan_.angle_increment+scan_yaw_,r=scan_.ranges[i];
      if(std::abs(wc::wrap(a))<wc::pi/2 && std::isfinite(r) && r>0 && r<=scan_.range_max)out.nearest=std::min(out.nearest,r);
      if(std::abs(wc::wrap(a))>half_angle_+1e-6)continue;
      ++selected;
      const double bx=std::cos(a),by=-std::sin(a);
      const double projection=normal_[0]*(r00*bx+r01*by)+normal_[1]*(r10*bx+r11*by);
      if(projection<=0.5)continue;
      if(std::isfinite(r) && r>0 && r>=scan_.range_min && r<=scan_.range_max){
        ++usable;++finite;const double d=r*projection+offset;depths.push_back(d);if(d>far)++far_count;
      }else if(std::isinf(r) && r>0 && inf_clear_ && scan_.range_max*projection+offset>far){++usable;++far_count;}
    }
    if(!selected)return out;
    out.usable_fraction=double(usable)/selected;out.finite_fraction=double(finite)/selected;
    out.far_fraction=double(far_count)/selected;out.normal_distance=wc::median(depths);
    out.valid=out.usable_fraction>=0.6;return out;
  }
  wc::Evidence evidence(const wc::ScanSummary& obs) const {
    if(!obs.valid || !scan_pose_ok_)return wc::Evidence::Unknown;
    if(obs.far_fraction>=0.8)return wc::Evidence::Open;
    if(obs.finite_fraction>=0.6 && std::isfinite(obs.normal_distance) &&
       std::abs(scan_n_+sensor_offset_+obs.normal_distance-wall_plane_)<=wall_tol_)
      return wc::Evidence::Wall;
    return wc::Evidence::Unknown;
  }
  bool idleHold() const {return state_==State::HOLD || state_==State::COMPLETE_HOLD;}
  bool aligned() const {return std::abs(wc::wrap(yaw_-yaw_target_))<yaw_tolerance_ && tilt_<tilt_limit_;}
  void refreshScanFrame(){
    scan_n_=normal_[0]*scan_position_[0]+normal_[1]*scan_position_[1];
    scan_s_=tangent_[0]*scan_position_[0]+tangent_[1]*scan_position_[1];
    const auto& q=scan_q_;
    const double sy=std::atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]));
    scan_yaw_error_=wc::wrap(sy-yaw_target_);
    scan_pose_ok_=scan_pose_received_ok_ && std::abs(scan_yaw_error_)<=yaw_tolerance_;
  }
  void setNormal(double yaw){
    yaw_target_=wc::wrap(yaw);normal_={std::cos(yaw_target_),std::sin(yaw_target_)};
    tangent_={-normal_[1],normal_[0]};refreshScanFrame();
  }
  void diagnostics(){
    const double now=steady();if(now-last_diagnostic_<diagnostic_period_)return;last_diagnostic_=now;
    RCLCPP_INFO(get_logger(),
      "STATE=%s reason=%s stage=%.1fs pos=(%.2f,%.2f,%.2f) setpoint_error=%.2fm yaw_error=%.1fdeg tilt=%.1fdeg odom_age=%.2fs scan_age=%.2fs corners=%d entrances=%zu",
      name(state_),state_reason_.c_str(),stage_time_,position_[0],position_[1],position_[2],trackingError(),
      wc::wrap(yaw_-yaw_target_)*180/wc::pi,tilt_*180/wc::pi,now-odom_rx_,now-scan_rx_,corners_,entrances_.size());
    if(have_wall_ && have_scan_){
      const auto o=observation();const auto e=evidence(o);
      RCLCPP_INFO(get_logger(),
        "WALL evidence=%s residual=%.3fm distance=%.3fm finite=%.2f far=%.2f pose_ok=%d yaw=%.2fdeg target_plane_error=%.3fm unknown=%.2fs updates=%u refits=%u",
        e==wc::Evidence::Wall?"WALL":e==wc::Evidence::Open?"OPEN":"UNKNOWN",
        scan_n_+sensor_offset_+o.normal_distance-wall_plane_,wall_plane_-normalPosition(),o.finite_fraction,o.far_fraction,int(scan_pose_ok_),
        yaw_target_*180/wc::pi,normal_[0]*setpoint_[0]+normal_[1]*setpoint_[1]+stand_off_-wall_plane_,unknown_time_,wall_updates_,refits_);
    }
  }
  void beginAlignment(){
    fit_hint_=yaw_target_;fit_phase_=0;fit_points_.clear();setpoint_=position_;setpoint_velocity_={0,0,0};
    fit_capture_elapsed_=0;fit_capture_frames_=0;fit_steps_=corner_pending_?6:4;
    fit_sweep_=(corner_pending_?65.0:30.0)*wc::pi/180;
    yaw_target_=wc::wrap(fit_hint_-fit_sweep_);transition(State::ALIGN_WALL,"Stepped yaw scan: fit wall only from settled poses");
  }
  void collectFit(){
    if(!odomFresh() || steady()-odom_rx_>0.15 || tilt_>tilt_limit_)return;
    const double w=scan_q_[0],x=scan_q_[1],y=scan_q_[2],z=scan_q_[3];
    const size_t stride=std::max<size_t>(1,scan_.ranges.size()/20);
    for(size_t i=0;i<scan_.ranges.size();i+=stride){
      const double r=scan_.ranges[i],a=scan_.angle_min+i*scan_.angle_increment+scan_yaw_;
      if(!std::isfinite(r) || r<scan_.range_min || r<=min_stop_ || r>scan_.range_max || r>acquire_max_)continue;
      const double bx=sensor_offset_+r*std::cos(a),by=-r*std::sin(a);
      // Full FRD -> NED quaternion rotation; sensor roll/pitch extrinsics assumed zero.
      const double nx=(1-2*(y*y+z*z))*bx+2*(x*y-w*z)*by;
      const double ny=2*(x*y+w*z)*bx+(1-2*(x*x+z*z))*by;
      const double nz=2*(x*z-w*y)*bx+2*(y*z+w*x)*by;
      if(std::abs(nz)>0.4)continue; // avoid mixing a distant floor/roof slice
      wc::Point2 p{scan_position_[0]+nx,scan_position_[1]+ny};
      if(corner_pending_ && old_normal_[0]*p.x+old_normal_[1]*p.y<old_plane_+0.05)continue; // exclude original facade
      // These are different sensor frames. Preserve repeated noisy returns:
      // 2.5 cm spatial deduplication reduced the real 9-ray corner to 5 points.
      // Geometric diversity is enforced by fit.span, not the frame count.
      if(fit_points_.size()<600)fit_points_.push_back(p);
    }
  }
  struct WallSample {wc::Point2 p;double time;};
  void clearWallTrack(){wall_samples_.clear();}
  // Only a continuous, close, planar SOLID patch can update the facade model.
  // Partial returns and open/gap evidence freeze it, including the closing edge.
  bool collectWallTrack(const wc::ScanSummary& obs){
    const double residual=scan_n_+sensor_offset_+obs.normal_distance-wall_plane_;
    if(gap_.pending() || !scan_pose_ok_ || !obs.valid || obs.finite_fraction<0.9 || obs.far_fraction>0.05 ||
       !std::isfinite(residual) || std::abs(residual)>gap_depth_*0.8 || yaw_motion_>4*wc::pi/180){
      clearWallTrack();return false;
    }
    const double w=scan_q_[0],x=scan_q_[1],y=scan_q_[2],z=scan_q_[3];
    std::vector<wc::Point2> points;
    const size_t stride=std::max<size_t>(1,scan_.ranges.size()/36);
    for(size_t i=0;i<scan_.ranges.size();i+=stride){
      const double a=scan_.angle_min+i*scan_.angle_increment+scan_yaw_,r=scan_.ranges[i];
      if(std::abs(a)>half_angle_+1e-6 || !std::isfinite(r) || r<scan_.range_min || r<=min_stop_ || r>acquire_max_)continue;
      const double bx=sensor_offset_+r*std::cos(a),by=-r*std::sin(a);
      const double nz=2*(x*z-w*y)*bx+2*(y*z+w*x)*by;
      if(std::abs(nz)>0.4)continue;
      points.push_back({scan_position_[0]+(1-2*(y*y+z*z))*bx+2*(x*y-w*z)*by,
                        scan_position_[1]+2*(x*y+w*z)*bx+(1-2*(x*x+z*z))*by});
    }
    if(points.size()<8){clearWallTrack();return false;}
    for(const auto& p:points)wall_samples_.push_back({p,mission_time_});
    wall_samples_.erase(std::remove_if(wall_samples_.begin(),wall_samples_.end(),
      [this](const WallSample& p){return mission_time_-p.time>track_window_;}),wall_samples_.end());
    if(wall_samples_.size()>600)wall_samples_.erase(wall_samples_.begin(),wall_samples_.end()-600);
    return true;
  }
  wc::WallFit trackedFit() const {
    if(wall_samples_.size()<24 || wall_samples_.back().time-wall_samples_.front().time<0.35)return {};
    std::vector<wc::Point2> points;for(const auto& p:wall_samples_)points.push_back(p.p);
    const auto fit=wc::fitWall(points,yaw_target_,fit_tol_,std::min(fit_span_,0.30),
      min_stop_+sensor_offset_,acquire_max_,{position_[0],position_[1]});
    if(!fit.valid || fit.count*10<points.size()*9 || fit.rms>fit_tol_*0.6 ||
       std::abs(wc::wrap(fit.yaw-yaw_target_))>20*wc::pi/180)return {};
    const double measured=fit.plane-std::cos(fit.yaw)*position_[0]-std::sin(fit.yaw)*position_[1];
    if(std::abs(measured-(wall_plane_-normalPosition()))>gap_depth_*0.8)return {};
    return fit;
  }
  void applyWallFit(const wc::WallFit& fit,bool direct){
    const double previous_yaw=yaw_target_,old_distance=wall_plane_-normalPosition();
    const double measured=fit.plane-std::cos(fit.yaw)*position_[0]-std::sin(fit.yaw)*position_[1];
    const double elapsed=wc::limit(mission_time_-last_track_update_,track_interval_,1.0);
    const double yaw=direct?fit.yaw:wc::wrap(yaw_target_+wc::limit(wc::wrap(fit.yaw-yaw_target_),
      -track_yaw_rate_*elapsed,track_yaw_rate_*elapsed));
    const double distance=direct?measured:old_distance+wc::limit(measured-old_distance,
      -track_plane_rate_*elapsed,track_plane_rate_*elapsed);
    setNormal(yaw);wall_plane_=normalPosition()+distance;have_wall_=true;
    // Coordinates used for gap widths belong to a fixed plane. Never rotate an
    // active gap; restart SOLID evidence after a permitted model update.
    gap_.reset();search_start_s_=tangent_[0]*search_origin_[0]+tangent_[1]*search_origin_[1];
    if(lap_started_ && corners_==0)lap_normal_=normal_;
    ++wall_updates_;last_track_update_=mission_time_;
    if(direct)RCLCPP_INFO(get_logger(),"Wall refit: yaw %.2f -> %.2fdeg distance=%.3fm span=%.2fm rms=%.3fm; automatic resume",
      previous_yaw*180/wc::pi,yaw_target_*180/wc::pi,distance,fit.span,fit.rms);
  }
  void updateWallTrack(const wc::ScanSummary& obs){
    if(!collectWallTrack(obs) || mission_time_-last_track_update_<track_interval_)return;
    const auto fit=trackedFit();if(fit.valid)applyWallFit(fit,false);
  }
  void beginRefit(const std::string& why,bool new_face=false){
    refit_new_face_=new_face;refit_settled_=0;clearWallTrack();++refits_;
    setpoint_=position_;setpoint_velocity_={0,0,0};tracking_bad_time_=0;
    if(new_face)gap_.reset();
    transition(State::REFIT_WALL,why+"; keep Offboard, refit then resume automatically");
  }
  void startSearch(bool new_face=true){
    gap_.reset();unknown_time_=0;last_evidence_wall_=steady();
    clearWallTrack();last_track_update_=mission_time_;
    if(new_face)search_origin_=position_;
    search_start_s_=tangent_[0]*search_origin_[0]+tangent_[1]*search_origin_[1];
    if(!lap_started_){
      lap_started_=true;lap_normal_=normal_;lap_start_=position_;
    }
    transition(State::SEARCH,"Along-wall search: wall/open/wall required; bounded open run triggers corner investigation");
  }
  void investigateCorner(){
    edge_s_=gap_.openingPosition();old_normal_=normal_;old_plane_=wall_plane_;
    corner_point_=goalAt(wall_plane_,edge_s_);
    // With a 3 m sensor an extra 0.3 m here hides most of the adjacent wall.
    corner_goal_=goalAt(wall_plane_-stand_off_,edge_s_+direction_*stand_off_);
    setpoint_=position_;setpoint_velocity_={0,0,0};
    transition(State::CORNER_APPROACH,"疑似墙壁边缘：缺少第二侧墙；先在原墙外侧确认相邻墙，再决定转角");
  }
  bool duplicateEntrance(const std::array<double,3>& centre) const {
    for(const auto& p:entrances_)if(std::hypot(p[0]-centre[0],p[1]-centre[1])<0.75)return true;
    return false;
  }
  void found(){
    setpoint_=goalAt(wall_plane_-stand_off_,candidate_s_);setpoint_velocity_={0,0,0};
    std::ostringstream s;
    const auto centre=goalAt(wall_plane_,candidate_s_);
    if(duplicateEntrance(centre)){gap_.reset();transition(State::SEARCH,"Already recorded opening; continue");return;}
    entrances_.push_back(centre);
    s<<"{\"frame\":\"px4_local_ned\",\"x\":"<<centre[0]<<",\"y\":"<<centre[1]
     <<",\"z\":"<<centre[2]<<",\"observed_width_m\":"<<candidate_width_
     <<",\"height_verified\":false,\"automatic_crossing\":false}";
    std_msgs::msg::String m;m.data=s.str();result_pub_->publish(m);
    transition(State::FOUND_HOLD,"找到入口：水平开口确认，高度/三维通行性未验证；悬停后继续搜索");
    RCLCPP_INFO(get_logger(),"找到入口 #%zu: x=%.2f y=%.2f z=%.2f width=%.2fm; pause %.1fs then continue",
      entrances_.size(),centre[0],centre[1],centre[2],candidate_width_,found_pause_);
    RCLCPP_INFO(get_logger(),"%s",m.data.c_str());
  }
  void tick(){
    const double wall_now=steady();diagnostics();
    const int64_t clock_now=get_clock()->now().nanoseconds();
    if(clock_now<=0)return;
    if(last_clock_==0){last_clock_=clock_now;return;}
    const double raw_dt=(clock_now-last_clock_)*1e-9;last_clock_=clock_now;
    if(raw_dt<0){
      if(active_){
        setpoint_=position_;setpoint_velocity_={0,0,0};
        RCLCPP_WARN(get_logger(),"ROS clock moved backwards; keeping Offboard and recapturing current setpoint");
      }
      stage_time_=0;if(active_ && odomFresh() && statusFresh())publishControl();return;
    }
    if(state_==State::DONE || state_==State::RELEASED)return;
    if(state_==State::LANDING){
      if(statusFresh() && !armed()){active_=false;transition(State::DONE,"Disarmed after landing");return;}
      if(statusFresh() && status_.nav_state==Status::NAVIGATION_STATE_AUTO_LAND){active_=false;return;}
      if(wall_now-land_start_>12){release("Land request not confirmed; check PX4");return;}
      if(wall_now-last_request_>=1)sendCommand(Command::VEHICLE_CMD_NAV_LAND);
      if(active_ && odomFresh() && statusFresh() && offboard())publishControl();
      return;
    }
    if(raw_dt==0){if(active_ && odomFresh() && statusFresh())publishControl();return;}
    if(active_ && raw_dt>0.5 && state_!=State::TAKEOFF){hold("ROS clock/timer gap over 0.5 s");}
    const double dt=std::min(raw_dt,0.1);
    if(state_==State::WAIT_INPUT){
      if(!auto_start_)return;
      if(!odomFresh() || !statusFresh() || !scanFresh()){
        RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),3000,
          "Waiting: odom=%d status=%d scan=%d. Check topic versions, use_sim_time, bridge and QoS.",
          int(odomFresh()),int(statusFresh()),int(scanFresh()));return;
      }
      if(armed()){
        RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),3000,"Already armed: refusing a new automatic takeoff; start from disarmed.");return;
      }
      // Before takeoff yaw may differ by 90 degrees. Check sensor health in
      // its own forward frame here; wall-normal projection is only valid AFTER alignment.
      const auto obs=wc::summarize(scan_.ranges,scan_.angle_min,scan_.angle_increment,
        scan_.range_min,scan_.range_max,scan_yaw_,0,half_angle_,
        std::min(acquire_max_,double(scan_.range_max)*0.8),inf_clear_);
      if(!obs.valid){RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),3000,"No usable forward scan (NaN/invalid/range too short)");return;}
      if(get_publishers_info_by_topic(prefix_+"/fmu/in/trajectory_setpoint").size()>1){
        RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),3000,"Another TrajectorySetpoint publisher exists. Stop the other controller.");return;
      }
      home_=position_;setpoint_=position_;yaw_command_=yaw_;takeoff_yaw_=yaw_;home_n_=normalPosition();home_s_=alongPosition();
      target_z_=home_[2]-height_;active_=true;mission_time_=0;
      last_request_=-1e9;transition(State::PRESTREAM,"Fresh inputs; streaming position setpoints before Offboard");
    }
    if(!active_)return;
    if(!odomFresh() || !statusFresh()){release("Odometry or status stale/invalid");return;}
    stage_time_+=dt;mission_time_+=dt;
    const bool flying=state_!=State::PRESTREAM && state_!=State::REQUEST_MODE && state_!=State::REQUEST_ARM;
    if(flying && (!armed() || !offboard())){release("Disarmed or exited Offboard; do not auto-rearm/re-enter");return;}
    if(!idleHold() && state_!=State::TAKEOFF && mission_time_>mission_timeout_){hold("Mission timeout");}
    if(flying && !idleHold() && state_!=State::TAKEOFF){
      if(!scanFresh()){hold("Front scan timeout");}
      if(trackingError()>max_lead_)tracking_bad_time_+=dt;else tracking_bad_time_=0;
      if(tracking_bad_time_>2.0)hold("Vehicle not following position target");
      if(std::hypot(position_[0]-home_[0],position_[1]-home_[1])>radius_limit_ ||
         std::abs(position_[2]-home_[2])>height_+1.0)
        hold("Mission local envelope exceeded");
    }
    if(wall_now-last_publisher_check_>1){
      last_publisher_check_=wall_now;
      if(get_publishers_info_by_topic(prefix_+"/fmu/in/trajectory_setpoint").size()>1){release("Competing controller detected");return;}
    }
    const bool new_scan=scan_seq_!=used_scan_seq_;
    wc::ScanSummary obs;
    if(new_scan){
      used_scan_seq_=scan_seq_;obs=observation();
      if(flying && !idleHold() && state_!=State::TAKEOFF && obs.nearest<min_stop_)
        hold("Obstacle closer than minimum_front_distance");
      if(flying && !idleHold() && state_!=State::TAKEOFF && state_!=State::ALIGN_WALL && state_!=State::REFIT_WALL && state_!=State::CORNER_TURN && state_!=State::CORNER_SEEK){
        if(!obs.valid){
          if(invalid_scan_since_<0)invalid_scan_since_=mission_time_;
          if(mission_time_-invalid_scan_since_>0.5)hold("Front scan contains insufficient valid readings");
        } else invalid_scan_since_=-1;
      }
    }
    switch(state_){
      case State::PRESTREAM:
        if(stage_time_>=prestream_time_)transition(State::REQUEST_MODE,"Requesting Offboard");
        break;
      case State::REQUEST_MODE:
        if(offboard()){transition(State::REQUEST_ARM,"Offboard confirmed; request arm");last_request_=-1e9;}
        else if(stage_time_>12)release("Offboard denied/timed out; inspect ACK and preflight status");
        else if(wall_now-last_request_>1)sendCommand(Command::VEHICLE_CMD_DO_SET_MODE,1,6);
        break;
      case State::REQUEST_ARM:
        if(!offboard()){release("Mode lost before arm");break;}
        if(armed())startTakeoff();
        else if(stage_time_>12)release("Arm denied/timed out; inspect PX4 ACK");
        else if(wall_now-last_request_>1)sendCommand(Command::VEHICLE_CMD_COMPONENT_ARM_DISARM,1);
        break;
      case State::TAKEOFF:{
        advanceTakeoff(dt);
        if(takeoffArrived(dt)){
          setpoint_=position_;setpoint_velocity_={0,0,0};beginAlignment();
        }else if(stage_time_>stage_timeout_){
          RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),3000,
            "TAKEOFF waiting: actual_z=%.3f commanded_z=%.3f final_z=%.3f; keep target, inspect PX4 if no climb",
            position_[2],setpoint_[2],target_z_);
        }
        break;
      }
      case State::ALIGN_WALL:{
        // Collect only after the position AND yaw have settled at each step.
        // A continuous sweep pairs delayed ranges with a newer yaw, shearing
        // the fitted wall; this was especially visible in the short corner fit.
        if(arrived(setpoint_,dt) && yaw_motion_<4*wc::pi/180){
          if(new_scan){collectFit();++fit_capture_frames_;}
          fit_capture_elapsed_+=dt;
          if(fit_capture_elapsed_>=0.35 && fit_capture_frames_>=3){
            if(fit_phase_<fit_steps_){
              ++fit_phase_;yaw_target_=wc::wrap(fit_hint_-fit_sweep_+2*fit_sweep_*fit_phase_/fit_steps_);
              settle_elapsed_=fit_capture_elapsed_=0;fit_capture_frames_=0;break;
            }
            const auto fit=wc::fitWall(fit_points_,fit_hint_,fit_tol_,fit_span_,min_stop_+sensor_offset_,acquire_max_,{position_[0],position_[1]});
            if(!fit.valid){hold("Cannot fit a wide planar wall: evidence insufficient; inspection incomplete. Check starting solid wall, range, corner visibility; resume to retry");break;}
            const bool was_corner=corner_pending_;
            if(corner_pending_){
              const double predicted=std::cos(fit.yaw)*corner_point_[0]+std::sin(fit.yaw)*corner_point_[1];
              if(std::abs(fit.plane-predicted)>0.6 || std::abs(wc::wrap(fit.yaw-fit_hint_))>15*wc::pi/180){
                hold("Adjacent wall inconsistent with a rectangular corner; inspection incomplete");break;
              }
              ++corners_;corner_pending_=false;
              RCLCPP_INFO(get_logger(),"墙壁边缘：相邻墙面已确认；第 %d 个墙角，继续绕行",corners_);
            }
            setNormal(fit.yaw);wall_plane_=fit.plane;have_wall_=true;standoff_s_=alongPosition();
            if(corners_==4 && (std::abs(wc::wrap(yaw_target_-std::atan2(lap_normal_[1],lap_normal_[0])))>15*wc::pi/180 ||
                std::abs(wall_plane_-(normal_[0]*lap_start_[0]+normal_[1]*lap_start_[1]+stand_off_))>0.6)){
              hold("Four turns but original wall not recovered; cannot claim completion");break;
            }
            RCLCPP_INFO(get_logger(),"Wall fit: yaw=%.1fdeg span=%.2fm inliers=%zu",yaw_target_*180/wc::pi,fit.span,fit.count);
            if(was_corner){
              corner_goal_={position_[0]+old_normal_[0]*(gap_.max_width+2*stand_off_+2),
                            position_[1]+old_normal_[1]*(gap_.max_width+2*stand_off_+2),target_z_};
              plane_samples_.clear();transition(State::CORNER_SEEK,"Adjacent wall observed from outside; move around verified convex corner");
            }else {preserve_search_origin_=false;transition(State::STANDOFF,"Wall normal measured; align heading and settle at stand-off");}
          }
        }
        if(state_==State::ALIGN_WALL && stage_time_>stage_timeout_)hold("Wall alignment timed out; inspect STATE yaw_error/tilt and scan geometry");
        break;
      }
      case State::REFIT_WALL:{
        const bool settled=arrived(setpoint_,dt);
        if(settled){
          refit_settled_+=dt;
          if(new_scan)collectWallTrack(obs);
        }else refit_settled_=0;
        if(refit_settled_>=refit_time_){
          const auto fit=trackedFit();
          if(fit.valid && !gap_.pending()){
            applyWallFit(fit,true);unknown_time_=0;last_evidence_wall_=wall_now;
            standoff_s_=alongPosition();preserve_search_origin_=!refit_new_face_;
            transition(State::STANDOFF,"Wall model corrected; restore wall distance then resume search");
          }else{
            // A mixed wall/empty fan needs a little more lateral baseline. Keep
            // the existing plane and gap history; never fit a doorway back wall.
            unknown_time_=0;last_evidence_wall_=wall_now;clearWallTrack();slow_until_=mission_time_+3.0;
            if(refit_new_face_){
              standoff_s_=alongPosition();preserve_search_origin_=false;
              transition(State::STANDOFF,"Local refit inconclusive; retain verified adjacent plane and continue");
            }else transition(State::SEARCH,"Refit inconclusive/edge transition; keep frozen plane and continue slowly");
          }
        }
        break;
      }
      case State::CORNER_APPROACH:
        moveTo(corner_goal_,search_speed_,dt);
        if(arrived(corner_goal_,dt)){
          setpoint_=position_;setpoint_velocity_={0,0,0};
          setNormal(yaw_target_-direction_*wc::pi/2);have_wall_=false;
          transition(State::CORNER_TURN,"Rotate outside corner, then seek perpendicular wall");
        }else if(stage_time_>stage_timeout_+gap_.max_width/search_speed_)hold("Cannot reach exterior corner staging point");
        break;
      case State::CORNER_TURN:
        if(arrived(setpoint_,dt)){
          corner_pending_=true;beginAlignment();
        }else if(stage_time_>stage_timeout_)hold("Corner yaw alignment timeout");
        break;
      case State::CORNER_SEEK:{
        const double old_n=old_normal_[0]*position_[0]+old_normal_[1]*position_[1];
        const double expected=normal_[0]*corner_point_[0]+normal_[1]*corner_point_[1];
        if(new_scan){
          const bool wall=obs.valid && scan_pose_ok_ && obs.finite_fraction>=0.6 && std::isfinite(obs.normal_distance) &&
            std::abs(scan_n_+sensor_offset_+obs.normal_distance-expected)<0.6 && old_n>old_plane_+0.35;
          if(wall)plane_samples_.push_back(obs.normal_distance);else plane_samples_.clear();
          if(plane_samples_.size()>=size_t(gap_.confirm_frames+2)){
            beginRefit("Reached adjacent facade; refine wall normal from stationary frontal returns",true);break;
          }
        }
        if(aligned())moveTo(corner_goal_,search_speed_,dt);
        if(stage_time_>(gap_.max_width+2*stand_off_+2)/search_speed_+stage_timeout_)
          hold("No adjacent wall confirmed: may be a wide opening, isolated obstacle, or non-rectangular building; incomplete");
        break;
      }
      case State::STANDOFF:{
        const auto goal=goalAt(wall_plane_-stand_off_,standoff_s_);
        moveTo(goal,approach_speed_,dt);
        if(arrived(goal,dt)){
          startSearch(!preserve_search_origin_);preserve_search_origin_=false;
        }else if(stage_time_>stage_timeout_)hold("Stand-off manoeuvre timeout");
        break;
      }
      case State::SEARCH:{
        if(new_scan){
          updateWallTrack(obs);
          obs=observation(); // the model may have rotated; use the same saved scan pose
          const auto e=evidence(obs);
          if(e==wc::Evidence::Unknown)unknown_time_+=wc::limit(wall_now-last_evidence_wall_,0.0,0.25);
          else unknown_time_=0;
          last_evidence_wall_=wall_now;
          const auto result=gap_.update(e,scan_s_);
          if(result.found){
            candidate_s_=(result.first+result.second)*0.5;candidate_width_=result.width;
            if(!duplicateEntrance(goalAt(wall_plane_,candidate_s_))){
              transition(State::CENTER,"Both wall edges found on same plane; return to opening midpoint");break;
            }
          }
          if(unknown_time_>evidence_timeout_){beginRefit("Wall/scan mismatch: automatic stationary re-fit");break;}
        }
        if(corners_==4){
          const double start_s=tangent_[0]*lap_start_[0]+tangent_[1]*lap_start_[1];
          if(direction_*(alongPosition()-start_s)>0.35 && !gap_.in_gap && new_scan && aligned() && evidence(obs)==wc::Evidence::Wall){
            setpoint_=position_;setpoint_velocity_={0,0,0};
            transition(State::COMPLETE_HOLD,"本高度外墙巡检一圈完成：四个相邻墙角已验证，并与起始墙段重叠；保持悬停");break;
          }
        }
        if(gap_.tooWide(alongPosition())){
          if(corners_>=4){hold("Passed four corners without start overlap; incomplete");break;}
          investigateCorner();break;
        }
        const double end=search_start_s_+direction_*face_limit_;
        if(direction_*(end-alongPosition())<arrive_tol_){hold("maximum_face_length reached without verified edge; inspection incomplete");break;}
        if(std::abs(wc::wrap(yaw_-yaw_target_))>yaw_tolerance_ || tilt_>tilt_limit_){
          setpoint_=position_;setpoint_velocity_={0,0,0};alignment_bad_time_+=dt;
          if(alignment_bad_time_>8)hold("Wall-facing attitude not maintained; resume after tilt/yaw recover");
          break;
        }
        alignment_bad_time_=0;
        // A nearby along-wall goal corrects normal error promptly. The global
        // face limit remains anchored to search_origin_ across model/refit changes.
        const double ahead=alongPosition()+direction_*std::min(1.0,std::abs(end-alongPosition()));
        const double cruise=mission_time_<slow_until_?std::min(search_speed_,0.12):search_speed_;
        moveTo(goalAt(wall_plane_-stand_off_,ahead),cruise,dt);break;
      }
      case State::CENTER:{
        const auto goal=goalAt(wall_plane_-stand_off_,candidate_s_);
        moveTo(goal,search_speed_,dt);
        if(arrived(goal,dt)){verify_elapsed_=0;transition(State::VERIFY,"Centred; confirm open slice using fresh scans");}
        else if(stage_time_>stage_timeout_+gap_.max_width/search_speed_)hold("Could not centre on candidate");
        break;
      }
      case State::VERIFY:{
        const bool settled=arrived(goalAt(wall_plane_-stand_off_,candidate_s_),dt);
        if(!settled){confirm_count_=0;verify_elapsed_=0;}
        if(new_scan && settled){
          if(evidence(obs)==wc::Evidence::Open)++confirm_count_;
          else {confirm_count_=0;verify_elapsed_=0;}
        }
        if(settled && confirm_count_>=gap_.confirm_frames)verify_elapsed_+=dt;
        if(verify_elapsed_>=stable_time_)found();
        else if(stage_time_>8){
          gap_.reset();transition(State::SEARCH,"Candidate rejected by centre recheck; continue facade search");
        }
        break;
      }
      case State::FOUND_HOLD:
        if(stage_time_>=found_pause_){
          gap_.reset();unknown_time_=0;last_evidence_wall_=wall_now;
          transition(State::SEARCH,"入口记录完成，继续寻找下一个入口");
        }
        break;
      case State::HOLD:case State::COMPLETE_HOLD:break;
      default:break;
    }
    if(active_)publishControl();
  }

  std::string prefix_,state_reason_;
  State resume_state_{State::TAKEOFF};
  double face_limit_,radius_limit_,found_pause_,fit_span_,fit_tol_,yaw_rate_,diagnostic_period_;
  double track_window_,track_interval_,track_yaw_rate_,track_plane_rate_,refit_time_;
  double last_track_update_{0},refit_settled_{0},slow_until_{0},fit_capture_elapsed_{0},yaw_motion_{0};
  unsigned wall_updates_{0},refits_{0};int fit_steps_{4},fit_capture_frames_{0};
  bool refit_new_face_{false},preserve_search_origin_{false},scan_pose_received_ok_{false};
  std::vector<WallSample> wall_samples_;
  std::array<double,3> search_origin_{};
  double yaw_command_{0},last_diagnostic_{-1e9},fit_hint_{0},fit_sweep_{0},search_start_s_{0},
    edge_s_{0},old_plane_{0};
  int fit_phase_{0},corners_{0};bool lap_started_{false},corner_pending_{false};
  std::array<double,4> q_{1,0,0,0},scan_q_{1,0,0,0};
  std::array<double,3> scan_position_{},corner_point_{},corner_goal_{},lap_start_{};
  std::array<double,2> old_normal_{},lap_normal_{};
  std::vector<wc::Point2> fit_points_;
  std::vector<std::array<double,3>> entrances_;
  int target_system_{1},source_system_{245},direction_{1},confirm_count_{0};
  bool auto_start_{true},inf_clear_{true},active_{false},have_odom_{false},have_status_{false},
    have_scan_{false},odom_valid_{false},have_wall_{false},scan_pose_ok_{false};
  double takeoff_accel_,takeoff_yaw_{0};
  double height_,yaw_target_,sensor_offset_,scan_yaw_,half_angle_,stand_off_,min_stop_,acquire_max_,
    gap_depth_,wall_tol_,evidence_timeout_,search_speed_,approach_speed_,vertical_speed_,max_accel_,max_lead_,
    arrive_tol_,settle_speed_,stable_time_,odom_timeout_,
    status_timeout_,scan_timeout_,mission_timeout_,stage_timeout_,prestream_time_,yaw_tolerance_,tilt_limit_;
  double yaw_{0},tilt_{0},home_n_{0},home_s_{0},target_z_{0},wall_plane_{0},standoff_s_{0},
    candidate_s_{0},candidate_width_{0},scan_n_{0},scan_s_{0},scan_yaw_error_{0},
    odom_rx_{-1e9},status_rx_{-1e9},scan_rx_{-1e9},last_request_{-1e9},last_publisher_check_{-1e9},
    stage_time_{0},mission_time_{0},settle_elapsed_{0},tracking_bad_time_{0},unknown_time_{0},
    invalid_scan_since_{-1},alignment_bad_time_{0},last_evidence_wall_{0},verify_elapsed_{0},land_start_{0};
  uint64_t last_odom_stamp_{0},last_status_stamp_{0},scan_seq_{0},used_scan_seq_{0};
  int64_t last_scan_stamp_{0},last_clock_{0};uint8_t odom_reset_{0};
  std::array<double,2> normal_{},tangent_{};
  std::array<double,3> position_{},velocity_{},home_{},setpoint_{},setpoint_velocity_{};
  std::vector<double> plane_samples_;
  wc::GapTracker gap_;State state_{State::WAIT_INPUT};Status status_{};
  sensor_msgs::msg::LaserScan scan_;
  rclcpp::Subscription<Odom>::SharedPtr odom_sub_;
  rclcpp::Subscription<Status>::SharedPtr status_sub_;
  rclcpp::Subscription<Ack>::SharedPtr ack_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr user_sub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<Command>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_,result_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc,char** argv){
  rclcpp::init(argc,argv);
  try{rclcpp::spin(std::make_shared<EntranceWallSearch>());}
  catch(const std::exception& e){
    RCLCPP_FATAL(rclcpp::get_logger("entrance_wall_search"),"%s",e.what());
    rclcpp::shutdown();return 1;
  }
  rclcpp::shutdown();return 0;
}