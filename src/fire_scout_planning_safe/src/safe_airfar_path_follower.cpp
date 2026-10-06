#include "fire_scout/ros_utils.hpp"
#include "fire_scout/tracker.hpp"
#include "fire_scout/execution_permit.hpp"
#include "fire_scout/passage_scheduler.hpp"
#include "fire_scout/route_topology.hpp"
#include "fire_scout/hold_safety.hpp"
#include "fire_scout/directional_motion.hpp"
#include "fire_scout/glass_risk.hpp"
#include "fire_scout/local_recovery.hpp"
#include "fire_scout/navigation_contract.hpp"
#include "fire_scout/autonomous_recovery.hpp"
#include "fire_scout/envelope_escape.hpp"
#include "fire_scout/px4_command_clock.hpp"
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <chrono>
#include <deque>
#include "fire_scout/retreat_route.hpp"
#include "fire_scout/input_freshness.hpp"
#include "fire_scout/map_pair_buffer.hpp"
#include <std_msgs/msg/bool.hpp>
#include "fire_scout/px4_topics.hpp"
#include "fire_scout/ultrasonic.hpp"
#include "fire_scout/pose_history.hpp"
#include "fire_scout/peer_safety.hpp"
#include <sstream>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/trigger.hpp>
using namespace fire_scout;
class SafeAirFarPathFollower : public rclcpp::Node {
public:
  SafeAirFarPathFollower()
      : Node("safe_airfar_path_follower"), tf_(get_clock()), listener_(tf_) {
    declare_parameter<std::string>("runtime_version","2.1.2-geometric-cooperative");
    px4_clock_.require_sim_clock=get_parameter("use_sim_time").as_bool();
    px4_clock_.policy=declare_parameter<std::string>("px4_timestamp_clock","auto");
    if(!px4_clock_.validPolicy()||(px4_clock_.require_sim_clock&&px4_clock_.policy=="system"))
      throw std::runtime_error("Gazebo use_sim_time requires auto/ros clock with PX4 UXRCE_DDS_SYNCT=0; real hardware may use system");
    separation_enabled_=declare_parameter("envelope_separation_enabled",true);
    separation_max_overlap_=declare_parameter("envelope_separation_max_overlap",.15);
    separation_speed_=declare_parameter("envelope_separation_speed",.10);
    if(!std::isfinite(separation_max_overlap_)||separation_max_overlap_<=0||separation_max_overlap_>.18||
       !std::isfinite(separation_speed_)||separation_speed_<.05||separation_speed_>.12)
      throw std::runtime_error("Invalid bounded envelope-separation configuration");
    frame_ = declare_parameter<std::string>("frame_id", "scout1/odom");
    grid_ = std::make_unique<Grid>(gridParameters(*this));
    gap_config_=gapParameters(*this);
    planner_execution_timeout_=declare_parameter("planner_execution_timeout",.8);
    passage_scheduler_enabled_=declare_parameter("passage_scheduler_enabled",false);
    passage_width_=declare_parameter("passage_admission_width",2.6);
    passage_lookahead_=declare_parameter("passage_admission_lookahead",5.0);
    passage_wait_distance_=declare_parameter("passage_wait_distance",2.0);
    passage_exit_distance_=declare_parameter("passage_release_distance",2.2);
    passage_lease_timeout_=declare_parameter("passage_lease_timeout",.8);
    passage_detect_period_=declare_parameter("passage_detect_period",.25);
    passage_wait_timeout_=declare_parameter("passage_wait_timeout",8.0);
    passage_wait_retry_=declare_parameter("passage_wait_retry_period",4.0);
    for(double x:{planner_execution_timeout_,passage_width_,passage_lookahead_,
      passage_wait_distance_,passage_exit_distance_,passage_lease_timeout_,
      passage_detect_period_,passage_wait_timeout_,passage_wait_retry_})
      if(!std::isfinite(x)||x<=0)throw std::runtime_error("Invalid route/passsage timing");
    if(planner_execution_timeout_<.2 || planner_execution_timeout_>2 ||
       passage_width_<2*grid_->cfg.inflation_xy || passage_width_>8 ||
       passage_lookahead_<1 || passage_lookahead_>8 ||
       passage_wait_distance_<=grid_->cfg.inflation_xy || passage_wait_distance_>4 ||
       passage_exit_distance_<=1.8 || passage_exit_distance_>6 ||
       passage_lease_timeout_<.2 || passage_lease_timeout_>3 ||
       passage_detect_period_<.05 || passage_detect_period_>1 ||
       passage_wait_timeout_<2 || passage_wait_timeout_>120 ||
       passage_wait_retry_<1 || passage_wait_retry_>60)
      throw std::runtime_error("Invalid route/passsage bounds");
    peer_safety_config_.enabled=declare_parameter("peer_safety_enabled",true);
    const double configured_peer_hard=declare_parameter(
      "peer_hard_separation",0.0);
    const double configured_peer_vertical=declare_parameter(
      "peer_vertical_separation",0.0);
    if(!std::isfinite(configured_peer_hard)||configured_peer_hard<0||
       !std::isfinite(configured_peer_vertical)||configured_peer_vertical<0)
      throw std::runtime_error("Peer separation overrides must be zero (automatic) or positive");
    peer_safety_config_.hard_separation=configured_peer_hard>0?
      configured_peer_hard:2*grid_->cfg.inflation_xy;
    peer_safety_config_.vertical_separation=configured_peer_vertical>0?
      configured_peer_vertical:2*grid_->cfg.inflation_z;
    peer_safety_config_.reaction_time=declare_parameter("peer_reaction_time",.45);
    peer_safety_config_.brake_accel=declare_parameter("peer_brake_accel",1.0);
    peer_safety_config_.prediction_horizon=declare_parameter(
      "peer_prediction_horizon",2.0);
    peer_safety_config_.passage_width=declare_parameter(
      "peer_passage_width",2*grid_->cfg.inflation_xy+.40);
    peer_safety_config_.passage_lookahead=declare_parameter(
      "peer_passage_lookahead",2.5);
    peer_safety_config_.passage_reservation_radius=declare_parameter(
      "peer_passage_reservation_radius",1.60);
    peer_safety_config_.passage_priority_hysteresis=declare_parameter(
      "peer_passage_priority_hysteresis",.20);
    peer_safety_config_.separation_escape_speed=declare_parameter(
      "peer_separation_escape_speed",.12);
    peer_safety_config_.rear_ignore_distance=declare_parameter(
      "peer_rear_ignore_distance",.15);
    peer_safety_config_.yield_retreat_speed=declare_parameter(
      "peer_yield_retreat_speed",.18);
    peer_yield_watchdog_.cfg.no_progress_timeout=declare_parameter(
      "peer_yield_deadlock_timeout",3.0);
    peer_yield_watchdog_.cfg.blocker_progress_distance=declare_parameter(
      "peer_blocker_progress_distance",.12);
    peer_yield_replan_period_=declare_parameter(
      "peer_yield_replan_period",1.0);
    peer_odom_timeout_=declare_parameter("peer_safety_odom_timeout",.80);
    peer_tf_timeout_=declare_parameter("peer_safety_tf_wait_timeout",0.0);
    own_priority_=static_cast<int>(declare_parameter<int64_t>("own_priority",0));
    const auto peer_topics=declare_parameter<std::vector<std::string>>(
      "peer_odom_topics",std::vector<std::string>{});
    const auto peer_priority_values=declare_parameter<std::vector<int64_t>>(
      "peer_priorities",std::vector<int64_t>{});
    if(!peer_safety_config_.valid() || !peer_yield_watchdog_.cfg.valid() ||
       !std::isfinite(peer_yield_replan_period_) ||
       peer_yield_replan_period_<=0 || peer_yield_replan_period_>10 ||
       !std::isfinite(peer_odom_timeout_) ||
       peer_odom_timeout_<=0 || !std::isfinite(peer_tf_timeout_) ||
       peer_tf_timeout_<0 || peer_tf_timeout_>.05 || own_priority_<0 ||
       peer_topics.size()!=peer_priority_values.size())
      throw std::runtime_error("Invalid peer safety configuration/topics");
    local_map_extent_=declare_parameter("local_map_half_extent",6.0);
    if(!std::isfinite(local_map_extent_)||local_map_extent_<4||local_map_extent_>30)
      throw std::runtime_error("local_map_half_extent must be in [4,30]");
    recovery_.cfg.enabled=declare_parameter("local_recovery_enabled",true);
    recovery_.cfg.wait=declare_parameter("local_recovery_wait",.9);
    recovery_.cfg.duration=declare_parameter("local_recovery_duration",3.5);
    recovery_.cfg.distance=declare_parameter("local_recovery_distance",.45);
    recovery_.cfg.speed=declare_parameter("local_recovery_speed",.25);
    recovery_.cfg.attempts=declare_parameter("local_recovery_attempts",2);
    if(!recovery_.cfg.valid())throw std::runtime_error("Invalid bounded recovery configuration");
    autonomous_recovery_=declare_parameter("autonomous_recovery",true);
    campaign_.cfg.max_distance=declare_parameter("recovery_total_distance",4.0);
    campaign_.cfg.max_actions=declare_parameter("recovery_total_actions",6);
    campaign_.cfg.cooldown=declare_parameter("recovery_retry_cooldown",2.5);
    escape_config_.max_length=declare_parameter("observed_escape_max_length",1.20);
    escape_config_.max_height_change=declare_parameter("observed_escape_height_change",.50);
    approach_limit_=declare_parameter("goal_approach_max_offset",.35);
    if(!campaign_.cfg.valid()||!std::isfinite(escape_config_.max_length)||escape_config_.max_length<.4||escape_config_.max_length>1.5||
       !std::isfinite(escape_config_.max_height_change)||escape_config_.max_height_change<0||escape_config_.max_height_change>.75||
       !std::isfinite(approach_limit_)||approach_limit_<.05||approach_limit_>.60)
      throw std::runtime_error("Invalid autonomous recovery/goal approach configuration");
    TrackerConfig c;
    c.lookahead = declare_parameter("lookahead", .65);
    c.max_speed_xy = declare_parameter("max_speed_xy", 1.8);
    c.max_speed_z = declare_parameter("max_speed_z", .50);
    c.max_accel_xy = declare_parameter("max_accel_xy", 1.5);
    c.max_accel_z = declare_parameter("max_accel_z", .70);
    c.kp_xy = declare_parameter("kp_xy", 1.2);
    c.kp_z = declare_parameter("kp_z", 1.0);
    c.kd_xy = declare_parameter("kd_xy", .10);
    c.kd_z = declare_parameter("kd_z", .20);
    c.cross_track_gain = declare_parameter("cross_track_gain", .9);
    c.cross_track_damping = declare_parameter("cross_track_damping", .65);
    c.cross_track_speed = declare_parameter("cross_track_speed", .45);
    c.goal_tolerance = declare_parameter("goal_tolerance", .18);
    c.yaw_rate = declare_parameter("max_yaw_rate", 1.2);
    c.yaw_accel = declare_parameter("max_yaw_accel", 1.2);
    c.yaw_deadband = declare_parameter("yaw_deadband", .025);
    c.yaw_preview_time = declare_parameter("yaw_preview_time", .30);
    c.max_jerk_xy=declare_parameter("max_jerk_xy",4.5);
    c.max_jerk_z=declare_parameter("max_jerk_z",3.0);
    c.lateral_accel=declare_parameter("max_lateral_accel",1.0);
    c.face_motion=declare_parameter("face_motion",true);
    adaptive_direction_=declare_parameter("adaptive_direction",true);
    c.relaxed_lateral_speed=declare_parameter("lateral_speed_limit",.60);
    c.relaxed_reverse_speed=declare_parameter("reverse_speed_limit",.40);
    direction_config_.max_lateral_speed=c.relaxed_lateral_speed;
    direction_config_.max_reverse_speed=c.relaxed_reverse_speed;
    nonforward_budget_.cfg.max_distance=declare_parameter("nonforward_max_distance",1.50);
    nonforward_budget_.cfg.max_seconds=declare_parameter("nonforward_max_seconds",8.0);
    nonforward_budget_.cfg.reset_forward_distance=declare_parameter("nonforward_reset_distance",.30);
    trail_.cfg.max_age=declare_parameter("retreat_history_seconds",10.0);
    trail_.cfg.tube_radius=declare_parameter("retreat_tracking_tolerance",.04);
    historical_retreat_enabled_=declare_parameter("historical_retreat_enabled",true);
    retreat_history_.cfg.history_seconds=declare_parameter("historical_retreat_seconds",30.0);
    retreat_history_.cfg.max_length=declare_parameter("historical_retreat_max_distance",.8);
    retreat_history_.cfg.preferred_length=std::min(.7,retreat_history_.cfg.max_length);
    retreat_history_.cfg.min_displacement=declare_parameter("historical_retreat_min_displacement",.35);
    retreat_history_.cfg.tube_radius=trail_.cfg.tube_radius;
    retreat_duration_=declare_parameter("historical_retreat_duration",8.0);
    if(!retreat_history_.cfg.valid() || !std::isfinite(retreat_duration_) ||
       retreat_duration_<1 || retreat_duration_>8)
      throw std::runtime_error("Invalid historical retreat limits");
    glass_risk_distance_=declare_parameter("glass_risk_distance",1.0);
    glass_release_delay_=declare_parameter("glass_release_delay",1.0);
    glass_boxes_=parseRiskBoxes(declare_parameter<std::vector<double>>("glass_risk_boxes",std::vector<double>{}));
    glass_topic_=declare_parameter<std::string>("glass_risk_topic","");
    glass_signal_timeout_=declare_parameter("glass_signal_timeout",1.0);
    if(!nonforward_budget_.cfg.valid() || !trail_.cfg.valid() ||
       !std::isfinite(glass_risk_distance_) || glass_risk_distance_<0 ||
       !std::isfinite(glass_release_delay_) || glass_release_delay_<0 ||
       !std::isfinite(glass_signal_timeout_) || glass_signal_timeout_<=0 ||
       trail_.cfg.tube_radius>.04 ||
       trail_.cfg.tube_radius>get_parameter("tracking_error_margin").as_double() ||
       trail_.cfg.tube_radius>get_parameter("safety_margin_z").as_double())
      throw std::runtime_error("Invalid directional risk/history parameters");
    c.heading_speed_scaling_enabled=declare_parameter("heading_speed_scaling_enabled",true);
    c.allow_forward_during_alignment=declare_parameter("allow_forward_during_alignment",true);
    c.heading_full_speed=declare_parameter("heading_full_speed_deg",10.0)*pi/180;
    c.heading_slow=declare_parameter("heading_slow_deg",25.0)*pi/180;
    c.heading_min_speed=declare_parameter("heading_min_speed_deg",60.0)*pi/180;
    c.heading_stop=declare_parameter("heading_stop_deg",85.0)*pi/180;
    c.heading_slow_speed_scale=declare_parameter("heading_slow_speed_scale",.60);
    c.heading_min_speed_scale=declare_parameter("heading_min_speed_scale",.25);
    c.min_forward_speed=declare_parameter("min_forward_speed",.12);
    c.narrow_speed=declare_parameter("narrow_passage_speed",.50);
    c.clearance_slow_distance=declare_parameter("clearance_slow_distance",.40);
    c.sharp_turn=declare_parameter("sharp_turn_stop_deg",80.0)*pi/180;
    if(c.max_jerk_xy<=0 || c.max_jerk_z<=0 || c.lateral_accel<=0 ||
       c.heading_full_speed<0 || c.heading_slow<=c.heading_full_speed ||
       c.heading_min_speed<=c.heading_slow || c.heading_stop<=c.heading_min_speed ||
       c.heading_stop>=pi/2 || c.heading_min_speed_scale<=0 ||
       c.heading_slow_speed_scale<c.heading_min_speed_scale || c.heading_slow_speed_scale>1 ||
       c.min_forward_speed<=0 || c.min_forward_speed>c.max_speed_xy ||
       c.narrow_speed<=0 || c.clearance_slow_distance<=0)
      throw std::runtime_error("Invalid dynamic/heading limits");
    map_timeout_=declare_parameter("map_timeout",5.0);
    map_age_slow_start_=declare_parameter("map_age_slow_start",1.5);
    if(!std::isfinite(map_age_slow_start_)||map_age_slow_start_<0||map_age_slow_start_>=map_timeout_)
      throw std::runtime_error("map_age_slow_start must be below map_timeout");
    takeoff_altitude_ = declare_parameter("takeoff_altitude", 1.5);
    prestream_seconds_ = declare_parameter("prestream_seconds", 1.5);
    startup_spread_enabled_=declare_parameter("startup_spread_enabled",true);
    startup_spread_offset_x_=declare_parameter("startup_spread_offset_x",0.0);
    startup_spread_offset_y_=declare_parameter("startup_spread_offset_y",0.0);
    startup_spread_spacing_=declare_parameter("startup_spread_spacing",9.0);
    startup_formation_rank_=static_cast<int>(declare_parameter<int64_t>(
      "startup_formation_rank",0));
    startup_formation_count_=static_cast<int>(declare_parameter<int64_t>(
      "startup_formation_count",1));
    startup_spread_speed_=declare_parameter("startup_spread_speed",.80);
    startup_spread_tolerance_=declare_parameter("startup_spread_tolerance",.25);
    startup_spread_obstacle_lookahead_=declare_parameter(
      "startup_spread_obstacle_lookahead",1.0);
    path_timeout_ = declare_parameter("path_timeout", 5.0);
    odom_timeout_ = declare_parameter("odom_timeout", 1.2);
    status_timeout_ = declare_parameter("status_timeout", 3.0);
    double rate = declare_parameter("control_rate_hz", 40.0);
    auto_arm_ = declare_parameter("auto_arm", true);
    use_ultrasonic_ = declare_parameter("use_ultrasonic_guard", true);
    require_ultrasonic_ = declare_parameter("require_ultrasonic", false);
    stop_distance_ = declare_parameter("stop_distance", .45);
    resume_distance_ = declare_parameter("resume_distance", .60);
    ultrasonic_timeout_ = declare_parameter("ultrasonic_timeout", .80);
    sonar_pose_tolerance_ = declare_parameter("ultrasonic_pose_tolerance", .08);
    sonar_pose_wait_timeout_ =
        declare_parameter("ultrasonic_pose_wait_timeout", .80);
    const int64_t sonar_pending_queue_size =
        declare_parameter<int64_t>("ultrasonic_pending_queue_size", 32);
    sonar_pending_queue_size_ = sonar_pending_queue_size > 0
                                    ? static_cast<size_t>(sonar_pending_queue_size)
                                    : 0;
    sonar_half_angle_=declare_parameter("ultrasonic_half_angle_deg",10.0)*pi/180;
    sonar_yaw_=declare_parameter("ultrasonic_yaw_deg",0.0)*pi/180;
    sonar_offset_={declare_parameter("ultrasonic_offset_x",.15),declare_parameter("ultrasonic_offset_y",0.0),declare_parameter("ultrasonic_offset_z",0.0)};
    sonar_inf_clear_=declare_parameter("ultrasonic_inf_is_clear",true);
    SonarGuardConfig sg;
    sg.required=require_ultrasonic_;
    sg.stop_distance=std::max(stop_distance_,grid_->cfg.inflation_xy-sonar_offset_.x+
                             declare_parameter("ultrasonic_distance_margin",.05));
    sg.resume_distance=std::max(resume_distance_,sg.stop_distance+.12);
    sg.brake_accel=declare_parameter("ultrasonic_brake_accel",1.2);
    sg.reaction_time=declare_parameter("ultrasonic_reaction_time",.30)+c.max_accel_xy/c.max_jerk_xy;
    sg.timeout=ultrasonic_timeout_;sg.half_angle=sonar_half_angle_;
    sg.max_tilt=declare_parameter("ultrasonic_max_tilt_deg",20.0)*pi/180;
    if(sg.brake_accel<=0 || sg.reaction_time<0 || sonar_half_angle_<=0 || sonar_half_angle_>pi/2 || ultrasonic_timeout_<=0 ||
       !std::isfinite(sonar_pose_tolerance_) || sonar_pose_tolerance_<=0 || sonar_pose_tolerance_>.25 ||
       !std::isfinite(sonar_pose_wait_timeout_) || sonar_pose_wait_timeout_<=0 || sonar_pose_wait_timeout_>3.0 ||
       sonar_pending_queue_size_==0 || sonar_pending_queue_size_>200)
      throw std::runtime_error("Invalid ultrasonic braking parameters");
    sonar_guard_=std::make_unique<SonarGuard>(sg);
    // Radar-certified motion may use a wider forward sector than the narrow
    // ultrasonic cone. Sonar still caps only the measured closing component.
    direction_config_.forward_half_angle=declare_parameter(
      "forward_motion_half_angle_deg",35.0)*pi/180;
    c.strict_forward_half_angle=direction_config_.forward_half_angle;
    direction_config_.sonar_half_angle=sonar_half_angle_;
    direction_config_.alignment_forward_speed_cap=
      declare_parameter("alignment_forward_speed_cap",.45);
    nonforward_budget_.cfg.alignment_half_angle=direction_config_.forward_half_angle;
    if(!direction_config_.valid() || c.relaxed_lateral_speed<=0 || c.relaxed_reverse_speed<=0 ||
       c.min_forward_speed>direction_config_.alignment_forward_speed_cap ||
       direction_config_.alignment_forward_speed_cap>c.max_speed_xy)
      throw std::runtime_error("Invalid side/reverse velocity limits");
    sonar_collision_=std::make_unique<Grid>(grid_->cfg);
    sonar_layer_.resolution=grid_->cfg.resolution;
    sonar_layer_.half_height=declare_parameter("follower_sonar_half_height",.20);
    sonar_layer_.ttl=declare_parameter("follower_sonar_memory_ttl",4.0);
    sonar_layer_.confirmation_window=declare_parameter(
      "follower_sonar_confirmation_window",.80);
    sonar_layer_.confirm_frames=static_cast<int>(declare_parameter<int64_t>(
      "follower_sonar_confirm_frames",2));
    sonar_layer_.clear_confirm_frames=static_cast<int>(declare_parameter<int64_t>(
      "follower_sonar_clear_confirm_frames",2));
    sonar_layer_.clear_endpoint_guard=declare_parameter(
      "follower_sonar_clear_endpoint_guard",.12);
    unlocalized_sonar_ttl_=declare_parameter("unlocalized_sonar_ttl",2.0);
    sonar_stamp_reset_threshold_=declare_parameter("ultrasonic_stamp_reset_threshold",.50);
    if(!std::isfinite(sonar_layer_.half_height)||sonar_layer_.half_height<0||
       !std::isfinite(sonar_layer_.ttl)||sonar_layer_.ttl<=0||
       !std::isfinite(sonar_layer_.confirmation_window)||sonar_layer_.confirmation_window<=0||
       sonar_layer_.confirm_frames<1||sonar_layer_.confirm_frames>5||
       sonar_layer_.clear_confirm_frames<1||sonar_layer_.clear_confirm_frames>5||
       !std::isfinite(sonar_layer_.clear_endpoint_guard)||sonar_layer_.clear_endpoint_guard<.05||
       !std::isfinite(unlocalized_sonar_ttl_)||unlocalized_sonar_ttl_<=0||
       !std::isfinite(sonar_stamp_reset_threshold_)||sonar_stamp_reset_threshold_<.10)
      throw std::runtime_error("Invalid follower sonar memory parameters");
    radar_match_distance_=declare_parameter("glass_radar_match_distance",.25);
    radar_risk_timeout_=declare_parameter("glass_radar_timeout",1.50);
    if(!std::isfinite(radar_match_distance_) || radar_match_distance_<=0 ||
       !std::isfinite(radar_risk_timeout_) || radar_risk_timeout_<=0)
      throw std::runtime_error("Invalid radar/sonar risk matching parameters");
    auto match_cfg=grid_->cfg;match_cfg.inflation_xy=radar_match_distance_;
    match_cfg.inflation_z=.30;match_cfg.recovery_xy=0;
    // This grid answers sensor correspondence, not flight feasibility. A sonar
    // return above cruise bounds must not masquerade as a radar obstacle.
    match_cfg.z_min=-1e6;match_cfg.z_max=1e6;
    radar_match_=std::make_unique<Grid>(match_cfg);
    px4_namespace_ = declare_parameter<std::string>("px4_namespace", "/scout1/fmu");
    while (px4_namespace_.size() > 1 && px4_namespace_.back() == '/') px4_namespace_.pop_back();
    if (px4_namespace_.empty() || px4_namespace_.front() != '/')
      throw std::runtime_error("px4_namespace must be an absolute ROS namespace such as /scout1/fmu");
    target_system_ = declare_parameter("target_system", 1);
    if (c.lookahead <= 0 || c.max_speed_xy <= 0 || c.max_speed_z <= 0 || c.max_accel_xy <= 0 ||
        c.max_accel_z <= 0 || c.kp_xy <= 0 || c.kp_z <= 0 || c.kd_xy < 0 || c.kd_z < 0 ||
        c.cross_track_gain <= 0 || c.cross_track_damping < 0 || c.cross_track_speed <= 0 ||
        c.yaw_rate <= 0 || c.yaw_accel <= 0 || rate < 5 || rate > 200 || path_timeout_ <= 0 ||
        odom_timeout_ <= 0 || resume_distance_ <= stop_distance_ ||
        takeoff_altitude_ < grid_->cfg.z_min || takeoff_altitude_ > grid_->cfg.z_max)
      throw std::runtime_error("Invalid follower parameters");
    for(double value:{c.lookahead,c.max_speed_xy,c.max_speed_z,c.max_accel_xy,c.max_accel_z,
                       c.max_jerk_xy,c.max_jerk_z,c.lateral_accel,c.kp_xy,c.kp_z,c.kd_xy,c.kd_z,
                       c.cross_track_gain,c.cross_track_damping,c.cross_track_speed,
                       c.goal_tolerance,c.yaw_rate,c.yaw_accel,c.yaw_preview_time,
                       c.heading_full_speed,c.heading_slow,c.heading_min_speed,c.heading_stop,
                       c.heading_slow_speed_scale,c.heading_min_speed_scale,c.min_forward_speed,
                       c.narrow_speed,c.clearance_slow_distance,c.sharp_turn,sg.stop_distance,
                       sg.resume_distance,sg.brake_accel,sg.reaction_time,sg.max_tilt,sonar_yaw_,
                       direction_config_.alignment_forward_speed_cap,rate,
                       startup_spread_offset_x_,startup_spread_offset_y_,
                       startup_spread_spacing_,startup_spread_speed_,
                       startup_spread_tolerance_,startup_spread_obstacle_lookahead_})
      if(!std::isfinite(value))throw std::runtime_error("Non-finite follower parameter");
    if(target_system_<1 || target_system_>255 || c.goal_tolerance<=0 || c.yaw_deadband<0 ||
       c.yaw_preview_time<0 || c.yaw_preview_time>1 ||
       map_timeout_<=0 || status_timeout_<=0 || prestream_seconds_<1.0 || !finite(sonar_offset_) ||
       std::hypot(startup_spread_offset_x_,startup_spread_offset_y_)>100.0 ||
       startup_spread_spacing_<.10 || startup_spread_spacing_>30.0 ||
       startup_formation_count_<1 || startup_formation_rank_<0 ||
       startup_formation_rank_>=startup_formation_count_ || startup_spread_speed_<=0 ||
       startup_spread_speed_>c.max_speed_xy || startup_spread_tolerance_<.10 ||
       startup_spread_tolerance_>.60 || startup_spread_obstacle_lookahead_<.50 ||
       startup_spread_obstacle_lookahead_>3.0)
      throw std::runtime_error("Invalid identity/timing/mount parameter");
    recovery_speed_=declare_parameter("recovery_speed",.25);
    if(!std::isfinite(recovery_speed_)||recovery_speed_<=0||recovery_speed_>c.narrow_speed)
      throw std::runtime_error("recovery_speed must be >0 and <= narrow_passage_speed");
    progress_watchdog_.timeout=declare_parameter("progress_timeout",2.0);
    progress_watchdog_.repeat=declare_parameter("progress_repeat",1.0);
    const double alignment=declare_parameter("max_alignment_time",8.0);
    for(double value:{progress_watchdog_.timeout,progress_watchdog_.repeat,alignment})
      if(!std::isfinite(value)||value<=0)throw std::runtime_error("Invalid progress watchdog timing");
    progress_watchdog_.max_alignment_time=std::max(alignment,pi/c.yaw_rate+2.0);
    tracker_ = std::make_unique<Tracker>(c);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        declare_parameter<std::string>("odom_topic", "/scout1/odom"), rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::SharedPtr m) {
          Vec3 p{m->pose.pose.position.x, m->pose.pose.position.y, m->pose.pose.position.z};
          auto q = m->pose.pose.orientation;
          if (m->header.frame_id != frame_ || !finite(p) || !std::isfinite(q.w) ||
              !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z))
            return;
          double t = stampNs(m->header.stamp) * 1e-9;
          if(t<=0)return;
          if(have_odom_&&t<odom_source_stamp_-sonar_stamp_reset_threshold_)
            handleOdomTimeRewind(t);
          if(have_odom_ && t<=odom_source_stamp_)return;
          PoseSample sample{t,p,q.w,q.x,q.y,q.z};
          if(!sample.normalize())return;
          poses_.add(sample);current_pose_=sample;
          // Position derivative is explicitly in the path frame; avoids ambiguous body/world twist
          // conventions.
          if (have_odom_) {
            double dt = t - odom_source_stamp_;
            if (dt > .001 && dt < .3) {
              Vec3 raw = limitNorm((p - current_) * (1 / dt), 5.0);
              double a = dt / (.06 + dt);
              velocity_ = velocity_ * (1 - a) + raw * a;
            }
          }
          current_ = p;
          yaw_current_ = sample.yaw();
          odom_source_stamp_ = t;
          odom_stamp_ = rclcpp::Time(m->header.stamp,get_clock()->get_clock_type());
          have_odom_ = true;
          trail_.add(p,t);
          if(state_==State::NAV && !recovery_.active())retreat_history_.add(p,t);
          drainSonar();
        });
    peer_observations_.resize(peer_topics.size());
    for(size_t i=0;i<peer_topics.size();++i){
      if(peer_topics[i].empty() || peer_priority_values[i]<0)
        throw std::runtime_error("Peer odometry topics must be non-empty and priorities nonnegative");
      peer_observations_[i].motion.priority=static_cast<int>(peer_priority_values[i]);
      peer_observations_[i].motion.index=i;
      peer_odom_subs_.push_back(create_subscription<nav_msgs::msg::Odometry>(
        peer_topics[i],rclcpp::SensorDataQoS(),
        [this,i](nav_msgs::msg::Odometry::SharedPtr message){
          receivePeerOdometry(i,*message);
        }));
    }
    execution_state_sub_=create_subscription<std_msgs::msg::String>(
      declare_parameter<std::string>("execution_state_topic","/scout1/planning/execution_state"),
      rclcpp::QoS(1).reliable().transient_local(),[this](std_msgs::msg::String::SharedPtr m){
        if(!route_channel_.permit(m->data,now().seconds(),steadySeconds()))return;
        if(!have_mission_goal_){mission_goal_=route_channel_.gate.permit.goal;have_mission_goal_=true;}
        synchronizeRoute();
      });
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      declare_parameter<std::string>("path_topic","/scout1/planning/global_path"),
      rclcpp::QoS(1).reliable(),[this](nav_msgs::msg::Path::SharedPtr m){
        if(m->header.frame_id!=frame_)return;
        std::vector<Vec3> p;
        for(const auto &x:m->poses){Vec3 v{x.pose.position.x,x.pose.position.y,x.pose.position.z};
          if(!finite(v))return;
          p.push_back(v);}
        if(route_channel_.path(stampNs(m->header.stamp),std::move(p)))synchronizeRoute();
      });
    auto mission=[this](geometry_msgs::msg::PoseStamped::SharedPtr m){
      Vec3 goal{m->pose.position.x,m->pose.position.y,m->pose.position.z};
      if(m->header.frame_id!=frame_||!finite(goal))return;
      if(!have_mission_goal_||distance(goal,mission_goal_)>.05){
        // A goal changes task intent, not the autonomous recovery allowance.
        // Preserve the campaign across messages; only real normal progress rearms it.
        if(recovery_.active())campaign_.finish(now().seconds());
        mission_goal_=goal;have_mission_goal_=true;recovery_.cancel();separating_active_=false;recovery_requested_=false;
        peer_yield_watchdog_.reset();peer_yield_active_=peer_yield_stalled_=false;
        peer_deadlock_recovery_=false;have_peer_blocker_position_=false;
        have_approach_=false;goal_geometry_blocked_=false;observed_escape_active_=false;observed_escape_.clear();
        retreat_route_=RetreatRoute{};history_retreat_active_=false;retreat_waypoint_=1;
        route_channel_.reset();tracker_->setPath({},current_);have_path_=false;pending_path_.clear();
        nonforward_budget_.clear();nonforward_session_=false;forward_reset_progress_=0;
        strict_after_unobserved_=false;observation_fallback_.clear();handoff_.clear();handoff_ready_=false;
      }
    };
    mission_sub_=create_subscription<geometry_msgs::msg::PoseStamped>(
      declare_parameter<std::string>("goal_topic","/scout1/mission/fire_goal"),10,mission);
    const auto rviz_goal=declare_parameter<std::string>("rviz_goal_topic","/goal_pose");
    if(!rviz_goal.empty())rviz_mission_sub_=create_subscription<geometry_msgs::msg::PoseStamped>(rviz_goal,10,mission);
    approach_sub_=create_subscription<nav_msgs::msg::Path>(
      declare_parameter<std::string>("goal_approach_topic","/scout1/planning/goal_approach"),
      rclcpp::QoS(1).reliable().transient_local(),[this](nav_msgs::msg::Path::SharedPtr m){
        if(m->header.frame_id!=frame_||m->poses.empty()||m->poses.size()>2)return;
        const auto &r=m->poses.front().pose.position;Vec3 requested{r.x,r.y,r.z};
        if(!finite(requested)||(have_mission_goal_&&distance(requested,mission_goal_)>.05))return;
        if(!have_mission_goal_){mission_goal_=requested;have_mission_goal_=true;}
        if(m->poses.size()==1){have_approach_=false;goal_geometry_blocked_=true;return;}
        const auto &e=m->poses.back().pose.position;Vec3 effective{e.x,e.y,e.z};
        if(!finite(effective)||distance(effective,requested)>approach_limit_+1e-6)return;
        approach_goal_=effective;have_approach_=true;goal_geometry_blocked_=false;
      });
    compact_maps_=compactMapTransport(*this);
    const auto occupied_topic=declare_parameter<std::string>("occupied_topic","/scout1/map/radar_occupied");
    const auto free_topic=declare_parameter<std::string>("free_topic","/scout1/map/radar_free");
    const auto snapshot_topic=declare_parameter<std::string>("map_snapshot_topic","/scout1/map/radar_snapshot");
    if(compact_maps_){
      if(snapshot_topic.empty()||snapshot_topic==occupied_topic||snapshot_topic==free_topic)
        throw std::runtime_error("Invalid follower snapshot topic");
      snapshot_sub_=create_subscription<sensor_msgs::msg::PointCloud2>(snapshot_topic,
        rclcpp::QoS(1).reliable().transient_local(),
        [this](sensor_msgs::msg::PointCloud2::SharedPtr m){
          if(m->header.frame_id!=frame_||!have_odom_)return;
          const auto stamp=stampNs(m->header.stamp);if(stamp<=0)return;
          observeMapInputStamp(stamp,last_occupied_input_stamp_ns_,"LOCAL_SNAPSHOT_STAMP_REWIND");
          if(stamp<=occupied_stamp_)return;
          try{
            const auto anchor=current_;const double pad=grid_->cfg.inflation_z+grid_->cfg.resolution;
            map_snapshot::Region region{
              key({anchor.x-local_map_extent_,anchor.y-local_map_extent_,grid_->cfg.z_min-pad-.50},grid_->cfg.resolution),
              key({anchor.x+local_map_extent_,anchor.y+local_map_extent_,grid_->cfg.z_max+pad+.50},grid_->cfg.resolution),true};
            auto decoded=parseMapSnapshot(*m,grid_->cfg.resolution,map_snapshot::default_max_cells,region);
            auto observed=std::make_unique<Grid>(grid_->cfg);
            observed->update(std::move(decoded.occupied),std::move(decoded.free));
            // Both safety and observed-free layers come from this same packet.
            // No intermediate callback can see a new occupied map paired with
            // an older free map. Only the bounded local region is expanded.
            grid_->update(observed->occupied,{});
            observed_grid_=std::move(observed);local_map_anchor_=observed_map_anchor_=anchor;
            occupied_stamp_=free_stamp_=stamp;last_free_input_stamp_ns_=stamp;
            have_map_=true;map_stamp_=rclcpp::Time(m->header.stamp,get_clock()->get_clock_type());
            ++snapshot_commits_;snapshot_bytes_=m->data.size();
          }catch(const std::runtime_error &e){
            ++snapshot_rejected_;RCLCPP_ERROR(get_logger(),"Local snapshot: %s",e.what());
          }
        });
    }else{
    map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        occupied_topic,
        rclcpp::QoS(1).reliable().transient_local(),
        [this](sensor_msgs::msg::PointCloud2::SharedPtr m) {
          if (m->header.frame_id != frame_)
            return;
          try {
            const auto stamp=stampNs(m->header.stamp);
            observeMapInputStamp(stamp,last_occupied_input_stamp_ns_,"OCCUPIED_MAP_STAMP_REWIND");
            observation_pairs_.putOccupied(stamp,m);
            if(stamp<occupied_stamp_ || (have_map_ && stamp==occupied_stamp_)){
              commitObservation();return;
            }
            if(!have_odom_)return;
            const Vec3 anchor=current_;
            auto occupied=localCloud(*m,anchor);occupied_stamp_=stamp;
            // Apply new obstacles immediately; free evidence is trusted only
            // when its source stamp matches this exact occupied snapshot.
            grid_->update(std::move(occupied),{});
            local_map_anchor_=anchor;
            have_map_ = true;
            map_stamp_=rclcpp::Time(m->header.stamp,get_clock()->get_clock_type());
            commitObservation();
          } catch (const std::runtime_error &e) {
            RCLCPP_ERROR(get_logger(), "Map: %s", e.what());
          }
        });
    free_sub_=create_subscription<sensor_msgs::msg::PointCloud2>(
        free_topic,
        rclcpp::QoS(1).reliable().transient_local(),
        [this](sensor_msgs::msg::PointCloud2::SharedPtr m){
          const auto stamp=stampNs(m->header.stamp);
          if(m->header.frame_id!=frame_)return;
          try{
            if(!have_odom_)return;
            observeMapInputStamp(stamp,last_free_input_stamp_ns_,"FREE_MAP_STAMP_REWIND");
            // Updating matched free evidence does not rebuild/copy occupied
            // buckets or duplicate the large free-space set in the executor.
            observation_pairs_.putFree(stamp,m);commitObservation();
          }catch(const std::runtime_error&e){RCLCPP_ERROR(get_logger(),"Free map: %s",e.what());}
        });
    }
    radar_only_sub_=create_subscription<sensor_msgs::msg::PointCloud2>(
        declare_parameter<std::string>("radar_only_topic","/scout1/map/radar_only_occupied"),
        rclcpp::QoS(1).reliable().transient_local(),
        [this](sensor_msgs::msg::PointCloud2::SharedPtr m){
          if(m->header.frame_id!=frame_)return;
          try{
            radar_match_->update(cloudSet(*m,grid_->cfg.resolution),{});
            radar_only_stamp_=rclcpp::Time(m->header.stamp,get_clock()->get_clock_type());
            have_radar_only_=true;
          }catch(const std::runtime_error&e){RCLCPP_ERROR(get_logger(),"Radar-only map: %s",e.what());}
        });
    if(!glass_topic_.empty())glass_sub_=create_subscription<std_msgs::msg::Bool>(glass_topic_,10,
        [this](std_msgs::msg::Bool::SharedPtr m){
          external_glass_risk_=m->data;have_glass_signal_=true;glass_signal_stamp_=now();
        });
    ultrasonic_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        declare_parameter<std::string>("ultrasonic_topic", "/scout1/ultrasonic/front/scan"),
        rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::LaserScan::SharedPtr m) {
          double source=stampNs(m->header.stamp)*1e-9;
          if(source<=0)return;
          double newest=last_sonar_source_;
          if(!sonar_pending_.empty())newest=std::max(newest,sonar_pending_.back().source);
          if(newest>0&&source<newest-sonar_stamp_reset_threshold_)
            resetSonarState("ULTRASONIC_STAMP_REWIND",true);
          if(source<=last_sonar_source_)return;
          for(const auto &pending:sonar_pending_)
            if(std::abs(pending.source-source)<1e-9)return;
          if(sonar_pending_.size()>=sonar_pending_queue_size_){
            sonar_pending_.pop_front();++sonar_pose_drops_;
          }
          sonar_pending_.push_back({std::move(m),source,std::chrono::steady_clock::now()});
          drainSonar();
        });
    status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
        topicParameter("status_topic",px4Topic<px4_msgs::msg::VehicleStatus>(px4_namespace_ + "/out/vehicle_status")),
        rclcpp::QoS(1).best_effort().transient_local(), [this](px4_msgs::msg::VehicleStatus::SharedPtr m) {
          px4_clock_.observe(m->timestamp,rosMicros(),systemMicros(),steadySeconds());
          px4_nav_state_=m->nav_state;
          px4_status_system_id_=m->system_id;
          armed_ = m->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
          offboard_ = m->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
          have_status_ = true;
          status_stamp_ = now();
        });
    land_sub_=create_subscription<px4_msgs::msg::VehicleLandDetected>(
      topicParameter("land_detected_topic",px4Topic<px4_msgs::msg::VehicleLandDetected>(px4_namespace_ + "/out/vehicle_land_detected")),
      rclcpp::QoS(1).best_effort().transient_local(),[this](px4_msgs::msg::VehicleLandDetected::SharedPtr m){
        px4_landed_=m->landed;have_landed_=true;
      });
    sonar_debug_pub_=create_publisher<sensor_msgs::msg::PointCloud2>(
      declare_parameter<std::string>("follower_sonar_debug_topic","/scout1/planning/follower_sonar_occupied"),1);
    hb_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
        topicParameter("offboard_topic",px4Topic<px4_msgs::msg::OffboardControlMode>(px4_namespace_ + "/in/offboard_control_mode")),
        10);
    sp_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        topicParameter("setpoint_topic",px4Topic<px4_msgs::msg::TrajectorySetpoint>(px4_namespace_ + "/in/trajectory_setpoint")), 10);
    command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
        topicParameter("command_topic",px4Topic<px4_msgs::msg::VehicleCommand>(px4_namespace_ + "/in/vehicle_command")), 10);
    target_pub_ =
        create_publisher<geometry_msgs::msg::PoseStamped>(declare_parameter<std::string>("tracking_target_topic","/scout1/planning/tracking_target"), 10);
    state_pub_ = create_publisher<std_msgs::msg::String>(declare_parameter<std::string>("follower_state_topic","/scout1/planning/follower_state"), 10);
    diagnostic_pub_=create_publisher<std_msgs::msg::String>(declare_parameter<std::string>("diagnostics_topic","/scout1/planning/control_diagnostics"),10);
    replan_pub_=create_publisher<std_msgs::msg::String>(declare_parameter<std::string>("replan_request_topic","/scout1/planning/replan_request"),10);
    sonar_reset_service_=create_service<std_srvs::srv::Trigger>(
      "~/reset_sonar_memory",[this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response>response){
          resetSonarState("MANUAL_SERVICE",true);response->success=true;
          response->message="Follower ultrasonic memory cleared.";
        });
    passage_request_pub_=create_publisher<nav_msgs::msg::Path>(
      declare_parameter<std::string>("passage_request_topic","/scout1/planning/passage_request"),rclcpp::QoS(1).reliable());
    passage_grant_sub_=create_subscription<nav_msgs::msg::Path>(
      declare_parameter<std::string>("passage_grant_topic","/scout1/planning/passage_grant"),
      rclcpp::QoS(1).reliable(),[this](nav_msgs::msg::Path::SharedPtr m){receivePassageGrant(*m);});
    timer_ = create_wall_timer(std::chrono::milliseconds(int(1000 / rate)), [this] { control(); });
    config_lock_=lockParameters(*this);
    RCLCPP_INFO(get_logger(),
                "V2.1.2 geometric follower: centered_formation=%d rank=%d/%d spacing=%.2fm local_offset=(%.2f,%.2f)m speed=%.2fm/s obstacle_lookahead=%.2fm; peer safety peers=%zu hard=%.3fm vertical=%.3fm passage=%.3fm rear_guard=%.2fm deadlock=%.2fs retreat=%.2fm/s; sonar ttl=%.2fs clear_frames=%d; staged heading speed=%d forward_during_alignment=%d angles=%.1f/%.1f/%.1f/%.1f deg min_speed=%.2f cap=%.2f m/s",
                startup_spread_enabled_,startup_formation_rank_,
                startup_formation_count_,startup_spread_spacing_,
                startup_spread_offset_x_,startup_spread_offset_y_,
                startup_spread_speed_,startup_spread_obstacle_lookahead_,
                peer_topics.size(),peer_safety_config_.hard_separation,
                peer_safety_config_.vertical_separation,
                peer_safety_config_.passage_width,
                peer_safety_config_.rear_ignore_distance,
                peer_yield_watchdog_.cfg.no_progress_timeout,
                peer_safety_config_.yield_retreat_speed,
                sonar_layer_.ttl,sonar_layer_.clear_confirm_frames,
                c.heading_speed_scaling_enabled,c.allow_forward_during_alignment,
                c.heading_full_speed*180/pi,c.heading_slow*180/pi,
                c.heading_min_speed*180/pi,c.heading_stop*180/pi,
                c.min_forward_speed,direction_config_.alignment_forward_speed_cap);
  }

private:
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr config_lock_;
  struct PendingSonar {
    sensor_msgs::msg::LaserScan::SharedPtr scan;
    double source{0};
    std::chrono::steady_clock::time_point received{};
  };
  enum class State { WAIT, PRESTREAM, REQUEST, TAKEOFF, STARTUP_SPREAD, NAV };
  struct PeerObservation {
    PeerKinematicState motion;
    double source_stamp{0};
    rclcpp::Time received{0,0,RCL_ROS_TIME};
    bool have{false};
  };
  static tf2::Transform toTransform(
      const geometry_msgs::msg::TransformStamped &message){
    const auto &q=message.transform.rotation;
    tf2::Quaternion rotation(q.x,q.y,q.z,q.w);
    if(rotation.length2()<1e-12)
      throw std::runtime_error("Invalid peer odometry TF quaternion");
    rotation.normalize();
    const auto &p=message.transform.translation;
    return tf2::Transform(rotation,tf2::Vector3(p.x,p.y,p.z));
  }
  void receivePeerOdometry(size_t index,const nav_msgs::msg::Odometry &message){
    if(index>=peer_observations_.size() || message.header.frame_id.empty())return;
    const Vec3 local{message.pose.pose.position.x,message.pose.pose.position.y,
                     message.pose.pose.position.z};
    const double source=stampNs(message.header.stamp)*1e-9;
    if(!finite(local)||source<=0)return;
    try{
      tf2::Transform transform;
      if(message.header.frame_id==frame_)transform.setIdentity();
      else transform=toTransform(tf_.lookupTransform(
        frame_,message.header.frame_id,
        rclcpp::Time(message.header.stamp,get_clock()->get_clock_type()),
        rclcpp::Duration::from_seconds(peer_tf_timeout_)));
      const auto converted=transform*tf2::Vector3(local.x,local.y,local.z);
      const Vec3 position{converted.x(),converted.y(),converted.z()};
      if(!finite(position))return;
      auto &peer=peer_observations_[index];
      if(peer.have && source<peer.source_stamp-.50){
        peer.motion.velocity={};peer.have=false;
      }
      if(peer.have && source<=peer.source_stamp)return;
      if(peer.have){
        const double dt=source-peer.source_stamp;
        if(dt>.001 && dt<.50){
          const Vec3 raw=limitNorm((position-peer.motion.position)*(1./dt),5.0);
          const double alpha=dt/(.08+dt);
          peer.motion.velocity=peer.motion.velocity*(1-alpha)+raw*alpha;
        }else peer.motion.velocity={};
      }
      peer.motion.position=position;
      peer.source_stamp=source;
      peer.received=now();peer.have=true;
    }catch(const tf2::TransformException &error){
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),3000,
        "Waiting for peer odometry TF %s <- %s: %s",frame_.c_str(),
        message.header.frame_id.c_str(),error.what());
    }catch(const std::runtime_error &error){
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),3000,
        "Rejected peer odometry: %s",error.what());
    }
  }
  std::vector<PeerKinematicState> freshPeerStates()const{
    std::vector<PeerKinematicState> result;
    result.reserve(peer_observations_.size());
    const auto time=now();
    for(const auto &observation:peer_observations_){
      if(!observation.have)continue;
      auto state=observation.motion;
      const double age=(time-observation.received).seconds();
      state.fresh=age>=-.05 && age<=peer_odom_timeout_;
      if(state.fresh)result.push_back(state);
    }
    return result;
  }
  PassageReservation passageReservation(){
    PassageReservation reservation;
    if(!peer_safety_config_.enabled || !have_map_ || !have_path_ ||
       (now()-path_stamp_).seconds()<-.05 ||
       (now()-path_stamp_).seconds()>path_timeout_)return reservation;
    const Vec3 immediate=tracker_->previewTarget(current_,velocity_)-current_;
    if(std::hypot(immediate.x,immediate.y)<.03)return reservation;
    const auto current_gap=localGap(*grid_,current_,immediate,
      tracker_->cfg.max_speed_xy,gap_config_);
    for(double ahead=.10;ahead<=peer_safety_config_.passage_lookahead+1e-9;
        ahead+=.10){
      const Vec3 point=tracker_->futurePoint(current_,ahead);
      const Vec3 before=tracker_->futurePoint(current_,std::max(0.,ahead-.10));
      const Vec3 after=tracker_->futurePoint(current_,ahead+.10);
      const Vec3 tangent=after-before;
      if(std::hypot(tangent.x,tangent.y)<.02)continue;
      const auto gap=localGap(*grid_,point,tangent,
        tracker_->cfg.max_speed_xy,gap_config_);
      if(gap.width<=0 || gap.width>peer_safety_config_.passage_width)continue;
      reservation.valid=true;reservation.anchor=point;
      const double anchor_distance=horizontalNorm(current_-point);
      reservation.own_inside=
        (current_gap.width>0 &&
         current_gap.width<=peer_safety_config_.passage_width &&
         anchor_distance<.70) || anchor_distance<.35 ||
        separating_active_ || recovery_.active();
      return reservation;
    }
    return reservation;
  }
  void applyPeerSafety(Control &control,std::string &effective){
    peer_yield_active_=false;
    peer_yield_stalled_=false;
    last_passage_reservation_=state_==State::NAV&&!passage_scheduler_enabled_?
      passageReservation():PassageReservation{};
    if((state_!=State::NAV && state_!=State::STARTUP_SPREAD) ||
       !peer_safety_config_.enabled){
      last_peer_decision_=PeerSafetyDecision{};
      last_peer_decision_.velocity=control.velocity;
      peer_yield_watchdog_.reset();
      return;
    }
    const auto peers=freshPeerStates();
    last_peer_decision_=constrainPeerMotion(
      current_,control.velocity,own_priority_,peers,
      peer_safety_config_,last_passage_reservation_);
    if(last_peer_decision_.limited){
      control.velocity=last_peer_decision_.velocity;
      switch(last_peer_decision_.reason){
        case PeerSafetyReason::HARD_SEPARATION:
          peer_yield_active_=last_peer_decision_.yielding;
          effective=last_peer_decision_.yielding?
            "PEER_HARD_SEPARATION_HOLD":"PEER_SEPARATING_ESCAPE";
          break;
        case PeerSafetyReason::PASSAGE_YIELD:
          // A non-zero command here is a bounded, non-closing retreat; the
          // passage reservation is still active and must keep its progress
          // watchdog running until the blocker clears.
          peer_yield_active_=true;
          effective=last_peer_decision_.yielding?
            "PEER_PASSAGE_YIELD":"PEER_PASSAGE_RETREAT";break;
        case PeerSafetyReason::PREDICTED_CONFLICT_YIELD:
          peer_yield_active_=true;
          effective=last_peer_decision_.yielding?
            "PEER_CONFLICT_YIELD":"PEER_CONFLICT_RETREAT";break;
        case PeerSafetyReason::SPEED_LIMIT:
          effective="PEER_SEPARATION_BRAKE";break;
        case PeerSafetyReason::CLEAR:break;
      }
    }
    Vec3 blocker{};bool have_blocker=false;
    if(last_peer_decision_.blocking_priority>=0){
      for(const auto &peer:peers)
        if(peer.priority==last_peer_decision_.blocking_priority){
          blocker=peer.position;have_blocker=true;break;
        }
    }
    if(have_blocker){peer_blocker_position_=blocker;have_peer_blocker_position_=true;}
    const double time=now().seconds();
    peer_yield_stalled_=peer_yield_watchdog_.update(
      time,peer_yield_active_,last_peer_decision_.blocking_priority,
      blocker,have_blocker);
    if(peer_yield_stalled_ && state_==State::NAV){
      effective=last_peer_decision_.reason==PeerSafetyReason::HARD_SEPARATION?
        "PEER_HARD_DEADLOCK_HOLD":"PEER_YIELD_DEADLOCK_HOLD";
      if(have_mission_goal_){
        peer_deadlock_recovery_=true;
        recovery_requested_=true;
        if(time<last_peer_yield_request_ ||
           time-last_peer_yield_request_>=peer_yield_replan_period_){
          replan_pub_->publish(textMessage("PEER_YIELD_DEADLOCK"));
          last_peer_yield_request_=time;
        }
      }
    }else if(peer_deadlock_recovery_ && !recovery_.active()){
      // The owner resumed useful motion before a recovery route began.  Do
      // not turn a transient deadline crossing into an unnecessary retreat.
      peer_deadlock_recovery_=false;
      recovery_requested_=false;
    }
    if(peer_yield_active_)control.target=current_;
  }
  std::string topicParameter(const std::string&name,const std::string&automatic){
    auto value=declare_parameter<std::string>(name,"");
    auto selected=value.empty()?automatic:value;
    RCLCPP_INFO(get_logger(),"PX4 topic %s=%s",name.c_str(),selected.c_str());
    return selected;
  }
  static double steadySeconds(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
  void resetSonarState(const char *reason,bool clear_queue){
    sonar_layer_.reset();sonar_memory_.clear();sonar_collision_->update(KeySet{},KeySet{});
    unlocalized_sonar_.clear();sonar_guard_->reset();last_sonar_source_=0;
    if(clear_queue)sonar_pending_.clear();
    have_ultrasonic_=false;front_distance_=std::numeric_limits<double>::infinity();
    ultrasonic_stamp_=rclcpp::Time(0,0,get_clock()->get_clock_type());
    ++sonar_epoch_resets_;
    RCLCPP_WARN(get_logger(),"SONAR_MEMORY_RESET reason=%s count=%zu",reason,sonar_epoch_resets_);
  }
  void resetMapEpoch(const char *reason){
    observation_pairs_.clear();observed_grid_.reset();occupied_stamp_=0;free_stamp_=-1;
    last_occupied_input_stamp_ns_=last_free_input_stamp_ns_=0;
    grid_->update(KeySet{},KeySet{});have_map_=false;
    radar_match_->update(KeySet{},KeySet{});have_radar_only_=false;
    map_stamp_=radar_only_stamp_=rclcpp::Time(0,0,get_clock()->get_clock_type());
    RCLCPP_WARN(get_logger(),"FOLLOWER_MAP_TIME_EPOCH_RESET reason=%s",reason);
  }
  void observeMapInputStamp(int64_t stamp,int64_t &latest,const char *reason){
    if(stamp<=0)return;
    const int64_t threshold=int64_t(sonar_stamp_reset_threshold_*1e9);
    if(latest>0&&stamp<latest-threshold)resetMapEpoch(reason);
    latest=std::max(latest,stamp);
  }
  void handleOdomTimeRewind(double source){
    route_channel_.reset();portal_departure_.clear();passage_wait_monitor_.reset();
    // Coordinator ownership survives rewinds; renew the same physical owner
    // with a new traversal id once fresh odometry/clock are available.
    portal_active_=portal_waiting_=passage_granted_=false;passage_reply_source_=-1;
    RCLCPP_WARN(get_logger(),
      "ODOMETRY_STAMP_REWIND old=%.6f new=%.6f: clearing time-dependent navigation state",
      odom_source_stamp_,source);
    resetSonarState("ODOMETRY_STAMP_REWIND",true);
    poses_=PoseHistory{};velocity_={};odom_source_stamp_=0;have_odom_=false;
    trail_.clear();frozen_trail_.clear();retreat_history_.clear();retreat_route_=RetreatRoute{};
    recovery_.reset();campaign_.reset();nonforward_budget_.clear();nonforward_session_=false;
    history_retreat_active_=observed_escape_active_=separating_active_=false;
    for(auto &peer:peer_observations_){
      peer.have=false;peer.source_stamp=0;peer.motion.velocity={};
      peer.received=rclcpp::Time(0,0,get_clock()->get_clock_type());
    }
    last_peer_decision_=PeerSafetyDecision{};
    last_passage_reservation_=PassageReservation{};
    peer_yield_watchdog_.reset();peer_yield_active_=peer_yield_stalled_=false;
    peer_deadlock_recovery_=false;have_peer_blocker_position_=false;
    last_peer_yield_request_=-1;
    observed_escape_.clear();observation_fallback_.clear();handoff_.clear();handoff_ready_=false;
    pending_path_.clear();have_path_=false;tracker_->setPath({},current_);tracker_->resetMotion();
    resetMapEpoch("ODOMETRY_STAMP_REWIND");
    have_status_=false;armed_=offboard_=false;have_landed_=false;px4_landed_=true;
    px4_nav_state_=-1;px4_status_system_id_=-1;state_=State::WAIT;holding_=false;
    startup_spread_complete_=startup_spread_blocked_=false;
    startup_spread_target_={};
    startup_spread_block_reason_="NOT_STARTED";
    px4_clock_.reset();
    path_stamp_=map_stamp_=odom_stamp_=status_stamp_=
      rclcpp::Time(0,0,get_clock()->get_clock_type());
    last_control_=last_diagnostic_=last_sonar_debug_=last_watchdog_=last_direction_=
      path_wait_since_=last_wait_request_=last_request_=state_since_=
      rclcpp::Time(0,0,get_clock()->get_clock_type());
  }
  uint64_t rosMicros(){return uint64_t(std::max<int64_t>(0,now().nanoseconds()/1000));}
  uint64_t systemMicros(){return uint64_t(std::max<int64_t>(0,px4_system_clock_.now().nanoseconds()/1000));}
  uint64_t stamp(){return px4_clock_.stamp(rosMicros(),systemMicros(),steadySeconds());}
  void command(uint32_t id, float p1, float p2 = 0) {
    px4_msgs::msg::VehicleCommand m{};
    m.timestamp = stamp();
    if(m.timestamp==0)return;
    m.command = id;
    m.param1 = p1;
    m.param2 = p2;
    m.target_system = uint8_t(target_system_);
    m.target_component = 1;
    m.source_system = 1;
    m.source_component = 1;
    m.from_external = true;
    command_pub_->publish(m);
  }
  void acceptSonar(const sensor_msgs::msg::LaserScan &m,double source,
                   const PoseSample &pose,bool fallback){
    auto reading=parseSonar(m.ranges,m.angle_min,m.angle_increment,m.range_min,m.range_max,
                            sonar_half_angle_,sonar_inf_clear_,sonar_yaw_);
    last_sonar_source_=source;
    sonar_reading_=reading;sonar_pose_=pose;
    front_distance_=reading.distance;
    ultrasonic_stamp_=rclcpp::Time(m.header.stamp,get_clock()->get_clock_type());
    have_ultrasonic_=reading.valid;
    ++sonar_matched_;
    if(fallback)++sonar_pose_fallbacks_;
    const double evidence_time=steadySeconds();
    const Vec3 origin=pose.position+pose.rotate(sonar_offset_);
    std::vector<Vec3>localized_hits;
    // The live guard brakes on the first hit. World-frame collision memory is
    // confirmed and expires, preventing one dropped/noisy ray from immobilising
    // a vehicle for the rest of a long simulation.
    if(reading.valid && reading.has_hit && pose.tilt()<sonar_guard_->cfg.max_tilt){
      if(reading.too_close)
        unlocalized_sonar_.add(origin,pose.yaw()+sonar_yaw_,sonar_half_angle_,evidence_time);
      const double cy=std::cos(sonar_yaw_),sy=std::sin(sonar_yaw_);
      for(Vec3 e:reading.endpoints){
        const Vec3 sensor{cy*e.x-sy*e.y,sy*e.x+cy*e.y,e.z};
        const Vec3 hit=pose.position+pose.rotate(sonar_offset_+sensor);
        if(finite(hit))localized_hits.push_back(hit);
      }
    }
    if(reading.valid&&pose.tilt()<sonar_guard_->cfg.max_tilt){
      std::vector<Vec3>free_endpoints;free_endpoints.reserve(reading.free_endpoints.size());
      const double cy=std::cos(sonar_yaw_),sy=std::sin(sonar_yaw_);
      for(Vec3 e:reading.free_endpoints){
        const Vec3 sensor{cy*e.x-sy*e.y,sy*e.x+cy*e.y,e.z};
        const Vec3 endpoint=pose.position+pose.rotate(sonar_offset_+sensor);
        if(finite(endpoint))free_endpoints.push_back(endpoint);
      }
      sonar_layer_.clearObserved(origin,free_endpoints,evidence_time);
    }
    sonar_layer_.integrate(localized_hits,evidence_time);
    refreshSonarMemory(evidence_time);
  }
  void refreshSonarMemory(double source_now){
    auto fresh=sonar_layer_.occupied(source_now);
    if(fresh!=sonar_memory_){
      sonar_memory_=std::move(fresh);
      sonar_collision_->update(sonar_memory_,{});
    }
    unlocalized_sonar_.prune(source_now,unlocalized_sonar_ttl_);
  }
  void drainSonar(){
    const auto steady_now=std::chrono::steady_clock::now();
    for(auto it=sonar_pending_.begin();it!=sonar_pending_.end();){
      if(it->source<=last_sonar_source_){it=sonar_pending_.erase(it);continue;}
      PoseSample pose;bool fallback=false;
      bool matched=poses_.at(it->source,pose,sonar_pose_tolerance_);
      if(!matched && have_odom_ &&
         std::abs(odom_source_stamp_-it->source)<=sonar_pose_tolerance_){
        pose=current_pose_;matched=true;fallback=true;
      }
      if(matched){
        acceptSonar(*it->scan,it->source,pose,fallback);
        it=sonar_pending_.erase(it);continue;
      }
      const double wait=std::chrono::duration<double>(steady_now-it->received).count();
      const bool pose_passed=have_odom_ &&
        odom_source_stamp_>it->source+sonar_pose_tolerance_;
      if(wait>sonar_pose_wait_timeout_ || pose_passed){
        ++sonar_pose_drops_;it=sonar_pending_.erase(it);
        RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),3000,
          "Dropping ultrasonic scan without matching odometry pose: pending=%zu matched=%zu dropped=%zu",
          sonar_pending_.size(),sonar_matched_,sonar_pose_drops_);
        continue;
      }
      ++it;
    }
  }
  SonarDecision frontLimitFor(Vec3 evaluated_velocity){
    if(!use_ultrasonic_)return {1e9,0,"ULTRASONIC_DISABLED"};
    double age=(now()-ultrasonic_stamp_).seconds();
    bool fresh=have_ultrasonic_ && age>=-.03 && age<ultrasonic_timeout_ &&
      std::abs(wrap(yaw_current_-sonar_pose_.yaw()))<sonar_half_angle_ &&
      distance(current_,sonar_pose_.position)<.25 && current_pose_.tilt()<sonar_guard_->cfg.max_tilt;
    double closing=maximumConeClosing(evaluated_velocity,
      sonar_pose_.yaw()+sonar_yaw_,sonar_half_angle_);
    return sonar_guard_->evaluate(sonar_reading_,fresh,closing,age);
  }
  SonarDecision frontLimit(){return frontLimitFor(velocity_);}
  KeySet localCloud(const sensor_msgs::msg::PointCloud2&m,Vec3 anchor)const{
    const double pad=grid_->cfg.inflation_z+grid_->cfg.resolution;
    return cloudSetRegion(m,grid_->cfg.resolution,
      {anchor.x-local_map_extent_,anchor.y-local_map_extent_,grid_->cfg.z_min-pad-.50},
      {anchor.x+local_map_extent_,anchor.y+local_map_extent_,grid_->cfg.z_max+pad+.50});
  }
  void commitObservation(){
    auto pair=observation_pairs_.takeNewestPair();if(!pair||!have_odom_)return;
    auto observed=std::make_unique<Grid>(grid_->cfg);
    const auto anchor=current_;
    observed->update(localCloud(*pair->occupied,anchor),localCloud(*pair->free,anchor));
    observed_grid_=std::move(observed);observed_map_anchor_=anchor;free_stamp_=pair->stamp;
  }
  bool collisionClear(Vec3 a,Vec3 b)const{
    return have_map_ && snapshotContainsSweep(local_map_anchor_,local_map_extent_,
      grid_->cfg.inflation_xy,grid_->cfg.resolution,a,b) &&
      (altitude_return_active_ ? grid_->boundaryReturn(a,b)&&sonar_collision_->boundaryReturn(a,b) :
       separating_active_ ? separatingSweep(*grid_,separation_map_,a,b)&&separatingSweep(*sonar_collision_,separation_sonar_,a,b) :
        grid_->segmentFrom(a,b,true) && sonar_collision_->segmentFrom(a,b,true)) &&
      unlocalized_sonar_.allows(a,b,grid_->cfg.inflation_xy+glass_risk_distance_,
                                steadySeconds(),unlocalized_sonar_ttl_);
  }
  bool awayFromSonar(Vec3 a,Vec3 b)const{
    const double r=grid_->cfg.resolution;
    for(Key k:sonar_memory_){
      Vec3 nearest{std::clamp(a.x,k.x*r,(k.x+1)*r),std::clamp(a.y,k.y*r,(k.y+1)*r),a.z};
      if(std::abs(center(k,r).z-a.z)>grid_->cfg.inflation_z+.25 || distance(a,nearest)>2.0)continue;
      if(dot(a-nearest,b-a)<-1e-9)return false;
    }
    return true;
  }
  bool verifiedRetreat(Vec3 a,Vec3 b){
    if(history_retreat_active_ && retreat_route_.covers(a,b,now().seconds()) &&
       maximumConeClosing(b-a,sonar_pose_.yaw()+sonar_yaw_,sonar_half_angle_)<=1e-8 &&
       awayFromSonar(a,b) && collisionClear(a,b))return true;
    const Vec3 body=velocityInBody(b-a,yaw_current_);
    return nonforward_session_ && body.x< -1e-5 &&
      maximumConeClosing(b-a,sonar_pose_.yaw()+sonar_yaw_,sonar_half_angle_)<=1e-8 &&
      frozen_trail_.retraces(a,b,now().seconds()) && awayFromSonar(a,b) && collisionClear(a,b);
  }
  bool separatingRetreat(Vec3 a,Vec3 b)const{
    return separating_active_&&maximumConeClosing(b-a,sonar_pose_.yaw()+sonar_yaw_,sonar_half_angle_)<=1e-8&&
      awayFromSonar(a,b)&&collisionClear(a,b)&&separationObserved(a,b);
  }
  bool separationObserved(Vec3 a,Vec3 b)const{
    return observedMapMotion(a,b)||(separating_active_&&separation_history_.validAt(now().seconds())&&
      separation_history_.covers(a,b,now().seconds())&&collisionClear(a,b));
  }
  bool observedMotion(Vec3 a,Vec3 b){
    if(separating_active_&&separationObserved(a,b)&&(!glass_risk_active_||separatingRetreat(a,b)))return true;
    if(glass_risk_active_)return verifiedRetreat(a,b)||separatingRetreat(a,b);
    return observedMapMotion(a,b) || verifiedRetreat(a,b) ||
      (history_retreat_active_ && retreat_route_.covers(a,b,now().seconds()) && collisionClear(a,b));
  }
  bool observedMapMotion(Vec3 a,Vec3 b)const{
    const double age=now().seconds()-double(free_stamp_)*1e-9;
    // Retained free evidence is always a real matched pair. Newer occupied
    // evidence remains authoritative and is checked independently below.
    return observed_grid_ && age>=-.05 && age<=map_timeout_ &&
      snapshotContainsSweep(observed_map_anchor_,local_map_extent_,grid_->cfg.inflation_xy,grid_->cfg.resolution,a,b) &&
      (separating_active_?separatingSweep(*observed_grid_,separation_map_,a,b,true):observed_grid_->observedSegment(a,b)) && collisionClear(a,b);
  }
  bool motionBudgetAvailable()const{
    // Escape distance, duration and attempt limits belong to its own episode;
    // exhausting an ordinary sideways tracking allowance must not deadlock it.
    return nonforward_budget_.available() || recovery_.active();
  }
  bool potentialGlassRisk(Vec3 a,Vec3 b)const{
    const double age=(now()-radar_only_stamp_).seconds();
    const bool radar_fresh=have_radar_only_ && age>=-.05 && age<=std::min(radar_risk_timeout_,map_timeout_);
    const Vec3 d=b-a;const double length2=dot(d,d);
    for(Key k:sonar_memory_){
      const Vec3 p=center(k,grid_->cfg.resolution);
      const Vec3 nearest=a+d*(length2>1e-12?std::clamp(dot(p-a,d)/length2,0.,1.):0.);
      if(distance(p,nearest)>grid_->cfg.inflation_xy+glass_risk_distance_)continue;
      if(!radar_fresh || radar_match_->segment(p,p,true))return true;
    }
    return false;
  }
  bool detectedGlassRisk(Vec3 a,Vec3 b)const{
    const bool signal_bad=!glass_topic_.empty() && (!have_glass_signal_ || external_glass_risk_ ||
      (now()-glass_signal_stamp_).seconds()<0 || (now()-glass_signal_stamp_).seconds()>glass_signal_timeout_);
    const double radius=grid_->cfg.inflation_xy+glass_risk_distance_;
    return signal_bad || potentialGlassRisk(a,b) ||
      unlocalized_sonar_.near(a,b,radius,steadySeconds(),unlocalized_sonar_ttl_) ||
      segmentNearAnyRiskBox(a,b,glass_boxes_,radius);
  }
  void updateGlassRisk(Vec3 target){
    const bool risk_now=detectedGlassRisk(current_,target);
    if(risk_now){glass_risk_active_=true;last_glass_risk_=now();}
    else if((now()-last_glass_risk_).seconds()>glass_release_delay_)glass_risk_active_=false;
  }
  void selectHeadingMode(){
    const Vec3 target=tracker_->previewTarget(current_,velocity_);
    const Vec3 course=target-current_;
    const bool outside=outsideForwardCone(course,yaw_current_,direction_config_.forward_half_angle);
    if(outside && !nonforward_session_){
      frozen_trail_=trail_;nonforward_session_=true;forward_reset_progress_=0;
    }
    updateGlassRisk(target);
    const double stopping_horizon=.20+tracker_->cfg.max_accel_xy/tracker_->cfg.max_jerk_xy+
      direction_config_.max_reverse_speed/(2*sonar_guard_->cfg.brake_accel);
    const double preview=std::min(norm(course),std::max(.10,direction_config_.max_reverse_speed*stopping_horizon));
    const Vec3 probe=current_+course*(preview/std::max(1e-9,norm(course)));
    retreat_mode_=verifiedRetreat(current_,probe);
    strict_after_unobserved_=observation_fallback_.blocks(now().seconds(),current_,course);
    if(strict_after_unobserved_ && observedMotion(current_,current_+unobserved_probe_)){
      strict_after_unobserved_=false;observation_fallback_.clear();
    }
    const bool relaxed=adaptive_direction_ && nonforward_budget_.available() &&
                       (!glass_risk_active_ || retreat_mode_) && !strict_after_unobserved_ && observedMotion(current_,probe);
    heading_mode_=relaxed?HeadingMode::Relaxed:HeadingMode::Strict;
  }
  void publish(Control u, const std::string &state) {
    const auto px4_stamp=stamp();
    if(px4_stamp==0){
      const std::string waiting="WAIT_PX4_CLOCK";
      if(last_state_!=waiting){state_pub_->publish(textMessage(waiting));last_state_=waiting;}
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"PX4 preflight clock unverified: policy=%s received=%llu ros=%llu system=%llu; with Gazebo /clock set PX4 UXRCE_DDS_SYNCT=0 before starting; no arm/mode command sent",
        px4_clock_.policy.c_str(),(unsigned long long)px4_clock_.sourceStamp(),(unsigned long long)rosMicros(),(unsigned long long)systemMicros());
      return;
    }
    std::string effective=state;
    if(px4_clock_.flightLocked()&&!px4_clock_.ready(steadySeconds())){
      // Preserve the live stream, but never repeat the previous climb command.
      u.velocity={};u.target=current_;u.yaw=yaw_current_;
      effective="PX4_CLOCK_DEGRADED_HOLD";
    }
    const Vec3 proposed_velocity=u.velocity;
    auto sonar=frontLimit();
    const bool following=state=="FOLLOW_BSPLINE" || state=="ALIGN_AND_ADVANCE" ||
                         state=="ROTATE_TO_PATH" ||
                         state=="LOCAL_PATH_BLOCKED" || state=="CERTIFIED_LOCAL_ESCAPE" ||
                         state=="CERTIFIED_HISTORY_RETREAT" || state=="CERTIFIED_OBSERVED_ESCAPE" || state=="SEPARATING_ENVELOPE_ESCAPE" || state=="PARTIAL_PATH_END";
    const bool navigation_motion=following && !u.blocked && !u.reached;
    bool alignment_forward_applied=false;
    if(state_==State::NAV){
      map_speed_factor_=mapAgeSpeedFactor((now()-map_stamp_).seconds(),map_age_slow_start_,map_timeout_);
      if(navigation_motion && recovery_.active())
        u.velocity=limitNorm(u.velocity,recovery_.cfg.speed*map_speed_factor_);
      const bool recovering=have_map_ && !grid_->segment(current_,current_);
      if(recovering){
        u.velocity=limitNorm(u.velocity,recovery_speed_*map_speed_factor_);
        if(navigation_motion && norm(u.velocity)>1e-6 && !separating_active_)
          effective="RECOVERY_MARGIN_ESCAPE";
      }
      if(navigation_motion && u.heading_wait && std::hypot(velocity_.x,velocity_.y)>.15)
        effective="BRAKE_AND_ALIGN";
      // Protect the complete braking horizon, not merely one command tick.
      const double reaction=.45+std::max(0.,(now()-map_stamp_).seconds())+
        tracker_->cfg.max_accel_xy/tracker_->cfg.max_jerk_xy;
      const auto limited=stoppingLimitedVelocity(current_,u.velocity,reaction,
        std::min(.7*tracker_->cfg.max_accel_xy,sonar_guard_->cfg.brake_accel),
        [this](Vec3 a,Vec3 b){return collisionClear(a,b);});
      if(distance(limited,u.velocity)>1e-5)effective="PREDICTIVE_CLEARANCE_BRAKE";
      u.velocity=limited;
      direction_config_.prediction_horizon=std::max(.60,reaction+norm(u.velocity)/
        (2*std::min(.7*tracker_->cfg.max_accel_xy,sonar_guard_->cfg.brake_accel)));
      // Holds, stale-path corrections and new sensor callbacks must also see
      // the latest risk; selecting a trajectory mode alone is insufficient.
      updateGlassRisk(current_+u.velocity*direction_config_.prediction_horizon);
      DirectionalRequest request;
      request.position=current_;request.velocity=u.velocity;request.body_yaw=yaw_current_;
      request.sonar_yaw=sonar_pose_.yaw()+sonar_yaw_;
      request.front_speed_cap=sonar.speed_limit;
      request.sonar_valid=!use_ultrasonic_ || sonar.reason!="ULTRASONIC_UNAVAILABLE";
      request.sonar_required=use_ultrasonic_&&require_ultrasonic_;
      request.allow_verified_retreat=adaptive_direction_ && motionBudgetAvailable() &&
        verifiedRetreat(current_,current_+u.velocity*direction_config_.prediction_horizon);
      request.allow_separating_retreat=separatingRetreat(current_,current_+u.velocity*direction_config_.prediction_horizon);
      request.allow_alignment_forward=navigation_motion && u.alignment_forward_enabled &&
        !u.heading_wait && tracker_->cfg.allow_forward_during_alignment;
      const bool station_state=state=="WAIT_PATH_HOLD" || state=="RECOVERY_BUDGET_EXHAUSTED" ||
        state=="TARGET_HOLD" || state=="TARGET_APPROACH_HOLD";
      const bool station_relaxed=station_state && !glass_risk_active_ && request.sonar_valid &&
        boundedStationKeeping(current_,u.target,u.velocity);
      request.relaxed=adaptive_direction_ && ((station_relaxed) ||
        ((heading_mode_==HeadingMode::Relaxed || recovery_.active()) && motionBudgetAvailable() &&
         (!glass_risk_active_ || request.allow_verified_retreat || request.allow_separating_retreat)));
      auto directional=constrainDirectional(request,direction_config_,
        [this](Vec3 a,Vec3 b){return observedMotion(a,b);},
        [this](Vec3 a,Vec3 b){return collisionClear(a,b);});
      u.velocity=directional.velocity;
      alignment_forward_applied=directional.alignment_forward;
      direction_reason_=directionalReasonName(directional.reason);
      if(directional.reason==DirectionalReason::UNOBSERVED_CORRIDOR && norm(request.velocity)>1e-7){
        strict_after_unobserved_=true;
        unobserved_probe_=request.velocity*direction_config_.prediction_horizon;
        observation_fallback_.reject(now().seconds(),current_,unobserved_probe_);
      }
      if(norm(request.velocity)>1e-7){
        if(directional.blocked || directional.limited)effective="DIRECTION_"+direction_reason_;
        if(directional.requires_alignment)u.heading_wait=true;
        if(directional.reason==DirectionalReason::COLLISION)u.blocked=true;
      }
      measured_braking_risk_=false;
      const double measured_speed=norm(velocity_);
      if(measured_speed>.05){
        const double travel=measured_speed*reaction+measured_speed*measured_speed/
          (2*std::min(.7*tracker_->cfg.max_accel_xy,sonar_guard_->cfg.brake_accel));
        measured_braking_risk_=!collisionClear(current_,current_+velocity_*(travel/measured_speed));
        // An opposite/away command is already braking. Do not zero it solely
        // because the vehicle has not yet stopped moving toward the obstacle.
        if(measured_braking_risk_ && dot(u.velocity,velocity_)>0){
          u.velocity={};effective="MEASURED_BRAKING_RISK";
        }
      }
      // Test the swept braking corridor of the COMMAND, not only the lookahead chord.
      const double v=norm(u.velocity);
      if(have_map_ && v>1e-6){
        const Vec3 direction=u.velocity*(1./v);
        const double reaction=.20+tracker_->cfg.max_accel_xy/tracker_->cfg.max_jerk_xy;
        const double travel=v*reaction+v*v/(2*sonar_guard_->cfg.brake_accel);
        if(!collisionClear(current_,current_+direction*travel)){
          double lo=0,hi=travel;
          for(int i=0;i<10;++i){double m=(lo+hi)*.5;if(collisionClear(current_,current_+direction*m))lo=m;else hi=m;}
          const double allowed=brakingSpeed(lo,0,sonar_guard_->cfg.brake_accel,reaction);
          u.velocity=limitNorm(u.velocity,allowed);effective="LOCAL_BRAKING_LIMIT";
        }
      }
    }
    // Pairwise separation and bottleneck ownership are evaluated after every
    // map/sonar/directional limit, immediately before the command is committed.
    // They therefore cannot be bypassed by a planner priority or a recovery
    // state, and a hold does not rely on seeing another UAV as a radar voxel.
    // This gate also applies to altitude return and local envelope recovery,
    // which can return before the normal NAV admission update.
    sendPassageRequest();
    if(passage_scheduler_enabled_&&portal_active_){
      const double t=now().seconds();
      portal_waiting_=!(passage_granted_&&t>=passage_grant_time_&&
        t-passage_grant_time_<=passage_lease_timeout_);
    }
    if(state_==State::NAV && portal_waiting_ && dot(u.velocity,portal_direction_)>0){
      const double cap=passageSpeedLimit(norm(u.velocity));
      if(cap<norm(u.velocity)-1e-6){u.velocity=limitNorm(u.velocity,cap);effective="PASSAGE_LEASE_WAIT";}
    }
    applyPeerSafety(u,effective);
    if(state=="PLANNER_INPUT_PAUSED"&&!peer_yield_active_)effective=state;
    sendPassageRequest();
    if(passage_wait_monitor_.update(steadySeconds(),effective=="PASSAGE_LEASE_WAIT",
        passage_wait_timeout_,passage_wait_retry_)){
      last_portal_request_=-10;sendPassageRequest();
      replan_pub_->publish(textMessage("PASSAGE_WAIT_TIMEOUT"));
    }
    // A lost path is timed separately; an empty route cannot supply an endpoint.
    const bool waiting_path=state=="WAIT_PATH_HOLD" ||
      state=="MAP_ENVELOPE_CONFLICT_HOLD" || state=="SONAR_ENVELOPE_CONFLICT_HOLD" ||
      state=="MAP_AND_SONAR_ENVELOPE_CONFLICT_HOLD" ||
      state=="RECOVERY_BUDGET_EXHAUSTED" || state=="GOAL_UNREACHABLE_HOLD";
    if(waiting_path){
      if(path_wait_since_.nanoseconds()==0)path_wait_since_=now();
      if((now()-path_wait_since_).seconds()>=progress_watchdog_.timeout &&
         (now()-last_wait_request_).seconds()>=progress_watchdog_.repeat){
        replan_pub_->publish(textMessage("WAITING_FOR_EXECUTABLE_PATH"));
        last_wait_request_=now();
      }
    }else path_wait_since_=rclcpp::Time(0,0,get_clock()->get_clock_type());
    // A moving passage owner is a legitimate queue, but a stagnant owner is
    // not.  Once the peer watchdog expires, feed the ordinary progress monitor
    // too so both replanning and certified recovery remain observable.
    const bool peer_progress_check=!peer_yield_active_||peer_yield_stalled_;
    const bool can_request=following && peer_progress_check && !portal_waiting_ &&
      !recovery_.active() && have_mission_goal_ &&
      distance(current_,navigationGoal())>std::max(.30,2*tracker_->cfg.goal_tolerance);
    double watch_dt=last_watchdog_.nanoseconds()==0?0:(now()-last_watchdog_).seconds();
    last_watchdog_=now();
    if(progress_watchdog_.update(current_,yaw_current_,watch_dt,can_request)){
      recovery_requested_=true;
      replan_pub_->publish(textMessage("STALLED_PROGRESS"));
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,
        "Progress timeout: translation stagnant %.2f s, overall %.2f s, useful displacement stagnant %.2f s (%s)",
        progress_watchdog_.translationStagnantSeconds(),progress_watchdog_.stagnantSeconds(),
        progress_watchdog_.usefulStagnantSeconds(),effective.c_str());
    }
    const double direction_dt=last_direction_.nanoseconds()==0?0:(now()-last_direction_).seconds();
    last_direction_=now();
    // A fixed-anchor hover servo is bounded in space; millimetre corrections
    // must not spend the complete side/back navigation budget before a goal
    // or an escape even starts. Deliberate navigation still counts tiny commands.
    if(state_==State::NAV && navigation_motion && !recovery_.active() && direction_dt>0 && direction_dt<.3){
      const bool aligned=std::abs(u.course_error)<direction_config_.forward_half_angle;
      nonforward_budget_.update(velocity_,u.velocity,yaw_current_,direction_dt,aligned);
      const Vec3 actual=velocityInBody(velocity_,yaw_current_);
      if(aligned && actual.x>.05 && !outsideForwardCone(velocity_,yaw_current_,direction_config_.forward_half_angle))
        forward_reset_progress_+=actual.x*direction_dt;
      else forward_reset_progress_=0;
      if(forward_reset_progress_>=nonforward_budget_.cfg.reset_forward_distance){
        nonforward_session_=false;forward_reset_progress_=0;
      }
    }
    tracker_->appliedVelocity(u.velocity);
    px4_msgs::msg::OffboardControlMode hb{};
    hb.timestamp = px4_stamp;
    hb.velocity = true;
    hb_pub_->publish(hb);
    px4_msgs::msg::TrajectorySetpoint sp{};
    sp.timestamp = px4_stamp;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    sp.position.fill(nan);
    sp.acceleration.fill(nan);
    sp.jerk.fill(nan);
    sp.velocity[0] = float(u.velocity.y);
    sp.velocity[1] = float(u.velocity.x);
    sp.velocity[2] = float(-u.velocity.z);
    sp.yaw = float(wrap(pi / 2 - u.yaw));
    sp.yawspeed = nan;
    sp_pub_->publish(sp);
    geometry_msgs::msg::PoseStamped target;
    target.header.stamp = now();
    target.header.frame_id = frame_;
    target.pose.position.x = u.target.x;
    target.pose.position.y = u.target.y;
    target.pose.position.z = u.target.z;
    target.pose.orientation.w = 1;
    target_pub_->publish(target);
    if (effective != last_state_) {
      state_pub_->publish(textMessage(effective));
      RCLCPP_INFO(get_logger(), "%s", effective.c_str());
      last_state_ = effective;
    }
    if((now()-last_diagnostic_).seconds()>.2){
      last_diagnostic_=now();
      const double sonar_evidence_now=steadySeconds();
      std::ostringstream os;
      if((now()-last_sonar_debug_).seconds()>=1.0){
        KeySet nearby;
        for(Key k:sonar_memory_)if(distance(grid_->point(k),current_)<3.)nearby.insert(k);
        sonar_debug_pub_->publish(makeCloud(nearby,grid_->cfg.resolution,frame_,now()));last_sonar_debug_=now();
      }
      os<<"px4_clock="<<px4_clock_.name()<<" px4_clock_healthy="<<px4_clock_.ready(steadySeconds())
        <<" px4_clock_locked="<<px4_clock_.flightLocked()<<" px4_sim_clock_required="<<px4_clock_.require_sim_clock<<" px4_timestamp_us="<<px4_stamp
        <<" px4_status_timestamp_us="<<px4_clock_.sourceStamp()<<" px4_nav_state="<<px4_nav_state_
        <<" px4_armed="<<armed_<<" px4_status_age="<<(have_status_?(now()-status_stamp_).seconds():-1.)
        <<" target_system="<<target_system_<<" px4_status_system_id="<<px4_status_system_id_
        <<" px4_landed="<<(have_landed_?int(px4_landed_):-1)<<" separation_active="<<separating_active_
        <<" separation_reason="<<separation_reason_<<" separation_observation_used="<<separation_turn_.used()<<" map_clearance="<<grid_->clearanceAt(current_)
        <<" sonar_clearance="<<sonar_collision_->clearanceAt(current_)<<" ";
      os<<"state="<<effective<<" source_state="<<state<<" sonar_range="<<front_distance_
        <<" sonar_age="<<(now()-ultrasonic_stamp_).seconds()<<" sonar_limit="<<sonar.speed_limit
        <<" brake_distance="<<sonar.stop_required<<" route_limit="<<u.speed_limit
        <<" measured_speed="<<norm(velocity_)<<" command_speed="<<norm(u.velocity)
        <<" yaw_error_deg="<<wrap(u.yaw-yaw_current_)*180/pi
        <<" course_error_deg="<<u.course_error*180/pi<<" heading_wait="<<u.heading_wait
        <<" alignment_active="<<u.alignment_active
        <<" heading_speed_scale="<<u.heading_speed_scale
        <<" alignment_forward_enabled="<<u.alignment_forward_enabled
        <<" alignment_min_speed="<<u.alignment_min_speed_active
        <<" alignment_forward_applied="<<alignment_forward_applied
        <<" alignment_forward_speed_cap="<<direction_config_.alignment_forward_speed_cap
        <<" stagnant_seconds="<<progress_watchdog_.stagnantSeconds()
        <<" translation_stagnant_seconds="<<progress_watchdog_.translationStagnantSeconds()
        <<" path_wait_seconds="<<(path_wait_since_.nanoseconds()==0?0:(now()-path_wait_since_).seconds())
        <<" normal_envelope_clear="<<(have_map_&&grid_->segment(current_,current_))
        <<" recovery_envelope_clear="<<(have_map_&&grid_->recoveryClear(current_))
        <<" command_accel="<<norm(tracker_->acceleration())<<" radius="<<grid_->cfg.inflation_xy
        <<" recovery_radius="<<grid_->cfg.recovery_xy<<" progress="<<tracker_->progress()
        <<" current_x="<<current_.x<<" current_y="<<current_.y<<" current_z="<<current_.z
        <<" map_age="<<(now()-map_stamp_).seconds()<<" odom_age="<<(now()-odom_stamp_).seconds()
        <<" heading_mode="<<(heading_mode_==HeadingMode::Relaxed?"BOUNDED_RELAXED":"FRONT_PROTECTED")
        <<" directional_guard="<<direction_reason_<<" glass_risk="<<glass_risk_active_
        <<" verified_retreat="<<retreat_mode_<<" sonar_memory_cells="<<sonar_memory_.size()
        <<" sonar_memory_oldest_age="<<sonar_layer_.oldestAge(sonar_evidence_now)
        <<" sonar_memory_newest_age="<<sonar_layer_.newestAge(sonar_evidence_now)
        <<" sonar_memory_expired_total="<<sonar_layer_.expiredTotal()
        <<" sonar_memory_cleared_total="<<sonar_layer_.clearedTotal()
        <<" sonar_memory_time_resets="<<sonar_layer_.timeResetTotal()
        <<" sonar_epoch_resets="<<sonar_epoch_resets_
        <<" unresolved_sonar_risks="<<unlocalized_sonar_.size(sonar_evidence_now,unlocalized_sonar_ttl_)
        <<" radar_only_age="<<(now()-radar_only_stamp_).seconds()
        <<" nonforward_distance="<<nonforward_budget_.usedDistance()
        <<" nonforward_seconds="<<nonforward_budget_.usedSeconds()
        <<" nonforward_budget_exhausted="<<nonforward_budget_.exhausted()
        <<" planner_permit_ready="<<route_channel_.gate.permit.ready
        <<" planner_permit_reason="<<route_channel_.gate.permit.reason
        <<" route_revision="<<route_channel_.active_revision
        <<" passage_active="<<portal_active_<<" passage_waiting="<<portal_waiting_
        <<" passage_granted="<<passage_granted_<<" passage_reply="<<passage_reply_state_
        <<" passage_wait_seconds="<<passage_wait_monitor_.seconds
        <<" passage_completions="<<passage_completions_
        <<" paired_free_map="<<bool(observed_grid_)<<" free_map_age="<<(now().seconds()-double(free_stamp_)*1e-9)
        <<" free_matches_latest_occupied="<<(free_stamp_==occupied_stamp_)
        <<" map_transport="<<(compact_maps_?"compact":"legacy")
        <<" snapshot_commits="<<snapshot_commits_<<" snapshot_rejected="<<snapshot_rejected_
        <<" snapshot_bytes="<<snapshot_bytes_
        <<" measured_braking_risk="<<measured_braking_risk_;
      os<<" strict_unobserved_fallback="<<strict_after_unobserved_
        <<" path_available="<<have_path_<<" path_points="<<pending_path_.size()
        <<" path_age="<<(now()-path_stamp_).seconds()<<" recovery_handoff_ready="<<handoff_ready_;
      const double peer_watch_time=now().seconds();
      os<<" runtime_version=2.1.2-geometric-cooperative"
        <<" startup_spread_enabled="<<startup_spread_enabled_
        <<" startup_spread_offset_x="<<startup_spread_offset_x_
        <<" startup_spread_offset_y="<<startup_spread_offset_y_
        <<" startup_spread_spacing="<<startup_spread_spacing_
        <<" startup_formation_rank="<<startup_formation_rank_
        <<" startup_formation_count="<<startup_formation_count_
        <<" startup_spread_target_x="<<startup_spread_target_.x
        <<" startup_spread_target_y="<<startup_spread_target_.y
        <<" startup_spread_remaining="<<horizontalNorm(startup_spread_target_-current_)
        <<" startup_spread_complete="<<startup_spread_complete_
        <<" startup_spread_blocked="<<startup_spread_blocked_
        <<" startup_spread_block_reason="<<startup_spread_block_reason_
        <<" peer_safety_reason="<<peerSafetyReasonName(last_peer_decision_.reason)
        <<" peer_fresh="<<last_peer_decision_.fresh_peers
        <<" peer_nearest_distance="<<last_peer_decision_.nearest_distance
        <<" peer_nearest_clearance="<<last_peer_decision_.nearest_clearance
        <<" peer_predicted_miss="<<last_peer_decision_.predicted_miss
        <<" peer_predicted_time="<<last_peer_decision_.predicted_time
        <<" peer_blocking_priority="<<last_peer_decision_.blocking_priority
        <<" peer_yield_active="<<peer_yield_active_
        <<" peer_yield_stalled="<<peer_yield_stalled_
        <<" peer_yield_seconds="<<peer_yield_watchdog_.activeSeconds(peer_watch_time)
        <<" peer_blocker_stagnant_seconds="<<peer_yield_watchdog_.stagnantSeconds(peer_watch_time)
        <<" peer_rear_yields_ignored="<<last_peer_decision_.rear_yields_ignored
        <<" peer_deadlock_recovery="<<peer_deadlock_recovery_
        <<" peer_blocker_position_valid="<<have_peer_blocker_position_
        <<" peer_blocker_x="<<peer_blocker_position_.x
        <<" peer_blocker_y="<<peer_blocker_position_.y
        <<" passage_reservation="<<last_passage_reservation_.valid
        <<" passage_own_inside="<<last_passage_reservation_.own_inside
        <<" passage_anchor_x="<<last_passage_reservation_.anchor.x
        <<" passage_anchor_y="<<last_passage_reservation_.anchor.y
        <<" sonar_queue="<<sonar_pending_.size()
        <<" sonar_matched="<<sonar_matched_
        <<" sonar_pose_fallbacks="<<sonar_pose_fallbacks_
        <<" sonar_pose_drops="<<sonar_pose_drops_
        <<" gap_width="<<gap_state_.width
        <<" gap_extra_margin="<<gap_state_.extra_margin<<" gap_speed_limit="<<gap_state_.speed
        <<" local_recovery="<<recovery_.reason<<" recovery_attempts="<<recovery_.attempts()
        <<" map_speed_factor="<<map_speed_factor_
        <<" proposed_vx="<<proposed_velocity.x<<" proposed_vy="<<proposed_velocity.y<<" proposed_vz="<<proposed_velocity.z
        <<" sent_vx="<<u.velocity.x<<" sent_vy="<<u.velocity.y<<" sent_vz="<<u.velocity.z
        <<" altitude_in_bounds="<<grid_->insideAltitude(current_)
        <<" map_geometry_clear="<<(have_map_&&grid_->geometrySegment(current_,current_))
        <<" sonar_geometry_clear="<<sonar_collision_->geometrySegment(current_,current_)
        <<" comfort_margin="<<grid_->cfg.comfort_margin
        <<" mission_distance="<<(have_mission_goal_?distance(current_,mission_goal_):-1.)
        <<" altitude_return_active="<<altitude_return_active_
        <<" recovery_total_actions="<<campaign_.actions()<<" recovery_distance_spent="<<campaign_.distanceSpent()
        <<" observed_escape_active="<<observed_escape_active_<<" goal_geometry_blocked="<<goal_geometry_blocked_
        <<" goal_adjusted="<<(have_approach_&&distance(approach_goal_,mission_goal_)>.01)
        <<" navigation_goal_distance="<<(have_mission_goal_?distance(current_,navigationGoal()):-1.)
        <<" useful_stagnant_seconds="<<progress_watchdog_.usefulStagnantSeconds()
        <<" history_points="<<retreat_history_.size()<<" history_retreat_active="<<history_retreat_active_
        <<" retreat_length="<<retreat_route_.length<<" retreat_displacement="<<retreat_route_.displacement;
      diagnostic_pub_->publish(textMessage(os.str()));
      if(u.blocked)replan_pub_->publish(textMessage("LOCAL_PATH_BLOCKED"));
      else if(sonar.reason=="ULTRASONIC_STOP")replan_pub_->publish(textMessage("ULTRASONIC_STOP"));
      RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),2000,"%s",os.str().c_str());
    }
  }
  void prepareHold(const std::string &reason) {
    if (!holding_) {
      hold_anchor_ = current_;
      holding_ = true;
      hold_braking_=norm(velocity_)>.05;
      // A withdrawn path must not keep commanding its previous turn.
      if(state_==State::NAV)tracker_->initializeYaw(yaw_current_);
    }
    hold_reason_=reason;
  }
  void hold(double dt, const std::string &reason) {
    prepareHold(reason);
    if(hold_braking_){
      // A transient input pause must decelerate at the current position, not
      // command a return to the point where braking began. Capture the final
      // stationary hold anchor only after actual motion has settled.
      hold_anchor_=current_;
      hold_braking_=norm(velocity_)>.05;
      publish(tracker_->hold(current_,current_,{},dt),reason);
      return;
    }
    // Old anchors can become unsafe when a new map arrives or while the vehicle
    // drifts out of an inflated margin. Never pull it back into that margin.
    if(state_==State::NAV && have_map_)
      hold_anchor_=safeHoldAnchor(*grid_,current_,hold_anchor_);
    publish(tracker_->hold(current_, hold_anchor_, velocity_, dt), reason);
  }
  bool recoveryCertificate(Vec3 a,Vec3 b){
    if(!adaptive_direction_||!collisionClear(a,b)||!observedMotion(a,b))return false;
    const auto sonar=frontLimit();
    if(use_ultrasonic_ && require_ultrasonic_ && sonar.reason=="ULTRASONIC_UNAVAILABLE" && !verifiedRetreat(a,b))return false;
    if(glass_risk_active_ && !verifiedRetreat(a,b))return false;
    const Vec3 direction=b-a;
    const Vec3 v=direction*(recovery_.cfg.speed/std::max(1e-9,norm(direction)));
    if(use_ultrasonic_ && maximumConeClosing(v,sonar_pose_.yaw()+sonar_yaw_,sonar_half_angle_)>1e-9 && sonar.speed_limit<=1e-6)return false;
    return true;
  }
  bool historicalCertificate(Vec3 a,Vec3 b,const RetreatRoute&proof){
    if(!adaptive_direction_||!proof.covers(a,b,now().seconds())||!collisionClear(a,b))return false;
    // Pure geometric check for each actual leg, not a current->future chord.
    // A live front range at the current position cannot certify or prohibit a
    // distant future leg around a corner; check it when that leg becomes active.
    return !detectedGlassRisk(a,b) || awayFromSonar(a,b);
  }
  bool historicalActiveCertificate(Vec3 a,Vec3 b,const RetreatRoute&proof){
    if(!historicalCertificate(a,b,proof))return false;
    const auto sonar=frontLimit();
    const bool nonclosing=maximumConeClosing(b-a,sonar_pose_.yaw()+sonar_yaw_,sonar_half_angle_)<=1e-8;
    if((glass_risk_active_ || detectedGlassRisk(a,b)) && (!nonclosing || !awayFromSonar(a,b)))return false;
    if(use_ultrasonic_&&require_ultrasonic_&&sonar.reason=="ULTRASONIC_UNAVAILABLE"&&
       (!nonclosing||!awayFromSonar(a,b)))return false;
    const Vec3 v=(b-a)*(recovery_.cfg.speed/std::max(1e-9,distance(a,b)));
    return !use_ultrasonic_ || maximumConeClosing(v,sonar_pose_.yaw()+sonar_yaw_,sonar_half_angle_)<=1e-9 || sonar.speed_limit>1e-6;
  }
  void endRecovery(){
    handoff_.clear();handoff_ready_=false;
    history_retreat_active_=false;observed_escape_active_=false;separating_active_=false;observed_escape_.clear();
    campaign_.finish(now().seconds());recovery_requested_=false;holding_=false;
    peer_deadlock_recovery_=false;peer_yield_stalled_=false;
    tracker_->resetMotion();tracker_->initializeYaw(yaw_current_);
    tracker_->setPath(have_path_?pending_path_:std::vector<Vec3>{},current_);
    replan_pub_->publish(textMessage("LOCAL_RECOVERY_FINISHED"));
  }
  // Invoked BEFORE the physical-envelope hold. There is no mission callback
  // dependency, no radius shrink, and no speculative flight into open-looking SDF.
  bool runEnvelopeSeparation(double dt){
    const double time=now().seconds();
    if(separation_turn_.active()){
      const double yaw=separation_turn_.step(time,dt,yaw_current_);
      tracker_->resetMotion();tracker_->initializeYaw(yaw);
      separation_reason_="OBSERVE_EXIT";
      publish({{},current_,yaw,0,false,false},"OBSERVE_SEPARATION_EXIT");return true;
    }
    if(!separating_active_){
      if(!separation_enabled_||!autonomous_recovery_||!adaptive_direction_)return false;
      if(recovery_.active()){recovery_.cancel();endRecovery();}
      const bool due=recovery_.due(time,true)||(recovery_.attempts()>=recovery_.cfg.attempts&&campaign_.retryReady(time));
      if(!due||!campaign_.retryReady(time)){separation_reason_="WAIT_OR_BUDGET";return false;}
      if(recoveryNeedsBraking(velocity_)){separation_reason_="WAIT_MEASURED_STOP";return false;}
      separation_map_=separationProof(*grid_,current_,separation_max_overlap_);
      separation_sonar_=separationProof(*sonar_collision_,current_,separation_max_overlap_);
      if(!separation_map_.valid||!separation_sonar_.valid){separation_reason_=!separation_map_.valid?"MAP_OVERLAP_TOO_DEEP":"SONAR_OVERLAP_TOO_DEEP";campaign_.searched(time);return false;}
      separation_history_=RetreatRoute{};
      separating_active_=true; // enable only the narrowly scoped witness queries
      auto route=separationRoute(current_,[this](Vec3 a,Vec3 b){
        return grid_->segment(b,b)&&sonar_collision_->segment(b,b)&&campaign_.novel(b)&&
          observedMapMotion(a,b)&&collisionClear(a,b)&&
          (!detectedGlassRisk(a,b)||separatingRetreat(a,b))&&
          (!use_ultrasonic_||(frontLimit().reason!="ULTRASONIC_UNAVAILABLE"&&
            (maximumConeClosing(b-a,sonar_pose_.yaw()+sonar_yaw_,sonar_half_angle_)<=1e-8||frontLimit().speed_limit>0)));
      },[this](Vec3 p){return std::min(grid_->clearanceAt(p),sonar_collision_->clearanceAt(p));});
      if(route.empty()&&historical_retreat_enabled_){
        auto historical=retreat_history_.select(current_,time,
          [this](Vec3 a,Vec3 b,const RetreatRoute&r){return r.covers(a,b,now().seconds())&&collisionClear(a,b)&&
            (!detectedGlassRisk(a,b)||(awayFromSonar(a,b)&&maximumConeClosing(b-a,sonar_pose_.yaw()+sonar_yaw_,sonar_half_angle_)<=1e-8));},
          [this](Vec3 p){return grid_->segment(p,p)&&sonar_collision_->segment(p,p)?
            std::min(grid_->clearanceAt(p),sonar_collision_->clearanceAt(p)):-1e6;},true);
        if(historical.validAt(time)&&campaign_.novel(historical.points.back())&&grid_->segment(historical.points.back(),historical.points.back())&&
           sonar_collision_->segment(historical.points.back(),historical.points.back())){
          separation_history_=std::move(historical);route=separation_history_.points;separation_waypoint_=1;
        }
      }
      if(route.empty()&&!separation_turn_.used()){
        auto intent=separationRoute(current_,[this](Vec3 a,Vec3 b){return grid_->segment(b,b)&&sonar_collision_->segment(b,b)&&collisionClear(a,b);},
          [this](Vec3 p){return std::min(grid_->clearanceAt(p),sonar_collision_->clearanceAt(p));});
        if(!intent.empty())separation_turn_.begin(time,current_,yaw_current_,intent.back());
        // Geometry chooses only where to LOOK. Translation still needs observed
        // space or a recent, frozen actual-flight tube after that turn.
      }
      if(route.empty()||!campaign_.permits(time,route)){
        separating_active_=false;separation_reason_="NO_OBSERVED_SEPARATION";campaign_.searched(time);return false;
      }
      if(recovery_.attempts()>=recovery_.cfg.attempts)recovery_.reset();
      if(!recovery_.beginCertifiedRoute(time,route.back(),8.0)||!campaign_.begin(time,route)){
        recovery_.cancel();separating_active_=false;separation_reason_="BUDGET_REJECTED";return false;
      }
      holding_=false;tracker_->resetMotion();tracker_->initializeYaw(yaw_current_);
      separation_reason_="MOVING_AWAY";replan_pub_->publish(textMessage("ENVELOPE_SEPARATION_STARTED"));
    }
    Vec3 target=recovery_.target();target.z=current_.z;
    if(separation_history_.validAt(time)){
      while(separation_waypoint_+1<separation_history_.points.size()&&
        distance(current_,separation_history_.points[separation_waypoint_])<.04&&
        separationObserved(current_,separation_history_.points[separation_waypoint_+1]))++separation_waypoint_;
      target=separation_history_.points[separation_waypoint_];target.z=current_.z;
    }
    bool valid=separationObserved(current_,target)&&collisionClear(current_,target)&&
      (!detectedGlassRisk(current_,target)||separatingRetreat(current_,target));
    // Reject newly blocked FUTURE legs too, before committing to a curved
    // historical retreat. Project only inside the original measured tube.
    if(!separation_history_.points.empty()){
      valid=valid&&separation_history_.validAt(time);
      Vec3 previous=current_;
      for(size_t i=separation_waypoint_;valid&&i<separation_history_.points.size();++i){
        Vec3 next=separation_history_.points[i];next.z=current_.z;
        valid=separation_history_.covers(previous,next,time)&&collisionClear(previous,next)&&
          (!detectedGlassRisk(previous,next)||awayFromSonar(previous,next));
        previous=next;
      }
    }
    if(!recovery_.step(time,current_,valid)){
      separation_reason_=recovery_.reason;endRecovery();return false;
    }
    auto u=tracker_->hold(current_,target,velocity_,dt);
    u.velocity.z=0;u.velocity=limitNorm(u.velocity,separation_speed_);u.reached=false;
    publish(u,"SEPARATING_ENVELOPE_ESCAPE");return true;
  }
  Vec3 navigationGoal()const{return have_approach_?approach_goal_:mission_goal_;}
  bool runRecovery(double dt,bool needed){
    if(recovery_.active() && !separating_active_){
      const double time=now().seconds();
      const bool fresh=have_path_&&plannerAllowed()&&(now()-path_stamp_).seconds()>=0&&(now()-path_stamp_).seconds()<path_timeout_;
      if(!fresh){handoff_.clear();handoff_ready_=false;}
      if(time<last_handoff_check_ || time-last_handoff_check_>=.20){
        last_handoff_check_=time;Vec3 join{};
        const bool certified=fresh&&certifiedRouteHandoff(pending_path_,current_,
          [this](Vec3 a,Vec3 b){return observedMapMotion(a,b)&&!detectedGlassRisk(a,b);},join);
        handoff_ready_=handoff_.update(certified,time,join);
      }
      if(handoff_ready_){
        if(!recovery_.step(time,current_,true)){endRecovery();return false;}
        // Brake before replacing a retreat with normal tracking. The fresh
        // prefix was independently certified; normal tracking rechecks it.
        if(recoveryNeedsBraking(velocity_)){
          publish(tracker_->hold(current_,current_,{},dt),"RECOVERY_HANDOFF_BRAKE");return true;
        }
        recovery_.cancel();endRecovery();return false;
      }
    }
    if(!recovery_.active() && (!have_mission_goal_||distance(current_,navigationGoal())<2*tracker_->cfg.goal_tolerance))return false;
    const bool ordinary_due=recovery_.due(now().seconds(),needed);
    const bool renewed_due=autonomous_recovery_&&needed&&recovery_.attempts()>=recovery_.cfg.attempts&&campaign_.retryReady(now().seconds());
    if((ordinary_due||renewed_due)&&(!autonomous_recovery_||campaign_.retryReady(now().seconds()))){
      // Freeze history only AFTER braking. Freezing a 4 cm tube while still
      // moving at 0.20--0.25 m/s; ordinary stopping drift immediately invalidated
      // both recorded escapes before reverse motion could begin.
      if(recoveryNeedsBraking(velocity_)){
        prepareHold("RECOVERY_BRAKING");
        publish(tracker_->hold(current_,current_,{},dt),"RECOVERY_BRAKING");return true;
      }
      if(!nonforward_session_){frozen_trail_=trail_;nonforward_session_=true;forward_reset_progress_=0;}
      updateGlassRisk(current_);
      bool started=false;
      auto begin_route=[&](const std::vector<Vec3>&route,double duration){
        if(autonomous_recovery_&&!campaign_.permits(now().seconds(),route))return false;
        if(autonomous_recovery_&&recovery_.attempts()>=recovery_.cfg.attempts)recovery_.reset();
        if(!recovery_.beginCertifiedRoute(now().seconds(),route.back(),duration))return false;
        if(autonomous_recovery_&&!campaign_.begin(now().seconds(),route)){recovery_.cancel();return false;}
        return true;
      };
      // Ordinary stalls prefer a goal-directed observed route.  A peer-yield
      // deadlock must not do that: the peer is deliberately absent from the
      // obstacle map, so a goal-directed search would repeatedly choose the
      // same blocked forward route.  It first uses actual flight history below.
      if(autonomous_recovery_&&!glass_risk_active_&&!peer_deadlock_recovery_){
        auto cfg=escape_config_;cfg.max_length=std::min(cfg.max_length,campaign_.cfg.max_distance-campaign_.distanceSpent());
        auto route=observedEscapeRoute(current_,navigationGoal(),cfg,
          [this](Vec3 a,Vec3 b){return observedMapMotion(a,b)&&!detectedGlassRisk(a,b)&&recoveryCertificate(a,b);},
          [this](Vec3 p){return std::min(grid_->clearanceAt(p),sonar_collision_->clearanceAt(p));},
          [this](Vec3 p){return campaign_.novel(p);});
        if(!route.empty()&&begin_route(route,8.0)){
          observed_escape_=std::move(route);observed_waypoint_=1;observed_escape_active_=true;
          recovery_.reason="CERTIFIED_OBSERVED_ESCAPE";started=true;
        }
      }
      if(!started&&historical_retreat_enabled_){
        auto selected=retreat_history_.select(current_,now().seconds(),
          [this](Vec3 a,Vec3 b,const RetreatRoute&r){return historicalCertificate(a,b,r);},
          [this](Vec3 p){return std::min(grid_->clearanceAt(p),sonar_collision_->clearanceAt(p));});
        if(selected.valid()&&historicalActiveCertificate(current_,selected.points[1],selected)&&begin_route(selected.points,retreat_duration_)){
          retreat_route_=std::move(selected);retreat_waypoint_=1;history_retreat_active_=true;started=true;
        }
      }
      // A vehicle which has no usable history (for example, it stopped at the
      // entrance immediately after takeoff) receives a short observed-space
      // target directly away from the stagnant blocker.  The normal map,
      // sonar, glass and final pairwise safety checks still certify every leg.
      if(!started&&autonomous_recovery_&&peer_deadlock_recovery_&&
         have_peer_blocker_position_&&!glass_risk_active_){
        Vec3 away=current_-peer_blocker_position_;away.z=0;
        const double away_norm=horizontalNorm(away);
        const double remaining=campaign_.cfg.max_distance-campaign_.distanceSpent();
        if(away_norm>.05&&remaining>=.40){
          auto cfg=escape_config_;
          cfg.max_length=std::min({cfg.max_length,.80,remaining});
          const Vec3 escape_goal=current_+away*(cfg.max_length/away_norm);
          auto route=observedEscapeRoute(current_,escape_goal,cfg,
            [this](Vec3 a,Vec3 b){return observedMapMotion(a,b)&&
              !detectedGlassRisk(a,b)&&recoveryCertificate(a,b);},
            [this](Vec3 p){return std::min(grid_->clearanceAt(p),sonar_collision_->clearanceAt(p));},
            [this](Vec3 p){return campaign_.novel(p);});
          if(!route.empty()&&begin_route(route,8.0)){
            observed_escape_=std::move(route);observed_waypoint_=1;
            observed_escape_active_=true;
            recovery_.reason="PEER_DEADLOCK_ESCAPE";started=true;
          }
        }
      }
      if(!started&&!autonomous_recovery_)started=recovery_.start(now().seconds(),current_,navigationGoal(),
          [this](Vec3 a,Vec3 b){return !detectedGlassRisk(a,b)&&recoveryCertificate(a,b);},
          [this](Vec3 p){return std::min(grid_->clearanceAt(p),sonar_collision_->clearanceAt(p));});
      if(started){
        handoff_.clear();handoff_ready_=false;
        holding_=false;tracker_->resetMotion();
        tracker_->initializeYaw(yaw_current_);
        replan_pub_->publish(textMessage(observed_escape_active_?"OBSERVED_ESCAPE_STARTED":history_retreat_active_?"HISTORY_RETREAT_STARTED":"LOCAL_RECOVERY_STARTED"));
      }else{if(autonomous_recovery_)campaign_.searched(now().seconds());replan_pub_->publish(textMessage("NO_CERTIFIED_ESCAPE"));}
    }
    if(!recovery_.active())return false;
    Vec3 target=recovery_.target();
    bool certified=false;
    if(observed_escape_active_){
      auto valid=[this](Vec3 a,Vec3 b){return observedMapMotion(a,b)&&!detectedGlassRisk(a,b)&&recoveryCertificate(a,b);};
      while(observed_waypoint_+1<observed_escape_.size()&&distance(current_,observed_escape_[observed_waypoint_])<.20&&
        valid(current_,observed_escape_[observed_waypoint_+1]))++observed_waypoint_;
      target=observed_escape_[observed_waypoint_];Vec3 previous=current_;certified=true;
      for(size_t i=observed_waypoint_;i<observed_escape_.size();++i){
        if(!valid(previous,observed_escape_[i])){certified=false;break;}previous=observed_escape_[i];
      }
      updateGlassRisk(target);
    }else if(history_retreat_active_){
      while(retreat_waypoint_+1<retreat_route_.points.size() &&
            distance(current_,retreat_route_.points[retreat_waypoint_])<.04 &&
            historicalActiveCertificate(current_,retreat_route_.points[retreat_waypoint_+1],retreat_route_))++retreat_waypoint_;
      certified=retreat_route_.certifiedFrom(current_,retreat_waypoint_,now().seconds(),
        [this](Vec3 a,Vec3 b,const RetreatRoute&r){return historicalCertificate(a,b,r);});
      target=retreat_route_.points[retreat_waypoint_];
      updateGlassRisk(target);
      certified=certified&&historicalActiveCertificate(current_,target,retreat_route_);
    }else{
      updateGlassRisk(target);certified=recoveryCertificate(current_,target);
    }
    if(!recovery_.step(now().seconds(),current_,certified)){
      endRecovery();return false;
    }
    auto u=tracker_->hold(current_,target,velocity_,dt);
    u.velocity=limitNorm(u.velocity,recovery_.cfg.speed);u.reached=false;
    publish(u,observed_escape_active_?"CERTIFIED_OBSERVED_ESCAPE":history_retreat_active_?"CERTIFIED_HISTORY_RETREAT":"CERTIFIED_LOCAL_ESCAPE");return true;
  }
  bool startupSpreadObstacle(const Control &control){
    Vec3 course=startup_spread_target_-current_;course.z=0;
    const double remaining=horizontalNorm(course);
    if(remaining<=startup_spread_tolerance_)return false;
    const Vec3 direction=course*(1./remaining);
    const double reaction=.30+tracker_->cfg.max_accel_xy/
      tracker_->cfg.max_jerk_xy;
    const double brake_accel=std::min(.7*tracker_->cfg.max_accel_xy,
      sonar_guard_->cfg.brake_accel);
    const double braking=startup_spread_speed_*reaction+
      startup_spread_speed_*startup_spread_speed_/(2*brake_accel);
    const double probe_distance=std::min(remaining,
      std::max(startup_spread_obstacle_lookahead_,braking));
    Vec3 probe=current_+direction*probe_distance;
    probe.z=startup_spread_target_.z;
    if(!collisionClear(current_,probe)){
      startup_spread_block_reason_="MAP_OR_SONAR_OCCUPIED";return true;
    }
    if(detectedGlassRisk(current_,probe)){
      startup_spread_block_reason_="GLASS_OR_UNLOCALIZED_SONAR";return true;
    }
    const double command_speed=horizontalNorm(control.velocity);
    if(command_speed>1e-5){
      const Vec3 command_direction{control.velocity.x/command_speed,
        control.velocity.y/command_speed,0};
      const double command_braking=command_speed*reaction+
        command_speed*command_speed/(2*brake_accel);
      Vec3 command_probe=current_+command_direction*
        std::max(.15,command_braking);
      command_probe.z=startup_spread_target_.z;
      if(!collisionClear(current_,command_probe) ||
         detectedGlassRisk(current_,command_probe)){
        startup_spread_block_reason_="EMITTED_COMMAND_OBSTACLE";return true;
      }
    }
    if(use_ultrasonic_ && command_speed>1e-5){
      const auto sonar=frontLimitFor(control.velocity);
      const double closing=maximumConeClosing(control.velocity,
        sonar_pose_.yaw()+sonar_yaw_,sonar_half_angle_);
      if(require_ultrasonic_ && sonar.reason=="ULTRASONIC_UNAVAILABLE"){
        startup_spread_block_reason_="ULTRASONIC_UNAVAILABLE";return true;
      }
      if(closing>1e-5 && (sonar.reason=="ULTRASONIC_STOP" ||
         sonar.reason=="ULTRASONIC_BRAKE" || sonar.speed_limit+1e-6<closing)){
        startup_spread_block_reason_="LIVE_ULTRASONIC_OBSTACLE";return true;
      }
    }
    startup_spread_block_reason_="CLEAR";
    return false;
  }
  void synchronizeRoute(){
    if(!have_mission_goal_)return;
    const bool changed=route_channel_.synchronize(now().seconds(),steadySeconds(),
      planner_execution_timeout_,mission_goal_);
    if(route_channel_.gate.allowed(route_channel_.active_revision,now().seconds(),
        steadySeconds(),planner_execution_timeout_))
      path_stamp_=rclcpp::Time(route_channel_.gate.permit.stamp_ns,get_clock()->get_clock_type());
    if(!changed)return;
    pending_path_=route_channel_.active_path;have_path_=!pending_path_.empty();
    handoff_.clear();handoff_ready_=false;last_handoff_check_=-1;
    if(state_==State::NAV&&!recovery_.active())tracker_->setPath(pending_path_,current_);
  }
  bool plannerAllowed()const{
    return have_mission_goal_&&route_channel_.allowed(now().seconds(),steadySeconds(),
      planner_execution_timeout_,mission_goal_);
  }
  void completePassage(const char *reason){
    portal_departure_.remember(portal_anchor_,current_,passage_exit_distance_);
    portal_active_=passage_granted_=portal_waiting_=false;
    portal_progress_.reset();passage_wait_monitor_.reset();
    passage_reply_state_=reason;++passage_completions_;
    last_portal_request_=-10;
  }
  void receivePassageGrant(const nav_msgs::msg::Path &m){
    if(!portal_active_||m.header.frame_id!=frame_||m.poses.size()>2)return;
    const double source=double(stampNs(m.header.stamp))*1e-9;
    if(source<portal_epoch_||source<passage_reply_source_||
       now().seconds()-source>passage_lease_timeout_||source>now().seconds()+.05)return;
    if(m.poses.empty()){
      passage_reply_source_=source;passage_granted_=false;passage_reply_state_="WAITING";return;
    }
    const auto &p=m.poses.front().pose.position;
    if(stampNs(m.poses.front().header.stamp)!=portal_id_||!finite({p.x,p.y,p.z})||
       distance({p.x,p.y,p.z},portal_anchor_)>.3)return;
    passage_reply_source_=source;
    if(m.poses.size()==2){
      const auto &q=m.poses[1].pose.position;
      // Completion is a measured exit, never an empty-grant or timeout bypass.
      // The current pose must still be outside the physical release radius;
      // the echoed measurement must have crossed its additional hysteresis.
      if(passageOutside({q.x,q.y,q.z},portal_anchor_,passage_exit_distance_)&&
         distance(current_,portal_anchor_)>passage_exit_distance_){
        completePassage("COMPLETED_ACK");sendPassageRequest();
      }else{passage_granted_=false;passage_reply_state_="COMPLETION_POSITION_MISMATCH";}
      return;
    }
    passage_granted_=true;passage_grant_time_=source;passage_reply_state_="GRANTED";
  }
  void updatePassageAdmission(){
    portal_waiting_=false;
    if(!passage_scheduler_enabled_)return;
    const double odom_age=(now()-odom_stamp_).seconds();
    if(!have_odom_ || odom_age<-.05 || odom_age>odom_timeout_)return;
    const double t=now().seconds();
    releasePassageIfExited();
    const bool terminal=have_mission_goal_&&distance(current_,navigationGoal())<=tracker_->cfg.goal_tolerance;
    if(!terminal&&!portal_active_&&state_==State::NAV&&have_path_&&!recovery_.active()&&plannerAllowed()&&
       (t<last_portal_detect_||t-last_portal_detect_>=passage_detect_period_)){
      last_portal_detect_=t;
      std::vector<Vec3> prefix{current_};
      for(double s=.35;s<=passage_lookahead_;s+=.35)
        prefix.push_back(tracker_->futurePoint(current_,s));
      TopologyConfig detection;detection.passage_width=passage_width_;
      detection.probe_distance=std::max(2.5,passage_width_*.5+.1);
      const auto deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(6);
      const auto gates=routePassages(*grid_,prefix,detection,[&]{
        return std::chrono::steady_clock::now()>=deadline;});
      for(const auto &gate:gates){
        if(portal_departure_.suppress(gate.center,current_))continue;
        portal_anchor_=gate.center;portal_direction_=gate.normal;
        if(dot(portal_direction_,tracker_->futurePoint(current_,.7)-current_)<0)portal_direction_=portal_direction_*-1.;
        portal_epoch_=t;portal_id_=std::max(portal_id_+1,now().nanoseconds());
        portal_active_=true;passage_granted_=false;portal_progress_.reset();
        passage_reply_source_=-1;passage_reply_state_="REQUESTED";last_portal_request_=-10;
        passage_wait_monitor_.reset();break;
      }
    }
    sendPassageRequest();
    if(!portal_active_)return;
    const bool lease=passage_granted_&&t>=passage_grant_time_&&
      t-passage_grant_time_<=passage_lease_timeout_;
    if(passage_granted_&&!lease)passage_reply_state_="LEASE_EXPIRED";
    portal_waiting_=!lease;
  }
  void releasePassageIfExited(){
    if(!portal_active_)return;
    const bool terminal=have_mission_goal_&&distance(current_,navigationGoal())<=tracker_->cfg.goal_tolerance;
    const bool granted=passage_granted_&&now().seconds()>=passage_grant_time_&&
      now().seconds()-passage_grant_time_<=passage_lease_timeout_;
    if(portal_progress_.exited(current_,portal_anchor_,portal_direction_,passage_exit_distance_,
        granted,terminal,!have_path_||recovery_.active()||
          (pending_path_.size()>1&&!routeCrossesPassage(pending_path_,
            {portal_anchor_,portal_direction_,passage_width_})))){
      completePassage("COMPLETED_LOCAL");
    }
  }
  void sendPassageRequest(){
    if(!passage_scheduler_enabled_)return;
    const double odom_age=(now()-odom_stamp_).seconds();
    if(!have_odom_ || odom_age<-.05 || odom_age>odom_timeout_)return;
    const double t=now().seconds();
    releasePassageIfExited();
    if(t-last_portal_request_>=.1||t<last_portal_request_){
      last_portal_request_=t;
      auto request=makePath(portal_active_?
        std::vector<Vec3>{portal_anchor_,current_,portal_anchor_+portal_direction_*passage_exit_distance_}:
        std::vector<Vec3>{current_},frame_,now());
      if(portal_active_)request.poses[0].header.stamp=rclcpp::Time(portal_id_,get_clock()->get_clock_type());
      passage_request_pub_->publish(request);
    }
  }
  double passageSpeedLimit(double max_speed)const{
    if(!portal_waiting_)return max_speed;
    const double remaining=dot(portal_anchor_-current_,portal_direction_)-passage_wait_distance_;
    const double reaction=.45+tracker_->cfg.max_accel_xy/tracker_->cfg.max_jerk_xy;
    const double a=.7*tracker_->cfg.max_accel_xy;
    return std::min(max_speed,std::max(0.,std::sqrt((a*reaction)*(a*reaction)+
      2*a*std::max(0.,remaining))-a*reaction));
  }
  void control() {
    if (!have_odom_)
      return;
    // Collision-memory age uses a monotonic process clock. A Gazebo /clock
    // rewind can no longer place old cells in the future and preserve them.
    refreshSonarMemory(steadySeconds());
    double dt = last_control_.nanoseconds() == 0 ? 1.0 / 30 : (now() - last_control_).seconds();
    last_control_ = now();
    if (dt <= 0)
      return;
    dt = std::min(dt, .15);
    if ((now() - odom_stamp_).seconds() > odom_timeout_ || (now()-odom_stamp_).seconds()<-.05) {
      tracker_->resetMotion();
      publish({{}, current_, tracker_->yaw(), 0, false, false}, "ODOM_STALE");
      return;
    }
    if(!px4_clock_.ready(steadySeconds())){
      tracker_->resetMotion();publish({{},current_,yaw_current_,0,false,false},
        px4_clock_.flightLocked()?"PX4_CLOCK_DEGRADED_HOLD":"WAIT_PX4_CLOCK");return;
    }
    if (state_ == State::WAIT) {
      tracker_->initializeYaw(yaw_current_);
      hold_anchor_ = current_;
      holding_ = true;
      takeoff_anchor_ = {current_.x, current_.y, takeoff_altitude_};
      startup_spread_target_={current_.x+startup_spread_offset_x_,
        current_.y+startup_spread_offset_y_,
        takeoff_altitude_};
      startup_spread_complete_=!startup_spread_enabled_ ||
        std::hypot(startup_spread_offset_x_,startup_spread_offset_y_)<=
          startup_spread_tolerance_;
      startup_spread_blocked_=false;
      startup_spread_block_reason_="PENDING";
      state_ = State::PRESTREAM;
      state_since_ = now();
    }
    if (state_ == State::PRESTREAM) {
      publish(tracker_->hold(current_, hold_anchor_, velocity_, dt), "PRESTREAM");
      // Lock BEFORE any mode/arm command can be sent. Waiting for the armed
      // status would leave a callback-delay window with an unlatched stream.
      if ((now() - state_since_).seconds() >= prestream_seconds_ && px4_clock_.latch(steadySeconds()))
        state_ = State::REQUEST;
      return;
    }
    bool status_fresh = have_status_ && (now() - status_stamp_).seconds() < status_timeout_;
    if (state_ == State::REQUEST) {
      if(!status_fresh)RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,
        "No fresh PX4 VehicleStatus: verify px4_msgs matches the running PX4 firmware and versioned status_topic.");
      publish(tracker_->hold(current_, hold_anchor_, velocity_, dt), "WAIT_ARM_OFFBOARD");
      if (status_fresh && armed_ && offboard_ && px4_clock_.flightLocked()) {
        state_ = State::TAKEOFF;
        takeoff_progress_.begin(now().seconds(),current_.z);
        state_since_ = now();
        return;
      }
      if (auto_arm_ && (now() - last_request_).seconds() > 1.0) {
        if (!status_fresh || !offboard_)
          command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
        if (!status_fresh || !armed_)
          command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1);
        last_request_ = now();
      }
      return;
    }
    if (!status_fresh || !armed_ || !offboard_) {
      hold(dt, "WAIT_PX4_STATUS_OR_OPERATOR_MODE");
      return;
    }
    if (state_ == State::TAKEOFF) {
      auto u=tracker_->hold(current_,takeoff_anchor_,velocity_,dt);
      // Avoid ground skidding while diagnosing a failed climb. Never increase
      // thrust limits, force-arm, disarm or switch modes to mask no response.
      if(current_.z<takeoff_anchor_.z-.5){Vec3 xy{u.velocity.x,u.velocity.y,0};xy=limitNorm(xy,.08);u.velocity.x=xy.x;u.velocity.y=xy.y;}
      const bool stalled=takeoff_progress_.stalled(now().seconds(),current_.z);
      publish(u,stalled?"TAKEOFF_NO_PROGRESS":"TAKEOFF");
      if(stalled)RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"Takeoff has no measured height progress for 8 s: check PX4 setpoint reception, actuator output and simulator contact; no force-arm/mode cycling attempted");
      if (std::abs(current_.z - takeoff_altitude_) < .12 && std::abs(velocity_.z) < .12) {
        startup_spread_target_={takeoff_anchor_.x+startup_spread_offset_x_,
          takeoff_anchor_.y+startup_spread_offset_y_,takeoff_altitude_};
        startup_spread_blocked_=false;
        if(startup_spread_enabled_ &&
           std::hypot(startup_spread_offset_x_,startup_spread_offset_y_)>
             startup_spread_tolerance_){
          state_=State::STARTUP_SPREAD;startup_spread_complete_=false;
          holding_=false;tracker_->resetMotion();tracker_->initializeYaw(yaw_current_);
        }else{
          state_=State::NAV;startup_spread_complete_=true;holding_=false;
          tracker_->setPath(have_path_?pending_path_:std::vector<Vec3>{},current_);
        }
      }
      return;
    }
    if(state_==State::STARTUP_SPREAD){
      const double map_age=(now()-map_stamp_).seconds();
      if(!have_map_ || map_age>map_timeout_ || map_age<-.05){
        startup_spread_blocked_=true;
        startup_spread_block_reason_="MAP_UNAVAILABLE";
        tracker_->resetMotion();
        publish({{},current_,yaw_current_,0,false,false},
          "STARTUP_SPREAD_WAIT_MAP");
        return;
      }
      const double remaining=horizontalNorm(startup_spread_target_-current_);
      if(remaining<=startup_spread_tolerance_ &&
         std::abs(current_.z-startup_spread_target_.z)<=.15 &&
         horizontalNorm(velocity_)<.12){
        startup_spread_complete_=true;startup_spread_blocked_=false;
        startup_spread_block_reason_="COMPLETE";
        state_=State::NAV;holding_=false;tracker_->resetMotion();
        tracker_->initializeYaw(yaw_current_);
        tracker_->setPath(have_path_?pending_path_:std::vector<Vec3>{},current_);
        publish({{},current_,yaw_current_,0,true,false},
          "STARTUP_SPREAD_COMPLETE");
        return;
      }
      auto control=tracker_->directMove(current_,startup_spread_target_,velocity_,dt,
        startup_spread_speed_,yaw_current_);
      if(startupSpreadObstacle(control)){
        startup_spread_blocked_=true;tracker_->resetMotion();
        publish({{},current_,yaw_current_,0,false,false},
          "STARTUP_SPREAD_OBSTACLE_HOLD");
        return;
      }
      startup_spread_blocked_=false;holding_=false;
      publish(control,control.heading_wait?"STARTUP_SPREAD_TURN":"STARTUP_SPREAD_MOVE");
      return;
    }
    if(!have_map_ || (now()-map_stamp_).seconds()>map_timeout_ || (now()-map_stamp_).seconds()<-.05){
      hold(dt,"WAIT_MAP_HOLD");return;
    }
    if(altitude_return_active_ || !grid_->insideAltitude(current_)){
      if(!altitude_return_active_){
        const double reserve=std::min(.15,.25*(grid_->cfg.z_max-grid_->cfg.z_min));
        altitude_return_target_={current_.x,current_.y,std::clamp(current_.z,grid_->cfg.z_min+reserve,grid_->cfg.z_max-reserve)};
        altitude_return_active_=true;tracker_->resetMotion();tracker_->initializeYaw(yaw_current_);
        if(recovery_.active())campaign_.finish(now().seconds());
        recovery_.cancel();history_retreat_active_=false;observed_escape_active_=false;separating_active_=false;observed_escape_.clear();
      }
      Vec3 target{current_.x,current_.y,altitude_return_target_.z};
      if(grid_->insideAltitude(current_) && std::abs(current_.z-target.z)<.025 && std::abs(velocity_.z)<.05){
        altitude_return_active_=false;holding_=false;tracker_->resetMotion();
        replan_pub_->publish(textMessage("ALTITUDE_RETURN_FINISHED"));
      }else{
        Control u{{},target,tracker_->yaw(),0,false,false};
        if(collisionClear(current_,target)){
          u=tracker_->hold(current_,target,velocity_,dt);u.velocity.x=u.velocity.y=0;
          u.velocity.z=std::clamp(u.velocity.z,-.12,.12);
          // A boundary return can never deepen the altitude violation.
          if(u.velocity.z*(target.z-current_.z)<0)u.velocity.z=0;
          publish(u,"ALTITUDE_RETURN");
        }else{
          tracker_->resetMotion();publish(u,"ALTITUDE_RETURN_BLOCKED");
        }
        return;
      }
    }
    const bool map_envelope_conflict=!grid_->recoveryClear(current_);
    const bool sonar_envelope_conflict=!sonar_collision_->recoveryClear(current_);
    const bool envelope_conflict=map_envelope_conflict||sonar_envelope_conflict;
    if((envelope_conflict||separating_active_||separation_turn_.active())&&runEnvelopeSeparation(dt))return;
    if(map_envelope_conflict||sonar_envelope_conflict){
      const char *reason=map_envelope_conflict&&sonar_envelope_conflict?
        "MAP_AND_SONAR_ENVELOPE_CONFLICT_HOLD":map_envelope_conflict?
        "MAP_ENVELOPE_CONFLICT_HOLD":"SONAR_ENVELOPE_CONFLICT_HOLD";
      prepareHold(reason);
      hold_anchor_=current_;
      tracker_->resetMotion();
      publish({{},current_,tracker_->yaw(),0,false,false},reason);
      return;
    }
    if(goal_geometry_blocked_){
      if(recovery_.active()){recovery_.cancel();endRecovery();}
      hold(dt,"GOAL_UNREACHABLE_HOLD");return;
    }
    synchronizeRoute();
    if(route_channel_.inputPaused(now().seconds(),steadySeconds(),planner_execution_timeout_)){
      hold(dt,"PLANNER_INPUT_PAUSED");return;
    }
    updatePassageAdmission();
    const bool missing_path=!have_path_ || !plannerAllowed();
    if(portal_waiting_ && passageSpeedLimit(tracker_->cfg.max_speed_xy)<.03){
      hold(dt,"PASSAGE_LEASE_WAIT");return;
    }
    if(runRecovery(dt,missing_path||recovery_requested_))return;
    if (missing_path) {
      hold(dt, autonomous_recovery_&&campaign_.exhausted()?"RECOVERY_BUDGET_EXHAUSTED":"WAIT_PATH_HOLD");
      return;
    }
    holding_ = false;
    recovery_.normalProgress(current_);campaign_.normalProgress(current_);separation_turn_.normalProgress(current_);
    selectHeadingMode();
    const auto target=tracker_->previewTarget(current_,velocity_);
    gap_state_=localGap(*grid_,current_,target-current_,tracker_->cfg.max_speed_xy,gap_config_);
    double adaptive_speed=gap_config_.enabled?std::max(gap_config_.min_speed,gap_state_.speed):tracker_->cfg.max_speed_xy;
    if(std::hypot(target.x-current_.x,target.y-current_.y)<.03 && std::abs(target.z-current_.z)>.05)
      adaptive_speed=tracker_->cfg.max_speed_z;
    map_speed_factor_=mapAgeSpeedFactor((now()-map_stamp_).seconds(),map_age_slow_start_,map_timeout_);
    adaptive_speed*=map_speed_factor_;
    auto clear = [this](Vec3 a, Vec3 b) { return collisionClear(a,b); };
    // Spending comfort clearance is permission for a controlled narrow pass,
    // not the wide-space gap speed. Look ahead early enough to decelerate.
    const double comfort=grid_->cfg.comfort_margin;
    if(comfort>0){
      const double horizon=std::max(.45,norm(velocity_)*1.5);
      for(double s=0;s<=horizon;s+=.10){
        const Vec3 q=s==0?current_:tracker_->futurePoint(current_,s);
        if(grid_->clearanceAt(q)<comfort+.04){adaptive_speed=std::min(adaptive_speed,tracker_->cfg.narrow_speed);break;}
      }
    }
    // Forward sonar range is NOT a global speed cap: the directional guard
    // below also permits certified movement away from the observed obstacle.
    auto u = tracker_->update(current_, velocity_, dt, clear, yaw_current_,
                              !grid_->segment(current_,current_)?recovery_speed_:adaptive_speed,
                              [this](Vec3 p){return grid_->clearanceAt(p);},heading_mode_,
                              [this](Vec3 a,Vec3 b){return observedMapMotion(a,b);});
    const bool partial_end=u.reached&&have_mission_goal_&&distance(current_,navigationGoal())>2*tracker_->cfg.goal_tolerance;
    publish(u, u.blocked ? "LOCAL_PATH_BLOCKED" : u.heading_wait ? "ROTATE_TO_PATH" :
      u.alignment_active ? "ALIGN_AND_ADVANCE" : partial_end ? "PARTIAL_PATH_END" :
      u.reached ? (have_approach_&&distance(approach_goal_,mission_goal_)>.01?
      "TARGET_APPROACH_HOLD":"TARGET_HOLD") : "FOLLOW_BSPLINE");
  }
  GeometricRouteChannel route_channel_;
  double planner_execution_timeout_{.8};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr execution_state_sub_;
  bool passage_scheduler_enabled_{false},portal_active_{false},portal_waiting_{false},passage_granted_{false};
  Vec3 portal_anchor_{},portal_direction_{};
  PassageProgress portal_progress_;PassageDepartureMemory portal_departure_;
  PassageWaitMonitor passage_wait_monitor_;
  int64_t portal_id_{0};double passage_reply_source_{-1};size_t passage_completions_{0};
  std::string passage_reply_state_{"IDLE"};
  double passage_wait_timeout_{8},passage_wait_retry_{4},passage_width_{2.6},passage_lookahead_{5};
  double passage_wait_distance_{2},passage_exit_distance_{2.2},passage_lease_timeout_{.8};
  double passage_grant_time_{-10},portal_epoch_{0},last_portal_request_{-10};
  double passage_detect_period_{.25},last_portal_detect_{-1};
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr passage_request_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr passage_grant_sub_;
  Px4CommandClock px4_clock_;TakeoffProgress takeoff_progress_;
  rclcpp::Clock px4_system_clock_{RCL_SYSTEM_TIME};
  PeerSafetyConfig peer_safety_config_;
  PeerSafetyDecision last_peer_decision_;
  PassageReservation last_passage_reservation_;
  PeerYieldWatchdog peer_yield_watchdog_;
  std::vector<PeerObservation>peer_observations_;
  int own_priority_{0};
  double peer_odom_timeout_{.80},peer_tf_timeout_{0};
  double peer_yield_replan_period_{1.0},last_peer_yield_request_{-1};
  bool peer_yield_active_{false},peer_yield_stalled_{false};
  bool peer_deadlock_recovery_{false},have_peer_blocker_position_{false};
  Vec3 peer_blocker_position_{};
  tf2_ros::Buffer tf_;
  tf2_ros::TransformListener listener_;
  int px4_nav_state_{-1},px4_status_system_id_{-1};bool have_landed_{false},px4_landed_{true};
  bool separation_enabled_{true},separating_active_{false};
  double separation_max_overlap_{.15},separation_speed_{.10};
  SeparationObservationTurn separation_turn_;RetreatRoute separation_history_;size_t separation_waypoint_{1};
  SeparationProof separation_map_,separation_sonar_;std::string separation_reason_{"IDLE"};
  rclcpp::Time last_sonar_debug_{0,0,RCL_ROS_TIME};
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sonar_debug_pub_;
  bool autonomous_recovery_{true},observed_escape_active_{false},have_approach_{false},goal_geometry_blocked_{false};
  double approach_limit_{.35};Vec3 approach_goal_{};size_t observed_waypoint_{1};
  RecoveryCampaign campaign_;ObservedEscapeConfig escape_config_;std::vector<Vec3>observed_escape_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr approach_sub_;
  DirectionalConfig direction_config_;
  GapPolicyConfig gap_config_;GapState gap_state_;
  Vec3 local_map_anchor_{};double local_map_extent_{6};
  LocalRecovery recovery_;bool recovery_requested_{false},have_mission_goal_{false};Vec3 mission_goal_{};
  RetreatRouteHistory retreat_history_;RetreatRoute retreat_route_;size_t retreat_waypoint_{1};
  bool historical_retreat_enabled_{true},history_retreat_active_{false},hold_braking_{false};
  double retreat_duration_{8},map_age_slow_start_{.6},map_speed_factor_{1};
  std::vector<Vec3>pending_path_;
  ObservationFallback observation_fallback_;
  StableRouteHandoff handoff_;bool handoff_ready_{false};double last_handoff_check_{-1};
  bool strict_after_unobserved_{false};Vec3 unobserved_probe_{};
  bool altitude_return_active_{false};Vec3 altitude_return_target_{};
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr mission_sub_,rviz_mission_sub_;
  NonForwardBudget nonforward_budget_;
  RecentCorridor trail_,frozen_trail_;
  HeadingMode heading_mode_{HeadingMode::Strict};
  std::unique_ptr<Grid>sonar_collision_,radar_match_;
  SonarLayer sonar_layer_;
  UnlocalizedSonarRisk unlocalized_sonar_;
  KeySet sonar_memory_;
  MapPairBuffer<sensor_msgs::msg::PointCloud2::SharedPtr>observation_pairs_;
  std::unique_ptr<Grid>observed_grid_;Vec3 observed_map_anchor_{};
  double radar_match_distance_{.25},radar_risk_timeout_{1.50};
  bool have_radar_only_{false};
  rclcpp::Time radar_only_stamp_{0,0,RCL_ROS_TIME};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr radar_only_sub_;
  int64_t occupied_stamp_{0},free_stamp_{-1};
  int64_t last_occupied_input_stamp_ns_{0},last_free_input_stamp_ns_{0};
  std::vector<RiskBox>glass_boxes_;
  std::string glass_topic_,direction_reason_{"INITIAL"};
  bool adaptive_direction_{true},glass_risk_active_{false},external_glass_risk_{false},have_glass_signal_{false};
  bool nonforward_session_{false},retreat_mode_{false},measured_braking_risk_{false};
  double glass_risk_distance_{1},glass_release_delay_{1},glass_signal_timeout_{1},forward_reset_progress_{0};
  rclcpp::Time glass_signal_stamp_{0,0,RCL_ROS_TIME},last_glass_risk_{0,0,RCL_ROS_TIME},last_direction_{0,0,RCL_ROS_TIME};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr free_sub_;
  bool compact_maps_{true};
  size_t snapshot_commits_{0},snapshot_rejected_{0},snapshot_bytes_{0};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr snapshot_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr glass_sub_;
  MotionProgressWatchdog progress_watchdog_;
  rclcpp::Time last_watchdog_{0,0,RCL_ROS_TIME},path_wait_since_{0,0,RCL_ROS_TIME},last_wait_request_{0,0,RCL_ROS_TIME};
  PoseHistory poses_;
  PoseSample current_pose_,sonar_pose_;
  std::deque<PendingSonar>sonar_pending_;
  SonarReading sonar_reading_;
  std::unique_ptr<SonarGuard>sonar_guard_;
  Vec3 sonar_offset_;
  double sonar_half_angle_,sonar_yaw_,last_sonar_source_{0},map_timeout_;
  double unlocalized_sonar_ttl_{2.0},sonar_stamp_reset_threshold_{.50},status_timeout_{3.0};
  double sonar_pose_tolerance_{.08},sonar_pose_wait_timeout_{.80};
  size_t sonar_pending_queue_size_{32},sonar_matched_{0},sonar_pose_fallbacks_{0},sonar_pose_drops_{0},sonar_epoch_resets_{0};
  bool sonar_inf_clear_;
  rclcpp::Time map_stamp_{0,0,RCL_ROS_TIME},last_diagnostic_{0,0,RCL_ROS_TIME};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostic_pub_,replan_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr sonar_reset_service_;
  std::string frame_, last_state_, hold_reason_;
  std::unique_ptr<Grid> grid_;
  std::unique_ptr<Tracker> tracker_;
  Vec3 current_, velocity_, hold_anchor_, takeoff_anchor_,startup_spread_target_;
  double yaw_current_{0}, odom_source_stamp_{0}, front_distance_{1e9};
  double recovery_speed_{.20};
  double takeoff_altitude_, prestream_seconds_, path_timeout_, odom_timeout_, stop_distance_,
      resume_distance_, ultrasonic_timeout_;
  double startup_spread_offset_x_{0},startup_spread_offset_y_{0};
  double startup_spread_spacing_{9.0},startup_spread_speed_{.80};
  double startup_spread_tolerance_{.25},startup_spread_obstacle_lookahead_{1.0};
  int startup_formation_rank_{0},startup_formation_count_{1};
  std::string startup_spread_block_reason_{"NOT_STARTED"};
  std::string px4_namespace_;
  int target_system_;
  bool startup_spread_enabled_{true},startup_spread_complete_{false},
      startup_spread_blocked_{false};
  bool auto_arm_, use_ultrasonic_, require_ultrasonic_, have_odom_{false}, have_path_{false},
      have_map_{false}, have_ultrasonic_{false}, have_status_{false}, armed_{false},
      offboard_{false}, holding_{false};
  State state_{State::WAIT};
  rclcpp::Time odom_stamp_{0, 0, RCL_ROS_TIME}, path_stamp_{0, 0, RCL_ROS_TIME},
      ultrasonic_stamp_{0, 0, RCL_ROS_TIME}, status_stamp_{0, 0, RCL_ROS_TIME},
      last_control_{0, 0, RCL_ROS_TIME}, state_since_{0, 0, RCL_ROS_TIME},
      last_request_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr>peer_odom_subs_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr ultrasonic_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr hb_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr sp_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafeAirFarPathFollower>());
  rclcpp::shutdown();
  return 0;
}
