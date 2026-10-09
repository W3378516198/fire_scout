#include "fire_scout/planner.hpp"
#include "fire_scout/entrance_routes.hpp"
#include "fire_scout/execution_permit.hpp"
#include "fire_scout/goal_approach.hpp"
#include "fire_scout/bspline.hpp"
#include "fire_scout/route_memory.hpp"
#include "fire_scout/route_switch.hpp"
#include "fire_scout/route_handoff.hpp"
#include "fire_scout/replan_schedule.hpp"
#include "fire_scout/profile_repair.hpp"
#include "fire_scout/turn_geometry.hpp"
#include "fire_scout/clearance_contract.hpp"
#include "fire_scout/map_pair_buffer.hpp"
#include "fire_scout/racer_task_allocator.hpp"
#include "fire_scout/ros_utils.hpp"
#include "fire_scout/map_input.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <sstream>
#include <atomic>
#include <future>
#include <limits>
#include <std_srvs/srv/trigger.hpp>
using namespace fire_scout;

namespace {

double stableRouteCost(const Grid &grid, const std::vector<Vec3> &route) {
  return straightRouteCost(grid,route);
}

SplineResult smoothPlannedRoute(
    const Grid &grid, const PlanResult &route, const SplineConfig &config,
    bool allow_unknown, const std::function<bool()> &cancelled,
    const RouteHandoffConfig &handoff = {}, double protected_prefix = 0) {
  if (!route.locally_patched) {
    auto result=smoothPath(grid, route.points, config, allow_unknown, cancelled);
    const double reserve=routeMargin(grid,route.points,config.clearance_reserve,allow_unknown);
    if(!result.points.empty()&&reserve>.01&&
       !routeHasMargin(grid,result.points,std::max(0.,reserve-.005),allow_unknown)){
      result.points=route.points;result.mode="CLEARANCE_RAW_FALLBACK";
    }
    return result;
  }
  SplineResult result;
  if (route.points.size() < 2 || route.patch_begin >= route.points.size() ||
      route.patch_end >= route.points.size() ||
      route.patch_end <= route.patch_begin)
    return result;
  std::vector<Vec3> patch(route.points.begin() + route.patch_begin,
                          route.points.begin() + route.patch_end + 1);
  auto smooth_patch =
      smoothPath(grid, patch, config, allow_unknown, cancelled);
  const auto &selected = smooth_patch.points.empty() ? patch : smooth_patch.points;
  std::vector<Vec3> combined;
  combined.reserve(route.points.size() + selected.size());
  for (size_t i = 0; i < route.patch_begin; ++i)
    combined.push_back(route.points[i]);
  for (const Vec3 point : selected)
    if (combined.empty() || distance(combined.back(), point) > 1e-6)
      combined.push_back(point);
  for (size_t i = route.patch_end + 1; i < route.points.size(); ++i)
    if (combined.empty() || distance(combined.back(), route.points[i]) > 1e-6)
      combined.push_back(route.points[i]);
  // Fitting the patch alone does not constrain its tangent to the old route.
  // Round BOTH seams inside bounded, collision-certified windows while keeping
  // the live commitment and the remainder of the old tail unchanged.
  combined=smoothRepairSeam(grid,combined,route.points[route.patch_begin],
    protected_prefix,handoff,allow_unknown,cancelled);
  combined=smoothRepairSeam(grid,combined,route.points[route.patch_end],
    protected_prefix,handoff,allow_unknown,cancelled);
  const double retained_margin=routeMargin(grid,route.points,config.clearance_reserve,allow_unknown);
  if (!routeGeometryValid(grid, combined, allow_unknown) ||
      (retained_margin>.01&&!routeHasMargin(grid,combined,std::max(0.,retained_margin-.005),allow_unknown))) {
    if (!routeGeometryValid(grid, route.points, allow_unknown)) return result;
    result.points = route.points;
    result.mode = "LOCAL_PATCH_RAW";
    return result;
  }
  result = std::move(smooth_patch);
  result.points = std::move(combined);
  result.mode = result.mode.empty() ? "LOCAL_PATCH_RAW"
                                    : "LOCAL_PATCH_" + result.mode;
  return result;
}

}  // namespace

class SafeAirFarLikePlanner : public rclcpp::Node {
public:
  SafeAirFarLikePlanner()
      : Node("safe_airfar_like_planner"), tf_(get_clock()), listener_(tf_) {
    declare_parameter<std::string>("runtime_version","2.1.10-incremental-detour");
    frame_ = declare_parameter<std::string>("frame_id", "scout1/odom");
    grid_ = std::make_shared<Grid>(gridParameters(*this));
    approach_config_.enabled=declare_parameter("allow_goal_approach",true);
    approach_config_.max_offset=declare_parameter("goal_approach_max_offset",.35);
    approach_config_.clearance=declare_parameter("goal_approach_clearance",.04);
    if(!approach_config_.valid())throw std::runtime_error("Invalid goal approach limits");
    search_snapshot_max_age_=declare_parameter("search_snapshot_max_age",1.0);
    if(!std::isfinite(search_snapshot_max_age_)||search_snapshot_max_age_<.25||search_snapshot_max_age_>30)
      throw std::runtime_error("search_snapshot_max_age must be .25..30 seconds");
    clearance_contract_.enabled=declare_parameter("clearance_review_enabled",true);
    clearance_contract_.reserve=declare_parameter("operational_clearance",.25);
    clearance_contract_.horizon=declare_parameter("clearance_review_horizon",6.0);
    clearance_contract_.review_period=declare_parameter("clearance_review_period",.30);
    clearance_contract_.retry_period=declare_parameter("clearance_repair_retry",2.0);
    if(!clearance_contract_.valid())throw std::runtime_error("Invalid clearance review limits");
    PlannerConfig c;
    c.allow_unknown = declare_parameter("allow_unknown", true);
    c.clearance_weight = declare_parameter("clearance_weight", 4.0);
    spline_.clearance_reserve=clearance_contract_.reserve;
    spline_.enabled = declare_parameter("use_bspline", true);
    spline_.control_spacing = declare_parameter("bspline_control_spacing", .40);
    spline_.sample_spacing = declare_parameter("bspline_sample_spacing", .06);
    spline_.attempts = declare_parameter("bspline_attempts", 4);
    spline_.max_control_spacing=declare_parameter("bspline_max_control_spacing",1.2);
    spline_.max_deviation=declare_parameter("bspline_max_deviation",.30);
    spline_.optimize=declare_parameter("bspline_optimize",true);
    spline_.jerk_weight=declare_parameter("bspline_jerk_weight",3.0);
    spline_.bending_weight=declare_parameter("bspline_bending_weight",.5);
    spline_.fitting_weight=declare_parameter("bspline_fitting_weight",1.0);
    spline_.max_control_offset=declare_parameter("bspline_max_control_offset",.20);
    spline_.optimization_budget_ms=declare_parameter("bspline_optimization_budget_ms",8.0);
    spline_.optimization_iterations=declare_parameter("bspline_optimization_iterations",96);
    spline_.feasibility_weight=declare_parameter("bspline_feasibility_weight",.20);
    spline_.nominal_speed=declare_parameter("bspline_nominal_speed",1.2);
    spline_.gap=gapParameters(*this);
    spline_.obstacle_weight=declare_parameter("bspline_obstacle_weight",12.0);
    if(!std::isfinite(spline_.obstacle_weight)||spline_.obstacle_weight<0||spline_.obstacle_weight>100)
      throw std::runtime_error("bspline_obstacle_weight must be in [0,100]");
    for(double v:{spline_.jerk_weight,spline_.bending_weight,spline_.fitting_weight,
                  spline_.max_control_offset,spline_.optimization_budget_ms})
      if(!std::isfinite(v)||v<0)
        throw std::runtime_error("B-spline optimization values must be finite and nonnegative");
    if(spline_.optimization_budget_ms>100.0 || spline_.optimization_iterations<0 ||
       spline_.optimization_iterations>1000)
      throw std::runtime_error("B-spline optimization requires budget 0..100 ms and iterations 0..1000");
    if(!std::isfinite(spline_.max_control_spacing)||!std::isfinite(spline_.max_deviation)||
       spline_.max_control_spacing<spline_.control_spacing||spline_.max_deviation<0)
      throw std::runtime_error("Invalid wide-curve parameters");
    if(c.clearance_weight<0 || spline_.control_spacing<=0 || spline_.sample_spacing<=0 || spline_.attempts<1 || spline_.attempts>6)
      throw std::runtime_error("Invalid clearance/B-spline parameters");
    c.unknown_cost = declare_parameter("unknown_cost", 2.0);
    c.vertical_weight = declare_parameter("vertical_weight", 1.5);
    c.altitude_weight = declare_parameter("altitude_weight", .25);
    c.altitude_boundary_weight=declare_parameter("altitude_boundary_weight",8.0);
    c.altitude_boundary_reserve=declare_parameter("altitude_boundary_reserve",.15);
    if(!std::isfinite(c.altitude_boundary_weight)||c.altitude_boundary_weight<0||
       !std::isfinite(c.altitude_boundary_reserve)||c.altitude_boundary_reserve<.05||c.altitude_boundary_reserve>.30)
      throw std::runtime_error("Invalid altitude boundary preference");
    c.enable_3d_search=declare_parameter("enable_3d_search",true);
    c.multi_altitude_slices=declare_parameter("multi_altitude_slices",true);
    c.flat_search_fraction=declare_parameter("flat_search_fraction",.45);
    c.vertical_bridge_step=declare_parameter("vertical_bridge_step",.30);
    c.vertical_bridge_max_change=declare_parameter("vertical_bridge_max_change",1.20);
    if(!std::isfinite(c.flat_search_fraction)||c.flat_search_fraction<.1||c.flat_search_fraction>.9||
       !std::isfinite(c.vertical_bridge_step)||c.vertical_bridge_step<.10||c.vertical_bridge_step>1.0||
       !std::isfinite(c.vertical_bridge_max_change)||c.vertical_bridge_max_change<0||c.vertical_bridge_max_change>3.0)
      throw std::runtime_error("Invalid 3-D search scheduling parameters");
    c.cruise_altitude = declare_parameter("cruise_altitude", 1.5);
    c.search_margin = declare_parameter("search_margin", 5.0);
    c.max_plan_distance = declare_parameter("max_plan_distance", 24.0);
    c.max_expansions = declare_parameter("max_astar_expansions", 100000);
    c.max_search_ms = declare_parameter("max_search_ms", 90.0);
    c.max_retry_search_ms=declare_parameter("max_retry_search_ms",360.0);
    c.max_search_margin=declare_parameter("max_search_margin",15.0);
    c.heuristic_weight=declare_parameter("heuristic_weight",1.5);
    c.coarse_detour_enabled=declare_parameter("coarse_detour_enabled",true);
    c.coarse_detour_resolution=declare_parameter("coarse_detour_resolution",.54);
    c.coarse_detour_ms=declare_parameter("coarse_detour_budget_ms",65.0);
    c.coarse_detour_min_distance=declare_parameter("coarse_detour_min_distance",6.0);
    if(!std::isfinite(c.coarse_detour_resolution)||c.coarse_detour_resolution<grid_->cfg.resolution||
       c.coarse_detour_resolution>1.0||!std::isfinite(c.coarse_detour_ms)||
       c.coarse_detour_ms<5||c.coarse_detour_ms>150||!std::isfinite(c.coarse_detour_min_distance)||
       c.coarse_detour_min_distance<2||c.coarse_detour_min_distance>20)
      throw std::runtime_error("Invalid coarse detour guide configuration");
    c.lock_valid_route=declare_parameter("lock_valid_route",true);
    if(!std::isfinite(c.max_retry_search_ms)||c.max_retry_search_ms<c.max_search_ms||
       !std::isfinite(c.max_search_margin)||c.max_search_margin<c.search_margin||
       !std::isfinite(c.heuristic_weight)||c.heuristic_weight<1||c.heuristic_weight>3)
      throw std::runtime_error("Invalid retry/heuristic parameters");
    c.replan_improvement_ratio = declare_parameter("replan_improvement_ratio", .18);
    c.replan_improvement_absolute = declare_parameter("replan_improvement_absolute", .45);
    if (c.unknown_cost < 0 || c.vertical_weight < 0 || c.altitude_weight < 0 ||
        c.max_search_ms <= 0 || c.max_expansions < 1 || c.search_margin <= 0 ||
        c.max_plan_distance < 1 || c.cruise_altitude < grid_->cfg.z_min ||
        c.cruise_altitude > grid_->cfg.z_max)
      throw std::runtime_error("Invalid planner parameters");
    for(double value:{c.clearance_weight,c.unknown_cost,c.vertical_weight,c.altitude_weight,c.cruise_altitude,
                       c.search_margin,c.max_plan_distance,c.max_search_ms,c.replan_improvement_ratio,
                       c.replan_improvement_absolute,spline_.control_spacing,spline_.sample_spacing})
      if(!std::isfinite(value))throw std::runtime_error("Non-finite planner parameter");
    c.reverse_weight=declare_parameter("reverse_weight",3.0);
    c.reverse_distance=declare_parameter("reverse_distance",3.0);
    c.local_repair_enabled=declare_parameter("local_repair_enabled",true);
    c.repair_pre_margin=declare_parameter("local_repair_pre_margin",.45);
    c.repair_post_margin=declare_parameter("local_repair_post_margin",.75);
    c.repair_max_span=declare_parameter("local_repair_max_span",6.0);
    c.repair_search_margin=declare_parameter("local_repair_search_margin",2.5);
    c.repair_search_ms=declare_parameter("local_repair_search_ms",140.0);
    c.repair_max_detour_ratio=declare_parameter("local_repair_max_detour_ratio",2.5);
    for(double v:{c.reverse_weight,c.reverse_distance,c.repair_pre_margin,
                  c.repair_post_margin,c.repair_max_span,c.repair_search_margin,
                  c.repair_search_ms,c.repair_max_detour_ratio})
      if(!std::isfinite(v)||v<0)throw std::runtime_error("Invalid route commitment parameter");
    if(c.repair_max_span<1.0||c.repair_search_margin<.5||
       c.repair_search_ms<20||c.repair_search_ms>c.max_retry_search_ms||
       c.repair_max_detour_ratio<1.0)
      throw std::runtime_error("Invalid local route repair parameters");
    planner_config_=c;
    planner_ = std::make_unique<Planner>(c);
    entrance_.count=declare_parameter("candidate_path_count",3);
    entrance_.budget_ms=declare_parameter("candidate_budget_ms",300.0);
    auto &topology=entrance_.topology;
    topology.enabled=declare_parameter("topology_enabled",true);
    topology.passage_width=declare_parameter("topology_passage_width",3.4);
    topology.probe_distance=declare_parameter("topology_probe_distance",2.5);
    topology.merge_distance=declare_parameter("topology_merge_distance",1.2);
    topology.gate_weight=declare_parameter("topology_shared_passage_weight",45.0);
    topology.cut_thickness=declare_parameter("topology_cut_thickness",.6);
    topology.commit_distance=declare_parameter("topology_commit_distance",2.0);
    topology.max_detour_ratio=declare_parameter("topology_max_detour_ratio",1.8);
    topology.max_extra_length=declare_parameter("topology_max_extra_length",16.0);
    topology.refresh_period=declare_parameter("topology_refresh_period",1.5);
    topology.search_resolution=declare_parameter("topology_search_resolution",.30);
    topology.prefer_distinct=declare_parameter("topology_prefer_distinct_passages",true);
    topology.max_gates=declare_parameter("topology_max_gates",8);
    if(!entrance_.valid())throw std::runtime_error("Invalid entrance candidate parameters");
    // Ordinary corridors retain the base route; alternatives require a shared entrance.
    peer_path_timeout_ = declare_parameter("peer_path_timeout", 3.0);
    peer_odom_timeout_ = declare_parameter("peer_odom_timeout", 1.5);
    peer_tf_timeout_ = declare_parameter("peer_tf_wait_timeout", 0.0);
    peer_replan_min_period_ =
        declare_parameter("peer_replan_min_period", 3.0);
    peer_route_separation_ = declare_parameter("peer_route_separation", 1.20);
    peer_nominal_speed_ = declare_parameter("peer_nominal_speed", 1.20);
    peer_time_step_ = declare_parameter("peer_time_step", .30);
    peer_conflict_horizon_ = declare_parameter("peer_conflict_horizon", 6.0);
    peer_conflict_threshold_ = declare_parameter("peer_conflict_threshold", .12);
    peer_route_penalty_weight_ =
        declare_parameter("peer_route_penalty_weight", 0.0);
    immediate_repair_distance_ =
        declare_parameter("route_immediate_repair_distance", 1.50);
    near_invalidation_confirm_time_ =
        declare_parameter("route_near_invalidation_confirm_time", .20);
    near_invalidation_confirm_updates_ = static_cast<size_t>(
        declare_parameter<int64_t>("route_near_invalidation_confirm_updates", 2));
    invalidation_confirm_time_ =
        declare_parameter("route_invalidation_confirm_time", .60);
    invalidation_confirm_updates_ = static_cast<size_t>(
        declare_parameter<int64_t>("route_invalidation_confirm_updates", 2));
    route_switch_.enabled =
        declare_parameter("route_switch_hysteresis_enabled", true);
    route_switch_.lookahead =
        declare_parameter("route_switch_lookahead", .90);
    route_switch_.backtrack_horizon =
        declare_parameter("route_switch_backtrack_horizon", 1.50);
    route_switch_.soft_angle = declare_parameter(
        "route_switch_angle_soft_deg", 20.0) * pi / 180.0;
    route_switch_.medium_angle = declare_parameter(
        "route_switch_angle_medium_deg", 45.0) * pi / 180.0;
    route_switch_.large_angle = declare_parameter(
        "route_switch_angle_large_deg", 90.0) * pi / 180.0;
    route_switch_.reverse_angle = declare_parameter(
        "route_switch_angle_reverse_deg", 120.0) * pi / 180.0;
    route_switch_.angle_weight =
        declare_parameter("route_switch_angle_weight", 2.50);
    route_switch_.backtrack_weight =
        declare_parameter("route_switch_backtrack_weight", 4.00);
    route_switch_.minimum_absolute_gain =
        declare_parameter("route_switch_min_absolute_gain", .15);
    route_switch_.minimum_gain_ratio =
        declare_parameter("route_switch_min_gain_ratio", .05);
    route_switch_.medium_gain_ratio =
        declare_parameter("route_switch_medium_gain_ratio", .12);
    route_switch_.large_gain_ratio =
        declare_parameter("route_switch_large_gain_ratio", .25);
    route_switch_.reverse_gain_ratio =
        declare_parameter("route_switch_reverse_gain_ratio", .40);
    route_switch_.confirmation_time =
        declare_parameter("route_switch_confirm_time", .75);
    route_switch_.cooldown =
        declare_parameter("route_switch_cooldown", 2.0);
    route_switch_.candidate_heading_tolerance = declare_parameter(
        "route_switch_candidate_heading_tolerance_deg", 18.0) * pi / 180.0;
    route_switch_.candidate_endpoint_tolerance = declare_parameter(
        "route_switch_candidate_endpoint_tolerance", .40);
    if (!std::isfinite(peer_path_timeout_) || peer_path_timeout_ <= 0 ||
        !std::isfinite(peer_odom_timeout_) || peer_odom_timeout_ <= 0 ||
        !std::isfinite(peer_tf_timeout_) || peer_tf_timeout_ < 0 ||
        peer_tf_timeout_ > .05 ||
        !std::isfinite(peer_replan_min_period_) || peer_replan_min_period_ < 0 ||
        !std::isfinite(peer_route_separation_) || peer_route_separation_ <= 0 ||
        !std::isfinite(peer_nominal_speed_) || peer_nominal_speed_ <= 0 ||
        !std::isfinite(peer_time_step_) || peer_time_step_ <= 0 ||
        !std::isfinite(peer_conflict_horizon_) || peer_conflict_horizon_ <= 0 ||
        !std::isfinite(peer_conflict_threshold_) || peer_conflict_threshold_ < 0 ||
        !std::isfinite(peer_route_penalty_weight_) || peer_route_penalty_weight_ < 0 ||
        !std::isfinite(immediate_repair_distance_) || immediate_repair_distance_ < .5 ||
        !std::isfinite(near_invalidation_confirm_time_) ||
        near_invalidation_confirm_time_ < 0 ||
        near_invalidation_confirm_updates_ < 1 ||
        near_invalidation_confirm_updates_ > 5 ||
        !std::isfinite(invalidation_confirm_time_) || invalidation_confirm_time_ < 0 ||
        invalidation_confirm_updates_ < 1 || invalidation_confirm_updates_ > 10 ||
        !route_switch_.valid())
      throw std::runtime_error("Invalid stable-route planner parameters");
    map_timeout_ = declare_parameter("map_timeout", 5.0);
    odom_timeout_ = declare_parameter("odom_timeout", 1.2);
    timestamp_reset_threshold_=declare_parameter("timestamp_reset_threshold",.50);
    handoff_config_.enabled=declare_parameter("route_handoff_enabled",true);
    handoff_config_.min_prefix=declare_parameter("route_handoff_min_prefix",1.0);
    handoff_config_.max_prefix=declare_parameter("route_handoff_max_prefix",2.5);
    handoff_config_.time=declare_parameter("route_handoff_time",1.2);
    handoff_config_.blend_min=declare_parameter("route_handoff_blend_min",.60);
    handoff_config_.blend_max=declare_parameter("route_handoff_blend_max",2.40);
    handoff_config_.max_deviation=declare_parameter("route_handoff_max_deviation",.30);
    handoff_config_.sample_spacing=declare_parameter("route_handoff_sample_spacing",.05);
    handoff_config_.budget_ms=declare_parameter("route_handoff_budget_ms",12.0);
    if(!handoff_config_.valid())throw std::runtime_error("Invalid route handoff limits");
    profile_tracker_.lookahead=declare_parameter("lookahead",.65);
    profile_tracker_.max_speed_xy=declare_parameter("max_speed_xy",1.8);
    profile_tracker_.max_accel_xy=declare_parameter("max_accel_xy",1.5);
    profile_tracker_.max_jerk_xy=declare_parameter("max_jerk_xy",4.5);
    profile_tracker_.lateral_accel=declare_parameter("max_lateral_accel",1.0);
    profile_tracker_.yaw_rate=declare_parameter("max_yaw_rate",1.2);
    spline_.max_velocity=profile_tracker_.max_speed_xy;
    spline_.max_acceleration=profile_tracker_.max_accel_xy;
    spline_.max_jerk=profile_tracker_.max_jerk_xy;
    if(!std::isfinite(spline_.feasibility_weight)||spline_.feasibility_weight<0||spline_.feasibility_weight>10||
       !std::isfinite(spline_.nominal_speed)||spline_.nominal_speed<=0||spline_.nominal_speed>spline_.max_velocity)
      throw std::runtime_error("Invalid B-spline feasibility parameters");
    profile_tracker_.sharp_turn=declare_parameter("sharp_turn_stop_deg",80.0)*pi/180.;
    profile_tracker_.turn.enabled=declare_parameter("turn_policy_enabled",true);
    profile_tracker_.turn.max_curvature=declare_parameter("turn_max_curvature",1.6);
    profile_tracker_.turn.min_curve_speed=declare_parameter("turn_min_curve_speed",.35);
    profile_tracker_.turn.capture_distance=declare_parameter("turn_capture_distance",.10);
    profile_tracker_.turn.stop_speed=declare_parameter("turn_stop_speed",.10);
    profile_tracker_.turn.yaw_tolerance=declare_parameter("turn_yaw_tolerance_deg",8.0)*pi/180.;
    profile_tracker_.turn.repair_budget_ms=declare_parameter("turn_repair_budget_ms",16.0);
    if(!profile_tracker_.turn.valid())throw std::runtime_error("Invalid global turn policy");
    for(double v:{profile_tracker_.lookahead,profile_tracker_.max_speed_xy,
        profile_tracker_.max_accel_xy,profile_tracker_.max_jerk_xy,
        profile_tracker_.lateral_accel,profile_tracker_.yaw_rate,profile_tracker_.sharp_turn})
      if(!std::isfinite(v)||v<=0)throw std::runtime_error("Invalid shared speed-profile limits");
    replan_period_ = declare_parameter("replan_period", .10);
    double replan = replan_period_;
    double heartbeat = declare_parameter("path_publish_period", .10);
    double poll = declare_parameter("planner_poll_period", .01);
    if (!std::isfinite(replan)||!std::isfinite(heartbeat)||!std::isfinite(poll)||
        replan < .02 || heartbeat < .02 || poll < .005 || poll > .10 || map_timeout_ <= 0 || odom_timeout_ <= 0 ||
        !std::isfinite(timestamp_reset_threshold_)||timestamp_reset_threshold_<.10)
      throw std::runtime_error("Invalid planner timing");
    compact_maps_=compactMapTransport(*this);
    local_fallback_enabled_=declare_parameter("local_map_fallback_enabled",true);
    const auto primary_snapshot=declare_parameter<std::string>("map_snapshot_topic","/scout1/map/radar_snapshot");
    const auto local_snapshot=declare_parameter<std::string>("local_map_snapshot_topic","/scout1/map/radar_snapshot");
    const auto occupied_topic=declare_parameter<std::string>("occupied_topic","/scout1/map/radar_occupied");
    const auto free_topic=declare_parameter<std::string>("free_topic","/scout1/map/radar_free");
    auto mapq = rclcpp::QoS(compact_maps_?1:4).reliable().transient_local();
    if(compact_maps_){
      if(primary_snapshot.empty() || (local_fallback_enabled_&&local_snapshot.empty()) ||
         primary_snapshot==occupied_topic || primary_snapshot==free_topic)
        throw std::runtime_error("Invalid planner snapshot topic");
      snapshot_sub_=create_subscription<sensor_msgs::msg::PointCloud2>(primary_snapshot,mapq,
        [this](sensor_msgs::msg::PointCloud2::SharedPtr m){
          if(m->header.frame_id!=frame_||stampNs(m->header.stamp)<=0){++map_rejected_;return;}
          observeInputStamp(stampNs(m->header.stamp),"PRIMARY_SNAPSHOT_STAMP_REWIND",last_primary_input_stamp_ns_);
          ++primary_received_;
          if(!primary_pending_||stampNs(m->header.stamp)>stampNs(primary_pending_->header.stamp))
            primary_pending_=std::move(m);
        });
      if(local_fallback_enabled_&&local_snapshot!=primary_snapshot)
        local_snapshot_sub_=create_subscription<sensor_msgs::msg::PointCloud2>(local_snapshot,mapq,
          [this](sensor_msgs::msg::PointCloud2::SharedPtr m){
            if(m->header.frame_id!=frame_||stampNs(m->header.stamp)<=0){++map_rejected_;return;}
            observeInputStamp(stampNs(m->header.stamp),"LOCAL_SNAPSHOT_STAMP_REWIND",last_local_input_stamp_ns_);
            ++local_received_;
            if(!local_pending_||stampNs(m->header.stamp)>stampNs(local_pending_->header.stamp))
              local_pending_=std::move(m);
          });
    }else{
    occ_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        occupied_topic, mapq,
        [this](sensor_msgs::msg::PointCloud2::SharedPtr m) {
          if (m->header.frame_id == frame_) {
            const auto stamp=stampNs(m->header.stamp);
            observeInputStamp(stamp,"OCCUPIED_MAP_STAMP_REWIND",last_occupied_input_stamp_ns_);
            // Newly received obstacles take effect immediately, even while
            // their larger matching free cloud is still in the DDS queue.
            if(!occupied_guard_ || stamp>occupied_guard_stamp_){
              try {
                auto guard=std::make_shared<Grid>(grid_->cfg);
                guard->update(cloudSet(*m,grid_->cfg.resolution),{});
                occupied_guard_=std::move(guard);occupied_guard_stamp_=stamp;
                if(have_odom_ && !owned_path_.empty())
                  observeInvalidation(inspectOwnedRoute(), false,
                                      "OCCUPIED_GUARD");
              }catch(const std::runtime_error&e){
                RCLCPP_ERROR(get_logger(),"Occupied guard parse: %s",e.what());return;
              }
            }
            map_pairs_.putOccupied(stamp,m);
            commit();
          }
        });
    free_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        free_topic, mapq,
        [this](sensor_msgs::msg::PointCloud2::SharedPtr m) {
          if (m->header.frame_id == frame_) {
            const auto stamp=stampNs(m->header.stamp);
            observeInputStamp(stamp,"FREE_MAP_STAMP_REWIND",last_free_input_stamp_ns_);
            map_pairs_.putFree(stamp,m);
            commit();
          }
        });
    }
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        declare_parameter<std::string>("odom_topic", "/scout1/odom"), rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::SharedPtr m) {
          Vec3 p{m->pose.pose.position.x, m->pose.pose.position.y, m->pose.pose.position.z};
          if (m->header.frame_id != frame_ || !finite(p))
            return;
          const auto stamp=stampNs(m->header.stamp);
          observeInputStamp(stamp,"ODOMETRY_STAMP_REWIND",last_odom_input_stamp_ns_);
          current_ = p;
          const auto &v=m->twist.twist.linear;
          measured_speed_=finite({v.x,v.y,v.z})?std::min(10.,norm({v.x,v.y,v.z})):0.;
          if(planner_)planner_->advanceCommittedProgress(p);
          if(!owned_path_.empty())owned_route_.updateProgress(p);
          // The execution certificate must follow the same continuous
          // odometry progress as route ownership, even while geometry is kept.
          if(!path_.empty())route_.updateProgress(p);
          have_odom_ = true;
          odom_stamp_ = rclcpp::Time(m->header.stamp,get_clock()->get_clock_type());
        });
    auto goalcb = [this](geometry_msgs::msg::PoseStamped::SharedPtr m) {
      if (m->header.frame_id != frame_) {
        RCLCPP_ERROR(get_logger(), "Goal frame must be %s (got %s)", frame_.c_str(),
                     m->header.frame_id.c_str());
        return;
      }
      Vec3 p{m->pose.position.x, m->pose.position.y, m->pose.position.z};
      if (!finite(p))
        return;
      if (p.z < grid_->cfg.z_min || p.z > grid_->cfg.z_max) {
        RCLCPP_ERROR(get_logger(), "Goal z %.2f is outside planning bounds", p.z);
        return;
      }
      const double goal_delta = have_goal_ ? distance(goal_, p) : 0.0;
      if (!have_goal_ || goal_delta > .05) {
        // A new goal cancels old search ownership, but a short currently safe
        // forward prefix can continue under a permit bearing the NEW goal.
        auto prefix=ready()?retainedForwardPrefix(*grid_,path_,current_,route_.progress(),
          measured_speed_,handoff_config_,planner_config_.allow_unknown):std::vector<Vec3>{};
        if(occupied_guard_&&!prefix.empty()&&!routeGeometryValid(*occupied_guard_,prefix,true))prefix.clear();
        goal_ = p;approach_={};last_search_refresh_=std::chrono::steady_clock::now();last_refresh_generation_=generation_;
        have_goal_ = true;
        ++(*goal_epoch_);
        if(planner_)planner_->reset();
        path_=std::move(prefix); route_.set(path_);owned_path_.clear();owned_route_.clear();
        clearInvalidation(false);
        last_invalidation_source_="NONE";last_invalidation_point_={};
        last_invalidation_distance_=std::numeric_limits<double>::infinity();
        temporary_=retry_=recovery_route_=false;
        replan_trigger_.clear();clearPendingSwitch();last_route_switch_={};
        entrance_review_.clear();partial_retry_.clear();profile_repair_requested_=false;clearance_review_.clear();
        reason_="GOAL_ACCEPTED";detail_.clear();
        force_ = true;
        publish();
        RCLCPP_INFO(get_logger(), "New goal [%.2f %.2f %.2f] delta=%.3f",
                    p.x, p.y, p.z, goal_delta);
      }else if((path_.empty()||retry_) && (now()-last_goal_retry_).seconds()>.5){
        // The same mission is a wake-up, not cancellation of accumulated work.
        // Autonomous retries already use current observations. Only a genuinely
        // different goal changes the epoch and discards its search frontier.
        last_goal_retry_=now();
        force_=true;reason_="GOAL_RETRY_RETAINED";publish();
      }
    };
    auto gt = declare_parameter<std::string>("goal_topic", "/scout1/mission/fire_goal");
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(gt, 10, goalcb);
    auto rv = declare_parameter<std::string>("rviz_goal_topic", "/goal_pose");
    if (!rv.empty() && rv != gt)
      rviz_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(rv, 10, goalcb);
    path_topic_ =
        declare_parameter<std::string>("path_topic", "/scout1/planning/global_path");
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
        path_topic_, rclcpp::QoS(1).reliable());
    approach_pub_=create_publisher<nav_msgs::msg::Path>(
      declare_parameter<std::string>("goal_approach_topic","/scout1/planning/goal_approach"),
      rclcpp::QoS(1).reliable().transient_local());
    raw_path_pub_ = create_publisher<nav_msgs::msg::Path>(
        declare_parameter<std::string>("raw_path_topic", "/scout1/planning/astar_path"),10);
    const auto peer_topics = declare_parameter<std::vector<std::string>>(
        "peer_path_topics", std::vector<std::string>{});
    const auto peer_odom_topics = declare_parameter<std::vector<std::string>>(
        "peer_odom_topics", std::vector<std::string>{});
    if (peer_topics.size() != peer_odom_topics.size())
      throw std::runtime_error(
          "peer_path_topics and peer_odom_topics must have the same length");
    peer_routes_.resize(peer_topics.size());
    for (size_t i = 0; i < peer_topics.size(); ++i) {
      if (peer_topics[i].empty() || peer_odom_topics[i].empty() ||
          peer_topics[i] == path_topic_)
        throw std::runtime_error(
            "peer topics must be non-empty and exclude own path_topic");
      peer_path_subs_.push_back(create_subscription<nav_msgs::msg::Path>(
          peer_topics[i], rclcpp::QoS(5).reliable(),
          [this, i](nav_msgs::msg::Path::SharedPtr msg) {
            receivePeerPath(i, *msg);
          }));
      peer_odom_subs_.push_back(create_subscription<nav_msgs::msg::Odometry>(
          peer_odom_topics[i], rclcpp::SensorDataQoS(),
          [this, i](nav_msgs::msg::Odometry::SharedPtr msg) {
            receivePeerOdom(i, *msg);
          }));
    }
    execution_pub_=create_publisher<std_msgs::msg::String>(
      declare_parameter<std::string>("execution_state_topic","/scout1/planning/execution_state"),
      rclcpp::QoS(1).reliable().transient_local());
    status_pub_ = create_publisher<std_msgs::msg::String>(declare_parameter<std::string>("planner_status_topic","/scout1/planning/status"),
                                                          rclcpp::QoS(1).reliable());
    replan_sub_=create_subscription<std_msgs::msg::String>(declare_parameter<std::string>("replan_request_topic","/scout1/planning/replan_request"),10,
      [this](std_msgs::msg::String::SharedPtr m){
        if(m->data=="LOCAL_PATH_BLOCKED"){
          // This is a request to re-check geometry, not permission to replace a
          // map-valid route. plan() only repairs after the current map agrees.
          follower_block_reported_=true;force_=true;
        }else if(m->data=="TURN_EXECUTION_REPAIR"){
          profile_repair_requested_=true;replan_trigger_="TURN_EXECUTION_REPAIR";force_=true;
        }else if(m->data=="PROFILE_LIMITED_PROGRESS"){
          profile_repair_requested_=true;force_=true;
        }else if(m->data=="STALLED_PROGRESS"){
          follower_stall_reports_++;
          reason_="FOLLOWER_STALL_ROUTE_RETAINED";
          detail_="planner geometry unchanged; follower recovery remains responsible";
        }else if(m->data=="WAITING_FOR_EXECUTABLE_PATH" || m->data=="PASSAGE_WAIT_TIMEOUT"){
          // Wake a bounded retry/review without erasing the route or A* frontier.
          force_=true;last_topology_review_={};entrance_review_.clear();
        }else if(m->data=="LOCAL_RECOVERY_FINISHED"){
          reason_="RECOVERY_FINISHED_ROUTE_RETAINED";
        }
      });
    // The executor keeps accepting goals/odometry while bounded search runs on a
    // private snapshot. Poll completion promptly instead of blocking subscriptions.
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(poll)), [this] { plan(); });
    heartbeat_ =
        create_wall_timer(std::chrono::milliseconds(int(1000 * heartbeat)), [this] { publish(); });
    reset_ = create_service<std_srvs::srv::Trigger>(
        "~/reset", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          ++(*goal_epoch_);
          if(planner_)planner_->reset();
          path_.clear(); route_.clear();owned_path_.clear();owned_route_.clear();
          entrance_review_.clear();partial_retry_.clear();profile_repair_requested_=false;clearance_review_.clear();
          clearInvalidation(false);
          last_invalidation_source_="NONE";last_invalidation_point_={};
          last_invalidation_distance_=std::numeric_limits<double>::infinity();
          temporary_=retry_=recovery_route_=false;
          replan_trigger_.clear();clearPendingSwitch();last_route_switch_={};
          reason_="ROUTE_RESET";detail_.clear();force_=true;
          publish();
          res->success = true;
          res->message = "Route reset; current goal retained.";
        });
    config_lock_=lockParameters(*this);
    RCLCPP_INFO(get_logger(),
                "V2.1.6 near-clearance planner: safe forward commitment, bounded spline handoff, paired-map invalidation, "
                "priority_peers=%zu switch_hysteresis=%s unknown=%s",
                peer_topics.size(),
                route_switch_.enabled ? "true" : "false",
                c.allow_unknown ? "penalized (simulation)" : "blocked");
  }

private:
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr config_lock_;
  void resetTimeEpoch(const char *reason){
    ++(*goal_epoch_);if(planner_)planner_->reset();
    path_.clear();route_.clear();owned_path_.clear();owned_route_.clear();
    clearInvalidation(false);clearPendingSwitch();last_route_switch_={};
    entrance_review_.clear();partial_retry_.clear();profile_repair_requested_=false;clearance_review_.clear();
    last_invalidation_source_="NONE";last_invalidation_point_={};
    last_invalidation_distance_=std::numeric_limits<double>::infinity();
    temporary_=retry_=recovery_route_=repair_authorized_=false;replan_trigger_.clear();
    map_pairs_.clear();occupied_guard_.reset();occupied_guard_stamp_=-1;
    primary_pending_.reset();local_pending_.reset();primary_grid_.reset();
    local_overlay_grid_.reset();local_overlay_.clear();
    local_overlay_stamp_ns_=last_compact_confirmation_stamp_ns_=-1;
    primary_stamp_ns_=local_committed_stamp_ns_=-1;
    last_primary_input_stamp_ns_=last_local_input_stamp_ns_=0;
    map_source_=MapInputSource::None;
    grid_=std::make_shared<Grid>(grid_->cfg);have_map_=have_odom_=false;
    map_stamp_=odom_stamp_=rclcpp::Time(0,0,get_clock()->get_clock_type());
    last_occupied_input_stamp_ns_=last_free_input_stamp_ns_=last_odom_input_stamp_ns_=0;
    for(auto &peer:peer_routes_)peer=PeerRoute{};
    ++generation_;force_=true;reason_="TIME_EPOCH_RESET";detail_=reason;
    RCLCPP_WARN(get_logger(),"PLANNER_TIME_EPOCH_RESET reason=%s",reason);
  }
  void observeInputStamp(int64_t stamp,const char *reason,int64_t &latest){
    if(stamp<=0)return;
    const int64_t threshold=int64_t(timestamp_reset_threshold_*1e9);
    if(latest>0&&stamp<latest-threshold)resetTimeEpoch(reason);
    latest=std::max(latest,stamp);
  }
  struct PeerRoute {
    std::vector<Vec3> points;
    std::vector<double> arc;
    int64_t stamp_ns{0};
    Vec3 position{};
    int64_t odom_ns{0};
    double progress{0};
    bool have_position{false},have_progress{false},progress_valid{false};

    void updateProgress() {
      progress_valid=false;
      if(!have_position || points.size()<2 || arc.empty())return;
      auto projection=project(points,arc,position,
        have_progress?std::max(0.,progress-.10):0.,
        std::min(arc.back(),have_progress?progress+2.:2.));
      if(!have_progress && projection.error>1.2){
        // Late subscribers can first observe a peer far along a full route.
        // Initialize globally only when no other distant branch is similarly
        // close. Once initialized, odometry can NEVER jump across a crossing.
        projection=project(points,arc,position);
        if(!std::isfinite(projection.error)||projection.error>1.2)return;
        for(size_t i=1;i<points.size();++i){
          const Vec3 segment=points[i]-points[i-1];
          const double length=arc[i]-arc[i-1];
          if(length<1e-8)continue;
          const double u=std::clamp(dot(position-points[i-1],segment)/(length*length),0.,1.);
          const double s=arc[i-1]+u*length;
          if(std::abs(s-projection.s)>2. &&
             distance(position,points[i-1]+segment*u)<=projection.error+.15)return;
        }
      }
      if(!std::isfinite(projection.error)||projection.error>1.2)return;
      progress=have_progress?std::max(progress,projection.s):projection.s;
      have_progress=progress_valid=true;
    }
    void setPoints(std::vector<Vec3> next) {
      bool identical=points.size()==next.size();
      if(identical)for(size_t i=0;i<points.size();++i)
        if(distance(points[i],next[i])>1e-6){identical=false;break;}
      if(identical){updateProgress();return;}
      auto next_arc=arcLengths(next);
      bool same_prefix=have_progress && !arc.empty() && !next_arc.empty() &&
        next_arc.back()>=progress && distance(points.front(),next.front())<1e-6;
      const double through=std::min(arc.empty()?0.:arc.back(),progress+.10);
      if(same_prefix){
        // Exact retained prefixes keep their arclength origin even when the
        // future tail changes. Check vertices from both polylines so a new
        // shortcut cannot masquerade as the old route at sparse samples.
        for(size_t i=0;i<points.size() && arc[i]<=through;++i)
          if(distance(points[i],atArc(next,next_arc,arc[i]))>1e-5){same_prefix=false;break;}
        for(size_t i=0;same_prefix && i<next.size() && next_arc[i]<=through;++i)
          if(distance(next[i],atArc(points,arc,next_arc[i]))>1e-5){same_prefix=false;break;}
        if(same_prefix && distance(atArc(points,arc,progress),atArc(next,next_arc,progress))>1e-5)
          same_prefix=false;
      }
      points=std::move(next);arc=std::move(next_arc);
      if(!same_prefix){progress=0;have_progress=false;}
      updateProgress();
    }
  };
  static tf2::Transform toTransform(
      const geometry_msgs::msg::TransformStamped &msg) {
    const auto &q = msg.transform.rotation;
    tf2::Quaternion rotation(q.x, q.y, q.z, q.w);
    if (rotation.length2() < 1e-12)
      throw std::runtime_error("Invalid peer-path TF quaternion");
    rotation.normalize();
    const auto &p = msg.transform.translation;
    return tf2::Transform(rotation, tf2::Vector3(p.x, p.y, p.z));
  }
  bool peerRouteChanged(const std::vector<Vec3> &old_route,
                        const std::vector<Vec3> &new_route) const {
    if (old_route.empty() != new_route.empty()) return true;
    if (old_route.empty()) return false;
    if (distance(old_route.front(), new_route.front()) > .20 ||
        distance(old_route.back(), new_route.back()) > .20 ||
        std::abs(pathLength(old_route) - pathLength(new_route)) > .30)
      return true;
    return routesDistinct(old_route, new_route, .12, .50);
  }
  void receivePeerPath(size_t index, const nav_msgs::msg::Path &msg) {
    if (index >= peer_routes_.size()) return;
    std::vector<Vec3> transformed;
    if (!msg.poses.empty() && !msg.header.frame_id.empty()) {
      try {
        tf2::Transform transform;
        if (msg.header.frame_id == frame_)
          transform.setIdentity();
        else
          transform = toTransform(tf_.lookupTransform(
              frame_, msg.header.frame_id,
              rclcpp::Time(0,0,get_clock()->get_clock_type()),
              rclcpp::Duration::from_seconds(peer_tf_timeout_)));
        transformed.reserve(msg.poses.size());
        for (const auto &pose : msg.poses) {
          const auto &p = pose.pose.position;
          const Vec3 local{p.x, p.y, p.z};
          if (!finite(local)) continue;
          const auto point = transform * tf2::Vector3(local.x, local.y, local.z);
          const Vec3 converted{point.x(), point.y(), point.z()};
          if (finite(converted)) transformed.push_back(converted);
        }
      } catch (const tf2::TransformException &error) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                             "Waiting for peer path TF %s <- %s: %s",
                             frame_.c_str(), msg.header.frame_id.c_str(),
                             error.what());
        return;
      } catch (const std::runtime_error &error) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                             "Rejected peer path: %s", error.what());
        return;
      }
    }
    auto &peer = peer_routes_[index];
    const bool changed = peerRouteChanged(peer.points, transformed);
    peer.setPoints(std::move(transformed));
    peer.stamp_ns = now().nanoseconds();
    if (changed) ++peer_generation_;
  }
  void receivePeerOdom(size_t index, const nav_msgs::msg::Odometry &msg) {
    if (index >= peer_routes_.size() || msg.header.frame_id.empty()) return;
    const Vec3 local{msg.pose.pose.position.x, msg.pose.pose.position.y,
                     msg.pose.pose.position.z};
    if (!finite(local)) return;
    try {
      tf2::Transform transform;
      if (msg.header.frame_id == frame_)
        transform.setIdentity();
      else
        transform = toTransform(tf_.lookupTransform(
            frame_, msg.header.frame_id,
            rclcpp::Time(msg.header.stamp, get_clock()->get_clock_type()),
            rclcpp::Duration::from_seconds(peer_tf_timeout_)));
      const auto point = transform * tf2::Vector3(local.x, local.y, local.z);
      auto &peer = peer_routes_[index];
      peer.position = {point.x(), point.y(), point.z()};
      peer.odom_ns = now().nanoseconds();
      peer.have_position = finite(peer.position);
      peer.updateProgress();
    } catch (const tf2::TransformException &error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                           "Waiting for peer odometry TF %s <- %s: %s",
                           frame_.c_str(), msg.header.frame_id.c_str(),
                           error.what());
    }
  }
  std::vector<std::vector<Vec3>> freshPeerRoutes(bool full=false) const {
    std::vector<std::vector<Vec3>> routes;
    const int64_t now_ns = now().nanoseconds();
    for (const auto &peer : peer_routes_)
      if (!peer.points.empty() && peer.stamp_ns > 0 && now_ns >= peer.stamp_ns &&
          double(now_ns - peer.stamp_ns) * 1e-9 <= peer_path_timeout_ &&
          peer.have_position && peer.progress_valid && peer.odom_ns > 0 && now_ns >= peer.odom_ns &&
          double(now_ns - peer.odom_ns) * 1e-9 <= peer_odom_timeout_) {
        auto suffix = routeSuffixAt(peer.points, peer.position, peer.progress);
        if (suffix.size() >= 2) {
          const auto arc = arcLengths(suffix);
          routes.push_back(routeSlice(
              suffix, 0, std::min(arc.back(), full?60.:peer_conflict_horizon_)));
        }
      }
    return routes;
  }
  bool conflictsWithPeers(const std::vector<Vec3> &route,
                          const std::vector<std::vector<Vec3>> &peers) const {
    if (route.size() < 2 || peers.empty()) return false;
    auto own = routeSuffixAt(route, current_, owned_route_.progress());
    if (own.size() < 2) return false;
    const auto arc = arcLengths(own);
    own = routeSlice(own, 0, std::min(arc.back(), peer_conflict_horizon_));
    // Static overlap alone is not a reason to abandon a route.  Only a
    // near-horizon time conflict can unlock peer avoidance.
    return routeTimeConflictLoss(own, peers, peer_route_separation_,
                                 peer_nominal_speed_, peer_time_step_) >
           peer_conflict_threshold_;
  }
  bool guardValid(RouteMemory route)const{
    return !occupied_guard_ || route.valid(*occupied_guard_,current_,true);
  }
  struct InvalidationGate {
    bool active{false};
    bool authorized{false};
    Vec3 point{};
    double distance{std::numeric_limits<double>::infinity()};
    size_t matched_updates{0};
    std::string source{"NONE"};
    std::chrono::steady_clock::time_point since{};
  };
  RouteInspection inspectOwnedRoute() const {
    auto selected = inspectRoute(*grid_, owned_path_, current_,
                                 planner_config_.allow_unknown,
                                 owned_route_.progress());
    if (occupied_guard_ && !owned_path_.empty()) {
      const auto guard = inspectRoute(*occupied_guard_, owned_path_, current_,
                                      true, owned_route_.progress());
      if (!guard.valid &&
          (selected.valid || guard.first_invalid_distance <
                                 selected.first_invalid_distance))
        selected = guard;
    }
    return selected;
  }
  void restoreOwnedRoute() {
    if (owned_path_.empty()) return;
    // Rejection is not a route change. Preserve the live arc origin, geometry
    // and follower profile while the complete owned tail is already executing.
    if(!path_.empty() && distance(path_.back(),owned_path_.back())<1e-6 &&
       route_.valid(*grid_,current_,planner_config_.allow_unknown) && guardValid(route_)) {
      temporary_=distance(path_.back(),approach_.valid?approach_.effective:goal_)>.15;
      return;
    }
    if(!owned_route_.updateProgress(current_))return;
    // Only an actually clipped/lost tail needs reinstating. Never prepend the
    // measured position: cross-track control handles that bounded displacement.
    auto suffix=routeSlice(owned_path_,std::max(0.,owned_route_.progress()-.25),
                           pathLength(owned_path_));
    if(suffix.size()<2)return;
    path_=std::move(suffix);route_.set(path_);route_.updateProgress(current_);
    temporary_=distance(path_.back(),approach_.valid?approach_.effective:goal_)>.15;
  }
  void clearInvalidation(bool restore) {
    const bool was_authorized = invalidation_.authorized;
    invalidation_ = InvalidationGate{};
    repair_authorized_ = false;
    // Execution can now trim a distant unsafe tail BEFORE the multi-frame
    // repair gate authorizes a search. Restore it if that transient evidence
    // disappears; otherwise the drone would stop at an obsolete prefix end.
    if (restore && !owned_path_.empty() &&
        (was_authorized || path_.empty() || distance(path_.back(),owned_path_.back())>.03))
      restoreOwnedRoute();
  }
  void observeInvalidation(const RouteInspection &inspection, bool matched,
                           const std::string &source) {
    if (inspection.valid) {
      clearInvalidation(true);
      return;
    }
    const auto wall = std::chrono::steady_clock::now();
    const bool same = invalidation_.active &&
        distance(invalidation_.point, inspection.first_invalid_point) <= .60;
    if (!same) {
      invalidation_ = InvalidationGate{};
      invalidation_.active = true;
      invalidation_.point = inspection.first_invalid_point;
      invalidation_.since = wall;
    }
    invalidation_.distance = inspection.first_invalid_distance;
    invalidation_.source = source;
    last_invalidation_source_=source;
    last_invalidation_point_=inspection.first_invalid_point;
    last_invalidation_distance_=inspection.first_invalid_distance;
    if (matched) ++invalidation_.matched_updates;
    const double age=std::chrono::duration<double>(wall-invalidation_.since).count();
    const bool near = inspection.first_invalid_distance <=
                      immediate_repair_distance_;
    // An occupied-only callback stops the follower immediately through its
    // local safety map, but it cannot rewrite the committed global route.  A
    // near obstacle must first survive the paired occupied/free map, which
    // rejects one-frame peer echoes without delaying physical braking.
    const bool confirmed=routeInvalidationConfirmed(
      inspection.first_invalid_distance,immediate_repair_distance_,matched,
      invalidation_.matched_updates,age,near_invalidation_confirm_updates_,
      near_invalidation_confirm_time_,invalidation_confirm_updates_,
      invalidation_confirm_time_);
    const bool near_persistent=confirmed&&near;
    if (confirmed) {
      invalidation_.authorized = true;
      repair_authorized_ = true;
      if (near_persistent) keepSafePrefix();
      force_ = retry_ = true;
      replan_trigger_ = near_persistent ? "NEAR_ROUTE_INVALID"
                                       : "CONFIRMED_ROUTE_INVALID";
      reason_ = near_persistent ? "NEAR_ROUTE_INVALID_REPAIR"
                                : "CONFIRMED_ROUTE_INVALID_REPAIR";
    } else {
      repair_authorized_ = false;
      reason_ = "VERIFY_ROUTE_INVALIDATION";
      std::ostringstream status;
      status << "source=" << source
             << " invalid_distance=" << inspection.first_invalid_distance
             << " invalid_point=" << inspection.first_invalid_point.x << ','
             << inspection.first_invalid_point.y << ','
             << inspection.first_invalid_point.z
             << " validation_age=" << age
             << " confirmations=" << invalidation_.matched_updates << '/'
             << (near?near_invalidation_confirm_updates_:
                       invalidation_confirm_updates_);
      detail_ = status.str();
    }
  }
  void commit() {
    auto pair=map_pairs_.takeNewestPair();if(!pair)return;
    try {
      // A worker retains its input map by shared ownership. Never mutate that
      // map on the executor: replacing the pointer avoids copying both large
      // voxel sets and their bucket index each time a search is dispatched.
      auto next=std::make_shared<Grid>(grid_->cfg);
      next->update(occupied_guard_ && pair->stamp==occupied_guard_stamp_ ? occupied_guard_->occupied :
                     cloudSet(*pair->occupied, grid_->cfg.resolution),
                   cloudSet(*pair->free, grid_->cfg.resolution));
      grid_=std::move(next);
    } catch (const std::runtime_error &e) {
      RCLCPP_ERROR(get_logger(), "Map parse: %s", e.what());
      return;
    }
    have_map_ = true;
    map_source_=MapInputSource::Primary;
    map_stamp_ = rclcpp::Time(pair->occupied->header.stamp, get_clock()->get_clock_type());
    ++generation_;
    if (!owned_path_.empty())
      observeInvalidation(inspectOwnedRoute(), true, "MATCHED_MAP");
  }

  void installSnapshotGrid(std::shared_ptr<Grid> next,int64_t source_stamp,MapInputSource source){
    const bool changed_source=source!=map_source_;
    grid_=std::move(next);occupied_guard_.reset();occupied_guard_stamp_=-1;
    have_map_=true;map_stamp_=rclcpp::Time(source_stamp,get_clock()->get_clock_type());
    map_source_=source;++generation_;++map_commits_;
    if(changed_source)RCLCPP_WARN(get_logger(),
      "PLANNER_MAP_SOURCE source=%s primary_age=%.3f local_stamp=%lld",
      mapInputName(source),primary_stamp_ns_>0?double(now().nanoseconds()-primary_stamp_ns_)*1e-9:-1.,
      static_cast<long long>(local_committed_stamp_ns_));
    // Confirmation belongs to an original, complete source packet, not to a
    // recomposed grid. refreshCompactMap() counts that packet at most once.
  }
  void refreshCompactMap(){
    if(!compact_maps_)return;
    bool primary_advanced=false,local_advanced=false,map_changed=false;
    if(primary_pending_){
      auto m=std::move(primary_pending_);const auto stamp=stampNs(m->header.stamp);
      if(stamp>primary_stamp_ns_){
        try{
          auto decoded=parseMapSnapshot(*m,grid_->cfg.resolution);
          auto next=std::make_shared<Grid>(grid_->cfg);
          next->update(std::move(decoded.occupied),std::move(decoded.free));
          primary_grid_=std::move(next);primary_stamp_ns_=stamp;primary_bytes_=m->data.size();
          primary_advanced=true;
        }catch(const std::runtime_error &e){
          ++map_rejected_;RCLCPP_ERROR(get_logger(),"Primary snapshot: %s",e.what());
        }
      }
    }
    const auto local_stamp=local_pending_?stampNs(local_pending_->header.stamp):-1;
    const auto current_stamp=now().nanoseconds();
    const auto selected=selectMapInput(primary_stamp_ns_,local_stamp,current_stamp,
      map_timeout_,local_fallback_enabled_);
    if(selected==MapInputSource::Primary){
      // The compact decoder validates the complete packet, but only hashes a
      // bounded near-field region. Do not decode a million historical free
      // voxels just to confirm an obstacle in front of the vehicle.
      if(have_odom_ && local_fallback_enabled_ && local_stamp>local_overlay_stamp_ns_ &&
         localOverlayEligible(primary_stamp_ns_,local_stamp,current_stamp,map_timeout_)){
        try{
          auto decoded=parseMapSnapshot(*local_pending_,grid_->cfg.resolution,
            map_snapshot::default_max_cells,localOverlayRegion(grid_->cfg,current_,6.0));
          auto next=std::make_shared<Grid>(grid_->cfg);
          next->update(std::move(decoded.occupied),{});
          local_overlay_grid_=std::move(next);local_overlay_stamp_ns_=local_stamp;
          local_bytes_=local_pending_->data.size();local_advanced=true;
        }catch(const std::runtime_error &e){
          ++map_rejected_;local_pending_.reset();
          RCLCPP_ERROR(get_logger(),"Local near-field snapshot: %s",e.what());
        }
      }
      const bool supplement=local_overlay_grid_ && local_fallback_enabled_ &&
        localOverlayEligible(primary_stamp_ns_,local_overlay_stamp_ns_,current_stamp,map_timeout_);
      auto next=supplement?local_overlay_.apply(primary_grid_,local_overlay_grid_->occupied):primary_grid_;
      if(!supplement)local_overlay_.clear();
      if(grid_!=next || map_source_!=selected || map_stamp_.nanoseconds()!=primary_stamp_ns_){
        // A fresh local supplement must NOT refresh the global map timestamp.
        installSnapshotGrid(std::move(next),primary_stamp_ns_,selected);map_changed=true;
      }
      if(!owned_path_.empty() && (map_changed||primary_advanced||local_advanced)){
        const auto inspection=inspectOwnedRoute();
        bool matched=false;int64_t confirmed_stamp=-1;const char *source="ATOMIC_MAP_CHECK";
        const auto confirms=[&](const Grid &evidence){
          const auto local=inspectRoute(evidence,owned_path_,current_,true,owned_route_.progress());
          return !local.valid && distance(local.first_invalid_point,inspection.first_invalid_point)<=.60;
        };
        // A local packet counts only if its own occupied evidence explains
        // the invalid point. Repeating an old PRIMARY obstacle alongside an
        // unrelated fresh local packet is not a second observation.
        if(!inspection.valid && supplement && local_advanced &&
           local_overlay_stamp_ns_>last_compact_confirmation_stamp_ns_ && confirms(*local_overlay_grid_)){
          matched=true;confirmed_stamp=local_overlay_stamp_ns_;source="LOCAL_NEARFIELD_MAP";
        }else if(!inspection.valid && primary_advanced &&
                 primary_stamp_ns_>last_compact_confirmation_stamp_ns_){
          const auto primary=inspectRoute(*primary_grid_,owned_path_,current_,
            planner_config_.allow_unknown,owned_route_.progress());
          if(!primary.valid && distance(primary.first_invalid_point,inspection.first_invalid_point)<=.60){
            matched=true;confirmed_stamp=primary_stamp_ns_;source="ATOMIC_MAP";
          }
        }
        if(matched)last_compact_confirmation_stamp_ns_=confirmed_stamp;
        observeInvalidation(inspection,matched,source);
      }
    }else if(selected==MapInputSource::LocalFallback &&
             (map_source_!=selected || local_stamp>local_committed_stamp_ns_)){
      try{
        auto decoded=parseMapSnapshot(*local_pending_,grid_->cfg.resolution);
        auto next=localFallbackGrid(grid_->cfg,std::move(decoded.occupied),
          std::move(decoded.free),primary_grid_.get());
        local_committed_stamp_ns_=local_stamp;local_bytes_=local_pending_->data.size();
        local_overlay_.clear();
        // Retain route ownership and arc progress. A new LOCAL map, with its
        // original source time, must certify geometry before any motion.
        installSnapshotGrid(std::move(next),local_stamp,selected);
        if(!owned_path_.empty()){
          const bool matched=local_stamp>last_compact_confirmation_stamp_ns_;
          if(matched)last_compact_confirmation_stamp_ns_=local_stamp;
          observeInvalidation(inspectOwnedRoute(),matched,"LOCAL_FALLBACK_MAP");
        }
      }catch(const std::runtime_error &e){
        ++map_rejected_;local_pending_.reset();
        RCLCPP_ERROR(get_logger(),"Local fallback snapshot: %s",e.what());
      }
    }
  }

  std::string inputState() const {
    if(!have_goal_)return "WAIT_GOAL";
    if(!have_map_)return "WAIT_MAP";
    if(!have_odom_)return "WAIT_ODOM";
    const double map_age=(now()-map_stamp_).seconds(),odom_age=(now()-odom_stamp_).seconds();
    if(map_age<-.05 || odom_age<-.05)return "FUTURE_INPUT_STAMP";
    if(map_age>map_timeout_)return "STALE_MAP";
    if(odom_age>odom_timeout_)return "STALE_ODOM";
    if(!grid_->insideAltitude(current_))return "ALTITUDE_OUT_OF_BOUNDS";
    return "READY";
  }
  bool ready() const {return inputState()=="READY";}
  struct PendingRouteSwitch {
    bool active{false};
    RouteTransition transition;
    std::chrono::steady_clock::time_point since{},last_seen{};
    std::string trigger;
    size_t observations{0};
  };
  void clearPendingSwitch(){pending_switch_=PendingRouteSwitch{};}
  bool switchConfirmationReady(const RouteSwitchAssessment &assessment,
                               const std::string &trigger,
                               std::chrono::steady_clock::time_point wall,
                               std::string &decision) {
    const bool have_last_switch=last_route_switch_.time_since_epoch().count()!=0;
    const bool cooling=have_last_switch &&
      std::chrono::duration<double>(wall-last_route_switch_).count()<route_switch_.cooldown;
    if(!assessment.needs_confirmation && !cooling){
      clearPendingSwitch();decision="SWITCH_BETTER_ROUTE";return true;
    }
    const bool recently_seen=pending_switch_.active &&
      std::chrono::duration<double>(wall-pending_switch_.last_seen).count()<=
        std::max(1.0,4.0*replan_period_);
    const bool same=recently_seen &&
      samePendingSwitch(pending_switch_.transition,assessment.transition,route_switch_);
    if(!same){
      pending_switch_.active=true;
      pending_switch_.transition=assessment.transition;
      pending_switch_.since=pending_switch_.last_seen=wall;
      pending_switch_.trigger=trigger;
      pending_switch_.observations=1;
      if(route_switch_.confirmation_time<=0 && !cooling){
        clearPendingSwitch();decision="SWITCH_BETTER_CONFIRMED";return true;
      }
      decision=cooling?"KEEP_ROUTE_SWITCH_COOLDOWN":"KEEP_ROUTE_SWITCH_PENDING";
      return false;
    }
    pending_switch_.transition=assessment.transition;
    pending_switch_.last_seen=wall;
    ++pending_switch_.observations;
    const bool confirmed=std::chrono::duration<double>(wall-pending_switch_.since).count()>=
      route_switch_.confirmation_time && pending_switch_.observations>=3;
    if(!confirmed){decision="KEEP_ROUTE_SWITCH_PENDING";return false;}
    if(cooling){decision="KEEP_ROUTE_SWITCH_COOLDOWN";return false;}
    clearPendingSwitch();decision="SWITCH_BETTER_CONFIRMED";return true;
  }
  struct WorkResult {
    TurnGeometryResult turn;
    std::unique_ptr<Planner> planner;
    PlanResult route;
    SplineResult smooth;
    uint64_t epoch{0};
    bool switch_bypass{false};
    bool safety_repair{false};
    std::string trigger;
    std::vector<RoutePassage> reserved_gates;
    size_t candidate_count{1},shared_gates{0},peer_epoch{0};
    double candidate_ms{0};
    RouteHandoffResult handoff;
    ProfileRepairResult profile;
    std::vector<Vec3> switch_points;
  };
  void keepSafePrefix(){
    RouteMemory retained=owned_path_.empty()?route_:owned_route_;
    auto prefix=retained.safePrefix(*grid_,current_,planner_config_.allow_unknown,clearance_contract_.reserve);
    if(occupied_guard_ && !prefix.empty()){
      RouteMemory guard_prefix;guard_prefix.set(std::move(prefix));
      prefix=guard_prefix.safePrefix(*occupied_guard_,current_,true,clearance_contract_.reserve);
    }
    // Repeated evidence for the same blocked tail must not shift the prefix
    // start every callback. Keep its revision/profile while its endpoint holds.
    RouteMemory checked=route_;
    if(!path_.empty()&&!prefix.empty()&&distance(path_.back(),prefix.back())<1e-5&&
       checked.valid(*grid_,current_,planner_config_.allow_unknown)&&guardValid(checked)){
      route_=std::move(checked);temporary_=true;return;
    }
    path_=std::move(prefix);route_.set(path_);route_.updateProgress(current_);temporary_=true;
  }
  void complete(){
    if(!work_.valid() || work_.wait_for(std::chrono::seconds(0))!=std::future_status::ready)return;
    WorkResult r;
    try {r=work_.get();}
    catch(const std::exception&e){
      planner_=std::make_unique<Planner>(planner_config_);
      if(!owned_path_.empty())
        planner_->setCommittedRoute(geometricRouteSuffixAt(
          owned_path_,current_,owned_route_.progress()));
      reason_="WORKER_ERROR_RETRY";detail_=e.what();force_=retry_=true;return;
    }
    planner_=std::move(r.planner);
    planner_->setCancellation({});
    if(r.epoch!=goal_epoch_->load()){
      planner_->reset();force_=true;return; // A superseded goal can never publish.
    }
    if(!ready()){
      // The worker may have adopted its proposal internally. Input loss must
      // not install that uncommitted tail behind the retained published route.
      if(!owned_path_.empty())planner_->setCommittedRoute(
        geometricRouteSuffixAt(owned_path_,current_,owned_route_.progress()));
      force_=true;reason_=inputState();return;
    }
    // Snapshot search may outlive a map update. Certify both the proposed
    // geometry and the owned route from the current position on the latest map.
    if(r.smooth.mode!="RETAINED_STRAIGHT_ROUTE"&&!r.smooth.points.empty()){
      r.turn=prepareTurnGeometry(*grid_,r.smooth.points,profile_tracker_.turn,handoff_config_,
        profile_tracker_.lateral_accel,profile_tracker_.yaw_rate,profile_tracker_.sharp_turn,
        r.handoff.applied?r.handoff.prefix_length:0.,clearance_contract_.reserve,
        planner_config_.allow_unknown);
      if(r.turn.smoothed||r.turn.polygonized){r.smooth.points=r.turn.points;
        r.smooth.mode+="_GLOBAL_TURN_POLICY";}
    }
    RouteMemory candidate;
    if(r.smooth.mode=="RETAINED_STRAIGHT_ROUTE"){
      // This worker returned the exact incumbent geometry. Its arc origin
      // did not move, so a new progress=0 memory would reject a vehicle that
      // has already flown beyond the initial 2 m projection window.
      candidate=owned_route_;
    }else candidate.set(r.smooth.points);
    bool candidate_valid=!r.smooth.points.empty() &&
      candidate.valid(*grid_,current_,planner_config_.allow_unknown) && guardValid(candidate);
    bool candidate_margin_valid=true;
    if(candidate_valid && r.smooth.mode!="RETAINED_STRAIGHT_ROUTE" && !r.route.points.empty()){
      const auto reference=routeSlice(routeSuffixFrom(r.route.points,current_),0,clearance_contract_.horizon);
      const auto actual=routeSlice(routeSuffixFrom(r.smooth.points,current_),0,clearance_contract_.horizon);
      const double reserve=routeMargin(*grid_,reference,clearance_contract_.reserve,planner_config_.allow_unknown);
      candidate_margin_valid=reserve<=.01||routeHasMargin(*grid_,actual,
        std::max(0.,reserve-.005),planner_config_.allow_unknown);
      candidate_valid=candidate_margin_valid;
    }
    const auto current_inspection=inspectOwnedRoute();
    const bool incumbent_valid=!owned_path_.empty() && current_inspection.valid;
    const bool recovered_during_search=r.safety_repair && incumbent_valid;
    const auto wall=std::chrono::steady_clock::now();
    RouteSwitchAssessment switch_assessment;
    bool have_switch_assessment=false;
    bool switch_allowed=candidate_valid && !recovered_during_search;
    std::string switch_decision;
    if(recovered_during_search){
      clearPendingSwitch();
      switch_decision="KEEP_ROUTE_MAP_RECOVERED";
    }else if(candidate_valid){
      const auto incumbent_suffix=routeSuffixAt(owned_path_,current_,owned_route_.progress());
      const auto &proposal=r.switch_points.empty()?r.smooth.points:r.switch_points;
      const auto challenger_suffix=routeSuffixFrom(proposal,current_);
      const double challenger_cost=std::max(stableRouteCost(*grid_,challenger_suffix),
        stableRouteCost(*grid_,routeSuffixFrom(r.smooth.points,current_)));
      const bool entrance_improved=entrance_.topology.enabled && r.peer_epoch==peer_generation_ &&
        sharedPassageCount(incumbent_suffix,r.reserved_gates)>
          sharedPassageCount(challenger_suffix,r.reserved_gates);
      if(!incumbent_valid){
        clearPendingSwitch();switch_decision=owned_path_.empty()?"INITIAL_ROUTE":"REPAIR_INVALID_ROUTE";
      }else if(r.trigger=="CLEARANCE_DEGRADED"){
        const auto transition=routeTransition(incumbent_suffix,challenger_suffix,current_,route_switch_);
        const Vec3 direction=horizontalRouteDirection(incumbent_suffix,current_,route_switch_.lookahead);
        const double old_back=routeBacktrackDistance(incumbent_suffix,current_,direction,route_switch_.backtrack_horizon);
        const double old_cost=stableRouteCost(*grid_,incumbent_suffix);
        const double old_margin=routeMargin(*grid_,routeSlice(incumbent_suffix,0,clearance_contract_.horizon),clearance_contract_.reserve);
        const double new_margin=std::min(
          routeMargin(*grid_,routeSlice(challenger_suffix,0,clearance_contract_.horizon),clearance_contract_.reserve),
          routeMargin(*grid_,routeSlice(routeSuffixFrom(r.smooth.points,current_),0,clearance_contract_.horizon),clearance_contract_.reserve));
        const double old_deficit=clearanceDeficit(*grid_,incumbent_suffix,clearance_contract_.horizon,clearance_contract_.reserve);
        const double new_deficit=clearanceDeficit(*grid_,routeSuffixFrom(r.smooth.points,current_),clearance_contract_.horizon,clearance_contract_.reserve);
        const bool margin_improved=r.route.clearance_near_repair?
          new_deficit<old_deficit-std::max(.002,.10*old_deficit):new_margin>old_margin+.015;
        switch_allowed=r.peer_epoch==peer_generation_ && transition.valid &&
          transition.angle<=route_switch_.soft_angle && transition.backtrack<=old_back+.02 &&
          margin_improved && challenger_cost<=old_cost+std::max(.75,.15*old_cost) &&
          sharedPassageCount(challenger_suffix,r.reserved_gates)<=sharedPassageCount(incumbent_suffix,r.reserved_gates);
        clearPendingSwitch();switch_decision=switch_allowed?"COMMIT_CLEARANCE_LOCAL_REPAIR":"KEEP_ROUTE_CLEARANCE_POLICY";
      }else if(r.trigger=="ROUTE_LOCAL_REFRESH"){
        // A certified local shortcut is reviewed immediately. Long global-route
        // cooldown/gain hysteresis must not freeze a no-longer-needed detour.
        const auto actual_suffix=routeSuffixFrom(r.smooth.points,current_);
        const auto transition=routeTransition(incumbent_suffix,actual_suffix,current_,route_switch_);
        const Vec3 direction=horizontalRouteDirection(incumbent_suffix,current_,route_switch_.lookahead);
        const double old_back=routeBacktrackDistance(incumbent_suffix,current_,direction,route_switch_.backtrack_horizon);
        switch_allowed=r.peer_epoch==peer_generation_ && transition.valid &&
          transition.angle<=route_switch_.soft_angle && transition.backtrack<=old_back+.02 &&
          challenger_cost<stableRouteCost(*grid_,incumbent_suffix)-.02 &&
          pathLength(actual_suffix)<pathLength(incumbent_suffix)-.10 &&
          sharedPassageCount(actual_suffix,r.reserved_gates)<=sharedPassageCount(incumbent_suffix,r.reserved_gates);
        clearPendingSwitch();switch_decision=switch_allowed?"COMMIT_LOCAL_REFRESH":"KEEP_ROUTE_LOCAL_REFRESH_POLICY";
      }else if(r.profile.applied){
        const auto transition=routeTransition(incumbent_suffix,challenger_suffix,current_,route_switch_);
        const Vec3 direction=horizontalRouteDirection(incumbent_suffix,current_,route_switch_.lookahead);
        const double old_back=routeBacktrackDistance(incumbent_suffix,current_,direction,route_switch_.backtrack_horizon);
        switch_allowed=r.peer_epoch==peer_generation_ && transition.valid &&
          transition.angle<=route_switch_.soft_angle && transition.backtrack<=old_back+.02 &&
          challenger_cost<=stableRouteCost(*grid_,incumbent_suffix)+.10 &&
          sharedPassageCount(challenger_suffix,r.reserved_gates)<=sharedPassageCount(incumbent_suffix,r.reserved_gates);
        clearPendingSwitch();switch_decision=switch_allowed?"COMMIT_PROFILE_LOCAL_REPAIR":"KEEP_ROUTE_PROFILE_POLICY";
      }else if(r.switch_bypass){
        clearPendingSwitch();switch_decision="COMMIT_AUTHORIZED_REPAIR";
      }else if(entrance_improved){
        // A bounded different-door route may be longer. Ordinary length-only
        // hysteresis would reject every useful separation. Confirm its heading
        // and geometry over successive reviews before replacing the route.
        switch_assessment=assessRouteSwitch(incumbent_suffix,challenger_suffix,current_,
          stableRouteCost(*grid_,incumbent_suffix),challenger_cost,route_switch_);
        have_switch_assessment=true;switch_assessment.needs_confirmation=true;
        switch_allowed=switchConfirmationReady(switch_assessment,"DISTINCT_PASSAGE_ROUTE",wall,switch_decision);
        if(switch_allowed)switch_decision="COMMIT_DISTINCT_PASSAGE";
      }else if(entrance_.topology.enabled && r.peer_epoch==peer_generation_ &&
          sharedPassageCount(challenger_suffix,r.reserved_gates)>
            sharedPassageCount(incumbent_suffix,r.reserved_gates)){
        clearPendingSwitch();switch_allowed=false;switch_decision="KEEP_COMMITTED_ENTRANCE";
      }else if(r.route.reason=="DISTINCT_PASSAGE_ROUTE"){
        // A changed reservation or an already committed approach must not turn
        // yesterday's entrance decision into a gratuitous detour today.
        clearPendingSwitch();switch_allowed=false;switch_decision="KEEP_ROUTE_NO_ENTRANCE_GAIN";
      }else if(!route_switch_.enabled){
        clearPendingSwitch();switch_decision="SWITCH_HYSTERESIS_DISABLED";
      }else{
        const auto incumbent=routeSuffixAt(owned_path_,current_,owned_route_.progress());
        const auto challenger=routeSuffixFrom(proposal,current_);
        const double old_total=stableRouteCost(*grid_,incumbent);
        const double new_total=challenger_cost;
        switch_assessment=assessRouteSwitch(
          incumbent,challenger,current_,old_total,new_total,route_switch_);
        have_switch_assessment=true;
        if(!switch_assessment.cost_allows){
          clearPendingSwitch();switch_allowed=false;
          switch_decision="KEEP_ROUTE_SWITCH_COST";
        }else{
          switch_allowed=switchConfirmationReady(
            switch_assessment,r.trigger,wall,switch_decision);
        }
      }
    }
    // Do not hide a large route change behind an identical near prefix: all
    // original switching/entrance decisions above use the unblended proposal.
    if(switch_allowed && incumbent_valid && !r.handoff.applied &&
       !r.switch_points.empty()){
      switch_allowed=false;switch_decision="KEEP_ROUTE_NO_SAFE_HANDOFF";
    }
    if(switch_allowed && incumbent_valid && (r.handoff.reason=="PREFIX_BEZIER_JOIN" || r.handoff.reason=="CLEARANCE_LOCAL_JOIN" || r.handoff.reason=="NEAR_CLEARANCE_JOIN" || r.handoff.reason=="LOCAL_REFRESH_JOIN" || r.profile.applied) &&
       measured_speed_>.10 && candidate.progress()+.20>=r.handoff.prefix_length){
      switch_allowed=false;switch_decision="KEEP_ROUTE_HANDOFF_OVERRUN";
    }
    const bool committed=candidate_valid && switch_allowed;
    if(committed){
      partial_retry_.clear();
      const bool replaced=incumbent_valid && !owned_path_.empty();
      if(r.smooth.mode=="RETAINED_STRAIGHT_ROUTE")restoreOwnedRoute();
      else {
        path_=std::move(r.smooth.points);route_=std::move(candidate);
        owned_path_=path_;owned_route_=route_;
      }
      temporary_=distance(path_.back(),approach_.valid?approach_.effective:goal_)>.15;
      retry_=false;
      recovery_route_=r.route.reason=="RECOVERY_MARGIN_ESCAPE";
      planner_->setCommittedRoute(r.smooth.mode=="RETAINED_STRAIGHT_ROUTE"
        ?geometricRouteSuffixAt(path_,current_,route_.progress()):path_);
      clearInvalidation(false);
      follower_block_reported_=false;
      if(replaced)last_route_switch_=wall;
      reason_=r.route.reason;
      force_=false;
    }else if(incumbent_valid){
      // A rejected/obsolete result must never truncate the still-valid owned
      // route. The worker may have adopted it internally, so restore ownership.
      restoreOwnedRoute();
      planner_->setCommittedRoute(geometricRouteSuffixAt(owned_path_,current_,owned_route_.progress()));
      retry_=false;force_=false;
      if(recovered_during_search)clearInvalidation(false);
      if(switch_decision.empty()){
        clearPendingSwitch();switch_decision="KEEP_ROUTE_CANDIDATE_INVALID";
      }
      reason_=switch_decision;
      r.smooth.mode="RETAINED_INCUMBENT";
      const double wall_s=std::chrono::duration<double>(wall.time_since_epoch()).count();
      if(r.trigger=="ENTRANCE_REVIEW"&&!pending_switch_.active)
        entrance_review_.noImprovement(wall_s,entrance_.topology.refresh_period);
      if(r.trigger=="PARTIAL_ROUTE_FINISHED")
        partial_retry_.rejected(wall_s,owned_path_.back(),approach_.effective,owned_route_.remaining());
    }else{
      clearPendingSwitch();keepSafePrefix();retry_=true;force_=true;
      reason_=r.smooth.points.empty()?r.route.reason:"RESULT_INVALIDATED_RETRY";
      r.smooth.mode=path_.empty()?"EMPTY_RETRY":"SAFE_PREFIX_RETRY";
      if(switch_decision.empty())switch_decision="NO_VALID_ROUTE";
    }
    raw_path_pub_->publish(makePath(candidate_valid?r.route.points:std::vector<Vec3>{},frame_,now()));
    std::ostringstream os;
    os<<"goal_epoch="<<r.epoch<<" reused="<<r.route.reused<<" ms="<<r.route.elapsed_ms
      <<" search_result="<<r.route.reason<<" smoothing="<<r.smooth.mode<<" spline_spans="<<r.smooth.spans
      <<" spline_control_spacing="<<r.smooth.control_spacing<<" spline_deviation="<<r.smooth.deviation
      <<" spline_candidate_valid="<<candidate_valid<<" candidate_margin_valid="<<candidate_margin_valid
      <<" route_committed="<<committed
      <<" spline_optimized="<<(candidate_valid&&r.smooth.optimized)
      <<" spline_opt_iterations="<<r.smooth.optimization_iterations
      <<" spline_opt_cost_before="<<r.smooth.optimization_cost_before
      <<" spline_opt_cost_after="<<r.smooth.optimization_cost_after
      <<" radius="<<grid_->cfg.inflation_xy<<" expanded="<<r.route.expansions
      <<" flat_expanded="<<r.route.flat_expansions<<" spatial_expanded="<<r.route.spatial_expansions
      <<" clearance_near_repair="<<r.route.clearance_near_repair
      <<" clearance_deficit_before="<<r.route.clearance_deficit_before<<" clearance_deficit_after="<<r.route.clearance_deficit_after
      <<" patch_search_resumed="<<r.route.patch_search_resumed
      <<" patch_expansions="<<r.route.patch_expansions<<" retained_search_nodes="<<r.route.retained_search_nodes
      <<" search_mode="<<r.route.search_mode<<" flat_altitude="<<r.route.flat_altitude
      <<" length="<<pathLength(path_)<<" target="<<goal_.x<<","<<goal_.y<<","<<goal_.z
      <<" candidates="<<r.candidate_count<<" candidate_ms="<<r.candidate_ms
      <<" shared_entrances="<<sharedPassageCount(path_,r.reserved_gates)
      <<" local_patch="<<r.route.locally_patched
      <<" patch_begin="<<r.route.patch_begin<<" patch_end="<<r.route.patch_end
      <<" switch_trigger="<<r.trigger<<" switch_decision="<<switch_decision
      <<" profile_repair="<<r.profile.reason<<" profile_before="<<r.profile.before_limit
      <<" profile_after="<<r.profile.after_limit;
    os<<" spline_time_scale="<<r.smooth.optimization_time_scale
      <<" spline_optimizer_interval="<<r.smooth.optimization_interval;
    os<<" turn_smoothed="<<r.turn.smoothed<<" turn_polygonized="<<r.turn.polygonized
      <<" turn_repair_samples="<<r.turn.remaining_repair_samples
      <<" turn_stop_vertices="<<r.turn.remaining_stops<<" turn_max_curvature="<<r.turn.max_curvature
      <<" turn_budget_exhausted="<<r.turn.budget_exhausted;
    os<<" handoff="<<r.handoff.reason<<" handoff_applied="<<r.handoff.applied
      <<" handoff_prefix="<<r.handoff.prefix_length<<" handoff_blend="<<r.handoff.blend_length;
    if(have_switch_assessment){
      os<<" switch_angle_deg="<<switch_assessment.transition.angle*180./pi
        <<" switch_backtrack="<<switch_assessment.transition.backtrack
        <<" switch_old_score="<<switch_assessment.old_total
        <<" switch_new_score="<<switch_assessment.new_total
        <<" switch_angle_penalty="<<switch_assessment.angle_penalty
        <<" switch_backtrack_penalty="<<switch_assessment.backtrack_penalty
        <<" switch_effective_new_score="<<switch_assessment.effective_new_total
        <<" switch_gain_ratio="<<switch_assessment.gain_ratio
        <<" switch_required_gain_ratio="<<switch_assessment.required_gain_ratio;
    }
    if(pending_switch_.active)
      os<<" switch_pending_age="
        <<std::chrono::duration<double>(wall-pending_switch_.since).count()
        <<" switch_pending_observations="<<pending_switch_.observations;
    os
      <<" invalidation_active="<<invalidation_.active
      <<" invalidation_authorized="<<invalidation_.authorized
      <<" invalidation_distance="<<invalidation_.distance
      <<" invalidation_confirmations="<<invalidation_.matched_updates
      <<" invalidation_source="<<invalidation_.source
      <<" invalidation_point="<<invalidation_.point.x<<","<<invalidation_.point.y
      <<","<<invalidation_.point.z
      <<" last_invalidation_source="<<last_invalidation_source_
      <<" last_invalidation_distance="<<last_invalidation_distance_
      <<" last_invalidation_point="<<last_invalidation_point_.x<<","<<last_invalidation_point_.y
      <<","<<last_invalidation_point_.z
      <<" follower_stall_reports="<<follower_stall_reports_
      <<" retained_search_nodes="<<planner_->retainedSearchNodes()<<" autonomous_search_refreshes="<<search_refreshes_<<" goal_adjusted="<<approach_.adjusted<<" goal_offset="<<distance(goal_,approach_.effective)
      <<" guide_expansions="<<r.route.guide_expansions
      <<" search_repair_events="<<r.route.search_repair_events
      <<" search_repair_removed="<<r.route.search_repair_removed
      <<" search_repair_preserved="<<r.route.search_repair_preserved
      <<" search_repair_pending="<<r.route.search_repair_pending
      <<" effective_target="<<approach_.effective.x<<","<<approach_.effective.y<<","<<approach_.effective.z;
    detail_=os.str();
    RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),1500,"%s %s",reason_.c_str(),detail_.c_str());
    publish();
  }
  void plan() {
    refreshCompactMap();
    complete();
    if(!ready()){
      // Input freshness gates publication/motion, not route ownership. A
      // delayed odometry/map callback must not erase a checked route and force
      // another empty-plan cycle once fresh evidence arrives. commit()/the
      // old_valid check recertify retained geometry against the new map.
      reason_=inputState();return;
    }
    if(!grid_->insideAltitude(current_)){
      reason_="ALTITUDE_OUT_OF_BOUNDS";detail_.clear();return;
    }
    if(work_.valid())return;
    const auto wall=std::chrono::steady_clock::now();
    if(!force_ && std::chrono::duration<double>(wall-last_plan_).count()<replan_period_)return;
    // Retry at the configured rate even if no new map is received. A goal change
    // bypasses this gate; bounded-search continuation does not spin at 50 Hz.
    if(retry_ && std::chrono::duration<double>(wall-last_plan_).count()<replan_period_)return;
    auto selection=resolveGoalApproach(*grid_,goal_,current_,approach_config_,planner_config_.allow_unknown,&approach_);
    if(!selection.valid){
      const auto inspection=inspectOwnedRoute();
      if(!owned_path_.empty() && inspection.valid){
        reason_="KEEP_ROUTE_GOAL_APPROACH_TRANSIENT";
        detail_="existing committed route remains map-valid";
        force_=retry_=false;last_plan_=wall;return;
      }
      approach_=selection;keepSafePrefix();clearPendingSwitch();
      retry_=true;force_=true;last_plan_=wall;
      reason_="GOAL_BLOCKED";
      std::ostringstream os;os<<"target="<<goal_.x<<","<<goal_.y<<","<<goal_.z
        <<" goal_center_occupied="<<grid_->occupied.count(grid_->toKey(goal_))
        <<" goal_clearance="<<grid_->clearanceAt(goal_)<<" goal_approach_limit="<<approach_config_.max_offset;
      detail_=os.str();publish();return;
    }
    if(!approach_.valid||distance(selection.effective,approach_.effective)>.10){
      const auto inspection=inspectOwnedRoute();
      if(owned_path_.empty() || !inspection.valid){
        if(planner_)planner_->reset();
        // A safe transient prefix belongs to execution, even though a new
        // goal no longer owns the previous full route.
        if(!owned_path_.empty()){path_.clear();route_.clear();}
        owned_path_.clear();owned_route_.clear();
        clearPendingSwitch();temporary_=false;
        last_search_refresh_=wall;last_refresh_generation_=generation_;
      }else{
        // A map-dependent endpoint snap is not a new mission. Keep the endpoint
        // used by the valid committed route until that route really becomes bad.
        selection=approach_;
      }
    }
    approach_=selection;
    const auto peer_routes = freshPeerRoutes();
    const auto full_peer_routes = freshPeerRoutes(true);
    const double wall_s=std::chrono::duration<double>(wall.time_since_epoch()).count();
    const bool topology_review=entrance_.topology.enabled && !full_peer_routes.empty() &&
      (pending_switch_.active || entrance_review_.shouldReview(*grid_,owned_path_,
       owned_route_.progress(),current_,full_peer_routes,entrance_.topology,wall_s,peer_generation_));
    // An unfinished search uses a frozen snapshot. A persistent mission must
    // not need a new goal message to refresh it. Refresh after a bounded age,
    // when the owned route is invalid (including a retained safe prefix) and
    // new evidence arrived. A prefix must not freeze an obsolete search forever.
    auto inspection=inspectOwnedRoute();
    const bool old_valid=!owned_path_.empty()&&inspection.valid;
    if(!old_valid&&planner_&&generation_!=last_refresh_generation_&&
       std::chrono::duration<double>(wall-last_search_refresh_).count()>search_snapshot_max_age_){
      if(planner_->refreshChangedSearches(*grid_)>0) {
        ++search_refreshes_;
      }
      last_search_refresh_=wall;
      last_refresh_generation_=generation_;
    }
    if(!old_valid && !owned_path_.empty())observeInvalidation(inspection,false,"PLAN_CHECK");
    const bool clearance_repair=old_valid && !repair_authorized_ &&
      clearance_review_.due(*grid_,owned_path_,owned_route_.progress(),now().seconds(),
        generation_,clearance_contract_,planner_config_.allow_unknown);
    if(follower_block_reported_ && old_valid && !clearance_repair){
      follower_block_reported_=false;retry_=force_=false;
      reason_="FOLLOWER_BLOCK_NOT_MAP_CONFIRMED";
      detail_="committed route is clear on the latest matched map; route retained";
      last_plan_=wall;return;
    }
    if(!old_valid && !owned_path_.empty() && !repair_authorized_){
      reason_="VERIFY_ROUTE_INVALIDATION";
      force_=false;last_plan_=wall;return;
    }
    const bool peer_changed = peer_generation_ != last_planned_peer_generation_;
    const bool peer_conflict = old_valid && peer_changed &&
      std::chrono::duration<double>(wall-last_peer_replan_).count() >=
        peer_replan_min_period_ && conflictsWithPeers(owned_path_, peer_routes);
    if(peer_conflict && planner_){planner_->requestRepair();last_peer_replan_=wall;}
    const bool finished_partial=temporary_&&!owned_path_.empty()&&
      (recovery_route_ ? owned_route_.remaining()<.30
                       : owned_route_.remaining()<2.0);
    if(finished_partial && old_valid && !repair_authorized_ && !peer_conflict &&
       !pending_switch_.active && !profile_repair_requested_ && !clearance_repair &&
       !partial_retry_.ready(wall_s,owned_path_.back(),approach_.effective,owned_route_.remaining())){
      reason_="KEEP_PARTIAL_ROUTE_RETRY_DELAY";force_=retry_=false;last_plan_=wall;return;
    }
    const bool profile_repair=profile_repair_requested_ && old_valid && !finished_partial &&
      !repair_authorized_ && !peer_conflict && !pending_switch_.active && !clearance_repair;
    if(old_valid && !finished_partial && !peer_conflict && !profile_repair && !clearance_repair &&
       !pending_switch_.active && !topology_review && planner_config_.lock_valid_route){
      reason_=temporary_?"KEEP_RECOVERY_ROUTE":"KEEP_VALID_SPLINE";
      std::ostringstream kept;
      kept<<"switch_decision=KEEP_COMMITTED_ROUTE route_remaining="
          <<owned_route_.remaining()<<" path_length="<<pathLength(owned_path_);
      detail_=kept.str();
      retry_=force_=false;last_plan_=wall;return;
    }
    std::string plan_trigger;
    if(clearance_repair)plan_trigger="CLEARANCE_DEGRADED";
    else if(profile_repair)plan_trigger=replan_trigger_=="TURN_EXECUTION_REPAIR"?
      "TURN_EXECUTION_REPAIR":"PROFILE_LIMITED_PROGRESS";
    else if(!replan_trigger_.empty())plan_trigger=replan_trigger_;
    else if(pending_switch_.active)plan_trigger=pending_switch_.trigger;
    else if(topology_review)plan_trigger="ENTRANCE_REVIEW";
    else if(peer_conflict)plan_trigger="PEER_ROUTE_CONFLICT";
    else if(finished_partial)plan_trigger="PARTIAL_ROUTE_FINISHED";
    else if(repair_authorized_)plan_trigger=replan_trigger_.empty()?"CONFIRMED_ROUTE_INVALID":replan_trigger_;
    else if(retry_)plan_trigger="ROUTE_RETRY";
    else plan_trigger=old_valid?"ROUTE_LOCAL_REFRESH":"ROUTE_REFRESH";
    const bool safety_repair=repair_authorized_;
    const bool switch_bypass=owned_path_.empty() || safety_repair || finished_partial;
    if((safety_repair || finished_partial) && planner_)planner_->requestRepair();
    replan_trigger_.clear();profile_repair_requested_=false;force_=false;last_plan_=wall;
    if(!planner_)planner_=std::make_unique<Planner>(planner_config_);
    const auto epoch=goal_epoch_->load();
    auto token=goal_epoch_;
    planner_->setCancellation([token,epoch]{return token->load()!=epoch;});
    auto snapshot=grid_;
    auto p=std::move(planner_);const auto start=current_,goal=approach_.effective;const auto cfg=spline_;
    const auto incumbent=routeSuffixAt(owned_path_,start,owned_route_.progress());
    const bool unknown=planner_config_.allow_unknown;
    last_planned_peer_generation_=peer_generation_;
    if(topology_review)last_topology_review_=wall;
    const auto entrance_config=entrance_;const auto pc=planner_config_;
    const auto committed_geometry=owned_path_;const auto peer_epoch=peer_generation_;
    const auto handoff_config=handoff_config_;
    const auto profile_tracker=profile_tracker_;const auto switch_config=route_switch_;
    const auto clearance_contract=clearance_contract_;
    const double committed_progress=owned_route_.progress(),speed=measured_speed_;
    auto handoff_prefix=retainedForwardPrefix(*snapshot,
      owned_path_.empty()?path_:owned_path_,start,
      owned_path_.empty()?route_.progress():owned_route_.progress(),
      measured_speed_,handoff_config,unknown);
    if(finished_partial && !handoff_prefix.empty()){
      const double keep=std::max(.30,owned_route_.remaining()-handoff_config.blend_min);
      if(pathLength(handoff_prefix)>keep)handoff_prefix=routeSlice(handoff_prefix,0,keep);
    }
    reason_="PLANNING";
    work_=std::async(std::launch::async,
      [this,p=std::move(p),snapshot=std::move(snapshot),start,goal,cfg,unknown,
       token,epoch,peer_routes,full_peer_routes,incumbent,switch_bypass,safety_repair,
       plan_trigger,entrance_config,pc,committed_geometry,peer_epoch,handoff_config,handoff_prefix,
       profile_tracker,switch_config,committed_progress,speed,clearance_contract]()mutable{
        const Grid &g=*snapshot;
        WorkResult out;out.epoch=epoch;out.switch_bypass=switch_bypass;
        out.safety_repair=safety_repair;out.trigger=plan_trigger;out.peer_epoch=peer_epoch;
        if(plan_trigger=="CLEARANCE_DEGRADED"){
          const auto cancelled=[token,epoch]{return token->load()!=epoch;};
          out.peer_epoch=peer_epoch;
          out.route=repairRouteClearance(g,committed_geometry,committed_progress,
            speed<=.15?std::min(.30,pathLength(handoff_prefix)):pathLength(handoff_prefix),
            clearance_contract,pc,cancelled);
          if(!out.route.points.empty()){
            if(out.route.clearance_near_repair){
              out.smooth.points=out.route.points;out.smooth.mode="NEAR_CLEARANCE_OFFSET";
            }else out.smooth=smoothPlannedRoute(g,out.route,cfg,unknown,cancelled,
              handoff_config,pathLength(handoff_prefix));
            const auto raw_arc=arcLengths(out.route.points);
            const double prefix=raw_arc[out.route.patch_begin];
            const double tail=raw_arc.back()-raw_arc[out.route.patch_end];
            auto patch=routeSlice(out.smooth.points,prefix,pathLength(out.smooth.points)-tail);
            if(!out.route.clearance_near_repair&&!routeHasMargin(g,patch,clearance_contract.reserve-.005,unknown)){
              out.smooth.points=out.route.points;out.smooth.mode="CLEARANCE_CERTIFIED_RAW";
            }
            out.switch_points=out.route.points;
            out.handoff.applied=true;out.handoff.reason=out.route.clearance_near_repair?
              "NEAR_CLEARANCE_JOIN":"CLEARANCE_LOCAL_JOIN";
            out.handoff.prefix_length=prefix;
            out.reserved_gates=reservablePassages(peerPassages(g,full_peer_routes,
              entrance_config.topology,cancelled),start,committed_geometry.back(),entrance_config.topology);
          }
          out.planner=std::move(p);return out;
        }
        if(plan_trigger=="PROFILE_LIMITED_PROGRESS"||plan_trigger=="TURN_EXECUTION_REPAIR"){
          const auto cancelled=[token,epoch]{return token->load()!=epoch;};
          const auto arc=arcLengths(committed_geometry);
          const Vec3 tangent=forwardRouteTangent(committed_geometry,arc,committed_progress);
          // Compare geometric feasibility at aligned yaw. A turn-repair
          // request may originate during alignment; live yaw/safety gates
          // still decide when its candidate can actually be flown.
          if(plan_trigger=="TURN_EXECUTION_REPAIR")
            out.profile=repairTurnExecutionRoute(g,committed_geometry,start,committed_progress,
              speed,profile_tracker,handoff_config,unknown,cancelled);
          else out.profile=repairProfileLimitedRoute(g,committed_geometry,start,committed_progress,
              tangent*speed,std::atan2(tangent.y,tangent.x),profile_tracker,switch_config,
              handoff_config,unknown,cancelled);
          out.peer_epoch=peer_epoch;out.route.reason=out.profile.reason;
          if(out.profile.applied){
            const auto reference=routeSlice(committed_geometry,std::max(0.,committed_progress-.25),pathLength(committed_geometry));
            const double reserve=routeMargin(g,reference,clearance_contract.reserve,unknown);
            if(reserve>.01&&!routeHasMargin(g,out.profile.points,std::max(0.,reserve-.005),unknown)){
              out.profile.applied=false;out.profile.reason="PROFILE_CLEARANCE_REJECTED";
              out.route.reason=out.profile.reason;out.planner=std::move(p);return out;
            }
            out.smooth.points=out.profile.points;out.route.points=out.profile.points;
            out.smooth.mode=out.profile.reason;out.switch_points=out.profile.points;
            out.handoff.applied=true;out.handoff.reason=out.profile.reason;
            out.handoff.prefix_length=out.profile.protected_length;
            out.reserved_gates=reservablePassages(peerPassages(g,full_peer_routes,
              entrance_config.topology,cancelled),start,committed_geometry.back(),entrance_config.topology);
          }
          out.planner=std::move(p);return out;
        }
        if(plan_trigger=="ROUTE_LOCAL_REFRESH"){
          const auto cancelled=[token,epoch]{return token->load()!=epoch;};
          auto shortcut=refreshLocalShortcut(g,committed_geometry,start,committed_progress,
            std::clamp(speed*handoff_config.time,handoff_config.min_prefix,handoff_config.max_prefix),
            clearance_contract.reserve,unknown,cancelled);
          if(shortcut.empty()){
            out.route.points=committed_geometry;out.route.reused=true;out.route.reason="LOCAL_REFRESH_NO_GAIN";
            out.smooth.points=committed_geometry;out.smooth.mode="RETAINED_STRAIGHT_ROUTE";
          }else {
            out.route.points=std::move(shortcut);out.route.reason="LOCAL_REFRESH_SHORTCUT";
            out.smooth=smoothPlannedRoute(g,out.route,cfg,unknown,cancelled,handoff_config,pathLength(handoff_prefix));
            out.handoff.prefix_length=pathLength(handoff_prefix);out.handoff.applied=true;
            out.handoff.reason="LOCAL_REFRESH_JOIN";out.switch_points=out.smooth.points;
            out.reserved_gates=reservablePassages(peerPassages(g,full_peer_routes,
              entrance_config.topology,cancelled),start,committed_geometry.back(),entrance_config.topology);
          }
          out.planner=std::move(p);return out;
        }
        p->clearRoutePenalties();
        p->addRoutePenalty(g,peer_routes,peer_route_separation_,
                           peer_route_penalty_weight_,peer_time_step_);
        out.route=p->plan(g,start,goal,
          incumbent.size()>=2?incumbent[1]-incumbent[0]:Vec3{});
        const auto cancelled=[token,epoch]{return token->load()!=epoch;};
        out.peer_epoch=peer_epoch;
        if(!out.route.points.empty() && !cancelled()){
          auto selection=selectEntranceRoute(g,start,std::move(out.route),pc,
            entrance_config,full_peer_routes,cancelled);
          out.route=std::move(selection.route);out.reserved_gates=std::move(selection.reserved);
          out.candidate_count=selection.evaluated;out.shared_gates=selection.shared;
          out.candidate_ms=selection.elapsed_ms;
          if(out.route.reused && !selection.changed && !switch_bypass && !committed_geometry.empty()){
            // No entrance gain: retain the exact old spline instead of fitting
            // another curve to an identical, freshly trimmed route every tick.
            out.smooth.points=committed_geometry;out.smooth.mode="RETAINED_STRAIGHT_ROUTE";
          }else{
            out.smooth=smoothPlannedRoute(g,out.route,cfg,unknown,cancelled,
              handoff_config,pathLength(handoff_prefix));
            if(selection.changed && sharedPassageCount(out.smooth.points,out.reserved_gates)>out.shared_gates){
              // Physical-map smoothing must not shortcut back through the
              // excluded door. Its checked raw route is the conservative fallback.
              out.smooth.points=out.route.points;out.smooth.mode="DISTINCT_PASSAGE_RAW";
            }
            if(!handoff_prefix.empty()&&!out.smooth.points.empty()&&
               out.route.reason!="RECOVERY_MARGIN_ESCAPE"&&!cancelled()){
              out.switch_points=out.smooth.points;
              out.handoff=connectRetainedPrefix(g,handoff_prefix,out.smooth.points,
                handoff_config,unknown,cancelled);
              if(out.handoff.applied && sharedPassageCount(out.handoff.points,out.reserved_gates)>
                  sharedPassageCount(out.smooth.points,out.reserved_gates)){
                out.handoff.applied=false;out.handoff.reason="HANDOFF_ENTRANCE_CONFLICT";
              }
              if(out.handoff.applied){
                out.smooth.points=out.handoff.points;
                out.smooth.mode+="_"+out.handoff.reason;
              }
            }
          }
        }
        if(!out.route.reused && !out.smooth.points.empty() && !out.route.points.empty()){
          const double reserve=routeMargin(g,out.route.points,clearance_contract.reserve,unknown);
          if(reserve>.01 && !routeHasMargin(g,out.smooth.points,std::max(0.,reserve-.005),unknown)){
            // Reject a lossy splice when moving; keep the incumbent until a
            // certified connection exists. An initial route can use its raw seed.
            if(!handoff_prefix.empty()){out.smooth.points.clear();out.smooth.mode="HANDOFF_CLEARANCE_REJECTED";}
            else {out.smooth.points=out.route.points;out.smooth.mode="CLEARANCE_RAW_FALLBACK";}
          }
        }
        out.planner=std::move(p);return out;
      });
  }
  void publish() {
    const auto input=inputState();
    // A distant invalid tail must not revoke an otherwise usable forward
    // corridor. Trim once, then keep that geometry and its braking endpoint
    // stable while the worker repairs the owned route. Close hazards, stale
    // inputs and the follower's own guards still stop motion immediately.
    RouteMemory checked=route_;
    if(input=="READY" && !path_.empty() &&
       (!checked.valid(*grid_,current_,planner_config_.allow_unknown)||!guardValid(checked)))
      keepSafePrefix();
    if(have_goal_ && approach_pub_){
      std::vector<Vec3> points{goal_};if(approach_.valid)points.push_back(approach_.effective);
      else if(reason_!="GOAL_BLOCKED")points.push_back(goal_);
      approach_pub_->publish(makePath(points,frame_,now()));
    }
    bool changed=path_.size()!=published_geometry_.size();
    if(!changed)for(size_t i=0;i<path_.size();++i)
      if(distance(path_[i],published_geometry_[i])>1e-6){changed=true;break;}
    if(changed || route_stamp_ns_<=0){
      route_stamp_ns_=std::max(route_stamp_ns_+1,now().nanoseconds());published_geometry_=path_;
    }
    auto geometry=makePath(path_,frame_,now());
    geometry.header.stamp=rclcpp::Time(route_stamp_ns_,get_clock()->get_clock_type());
    // Transient input loss keeps RViz geometry and route ownership. Motion is
    // independently gated; cached geometry never certifies a stale map.
    path_pub_->publish(geometry);
    // This is a heartbeat, not a new path. Retain the published curve's arc
    // progress; valid() still checks its current join and every future leg
    // against the latest grid and occupied guard.
    RouteMemory candidate=route_;
    const bool certified=input=="READY" && !path_.empty() &&
      candidate.valid(*grid_,current_,planner_config_.allow_unknown) && guardValid(candidate);
    const auto execution_reason=input!="READY"?input:
      path_.empty()?"NO_ROUTE":certified?"READY":"ROUTE_NOT_CERTIFIED";
    execution_pub_->publish(textMessage(RoutePermit{now().nanoseconds(),route_stamp_ns_,
      certified,execution_reason,goal_,execution_instance_,
      input=="READY"&&(work_.valid()||retry_||repair_authorized_)}.encode()));
    std::ostringstream state;
    state<<reason_<<" input_state="<<input<<" route_revision="<<route_stamp_ns_
      <<" executable="<<certified<<" map_age="<<(have_map_?(now()-map_stamp_).seconds():-1.)
      <<" odom_age="<<(have_odom_?(now()-odom_stamp_).seconds():-1.)
      <<" local_overlay_occ="<<local_overlay_.addedOccupied()
      <<" local_overlay_stamp="<<local_overlay_stamp_ns_
      <<" map_source="<<mapInputName(map_source_)
      <<" primary_map_age="<<(primary_stamp_ns_>0?double(now().nanoseconds()-primary_stamp_ns_)*1e-9:-1.)
      <<" local_map_age="<<(local_pending_?double(now().nanoseconds()-stampNs(local_pending_->header.stamp))*1e-9:-1.)
      <<" map_rx="<<primary_received_<<'/'<<local_received_<<" map_commits="<<map_commits_
      <<" map_rejected="<<map_rejected_<<" map_bytes="<<primary_bytes_<<'/'<<local_bytes_
      <<" published_route_progress="<<route_.progress()
      <<" owned_route_progress="<<owned_route_.progress()<<" "<<detail_;
    status_pub_->publish(textMessage(state.str()));
  }
  std::string frame_, path_topic_, reason_{"WAIT_INPUT"}, detail_;
  std::shared_ptr<Grid> grid_;
  PlannerConfig planner_config_;
  RouteHandoffConfig handoff_config_;
  TrackerConfig profile_tracker_;
  bool profile_repair_requested_{false};
  ClearanceContract clearance_contract_;ClearanceReview clearance_review_;
  SharedEntranceReview entrance_review_;
  PartialRouteRetryGate partial_retry_;
  double measured_speed_{0};
  RouteSwitchConfig route_switch_;
  EntranceConfig entrance_;
  std::chrono::steady_clock::time_point last_topology_review_{};
  std::vector<Vec3> published_geometry_;
  int64_t route_stamp_ns_{0};
  int64_t execution_instance_{std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count()};
  double peer_path_timeout_{3.0},peer_odom_timeout_{1.5};
  double peer_tf_timeout_{0},peer_replan_min_period_{3.0};
  double peer_route_separation_{1.20},peer_nominal_speed_{1.20};
  double peer_time_step_{.30},peer_conflict_horizon_{6.0};
  double peer_conflict_threshold_{.12},peer_route_penalty_weight_{8.0};
  double immediate_repair_distance_{1.50},near_invalidation_confirm_time_{.20};
  double invalidation_confirm_time_{.60};
  size_t near_invalidation_confirm_updates_{2},invalidation_confirm_updates_{2};
  std::unique_ptr<Planner> planner_;
  std::shared_ptr<std::atomic<uint64_t>> goal_epoch_{std::make_shared<std::atomic<uint64_t>>(0)};
  std::future<WorkResult> work_;
  std::chrono::steady_clock::time_point last_plan_{},last_peer_replan_{},last_route_switch_{};
  double replan_period_{.10},search_snapshot_max_age_{1.0};
  std::chrono::steady_clock::time_point last_search_refresh_{};size_t last_refresh_generation_{0},search_refreshes_{0};
  SplineConfig spline_;
  RouteMemory route_,owned_route_;
  PendingRouteSwitch pending_switch_;
  InvalidationGate invalidation_;
  std::string last_invalidation_source_{"NONE"};
  Vec3 last_invalidation_point_{};
  double last_invalidation_distance_{std::numeric_limits<double>::infinity()};
  bool temporary_{false},retry_{false},recovery_route_{false};
  bool repair_authorized_{false},follower_block_reported_{false};
  size_t follower_stall_reports_{0};
  std::string replan_trigger_;
  GoalApproachConfig approach_config_;GoalApproach approach_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr approach_pub_;
  Vec3 current_, goal_;
  std::vector<Vec3> path_,owned_path_;
  bool have_map_{false}, have_odom_{false}, have_goal_{false}, force_{true};
  size_t generation_{0};
  size_t peer_generation_{0},last_planned_peer_generation_{0};
  double map_timeout_, odom_timeout_,timestamp_reset_threshold_{.50};
  int64_t last_occupied_input_stamp_ns_{0},last_free_input_stamp_ns_{0},last_odom_input_stamp_ns_{0};
  rclcpp::Time map_stamp_{0, 0, RCL_ROS_TIME}, odom_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_goal_retry_{0,0,RCL_ROS_TIME};
  MapPairBuffer<sensor_msgs::msg::PointCloud2::SharedPtr> map_pairs_{4};
  bool compact_maps_{true},local_fallback_enabled_{true};
  MapInputSource map_source_{MapInputSource::None};
  int64_t primary_stamp_ns_{-1},local_committed_stamp_ns_{-1};
  int64_t last_primary_input_stamp_ns_{0},last_local_input_stamp_ns_{0};
  size_t primary_received_{0},local_received_{0},map_commits_{0},map_rejected_{0};
  size_t primary_bytes_{0},local_bytes_{0};
  std::shared_ptr<Grid> primary_grid_,local_overlay_grid_;
  LocalOccupiedOverlay local_overlay_;
  int64_t local_overlay_stamp_ns_{-1},last_compact_confirmation_stamp_ns_{-1};
  sensor_msgs::msg::PointCloud2::SharedPtr primary_pending_,local_pending_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr snapshot_sub_,local_snapshot_sub_;
  std::shared_ptr<Grid> occupied_guard_;
  int64_t occupied_guard_stamp_{-1};
  std::vector<PeerRoute> peer_routes_;
  tf2_ros::Buffer tf_;
  tf2_ros::TransformListener listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr occ_sub_, free_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_, rviz_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_, raw_path_pub_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr> peer_path_subs_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> peer_odom_subs_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_,execution_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr replan_sub_;
  rclcpp::TimerBase::SharedPtr timer_, heartbeat_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_;
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafeAirFarLikePlanner>());
  rclcpp::shutdown();
  return 0;
}
