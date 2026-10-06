#include "fire_scout/planner.hpp"
#include "fire_scout/entrance_routes.hpp"
#include "fire_scout/execution_permit.hpp"
#include "fire_scout/goal_approach.hpp"
#include "fire_scout/bspline.hpp"
#include "fire_scout/route_memory.hpp"
#include "fire_scout/route_switch.hpp"
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
    bool allow_unknown, const std::function<bool()> &cancelled) {
  if (!route.locally_patched) {
    return smoothPath(grid, route.points, config, allow_unknown, cancelled);
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
  if (!routeGeometryValid(grid, combined, allow_unknown)) {
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
    declare_parameter<std::string>("runtime_version","2.1.2-straight-route-cooperative");
    frame_ = declare_parameter<std::string>("frame_id", "scout1/odom");
    grid_ = std::make_shared<Grid>(gridParameters(*this));
    approach_config_.enabled=declare_parameter("allow_goal_approach",true);
    approach_config_.max_offset=declare_parameter("goal_approach_max_offset",.35);
    approach_config_.clearance=declare_parameter("goal_approach_clearance",.04);
    if(!approach_config_.valid())throw std::runtime_error("Invalid goal approach limits");
    search_snapshot_max_age_=declare_parameter("search_snapshot_max_age",8.0);
    if(!std::isfinite(search_snapshot_max_age_)||search_snapshot_max_age_<3||search_snapshot_max_age_>30)
      throw std::runtime_error("search_snapshot_max_age must be 3..30 seconds");
    PlannerConfig c;
    c.allow_unknown = declare_parameter("allow_unknown", true);
    c.clearance_weight = declare_parameter("clearance_weight", 4.0);
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
    replan_period_ = declare_parameter("replan_period", .25);
    double replan = replan_period_;
    double heartbeat = declare_parameter("path_publish_period", .2);
    if (replan <= 0 || heartbeat <= 0 || map_timeout_ <= 0 || odom_timeout_ <= 0 ||
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
        goal_ = p;approach_={};last_search_refresh_=std::chrono::steady_clock::now();last_refresh_generation_=generation_;
        have_goal_ = true;
        ++(*goal_epoch_);
        if(planner_)planner_->reset();
        path_.clear(); route_.clear();owned_path_.clear();owned_route_.clear();
        clearInvalidation(false);
        last_invalidation_source_="NONE";last_invalidation_point_={};
        last_invalidation_distance_=std::numeric_limits<double>::infinity();
        temporary_=retry_=recovery_route_=false;
        replan_trigger_.clear();clearPendingSwitch();last_route_switch_={};
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
        }else if(m->data=="STALLED_PROGRESS"){
          follower_stall_reports_++;
          reason_="FOLLOWER_STALL_ROUTE_RETAINED";
          detail_="planner geometry unchanged; follower recovery remains responsible";
        }else if(m->data=="WAITING_FOR_EXECUTABLE_PATH" || m->data=="PASSAGE_WAIT_TIMEOUT"){
          // Wake a bounded retry/review without erasing the route or A* frontier.
          force_=true;last_topology_review_={};
        }else if(m->data=="LOCAL_RECOVERY_FINISHED"){
          reason_="RECOVERY_FINISHED_ROUTE_RETAINED";
        }
      });
    // The executor keeps accepting goals/odometry while bounded search runs on a
    // private snapshot. Poll completion promptly instead of blocking subscriptions.
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this] { plan(); });
    heartbeat_ =
        create_wall_timer(std::chrono::milliseconds(int(1000 * heartbeat)), [this] { publish(); });
    reset_ = create_service<std_srvs::srv::Trigger>(
        "~/reset", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          ++(*goal_epoch_);
          if(planner_)planner_->reset();
          path_.clear(); route_.clear();owned_path_.clear();owned_route_.clear();
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
                "V2.1.2 straight-route planner: tangent-continuous retained route, paired-map invalidation, "
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
    last_invalidation_source_="NONE";last_invalidation_point_={};
    last_invalidation_distance_=std::numeric_limits<double>::infinity();
    temporary_=retry_=recovery_route_=repair_authorized_=false;replan_trigger_.clear();
    map_pairs_.clear();occupied_guard_.reset();occupied_guard_stamp_=-1;
    primary_pending_.reset();local_pending_.reset();primary_grid_.reset();
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
    int64_t stamp_ns{0};
    Vec3 position{};
    int64_t odom_ns{0};
    bool have_position{false};
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
    peer.points = std::move(transformed);
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
          peer.have_position && peer.odom_ns > 0 && now_ns >= peer.odom_ns &&
          double(now_ns - peer.odom_ns) * 1e-9 <= peer_odom_timeout_) {
        auto suffix = routeSuffixAt(peer.points, peer.position);
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
    auto own = routeSuffixAt(route, current_, route_.progress());
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
    auto suffix = routeSuffixAt(owned_path_, current_, owned_route_.progress());
    if (suffix.size() < 2) return;
    path_ = std::move(suffix);
    route_.set(path_);
    temporary_ = distance(path_.back(), approach_.valid ? approach_.effective
                                                        : goal_) > .15;
  }
  void clearInvalidation(bool restore) {
    const bool was_authorized = invalidation_.authorized;
    invalidation_ = InvalidationGate{};
    repair_authorized_ = false;
    if (restore && was_authorized && !owned_path_.empty()) restoreOwnedRoute();
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
    if(!owned_path_.empty())observeInvalidation(inspectOwnedRoute(),true,
      source==MapInputSource::LocalFallback?"LOCAL_FALLBACK_MAP":"ATOMIC_MAP");
  }
  void refreshCompactMap(){
    if(!compact_maps_)return;
    if(primary_pending_){
      auto m=std::move(primary_pending_);const auto stamp=stampNs(m->header.stamp);
      if(stamp>primary_stamp_ns_){
        try{
          auto decoded=parseMapSnapshot(*m,grid_->cfg.resolution);
          auto next=std::make_shared<Grid>(grid_->cfg);
          next->update(std::move(decoded.occupied),std::move(decoded.free));
          primary_grid_=std::move(next);primary_stamp_ns_=stamp;primary_bytes_=m->data.size();
        }catch(const std::runtime_error &e){
          ++map_rejected_;RCLCPP_ERROR(get_logger(),"Primary snapshot: %s",e.what());
        }
      }
    }
    const auto local_stamp=local_pending_?stampNs(local_pending_->header.stamp):-1;
    const auto selected=selectMapInput(primary_stamp_ns_,local_stamp,now().nanoseconds(),
      map_timeout_,local_fallback_enabled_);
    if(selected==MapInputSource::Primary){
      if(map_source_!=selected || map_stamp_.nanoseconds()!=primary_stamp_ns_)
        installSnapshotGrid(primary_grid_,primary_stamp_ns_,selected);
    }else if(selected==MapInputSource::LocalFallback &&
             (map_source_!=selected || local_stamp>local_committed_stamp_ns_)){
      try{
        auto decoded=parseMapSnapshot(*local_pending_,grid_->cfg.resolution);
        auto next=localFallbackGrid(grid_->cfg,std::move(decoded.occupied),
          std::move(decoded.free),primary_grid_.get());
        local_committed_stamp_ns_=local_stamp;local_bytes_=local_pending_->data.size();
        // Retain route ownership and arc progress. A new LOCAL map, with its
        // original source time, must certify geometry before any motion.
        installSnapshotGrid(std::move(next),local_stamp,selected);
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
  };
  void keepSafePrefix(){
    RouteMemory retained=owned_route_;
    auto prefix=retained.safePrefix(*grid_,current_,planner_config_.allow_unknown);
    if(occupied_guard_ && !prefix.empty()){
      RouteMemory guard_prefix;guard_prefix.set(std::move(prefix));
      prefix=guard_prefix.safePrefix(*occupied_guard_,current_,true);
    }
    path_=std::move(prefix);route_.set(path_);temporary_=true;
  }
  void complete(){
    if(!work_.valid() || work_.wait_for(std::chrono::seconds(0))!=std::future_status::ready)return;
    WorkResult r;
    try {r=work_.get();}
    catch(const std::exception&e){
      planner_=std::make_unique<Planner>(planner_config_);
      if(!owned_path_.empty())
        planner_->setCommittedRoute(routeSuffixAt(
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
        routeSuffixAt(owned_path_,current_,owned_route_.progress()));
      force_=true;reason_=inputState();return;
    }
    // Snapshot search may outlive a map update. Certify both the proposed
    // geometry and the owned route from the current position on the latest map.
    RouteMemory candidate;
    if(r.smooth.mode=="RETAINED_STRAIGHT_ROUTE"){
      // This worker returned the exact incumbent geometry. Its arc origin
      // did not move, so a new progress=0 memory would reject a vehicle that
      // has already flown beyond the initial 2 m projection window.
      candidate=owned_route_;
    }else candidate.set(r.smooth.points);
    const bool candidate_valid=!r.smooth.points.empty() &&
      candidate.valid(*grid_,current_,planner_config_.allow_unknown) && guardValid(candidate);
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
      const auto challenger_suffix=routeSuffixFrom(r.smooth.points,current_);
      const bool entrance_improved=entrance_.topology.enabled && r.peer_epoch==peer_generation_ &&
        sharedPassageCount(incumbent_suffix,r.reserved_gates)>
          sharedPassageCount(challenger_suffix,r.reserved_gates);
      if(!incumbent_valid){
        clearPendingSwitch();switch_decision=owned_path_.empty()?"INITIAL_ROUTE":"REPAIR_INVALID_ROUTE";
      }else if(r.switch_bypass){
        clearPendingSwitch();switch_decision="COMMIT_AUTHORIZED_REPAIR";
      }else if(entrance_improved){
        // A bounded different-door route may be longer. Ordinary length-only
        // hysteresis would reject every useful separation. Confirm its heading
        // and geometry over successive reviews before replacing the route.
        switch_assessment=assessRouteSwitch(incumbent_suffix,challenger_suffix,current_,
          stableRouteCost(*grid_,incumbent_suffix),stableRouteCost(*grid_,challenger_suffix),route_switch_);
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
        const auto challenger=routeSuffixFrom(r.smooth.points,current_);
        const double old_total=stableRouteCost(*grid_,incumbent);
        const double new_total=stableRouteCost(*grid_,challenger);
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
    const bool committed=candidate_valid && switch_allowed;
    if(committed){
      const bool replaced=incumbent_valid && !owned_path_.empty();
      path_=std::move(r.smooth.points);route_=std::move(candidate);
      owned_path_=path_;owned_route_=route_;
      temporary_=distance(path_.back(),approach_.valid?approach_.effective:goal_)>.15;
      retry_=false;
      recovery_route_=r.route.reason=="RECOVERY_MARGIN_ESCAPE";
      planner_->setCommittedRoute(r.smooth.mode=="RETAINED_STRAIGHT_ROUTE"
        ?routeSuffixAt(path_,current_,route_.progress()):path_);
      clearInvalidation(false);
      follower_block_reported_=false;
      if(replaced)last_route_switch_=wall;
      reason_=r.route.reason;
      force_=false;
    }else if(incumbent_valid){
      // A rejected/obsolete result must never truncate the still-valid owned
      // route. The worker may have adopted it internally, so restore ownership.
      restoreOwnedRoute();
      planner_->setCommittedRoute(path_);
      retry_=false;force_=false;
      if(recovered_during_search)clearInvalidation(false);
      if(switch_decision.empty()){
        clearPendingSwitch();switch_decision="KEEP_ROUTE_CANDIDATE_INVALID";
      }
      reason_=switch_decision;
      r.smooth.mode="RETAINED_INCUMBENT";
    }else{
      clearPendingSwitch();keepSafePrefix();retry_=true;force_=true;
      reason_=r.smooth.points.empty()?r.route.reason:"RESULT_INVALIDATED_RETRY";
      r.smooth.mode=path_.empty()?"EMPTY_RETRY":"SAFE_PREFIX_RETRY";
      if(switch_decision.empty())switch_decision="NO_VALID_ROUTE";
    }
    raw_path_pub_->publish(makePath(candidate_valid?r.route.points:std::vector<Vec3>{},frame_,now()));
    std::ostringstream os;
    os<<"goal_epoch="<<r.epoch<<" reused="<<r.route.reused<<" ms="<<r.route.elapsed_ms
      <<" smoothing="<<r.smooth.mode<<" spline_spans="<<r.smooth.spans
      <<" spline_control_spacing="<<r.smooth.control_spacing<<" spline_deviation="<<r.smooth.deviation
      <<" spline_candidate_valid="<<candidate_valid<<" route_committed="<<committed
      <<" spline_optimized="<<(candidate_valid&&r.smooth.optimized)
      <<" spline_opt_iterations="<<r.smooth.optimization_iterations
      <<" spline_opt_cost_before="<<r.smooth.optimization_cost_before
      <<" spline_opt_cost_after="<<r.smooth.optimization_cost_after
      <<" radius="<<grid_->cfg.inflation_xy<<" expanded="<<r.route.expansions
      <<" flat_expanded="<<r.route.flat_expansions<<" spatial_expanded="<<r.route.spatial_expansions
      <<" search_mode="<<r.route.search_mode<<" flat_altitude="<<r.route.flat_altitude
      <<" length="<<pathLength(path_)<<" target="<<goal_.x<<","<<goal_.y<<","<<goal_.z
      <<" candidates="<<r.candidate_count<<" candidate_ms="<<r.candidate_ms
      <<" shared_entrances="<<sharedPassageCount(path_,r.reserved_gates)
      <<" local_patch="<<r.route.locally_patched
      <<" patch_begin="<<r.route.patch_begin<<" patch_end="<<r.route.patch_end
      <<" switch_trigger="<<r.trigger<<" switch_decision="<<switch_decision;
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
        path_.clear();route_.clear();owned_path_.clear();owned_route_.clear();
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
    const bool topology_review=entrance_.topology.enabled && !full_peer_routes.empty() &&
      (pending_switch_.active || last_topology_review_.time_since_epoch().count()==0 ||
       std::chrono::duration<double>(wall-last_topology_review_).count()>=entrance_.topology.refresh_period);
    // An unfinished search uses a frozen snapshot. A persistent mission must
    // not need a new goal message to refresh it. Refresh after a bounded age,
    // only when no route exists AND new map evidence has arrived.
    if(owned_path_.empty()&&planner_&&generation_!=last_refresh_generation_&&
       std::chrono::duration<double>(wall-last_search_refresh_).count()>search_snapshot_max_age_){
      if(planner_->refreshChangedSearches(*grid_)>0) {
        ++search_refreshes_;
      }
      last_search_refresh_=wall;
      last_refresh_generation_=generation_;
    }
    auto inspection=inspectOwnedRoute();
    const bool old_valid=!owned_path_.empty()&&inspection.valid;
    if(!old_valid && !owned_path_.empty())observeInvalidation(inspection,false,"PLAN_CHECK");
    if(follower_block_reported_ && old_valid){
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
    if(old_valid && !finished_partial && !peer_conflict &&
       !pending_switch_.active && !topology_review && planner_config_.lock_valid_route){
      reason_=temporary_?"KEEP_RECOVERY_ROUTE":"KEEP_VALID_SPLINE";
      std::ostringstream kept;
      kept<<"switch_decision=KEEP_COMMITTED_ROUTE route_remaining="
          <<owned_route_.remaining()<<" path_length="<<pathLength(owned_path_);
      detail_=kept.str();
      retry_=force_=false;last_plan_=wall;return;
    }
    std::string plan_trigger;
    if(!replan_trigger_.empty())plan_trigger=replan_trigger_;
    else if(pending_switch_.active)plan_trigger=pending_switch_.trigger;
    else if(topology_review)plan_trigger="ENTRANCE_REVIEW";
    else if(peer_conflict)plan_trigger="PEER_ROUTE_CONFLICT";
    else if(finished_partial)plan_trigger="PARTIAL_ROUTE_FINISHED";
    else if(repair_authorized_)plan_trigger=replan_trigger_.empty()?"CONFIRMED_ROUTE_INVALID":replan_trigger_;
    else if(retry_)plan_trigger="ROUTE_RETRY";
    else plan_trigger="ROUTE_REFRESH";
    const bool safety_repair=repair_authorized_;
    const bool switch_bypass=owned_path_.empty() || safety_repair || finished_partial;
    if((safety_repair || finished_partial) && planner_)planner_->requestRepair();
    replan_trigger_.clear();force_=false;last_plan_=wall;
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
    reason_="PLANNING";
    work_=std::async(std::launch::async,
      [this,p=std::move(p),snapshot=std::move(snapshot),start,goal,cfg,unknown,
       token,epoch,peer_routes,full_peer_routes,incumbent,switch_bypass,safety_repair,
       plan_trigger,entrance_config,pc,committed_geometry,peer_epoch]()mutable{
        const Grid &g=*snapshot;
        WorkResult out;out.epoch=epoch;out.switch_bypass=switch_bypass;
        out.safety_repair=safety_repair;out.trigger=plan_trigger;
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
            out.smooth=smoothPlannedRoute(g,out.route,cfg,unknown,cancelled);
            if(selection.changed && sharedPassageCount(out.smooth.points,out.reserved_gates)>out.shared_gates){
              // Physical-map smoothing must not shortcut back through the
              // excluded door. Its checked raw route is the conservative fallback.
              out.smooth.points=out.route.points;out.smooth.mode="DISTINCT_PASSAGE_RAW";
            }
          }
        }
        out.planner=std::move(p);return out;
      });
  }
  void publish() {
    const auto input=inputState();
    if(have_goal_ && approach_pub_){
      std::vector<Vec3> points{goal_};if(approach_.valid)points.push_back(approach_.effective);
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
      certified,execution_reason,goal_,execution_instance_}.encode()));
    std::ostringstream state;
    state<<reason_<<" input_state="<<input<<" route_revision="<<route_stamp_ns_
      <<" executable="<<certified<<" map_age="<<(have_map_?(now()-map_stamp_).seconds():-1.)
      <<" odom_age="<<(have_odom_?(now()-odom_stamp_).seconds():-1.)
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
  double replan_period_{.25},search_snapshot_max_age_{8.0};
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
  std::shared_ptr<Grid> primary_grid_;
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
