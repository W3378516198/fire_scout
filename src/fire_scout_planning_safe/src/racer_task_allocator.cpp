#include "fire_scout/racer_task_allocator.hpp"
#include "fire_scout/ros_utils.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

using namespace fire_scout;

// The executable name is kept for launch compatibility.  V19 is no longer an
// exploration task allocator: it dispatches one final arrival slot per UAV and
// leaves complete-route generation and local repair to each local planner.
class RacerTaskAllocator : public rclcpp::Node {
public:
  RacerTaskAllocator()
      : Node("racer_task_allocator"), tf_(get_clock()), listener_(tf_) {
    declare_parameter<std::string>("runtime_version", "19.1-arrival-slot-coordinator");
    world_frame_ = declare_parameter<std::string>("world_frame", "world");
    mission_goal_topic_ =
        declare_parameter<std::string>("fire_goal_topic", "/fire_goal");
    rviz_goal_topic_ =
        declare_parameter<std::string>("rviz_fire_goal_topic", "/goal_pose");
    enabled_ = declare_parameter("enabled", false);
    require_all_robots_ = declare_parameter("require_all_robots", true);
    map_timeout_ = declare_parameter("map_timeout", 5.0);
    odom_timeout_ = declare_parameter("odom_timeout", 1.5);
    tf_timeout_ = declare_parameter("tf_wait_timeout", 0.20);
    goal_reached_distance_ =
        declare_parameter("goal_reached_distance", 0.35);
    goal_update_distance_ = declare_parameter("goal_update_distance", 0.20);
    goal_refresh_period_ = declare_parameter("goal_refresh_period", 2.0);
    planner_failure_delay_ =
        declare_parameter("planner_failure_delay", 4.0);
    blocked_path_timeout_ =
        declare_parameter("blocked_path_timeout", 12.0);
    blocked_slot_radius_ =
        declare_parameter("blocked_slot_radius", 0.45);
    slot_invalid_confirm_time_ =
        declare_parameter("terminal_slot_invalid_confirm_time", 5.0);
    const double rate = declare_parameter("assignment_rate", 1.0);

    slot_config_.resolution = declare_parameter("voxel_size", 0.15);
    slot_config_.radius = declare_parameter("terminal_slot_radius", 1.0);
    slot_config_.max_radius =
        declare_parameter("terminal_slot_max_radius", 2.0);
    slot_config_.radius_step =
        declare_parameter("terminal_slot_radius_step", 0.35);
    slot_config_.min_separation =
        declare_parameter("terminal_min_separation", 0.90);
    slot_config_.clearance_xy =
        declare_parameter("terminal_clearance_xy", 0.48);
    slot_config_.clearance_z =
        declare_parameter("terminal_clearance_z", 0.25);
    slot_config_.z_min = declare_parameter("z_min", 0.65);
    slot_config_.z_max = declare_parameter("z_max", 3.0);
    slot_config_.angular_samples = static_cast<size_t>(
        declare_parameter<int64_t>("terminal_angular_samples", 16));
    slot_config_.include_exact_goal =
        declare_parameter("terminal_include_exact_goal", true);

    const auto names = declare_parameter<std::vector<std::string>>(
        "scout_names", {"scout1", "scout2", "scout3", "scout4"});
    const auto frames = declare_parameter<std::vector<std::string>>(
        "scout_frames",
        {"scout1/odom", "scout2/odom", "scout3/odom", "scout4/odom"});
    const auto odom_topics = declare_parameter<std::vector<std::string>>(
        "odom_topics", {"/scout1/odom", "/scout2/odom", "/scout3/odom",
                        "/scout4/odom"});
    const auto goal_topics = declare_parameter<std::vector<std::string>>(
        "goal_topics", {"/scout1/mission/fire_goal", "/scout2/mission/fire_goal",
                        "/scout3/mission/fire_goal", "/scout4/mission/fire_goal"});
    const auto status_topics = declare_parameter<std::vector<std::string>>(
        "planner_status_topics",
        {"/scout1/planning/status", "/scout2/planning/status",
         "/scout3/planning/status", "/scout4/planning/status"});

    if (world_frame_.empty() || mission_goal_topic_.empty() ||
        !slot_config_.valid() || !std::isfinite(map_timeout_) ||
        map_timeout_ <= 0 || !std::isfinite(odom_timeout_) ||
        odom_timeout_ <= 0 || !std::isfinite(tf_timeout_) || tf_timeout_ < 0 ||
        !std::isfinite(goal_reached_distance_) || goal_reached_distance_ <= 0 ||
        !std::isfinite(goal_update_distance_) || goal_update_distance_ < 0 ||
        !std::isfinite(goal_refresh_period_) || goal_refresh_period_ < .25 ||
        !std::isfinite(planner_failure_delay_) || planner_failure_delay_ < 0 ||
        !std::isfinite(blocked_path_timeout_) || blocked_path_timeout_ < 1.0 ||
        !std::isfinite(blocked_slot_radius_) || blocked_slot_radius_ <= 0 ||
        !std::isfinite(slot_invalid_confirm_time_) ||
        slot_invalid_confirm_time_ < 0 || slot_invalid_confirm_time_ > 10.0 ||
        !std::isfinite(rate) || rate <= 0 || rate > 20 || names.empty())
      throw std::runtime_error("Invalid route coordinator parameters");
    const size_t count = names.size();
    if (frames.size() != count || odom_topics.size() != count ||
        goal_topics.size() != count || status_topics.size() != count)
      throw std::runtime_error(
          "All per-scout coordinator arrays must have scout_names length");

    std::unordered_set<std::string> unique;
    robots_.resize(count);
    for (size_t i = 0; i < count; ++i) {
      if (names[i].empty() || frames[i].empty() || odom_topics[i].empty() ||
          goal_topics[i].empty() || status_topics[i].empty() ||
          !unique.insert(names[i]).second || !unique.insert(frames[i]).second ||
          !unique.insert(odom_topics[i]).second ||
          !unique.insert(goal_topics[i]).second ||
          !unique.insert(status_topics[i]).second)
        throw std::runtime_error(
            "Coordinator names, frames and topics must be non-empty and unique");
      auto &robot = robots_[i];
      robot.name = names[i];
      robot.frame = frames[i];
      robot.goal_pub = create_publisher<geometry_msgs::msg::PoseStamped>(
          goal_topics[i], rclcpp::QoS(10).reliable());
      robot.odom_sub = create_subscription<nav_msgs::msg::Odometry>(
          odom_topics[i], rclcpp::SensorDataQoS(),
          [this, i](nav_msgs::msg::Odometry::SharedPtr msg) {
            receiveOdom(i, *msg);
          });
      robot.status_sub = create_subscription<std_msgs::msg::String>(
          status_topics[i], rclcpp::QoS(10).reliable(),
          [this, i](std_msgs::msg::String::SharedPtr msg) {
            receivePlannerStatus(i, msg->data);
          });
    }

    mission_goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        mission_goal_topic_, rclcpp::QoS(10).reliable(),
        [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          receiveMissionGoal(*msg);
        });
    if (!rviz_goal_topic_.empty() && rviz_goal_topic_ != mission_goal_topic_)
      rviz_goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
          rviz_goal_topic_, rclcpp::QoS(10).reliable(),
          [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            receiveMissionGoal(*msg);
          });

    occupied_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        declare_parameter<std::string>("occupied_topic",
                                       "/fire_scout/map/fused_occupied"),
        rclcpp::QoS(1).reliable().transient_local(),
        [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          receiveMap(*msg);
        });
    candidate_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        declare_parameter<std::string>("candidate_goal_topic",
                                       "/fire_scout/racer/candidate_goals"),
        rclcpp::QoS(1).reliable().transient_local());
    assigned_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        declare_parameter<std::string>("assigned_topic",
                                       "/fire_scout/racer/assigned_goals"),
        rclcpp::QoS(1).reliable().transient_local());
    status_pub_ = create_publisher<std_msgs::msg::String>(
        declare_parameter<std::string>("status_topic",
                                       "/fire_scout/racer/status"),
        rclcpp::QoS(1).reliable().transient_local());

    enable_service_ = create_service<std_srvs::srv::SetBool>(
        "~/set_enabled",
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
               std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
          enabled_ = request->data;
          force_assignment_ = enabled_;
          response->success = true;
          response->message = enabled_
              ? "Route coordination enabled; final arrival goals will be dispatched."
              : "Route coordination disabled; already published goals are retained.";
          publishStatus();
        });
    replan_service_ = create_service<std_srvs::srv::Trigger>(
        "~/replan",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          clearAssignments(false);
          force_assignment_ = true;
          response->success = true;
          response->message =
              "Arrival assignments cleared; routes will be selected again.";
        });
    reset_service_ = create_service<std_srvs::srv::Trigger>(
        "~/reset",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          clearAssignments(true);
          blocked_slots_.clear();
          force_assignment_ = true;
          response->success = true;
          response->message =
              "Arrival assignments, completion state and blocked slots cleared.";
        });
    clear_goal_service_ = create_service<std_srvs::srv::Trigger>(
        "~/clear_fire_goal",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          have_goal_ = false;
          goal_ns_ = 0;
          slots_.clear();
          blocked_slots_.clear();
          clearAssignments(true);
          publishClouds();
          response->success = true;
          response->message = "Shared mission goal cleared.";
        });

    timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / rate)),
        [this] { tick(); });
    config_lock_ = lockParameters(*this);
    RCLCPP_INFO(get_logger(),
                "V19.1 arrival-slot coordinator ready: scouts=%zu world=%s enabled=%s "
                "terminal_radius=%.2f..%.2f separation=%.2f slot_hold=%.2fs",
                robots_.size(), world_frame_.c_str(),
                enabled_ ? "true" : "false", slot_config_.radius,
                slot_config_.max_radius, slot_config_.min_separation,
                slot_invalid_confirm_time_);
  }

private:
  struct Assignment {
    bool active{false};
    int64_t slot_id{0};
    Vec3 target{};
    int64_t assigned_ns{0};
    int64_t last_publish_ns{0};
  };
  struct Robot {
    std::string name;
    std::string frame;
    Vec3 position{};
    int64_t odom_ns{0};
    std::string planner_status;
    int64_t status_ns{0};
    int64_t no_path_since_ns{0};
    int64_t slot_missing_since_ns{0};
    bool mission_complete{false};
    Assignment assignment;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub;
  };
  struct BlockedSlot {
    Vec3 center{};
    double radius{0};
  };

  static builtin_interfaces::msg::Time timeMessage(int64_t stamp_ns) {
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<int32_t>(stamp_ns / 1000000000LL);
    stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1000000000LL);
    return stamp;
  }
  static tf2::Transform toTransform(
      const geometry_msgs::msg::TransformStamped &msg) {
    const auto &q = msg.transform.rotation;
    tf2::Quaternion rotation(q.x, q.y, q.z, q.w);
    if (rotation.length2() < 1e-12)
      throw std::runtime_error("Invalid TF quaternion");
    rotation.normalize();
    const auto &p = msg.transform.translation;
    return tf2::Transform(rotation, tf2::Vector3(p.x, p.y, p.z));
  }
  geometry_msgs::msg::TransformStamped lookup(
      const std::string &target, const std::string &source,
      const builtin_interfaces::msg::Time &stamp) {
    if (target == source) {
      geometry_msgs::msg::TransformStamped identity;
      identity.header.frame_id = target;
      identity.child_frame_id = source;
      identity.header.stamp = stamp;
      identity.transform.rotation.w = 1.0;
      return identity;
    }
    return tf_.lookupTransform(
        target, source, rclcpp::Time(stamp, get_clock()->get_clock_type()),
        rclcpp::Duration::from_seconds(tf_timeout_));
  }
  bool fresh(int64_t stamp, double timeout, int64_t now_ns) const {
    return stamp > 0 && now_ns >= stamp &&
           double(now_ns - stamp) * 1e-9 <= timeout;
  }
  void clearAssignments(bool clear_completion) {
    for (auto &robot : robots_) {
      robot.assignment = Assignment{};
      robot.no_path_since_ns = 0;
      robot.slot_missing_since_ns = 0;
      if (clear_completion) robot.mission_complete = false;
    }
  }
  void receiveOdom(size_t index, const nav_msgs::msg::Odometry &msg) {
    if (msg.header.frame_id.empty()) return;
    const Vec3 local{msg.pose.pose.position.x, msg.pose.pose.position.y,
                     msg.pose.pose.position.z};
    if (!finite(local)) return;
    try {
      const auto transform =
          toTransform(lookup(world_frame_, msg.header.frame_id, msg.header.stamp));
      const auto point = transform * tf2::Vector3(local.x, local.y, local.z);
      const Vec3 world{point.x(), point.y(), point.z()};
      if (!finite(world)) return;
      robots_.at(index).position = world;
      robots_.at(index).odom_ns = now().nanoseconds();
    } catch (const std::exception &error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                           "Waiting for %s odometry TF: %s",
                           robots_.at(index).name.c_str(), error.what());
    }
  }
  void receivePlannerStatus(size_t index, const std::string &status) {
    auto &robot = robots_.at(index);
    robot.planner_status = status;
    robot.status_ns = now().nanoseconds();
    if (!robot.assignment.active) {
      robot.no_path_since_ns = 0;
      return;
    }
    if (status.rfind("NO_PATH_RETRY", 0) == 0 ||
        status.rfind("GOAL_BLOCKED", 0) == 0) {
      if (robot.no_path_since_ns == 0) robot.no_path_since_ns = robot.status_ns;
    } else if (status.rfind("PLANNING", 0) != 0 &&
               status.rfind("SEARCH_BUDGET_RETRY", 0) != 0) {
      robot.no_path_since_ns = 0;
    }
  }
  void receiveMissionGoal(const geometry_msgs::msg::PoseStamped &msg) {
    if (msg.header.frame_id.empty()) {
      RCLCPP_WARN(get_logger(), "Rejected mission goal with empty frame_id");
      return;
    }
    const Vec3 local{msg.pose.position.x, msg.pose.position.y,
                     msg.pose.position.z};
    if (!finite(local)) return;
    try {
      Vec3 world = local;
      if (msg.header.frame_id != world_frame_) {
        const auto transform =
            toTransform(lookup(world_frame_, msg.header.frame_id, msg.header.stamp));
        const auto point = transform * tf2::Vector3(local.x, local.y, local.z);
        world = {point.x(), point.y(), point.z()};
      }
      if (!finite(world) || world.z < slot_config_.z_min ||
          world.z > slot_config_.z_max)
        throw std::runtime_error("mission goal is outside altitude bounds");
      const bool changed = !have_goal_ ||
                           distance(world, mission_goal_) > goal_update_distance_;
      mission_goal_ = world;
      goal_ns_ = now().nanoseconds();
      have_goal_ = true;
      if (changed) {
        clearAssignments(true);
        blocked_slots_.clear();
        force_assignment_ = true;
        map_changed_ = true;
      }
      RCLCPP_INFO(get_logger(), "Mission goal %s world=[%.2f %.2f %.2f]",
                  changed ? "accepted" : "refreshed", world.x, world.y,
                  world.z);
    } catch (const std::exception &error) {
      RCLCPP_WARN(get_logger(), "Rejected mission goal: %s", error.what());
    }
  }
  void receiveMap(const sensor_msgs::msg::PointCloud2 &msg) {
    if (msg.header.frame_id != world_frame_) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000,
                            "Coordinator requires occupied map in %s (got %s)",
                            world_frame_.c_str(), msg.header.frame_id.c_str());
      return;
    }
    try {
      occupied_ = cloudSet(msg, slot_config_.resolution);
      map_ns_ = now().nanoseconds();
      have_map_ = true;
      map_changed_ = true;
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(get_logger(), "Rejected occupied map: %s", error.what());
    }
  }
  bool blocked(Vec3 point) const {
    for (const auto &zone : blocked_slots_)
      if (distance(point, zone.center) <= zone.radius) return true;
    return false;
  }
  void refreshSlots() {
    slots_.clear();
    if (!have_goal_) {
      publishClouds();
      return;
    }
    const size_t requested = std::max<size_t>(robots_.size() * 4, 12);
    auto candidates = makeArrivalSlots(occupied_, mission_goal_, requested,
                                       slot_config_);
    for (const auto &slot : candidates)
      if (!blocked(slot.target)) slots_.push_back(slot);
    map_changed_ = false;
    publishClouds();
  }
  bool publishGoal(Robot &robot, const ArrivalSlot &slot, int64_t now_ns) {
    try {
      const auto stamp = timeMessage(now_ns);
      const auto transform = toTransform(lookup(robot.frame, world_frame_, stamp));
      const auto point = transform *
          tf2::Vector3(slot.target.x, slot.target.y, slot.target.z);
      geometry_msgs::msg::PoseStamped goal;
      goal.header.stamp = stamp;
      goal.header.frame_id = robot.frame;
      goal.pose.position.x = point.x();
      goal.pose.position.y = point.y();
      goal.pose.position.z = point.z();
      goal.pose.orientation.w = 1.0;
      robot.goal_pub->publish(goal);
      const bool same_assignment=robot.assignment.active &&
          robot.assignment.slot_id == slot.id;
      const int64_t assigned_ns = same_assignment
                                      ? robot.assignment.assigned_ns
                                      : now_ns;
      robot.assignment = {true, slot.id, slot.target, assigned_ns, now_ns};
      if(!same_assignment){
        robot.no_path_since_ns = 0;
        robot.slot_missing_since_ns = 0;
      }
      RCLCPP_INFO(get_logger(),
                  "%s final slot for %s world=[%.2f %.2f %.2f] "
                  "local=[%.2f %.2f %.2f]",
                  same_assignment?"Refreshed":"Assigned",robot.name.c_str(),
                  slot.target.x, slot.target.y,
                  slot.target.z, point.x(), point.y(), point.z());
      return true;
    } catch (const std::exception &error) {
      RCLCPP_WARN(get_logger(), "Cannot publish %s final goal: %s",
                  robot.name.c_str(), error.what());
      return false;
    }
  }
  void updateAssignments(int64_t now_ns) {
    for (auto &robot : robots_) {
      if (!robot.assignment.active ||
          !fresh(robot.odom_ns, odom_timeout_, now_ns))
        continue;
      if (distance(robot.position, robot.assignment.target) <=
          goal_reached_distance_) {
        robot.mission_complete = true;
        robot.assignment = Assignment{};
        robot.no_path_since_ns = 0;
        robot.slot_missing_since_ns = 0;
        RCLCPP_INFO(get_logger(), "%s reached its final arrival slot",
                    robot.name.c_str());
        force_assignment_ = true;
        continue;
      }
      const bool immediate_goal_failure =
          robot.planner_status.rfind("GOAL_BLOCKED", 0) == 0 &&
          robot.status_ns > robot.assignment.assigned_ns &&
          double(now_ns - robot.assignment.assigned_ns) * 1e-9 >=
              planner_failure_delay_;
      const bool persistent_no_path = robot.no_path_since_ns > 0 &&
          now_ns >= robot.no_path_since_ns &&
          double(now_ns - robot.no_path_since_ns) * 1e-9 >=
              blocked_path_timeout_;
      if (immediate_goal_failure || persistent_no_path) {
        blocked_slots_.push_back(
            {robot.assignment.target, blocked_slot_radius_});
        RCLCPP_WARN(get_logger(),
                    "%s rejected final slot; selecting another endpoint",
                    robot.name.c_str());
        robot.assignment = Assignment{};
        robot.no_path_since_ns = 0;
        robot.slot_missing_since_ns = 0;
        force_assignment_ = true;
        map_changed_ = true;
      }
    }
  }
  void allocate(int64_t now_ns) {
    refreshSlots();
    std::unordered_set<int64_t> reserved;
    std::vector<Vec3> reserved_targets;
    for (auto &robot : robots_) {
      if (!robot.assignment.active) continue;
      auto found = std::find_if(slots_.begin(), slots_.end(),
          [&](const ArrivalSlot &slot) {
            return slot.id == robot.assignment.slot_id;
          });
      if (found == slots_.end()) {
        if(robot.slot_missing_since_ns==0){
          robot.slot_missing_since_ns=now_ns;
          RCLCPP_WARN(get_logger(),
            "%s final slot is absent from one map snapshot; retaining it for %.2fs",
            robot.name.c_str(),slot_invalid_confirm_time_);
        }
        if(!slotMissingConfirmed(robot.slot_missing_since_ns,now_ns,
                                 slot_invalid_confirm_time_)){
          reserved_targets.push_back(robot.assignment.target);
          if(double(now_ns-robot.assignment.last_publish_ns)*1e-9>=goal_refresh_period_){
            const ArrivalSlot retained{robot.assignment.slot_id,
                                       robot.assignment.target};
            publishGoal(robot,retained,now_ns);
          }
          continue;
        }
        RCLCPP_WARN(get_logger(),
          "%s final slot remained invalid for %.2fs; selecting another endpoint",
          robot.name.c_str(),slot_invalid_confirm_time_);
        robot.assignment = Assignment{};
        robot.no_path_since_ns=0;
        robot.slot_missing_since_ns=0;
        continue;
      }
      if(robot.slot_missing_since_ns>0)
        RCLCPP_INFO(get_logger(),"%s final slot recovered before reassignment",
                    robot.name.c_str());
      robot.slot_missing_since_ns=0;
      reserved.insert(found->id);
      reserved_targets.push_back(found->target);
      if (double(now_ns - robot.assignment.last_publish_ns) * 1e-9 >=
          goal_refresh_period_)
        publishGoal(robot, *found, now_ns);
    }

    struct Choice { size_t robot; size_t slot; double cost; };
    std::vector<Choice> choices;
    for (size_t r = 0; r < robots_.size(); ++r) {
      const auto &robot = robots_[r];
      if (robot.assignment.active || robot.mission_complete ||
          !fresh(robot.odom_ns, odom_timeout_, now_ns))
        continue;
      for (size_t s = 0; s < slots_.size(); ++s)
        if (!reserved.count(slots_[s].id) &&
            !conflictsWithReservedSlot(slots_[s].target,reserved_targets,
                                       slot_config_.min_separation))
          choices.push_back({r, s, distance(robot.position, slots_[s].target)});
    }
    std::sort(choices.begin(), choices.end(), [](const Choice &a,
                                                  const Choice &b) {
      if (a.cost != b.cost) return a.cost < b.cost;
      if (a.robot != b.robot) return a.robot < b.robot;
      return a.slot < b.slot;
    });
    std::unordered_set<size_t> assigned_robots;
    for (const auto &choice : choices) {
      if (assigned_robots.count(choice.robot) ||
          reserved.count(slots_[choice.slot].id) ||
          conflictsWithReservedSlot(slots_[choice.slot].target,reserved_targets,
                                    slot_config_.min_separation))
        continue;
      if (publishGoal(robots_[choice.robot], slots_[choice.slot], now_ns)) {
        assigned_robots.insert(choice.robot);
        reserved.insert(slots_[choice.slot].id);
        reserved_targets.push_back(slots_[choice.slot].target);
      }
    }
    force_assignment_ = false;
    publishClouds();
  }
  void publishClouds() {
    KeySet candidates, assigned;
    for (const auto &slot : slots_)
      candidates.insert(key(slot.target, slot_config_.resolution));
    for (const auto &robot : robots_)
      if (robot.assignment.active)
        assigned.insert(key(robot.assignment.target, slot_config_.resolution));
    const auto stamp = timeMessage(now().nanoseconds());
    candidate_pub_->publish(makeCloud(candidates, slot_config_.resolution,
                                      world_frame_, stamp));
    assigned_pub_->publish(makeCloud(assigned, slot_config_.resolution,
                                     world_frame_, stamp));
  }
  void publishStatus() {
    const int64_t now_ns = now().nanoseconds();
    size_t fresh_robots = 0, assigned = 0, completed = 0, held_slots = 0;
    for (const auto &robot : robots_) {
      fresh_robots += fresh(robot.odom_ns, odom_timeout_, now_ns);
      assigned += robot.assignment.active;
      completed += robot.mission_complete;
      held_slots += robot.slot_missing_since_ns > 0;
    }
    std::ostringstream status;
    status << "enabled=" << enabled_ << " mode=route_coordinated"
           << " goal=" << (have_goal_ ? "set" : "unset")
           << " map=" << (fresh(map_ns_, map_timeout_, now_ns) ? "fresh" : "stale")
           << " robots=" << fresh_robots << '/' << robots_.size()
           << " terminal_candidates=" << slots_.size()
           << " assigned=" << assigned << " reached=" << completed
           << " held_slots=" << held_slots
           << " blocked_slots=" << blocked_slots_.size()
           << " tasks=";
    for (size_t i = 0; i < robots_.size(); ++i) {
      if (i) status << ',';
      const auto &robot = robots_[i];
      status << robot.name << ':';
      if (robot.assignment.active)
        status << "route@" << robot.assignment.target.x << '/'
               << robot.assignment.target.y << '/'
               << robot.assignment.target.z;
      else if (robot.mission_complete)
        status << "goal_reached";
      else
        status << "idle";
    }
    status_pub_->publish(textMessage(status.str()));
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "%s",
                         status.str().c_str());
  }
  void tick() {
    const int64_t now_ns = now().nanoseconds();
    updateAssignments(now_ns);
    size_t fresh_robots = 0;
    for (const auto &robot : robots_)
      fresh_robots += fresh(robot.odom_ns, odom_timeout_, now_ns);
    const bool robot_gate = require_all_robots_
                                ? fresh_robots == robots_.size()
                                : fresh_robots > 0;
    const bool map_fresh = have_map_ && fresh(map_ns_, map_timeout_, now_ns);
    if (have_goal_ && map_fresh && map_changed_ && (!enabled_ || !robot_gate))
      refreshSlots();
    if (enabled_ && have_goal_ && map_fresh && robot_gate &&
        (map_changed_ || force_assignment_))
      allocate(now_ns);
    else if (enabled_ && have_goal_ && map_fresh && robot_gate) {
      for (auto &robot : robots_)
        if (robot.assignment.active &&
            double(now_ns - robot.assignment.last_publish_ns) * 1e-9 >=
                goal_refresh_period_) {
          const ArrivalSlot slot{robot.assignment.slot_id,
                                 robot.assignment.target};
          publishGoal(robot, slot, now_ns);
        }
    }
    publishStatus();
  }

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr config_lock_;
  std::string world_frame_, mission_goal_topic_, rviz_goal_topic_;
  bool enabled_{false}, require_all_robots_{true};
  bool have_map_{false}, have_goal_{false}, map_changed_{false};
  bool force_assignment_{true};
  Vec3 mission_goal_{};
  int64_t goal_ns_{0}, map_ns_{0};
  double map_timeout_{5.0}, odom_timeout_{1.5}, tf_timeout_{.2};
  double goal_reached_distance_{.35}, goal_update_distance_{.2};
  double goal_refresh_period_{2.0}, planner_failure_delay_{4.0};
  double blocked_path_timeout_{12.0}, blocked_slot_radius_{.45};
  double slot_invalid_confirm_time_{1.20};
  ArrivalSlotConfig slot_config_;
  KeySet occupied_;
  std::vector<ArrivalSlot> slots_;
  std::vector<BlockedSlot> blocked_slots_;
  std::vector<Robot> robots_;
  tf2_ros::Buffer tf_;
  tf2_ros::TransformListener listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr occupied_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      mission_goal_sub_, rviz_goal_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      candidate_pub_, assigned_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
      replan_service_, reset_service_, clear_goal_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RacerTaskAllocator>());
  rclcpp::shutdown();
  return 0;
}
