#!/usr/bin/env python3

import copy
import math
import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _as_bool(value: str) -> bool:
    value = value.strip().lower()
    if value in ("true", "1", "yes", "on"):
        return True
    if value in ("false", "0", "no", "off"):
        return False
    raise ValueError(f"Expected boolean value, got: {value}")


def _node_params(params_file: str, node_name: str):
    """Load the common /** block plus one node block as a plain parameter dict.

    Passing a plain dict is intentional: it keeps the original algorithm tuning
    when the node is placed in /scoutN namespaces, without requiring a separate
    safe_airfar.yaml copy for every vehicle.
    """
    with open(params_file, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    params = {}
    common = data.get("/**", {}).get("ros__parameters", {})
    specific = data.get(node_name, {}).get("ros__parameters", {})
    params.update(copy.deepcopy(common))
    params.update(copy.deepcopy(specific))
    return params


def _apply_optional_geometry(context, params):
    for name in ("vehicle_wheelbase", "propeller_diameter"):
        value = LaunchConfiguration(name).perform(context).strip()
        if value:
            params[name] = float(value)


def _parse_origins(value: str, count: int):
    """Parse x,y,z,yaw entries separated by semicolons.

    Each transform is T_world_scout_odom. Extra entries are allowed so the
    default four-UAV simulation string also works for scout_count < 4.
    """
    entries = []
    for raw_entry in value.split(";"):
        raw_entry = raw_entry.strip()
        if not raw_entry:
            continue
        fields = [float(field.strip()) for field in raw_entry.split(",")]
        if len(fields) != 4:
            raise ValueError(
                "Each scout_origins entry must be x,y,z,yaw_radians")
        if not all(math.isfinite(field) for field in fields):
            raise ValueError("scout_origins entries must all be finite")
        entries.append(fields)
    if len(entries) < count:
        raise ValueError(
            f"scout_origins provides {len(entries)} transforms for {count} scouts")
    return entries[:count]


def _centered_formation(origins, spacing: float):
    """Return collision-minimizing local offsets for an equal-spacing line.

    The final line is centered on the fleet's initial world XY centroid and is
    aligned with world Y.  Vehicles retain their initial left-to-right order so
    their straight staging segments do not cross.  Each world displacement is
    transformed back into that vehicle's possibly yawed local odom frame.
    """
    count = len(origins)
    if count == 0:
        return []
    center_x = sum(origin[0] for origin in origins) / count
    center_y = sum(origin[1] for origin in origins) / count
    order = sorted(range(count),
                   key=lambda index: (origins[index][1],
                                      origins[index][0], index))
    result = [None] * count
    for rank, index in enumerate(order):
        x, y, _z, yaw = origins[index]
        target_x = center_x
        target_y = center_y + (rank - 0.5 * (count - 1)) * spacing
        world_dx = target_x - x
        world_dy = target_y - y
        local_dx = math.cos(yaw) * world_dx + math.sin(yaw) * world_dy
        local_dy = -math.sin(yaw) * world_dx + math.cos(yaw) * world_dy
        result[index] = {
            "local_x": local_dx,
            "local_y": local_dy,
            "rank": rank,
            "target_world_x": target_x,
            "target_world_y": target_y,
        }
    return result


def _make_actions(context):
    params_file = LaunchConfiguration("params_file").perform(context)
    scout_count = int(LaunchConfiguration("scout_count").perform(context))
    first_scout_index = int(LaunchConfiguration("first_scout_index").perform(context))
    first_target_system = int(LaunchConfiguration("first_target_system").perform(context))

    use_sim_time = _as_bool(LaunchConfiguration("use_sim_time").perform(context))
    start_mapper = _as_bool(LaunchConfiguration("start_mapper").perform(context))
    start_fusion = _as_bool(LaunchConfiguration("start_fusion").perform(context))
    start_racer = _as_bool(LaunchConfiguration("start_racer").perform(context))
    racer_enabled = _as_bool(LaunchConfiguration("racer_enabled").perform(context))
    fire_goal_topic = LaunchConfiguration("fire_goal_topic").perform(context).strip()
    start_planner = _as_bool(LaunchConfiguration("start_planner").perform(context))
    start_follower = _as_bool(LaunchConfiguration("start_follower").perform(context))
    auto_arm = _as_bool(LaunchConfiguration("auto_arm").perform(context))
    startup_spread_enabled = _as_bool(
        LaunchConfiguration("startup_spread_enabled").perform(context))
    startup_spacing_override = LaunchConfiguration(
        "startup_spread_spacing").perform(context).strip()
    legacy_distance_override = LaunchConfiguration(
        "startup_spread_distance").perform(context).strip()
    startup_spread_speed = float(
        LaunchConfiguration("startup_spread_speed").perform(context))
    publish_world_transforms = _as_bool(
        LaunchConfiguration("publish_world_transforms").perform(context))
    world_frame = LaunchConfiguration("world_frame").perform(context).strip()
    mapper_base = _node_params(params_file, "radar_free_space_mapper")
    voxel_override = LaunchConfiguration("fusion_voxel_size").perform(context).strip()
    voxel_size = float(voxel_override or mapper_base.get("voxel_size", .18))
    passage_requested = _as_bool(LaunchConfiguration("start_passage_coordinator").perform(context))
    racer_task_mode = LaunchConfiguration("racer_task_mode").perform(context).strip()
    if racer_task_mode != "goal_guided":
        raise ValueError("This package supports racer_task_mode=goal_guided (terminal goals only)")

    if scout_count < 1:
        raise ValueError("scout_count must be >= 1")
    if first_scout_index < 1:
        raise ValueError("first_scout_index must be >= 1")
    if first_target_system < 1 or first_target_system + scout_count - 1 > 255:
        raise ValueError("target_system range must stay within 1..255")
    if start_racer and not start_fusion:
        raise ValueError("start_racer=true requires start_fusion=true")
    if start_racer and not fire_goal_topic:
        raise ValueError("fire_goal_topic must not be empty")
    if not 0.05 <= voxel_size <= 0.50:
        raise ValueError("fusion_voxel_size must be in [0.05, 0.50] m")
    if (not math.isfinite(startup_spread_speed) or
            not 0.05 <= startup_spread_speed <= 5.0):
        raise ValueError("startup_spread_speed must be in [0.05, 5.0] m/s")

    coordinator_base = _node_params(params_file, "passage_coordinator")
    fusion_base = _node_params(params_file, "multi_uav_map_fusion")
    racer_base = _node_params(params_file, "racer_task_allocator")
    planner_base = _node_params(params_file, "safe_airfar_like_planner")
    follower_base = _node_params(params_file, "safe_airfar_path_follower")

    if startup_spacing_override and legacy_distance_override:
        raise ValueError(
            "Set only startup_spread_spacing or legacy startup_spread_distance")
    spacing_source = (startup_spacing_override or legacy_distance_override or
                      str(follower_base.get("startup_spread_spacing", 9.0)))
    startup_spread_spacing = float(spacing_source)
    if (not math.isfinite(startup_spread_spacing) or
            not 0.10 <= startup_spread_spacing <= 30.0):
        raise ValueError("startup_spread_spacing must be in [0.10, 30] m")

    origins = _parse_origins(
        LaunchConfiguration("scout_origins").perform(context), scout_count)
    formation = (_centered_formation(origins, startup_spread_spacing)
                 if startup_spread_enabled else
                 [{"local_x": 0.0, "local_y": 0.0, "rank": index,
                   "target_world_x": origins[index][0],
                   "target_world_y": origins[index][1]}
                  for index in range(scout_count)])

    actions = []
    scouts = [f"scout{first_scout_index + offset}" for offset in range(scout_count)]
    frames = [f"{scout}/odom" for scout in scouts]
    occupied_topics = [f"/{scout}/map/radar_occupied" for scout in scouts]
    free_topics = [f"/{scout}/map/radar_free" for scout in scouts]
    fused_occupied_topics = [f"/{scout}/map/fused_occupied" for scout in scouts]
    fused_free_topics = [f"/{scout}/map/fused_free" for scout in scouts]
    snapshot_topics = [f"/{scout}/map/radar_snapshot" for scout in scouts]
    fused_snapshot_topics = [f"/{scout}/map/fused_snapshot" for scout in scouts]
    odom_topics = [f"/{scout}/odom" for scout in scouts]
    goal_topics = [f"/{scout}/mission/fire_goal" for scout in scouts]
    planner_status_topics = [f"/{scout}/planning/status" for scout in scouts]
    path_topics = [f"/{scout}/planning/global_path" for scout in scouts]

    if start_fusion:
        if not world_frame:
            raise ValueError("world_frame must not be empty when start_fusion is true")
        if publish_world_transforms:
            for scout, frame, origin in zip(scouts, frames, origins):
                x, y, z, yaw = origin
                actions.append(Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name=f"{scout}_world_odom_static_tf",
                    output="screen",
                    arguments=[
                        "--x", str(x), "--y", str(y), "--z", str(z),
                        "--yaw", str(yaw), "--pitch", "0.0", "--roll", "0.0",
                        "--frame-id", world_frame,
                        "--child-frame-id", frame,
                    ],
                ))

        p = copy.deepcopy(fusion_base)
        p.update({
            "use_sim_time": use_sim_time,
            "world_frame": world_frame,
            "scout_names": scouts,
            "source_frames": frames,
            "occupied_topics": occupied_topics,
            "free_topics": free_topics,
            "odom_topics": odom_topics,
            "fused_occupied_topics": fused_occupied_topics,
            "fused_free_topics": fused_free_topics,
            "source_snapshot_topics": snapshot_topics,
            "fused_snapshot_topics": fused_snapshot_topics,
            "voxel_size": voxel_size,
            "source_timeout": float(LaunchConfiguration("fusion_source_timeout").perform(context)),
            "publish_rate": float(LaunchConfiguration("fusion_publish_rate").perform(context)),
        })
        actions.append(Node(
            package="fire_scout_planning_safe",
            executable="multi_uav_map_fusion",
            name="multi_uav_map_fusion",
            output="screen",
            parameters=[p],
        ))

    if start_racer:
        p = copy.deepcopy(racer_base)
        p.update({
            "use_sim_time": use_sim_time,
            "enabled": racer_enabled,
            "fire_goal_topic": fire_goal_topic,
            "world_frame": world_frame,
            "scout_names": scouts,
            "scout_frames": frames,
            "odom_topics": odom_topics,
            "goal_topics": goal_topics,
            "planner_status_topics": planner_status_topics,
            "voxel_size": voxel_size,
        })
        # Let mapping/fusion publish their first coherent snapshot before the
        # coordinator starts.  Respawn remains an operational safeguard; a
        # process fault is still an error and should be investigated.
        actions.append(TimerAction(
            period=4.0,
            actions=[Node(
                package="fire_scout_planning_safe",
                executable="racer_task_allocator",
                name="racer_task_allocator",
                output="screen",
                parameters=[p],
                respawn=True,
                respawn_delay=2.0,
            )],
        ))

    passage_enabled = passage_requested and start_fusion and start_follower and scout_count > 1
    if passage_enabled:
        p = copy.deepcopy(coordinator_base)
        # Both endpoints share the same physical exit radius and .20m hysteresis.
        p.update({"use_sim_time": use_sim_time, "world_frame": world_frame,
                  "scout_names": scouts,
                  "passage_exit_radius": float(follower_base.get("passage_release_distance", 2.2)),
                  "passage_request_timeout": float(follower_base.get("passage_lease_timeout", .8))})
        actions.append(Node(package="fire_scout_planning_safe", executable="passage_coordinator",
                            name="passage_coordinator", output="screen", parameters=[p]))

    for offset in range(scout_count):
        scout_index = first_scout_index + offset
        scout = f"scout{scout_index}"
        prefix = f"/{scout}"
        frame = f"{scout}/odom"
        target_system = first_target_system + offset
        planner_occupied_topic = (
            f"{prefix}/map/fused_occupied" if start_fusion
            else f"{prefix}/map/radar_occupied")
        planner_free_topic = (
            f"{prefix}/map/fused_free" if start_fusion
            else f"{prefix}/map/radar_free")

        if start_mapper:
            p = copy.deepcopy(mapper_base)
            p["use_sim_time"] = use_sim_time
            # One resolution is mandatory: mapper points, fused voxel keys,
            # planner collision geometry and follower checks must not disagree.
            p["voxel_size"] = voxel_size
            _apply_optional_geometry(context, p)
            p.update({
                "front_topic": f"{prefix}/radar/front/mmwave_points",
                "left_topic": f"{prefix}/radar/left/mmwave_points",
                "right_topic": f"{prefix}/radar/right/mmwave_points",
                "occupied_topic": f"{prefix}/map/radar_occupied",
                "free_topic": f"{prefix}/map/radar_free",
                "map_snapshot_topic": f"{prefix}/map/radar_snapshot",
                "ultrasonic_occupied_topic": f"{prefix}/map/ultrasonic_occupied",
                "odom_topic": f"{prefix}/odom",
                "ultrasonic_topic": f"{prefix}/ultrasonic/front/scan",
                "map_frame": frame,
                "frame_prefix": f"{scout}/",
                "visualization_topic": f"{prefix}/map/radar_cloud",
                "radar_only_topic": f"{prefix}/map/radar_only_occupied",
            })
            actions.append(Node(
                package="fire_scout_planning_safe",
                executable="radar_free_space_mapper",
                namespace=scout,
                name="radar_free_space_mapper",
                output="screen",
                parameters=[p],
            ))

        if start_planner:
            p = copy.deepcopy(planner_base)
            p["use_sim_time"] = use_sim_time
            p["voxel_size"] = voxel_size
            _apply_optional_geometry(context, p)
            p.update({
                # Only the planner consumes the cooperative map. The follower
                # below deliberately retains the local safety map.
                "occupied_topic": planner_occupied_topic,
                "free_topic": planner_free_topic,
                "map_snapshot_topic": (f"{prefix}/map/fused_snapshot" if start_fusion
                                       else f"{prefix}/map/radar_snapshot"),
                "local_map_snapshot_topic": f"{prefix}/map/radar_snapshot",
                "odom_topic": f"{prefix}/odom",
                "goal_topic": f"{prefix}/mission/fire_goal",
                # Do not let one global RViz /goal_pose command every UAV.
                "rviz_goal_topic": f"{prefix}/goal_pose",
                "path_topic": f"{prefix}/planning/global_path",
                "execution_state_topic": f"{prefix}/planning/execution_state",
                "raw_path_topic": f"{prefix}/planning/astar_path",
                "planner_status_topic": f"{prefix}/planning/status",
                "replan_request_topic": f"{prefix}/planning/replan_request",
                "goal_approach_topic": f"{prefix}/planning/goal_approach",
                "frame_id": frame,
            })
            # Deterministic prioritized planning: each scout avoids only
            # already committed routes from lower-index scouts.  Do not pass
            # an empty sequence: ROS 2 Humble normalizes [] to (), whose
            # element type cannot be inferred by the parameter frontend.
            peer_path_topics = path_topics[:offset] if start_fusion else []
            if peer_path_topics:
                p["peer_path_topics"] = peer_path_topics
                p["peer_odom_topics"] = odom_topics[:offset]
            else:
                p.pop("peer_path_topics", None)
                p.pop("peer_odom_topics", None)
            actions.append(Node(
                package="fire_scout_planning_safe",
                executable="safe_airfar_like_planner",
                namespace=scout,
                name="safe_airfar_like_planner",
                output="screen",
                parameters=[p],
            ))

        if start_follower:
            p = copy.deepcopy(follower_base)
            p["use_sim_time"] = use_sim_time
            p["auto_arm"] = auto_arm
            p["passage_scheduler_enabled"] = passage_enabled
            p["voxel_size"] = voxel_size
            # Planning remains acyclic (lower index has route priority), while
            # the last-metre collision layer must be symmetric.  Every follower
            # therefore receives every other vehicle's odometry plus the stable
            # priority used only to break predicted-conflict/passage ties.
            peer_offsets = ([i for i in range(scout_count) if i != offset]
                            if start_fusion else [])
            p["own_priority"] = offset
            if peer_offsets:
                p["peer_odom_topics"] = [odom_topics[i] for i in peer_offsets]
                p["peer_priorities"] = peer_offsets
            else:
                p.pop("peer_odom_topics", None)
                p.pop("peer_priorities", None)
            _apply_optional_geometry(context, p)
            speed = LaunchConfiguration("max_speed_xy").perform(context).strip()
            if speed:
                p["max_speed_xy"] = float(speed)
            if startup_spread_speed > float(p.get("max_speed_xy", 1.8)):
                raise ValueError(
                    "startup_spread_speed must not exceed max_speed_xy")
            staging = formation[offset]
            p.update({
                "startup_spread_enabled": startup_spread_enabled,
                "startup_spread_offset_x": staging["local_x"],
                "startup_spread_offset_y": staging["local_y"],
                "startup_spread_spacing": startup_spread_spacing,
                "startup_formation_rank": staging["rank"],
                "startup_formation_count": scout_count,
                "startup_spread_speed": startup_spread_speed,
                "goal_topic": f"{prefix}/mission/fire_goal",
                "rviz_goal_topic": f"{prefix}/goal_pose",
                "goal_approach_topic": f"{prefix}/planning/goal_approach",
                "path_topic": f"{prefix}/planning/global_path",
                "execution_state_topic": f"{prefix}/planning/execution_state",
                "passage_request_topic": f"{prefix}/planning/passage_request",
                "passage_grant_topic": f"{prefix}/planning/passage_grant",
                "odom_topic": f"{prefix}/odom",
                "occupied_topic": f"{prefix}/map/radar_occupied",
                "free_topic": f"{prefix}/map/radar_free",
                "radar_only_topic": f"{prefix}/map/radar_only_occupied",
                "ultrasonic_topic": f"{prefix}/ultrasonic/front/scan",
                "map_snapshot_topic": f"{prefix}/map/radar_snapshot",
                "follower_sonar_debug_topic": f"{prefix}/planning/follower_sonar_occupied",
                "tracking_target_topic": f"{prefix}/planning/tracking_target",
                "follower_state_topic": f"{prefix}/planning/follower_state",
                "diagnostics_topic": f"{prefix}/planning/control_diagnostics",
                "replan_request_topic": f"{prefix}/planning/replan_request",
                "frame_id": frame,
                # Used by the follower to derive version-aware PX4 topics.
                "px4_namespace": f"{prefix}/fmu",
                "target_system": target_system,
                # Keep these empty so px4Topic<T>() can select _vN suffixes
                # automatically from the installed px4_msgs message version.
                "status_topic": "",
                "land_detected_topic": "",
                "offboard_topic": "",
                "setpoint_topic": "",
                "command_topic": "",
            })
            actions.append(Node(
                package="fire_scout_planning_safe",
                executable="safe_airfar_path_follower",
                namespace=scout,
                name="safe_airfar_path_follower",
                output="screen",
                parameters=[p],
            ))

    return actions


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("fire_scout_planning_safe"),
        "config", "safe_airfar.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("scout_count", default_value="4"),
        DeclareLaunchArgument("first_scout_index", default_value="1"),
        # Your current PX4 commands use -i 1..4. PX4 SITL assigns
        # UXRCE_DDS_KEY/MAV_SYS_ID = px4_instance + 1, hence 2..5.
        DeclareLaunchArgument("first_target_system", default_value="2"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        # Kept false for the current adaptation step. Set true later when you
        # want this package to instantiate one evidence mapper per scout.
        DeclareLaunchArgument("start_mapper", default_value="false"),
        # Fusion is opt-in so the existing independent multi-UAV launch remains
        # a safe fallback while world->odom transforms are being checked.
        DeclareLaunchArgument("start_fusion", default_value="false"),
        DeclareLaunchArgument("start_racer", default_value="false"),
        DeclareLaunchArgument("racer_task_mode", default_value="goal_guided"),
        DeclareLaunchArgument("start_passage_coordinator", default_value="true"),
        # Safe default: inspect inputs and candidate arrival goals before dispatch.
        DeclareLaunchArgument("racer_enabled", default_value="false"),
        DeclareLaunchArgument("fire_goal_topic", default_value="/fire_goal"),
        DeclareLaunchArgument("world_frame", default_value="world"),
        DeclareLaunchArgument("publish_world_transforms", default_value="true"),
        # T_world_scout_odom = x,y,z,yaw(rad). The first three defaults match
        # the current PX4_GZ_MODEL_POSE values (-9, -1, +8 m in world Y).
        # Override the complete list whenever any simulator spawn changes.
        DeclareLaunchArgument(
            "scout_origins",
            default_value="-12.5,-9.0,0.2,0.0;-12.5,-1.0,0.2,0.0;"
                          "-12.5,8.0,0.2,0.0;-12.5,17.0,0.2,0.0"),
        DeclareLaunchArgument("fusion_voxel_size", default_value=""),
        DeclareLaunchArgument("fusion_source_timeout", default_value="5.0"),
        DeclareLaunchArgument("fusion_publish_rate", default_value="1.0"),
        DeclareLaunchArgument("start_planner", default_value="true"),
        DeclareLaunchArgument("start_follower", default_value="true"),
        # Safer default for initial multi-UAV validation. Enable explicitly once
        # topic routing and target_system values are verified.
        DeclareLaunchArgument("auto_arm", default_value="false"),
        DeclareLaunchArgument("startup_spread_enabled", default_value="true"),
        # Empty means: read startup_spread_spacing from safe_airfar.yaml.
        DeclareLaunchArgument("startup_spread_spacing", default_value=""),
        # Deprecated parameter name. In V22.1 its value is interpreted as the
        # final adjacent centre-to-centre spacing, just like the new argument.
        DeclareLaunchArgument("startup_spread_distance", default_value=""),
        DeclareLaunchArgument("startup_spread_speed", default_value="0.80"),
        DeclareLaunchArgument("vehicle_wheelbase", default_value=""),
        DeclareLaunchArgument("propeller_diameter", default_value=""),
        DeclareLaunchArgument("max_speed_xy", default_value=""),
        OpaqueFunction(function=_make_actions),
    ])
