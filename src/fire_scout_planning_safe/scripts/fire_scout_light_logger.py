#!/usr/bin/env python3
"""Small, analysis-oriented Fire Scout logger.

It records state changes and sampled summaries instead of full point clouds.
The result stays useful for route flips, motion holds and disappearing maps
without producing a multi-gigabyte rosbag.
"""

import argparse
import hashlib
import json
import math
import os
import signal
import time
import struct
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry, Path as PathMessage
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import String


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def finite(value):
    return math.isfinite(float(value))


class CompactWriter:
    def __init__(self, filename, max_bytes):
        self.filename = Path(filename)
        self.stream = self.filename.open("w", encoding="utf-8", buffering=1)
        self.max_bytes = max_bytes
        self.bytes_written = 0
        self.full = False

    def write(self, record):
        if self.full:
            return False
        line = json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n"
        encoded = line.encode("utf-8")
        if self.bytes_written + len(encoded) > self.max_bytes:
            marker = json.dumps({
                "type": "logger_limit",
                "wall_time": time.time(),
                "message": "Configured log-size limit reached cleanly",
                "max_bytes": self.max_bytes,
            }, ensure_ascii=False, separators=(",", ":")) + "\n"
            self.stream.write(marker)
            self.stream.flush()
            self.full = True
            return False
        self.stream.write(line)
        self.bytes_written += len(encoded)
        return True

    def close(self):
        if not self.stream.closed:
            self.stream.flush()
            self.stream.close()


class FireScoutLightLogger(Node):
    def __init__(self, output, scouts, duration, max_mb):
        super().__init__("fire_scout_light_logger")
        self.writer = CompactWriter(output, int(max_mb * 1024 * 1024))
        self.start_wall = time.monotonic()
        self.duration = duration
        self.stop_requested = False
        self.stats = {}
        self.last_logged = {}
        self.last_value = {}
        self.last_map_state = {}
        self._subscriptions_keepalive = []
        self.qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        for scout in range(1, scouts + 1):
            root = f"/scout{scout}"
            self.add_string(f"{root}/planning/status", "planner_status", 0.0, 5.0)
            self.add_string(f"{root}/planning/follower_state", "follower_state", 0.0, 5.0)
            self.add_string(f"{root}/planning/control_diagnostics", "control", 1.0, 5.0)
            self.add_path(f"{root}/planning/global_path")
            self.add_string(f"{root}/planning/execution_state", "execution_permit", 1.0, 3.0)
            self.add_path(f"{root}/planning/passage_request")
            self.add_path(f"{root}/planning/passage_grant")
            self.add_cloud(f"{root}/planning/follower_sonar_occupied")
            self.add_odom(f"{root}/odom")
            self.add_sonar(f"{root}/ultrasonic/front/scan")
            for suffix in (
                "radar_occupied", "radar_only_occupied",
                "fused_occupied", "ultrasonic_occupied",
                "radar_snapshot", "fused_snapshot",
            ):
                self.add_cloud(f"{root}/map/{suffix}")

        self.add_string("/fire_scout/map/fusion_status", "fusion_status", 0.0, 3.0)
        self.add_string("/fire_scout/coordination/status", "passage_status", 0.0, 3.0)
        self.add_string("/fire_scout/racer/status", "racer_status", 0.0, 3.0)
        self.add_cloud("/fire_scout/map/fused_occupied")
        # Do not subscribe to optional full XYZ free clouds: a diagnostic logger
        # must not enable the large traffic it is investigating. Snapshot
        # headers report both counts and bytes without expanding their cells.
        self.health_timer = self.create_timer(2.0, self.health)
        self.system_timer = self.create_timer(5.0, self.system_health)
        self.writer.write({
            "type": "logger_start", "wall_time": time.time(),
            "ros_time": self.get_clock().now().nanoseconds * 1e-9,
            "scout_count": scouts, "duration": duration, "max_mb": max_mb,
            "pid": os.getpid(),
        })

    def register(self, topic, kind):
        self.stats[topic] = {
            "kind": kind, "count": 0, "interval_count": 0,
            "last_wall": None, "last_source": None,
            "source_changed_wall": None,
        }

    def seen(self, topic, source=None):
        now = time.monotonic()
        stat = self.stats[topic]
        stat["count"] += 1
        stat["interval_count"] += 1
        stat["last_wall"] = now
        if source is not None and finite(source):
            if stat["last_source"] is None or abs(source - stat["last_source"]) > 1e-9:
                stat["source_changed_wall"] = now
            stat["last_source"] = source

    def base(self, kind, topic, source=None):
        record = {
            "type": kind, "topic": topic, "wall_time": time.time(),
            "elapsed": round(time.monotonic() - self.start_wall, 3),
            "ros_time": round(self.get_clock().now().nanoseconds * 1e-9, 6),
        }
        if source is not None:
            record["source_stamp"] = round(source, 6)
        return record

    def due(self, topic, period):
        now = time.monotonic()
        previous = self.last_logged.get(topic, -1e30)
        if now - previous < period:
            return False
        self.last_logged[topic] = now
        return True

    def add_string(self, topic, kind, min_period, heartbeat):
        self.register(topic, kind)

        def callback(msg):
            self.seen(topic)
            value = msg.data[:8192]
            changed = value != self.last_value.get(topic)
            now = time.monotonic()
            previous = self.last_logged.get(topic, -1e30)
            if now - previous < min_period:
                return
            if not changed and now - previous < heartbeat:
                return
            self.last_logged[topic] = now
            record = self.base(kind, topic)
            record.update({"changed": changed, "data": value})
            self.writer.write(record)
            self.last_value[topic] = value

        self._subscriptions_keepalive.append(self.create_subscription(String, topic, callback, self.qos))

    def add_odom(self, topic):
        self.register(topic, "odom")

        def callback(msg):
            source = stamp_seconds(msg.header.stamp)
            self.seen(topic, source)
            if not self.due(topic, 0.5):
                return
            p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
            record = self.base("odom", topic, source)
            record.update({
                "frame": msg.header.frame_id,
                "p": [round(p.x, 3), round(p.y, 3), round(p.z, 3)],
                "q": [round(q.w, 4), round(q.x, 4), round(q.y, 4), round(q.z, 4)],
                "linear": [round(v.linear.x, 3), round(v.linear.y, 3), round(v.linear.z, 3)],
                "angular_z": round(v.angular.z, 3),
            })
            self.writer.write(record)

        self._subscriptions_keepalive.append(self.create_subscription(Odometry, topic, callback, self.qos))

    def add_path(self, topic):
        self.register(topic, "path")

        def callback(msg):
            source = stamp_seconds(msg.header.stamp)
            self.seen(topic, source)
            raw = []
            for pose in msg.poses:
                p = pose.pose.position
                raw.append((round(p.x, 3), round(p.y, 3), round(p.z, 3)))
            digest = hashlib.blake2b(repr(raw).encode("utf-8"), digest_size=8).hexdigest()
            changed = digest != self.last_value.get(topic)
            if not changed and not self.due(topic, 5.0):
                return
            self.last_logged[topic] = time.monotonic()
            stride = max(1, math.ceil(len(raw) / 200))
            sampled = raw[::stride]
            if raw and sampled[-1] != raw[-1]:
                sampled.append(raw[-1])
            record = self.base("path", topic, source)
            record.update({
                "frame": msg.header.frame_id, "changed": changed,
                "hash": digest, "points": len(raw), "stride": stride,
                "geometry": sampled,
            })
            self.writer.write(record)
            self.last_value[topic] = digest

        self._subscriptions_keepalive.append(self.create_subscription(PathMessage, topic, callback, self.qos))

    def add_cloud(self, topic):
        self.register(topic, "map")

        def callback(msg):
            source = stamp_seconds(msg.header.stamp)
            self.seen(topic, source)
            points = int(msg.width) * int(msg.height)
            empty_changed = (points == 0) != self.last_value.get(topic + "#empty")
            if not empty_changed and not self.due(topic, 1.0):
                return
            self.last_logged[topic] = time.monotonic()
            record = self.base("map", topic, source)
            record.update({
                "frame": msg.header.frame_id, "points": points,
                "bytes": len(msg.data), "point_step": int(msg.point_step),
                "row_step": int(msg.row_step), "dense": bool(msg.is_dense),
                "empty_transition": empty_changed,
            })
            if topic.endswith("_snapshot") and len(msg.data) >= 24:
                magic, version, occupied, free, state, resolution = struct.unpack_from("<IIIIIf", msg.data)
                record.update({"snapshot_format": version, "occupied": occupied,
                               "free": free, "runs": max(0, points - 1),
                               "voxel_size": resolution,
                               "valid_snapshot_header": magic == 0x46534D31 and state == 0})
            self.writer.write(record)
            self.last_value[topic + "#empty"] = points == 0

        self._subscriptions_keepalive.append(self.create_subscription(PointCloud2, topic, callback, self.qos))

    def add_sonar(self, topic):
        self.register(topic, "sonar")

        def callback(msg):
            source = stamp_seconds(msg.header.stamp)
            self.seen(topic, source)
            if not self.due(topic, 1.0):
                return
            valid = [float(v) for v in msg.ranges if finite(v)]
            record = self.base("sonar", topic, source)
            record.update({
                "samples": len(msg.ranges), "finite": len(valid),
                "positive_inf": sum(1 for v in msg.ranges if math.isinf(v) and v > 0),
                "negative_inf": sum(1 for v in msg.ranges if math.isinf(v) and v < 0),
                "minimum": round(min(valid), 3) if valid else None,
                "range_min": round(float(msg.range_min), 3),
                "range_max": round(float(msg.range_max), 3),
            })
            self.writer.write(record)

        self._subscriptions_keepalive.append(self.create_subscription(LaserScan, topic, callback, self.qos))

    def health(self):
        now = time.monotonic()
        topics = {}
        for topic, stat in self.stats.items():
            age = None if stat["last_wall"] is None else now - stat["last_wall"]
            source_age = (None if stat["source_changed_wall"] is None
                          else now - stat["source_changed_wall"])
            topics[topic] = {
                "kind": stat["kind"], "count": stat["count"],
                "hz_2s": round(stat["interval_count"] / 2.0, 2),
                "age": None if age is None else round(age, 3),
                "source_age": None if source_age is None else round(source_age, 3),
                "source_stamp": stat["last_source"],
            }
            stat["interval_count"] = 0
            if stat["kind"] == "map":
                state = "missing" if age is None else (
                    "fresh" if age <= 1.5 else "delayed" if age <= 5.0 else "stale")
                if state != self.last_map_state.get(topic):
                    record = self.base("map_health_transition", topic)
                    record.update({"state": state, "receive_age": topics[topic]["age"],
                                   "source_age": topics[topic]["source_age"]})
                    self.writer.write(record)
                    self.last_map_state[topic] = state
        self.writer.write({
            "type": "topic_health", "wall_time": time.time(),
            "elapsed": round(now - self.start_wall, 3), "topics": topics,
        })
        if self.duration > 0 and now - self.start_wall >= self.duration:
            self.stop_requested = True
        if self.writer.full:
            self.stop_requested = True

    def system_health(self):
        record = self.base("system", "/proc")
        try:
            record["loadavg"] = [round(float(v), 3) for v in Path("/proc/loadavg").read_text().split()[:3]]
            memory = {}
            for line in Path("/proc/meminfo").read_text().splitlines():
                key, value = line.split(":", 1)
                if key in ("MemTotal", "MemAvailable", "SwapTotal", "SwapFree"):
                    memory[key] = int(value.strip().split()[0])
            record["memory_kib"] = memory
        except (OSError, ValueError):
            record["read_error"] = True
        self.writer.write(record)

    def close(self):
        self.writer.write({
            "type": "logger_stop", "wall_time": time.time(),
            "elapsed": round(time.monotonic() - self.start_wall, 3),
            "bytes": self.writer.bytes_written,
        })
        self.writer.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--scouts", type=int, default=4)
    parser.add_argument("--duration", type=float, default=900.0,
                        help="seconds; 0 records until Ctrl-C")
    parser.add_argument("--max-mb", type=float, default=24.0)
    args = parser.parse_args()
    if not 1 <= args.scouts <= 32 or args.duration < 0 or args.max_mb < 2:
        parser.error("scouts must be 1..32, duration >= 0, max-mb >= 2")

    rclpy.init(args=None)
    node = FireScoutLightLogger(args.output, args.scouts, args.duration, args.max_mb)
    print("LOGGER_READY", flush=True)

    def stop(_signum, _frame):
        node.stop_requested = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        while rclpy.ok() and not node.stop_requested:
            rclpy.spin_once(node, timeout_sec=0.25)
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
