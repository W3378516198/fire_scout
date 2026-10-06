# V19.1 超声历史记忆永久停飞修复

> 本文保留为超声修复的历史说明。V20 已继承该修复；当前规划器和跟随器的
> 版本号请参见 `V20_PEER_SAFETY_AND_ROUTE_CONTINUITY_ZH.md`。

## 已修复的故障链

旧实现用可回退的 ROS/仿真时间给超声占据记忆计龄，同时拒绝所有不大于上次
时间戳的新消息。Gazebo 重置、`/clock` 回退或传感器时间重新起算后，旧单元会像
来自“未来”一样永不过期，新扫描又会被当作旧消息持续丢弃。因此可能出现：

- 规划路径一直存在；
- 当前超声距离已经恢复到量程上限；
- `sonar_memory_cells` 长时间完全不变；
- 跟随器始终停在 `SONAR_ENVELOPE_CONFLICT_HOLD`，速度为零。

## V19.1 的处理

1. 超声占据 TTL 改用进程单调时钟，不再受 `/clock` 回退影响。
2. 超声、里程计、雷达或地图源时间戳明显回退时，节点开启新时间代并原子清除
   对应的旧缓存；规划器、单机地图和融合地图也允许接收新时间代数据。
3. 有效扫描中的自由射线连续确认两帧后，会主动删除射线内部的旧回波；当前测距
   端点保留 0.12 m 保护段，真实障碍不会被同一帧清除。
4. 即使没有新扫描，控制循环仍按单调时钟执行 TTL 清理；地图发布器也会把超声
   障碍的过期变化发布出去。
5. 跟随器把地图冲突和超声冲突区分为：
   `MAP_ENVELOPE_CONFLICT_HOLD`、`SONAR_ENVELOPE_CONFLICT_HOLD`、
   `MAP_AND_SONAR_ENVELOPE_CONFLICT_HOLD`。

## 新诊断字段

- `sonar_memory_oldest_age` / `sonar_memory_newest_age`
- `sonar_memory_expired_total`
- `sonar_memory_cleared_total`
- `sonar_epoch_resets`
- `runtime_version=19.1-sonar-memory-release`

紧急排查时可只清除某架无人机跟随器的超声记忆：

```bash
ros2 service call /scout1/safe_airfar_path_follower/reset_sonar_memory \
  std_srvs/srv/Trigger '{}'
```

这项服务只用于诊断或已确认的时间代残留；正常运行会自动清理，不应周期性调用。

## 必须干净重编译

只替换 YAML 或重新 `source` 不能更新已经运行的 C++ 进程。先结束旧 launch，再在
工作空间中运行本包的 `scripts/install_fire_scout_planning_safe.sh`。启动后检查：

```bash
ros2 param get /scout1/safe_airfar_path_follower runtime_version
ros2 param get /scout1/safe_airfar_like_planner runtime_version
ros2 param get /scout1/radar_free_space_mapper runtime_version
```

应分别看到 `19.1-sonar-memory-release`、
`19.1-incremental-repair-time-reset` 和 `19.1-sonar-clock-reset`。
