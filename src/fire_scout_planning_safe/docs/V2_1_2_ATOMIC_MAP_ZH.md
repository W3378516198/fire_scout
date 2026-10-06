# 2.1.2：融合更新中断与本机地图接管

## 日志定位与修复范围

本次源码与 2.1.1 一致。对应 `fire_scout_log_20261002T104950Z` 末尾，一号已到达目标，二号恢复正常跟踪，三号停在 `PLANNER_INPUT_PAUSED`。三号规划器融合地图年龄约 25.5 秒，而本机地图年龄仅 0.176 秒；硬包络检查通过，Offboard/解锁状态正常，入口预约已获准。融合状态明确显示 `scout3:stale`。

日志能确认融合端失去可用的新地图，但未记录自由地图的实际接收和配对统计，不能据此认定是某个具体 DDS 丢包错误。源码中的两路时间戳严格配对、订阅深度 1、四组缓存存在可复现的配对饥饿：自由云比占据云迟超过窗口时，配对所需的占据样本已被淘汰。日志中共享自由空间达 83 万体素，原 XYZ 格式约 13 MB/份，容易放大这种不同步。

## 数据传输

- mapper 发布 `/scoutN/map/radar_snapshot`；占据、自由、分辨率、坐标系、原始观测时间属于同一个消息。
- fusion 仅保留每机最新的完整待处理快照，每次融合刷新处理一次；收到后不逐帧转换整张地图。TF 未就绪时允许下一份新快照替换旧的待处理快照。
- fusion 发布 `/scoutN/map/fused_snapshot`，planner 使用这一原子快照；时间戳仍来自该无人机自己的 mapper。其他无人机的新消息不能延长本机已过期地图的有效期。
- follower 始终使用本机 radar_snapshot，仅展开自身周围的体素区域，占据和自由层在同一个回调中提交。
- 这是无损、完整快照，采用整数体素键与沿 x 方向的连续体素行程压缩；没有降低地图分辨率、删掉自由体素或缩小机体。
- 丢掉任意几份旧快照后，下一份完整消息仍可直接恢复；没有依赖前一消息的增量链。本版尚不是分布式地图碎片交换协议。

内部消息仍采用标准 `sensor_msgs/msg/PointCloud2`，不新增 ROS 自定义接口或第三方压缩依赖。`_snapshot` 的字段是体素行程记录，不是 XYZ 点云，不能直接选择它来显示点云。RViz 的占据地图话题继续使用 `/scoutN/map/fused_occupied` 或 `/fire_scout/map/fused_occupied`，路径继续使用 `/scoutN/planning/global_path`。

compact 模式默认 `publish_free_clouds: false`，关闭大尺寸 XYZ 自由地图显示输出，避免已有 RViz 显示项重新产生同样的负载。完整自由空间仍包含在压缩快照中。需要调试自由点云显示时将该参数设为 true，并重启；有订阅者时才生成该 XYZ 点云。legacy 模式始终发布配对的占据/自由 XYZ 消息，不受此显示开关影响。

## 规划输入接管

1. 融合快照新鲜时使用 PRIMARY。
2. 融合快照过期、本机完整快照新鲜且开关启用时，使用 LOCAL_FALLBACK。本机占据/自由作为新证据；保留上一次融合占据中的未被本机自由观察清除的障碍，排除旧融合自由空间。
3. 接管使用本机消息本来的时间戳，不把旧融合地图重新盖章为“新鲜”。地图来源和两路年龄分别记录。
4. 原路径所有权、曲线和沿路径进度保留；当前地图仍必须验证路径，原有机体包络、跟踪器局部避障、声呐和其他无人机保护继续生效。
5. 两路地图均过期、里程计过期或当前路径不通过检查时仍暂停；融合恢复后重新检查并切回 PRIMARY。

| YAML 参数 | 默认 | 作用 |
|---|---|---|
| 公共 `map_transport` | compact | compact 使用新快照；legacy 使用旧双点云传输，整套节点应保持一致 |
| 公共 `publish_free_clouds` | false | compact 模式是否允许生成大尺寸 XYZ 自由地图显示输出 |
| planner `local_map_fallback_enabled` | true | 融合过期时允许新鲜本机地图接管 |
| `map_snapshot_topic` | 各机 radar_snapshot | multi launch 会给 planner 改为各机 fused_snapshot |
| planner `local_map_snapshot_topic` | 各机 radar_snapshot | 独立本机证据入口 |

原有 `map_timeout`、`source_timeout`、机体尺寸、膨胀、软净空、速度和入口预约参数均保持原值。传感器真正中断不能靠本次优化绕过。

## 检查与录制

启动和发布目标的命令沿用原有版本，详见 README。先整体停止旧规划 launch，再使用新包同时启动 mapper/fusion/planner/follower，避免新旧传输方式混用。

```bash
source /opt/ros/humble/setup.bash
source ~/fire_scout_ws/install/setup.bash
export ROS_DOMAIN_ID=0
timeout -k 2s 12s ros2 topic echo --once /fire_scout/map/fusion_status --field data
timeout -k 2s 12s ros2 topic echo --once /scout3/planning/status --field data
timeout -k 2s 12s ros2 topic echo --once /scout3/planning/control_diagnostics --field data
```

融合状态包含 `transport=compact` 和每机 `rx/commit/coalesced/rejected/input_age/age/bytes`。coalesced 是主动省略被更新快照替代的旧快照，不表示占据/自由丢失；每个 commit 都是一份完整地图。

planner 包含 `map_source=PRIMARY/LOCAL_FALLBACK`、`primary_map_age/local_map_age/map_rx/map_commits/map_rejected/map_bytes`。融合年龄增大而本机年龄保持小值、执行许可正常时，是本机接管工作。若两路都增大，会按旧安全规则报 STALE_MAP。拒绝计数增长时检查日志中的分辨率、帧名、包格式或时间重置错误。

新日志程序记录 snapshot 的两类体素数、字节数、源时间，不订阅完整 XYZ 自由地图；记录性能不再依赖传送那些大显示消息。

```bash
bash ~/fire_scout_ws/src/fire_scout_planning_safe/scripts/collect_fire_scout_logs.sh \
  --duration 120 --scouts 3
```

## 已验证和实际限制

11 个 C++ 测试、4 个 Python 测试通过。节点方法测试编译和运行实际源码中的新快照封装、发布、接收、融合处理与规划接管函数；ROS、TF 和日志接口为测试桩。它覆盖旧配对饥饿复现、整包丢帧恢复、融合中断和恢复、本机接管、双方过期拒绝、路径进度保留、TF 延迟、损坏包拒绝、区域展开及旧模式输出。

人工生成的 80 万连续自由体素测试，XYZ 数据约 12.8 MB，压缩快照约 96 KB，并验证逐个体素无损恢复。实际雷达地图的压缩比取决于自由空间连续程度，不能把该比例当成实际场景结果。

当前环境没有 ROS2/PX4/Gazebo，未进行完整 ROS 链接构建、DDS 压力测试或飞行验证。此版本移除了已复现的双点云配对失效方式并增加输入接管，但不承诺消除真实障碍、门口排队、局部恢复或硬件通信故障造成的全部暂停。全局融合仍需遍历当前体素地图，大世界的增量处理和分布式交换留待后续迭代。
