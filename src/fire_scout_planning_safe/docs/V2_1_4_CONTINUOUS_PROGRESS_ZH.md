# 2.1.4 连续飞行与多机计算修复

本版针对“直路上频繁停下、缓慢爬行、后机被堵住”的可复现代码问题修改。它不是单纯增大最大速度或缩小碰撞包络。

## 修改内容

1. **真正保留原路径**：拒绝候选路线时保持当前几何、路线版本和弧长进度。安全前缀截取、旧尾恢复不再把实时里程计位置作为一条新的短折线插入原路径。控制器继续承担横向误差修正。
2. **安全前缀逐段验证**：保留曲线的每一条未来边都须通过检查，不能只检查一条连接弦就认为弯曲前段安全。远处障碍仍截断路线并保留制动终点；新结果仍在最新地图上重新认证。
3. **实际入口竞争才搜索**：按小批次检查共享入口，存在其他无人机路线本身不再触发周期性完整搜索。入口无收益复查进入冷却；真实地图失效、任务变化、近时域同伴冲突保留原处理。有效临时路线扩展失败后按 0.25、0.5、1 秒退避，前进或任务变化可提前唤醒。
4. **同伴路线进度连续**：同伴已飞过起点数米时仍能提取其前方路线，相同路径心跳不重置进度，自交处使用有界投影。首次观测无法分辨所在支路时不猜测；跟随器的实时同伴位置保护保持独立。
5. **本机近场补充**：主融合地图仍新鲜时，更新的本机原子快照可补充周围 6 m（另含机体包络）的占据证据，进入路线检查及修复。只累计不同源时间戳且确实命中同一失效位置的证据；重复轮询和融合追上同一帧不重复确认。本机旧自由体素不会清除主地图障碍，也不会刷新主地图寿命；历史障碍消退仍由融合证据规则处理。
6. **区分低限速和真实停滞**：正常安全条件下，如果跟随器自身曲率速度剖面持续极低，先请求局部几何修复。正常遵从慢速指令前进时不再仅因旧停滞阈值自动后退；指令长期无响应或一直零速仍升级到原有认证恢复。新目标、仿真时间回退、退出低限速模式时不会继承过期停滞倒计时。
7. **局部速度剖面修复**：只尝试调整前方约 3 m 内的局部弯折，至少保留 0.30 m 原曲线及远端尾部。连接曲线和发布折线均须碰撞认证，实际 Tracker 剖面须显著改善，并限制转向、后退、路径代价和入口竞争。没有认证收益就继续原路线，不强行提高速度下限。
8. **融合计算减负**：公共自由体素统一整理，个体置信度主要处理占据/冲突集合；平移坐标系使用线性快照编码，旋转或量化边界自动回退。地图范围、障碍置信度、失效超时和数据内容保持一致。

## 参数

上传文件中的 `clearance_weight: 7.0` 保留。默认最高水平速度仍为 1.8 m/s；本版先去除异常限速，不进一步抬高安全相关速度。

`lookahead`、`max_speed_xy`、`max_accel_xy`、`max_jerk_xy`、`max_lateral_accel`、`max_yaw_rate`、`sharp_turn_stop_deg` 放在 YAML 的通用 `/**` 段供规划器与跟随器共享。启动文件也兼容旧配置中跟随器段的覆盖值。`max_speed_xy` 命令行覆盖同时传给两者。

跟随器新增：

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `progress_profile_speed_threshold` | 0.12 m/s | 安全条件正常时，低于此剖面限速才进入专用判断 |
| `progress_profile_repair_delay` | 2.0 s | 持续低限速后首次请求修复 |
| `progress_profile_repair_repeat` | 2.0 s | 修复请求重复间隔 |
| `progress_profile_recovery_grace` | 8.0 s | 没有运动响应或零指令零运动的恢复观察窗口 |

以上进展超时使用 ROS/仿真时间；暂停仿真不会消耗窗口。规划器保留 0.10 秒检查/路线心跳、0.01 秒异步结果轮询。这些是调度周期，不代表每次都重新搜索，也不能保证 CPU 繁忙时准时完成。

## 安装与运行

先结束旧规划 launch。将压缩包中的整个 `fire_scout_planning_safe` 目录替换到 `~/fire_scout_ws/src/`。旧源码若需要备份，请放在工作空间外，避免 colcon 发现两个同名包。

```bash
source /opt/ros/humble/setup.bash
cd ~/fire_scout_ws
bash src/fire_scout_planning_safe/scripts/install_fire_scout_planning_safe.sh "$PWD"
source install/setup.bash
export ROS_DOMAIN_ID=0
```

脚本会备份该包的旧构建产物再重新编译。运行中的旧进程不会因重新编译或 source 自动更新，必须重新启动 launch。

继续使用原有 PX4/Gazebo 启动位置及当前启动参数；例如四机先起飞展开、再启用任务的 launch：

```bash
ros2 launch fire_scout_planning_safe multi_racer_navigation.launch.py \
  scout_count:=4 use_sim_time:=true start_mapper:=true \
  racer_enabled:=false auto_arm:=true
```

`racer_enabled:=false` 表示保持原有的起飞/展开流程，尚未启用任务分配。世界与 odom 的启动位置配置必须继续匹配实际 PX4 的模型位置。

## 复测日志

```bash
ros2 run fire_scout_planning_safe collect_fire_scout_logs.sh \
  --duration 900 --scouts 4 --max-mb 24
```

24 MB 是未压缩事件文件上限，到达上限会提前结束并打包；不录制完整点云。新增诊断沿现有状态话题自动收集，无需额外话题或大 rosbag。

- 跟随器：`progress_cause`、`profile_limited_seconds`、`profile_expected_distance`、`profile_actual_displacement`。
- 规划器：`PROFILE_LIMITED_PROGRESS`、`COMMIT_PROFILE_LOCAL_REPAIR`、`profile_repair`、`profile_before`、`profile_after`、`local_overlay_occ`、`local_overlay_stamp`。
- 融合器：`common_free`、`weighted_cells`、`fast_snapshots`、`processing_ms`。最后一项是发布处理阶段耗时，不含接收解码。

## 已验证与未验证

回归测试使用实际 C++ 核心和提取的真实节点方法，交通层使用桩，不是 DDS/Gazebo 飞行测试。完整结果见同目录 `v2_1_4_validation.json`。

- 20 m 密集直线、1～10 cm 横向偏差、飞行进度超过 8 m：拒绝候选、裁剪及恢复尾部后不产生短折角限速。原始最小复现中 1 cm 偏差将限速降为约 0.041 m/s；修复后直线剖面保持 1.8 m/s。它是允许速度上限，不是保证瞬时飞行速度。
- 低速指令正确执行、真实冻结、零指令、转向等待、恢复正常剖面后的倒计时分别测试。
- 近场地图去重确认、过期输入停发许可、障碍安全前缀、同伴路线进度和入口复查均有回归。
- 融合输出与原规则逐字节比较。四份各百万体素的合成基准中，旧核心流程约 8604 ms，新流程约 1044 ms，约 8.24 倍；该测量不含 ROS、DDS、Gazebo，不能据此保证实时因子或实飞速度。

此执行环境没有 ROS 2/PX4/Gazebo，因此未运行完整 colcon 节点构建或四机飞行。机体碰撞、超声、制动、观测方向和机间保护仍可能在确有风险时减速/停车；还需用新包飞行日志确认剩余停顿原因。
