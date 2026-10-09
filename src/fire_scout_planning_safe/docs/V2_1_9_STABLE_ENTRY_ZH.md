# 2.1.9 路径衔接、稳定转向与防卡监测

针对 2026-10-08 的 021310、021621 两份轻量日志。本版修改规划和跟随源码，解决安全条件正常时仍因短折角、微小 S 弯和反复换转向目标而慢飞的问题。

## 修改逻辑

| 日志中定位的原因 | 本版处理 |
| --- | --- |
| 继承旧路径时，把实测位置接到旧路径上的点，生成几厘米的侧向或回头折角 | 继承段直接切取原曲线，实测位置到路径的连接只用于独立安全检查。搜索因计算预算分多次完成时也保持原搜索几何。 |
| 检查发现新路径包含旧前段，实际却返回了包含坏连接头的新路径 | 返回精确旧前段加新路径的后段，重新检查整个结果；去掉前面的连接钩。 |
| 一个很近的连接点在身后，跟随器先朝它转，再朝后段转 | 实测速度不超过 `turn_stop_speed`，且前方接回段通过检查时，直接接入紧邻的出弯段。不能跨过下一个停车折角，受阻则保持停止。 |
| 转向中小幅改线，使转向目标和接回目标反复切换 | 锁存当前折角的出弯方向；小幅路径更新保留仍对应同一局部几何的接回状态。真正改变出弯方向时重新判断。 |
| 已停住，但前段继承保住了导致卡停的短 S 弯 | 允许停稳后修改近端坏曲线，用五次平滑连接段接回原尾段。依次尝试约 0.65、1.0、1.4 m 的局部范围，检查完整曲线、采样连线、偏离量、净空和速度剖面改善。运动中仍保护前段。 |
| 跟随、对准、转向来回切换或路线不断更新，使卡停计时归零 | 增加跨状态、跨路径版本的净位移监测。来回小幅挪动不会不断获得新等待窗口；真实机间排队或外部安全限制暂停此计时。 |

局部修补提交前仍由生产节点检查最新地图、机间约束和路径版本。真实急折角、障碍、目标悬停和入口排队仍可以导致停车。

## 相关参数

这些参数在 `config/safe_airfar.yaml` 中：

| 参数 | 默认值 | 用途 |
| --- | --- | --- |
| `replan_period` | 0.05 s | 墙钟下路线检查、局部刷新和有界搜索续跑的目标周期 |
| `path_publish_period` | 0.05 s | 路径发布目标周期 |
| `turn_stop_speed` | 0.10 m/s | 原地转向及停稳入口修补的实测速度条件 |
| `progress_execution_useful_distance` | 0.25 m | 实际净位移达到该值后重置跨状态监测 |
| `progress_profile_repair_delay` | 2.0 s | 受限执行开始后首次请求局部修补的时间 |
| `progress_profile_repair_repeat` | 2.0 s | 持续受限时的修补请求间隔 |
| `progress_profile_recovery_grace` | 8.0 s | 净位移不足时升级现有认证恢复的基本窗口 |

防卡窗口采用 ROS/仿真时间。修补后若已出现正常离开条件，最多补充一次 2 秒响应窗口，不因每次短暂进入跟随状态而重新计时。新任务、已到目标、进入恢复和时钟回退会重置监测。

最大水平速度仍为 1.8 m/s，路线检查没有增加低频冷却。目标周期不等于重做全局搜索的频率，也不保证高负载下实际达到 20 Hz。日志中的低仿真实时因子会拉长实际飞行和等待时间。

## 替换和编译

先 Ctrl-C 结束旧规划 launch。将压缩包内完整的 `fire_scout_planning_safe` 文件夹替换到 `~/fire_scout_ws/src/`；旧源码备份放在 `src/` 外，避免出现同名包。自定义起点、展开间距等按项合并，不要用旧 YAML 整体覆盖新配置。

```bash
cd ~/fire_scout_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select fire_scout_planning_safe --cmake-clean-cache \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
export ROS_DOMAIN_ID=0
```

也可使用包内 `scripts/install_fire_scout_planning_safe.sh`，它会备份本包旧构建产物后重编译。完成编译后重新启动节点，`source` 不会替换已在运行的旧进程。

PX4、Gazebo 和传感器/定位桥接仍按原流程启动。四机规划示例：

```bash
ros2 launch fire_scout_planning_safe multi_racer_navigation.launch.py \
  scout_count:=4 use_sim_time:=true start_mapper:=true \
  racer_enabled:=false auto_arm:=true
```

三机使用 `scout_count:=3`。默认 world/odom 起点为 `scout1=(-12.5,-9,0.2)`、`scout2=(-12.5,-1,0.2)`、`scout3=(-12.5,8,0.2)`、`scout4=(-12.5,17,0.2)`；必须与实际 `PX4_GZ_MODEL_POSE` 对应。修改起点时同时设置 launch 的 `scout_origins`。Racer 与目标点接口沿用原流程，见 [任务分配说明](RACER_TASK_ALLOCATION_ZH.md)。

核对本次修改的两个节点：

```bash
ros2 param get /scout1/safe_airfar_like_planner runtime_version
ros2 param get /scout1/safe_airfar_path_follower runtime_version
```

应分别为 `2.1.9-geometric-prefix` 和 `2.1.9-stable-entry-progress`。没有修改的其他模块可以继续显示各自的历史标识。

## 轻量日志

```bash
ros2 run fire_scout_planning_safe collect_fire_scout_logs.sh \
  --duration 900 --scouts 4 --max-mb 24
```

900 秒为实际录制时间。24 MiB 是事件文件上限，达到后结束录制并压缩；归档另含少量配置和系统状态。三机改为 `--scouts 3`。

新增信息：

- `execution_useful_seconds`、`execution_net_displacement`：跨状态累计时间和实际净位移。
- `EXECUTION_LOCAL_STAGNATION`、`EXECUTION_PROGRESS_TIMEOUT`：局部受限或基本恢复窗口到期。
- `STOPPED_FORWARD_ENTRY_REPAIR`：停稳后近端平滑连接修补成功。
- 全局路径记录中的 `near_geometry`：飞机附近最多 64 个点，保留五位小数，便于识别厘米级变化。
- `/scoutN/planning/map_conflict_snapshot`：发生当前机体地图包络冲突时，记录 2 m 内最近的最多 12 个占据体素中心、地图分辨率和机体包络。不影响控制权限。

仍记录地图点数、源时间戳、接收频率和系统负载；不保存完整点云。详细用法见 [日志采集说明](LOG_COLLECTION_ZH.md)。

## 验证范围

33 项回归检查通过：23 项 C++ 核心测试及 10 项 Python/生产方法片段/launch 测试。新增用例覆盖日志中的短入口折角、带惯性和偏航响应的接回、微小 S 弯、路径小幅更新、搜索续跑、受阻连接、实际运动时禁止停稳修补、反复小幅挪动和日志大小/精度边界。

结果见 `v2_1_9_validation.json`。同时检查 Python、shell、YAML/XML 语法与源码清单。当前环境没有 ROS 2、colcon、CMake、PX4/Gazebo，未进行完整 ROS 节点编译、DDS/TF 联调或多机飞行；需要在原场景编译后复测。
