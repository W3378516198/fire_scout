# 2.1.8 连续执行与入口预约修复

针对 2026-10-07 的 114830、115657、115959 三份轻量日志中已经定位的问题。此次修改跟随器、几何修补与预约协调源码，不通过整体提高速度上限或缩小机体安全包络来掩盖卡停。

## 行为变化

| 故障链路 | 修改后行为 |
| --- | --- |
| 连续弯道每隔约 3 cm 被设置一个停车点，反复起步 | 区分几何修整目标与执行停车条件。连续小角度采样点使用曲率/偏航速度限速，不清零速度；真实离散急折角仍停车转向。 |
| 原指令滤波器仍保持高速，而新转向点已很近 | 接近停车点时，输出速度不能超过新剖面的允许速度；实测惯性仍由刹车扫掠检查处理。此限制是减速指令，不代表飞机瞬时停稳。 |
| 刹车越过旧转向点后，转回去追逐身后目标 | 实测停稳后，仅在紧邻的出弯段内选择重接点，不能跨过下一个停车折角；检查连接段后再对准、接回。连接不安全则保持停止并请求几何修补。 |
| 转向卡停未进入剖面修补，一直转向或直接脱困 | TURN 状态纳入有界进度监测，默认约 2 秒请求一次几何修补；8 秒窗口仍没有有效响应时才升级脱困。地图/机间等外部限制不被这一修补流程绕过。 |
| 已停住时，仍强制继承前方 30 cm，坏折角无法被修改 | 明确的转向卡停修补可在实测低速时修改近端弯道；运动中保留按速度、加速度和 jerk 计算的前缀。候选仍检查当前地图、净空、机间状态、方向和后退量。 |
| 脱困刚开始，旧路线心跳又把控制权拿回，重新遇到同一个卡点 | 相同路线须先有至少 0.35 m 实际位移，再允许稳定的观测空间交接；有新路线时可重新检查。交接仍要求实测刹车，并清除旧转向锁存。长路线按当前局部进度重接，不回到路线头部。 |
| 已申请新入口，旧入口因请求锚点不同而一直隔离 | 每次新鲜请求均用该机的实测位置检查旧预约的退出条件。确已离开才释放旧预约，同时保留新申请；首次迁移被拒绝后，后续心跳会继续复核。失联、位置过期或仍在旧入口内不会释放。 |

连续曲线依然受到横向加速度和偏航速度约束，典型关系为 `v <= sqrt(a_lat / kappa)`；严格朝向模式还限制 `v <= 0.75 * yaw_rate / yaw_curvature`。这次并未加入 TEB、MPC 或完整 EGO 规划器。

`turn_max_curvature=1.60` 和 `turn_min_curve_speed=0.35` 是**优先几何修整的目标**，不再单独使密集曲线每个采样点停车。无法在当前走廊中改善的连续弯道可以按动力学可行速度通过。局部方向跳变至少 20° 且超出曲率/速度目标，或超过 `sharp_turn_stop_deg`，仍作为停车折角。该区分已写入 YAML 注释。

默认最大水平速度仍为 1.8 m/s。`replan_period`、`path_publish_period` 的目标周期仍为 0.05 秒，`lock_valid_route=false`。这不保证在仿真负载过高时实际达到 20 Hz；几份日志中的低实时因子仍会拉长实际等待时间。

## 安装

先结束旧规划 launch。将压缩包内整个 `fire_scout_planning_safe` 文件夹替换到 `~/fire_scout_ws/src/`。如需保留自定义起点、展开间距和世界配置，请先备份并逐项合并；不要用旧 YAML 整体覆盖新配置。旧源码备份放在 `src/` 之外，避免出现两个同名包。

```bash
cd ~/fire_scout_ws
bash src/fire_scout_planning_safe/scripts/install_fire_scout_planning_safe.sh "$PWD"
source install/setup.bash
export ROS_DOMAIN_ID=0
```

安装脚本备份本包旧构建产物后重新编译。启动原有 PX4、Gazebo 和桥接，继续使用原来的目标接口。三机示例：

```bash
ros2 launch fire_scout_planning_safe multi_racer_navigation.launch.py \
  scout_count:=3 use_sim_time:=true start_mapper:=true \
  racer_enabled:=false auto_arm:=true
```

四机将 `scout_count:=3` 改成 `scout_count:=4`。数量、命名空间、world/odom 起点必须与实际 PX4 模型对应；本版不另行更改你的 SDF 或起飞坐标。

检查运行中的进程：

```bash
ros2 param get /scout1/safe_airfar_like_planner runtime_version
ros2 param get /scout1/safe_airfar_path_follower runtime_version
```

期望为 `2.1.8-execution-aware-repair` 与 `2.1.8-continuous-turn-rejoin`。协调器诊断为 `runtime=2.1.8`。未修改的建图等模块可以继续显示自己的历史标识。

## 轻量记录与诊断

```bash
ros2 run fire_scout_planning_safe collect_fire_scout_logs.sh \
  --duration 900 --scouts 3 --max-mb 24
```

四机使用 `--scouts 4`。900 秒为实际录制时间，24 MB 为事件文件上限，达到上限提前结束并打包。最终压缩包还包含少量配置和系统状态，不能把 24 MB 理解成整个归档的严格大小上限。仍不录完整点云或大型 rosbag。

新增诊断自动进入现有轻量日志：

- `TURN_REJOIN`：越点后的局部接回过程；`TURN_REJOIN_BLOCKED`：没有通过检查的连接段。
- `TURN_EXECUTION_REPAIR`：转向执行受限，申请局部几何修补。
- `turn_limited_seconds`：转向受限监测累计时间。
- `turn_repair_samples` 与 `turn_stop_vertices`：需要改善的采样点与实际停车折角，二者不再混淆。
- `migrations_completed`、`owner_anchorN`、`epochN`：旧入口迁移次数和每个预约的冻结锚点/编号，便于核对真正的退出条件。

正常的目标悬停、真实障碍、机间让行和有效入口排队仍可能停止飞行。下一轮重点检查是否还存在“安全检查通过、没有推进、反复转向或微小起步”的模式。

## 验证范围

31 个测试目标：22 个 C++ 核心测试与 9 个生产节点方法/launch 测试。生产方法测试使用真实方法体与核心算法，仅替换消息传输等环境接口；不是完整 ROS 2/DDS/TF 联调。

新增或扩充覆盖：

- 约 3 cm 采样、半径 0.55 m 与约 0.333 m 的连续弯道；简化飞行器含 0.25 s 速度响应和加速度限制、偏航响应，验证不再逐点停车。
- 高速旧指令切换到短停车距离时的输出限速、70 cm 越点接回、连接受阻保持、不得跨越下一折角、自交附近的有界重接、长路线恢复。
- 转向修补请求实际进入 plan → worker → complete；低速近端折角可被改善，健康路径、机间状态变化和地图检查继续约束提交。
- 首次申请新入口时仍在旧入口内，后续心跳才飞出；旧请求乱序、失联、时钟回退、占用内禁止释放与原有 FIFO。
- 同路线心跳不能立即取消未移动的脱困；有新路线仍须观测空间检查、稳定交接与实测停稳。

另做 Python 语法、YAML/XML、shell 语法及源码清单检查。当前环境没有 ROS 2、colcon、CMake、PX4/Gazebo，未进行完整节点构建或多机飞行测试。上述通过结果不等于保证真实仿真中不再出现卡停，仍需用相同场景复测。
