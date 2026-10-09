# 2.1.5 运行余量与实际速度制动

## 本次修复

本版依据 20261007T054340Z 日志所暴露的链路修改：4 号机局部地图余量逐步降低，指令已经减速/归零时实际速度仍非零；之后 3 号机接近已停前机，进入机间硬包络。日志没有完整点云，不能据此还原墙面或宣称已精确重放整个飞行场景。回归使用相应的速度/距离条件和合成障碍模型。

1. **硬边界之前检查运行余量**：每 0.30 ROS 秒检查前方 6 m 的连续线段扫掠，连续两份不同安装地图都不满足运行余量时触发 `CLEARANCE_DEGRADED`。单纯“路径没有碰硬边界”不再永远锁住贴墙路径；同一地图的反复轮询不重复累计证据。
2. **只修补局部区间**：搜索首个余量不足区间附近的替代段，保留原路径前方承诺段和后部路线。临时搜索地图额外增加 0.25 m 运行余量，实时地图的机体半径及恢复半径不改变。仍检查转向、后退、额外代价、入口竞争、最新地图及前缀是否已被飞过。没有更好的局部解时保留硬边界有效的原路线，2 ROS 秒后才允许再次尝试。
3. **平滑与连接不随意消耗原有余量**：B 样条障碍目标至少采用运行余量；平滑、接头和低速局部修补还检查参考路线已有的连续净空，最高认证到 0.25 m，允许 5 mm 数值余差。不符合则使用已认证原折线，或在移动交接时拒绝候选、保留旧路线。发布前还按最新地图验证；没有声称每个狭窄门缝都必然有 0.25 m 余量。
4. **实际速度制动**：使用里程计速度检查动量方向的完整停距，同时检查响应延迟之后的候选指令扫掠。减速指令不会被误当作已经减速的实际运动。配置响应时间 0.80 s，另外加地图年龄；采用 0.65 m/s² 制动能力估计。相关停距为 `v·T + v²/(2a)`。
5. **后机提早减速**：机间半空间约束增加实际相对闭合速度、响应距离和硬包络外 0.20 m 制动余量。前机已停时，后机不能因为自己的速度指令较小就绕过实际动量检查。远离同伴的运动仍可保留。
6. **机间投影后再次查地图**：机间约束可能改变速度方向，最终方向必须再次通过地图/超声碰撞与制动检查。已有的严格认证脱困和有界高度返回仍保留。

## 关键参数

这些字段已写入 `config/safe_airfar.yaml`，通常不需要另行添加 launch 参数。

| 位置 | 参数 | 默认值 | 作用 |
|---|---|---:|---|
| 通用 `/**` | `operational_clearance` | 0.25 m | 规划复查、平滑认证及跟随器的运行余量目标 |
| 规划器 | `clearance_review_enabled` | true | 启用提前余量检查 |
| 规划器 | `clearance_review_horizon` | 6.0 m | 前方检查长度 |
| 规划器 | `clearance_review_period` | 0.30 s | ROS/仿真时间检查周期 |
| 规划器 | `clearance_repair_retry` | 2.0 s | 无收益修补的重试间隔 |
| 跟随器 | `measured_brake_response_time` | 0.80 s | 飞控响应预留，另计地图年龄 |
| 跟随器 | `measured_brake_deceleration` | 0.65 m/s² | 制动能力估计，与最大期望加速度不同 |
| 跟随器 | `peer_reaction_time` | 0.80 s | 机间闭合速度响应预留 |
| 跟随器 | `peer_brake_accel` | 0.65 m/s² | 机间闭合制动能力估计 |
| 跟随器 | `peer_braking_reserve` | 0.20 m | 两机硬包络之外的制动预留 |

`preferred_clearance: 0.75` 和 `clearance_weight: 7.0` 仍属于路径搜索的软代价。它们不会强制每个实际飞行位置距离墙壁 0.75 m。

窄通道当前位置无法满足 0.25 m 额外余量时，跟随器以当前位置可用余量减 5 mm 作为此次制动查询的额外边界，保留平行或远离墙壁的运动；硬包络始终保留。若既无余量足够的局部替代路线，也无可认证运动，仍会停车，不能通过缩小机体尺寸强行通行。本版不是“任何位置都能自行脱困”的保证。

## 安装

结束旧规划 launch，将压缩包内整个 `fire_scout_planning_safe` 文件夹替换到 `~/fire_scout_ws/src/`。如需备份旧源码，把备份放在工作空间外，避免 colcon 发现两个同名包。

```bash
source /opt/ros/humble/setup.bash
cd ~/fire_scout_ws
bash src/fire_scout_planning_safe/scripts/install_fire_scout_planning_safe.sh "$PWD"
source install/setup.bash
export ROS_DOMAIN_ID=0
```

安装脚本备份本包旧的 build/install 产物后重新编译。直接完整替换源码并自行执行 colcon 也可以；编译后须结束旧进程并重新启动。

继续使用原来的 PX4/Gazebo 世界、启动位置与四机配置。例如先起飞展开，暂不启用任务：

```bash
ros2 launch fire_scout_planning_safe multi_racer_navigation.launch.py \
  scout_count:=4 use_sim_time:=true start_mapper:=true \
  racer_enabled:=false auto_arm:=true
```

随后仍按原有方式启用 racer 和发送任务目标；本次没有改任务接口。世界坐标和各机 odom 原点配置需匹配实际模型启动位置。

可以检查正在运行的修改模块：

```bash
ros2 param get /scout1/safe_airfar_like_planner runtime_version
ros2 param get /scout1/safe_airfar_path_follower runtime_version
```

分别应为 `2.1.5-clearance-contract` 和 `2.1.5-measured-braking`。没有修改的建图、融合等模块保留各自的模块版本，不用据它们的版本字符串判定本次修改是否生效。

## 复测与轻量日志

```bash
ros2 run fire_scout_planning_safe collect_fire_scout_logs.sh \
  --duration 900 --scouts 4 --max-mb 24
```

24 MB 是未压缩事件文件上限，到达上限会提前停止录制并打包。不增加完整点云或大型 rosbag，新增字段随现有状态话题录制：

- 规划器：`switch_trigger=CLEARANCE_DEGRADED`、`COMMIT_CLEARANCE_LOCAL_REPAIR`、`KEEP_ROUTE_CLEARANCE_POLICY`、`CLEARANCE_NO_LOCAL_ALTERNATIVE`、`CLEARANCE_CERTIFIED_RAW`、`candidate_margin_valid`。
- 控制器：`MEASURED_RESPONSE_BRAKE`、`MEASURED_BRAKING_RISK`、`PEER_PROJECTED_MAP_BRAKE`。
- 数值：`map_clearance`、`measured_speed`、`command_speed`、`measured_stop_distance`、`operational_clearance`、`peer_measured_braking_risk`、`peer_required_braking_clearance`、`peer_nearest_distance`。

请重点看危险接近前是否已开始局部修补/减速，以及 `map_clearance` 是否仍跌到负值、机间距离是否再次小于约 0.938 m。`measured_stop_distance` 是当前模型所需路径长度，不是测得的真实制动距离。

## 验证范围

完整清单见 `v2_1_5_validation.json`。25 个测试目标覆盖可移植 C++ 核心、提取的真实节点方法，以及 1～4 机 launch 最终参数；通信接口采用桩。

新增回归包括：

- 两端都净空的长线段中间存在障碍，连续余量查询能检出。
- 原路径硬边界有效但只有约 0.13 m 余量，经两份地图确认后通过真实 `plan → worker → complete` 提交局部绕离；保留前缀/尾部，成功后不继续改路。
- 重复地图不重复确认，重试有冷却；窄通道无解不虚构路径。
- 指令已降到较小值、实际速度仍为 1.861～1.98 m/s 的实际动量保护。
- 0.65 s 一阶速度响应、减速度上限 0.65 m/s² 的合成接近测试：墙外运行余量维持约 0.25 m，停止前机的最终机间距离约 1.14 m。该数值仅属于此测试模型。
- 窄直通道平行运动、远离同伴、地图认证反向制动、有界高度返回，以及机间投影后的墙面复检。
- 原有路径连续性、恢复、入口调度、地图新鲜度、融合和同伴进度测试。

当前执行环境没有 ROS 2、PX4 或 Gazebo，未进行完整 colcon 节点构建，也未进行四机联仿。这些测试不能证明所有飞控响应、地图误差及低实时因子条件下都不再卡住；须以新包联仿日志继续确认。
