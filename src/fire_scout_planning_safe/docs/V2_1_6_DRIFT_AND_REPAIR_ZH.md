# 2.1.6 低速漂移、局部修补与高度接近修复

## 修复依据

本次依据 `fire_scout_log_20261007T071201Z` 的状态和运动记录，并使用真实源码构造回归。4 号机多次停车时硬包络检查通过、局部地图新鲜、同伴很远，不能一概归因于进入硬膨胀区。

2.1.5 的制动查询把软运行余量并入了碰撞回调。当前位置只有有限余量时，预测中约 0.016～0.06 m/s 的轻微漂移会越过这个软边界；缩小候选速度又不会改变这段已测得的漂移，于是平行或远离墙面的候选也可能全部被否决。这是上一版实现中的缺陷，本版将两类约束分开。

日志还显示搜索预算耗尽后重复尝试，以及两机原先高度不同、升降过程中才突然进入相同高度包络的情况。本版也修复相应链路。日志未包含完整点云，下面的回归不是整个 SDF 飞行过程的精确重放。

## 修改内容

### 1. 硬碰撞制动与软余量限速分离

- 硬碰撞查询继续检查实际速度的停距、响应延迟内运动，以及候选指令的连续扫掠。停距模型仍为 `v*T + v²/(2a)`，响应时间另计地图年龄。
- 软余量只评价候选指令的舒适净空，不再把测量漂移作为否决所有候选的硬条件。
- 软余量无法满足时，允许经过硬检查的指令低速继续。默认 `comfort_speed_cap=0.20 m/s`；它是软限速的下限，不是飞机必须达到的最低速度。如果舒适净空允许，可以更快；如果请求本来更小或硬约束更严格，则仍采用更小值，必要时为零。
- 去掉实际速度 0.05 m/s 两侧不一致的动量判断。零长度的延迟段也不再阻止已经通过完整认证的静止脱离动作。
- 机间投影改变速度后，仍重新进行地图硬碰撞和制动检查。

对应实现：`include/fire_scout/measured_braking.hpp` 和 `src/safe_airfar_path_follower.cpp`。

### 2. 已经贴近墙面时允许渐进改善路径

原先的额外余量搜索仍优先执行。若当前位置或接头本身不能满足完整 0.25 m 余量，增加局部退路：

1. 低速时保留前方约 0.30 m 路径；正常运动仍保留原来的移动交接前缀。
2. 在后续有限区间内，用五次平滑过渡逐步偏移，再接回原路线；偏移候选为两侧 0.15、0.25、0.40、0.60 m。
3. 连续检查整条候选的硬碰撞，并限制对应点的净空不得比参考路径恶化超过数值容差。
4. 用前方累计余量不足量评价改善，不再要求包含当前位置的全段最小净空立即增加。当前位置固定时，后者往往根本不可能。
5. 提交时继续经过原有转向幅度、后退、代价、入口竞争、最新地图、前缀是否已飞过等检查。

只改善局部区间，保留原任务和后部路径；找不到可认证且有改善的候选时，不虚构一条可飞路线。偏移采样折线在发布前经过连续碰撞检查，不等同于整个路径获得了动力学最优保证。

对应实现：`include/fire_scout/clearance_contract.hpp` 和规划器节点的 worker/complete 链路。

### 3. 局部搜索继续上次进度

- 局部修补器跨调用保留搜索树和开放集合，预算耗尽后的下一轮继续搜索。
- 优先修补第一个相连失效区间，避免远处另一处障碍把局部修补扩大成整段重新规划。
- 修补后的后部若还有独立障碍，可提交经过认证的临时安全前段，并继续保留原始任务目标。
- 旧路线已经失效、但仍持有安全前缀时，也允许按原有搜索快照时限刷新搜索地图；此前这一分支可能被“还有旧路线”挡住。
- 失败搜索保留真实耗时、扩展数和结果，不再将部分失败尝试误显示为零耗时、零扩展。

此次主要减少重复工作，没有盲目增加规划频率或解除单次计算预算。低实时因子依然会放大按仿真时间计时的等待，不能承诺所有停顿均已消失。

对应实现：`include/fire_scout/planner.hpp`、`route_repair.hpp` 和规划器节点。

### 4. 提前防护升降中的机间接近

- 除当前高度差外，还按实际速度和请求速度检查预测时域内是否会进入同一垂直包络。
- 若水平距离也可能过近，提前限制正在闭合高度差的升降速度；不会为了避让自动制造新的升降方向。
- 垂直制动使用单独的减速度估计和包络外余量。
- 高度相隔足够且不在会合的两架飞机，不会仅因水平位置重叠就被误判成硬碰撞。

对应实现：`include/fire_scout/peer_safety.hpp` 和跟随器节点。

## 配置

新增字段位于 `config/safe_airfar.yaml` 的跟随器参数段，launch 会传递这些参数。

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `comfort_speed_cap` | 0.20 m/s | 软余量无法满足时的候选限速下限；不提高原指令，不覆盖硬制动 |
| `peer_vertical_brake_accel` | 0.50 m/s² | 升降会合的制动能力估计 |
| `peer_vertical_braking_reserve` | 0.10 m | 垂直硬包络之外的制动预留 |

原有 `operational_clearance=0.25 m`、`clearance_weight=7.0`、`preferred_clearance=0.75 m`、最大水平速度 1.8 m/s 继续使用。机体硬包络尺寸、超声约束以及起飞展开配置不变。软代价权重增大不代表真实飞行位置一定有相同数值的离墙距离。

## 安装与复测

先结束旧规划 launch。将压缩包内整个 `fire_scout_planning_safe` 文件夹替换到 `~/fire_scout_ws/src/`。旧源码备份放到工作空间外，避免同名包被重复发现。

```bash
cd ~/fire_scout_ws
bash src/fire_scout_planning_safe/scripts/install_fire_scout_planning_safe.sh "$PWD"
source install/setup.bash
export ROS_DOMAIN_ID=0
```

脚本会加载 ROS 2 Humble，备份本包旧构建产物并重新编译。直接完整替换源码、自行 colcon 编译也可以，但编译和 source 不会更新已经运行的旧进程。

继续使用与实际模型起点匹配的原有四机启动配置。例如先起飞展开：

```bash
ros2 launch fire_scout_planning_safe multi_racer_navigation.launch.py \
  scout_count:=4 use_sim_time:=true start_mapper:=true \
  racer_enabled:=false auto_arm:=true
```

随后按原来的接口启用任务并发送目标。无人机数量和起点仍须与 PX4、world/odom 配置一致。

检查修改模块是否已经更新：

```bash
ros2 param get /scout1/safe_airfar_like_planner runtime_version
ros2 param get /scout1/safe_airfar_path_follower runtime_version
```

分别应为 `2.1.6-near-clearance-repair` 和 `2.1.6-drift-safe-braking`。未修改模块保留自己的版本标识。

轻量录制继续使用：

```bash
ros2 run fire_scout_planning_safe collect_fire_scout_logs.sh \
  --duration 900 --scouts 4 --max-mb 24
```

900 秒为实际录制时间；24 MB 为未压缩事件文件上限，到达上限会提前结束并打包。仍不录制完整点云或大型 rosbag。

## 新增诊断

| 字段或状态 | 用途 |
|---|---|
| `COMFORT_SPEED_LIMIT` / `PEER_PROJECTED_COMFORT_LIMIT` | 仅软余量引起限速，区别于硬制动 |
| `comfort_limited` | 本轮是否发生软余量限速 |
| `clearance_near_repair` | 是否使用近端渐进余量修补 |
| `clearance_deficit_before` / `clearance_deficit_after` | 候选提交前后累计余量不足量；越小越好 |
| `NEAR_CLEARANCE_JOIN` | 保留前缀的近端局部交接 |
| `patch_search_resumed` / `retained_search_nodes` | 局部搜索是否延续、保留多少搜索节点 |
| `patch_expansions` / `search_result` | 本次修补工作量和实际搜索结果 |
| `peer_predicted_vertical_peers` / `peer_vertical_limited` | 预测高度会合和垂直限速是否触发 |

硬风险仍可产生 `MEASURED_BRAKING_RISK`、`MEASURED_RESPONSE_BRAKE`、`PEER_PROJECTED_MAP_BRAKE`。判断停车原因应同时看实际速度、指令速度、硬净空、地图年龄和机间状态，不能只看某一个状态名。

## 验证范围

完整结果见 `v2_1_6_validation.json`。19 个可移植 C++ 测试目标及 8 个节点方法/launch 测试目标全部通过；节点测试编译提取的生产方法，通信接口使用桩。

新增或扩展回归覆盖：

- 0.016～0.06 m/s 漂移时，硬碰撞检查通过的平行、远离动作不再因软余量被锁死；真正硬碰撞风险仍制动。
- 静止状态下的严格认证脱离，以及机间投影后的地图复检。
- 从约 0.13 m 净空起步的局部偏移，保留前缀和尾部，并通过真实规划节点提交链路。
- 每轮仅 8 个扩展的局部搜索跨多轮完成；两个分离障碍时先处理第一个、临时路线止于后续障碍之前。
- 升降会合提前限速、分层不接近时不误停。带 0.50 s 一阶响应及 0.50 m/s² 垂直减速度的双机模型，最小高度差约 0.644 m，测试硬边界为 0.544 m；该结果仅属于合成模型。
- 原有路线交接、余量认证、入口调度、机间进度、地图与融合链路；1～4 机 launch 参数传递。

当前环境没有 ROS 2、PX4 或 Gazebo，未完成实际节点 colcon 构建或四机联仿。上述测试用于确认实现和回归边界，不能替代对飞控响应、点云误差及低实时因子下实际飞行的验证。
