# 2.1.7 全路径转弯策略与快速局部刷新

## 改动及依据

此前日志显示低几何速度上限、转向等待和历史退让共同造成停顿；不能仅用“进入硬膨胀区”解释全部情况。已有日志没有完整点云，本次验证是使用真实生产方法和构造场景回归，不能称为原 SDF 飞行的精确重放。

本版按最新要求，将连续转弯能否执行与几何平滑一起考虑；不能安全连续通过的转向明确停车旋转，并解除先前为了稳路线而设置的有效路线锁定与换路冷却。已有机体包络、超声、地图年龄、机间制动和入口预约继续参与判断。

## 1. 全路径检查与停车转向

`turn_policy.hpp` 对发布路径的相邻线段估计离散曲率、航向变化率和可连续通过速度。横向加速度的速度尺度为 `sqrt(a_lat / curvature)`，偏航速度尺度为 `0.75 * yaw_rate / yaw_curvature`，取两者中更小者。它是离散几何筛查，不能理解为所有连续样条点都获得解析曲率上界证书。

`turn_geometry.hpp` 检查整条候选路径，尝试把不合适的局部弯曲改为认证的五次 Bézier 过渡。新段及接缝须满足相同转弯策略、参考走廊偏移限制和机体/净空检查；正在飞行的短前缀和未修改的后部保留。单次修整受时间预算和最多 16 个区域限制；预算耗尽时保留安全几何，剩余不合适转弯由执行器停车处理。

如果曲线不可认证，尝试将厘米级的密集弯曲整理为有限折线转向点；所有新弦仍需碰撞与余量检查。不能整理则保留原路径，不用提高最低速度绕过约束。

跟随器在整条路径建立速度剖面，所有不合适的折角都能进入以下状态。即便一条新路径只有两个点、没有内部曲率样本，大角度朝向变化也适用停车旋转。

| 状态 | 执行含义 |
|---|---|
| `TURN_APPROACH` | 接近转向点并按速度剖面减速，仍可能有平移 |
| `TURN_BRAKE` | 平移指令为零，等待实际三维速度降到阈值，暂不主动旋转 |
| `TURN_IN_PLACE` | 平移指令为零，按偏航速率/加速度限制旋转 |
| 正常跟随 | 实际偏航对准后解除转向状态，重新按加速度/jerk 限制出发 |

重复发布同一条路径不会重置偏航加速度。若旧接缝已在身后，而且实际航向已对准离开方向，则继续前进，不返回过时接缝。停车指令不代表实际飞机瞬间停止；原有实测制动与机间安全层仍会评价动量，必要时安全脱离动作可优先于普通转向。

## 2. 参考 EGO 的范围

参考原始 EGO-Planner 论文：<https://arxiv.org/abs/2008.08835>，尤其是 smoothness、dynamical feasibility、time allocation/refinement 的思路。

本包的 B 样条优化已有拟合、二阶弯曲和三阶差分平滑代价，本版增加名义时间间隔下速度、加速度、jerk 超限的平方惩罚，并保留控制点移动信赖区域、端部位置/切线/二阶导数及最终碰撞认证。

优化后根据一阶/二阶/三阶差分超限比例，计算时间伸长系数 `max(1, velocity_ratio, sqrt(acceleration_ratio), cbrt(jerk_ratio))`；在剩余预算内按更长名义时间间隔再拟合。再拟合仍不能超出原始控制点的移动区域，并且只有原始目标代价降低才接受。

这里的差分和时间尺度是优化器的可行性近似，尤其夹持样条端部的解析导数系数并不等同于均匀内部差分；不是整条连续样条的精确动力学认证。诊断中的 `spline_time_scale`、`spline_optimizer_interval` 也不是发给 PX4 的轨迹时间戳。当前发布的是几何 `nav_msgs/Path`，实际速度、横向加速度、偏航速度、制动和 jerk 由跟随器独立限制。

未移植 EGO 的完整 rebound 碰撞优化、L-BFGS 求解器、ESDF-free 障碍推斥向量构造、各向异性再拟合，以及完整定时轨迹协议。全路径离散曲率筛查和原地停车转向是本包新增策略，不应称为 EGO 原有固定曲率上限算法；也没有引入 TEB/MPC/DWA。

## 3. 刷新策略

默认 `lock_valid_route=false`、`route_switch_cooldown=0.0`。墙钟规划目标周期和路径/执行许可心跳目标周期均为 0.05 秒，异步结果轮询 0.01 秒。它们是调度目标，不是完整 A* 每秒保证运行 20 次；多机 CPU 负载和仿真实时因子仍会影响端到端延迟。

有效路线不必重跑全局 A*。每次检查后，在保留按实际速度确定的前方 1.0～2.5 m 短前缀的情况下，尝试缩短之后的局部绕行。局部探测最多考虑前方 5 m 区间，限制候选相对原走廊偏离不超过 0.60 m，新弦需要机体之外默认 0.25 m 余量认证，并保持远端不变。

局部缩短仍经过平滑、全路径转弯检查、最新地图和新障碍 guard 认证。提交须有实际长度和代价改善、保持小角度/后退限制、不能增加共享入口，并且机间状态编号与生成候选时一致。这样小修改可以立即提交，不必等待全局候选的冷却或较大的改善比例。全局换入口等大幅改路仍保留角度、代价与连续确认，避免每次地图变化都换一条完全不同路线。

前方余量复核改为 0.10 ROS 秒，失败后的再尝试间隔 0.30 ROS 秒。已修正对应参数校验范围。新障碍对路径失效的检查与执行层安全制动仍独立运行，不等待这个软净空复核周期。实际触发时机还取决于地图是否更新以及其原有证据确认。

## 配置

参数位于 `config/safe_airfar.yaml`；规划器与跟随器共用转弯参数。

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `turn_policy_enabled` | true | 启用全路径转弯策略 |
| `turn_max_curvature` | 1.60 /m | 连续飞行的离散几何曲率筛查上限，尺度约为半径 0.625 m |
| `turn_min_curve_speed` | 0.35 m/s | 低于此几何可行速度时尝试平滑/停车转向；不是指令速度下限 |
| `turn_capture_distance` | 0.10 m | 转向点捕获距离 |
| `turn_stop_speed` | 0.10 m/s | 实际三维速度低于此值才旋转 |
| `turn_yaw_tolerance_deg` | 8° | 实际航向对准容差 |
| `turn_repair_budget_ms` | 16 ms | 单次全路径几何修整预算 |
| `bspline_feasibility_weight` | 0.20 | 差分动力学超限代价权重 |
| `bspline_nominal_speed` | 1.20 m/s | 几何优化的名义速度 |
| `lock_valid_route` | false | 不再永久锁住有效路线 |
| `route_switch_cooldown` | 0 s | 取消人为换路冷却 |
| `replan_period` | 0.05 s | 墙钟路线检查/局部刷新/有界搜索目标周期 |
| `path_publish_period` | 0.05 s | 路径与执行许可目标心跳周期 |
| `clearance_review_period` | 0.10 ROS s | 软余量复核间隔 |
| `clearance_repair_retry` | 0.30 ROS s | 软余量修补再尝试间隔 |

最大水平速度仍为 1.8 m/s，不通过增大速度来掩盖急转问题。`sharp_turn_stop_deg=80` 的折角判断与新曲率/可行速度判断共同作用。

## 安装和复测

先结束旧规划 launch，将完整 `fire_scout_planning_safe` 文件夹替换到 `~/fire_scout_ws/src/`；旧源码备份放在工作空间外。然后：

```bash
cd ~/fire_scout_ws
bash src/fire_scout_planning_safe/scripts/install_fire_scout_planning_safe.sh "$PWD"
source install/setup.bash
export ROS_DOMAIN_ID=0
```

脚本会备份本包旧构建产物后重新编译。启动 PX4 和原有桥接后，继续使用与真实模型起点一致的四机配置，例如：

```bash
ros2 launch fire_scout_planning_safe multi_racer_navigation.launch.py \
  scout_count:=4 use_sim_time:=true start_mapper:=true \
  racer_enabled:=false auto_arm:=true
```

按原有接口启用 racer 和发送目标。节点数量、PX4 命名空间与 world/odom 起点必须一致，完整包保留原先的 N 机起飞展开配置。

确认新进程已加载：

```bash
ros2 param get /scout1/safe_airfar_like_planner runtime_version
ros2 param get /scout1/safe_airfar_path_follower runtime_version
ros2 param get /scout1/safe_airfar_like_planner lock_valid_route
ros2 param get /scout1/safe_airfar_like_planner replan_period
```

期望分别为 `2.1.7-global-turn-policy`、`2.1.7-stop-turn-depart`、`false`、`0.05`。仅重新 source 不能更新还在运行的旧进程；其他未修改模块保留自己的版本标识。

轻量录制仍可使用：

```bash
ros2 run fire_scout_planning_safe collect_fire_scout_logs.sh \
  --duration 900 --scouts 4 --max-mb 24
```

900 秒为实际录制时间；24 MB 为未压缩事件文件上限，达到上限提前结束并打包。不录完整点云或大型 rosbag。

## 新诊断与验证边界

规划器增加 `turn_smoothed`、`turn_polygonized`、`turn_stop_vertices`、`turn_max_curvature`、`turn_budget_exhausted` 和优化时间尺度字段；局部缩短可见 `ROUTE_LOCAL_REFRESH` / `COMMIT_LOCAL_REFRESH`。跟随器增加 `turn_phase`、`turn_stop`、`turn_curvature`、`turn_anchor_distance`。

29 个测试目标通过：21 个 C++ 核心目标，8 个生产节点方法/launch 目标。新增覆盖连续弯道、90°/180°刹车旋转离开、实测速度未停稳、同路径心跳、对准释放、硬阻挡、移动前缀保护、障碍否决局部捷径、差分目标梯度、名义时间调整、实际 plan→worker→complete 局部刷新及候选等待期间机间状态变化拒绝。

部分历史回归显式关闭新策略以保留对旧低速剖面函数的测试；新增全路径回归及实际局部刷新流程使用启用的新策略。生产节点方法测试只替换消息传输接口，并非 DDS/TF/PX4 联仿。

当前环境没有 ROS 2、colcon、CMake、PX4/Gazebo，因此未完成 ROS 2 全包构建或四机飞行测试。目标刷新频率和连续性改善需在实际仿真复测，不能保证所有安全等待都消失。验证记录见 `v2_1_7_validation.json`，源码完整性可在包目录运行 `sha256sum -c SOURCE_SHA256SUMS.txt`。
