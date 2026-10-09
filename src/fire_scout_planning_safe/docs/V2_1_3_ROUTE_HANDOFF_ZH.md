# 2.1.3：安全前段保留与连续路径交接

本版基于本次上传的源码修改。目标是减少换路造成的停顿和折角，让飞机在后段搜索期间继续执行仍安全的前段。

## 执行过程

1. 从当前沿旧路径的进度截取前段，保留原有顶点，不直接用一条直线替代弯曲的旧段。长度为 `clamp(实测速度 × route_handoff_time, min_prefix, max_prefix)`，默认 1～2.5 m；临近障碍或路线末端时可以缩短，不足 0.3 m 时不作前段承诺。
2. 后段继续由原来的规划器产生。有效路线锁定、局部修补、入口选择、角度/代价迟滞、障碍多帧确认均沿用原策略。
3. 用短五次 Bézier 样条连接前段末端与后段。它等价于一个夹持五次 B 样条跨度，匹配两端切线，端点二阶导数为零。局部修补的入口、出口也在短窗口内尝试平滑；窗口之外的路线保留。
4. 曲线必须满足偏离限制，通过机体包络碰撞检查和未知空间策略。采用递归凸包验证整段解析曲线，同时检查实际发布的采样折线；发布前还会用最新地图和占据保护层再次复核。
5. 跟踪器保留速度、加速度和适用的航向滤波状态。新几何只在前方有限窗口内投影，避免交叉路线使进度跳到远处另一条分支；已通过的同一拐角不会仅因路径换了一次又被重复执行。

五次连接解决的是几何衔接。`nav_msgs/Path` 仍是几何路径，速度、加速度、加加速度及转向限幅仍由原跟踪器执行；本版没有引入 TEB、MPC 或 DWA，也不声称输出带时间参数的完整 C2 飞行轨迹。

## 远处路径失效时

旧实现会因为整条路径的远端失效而撤销执行许可，飞机可能在仍可飞的前段就暂停。本版只发布当前检查通过的安全前缀，并给它独立匹配的执行许可；末端速度规划仍会在前缀结束前制动，等待修补结果。

完整旧路线仍作为规划器的修补依据。障碍证据在确认前消失时，会恢复经过重新检查的旧后段，避免一直停在已经过时的临时终点。

收到新目标时，允许短暂保留安全前段，但只有匹配**新目标**的执行许可才能继续授权飞行；旧目标许可不能被挪用。异步旧目标结果通过 goal epoch 被丢弃。

保留前段不能掩盖一次大幅换路：原有角度/回退判断针对未拼接的提议路线进行，代价同时考虑实际连接后的路线；连接也不能重新穿回原本避开的共享入口。安全旧路线存在而连接失败时，继续保留旧路线。旧路线已失效时，仍允许原来的安全修补与制动处理。

若前段也出现障碍、地图或里程计过期、其他无人机间距不足，或者规划未能在前段耗尽前完成，仍会减速或停止。本版不能保证任何换路都不停。

## 参数

公共开关位于 `config/safe_airfar.yaml` 的 `/**` 下；其余位于 `safe_airfar_like_planner` 下。修改后重启节点。

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `route_handoff_enabled` | true | 规划器与跟随器的连续交接开关 |
| `route_handoff_min_prefix` | 1.0 m | 通常保留长度下限；安全检查可缩短或取消 |
| `route_handoff_max_prefix` | 2.5 m | 保留长度上限 |
| `route_handoff_time` | 1.2 s | 以实测速度确定保留长度的前瞻时长 |
| `route_handoff_blend_min` | 0.60 m | 新路线投影点之后的最小连接终点距离 |
| `route_handoff_blend_max` | 2.40 m | 连接终点搜索窗口上限 |
| `route_handoff_max_deviation` | 0.30 m | 连接样点相对旧前段/新路线的偏离上限 |
| `route_handoff_sample_spacing` | 0.05 m | 连接段发布点的最大相邻距离 |
| `route_handoff_budget_ms` | 12 ms | 单次连接操作的软时间预算；入口、出口窗口分别计时 |
| `replan_period` | 0.10 s | 路线评估及有界搜索续跑的最小启动间隔，原为 0.25 s |
| `planner_poll_period` | 0.01 s | 异步结果轮询间隔，原固定 0.02 s |
| `path_publish_period` | 0.10 s | 路径/执行许可心跳间隔，原为 0.20 s |

规划结果完成并复核后直接发布，无须等下一次常规心跳。100 Hz 是结果轮询目标，10 Hz 是检查/心跳目标，都不是完整 A* 搜索频率，也不是实时性保证。有效路径保持时不会为了刷新频率重复搜索。

总响应时间仍包含传感器与地图更新、原有障碍确认、搜索和 ROS 执行器调度。本次没有提高整张融合地图发布频率，也没有缩短原有障碍确认与换路迟滞时间。上传包中的 `preferred_clearance: 0.75`、`max_speed_xy: 1.80`、多机起点和起飞展开参数均保留。

## 安装与运行

停止旧规划 launch，将压缩包中的整个 `fire_scout_planning_safe` 文件夹替换到 `~/fire_scout_ws/src/`。旧源码备份请放在工作空间 `src` 外，避免 colcon 发现重名包。

```bash
source /opt/ros/humble/setup.bash
cd ~/fire_scout_ws
colcon build --packages-select fire_scout_planning_safe --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
export ROS_DOMAIN_ID=0
```

沿用原启动命令以及和仿真实际一致的 `scout_count`、`scout_origins`。默认配置仍是四架；三架场景需明确 `scout_count:=3`。已在空中启动 racer、目标点发布等使用方式不变。重启 launch 后可核对：

```bash
ros2 param get /scout1/safe_airfar_like_planner runtime_version
ros2 param get /scout1/safe_airfar_path_follower runtime_version
```

期望分别为 `2.1.3-continuous-route-handoff` 和 `2.1.3-continuous-route-follower`。也可使用包内原有安装脚本，它会备份该包旧构建产物后编译。

## 诊断字段

`/scoutN/planning/status` 新增：`handoff`、`handoff_applied`、`handoff_prefix`、`handoff_blend`。它们出现在规划完成的状态详情中，可由原有轻量日志程序记录。

| 状态/原因 | 含义 |
|---|---|
| `PREFIX_ALREADY_SHARED` | 新路线已有相同前段，无需再制造连接曲线 |
| `PREFIX_BEZIER_JOIN` | 安全前段和新后段连接成功 |
| `PREFIX_UNSAFE` | 前段未通过快照地图检查 |
| `NO_SAFE_BLEND` / `HANDOFF_BUDGET` | 没有找到符合条件的连接，或达到软预算 |
| `HANDOFF_ENTRANCE_CONFLICT` | 连接会增加与已有预约入口的重合 |
| `KEEP_ROUTE_NO_SAFE_HANDOFF` | 旧路线有效，连接失败，因此继续旧路线 |
| `KEEP_ROUTE_HANDOFF_OVERRUN` | 结果返回时飞机已快耗尽原预留段，继续有效旧路线 |

录制十分钟示例：

```bash
bash ~/fire_scout_ws/src/fire_scout_planning_safe/scripts/collect_fire_scout_logs.sh \
  --duration 600 --scouts 4 --max-mb 24
```

## 验证范围

12 个 C++ 核心测试和 4 个 Python 测试通过，详细结果见 `v2_1_3_validation.json`。新增覆盖：精确保留弯曲前段、三维连接、采样间距、解析曲线中部碰撞、未知空间拒绝、取消、局部修补接缝、移动时速度连续、交叉路线进度、真实工作线程体的连接分支、未拼接提议的角度判定、最新地图否决结果、远障截短/恢复、近障及过期输入拒绝、1～4 机 launch 配置传播。

节点级测试提取并编译实际源码中的函数和工作线程体，ROS 消息传输使用测试桩。测试发现上传包中的旧 launch 测试仍把舒适距离写死为 0.23，与当前 YAML 的 0.75 不一致；已改为检查配置实际传递，并增加自定义配置传递测试，没有改小运行参数。

当前环境无 ROS2/PX4/Gazebo，未完成完整 ROS 节点链接构建、真实 DDS 调度、仿真飞行或实机验证。测试结果不能代替这些验证。
