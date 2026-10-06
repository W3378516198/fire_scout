# V22.1 任意机数中心对称起飞编队

## 编队规则

本版本不再写死三机的 `-9/0/+9 m` 相对位移。启动文件读取实际使用的全部
`scout_origins`，计算机群出生位置的 world XY 中心，并沿 world Y 轴生成一条
中心对称、等间距的最终编队线。

设无人机数量为 `N`，Y 方向从小到大排序后的编号为 `k=0...N-1`，Y 坐标平均值
为 `y_center`，YAML 中配置的最终相邻中心间距为 `d`，则：

```text
y_target(k) = y_center + (k - (N - 1) / 2) × d
```

所有目标的 world X 坐标取出生点 X 坐标的平均值。启动文件保持无人机原来的
左右顺序，再把 world 位移按各自 `scout_origins` 的 yaw 转换为局部 odom X/Y
位移，因此不会因为编号顺序不同而让直线路径互相穿插。

- 三机槽位相对中心为：`[-d, 0, +d]`；
- 四机槽位相对中心为：`[-1.5d, -0.5d, +0.5d, +1.5d]`；
- 五机及更多数量使用同一公式自动生成；
- 一机时目标就是自身中心，不产生水平移动。

这里的 `d` 是最终相邻两架无人机的中心到中心距离，不是每架无人机额外飞行的
距离，也不是扣除机体尺寸后的净空。

## 在 YAML 中设置距离

编辑 `config/safe_airfar.yaml`：

```yaml
safe_airfar_path_follower:
  ros__parameters:
    startup_spread_enabled: true
    startup_spread_spacing: 9.0
    startup_spread_speed: 0.80
```

其中 `startup_spread_spacing` 就是最终相邻间距。`startup_spread_offset_x/y`、
`startup_formation_rank/count` 由多机启动文件计算并覆盖，不需要手工设置。

如果只是临时测试，也可以在启动命令中使用
`startup_spread_spacing:=6.0` 覆盖 YAML。首个 V22 包中的旧参数
`startup_spread_distance:=...` 仍可识别；在 V22.1 中它同样表示“最终相邻间距”，
不再表示每架无人机各自增加的位移。不要同时传入新旧两个参数。

## 飞行与障碍处理

每架无人机到达 `takeoff_altitude` 后，直接飞向自己的固定编队位置。该阶段：

- 不读取全局路线来改变编队目标；
- 不调用局部绕行、重新规划或自主脱困；
- 检查雷达地图、超声记忆、实时超声、玻璃风险和完整制动距离；
- 发现障碍就发送零速度悬停，障碍消失后仍恢复原固定直线；
- 独立的无人机间硬距离保护始终有效。

主要状态：

- `STARTUP_SPREAD_TURN`
- `STARTUP_SPREAD_MOVE`
- `STARTUP_SPREAD_WAIT_MAP`
- `STARTUP_SPREAD_OBSTACLE_HOLD`
- `STARTUP_SPREAD_COMPLETE`

## 启动

间距直接读取 YAML，因此正常启动不需要再写距离参数：

```bash
ros2 launch fire_scout_planning_safe multi_racer_navigation.launch.py \
  scout_count:=3 \
  use_sim_time:=true \
  start_mapper:=true \
  racer_enabled:=false \
  auto_arm:=true
```

切换为四机时只需保证第四套 PX4、传感器和命名空间已启动，并改为：

```bash
ros2 launch fire_scout_planning_safe multi_racer_navigation.launch.py \
  scout_count:=4 \
  use_sim_time:=true \
  start_mapper:=true \
  racer_enabled:=false \
  auto_arm:=true
```

## `scout_origins` 必须与 PX4 出生点一致

当前启动文件默认四个 world 出生点为：

```text
scout1: -12.5, -9.0,  0.2, 0
scout2: -12.5, -1.0,  0.2, 0
scout3: -12.5,  8.0,  0.2, 0
scout4: -12.5, 17.0,  0.2, 0
```

前三个对应你当前给出的 PX4 命令。第四个只是沿现有间距继续排列的默认位置；
实际增加第四架时，应让第四架 `PX4_GZ_MODEL_POSE` 与这里一致，或者在 launch 中
完整传入实际 `scout_origins`。两者不一致会同时造成融合地图错位和编队目标错误。

在默认出生点和 `startup_spread_spacing: 9.0` 下，三机只会微调到严格等间距，
不会再各自额外飞 9 m；四机也会围绕四机出生位置的共同中心做小幅对齐。

## 版本检查

```bash
ros2 param get /scout1/safe_airfar_path_follower runtime_version
```

应返回：

```text
22.1-centered-formation
```
