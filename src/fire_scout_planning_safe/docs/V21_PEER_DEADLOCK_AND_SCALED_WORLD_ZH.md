# V21 入口队列防死锁与 2× 楼层说明

> 本文保留为 V21 历史说明。当前跟随器为 V22.1；任意机数的等间距起飞编队请读
> `V22_STARTUP_SPREAD_ZH.md`。

## 多机入口死锁修复

V20 在所有让行状态中停用了停滞恢复。当前机如果按静态编号向后方同伴让行，
而后方同伴又被当前机的硬距离挡住，会形成永久闭环。V21 保留 0.938 m 左右的
硬距离，不通过关闭避碰解决堵塞，改为：

1. 沿当前通道/指令方向位于后方的同伴不能要求前机停车，后机负责制动；
2. 通道所有者持续移动时自动续租，不把正常排队误判为故障；
3. 阻挡者在完整 3 s 窗口内净位移不足 0.12 m 时发布
   `PEER_YIELD_DEADLOCK`，往返摆动不计作有效进展；
4. 死锁恢复优先使用已飞过并仍通过地图、超声检查的历史轨迹后退；
5. 没有历史轨迹时，只搜索一条远离阻挡者的短距离已观测路线；
6. 让行状态允许不接近任何同伴的低速退让，但所有普通制动半空间仍然有效。

新增/更新的诊断字段包括：

- `peer_yield_stalled`
- `peer_yield_seconds`
- `peer_blocker_stagnant_seconds`
- `peer_rear_yields_ignored`
- `peer_deadlock_recovery`
- `peer_blocker_x` / `peer_blocker_y`

新增状态包括 `PEER_PASSAGE_RETREAT`、`PEER_CONFLICT_RETREAT`、
`PEER_YIELD_DEADLOCK_HOLD` 和 `PEER_HARD_DEADLOCK_HOLD`。

## 2× 楼层模型

配套的 `fire_scout_floor_2x.sdf` 将所有静态模型的平移、箱体尺寸、圆柱半径和
长度统一放大 2 倍。缩放锚点为当前多机出生队列中心 `(-12.5, 0, 0)`，因此楼层
内部任意两点距离严格翻倍，同时现有出生区域仍位于建筑外，不需要为了首次测试
立即修改 `multi_scout.launch.py`。

原 20×14×4 m 建筑变为 40×28×8 m，1.20 m 内部门洞变为 2.40 m。无人机、
重力、物理步长和 ROS 参数中的 x500 真实碰撞包络没有缩放。

这里的 2× 是仿真代理比例：约 0.50 m 轴距的 x500 在 2× 环境中等效为约
0.25 m 级的小型折叠无人机。因此，40×28×8 个仿真米仍对应约 20×14×4 m
的现实楼层，2.40 m 仿真门洞对应约 1.20 m 的现实门洞。不要把
`vehicle_wheelbase` 或 `propeller_diameter` 改小来“配合”世界；规划器仍须使用
x500 在仿真中的真实碰撞尺寸。

SDF 内部 world 名称仍为 `fire_scout_floor`。把配套文件替换到仿真实际读取的
world 路径即可；现有 `scout_origins` 不需要随 2× 世界修改。

节点包默认参数也与该世界配对：起飞/巡航高度为 3.0 m，规划高度上限为
6.0 m，单段规划上限为 48 m。最低安全高度、x500 碰撞包络、近距保护和制动
距离仍按仿真机真实尺寸计算，没有跟着世界放大。

旧世界中的目标点 `(x, y, z)` 如需落在放大后相同的建筑语义位置，应换算为：

```text
x_new = -12.5 + 2 × (x_old + 12.5) = 2 × x_old + 12.5
y_new = 2 × y_old
z_new = 2 × z_old
```

例如旧世界目标 `(0, 0, 1.5)` 对应新世界 `(12.5, 0, 3.0)`。这是 world 坐标
的换算；各无人机局部 odom 坐标仍由启动文件中的静态 TF 自动处理。

## 运行版本检查

```bash
ros2 param get /scout1/safe_airfar_path_follower runtime_version
ros2 param get /scout1/safe_airfar_like_planner runtime_version
```

应分别为：

```text
21.0-peer-deadlock-recovery
21.0-stable-route-continuity
```
