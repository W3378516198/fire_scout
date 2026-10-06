# 多机启动时 odom 与 base_link TF 断链排查

## 已确认的代码缺陷

旧版 `px4_odom_adapter` 只接受 PX4 `VehicleOdometry.pose_frame=1`
（`POSE_FRAME_NED`）。PX4 也允许输出 `pose_frame=2`（固定 `FRD` 局部坐标系），
尤其在全局航向尚未对齐或室内定位条件下。旧节点会把这些合法消息全部丢弃，
因此同时缺少：

- `/scoutN/odom`；
- `scoutN/odom -> scoutN/base_link` 动态 TF。

旧节点只打印一次警告，且 `/clock` 为零时完全静默，所以表现为偶发 TF 断链，
但启动日志里往往看不出原因。

## 本版本修复

1. 同时接受 NED 和 FRD，并分别转换为 ROS ENU/FLU 约定；
2. 检查完整四元数及位置有效性；
3. `/clock`、PX4 输入、非法帧及 TF 发布分别给出持续但限频的诊断；
4. 缓存时钟启动前收到的最新 PX4 样本，时钟有效后立即补发；
5. Gazebo 桥接和定位节点退出后由 launch 自动重启；
6. RViz 默认延迟 2 秒启动，减少正常启动窗口内的假报警。

FRD 是局部固定航向坐标系。它能安全恢复每台无人机自己的
`odom -> base_link`，但使用公共 `world` 融合多机地图时，仍应确认每台
`world -> scoutN/odom` 的静态 yaw 与该机初始航向一致。

## 编译

```bash
cd ~/fire_scout_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  fire_scout_localization fire_scout_bringup \
  --symlink-install
source install/setup.bash
```

原启动命令可以继续使用。新参数默认已经启用：

```bash
ros2 launch fire_scout_bringup multi_scout.launch.py \
  scout_count:=4 \
  first_model_index:=1 \
  use_sim_time:=true \
  accept_frd_pose_frame:=true \
  use_rviz:=true
```

正常运行时每个适配器会定期输出：

```text
ODOM_TF_OK ... frame=1 ...
```

或：

```text
ODOM_TF_OK ... frame=2 ...
```

## 一键诊断

保持故障现场运行，另开终端执行：

```bash
source /opt/ros/humble/setup.bash
source ~/fire_scout_ws/install/setup.bash

ros2 run fire_scout_bringup diagnose_multi_tf.sh \
  4 "$HOME/Downloads/fire_scout_tf_diagnostic.log"
```

诊断顺序为：`/clock`、PX4 原始里程计、适配器参数、ROS 里程计、TF。判断规则：

- `/clock` 缺失或保持 0：检查 scout1 Gazebo 桥接、`GZ_PARTITION`；
- PX4 话题没有发布者或数据：检查 PX4 DDS 命名空间、Micro XRCE Agent、
  `ROS_DOMAIN_ID` 和 `px4_msgs` 版本；
- PX4 有数据但 `/scoutN/odom` 没数据：查看适配器日志中的 `pose_frame`、
  四元数或时钟拒绝原因；
- `/scoutN/odom` 正常但 `tf2_echo` 失败：检查 `/tf` 是否存在重复发布者或
  frame 名称冲突。
