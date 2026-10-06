# 轻量日志采集

安装并 `source` 工作空间后运行：

```bash
ros2 run fire_scout_planning_safe collect_fire_scout_logs.sh \
  --duration 900 --scouts 4 --max-mb 24
```

- `--duration 900`：录制 15 分钟；设置为 `0` 时按 Ctrl-C 结束。
- `--scouts 4`：期望记录的无人机数量。
- `--max-mb 24`：未压缩事件文件硬上限；达到上限会安全停止并打包。
- `--output DIR`：输出目录，默认 `~/fire_scout_logs`。

脚本不依赖 `ros2 daemon`，也不会因为启动时尚未发现话题而退出。它直接订阅 ROS 2，
记录规划/跟随状态变化、2 Hz 里程计、变化后的路径几何、1 Hz 超声摘要，以及点云的
点数、字节数、源时间戳和接收频率；不保存大体积点云数据。每两秒的 topic health
会标出地图 `missing/delayed/stale`，同时区分“没有收到新消息”和“重复发布旧时间戳”。

结束后会得到：

```text
~/fire_scout_logs/fire_scout_log_YYYYMMDDTHHMMSSZ.tar.xz
```

把这个压缩包发送回来即可。正常 10～15 分钟记录通常远低于上限；最终大小取决于状态
字符串变化频率和压缩率。
