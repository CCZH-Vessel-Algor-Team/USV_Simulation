# env_panel

提供 RViz 环境、目标船和风暴场配置面板；环境面板用于设置风、水流、波浪和太阳光。
另含 CPU 占用面板，用于仿真/实船调试时定位 CPU 热点。

## 构建

```bash
colcon build --packages-select env_panel --symlink-install
source install/setup.bash
```

## 启动

先启动 `usv_sim_full` 仿真，再在 RViz 中选择：

```text
Panels -> Add New Panel -> env_panel/VrxEnvPanel
```

面板只发布环境控制话题，由 Gazebo 插件计算船体受力。
目标船和风暴场面板可分别选择 `env_panel/DynamicShipConfig` 与 `env_panel/StormFieldConfig`。

## 视角控制面板

`env_panel/UsvViewPanel` 使用 RViz 原生 `TopDownOrtho` 和
`ThirdPersonFollower` 控制器，提供全局俯视、跟随俯视与斜后跟随三个快捷视角。
每次点击只应用一次远景预设；本船跟随由 RViz TF 机制完成，之后的鼠标旋转、平移
和缩放不会被面板持续覆盖。

全局俯视在点击时读取本船 TF，并以本船当前位置为中心定位一次，之后不跟随；
跟随俯视则持续以 `usv_1/base_link` 为中心移动。

```text
Panels -> Add New Panel -> env_panel/UsvViewPanel
```

## 雷达控制面板

`env_panel/RadarPanel` 将雷达参数与控制服务集成到 RViz。面板连接
`/radar_control_node`，并使用 `/radar/set_power`、`/radar/set_transmit` 和
`/radar/query_all` 服务。

```text
Panels -> Add New Panel -> env_panel/RadarPanel
```

面板支持量程、转速、增益、海杂波、旁瓣、雨雪杂波、高级滤波及惯导配置，
并订阅 `/parameter_events` 同步外部参数修改。

## CPU 占用面板

`env_panel/CpuPanel` 读取 Linux `/proc` 做两次快照差分，显示 CPU 占用最高的 N 个进程
（`#` / 进程 / PID / CPU% / 占比），用于快速定位仿真或实船上的 CPU 热点。

```text
Panels -> Add New Panel -> env_panel/CpuPanel      # 面板名：CPU 占用
```

- 数据来源：`/proc/<pid>/cmdline`（关键词过滤）+ `/proc/<pid>/stat` 的 `utime+stime` 差分；
  整机利用率另取自 `/proc/stat`；
- 口径一 —— **CPU% 相对 1 个逻辑核**：多线程进程可 >100%，全部进程加起来的上限是 `核数 × 100%`
  （14 核 → 1400%），**不是 100%**，这与 `top` 的每进程 %CPU 约定一致；
- 口径二 —— **占比**：该进程占"被采样进程合计"的比例，**被采样进程加起来 = 100%**；
- 口径三 —— **整机**：`100% × (1 − idle/总时间)`，来自 `/proc/stat`，天然 0–100%，是真正的整机利用率；
- 刷新：1 Hz；数值为 5 s 滑动平均；排序按平滑值降序（同值时按 PID 升序）；
- 交互：`N` 选择 Top 3/5/10、暂停、导出 TSV、双击某行复制 `ps -p <PID> -o pid,pcpu,cmd`；
- 不做仿真/业务侧过滤：只按关键词表与 `/install/`、`/opt/ros/humble/lib/` 路径标记纳入采样。

## 文档

- [功能包架构](docs/ARCHITECTURE.md)
- [数据流输入输出](docs/DATA_FLOW.md)
- [RViz 无人船视角控制面板技术说明](docs/USV_VIEW_PANEL.md)
- [变更记录](CHANGELOG.md)
