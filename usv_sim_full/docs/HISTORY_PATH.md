# 本船历史航迹使用说明

本功能将本船实际定位累计为连续航迹，在 RViz 原生 Path Display 中显示。航迹逐帧更新，只保留指定时间窗口内的位置；收到新规划任务后清空上一段。另提供一个简洁的 RViz 设置面板插件，直接调整现有节点的保留时长，不修改 RViz 本身或航迹生成逻辑。

## 快速使用

| 需要做什么 | 操作入口 |
|---|---|
| 修改重启后的默认保留时长 | 编辑 [`usv_sim_full/config/history_path.yaml`](../config/history_path.yaml) 的 `history_duration_sec`，然后重启航迹节点或原仿真入口 |
| 当次运行立即调整时长 | RViz 的“历史航迹设置”面板，输入秒数并点击“应用” |
| 手动添加时间设置插件 | `Panels → Add New Panel → env_panel/HistoryPathSettings → OK` |
| 手动添加航迹线显示 | Displays 中 `Add → By display type → Path`，Topic 设置为 `/usv_1/history_path` |
| 显示或隐藏航迹 | 勾选或取消勾选 Displays 中“本船历史航迹” |

当前配置文件的保留时长为 **90 秒**。实际值以 YAML 和节点当前参数为准；如果显式使用了启动覆盖或面板设置，会优先使用覆盖后的值。面板设置不写回 YAML。

默认配置文件的完整位置：

```text
/home/cczhtc/usv_ws_test/src/usv_simulation/usv_sim_full/config/history_path.yaml
```

下面的“修改显示时长”章节说明持久设置、运行时设置以及插件添加的完整步骤；“新增及修改文件位置”列出源码、配置和文档所在位置。

## 功能汇总

整理日期：2026-11-20。实现集中在 `usv_simulation` 的 `usv_sim_full` 和 `env_panel` 两个已有包中。本功能在 `usv_bringup` 中没有保留代码或配置改动。

| 已实现功能 | 最终行为 |
|---|---|
| 连续实际航迹 | 累计本船 `/usv_1/odom`，以 `nav_msgs/msg/Path` 发布，在 RViz 原生 Path 中显示青绿色连续线 |
| 实时更新 | 逐帧处理定位，不再等待移动距离或固定采样间隔；TF 晚到时短暂排队，转换可用后更新 |
| 限时保留 | 当前 YAML 默认保留最近 90 秒，按定位原时间戳移除过期点；停止收到定位时仍定时清理 |
| 新规划清空 | 兼容规划请求、航线下发、目标话题及 Nav2 action，新任务清除上一段航迹和待转换定位 |
| RViz 设置插件 | `env_panel/HistoryPathSettings` 面板可读取保留时长、修改秒数并点击“应用” |
| 持久默认配置 | `config/history_path.yaml` 保存启动默认值；面板和参数命令仅修改当次运行值 |
| 仿真启动接入 | 由 `main.launch.py` 默认启动节点，并向会话使用的 RViz 配置补入显示项；设置面板不自动打开，按需手动添加 |
| 时间重置与容量限制 | 仿真时间倒退时清空历史；保留点数受 `max_points` 限制 |

## 1. 新增及修改文件位置

工作区：`/home/cczhtc/usv_ws_test`。下表路径均相对于 `src/usv_simulation/`，按当前相对 `origin/develop` 的航迹功能差异整理。

### 1.1 新增文件

| 文件 | 作用 |
|---|---|
| [`usv_sim_full/usv_sim_full/scripts/history_path.py`](../usv_sim_full/scripts/history_path.py) | 独立航迹节点：定位接收、TF 等待、历史累计、过期移除、新任务清空 |
| [`usv_sim_full/config/history_path.yaml`](../config/history_path.yaml) | 启动默认值：保留时长、输出坐标系、清空事件、点数上限和清理频率 |
| [`env_panel/src/history_path_panel.cpp`](../../env_panel/src/history_path_panel.cpp) | RViz 设置面板，通过参数服务读取及修改现有节点的保留时长 |
| [`usv_sim_full/test/test_history_path.py`](../test/test_history_path.py) | 验证逐帧更新、TF 晚到、时间窗口、任务清空、时间倒退和配置覆盖 |
| [`env_panel/test/test_history_path_panel.cpp`](../../env_panel/test/test_history_path_panel.cpp) | 验证插件加载、当前值读取、应用设置，以及其他节点参数保持不变 |
| [`usv_sim_full/docs/HISTORY_PATH.md`](HISTORY_PATH.md) | 本文档：功能汇总、文件清单、实现原理和配置使用说明 |

### 1.2 修改已有文件

| 文件 | 本功能的修改内容 |
|---|---|
| [`usv_sim_full/launch/main.launch.py`](../launch/main.launch.py) | 增加航迹节点启动、配置加载、启用开关和可选时长覆盖；给生成的会话 RViz 配置补入航迹显示，不自动添加设置面板，不修改外部底稿源文件 |
| [`usv_sim_full/setup.py`](../setup.py) | 注册 `history_path` 可执行入口，仅新增本航迹说明文档的安装规则；配置和 RViz 文件沿用原有安装机制 |
| [`usv_sim_full/package.xml`](../package.xml) | 声明 `action_msgs` 与 `tf2_geometry_msgs` 运行依赖 |
| [`usv_sim_full/rviz/three_vision_one_mmwave.rviz`](../rviz/three_vision_one_mmwave.rviz) | 加入“本船历史航迹”Path 显示项及 `env_panel/HistoryPathSettings` 面板 |
| [`env_panel/CMakeLists.txt`](../../env_panel/CMakeLists.txt) | 将设置面板源码加入现有插件库，并注册面板测试 |
| [`env_panel/plugin_description.xml`](../../env_panel/plugin_description.xml) | 注册插件类 `env_panel/HistoryPathSettings`，保留原有插件注册 |
| [`usv_sim_full/test/test_simulation_launch_contracts.py`](../test/test_simulation_launch_contracts.py) | 验证仿真层启动所有权、外部 RViz 底稿不被修改、重复补入不产生重复项及关闭开关 |
| [`usv_sim_full/README.md`](../README.md) | 增加历史航迹功能及本文档入口 |
| [`env_panel/README.md`](../../env_panel/README.md) | 增加历史航迹设置面板的使用入口 |
| [`usv_sim_full/CHANGELOG.md`](../CHANGELOG.md) | 记录航迹功能变更 |
| [`env_panel/CHANGELOG.md`](../../env_panel/CHANGELOG.md) | 记录设置插件新增 |

节点名称：`/history_path`。默认输入 `/usv_1/odom`，输出 `/usv_1/history_path`，输出类型为 `nav_msgs/msg/Path`。

## 2. 实现原理

```text
/usv_1/odom（本船实际定位）
    ↓
等待该定位时间戳对应的 TF
    ↓
转换到 map 坐标系，追加历史位置，移除过期点
    ↓
/usv_1/history_path（完整的当前历史航迹）
    ↓
RViz 原生 Path Display → 本船历史航迹
```

### 2.1 实时更新与 TF 等待

每收到一帧有效定位，节点就尝试转换并更新航迹，不按移动距离或固定采样间隔取点。

定位和 TF 来自不同消息，可能先后到达。若定位先到，节点将其放入待转换队列，等对应时间戳的 TF 可用后再处理，避免查询不到 TF 就丢帧。后续定位到达时会继续处理队列；没有新定位时，定时器也会尝试处理。待转换队列最多保留 100 帧，超过容量会移除最早的待处理定位。

转换后的历史位置固定保存在 `map` 中，并保留原始定位时间戳。每次发布的 Path 都包含当前保留的历史位置，因此 RViz 的 `Buffer Length` 应为 **1**。可见刷新速度还受定位输入频率、TF 到达延迟和 RViz 渲染帧率影响。

### 2.2 按时间自动移除

节点按以下规则裁剪历史位置：

```text
删除：位置时间戳 < 当前 ROS 时间 − history_duration_sec
```

当前 YAML 配置显示最近 90 秒。即使停止接收定位，定时器仍会清理过期点。仿真使用 ROS 仿真时间：暂停仿真时保留时长也暂停；时间倒退时清空历史及待转换定位。

`max_points` 同时限制内存和单条 Path 的大小。时间窗口和点数上限中，先达到的限制生效。例如 50 Hz 定位保留 300 秒，需至少 15000 点；默认 10000 点在持续 50 Hz 输入时约覆盖 200 秒。

### 2.3 新规划自动清空

以下任一事件都会触发清空：

| 入口 | 清空时机 |
|---|---|
| `/mission/waypoints` | 收到新规划请求时，不等待候选计算或航线确认 |
| `/route_planner/selected_path` | 收到非空的已选航线时 |
| `/usv_1/goal_pose` | 收到新的导航目标时 |
| `navigation_actions` 中配置的 Nav2 action | 状态中出现新的、已接受或正在执行的任务 UUID 时 |

默认监听的 action 为 `/usv_1/navigate_to_pose`、`/usv_1/navigate_through_poses` 和 `/usv_1/follow_waypoints`，对应状态话题为各 action 名称后加 `/_action/status`。

RViz 导航对话框可能直接发送 action，不经过前面三个话题，因此 action 状态监听也是必要的。相同任务 UUID 的重复状态不会重复清空。导航过程中周期更新的 `/usv_1/plan` 不作为清空触发条件。

清空时，节点同时移除已累计位置和待转换定位，并立即发布空 Path，擦除 RViz 上的旧线条；后续有效定位再形成新航迹。节点只清理本船实际历史航迹，不清除规划路径显示项。

## 3. 修改显示时长

### 3.1 在 RViz 插件面板中设置（推荐）

默认会话配置已加入“历史航迹设置”面板。如果使用其他 RViz 配置，按以下步骤手动添加：

1. 确保已在加载了工作区安装环境的终端启动 RViz，并与仿真使用相同的 ROS 通信域。
2. 点击顶部菜单 **Panels（面板）→ Add New Panel（添加新面板）**。
3. 在类型列表中选择 **`env_panel/HistoryPathSettings`**，点击 **OK**。
4. 面板连接 `/history_path` 节点后，会自动读取实际保留时长。
5. 修改“历史保留时长”，单位为秒，然后点击 **“应用”**，无需重启节点或 RViz。
6. 点击 **“读取当前值”** 可以再次查询节点当前的设置。

打开面板本身不会覆盖节点配置。这个插件只提供时间设置；航迹线仍由原生 Path 显示项绘制，添加方法见第 4 节。显示或隐藏航迹使用 Displays 中“本船历史航迹”前的勾选框。

插件只调用 `/history_path/get_parameters` 和 `/history_path/set_parameters`，只修改 `history_duration_sec`，不发布定位、路径、规划或控制命令。实时更新、TF 等待、过期裁剪和新任务清空逻辑仍由原来的 Python 节点负责。

航迹节点、配置和文档位于 `usv_simulation/usv_sim_full`，设置插件位于 `usv_simulation/env_panel`，无需修改 `usv_bringup` 或 RViz 本身。构建并加载环境：

```bash
cd /home/cczhtc/usv_ws_test
source /opt/ros/humble/setup.zsh
colcon build --packages-select env_panel usv_sim_full --symlink-install
source install/setup.zsh
```

构建后从加载了上述环境的终端重启原来的仿真启动入口。`usv_sim_full/main.launch.py` 默认启动航迹节点，在生成的会话 RViz 配置中加入历史航迹和面板；即使上层传入自己的 RViz 配置底稿，也由仿真层补入，不修改底稿源文件。因此原来的 `usv_bringup system.launch.py` 仿真入口仍可使用，无需在 bringup 中增加本功能的启动或配置代码。

需要手动添加面板时，选择 `Panels → Add New Panel → env_panel/HistoryPathSettings`。直接启动 RViz 或只启动导航、不运行仿真主入口时，可单独运行航迹节点：

```bash
ros2 run usv_sim_full history_path --ros-args -p use_sim_time:=true \
  --params-file "$(ros2 pkg prefix usv_sim_full)/share/usv_sim_full/config/history_path.yaml"
```

仿真已有该节点时，不要重复启动；非仿真场景使用 `use_sim_time:=false`。

面板中的修改是运行时设置，不会写回 YAML；节点重启后恢复启动配置。希望跨重启保留的默认值仍按下一节修改。节点未运行或通信域不一致时，面板显示“等待历史航迹节点”，不会影响其他功能。

### 3.2 在配置文件中持久设置

编辑：

```text
/home/cczhtc/usv_ws_test/src/usv_simulation/usv_sim_full/config/history_path.yaml
```

例如只显示最近两分钟：

```yaml
history_path:
  ros__parameters:
    history_duration_sec: 120.0
```

当前文件中的值是 `history_duration_sec: 90.0`；上面的 `120.0` 是修改示例。只修改现有文件中的这一项，保留其他配置。单位为秒，必须为大于 0 的有限数值；建议使用 `120.0` 这样的浮点写法。常用值为 `60.0`（一分钟）、`120.0`（两分钟）、`300.0`（五分钟）。

当前工作区采用 `--symlink-install` 安装，修改源配置后重启航迹节点或原来的仿真启动即可；不需要重启 RViz 来加载这个时长。仅重启 RViz 不会重新加载航迹节点的配置。

如果使用普通安装，修改源配置后先重新构建并加载环境，再重启原来的仿真入口：

```bash
cd /home/cczhtc/usv_ws_test
colcon build --packages-select usv_sim_full --symlink-install
source install/setup.zsh
```

这些命令只构建和加载环境，不会替换已经运行的进程。重启时沿用原来的启动入口，不要重复启动已有节点。

### 3.3 命令行运行时修改

在已加载相同 ROS 环境和通信域的终端执行：

```bash
ros2 param set /history_path history_duration_sec 120.0
ros2 param get /history_path history_duration_sec
```

无需重启，下一次定位处理或定时清理时生效。运行时修改不会写回 YAML，节点重启后恢复启动配置。缩短窗口后已删除的位置不会因为随后调大窗口而恢复。

### 3.4 启动时临时覆盖

直接启动仿真主入口时，可以显式指定：

```bash
ros2 launch usv_sim_full main.launch.py use_sim_time:=true history_duration_sec:=120.0
```

上述示例用于直接启动仿真主入口。`enable_history_path:=false` 可关闭该入口提供的航迹节点及会话配置补入；使用上层系统入口时，直接修改 `config/history_path.yaml` 中的默认时长或使用设置面板即可，无需向 bringup 增加参数。

生效顺序为：**配置文件 → 显式启动覆盖 → 运行时参数修改**。启动参数默认为空，因此未显式覆盖时，以 `history_path.yaml` 为准。

## 4. RViz 设置与其他参数

设置插件和航迹显示项需要分别添加：**Panel 负责设置时长，Path 负责画线**。默认会话配置已包含两者。使用其他配置时，按以下步骤添加航迹显示项：

1. 在左侧 **Displays（显示项）** 点击 **Add**。
2. 在 **By display type** 中选择 **Path**，点击 **OK**。
3. 将名称改为“本船历史航迹”，设置 Topic 为 `/usv_1/history_path`。
4. 将 **Buffer Length 设置为 1**，并按下表核对其他属性。

Path 关键属性：

| 属性 | 设置 |
|---|---|
| Global Options / Fixed Frame | `map` |
| Topic | `/usv_1/history_path` |
| Buffer Length | `1` |
| Line Style | `Billboards` |
| Line Width | `0.2` |
| Durability Policy | `Transient Local` |
| Reliability Policy | `Reliable` |

时间窗口由航迹节点管理，原生 Path 属性栏没有按秒设置的 `Decay Time`。`Buffer Length` 表示缓存多少条完整路径消息，不表示显示多少秒；设置大于 1 会叠加旧消息，影响清空效果。

其他配置也位于 `history_path.yaml`：

| 参数 | 默认值 | 用途 |
|---|---|---|
| `fixed_frame` | `map` | 保存与发布历史位置的坐标系 |
| `max_points` | `10000` | 历史点数上限，达到后移除最早点 |
| `cleanup_rate` | `5.0` | 无新定位时的待转换处理和过期清理频率，单位 Hz；不限制逐帧更新频率 |
| `path_topic` | `/usv_1/history_path` | 输出航迹话题 |
| `mission_topic`、`route_topic`、`goal_topic` | 见文件 | 新规划、新航线、新目标的清空入口 |
| `navigation_actions` | 三个 Nav2 action | 直接发送 action 的导航任务清空入口 |

除 `history_duration_sec` 外，这些节点参数在运行时只读，修改文件后需重启节点。输入定位话题由 `main.launch.py` 的 `odom_topic` 传入，默认 `/usv_1/odom`。


## 5. 关键实现位置与影响范围

| 位置 | 负责逻辑 |
|---|---|
| `history_path.py`：`Trail` | 保存带原时间戳的位置、限制点数、移除过期点 |
| `history_path.py`：`_on_odom`、`_flush_pending` | 接收定位、等待对应时刻的 TF、转换到输出坐标系后追加历史 |
| `history_path.py`：`_publish`、`_expire` | 发布完整历史 Path，处理无新定位时的待转换队列及过期点 |
| `history_path.py`：`_on_action_status`、`_reset` | 按新任务 UUID 识别任务切换，清空旧数据并立即发布空 Path |
| `main.launch.py`：`_history_path_action` | 加载 YAML，应用显式覆盖并启动独立节点 |
| `main.launch.py`：`_append_history_displays` | 仅向会话 RViz 配置补入航迹显示和设置面板 |
| `history_path_panel.cpp`：`refresh`、`apply`、`tick` | 异步读取和设置时长，在 Qt 定时回调中处理响应，避免阻塞面板 |

航迹节点只消费定位、TF 和任务事件，发布独立的 `/usv_1/history_path`；设置面板只读写 `/history_path` 的时长参数。清空航迹不会取消导航任务，也不会改变定位或控制指令。会增加与定位频率、保留点数相关的计算及路径消息传输开销。

本次功能不包含水深、雷达、船体控制或图形驱动的修改；同步远程版本得到的水深功能不计入本清单。仿真启动、插件构建和配置文件中的新增条目均为接入本航迹功能所需。


## 6. 使用时的常见问题

| 现象 | 检查方法 |
|---|---|
| 添加面板列表中没有 `HistoryPathSettings` | 先构建 `env_panel`，重新 `source install/setup.zsh`，再从这个终端启动 RViz；插件完整类名为 `env_panel/HistoryPathSettings` |
| 面板显示“等待历史航迹节点” | 用 `ros2 node list` 确认 `/history_path` 正在运行，并确认 RViz 与仿真使用相同 ROS 环境和通信域 |
| 修改 YAML 后没有立即变化 | YAML 不会热加载，需要重启航迹节点；当次运行即时调整使用面板或 `ros2 param set` |
| 重启后面板修改的时长恢复 | 面板只修改运行时参数；跨重启保留的值需写入 YAML |
| 只看到设置面板，没有航迹线 | 核对是否添加并启用了 Path 显示项，Topic 为 `/usv_1/history_path`，Fixed Frame 为 `map` |
| 新任务清空后仍叠着旧线 | 核对本船历史航迹的 Buffer Length 为 1，并与规划路径显示项区分 |

确认节点当前保留时长：

```bash
ros2 param get /history_path history_duration_sec
```

所有操作均针对历史航迹显示及其设置插件，不需要修改水深、雷达、导航控制或 RViz 图形驱动配置。
