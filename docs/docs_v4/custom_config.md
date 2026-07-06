# 自建仿真配置包指南

本文说明如何**复制现有 YAML、重命名并放到独立目录**，在不写新 Launch 的前提下，用 `main.launch.py`（或上层 bringup）启动你自己的仿真场景。

适用读者：需要多套场景（不同世界、船位、传感器组合），但希望复用现有 `session_manager` + `main.launch` 机制的用户。

**整机字段含义**：见 [main_launch.md](main_launch.md) §4 与 [`full_config.reference.yaml`](../../usv_sim_full/config/full_config.reference.yaml)。

---

## 1. 机制说明

```
你的 config/<场景名>/
    ├── full_config.yaml      ← main.launch 的 config_path
    └── sensor_config.yaml    ← 由 full_config 顶栏 sensor_config_path 引用
            │
            ▼
    session_manager 解析 → 会话目录（URDF、桥接、RViz…）
            │
            ▼
    main.launch.py 拉起 Gazebo + 多船 + 场景节点
```

- **不需要新 Launch 文件**：只要 YAML 结构兼容 `full_config`，`config_path:=` 指向你的 `full_config.yaml` 即可。
- **相对路径基准**：`sensor_config_path`、`mesh_profile`、`gazebo_mesh_profile` 等路径，均相对于**当前这份 `full_config.yaml` 所在目录**解析（与 `session_manager` 行为一致）。
- **修改后生效**：改 YAML → **重启 launch**；不要改 `/tmp/usv_sim_sessions/...` 里的生成物。

---

## 2. 推荐目录布局

在包内新建子目录（与 `three_vision_one_mmwave/` 同级）：

```text
usv_sim_full/config/my_open_water/
├── full_config.yaml
└── sensor_config.yaml
```

也可放在工作区任意位置，启动时用**绝对路径**或**相对 cwd 的路径**传入 `config_path`。

---

## 3. 分步操作

### 步骤 1：选择模板并复制

按需求选一个最接近的模板目录或文件：

| 模板 | 路径 | 适合 |
|------|------|------|
| 默认整机 | `config/full_config.yaml` + `config/sensor_config.yaml` | 通用多传感器 |
| 毫米波最小 | `config/mmwave_sydney_minimal.yaml` | 轻量、少障碍 |
| 三视觉 Demo | `config/three_vision_one_mmwave/` 整个目录 | 视觉 + 毫米波 + 真值航路点 |
| 真值隔离测试 | `full_config.yaml` 或 `three_vision_one_mmwave/full_config.yaml` | `ground_truth_test.launch.py config_path:=...`（见 [main_launch.md](main_launch.md)） |
| 认证基底 | `config/certi_senario.yaml` | 单船认证（会遇用 `certifi_launch`，见 [certifi_launch.md](certifi_launch.md)） |

**复制示例**（在仓库内）：

```bash
cd src/usv_simulation/usv_sim_full/config
cp -r three_vision_one_mmwave my_open_water
# 或仅复制两个文件：
mkdir -p my_open_water
cp full_config.yaml my_open_water/
cp sensor_config.yaml my_open_water/
```

### 步骤 2：重命名与改标识

1. 目录名即场景包名（自定，如 `my_open_water`）。
2. 编辑 `my_open_water/full_config.yaml` 顶部注释，写明用途。
3. 建议改 `robot_1.name`（避免与其它场景默认 `usv_1` 混淆时的话题冲突 — 通常单实例无妨）。
4. 改 `environment.world_name`、`robot_1.spawn_pose` 等核心项。

### 步骤 3：修正相对路径（关键）

复制后**必须检查**下列路径是否仍指向正确文件：

| 在 full_config 中的键 | 典型写法（配置在同目录） | 说明 |
|----------------------|--------------------------|------|
| `sensor_config_path` | `sensor_config.yaml` | 与 `three_vision_one_mmwave` 一致，勿写 `config/sensor_config.yaml` 除非文件在子目录 |
| `scenario.dynamic_obstacles[].mesh_profile` | `../../description/models/...` | 相对 **full_config 所在目录** |
| `scenario.ground_truth_sim.gazebo_mesh_profile` | 同上 | Demo 模板常用 `../../description/...` |
| `robot_1.overrides.visual_mesh` | `package://usv_sim_full/...` | `package://` 与目录无关，一般不用改 |

**错误示例**：从 `config/full_config.yaml` 复制到 `config/my_open_water/full_config.yaml` 后，仍写 `sensor_config_path: config/sensor_config.yaml` — 会解析为 `config/my_open_water/config/sensor_config.yaml`（不存在）。

**正确示例**：

```yaml
sensor_config_path: sensor_config.yaml
```

若内参要与默认包共用、不复制 `sensor_config.yaml`：

```yaml
sensor_config_path: ../sensor_config.yaml
```

### 步骤 4：注册到安装规则（仅当放在 `usv_sim_full/config/<新目录>/`）

`setup.py` 默认只安装部分子目录。新建目录后需增加 `data_files`，否则 `colcon build` 后 `ros2 pkg prefix` 下找不到你的配置：

```python
# usv_sim_full/setup.py 中 data_files 增加一项，例如：
(os.path.join('share', package_name, 'config', 'my_open_water'),
 glob('config/my_open_water/*')),
```

然后：

```bash
colcon build --packages-select usv_sim_full --symlink-install
source install/setup.bash
```

**免改 setup.py 的两种方式**：

1. **开发期**：`config_path` 直接指向**源码树**路径（`symlink-install` 下改 YAML 即生效）：
   ```bash
   ros2 launch usv_sim_full main.launch.py \
     config_path:=src/usv_simulation/usv_sim_full/config/my_open_water/full_config.yaml
   ```
2. **包外配置**：YAML 放在任意路径，用绝对路径传入 `config_path`。

### 步骤 5：启动与验证

```bash
source install/setup.bash

ros2 launch usv_sim_full main.launch.py \
  config_path:=src/usv_simulation/usv_sim_full/config/my_open_water/full_config.yaml

# 预检：不启 Gazebo，只看 session_manager 是否成功
ros2 run usv_sim_full session_manager \
  --config-path src/usv_simulation/usv_sim_full/config/my_open_water/full_config.yaml \
  --verbose
```

验证清单：

```bash
ros2 node list | grep -E 'scenario_manager|usv_sim'
ros2 topic list | grep <你的 robot_1.name>
```

### 步骤 6：（可选）给上层 bringup 用同一配置

三视觉全栈默认读 `three_vision_one_mmwave/full_config.yaml`。换成你的包时：

```bash
ros2 launch usv_sim_full nav2_sim_three_vision_mmwave_bringup.launch.py \
  config_path:=src/usv_simulation/usv_sim_full/config/my_open_water/full_config.yaml \
  enable_nav2:=false   # 先只验证仿真侧
```

若你的场景**没有** `ground_truth_sensor_sim` 所需真值航路点，全栈感知链可能无数据 — 此时仅用 `main.launch.py` 更合适。全栈要求见 [DEMO_RUN.md](DEMO_RUN.md)。

---

## 4. 最小 `full_config` 骨架

从零写时可参考（字段详解见 reference）：

```yaml
environment:
  world_name: sydney_regatta
sensor_config_path: sensor_config.yaml

obstacles:
  fixed_list: []

scenario:
  dynamic_obstacles: []
  ground_truth_sim:
    enabled: false

visualization:
  launch_rviz: true
  enable_telemetry: true

robot_1:
  name: usv_1
  enable_env_dynamics: true
  xacro_template: wamv_no_battery.urdf.xacro
  spawn_pose: [0.0, 0.0, 0.5, 0.0, 0.0, 0.0]
  sensors: []   # 从模板复制完整 sensors 列表更省事
```

---

## 5. 与认证会遇路径的区别

| 方式 | 配置来源 | Launch |
|------|----------|--------|
| 自建包（本文） | 直接编辑 `full_config.yaml` | `main.launch.py config_path:=...` |
| 认证会遇 | `certi_senario.yaml` + `certificate_case/*.yaml` → **合并** | `certifi_launch.launch.py` |

认证场景的目标船由 `merge_certi_config.py` 写入 `scenario.dynamic_obstacles`，不要与手写 `dynamic_obstacles` 混用同一流程。见 [certifi_launch.md](certifi_launch.md)。

---

## 6. 常见问题

| 现象 | 原因 | 处理 |
|------|------|------|
| `sensor_config` / mesh 找不到 | 相对路径未按新目录调整 | 对照 §3 步骤 3 |
| install 后找不到配置 | 新子目录未写入 `setup.py` | §3 步骤 4，或用源码路径 |
| `session_manager` 报错 | YAML 语法或必填字段缺失 | `--verbose` 单独运行；对照模板 |
| 与默认 Demo 行为不一致 | 漏复制 `sensor_config` 或 sensors 列表 | 整目录复制 `three_vision_one_mmwave` 再删减 |
| 改了文件没生效 | 未重启 launch；误改会话目录 | 重启；只改源 YAML |

---

## 7. 相关文档

| 文档 | 说明 |
|------|------|
| [main_launch.md](main_launch.md) | `main.launch` 参数、生成逻辑、常用字段 |
| [`config/notes_config.md`](../../usv_sim_full/config/notes_config.md) | 配置文件与消费者对照 |
| [`full_config.reference.yaml`](../../usv_sim_full/config/full_config.reference.yaml) | 字段字典 |
| [certifi_launch.md](certifi_launch.md) | 认证 YAML 合并启动 |
