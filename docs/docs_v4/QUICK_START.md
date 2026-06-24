# 快速开始（docs_v4）

面向 **本机 Ubuntu 22.04**，不依赖 Docker。默认演示 **三视觉 + 毫米波 + 后融合 + Nav2 COLREGS** 全栈仿真。

## 0. 支持矩阵

| 项 | 要求 |
|----|------|
| OS | Ubuntu **22.04** (Jammy) |
| ROS 2 | **Humble**（`ros-humble-desktop` 或 `ros-humble-ros-base` + 常用工具） |
| Gazebo | **Harmonic** + `ros-humble-ros-gzharmonic` |
| 导航 | 自建 **`src/usv_nav`**（由 `build_demo.sh` 编入主工作区 `install/`） |
| 工作区 | 含 `src/` 的 **usv_ws** 根目录（下文记为 `<ws>`） |

发版与版本规则见 [`docs/RELEASE_VERSIONING.md`](../../../../docs/RELEASE_VERSIONING.md)。

---

## 1. 系统依赖（首次）

```bash
# ROS 2 Humble（若尚未安装）
sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions python3-rosdep

# Gazebo Harmonic 桥接
sudo apt install -y ros-humble-ros-gzharmonic

# 仿真常见依赖（GPS 驱动等）
sudo apt install -y ros-humble-nmea-navsat-driver

# rosdep 初始化（每台机器一次）
sudo rosdep init || true
rosdep update
```

将 ROS 写入 shell（示例）：

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

---

## 2. 获取代码

在任意目录克隆集成仓（路径自定，不要写死用户主目录）：

```bash
git clone <usv_ws-repo-url> usv_ws
cd usv_ws
```

若已启用 git 子模块，执行 `git submodule update --init --recursive`。  
子模块尚未落地时，确保 `src/usv_simulation` 与 `src/usv_nav` 目录已存在。

---

## 3. 构建

### 3.1 一键（推荐）

在 `<ws>` 根目录：

```bash
chmod +x scripts/build_demo.sh scripts/clean_build.sh
./scripts/build_demo.sh
```

脚本会：

1. 若 `install/` 中尚无 nav 包，将 **`src/usv_nav`** 编入主工作区 **`install/`**（build 在 `build/usv_nav/`）
2. 调用 **`scripts/clean_build.sh --packages-up-to usv_sim_full`** 构建仿真主线

### 3.2 分步（便于排错）

```bash
cd <ws>
source /opt/ros/humble/setup.bash

# A. 导航包 → 写入 install/
rosdep install --from-paths src/usv_nav/src --ignore-src -r -y
colcon --log-base log/usv_nav build \
  --symlink-install --parallel-workers 2 \
  --base-paths src/usv_nav/src \
  --build-base build/usv_nav \
  --install-base install \
  --allow-overriding nav2_common nav2_msgs navigation2

# B. 主工作区
rosdep install --from-paths src/usv_interfaces src/usv_simulation src/usv_perception src/usv_fusion --ignore-src -r -y
source install/setup.bash
./scripts/clean_build.sh --packages-up-to usv_sim_full
```

> **说明**：必须使用 **`--packages-up-to usv_sim_full`**，以便按 `package.xml` 递归编译 `gy_radar_driver`、`usv_late_fusion`、`convert_to_trackship` 等依赖。仅 `--packages-select usv_sim_full` 不足以跑全栈 Demo。

更多构建说明见 [BUILD.md](BUILD.md)。

---

## 4. 运行环境

每个新终端：

```bash
cd <ws>
source /opt/ros/humble/setup.bash
source install/setup.bash
```

可选：Gazebo 模型路径（若资源找不到）：

```bash
export GZ_SIM_RESOURCE_PATH="$(ros2 pkg prefix wamv_description)/share/wamv_description/models:${GZ_SIM_RESOURCE_PATH:-}"
```

---

## 5. 启动 Demo

### 5.1 最小仿真（无 Nav2）

```bash
ros2 launch usv_sim_full main.launch.py
```

### 5.2 正式 Demo：三视觉 + 毫米波 + Nav2 全栈

```bash
ros2 launch usv_sim_full nav2_sim_three_vision_mmwave_bringup.launch.py
```

常用变体：

```bash
# 无 GUI（服务器 / CI 冒烟）
ros2 launch usv_sim_full nav2_sim_three_vision_mmwave_bringup.launch.py gz_headless:=true

# 仅仿真 + 感知，不启 Nav2
ros2 launch usv_sim_full nav2_sim_three_vision_mmwave_bringup.launch.py enable_nav2:=false
```

架构与话题说明见 [`usv_sim_full/docs/nav2_sim_three_vision_mmwave_architecture.md`](../../usv_sim_full/docs/nav2_sim_three_vision_mmwave_architecture.md)。  
TF 见 [`docs/sim_tf_tree.md`](../../../../docs/sim_tf_tree.md)。

---

## 6. 最小验证

```bash
ros2 node list | head
ros2 topic list | grep -E 'tracked_ship|fusion|scan'
ros2 run tf2_tools view_frames   # 可选：检查 TF 树
```

---

## 7. 常见问题

| 现象 | 处理 |
|------|------|
| `package 'usv_late_fusion' not found` | 使用 `./scripts/build_demo.sh` 或 `--packages-up-to usv_sim_full` 重编 |
| `nav2_colregs_msgs` 编译失败 | 先运行 `./scripts/build_demo.sh` 或按 [BUILD.md](BUILD.md) 将 nav 编进 `install/` |
| CMake 路径指向旧机器 | `rm -rf build install log` 后 `./scripts/build_demo.sh` |
| Gazebo 找不到模型 | 设置上文 `GZ_SIM_RESOURCE_PATH` |
| RViz 无相机图 | Image 显示 QoS 改为 **Best Effort**；或用 `sim_test` 自检 |

---

## 8. 下一步

- 配置索引：[`usv_sim_full/config/notes_config.md`](../../usv_sim_full/config/notes_config.md)
- Launch 参数详解：[DEMO_RUN.md](DEMO_RUN.md)
- 变更记录：根目录 [`CHANGELOG.md`](../../../../CHANGELOG.md)
