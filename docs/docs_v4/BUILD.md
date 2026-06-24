# 构建说明（docs_v4）

## 工作区结构

```text
<ws>/                    # usv_ws 集成仓（colcon 根）
├── src/
│   ├── usv_interfaces/
│   ├── usv_simulation/  # 仿真栈（usv_sim_full 等）
│   ├── usv_nav/         # Nav2 + COLREGS 源码（COLCON_IGNORE，单独编进 install/）
│   ├── usv_perception/
│   └── usv_fusion/
├── scripts/
│   ├── build_demo.sh    # 推荐：nav → install/ + sim 一键
│   └── clean_build.sh   # 主工作区构建（自动 source install/）
├── build/
│   ├── usv_nav/         # usv_nav 独立 build 产物
│   └── ...
├── log/
│   └── usv_nav/         # usv_nav 独立 log
└── install/             # 统一 install（nav + 仿真 + 业务包）
```

`usv_nav` 源码在 `src/usv_nav/`，根目录有 **`COLCON_IGNORE`**，主工作区全量 `colcon build` 会跳过它。  
Demo 脚本通过 `--base-paths src/usv_nav/src --install-base install` 将其**写入同一 `install/`**。

---

## 推荐命令

```bash
cd <ws>
./scripts/build_demo.sh
```

等价于：

1. `colcon build` @ `src/usv_nav/src` → 输出到 `<ws>/install/`（build 在 `build/usv_nav/`）
2. `colcon build --packages-up-to usv_sim_full` @ `<ws>/`

---

## `--packages-up-to usv_sim_full` 会拉起的包

由 `usv_sim_full/package.xml` 的 `depend` / `exec_depend` 递归决定，包括但不限于：

| 类别 | 包（示例） |
|------|------------|
| 接口 | `usv_interfaces` |
| 仿真核心 | `usv_sim_full`、`wamv_gazebo`、VRX 相关包 |
| 传感器 | `usv_mmwave_sim`、`gy_radar_driver`、`radar_gz_bridge` |
| 真值 / 融合 | `ground_truth_sensor_sim`、`ground_truth_sim`、`usv_late_fusion` |
| Nav2 接入 | `convert_to_trackship`（**编译期**依赖 `nav2_colregs_msgs`，来自 `install/` 中的 nav 包） |

**运行时** Nav2 / COLREGS 包同样在 **`source install/setup.bash`** 中，无需再 source 独立 underlay。

---

## 仅编 usv_nav（手动）

```bash
cd <ws>
source /opt/ros/humble/setup.bash
colcon --log-base log/usv_nav build \
  --symlink-install --parallel-workers 2 \
  --base-paths src/usv_nav/src \
  --build-base build/usv_nav \
  --install-base install \
  --allow-overriding nav2_common nav2_msgs navigation2
```

---

## rosdep 范围

仿真主线推荐：

```bash
rosdep install --from-paths \
  src/usv_interfaces \
  src/usv_simulation \
  src/usv_perception \
  src/usv_fusion \
  --ignore-src -r -y
```

Nav2 依赖（首次编 nav 前）：

```bash
rosdep install --from-paths src/usv_nav/src --ignore-src -r -y
```

`-r`：个别键缺失时继续（如实机雷达 SDK）；主链路由 `packages-up-to` 约束。

---

## 仅编接口或单包

```bash
./scripts/clean_build.sh --packages-select usv_interfaces
./scripts/clean_build.sh --packages-select gy_radar_driver usv_sim_full
```

> 编译依赖 Nav2 的包前，须先通过 `build_demo.sh` 或上文手动命令将 nav 写入 `install/`。

---

## 清理重编

跨机器拷贝工作区或 CMake 报旧路径时：

```bash
cd <ws>
rm -rf build install log
./scripts/build_demo.sh
```

仅重编 nav：

```bash
rm -rf build/usv_nav log/usv_nav install/nav2_* install/nav2_colregs_*
./scripts/build_demo.sh
```

---

## 与 CI 对齐

GitHub Actions 仿真 job 应：

1. `ubuntu-22.04` + ROS Humble
2. 将 `usv_nav` 编进 `<ws>/install/`（与 `build_demo.sh` 中 nav 步骤一致）
3. `./scripts/clean_build.sh --packages-up-to usv_sim_full`

见仓库 [`.github/workflows/ci.yml`](../../../../.github/workflows/ci.yml)（若 CI 仍使用 `src/usv_nav/install`，需与脚本策略同步更新）。
