# USV_Simulation - 无人水面航行器仿真平台

[![ROS 2](https://img.shields.io/badge/ROS-2_Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange.svg)](https://gazebosim.org/docs/harmonic/)
[![VRX](https://img.shields.io/badge/VRX-Competition-green.svg)](https://github.com/osrf/vrx)
[![License](https://img.shields.io/badge/license-Apache_2.0-blue.svg)](../LICENSE)

本目录是工作区 **[USV_ROS](../README.md)** 中的仿真栈。支持矩阵与发版说明以仓库根 [README.md](../README.md)、[CHANGELOG.md](../CHANGELOG.md)、[docs/RELEASE_VERSIONING.md](../docs/RELEASE_VERSIONING.md) 为准。

基于 **ROS 2 Humble + Gazebo Harmonic（gz）+ VRX** 的无人水面航行器（USV）高保真度仿真平台。

## 📚 文档导航（权威入口）

| 读者 | 文档 |
|------|------|
| **日常使用者（推荐）** | [docs/docs_v4/QUICK_START.md](docs/docs_v4/QUICK_START.md) |
| **构建细节** | [docs/docs_v4/BUILD.md](docs/docs_v4/BUILD.md) |
| **Demo 参数** | [docs/docs_v4/DEMO_RUN.md](docs/docs_v4/DEMO_RUN.md) |
| **架构与目录** | [docs/docs_v3/仿真仓库结构说明.md](docs/docs_v3/仿真仓库结构说明.md) |
| **全栈架构** | [usv_sim_full/docs/nav2_sim_three_vision_mmwave_architecture.md](usv_sim_full/docs/nav2_sim_three_vision_mmwave_architecture.md) |

历史文档（v1/v2/v3，可能含旧版 Docker 表述）：[docs/docs_v1](docs/docs_v1/)、[docs/docs_v2](docs/docs_v2/)、[docs/docs_v3](docs/docs_v3/)。

## 🚀 快速开始

在工作区根目录 `<ws>`（包含 `src/`）：

```bash
./scripts/build_demo.sh
source /opt/ros/humble/setup.bash
source src/usv_nav/install/setup.bash
source install/setup.bash
ros2 launch usv_sim_full nav2_sim_three_vision_mmwave_bringup.launch.py
```

环境依赖与分步构建见 [docs/docs_v4/QUICK_START.md](docs/docs_v4/QUICK_START.md)。

## 🐛 常见问题

```bash
# Gazebo 资源路径
export GZ_SIM_RESOURCE_PATH="$(ros2 pkg prefix wamv_description)/share/wamv_description/models:${GZ_SIM_RESOURCE_PATH:-}"

# 依赖
rosdep install --from-paths src/usv_interfaces src/usv_simulation --ignore-src -r -y
```

## 📄 许可证

工作区默认以 [Apache 2.0](../LICENSE) 分发；第三方组件见 [third_party/README.md](third_party/README.md)。
