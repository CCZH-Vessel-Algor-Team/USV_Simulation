# third_party（ROS 2 Humble / 源码依赖）

本目录存放**与发行版对齐、从上游拉取的源码**，便于本地修改与固定版本。工作区根目录后续整理发行说明时，可将本节摘要并入主 `README.md`。

## marine_msgs（含 `marine_sensor_msgs`）

| 项 | 值 |
|----|-----|
| 上游 | https://github.com/apl-ocean-engineering/marine_msgs |
| 分支 | `ros2`（与 [rosdistro humble `marine_msgs` source](https://github.com/ros/rosdistro/blob/master/humble/distribution.yaml) 一致） |
| 抓取方式 | `git clone -b ros2` 后**已删除内层 `.git`**，作为普通目录纳入本工作区；更新上游时请对照下表重新 clone 或手动合并 |
| 本次抓取提交 | `14bddb417a51767600e3feada3bc7fc378d0e6f9` |
| 包内版本号 | `marine_sensor_msgs` / `marine_acoustic_msgs` 均为 **2.1.0**（与 humble release **2.1.0-1** 对应） |

**colcon 包**：`marine_acoustic_msgs`、`marine_sensor_msgs`（均在 `src/usv_simulation/third_party/marine_msgs/` 下）。

**构建示例**：

```bash
cd /path/to/USV_ROS
source /opt/ros/humble/setup.bash
colcon build --packages-select marine_acoustic_msgs marine_sensor_msgs --symlink-install
```

## foxglove-sdk（含 ROS 包 `foxglove_msgs`、`foxglove_bridge`）

| 项 | 值 |
|----|-----|
| 上游 | https://github.com/foxglove/foxglove-sdk |
| 路径 | `src/usv_simulation/third_party/foxglove-sdk/`（相对 USV_ROS 根目录） |
| 目录内 ROS 包 | `ros/src/foxglove_msgs`、`ros/src/foxglove_bridge` |
| 说明 | 仓库根目录无 `package.xml`，**colcon 仅会构建上述两个 ROS 包**；其余 Rust/TS/Python 为 SDK 本体，按需在上游仓库内开发 |
| **发行 / CI 建议** | 仿真主线**不依赖** Foxglove。若希望缩短 clone 与构建时间：可放置 `foxglove-sdk/ros/src/COLCON_IGNORE` 跳过两 ROS 包（说明见 [`foxglove-sdk/ros/COLCON_IGNORE.md`](foxglove-sdk/ros/COLCON_IGNORE.md)），或改为 submodule / 独立文档说明「可选安装」。集成仓当前**未启用** GitHub Actions CI。 |

若目录为完整 git clone，**内层保留 `.git`**，便于单独 `git pull` 同步上游；若希望主仓库以单树提交所有文件，可自行改为 submodule 或去掉内层 `.git`（需自行权衡体积与更新流程）。

**构建示例**：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select foxglove_msgs foxglove_bridge --symlink-install
```

## 与自有包的构建顺序

`usv_interfaces` 等依赖 `marine_sensor_msgs` 时，应先编 marine 接口包再编自有包，例如：

```bash
colcon build --packages-select marine_acoustic_msgs marine_sensor_msgs usv_interfaces --symlink-install
```

---

*文档生成日期：2026-03-28（抓取记录见上表 commit）。*
