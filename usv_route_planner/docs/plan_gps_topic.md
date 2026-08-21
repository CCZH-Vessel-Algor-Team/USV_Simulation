# `/route_planner/plan_gps` 话题数据接口文档

`/route_planner/plan_gps` 是 `usv_route_planner` 节点将 Nav2 全局规划路径
`/usv_1/plan`（`map` 局部米制坐标）转换后的 **WGS84 经纬度航线**话题，供上位机
直接按经纬度标绘，避免上位机自行做 UTM 换算导致的位置偏差。

配套话题 `/route_planner/plan_gps_path` 提供同一条航线的 `nav_msgs/Path` 形式
（`x = 经度, y = 纬度`）。

---

## 1. 基本信息

| 项目 | 内容 |
| :--- | :--- |
| 话题名 | `/route_planner/plan_gps` |
| 消息类型 | `usv_interfaces/msg/WaypointList` |
| 发布节点 | `usv_route_planner` |
| QoS | `rclcpp::QoS(10).reliable()`（深度 10，可靠，不保持历史） |
| 帧 / frame_id | `wgs84` |
| 坐标基准 | WGS84 经纬度（度），与候选航线 `gps_path` 完全一致 |

配套话题：

| 话题名 | 消息类型 | 说明 |
| :--- | :--- | :--- |
| `/route_planner/plan_gps_path` | `nav_msgs/msg/Path` | 同航线 Path 形式，`position.x = 经度`，`position.y = 纬度` |

---

## 2. 消息定义

### 2.1 `usv_interfaces/msg/WaypointList`

```
std_msgs/Header header
usv_interfaces/Waypoint[] waypoints
```

`std_msgs/Header`：

| 字段 | 类型 | 说明 |
| :--- | :--- | :--- |
| `stamp` | `builtin_interfaces/Time` | 与来源 `/usv_1/plan` 消息的时间戳一致 |
| `frame_id` | `string` | 固定为 `wgs84` |

### 2.2 `usv_interfaces/msg/Waypoint`

| 字段 | 类型 | 单位 | 说明 |
| :--- | :--- | :--- | :--- |
| `latitude` | `float64` | 度（°） | 航点纬度，WGS84 |
| `longitude` | `float64` | 度（°） | 航点经度，WGS84 |
| `heading_target` | `float64` | 弧度（rad） | 期望到达该点时的艏向角 |
| `speed_target` | `float32` | m/s | 航段期望速度，取自 `default_speed_mps`（默认 2.0） |

---

## 3. 字段语义说明

### 3.1 坐标基准

经纬度由来源 `/usv_1/plan` 的 `map` 局部米制坐标换算得到，换算公式与候选航线
`gps_path` 完全一致（`route_planner_node.cpp::mapToGps`）：

```text
map_x = UTM_easting  - 736235
map_y = UTM_northing - 3846381
```

UTM 投影参数（可配置）：

| 参数 | 默认值 |
| :--- | :--- |
| `utm_zone` | 50 |
| `utm_north` | true |
| `utm_reference_easting` | 736235.0 |
| `utm_reference_northing` | 3846381.0 |

### 3.2 `heading_target`

由相邻航点在 `map` 帧内的方向角计算：`atan2(Δy, Δx)`（ENU 数学约定，
0 rad = 正东，π/2 = 正北）。该角度与来源路径方向一致，与候选航线 `gps_path`
的 `heading_target` 计算方式相同。

### 3.3 航点密度

`/route_planner/plan_gps` 对 `/usv_1/plan` 的**全部 pose** 逐一转换，为密集航点序列。
注意与候选消息 `candidates.gps_path` 的区别：`gps_path` 只包含起点、LOS 关键拐点
和终点（稀疏）；`plan_gps` 是当前执行的完整规划路径。

### 3.4 与 `/route_planner/plan_gps_path` 的关系

两者由同一次转换同时发布，数据源相同：

| 话题 | 表示 | 位置字段 |
| :--- | :--- | :--- |
| `/route_planner/plan_gps` | `WaypointList` | `waypoints[i].latitude/longitude` |
| `/route_planner/plan_gps_path` | `nav_msgs/Path` | `poses[i].pose.position.x = 经度`，`.y = 纬度` |

---

## 4. 发布时机与更新频率

- 每当 `/usv_1/plan`（Nav2 `planner_server` 的全局规划路径）发布新消息时触发转换并发布；
- 无独立定时器，更新频率跟随 `/usv_1/plan` 的发布频率（一般为规划/执行期间持续发布）；
- 不进行历史保持（非 transient local），新订阅者只收到后续消息；
- 若 `/usv_1/plan` 的 `frame_id` 与 `chart_map_frame`（默认 `map`）不一致，消息将被忽略并告警一次。

---

## 5. 配置参数

参数均在 `usv_route_planner` 命名空间下，默认值见
[`config/route_planner.yaml`](../config/route_planner.yaml)：

| 参数 | 默认值 | 说明 |
| :--- | :--- | :--- |
| `enable_plan_gps_republish` | `true` | 是否启用 plan → GPS 转发 |
| `nav2_plan_topic` | `/usv_1/plan` | 输入的 Nav2 全局规划路径话题 |
| `plan_gps_topic` | `/route_planner/plan_gps` | 输出的 GPS 航线话题（WaypointList） |
| `plan_gps_path_topic` | `/route_planner/plan_gps_path` | 输出的 GPS 航线话题（Path） |

---

## 6. 使用示例

```bash
# 查看 GPS 航线（WaypointList）
ros2 topic echo /route_planner/plan_gps --once

# 查看 GPS 航线（Path 形式）
ros2 topic echo /route_planner/plan_gps_path --once

# 查看话题信息
ros2 topic info /route_planner/plan_gps -v
```

输出示例：

```yaml
header:
  stamp:
    sec: 1787306270
    nanosec: 275718669
  frame_id: wgs84
waypoints:
- latitude: 34.693078857514806
  longitude: 119.48004974992595
  heading_target: 0.2015174677980438
  speed_target: 2.0
```

---

## 7. 上位机接入建议

- 直接按 `latitude`/`longitude` 在电子海图上标绘，无需再做 UTM/ENU 换算；
- 不要对 `/usv_1/plan`（`map` 局部米制）自行用非 736235/3846381 基准换算，
  否则会出现整体偏移；
- 如需与候选航线叠加比对，两者的经纬度坐标基准一致（同一 `mapToGps` 算法）。

---

## 8. 相关文件

| 文件 | 说明 |
| :--- | :--- |
| `src/route_planner_node.cpp` | 实现（`onNav2Plan` / `makeGpsPath` / `makeGpsPathAsPath`） |
| `config/route_planner.yaml` | 参数配置 |
| `usv_interfaces/msg/WaypointList.msg` | 消息定义 |
| `usv_interfaces/msg/Waypoint.msg` | 航点字段定义 |
