# 认证会遇场景（certificate_case）

规则对照见 [senario_cule.md](senario_cule.md)。共 **41** 个场景（C1×10 + C2×5 + C3×15 + C4×11）。

## 批量生成

```bash
python3 src/usv_simulation/usv_sim_full/tools/generate_certificate_cases.py
```

会覆盖本目录下全部 `C*.yaml`（语义来自 `senario_cule.md` 表格）。生成后可用合并脚本逐案校验：

```bash
python3 src/usv_simulation/usv_sim_full/tools/merge_certi_config.py \
  --case src/usv_simulation/usv_sim_full/config/certificate_case/C1-001.yaml
```

## Case 文件格式（推荐）

```yaml
scenario_id: C1-001
description: 危险对遇

own_ship:
  initial_speed_knots: 10.0
  initial_heading_deg: 0.0

target_ships:
  - id: TS1
    type: head_on              # head_on | crossing_right | crossing_left | overtaking | overtaken
    is_dangerous: true
    target_dcpa_meters: 5.0
    target_tcpa_seconds: 10.0
    speed_knots: 14.0
    encounter_range_max_m: 50.0
    # 多船（C3/C4）：第二艘常用 sequence_index: 1；C3 依次会遇可加 spawn_delay_sec
```

默认初始距离：`min((本船速+目标速)×target_tcpa_seconds, encounter_range_max_m)`。

合并脚本写入 `scenario.dynamic_obstacles`（含 `spawn_heading_deg`、`spawn_delay_sec`）；`scenario_manager_node` 对延迟项使用一次性定时器生成。

仍支持旧版 `meta` + `encounter` 块。

## 场景索引

| 类别 | 编号 | 名称 |
|------|------|------|
| C1 | C1-001～010 | 单船：危险/非危险 对遇、右交叉、左交叉、追越、被追越 |
| C2 | C2-001～005 | 危险主目标 + 非危险远距干扰船（TS2） |
| C3 | C3-001～015 | 两艘危险船**依次**会遇（TS2 `spawn_delay_sec`≈95s） |
| C4 | C4-001～011 | 两艘危险船**同时**会遇 |

## 合并与启动

```bash
colcon build --packages-select usv_sim_full --symlink-install
source install/setup.bash

ros2 launch usv_sim_full certifi_launch.launch.py \
  case_config:=src/usv_simulation/usv_sim_full/config/certificate_case/C1-001.yaml
```

合并产物：`config/generated/<scenario_id>.merged.yaml`（`*.merged.yaml` 已 gitignore）。
