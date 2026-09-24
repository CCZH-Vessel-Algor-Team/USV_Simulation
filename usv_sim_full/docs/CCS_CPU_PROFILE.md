# CCS / USV 仿真 CPU 监测工具

脚本：`tools/ccs_cpu_profile.py`

对已启动的 CCS / USV 仿真相关进程做 **Linux `/proc` 差分采样**，输出进程级与类别级 CPU 排名。  
**不依赖** `ros2 node list` / DDS 发现，适合节点列表不全时做负载画像。

## 环境要求

| 项 | 要求 |
|----|------|
| OS | **仅 Linux**（依赖 `/proc`） |
| Python | 3.8+（标准库即可，无第三方依赖） |
| ROS | 不强制；仿真需已在本机跑起来 |
| 权限 | 能读目标进程的 `/proc/<pid>/stat` 与 `cmdline` |

macOS / Windows **不能直接运行**。

## 快速开始

仿真已拉起后，在任意目录：

```bash
# 源码路径（git pull 后即可用，无需先 colcon）
python3 src/usv_simulation/usv_sim_full/tools/ccs_cpu_profile.py --duration 30

# 或安装后
source install/setup.bash
python3 "$(ros2 pkg prefix usv_sim_full)/share/usv_sim_full/tools/ccs_cpu_profile.py" --duration 30
```

工作区根目录名因机器而异时，按实际路径替换 `src/usv_simulation/...`。

## 常用参数

```bash
python3 .../ccs_cpu_profile.py \
  --duration 30 \
  --min-pct 0.4 \
  --top 55 \
  --tsv /tmp/ccs_cpu_rank.tsv \
  --path-marker /home/you/ws/install/ \
  --keyword my_extra_node
```

| 参数 | 含义 | 默认 |
|------|------|------|
| `--duration` | 采样窗口（秒） | `30` |
| `--min-pct` | 忽略低于该平均 CPU% 的进程 | `0.4` |
| `--top` | 进程表最多打印行数；`0` 表示全部 | `55` |
| `--path-marker` | 额外 cmdline 路径子串（可重复） | 内置 `/install/`、`/opt/ros/humble/lib/` |
| `--keyword` | 额外 cmdline 关键词（可重复） | 内置 CCS/Nav2/融合等相关词 |
| `--exclude` | 逗号分隔、从采样中排除的子串 | `ccs_cpu_profile` |
| `--tsv` | 写出 TSV：`cpu pid name cmd` | 不写 |
| `--no-category` | 不打印类别汇总 | 关 |

查看帮助：

```bash
python3 .../ccs_cpu_profile.py -h
```

## 输出说明

- **CPU%**：采样窗口内平均占用；**100% = 1 个逻辑核**（多线程可 >100%）。
- **NAME**：优先用 ROS `__ns:=` / `__node:=`；否则用可执行文件名；Gazebo 归为 `gz-sim-server` / `gz-sim-gui`。
- **aggregated by name**：同名进程 CPU 相加。
- **by category**：粗粒度归类（Gazebo / Nav2 / late_fusion / ENC 等），便于横向对比机器。

## 换机同步测试建议

1. 在 `USV_Simulation` 仓库检出本工具所在分支并 pull。
2. 本机照常拉起 CCS 认证仿真（或等价完整栈）。
3. 若 workspace 不在默认路径、或进程 cmdline 不含 `/install/`，加上本机 install 前缀：

```bash
python3 tools/ccs_cpu_profile.py \
  --duration 30 \
  --path-marker "$HOME/your_ws/install/" \
  --tsv /tmp/ccs_cpu_rank.tsv
```

4. 对比两台机器的类别汇总与 Top 进程；注意核数/睿频不同会导致绝对 CPU% 不可直接等同。

## 方法说明（简要）

1. 扫描 `/proc/<pid>/cmdline`，按路径标记与关键词过滤。
2. 读取 `/proc/<pid>/stat` 的 `utime+stime` 作为起点。
3. `sleep(duration)` 后再读一次，用 `SC_CLK_TCK` 换算为 CPU%。
4. 不启动、不清理仿真；本工具只做观测。

## 局限

- 只覆盖 cmdline 能匹配到的进程；纯短名且无关键词的进程需 `--keyword` / `--path-marker`。
- 不解析容器/cgroup 配额；在 Docker 内采样时解读需结合容器 CPU 限制。
- 瞬时尖峰会被窗口平均抹平；需要尖峰可把 `--duration` 调小多采几次。
