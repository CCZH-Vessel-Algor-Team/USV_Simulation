// Copyright 2026 CCZH. MIT License.
//
// Linux /proc CPU 采样器及 RViz2 CPU Top-N 面板实现。

#include "env_panel/cpu_panel.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <numeric>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <thread>

#include <dirent.h>
#include <unistd.h>

#include <QClipboard>
#include <QComboBox>
#include <QFile>
#include <QFileDialog>
#include <QFontDatabase>
#include <QGuiApplication>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QTextStream>
#include <QTimer>
#include <QToolButton>
#include <QVBoxLayout>

namespace env_panel
{
namespace
{
constexpr const char * kGreen = "#6cbf7a";
constexpr const char * kAmber = "#e0a44a";
constexpr const char * kRed = "#e05c5c";

const std::vector<std::string> kKeywords = {
  "gz sim", "gz-sim", "nav2_", "usv_", "late_fusion", "component_container",
  "robot_state_publisher", "parameter_bridge", "rviz", "ground_truth", "enc_ground",
  "radar_", "mmwave", "convert_to_track", "route_planner", "controller_server",
  "planner_server", "bt_navigator", "smoother_server", "behavior_server",
  "velocity_smoother", "waypoint_follower", "map_server", "lifecycle",
  "scenario_manager", "dynamic_ship", "dynamic_buoy", "storm_field", "tf_namespace",
  "odom_tf", "gz_spawn", "usv_sim_wrapper", "ros_gz", "colregs", "costmap",
  "maritime", "sim_vision", "sim_mmwave", "sim_ais", "tracked_ship", "depth_provider",
  "grounding", "ukc_", "hull_draft", "adaptive_radar", "vector_object",
  "target_snapshot", "route_depth", "static_transform", "clearing_scan",
  "cmd_vel_to_thruster", "ais_aggregator", "ais_perception",
};
const std::vector<std::string> kPathMarkers = {"/install/", "/opt/ros/humble/lib/"};
const std::vector<std::string> kExcludes = {"ccs_cpu_profile", "usv_cpu_panel"};

std::string to_lower(std::string text)
{
  std::transform(text.begin(), text.end(), text.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return text;
}

std::string read_file(const std::string & path)
{
  std::ifstream file(path, std::ios::in | std::ios::binary);
  if (!file) {throw std::runtime_error("open failed: " + path);}
  std::ostringstream out;
  out << file.rdbuf();
  return out.str();
}

std::string read_cmdline(int pid)
{
  std::string cmd = read_file("/proc/" + std::to_string(pid) + "/cmdline");
  std::replace(cmd.begin(), cmd.end(), '\0', ' ');
  const auto first = cmd.find_first_not_of(' ');
  if (first == std::string::npos) {return {};}
  return cmd.substr(first, cmd.find_last_not_of(' ') - first + 1);
}

struct ProcessStat
{
  uint64_t jiffies = 0;
  uint64_t start_time = 0;
};

bool read_process_stat(int pid, ProcessStat & out)
{
  const std::string stat = read_file("/proc/" + std::to_string(pid) + "/stat");
  const auto begin_name = stat.find('(');
  const auto end_name = stat.rfind(')');
  if (begin_name == std::string::npos || end_name == std::string::npos ||
    end_name <= begin_name || end_name + 2 >= stat.size())
  {
    return false;
  }
  std::istringstream fields(stat.substr(end_name + 2));
  std::string value;
  uint64_t user = 0;
  uint64_t system = 0;
  for (int index = 0; fields >> value; ++index) {
    if (index == 11) {user = std::strtoull(value.c_str(), nullptr, 10);}
    if (index == 12) {system = std::strtoull(value.c_str(), nullptr, 10);}
    if (index == 19) {out.start_time = std::strtoull(value.c_str(), nullptr, 10);}
  }
  out.jiffies = user + system;
  return out.start_time > 0;
}

bool read_system_cpu(std::array<uint64_t, 8> & times)
{
  std::ifstream file("/proc/stat");
  std::string tag;
  file >> tag;
  if (!file || tag != "cpu") {return false;}

  std::array<uint64_t, 10> raw{};
  for (auto & item : raw) {
    if (!(file >> item)) {return false;}
  }
  // /proc/stat 的 user/nice 已包含 guest/guest_nice；与 top 一样先扣除，避免重复。
  times = {raw[0] - raw[8], raw[1] - raw[9], raw[2], raw[3],
    raw[4], raw[5], raw[6], raw[7]};
  return true;
}

bool contains_any(const std::string & text, const std::vector<std::string> & words)
{
  return std::any_of(words.begin(), words.end(), [&](const std::string & word) {
    return text.find(word) != std::string::npos;
  });
}

bool interesting(const std::string & cmd)
{
  if (cmd.empty()) {return false;}
  const std::string lower = to_lower(cmd);
  if (contains_any(lower, kExcludes)) {return false;}
  return contains_any(lower, kPathMarkers) || contains_any(lower, kKeywords);
}

QTableWidgetItem * make_table_item(int column)
{
  auto * item = new QTableWidgetItem();
  if (column == 0 || column >= 2) {
    item->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
  }
  if (column != 0) {
    item->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
  }
  return item;
}

std::string short_name(const std::string & cmd)
{
  static const std::regex node_re("__node:=(\\S+)");
  static const std::regex ns_re("__ns:=(\\S+)");
  std::smatch match;
  std::string node;
  std::string node_namespace;
  if (std::regex_search(cmd, match, node_re)) {node = match[1].str();}
  if (std::regex_search(cmd, match, ns_re)) {node_namespace = match[1].str();}
  if (!node.empty()) {
    node_namespace.erase(0, node_namespace.find_first_not_of('/'));
    return node_namespace.empty() ? node : "/" + node_namespace + "/" + node;
  }
  if (cmd.find("gz sim gui") != std::string::npos) {return "gz-sim-gui";}
  if (cmd.find("gz sim server") != std::string::npos) {return "gz-sim-server";}
  if (cmd.find("gz") != std::string::npos && cmd.find("sim") != std::string::npos) {
    return "gz-sim";
  }

  std::istringstream args(cmd);
  std::string executable;
  for (std::string arg; args >> arg;) {
    if (executable.empty()) {executable = arg;}
    if (arg.find("/lib/") != std::string::npos ||
      (arg.size() > 3 && arg.compare(arg.size() - 3, 3, ".py") == 0))
    {
      executable = arg;
      break;
    }
  }
  const auto slash = executable.find_last_of('/');
  return slash == std::string::npos ? executable : executable.substr(slash + 1);
}
}  // namespace

CpuSampler::CpuSampler()
{
  clk_tck_ = static_cast<double>(sysconf(_SC_CLK_TCK));
  if (clk_tck_ <= 0.0) {clk_tck_ = 100.0;}
  cores_ = std::max(1L, sysconf(_SC_NPROCESSORS_ONLN));
  last_stamp_ = now_seconds();
}

CpuSampler::~CpuSampler()
{
  stop();
}

double CpuSampler::now_seconds()
{
  using Clock = std::chrono::steady_clock;
  return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
}

void CpuSampler::start()
{
  if (running_.exchange(true)) {return;}
  thread_ = std::thread(&CpuSampler::run, this);
}

void CpuSampler::stop()
{
  if (!running_.exchange(false)) {return;}
  if (thread_.joinable()) {thread_.join();}
}

void CpuSampler::set_top_n(int n)
{
  std::lock_guard<std::mutex> lock(mutex_);
  top_n_ = std::clamp(n, 1, 32);
}

void CpuSampler::run()
{
  sample_once();
  while (running_.load()) {
    for (int i = 0; i < 10 && running_.load(); ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (running_.load() && !paused_.load()) {sample_once();}
  }
}

void CpuSampler::sample_once()
{
  const double now = now_seconds();
  const double elapsed = std::max(1e-3, now - last_stamp_);
  last_stamp_ = now;

  double next_system_pct = system_pct_;
  bool system_updated = false;
  std::array<uint64_t, 8> system_times{};
  if (read_system_cpu(system_times)) {
    const uint64_t previous_total = std::accumulate(
      sys_jiffies_.begin(), sys_jiffies_.end(), uint64_t{0});
    const uint64_t current_total = std::accumulate(
      system_times.begin(), system_times.end(), uint64_t{0});
    if (previous_total > 0 && current_total > previous_total) {
      std::array<double, 8> percentages{};
      const double total_delta = static_cast<double>(current_total - previous_total);
      for (size_t i = 0; i < percentages.size(); ++i) {
        const uint64_t delta = system_times[i] >= sys_jiffies_[i] ?
          system_times[i] - sys_jiffies_[i] : 0;
        percentages[i] = 100.0 * static_cast<double>(delta) / total_delta;
      }
      // 与 top 的状态口径一致：us + ni + sy + hi + si + st；idle 和 iowait 不算忙碌。
      const double instant_busy = percentages[0] + percentages[1] + percentages[2] +
        percentages[5] + percentages[6] + percentages[7];
      system_history_.push_back(std::clamp(instant_busy, 0.0, 100.0));
      while (system_history_.size() > window_samples_) {system_history_.pop_front();}
      next_system_pct = std::accumulate(
        system_history_.begin(), system_history_.end(), 0.0) / system_history_.size();
      system_updated = true;
    }
    sys_jiffies_ = system_times;
  }

  DIR * dir = opendir("/proc");
  if (dir == nullptr) {return;}
  std::vector<int> pids;
  while (const dirent * entry = readdir(dir)) {
    const std::string name = entry->d_name;
    if (!name.empty() && std::all_of(name.begin(), name.end(), ::isdigit)) {
      pids.push_back(std::atoi(name.c_str()));
    }
  }
  closedir(dir);

  std::map<int, ProcState> fresh;
  for (const int pid : pids) {
    try {
      ProcessStat process_stat;
      if (!read_process_stat(pid, process_stat)) {continue;}
      const std::string cmd = read_cmdline(pid);
      if (!interesting(cmd)) {continue;}

      ProcState current;
      current.jiffies = process_stat.jiffies;
      current.start_time = process_stat.start_time;
      current.cmd = cmd;
      current.name = short_name(cmd);
      if (const auto previous = state_.find(pid);
        previous != state_.end() && previous->second.start_time == process_stat.start_time)
      {
        current.history = previous->second.history;
        const uint64_t delta = process_stat.jiffies >= previous->second.jiffies ?
          process_stat.jiffies - previous->second.jiffies : 0;
        current.history.push_back(100.0 * static_cast<double>(delta) / clk_tck_ / elapsed);
        while (current.history.size() > window_samples_) {current.history.pop_front();}
      }
      fresh.emplace(pid, std::move(current));
    } catch (const std::exception &) {
      // 进程可能在读取过程中退出，下一轮自然消失。
    }
  }
  state_ = std::move(fresh);

  std::vector<ProcSample> rows;
  rows.reserve(state_.size());
  for (const auto & [pid, state] : state_) {
    if (state.history.empty()) {continue;}
    rows.push_back({
      pid,
      std::accumulate(state.history.begin(), state.history.end(), 0.0) / state.history.size(),
      state.name,
      state.cmd});
  }
  std::sort(rows.begin(), rows.end(), [](const ProcSample & left, const ProcSample & right) {
    return left.cpu_pct != right.cpu_pct ? left.cpu_pct > right.cpu_pct : left.pid < right.pid;
  });
  const double sampled_total = std::accumulate(
    rows.begin(), rows.end(), 0.0,
    [](double sum, const ProcSample & sample) {return sum + sample.cpu_pct;});

  std::lock_guard<std::mutex> lock(mutex_);
  sorted_ = std::move(rows);
  total_pct_ = sampled_total;
  if (system_updated) {system_pct_ = next_system_pct;}
}

CpuSnapshot CpuSampler::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return {
    sorted_,
    std::min(sorted_.size(), static_cast<size_t>(top_n_)),
    total_pct_,
    system_pct_,
    cores_};
}

CpuPanel::CpuPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  buildUi();
  sampler_.start();

  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, &CpuPanel::refresh);
  timer_->start(1000);
}

CpuPanel::~CpuPanel()
{
  sampler_.stop();
}

void CpuPanel::onInitialize()
{
  // 本面板只读 Linux /proc，不需要 ROS 节点 / DisplayContext，因此这里不做额外初始化。
}

QColor CpuPanel::thresholdColor(double pct)
{
  if (pct >= 20.0) {return QColor(kRed);}
  if (pct >= 5.0) {return QColor(kAmber);}
  return QColor(kGreen);
}

void CpuPanel::buildUi()
{
  setMinimumWidth(360);

  auto * root = new QVBoxLayout(this);
  root->setContentsMargins(4, 4, 4, 4);
  root->setSpacing(4);

  // ── 工具条 ───────────────────────────────────────────────
  auto * bar = new QHBoxLayout();
  bar->setSpacing(4);
  auto * title = new QLabel(QStringLiteral("Top-N 进程 CPU 占用"), this);
  QFont tf = title->font();
  tf.setBold(true);
  title->setFont(tf);
  bar->addWidget(title);
  bar->addStretch(1);

  bar->addWidget(new QLabel(QStringLiteral("N:"), this));
  topn_box_ = new QComboBox(this);
  topn_box_->addItem(QStringLiteral("3"), 3);
  topn_box_->addItem(QStringLiteral("5"), 5);
  topn_box_->addItem(QStringLiteral("10"), 10);
  topn_box_->setCurrentIndex(1);
  connect(topn_box_, QOverload<int>::of(&QComboBox::currentIndexChanged),
    this, &CpuPanel::onTopNChanged);
  bar->addWidget(topn_box_);

  pause_button_ = new QToolButton(this);
  pause_button_->setText(QStringLiteral("暂停"));
  pause_button_->setToolButtonStyle(Qt::ToolButtonTextOnly);
  connect(pause_button_, &QToolButton::clicked, this, &CpuPanel::togglePause);
  bar->addWidget(pause_button_);

  auto * export_button = new QToolButton(this);
  export_button->setText(QStringLiteral("导出 TSV"));
  export_button->setToolButtonStyle(Qt::ToolButtonTextOnly);
  connect(export_button, &QToolButton::clicked, this, &CpuPanel::exportTsv);
  bar->addWidget(export_button);

  root->addLayout(bar);

  // ── 汇总（两行：整机利用率 / 采样合计） ───────────────────
  summary_ = new QLabel(this);
  summary_->setTextFormat(Qt::RichText);
  summary_->setWordWrap(true);
  root->addWidget(summary_);

  // ── 表格：# / 进程 / PID / CPU% / 占比 ────────────────────
  table_ = new QTableWidget(0, 5, this);
  table_->setHorizontalHeaderLabels(
    {QStringLiteral("#"), QStringLiteral("进程"), QStringLiteral("PID"),
      QStringLiteral("CPU%"), QStringLiteral("占比")});
  table_->verticalHeader()->setVisible(false);
  table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  table_->setSelectionBehavior(QAbstractItemView::SelectRows);
  table_->setSelectionMode(QAbstractItemView::SingleSelection);
  table_->setShowGrid(false);
  table_->setWordWrap(false);
  table_->horizontalHeader()->setStretchLastSection(false);
  table_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Fixed);
  table_->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
  // PID 位数随系统运行时间增长，并受桌面字体缩放影响，按内容自动留足宽度。
  table_->horizontalHeader()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
  table_->horizontalHeader()->setSectionResizeMode(3, QHeaderView::Fixed);
  table_->horizontalHeader()->setSectionResizeMode(4, QHeaderView::Fixed);
  table_->setColumnWidth(0, 30);    // #
  table_->setColumnWidth(3, 62);    // CPU%
  table_->setColumnWidth(4, 68);    // 占比
  table_->horizontalHeader()->setHighlightSections(false);
  table_->setFocusPolicy(Qt::NoFocus);
  table_->verticalHeader()->setDefaultSectionSize(22);

  connect(table_, &QTableWidget::cellDoubleClicked, this, &CpuPanel::onCellDoubleClicked);
  root->addWidget(table_, 1);

  // 仅在复制或导出后短暂显示状态；默认不占用底部空间。
  footer_ = new QLabel(this);
  footer_->setTextFormat(Qt::RichText);
  footer_->setWordWrap(true);
  root->addWidget(footer_);

  refresh();
}

void CpuPanel::ensureRows(int rows)
{
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < 5; ++c) {
      if (table_->item(r, c) != nullptr) {continue;}
      table_->setItem(r, c, make_table_item(c));
    }
  }
}

void CpuPanel::refresh()
{
  const CpuSnapshot snapshot = sampler_.snapshot();
  const auto & rows = snapshot.processes;
  const int n = static_cast<int>(snapshot.visible_count);
  table_->setRowCount(std::max(n, 1));
  ensureRows(std::max(n, 1));      // 行数变化后可能需要重建单元格

  if (n == 0) {
    table_->item(0, 0)->setText(QString());
    table_->item(0, 1)->setText(QStringLiteral("（等待采样…）"));
    table_->item(0, 2)->setText(QString());
    table_->item(0, 3)->setText(QString());
    table_->item(0, 4)->setText(QString());
  }

  double top_sum = 0.0;
  for (int i = 0; i < n; ++i) {
    const auto & s = rows[i];
    top_sum += s.cpu_pct;
    const QColor color = thresholdColor(s.cpu_pct);

    table_->item(i, 0)->setText(QString::number(i + 1));
    auto * name_item = table_->item(i, 1);
    name_item->setText(QString::fromStdString(s.name));
    name_item->setToolTip(QStringLiteral("PID %1\n%2")
      .arg(s.pid).arg(QString::fromStdString(s.cmd)));
    table_->item(i, 2)->setText(QString::number(s.pid));
    auto * cpu_item = table_->item(i, 3);
    cpu_item->setText(QString::number(s.cpu_pct, 'f', 1));
    cpu_item->setForeground(color);
    QFont f = cpu_item->font();
    f.setBold(s.cpu_pct >= 20.0);
    cpu_item->setFont(f);
  }

  const double total = snapshot.total_pct;
  const long cores = snapshot.cores;
  const double core_capacity = 100.0 * static_cast<double>(cores);
  const double sampled_share = core_capacity > 0.0 ? (total / core_capacity) * 100.0 : 0.0;
  const double sys_pct = snapshot.system_pct;

  // 每行的“占比”：占被采样进程合计的比例（所有被采样进程加起来 = 100%）。
  for (int i = 0; i < n; ++i) {
    auto * share_item = table_->item(i, 4);
    if (total > 0.0) {
      share_item->setText(QString::number(rows[i].cpu_pct / total * 100.0, 'f', 1));
    } else {
      share_item->setText(QStringLiteral("—"));
    }
  }

  summary_->setText(QStringLiteral(
    "USV运行算法：%1 个进程 · 占用总算力 <b>%2%</b>　·　5 s · 1 Hz<br>"
    "整机（5 s）<b>%3%</b>（%4/%5 核）　·　Top%6 合计 <b>%7%</b>")
    .arg(snapshot.processes.size())
    .arg(sampled_share, 0, 'f', 1)
    .arg(sys_pct, 0, 'f', 1)
    .arg(sys_pct / 100.0 * static_cast<double>(cores), 0, 'f', 1)
    .arg(cores)
    .arg(n)
    .arg(top_sum, 0, 'f', 1));
}

void CpuPanel::togglePause()
{
  const bool paused = !sampler_.paused();
  sampler_.set_paused(paused);
  pause_button_->setText(paused ? QStringLiteral("继续") : QStringLiteral("暂停"));
}

void CpuPanel::onTopNChanged(int index)
{
  const int n = topn_box_->itemData(index).toInt();
  sampler_.set_top_n(n > 0 ? n : 5);
  refresh();
}

void CpuPanel::onCellDoubleClicked(int row, int column)
{
  (void)column;
  auto * item = table_->item(row, 2);
  if (item == nullptr || item->text().isEmpty()) {return;}
  const QString cmd = QStringLiteral("ps -p %1 -o pid,pcpu,cmd").arg(item->text());
  QGuiApplication::clipboard()->setText(cmd);
  footer_->setText(QStringLiteral("已复制到剪贴板：<code>%1</code>").arg(cmd));
  QTimer::singleShot(2500, this, [this]() {footer_->clear();});
}

void CpuPanel::exportTsv()
{
  const QString path = QFileDialog::getSaveFileName(
    this, QStringLiteral("导出 CPU 采样"), QStringLiteral("cpu_sample.tsv"),
    QStringLiteral("TSV (*.tsv);;所有文件 (*)"));
  if (path.isEmpty()) {return;}

  QFile file(path);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
    footer_->setText(QStringLiteral("导出失败：%1").arg(file.errorString()));
    QTimer::singleShot(2500, this, [this]() {footer_->clear();});
    return;
  }
  const CpuSnapshot snapshot = sampler_.snapshot();
  QTextStream out(&file);
  out << "# cores=" << snapshot.cores
      << " system_pct=" << QString::number(snapshot.system_pct, 'f', 2)
      << " sampled_total_pct=" << QString::number(snapshot.total_pct, 'f', 2) << '\n';
  out << "cpu\tpid\tname\tcmd\n";
  for (const auto & s : snapshot.processes) {
    QString cmd = QString::fromStdString(s.cmd);
    cmd.replace('\t', ' ');
    cmd.replace('\n', ' ');
    out << QString::number(s.cpu_pct, 'f', 2) << '\t' << s.pid << '\t'
        << QString::fromStdString(s.name) << '\t' << cmd << '\n';
  }
  file.close();
  footer_->setText(QStringLiteral("已导出 %1 行 → <code>%2</code>")
    .arg(snapshot.processes.size()).arg(path));
  QTimer::singleShot(2500, this, [this]() {footer_->clear();});
}

}  // namespace env_panel

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(env_panel::CpuPanel, rviz_common::Panel)
