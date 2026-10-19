// Copyright 2026 CCZH. MIT License.
//
// Linux /proc CPU 采样器及 RViz2 CPU Top-N 面板声明。

#ifndef ENV_PANEL__CPU_PANEL_HPP_
#define ENV_PANEL__CPU_PANEL_HPP_

#include <atomic>
#include <array>
#include <cstdint>
#include <deque>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace env_panel
{

struct ProcSample
{
  int pid = 0;
  double cpu_pct = 0.0;      ///< 5 s 平均 CPU%，100% = 1 个逻辑核
  std::string name;
  std::string cmd;
};

struct CpuSnapshot
{
  std::vector<ProcSample> processes;
  size_t visible_count = 0;
  double total_pct = 0.0;
  double system_pct = 0.0;
  long cores = 1;
};

class CpuSampler
{
public:
  CpuSampler();
  ~CpuSampler();

  void start();
  void stop();
  void set_paused(bool paused) {paused_.store(paused);}
  bool paused() const {return paused_.load();}
  void set_top_n(int n);
  CpuSnapshot snapshot() const;

private:
  struct ProcState
  {
    uint64_t jiffies = 0;
    uint64_t start_time = 0;
    std::deque<double> history;
    std::string name;
    std::string cmd;
  };

  void run();
  void sample_once();
  static double now_seconds();

  std::thread thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> paused_{false};
  mutable std::mutex mutex_;
  std::map<int, ProcState> state_;
  std::vector<ProcSample> sorted_;
  double total_pct_ = 0.0;
  double system_pct_ = 0.0;
  std::deque<double> system_history_;
  std::array<uint64_t, 8> sys_jiffies_{};
  int top_n_ = 5;
  size_t window_samples_ = 5;
  double clk_tck_ = 100.0;
  long cores_ = 1;
  double last_stamp_ = 0.0;
};

}  // namespace env_panel

#include <QColor>
#include <QString>

#include <rviz_common/panel.hpp>

class QComboBox;
class QLabel;
class QTableWidget;
class QTimer;
class QToolButton;

namespace env_panel
{

class CpuPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit CpuPanel(QWidget * parent = nullptr);
  ~CpuPanel() override;
  void onInitialize() override;

private Q_SLOTS:
  void refresh();
  void togglePause();
  void exportTsv();
  void onTopNChanged(int index);
  void onCellDoubleClicked(int row, int column);

private:
  void buildUi();
  void ensureRows(int rows);
  static QColor thresholdColor(double pct);

  CpuSampler sampler_;
  QTimer * timer_ = nullptr;
  QTableWidget * table_ = nullptr;
  QLabel * summary_ = nullptr;
  QLabel * footer_ = nullptr;
  QToolButton * pause_button_ = nullptr;
  QComboBox * topn_box_ = nullptr;
};

}  // namespace env_panel

#endif  // ENV_PANEL__CPU_PANEL_HPP_
