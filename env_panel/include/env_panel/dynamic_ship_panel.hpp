#ifndef ENV_PANEL__DYNAMIC_SHIP_PANEL_HPP_
#define ENV_PANEL__DYNAMIC_SHIP_PANEL_HPP_

#include <memory>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

#include <QComboBox>
#include <QElapsedTimer>
#include <QMap>
#include <QLabel>
#include <QListWidget>
#include <QPushButton>
#include <QSlider>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/string.hpp>
#include <usv_interfaces/msg/dynamic_ship_snapshot.hpp>
#include <usv_interfaces/srv/delete_dynamic_ship.hpp>
#include <usv_interfaces/srv/clear_dynamic_ships.hpp>
#include <usv_interfaces/srv/list_dynamic_ship_models.hpp>
#include <usv_interfaces/srv/set_dynamic_ship_config_v2.hpp>

namespace env_panel
{

class DynamicShipPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit DynamicShipPanel(QWidget * parent = nullptr);
  ~DynamicShipPanel() override;

  void onInitialize() override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config & config) override;

private Q_SLOTS:
  void onHeadingChanged(int value);
  void onSpeedChanged(int value);
  void onLegDistChanged(int value);
  void onModelChanged(int index);
  void onDeleteClicked();
  void onClearAllClicked();
  void publishConfig();

private:
  friend class DynamicShipPanelTestPeer;
  void setupUi();
  void onNamesReceived(const std_msgs::msg::String::SharedPtr msg);
  void refreshCatalog();
  void updateModelSummary();
  void updateShipList();
  void configEdited();

  rclcpp::Node::SharedPtr client_node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread spin_thread_;

  rclcpp::Client<usv_interfaces::srv::DeleteDynamicShip>::SharedPtr
      delete_client_;
  rclcpp::Client<usv_interfaces::srv::ClearDynamicShips>::SharedPtr
      clear_client_;
  rclcpp::Client<usv_interfaces::srv::SetDynamicShipConfigV2>::SharedPtr
      config_client_;
  rclcpp::Client<usv_interfaces::srv::ListDynamicShipModels>::SharedPtr models_client_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr names_sub_;
  rclcpp::Subscription<usv_interfaces::msg::DynamicShipSnapshot>::SharedPtr snapshot_sub_;

  double heading_deg_;
  double speed_;
  double half_distance_;
  std::string model_id_{"legacy_10m"};
  // All request / widget state is confined to the Qt thread. Executor callbacks
  // queue results here, including acknowledgements and request timeouts.
  QElapsedTimer elapsed_;
  uint64_t revision_{1}, applied_revision_{0}, config_token_{0}, catalog_token_{0};
  bool config_pending_{false}, catalog_pending_{false}, catalog_ready_{false};
  int64_t config_request_id_{0}, catalog_request_id_{0};
  qint64 config_sent_ms_{0}, catalog_sent_ms_{0}, next_config_ms_{0}, next_catalog_ms_{0};
  std::vector<usv_interfaces::msg::DynamicShipModelInfo> models_;
  QMap<QString, QString> instance_models_;

  QSlider * heading_slider_;
  QSlider * speed_slider_;
  QSlider * leg_dist_slider_;
  QComboBox * model_combo_;
  QLabel * model_summary_;
  QLabel * config_status_;
  QLabel * action_status_;
  QLabel * heading_label_;
  QLabel * speed_label_;
  QLabel * leg_dist_label_;
  QListWidget * ship_list_;
  QPushButton * clear_all_button_;
  QTimer * config_timer_;

  std::vector<std::string> ship_names_;
};

}  // namespace env_panel

#endif  // ENV_PANEL__DYNAMIC_SHIP_PANEL_HPP_
