/** * @file adaptive_radar_grid_map_node.cpp
 * @brief 自适应雷达回波建图节点 - 衰减/余辉版 (Decay/Persistence)
 * * 功能更新：
 * 1. [原有] Epoch 标签法解决量程切换问题
 * 2. [新增] 地图衰减机制：模拟雷达余辉，自动清除旧障碍物和噪点
 * 3. [优化] 适合船只移动的 Egocentric（以自我为中心）建图
 * 4. [新增] 地图跟随船只移动 (recenterMap)
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <marine_sensor_msgs/msg/radar_sector.hpp>
#include <rcl_interfaces/msg/parameter_event.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>

#include <deque>
#include <mutex>
#include <cmath>
#include <algorithm>
#include "usv_interfaces/topics.hpp"
#define NS_HEAD namespace seaward {
#define NS_FOOT }

template<typename T>
constexpr const T& clamp_value(const T& value, const T& low, const T& high)
{
  return (value < low) ? low : (value > high) ? high : value;
}

NS_HEAD

class AdaptiveRadarGridMapNode : public rclcpp::Node
{
public:
  // ==================== 参数结构体 ====================
  struct Parameters
  {
    struct Map {
      std::string frame_id = usv_interfaces::FRAME_MAP;
      double length = 200.0;
      double width = 200.0;
      double resolution = 1.0;
      double pub_interval = 0.1;
      double move_threshold_factor = 10.0;  // [新增] 地图移动阈值倍数
    } map;

    struct Filter {
      double near_clutter_range = 5.0;
      bool invert_angle = true;
    } filter;

    struct Intensity {
      double min_value = 0.0;
      double max_value = 15.0;
      double threshold = 1.0;
      std::string mapping_type = "linear";
      double exponential_base = 2.0;
      double sigmoid_steepness = 0.5;
      double sigmoid_midpoint = 7.5;
      bool low_as_free = true;
      double free_threshold = 2.0;
    } intensity;

    struct Adaptive {
      bool enable = true;
      double size_margin = 1.5;
      double resolution_factor = 2.0;
      double min_resolution = 0.5;
      double max_resolution = 200.0;
      double change_threshold = 0.1;
    } adaptive;

    // 衰减/余辉参数
    struct Decay {
      bool enable = true;
      double rate = 0.99;
      double frequency = 2.0;
      double min_threshold = 0.05;
    } decay;

    size_t max_queue_size = 1000;
    std::string control_node_name = "radar_control_node";
    std::string data_node_name = "radar_data_node";
  };

  struct DetectedRadarParams {
    double range_max = 0.0;
    double range_min = 0.0;
    double range_resolution = 0.0;
    int num_echoes = 0;
    double configured_range = 0.0;
    bool valid = false;
  };

  // 带标签的消息结构
  struct TaggedSector {
    marine_sensor_msgs::msg::RadarSector::SharedPtr msg;
    uint64_t epoch;
  };

  AdaptiveRadarGridMapNode()
    : Node("adaptive_radar_grid_map")
  {
    declareParameters();
    updateParameters();

    control_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
        this, parameters_.control_node_name);
    data_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
        this, parameters_.data_node_name);

    param_event_sub_ = this->create_subscription<rcl_interfaces::msg::ParameterEvent>(
        "/parameter_events", 10,
        std::bind(&AdaptiveRadarGridMapNode::parameterEventCallback, this, std::placeholders::_1));

    grid_map_publisher_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
        usv_interfaces::TOPIC_MAP_NAVRADAR_GRIDMAP, rclcpp::QoS(1).transient_local());

    costmap_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        usv_interfaces::TOPIC_MAP_NAVRADAR_OUUCPANCYGRID, 10);

    rclcpp::QoS qos_profile(50);
    qos_profile.best_effort();

    radar_sector_subscriber_ = this->create_subscription<marine_sensor_msgs::msg::RadarSector>(
        usv_interfaces::TOPIC_SENSOR_NAV_SECTOR, qos_profile,
        std::bind(&AdaptiveRadarGridMapNode::radarSectorCallback, this, std::placeholders::_1));

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    initializeMap();

    costmap_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(parameters_.map.pub_interval * 1000)),
        std::bind(&AdaptiveRadarGridMapNode::publishCostmap, this));

    queue_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&AdaptiveRadarGridMapNode::processQueue, this));

    sync_timer_ = this->create_wall_timer(
        std::chrono::seconds(2),
        std::bind(&AdaptiveRadarGridMapNode::syncParametersFromOtherNodes, this));

    restartDecayTimer();

    RCLCPP_INFO(this->get_logger(), "自适应雷达建图节点已启动 (Epoch标签 + 余辉衰减 + 地图跟随)");
    RCLCPP_INFO(this->get_logger(), "余辉模式: %s, 速率 %.3f @ %.1f Hz",
                parameters_.decay.enable ? "启用" : "禁用",
                parameters_.decay.rate, parameters_.decay.frequency);
  }

private:
  // ==================== Epoch 机制 ====================
  std::atomic<uint64_t> map_epoch_{0};

  // ==================== 强度映射 ====================
  double mapIntensityToProbability(double intensity)
  {
    const auto& p = parameters_.intensity;
    if (intensity < p.threshold) return -1.0;
    
    double normalized = (intensity - p.min_value) / (p.max_value - p.min_value);
    normalized = clamp_value(normalized, 0.0, 1.0);
    
    double probability = 0.0;
    if (p.mapping_type == "linear") probability = normalized;
    else if (p.mapping_type == "exponential") 
      probability = (std::pow(p.exponential_base, normalized) - 1.0) / (p.exponential_base - 1.0);
    else if (p.mapping_type == "sigmoid") {
      double x = intensity - p.sigmoid_midpoint;
      probability = 1.0 / (1.0 + std::exp(-p.sigmoid_steepness * x));
    }
    else probability = normalized;
    
    return clamp_value(probability, 0.0, 1.0);
  }

  void declareParameters()
  {
    this->declare_parameter("map.frame_id", parameters_.map.frame_id);
    this->declare_parameter("map.length", parameters_.map.length);
    this->declare_parameter("map.width", parameters_.map.width);
    this->declare_parameter("map.resolution", parameters_.map.resolution);
    this->declare_parameter("map.pub_interval", parameters_.map.pub_interval);
    this->declare_parameter("map.move_threshold_factor", parameters_.map.move_threshold_factor);  // [新增]
    this->declare_parameter("max_queue_size", static_cast<int>(parameters_.max_queue_size));
    
    this->declare_parameter("filter.near_clutter_range", parameters_.filter.near_clutter_range);
    this->declare_parameter("filter.invert_angle", parameters_.filter.invert_angle);

    this->declare_parameter("intensity.min_value", parameters_.intensity.min_value);
    this->declare_parameter("intensity.max_value", parameters_.intensity.max_value);
    this->declare_parameter("intensity.threshold", parameters_.intensity.threshold);
    this->declare_parameter("intensity.mapping_type", parameters_.intensity.mapping_type);
    this->declare_parameter("intensity.exponential_base", parameters_.intensity.exponential_base);
    this->declare_parameter("intensity.sigmoid_steepness", parameters_.intensity.sigmoid_steepness);
    this->declare_parameter("intensity.sigmoid_midpoint", parameters_.intensity.sigmoid_midpoint);

    this->declare_parameter("adaptive.enable", parameters_.adaptive.enable);
    this->declare_parameter("adaptive.size_margin", parameters_.adaptive.size_margin);
    this->declare_parameter("adaptive.resolution_factor", parameters_.adaptive.resolution_factor);
    this->declare_parameter("adaptive.min_resolution", parameters_.adaptive.min_resolution);
    this->declare_parameter("adaptive.max_resolution", parameters_.adaptive.max_resolution);
    this->declare_parameter("adaptive.change_threshold", parameters_.adaptive.change_threshold);

    this->declare_parameter("decay.enable", parameters_.decay.enable);
    this->declare_parameter("decay.rate", parameters_.decay.rate);
    this->declare_parameter("decay.frequency", parameters_.decay.frequency);
    this->declare_parameter("decay.min_threshold", parameters_.decay.min_threshold);

    this->declare_parameter("control_node_name", parameters_.control_node_name);
    this->declare_parameter("data_node_name", parameters_.data_node_name);

    this->declare_parameter("detected.range_max", 0.0);
    this->declare_parameter("detected.range_min", 0.0);
    this->declare_parameter("detected.range_resolution", 0.0);
    this->declare_parameter("detected.num_echoes", 0);
    this->declare_parameter("detected.configured_range", 0.0);
    this->declare_parameter("active.map_length", parameters_.map.length);
    this->declare_parameter("active.map_width", parameters_.map.width);
    this->declare_parameter("active.map_resolution", parameters_.map.resolution);
  }

  void updateParameters()
  {
    this->get_parameter("map.frame_id", parameters_.map.frame_id);
    this->get_parameter("map.length", parameters_.map.length);
    this->get_parameter("map.width", parameters_.map.width);
    this->get_parameter("map.resolution", parameters_.map.resolution);
    this->get_parameter("map.pub_interval", parameters_.map.pub_interval);
    this->get_parameter("map.move_threshold_factor", parameters_.map.move_threshold_factor);  // [新增]
    
    int queue_size;
    this->get_parameter("max_queue_size", queue_size);
    parameters_.max_queue_size = static_cast<size_t>(queue_size);
    
    this->get_parameter("filter.near_clutter_range", parameters_.filter.near_clutter_range);
    this->get_parameter("filter.invert_angle", parameters_.filter.invert_angle);

    this->get_parameter("intensity.min_value", parameters_.intensity.min_value);
    this->get_parameter("intensity.max_value", parameters_.intensity.max_value);
    this->get_parameter("intensity.threshold", parameters_.intensity.threshold);
    this->get_parameter("intensity.mapping_type", parameters_.intensity.mapping_type);
    
    this->get_parameter("adaptive.enable", parameters_.adaptive.enable);
    this->get_parameter("adaptive.size_margin", parameters_.adaptive.size_margin);
    this->get_parameter("adaptive.resolution_factor", parameters_.adaptive.resolution_factor);
    this->get_parameter("adaptive.min_resolution", parameters_.adaptive.min_resolution);
    this->get_parameter("adaptive.max_resolution", parameters_.adaptive.max_resolution);
    this->get_parameter("adaptive.change_threshold", parameters_.adaptive.change_threshold);

    this->get_parameter("control_node_name", parameters_.control_node_name);
    this->get_parameter("data_node_name", parameters_.data_node_name);

    bool old_decay_enable = parameters_.decay.enable;
    double old_decay_freq = parameters_.decay.frequency;

    this->get_parameter("decay.enable", parameters_.decay.enable);
    this->get_parameter("decay.rate", parameters_.decay.rate);
    this->get_parameter("decay.frequency", parameters_.decay.frequency);
    this->get_parameter("decay.min_threshold", parameters_.decay.min_threshold);

    if (parameters_.decay.enable && 
       (std::abs(old_decay_freq - parameters_.decay.frequency) > 0.1 || old_decay_enable != parameters_.decay.enable)) {
      restartDecayTimer();
    }

    active_length_ = parameters_.map.length;
    active_width_ = parameters_.map.width;
    active_resolution_ = parameters_.map.resolution;
  }

  void restartDecayTimer()
  {
    if (!parameters_.decay.enable || parameters_.decay.frequency <= 0.001) {
      if (decay_timer_) decay_timer_->cancel();
      return;
    }

    int period_ms = static_cast<int>(1000.0 / parameters_.decay.frequency);
    decay_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(period_ms),
        std::bind(&AdaptiveRadarGridMapNode::decayMapCallback, this));
  }

  void decayMapCallback()
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (!map_ptr_) return;

    if (map_ptr_->exists("probability")) {
      auto& prob_data = map_ptr_->get("probability");
      prob_data = prob_data * static_cast<float>(parameters_.decay.rate);

      if (parameters_.decay.min_threshold > 0.0) {
        prob_data = prob_data.unaryExpr([this](float v) {
          return (v < parameters_.decay.min_threshold) ? 0.0f : v;
        });
      }
    }

    if (map_ptr_->exists("intensity")) {
      map_ptr_->get("intensity") *= static_cast<float>(parameters_.decay.rate);
    }
  }

  void initializeMap()
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    uint64_t new_epoch = map_epoch_.fetch_add(1, std::memory_order_release) + 1;

    RCLCPP_WARN(this->get_logger(),
                "[MAP RESET] epoch=%lu, size=%.1fx%.1f, res=%.2f",
                new_epoch, active_length_, active_width_, active_resolution_);

    map_ptr_.reset(new grid_map::GridMap(
        {"probability", "intensity", "process_time"}));

    map_ptr_->setFrameId(parameters_.map.frame_id);
    map_ptr_->setGeometry(
        grid_map::Length(active_length_, active_width_),
        active_resolution_);
    
    map_ptr_->get("probability").setZero();
    map_ptr_->get("intensity").setZero();

    this->set_parameter(rclcpp::Parameter("active.map_length", active_length_));
    this->set_parameter(rclcpp::Parameter("active.map_width", active_width_));
    this->set_parameter(rclcpp::Parameter("active.map_resolution", active_resolution_));
  }

  // ==================== [新增] 地图跟随船只移动 ====================
  void recenterMap(const grid_map::Position& new_center)
  {
    if (!map_ptr_) return;

    grid_map::Position old_center = map_ptr_->getPosition();
    double distance = (new_center - old_center).norm();
    double move_threshold = parameters_.map.move_threshold_factor * active_resolution_;

    if (distance > move_threshold) {
      map_ptr_->move(new_center);
      
      // ★ 关键：初始化新滚入的区域（NAN → 0）
      for (const auto& layer : {"probability", "intensity"}) {
        if (map_ptr_->exists(layer)) {
          auto& data = map_ptr_->get(layer);
          data = data.unaryExpr([](float v) {
            return std::isnan(v) ? 0.0f : v;
          });
        }
      }

      RCLCPP_DEBUG(this->get_logger(), 
                  "地图滚动：移动 %.2f 米", distance);
    }
  }
  void parameterEventCallback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
  {
    bool is_control = (event->node == "/" + parameters_.control_node_name);
    bool is_data = (event->node == "/" + parameters_.data_node_name);
    
    if (event->node == this->get_fully_qualified_name()) {
        updateParameters();
        return;
    }

    if (!is_control && !is_data) return;

    for (const auto& param : event->changed_parameters) 
      handleExternalParameterChange(event->node, param, is_control);
    for (const auto& param : event->new_parameters) 
      handleExternalParameterChange(event->node, param, is_control);
  }

  void handleExternalParameterChange([[maybe_unused]] const std::string& node_name,
                                      const rcl_interfaces::msg::Parameter& param,
                                      bool is_control_node)
  {
    if (is_control_node) {
      if (param.name == "radar_range" || param.name == "range") {
        double range = extractDoubleValue(param);
        if (range > 0) {
          detected_params_.configured_range = range;
          this->set_parameter(rclcpp::Parameter("detected.configured_range", range));
          if (parameters_.adaptive.enable) checkAndAdaptMapParameters();
        }
      }
    } else {
      bool needs_update = false;
      if (param.name == "range_max") {
        detected_params_.range_max = extractDoubleValue(param);
        this->set_parameter(rclcpp::Parameter("detected.range_max", detected_params_.range_max));
        needs_update = true;
      }
      else if (param.name == "range_min") {
        detected_params_.range_min = extractDoubleValue(param);
        this->set_parameter(rclcpp::Parameter("detected.range_min", detected_params_.range_min));
      }
      else if (param.name == "num_echoes") {
        detected_params_.num_echoes = extractIntValue(param);
        this->set_parameter(rclcpp::Parameter("detected.num_echoes", detected_params_.num_echoes));
        updateRangeResolution();
        needs_update = true;
      }

      if (needs_update && parameters_.adaptive.enable) checkAndAdaptMapParameters();
    }
  }

  double extractDoubleValue(const rcl_interfaces::msg::Parameter& param)
  {
    if (param.value.type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
      return param.value.double_value;
    else if (param.value.type == rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
      return static_cast<double>(param.value.integer_value);
    return 0.0;
  }

  int extractIntValue(const rcl_interfaces::msg::Parameter& param)
  {
    if (param.value.type == rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
      return static_cast<int>(param.value.integer_value);
    else if (param.value.type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
      return static_cast<int>(param.value.double_value);
    return 0;
  }

  void updateRangeResolution()
  {
    if (detected_params_.range_max > 0 && detected_params_.num_echoes > 0) {
      detected_params_.range_resolution = detected_params_.range_max / detected_params_.num_echoes;
      this->set_parameter(rclcpp::Parameter("detected.range_resolution", detected_params_.range_resolution));
    }
  }

  void syncParametersFromOtherNodes()
  {
    if (control_param_client_->service_is_ready()) {
      auto future = control_param_client_->get_parameters({"radar_range", "range"});
      if (future.wait_for(std::chrono::milliseconds(100)) == std::future_status::ready) {
        try {
          for (const auto& param : future.get()) {
            if (param.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET) {
              double range = param.as_double();
              if (range > 0 && std::abs(range - detected_params_.configured_range) > 1.0) {
                detected_params_.configured_range = range;
                this->set_parameter(rclcpp::Parameter("detected.configured_range", range));
                if (parameters_.adaptive.enable) checkAndAdaptMapParameters();
              }
            }
          }
        } catch (...) {}
      }
    }
  }

  void detectParamsFromSector(const marine_sensor_msgs::msg::RadarSector::SharedPtr& msg)
  {
    double new_range_max = msg->range_max;
    detected_params_.range_min = msg->range_min;
    this->set_parameter(rclcpp::Parameter("detected.range_min", detected_params_.range_min));
    int new_num_echoes = 0;
    if (!msg->intensities.empty()) new_num_echoes = static_cast<int>(msg->intensities[0].echoes.size());

    bool changed = false;
    if (std::abs(new_range_max - detected_params_.range_max) > 1.0) {
      detected_params_.range_max = new_range_max;
      this->set_parameter(rclcpp::Parameter("detected.range_max", new_range_max));
      changed = true;
    }
    if (new_num_echoes != detected_params_.num_echoes) {
      detected_params_.num_echoes = new_num_echoes;
      this->set_parameter(rclcpp::Parameter("detected.num_echoes", new_num_echoes));
      changed = true;
    }

    if (changed) {
      updateRangeResolution();
      detected_params_.valid = true;
      if (parameters_.adaptive.enable) checkAndAdaptMapParameters();
    }
  }

  void checkAndAdaptMapParameters()
  {
    double effective_range = detected_params_.configured_range > 0 
                            ? detected_params_.configured_range 
                            : detected_params_.range_max;
    if (effective_range <= 0) return;
    double expected_length = effective_range * 2.0 * parameters_.adaptive.size_margin;
    double change_ratio = std::abs(expected_length - active_length_) / active_length_;

    if (change_ratio > parameters_.adaptive.change_threshold) {
      adaptMapParameters(effective_range);
    }
  }

  void adaptMapParameters(double effective_range)
  {
    active_length_ = effective_range * 2.0 * parameters_.adaptive.size_margin;
    active_width_  = active_length_;
    int echoes = (detected_params_.num_echoes > 0) ? detected_params_.num_echoes : 1024;
    double current_range_res = effective_range / static_cast<double>(echoes);
    active_resolution_ = current_range_res * parameters_.adaptive.resolution_factor;
    active_resolution_ = clamp_value(active_resolution_, parameters_.adaptive.min_resolution, parameters_.adaptive.max_resolution);
    initializeMap();
  }

  void radarSectorCallback(const marine_sensor_msgs::msg::RadarSector::SharedPtr msg)
  {
    detectParamsFromSector(msg);
    addToQueue(msg);
  }

  void addToQueue(const marine_sensor_msgs::msg::RadarSector::SharedPtr msg)
  {
    if (!tf_ready_) {
      if (tf_buffer_->canTransform(parameters_.map.frame_id, msg->header.frame_id,
              rclcpp::Time(msg->header.stamp), tf2::durationFromSec(0.01))) {
        tf_ready_ = true;
      } else return;
    }

    if (radar_sector_queue_.size() >= parameters_.max_queue_size) radar_sector_queue_.pop_front();
    
    uint64_t current_epoch = map_epoch_.load(std::memory_order_acquire);
    radar_sector_queue_.push_back({msg, current_epoch});
  }

  void processQueue()
  {
    uint64_t current_epoch = map_epoch_.load(std::memory_order_acquire);

    while (!radar_sector_queue_.empty()) {
      auto& item = radar_sector_queue_.front();
      if (item.epoch != current_epoch) {
        radar_sector_queue_.pop_front();
        continue;
      }
      if (!tf_buffer_->canTransform(parameters_.map.frame_id, item.msg->header.frame_id,
              rclcpp::Time(item.msg->header.stamp), tf2::durationFromSec(0.01))) {
        return;
      }
      processMsg(item.msg);
      radar_sector_queue_.pop_front();
    }
  }

  void processMsg(const marine_sensor_msgs::msg::RadarSector::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (!map_ptr_) return;

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(parameters_.map.frame_id,
          msg->header.frame_id, rclcpp::Time(msg->header.stamp));
    } catch (const tf2::TransformException& ex) { return; }

    double x0 = transform.transform.translation.x;
    double y0 = transform.transform.translation.y;

    // ==================== [新增] 地图跟随船只移动 ====================
    recenterMap(grid_map::Position(x0, y0));
    // ===============================================================

    tf2::Quaternion q(transform.transform.rotation.x, transform.transform.rotation.y,
        transform.transform.rotation.z, transform.transform.rotation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    double angle_sign = parameters_.filter.invert_angle ? -1.0 : 1.0;

    for (size_t i = 0; i < msg->intensities.size(); ++i) {
      double angle = angle_sign * (msg->angle_start + i * msg->angle_increment) + yaw;
      double c = std::cos(angle);
      double s = std::sin(angle);

      const auto& echoes = msg->intensities[i].echoes;
      if (echoes.empty()) continue;

      double range_step = (msg->range_max - msg->range_min) / static_cast<double>(echoes.size());

      for (size_t j = 0; j < echoes.size(); ++j) {
        float intensity = echoes[j];
        if (intensity <= 0.0f) continue;
        double range = msg->range_min + j * range_step;
        if (range <= parameters_.filter.near_clutter_range) continue;

        double mx = x0 + range * c;
        double my = y0 + range * s;
        grid_map::Position pos(mx, my);

        if (!map_ptr_->isInside(pos)) continue;

        map_ptr_->atPosition("intensity", pos) = intensity;

        double prob = mapIntensityToProbability(intensity);
        if (prob < 0.0) continue;

        float& cell = map_ptr_->atPosition("probability", pos);
        if (std::isnan(cell)) cell = prob;
        else cell = std::max(cell, static_cast<float>(prob));
        
        map_ptr_->atPosition("process_time", pos) = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
      }
    }
  }

  void publishCostmap()
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (!map_ptr_) return;

    nav_msgs::msg::OccupancyGrid occupancy_grid;
    grid_map::GridMapRosConverter::toOccupancyGrid(
        *map_ptr_, "probability", 0.0, 1.0, occupancy_grid);

    double valid_range = detected_params_.configured_range > 0 
                         ? detected_params_.configured_range 
                         : detected_params_.range_max;
    
    if (valid_range > 0) {
        double valid_range_sq = valid_range * valid_range;
        float resolution = occupancy_grid.info.resolution;
        double origin_x = occupancy_grid.info.origin.position.x;
        double origin_y = occupancy_grid.info.origin.position.y;
        int width = occupancy_grid.info.width;
        int height = occupancy_grid.info.height;

        // 获取地图中心位置用于距离计算
        grid_map::Position map_center = map_ptr_->getPosition();

        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int index = x + y * width;
                
                if (occupancy_grid.data[index] > 0) continue;

                double w_x = origin_x + (x + 0.5) * resolution;
                double w_y = origin_y + (y + 0.5) * resolution;
                
                // 计算相对于地图中心（船只位置）的距离
                double dx = w_x - map_center.x();
                double dy = w_y - map_center.y();
                double dist_sq = dx * dx + dy * dy;

                if (dist_sq > valid_range_sq) {
                    occupancy_grid.data[index] = -1; 
                }
            }
        }
    }

    costmap_publisher_->publish(occupancy_grid);

    auto message = grid_map::GridMapRosConverter::toMessage(*map_ptr_);
    grid_map_publisher_->publish(std::move(message));
  }

  // ==================== 成员变量 ====================
  Parameters parameters_;
  DetectedRadarParams detected_params_;

  double active_length_{200.0};
  double active_width_{200.0};
  double active_resolution_{1.0};

  std::unique_ptr<grid_map::GridMap> map_ptr_;
  std::mutex map_mutex_;

  std::shared_ptr<rclcpp::AsyncParametersClient> control_param_client_;
  std::shared_ptr<rclcpp::AsyncParametersClient> data_param_client_;

  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr param_event_sub_;
  rclcpp::Subscription<marine_sensor_msgs::msg::RadarSector>::SharedPtr radar_sector_subscriber_;

  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_publisher_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::TimerBase::SharedPtr costmap_timer_;
  rclcpp::TimerBase::SharedPtr queue_timer_;
  rclcpp::TimerBase::SharedPtr sync_timer_;
  rclcpp::TimerBase::SharedPtr decay_timer_;

  std::deque<TaggedSector> radar_sector_queue_;
  bool tf_ready_{false};
};

NS_FOOT

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<seaward::AdaptiveRadarGridMapNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}