#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <marine_sensor_msgs/msg/radar_sector.hpp>
#include <cmath>
#include <vector>
#include <algorithm>

using namespace std::chrono_literals;

class RadarMappingNode : public rclcpp::Node {
public:
    RadarMappingNode() : Node("radar_mapping_node") {
        // 参数声明
        this->declare_parameter("map_size_meter", 600.0);    // 地图边长 (米)
        this->declare_parameter("map_resolution", 0.5);      // 分辨率 (米/像素)
        this->declare_parameter("decay_rate", 2.0);          // 衰减速度 (数值/秒)
        this->declare_parameter("hit_increment", 15);        // 击中一次增加的数值
        this->declare_parameter("frame_id", "radar_link");

        // 获取参数
        map_size_m_ = this->get_parameter("map_size_meter").as_double();
        resolution_ = this->get_parameter("map_resolution").as_double();
        decay_rate_ = this->get_parameter("decay_rate").as_double();
        hit_inc_ = this->get_parameter("hit_increment").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();

        // 计算网格尺寸
        grid_width_ = static_cast<int>(map_size_m_ / resolution_);
        grid_height_ = grid_width_;
        grid_center_x_ = grid_width_ / 2;
        grid_center_y_ = grid_height_ / 2;

        // 初始化地图数据缓冲区 (使用 float 存储中间值以实现平滑衰减)
        map_buffer_.resize(grid_width_ * grid_height_, 0.0f);

        // 订阅雷达扇区数据
        sub_ = this->create_subscription<marine_sensor_msgs::msg::RadarSector>(
            "radar/sector", 
            rclcpp::QoS(10).best_effort(), 
            std::bind(&RadarMappingNode::onSector, this, std::placeholders::_1));

        // 发布栅格地图
        pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("radar/grid_map", 10);

        // 定时器：处理衰减和发布 (10Hz)
        decay_timer_ = this->create_wall_timer(
            100ms, std::bind(&RadarMappingNode::decayLoop, this));

        RCLCPP_INFO(this->get_logger(), "Mapping Node Started. Grid: %dx%d, Res: %.2fm", 
            grid_width_, grid_height_, resolution_);
    }

private:
    // 地图参数
    double map_size_m_;
    double resolution_;
    double decay_rate_;
    int hit_inc_;
    std::string frame_id_;

    int grid_width_, grid_height_;
    int grid_center_x_, grid_center_y_;
    
    // 地图缓冲区：0.0 (空) ~ 100.0 (完全占用)
    std::vector<float> map_buffer_;

    rclcpp::Subscription<marine_sensor_msgs::msg::RadarSector>::SharedPtr sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr decay_timer_;

    void onSector(const marine_sensor_msgs::msg::RadarSector::SharedPtr msg) {
        if (msg->intensities.empty()) return;

        const auto& echo = msg->intensities[0];
        size_t n_points = echo.echoes.size();
        float range_res = msg->range_max / static_cast<float>(n_points);
        float angle = msg->angle_start; // 弧度

        // 预计算三角函数
        float cos_a = std::cos(angle);
        float sin_a = std::sin(angle);

        // 遍历这条扫描线上的所有点
        for (size_t i = 0; i < n_points; ++i) {
            float intensity = echo.echoes[i];

            // 只有强度超过一定阈值才更新地图 (简单的噪声过滤)
            if (intensity > 5.0f) {
                float range = i * range_res;
                
                // 1. 计算物理坐标 (雷达系)
                // 注意坐标系：ROS通常是 X前 Y左
                float rx = range * cos_a;
                float ry = range * sin_a;

                // 2. 转换为栅格坐标
                // 地图中心是 (0,0)
                int gx = static_cast<int>(rx / resolution_) + grid_center_x_;
                int gy = static_cast<int>(ry / resolution_) + grid_center_y_;

                // 3. 边界检查
                if (gx >= 0 && gx < grid_width_ && gy >= 0 && gy < grid_height_) {
                    int index = gy * grid_width_ + gx;
                    
                    // 4. 累积强度 (Accumulation)
                    // 并不是直接设为100，而是累加，体现概率
                    // 强度越大，增加的权重越大
                    float increment = hit_inc_ * (intensity / 15.0f); 
                    map_buffer_[index] += increment;
                    
                    // 封顶 100
                    if (map_buffer_[index] > 100.0f) map_buffer_[index] = 100.0f;
                }
            }
        }
    }

    void decayLoop() {
        // 1. 构建 ROS 消息
        nav_msgs::msg::OccupancyGrid grid_msg;
        grid_msg.header.stamp = this->now();
        grid_msg.header.frame_id = frame_id_;
        
        grid_msg.info.resolution = resolution_;
        grid_msg.info.width = grid_width_;
        grid_msg.info.height = grid_height_;
        
        // 设置地图原点：使得 (0,0) 位于地图中心
        grid_msg.info.origin.position.x = -(map_size_m_ / 2.0);
        grid_msg.info.origin.position.y = -(map_size_m_ / 2.0);
        grid_msg.info.origin.position.z = 0.0;
        grid_msg.info.origin.orientation.w = 1.0;

        grid_msg.data.resize(grid_width_ * grid_height_);

        // 2. 执行衰减并填充消息
        // 每次循环衰减的值 (Decay amount per cycle)
        // Timer是 100ms (10Hz)，所以每次衰减 decay_rate / 10
        float decay_step = decay_rate_ / 10.0f; 

        for (size_t i = 0; i < map_buffer_.size(); ++i) {
            // 衰减逻辑
            if (map_buffer_[i] > 0.0f) {
                map_buffer_[i] -= decay_step;
                if (map_buffer_[i] < 0.0f) map_buffer_[i] = 0.0f;
            }

            // 填充到消息 (int8: 0-100, -1 未知)
            // 这里我们认为没有回波的地方是 0 (Free)，有回波的是 1-100 (Occupied)
            // 如果你想让地图背景透明，可以设定阈值，低于阈值的设为 -1
            if (map_buffer_[i] < 1.0f) {
                grid_msg.data[i] = 0; // Free space
            } else {
                grid_msg.data[i] = static_cast<int8_t>(map_buffer_[i]);
            }
        }

        // 3. 发布
        pub_->publish(grid_msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RadarMappingNode>());
    rclcpp::shutdown();
    return 0;
}