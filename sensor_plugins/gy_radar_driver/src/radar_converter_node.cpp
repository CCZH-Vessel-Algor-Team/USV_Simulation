#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "marine_sensor_msgs/msg/radar_sector.hpp"
#include <cmath>
#include "usv_interfaces/topics.hpp"
//radarsector转loudpoint2
class RadarConverter : public rclcpp::Node {
public:
    RadarConverter() : Node("radar_converter") {
        // 订阅者 QoS 设置为 Best Effort 以匹配发布者
        sub_ = this->create_subscription<marine_sensor_msgs::msg::RadarSector>(
            usv_interfaces::TOPIC_SENSOR_NAV_SECTOR, 
            rclcpp::QoS(10).best_effort(), 
            std::bind(&RadarConverter::onSector, this, std::placeholders::_1));
        
        // 发布点云
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            usv_interfaces::TOPIC_SENSOR_NAV_POINTS,
            rclcpp::QoS(10).reliable().durability_volatile());
        
        RCLCPP_INFO(this->get_logger(), "Converter Node Started: RadarSector -> PointCloud2");
    }

private:
    void onSector(const marine_sensor_msgs::msg::RadarSector::SharedPtr msg) {
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header = msg->header;
        cloud.height = 1;
        cloud.width = 0;
        cloud.is_dense = false;
        
        sensor_msgs::PointCloud2Modifier modifier(cloud);

        // 核心修复：显式定义所有字段，避免 "Field intensity does not exist" 错误
        // 使用 setPointCloud2Fields 而不是 ByString，更稳定
        modifier.setPointCloud2Fields(
            4, 
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32,
            "intensity", 1, sensor_msgs::msg::PointField::FLOAT32
        );

        if (msg->intensities.empty()) return;
        const auto& echo = msg->intensities[0];
        size_t n_points = echo.echoes.size();
        
        // 设置点云大小
        modifier.resize(n_points);
        
        // 创建迭代器填充数据
        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_intensity(cloud, "intensity");

        float resolution = msg->range_max / (float)n_points;
        float angle = msg->angle_start; 

        int valid_count = 0;
        for (size_t i = 0; i < n_points; ++i) {
            float intensity = echo.echoes[i];
            
            // 简单过滤：只保留有强度的点
            if (intensity > 2.0f) { 
                float r = i * resolution;
                
                // 极坐标 -> 笛卡尔坐标
                // 注意：根据安装方向可能需要调整 sin/cos 符号
                *iter_x = r * std::cos(angle);
                *iter_y = r * std::sin(angle);
                *iter_z = 0.0;
                *iter_intensity = intensity;

                ++iter_x; ++iter_y; ++iter_z; ++iter_intensity;
                valid_count++;
            }
        }
        
        // 调整最终大小（去除过滤掉的点）
        modifier.resize(valid_count);
        
        if (valid_count > 0) {
            pub_->publish(cloud);
        }
    }

    rclcpp::Subscription<marine_sensor_msgs::msg::RadarSector>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RadarConverter>());
    rclcpp::shutdown();
    return 0;
}