/**
 * @file arpa_distance_debug.cpp
 * @brief ARPA距离诊断工具 - 对比原始数据与ROS消息
 */

#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include "usv_interfaces/msg/nav_radar_object.hpp"

// SDK头文件
#include "gy_sdk/targettrackingclient.h"
#include "gy_sdk/TargetProtocol.h"
#include "gy_sdk/marpaobserver.h"
#include "gy_sdk/multiradarclient.h"

using namespace NaviRadar;
using namespace NaviRadar::Target;

class ArpaDebugNode : public rclcpp::Node, 
                      public iTargetTrackingClientObserver
{
public:
    ArpaDebugNode() : Node("arpa_debug_node")
    {
        RCLCPP_INFO(get_logger(), "=== ARPA 距离诊断工具启动 ===");

        // 订阅ROS话题（对比用）
        sub_arpa_ = create_subscription<usv_interfaces::msg::NavRadarObject>(
            "radar/arpa_target", 10,
            [this](const usv_interfaces::msg::NavRadarObject::SharedPtr msg) {
                ros_distance_ = msg->distance;
                ros_bearing_ = msg->bearing;
            });

        // 连接SDK
        client_ = new TargetTrackingClient();
        client_->addClientObserver(this);

        connect_timer_ = create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&ArpaDebugNode::tryConnect, this));
    }

    ~ArpaDebugNode() override
    {
        if (client_) {
            client_->disconnect();
            delete client_;
        }
    }

    // SDK回调：打印原始数据
    void UpdateTarget(const tTrackedTarget* pTarget) override
    {
        if (!pTarget || pTarget->targetValid == 0) return;

        RCLCPP_INFO(get_logger(), 
            "\n========== Target #%d ==========\n"
            "  [SDK原始数据]\n"
            "    Absolute.distance_m   = %u (原始uint32)\n"
            "    Absolute.bearing_ddeg = %u (0.1度单位)\n"
            "    Relative.distance_m   = %u\n"
            "    Absolute Valid        = %d\n"
            "\n"
            "  [ROS消息数据]\n"
            "    distance (float)      = %.3f\n"
            "    bearing (rad)         = %.6f (%.2f度)\n"
            "\n"
            "  [诊断]\n"
            "    距离差异 = %.3f m\n"
            "    可能缩放系数 = %.4f\n"
            "================================\n",
            pTarget->serverTargetID,
            // SDK原始
            pTarget->infoAbsolute.distance_m,
            pTarget->infoAbsolute.bearing_ddeg,
            pTarget->infoRelative.distance_m,
            pTarget->infoAbsoluteValid,
            // ROS消息
            ros_distance_,
            ros_bearing_, ros_bearing_ * 180.0 / M_PI,
            // 诊断
            pTarget->infoAbsolute.distance_m - ros_distance_,
            pTarget->infoAbsolute.distance_m / (ros_distance_ + 0.001)  // 防止除0
        );

        // 额外检查：打印原始字节
        const uint8_t* raw = reinterpret_cast<const uint8_t*>(&pTarget->infoAbsolute.distance_m);
        RCLCPP_INFO(get_logger(), 
            "  [原始字节] distance_m = 0x%02X%02X%02X%02X",
            raw[3], raw[2], raw[1], raw[0]);
    }

    void UpdateTargetString(const std::string&) override {}
    void UpdateBuffToSave(int, const char*, int) override {}

private:
    void tryConnect()
    {
        if (connected_) return;

        auto pmc = MultiRadarClient::getInstance();
        pmc->query();

        char radars[10][MultiRadarClient::sMaxSeialNumberSize] = {};
        int n = pmc->getRadar(radars, 10);

        if (n <= 0) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "未找到雷达，继续等待...");
            return;
        }

        std::string serial = radars[0];
        RCLCPP_INFO(get_logger(), "连接雷达: %s", serial.c_str());

        if (!client_->connect(serial.c_str(), 0)) {
            RCLCPP_ERROR(get_logger(), "连接失败");
            return;
        }

        connected_ = true;
        connect_timer_->cancel();
        RCLCPP_INFO(get_logger(), "✓ SDK连接成功，开始监听目标数据...");
    }

    TargetTrackingClient* client_{nullptr};
    rclcpp::Subscription<usv_interfaces::msg::NavRadarObject>::SharedPtr sub_arpa_;
    rclcpp::TimerBase::SharedPtr connect_timer_;
    
    bool connected_{false};
    float ros_distance_{0.0f};
    float ros_bearing_{0.0f};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArpaDebugNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}