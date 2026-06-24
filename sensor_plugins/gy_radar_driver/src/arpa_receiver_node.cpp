/**
 * @file arpa_receiver_node.cpp
 * @brief 雷达ARPA目标跟踪节点
 * 
 * 功能：
 * 1. 通过SDK接收ARPA目标数据
 * 2. 发布单个目标消息和帧目标消息
 * 3. 提供目标捕获/取消服务
 * 
 * 发布话题：
 *   - radar/arpa_target (ArpaTarget) : 单个目标实时更新
 *   - radar/arpa_frame (ArpaFrame) : 批量目标帧
 * 
 * 服务：
 *   - radar/cancel_all_targets : 取消所有目标跟踪
 */

#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <string>
#include <thread>
#include <chrono>
#include <mutex>
#include <vector>

// ROS消息
#include "usv_interfaces/msg/nav_radar_object.hpp"
#include "usv_interfaces/msg/nav_radar_frame.hpp"
#include "usv_interfaces/topics.hpp"

// ROS服务
#include "std_srvs/srv/trigger.hpp"

// SDK头文件 - 关键：必须包含 marpaobserver.h
#include "gy_sdk/multiradarclient.h"
#include "gy_sdk/targettrackingclient.h"
#include "gy_sdk/TargetProtocol.h"
#include "gy_sdk/marpaobserver.h"

using namespace NaviRadar;
using namespace NaviRadar::Target;
using namespace std::chrono_literals;

/**
 * @class ArpaReceiverNode
 * @brief ARPA目标跟踪节点
 */
class ArpaReceiverNode : public rclcpp::Node, 
                         public iTargetTrackingClientObserver
{
public:
    ArpaReceiverNode() : Node("arpa_receiver_node")
    {
        RCLCPP_INFO(get_logger(), ">>> Initializing ArpaReceiverNode...");

        // ================= 参数声明 =================
        declare_parameter("frame_id", usv_interfaces::FRAME_NAVRADAR);
        declare_parameter("publish_rate", 20.0);      // 帧发布频率
        declare_parameter("debug_mode", false);
        declare_parameter("danger_distance_m", 500);  // 危险距离(米)
        declare_parameter("danger_time_sec", 300);    // 危险时间(秒)

        // ================= 创建发布者 =================
        pub_target_ = create_publisher<usv_interfaces::msg::NavRadarObject>(
            usv_interfaces::TOPIC_PERCEPTION_NAV, 50);
        pub_frame_ = create_publisher<usv_interfaces::msg::NavRadarFrame>(
            usv_interfaces::TOPIC_PERCEPTION_NAV_FRAME, 10);

        // ================= 创建服务 =================
        srv_cancel_all_ = create_service<std_srvs::srv::Trigger>(
            "radar/cancel_all_targets",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
                if (!connected_ || !client_) {
                    res->success = false;
                    res->message = "Not connected";
                    return;
                }
                bool ok = client_->cancelAll();
                res->success = ok;
                res->message = ok ? "All targets cancelled" : "Cancel failed";
                RCLCPP_INFO(get_logger(), "Cancel all targets: %s", res->message.c_str());
            });

        // ================= 初始化SDK客户端 =================
        client_ = new TargetTrackingClient();
        client_->addClientObserver(this);

        // ================= 定时器 =================
        connect_timer_ = create_wall_timer(
            1000ms, std::bind(&ArpaReceiverNode::tryConnect, this));

        double rate = get_parameter("publish_rate").as_double();
        publish_timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / rate),
            std::bind(&ArpaReceiverNode::publishFrameLoop, this));

        RCLCPP_INFO(get_logger(), "ArpaReceiverNode initialized. Waiting for radar...");
    }

    ~ArpaReceiverNode() override
    {
        if (client_) {
            client_->disconnect();
            delete client_;
        }
    }

    // ================= SDK回调：目标更新 =================
    void UpdateTarget(const tTrackedTarget* pTarget) override
    {
        if (!pTarget || pTarget->targetValid == 0) return;

        auto msg = createTargetMessage(pTarget);

        // 立即发布单个目标
        pub_target_->publish(msg);

        // 调试输出
        if (get_parameter("debug_mode").as_bool()) {
            RCLCPP_INFO(get_logger(), "[Target] ID: %d | Dist: %.1fm | Spd: %.1fm/s | State: %s",
                msg.target_number, msg.distance, msg.speed, msg.target_status.c_str());
        }

        // 存入缓冲区用于帧发布
        {
            std::lock_guard<std::mutex> lock(buffer_mutex_);
            target_buffer_.push_back(msg);
        }
    }

    void UpdateTargetString(const std::string& /*str*/) override 
    {
        // 保留空实现
    }

    void UpdateBuffToSave(int /*type*/, const char* /*pBuff*/, int /*len*/) override 
    {
        // 保留空实现
    }

private:
    /**
     * @brief 创建目标消息
     */
    usv_interfaces::msg::NavRadarObject createTargetMessage(const tTrackedTarget* pTarget)
    {
        usv_interfaces::msg::NavRadarObject msg;

        // Header
        msg.header.stamp = this->now();
        msg.header.frame_id = get_parameter("frame_id").as_string();

        // 目标ID
        msg.target_number = pTarget->serverTargetID;

        // 状态映射
        switch (pTarget->targetState) {
            case eAcquiringTarget:
                msg.target_status = "Q";  // Acquiring
                break;
            case eSafeTarget:
            case eDangerousTarget:
            case eInGuardZone:
                msg.target_status = "T";  // Tracking
                break;
            case eLostTarget:
            case eLostingTarget:
            case eOutOfRange:
            case eLostOutOfRange:
                msg.target_status = "L";  // Lost
                break;
            default:
                msg.target_status = "L";
                break;
        }

        // 运动信息：优先使用绝对坐标
        bool use_absolute = (pTarget->infoAbsoluteValid == 1);

        if (use_absolute) {
            msg.distance = static_cast<float>(pTarget->infoAbsolute.distance_m);
            msg.bearing = static_cast<float>(pTarget->infoAbsolute.bearing_ddeg * 0.1) * static_cast<float>(M_PI) / 180.0f;
            msg.bearingtr = "T";  // True North
            msg.speed = static_cast<float>(pTarget->infoAbsolute.speed_dmps * 0.1);
            msg.course = static_cast<float>(pTarget->infoAbsolute.course_ddeg * 0.1);
            msg.coursetr = "T";
        } else {
            if (get_parameter("debug_mode").as_bool()) {
                RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000,
                    "Target %d: Absolute info invalid, using Relative.", msg.target_number);
            }
            msg.distance = static_cast<float>(pTarget->infoRelative.distance_m);
            msg.bearing = static_cast<float>(pTarget->infoRelative.bearing_ddeg * 0.1) * static_cast<float>(M_PI) / 180.0f;
            msg.bearingtr = "R";  // Relative
            msg.speed = static_cast<float>(pTarget->infoRelative.speed_dmps * 0.1);
            msg.course = static_cast<float>(pTarget->infoRelative.course_ddeg * 0.1);
            msg.coursetr = "R";
        }

        // 经纬度
        msg.latitude = std::abs(pTarget->latitude);
        msg.la_ns = (pTarget->latitude >= 0) ? "N" : "S";
        msg.longitude = std::abs(pTarget->longitude);
        msg.lng_ns = (pTarget->longitude >= 0) ? "E" : "W";

        // CPA/TCPA
        msg.cpa = static_cast<float>(pTarget->CPA_m);
        msg.tcpa = static_cast<float>(pTarget->TCPA_sec);

        return msg;
    }

    /**
     * @brief 帧发布循环
     */
    void publishFrameLoop()
    {
        usv_interfaces::msg::NavRadarFrame frame_msg;
        frame_msg.header.stamp = this->now();
        frame_msg.header.frame_id = get_parameter("frame_id").as_string();

        size_t count = 0;
        {
            std::lock_guard<std::mutex> lock(buffer_mutex_);
            count = target_buffer_.size();
            if (!target_buffer_.empty()) {
                frame_msg.targets = std::move(target_buffer_);
                target_buffer_.clear();
            }
        }

        pub_frame_->publish(frame_msg);

        if (count > 0) {
            RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
                "[Frame] Published %zu targets", count);
        }
    }

    /**
     * @brief 尝试连接雷达
     */
    void tryConnect()
    {
        if (connected_) return;

        auto pmc = MultiRadarClient::getInstance();
        pmc->query();

        char radars[10][MultiRadarClient::sMaxSeialNumberSize] = {};
        int n = pmc->getRadar(radars, 10);

        if (n <= 0) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "No radar found. Retrying...");
            return;
        }

        std::string serial = radars[0];
        RCLCPP_INFO(get_logger(), "Found radar: %s. Connecting ARPA client...", serial.c_str());

        if (!client_->connect(serial.c_str(), 0)) {
            RCLCPP_ERROR(get_logger(), "Failed to connect ARPA client to: %s", serial.c_str());
            return;
        }

        // 设置危险参数
        int danger_dist = get_parameter("danger_distance_m").as_int();
        int danger_time = get_parameter("danger_time_sec").as_int();
        client_->setDangerDistance(static_cast<uint32_t>(danger_dist));
        client_->setDangerTime(static_cast<uint32_t>(danger_time));

        connected_ = true;
        connect_timer_->cancel();

        RCLCPP_INFO(get_logger(), ">>> ARPA Client connected! <<<");
    }

    // SDK客户端
    TargetTrackingClient* client_{nullptr};

    // 发布者
    rclcpp::Publisher<usv_interfaces::msg::NavRadarObject>::SharedPtr pub_target_;
    rclcpp::Publisher<usv_interfaces::msg::NavRadarFrame>::SharedPtr pub_frame_;

    // 服务
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_cancel_all_;

    // 定时器
    rclcpp::TimerBase::SharedPtr connect_timer_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    // 状态
    bool connected_{false};

    // 目标缓冲区
    std::vector<usv_interfaces::msg::NavRadarObject> target_buffer_;
    std::mutex buffer_mutex_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<ArpaReceiverNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("main"), "Exception: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}