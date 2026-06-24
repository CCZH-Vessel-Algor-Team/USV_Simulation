/**
 * @file radar_spoke_sdk_node.cpp
 * @brief 基于SDK的雷达Spoke数据发布节点
 * 
 * 功能：
 * 1. 通过SDK回调获取处理后的Spoke数据(4bit)和原始Spoke数据(32bit)
 * 2. 转换为ROS消息发布
 * 3. 支持marine_sensor_msgs标准格式
 * 
 * 替代原有的UDP直接读取方式，统一使用SDK接口
 */

#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <string>
#include <thread>
#include <chrono>
#include <mutex>
#include <atomic>
#include <vector>

// ROS消息
#include "marine_sensor_msgs/msg/radar_sector.hpp"
#include "marine_sensor_msgs/msg/radar_echo.hpp"
#include "std_msgs/msg/header.hpp"

// SDK头文件
#include "gy_sdk/ImageClient.h"
#include "gy_sdk/ImageClientObserver.h"
#include "gy_sdk/multiradarclient.h"
#include "gy_sdk/NavRadarProtocol.h"
#include "gy_sdk/NavRadarSpoke.h"

using namespace NaviRadar;
using namespace std::chrono_literals;

/**
 * @class RadarSpokeSdkNode
 * @brief 基于SDK的雷达Spoke数据发布节点
 * 
 * 实现 iImageClientSpokeObserver 接口接收Spoke数据
 * 实现 iImageClientStateObserver 接口接收状态数据
 */
class RadarSpokeSdkNode : public rclcpp::Node,
                          public iImageClientSpokeObserver,
                          public iImageClientStateObserver
{
public:
    RadarSpokeSdkNode() : Node("radar_spoke_sdk_node")
    {
        RCLCPP_INFO(get_logger(), ">>> Initializing RadarSpokeSdkNode (SDK-based)...");

        // ================= 参数声明 =================
        declare_parameter("frame_id", "radar_link");
        declare_parameter("publish_original", false);  // 是否发布原始32bit数据
        declare_parameter("enable_spoke_output", true); // 是否启用Spoke输出

        // ================= 创建发布者 =================
        // 主要发布者：处理后的4bit Spoke数据 (兼容marine_sensor_msgs)
        pub_sector_ = create_publisher<marine_sensor_msgs::msg::RadarSector>(
            "radar/sector", rclcpp::QoS(100).best_effort());

        // 可选发布者：原始32bit数据
        if (get_parameter("publish_original").as_bool()) {
            pub_sector_raw_ = create_publisher<marine_sensor_msgs::msg::RadarSector>(
                "radar/sector_raw", rclcpp::QoS(100).best_effort());
        }

        // ================= 初始化SDK客户端 =================
        client_ = new tImageClient();
        if (!client_) {
            throw std::runtime_error("Failed to create tImageClient");
        }

        // 注册观察者
        client_->AddSpokeObserver(this);
        client_->AddStateObserver(this);

        // ================= 启动连接定时器 =================
        connect_timer_ = create_wall_timer(
            1000ms, std::bind(&RadarSpokeSdkNode::tryConnect, this));

        // ================= 看门狗定时器 =================
        watchdog_timer_ = create_wall_timer(
            5000ms, std::bind(&RadarSpokeSdkNode::sendWatchdog, this));

        RCLCPP_INFO(get_logger(), "RadarSpokeSdkNode initialized. Waiting for radar...");
    }

    ~RadarSpokeSdkNode() override
    {
        if (client_) {
            client_->disconnect();
            delete client_;
            client_ = nullptr;
        }
    }

    // ================= SDK Spoke回调：处理后的4bit数据 =================
    void UpdateSpoke(const Spoke::SPOKE* pSpoke) override
    {
        if (!pSpoke) return;

        auto msg = std::make_unique<marine_sensor_msgs::msg::RadarSector>();
        
        // Header
        msg->header.stamp = this->now();
        msg->header.frame_id = get_parameter("frame_id").as_string();

        // 角度信息：SDK的azimuth范围是0-4095，映射到0-2π
        constexpr float AZIMUTH_SCALE = 2.0f * M_PI / 4096.0f;
        msg->angle_start = static_cast<float>(pSpoke->header.spokeAzimuth) * AZIMUTH_SCALE;
        msg->angle_increment = 0.0f;  // 单条spoke

        // 量程信息
        uint32_t range_mm = Spoke::GetSpokeRange_mm(pSpoke->header);
        msg->range_min = 0.0f;
        msg->range_max = static_cast<float>(range_mm) / 1000.0f;  // 转换为米

        // 回波数据：4bit打包格式，解包为1024个float
        marine_sensor_msgs::msg::RadarEcho echo;
        echo.echoes.reserve(pSpoke->header.nOfSamples);
        
        // 每个字节包含2个4bit采样值
        for (uint32_t i = 0; i < pSpoke->header.nOfSamples / 2 && i < sizeof(pSpoke->data); ++i) {
            uint8_t byte = pSpoke->data[i];
            // 低4位是第一个采样，高4位是第二个采样
            echo.echoes.push_back(static_cast<float>(byte & 0x0F));
            echo.echoes.push_back(static_cast<float>((byte >> 4) & 0x0F));
        }

        msg->intensities.push_back(echo);

        // 发布
        pub_sector_->publish(std::move(msg));

        // 统计
        spoke_count_++;
    }

    // ================= SDK Spoke回调：原始32bit数据 =================
    void UpdateOriginalSpoke(const Spoke::tOriginalSpoke* pSpoke) override
    {
        if (!pSpoke || !pub_sector_raw_) return;

        auto msg = std::make_unique<marine_sensor_msgs::msg::RadarSector>();
        
        // Header
        msg->header.stamp = this->now();
        msg->header.frame_id = get_parameter("frame_id").as_string();

        // 角度信息
        constexpr float AZIMUTH_SCALE = 2.0f * M_PI / 4096.0f;
        msg->angle_start = static_cast<float>(pSpoke->header.spokeAzimuth) * AZIMUTH_SCALE;
        msg->angle_increment = 0.0f;

        // 量程信息（原始数据量程是显示量程的1.25倍）
        uint32_t range_mm = Spoke::GetSpokeRange_mm(pSpoke->header);
        msg->range_min = 0.0f;
        msg->range_max = static_cast<float>(range_mm) / 1000.0f;

        // 回波数据：每4字节一个32bit采样值，共512个
        marine_sensor_msgs::msg::RadarEcho echo;
        echo.echoes.reserve(512);
        
        const uint8_t* data = pSpoke->data;
        for (size_t i = 0; i + 3 < sizeof(pSpoke->data); i += 4) {
            uint32_t value = static_cast<uint32_t>(data[i]) |
                            (static_cast<uint32_t>(data[i+1]) << 8) |
                            (static_cast<uint32_t>(data[i+2]) << 16) |
                            (static_cast<uint32_t>(data[i+3]) << 24);
            echo.echoes.push_back(static_cast<float>(value));
        }

        msg->intensities.push_back(echo);
        pub_sector_raw_->publish(std::move(msg));
    }

    // ================= SDK Buffer回调 (用于数据保存，暂不使用) =================
    void UpdateBuffToSave(int /*type*/, const char* /*pBuff*/, int /*len*/) override {}

    // ================= SDK State回调 =================
    void UpdateMode(const tMode* pMode) override
    {
        if (pMode) {
            radar_state_.store(pMode->state, std::memory_order_relaxed);
            RCLCPP_DEBUG(get_logger(), "Radar state: %d", pMode->state);
        }
    }

    void UpdateSetup(const tSetup* pSetup) override
    {
        if (pSetup) {
            current_range_dm_.store(pSetup->range_dm, std::memory_order_relaxed);
            RCLCPP_DEBUG(get_logger(), "Radar range: %d dm", pSetup->range_dm);
        }
    }

    void UpdateRadarError(const tRadarError* pError) override
    {
        if (pError) {
            RCLCPP_ERROR(get_logger(), "Radar Error: type=%d param1=%d param2=%d",
                pError->type, pError->param1, pError->param2);
        }
    }

    void UpdateBoatInsInfo(const tBoatInsInfo* info) override
    {
        if (info && info->currValid) {
            RCLCPP_DEBUG(get_logger(), "[INS] Lat: %.6f, Lng: %.6f, Heading: %.2f",
                info->currLngLat.lat, info->currLngLat.lng, info->heading);
        }
    }

    // 其他State回调的空实现
    void UpdateAdvancedSetup(const tAdvancedSetup*) override {}
    void UpdateSetupExtended(const tSetupExtended*) override {}
    void UpdateProperties(const tProperties*) override {}
    void UpdateConfiguration(const tConfiguration*) override {}
    void UpdateGuardZoneAlarm(const tGuardZoneAlarm*) override {}
    void UpdateProcessCfg(const tProcessConfiguration*) override {}
    void UpdateFactoryCfg(const tFactoryConfiguration*) override {}
    void UpdateStcCfg(const tStcConfiguration*) override {}
    void UpdateDebuggingPara(const tDebuggingPara*) override {}

private:
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
                "No radar found via SDK query. Retrying...");
            return;
        }

        std::string serial = radars[0];
        RCLCPP_INFO(get_logger(), "Found radar serial: %s. Connecting...", serial.c_str());

        if (!client_->connect(serial.c_str(), 0)) {
            RCLCPP_ERROR(get_logger(), "Failed to connect to radar: %s", serial.c_str());
            return;
        }

        std::this_thread::sleep_for(200ms);

        // 确保Spoke输出已启用
        if (get_parameter("enable_spoke_output").as_bool()) {
            client_->SetSpokeOutputEnable(true);
        }

        // 查询所有状态
        client_->QueryAll();
        std::this_thread::sleep_for(200ms);

        // 开机并发射
        client_->SetPower(true);
        std::this_thread::sleep_for(500ms);
        client_->SetTransmit(true);

        connected_ = true;
        connect_timer_->cancel();
        
        RCLCPP_INFO(get_logger(), ">>> Radar connected and transmitting! <<<");
    }

    /**
     * @brief 发送看门狗心跳
     */
    void sendWatchdog()
    {
        if (!connected_) return;
        
        client_->SendClientWatchdog();
        
        // 定期打印统计信息
        static auto last_log_time = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_log_time).count() >= 10) {
            RCLCPP_INFO(get_logger(), "[Stats] Spoke count: %lu, Range: %d dm",
                spoke_count_.load(), current_range_dm_.load());
            last_log_time = now;
        }
    }

    // SDK客户端
    tImageClient* client_{nullptr};

    // 发布者
    rclcpp::Publisher<marine_sensor_msgs::msg::RadarSector>::SharedPtr pub_sector_;
    rclcpp::Publisher<marine_sensor_msgs::msg::RadarSector>::SharedPtr pub_sector_raw_;

    // 定时器
    rclcpp::TimerBase::SharedPtr connect_timer_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;

    // 状态
    bool connected_{false};
    std::atomic<int> radar_state_{-1};
    std::atomic<uint32_t> current_range_dm_{0};
    std::atomic<uint64_t> spoke_count_{0};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<RadarSpokeSdkNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("main"), "Exception: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}