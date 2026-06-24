/**
 * @file radar_data_node.cpp
 * @brief 融合版雷达数据节点 - 修复版
 * 
 * 修复内容：
 * 1. 修正nOfSamples=0时的采样点数计算逻辑
 * 2. 增加range_cells参数发布，供建图节点使用
 * 3. 增加调试输出，帮助诊断量程切换问题
 * 4. [新增] 修复采样点数计算：使用 sizeof(data)*2 而非 rangeCells
 */

#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <string>
#include <thread>
#include <chrono>
#include <atomic>
#include <vector>
#include <mutex>

#include "marine_sensor_msgs/msg/radar_sector.hpp"
#include "marine_sensor_msgs/msg/radar_echo.hpp"

// SDK 头文件
#include "gy_sdk/ImageClient.h"
#include "gy_sdk/ImageClientObserver.h"
#include "gy_sdk/multiradarclient.h"
#include "gy_sdk/NavRadarProtocol.h"
#include "gy_sdk/NavRadarSpoke.h"

#include "usv_interfaces/topics.hpp"
using namespace NaviRadar;
using namespace std::chrono_literals;

class RadarDataNode : public rclcpp::Node,
                      public iImageClientSpokeObserver,
                      public iImageClientStateObserver
{
public:
    RadarDataNode() : Node("radar_data_node")
    {
        RCLCPP_INFO(get_logger(), ">>> Initializing RadarDataNode (Fixed Version)...");

        // 1. 参数声明
        declare_parameter("frame_id", usv_interfaces::FRAME_NAVRADAR);
        declare_parameter("publish_original", true);
        declare_parameter("enable_spoke_output", true);
        declare_parameter("debug_spoke_info", true);  // 新增：调试开关

        // 发布检测到的参数（供其他节点读取）
        declare_parameter("range_max", 0.0);
        declare_parameter("range_min", 0.0);
        declare_parameter("num_echoes", 0);
        declare_parameter("range_cells", 0);  // 新增

        // 2. 创建发布者
        pub_sector_ = create_publisher<marine_sensor_msgs::msg::RadarSector>(
            usv_interfaces::TOPIC_SENSOR_NAV_SECTOR, rclcpp::QoS(100).best_effort());

        if (get_parameter("publish_original").as_bool()) {
            pub_sector_raw_ = create_publisher<marine_sensor_msgs::msg::RadarSector>(
                "radar/sector_raw", rclcpp::QoS(100).best_effort());
            RCLCPP_INFO(get_logger(), "Original spoke publishing enabled");
        }

        // 3. 初始化 SDK 客户端
        client_ = new tImageClient();
        if (!client_) {
            throw std::runtime_error("Failed to create tImageClient");
        }

        client_->AddSpokeObserver(this);
        client_->AddStateObserver(this);

        // 4. 定时器
        connect_timer_ = create_wall_timer(
            1000ms, std::bind(&RadarDataNode::tryConnect, this));

        watchdog_timer_ = create_wall_timer(
            5000ms, std::bind(&RadarDataNode::sendWatchdog, this));

        RCLCPP_INFO(get_logger(), "RadarDataNode initialized. Waiting for radar...");
    }

    ~RadarDataNode() override
    {
        if (client_) {
            client_->disconnect();
            delete client_;
            client_ = nullptr;
        }
    }

    /**
     * @brief 处理压缩后的4bit Spoke数据 - 修复版
     */
    void UpdateSpoke(const Spoke::SPOKE* pSpoke) override
    {
        if (!pSpoke) return;

        auto msg = std::make_unique<marine_sensor_msgs::msg::RadarSector>();
        
        msg->header.stamp = this->now();
        msg->header.frame_id = get_parameter("frame_id").as_string();

        // 1. 角度计算
        uint32_t azimuth = pSpoke->header.spokeAzimuth;
        if (azimuth > 4095) azimuth = 4095;
        constexpr float AZIMUTH_SCALE = 2.0f * static_cast<float>(M_PI) / 4096.0f;
        msg->angle_start = static_cast<float>(azimuth) * AZIMUTH_SCALE;
        msg->angle_increment = 0.0f;

        // 2. 量程计算
        uint32_t range_mm = Spoke::GetSpokeRange_mm(pSpoke->header);
        msg->range_min = 0.0f;
        msg->range_max = static_cast<float>(range_mm) / 1000.0f;

        // ============ 关键修复：采样点数计算 ============
        // 修复：实际采样点数 = sizeof(data) * 2（因为每字节包含2个4bit采样）
        // rangeCells 是距离单元数，不是采样点数！
        
        uint32_t original_nOfSamples = pSpoke->header.nOfSamples;
        uint32_t original_rangeCells = pSpoke->header.rangeCells;
        
        // 实际采样点数：数据缓冲区大小 * 2
        uint32_t num_samples = sizeof(pSpoke->data) * 2;  // 512 * 2 = 1024

        // 调试输出
        if (get_parameter("debug_spoke_info").as_bool()) {
            static float last_range = 0;
            
            if (std::abs(msg->range_max - last_range) > 10.0f) {
                RCLCPP_INFO(get_logger(), 
                    "[Spoke] range=%.0fm, 采样点=%u (nOfSamples=%u, rangeCells=%u, sizeof(data)=%zu)",
                    msg->range_max, num_samples, original_nOfSamples,
                    original_rangeCells, sizeof(pSpoke->data));
                last_range = msg->range_max;
            }
        }

        // 更新参数（供其他节点读取）
        updateDetectedParams(msg->range_max, msg->range_min, 
                            static_cast<int>(num_samples), 
                            static_cast<int>(original_rangeCells));

        // 3. 解包全部 4bit 数据
        marine_sensor_msgs::msg::RadarEcho echo;
        echo.echoes.reserve(num_samples);
        
        // 解包所有数据
        for (size_t i = 0; i < sizeof(pSpoke->data); ++i) {
            uint8_t byte = pSpoke->data[i];
            // 低4位是第一个采样，高4位是第二个采样
            echo.echoes.push_back(static_cast<float>(byte & 0x0F));
            echo.echoes.push_back(static_cast<float>((byte >> 4) & 0x0F));
        }

        msg->intensities.push_back(std::move(echo));
        pub_sector_->publish(std::move(msg));
        spoke_count_++;
    }

    /**
     * @brief 处理原始32bit Spoke数据
     */
    void UpdateOriginalSpoke(const Spoke::tOriginalSpoke* pSpoke) override
    {
        if (!pSpoke || !pub_sector_raw_) return;

        auto msg = std::make_unique<marine_sensor_msgs::msg::RadarSector>();
        
        msg->header.stamp = this->now();
        msg->header.frame_id = get_parameter("frame_id").as_string();

        uint32_t azimuth = pSpoke->header.spokeAzimuth;
        if (azimuth > 4095) azimuth = 4095;
        constexpr float AZIMUTH_SCALE = 2.0f * static_cast<float>(M_PI) / 4096.0f;
        msg->angle_start = static_cast<float>(azimuth) * AZIMUTH_SCALE;
        msg->angle_increment = 0.0f;

        uint32_t range_mm = Spoke::GetSpokeRange_mm(pSpoke->header);
        msg->range_min = 0.0f;
        msg->range_max = static_cast<float>(range_mm) / 1000.0f;

        marine_sensor_msgs::msg::RadarEcho echo;
        constexpr size_t EXPECTED_RAW_SAMPLES = 512;
        echo.echoes.reserve(EXPECTED_RAW_SAMPLES);
        
        const uint8_t* data = pSpoke->data;
        for (size_t i = 0; i + 3 < sizeof(pSpoke->data); i += 4) {
            uint32_t value = static_cast<uint32_t>(data[i]) |
                            (static_cast<uint32_t>(data[i+1]) << 8) |
                            (static_cast<uint32_t>(data[i+2]) << 16) |
                            (static_cast<uint32_t>(data[i+3]) << 24);
            echo.echoes.push_back(static_cast<float>(value));
        }

        msg->intensities.push_back(std::move(echo));
        pub_sector_raw_->publish(std::move(msg));
    }

    void UpdateBuffToSave(int, const char*, int) override {}

    void UpdateMode(const tMode* pMode) override
    {
        if (pMode) {
            radar_state_.store(pMode->state, std::memory_order_relaxed);
        }
    }

    void UpdateSetup(const tSetup* pSetup) override
    {
        if (pSetup) {
            current_range_dm_.store(pSetup->range_dm, std::memory_order_relaxed);
            RCLCPP_DEBUG(get_logger(), "Range updated: %d dm", pSetup->range_dm);
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
     * @brief 更新检测到的参数
     */
    void updateDetectedParams(float range_max, float range_min, 
                              int num_echoes, int range_cells)
    {
        static float last_range_max = 0;
        static int last_num_echoes = 0;

        // 只在参数变化时更新
        if (std::abs(range_max - last_range_max) > 1.0f || 
            num_echoes != last_num_echoes) {
            
            this->set_parameter(rclcpp::Parameter("range_max", 
                static_cast<double>(range_max)));
            this->set_parameter(rclcpp::Parameter("range_min", 
                static_cast<double>(range_min)));
            this->set_parameter(rclcpp::Parameter("num_echoes", num_echoes));
            this->set_parameter(rclcpp::Parameter("range_cells", range_cells));

            RCLCPP_INFO(get_logger(), 
                "[参数更新] range: %.0f-%.0fm, echoes: %d, cells: %d",
                range_min, range_max, num_echoes, range_cells);

            last_range_max = range_max;
            last_num_echoes = num_echoes;
        }
    }

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
        RCLCPP_INFO(get_logger(), "Found radar: %s. Connecting...", serial.c_str());

        if (!client_->connect(serial.c_str(), 0)) {
            RCLCPP_ERROR(get_logger(), "Failed to connect: %s", serial.c_str());
            return;
        }

        std::this_thread::sleep_for(200ms);

        if (get_parameter("enable_spoke_output").as_bool()) {
            client_->SetSpokeOutputEnable(true);
        }

        client_->QueryAll();
        std::this_thread::sleep_for(200ms);

        client_->SetPower(true);
        std::this_thread::sleep_for(500ms);
        client_->SetTransmit(true);

        connected_ = true;
        connect_timer_->cancel();
        
        RCLCPP_INFO(get_logger(), ">>> Radar connected and transmitting! <<<");
    }

    void sendWatchdog()
    {
        if (!connected_) return;
        
        client_->SendClientWatchdog();
        
        static auto last_log = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_log).count() >= 10) {
            RCLCPP_INFO(get_logger(), "[Stats] Spoke count: %lu, Range: %d dm",
                spoke_count_.load(), current_range_dm_.load());
            last_log = now;
        }
    }

    tImageClient* client_{nullptr};
    rclcpp::Publisher<marine_sensor_msgs::msg::RadarSector>::SharedPtr pub_sector_;
    rclcpp::Publisher<marine_sensor_msgs::msg::RadarSector>::SharedPtr pub_sector_raw_;
    
    rclcpp::TimerBase::SharedPtr connect_timer_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;

    bool connected_{false};
    std::atomic<int> radar_state_{-1};
    std::atomic<uint32_t> current_range_dm_{0};
    std::atomic<uint64_t> spoke_count_{0};

    // 修改：根据实际雷达规格调整
    static constexpr size_t MAX_SAMPLES = 2048;
    static constexpr size_t MIN_SAMPLES = 256;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<RadarDataNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("main"), "Exception: %s", e.what());
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}