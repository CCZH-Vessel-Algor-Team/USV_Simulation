/**
 * @file radar_control_node.cpp
 * @brief 统一的雷达控制节点
 * 
 * 功能：
 * 1. 雷达连接管理（自动发现、连接、重连）
 * 2. 参数控制（量程、增益、海浪抑制等）
 * 3. 惯导配置
 * 4. 状态监控与发布
 * 5. 提供服务接口
 * 
 * 整合了原有的 radar_controller 和 radar_ins_config_node 功能
 */

#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <chrono>
#include <atomic>
#include <mutex>
#include <arpa/inet.h>

// ROS消息和服务
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

// SDK
#include "gy_sdk/ImageClient.h"
#include "gy_sdk/ImageClientObserver.h"
#include "gy_sdk/multiradarclient.h"
#include "gy_sdk/NavRadarProtocol.h"

using namespace NaviRadar;
using namespace std::chrono_literals;

/**
 * @class RadarControlNode
 * @brief 统一的雷达控制节点
 */
class RadarControlNode : public rclcpp::Node,
                         public iImageClientStateObserver
{
public:
    RadarControlNode() : Node("radar_control_node")
    {
        RCLCPP_INFO(get_logger(), ">>> Initializing RadarControlNode...");

        // ================= 参数声明 =================
        declareAllParameters();

        // ================= 初始化SDK客户端 =================
        client_ = new tImageClient();
        if (!client_) {
            throw std::runtime_error("Failed to create tImageClient");
        }
        client_->AddStateObserver(this);

        // ================= 参数回调 =================
        param_callback_handle_ = add_on_set_parameters_callback(
            std::bind(&RadarControlNode::onParamChange, this, std::placeholders::_1));

        // ================= 创建服务 =================
        createServices();

        // ================= 启动定时器 =================
        // 连接定时器
        connect_timer_ = create_wall_timer(
            1000ms, std::bind(&RadarControlNode::tryConnect, this));
        
        // 控制循环定时器
        control_timer_ = create_wall_timer(
            500ms, std::bind(&RadarControlNode::controlLoop, this));

        RCLCPP_INFO(get_logger(), "RadarControlNode initialized. Waiting for radar...");
    }

    ~RadarControlNode() override
    {
        if (client_) {
            client_->SetTransmit(false);
            client_->SetPower(false);
            std::this_thread::sleep_for(200ms);
            client_->disconnect();
            delete client_;
            client_ = nullptr;
        }
    }

    // ================= SDK State回调 =================
    void UpdateMode(const tMode* pMode) override
    {
        if (pMode) {
            radar_state_.store(pMode->state, std::memory_order_relaxed);
        }
    }

    void UpdateBoatInsInfo(const tBoatInsInfo* info) override
    {
        if (!info) return;

        std::lock_guard<std::mutex> lock(ins_mutex_);
        last_ins_info_ = *info;

        if (info->currValid) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                "[INS Valid] Lat: %.6f, Lng: %.6f | Heading: %.2f | Speed: %.2f m/s | Freq: %d Hz",
                info->currLngLat.lat, info->currLngLat.lng,
                info->heading, info->speed, info->packFreq);
        } else {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "[INS Invalid] Freq: %d Hz | InvalidGroup: %d | TimeOut: %d",
                info->packFreq, info->invalidGroupCnt, info->timeOutCnt);
        }
    }

    void UpdateSetup(const tSetup* pSetup) override
    {
        if (pSetup) {
            RCLCPP_DEBUG(get_logger(), "Setup received: range=%d dm", pSetup->range_dm);
        }
    }

    void UpdateRadarError(const tRadarError* pError) override
    {
        if (pError) {
            RCLCPP_ERROR(get_logger(), "Radar Error: type=%d param1=%d",
                pError->type, pError->param1);
        }
    }

    // 其他回调的空实现
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
     * @brief 声明所有参数
     */
    void declareAllParameters()
    {
        // 雷达控制参数
        declare_parameter("radar_range", 1000);           // 量程(米)
        declare_parameter("radar_rpm", 0);                // 转速档位 0-4
        declare_parameter("radar_gain", -1);              // 增益 -1=自动, 0-255=手动
        declare_parameter("sea_clutter", -1);             // 海浪抑制 -1=自动, 0-255=手动
        declare_parameter("rain_clutter", 0);             // 雨雪抑制 0-255
        declare_parameter("side_lobe", -1);               // 旁瓣抑制 -1=自动, 0-255=手动
        declare_parameter("beam_sharpening", 2);          // 波束锐化 0-3
        declare_parameter("interference_reject", 0);      // 干扰抑制 0-3
        declare_parameter("noise_reject", 0);             // 噪声抑制 0-3
        declare_parameter("target_stretch", true);        // 目标扩展
        declare_parameter("target_boost", 2);             // 目标增强 0-2

        // 惯导参数
        declare_parameter("enable_ins", true);
        declare_parameter("ins_ip", "224.100.100.30");
        declare_parameter("ins_port", 20842);

        // 系统参数
        declare_parameter("auto_transmit", true);         // 连接后自动发射
        declare_parameter("watchdog_interval_ms", 5000);  // 看门狗间隔
    }

    /**
     * @brief 创建服务
     */
    void createServices()
    {
        // 发射控制服务
        srv_transmit_ = create_service<std_srvs::srv::SetBool>(
            "radar/set_transmit",
            [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                   std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
                if (!connected_) {
                    res->success = false;
                    res->message = "Radar not connected";
                    return;
                }
                bool ok = client_->SetTransmit(req->data);
                res->success = ok;
                res->message = ok ? (req->data ? "Transmit ON" : "Transmit OFF") : "Command failed";
            });

        // 电源控制服务
        srv_power_ = create_service<std_srvs::srv::SetBool>(
            "radar/set_power",
            [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                   std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
                if (!connected_) {
                    res->success = false;
                    res->message = "Radar not connected";
                    return;
                }
                bool ok = client_->SetPower(req->data);
                res->success = ok;
                res->message = ok ? (req->data ? "Power ON" : "Power OFF") : "Command failed";
            });

        // 查询所有状态服务
        srv_query_all_ = create_service<std_srvs::srv::Trigger>(
            "radar/query_all",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
                if (!connected_) {
                    res->success = false;
                    res->message = "Radar not connected";
                    return;
                }
                bool ok = client_->QueryAll();
                res->success = ok;
                res->message = ok ? "Query sent" : "Query failed";
            });
    }

    /**
     * @brief 参数变化回调
     */
    rcl_interfaces::msg::SetParametersResult onParamChange(
        const std::vector<rclcpp::Parameter>& params)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        if (!connected_) {
            RCLCPP_WARN(get_logger(), "Radar not connected, parameters will apply after connection");
            return result;
        }

        for (const auto& p : params) {
            const std::string& name = p.get_name();

            if (name == "radar_range") {
                int range = p.as_int();
                if (range > 0) {
                    client_->SetRange(static_cast<uint32_t>(range));
                    RCLCPP_INFO(get_logger(), "Set range: %d m", range);
                }
            }
            else if (name == "radar_rpm") {
                int rpm = p.as_int();
                if (rpm >= 0 && rpm <= 4) {
                    client_->SetFastScanMode(static_cast<uint8_t>(rpm));
                    const char* rpm_str[] = {"24", "36", "48", "60", "96"};
                    RCLCPP_INFO(get_logger(), "Set RPM: %s rpm", rpm_str[rpm]);
                }
            }
            else if (name == "radar_gain") {
                int gain = p.as_int();
                if (gain < 0) {
                    client_->SetGain(eUserGainAuto, 0);
                    RCLCPP_INFO(get_logger(), "Set gain: AUTO");
                } else {
                    client_->SetGain(eUserGainManual, static_cast<uint8_t>(std::min(gain, 255)));
                    RCLCPP_INFO(get_logger(), "Set gain: MANUAL %d", gain);
                }
            }
            else if (name == "sea_clutter") {
                int sea = p.as_int();
                if (sea < 0) {
                    client_->SetSeaClutter(eUserGainAuto, 0);
                    RCLCPP_INFO(get_logger(), "Set sea clutter: AUTO");
                } else {
                    client_->SetSeaClutter(eUserGainManual, static_cast<uint8_t>(std::min(sea, 255)));
                    RCLCPP_INFO(get_logger(), "Set sea clutter: MANUAL %d", sea);
                }
            }
            else if (name == "rain_clutter") {
                int rain = p.as_int();
                client_->SetRain(static_cast<uint8_t>(std::min(std::max(rain, 0), 255)));
                RCLCPP_INFO(get_logger(), "Set rain clutter: %d", rain);
            }
            else if (name == "side_lobe") {
                int side = p.as_int();
                if (side < 0) {
                    client_->SetSideLobe(eUserGainAuto, 0);
                    RCLCPP_INFO(get_logger(), "Set side lobe: AUTO");
                } else {
                    client_->SetSideLobe(eUserGainManual, static_cast<uint8_t>(std::min(side, 255)));
                    RCLCPP_INFO(get_logger(), "Set side lobe: MANUAL %d", side);
                }
            }
            else if (name == "beam_sharpening") {
                int beam = p.as_int();
                if (beam >= 0 && beam <= 3) {
                    client_->SetBeamSharpening(static_cast<uint8_t>(beam));
                    RCLCPP_INFO(get_logger(), "Set beam sharpening: %d", beam);
                }
            }
            else if (name == "interference_reject") {
                int ir = p.as_int();
                if (ir >= 0 && ir <= 3) {
                    client_->SetInterferenceReject(static_cast<uint8_t>(ir));
                    RCLCPP_INFO(get_logger(), "Set interference reject: %d", ir);
                }
            }
            else if (name == "noise_reject") {
                int nr = p.as_int();
                if (nr >= 0 && nr <= 3) {
                    client_->SetNoiseReject(static_cast<uint8_t>(nr));
                    RCLCPP_INFO(get_logger(), "Set noise reject: %d", nr);
                }
            }
            else if (name == "target_stretch") {
                bool stretch = p.as_bool();
                client_->SetTargetStretch(stretch);
                RCLCPP_INFO(get_logger(), "Set target stretch: %s", stretch ? "ON" : "OFF");
            }
            else if (name == "target_boost") {
                int boost = p.as_int();
                if (boost >= 0 && boost <= 2) {
                    client_->SetTargetBoost(static_cast<uint8_t>(boost));
                    RCLCPP_INFO(get_logger(), "Set target boost: %d", boost);
                }
            }
        }

        return result;
    }

    /**
     * @brief 应用所有参数到雷达
     */
    void applyAllParameters()
    {
        RCLCPP_INFO(get_logger(), "Applying all parameters to radar...");

        // 量程
        int range = get_parameter("radar_range").as_int();
        if (range > 0) client_->SetRange(static_cast<uint32_t>(range));

        // 转速
        int rpm = get_parameter("radar_rpm").as_int();
        if (rpm >= 0 && rpm <= 4) client_->SetFastScanMode(static_cast<uint8_t>(rpm));

        // 增益
        int gain = get_parameter("radar_gain").as_int();
        if (gain < 0) {
            client_->SetGain(eUserGainAuto, 0);
        } else {
            client_->SetGain(eUserGainManual, static_cast<uint8_t>(std::min(gain, 255)));
        }

        // 海浪抑制
        int sea = get_parameter("sea_clutter").as_int();
        if (sea < 0) {
            client_->SetSeaClutter(eUserGainAuto, 0);
        } else {
            client_->SetSeaClutter(eUserGainManual, static_cast<uint8_t>(std::min(sea, 255)));
        }

        // 雨雪抑制
        int rain = get_parameter("rain_clutter").as_int();
        client_->SetRain(static_cast<uint8_t>(std::min(std::max(rain, 0), 255)));

        // 旁瓣抑制
        int side = get_parameter("side_lobe").as_int();
        if (side < 0) {
            client_->SetSideLobe(eUserGainAuto, 0);
        } else {
            client_->SetSideLobe(eUserGainManual, static_cast<uint8_t>(std::min(side, 255)));
        }

        // 波束锐化
        int beam = get_parameter("beam_sharpening").as_int();
        if (beam >= 0 && beam <= 3) client_->SetBeamSharpening(static_cast<uint8_t>(beam));

        // 干扰抑制
        int ir = get_parameter("interference_reject").as_int();
        if (ir >= 0 && ir <= 3) client_->SetInterferenceReject(static_cast<uint8_t>(ir));

        // 噪声抑制
        int nr = get_parameter("noise_reject").as_int();
        if (nr >= 0 && nr <= 3) client_->SetNoiseReject(static_cast<uint8_t>(nr));

        // 目标扩展
        bool stretch = get_parameter("target_stretch").as_bool();
        client_->SetTargetStretch(stretch);

        // 目标增强
        int boost = get_parameter("target_boost").as_int();
        if (boost >= 0 && boost <= 2) client_->SetTargetBoost(static_cast<uint8_t>(boost));

        RCLCPP_INFO(get_logger(), "All parameters applied");
    }

    /**
     * @brief 配置惯导
     */
    void configureIns()
    {
        bool ins_enabled = get_parameter("enable_ins").as_bool();
        
        if (ins_enabled) {
            std::string ins_ip = get_parameter("ins_ip").as_string();
            int ins_port = get_parameter("ins_port").as_int();
            
            uint32_t ip_addr = inet_addr(ins_ip.c_str());

            if (client_->SetInsAddress(ip_addr, static_cast<uint16_t>(ins_port))) {
                RCLCPP_INFO(get_logger(), "INS Address configured: %s:%d", ins_ip.c_str(), ins_port);
            } else {
                RCLCPP_ERROR(get_logger(), "Failed to set INS Address");
            }

            if (client_->SetInsEnable(true)) {
                RCLCPP_INFO(get_logger(), "INS Enabled");
            } else {
                RCLCPP_ERROR(get_logger(), "Failed to enable INS");
            }
        } else {
            client_->SetInsEnable(false);
            RCLCPP_INFO(get_logger(), "INS Disabled by parameter");
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
        RCLCPP_INFO(get_logger(), "Found radar: %s. Connecting...", serial.c_str());

        if (!client_->connect(serial.c_str(), 0)) {
            RCLCPP_ERROR(get_logger(), "Failed to connect to radar: %s", serial.c_str());
            return;
        }

        std::this_thread::sleep_for(200ms);

        // 配置惯导
        configureIns();

        // 查询状态
        client_->QueryAll();
        std::this_thread::sleep_for(200ms);

        // 自动开机发射
        if (get_parameter("auto_transmit").as_bool()) {
            client_->SetPower(true);
            std::this_thread::sleep_for(500ms);
            client_->SetTransmit(true);
        }

        connected_ = true;
        connect_timer_->cancel();
        
        RCLCPP_INFO(get_logger(), ">>> Radar connected and initialized! <<<");

        // 应用所有参数
        applyAllParameters();
    }

    /**
     * @brief 控制循环
     */
    void controlLoop()
    {
        if (!connected_) return;

        // 发送看门狗
        client_->SendClientWatchdog();

        // 定期查询状态
        static int counter = 0;
        if (++counter % 20 == 0) {
            client_->QueryAll();
        }

        // 检查雷达状态，确保在发射
        int state = radar_state_.load();
        if (get_parameter("auto_transmit").as_bool() && state != eTransmit) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "Radar not transmitting (state=%d), attempting to restart...", state);
            client_->SetPower(true);
            client_->SetTransmit(true);
        }
    }

    // SDK客户端
    tImageClient* client_{nullptr};

    // 服务
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_transmit_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_power_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_query_all_;

    // 定时器
    rclcpp::TimerBase::SharedPtr connect_timer_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // 参数回调句柄
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // 状态
    bool connected_{false};
    std::atomic<int> radar_state_{-1};

    // 惯导数据
    std::mutex ins_mutex_;
    tBoatInsInfo last_ins_info_{};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<RadarControlNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("main"), "Exception: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}