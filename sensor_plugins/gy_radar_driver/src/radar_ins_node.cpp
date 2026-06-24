#include <rclcpp/rclcpp.hpp>
#include <arpa/inet.h>
#include <memory>
#include <string>
#include <thread> // 需要这个来 sleep
#include <chrono>

#include "gy_sdk/ImageClient.h"
#include "gy_sdk/ImageClientObserver.h"
#include "gy_sdk/NavRadarProtocol.h"
#include "gy_sdk/multiradarclient.h" // 引入多雷达客户端用于扫描

using namespace NaviRadar;
using namespace std::chrono_literals;

class RadarStateObserver : public iImageClientStateObserver
{
public:
    RadarStateObserver(rclcpp::Logger logger) : logger_(logger) {}

    void UpdateBoatInsInfo(const tBoatInsInfo* info) override
    {
        RCLCPP_INFO(logger_, 
            "--- [INS Status] --- Valid: %s | Freq: %d Hz | Heading: %.2f | Speed: %.2f m/s", 
            info->currValid ? "YES" : "NO", 
            info->packFreq,
            info->heading,
            info->speed
        );
        if (!info->currValid && info->packFreq > 0) {
             RCLCPP_WARN(logger_, "Receiving data but INVALID content.");
        }
    }

    // 其他纯虚函数空实现...
    void UpdateMode(const tMode*) override {}
    void UpdateSetup(const tSetup*) override {}
    void UpdateAdvancedSetup(const tAdvancedSetup*) override {}
    void UpdateSetupExtended(const tSetupExtended*) override {}
    void UpdateProperties(const tProperties*) override {}
    void UpdateConfiguration(const tConfiguration*) override {}
    void UpdateGuardZoneAlarm(const tGuardZoneAlarm*) override {}
    void UpdateRadarError(const tRadarError*) override {}
    void UpdateProcessCfg(const tProcessConfiguration*) override {}
    void UpdateFactoryCfg(const tFactoryConfiguration*) override {}
    void UpdateStcCfg(const tStcConfiguration*) override {}
    void UpdateDebuggingPara(const tDebuggingPara*) override {}

private:
    rclcpp::Logger logger_;
};

class RadarInsNode : public rclcpp::Node
{
public:
    RadarInsNode() : Node("radar_ins_config_node")
    {
        // 只需要配置惯导的IP和端口，不需要手动填序列号了
        this->declare_parameter<std::string>("ins_ip", "224.100.100.30");
        this->declare_parameter<int>("ins_port", 20842);

        image_client_ = std::make_shared<tImageClient>();
        state_observer_ = std::make_shared<RadarStateObserver>(this->get_logger());
        image_client_->AddStateObserver(state_observer_.get());

        // 使用定时器去尝试连接，而不是在构造函数里死等
        timer_ = this->create_wall_timer(
            1000ms, std::bind(&RadarInsNode::try_connect_radar, this));
            
        RCLCPP_INFO(this->get_logger(), "Radar INS Node Started. Scanning for radar...");
    }

    ~RadarInsNode()
    {
        if(image_client_) image_client_->disconnect();
    }

private:
    void try_connect_radar()
    {
        // 如果已经连接了，就只发心跳（虽然只配置惯导可能不需要心跳，但发了保险）
        // 这里简单处理：连上了就取消定时器，不再扫描
        
        // 1. 扫描雷达
        auto pmc = MultiRadarClient::getInstance();
        pmc->query();
        
        // 给一点点时间让SDK去发现设备
        // std::this_thread::sleep_for(100ms); // 在定时器回调里sleep不太好，但这里是为了演示逻辑
        
        char radars[10][MultiRadarClient::sMaxSeialNumberSize] = {};
        int n = pmc->getRadar(radars, 10);
        
        if (n <= 0) {
            RCLCPP_WARN(this->get_logger(), "No radar found. Retrying...");
            return; 
        }

        std::string serial = radars[0];
        RCLCPP_INFO(this->get_logger(), "Found radar: %s. Connecting...", serial.c_str());

        // 2. 连接扫描到的第一台雷达
        if (image_client_->connect(serial.c_str(), 0))
        {
            RCLCPP_INFO(this->get_logger(), "Radar connected successfully!");
            
            // 3. 配置惯导
            configure_ins();
            
            // 停止定时器，不再尝试连接
            timer_->cancel();
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to %s", serial.c_str());
        }
    }

    void configure_ins()
    {
        std::string ins_ip_str = this->get_parameter("ins_ip").as_string();
        int ins_port = this->get_parameter("ins_port").as_int();
        uint32_t ip_addr = inet_addr(ins_ip_str.c_str());

        if (image_client_->SetInsAddress(ip_addr, static_cast<uint16_t>(ins_port))) {
            RCLCPP_INFO(this->get_logger(), "Set INS Address to %s:%d success", ins_ip_str.c_str(), ins_port);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to set INS Address!");
        }

        if (image_client_->SetInsEnable(true)) {
            RCLCPP_INFO(this->get_logger(), "Enabled INS input success");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to enable INS input!");
        }
    }

    std::shared_ptr<tImageClient> image_client_;
    std::shared_ptr<RadarStateObserver> state_observer_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RadarInsNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}