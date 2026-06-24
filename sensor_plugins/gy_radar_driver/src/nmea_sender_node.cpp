#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nmea_msgs/msg/sentence.hpp> // 引入 NMEA 消息
#include <sys/socket.h>
#include <arpa/inet.h>
#include <cmath>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>

class NmeaSenderNode : public rclcpp::Node {
public:
    NmeaSenderNode() : Node("nmea_sender_node") {
        // 声明参数
        this->declare_parameter("use_nmea_relay", true);        // 是否启用直接转发 NMEA 模式
        this->declare_parameter("nmea_topic", "/nmea_sentence"); // NMEA 话题名称
        this->declare_parameter("use_odom_gen", false);         // 是否启用 Odom 生成模式
        this->declare_parameter("odom_topic", "/odom");         // 里程计话题名称
        this->declare_parameter("target_ip", "224.100.100.30"); // 雷达监听地址
        this->declare_parameter("target_port", 20842);          // 雷达监听端口

        bool use_nmea_relay = this->get_parameter("use_nmea_relay").as_bool();
        bool use_odom_gen = this->get_parameter("use_odom_gen").as_bool();
        target_ip_ = this->get_parameter("target_ip").as_string();
        target_port_ = this->get_parameter("target_port").as_int();

        // 初始化 UDP Socket
        initSocket();

        // 模式 1: 直接转发 AIS/GNSS 的 NMEA 语句 (推荐)
        if (use_nmea_relay) {
            std::string topic = this->get_parameter("nmea_topic").as_string();
            sub_nmea_ = this->create_subscription<nmea_msgs::msg::Sentence>(
                topic, 10, std::bind(&NmeaSenderNode::onNmea, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "Mode: RELAY. Forwarding %s to %s:%d", 
                topic.c_str(), target_ip_.c_str(), target_port_);
        }

        // 模式 2: 从 Odom 生成 NMEA (备用)
        if (use_odom_gen) {
            std::string topic = this->get_parameter("odom_topic").as_string();
            sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
                topic, 10, std::bind(&NmeaSenderNode::onOdom, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "Mode: GENERATE. Converting %s to NMEA sent to %s:%d", 
                topic.c_str(), target_ip_.c_str(), target_port_);
        }
    }

    ~NmeaSenderNode() {
        if (sock_ >= 0) close(sock_);
    }

private:
    int sock_ = -1;
    struct sockaddr_in target_addr_;
    std::string target_ip_;
    int target_port_;
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<nmea_msgs::msg::Sentence>::SharedPtr sub_nmea_;

    void initSocket() {
        sock_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Create socket failed");
            return;
        }

        // 允许广播和组播
        int broadcast = 1;
        setsockopt(sock_, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));

        memset(&target_addr_, 0, sizeof(target_addr_));
        target_addr_.sin_family = AF_INET;
        target_addr_.sin_port = htons(target_port_);
        target_addr_.sin_addr.s_addr = inet_addr(target_ip_.c_str());
    }

    // --- 转发逻辑 ---
    void onNmea(const nmea_msgs::msg::Sentence::SharedPtr msg) {
        std::string sentence = msg->sentence;
        
        // 过滤：只发送雷达需要的导航数据
        // 通常雷达需要: $--HDT (艏向), $--VTG (地面速度/航向), $--RMC (综合定位)
        // AIS 数据 ($AIVDM) 发给雷达通常没用，甚至可能导致缓冲区溢出，建议过滤
        bool useful = false;
        if (sentence.find("HDT") != std::string::npos) useful = true; // 艏向 (最重要)
        if (sentence.find("VTG") != std::string::npos) useful = true; // 航速 (ARPA需要)
        if (sentence.find("RMC") != std::string::npos) useful = true; // 定位+时间
        if (sentence.find("GGA") != std::string::npos) useful = true; // 定位
        if (sentence.find("GLL") != std::string::npos) useful = true; // 定位

        if (useful) {
            // NMEA 协议要求以 \r\n 结尾
            if (sentence.length() < 2 || sentence.substr(sentence.length()-2) != "\r\n") {
                sentence += "\r\n";
            }
            sendUdp(sentence);
        }
    }

    // --- 生成逻辑 (备用) ---
    std::string calculateChecksum(const std::string& sentence) {
        int checksum = 0;
        for (char c : sentence) {
            checksum ^= c;
        }
        std::stringstream ss;
        ss << "*" << std::uppercase << std::hex << std::setw(2) << std::setfill('0') << checksum;
        return ss.str();
    }

    double quaternionToHeading(double x, double y, double z, double w) {
        double siny_cosp = 2 * (w * z + x * y);
        double cosy_cosp = 1 - 2 * (y * y + z * z);
        double yaw_rad = std::atan2(siny_cosp, cosy_cosp);
        double yaw_deg = yaw_rad * 180.0 / M_PI;
        double heading = 90.0 - yaw_deg;
        if (heading < 0) heading += 360.0;
        if (heading >= 360.0) heading -= 360.0;
        return heading;
    }

    void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
        double vx = msg->twist.twist.linear.x;
        double vy = msg->twist.twist.linear.y;
        double speed_ms = std::sqrt(vx*vx + vy*vy);
        double speed_knots = speed_ms * 1.94384; 

        double heading = quaternionToHeading(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );

        std::stringstream ss_hdt;
        ss_hdt << "HEHDT," << std::fixed << std::setprecision(2) << heading << ",T";
        std::string msg_hdt = "$" + ss_hdt.str() + calculateChecksum(ss_hdt.str()) + "\r\n";
        sendUdp(msg_hdt);

        std::stringstream ss_vtg;
        ss_vtg << "GPVTG," << std::fixed << std::setprecision(2) << heading << ",T,,M,"
               << speed_knots << ",N," << (speed_ms * 3.6) << ",K,A";
        std::string msg_vtg = "$" + ss_vtg.str() + calculateChecksum(ss_vtg.str()) + "\r\n";
        sendUdp(msg_vtg);
    }

    void sendUdp(const std::string& data) {
        if (sock_ >= 0) {
            sendto(sock_, data.c_str(), data.length(), 0, 
                   (struct sockaddr*)&target_addr_, sizeof(target_addr_));
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NmeaSenderNode>());
    rclcpp::shutdown();
    return 0;
}