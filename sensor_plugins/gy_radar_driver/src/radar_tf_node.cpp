#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cmath>
#include <string>

#include "usv_interfaces/msg/nav_radar_object.hpp"
#include "usv_interfaces/msg/nav_radar_frame.hpp"
#include "usv_interfaces/topics.hpp"

using std::placeholders::_1;

class RadarTfNode : public rclcpp::Node
{
public:
    RadarTfNode() : Node("radar_tf_node")
    {
        // 声明参数
        this->declare_parameter("parent_frame", usv_interfaces::FRAME_NAVRADAR);
        this->declare_parameter("child_frame_prefix", "arpa_target_");
        this->declare_parameter("coordinate_system", "head_up");  // "north_up" 或 "head_up"
        this->declare_parameter("invert_bearing", false);          // 调试用：反转方位角
        this->declare_parameter("debug_log", false);

        // 获取参数
        parent_frame_ = this->get_parameter("parent_frame").as_string();
        child_prefix_ = this->get_parameter("child_frame_prefix").as_string();
        coordinate_system_ = this->get_parameter("coordinate_system").as_string();
        invert_bearing_ = this->get_parameter("invert_bearing").as_bool();
        debug_log_ = this->get_parameter("debug_log").as_bool();

        // TF 广播器
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // 订阅 ARPA Frame
        subscription_ = this->create_subscription<usv_interfaces::msg::NavRadarFrame>(
            usv_interfaces::TOPIC_PERCEPTION_NAV_FRAME, 
            10, 
            std::bind(&RadarTfNode::arpa_frame_callback, this, _1));

        RCLCPP_INFO(this->get_logger(), 
                    "Radar TF Node 启动 | Parent: %s | System: %s | Invert: %s",
                    parent_frame_.c_str(), 
                    coordinate_system_.c_str(),
                    invert_bearing_ ? "true" : "false");
    }

private:
    void arpa_frame_callback(const usv_interfaces::msg::NavRadarFrame::SharedPtr msg)
    {
        if (msg->targets.empty()) return;

        for (const auto& target : msg->targets) {

            if (target.target_status != "T"){
                continue;
            }
            // === 关键修正：bearing 已经是弧度 ===
            // arpa_receiver_node 已经做了：bearing_ddeg * 0.1 * M_PI / 180.0
            // 所以这里 target.bearing 直接就是弧度！
            double bearing_rad = target.bearing;
            
            // 可选：反转方向（用于调试）
            if (invert_bearing_) {
                bearing_rad = -bearing_rad;
            }

            // === 坐标系转换 ===
            double ros_bearing;
            if (coordinate_system_ == "north_up") {
                // 北向上模式：航海坐标 → ROS坐标
                // 航海: 北=0°顺时针, ROS: X轴=0°逆时针
                // 转换公式: ROS角度 = 90° - 航海角度
                ros_bearing = (M_PI / 2.0) - bearing_rad;
            } else {
                // 船头向上模式：只需反转顺逆时针
                ros_bearing = -bearing_rad;
            }

            // === 距离已经是米，直接使用 ===
            double distance_m = target.distance;

            // === 计算笛卡尔坐标 ===
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = msg->header.stamp;
            t.header.frame_id = parent_frame_;
            t.child_frame_id = child_prefix_ + std::to_string(target.target_number);

            t.transform.translation.x = distance_m * std::cos(ros_bearing);
            t.transform.translation.y = distance_m * std::sin(ros_bearing);
            t.transform.translation.z = 0.0;

            // === 航向转换（course 也已经是弧度）===
            double course_rad = target.course * (M_PI / 180.0);  // ⬅️ 注意：course 是度数
            double ros_course;
            if (coordinate_system_ == "north_up") {
                ros_course = (M_PI / 2.0) - course_rad;
            } else {
                ros_course = -course_rad;
            }

            tf2::Quaternion q;
            q.setRPY(0, 0, ros_course);
            t.transform.rotation.x = q.x();
            t.transform.rotation.y = q.y();
            t.transform.rotation.z = q.z();
            t.transform.rotation.w = q.w();

            tf_broadcaster_->sendTransform(t);

            // === 调试日志 ===
            if (debug_log_ && target.target_number == 1) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "Target #%d: bearing=%.2frad(%.1f°) dist=%.1fm course=%.1f° -> x=%.1f y=%.1f",
                    target.target_number,
                    bearing_rad, bearing_rad * 180.0 / M_PI,
                    distance_m, target.course,
                    t.transform.translation.x, t.transform.translation.y);
            }
        }
    }

    // 成员变量
    std::string parent_frame_;
    std::string child_prefix_;
    std::string coordinate_system_;
    bool invert_bearing_;
    bool debug_log_;
    
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<usv_interfaces::msg::NavRadarFrame>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RadarTfNode>());
    rclcpp::shutdown();
    return 0;
}