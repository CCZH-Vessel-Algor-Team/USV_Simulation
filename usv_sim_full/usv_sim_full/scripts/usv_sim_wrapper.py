#!/usr/bin/env python3
"""
*****************************************************************************************
*  Copyright (C) 2026 MurphyChen, All Rights Reserved                                   *
*                                                                                       *
*  @brief    USV Simulation Interface Wrapper (Vehicle State Publisher)                 *
*  @author   MurphyChen                                                               *
*  @version  1.0.0                                                                      *
*  @date     2026.3.18                                                                *
*****************************************************************************************
"""

import rclpy
from rclpy.node import Node
import math

# 标准消息
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

# 全局自定义接口与常量
try:
    from usv_interfaces.msg import VesselState
except ImportError as e:
    print(f"Failed to import usv_interfaces: {e}. Ensure the package is built and sourced.")
    raise

try:
    from usv_interfaces import topics as _topics
    _TOPIC_VESSEL_STATE = _topics.TOPIC_VESSEL_STATE
    _TEMPLATE_GPS       = _topics.TEMPLATE_GPS
except ImportError:
    # fallback 常量，当 usv_interfaces topics 模块未安装时使用
    _TOPIC_VESSEL_STATE = "/usv/state/vessel"
    _TEMPLATE_GPS       = "/sensors/gps/{sensor_name}/fix"

class UsvSimWrapper(Node):
    def __init__(self):
        super().__init__('usv_sim_wrapper')
        
        # --- 内部状态缓存 ---
        # GPS频率低，缓存最近的一次数据
        self._latest_latitude = 0.0
        self._latest_longitude = 0.0
        self._latest_altitude = 0.0
        self._has_gps = False
        
        # --- 话题名称获取 ---
        # 默认从常量获取发布话题
        self.pub_state_topic = _TOPIC_VESSEL_STATE
        
        # 订阅话题名，一般通过参数透传，此处给入仿真的默认映射结构
        default_odom_topic = '/usv_1/odom'
        # 根据我们之前的配置，默认使用的名字可能是 "gps_sensor" 或者 "main"
        default_gps_topic = _TEMPLATE_GPS.format(sensor_name="gps_sensor")
        # 如果配置文件明确定义了 override_topic，通常是 /sensors/gps/data
        # 为了更简单地启动，我们允许把它作为节点参数传入
        
        self.declare_parameter('odom_topic', default_odom_topic)
        self.declare_parameter('gps_topic', '/sensors/gps/data') # match the full_config override behavior
        
        odom_topic = self.get_parameter('odom_topic').value
        gps_topic = self.get_parameter('gps_topic').value

        # --- 发布者 ---
        self.state_pub = self.create_publisher(VesselState, self.pub_state_topic, 10)
        self.get_logger().info(f"Publishing VesselState on: {self.pub_state_topic}")
        
        # --- 订阅者 ---
        # 订阅低频GPS
        self.gps_sub = self.create_subscription(
            NavSatFix, 
            gps_topic, 
            self.gps_callback, 
            10
        )
        self.get_logger().info(f"Subscribed to GPS: {gps_topic}")
        
        # 订阅高频Odom，作为状态发布的主驱动引擎
        self.odom_sub = self.create_subscription(
            Odometry, 
            odom_topic, 
            self.odom_callback, 
            10
        )
        self.get_logger().info(f"Subscribed to Odom: {odom_topic}")

    def gps_callback(self, msg: NavSatFix):
        """缓存GPS数据"""
        self._latest_latitude = msg.latitude
        self._latest_longitude = msg.longitude
        self._latest_altitude = msg.altitude
        self._has_gps = True

    def odom_callback(self, msg: Odometry):
        """以高频Odom为切入点，融合GPS并打包出VesselState"""
        
        state_msg = VesselState()
        
        # 1. 继承Header信息，以Odom的时间戳为准
        state_msg.header = msg.header
        
        # 2. 注入内部缓存的GNSS地理位置
        if self._has_gps:
            state_msg.latitude = self._latest_latitude
            state_msg.longitude = self._latest_longitude
            state_msg.altitude = float(self._latest_altitude)
        else:
            state_msg.latitude = 0.0
            state_msg.longitude = 0.0
            state_msg.altitude = 0.0
            
        # 3. 提取局部运动状态 (Odom -> Pose/Twist)
        state_msg.pose = msg.pose.pose
        state_msg.velocity = msg.twist.twist
        
        # 4. 解析四元数为欧拉角，赋予 roll/pitch/yaw 以便算法和UI使用
        try:
            q = msg.pose.pose.orientation
            
            # 使用数学公式将欧拉角从四元数中解算出来
            sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
            cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
            roll = math.atan2(sinr_cosp, cosr_cosp)

            sinp = math.sqrt(1.0 + 2.0 * (q.w * q.y - q.x * q.z))
            cosp = math.sqrt(1.0 - 2.0 * (q.w * q.y - q.x * q.z))
            pitch = 2.0 * math.atan2(sinp, cosp) - math.pi / 2.0

            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            state_msg.roll = float(roll)
            state_msg.pitch = float(pitch)
            state_msg.yaw = float(yaw)
            
        except Exception as e:
            self.get_logger().error(f"Error converting quaternion to euler: {e}")
            state_msg.roll = 0.0
            state_msg.pitch = 0.0
            state_msg.yaw = 0.0

        # 5. 组装设备健康等伪装数据 (作为仿真模拟)
        state_msg.battery_voltage = 48.0       # 模拟满电48V
        state_msg.battery_percentage = 99.9    # 模拟电量
        state_msg.leak_detected = False
        state_msg.cpu_temperature = 55.0

        # 最终发布
        self.state_pub.publish(state_msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = UsvSimWrapper()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Wrapper exception: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
