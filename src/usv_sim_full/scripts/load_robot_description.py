#!/usr/bin/env python3
"""
******************************************************************************************
*  Copyright (C) 2026 MurphyChen, All Rights Reserved                                  *
*                                                                                        *
*  @brief    机器人描述加载器 - 从文件加载机器人描述并发布到参数服务器              *
*  @author   MurphyChen                                                                *
*  @version  1.0.0                                                                       *
*  @date     2026.1.21                                                                 *
******************************************************************************************
"""

import rclpy
from rclpy.node import Node
import sys


class RobotDescriptionLoader(Node):
    def __init__(self, urdf_path):
        super().__init__('robot_description_loader')
        
        # 读取URDF文件内容
        with open(urdf_path, 'r') as f:
            robot_description = f.read()
        
        # 声明并设置robot_description参数
        self.declare_parameter('robot_description', robot_description)
        
        self.get_logger().info(f"Robot description loaded from: {urdf_path}")
        self.get_logger().info("Robot description published to parameter server")


def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) < 2:
        print("Usage: ros2 run usv_sim_full load_robot_description.py <urdf_path>")
        return
    
    urdf_path = sys.argv[1]
    loader = RobotDescriptionLoader(urdf_path)
    
    # 短暂运行后退出，让参数发布完成
    rclpy.spin_once(loader, timeout_sec=0.1)
    loader.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()