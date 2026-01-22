#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import sys, select, termios, tty
import math

# 配置参数
SPEED_VAL = 5.0          # 固定速度
ANGLE_VAL = math.pi / 4  # 45度 转换为弧度 (约0.785)

msg = """
-----------------------------------------
   VRX 双推进器独立控制键盘 (Dual Teleop)
-----------------------------------------
   左侧推进器 (WASD)      |    右侧推进器 (方向键)
-------------------------|-----------------------
   W: 前进 (Thrust 5)    |    ↑: 前进 (Thrust 5)
   S: 后退 (Thrust -5)   |    ↓: 后退 (Thrust -5)
   A: 左转 (Angle +45°)  |    ←: 左转 (Angle +45°)
   D: 右转 (Angle -45°)  |    →: 右转 (Angle -45°)
-----------------------------------------
   空格键 (Space): 紧急停止 / 复位归零
   CTRL-C: 退出
-----------------------------------------
"""

class DualThrusterTeleop(Node):
    def __init__(self):
        super().__init__('dual_thruster_teleop')
        
        # 定义话题 (基于标准 VRX 话题命名)
        # 注意：如果你的仿真环境话题名不同，请在此处修改
        self.pub_l_thrust = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.pub_l_pos    = self.create_publisher(Float64, '/wamv/thrusters/left/pos', 10)
        self.pub_r_thrust = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)
        self.pub_r_pos    = self.create_publisher(Float64, '/wamv/thrusters/right/pos', 10)

        # 初始化状态
        self.l_thrust = 0.0
        self.l_pos = 0.0
        self.r_thrust = 0.0
        self.r_pos = 0.0

        # 终端设置用于读取按键
        self.settings = termios.tcgetattr(sys.stdin)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
            # 处理方向键的转义序列 (通常是 \x1b[A, \x1b[B 等)
            if key == '\x1b':
                key += sys.stdin.read(2)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def publish_commands(self):
        # 创建消息并发布
        msg_l_t = Float64(); msg_l_t.data = self.l_thrust
        msg_l_p = Float64(); msg_l_p.data = self.l_pos
        msg_r_t = Float64(); msg_r_t.data = self.r_thrust
        msg_r_p = Float64(); msg_r_p.data = self.r_pos

        self.pub_l_thrust.publish(msg_l_t)
        self.pub_l_pos.publish(msg_l_p)
        self.pub_r_thrust.publish(msg_r_t)
        self.pub_r_pos.publish(msg_r_p)

    def print_status(self):
        # 简单的状态打印，避免刷屏太快
        print(f"\r左: T={self.l_thrust: .1f}, A={math.degrees(self.l_pos): .0f}° | 右: T={self.r_thrust: .1f}, A={math.degrees(self.r_pos): .0f}°", end="")

def main(args=None):
    rclpy.init(args=args)
    node = DualThrusterTeleop()
    
    print(msg)

    try:
        while rclpy.ok():
            key = node.getKey()
            
            updated = False

            # === 左侧推进器逻辑 (WASD) ===
            if key == 'w':
                node.l_thrust = SPEED_VAL
                updated = True
            elif key == 's':
                node.l_thrust = -SPEED_VAL
                updated = True
            elif key == 'a':
                node.l_pos = ANGLE_VAL # 向左转
                updated = True
            elif key == 'd':
                node.l_pos = -ANGLE_VAL # 向右转
                updated = True

            # === 右侧推进器逻辑 (方向键) ===
            # 方向键通常返回转义序列：上=\x1b[A, 下=\x1b[B, 右=\x1b[C, 左=\x1b[D
            elif key == '\x1b[A': # Up
                node.r_thrust = SPEED_VAL
                updated = True
            elif key == '\x1b[B': # Down
                node.r_thrust = -SPEED_VAL
                updated = True
            elif key == '\x1b[D': # Left Arrow
                node.r_pos = ANGLE_VAL
                updated = True
            elif key == '\x1b[C': # Right Arrow
                node.r_pos = -ANGLE_VAL
                updated = True

            # === 停止/复位 (空格) ===
            elif key == ' ':
                node.l_thrust = 0.0
                node.l_pos = 0.0
                node.r_thrust = 0.0
                node.r_pos = 0.0
                updated = True
                print("\n[STOP] 所有推进器已归零")

            # === 退出 ===
            elif key == '\x03': # CTRL-C
                break

            if updated:
                node.publish_commands()
                node.print_status()

    except Exception as e:
        print(e)

    finally:
        # 退出前发送停止指令
        node.l_thrust = 0.0; node.r_thrust = 0.0
        node.publish_commands()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
