#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import sys, select, termios, tty
import math

# === 配置参数 ===
THRUST_STEP = 5.0           # 每次按键增加/减少的推力值
ANGLE_STEP_DEG = 15.0       # 每次按键增加/减少的角度(度)
ANGLE_STEP_RAD = math.radians(ANGLE_STEP_DEG) # 转换为弧度

# 最大值限制 (可选，防止数值无限增加)
MAX_THRUST = 1000.0
MAX_ANGLE_DEG = 180.0

msg = f"""
-----------------------------------------
   VRX 增量式/巡航控制键盘 (Cruise Control)
-----------------------------------------
   当前设置:
   速度步进: {THRUST_STEP} | 角度步进: {ANGLE_STEP_DEG}°
-----------------------------------------
   左侧推进器 (WASD)      |    右侧推进器 (方向键)
-------------------------|-----------------------
   W: 推力 +{THRUST_STEP}       |    ↑: 推力 +{THRUST_STEP}
   S: 推力 -{THRUST_STEP}       |    ↓: 推力 -{THRUST_STEP}
   A: 角度 +{ANGLE_STEP_DEG}° (左)  |    ←: 角度 +{ANGLE_STEP_DEG}° (左)
   D: 角度 -{ANGLE_STEP_DEG}° (右)  |    →: 角度 -{ANGLE_STEP_DEG}° (右)
-----------------------------------------
   空格键 (Space): 【急停】所有状态归零
   Q 键: 仅重置角度 (推力保持)
   E 键: 仅重置推力 (角度保持)
   CTRL-C: 退出
-----------------------------------------
"""

class IncrementalTeleop(Node):
    def __init__(self):
        super().__init__('incremental_teleop')
        
        # 定义话题
        self.pub_l_thrust = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.pub_l_pos    = self.create_publisher(Float64, '/wamv/thrusters/left/pos', 10)
        self.pub_r_thrust = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)
        self.pub_r_pos    = self.create_publisher(Float64, '/wamv/thrusters/right/pos', 10)

        # 初始化状态 (这些变量会一直保持，直到被修改)
        self.l_thrust = 0.0
        self.l_pos = 0.0
        self.r_thrust = 0.0
        self.r_pos = 0.0

        self.settings = termios.tcgetattr(sys.stdin)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
            if key == '\x1b':
                key += sys.stdin.read(2)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def constrain(self, val, min_val, max_val):
        return max(min(val, max_val), min_val)

    def publish_commands(self):
        # 限制角度在 -180 到 180 度之间 (可选)
        # self.l_pos = self.constrain(self.l_pos, -math.pi, math.pi)
        # self.r_pos = self.constrain(self.r_pos, -math.pi, math.pi)

        # 发布消息
        msg_l_t = Float64(); msg_l_t.data = float(self.l_thrust)
        msg_l_p = Float64(); msg_l_p.data = float(self.l_pos)
        msg_r_t = Float64(); msg_r_t.data = float(self.r_thrust)
        msg_r_p = Float64(); msg_r_p.data = float(self.r_pos)

        self.pub_l_thrust.publish(msg_l_t)
        self.pub_l_pos.publish(msg_l_p)
        self.pub_r_thrust.publish(msg_r_t)
        self.pub_r_pos.publish(msg_r_p)

    def print_status(self):
        # 格式化输出当前状态
        l_deg = math.degrees(self.l_pos)
        r_deg = math.degrees(self.r_pos)
        print(f"\r[状态] 左: T={self.l_thrust:.1f}, A={l_deg:.0f}° | 右: T={self.r_thrust:.1f}, A={r_deg:.0f}°   ", end="")

def main(args=None):
    rclpy.init(args=args)
    node = IncrementalTeleop()
    
    print(msg)
    node.print_status() # 初始打印

    try:
        while rclpy.ok():
            key = node.getKey()
            updated = False

            # === 左侧控制 (累加模式) ===
            if key == 'w':
                node.l_thrust += THRUST_STEP
                updated = True
            elif key == 's':
                node.l_thrust -= THRUST_STEP
                updated = True
            elif key == 'a':
                node.l_pos += ANGLE_STEP_RAD # 向左转
                updated = True
            elif key == 'd':
                node.l_pos -= ANGLE_STEP_RAD # 向右转
                updated = True

            # === 右侧控制 (累加模式) ===
            elif key == '\x1b[A': # Up Arrow
                node.r_thrust += THRUST_STEP
                updated = True
            elif key == '\x1b[B': # Down Arrow
                node.r_thrust -= THRUST_STEP
                updated = True
            elif key == '\x1b[D': # Left Arrow
                node.r_pos += ANGLE_STEP_RAD
                updated = True
            elif key == '\x1b[C': # Right Arrow
                node.r_pos -= ANGLE_STEP_RAD
                updated = True

            # === 辅助功能 ===
            elif key == ' ': # 急停 (归零)
                node.l_thrust = 0.0; node.l_pos = 0.0
                node.r_thrust = 0.0; node.r_pos = 0.0
                print("\n[STOP] 全船急停！")
                updated = True
            
            elif key == 'q': # 仅重置角度 (方便回正)
                node.l_pos = 0.0; node.r_pos = 0.0
                print("\n[Info] 推进器回正")
                updated = True
            
            elif key == 'e': # 仅重置推力 (停车)
                node.l_thrust = 0.0; node.r_thrust = 0.0
                print("\n[Info] 停车")
                updated = True

            elif key == '\x03': # CTRL-C
                break

            # 只有当按键改变了状态时，才重新发布消息
            if updated:
                node.publish_commands()
                node.print_status()

    except Exception as e:
        print(e)

    finally:
        # 退出时归零，防止船一直跑
        msg_zero = Float64(); msg_zero.data = 0.0
        node.pub_l_thrust.publish(msg_zero); node.pub_r_thrust.publish(msg_zero)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
