#!/usr/bin/env python3
"""
USV Environment Dynamics Node
Applies wind and current forces directly to the USV model in Gazebo.
"""

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import math
import subprocess

try:
    import gz.transport13 as transport
    from gz.msgs10.entity_wrench_pb2 import EntityWrench
    from gz.msgs10.entity_pb2 import Entity
except ImportError as e:
    print(f"Error importing gz transport: {e}")
    sys.exit(1)

class UsvEnvDynamicsNode(Node):
    def __init__(self):
        super().__init__('usv_env_dynamics_node')
        
        self.declare_parameter('model_name', 'wamv')
        self.declare_parameter('k_wind', 5000.0)      # Force coefficient for wind
        self.declare_parameter('k_current', 50000.0)  # Force coefficient for current
        self.declare_parameter('publish_rate', 20.0)  # Hz
        
        self.model_name = self.get_parameter('model_name').value
        self.k_wind = self.get_parameter('k_wind').value
        self.k_current = self.get_parameter('k_current').value
        self.publish_rate = self.get_parameter('publish_rate').value

        self.wind_speed = 0.0
        self.wind_direction = 0.0
        self.current_x = 0.0
        self.current_y = 0.0

        # Subscriptions
        self.create_subscription(Float64, '/vrx/environment/wind/speed', self.wind_speed_cb, 10)
        self.create_subscription(Float64, '/vrx/environment/wind/direction_deg', self.wind_dir_cb, 10)
        self.create_subscription(Twist, '/vrx/environment/current', self.current_cb, 10)

        # Gazebo connection
        self.gz_node = transport.Node()
        self.gz_pub = None
        self.world_name = None

        # Timer
        period = 1.0 / self.publish_rate
        self.timer = self.create_timer(period, self.update_physics)
        self.get_logger().info("USV Environment Dynamics Node Started. Waiting for Gazebo World...")

    def find_gz_world(self):
        try:
            out = subprocess.check_output(['gz', 'topic', '-l'], timeout=2).decode()
            for line in out.split('\n'):
                if line.startswith('/world/') and '/wrench' in line:
                    return line.split('/')[2]
        except Exception:
            pass
        return None

    def wind_speed_cb(self, msg):
        self.wind_speed = msg.data

    def wind_dir_cb(self, msg):
        self.wind_direction = msg.data

    def current_cb(self, msg):
        self.current_x = msg.linear.x
        self.current_y = msg.linear.y

    def update_physics(self):
        # 1. Ensure Gazebo publisher is ready
        if not self.gz_pub:
            self.world_name = self.find_gz_world()
            if self.world_name:
                topic = f'/world/{self.world_name}/wrench'
                self.gz_pub = self.gz_node.advertise(topic, EntityWrench)
                self.get_logger().info(f'Gazebo publisher hooked to {topic}')
            else:
                return

        # 2. Wind Force Calculation
        # F_wind = K_wind * v_wind^2
        # Note: wind_direction denotes where it is going to
        angle_rad = math.radians(self.wind_direction)
        wind_f_mag = self.k_wind * (self.wind_speed ** 2)
        fw_x = wind_f_mag * math.cos(angle_rad)
        fw_y = wind_f_mag * math.sin(angle_rad)

        # 3. Current Force Calculation
        # F_current = K_current * |v_current| * v_current
        cx_mag = abs(self.current_x)
        cy_mag = abs(self.current_y)
        fc_x = self.k_current * (cx_mag * self.current_x)
        fc_y = self.k_current * (cy_mag * self.current_y)

        # 4. Total force to apply
        total_fx = fw_x + fc_x
        total_fy = fw_y + fc_y
        
        if total_fx == 0.0 and total_fy == 0.0:
            return

        # 5. Send Wrench to Gazebo Model
        msg = EntityWrench()
        msg.entity.name = self.model_name
        msg.entity.type = Entity.MODEL
        
        msg.wrench.force.x = total_fx
        msg.wrench.force.y = total_fy
        msg.wrench.force.z = 0.0
        
        # We assume torques are zero for simplified environment dynamics
        msg.wrench.torque.x = 0.0
        msg.wrench.torque.y = 0.0
        msg.wrench.torque.z = 0.0

        self.gz_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = UsvEnvDynamicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
