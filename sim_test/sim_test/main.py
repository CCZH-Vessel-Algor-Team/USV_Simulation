import os
import sys
import rclpy
import threading
import argparse

from PyQt5.QtWidgets import QApplication
from ament_index_python.packages import get_package_share_directory

from sim_test.config_parser import parse_config
from sim_test.hz_monitor import MonitorNode, spin_thread
from sim_test.gui import MonitorGUI

def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser()
    parser.add_argument('--config', type=str, default='', help='Path to full_config.yaml')
    parsed_args, _ = parser.parse_known_args()

    config_path = parsed_args.config
    if not config_path:
        try:
            pkg_share = get_package_share_directory('usv_sim_full')
            config_path = os.path.join(pkg_share, 'config', 'full_config.yaml')
        except Exception:
            print('Error: usv_sim_full not found. Build workspace and pass --config <path>.')
            sys.exit(1)

    config_data = parse_config(config_path)
    if not config_data:
        print(f"Error: Could not load config from {config_path}")
        sys.exit(1)

    # 启动 ROS Node 和自旋线程
    node = MonitorNode()
    t = threading.Thread(target=spin_thread, args=(node,), daemon=True)
    t.start()

    # 启动 GUI 应用
    app = QApplication(sys.argv)
    gui = MonitorGUI(node, config_data)
    gui.show()

    ret = app.exec_()
    
    # 清理
    rclpy.shutdown()
    t.join(timeout=1.0)
    sys.exit(ret)

if __name__ == '__main__':
    main()
