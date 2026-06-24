import sys
import os
import subprocess
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QTableWidget, QTableWidgetItem, 
                             QPushButton, QHeaderView, QGroupBox)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QColor

class MonitorGUI(QMainWindow):
    def __init__(self, ros_node, config_data):
        super().__init__()
        self.ros_node = ros_node
        self.config_data = config_data
        self.init_ui()

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(1000)

    def init_ui(self):
        self.setWindowTitle("USV Simulation Health Monitor - V2")
        self.resize(1000, 700)
        
        main_widget = QWidget()
        main_layout = QVBoxLayout(main_widget)
        self.setCentralWidget(main_widget)

        top_layout = QHBoxLayout()
        self.lbl_status = QLabel("Status: Waiting...")
        self.lbl_status.setStyleSheet("font-size: 16px; font-weight: bold;")
        top_layout.addWidget(self.lbl_status)
        
        btn_rqt_graph = QPushButton("Open rqt_graph")
        btn_rqt_graph.clicked.connect(lambda: subprocess.Popen(["rqt_graph"]))
        top_layout.addWidget(btn_rqt_graph)

        btn_rqt_image = QPushButton("Open rqt_image_view")
        btn_rqt_image.clicked.connect(lambda: subprocess.Popen(["rqt_image_view"]))
        top_layout.addWidget(btn_rqt_image)
        
        main_layout.addLayout(top_layout)

        node_group = QGroupBox("Expected Core Nodes")
        node_layout = QVBoxLayout()
        node_group.setLayout(node_layout)
        
        self.node_labels = {}
        for n in self.config_data['expected_nodes']:
            lbl = QLabel(f"🔴 {n}")
            self.node_labels[n] = lbl
            node_layout.addWidget(lbl)
        
        main_layout.addWidget(node_group)

        topic_group = QGroupBox("Expected Bridge Topics")
        topic_layout = QVBoxLayout()
        topic_group.setLayout(topic_layout)

        self.table = QTableWidget()
        self.table.setColumnCount(6)
        self.table.setHorizontalHeaderLabels([
            "Direction", "GZ Topic", "ROS Topic", 
            "GZ Status", "ROS Hz", "Sys Status"
        ])
        self.table.horizontalHeader().setSectionResizeMode(1, QHeaderView.Stretch)
        self.table.horizontalHeader().setSectionResizeMode(2, QHeaderView.Stretch)
        
        expected_topics = self.config_data['expected_topics']
        self.table.setRowCount(len(expected_topics))
        
        for i, t in enumerate(expected_topics):
            # Direction
            self.table.setItem(i, 0, QTableWidgetItem(t.get('direction', 'GZ_TO_ROS')))
            # GZ Topic
            self.table.setItem(i, 1, QTableWidgetItem(t['gz_name']))
            # ROS Topic
            self.table.setItem(i, 2, QTableWidgetItem(t['ros_name']))
            
            # GZ status
            gz_item = QTableWidgetItem("Waiting")
            gz_item.setTextAlignment(Qt.AlignCenter)
            self.table.setItem(i, 3, gz_item)

            # ROS HZ
            self.table.setItem(i, 4, QTableWidgetItem("0.0"))
            
            # Sys status
            sys_item = QTableWidgetItem("Waiting")
            sys_item.setTextAlignment(Qt.AlignCenter)
            self.table.setItem(i, 5, sys_item)

        topic_layout.addWidget(self.table)
        main_layout.addWidget(topic_group)

    def update_ui(self):
        status = self.ros_node.get_status()
        active_nodes = status['nodes']
        topics_hz = status['topics_hz']
        active_gz_topics = status['gz_topics']

        all_good = True

        for n, lbl in self.node_labels.items():
            is_active = any(n in an for an in active_nodes)
            if is_active:
                lbl.setText(f"🟢 {n}")
                lbl.setStyleSheet("color: green;")
            else:
                lbl.setText(f"🔴 {n}")
                lbl.setStyleSheet("color: red;")
                all_good = False

        for i, t in enumerate(self.config_data['expected_topics']):
            gz_name = t['gz_name']
            ros_name = t['ros_name']
            direction = t.get('direction', 'GZ_TO_ROS')
            exp_hz = t['expected_hz']
            actual_hz = topics_hz.get(ros_name, 0.0)

            # Update GZ status
            gz_item = self.table.item(i, 3)
            gz_exists = gz_name in active_gz_topics
            if gz_exists:
                gz_item.setText("🟢 Exists")
                gz_item.setBackground(QColor(100, 255, 100))
            else:
                gz_item.setText("🔴 Missing")
                gz_item.setBackground(QColor(255, 100, 100))
                if direction == 'GZ_TO_ROS':
                    all_good = False

            # Update ROS hz
            self.table.item(i, 4).setText(f"{actual_hz:.1f}")

            # Update overall status
            sys_item = self.table.item(i, 5)
            
            if direction == 'GZ_TO_ROS':
                if not gz_exists:
                    sys_item.setText("🔴 Dead in GZ")
                    sys_item.setBackground(QColor(255, 100, 100))
                elif actual_hz == 0:
                    sys_item.setText("🔴 Bridge/ROS Fail")
                    sys_item.setBackground(QColor(255, 100, 100))
                    all_good = False
                elif exp_hz > 0 and actual_hz < (exp_hz * 0.5):
                    sys_item.setText("🟡 Low Hz")
                    sys_item.setBackground(QColor(255, 255, 100))
                else:
                    sys_item.setText("🟢 OK")
                    sys_item.setBackground(QColor(100, 255, 100))
            else:
                # ROS_TO_GZ
                sys_item.setText("🟢 OK (Send)")
                sys_item.setBackground(QColor(100, 255, 100))

        if all_good:
            self.lbl_status.setText("Status: ALL GREEN")
            self.lbl_status.setStyleSheet("color: green; font-size: 16px; font-weight: bold;")
        else:
            self.lbl_status.setText("Status: WARNING / ERROR / OFFLINE")
            self.lbl_status.setStyleSheet("color: red; font-size: 16px; font-weight: bold;")
