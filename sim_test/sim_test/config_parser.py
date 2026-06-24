import yaml
import os
import sys

try:
    from usv_sim_full.scripts import session_manager
except ImportError as e:
    print(f'Failed to import session_manager: {e}')
    session_manager = None

def parse_config(config_path):
    if not os.path.exists(config_path):
        return None

    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    robot_name = config.get('robot', {}).get('name', 'usv_sim_full')
    
    expected_nodes = [
        'scenario_manager_node',
        'usv_sim_wrapper',
        'robot_state_publisher',
        'radar_controller',
        'bridge'
    ]

    expected_topics = []

    if session_manager is not None:
        bridges = session_manager.generate_bridge_config(config)
        
        for b in bridges:
            expected_topics.append({
                'ros_name': b['ros_topic_name'],
                'ros_type': b['ros_type_name'],
                'gz_name': b['gz_topic_name'],
                'gz_type': b['gz_type_name'],
                'direction': b['direction'],
                'expected_hz': 0
            })
            
        sensors = config.get('sensors', [])
        for s in sensors:
            name = str(s.get('name', '')).lower()
            hz = s.get('update_rate', 0)
            if hz > 0:
                for et in expected_topics:
                    if name in et['ros_name']:
                        et['expected_hz'] = hz

    return {
        'robot_name': robot_name,
        'expected_nodes': expected_nodes,
        'expected_topics': expected_topics
    }
