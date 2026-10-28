"""虚拟动态目标船（无 Gazebo）行为测试。"""

import pytest

from geometry_msgs.msg import Pose, Twist

from usv_sim_full.scripts.dynamic_ship_manager_node import DynamicShip

TARGET_ID = '12345678-9abc-def0-1234-56789abcdef0'


class _FakePublisher:
    def __init__(self, msg_type, topic, qos):
        self.msg_type = msg_type
        self.topic = topic
        self.qos = qos
        self.published = []

    def publish(self, msg):
        self.published.append(msg)


class _FakeNode:
    def __init__(self):
        self.publishers = []

    def create_publisher(self, msg_type, topic, qos):
        publisher = _FakePublisher(msg_type, topic, qos)
        self.publishers.append(publisher)
        return publisher


def _make_ship(is_virtual, speed=3.0, half_distance=50.0):
    node = _FakeNode()
    pose = Pose()
    pose.position.x = 0.0
    pose.position.y = 0.0
    pose.position.z = 0.0
    pose.orientation.w = 1.0
    ship = DynamicShip(
        model_name='dyn_target_1',
        target_id=TARGET_ID,
        pose=pose,
        half_distance=half_distance,
        shape='mesh_profile',
        speed=speed,
        color=(1.0, 0.2, 0.2),
        mesh_profile='',
        node=node,
        config_base_dir='',
        world_name='test_world',
        is_virtual=is_virtual,
    )
    return ship, node


def test_virtual_ship_does_not_create_gazebo_control_publisher():
    ship, node = _make_ship(is_virtual=True)

    assert node.publishers == []
    assert ship.cmd_vel_pub is None


def test_gazebo_ship_creates_control_publisher():
    ship, node = _make_ship(is_virtual=False)

    assert len(node.publishers) == 1
    assert node.publishers[0].topic == '/model/dyn_target_1/cmd_vel'
    assert ship.cmd_vel_pub is node.publishers[0]


def test_virtual_ship_command_is_not_published():
    ship, node = _make_ship(is_virtual=True)

    assert ship.publish_cmd_vel(Twist()) is False
    assert node.publishers == []


def test_virtual_ship_reverses_at_waypoint():
    ship, _ = _make_ship(is_virtual=True)

    for _ in range(340):
        ship.compute_cmd_vel(0.1, predict=True)

    assert ship.direction == -1
    assert ship.current_x < 100.0


def test_virtual_ship_twist_is_zero_while_turning():
    ship, _ = _make_ship(is_virtual=True)

    for _ in range(340):
        ship.compute_cmd_vel(0.1, predict=True)

    assert ship._turning_remaining > 0.0
    twist = ship.get_current_twist()
    assert twist.linear.x == pytest.approx(0.0)
    assert twist.linear.y == pytest.approx(0.0)
