"""Localization ownership contracts, without launching ROS or Gazebo processes."""

import importlib.util
import json
import re
import shutil
import subprocess
from pathlib import Path
from types import SimpleNamespace

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction

from usv_sim_full.launch_config_helpers import build_mmwave_4d_cloud_parameters


PACKAGE = Path(__file__).resolve().parents[1]
SIMULATION = PACKAGE.parent


@pytest.fixture
def launches(monkeypatch):
    modules = {}
    for name, relative_path in (
        ("main", "launch/main.launch.py"),
        ("robot", "launch/components/robot_bringup.launch.py"),
    ):
        spec = importlib.util.spec_from_file_location(name, PACKAGE / relative_path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        monkeypatch.setattr(module, "Node", lambda **kwargs: SimpleNamespace(**kwargs))
        modules[name] = module
    monkeypatch.setattr(
        modules["main"], "get_package_share_directory", lambda package: str(PACKAGE)
    )
    return modules


def _context(description, **overrides):
    context = LaunchContext()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    context.launch_configurations.update(overrides)
    return context


def _robot_description(module, monkeypatch):
    callbacks = {}

    def capture_callback(*, function):
        callbacks[function.__name__] = function
        return OpaqueFunction(function=function)

    monkeypatch.setattr(module, "OpaqueFunction", capture_callback)
    return module.generate_launch_description(), callbacks


@pytest.mark.parametrize("entry", ["main", "robot"])
def test_external_localization_defaults_to_false(launches, entry):
    context = _context(launches[entry].generate_launch_description())
    assert context.launch_configurations["external_localization"] == "false"


@pytest.mark.parametrize("robot_name", ["usv_1", "usv_2"])
@pytest.mark.parametrize("external", ["false", "true", "0", "1"])
def test_external_mode_skips_rsp_and_legacy_mode_keeps_model_tf(
    launches, monkeypatch, tmp_path, robot_name, external
):
    module = launches["robot"]
    description, callbacks = _robot_description(module, monkeypatch)
    isolated = external in ("true", "1")
    urdf = tmp_path / "robot.urdf"
    original = '<robot><mesh filename="model://hull/mesh/hull.dae"/></robot>'
    converted = original.replace("model://hull/", "package://usv_sim_full/description/models/hull/")
    if not isolated:
        urdf.write_text(original)
    conversions = []

    def convert(content):
        conversions.append(content)
        return converted

    monkeypatch.setattr(module, "convert_model_uri_to_package_uri", convert)
    context = _context(
        description, robot_name=robot_name, urdf_path=str(urdf),
        external_localization=external, use_sim_time="true",
    )
    nodes = callbacks["launch_robot_state_publisher"](context)
    if isolated:
        assert nodes == [] and conversions == []
        assert not urdf.exists()
    else:
        assert len(nodes) == 1 and conversions == [original]
        node = nodes[0]
        assert node.package == node.executable == node.name == "robot_state_publisher"
        assert node.parameters == [{"robot_description": converted, "use_sim_time": True}]
        assert node.remappings == [("joint_states", f"/model/{robot_name}/joint_state")]
        assert urdf.read_text() == original


@pytest.mark.parametrize("robot_name", ["usv_1", "usv_2"])
@pytest.mark.parametrize("external", ["false", "true", "0", "1"])
def test_robot_bridge_isolation_and_truth_draft(launches, monkeypatch, robot_name, external):
    description, callbacks = _robot_description(launches["robot"], monkeypatch)
    context = _context(
        description,
        robot_name=robot_name,
        bridge_config_path="/not-read/bridge.yaml",
        external_localization=external,
    )
    nodes = callbacks["launch_unnamespaced_gz_ros_stack"](context)
    bridge = next(node for node in nodes if node.package == "ros_gz_bridge")
    public_odom = f"/{robot_name}/odom"
    truth_odom = f"/{robot_name}/source/gazebo/odom"
    isolated = external in ("true", "1")
    expected_remaps = [
        (public_odom, truth_odom),
        (f"/{robot_name}/sensors/gps/gps_sensor/data", f"/{robot_name}/source/gazebo/gps"),
        (f"/{robot_name}/sensors/imu/imu_sensor/data", f"/{robot_name}/source/gazebo/imu"),
    ]
    assert getattr(bridge, "remappings", []) == (expected_remaps if isolated else [])
    assert bridge.parameters[0]["config_file"] == "/not-read/bridge.yaml"
    # An exact three-topic remap leaves every ROS_TO_GZ thruster channel unchanged.
    assert all("thrusters" not in topic for pair in expected_remaps for topic in pair)
    assert all(not hasattr(node, "remappings") for node in nodes if node is not bridge)
    draft = next(node for node in nodes if node.executable == "hull_draft_publisher")
    assert draft.parameters[0]["odom_topic"] == (truth_odom if isolated else public_odom)
    assert draft.parameters[0]["draft_topic"] == f"/{robot_name}/hull_draft"
    odom_tf = [node for node in nodes if node.executable == "odom_tf_broadcaster"]
    assert len(odom_tf) == (0 if isolated else 1)
    radar = next(node for node in nodes if node.executable == "radar_gz_bridge")
    assert radar.parameters[0]["gz_topic"] == f"/{robot_name}/radar/spokes"


@pytest.mark.parametrize("external", ["false", "true"])
@pytest.mark.parametrize("enabled", ["false", "true"])
def test_external_mode_wins_over_legacy_robot_localization(
    launches, monkeypatch, capsys, external, enabled
):
    description, callbacks = _robot_description(launches["robot"], monkeypatch)
    context = _context(
        description,
        robot_name="usv_1",
        external_localization=external,
        enable_robot_localization=enabled,
    )
    nodes = callbacks["schedule_robot_localization"](context)
    if external == "true":
        assert nodes == []
        if enabled == "true":
            assert "overrides enable_robot_localization=true" in capsys.readouterr().out
    elif enabled == "false":
        assert nodes == []
    else:
        assert len(nodes) == 1 and isinstance(nodes[0], TimerAction)
        assert nodes[0].period == 5.0


@pytest.mark.parametrize("external", ["false", "true"])
@pytest.mark.parametrize("relay", ["false", "true"])
def test_main_propagation_ownership_and_mmwave_mounts(
    launches, monkeypatch, capsys, external, relay
):
    main = launches["main"]
    sensor = {
        "name": "mmwave_front",
        "type": "mmwave_radar",
        "parent_link": "base_link",
        "xyz": [1, -2, 1.9],
        "rpy": [0.1, -0.2, 0.3],
        "override_topic": "/sensors/mmwave/mmwave_front/points",
    }
    config = {
        "visualization": {"launch_rviz": False},
        "robot_1": {"name": "usv_1", "sensors": [sensor]},
        "robot_2": {"name": "usv_2", "sensors": [sensor]},
    }
    session = {
        "rviz_config_path": "/not-read/session.rviz",
        "obstacle_layout_path": "/not-read/obstacles.json",
        "robots": [
            {"name": name, "urdf_path": "/not-read/robot.urdf", "bridge_yaml_path": "/not-read/bridge.yaml"}
            for name in ("usv_1", "usv_2")
        ],
    }
    monkeypatch.setattr(main.yaml, "safe_load", lambda stream: config)
    monkeypatch.setattr(main, "session_manager_executable_path", lambda: "/not-executed/session_manager")
    monkeypatch.setattr(main.subprocess, "run", lambda *args, **kwargs: SimpleNamespace(stdout=json.dumps(session)))
    monkeypatch.setattr(main, "_append_displays_to_rviz_config", lambda *args: None)
    monkeypatch.setattr(main, "load_mmwave_sensor_defaults", lambda *args: {})
    monkeypatch.setattr(main, "load_mmwave_cluster_defaults", lambda *args: {})
    context = _context(
        main.generate_launch_description(),
        external_localization=external,
        enable_robot_localization="true",
        use_static_map_odom_tf="true",
        enable_tf_namespace_relay=relay,
        enable_dynamic_buoy_manager="false",
    )
    items = main.launch_setup(context)
    robot_includes = [
        item for item in items
        if isinstance(item, IncludeLaunchDescription) and "robot_name" in dict(item.launch_arguments)
    ]
    assert len(robot_includes) == 2
    for item in robot_includes:
        arguments = dict(item.launch_arguments)
        assert arguments["external_localization"] == external
        assert arguments["enable_robot_localization"] == ("false" if external == "true" else "true")
        assert arguments["localization_params_file"] == context.launch_configurations["localization_params_file"]
        assert arguments["localization_start_delay"] == context.launch_configurations["localization_start_delay"]
    nodes = [item for item in items if isinstance(item, SimpleNamespace)]
    expected_legacy_count = 0 if external == "true" else 2
    for executable in ("usv_sim_wrapper", "static_transform_publisher"):
        legacy_nodes = [node for node in nodes if node.executable == executable]
        assert len(legacy_nodes) == expected_legacy_count
    if external == "true":
        assert "overrides enable_robot_localization/use_static_map_odom_tf" in capsys.readouterr().out
    relays = [node for node in nodes if node.executable == "tf_namespace_relay"]
    assert len(relays) == (2 if relay == "true" else 0)
    clouds = [node for node in nodes if node.executable == "mmwave_4d_cloud_node"]
    assert len(clouds) == 2
    for robot_name, node in zip(("usv_1", "usv_2"), clouds):
        params = node.parameters[0]
        prefix = f"/{robot_name}/sensors/mmwave/mmwave_front"
        assert params["input_topic"] == f"{prefix}/points_gz"
        assert params["output_topic"] == f"{prefix}/points"
        if external == "true":
            assert params["odom_topic"] == f"/{robot_name}/source/gazebo/odom"
            assert params["use_truth_pose"] is True
            assert params["sensor_xyz"] == [1.0, -2.0, 1.9]
            assert params["sensor_rpy"] == [0.1, -0.2, 0.3]
        else:
            assert params["odom_topic"] == f"/{robot_name}/odom"
            assert "use_truth_pose" not in params
            assert "sensor_xyz" not in params and "sensor_rpy" not in params


def test_truth_mount_defaults_match_sensor_macros():
    params = build_mmwave_4d_cloud_parameters({}, "/in", "/out", "/truth", truth_sensor={})
    assert params["sensor_xyz"] == params["sensor_rpy"] == [0.0, 0.0, 0.0]
    assert all(isinstance(value, float) for value in params["sensor_xyz"])


def test_truth_mount_rejects_non_base_parent():
    with pytest.raises(ValueError, match="parent_link=base_link"):
        build_mmwave_4d_cloud_parameters(
            {}, "/in", "/out", "/truth", truth_sensor={"parent_link": "moving_link"}
        )


def test_mmwave_truth_geometry_has_no_estimated_tf_fallback():
    source = (SIMULATION / "sensor_plugins/usv_mmwave_sim/src/mmwave_4d_cloud_node.cpp").read_text()
    callback = source.split("void onCloud(", 1)[1]
    truth, legacy = callback.split("if (use_truth_pose_) {", 1)[1].split("} else {", 1)
    assert "lookupTransform" not in truth and "tf_buffer_" not in truth
    assert re.search(r"if \(!last_odom_\) \{[^}]*return;", truth)
    assert "tf2::fromMsg(pose.orientation, q_wb)" in truth
    assert "q_ws = q_wb * sensor_rotation_" in truth
    assert "tf2::quatRotate(q_wb, sensor_xyz_)" in truth
    assert "v_b + w_b.cross(sensor_xyz_)" in truth
    assert "lookupTransform" in legacy
    assert "out.header = msg->header" in callback
    assert 'declare_parameter<bool>("use_truth_pose", false)' in source


@pytest.fixture(scope="module")
def truth_time_gate(tmp_path_factory):
    compiler = shutil.which("c++")
    if compiler is None:
        pytest.skip("A C++ compiler is required for the standalone truth timestamp tests")
    source = (SIMULATION / "sensor_plugins/usv_mmwave_sim/src/mmwave_4d_cloud_node.cpp").read_text()
    assert 'declare_parameter<double>("truth_odom_max_dt", 0.25)' in source
    config_check = re.search(
        r"    if \(!std::isfinite\(truth_odom_max_dt_\).*?\n    \}", source, re.DOTALL
    ).group()
    callback = source.split("void onCloud(", 1)[1]
    stamp_check = callback.split("if (use_truth_pose_) {", 1)[1].split(
        "// Compose truth base pose", 1
    )[0]
    # Execute the production early-return guards without ROS, TF, publishers or simulation.
    program = r"""
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#define RCLCPP_WARN_THROTTLE(...) ((void)0)
struct Message {
  struct {struct {std::int32_t sec; std::uint32_t nanosec;} stamp;} header;
};
struct TruthGate {
  std::shared_ptr<Message> last_odom_;
  double truth_odom_max_dt_;
  bool accepted = false;
  explicit TruthGate(double max_dt) : truth_odom_max_dt_(max_dt) {
@CONFIG_CHECK@
  }
  void onCloud(const std::shared_ptr<Message> &msg) {
@STAMP_CHECK@
    accepted = true;
  }
};
int main(int argc, char **argv) {
  if (argc != 6) {return 2;}
  try {
    TruthGate gate(std::stod(argv[1]));
    if (std::string(argv[2]) != "missing") {
      gate.last_odom_ = std::make_shared<Message>();
      gate.last_odom_->header.stamp.sec = std::stoi(argv[2]);
      gate.last_odom_->header.stamp.nanosec = std::stoul(argv[3]);
    }
    auto cloud = std::make_shared<Message>();
    cloud->header.stamp.sec = std::stoi(argv[4]);
    cloud->header.stamp.nanosec = std::stoul(argv[5]);
    gate.onCloud(cloud);
    std::cout << (gate.accepted ? "accept" : "drop");
  } catch (const std::invalid_argument &) {
    std::cout << "invalid-config";
  }
}
""".replace("@CONFIG_CHECK@", config_check).replace("@STAMP_CHECK@", stamp_check)
    executable = tmp_path_factory.mktemp("truth_time_gate") / "truth_time_gate"
    result = subprocess.run(
        [compiler, "-x", "c++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
         "-o", str(executable), "-"],
        input=program, text=True, capture_output=True,
    )
    assert result.returncode == 0, result.stdout + result.stderr

    def run(odom_stamp, cloud_stamp, max_dt=0.25):
        arguments = [max_dt, *(odom_stamp or ("missing", 0)), *cloud_stamp]
        return subprocess.check_output([str(executable), *map(str, arguments)], text=True)

    return run


@pytest.mark.parametrize("odom_stamp,cloud_stamp,expected", [
    (None, (10, 0), "drop"),
    ((10, 0), (10, 0), "accept"),
    ((10, 0), (10, 100000000), "accept"),
    ((10, 100000000), (10, 0), "accept"),
    ((10, 0), (10, 250000000), "accept"),
    ((10, 250000000), (10, 0), "accept"),
    ((10, 0), (10, 250000001), "drop"),
    ((10, 250000001), (10, 0), "drop"),
    ((10, 0), (11, 0), "drop"),
    ((11, 0), (10, 0), "drop"),
    ((10, 900000000), (11, 150000000), "accept"),
    ((11, 150000000), (10, 900000000), "accept"),
    ((2147483647, 0), (2147483647, 250000001), "drop"),
    ((0, 0), (0, 1), "drop"),
    ((0, 1), (0, 0), "drop"),
    ((0, 1), (0, 2), "accept"),
    ((-1, 999999999), (0, 1), "drop"),
    ((0, 1), (-1, 999999999), "drop"),
    ((10, 1000000000), (11, 0), "drop"),
    ((11, 0), (10, 1000000000), "drop"),
    ((10, 4294967295), (14, 0), "drop"),
])
def test_truth_timestamp_gate(truth_time_gate, odom_stamp, cloud_stamp, expected):
    assert truth_time_gate(odom_stamp, cloud_stamp) == expected


@pytest.mark.parametrize("max_dt,expected", [(0.1, "drop"), (0.25, "accept"), (0.5, "accept")])
def test_truth_timestamp_tolerance_is_configurable(truth_time_gate, max_dt, expected):
    assert truth_time_gate((10, 0), (10, 200000000), max_dt) == expected


@pytest.mark.parametrize("max_dt", [0.0, -0.1, float("nan"), float("inf"), -float("inf")])
def test_truth_timestamp_tolerance_rejects_invalid_config(truth_time_gate, max_dt):
    assert truth_time_gate((10, 0), (10, 0), max_dt) == "invalid-config"
