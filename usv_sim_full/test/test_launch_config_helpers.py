"""launch_config_helpers 单元测试。"""

from __future__ import annotations

import os
import tempfile

import yaml

from usv_sim_full.launch_config_helpers import (
    merge_ground_truth_gazebo_entity_params,
    write_ground_truth_entity_params_yaml,
)


def test_merge_entity_params_includes_mesh_profile():
    full_config = os.path.abspath(
        os.path.join(
            os.path.dirname(__file__),
            "..",
            "config",
            "full_config.yaml",
        )
    )
    scen_gt_cfg = {
        "enabled": True,
        "gazebo_visual": True,
        "gazebo_target_geometry": "mesh_profile",
        "gazebo_mesh_profile": "../description/models/target_ship/10m_mesh_profile.yaml",
        "model_mass_kg": 50.0,
    }
    params = merge_ground_truth_gazebo_entity_params(
        "sydney_regatta",
        scen_gt_cfg,
        "sim/ground_truth",
        "gt_ctrv_",
        10.0,
        1.0,
        full_config,
    )
    assert params["gazebo_target_geometry"] == "mesh_profile"
    mesh_path = params["gazebo_mesh_profile"]
    assert os.path.isfile(mesh_path)
    assert mesh_path.endswith("10m_mesh_profile.yaml")


def test_write_entity_params_yaml_includes_mesh_profile():
    full_config = os.path.abspath(
        os.path.join(
            os.path.dirname(__file__),
            "..",
            "config",
            "full_config.yaml",
        )
    )
    scen_gt_cfg = {
        "gazebo_target_geometry": "mesh_profile",
        "gazebo_mesh_profile": "../description/models/target_ship/10m_mesh_profile.yaml",
    }
    with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as f:
        dest = f.name
    try:
        write_ground_truth_entity_params_yaml(
            scen_gt_cfg,
            dest,
            full_config_path=full_config,
        )
        with open(dest, "r", encoding="utf-8") as f:
            payload = yaml.safe_load(f)
        ros_params = payload["scenario_ground_truth_gazebo_entity"]["ros__parameters"]
        assert ros_params["gazebo_target_geometry"] == "mesh_profile"
        assert os.path.isfile(ros_params["gazebo_mesh_profile"])
    finally:
        os.unlink(dest)
