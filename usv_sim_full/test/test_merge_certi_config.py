"""认证配置合并工具的路径重定位测试。"""

import importlib.util
from pathlib import Path


_TOOL_PATH = Path(__file__).parent.parent / "tools" / "merge_certi_config.py"
_SPEC = importlib.util.spec_from_file_location("merge_certi_config", _TOOL_PATH)
merge_certi_config = importlib.util.module_from_spec(_SPEC)
assert _SPEC.loader is not None
_SPEC.loader.exec_module(merge_certi_config)


def test_sensor_config_path_is_rebased_for_generated_config(tmp_path):
    config_dir = tmp_path / "config"
    sensor_file = config_dir / "three_vision_one_mmwave" / "sensor_config.yaml"
    sensor_file.parent.mkdir(parents=True)
    sensor_file.touch()
    base_path = config_dir / "certi_senario.yaml"
    base_path.touch()
    out_path = config_dir / "generated" / "C1-001.merged.yaml"

    resolved = merge_certi_config.resolve_sensor_config_rel(
        str(base_path),
        str(out_path),
        "three_vision_one_mmwave/sensor_config.yaml",
    )

    assert resolved == "../three_vision_one_mmwave/sensor_config.yaml"


def test_absolute_sensor_config_path_is_unchanged(tmp_path):
    sensor_file = tmp_path / "sensor_config.yaml"
    sensor_file.touch()

    resolved = merge_certi_config.resolve_sensor_config_rel(
        str(tmp_path / "certi_senario.yaml"),
        str(tmp_path / "generated" / "C1-001.merged.yaml"),
        str(sensor_file),
    )

    assert resolved == str(sensor_file)
