"""会话管理器的 URI 后处理测试。"""

from usv_sim_full.scripts.session_manager import post_process_urdf_for_gazebo


def test_post_process_urdf_converts_vrx_model_uri(tmp_path):
    urdf_path = tmp_path / "robot.urdf"
    urdf_path.write_text(
        '<mesh filename="package://vrx_gz/models/body-Base/mesh/M5_body.dae"/>',
        encoding="utf-8",
    )

    post_process_urdf_for_gazebo(str(urdf_path))

    assert urdf_path.read_text(encoding="utf-8") == (
        '<mesh filename="model://body-Base/mesh/M5_body.dae"/>'
    )
