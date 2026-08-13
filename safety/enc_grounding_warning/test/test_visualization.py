from enc_grounding_warning.route_depth_publisher_node import depth_to_color


def test_depth_gradient_is_warm_to_cold():
    r_shallow, g_shallow, b_shallow = depth_to_color(0.0, 0.0, 5.0)
    r_deep, g_deep, b_deep = depth_to_color(5.0, 0.0, 5.0)

    assert r_shallow > b_shallow      # 浅水偏暖
    assert b_deep > r_deep            # 深水偏冷
    assert r_shallow > r_deep


def test_depth_gradient_midpoint_is_yellowish():
    r, g, b = depth_to_color(2.5, 0.0, 5.0)
    assert r > 0.9
    assert g > 0.8
