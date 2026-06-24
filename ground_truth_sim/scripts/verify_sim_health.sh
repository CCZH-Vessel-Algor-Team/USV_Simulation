#!/bin/bash
# USV Sim 验证测试脚本
# 使用前确保在另一个终端已经启动了 sim_vision 和 sim_mmwave 节点：
# ros2 launch ground_truth_sensor_sim three_sensor_sim.launch.py

# 设置超时时间（秒）
TIMEOUT=3

echo "=========================================================="
echo "    USV 仿真数据源 (Sim Packages) 自动健康检查工具"
echo "=========================================================="
echo ""

# 环境初始化检查
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ 错误: 未找到 ROS 2 环境。请先执行 'source /opt/ros/<distro>/setup.bash' 和 'source install/setup.bash'"
    exit 1
fi

echo "正在检查仿真环境主题健康度..."
echo "=========================================================="

# 1. 检查地面真值 (Ground Truth)
echo "[1] 检查 Ground Truth: /sim/ground_truth"
GT_MSG_TYPE=$(ros2 topic type /sim/ground_truth 2>/dev/null)
if [ -n "$GT_MSG_TYPE" ]; then
    echo "  -> 消息类型: $GT_MSG_TYPE"
    # 获取频率
    GT_HZ=$(timeout $TIMEOUT ros2 topic hz /sim/ground_truth | grep -oP 'average rate: \K[0-9.]+' | head -n 1)
    if [ -n "$GT_HZ" ]; then
        echo "  -> 频率: ${GT_HZ} Hz (目标: 50.0 Hz)"
        echo "  ✅ 真值节点运行正常。"
    else
        echo "  ❌ 错误: 未能获取 /sim/ground_truth 频率。"
    fi
else
    echo "  ❌ 错误: 无法获取 /sim/ground_truth，请确认节点已启动。"
fi
echo ""

# 2. 检查视觉与雷达方向检测
DIRECTIONS=("front" "rear" "left" "right")

# 测试视觉话题
for dir in "${DIRECTIONS[@]}"; do
    VISION_TOPIC="/vision/${dir}/detections"
    echo "[-] 检查视觉 (${dir}): $VISION_TOPIC"
    V_MSG_TYPE=$(ros2 topic type $VISION_TOPIC 2>/dev/null)
    if [ -n "$V_MSG_TYPE" ]; then
        echo "  -> 消息类型: $V_MSG_TYPE"
        V_HZ=$(timeout $TIMEOUT ros2 topic hz $VISION_TOPIC | grep -oP 'average rate: \K[0-9.]+' | head -n 1)
        if [ -n "$V_HZ" ]; then
            echo "  -> 频率: ${V_HZ} Hz (目标: 25.0 Hz)"
        else
            echo "  ❌ 警告: 未收到 $VISION_TOPIC 数据频率反馈。"
        fi
    else
        echo "  ❌未找到: $VISION_TOPIC"
    fi
done
echo ""

# 测试雷达话题
for dir in "${DIRECTIONS[@]}"; do
    RADAR_TOPIC="/mmwave/${dir}/targets"
    echo "[-] 检查毫米波 (${dir}): $RADAR_TOPIC"
    R_MSG_TYPE=$(ros2 topic type $RADAR_TOPIC 2>/dev/null)
    if [ -n "$R_MSG_TYPE" ]; then
        echo "  -> 消息类型: $R_MSG_TYPE"
        R_HZ=$(timeout $TIMEOUT ros2 topic hz $RADAR_TOPIC | grep -oP 'average rate: \K[0-9.]+' | head -n 1)
        if [ -n "$R_HZ" ]; then
            echo "  -> 频率: ${R_HZ} Hz (目标: 15.0 Hz)"
        else
            echo "  ❌ 警告: 未收到 $RADAR_TOPIC 数据频率反馈。"
        fi
    else
         echo "  ❌未找到: $RADAR_TOPIC"
    fi
done
echo "=========================================================="

echo ""
echo "🌟 最终诊断总结与指引："
echo "1. 真值数据 (/sim/ground_truth) 应正常输出，预期消息类型为 \`usv_interfaces/msg/GlobalTrackArray\`，频率为 50 Hz。"
echo "2. 所有 4 个方向的视觉数据 (/vision/*/detections) 应正常输出，预期消息类型为 \`usv_interfaces/msg/VisionDetectionArray\`，频率为 25 Hz。"
echo "3. 所有 4 个方向的毫米波雷达数据 (/mmwave/*/targets) 应正常输出，预期消息类型为 \`usv_interfaces/msg/MmwaveTargetArray\`，频率为 15 Hz。"
echo ""
echo ">> 如果上述所有子项均显示频率，则代表仿真层消息路由健康！"
echo ">> [交互操作]: \e[32m请您在已经拉起的 RViz2 界面中，检查是否能正常看到各方向由模拟发出的传感器标识渲染块。\e[0m"
echo "=========================================================="
