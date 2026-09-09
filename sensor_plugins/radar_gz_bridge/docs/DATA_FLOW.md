# 数据流输入输出

| 方向 | 名称 | 类型 | 说明 |
| --- | --- | --- | --- |
| 输入 | `gz_topic` 参数指定的话题。 | `gz.msgs.Float_V`。 | 方位角、分辨率和回波强度。 |
| 输出 | `ros_topic` 参数指定的话题。 | `marine_sensor_msgs/msg/RadarSector`。 | ROS 2 海事雷达扇区。 |
| 参数 | `frame_id`。 | 字符串。 | 输出消息坐标系。 |
| 参数 | `range_min`、`range_max`、`rotation_period`。 | 浮点数。 | 输出扇区的量程和扫描周期。 |
