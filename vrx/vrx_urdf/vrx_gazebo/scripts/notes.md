# vrx_gazebo/scripts 目录说明

包含用于生成/配置 WAM-V 模型的小脚本：

- __init__.py
  - 包初始化（空文件）。
- generate_wamv.py
  - 调用 `vrx_gazebo.configure_wamv.main()` 的轻量启动脚本，用于生成或配置 WAM-V 相关文件（基于上游 `vrx_gazebo` 包）。

此目录依赖 `vrx_gazebo` 包中的实现，通常不需要修改；可在清理时保留为轻量入口脚本。
