# USV仿真系统架构V2.0 - 阶段一重构总结

## 重构后的目录结构

```
usv_sim_full/
├── config/                          # 配置文件
├── description/                     # 🎯 本地化资源目录
│   ├── models/                      # 3D模型文件
│   │   ├── WAM-V-Base/             # 船体基础模型
│   │   │   └── mesh/
│   │   │       ├── M5_body.dae
│   │   │       ├── WAM-V-Base.dae
│   │   │       ├── WAM-V_Albedo.png
│   │   │       ├── WAM-V_Normal.png
│   │   │       ├── WAM-V_Roughness.png
│   │   │       └── WAM-V_Metalness.png
│   │   ├── engine/                 # 发动机模型
│   │   │   └── mesh/
│   │   │       └── engine.dae
│   │   └── propeller/              # 螺旋桨模型
│   │       └── mesh/
│   │           └── propeller.dae
│   └── urdf/                        # URDF/Xacro文件
│       ├── battery.xacro           # 电池组件
│       ├── cpu_cases.xacro         # CPU机箱组件
│       ├── wamv_base.urdf.xacro    # 基础船体描述
│       └── thrusters/              # 推进器组件
│           └── engine.xacro
├── launch/                          # 启动文件
│   ├── main.launch.py
│   └── components/
│       ├── infra_sim.launch.py     # 基础设施启动（已修改）
│       ├── robot_bringup.launch.py
│       └── visualization.launch.py
├── scripts/                         # 脚本文件
│   └── session_manager.py          # 会话管理器（已修改）
├── templates/                       # 模板文件
├── test_env/                        # 测试环境
├── worlds/                          # 🎯 本地化世界文件
│   ├── sydney_regatta.sdf
│   ├── wayfinding_task.sdf
│   ├── perception_task.sdf
│   └── ... (其他SDF世界文件)
├── package.xml                     # 包依赖（已修改）
└── setup.py                        # 构建配置（已修改）
```

## 主要修改内容

### 1. 路径引用修正

**wamv_base.urdf.xacro** 中的关键修改：
```xml
<!-- 修改前 -->
<mesh filename="package://wamv_description/models/WAM-V-Base/mesh/M5_body.dae"/>
<albedo_map>model://wamv_description/models/WAM-V-Base/mesh/WAM-V_Albedo.png</albedo_map>

<!-- 修改后 -->
<mesh filename="package://usv_sim_full/description/models/WAM-V-Base/mesh/M5_body.dae"/>
<albedo_map>model://usv_sim_full/description/models/WAM-V-Base/mesh/WAM-V_Albedo.png</albedo_map>
```

### 2. 构建配置更新

**setup.py** 添加了新目录的安装配置：
```python
(os.path.join('share', package_name, 'description'), glob('description/**/*', recursive=True)),
(os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
```

**package.xml** 移除了外部依赖：
```xml
<!-- 移除了这些依赖 -->
<!-- <depend>vrx_gz</depend> -->
<!-- <depend>wamv_gazebo</depend> -->
<!-- <depend>wamv_description</depend> -->
```

### 3. Launch文件适配

**infra_sim.launch.py** 中环境变量设置的修改：
```python
# 修改前
wamv_path = get_package_share_directory('wamv_description')
wamv_models_path = os.path.join(wamv_path, 'models')

# 修改后  
usv_sim_path = get_package_share_directory('usv_sim_full')
usv_models_path = os.path.join(usv_sim_path, 'description')
```

### 4. 脚本逻辑更新

**session_manager.py** 中xacro文件查找逻辑：
```python
# 修改后优先使用本地description目录
usv_sim_path = get_package_share_directory('usv_sim_full')
xacro_input = os.path.join(usv_sim_path, 'description', 'urdf', xacro_template)
```

## 验证要点

✅ 所有xacro文件中的`package://wamv_description/`引用已替换为`package://usv_sim_full/description/`

✅ 所有`model://wamv_description/`引用已替换为`model://usv_sim_full/description/`

✅ GZ_SIM_RESOURCE_PATH环境变量正确指向本地description目录

✅ setup.py包含了新目录的安装配置

✅ package.xml移除了对vrx_gz和wamv_description的依赖

✅ 原src/vrx目录已删除

## 下一步建议

1. 编译测试：`colcon build --packages-select usv_sim_full`
2. 运行仿真测试基本功能
3. 验证传感器和桥接功能正常
4. 测试不同的世界场景加载