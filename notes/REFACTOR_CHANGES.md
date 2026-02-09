# USV仿真系统V2.0重构变更记录

## 目录结构调整

### 新增目录
```
src/usv_sim_full/
├── description/                    # 新增：本地化资源目录
│   ├── models/                     # 迁移自 wamv_description/models/
│   └── urdf/                       # 迁移自 wamv_description/urdf/
└── worlds/                         # 新增：本地化世界文件目录
```

### 删除目录
```
src/vrx/                           # 已删除整个VRX目录
```

## 文件修改详情

### 1. Xacro文件路径修正

#### wamv_base.urdf.xacro
```diff
- <mesh filename="package://wamv_description/models/WAM-V-Base/mesh/M5_body.dae"/>
+ <mesh filename="package://usv_sim_full/description/models/WAM-V-Base/mesh/M5_body.dae"/>

- <albedo_map>model://wamv_description/models/WAM-V-Base/mesh/WAM-V_Albedo.png</albedo_map>
+ <albedo_map>model://usv_sim_full/description/models/WAM-V-Base/mesh/WAM-V_Albedo.png</albedo_map>
```

#### battery.xacro
```diff
- <mesh filename="package://vrx_gazebo/models/battery/mesh/battery.dae"/>
+ <mesh filename="package://usv_sim_full/description/models/battery/mesh/battery.dae"/>

- <albedo_map>model://vrx_gazebo/models/battery/mesh/battery_Albedo.png</albedo_map>
+ <albedo_map>model://usv_sim_full/description/models/battery/mesh/battery_Albedo.png</albedo_map>
```

#### thrusters/engine.xacro
```diff
- <mesh filename="package://wamv_description/models/engine/mesh/engine.dae"/>
+ <mesh filename="package://usv_sim_full/description/models/engine/mesh/engine.dae"/>

- <mesh filename="package://wamv_description/models/propeller/mesh/propeller.dae"/>
+ <mesh filename="package://usv_sim_full/description/models/propeller/mesh/propeller.dae"/>
```

#### cpu_cases.xacro
```diff
- <mesh filename="package://vrx_gazebo/models/cpu_cases/mesh/cpu_cases.dae"/>
+ <mesh filename="package://usv_sim_full/description/models/cpu_cases/mesh/cpu_cases.dae"/>

- <albedo_map>model://vrx_gazebo/models/cpu_cases/mesh/cpu_cases_Albedo.png</albedo_map>
+ <albedo_map>model://usv_sim_full/description/models/cpu_cases/mesh/cpu_cases_Albedo.png</albedo_map>
```

### 2. 构建配置更新

#### setup.py
```diff
+ (os.path.join('share', package_name, 'description'), glob('description/**/*', recursive=True)),
+ (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
```

#### package.xml
```diff
- <depend>vrx_gz</depend>
- <depend>wamv_gazebo</depend>
- <depend>wamv_description</depend>
```

### 3. Launch文件修改

#### infra_sim.launch.py
```diff
- wamv_path = get_package_share_directory('wamv_description')
- wamv_models_path = os.path.join(wamv_path, 'models')
+ usv_sim_path = get_package_share_directory('usv_sim_full')
+ usv_models_path = os.path.join(usv_sim_path, 'description')

- vrx_gz_path = get_package_share_directory('vrx_gz')
- world_file = os.path.join(vrx_gz_path, "worlds", "sydney_regatta.sdf")
+ world_file = os.path.join(usv_sim_path, "worlds", "sydney_regatta.sdf")
```

### 4. 脚本逻辑更新

#### session_manager.py
```diff
else:
+     # 使用本地description目录中的xacro文件
+     usv_sim_path = get_package_share_directory('usv_sim_full')
+     xacro_input = os.path.join(usv_sim_path, 'description', 'urdf', xacro_template)
+     if not os.path.exists(xacro_input):
+         # 回退到原来的查找方式
```

## 资源迁移清单

### 模型文件迁移
- `src/vrx/vrx_urdf/wamv_description/models/` → `src/usv_sim_full/description/models/`
- 包含：WAM-V-Base, engine, propeller 三个模型

### URDF文件迁移  
- `src/vrx/vrx_urdf/wamv_description/urdf/` → `src/usv_sim_full/description/urdf/`
- 包含：battery.xacro, cpu_cases.xacro, wamv_base.urdf.xacro, thrusters/engine.xacro

### 世界文件迁移
- `src/vrx/vrx_gz/worlds/` → `src/usv_sim_full/worlds/`
- 包含：所有SDF世界文件

## 验证检查点

- [x] 所有xacro文件路径引用已更新
- [x] 所有mesh文件路径引用已更新  
- [x] 构建配置已更新
- [x] 依赖声明已清理
- [x] Launch文件环境变量已修正
- [x] 脚本查找逻辑已适配
- [x] 原VRX目录已删除
- [ ] 编译测试通过
- [ ] 基本仿真功能验证
- [ ] 传感器桥接功能验证