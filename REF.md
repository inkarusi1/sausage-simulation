# PositionBasedDynamics 项目参考

本文档记录本项目中软体、刚体、碰撞、摩擦、角速度、杠杆、接触冲量、布料、重力等仿真效果的实现位置与使用方法，便于以后修改或设计场景时快速查阅。内容以源码和 `data/scenes` 场景文件为准；如果示例 JSON、`doc/file_format.md` 与源码参数名不一致，优先相信源码注册的参数名。

## 运行入口

主要场景通过 `bin/SceneLoaderDemo.exe` 加载 JSON：

| bat | 实际加载的场景 |
| --- | --- |
| `bin/ArmadilloCollisionScene.bat` | `../data/scenes/ArmadilloCollisionScene.json` |
| `bin/ClothCollisionScene.bat` | `../data/scenes/ClothCollisionScene.json` |
| `bin/DeformableSolidCollisionScene.bat` | `../data/scenes/DeformableSolidCollisionScene.json` |
| `bin/PileScene.bat` | `../data/scenes/PileScene.json` |
| `bin/CarScene.bat` | `../data/scenes/CarScene.json` |
| `bin/CarScene_light.bat` | `../data/scenes/CarScene_light.json` |

`SceneLoaderDemo` 的核心流程在 `Demos/SceneLoaderDemo/SceneLoaderDemo.cpp`：

1. 初始化 `SimulationModel`。
2. 创建 `CubicSDFCollisionDetection`。
3. 调用 `buildModel()`。
4. 通过 `SceneLoader::readScene(true)` 读取 JSON。
5. 创建刚体、三角网格布料、四面体软体、关节、马达、弹簧/阻尼器和碰撞对象。

读取参数的通用逻辑在 `Utils/SceneLoader.cpp` 的 `readParameterObject()`。它会按 `ParameterObject` 中注册的参数名精确匹配 JSON 字段，因此字段名写错通常不会报错，只会保留默认值。

## 仿真步进流程

主要步进逻辑位于 `Simulation/TimeStepController.cpp`，相关积分在 `PositionBasedDynamics/TimeIntegration.cpp`：

1. `TimeStep::clearAccelerations()` 为动态刚体和粒子写入重力加速度。
2. 每个 `subSteps` 中，对刚体位置、粒子位置、刚体旋转做半隐式 Euler 积分。
3. 迭代求解位置约束，次数由 `maxIterations` 控制。
4. 根据位置差更新速度，`velocityUpdateMethod` 控制一阶或二阶更新。
5. 更新刚体网格变换。
6. 碰撞检测生成接触约束。
7. 迭代求解速度约束和接触冲量，次数由 `maxIterationsV` 控制。
8. 更新马达目标序列并推进全局时间。

这个顺序很重要：碰撞检测是在位置更新后进行的，接触和摩擦主要在速度约束阶段影响刚体速度、角速度和粒子速度。

## JSON 参数名注意事项

这些字段名容易踩坑，修改场景时建议优先使用下面的源码参数名：

| 目标 | 推荐字段名 | 常见旧字段或误写 | 说明 |
| --- | --- | --- | --- |
| 时间步长 | `timeStepSize` | 无 | `TimeManager` 参数。 |
| 子步数 | `subSteps` | 无 | `TimeStepController` 参数。 |
| 位置约束迭代 | `maxIterations` | `maxIter` | `maxIter` 在当前通用读取逻辑中可能被忽略。 |
| 速度约束迭代 | `maxIterationsV` | `maxIterVel` | `maxIterVel` 在当前通用读取逻辑中可能被忽略。 |
| 速度更新方式 | `velocityUpdateMethod` | 无 | 0 为一阶，1 为二阶。 |
| 重力 | `gravitation` | `gravity` | `Simulation` 注册的是 `gravitation`。`SceneLoader` 会读取 `gravity` 到 `SceneData`，但 `SceneLoaderDemo` 不会自动把它写入全局重力。 |
| 软体方法 | `solidSimulationMethod` | `tetModelSimulationMethod` | `SimulationModel` 注册的是 `solidSimulationMethod`。 |
| 布料方法 | `clothSimulationMethod` | 无 | 控制三角模型的布料求解方式。 |
| 接触容差 | `contactTolerance` | 无 | 在 `CollisionDetection` 中使用。 |

建议：新写 JSON 时同时保留 `gravity` 兼容自定义代码也可以，但必须写 `gravitation` 才能保证 `SceneLoaderDemo` 中生效。

## 碰撞对象类型

类型枚举来自 `Utils/SceneLoader.h` 中的 `CollisionObjectTypes`：

| `collisionObjectType` | 类型 | 用途 |
| --- | --- | --- |
| 0 | None | 不创建碰撞形状。 |
| 1 | Sphere | 球体碰撞。 |
| 2 | Box | 盒体碰撞，常用于地面或简单刚体。 |
| 3 | Cylinder | 圆柱碰撞。 |
| 4 | Torus | 圆环碰撞。 |
| 5 | SDF | 使用有符号距离场，适合复杂网格。 |
| 6 | HollowSphere | 空心球。 |
| 7 | HollowBox | 空心盒。 |

SDF 碰撞由 `Simulation/CubicSDFCollisionDetection.cpp` 和 `Simulation/DistanceFieldCollisionDetection.cpp` 参与实现。复杂网格通常设置 `collisionObjectType: 5`，并使用 `resolutionSDF` 控制距离场分辨率。分辨率越高，碰撞轮廓越细，但预处理和查询成本也更高。

## 自定义形状可碰撞物体

从零构建一个自定义形状时，先决定它在仿真中属于哪一类：

| 目标 | JSON 列表 | 模型文件 | 碰撞方式 |
| --- | --- | --- | --- |
| 自定义动态/静态刚体 | `RigidBodies` | `geometryFile`，通常是 `.obj` | 简单形状或 SDF。 |
| 自定义软体实体 | `TetModels` | `nodeFile`、`eleFile`、`visFile` | 通常用 `visFile` 生成 SDF。 |
| 自定义布料/薄片 | `TriangleModels` | `geometryFile`，三角网格 `.obj` | 粒子/三角模型接触，不按刚体 SDF 处理。 |

### 自定义刚体网格

最常用做法是把自定义 `.obj` 放到 `data/models`，然后在场景 JSON 的 `RigidBodies` 里引用它。复杂形状建议使用 `collisionObjectType: 5`，也就是 SDF。

最小模板：

```json
"RigidBodies": [
  {
    "id": 100,
    "geometryFile": "../models/my_shape.obj",
    "isDynamic": 1,
    "density": 300,
    "translation": [0, 5, 0],
    "rotationAxis": [0, 1, 0],
    "rotationAngle": 0.0,
    "scale": [1, 1, 1],
    "velocity": [0, 0, 0],
    "angularVelocity": [0, 0, 0],
    "restitution": 0.2,
    "friction": 0.3,
    "collisionObjectType": 5,
    "collisionObjectFileName": "",
    "collisionObjectScale": [1, 1, 1],
    "resolutionSDF": [30, 30, 30],
    "testMesh": 1
  }
]
```

关键点：

- `geometryFile` 是渲染网格，也会用于质量/惯性计算。
- `collisionObjectType: 5` 表示使用 SDF 碰撞，适合任意外形。
- `collisionObjectFileName: ""` 时，当前 `SceneLoaderDemo` 会从 `geometryFile` 自动生成 SDF，并缓存到 `data/scenes/Cache`。
- `collisionObjectFileName` 非空时，源码当前是直接加载 `.csdf` 距离场文件，例如 `../sdf/bunny_10k.csdf`。不要在这里填普通 `.obj`，除非同时改加载代码。
- `collisionObjectScale` 应该和碰撞形状的缩放一致，通常与 `scale` 相同。
- `resolutionSDF` 越高，轮廓越准确，首次生成越慢，缓存文件越大。
- `testMesh: 1` 表示用该物体网格顶点参与和其他 SDF 的碰撞测试，通常保持开启。

如果只是近似碰撞，不必上 SDF。可以用简单碰撞体当 proxy：

```json
{
  "id": 101,
  "geometryFile": "../models/my_visual_mesh.obj",
  "isDynamic": 1,
  "density": 300,
  "translation": [0, 3, 0],
  "rotationAxis": [1, 0, 0],
  "rotationAngle": 0.0,
  "scale": [1, 1, 1],
  "collisionObjectType": 2,
  "collisionObjectScale": [1.5, 0.8, 2.0],
  "friction": 0.4,
  "restitution": 0.1
}
```

这种方式渲染仍用 `geometryFile`，碰撞近似为 box、sphere、cylinder 或 torus。优点是快且稳定，缺点是轮廓不精确。

### 自定义软体实体

软体不是只放一个 `.obj` 就能模拟体积变形；它需要四面体网格：

- `nodeFile`：节点位置。
- `eleFile`：四面体单元。
- `visFile`：可视化表面网格，通常也是 SDF 的来源。

最小模板：

```json
"TetModels": [
  {
    "id": 0,
    "nodeFile": "../models/my_soft_body.node",
    "eleFile": "../models/my_soft_body.ele",
    "visFile": "../models/my_soft_body.obj",
    "translation": [0, 6, 0],
    "rotationAxis": [1, 0, 0],
    "rotationAngle": 0.0,
    "scale": [1, 1, 1],
    "staticParticles": [],
    "restitution": 0.1,
    "friction": 0.2,
    "collisionObjectType": 5,
    "collisionObjectFileName": "",
    "collisionObjectScale": [1, 1, 1],
    "resolutionSDF": [30, 30, 30],
    "testMesh": 1
  }
]
```

关键点：

- 四面体软体的真实仿真自由度来自 `.node` 和 `.ele`，`visFile` 主要用于显示和 SDF。
- 固定软体某些点时，把粒子索引放进 `staticParticles`。
- `solidSimulationMethod`、体积/应变相关刚度和迭代次数决定软硬程度。
- 如果只有表面 `.obj`，需要先用外部工具生成四面体网格，再放进 `TetModels`。

### 自定义布料

布料使用 `TriangleModels`，模型应该是三角网格，常见是规则平面或衣料表面：

```json
"TriangleModels": [
  {
    "id": 0,
    "geometryFile": "../models/my_cloth.obj",
    "translation": [0, 8, 0],
    "rotationAxis": [1, 0, 0],
    "rotationAngle": 0.0,
    "scale": [1, 1, 1],
    "staticParticles": [0, 10],
    "restitution": 0.1,
    "friction": 0.2
  }
]
```

关键点：

- `staticParticles` 使用的是网格顶点索引，改模型后必须重新确认索引。
- 布料碰撞来自粒子/三角模型，不需要给布料写 `collisionObjectType: 5`。
- 布料太容易穿透时，优先增加 `subSteps`、`maxIterations`、`maxIterationsV`，并减小 `timeStepSize`。

### 自定义形状资源检查

导入新模型前建议检查：

- 模型路径相对场景 JSON 所在目录，例如 `data/scenes/*.json` 中使用 `../models/name.obj`。
- 模型尺寸不要过大或过小，最好先用已有 cube/sphere 对照比例。
- 用 SDF 的网格最好是闭合表面，否则内外侧和质量/惯性可能异常。
- 模型原点会影响旋转和杠杆效应；偏离质心太远时，旋转表现可能很怪。
- 法线、面片朝向和非流形边可能影响 SDF 生成。
- 第一次自动生成 SDF 会比较慢，成功后检查 `data/scenes/Cache` 是否出现 `.csdf` 缓存。
- 若修改了 `.obj` 但效果没变，清理对应缓存或提高 `resolutionSDF` 后重试。

## 四个重点场景

### ArmadilloCollisionScene

文件：`data/scenes/ArmadilloCollisionScene.json`

特点：

- 包含 3 个 `TetModels`，模型为 armadillo 四面体软体。
- 软体使用 SDF 碰撞，`collisionObjectType: 5`，并设置 `resolutionSDF: [20,20,20]`。
- 包含一个静态地面盒体，`collisionObjectType: 2`。
- 主要用于观察四面体软体之间，以及软体和地面之间的接触、回弹和摩擦。

适合参考：

- 软体与软体、软体与静态刚体的碰撞配置。
- 四面体模型 `TetModels` 的路径、缩放、平移、旋转和碰撞参数。
- 复杂网格使用 SDF 的最低可用配置。

### ClothCollisionScene

文件：`data/scenes/ClothCollisionScene.json`

特点：

- 包含一个 `TriangleModels` 布料，来自 `plane_50x50.obj`。
- 布料整体放大和平移，常见配置为 `scale: [10,10,10]`、`translation: [5,8,0]`。
- 固定粒子通常设置为角点，例如 `[0,49]`，用于形成悬挂布料。
- 场景中有静态地面、动态球体、动态圆柱和动态圆环。
- 圆柱设置了初始 `angularVelocity: [1,0,0]`，可观察旋转刚体对布料接触的影响。
- 该场景的地面摩擦为 `0.0`，动态球/圆柱/圆环摩擦通常为 `0.2`、回弹为 `0.6`；也就是说，原仓库示例不会靠很大的摩擦来产生翻转，而是主要靠接触冲量、偏心接触点、惯性张量和角速度积分。

适合参考：

- 三角布料的生成、固定点、弯曲约束和拉伸约束。
- 粒子和刚体碰撞。
- 运动刚体、旋转刚体和布料的摩擦耦合。
- 布料接触复杂形状时的穿透、滑动、堆叠表现。

### DeformableSolidCollisionScene

文件：`data/scenes/DeformableSolidCollisionScene.json`

特点：

- 包含一个四面体 armadillo 软体。
- 包含静态地面和动态球体、圆柱、圆环。
- 圆柱同样配置了初始角速度。
- 主要用于观察可变形四面体软体和刚体碰撞。

适合参考：

- `TetModels` 与 `RigidBodies` 的混合场景。
- 可变形实体被刚体撞击、压迫、滚动接触时的响应。
- 初始角速度对接触冲量、切向摩擦和软体变形的影响。

### PileScene

文件：`data/scenes/PileScene.json`

特点：

- 包含一个静态地面和多根静态圆柱桩。
- 包含多个动态复杂刚体，常见为 bunny、armadillo 等网格，并使用 SDF 碰撞。
- 刚体之间会在重力下堆叠、碰撞、滑动和反弹。
- 场景中 `restitution` 较高时能看到更明显的弹跳，`friction` 较低时滑动更明显。

适合参考：

- 大量刚体碰撞和堆叠。
- 静态障碍物阵列。
- 复杂网格刚体的 SDF 碰撞。
- 接触冲量在多体堆叠中的稳定性。

## 其他值得参考的场景和 Demo

### CarScene / CarScene_light

文件：`data/scenes/CarScene.json`、`data/scenes/CarScene_light.json`、`data/scenes/CarScene.py`

特点：

- 多刚体车辆结构。
- 使用铰链、马达、阻尼器模拟转向、轮子转动和悬挂。
- 适合参考刚体关节、目标角度马达、目标速度马达和阻尼器。
- 场景中可能写有 `gravity`，如果希望在 `SceneLoaderDemo` 中确定生效，建议补写 `gravitation`。

### ClothOnBunny

文件：`data/scenes/ClothOnBunny.json`

特点：

- 布料覆盖复杂 SDF 网格。
- 适合参考布料和复杂静态或动态刚体的接触。

### Wilberforce_scene

文件：`data/scenes/Wilberforce_scene.json`

特点：

- 包含大量刚体和 `CosseratJoints`。
- 主要由 `Demos/StiffRodsDemos` 下的刚性杆专用加载器和求解器使用，不是普通 `SceneLoaderDemo` 的典型场景。
- 适合参考树状杆、Cosserat joint、Wilberforce pendulum 这类杆件结构。

### 其他 Demo 目录

| 目录 | 参考价值 |
| --- | --- |
| `Demos/FluidDemo` | 流体仿真、粒子流体相关流程。 |
| `Demos/PositionBasedElasticRodsDemo` | 弹性杆。 |
| `Demos/StiffRodsDemos` | 刚性杆、Cosserat 约束、专用场景加载。 |
| `Demos/GenericConstraintsDemos` | 通用约束、关节、马达示例。 |
| `Demos/SausageDemo`、`SausageCourseDemo`、`SausagePhysicsCourseDemo` | 教学和调试性质的约束/物理示例。 |

## 关键物理效果实现位置

### 重力

源码位置：

- `Simulation/Simulation.cpp`
- `Simulation/TimeStep.cpp`

实现要点：

- 全局重力参数名为 `gravitation`，默认值通常为 `[0,-9.81,0]`。
- `TimeStep::clearAccelerations()` 每帧把重力写入动态刚体和粒子的加速度。
- 质量为 0 的静态刚体不会被重力积分。
- 布料粒子和四面体软体粒子也会受到重力，除非粒子被固定或质量为 0。

修改建议：

- 场景级改重力：在 JSON 的 `Simulation` 中写 `gravitation`。
- 局部无重力或弱重力：可以改粒子质量、固定点，或在自定义 Demo 中覆盖加速度。

### 刚体、质量和惯性

源码位置：

- `Simulation/RigidBody.h`
- `Utils/VolumeIntegration.cpp`
- `PositionBasedDynamics/TimeIntegration.cpp`

实现要点：

- 刚体包含位置、旋转四元数、速度、角速度、质量、惯性张量等状态。
- 若 JSON 中设置 `density`，网格体积积分会计算质量和惯性。
- `isDynamic: false` 或质量为 0 的刚体作为静态物体处理。
- `angularVelocity` 可在 JSON 中设置初始角速度。
- 旋转积分通过角速度更新四元数，接触冲量也会改变角速度。

修改建议：

- 要让物体更难被撞动：提高密度或质量，或提高惯性。
- 要让物体明显旋转：设置初始 `angularVelocity`，或让接触点远离质心以产生更大力矩。
- 要做固定地面或墙：使用静态刚体，并给它简单碰撞体，例如 box。

### 角速度和杠杆效应

源码位置：

- `PositionBasedDynamics/PositionBasedRigidBodyDynamics.cpp`
- `PositionBasedDynamics/TimeIntegration.cpp`

实现要点：

- 接触冲量不只改变线速度，也会通过接触点相对质心的偏移 `r` 改变角速度。
- 典型形式是使用 `r.cross(p)` 计算力矩方向，其中 `p` 是接触冲量。
- 接触点离质心越远，杠杆臂越大，越容易产生明显旋转。
- 圆柱、圆环等带初始角速度的刚体在接触布料或软体时，会通过切向相对速度触发摩擦响应。

修改建议：

- 想增强“被撞后旋转”的效果：让碰撞发生在偏心位置，降低角惯性，或提高接触冲量/回弹。
- 想减少旋转：提高惯性，降低回弹，增加阻尼，或让碰撞更接近质心。

### 接触冲量、回弹和摩擦

源码位置：

- `Simulation/CollisionDetection.cpp`
- `Simulation/DistanceFieldCollisionDetection.cpp`
- `Simulation/Constraints.cpp`
- `PositionBasedDynamics/PositionBasedRigidBodyDynamics.cpp`

实现要点：

- 碰撞检测阶段生成接触约束。
- 速度求解阶段计算法向接触冲量，阻止继续穿透，并按 `restitution` 引入回弹。
- 如果接触深度 `d < 0`，求解器会加入惩罚性质的修正冲量。
- 切向方向会计算动态摩擦冲量，影响滑动和滚动。
- 刚体接触会同时修正 `v` 和 `angularVelocity`。
- 粒子、刚体、四面体之间有不同的 contact constraint 包装，统一进入约束求解。

系数组合规则需要注意：

- 在 `DistanceFieldCollisionDetection` 中，两个物体的回弹系数常按乘积组合。
- 摩擦系数常按和组合。
- 因此单个物体把 `friction` 调大，接触对的摩擦会明显增强；把某个物体 `restitution` 设小，整体回弹也会被压低。

修改建议：

- 弹跳更强：提高双方 `restitution`，同时提高 `maxIterationsV` 保证速度约束求解充分。
- 滑动更少：提高 `friction`。
- 接触更稳定：适当增加 `subSteps`、`maxIterations`、`maxIterationsV`，减小 `timeStepSize`。
- 复杂网格穿透明显：提高 SDF 分辨率或简化碰撞形状，检查缩放和模型原点。

### 碰撞检测和 SDF

源码位置：

- `Simulation/CollisionDetection.cpp`
- `Simulation/DistanceFieldCollisionDetection.cpp`
- `Simulation/CubicSDFCollisionDetection.cpp`
- `Simulation/Discregrid/*`

实现要点：

- 简单形状可直接使用 sphere、box、cylinder、torus。
- 复杂网格使用 SDF 碰撞，距离场用于查询穿透深度和法线。
- `resolutionSDF` 控制 SDF 栅格分辨率。
- 碰撞检测会创建粒子-刚体、刚体-刚体、粒子-四面体等不同类型接触。

修改建议：

- 复杂物体如果只是当障碍物，优先考虑简单 proxy 碰撞体，性能和稳定性更好。
- 如果必须保留复杂轮廓，使用 SDF，并检查 `scale`、`translation`、`rotationAxis`、`rotationAngle`。
- 动态 SDF 刚体数量较多时，优先降低网格复杂度或 SDF 分辨率。

### 布料

源码位置：

- `Simulation/SimulationModel.cpp`
- `Simulation/TriangleModel.*`
- `PositionBasedDynamics/PositionBasedDynamics.*`
- `Demos/SceneLoaderDemo/SceneLoaderDemo.cpp`

实现要点：

- 布料通常来自 `TriangleModels`。
- 顶点作为粒子参与积分和碰撞。
- 三角边和弯曲关系生成距离、FEM 或弯曲约束，具体方法由 `clothSimulationMethod` 和相关刚度参数控制。
- `staticParticles` 或固定粒子数组可让布料某些顶点质量为 0，从而形成挂点。
- 布料与刚体接触时，本质是粒子-刚体接触约束。

修改建议：

- 布料太软：提高拉伸/弯曲刚度，增加位置迭代。
- 布料抖动或穿透：减小时间步，增加 `subSteps` 和速度约束迭代，合理提高 `contactTolerance`。
- 挂布或旗帜：固定上边或角点粒子。

### 四面体软体

源码位置：

- `Simulation/SimulationModel.cpp`
- `Simulation/TetModel.*`
- `Simulation/Constraints.cpp`
- `PositionBasedDynamics/PositionBasedDynamics.*`

实现要点：

- 软体实体来自 `TetModels`。
- 四面体网格的粒子参与积分、碰撞和体积/应变约束。
- 仿真方法由 `solidSimulationMethod` 控制。
- SDF 碰撞可用于软体外表面和刚体/地面的接触。

修改建议：

- 软体太塌：提高体积/应变相关刚度，增加 `maxIterations`。
- 软体太硬：降低刚度或迭代次数。
- 与刚体互动不明显：调整密度、摩擦、回弹和重力。

### 静态物体和固定粒子

实现要点：

- 静态刚体通常设置 `isDynamic: false` 或质量为 0。
- 固定布料/软体粒子通常通过 `staticParticles` 或相关固定粒子配置实现。
- 静态物体仍可参与碰撞，但不被接触冲量推动。

修改建议：

- 地面、墙、桩、支架使用静态刚体。
- 悬挂布料、固定软体端点使用固定粒子。
- 如果一个物体意外不动，优先检查质量、密度、`isDynamic` 和固定粒子配置。

### 关节、马达、弹簧和阻尼器

源码位置：

- `Simulation/SimulationModel.cpp`
- `Simulation/Constraints.cpp`
- `Utils/SceneLoader.cpp`
- `data/scenes/SceneGenerator.py`
- `data/scenes/CarScene.py`

实现要点：

- 刚体连接通过 ball joint、hinge joint、slider joint、universal joint 等约束实现。
- 马达可控制目标角度或目标速度。
- 弹簧和阻尼器可模拟悬挂、连接和缓冲。
- `CarScene` 是车辆关节和马达最直接的参考场景。

修改建议：

- 做机械结构时，先从 `CarScene.py` 复制生成逻辑，再逐步改关节坐标和轴向。
- 目标速度马达适合轮子，目标角度马达适合转向。
- 关节抖动时，优先检查锚点是否在两个刚体的局部坐标中一致，再调迭代次数和刚度。

## SceneGenerator.py 使用建议

文件：`data/scenes/SceneGenerator.py`

它适合用 Python 批量生成场景，尤其是重复刚体、车辆、堆叠、关节结构。优点是参数一致、可复现，比手改大型 JSON 更安全。

使用建议：

- 参考 `PileScene.py` 生成大量刚体。
- 参考 `CarScene.py` 生成车辆、多关节和马达。
- 生成后检查 `Simulation` 字段名，确保使用 `maxIterations`、`maxIterationsV`、`gravitation`、`solidSimulationMethod`。
- 注意个别生成函数的字段名可能与 `SceneLoader` 期望不完全一致，例如某些 rigid-body-particle joint 字段需要核对 `SceneLoader::readRigidBodyParticleBallJoints()`。

## 修改和设计场景的实用清单

### 新建刚体碰撞场景

1. 在 `RigidBodies` 中定义模型路径、平移、旋转、缩放、密度或质量。
2. 给每个需要碰撞的物体设置 `collisionObjectType`。
3. 复杂网格使用 SDF，并设置合理的 `resolutionSDF`。
4. 地面或墙设置为静态刚体。
5. 在 `Simulation` 中设置 `gravitation`、`timeStepSize`、`subSteps`、`maxIterations`、`maxIterationsV`。

### 新建布料场景

1. 在 `TriangleModels` 中设置布料网格。
2. 设置 `staticParticles` 固定挂点。
3. 设置布料方法、拉伸/弯曲刚度。
4. 加入静态或动态碰撞刚体。
5. 若布料穿透，优先调小时间步并增加子步和速度迭代。

### 新建软体场景

1. 在 `TetModels` 中设置四面体网格。
2. 选择 `solidSimulationMethod`。
3. 设置软体刚度、泊松比、杨氏模量等相关参数。
4. 给软体和刚体都配置碰撞体。
5. 通过质量、密度、摩擦、回弹控制碰撞表现。

### 调整效果的快速方向

| 目标效果 | 优先调整 |
| --- | --- |
| 更强重力 | `gravitation` 的 y 分量更负。 |
| 更慢动作 | 减小重力或增大质量/阻尼，或调小时间步并录制更多帧。 |
| 更硬布料/软体 | 增加刚度和 `maxIterations`。 |
| 更软布料/软体 | 降低刚度或迭代次数。 |
| 更弹 | 提高 `restitution`，增加 `maxIterationsV`。 |
| 更粘/更不滑 | 提高 `friction`。 |
| 更稳定 | 增加 `subSteps`，减小 `timeStepSize`。 |
| 更明显旋转 | 偏心碰撞、降低惯性、设置初始 `angularVelocity`。 |
| 更少穿透 | 提高接触容差/迭代，增加 SDF 分辨率，减小时间步。 |

## 从零构建物理仿真的缺漏补充

如果目标是从空白文件开始设计一个新场景，原来的文档还缺少“完整落地顺序”。建议按下面流程做。

### 最小场景骨架

一个可运行场景至少需要 `Name`、`Simulation` 和一个或多个模型列表。下面是新建刚体场景时的骨架：

```json
{
  "Name": "MyScene",
  "Simulation": {
    "timeStepSize": 0.005,
    "subSteps": 2,
    "maxIterations": 5,
    "maxIterationsV": 5,
    "velocityUpdateMethod": 0,
    "gravitation": [0, -9.81, 0],
    "contactTolerance": 0.05
  },
  "RigidBodies": [],
  "TriangleModels": [],
  "TetModels": []
}
```

注意：已有示例里可能写 `maxIterVel` 或 `tetModelSimulationMethod`，但从零写新场景时建议使用 `maxIterationsV` 和 `solidSimulationMethod`。

### 推荐搭建顺序

1. 先只放静态地面和一个动态 cube/sphere，确认运行入口、路径和重力都正常。
2. 再把 cube/sphere 替换成自己的 `geometryFile`，先用 box 或 sphere proxy 碰撞。
3. 复杂轮廓需要后，再切换到 `collisionObjectType: 5` 的 SDF。
4. 确认刚体能稳定碰撞后，再加入布料、软体、关节、马达或多物体堆叠。
5. 每增加一种物理效果，只改一两个参数并观察结果，避免同时改刚度、质量、摩擦、时间步导致无法判断问题来源。

### 新模型导入前的必查项

| 检查项 | 原因 |
| --- | --- |
| 路径是否相对 `data/scenes` 正确 | bat 从 `bin` 运行，但 JSON 内部路径按场景文件位置解析。 |
| 模型是否在合理尺度 | 尺寸极端会让重力、接触容差、SDF 分辨率都变得难调。 |
| 质心/原点是否合理 | 原点偏移会影响旋转、惯性和杠杆效果。 |
| 网格是否闭合 | SDF、体积、惯性对开放网格更容易出异常。 |
| 是否需要碰撞 proxy | 复杂视觉网格不一定适合直接碰撞。 |
| `scale` 和 `collisionObjectScale` 是否一致 | 不一致会导致看到的位置和碰撞轮廓对不上。 |
| 动态属性是否正确 | `isDynamic: 0` 或质量为 0 的物体不会被推动。 |
| SDF 缓存是否过期 | 改模型后旧 `.csdf` 可能让碰撞仍像旧形状。 |

### 调试顺序

遇到“物体不动、穿透、爆炸、旋转异常”时，按这个顺序排查：

1. 确认 `gravitation` 是否存在，物体是否 `isDynamic: 1`。
2. 确认 `collisionObjectType` 不是 0，`collisionObjectScale` 与显示模型匹配。
3. 对 SDF 物体，确认自动生成或加载的 `.csdf` 是否正确。
4. 把场景简化到一个动态物体和一个静态地面。
5. 减小 `timeStepSize`，增加 `subSteps`。
6. 增加 `maxIterations` 修位置约束，增加 `maxIterationsV` 修速度接触和摩擦。
7. 降低 `restitution`，避免反弹过强导致抖动。
8. 检查模型原点和缩放，尤其是自定义网格。

### 哪些事情 JSON 不够，需要改 C++ 源码

这些需求通常不是只改场景文件能完成：

- 新增一种碰撞体类型，例如胶囊、凸包或自定义 analytic SDF。
- 新增外力、风场、吸引力或区域力。
- 新增材质组合规则，例如摩擦不再相加、回弹不再相乘。
- 新增传感器、触发器、破坏、粘连、切割等事件逻辑。
- 新增场景字段并希望 `SceneLoaderDemo` 识别。

对应修改位置通常包括：

- `Utils/SceneLoader.h/.cpp`：新增 JSON 字段或枚举。
- `Demos/SceneLoaderDemo/SceneLoaderDemo.cpp`：把新字段真正构建成模型、约束或碰撞对象。
- `Simulation/SimulationModel.*`：新增模型、约束或全局参数。
- `Simulation/CollisionDetection.*` 和 `DistanceFieldCollisionDetection.cpp`：新增碰撞检测或接触生成。
- `PositionBasedDynamics/*`：新增约束求解、冲量或积分逻辑。

## 新增课程 demo 的可复用设计

本节记录课程作业新增 demo 中已经实现的可复用设计。它们不同于原仓库 `SceneLoaderDemo` 的 JSON 通用加载体系：这些设计主要在 `Demos/SausageRodCourseDemo` 和 `Demos/SausageRodEditorDemo` 中手写 C++ 场景与交互逻辑，用于在香肠课程场景里取得更低开销、更少穿模和暂停编辑能力。

### 中心线杆香肠与胶囊半径碰撞壳

文件：`Demos/SausageRodCourseDemo/main.cpp`、`Demos/SausageRodEditorDemo/main.cpp`

设计：

- 香肠不再用高分辨率四面体软体，也不是一串动态刚体球。
- 香肠主体用少量中心线点 `RodPoint` 表示。
- 相邻点和跨段点使用距离约束保持长度和弯曲刚度。
- 低柔软度时使用 shape matching 保持整体接近刚体。
- 碰撞时把静态障碍物 SDF 用 `sausageRadius + collisionSkin` 外扩，把中心线点和线段采样点投影到障碍物外侧。
- 渲染时根据中心线生成连续管状表面，避免早期粒子链/球链出现明显分段。

适合复用：

- 细长柔体、绳状物、管状软体、低开销演示级软体。
- 需要比 TetModel 更快、比刚体链更连续的课程 demo。

限制：

- 它不是原仓库完整的四面体软体求解器。
- 体积形变是杆约束近似，不适合展示真实三维实体应力分布。
- 接触响应是自定义 PBD 风格，不直接进入原仓库 `TimeStepController` 的刚体接触冲量管线。

### 静态复杂障碍物的程序网格 + SDF

文件：`Demos/SausageRodCourseDemo/main.cpp`、`Demos/SausageRodEditorDemo/main.cpp`

设计：

- 圆环和半管滑轨由 C++ 程序生成三角网格。
- 使用 `Discregrid::TriangleMeshDistance` 生成 `CubicSDFCollisionDetection::Grid`。
- 渲染网格和碰撞 SDF 来自同一套程序网格，减少视觉层与碰撞层不一致。
- 圆环、滑轨等障碍物作为静态 `RigidBody`，碰撞查询时使用其刚体 transform 做 world/local 坐标转换。

适合复用：

- 不想维护外部 `.obj` 文件、但需要自定义形状并支持碰撞的 demo。
- 视觉网格和碰撞形状必须一致的课程场景。

限制：

- SDF 网格首次生成有成本。
- 运行时直接缩放复杂 SDF 网格容易造成视觉与碰撞场不一致；建议重建网格/SDF，而不是只改可视 transform。

### 自定义接触响应中的摩擦和回弹

文件：`Demos/SausageRodCourseDemo/main.cpp`、`Demos/SausageRodEditorDemo/main.cpp`

设计：

- 碰撞投影阶段记录接触法线、摩擦系数和恢复系数。
- 摩擦/回弹仍从障碍物 `RigidBody::getFrictionCoeff()` 和 `getRestitutionCoeff()` 读取。
- 法线回弹使用碰撞前预测速度 `predictedV` 计算，避免只做“推出物体”导致完全没有反弹。
- 切向速度按摩擦系数阻尼，滑轨设置低摩擦，平台设置高摩擦。

适合复用：

- 自定义粒子/杆求解器没有直接接入原仓库接触冲量，但仍想复用刚体材质参数时。

限制：

- 这是自定义近似响应，不等价于原仓库速度约束阶段的完整接触冲量求解。
- 如果需要多刚体堆叠、角速度和力矩的严格效果，仍应优先使用原生刚体模型和 `TimeStepController`。

### 课程 demo 轻量动态物体的翻转/角速度补充

文件：

- 原仓库依据：`PositionBasedDynamics/PositionBasedRigidBodyDynamics.cpp`
- 原仓库旋转积分：`PositionBasedDynamics/TimeIntegration.cpp`
- 原仓库约束包装：`Simulation/Constraints.cpp`
- 课程落地位置：`Demos/SausageRodEditorDemo/main.cpp`

原仓库刚体接触不是只改中心速度。`RigidBodyContactConstraint` 和 `ParticleRigidBodyContactConstraint` 会把接触点相对质心的偏移记为 `r = contactPoint - centerOfMass`，接触点速度使用：

```cpp
u = v + omega.cross(r);
```

速度约束求解出冲量 `p` 后，线速度和角速度分别按下面方式修正：

```cpp
v += invMass * p;
omega += inertiaInverseW * r.cross(p);
```

这就是刚体被偏心撞击后会翻转/滚动的核心：接触点离质心越远，`r.cross(p)` 产生的角动量越明显。随后 `TimeIntegration::semiImplicitEulerRotation()` 用角速度积分四元数：

```cpp
rotation += h * 0.5 * (angularVelocityQuaternion * rotation);
rotation.normalize();
```

`SausageRodEditorDemo` 当前没有完整接入 `TimeStepController` 的原生刚体 step，所以编辑器新增动态物体使用轻量实现：仍由 demo 自己积分位置和重力，但碰撞时按上面的原仓库思路在接触采样点施加冲量，并同步更新 `EditableRigidBody::angularVelocity`。盒体使用角点/面中心采样，锥体和滑轨使用网格顶点抽样；因此盒体角点撞平台、圆环或滑轨边缘时会获得角速度，而不是只平移弹开。

实现注意：

- fixed 切 dynamic 时不能只把质量从 0 改成非 0；还要给物体设置与当前尺寸匹配的惯性张量，并刷新 `updateInertiaW()`。否则会出现“质量变轻但惯性仍像高密度物体”的状态，翻转会非常迟钝。
- 每次直接改 transform 或自己积分旋转后，都要同步 `rotationMatrix`、`updateInertiaW()`、`updateInverseTransformation()` 和渲染网格变换；否则碰撞查询、惯性方向和视觉朝向会逐渐不同步。
- 参考 `ClothCollisionScene`，演示级动态物体的摩擦不宜过大。摩擦过高会吃掉切向速度，让物体像被地面或障碍物拖住；自然翻滚主要来自偏心法向冲量，不应主要靠摩擦硬拽。
- 尖锐 SDF（例如数学尖锥）容易在粒子/杆采样接触中产生法线突变，使香肠被尖点挂住。课程 demo 中锥体应使用轻微平顶/圆钝化的程序网格，并降低该类编辑器 SDF 物体对香肠的摩擦。
- `CubicSDFCollisionDetection::CubicSDFCollisionObject` 自带 `m_scale`，距离查询时会把局部点除以该 scale，再把距离乘回 scale。因此对 SDF 物体做统一缩放时，可以同步缩放可视局部网格和 `m_scale`，不必每次重新生成距离场。非均匀缩放仍建议重建网格和 SDF。
- 编辑器状态保存不等同于上游 `SceneLoaderDemo` JSON。`SausageRodEditorDemo` 使用独立的轻量 JSON 保存当前香肠、可编辑刚体 transform、形状类型、缩放、fixed/dynamic、速度和角速度，加载时先重建课程基础场景，再按记录复原编辑状态。

限制：

- 这是课程 demo 内的近似刚体响应，不等价于原仓库完整刚体接触约束和多轮速度迭代。
- 多动态物体互碰仍是轻量级分离和速度响应，不适合替代稳定堆叠场景。
- 如果目标是严格的翻滚、堆叠、角动量守恒和多体接触，应继续接入 `Simulation::getCurrent()->getTimeStep()->step(*model)` 与 `setCollisionDetection(*model, cd)`，并处理 `rodSubStep()` 与 `TimeManager` 的时间推进统一。

### 墙钟时间节流

文件：`Demos/SausageRodCourseDemo/main.cpp`、`Demos/SausageRodEditorDemo/main.cpp`

设计：

- `MiniGL` 的 idle 回调可能按机器性能高频执行。
- 新 demo 使用 `std::chrono::steady_clock` 统计真实时间，把仿真小步放进 accumulator。
- `playbackSpeed` 控制展示播放倍率，避免程序在高性能机器上“快得离谱”。

适合复用：

- 不走原仓库 `TimeStepController::step()`、而是在 demo 内部手写步进循环的场景。

限制：

- 这控制的是展示速度，不改变物理世界中的重力常量。

### 暂停编辑对象表

文件：`Demos/SausageRodEditorDemo/main.cpp`

设计：

- 新增 `EditableRigidBody` 表，记录可编辑刚体的 `bodyIndex`、名称、box scale、是否可缩放、是否可删除、摩擦和回弹。
- 初始场景中的平台、两个圆环和半管滑轨会登记到编辑器表。
- 香肠作为特殊对象单独编辑，不走 `RigidBody` 表。
- 暂停时，左键和编辑快捷键才会处理；未暂停时尽量交回原 viewer/仿真逻辑。

适合复用：

- 只想给某个 demo 加局部编辑能力，而不是把编辑器做进整个 `DemoBase`。
- 需要明确区分“原始仿真模式”和“暂停编辑模式”的课程项目。

限制：

- 当前不是持久化场景编辑器，修改不会自动写回 JSON。
- 选择依据是近似的屏幕反投影最近对象，不是完整网格拾取。

### 暂停平移/旋转刚体的同步步骤

文件：`Demos/SausageRodEditorDemo/main.cpp` 中 `syncRigidBodyTransform()`

编辑静态刚体时不能只调用 `setPosition()` 或 `setRotation()`。新增 demo 中统一同步：

1. 当前、上一帧、旧位置。
2. 当前、上一帧、旧旋转。
3. 线速度和角速度清零。
4. rotation matrix。
5. `updateInverseTransformation()`，保证 SDF world/local 查询跟随刚体 transform。
6. `getGeometry().updateMeshTransformation(...)`，保证可视网格跟随 transform。

这个步骤尤其重要：原仓库 `RigidBody::rotationUpdated()` 对质量为 0 的静态刚体不会自动更新全部状态，因此暂停编辑静态 SDF 障碍物时需要显式同步。

### 运行时新增/删除盒体

文件：`Demos/SausageRodEditorDemo/main.cpp`

设计：

- `n` 新增一个静态 box 刚体。
- 新增时同时调用 `addBody(...)` 和 `addCollisionBox(...)`，保证可见且可碰撞。
- 新增盒体登记到 `EditableRigidBody` 表，支持移动、旋转、缩放和删除。
- `d` 删除只作用于编辑器新增盒体。

删除策略：

- 原仓库没有安全删除单个刚体并重映射所有碰撞对象/约束的通用 API。
- 当前 demo 的“删除”采用停用并移动到远处的方式，避免破坏原始刚体 vector 下标和碰撞对象引用。

缩放策略：

- 只对 box 类对象开放运行时缩放。
- 缩放时重新初始化该刚体的 cube 网格，并同步 `DistanceFieldCollisionBox::m_box`。
- 对圆环/滑轨这类复杂 SDF 网格不开放运行时缩放，应重建网格和 SDF。

## 源码参考地图

| 功能 | 主要文件 |
| --- | --- |
| 场景读取 | `Utils/SceneLoader.h`、`Utils/SceneLoader.cpp` |
| SceneLoaderDemo 构建流程 | `Demos/SceneLoaderDemo/SceneLoaderDemo.cpp` |
| 参数读取 | `Demos/Common/DemoBase.cpp`、`Utils/SceneLoader.cpp` |
| 全局仿真参数和重力 | `Simulation/Simulation.cpp` |
| 时间步和加速度 | `Simulation/TimeStep.cpp` |
| 主步进循环 | `Simulation/TimeStepController.cpp` |
| 刚体状态 | `Simulation/RigidBody.h` |
| 质量和惯性积分 | `Utils/VolumeIntegration.cpp` |
| 位置和旋转积分 | `PositionBasedDynamics/TimeIntegration.cpp` |
| 刚体接触冲量 | `PositionBasedDynamics/PositionBasedRigidBodyDynamics.cpp` |
| 碰撞检测抽象 | `Simulation/CollisionDetection.*` |
| SDF 碰撞 | `Simulation/DistanceFieldCollisionDetection.cpp`、`Simulation/CubicSDFCollisionDetection.cpp` |
| 约束包装 | `Simulation/Constraints.cpp` |
| 布料模型 | `Simulation/TriangleModel.*` |
| 四面体软体模型 | `Simulation/TetModel.*` |
| 模型和约束管理 | `Simulation/SimulationModel.cpp`、`Simulation/SimulationModel.h` |
| 杆件专用场景 | `Demos/StiffRodsDemos/*` |
| 场景格式说明 | `doc/file_format.md` |
| 场景生成 | `data/scenes/SceneGenerator.py`、`data/scenes/PileScene.py`、`data/scenes/CarScene.py` |

## 仓库特有使用习惯

- `bin/resources/shaders` 是运行时 shader 的主要位置。
- `data/models`、`data/sdf`、`data/scenes/Cache` 保存模型、距离场和缓存数据。
- 从 `bin` 目录运行 bat 时，场景路径使用 `../data/scenes/...`。
- 普通 JSON 场景优先走 `SceneLoaderDemo`；杆件/Cosserat 场景可能需要对应专用 Demo。
- 旧 JSON 和文档中可能存在旧参数名，实际修改时务必对照 `ParameterObject` 注册名。
- 如果场景没有按预期受重力影响，第一件事是检查是否写了 `gravitation` 而不是只写 `gravity`。
- 如果碰撞表现异常，依次检查碰撞类型、SDF 分辨率、模型缩放、刚体动态属性、质量/密度、摩擦/回弹、时间步和迭代次数。
