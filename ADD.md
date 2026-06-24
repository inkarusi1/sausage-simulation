# PositionBasedDynamics 运行时暂停编辑能力调研

本文记录对“项目运行后按空格暂停时，能否用鼠标拖动物品、缩放物品、旋转物品、新增/删除物品”的调研和改造建议。它不是仓库已有功能清单，而是以后为项目设计交互式编辑功能时的参考。

## 总结

当前仓库的 demo 是仿真 viewer，不是场景编辑器。按空格暂停后，现成能力不能完整达到运行时编辑物体的效果。

| 目标 | 暂停状态下现成支持 | 结论 |
| --- | --- | --- |
| 鼠标拖动物品 | 不满足编辑器式拖动 | 现有拖动只给选中动态刚体/粒子加速度或速度。暂停时仿真步进 return，不会积分位置，所以物体不会被实时拖到鼠标位置。 |
| 缩放物品 | 不支持 | `scale` 是 JSON 加载时参数。GUI 里的 Scale 只缩放 ImGui 界面，不缩放场景物体。 |
| 旋转物品 | 不支持 | Alt+左键旋转的是相机视图；物体旋转来自仿真角速度、碰撞或代码设置，没有旋转 gizmo。 |
| 新增物品 | 不支持现成 UI | 源码能构造物体，但运行时新增需要同步模型数组、碰撞对象、约束、ID 映射和选择状态。 |
| 删除物品 | 不支持单个删除 | 只有整体 `cleanup()`；单个删除会牵动刚体下标、碰撞对象、约束、接触缓存和选择列表。 |

## 关键源码证据

### 暂停逻辑

文件：`Demos/SceneLoaderDemo/SceneLoaderDemo.cpp`

`timeStep()` 中：

- 读取 `DemoBase::PAUSE`。
- 若 pause 为 true，直接 `return`。
- 因此暂停时不会调用 `Simulation::getCurrent()->getTimeStep()->step(*model)`。
- 渲染和鼠标回调仍在窗口循环中，但物理积分不会推进。

这意味着：暂停时即使鼠标回调改了速度，物体位置也不会因为仿真积分而变化。

### 现有鼠标选择和拖动

文件：

- `Demos/Visualization/MiniGL.cpp`
- `Demos/Visualization/Selection.h`
- `Demos/Common/DemoBase.cpp`

现有流程：

1. `DemoBase::init()` 注册 `MiniGL::setSelectionFunc(selection, this)`。
2. 鼠标左键无修饰键框选，松开时触发 `DemoBase::selection()`。
3. 粒子选择通过 `Selection::selectRect()` 检查粒子位置。
4. 刚体选择只检查刚体质心位置，不检查完整网格轮廓。
5. 选中后设置 `MiniGL::setMouseMoveFunc(2, mouseMove)`。
6. `DemoBase::mouseMove()` 反投影鼠标位置，得到 `diff`。
7. 对动态刚体执行 `rb[i]->getVelocity() += 3.0 / h * diff`。
8. 对动态粒子执行 `pd.getVelocity(i) += 5.0 * diff / h`。

所以现有“拖动”是给速度一个增量，更像鼠标施力或推拽物体。它不是直接改 `position`，也不是暂停编辑中的 transform 操作。

### 相机交互不是物体交互

文件：`Demos/Visualization/MiniGL.cpp`

已有视图操作：

- 左键 + Ctrl：沿视图 z 方向平移场景。
- 左键 + Shift：沿 x/y 平移场景。
- 左键 + Alt：旋转场景视图。
- 滚轮默认改变 `movespeed`，影响相机/视图移动速度。

这些都是相机或视图变换，不会改变物体本身。

### GUI 功能边界

文件：`Demos/Common/Simulator_GUI_imgui.cpp`

GUI 里有：

- 空格暂停/继续。
- `r` reset。
- `w` wireframe。
- 时间步长、仿真参数、渲染参数。
- GUI Scale 100% 到 200%。

GUI Scale 是界面缩放，不是物体缩放。没有发现新增物体、删除物体、物体平移/旋转/缩放的面板。

## 单项分析

### 鼠标拖动物品

现状：

- 运行时可以框选动态刚体或粒子。
- 继续运行时，中键拖动会给选中对象增加速度。
- 暂停时回调仍可执行，但只改速度，不改位置；因为 `timeStep()` 暂停直接 return，所以不会看到物体被实时拖动。

要达到“暂停后鼠标拖动物品”的效果，需要新增编辑模式：

1. 鼠标点击或框选得到目标物体。
2. 将鼠标屏幕坐标反投影到拖动平面或射线命中点。
3. 暂停时直接设置物体位置，而不是只改速度。
4. 对刚体同步：
   - `setPosition()`
   - `setLastPosition()`
   - `setOldPosition()`
   - 需要时清零 `setVelocity()`
   - `getGeometry().updateMeshTransformation(...)`
   - 碰撞 AABB/BVH 状态
5. 对粒子同步：
   - 当前、上一帧、旧位置
   - 速度清零或按拖动速度重设
6. 松开鼠标后再恢复仿真。

建议：暂停编辑时使用“直接设置 transform”的方式；运行中交互时才使用“施力/改速度”的方式。

### 缩放物品

现状：

- JSON 的 `scale`、`collisionObjectScale` 只在加载/构建模型时使用。
- 没有运行时缩放 setter 或 UI。

缩放比平移更危险，因为它会影响：

- 渲染网格局部坐标。
- 刚体质量和惯性。
- 碰撞体尺寸。
- SDF 距离场。
- 关节锚点和约束位置。

建议实现方式：

- 不建议只把现有 mesh 顶点乘一个比例。
- 暂停编辑时，修改对象的 scene data scale，然后重建该对象或重建整个场景。
- 如果是 SDF 碰撞，缩放后应同步 `collisionObjectScale`，必要时重新生成 SDF。
- 如果物体带关节，需要根据缩放策略重新计算关节锚点。

最稳路线：缩放等价于“删除旧物体，以新 scale 创建新物体”。

### 旋转物品

现状：

- 物体可以因物理仿真而旋转，例如设置 `angularVelocity` 或碰撞产生角速度。
- 现成鼠标旋转操作作用于视图，不作用于物体。
- `RigidBody` 有 `setRotation()` 和 `rotationUpdated()`，但没有 UI 接线。

暂停编辑时要旋转刚体，需要同步：

- 当前旋转、上一帧旋转、旧旋转。
- 旋转矩阵、惯性世界矩阵、world-to-local 变换。
- 可视 mesh 变换。
- 碰撞对象状态。

建议实现：

1. 添加旋转 gizmo 或键盘快捷轴向旋转。
2. 设置刚体 quaternion。
3. 同步 last/old quaternion，避免恢复仿真后产生异常角速度。
4. 调用 `rotationUpdated()`。
5. 调用 `updateMeshTransformation()`。
6. 更新碰撞 AABB/BVH。

对布料和软体，旋转不是单个刚体 quaternion，而是要旋转一组粒子位置，因此应操作粒子集合并同步 old/last positions。

### 新增物品

现状：

- `SceneLoaderDemo::buildModel()` 在加载时一次性创建刚体、布料、软体、碰撞对象、关节和 ID 映射。
- `SimulationModel` 有 `addTriangleModel()`、`addTetModel()`、`addLineModel()` 和多种 `add*Joint()`。
- 刚体没有统一 public `addRigidBody()` 封装；现有 demo 常直接访问 `getRigidBodies()`，resize 后 `new RigidBody()`。
- 新增碰撞体要调用 `addCollisionBox()`、`addCollisionSphere()`、`addCubicSDFCollisionObject()` 等。

如果要暂停后新增物品，有两条路线。

路线 A：重建式，推荐

1. 暂停。
2. 在 editor scene 或 `SceneData` 中添加一个 `RigidBodyData` / `TriangleModelData` / `TetModelData`。
3. 清理当前 `SimulationModel` 和 `CollisionDetection`。
4. 用修改后的 scene data 调用构建流程。
5. 恢复选择状态和 GUI。

优点：稳定，索引、SDF、约束、碰撞对象统一重建。缺点：不能保留所有物体的当前动态状态，除非额外做状态映射。

路线 B：live add

需要新增统一函数，例如：

- `addRigidBodyFromData(...)`
- `addCollisionObjectForRigidBody(...)`
- `addTriangleModelFromData(...)`
- `addTetModelFromData(...)`

同时要更新：

- `SimulationModel` 的 vector。
- 碰撞检测对象列表。
- ID 到 index 的映射。
- constraint groups。
- GUI/选择状态。

### 删除物品

现状：

- `SimulationModel::cleanup()` 会整体删除所有刚体、三角模型、四面体模型、线模型和约束。
- `CollisionDetection::cleanup()` 会整体删除所有碰撞对象。
- 没有发现安全删除单个刚体、单个碰撞对象、单个约束的 API。

删除单个物体要处理：

- 刚体 vector 下标变化。
- 碰撞对象 `bodyIndex`。
- 关节、弹簧、马达、阻尼器中引用的 body index。
- 接触约束缓存。
- 选择列表。
- Scene JSON 中的 id 映射。

建议实现：

- 暂停编辑器初版使用重建式删除：从 scene data 删除目标对象，然后整体重建。
- 若以后需要 live delete，必须写 `removeRigidBody(index)` 及相关索引重映射逻辑。
- 删除带关节物体时，应同时删除所有引用该物体的约束。

## 推荐功能设计

### 建议的编辑模式

新增一个明确的 editor mode：

- `Simulate`：原有仿真模式，鼠标可施力拖拽。
- `EditPaused`：暂停编辑模式，鼠标直接改 transform。

切换规则：

1. 空格暂停进入可编辑状态。
2. 用户选择对象。
3. 工具栏或快捷键选择 Move / Rotate / Scale / Add / Delete。
4. 编辑操作修改 scene data 和运行时对象。
5. 用户恢复仿真时，清理速度或按操作差分生成速度。

### 推荐优先级

1. 选中对象信息面板：显示 rigid body index、scene id、position、rotation、scale、mass、dynamic。
2. 暂停平移刚体：最容易实现。
3. 暂停旋转刚体：需要 quaternion 和 mesh/collision 同步。
4. 重建式新增/删除刚体：比 live 增删安全。
5. 重建式缩放刚体：避免质量/惯性/SDF 不一致。
6. 布料/软体粒子集合编辑：放后面，因为粒子状态同步和约束影响更复杂。

### 推荐新增源码模块

可以新增一个编辑控制层，避免把逻辑散在 `DemoBase` 和 `SceneLoaderDemo`：

- `Demos/Common/SceneEditor.h/.cpp`
- `Demos/Common/TransformGizmo.h/.cpp`
- `Demos/Common/EditorSelection.h/.cpp`

职责：

- 选择对象。
- 管理编辑模式。
- 平移、旋转、缩放对象。
- 新增/删除 scene data。
- 调用重建流程。
- 维护选中对象和 GUI 面板。

### 最小可行实现

第一版只支持刚体，流程如下：

1. 空格暂停。
2. 左键点击或框选刚体质心。
3. ImGui 面板显示 position / rotation / scale。
4. 修改 position 时立即 `setPosition()`，同步 old/last position，更新 mesh。
5. 修改 rotation 时 `setRotation()`，同步 old/last rotation，调用 `rotationUpdated()`，更新 mesh。
6. 修改 scale、新增、删除时走 scene data 重建。
7. 恢复仿真前清零选中刚体速度和角速度，避免编辑产生爆炸。

## 重要风险

- 暂停编辑后恢复仿真，如果 old/last position 没同步，速度更新会突然变大。
- 直接缩放网格但不重算质量/惯性，会导致物理行为错误。
- 直接缩放 SDF 物体但不更新 `collisionObjectScale`，视觉和碰撞会错位。
- 删除物体后不更新约束引用，会造成越界或错误约束。
- 新增物体后不重建 collision detection，物体可能可见但不可碰撞。
- 布料/软体不能按刚体 transform 简单处理，需要整体变换粒子位置。
