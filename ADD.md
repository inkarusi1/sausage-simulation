# SausageRodEditorDemo 后续功能可行性分析

本文记录基于 `README.md`、`EXT.md`、`OP.md`、`REF.md`、`EXP.md` 和 `Demos/SausageRodEditorDemo/main.cpp` 的新增功能调研结果。重点是后续迭代 `SausageRodEditorDemo` 暂停编辑器和运行时编辑能力时，哪些可以直接扩展，哪些需要补物理链路。

## 结论总览

| 需求 | 结论 | 主要缺口 |
| --- | --- | --- |
| `z/x/c` 分别绕 Z/X/Y 单方向旋转，并保持当前相对全局坐标系 | 可实现，改动小（已完成） | 已改成 `z/x/c` 分别绕全局 Z/X/Y 单方向旋转；旧的 Y 轴正反绑定已清掉 |
| 删除数字快捷键调柔软度 | 可实现，改动小（已完成） | 已移除 `0/1/5/9` 注册和提示文案；`+/-` 继续保留为细调 |
| 新增对象沿自身正/左/上方向拉伸 | 盒体可较快实现；复杂 SDF 物体需重建（盒体已完成） | 盒体已按局部轴拉伸并同步碰撞盒；球/锥/轨道还要各自重建 |
| 可新增盒、球、锥体、圆环、弯滑轨、直/平滑轨 | 盒/球/锥体/圆环/弯滑轨/直滑轨已完成 | 数字键选择形状，`n` 新增；`4` 为较粗圆环，`5/6` 为弯曲/直滑轨 |
| 放大缩小所有可编辑对象 | 等比缩放已完成 | 香肠、原始圆环/滑轨、平台和新增对象均可用 `[` / `]` 缩放；非等比 SDF 缩放仍建议重建 |
| 保存与加载上次编辑场景 | 轻量 JSON 已完成 | `p` 保存、`b` 加载，文件为 `bin/SausageRodEditorScene.json`；不是通用 SceneLoader 格式 |
| 新增物体可选择固定/非固定，非固定受重力 | 可实现，但不是只改 `dynamic=true`（轻量版已完成） | 已改为选中编辑器新增对象后按 `t` 切换 fixed/dynamic；原生 PBD 刚体 step 还没接 |
| 运行状态中让选中物体平移/旋转 | 可设计，建议区分“运动控制”和“直接编辑”（刚体 kinematic 控制已完成） | 已做运行中 kinematic 编辑；动态刚体速度控制还没做 |
| 香肠除柔软度外设置“粘度”，100% 完全粘住 | 不能直接靠原有参数完成；需新增粘附逻辑 | 现有主要是摩擦/回弹/接触约束；当前 rod 香肠不是 `ParticleData`，不能直接套用已有 particle joint |

## 当前实现边界

- `SausageRodEditorDemo` 是局部手写 C++ demo，不是原仓库通用场景编辑器。
- 当前香肠是自写 `RodPoint` 中心线模型，运动由 `rodSubStep()` 推进。
- 当前障碍物是 `RigidBody` + `CubicSDFCollisionDetection` 碰撞对象，但主要作为香肠采样碰撞的静态障碍。
- `DemoBase::step()` 只做导出，不会推进 PBD 刚体求解；当前 editor demo 的 `timeStep()` 调用 `base->step()`，没有调用 `Simulation::getCurrent()->getTimeStep()->step(*model)`。
- `cd` 当前用于香肠自写碰撞查询，但源码中没有看到 `Simulation::getCurrent()->getTimeStep()->setCollisionDetection(*model, cd)`。
- 因此，当前新增对象已经支持 fixed/dynamic 的轻量模式和运行中 kinematic 编辑（已完成），但还没有接入原生 PBD 刚体 step，也还没有把香肠接触反作用力推回动态刚体。
- 另外，场景里已经把摩擦和接触切向阻尼调低了一些，减少了拖拽感和下落迟滞（已完成）。

## 1. `z/x/c` 全局轴单方向旋转（已完成）

已完成。原先源码中的问题是：

- `rotateSausageY(angle)` 只支持绕全局 Y 轴旋转香肠中心线。
- `rotateSelectedEditorObjectY(angle)` 只支持绕全局 Y 轴旋转选中刚体。
- `z` 和 `x` 原先分别绑定到 Y 轴负/正方向旋转。

已按下面方式完成：

1. `rotateSausageAroundAxis(axis, angle)` 已完成。
2. `rotateSelectedEditorObjectAroundGlobalAxis(axis, angle)` 已完成。
3. 保持“相对全局坐标系”的做法已完成：刚体四元数用 `qDelta * currentRotation`，不是 `currentRotation * qDelta`。
4. 按键绑定已完成：
   - `z`: 绕全局 Z 轴单方向旋转。
   - `x`: 绕全局 X 轴单方向旋转。
   - `c`: 绕全局 Y 轴单方向旋转。

“单方向旋转”意味着每次按键只加同一个正角度，例如 `+10` 度。若以后需要反向旋转，建议用 UI 按钮或明确的反向模式；不要马上占用数字键。

## 2. 删除数字柔软度快捷键（已完成）

可实现。原先 `main.cpp` 中注册了：

- `0`: softness 0%
- `1`: softness 10%
- `5`: softness 50%
- `9`: softness 100%

这部分已完成：`MiniGL::addKeyFunc()` 里的 `0/1/5/9` 注册已经删掉，启动提示里的 `0/1/5/9` 文案也去掉了；`+/-` 继续留作柔软度细调。

注意：`OP.md` 中已经记录 `1/2/3/4` 在 MiniGL 里也有视图旋转相关风险。当前课程 demo 按需求把数字键重新用于“选择新建形状”，但不再用于柔软度预设；正式扩展更多形状时仍应集中维护这组快捷键。

## 3. 沿对象自身正/左/上方向拉伸（盒体已完成）

盒体局部方向拉伸已完成，复杂物体仍要分层处理。

基础缩放路径仍沿用 `scaleSelectedEditorObject(factor)`，它做的是：

1. 修改 `EditableRigidBody::boxScale`。
2. 用 cube 网格重新 `initBody()`。
3. 调 `syncRigidBodyTransform()`。
4. 更新 `DistanceFieldCollisionBox::m_box`。

局部方向拉伸建议定义为：

| 方向 | 局部轴 | 世界方向计算 |
| --- | --- | --- |
| 正方向/front | local `+Z` | `R * (0,0,1)` |
| 左方向/left | local `-X` | `R * (-1,0,0)` |
| 上方向/up | local `+Y` | `R * (0,1,0)` |

若要求“从某一侧拉伸，而不是从中心同时放大”，需要在改变尺寸的同时移动中心：

- 沿 local `+Z` 增加长度 `d`：`boxScale.z += d`，中心移动 `+0.5*d*(R*Z)`。
- 沿 local `-X` 增加长度 `d`：`boxScale.x += d`，中心移动 `-0.5*d*(R*X)`。
- 沿 local `+Y` 增加长度 `d`：`boxScale.y += d`，中心移动 `+0.5*d*(R*Y)`。

限制：

- 盒体：最适合先实现，视觉和 `DistanceFieldCollisionBox` 都能同步。
- 球体：非均匀拉伸会变成椭球，但现有 `addCollisionSphere` 只能表示球；要么只允许等比缩放，要么改用 SDF/box 代理。
- 锥体、滑轨、圆环等 SDF 物体：当前只开放等比缩放，并同步视觉局部网格、包围半径和 `CubicSDFCollisionObject::m_scale`；若要做非等比缩放或改变几何参数，仍应重建可视网格和 SDF。

## 4. 新增物体类型（盒/球/锥体/圆环/滑轨已完成）

当前已把新增盒/球/锥体/圆环/弯滑轨/直滑轨纳入同一套 `EditableRigidBody` 记录，包括形状类型、局部尺寸/半径、是否 SDF、是否动态、是否可删除。数字键选择待新增形状，`n` 统一新增当前形状。

| 类型 | 可行性 | 实现路线 |
| --- | --- | --- |
| 盒体 | 已有（已完成） | 复用 `cube.obj`、`addCollisionBox()`、`boxScale` |
| 球体 | 容易（已完成） | `bin/resources/models/sphere.obj` 已存在；新增 `addCollisionSphere()` helper |
| 锥体 | 中等（已完成：程序 mesh + SDF） | 原仓库没有 cone 解析碰撞；需程序生成 cone mesh 并用 `addCubicSDFCollisionObject()`，或新增 `DistanceFieldCollisionCone` |
| 圆环 | 已完成 | 复用程序 torus 网格 + SDF；新增圆环 tube 半径已加粗，减少视觉与碰撞边缘误差 |
| 弯滑轨 | 已完成 | 复用 `makeCoursePath()` 形状并居中生成副本，使用半管 mesh + SDF |
| 直/平滑轨 | 已完成 | 用直线路径复用半管网格生成器，使用半管 mesh + SDF |

球体新增时还要补（已完成的先留档）：

- `addCollisionSphere(body, radius)` helper。（已完成）
- `EditableRigidBody` 中保存 radius 或统一保存 `Vector3r scale`。（已完成）
- 删除时沿用“停用并移到远处”，避免 erase 刚体导致索引和碰撞对象引用失效。（已完成）

锥体新增时要注意（已完成的实现也保留说明）：

- `bin/resources/models` 里目前有 `cube.obj`、`sphere.obj`、`cylinder.obj`、`torus.obj`，没有看到 cone 模型。（已完成：当前用程序 mesh 兜底）
- 若临时需要锥体，可程序生成 mesh；若希望准确碰撞，优先用 mesh SDF。（已完成：当前用程序 mesh + mesh SDF）
- 当前锥体支持新增、移动、旋转和等比缩放；若以后要频繁非等比缩放/拉伸，仍需把重建 SDF 的成本纳入交互设计。

## 5. 固定/非固定新增物体（轻量版已完成）

概念上可实现。当前 `addBody(..., dynamic, ...)` 已经有 `dynamic` 参数；交互上已改为“生成后选中对象，再按 `t` 切换 fixed/dynamic”：

- `dynamic=false` 时会 `setMass(0)`，作为固定/静态刚体。
- `dynamic=true` 时保留由密度和网格计算出的质量，理论上可被重力和接触约束影响。

但当前 editor demo 缺两条关键链路：

1. 需要调用 `Simulation::getCurrent()->getTimeStep()->setCollisionDetection(*model, cd)`，让 PBD 求解器使用当前碰撞对象。
2. 需要在 `timeStep()` 中调用 `Simulation::getCurrent()->getTimeStep()->step(*model)`，否则动态刚体不会被 PBD 刚体积分推进。

还要处理时间推进：

- 当前 `rodSubStep()` 自己调用 `TimeManager::setTime(time + timeStepSize)`。
- `TimeStepController::step(*model)` 也会推进全局时间。
- 如果两者都保留，会导致时间被推进两次。建议后续接入动态刚体时，让 `TimeStepController` 负责全局时间推进，去掉或改造 `rodSubStep()` 末尾的手动时间推进。

更细的物理边界：

- 动态新增物体与静态平台、盒体、SDF 障碍的轻量接触响应已接入（已完成），并已改为对盒/锥/滑轨使用表面采样，减少外接球导致的圆环误碰；但还不是原 PBD 刚体流程。
- 动态新增物体之间已有轻量级互碰分离和速度响应（已完成），当前仍是演示级近似，不是完整刚体接触约束。
- 香肠当前不是 `SimulationModel` 里的粒子/软体，香肠与新增动态物体没有完整 rod-rigid 双向冲量。为避免“新增物体撞到香肠把香肠直接撞飞”，当前动态新增物体不会继续作为香肠的静态障碍参与 rod 采样修正。
- 因此如果目标是“香肠和非固定盒子/球体互相自然撞击”，还需要在 `collideSampleWithObjects()` 或接触后处理里实现稳定的双向冲量/位置修正，或者把香肠迁移回原仓库的粒子/四面体软体体系。

## 6. 运行状态中控制选中物体移动/旋转（刚体 kinematic 控制已完成）

可以设计，但建议区分两类行为。

原先暂停编辑函数都以 `editorIsPaused()` 作为入口保护；当前已放宽刚体的平移/旋转，香肠本体仍只允许暂停编辑：

- `translateSelectedEditorObject()`：刚体运行中可用，香肠仍需暂停。
- `rotateSelectedEditorObjectAroundGlobalAxis()`：刚体运行中可用，香肠仍需暂停。
- `scaleSelectedEditorObject()`：仍限暂停编辑。
- `stretchSelectedBoxLocal()`：仍限暂停编辑。
- `addEditorBox()` / `addEditorSphere()` / `addEditorCone()` / `deleteSelectedEditorObject()`：仍限暂停编辑。

运行中控制建议：当前已完成的是对固定/静态障碍的 kinematic 直接编辑；动态刚体速度命令仍是后续项。

| 模式 | 适用对象 | 行为 | 风险 |
| --- | --- | --- | --- |
| 运动控制模式 | 动态刚体 | 按键设置线速度/角速度，交给 PBD 积分 | 需要先接入 PBD 刚体 step；停止键或阻尼要设计清楚 |
| 运动中 kinematic 编辑 | 固定/静态障碍 | 按键直接改 transform，并同步 old/last 状态 | 本质是移动障碍，可能把香肠或动态物体挤穿，需要限制步长 |
| 暂停直接编辑 | 所有当前可编辑对象 | 沿用现有 transform 同步和清零速度 | 已有能力，最稳定 |

若使用当前 `MiniGL::addKeyFunc()`，按键是离散触发；想要“按住持续运动”，需要增加 key state 记录或在 idle loop 中轮询 GLFW 按键状态。否则只能依赖键盘自动重复，手感会不稳定。

建议实现策略：

1. 保留暂停状态下的直接编辑。
2. 运行状态下，同一批移动/旋转键进入“速度命令”分支。
3. 对动态刚体设置 `velocity`/`angularVelocity`。
4. 对静态障碍使用小步长 kinematic transform，并继续调用完整的 `syncRigidBodyTransform()`。
5. 对香肠运行中控制要谨慎：当前 rod 是自写粒子链，运行时直接移动全部点会像瞬移；若要“带物理感”应施加目标速度或外力，而不是直接改位置。

## 7. 香肠“粘度/粘住”能力

不能直接用原仓库现有参数做到“100% 粘在其他物体上”。

当前可复用能力：

- `RigidBody::setFrictionCoeff()` 和 `getFrictionCoeff()`。
- `DistanceFieldCollisionDetection` 中接触双方摩擦系数大致按和组合。
- 原仓库有 `RigidBodyContactConstraint`、`ParticleRigidBodyContactConstraint`、`RigidBodyParticleBallJoint` 等约束。
- 当前 `SausageRodEditorDemo` 的 rod 接触会读取障碍刚体的 friction/restitution。

当前不足：

- 源码中没有找到通用 `viscosity`、`adhesion` 或 `stickiness` 参数。
- 当前 rod 的 `applyContactVelocities()` 只是把切向速度按摩擦阻尼，且阻尼上限是 `0.85`，高摩擦也不会变成完全粘住。
- 原仓库的 `RigidBodyParticleBallJoint` 依赖 `SimulationModel::ParticleData`，而当前香肠点是 demo 内部 `vector<RodPoint>`，不能直接注册成这个 joint。

建议把用户语义拆成两类参数：

| 参数名 | 物理含义 | 是否能直接用现有功能 |
| --- | --- | --- |
| `friction` | 接触切向滑动阻尼/摩擦 | 可以，当前已有 |
| `internalDamping` 或 `viscosity` | 香肠内部速度阻尼，让形变恢复更慢/更黏 | 需要在 rod 速度或约束求解中新增 |
| `adhesion` 或 `stickiness` | 接触后粘附到其他物体，100% 时不再相对滑动 | 需要新增接触粘附约束 |

若目标是“100% 粘在其他物体上”，推荐新增 stickiness/adhesion，而不是只叫 viscosity。实现方向：

1. 接触发生时，按粘度概率或阈值创建 sticky contact。
2. 记录香肠 sample/rod point、目标刚体 bodyIndex、刚体局部接触点、接触法线和初始距离。
3. 每个 rod 子步求解一个粘附约束，使香肠点保持在目标刚体的局部接触点附近。
4. `stickiness=100%` 时不允许切向相对速度；`0%` 时只使用普通摩擦。
5. 可选：设置断裂阈值，拉力过大时脱粘。

如果以后把香肠迁移成 `ParticleData` 或 TetModel，可以更直接复用原仓库的 particle-rigidbody contact/joint 体系；在当前 rod 实现中，更现实的是写一个小型 sticky contact 列表。

## 8. 保存/加载编辑场景（轻量 JSON 已完成）

当前已实现 `SausageRodEditorDemo` 局部存档：

- `p` 保存到 `bin/SausageRodEditorScene.json`。
- `b` 从 `bin/SausageRodEditorScene.json` 加载。
- 保存内容包括柔软度、香肠长度/半径和中心线状态、原始可编辑物体的 transform/缩放、编辑器新增物体的类型/动态状态/速度/角速度、删除后的 inactive 状态。
- 加载时先重建基础课程场景，再按保存记录恢复编辑状态。

边界：

- 这不是原仓库通用 `SceneLoaderDemo` JSON 导出。
- 当前适合“下次继续编辑同一个课程 demo”；若要导出给其他 PBD demo 使用，需要额外做格式映射。

## 推荐实施顺序

1. 清理快捷键：移除 `0/1/5/9` 柔软度预设；实现 `z/x/c` 全局轴单向旋转；同步 `OP.md`、`README.md`。（已完成）
2. 重构新增物体工厂：支持 box/sphere/cone/ring/弯滑轨/直滑轨，并把 fixed/dynamic 保存在编辑表。（已完成）
3. 先做盒体局部方向拉伸：正/左/上三方向，保证视觉和 box 碰撞一致。（已完成）
4. 接入 PBD 刚体 step：让非固定新增物体真正受重力；处理 TimeManager 只推进一次。
5. 增加弯滑轨/直滑轨工厂：复用半管 mesh + SDF 生成逻辑，避免视觉和碰撞分离。（已完成）
6. 增加运行中运动控制：静态/固定刚体的小步 kinematic transform 已完成；动态物体速度/角速度控制未完成。
7. 单独设计粘附系统：先做 `stickiness`，再考虑是否需要内部 `viscosity`。
8. 保存/加载轻量编辑状态已完成；后续如需通用场景导出，再对接 `SceneLoaderDemo` JSON。

## 实现后必须同步的文档

- `OP.md`：快捷键和鼠标操作发生变化时必须更新，尤其是数字键语义、`z/x/c` 语义变化、新增运行中控制键。
- `README.md`：推荐 demo 的简要操作说明要同步。
- `EXT.md`：功能状态从“调研/未实现”变为“已实现”后更新。
- `REF.md`：只在新增了稳定的物理能力或仓库可复用实现后再写入；不要把临时设计草案放进 `REF.md`。
