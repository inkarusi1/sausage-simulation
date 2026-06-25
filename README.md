# PositionBasedDynamics 课程项目文档入口

本仓库是在开源项目 PositionBasedDynamics 基础上完成虚拟现实实验课程期末项目的工作区。原开源项目 README 已保留为：

- `README.old.md`：PositionBasedDynamics 上游项目介绍、依赖、构建和原始功能说明。

当前课程项目主要关注“香肠下落、圆环、半管滑轨、平台、柔软度控制、暂停编辑”等演示功能。建议优先阅读下面几份本地文档。

## 文档导航

| 文档 | 用途 | 建议阅读时机 |
| --- | --- | --- |
| `REF.md` | 物理仿真和源码设计参考。包含原仓库刚体、软体、布料、SDF、接触冲量、重力、场景构建等位置说明，也记录了课程新增 demo 的可复用设计。 | 修改物理效果、碰撞、柔软度、SDF、运行时编辑实现时先看。 |
| `OP.md` | 键鼠操作和快捷键冲突参考。区分原仓库全局操作、已有 demo 快捷键和 `SausageRodEditorDemo` 的暂停编辑操作。 | 增加交互、改快捷键、调试鼠标操作前先看。 |
| `EXT.md` | 扩展功能调研与当前落地状态。说明原仓库暂停后为什么不能直接编辑物体，以及当前新增 demo 已实现/未实现的编辑能力。 | 想继续扩展拖动、旋转、缩放、新增、删除、保存场景时先看。 |
| `EXP.md` | 本次课程项目迭代中的必要设计与调试经验总结。 | 回顾为什么采用当前方案、避免重复踩坑时看。 |

## 当前主要 demo

| Demo | 可执行文件 | 说明 |
| --- | --- | --- |
| `SausageRodEditorDemo` | `bin/SausageRodEditorDemo.exe` | 当前推荐入口。基于中心线杆香肠，支持暂停后选择、拖动、移动、旋转、缩放盒体、新增/删除盒体。 |
| `SausageRodCourseDemo` | `bin/SausageRodCourseDemo.exe` | 较轻量的香肠物理过程 demo，无暂停编辑层。 |
| `SausagePhysicsCourseDemo` | `bin/SausagePhysicsCourseDemo.exe` | 早期四面体/软体方向实现，保留作对比。 |

## 推荐阅读顺序

1. 只想运行或演示：先看 `OP.md` 的 `SausageRodEditorDemo` 暂停编辑操作。
2. 想理解为什么这样设计：看 `EXP.md`，再看 `REF.md` 的“新增课程 demo 的可复用设计”。
3. 想继续开发暂停编辑器：看 `EXT.md`，再对照 `OP.md` 避免快捷键冲突。
4. 想回到原仓库功能或 JSON 场景：看 `REF.md` 和 `README.old.md`。

## 运行提示

从 `bin` 目录直接运行推荐 demo：

```bat
SausageRodEditorDemo.exe
```

常用操作：

- Space：暂停/继续。
- 暂停后左键拖动：选择并移动最近可编辑对象。
- 暂停后 `q`：切换选中对象。
- 暂停后 `i/k/j/l/u/o`：沿 y/x/z 轴移动。
- 暂停后 `z/x`：绕 y 轴旋转。
- 暂停后 `[` / `]`：缩放盒体类对象。
- 暂停后 `n` / `d`：新增/删除编辑器新增盒体。

更完整的键鼠说明以 `OP.md` 为准。

## 开发注意

- 原仓库的通用加载和物理步进主要通过 `SceneLoaderDemo`、`SimulationModel`、`TimeStepController`、`CubicSDFCollisionDetection` 等实现。
- 课程新增的香肠 demo 为手写 C++ 场景，不完全等同于 JSON 场景加载流程。
- 暂停编辑能力目前属于 `SausageRodEditorDemo` 的局部功能，不是全仓库通用编辑器。
- 复杂 SDF 物体运行时不建议直接缩放；应重建网格和 SDF，避免视觉与碰撞错位。
- 删除原始场景对象会牵动刚体下标、碰撞对象和约束引用；当前只允许删除编辑器新增盒体。
