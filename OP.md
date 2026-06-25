# PositionBasedDynamics 现存操作输入参考

本文记录当前项目中已经存在的快捷键和鼠标操作，主要用于后续设计新交互功能时避免冲突。源码入口主要在 `Demos/Visualization/MiniGL.cpp`、`Demos/Common/Simulator_GUI_imgui.cpp`、`Demos/Common/DemoBase.cpp` 和各 demo 的 `MiniGL::addKeyFunc(...)`。

## 全局/通用快捷键

| 输入 | 当前用途 | 主要来源 | 冲突风险 |
| --- | --- | --- | --- |
| Space | 暂停/继续仿真 | `Simulator_GUI_imgui::init()` 注册 `switchPause()`；文档也列出 | 高。编辑模式切换不要直接复用空格。 |
| `r` | reset 仿真 | 多数 demo 注册 `reset`；GUI 也注册 `DemoBase::reset` | 高。很多 demo 都占用，且可能存在多个 `r` 回调。 |
| `w` | 线框/填充渲染切换 | `Simulator_GUI_imgui::init()` 注册 `switchDrawMode()`；文档也列出 | 高。不要用作编辑器的移动/世界模式切换。 |
| ESC | 退出程序/离开主循环 | `MiniGL::keyboard()` | 高。不要改成删除或取消物体，除非加二级确认或模式判断。 |
| `A` | 视图沿 z 方向移动 | `MiniGL::keyboard()` | 中。源码级占用，文档未显式列出。 |
| `Y` | 视图沿反 z 方向移动 | `MiniGL::keyboard()` | 中。源码级占用，文档未显式列出。 |
| 方向键 Up/Down/Left/Right | 平移视图 | `MiniGL::keyboard()` | 高。不要用方向键做物体微调，除非先关闭/接管相机控制。 |
| F5 | 退出 breakpoint loop | `MiniGL::keyboard()` | 低到中。调试相关，尽量保留。 |
| `1` / `2` / `3` / `4` / `5` / `6` | MiniGL 中部分数字键作为视图旋转备用键；`SausageRodEditorDemo` 暂停时用于选择新建形状 | `MiniGL::char_callback()` / `SausageRodEditorDemo` | 中。当前课程 demo 明确复用数字键做形状选择，其他 demo 仍需谨慎。 |

注意：`MiniGL::char_callback()` 会遍历 `keyfunc`，匹配 `addKeyFunc` 注册的字符；同一个键如果被多个地方注册，可能执行多个回调。后续新增快捷键最好集中管理，避免继续分散调用 `MiniGL::addKeyFunc()`。

## Demo 特有快捷键

| 输入 | 当前用途 | 主要 demo |
| --- | --- | --- |
| `s` | 单步执行 | `Demos/StiffRodsDemos/DirectPositionBasedSolverForStiffRodsDemo.cpp` |
| `-` / `_` | 降低 softness | `SausageCourseDemo`、`SausagePhysicsCourseDemo`、`SausageRodCourseDemo`、`SausageRodEditorDemo` |
| `+` / `=` | 提高 softness | `SausageCourseDemo`、`SausagePhysicsCourseDemo`、`SausageRodCourseDemo`、`SausageRodEditorDemo` |
| `0` | softness 设为 0 | Sausage 系列旧 demo；`SausageRodEditorDemo` 已移除数字预设 |
| `1` | softness 设为 10 | Sausage 系列旧 demo，同时也可能触发 MiniGL 的数字键视图旋转逻辑；`SausageRodEditorDemo` 已移除数字预设 |
| `5` | softness 设为 50 | Sausage 系列旧 demo；`SausageRodEditorDemo` 已移除数字预设 |
| `9` | softness 设为 100 | Sausage 系列旧 demo；`SausageRodEditorDemo` 已移除数字预设 |

这些 demo 特有键不一定影响 `SceneLoaderDemo`，但如果以后把编辑器做成通用 `DemoBase` 功能，仍应避免直接复用。

## 鼠标操作

| 输入 | 当前用途 | 主要来源 | 冲突风险 |
| --- | --- | --- | --- |
| 左键拖动，无修饰键 | 框选粒子/刚体质心，松开后触发选择 | `MiniGL::mousePress()` + `DemoBase::selection()` | 高。编辑器选择可以复用，但要明确点击选择和框选的规则。 |
| 中键拖动，源码 button 值 2 | 对已选动态刚体/粒子施加速度增量 | `DemoBase::selection()` 注册 `MiniGL::setMouseMoveFunc(2, mouseMove)` | 高。若做暂停拖动物体，需要覆盖或区分现有“施力拖拽”。 |
| 左键 + Ctrl 拖动 | 沿视图 z 方向平移场景 | `MiniGL::mouseMove()` | 高。不要用 Ctrl+左键做物体缩放/深度拖动，除非接管相机控制。 |
| 左键 + Shift 拖动 | 沿 x/y 平移场景 | `MiniGL::mouseMove()` | 高。不要用 Shift+左键做物体平面拖动，除非模式化处理。 |
| 左键 + Alt 拖动 | 旋转场景视图 | `MiniGL::mouseMove()` | 高。不要直接用 Alt+左键做物体旋转。 |
| 鼠标滚轮 | 默认改变 `movespeed`，即视图移动速度；ImGui 可先捕获滚轮 | `MiniGL::mouseWheel()`、`Simulator_GUI_imgui::init()` | 中。不要直接作为物体缩放，除非在编辑模式中显式接管。 |
| 在 ImGui 窗口/控件上点击或滚轮 | ImGui 捕获输入，不传给场景控制 | `Simulator_GUI_imgui::init()` 注册 ImGui 回调 | 中。新增编辑器 UI 时要注意 `WantCaptureMouse/Keyboard`。 |

## `SausageRodEditorDemo` 暂停编辑操作

本节只记录新增 demo `Demos/SausageRodEditorDemo` 的运行时编辑输入，和上面的仓库原有全局操作分开看。入口可执行文件为 `bin/SausageRodEditorDemo.exe`。这些输入只有在 Space 暂停后才处理；未暂停时尽量交回原有 viewer/仿真逻辑。

| 输入 | 暂停编辑用途 | 作用对象 | 与原有操作的区分 |
| --- | --- | --- | --- |
| 左键拖动，无修饰键 | 选择最近的可编辑对象并平移 | 香肠、平台、两个圆环、半管滑轨、编辑器新增盒体/球体/锥体/圆环/滑轨 | 只在暂停时由 `MiniGL::addMousePressFunc/addMouseMoveFunc` 抢占；运行时仍保留原 viewer 行为。 |
| `q` | 切换选中对象 | 可编辑对象列表 | 未占用原有高风险键；只在暂停时生效。 |
| `i` / `k` | 沿 y 轴上/下移动 | 当前选中对象 | 不使用方向键，避免和全局视图平移冲突。 |
| `j` / `l` | 沿 x 轴左/右移动 | 当前选中对象 | 不使用方向键，避免和全局视图平移冲突。 |
| `u` / `o` | 沿 z 轴前/后移动 | 当前选中对象 | 避开 `A`/`Y` 这类源码级相机移动键。 |
| `z` / `x` / `c` | 分别绕全局 Z/X/Y 轴正向旋转 | 当前选中对象 | 不复用 Alt+左键，避免和相机旋转混淆。 |
| `[` / `]` | 缩小/放大当前选中对象 | 香肠、平台、圆环、半管滑轨、编辑器新增物体 | 盒体同步 box 碰撞；球体同步 sphere 碰撞；SDF 圆环/滑轨/锥体同步视觉网格和 SDF 缩放值；香肠同步中心线长度与半径。 |
| `f` / `g` / `h` | 沿局部前/左/上方向拉伸盒体 | 平台、编辑器新增盒体 | 从对应一侧拉伸并平移中心，保持视觉盒体与 box 碰撞一致。 |
| `1` / `2` / `3` / `4` / `5` / `6` | 选择新建形状：盒体/球体/锥体/圆环/弯曲滑轨/直滑轨 | 后续 `n` 新建的对象 | 数字键不再用于柔软度预设；这里作为编辑器形状选择入口。 |
| `n` | 新增当前数字键选中的形状 | 编辑器新增对象 | 新增物体会登记到编辑器对象表和碰撞检测对象列表。 |
| `m` / `v` | 快速新增球体/锥体 | 编辑器新增对象 | 保留为兼容快捷键；推荐后续用数字键选择形状后按 `n`。 |
| `t` | 切换选中编辑器新增对象 fixed/dynamic | 仅编辑器新增对象 | 不是生成前开关；必须先选中已生成对象再切换。 |
| `d` | 删除当前编辑器新增对象 | 仅编辑器新增对象 | 不删除原始场景对象，避免破坏原场景索引、约束和碰撞对象引用。 |
| `p` | 保存当前编辑场景 | 香肠、原始可编辑对象、编辑器新增对象 | 写入 `bin/SausageRodEditorScene.json`，记录新增/删除、缩放、位置、旋转、动态状态等轻量编辑状态。 |
| `b` | 加载上次保存的编辑场景 | 当前 demo 场景 | 从 `bin/SausageRodEditorScene.json` 重建场景；会先恢复基础场景再套用保存的编辑状态。 |

注意：

- Space 仍然只使用原有暂停/继续功能；新增编辑器不重新定义 Space。
- `r` 仍保留为 reset，不作为编辑器删除或撤销。
- `w`、ESC、方向键、鼠标滚轮、中键拖动和修饰键左键操作没有被新增编辑器占用。
- 数字键在 `SausageRodEditorDemo` 中不再用于柔软度预设，当前改为编辑器新建形状选择；其他 Sausage 旧 demo 仍可能保留数字预设。

## 后续新增快捷键建议

- 编辑器功能优先使用 ImGui 工具栏、菜单或显式模式按钮。
- 若必须使用快捷键，建议先建立统一输入路由，而不是继续散落 `MiniGL::addKeyFunc()`。
- 避免直接占用 Space、`r`、`w`、ESC、方向键、左键修饰组合和中键拖动。
- 数字键已有隐性风险，尤其 `1` 同时被 MiniGL 和 Sausage demo 使用，新增功能尽量避开数字键。
