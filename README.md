# PositionBasedDynamics 课程项目文档入口

本仓库是在开源项目 PositionBasedDynamics 基础上完成虚拟现实实验课程期末项目的工作区。原开源项目 README 已保留为：

- `README.old.md`：PositionBasedDynamics 上游项目介绍、依赖、构建和原始功能说明。

当前课程项目主要关注“香肠下落、圆环、半管滑轨、平台、柔软度控制、暂停编辑”等演示功能。建议优先阅读下面几份本地文档。

## 构建说明（首次 clone 后必读）

> **git 仓库只包含源码、运行资源和必要的第三方源码，不包含 `build-vs2/` 编译目录和 `bin/*.exe`。**
> clone 后运行脚本即可生成 VS 项目并编译 Release 版。

### 前提条件

- Windows 10/11 64-bit
- Visual Studio 2022（含 **"使用 C++ 的桌面开发"** 工作负载，内含 MSBuild 和 CMake）
- 不需要提交或下载 `build-vs2/`、`.pdb`、`.lib`、`.obj` 等大文件；这些都会本地生成。

### 一键构建（推荐）

在仓库根目录打开 PowerShell，执行：

```powershell
powershell -ExecutionPolicy Bypass -File .\setup_and_build.ps1
```

脚本会自动完成：
1. 自动查找 Visual Studio / Build Tools 的 CMake 和 MSBuild。
2. 调用 CMake 生成 `build-vs2/`。
3. 使用仓库内置的 `extern/Discregrid` 和 `extern/GenericParameters` 源码构建依赖，通常不需要联网下载外部项目。
4. 编译 `SausageRodEditorDemo`（Release x64）。
5. 复制 `data/models`、shader 和字体资源到 `bin/resources/`。
6. 产物输出到 `bin/SausageRodEditorDemo.exe`。

常用参数：

```powershell
# 重新生成构建目录
powershell -ExecutionPolicy Bypass -File .\setup_and_build.ps1 -Rebuild

# Debug 版，仅开发调试时需要
powershell -ExecutionPolicy Bypass -File .\setup_and_build.ps1 -Config Debug

# 使用自定义构建目录
powershell -ExecutionPolicy Bypass -File .\setup_and_build.ps1 -BuildDir build-local
```

> 若直接执行 `.\setup_and_build.ps1` 报“执行策略”错误，用上面的 `powershell -ExecutionPolicy Bypass -File ...` 形式即可，不需要修改系统策略。

### 手动构建

**第一步：CMake 配置（仅首次需要）**

```powershell
cmake -S . -B build-vs2 -G "Visual Studio 17 2022" -A x64
```

**第二步：MSBuild 编译**

```powershell
& 'C:\Program Files\Microsoft Visual Studio\2022\Professional\MSBuild\Current\Bin\amd64\MSBuild.exe' `
  'build-vs2\Demos\SausageRodEditorDemo\SausageRodEditorDemo.vcxproj' `
  /p:Configuration=Release /p:Platform=x64 /m
```

> Community / Enterprise / Build Tools 的 MSBuild 路径不同；推荐优先用 `setup_and_build.ps1`，脚本会自动查找。

**第三步：运行**

```bat
cd bin
SausageRodEditorDemo.exe
```

### git 仓库结构说明

| 路径 | 是否在 git | 说明 |
| --- | --- | --- |
| `Demos/`, `PositionBasedDynamics/`, 等 | ✓ 已追踪 | C++ 源码 |
| `extern/` | ✓ 已追踪 | 第三方依赖头文件（eigen、glfw、imgui 等） |
| `extern/Discregrid`, `extern/GenericParameters` | ✓ 已追踪 | 构建所需外部源码的最小副本，避免首次构建时必须联网 |
| `data/` | ✓ 已追踪 | 模型、场景 JSON、SDF 缓存 |
| `build-vs2/` | ✗ 忽略 | CMake 生成的 VS 项目 + 编译中间产物（~900MB） |
| `bin/*.exe` | ✗ 忽略 | 编译输出的可执行文件 |
| `bin/resources/` | ✗ 忽略 | 构建时自动从 `data/` 复制的运行时资源 |

### 提交前检查

GitHub 单文件超过 100MB 无法普通上传。本项目的 `.gitignore` 已忽略常见大产物：

- `build*/`
- `bin/*.exe`
- `*.pdb`
- `*.lib`
- `*.obj`
- `*.ilk`

提交前建议执行：

```powershell
git status --short --ignored
```

只应提交源码、文档、`data/` 资源和 `extern/` 中必要依赖源码，不要提交本地编译产物。

---

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
| `SausageRodEditorDemo` | `bin/SausageRodEditorDemo.exe` | 当前推荐入口。基于中心线杆香肠，支持暂停后选择、拖动、移动、全局轴旋转、盒体拉伸、缩放香肠/原场景物体/新增物体、新增盒体/球体/锥体/圆环/滑轨、删除新增物体、保存与加载编辑场景。 |
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
- 暂停后 `z/x/c`：绕全局 Z/X/Y 轴旋转。
- 暂停后 `[` / `]`：缩放当前选中对象，包括香肠、原场景 SDF 物体和编辑器新增物体。
- 暂停后 `f/g/h`：沿局部前/左/上方向拉伸盒体。
- 暂停后 `1/2/3/4/5/6`：选择新建形状：盒体、球体、锥体、圆环、弯曲滑轨、直滑轨。
- 暂停后 `n`：新建当前数字键选中的形状。
- 暂停后 `t` / `d`：切换选中编辑器新增对象的固定/动态状态，或删除编辑器新增对象。
- 暂停后 `p` / `b`：保存/加载编辑场景，文件位于 `bin/SausageRodEditorScene.json`。

更完整的键鼠说明以 `OP.md` 为准。

## 开发注意

- 原仓库的通用加载和物理步进主要通过 `SceneLoaderDemo`、`SimulationModel`、`TimeStepController`、`CubicSDFCollisionDetection` 等实现。
- 课程新增的香肠 demo 为手写 C++ 场景，不完全等同于 JSON 场景加载流程。
- 暂停编辑能力目前属于 `SausageRodEditorDemo` 的局部功能，不是全仓库通用编辑器。
- 复杂 SDF 物体的缩放会同步视觉网格和 SDF 缩放值；若以后改为非等比缩放，仍应重建网格和 SDF。
- 删除原始场景对象会牵动刚体下标、碰撞对象和约束引用；当前只允许删除编辑器新增对象。
