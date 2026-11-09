# CellCultureLab

基于 **Isaac Sim 5.1** 的细胞培养实验室数字孪生与可交互资产工程。目标是在一个大的 USD 场景里放置多个能被夹具/机器人真实操作、能被脚本判定成功/失败的子任务，为后续 VLA 数据生成和机器人策略提供环境。

## 当前入口

| 用途 | 入口 |
|---|---|
| 完整实验室（推荐） | `assets/scenes/biolab_main_v06.usda` |
| 一键打开 v06 | `启动场景_v06.bat` |
| 旧版 v04 场景 | `启动场景_v04.bat` |
| 材质/朝向查看层 | `outputs/gui_review/biolab_v06_color_zup.usda` |
| 视觉诊断（mini + 8 吸头） | `assets/scenes/biolab_native_mini_8tips_candidate.usda` |

## 快速开始

在项目根目录打开 PowerShell：

```powershell
# 确认 Isaac Sim 的 Python
Test-Path 'D:\IsaacSim5\env\Scripts\python.exe'

# 打开完整实验室 v06
& 'D:\IsaacSim5\env\Scripts\python.exe' scripts\open_scene.py assets\scenes\biolab_main_v06.usda

# 离线体检：引用、加载噪声、刚体/碰撞/关节计数
& 'D:\IsaacSim5\env\Scripts\python.exe' scripts\verify_scene.py assets\scenes\biolab_main_v06.usda

# mini 开合盖/转子闭环
& 'D:\IsaacSim5\env\Scripts\python.exe' scripts\run_mini_lid_task_v2.py --build-only
& 'D:\IsaacSim5\env\Scripts\python.exe' scripts\run_mini_lid_task_v2.py --sequence lid_loop --gravity none --mode cpu

# 5430 手动诊断场景
& 'D:\IsaacSim5\env\Scripts\python.exe' scripts\open_5430_manual_gui.py
```

Isaac 很吃内存，**一次只开一个**。这台机器空载提交约 24 GB，跑 8 吸头以上场景前先确认没有残留 Python 进程。

## 目录

```text
CellCultureLab/
  assets/
    scenes/              主场景 v01–v07、mini 与 8 吸头候选场景
    instruments*/        六类仪器的多版转换产物
    interactive/         96 枪头、移液枪及其关节
    labutopia_assets/    LabUtopia lab_021 房间外壳
  upstream/              AutoBio MJCF/OBJ、MuJoCo 真值、3.3.0 参考
  scripts/               构建、转换、验证、测量、任务 runner
  outputs/
    runtime_validation/  运行时证据（JSON/日志/逐步轨迹）
    gui_review/          GUI 查看层与材质审查
  docs/                  交接、专题报告、原始测量数据
  _archive/              本次整理归档，不参与运行
```

## 当前状态（摘要）

- **可用**：v06 场景结构、引用完整性、GUI 查看层、离线体检脚本。
- **短时通过**：v04 交互耗材、8 吸头 GPU 120 步、mini 关节级开合盖/转子闭环。
- **未验收**：5430 单管插入、机器人抓取/移液、C1000 软锁与 detent、五类缺失工位。
- **未通过/暂缓**：96 精密吸头规模化、分级碰撞、完整 AutoBio 任务键。
- **占位内容**：50 mL 管默认色、LabUtopia 灰色贴图；`visual_data_ready=false`。

详细状态与证据边界见 [docs/README.md](docs/README.md)。

## 重要约定

- 派生场景用 **Export/另存新路径**，不要 `Save` 覆盖 v06 等源图层。
- 判断物体有没有动，读 **PhysX/tensor view**，不要读 USD xform。
- 关节的 `body0/body1` 是 **relationship**，不是 attribute。
- `/physicsScene` 需要同时应用 `UsdPhysics.Scene` 与 `PhysxSchema.PhysxSceneAPI`。
- 任何“通过”都要写清范围：几何 / 关节级 / 短时 / 完整任务。

## 文档

- [docs/README.md](docs/README.md)：文档索引与阅读顺序。
- [docs/千问继续工作提示词库_20261019.md](docs/千问继续工作提示词库_20261019.md)：当前交接总纲。
- [docs/未解决与绕行清单_20261018.md](docs/未解决与绕行清单_20261018.md)：未解决项与历史撤回。
- [docs/细胞培养子任务覆盖矩阵.md](docs/细胞培养子任务覆盖矩阵.md)：18 步流程对照。
