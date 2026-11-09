"""Freeze a lab handoff with raw evidence and verifiable file identities."""
from pathlib import Path
from datetime import datetime, timezone
import argparse,collections,hashlib,json,os,shutil,subprocess,sys,zipfile

ROOT=Path(__file__).resolve().parents[1]
def sha(p):
 h=hashlib.sha256()
 with p.open('rb') as f:
  for block in iter(lambda:f.read(1048576),b''):h.update(block)
 return h.hexdigest()
def write(p,s):p.parent.mkdir(parents=True,exist_ok=True);p.write_text(s,encoding='utf8')

VERIFY=r'''from pathlib import Path
from datetime import datetime,timezone
import argparse,hashlib,importlib.util,json,sys
base=Path(__file__).resolve().parents[1]
def sha(p):
 h=hashlib.sha256()
 with p.open('rb') as f:
  for b in iter(lambda:f.read(1048576),b''):h.update(b)
 return h.hexdigest()
ap=argparse.ArgumentParser();ap.add_argument('--evidence',action='store_true');a=ap.parse_args()
m=json.loads((base/'MANIFEST.json').read_text(encoding='utf8'));bad=[]
for row in m['files']:
 p=base/row['path']
 if not p.is_file():bad.append({'path':row['path'],'reason':'missing'})
 elif p.stat().st_size!=row['bytes'] or sha(p)!=row['sha256']:bad.append({'path':row['path'],'reason':'hash_or_size_mismatch'})
out={'checked_at_utc':datetime.now(timezone.utc).isoformat(),'files_checked':len(m['files']),'hash_integrity_passed':not bad,'mismatches':bad,'physics_rerun':False}
if a.evidence:
 p=base/'工具/handoff_review.py';spec=importlib.util.spec_from_file_location('evidence_check',p);mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
 out['evidence']=mod.review(base/'工程快照/CellCultureLab')
folder=base/'_verification_runs';folder.mkdir(exist_ok=True);dest=folder/(datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.json')
dest.write_text(json.dumps(out,ensure_ascii=False,indent=2),encoding='utf8')
ok=out['hash_integrity_passed'] and out.get('evidence',{}).get('all_checks_passed',True)
print(json.dumps({'passed':ok,'files_checked':len(m['files']),'mismatches':len(bad),'physics_rerun':False,'report':str(dest)},ensure_ascii=False))
sys.exit(0 if ok else 1)
'''

RUNNER=r'''from pathlib import Path
import argparse,os,runpy,sys
ap=argparse.ArgumentParser();ap.add_argument('--project',type=Path,required=True);ap.add_argument('--script',required=True);a,rest=ap.parse_known_args()
root=a.project.resolve();script=(root/a.script).resolve()
if root not in script.parents:raise RuntimeError('Script must be inside working project.')
if '工程快照' in root.parts:raise RuntimeError('Run against the prepared working copy, not the immutable snapshot.')
sys.path.insert(0,str(Path(__file__).parent));from handoff_portable import usd_bootstrap
usd_bootstrap();sys.path.insert(0,str(root/'scripts'));os.chdir(root);sys.argv=[str(script),*rest]
runpy.run_path(str(script),run_name='__main__')
'''

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--destination',type=Path,required=True);a=ap.parse_args();base=a.destination.resolve()
 if base.exists():raise RuntimeError('Handoff destination already exists; choose a new path.')
 if base==ROOT or ROOT in base.parents:raise RuntimeError('Put handoff outside source project to prevent recursive copying.')
 base.mkdir(parents=True);snap=base/'工程快照/CellCultureLab';excluded=[]
 def ignore(folder,names):
  skipped=[]
  for name in names:
   p=Path(folder)/name
   if name in ('__pycache__','.git','.runtime','_archive','kit') or p.suffix.lower() in ('.pyc','.zip'):
    skipped.append(name);excluded.append(str(p.relative_to(ROOT)))
  return skipped
 shutil.copytree(ROOT,snap,ignore=ignore)
 vendor=base/'依赖/IsaacAssets5.1';source=Path.home()/'IsaacAssets/5.1'
 official=json.loads((ROOT/'outputs/gui_review/official_robot_dependencies.json').read_text(encoding='utf8'))
 official_checks=[]
 for row in official['files']:
  p=Path(row['file']);rel=p.relative_to(source);target=vendor/rel;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,target)
  official_checks.append({'relative':rel.as_posix(),'recorded_sha256':row['sha256'],'current_sha256':sha(p),'matches':sha(p)==row['sha256']})
 write(base/'留痕/官方机器人资源核对.json',json.dumps(official_checks,ensure_ascii=False,indent=2))
 write(base/'留痕/排除项.json',json.dumps({'excluded':excluded,'reason':'Caches, bytecode, previous generated zip, and an explicitly archived garbage/log folder; raw runtime reports and diagnostic failures are retained.'},ensure_ascii=False,indent=2))
 for n in ('handoff_review.py','handoff_portable.py'):shutil.copy2(ROOT/'scripts'/n,base/'工具'/n) if (base/'工具').exists() else ((base/'工具').mkdir(),shutil.copy2(ROOT/'scripts'/n,base/'工具'/n))
 write(base/'工具/verify_package.py',VERIFY);write(base/'工具/run_project.py',RUNNER)
 write(base/'00_先读我.md', '''# CellCultureLab 工作与环境交接

整理日期：2026-11-09。此包主要保存 2026-10-17 至 2026-10-20 阶段留下的工程、报告与原始记录；今天新增的是整理、哈希核对和离线复查，不是新增仿真实验。

## 先看哪几份

1. `01_工作总结.md`：可以转交老师的简要说明。
2. `02_证据索引.md`：每项说法对应的脚本、资产、报告、条件和边界。
3. `03_使用与复查.md`：先验哈希，再准备独立工作副本，最后按需打开或复跑。
4. `04_环境与依赖.md`：当前安装、历史参考环境、缺失项。
5. `05_留痕与限制.md`：哪些是原始材料，哪些是今天新整理的，能证明什么。

## 包内结构

```text
工程快照/CellCultureLab/   原工程快照；历史文件原字节保留
依赖/IsaacAssets5.1/       本机已有的17个官方机器人/夹爪依赖文件
工具/                     哈希校验、离线证据复查、路径迁移工具
留痕/                     本次复查、迁移、检查日志
MANIFEST.json             包内文件的大小、SHA-256和观察到的修改时间
```

快照不要直接用于写入式试验。迁移工具会另外复制出工作目录，只在副本中改路径，并逐项记录修改前后的哈希。历史结果保留原样，新运行结果单独输出。

## 一句话状态

完成六台设备的结构整理、21关节隔离验证，以及mini低速开合和C1000硬锁的单件试验；机械臂完整任务、整场景接触、视觉数据质量均未验收。此包是工程和实验记录交接，不是已完成的机器人任务产品。

## 原文档的旧口径

快照中的 README 和早期文档可能仍写“8吸头短测通过”等历史状态。当前统一口径见本包证据索引和 `工程快照/CellCultureLab/docs/手动接管状态_20261019.md`；早期材料不删除、不改成新结论。
''')
 write(base/'01_工作总结.md','''# 近期实验室仿真工作

在师兄已有的实验室场景和双机械臂布局上，继续做仪器模型整理和单件交互测试。

1. 整理六台独立仪器USD：Eppendorf 5430、5910 Ri、Tiangen TGear mini、Bio-Rad C1000、ThermoMixer C、Vortex-Genie 2。对照源模型核对2,180个几何元素，并保存构建与独立检查脚本。
2. 接通刚体姿态、关节位置和速度读取，对21个可动关节完成分批隔离测试。测试条件是零重力、关闭接触、施加测试驱动，检查运动方向和连接约束。
3. mini离心机完成MuJoCo 3.3.0八组指定参考协议对照；另外完成真实重力、240 Hz下的连续低速开合，记录1,140步，峰值角速度1.00067 rad/s，最大跟踪误差约0.096°，24项判据在保存报告中通过。
4. C1000完成独立硬锁功能试验，覆盖锁定受力、解锁开盖、误锁、回关和重新上锁。连续3,032步，9项功能判据通过，两次最小物理应用与一次完整应用保存的采样轨迹一致。

模型、源数据、脚本、日志与结果都随工程交接。2026-11-09又进行了文件身份和保存数据的离线复查，具体见证据索引。

目前完成的是设备模型与单件试验。机械臂操作设备的完整任务尚未跑通，整场景碰撞、接触与资源问题仍需继续处理。下一步先选择一个简单操作，把机器人动作、设备状态和任务判断接起来。
''')
 write(base/'02_证据索引.md','''# 说法与证据对应表

以下相对路径均以 `工程快照/CellCultureLab/` 为起点。报告是历史保存结果；本次离线复查只重新读取文件和计算部分指标，没有重新运行物理引擎。

| 编号 | 可陈述的工作 | 对应资产 / 方法 | 原始结果与范围 |
| --- | --- | --- | --- |
| E01 | 六台仪器、2,180个源几何元素核对 | `assets/instruments5/`；`scripts/mjcf_instrument_to_usd_v5.py`；`scripts/check_v5_independent.py`；源数据 `upstream/truth/` | `outputs/runtime_validation/v5_independent.json`：6项通过；几何、轴向、锚点和质量结构范围，不证明插装或机器人任务 |
| E02 | 21个关节完成隔离验证 | `scripts/check_actuation_direction.py`；`scripts/review_joint_constraints.py`；`scripts/runtime_joint_math.py` | `outputs/runtime_validation/v5_runtime_acceptance_20261019.json`：21/21；报告内含逐关节证据路径和资产哈希；CPU、禁接触、零重力、测试驱动 |
| E03 | mini八组指定参考协议对照 | `assets/instruments6_pilot/centrifuge_tiangen_tgear_mini.usda`；`scripts/compare_mini_physics.py`；`upstream/physics_reference/centrifuge_tiangen_tgear_mini.json` | `outputs/runtime_validation/mini_articulation_330_comparison.json`：8组行级判定通过，最大角差约0.000208°；原报告 `original_version_acceptance=false` 原样保留，不概括成完整原版本/全轨迹验收 |
| E04 | mini连续低速开合 | `scripts/run_mini_lid_task_v2.py`；`assets/scenes/mini_closed_loop_v2.usda`；`docs/低速开合修订2_20261019.md` | `outputs/runtime_validation/mini_lid_slow_real_cpu_rev2.json` 与同名 `.steps.jsonl`、`.log`；1,140步；CPU、240Hz、真实重力、力矩反馈；非机器人操作 |
| E05 | C1000硬锁功能试点 | `scripts/check_c1000_isaac_lock.py`；`scripts/instrument_behavior.py`；`outputs/runtime_validation/c1000_isaac_lock_pilot/asset.usda` 与 `scene.usda` | 同目录 `reviewed_summary.json`、`cycle_diagnostic.json`、`cycle_physics_only.json`、`cycle_physics_only_repeat.json` 及日志；3,032步、9项；硬锁近似、测试夹具输入，非源软锁动力学等价 |
| E06 | 本地机器人依赖与原场景查看层 | `assets/scenes/biolab_main_v06.usda`；`outputs/gui_review/biolab_v06_original_local.usda` | `outputs/gui_review/official_robot_dependencies.json`、`original_local_build.json`、`original_gui_observation.json`；本地化与显示范围，不证明碰撞布局和完整播放 |

## 今天重新核了什么

`留痕/原始证据复查.json` 中逐项记录：21个关节证据中的当前资产哈希；mini参考数据哈希；1,140步序号、有限性及前后状态连续；从全量记录重算峰值速度与力矩饱和步数；mini场景和资产哈希；runner与修订2文档的哈希前缀；C1000三个保存采样轨迹的逐项一致性、runner及资产哈希；v06源场景哈希。

24项与9项判据的“通过”来自原报告。本次只复算明确列出的指标，不宣称独立重做了所有判据，更没有重新运行仿真。

## 证据中需要一起说明的地方

- mini低速报告保留了通用 `suspicious_no_response` 和 `zero_response` 提示，其中包括目标为静止的 `hold_open`。该字段与保持段的专用角度、速度、补偿力矩判据并存。原文件不删除字段；复查时一起阅读，不能把通用提示改写成额外通过结论。
- mini逐步记录保存了场景与资产哈希，没有直接保存runner哈希；runner身份由现有修订2文档中的前16位哈希交叉核对，不冒充逐步文件本身有该字段。
- C1000比较的是三个产物里保存的采样轨迹；它们各运行3,032步，不代表每份文件都逐步保存了3,032条采样。
- mini的开合动图位于 `outputs/blog_20261020/images/`，为源几何与保存角度的离线回放，不是GUI录屏。设备总览统一配色，也不代表原材质验收。

## 当前没有完成的内容

机器人抓取、插管、移液等完整任务；AutoBio 12个任务键接入；5430单管插入验收；C1000源软锁完整等价；整场景GPU接触与稳定播放；真实材质/纹理及视觉采集；五类缺失工位。96精密吸头暂缓；8吸头有早期短测记录，但后续复跑被资源守卫中止，不能称当前稳定运行。
''')
 write(base/'03_使用与复查.md','''# 使用与复查

先解压到一个独立目录。不要修改“工程快照”，也不要把完整Isaac安装环境混进快照。

## 1. 验证包内文件与原始记录

在交接目录打开PowerShell，普通Python即可：

```powershell
python 工具/verify_package.py --evidence
```

它核对MANIFEST列出的每个文件和14项离线证据检查，结果另存 `_verification_runs/`，不覆盖原记录。可用 `D:/IsaacSim5/env/Scripts/python.exe` 替代python。失败时保留新报告，先核对缺失/哈希差异，不手改历史报告凑通过。

## 2. 生成可迁移的工作副本

这一步需要已安装Isaac Sim 5.1的Python，因为要使用随安装提供的USD库。将路径换成接收方实际安装位置：

```powershell
$isaacPython = 'D:/IsaacSim5/env/Scripts/python.exe'
& $isaacPython 工具/handoff_portable.py prepare --source 工程快照/CellCultureLab --dest 工作副本/CellCultureLab --external 依赖/IsaacAssets5.1 --official-manifest 工程快照/CellCultureLab/outputs/gui_review/official_robot_dependencies.json
```

目标目录若存在会直接停止，避免覆盖已有工作。工具只在新副本中把旧机器绝对路径和已缓存官方机器人URL改为相对路径，更新两份构建配置的路径和副本哈希，另存 `HANDOFF_RELOCATION.json`。原快照、原轨迹和原报告均不改。迁移后新资产的哈希不同是明确登记的派生结果，不能拿它替换原实验身份。

## 3. 离线检查资产路径

```powershell
& $isaacPython 工具/handoff_portable.py audit --project 工作副本/CellCultureLab --output _verification_runs/接收方资产路径.json
```

存在的外部URI和缺失项会保留在JSON里。USD文件存在检查不等于MDL着色器、视觉外观或仿真通过。

## 4. 查看场景（不自动播放）

```powershell
& $isaacPython 工具/run_project.py --project 工作副本/CellCultureLab --script scripts/open_scene.py outputs/gui_review/biolab_v06_original_local.usda
```

这是原材质查看层，仍需接收方机器完成渲染依赖加载。主场景存在已知碰撞与GPU容量/资源问题，打开不代表可以直接采集数据。要检查设备本体，也可在GUI打开 `assets/instruments5/` 下某台仪器。

## 5. 按需复跑单件试验

下面是复跑入口，不是本次已重新运行的声明。每次只启动一个物理进程，输出放新文件夹。不要覆盖历史报告。

```powershell
New-Item -ItemType Directory -Force 工作副本/CellCultureLab/outputs/handoff_new_runs
& $isaacPython 工具/run_project.py --project 工作副本/CellCultureLab --script scripts/run_mini_lid_task_v2.py --sequence lid_slow --gravity real --mode cpu --record-steps full --report outputs/handoff_new_runs/mini_slow.json
& $isaacPython 工具/run_project.py --project 工作副本/CellCultureLab --script scripts/compare_mini_physics.py --reference upstream/physics_reference/centrifuge_tiangen_tgear_mini.json --report outputs/handoff_new_runs/mini_330.json
& $isaacPython 工具/run_project.py --project 工作副本/CellCultureLab --script scripts/check_c1000_isaac_lock.py --case cycle --physics-only --output outputs/handoff_new_runs/c1000_cycle.json
```

mini构建元数据最初基于候选3.16.0生成，参考对照必须显式传入上面的3.3.0参考文件。C1000使用诊断桥参考和硬锁近似，不能当原Linux插件等价复现。

如要重新运行独立几何核对，可用run_project入口执行 `scripts/check_v5_independent.py`；它会在工作副本内写结果。运行后记录新环境、新输入、新日志、新资产哈希，不把新文件拼进原始证据链。
''')
 write(base/'04_环境与依赖.md','''# 环境与依赖

## 原机器当前查询（2026-11-09）

实际查询文件为 `留痕/当前解释器.json`：Windows；Python 3.11.9；Isaac Sim 5.1.0.0；numpy 1.26.0；psutil 5.9.8；matplotlib 3.10.3。路径为 `D:/IsaacSim5/env/Scripts/python.exe`。这些是当前解释器元数据查询，不是本次运行了物理测试。

## 历史MuJoCo参考环境

原记录 `outputs/runtime_validation/reference_environment_330.json` 记载曾用独立目录安装MuJoCo 3.3.0。2026-11-09检查发现该目录已不存在，当前Isaac解释器也查不到mujoco分发包。因此本包交付已保存的3.3.0参考数据、导出脚本与历史环境记录，不声称参考生成环境现在可直接使用。

要重新导出参考，需要单独恢复相应版本和依赖，再记录新的包版本与模型哈希。Linux `libmjlab.so.3.3.0` 不作为Windows原生库使用，C1000/涡旋桥接参考的范围见原专题报告。

## 本包包含与未包含

- 包含：工程资产、MJCF/OBJ等上游数据、测试脚本、原始日志与报告、文档、17个本机已有的官方机器人与夹爪依赖文件。
- 未包含：Isaac Sim安装、Python虚拟环境、显卡驱动、渲染缓存、已不存在的MuJoCo独立安装目录。
- 官方资源来源URL和历史SHA-256保存在 `outputs/gui_review/official_robot_dependencies.json`，本次逐项核对另存于留痕目录。
- 场景来自师兄已有工作；AutoBio和LabUtopia等上游数据继续保留原路径/来源记录；本包不把这些内容算作自主从零建模。当前快照未发现完整统一的第三方许可清单，交接给组内继续核对，不能据此宣称全部资产可任意公开发布。

## 路径与复现边界

历史日志保留旧机器的绝对路径，这是原记录的一部分。路径迁移只发生在新工作副本并有差异清单。文件能解析、哈希一致、数据连续，分别只证明对应层面的事实，不等于新机器上已重跑所有实验。
''')
 write(base/'05_留痕与限制.md','''# 工作留痕与证据边界

## 1. 哪些是以前留下的

工程快照中的源模型、构建/验证脚本、USD、原始JSON/JSONL、日志、专题说明及历史修订按当前源文件原字节复制。原来的失败记录、撤回说明和未验收项一起保留，不删掉不利结果，也不在原报告里改“通过”。

## 2. 哪些是今天新增的

2026-11-09新增交接说明、工作总结、证据索引、离线复查、文件清单与SHA-256、依赖和迁移工具。本次没有启动GUI或物理仿真，没有声称取得新实验结果。

原工程未建立可用于本项目逐次追溯的独立Git历史；Git探测指向了更上层的个人目录。没有把无关仓库记录拿来充当本项目历史，也没有倒填提交。

文件清单保存观察到的修改时间，但文件系统时间可改变，不作为可靠的历史日期或作者身份证明。当前哈希证明交接时的字节身份和传输完整性，不能单独证明是谁做的、何时第一次做的或物理结果绝对正确。压缩包旁的SHA-256还需通过独立渠道核对；未做可信时间戳或数字签名。

## 3. 本次复查的操作记录

- 原始证据复查：`留痕/原始证据复查.json`，14项，包含复算项与引用项的区别。
- 源文件复制核对：`留痕/源文件复制核对.json`，快照与打包前源文件逐项比对。
- 本机官方资源核对：`留痕/官方机器人资源核对.json`。
- 迁移工具本机检查：`留痕/路径迁移检查.json` 和相关日志；只检查打包路径修复、离线USD组合和结构检查，不替代动态仿真。
- 包完整性清单：`MANIFEST.json`；它不包含自己的哈希，压缩包外另提供完整ZIP的SHA-256。

## 4. 后续每次运行至少留下这些

```text
运行编号和真实开始/结束时间
使用的脚本及SHA-256
资产、场景、参考数据的SHA-256
解释器、引擎版本、CPU/GPU模式、步频、重力、接触设置
命令行和预先声明的判据
完整stdout/stderr、退出码、原始状态记录
判据结果、未完成项和异常
```

新运行另建目录，改脚本后换运行编号。没有跑到的写“未测/中断”，不写零；结论必须区分几何、隔离关节、单件物理与完整机器人任务。
''')
 # Keep reviewer-generated files outside the immutable original project tree as a clear layer.
 for n in ('原始证据复查.json','当前解释器.json'):
  shutil.copy2(ROOT/'outputs/handoff_20261109'/n,base/'留痕'/n)
 copied=[]
 for p in sorted(snap.rglob('*')):
  if p.is_file():
   rel=p.relative_to(snap);original=ROOT/rel;copied.append({'path':rel.as_posix(),'sha256':sha(p),'matches_source':sha(original)==sha(p)})
 write(base/'留痕/源文件复制核对.json',json.dumps({'all_match':all(x['matches_source'] for x in copied),'files':copied},ensure_ascii=False,indent=2))
 print(json.dumps({'package':str(base),'snapshot_files':len(copied),'official_files':len(official_checks),'copy_matches':all(x['matches_source'] for x in copied)},ensure_ascii=False))

if __name__=='__main__':main()
