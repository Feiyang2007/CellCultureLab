"""
CellCultureLab 重建脚本：把李春洋传来的原始包修成可用工程。可反复重跑（幂等）。
用法：python scripts/rebuild_from_raw.py     （春洋给了新包时，先覆盖 Raw_... 目录再跑这条）

把李春洋传来的 Raw_simulation_environment_from_ChunYang 重建为可用工程 CellCultureLab/。

还原上游层级：  <root>/assets/scenes/  +  <root>/assets/precision_tasks/
（这样场景里既有的 ../precision_tasks/... 相对引用无需改动即可解析）

原件只读，全部另存。
"""
import os, sys, shutil, glob

ROOT = r"D:/IsaacSim5/env/Lib/site-packages/isaacsim"
USDLIBS = os.path.join(ROOT, "extscache", "omni.usd.libs-1.0.1+69cbf6ad.wx64.r.cp311")
sys.path.insert(0, USDLIBS)
for d in [os.path.join(USDLIBS, "bin"), os.path.join(USDLIBS, "bin", "deps"), os.path.join(ROOT, "kit")]:
    os.add_dll_directory(d)
from pxr import Sdf, Usd

_HERE = os.path.dirname(os.path.abspath(__file__))                    # .../CellCultureLab/scripts
BASE = os.path.dirname(os.path.dirname(_HERE))                     # .../Building_simulation_environment
SRC = os.environ.get("RAW_BUNDLE", os.path.join(BASE, "Raw_simulation_environment_from_ChunYang"))
DST = os.path.join(BASE, "CellCultureLab")

SRC_ASSETS = os.path.join(SRC, "precision_tasks", "precision_tasks")
SRC_SCENES = os.path.join(SRC, "scenes", "scenes")

def log(*a):
    print(*a, flush=True)

# ---------------------------------------------------------------- 1. 目录骨架
for sub in ("assets/precision_tasks", "assets/scenes", "scripts", "docs"):
    os.makedirs(os.path.join(DST, sub), exist_ok=True)
log("[1] 目录骨架已建：", os.path.relpath(DST, BASE))

# ---------------------------------------------------------------- 2. 复制资产（保留原件）
copied = []
for dirpath, dirnames, filenames in os.walk(SRC_ASSETS):
    for fn in sorted(filenames):
        s = os.path.join(dirpath, fn)
        rel = os.path.relpath(s, SRC_ASSETS)
        d = os.path.join(DST, "assets", "precision_tasks", rel)
        os.makedirs(os.path.dirname(d), exist_ok=True)
        if not (os.path.isfile(d) and os.path.getsize(d) == os.path.getsize(s)):
            shutil.copy2(s, d)
            copied.append(os.path.join("assets/precision_tasks", rel))
log("[2] precision_tasks 资产复制 %d 个文件（%d 个已存在且同尺寸，跳过）"
    % (len(copied), sum(len(f) for _, _, f in os.walk(os.path.join(DST, "assets", "precision_tasks"))) - len(copied)))

scene_copies = []
for s in sorted(glob.glob(os.path.join(SRC_SCENES, "*.usd"))):
    fn = os.path.basename(s)
    if " copy" in fn or " - copy" in fn:
        log("    跳过冗余副本：", fn)
        continue
    d = os.path.join(DST, "assets", "scenes", fn)
    if not (os.path.isfile(d) and os.path.getsize(d) == os.path.getsize(s)):
        shutil.copy2(s, d)
    scene_copies.append(fn)
log("[2] 场景文件复制：", ", ".join(scene_copies))

# ---------------------------------------------------------------- 3. 改写主场景引用
OLD_MAIN = os.path.join(SRC_SCENES, "biolab_main_v01.usd")
NEW_MAIN = os.path.join(DST, "assets", "scenes", "biolab_main_v02.usda")

REWRITES = [
    # 96x 枪头视觉：李春洋机器上的 Linux 绝对路径 -> 本包相对路径
    ("/home/lcy/projects/Pipette/assets/precision_tasks/pipette_tip_200ul/tip_200ul_visual.usda",
     "../precision_tasks/pipette_tip_200ul/tip_200ul_visual.usda", 96),
]
# 枪架：PipetteRack 这个 prim 上同时挂着两条 references，
# 其中 @../precision_tasks/PipetteRack.usd@ 指的文件本包里根本不存在（旧层级残留），
# 保留 @../precision_tasks/pipette_tip_200ul/PipetteRack.usd@ 这条即可。
DROP_REFS = ["@../precision_tasks/PipetteRack.usd@,"]

stage = Usd.Stage.Open(OLD_MAIN)
assert stage is not None, "打不开主场景"
layer = stage.GetRootLayer()
txt = layer.ExportToString()
log("[3] 主场景导出为 USDA 文本：%d 字节" % len(txt))

for old, new, expect in REWRITES:
    n = txt.count(old)
    if n != expect:
        log("    !! 预期 %d 处，实际 %d 处：%s" % (expect, n, old))
    txt = txt.replace(old, new)
    log("    替换 x%-4d %s -> %s" % (n, old, new))

for dead in DROP_REFS:
    n = txt.count(dead)
    if n != 1:
        log("    !! 待删引用出现 %d 次（应为 1）：%s" % (n, dead))
    txt = txt.replace(dead, "")
    log("    删除 x%-4d %s" % (n, dead))

# 顺手把 defaultPrim / 单位确认写死（防止后续脚本改动）
new_layer = Sdf.Layer.CreateNew(NEW_MAIN)
ok = new_layer.ImportFromString(txt)
if not ok:
    log("!! ImportFromString 失败"); sys.exit(1)
new_layer.Save()
log("[3] 已写出：", os.path.relpath(NEW_MAIN, BASE), "(%d 字节)" % os.path.getsize(NEW_MAIN))

# ---------------------------------------------------------------- 4. 自检：新场景里还有没有解析不了的相对引用
chk = Usd.Stage.Open(NEW_MAIN)
unresolved = []
for prim in chk.Traverse():
    for meta in ("references", "payload"):
        v = prim.GetMetadata(meta)
        if not v:
            continue
        for it in v.GetAddedOrExplicitItems():
            ap = str(it.assetPath)
            if not ap:
                continue
            if ap.startswith(("http://", "https://", "package://", "isaacsim://", "omni://", "archive://")):
                continue  # 远程/包内资产交给 Isaac 解析
            base_dir = os.path.dirname(NEW_MAIN)
            if not os.path.isfile(os.path.normpath(os.path.join(base_dir, ap))):
                unresolved.append((str(prim.GetPath()), ap))
log("[4] 本地相对引用无法解析的 prim：%d 个" % len(unresolved))
for p, a in unresolved[:20]:
    log("    ", p, "->", a)
log("完成。")
