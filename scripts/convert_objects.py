"""
把 AutoBio 上游"物体类"资产（培养皿、离心管、冻存管、孔板、枪头盒、架）转成可抓取的 USD。

与 convert_instruments.py 的唯一实质差别：**fix_base = False**（耗材不能被钉住）。

三处必要的设计（都是踩出来的）：
  1. XML 一律落到 upstream/mjcf/（和仪器同一目录）。换个目录会让导入器把
     "../assets/..." 解析错，它在网格旁边写 visual_tmp/*.tmp.usd 时拿到空 stage 会**硬崩**。
  2. 每个物体单独起一个进程导入。一个崩了不会带走整批。
  3. 引用命中不了的直接跳过并报告（上游 model/object/centrifuge_1-5ml.xml 与
     centrifuge_15ml.xml 就指向了不存在的路径，属上游缺陷，不去硬凑）。

用法：
    PY="D:/IsaacSim5/env/Scripts/python.exe"
    $PY scripts/convert_objects.py                    # 批量（内部逐个子进程）
    $PY scripts/convert_objects.py --one cryovial_1-8ml   # 只转一个（子进程自己用）
产物：assets/objects/<名字>.usd（+ configuration/ 四件套）
"""
import os, re, sys, glob, json, shutil, subprocess

REPO = os.environ.get("AUTOBIO_REPO", r"C:/Users/17690/.codex/tmp-repos/AutoBio")
PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC_MODEL = os.path.join(REPO, "autobio", "model", "object")
SRC_ASSETS = os.path.join(REPO, "autobio", "assets")
STAGE = os.path.join(PROJ, "upstream", "mjcf")          # 与仪器共用，别换目录
# 导入器会把 visual_tmp/*.tmp.usd 写到"输出目录的上一级 + 资产相对路径"处，
# 所以输出必须和 upstream/assets 在同一棵子树里，否则路径自相矛盾 → 拿到空 stage 硬崩。
OUT = os.environ.get("OBJ_OUT", os.path.join(PROJ, "upstream", "assets", "objects"))
PREFIX = re.compile(r'file="((?:instrument|container|rack|tool|robot|common|misc)/)')
MB = lambda n: "%.1f MB" % (n / 1e6)


# ============================================================ 子进程：转一个
def convert_one(stem):
    os.makedirs(OUT, exist_ok=True)
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})
    import omni.usd
    from pxr import Usd, UsdGeom, UsdPhysics
    from isaacsim.asset.importer.mjcf import _mjcf

    xml = os.path.join(STAGE, stem + ".xml")
    dest = os.path.join(OUT, stem + ".usd").replace("\\", "/")
    omni.usd.get_context().new_stage()
    cfg = _mjcf.ImportConfig()
    cfg.fix_base = False
    cfg.merge_fixed_joints = False
    _mjcf.acquire_mjcf_interface().create_asset_mjcf(xml.replace("\\", "/"), "/Obj", cfg, dest)
    app.close()

    # 数字要分别看两个文件：网格/碰撞在 _physics.usd，外形尺寸看外层
    phys = os.path.join(OUT, "configuration", stem + "_physics.usd")
    from pxr import Usd as U2, UsdGeom as G2, UsdPhysics as P2
    probe = U2.Stage.Open(phys if os.path.isfile(phys) else dest)
    prims = list(probe.TraverseAll())
    rb = sum(1 for p in prims if p.HasAPI(P2.RigidBodyAPI))
    co = sum(1 for p in prims if p.HasAPI(P2.CollisionAPI))
    me = sum(1 for p in prims if str(p.GetTypeName()) == "Mesh")
    mass = None
    for p in prims:
        a = p.GetAttribute("physics:mass")
        if a and a.HasAuthoredValue():
            mass = a.Get(); break
    top = U2.Stage.Open(dest)
    root = top.GetDefaultPrim()
    from pxr import UsdTimeCode
    bb = G2.BBoxCache(UsdTimeCode.Default(), [G2.Tokens.default_, G2.Tokens.render], False)
    rng = bb.ComputeWorldBound(root).ComputeAlignedRange() if root else None
    size = "尺寸未知"
    if rng and not rng.IsEmpty():
        a, b = rng.GetMin(), rng.GetMax()
        size = "外形 %.4f×%.4f×%.4f m" % (b[0] - a[0], b[1] - a[1], b[2] - a[2])
    print("RESULT " + json.dumps({"stem": stem, "rb": rb, "co": co, "mesh": me,
                                  "mass": mass, "size": size}, ensure_ascii=False))


# ============================================================ 父进程：暂存 + 逐个跑
def stage_all():
    os.makedirs(STAGE, exist_ok=True)
    xmls = sorted(glob.glob(os.path.join(SRC_MODEL, "*.xml")))
    # 先把同目录所有 xml 原样落一份，保证彼此引用得到
    for p in xmls:
        shutil.copy2(p, os.path.join(STAGE, os.path.basename(p)))
    stems, skipped = [], []
    for p in xmls:
        base = os.path.basename(p)
        stem = base.replace(".gen.xml", "").replace(".xml", "")
        if "-simple" in stem or stem in [t[0] for t in stems]:
            continue
        gen = os.path.join(STAGE, stem + ".gen.xml")
        plain = os.path.join(STAGE, stem + ".xml")
        use_gen = os.path.isfile(gen)
        src = gen if use_gen else plain
        txt = open(src, encoding="utf-8").read()
        n = len(PREFIX.findall(txt))
        txt = PREFIX.sub(r'file="../assets/\1', txt)
        txt = re.sub(r'\s*assetdir="[^"]*"', "", txt)
        out = os.path.join(STAGE, stem + ".xml")
        open(out, "w", encoding="utf-8", newline="\n").write(txt)
        miss = [m for m in re.findall(r'file="([^"]+)"', txt)
                if m.endswith((".obj", ".stl", ".STL", ".png", ".dae"))
                and not os.path.isfile(os.path.normpath(os.path.join(STAGE, m)))]
        if miss:
            skipped.append((stem, miss))
            continue
        stems.append((stem, "gen" if use_gen else "plain", n))
    return stems, skipped


if "--one" in sys.argv:
    convert_one(sys.argv[sys.argv.index("--one") + 1])
    sys.exit(0)

print("=" * 90)
stems, skipped = stage_all()
print("暂存到 %s（与仪器同目录）" % os.path.relpath(STAGE, PROJ))
print("可转物体 %d 个；上游引用本身断链跳过 %d 个" % (len(stems), len(skipped)))
for s, miss in skipped:
    print("   [跳过] %-28s 指向上游不存在的路径：%s" % (s, miss[0]))
print()

results = []
for stem, kind, nref in stems:
    print("[转] %-30s (取 %s, 引用 %d 条)" % (stem, kind, nref), flush=True)
    r = subprocess.run([sys.executable, os.path.abspath(__file__), "--one", stem],
                       capture_output=True, text=True, errors="replace")
    line = [l for l in (r.stdout or "").splitlines() if l.startswith("RESULT ")]
    if line:
        d = json.loads(line[-1][7:])
        results.append(d)
        ok = "✔" if d["rb"] >= 1 and d["co"] >= 1 else "✘"
        print("    %s 刚体%-3d 碰撞%-3d 网格%-4d 质量%-8s %s"
              % (ok, d["rb"], d["co"], d["mesh"],
                 ("%.4g kg" % d["mass"]) if d["mass"] else "?", d["size"]))
    else:
        results.append({"stem": stem, "rb": -1, "co": -1, "mesh": -1, "mass": None,
                        "size": "崩溃/无输出"})
        err = [l for l in (r.stderr or "").splitlines()
               if "Failed to open layer" in l or "Fatal" in l or "MJCF asset convert failed" in l]
        print("    ✘ 进程退出码 %d" % r.returncode, ("| " + err[0][:110]) if err else "")

print("\n" + "=" * 90)
good = [d for d in results if d["rb"] >= 1 and d["co"] >= 1]
print("成功 %d / %d；可用（有刚体又有碰撞，才谈得上抓取）：" % (len(good), len(results)))
for d in sorted(good, key=lambda x: x["stem"]):
    print("   %-30s 刚体%-3d 碰撞%-3d %s" % (d["stem"], d["rb"], d["co"], d["size"]))
bad = [d for d in results if d not in good]
if bad:
    print("没通过的：", ", ".join(d["stem"] for d in bad))
print("产物目录：", os.path.relpath(OUT, PROJ), " 体积 ", MB(sum(
    os.path.getsize(os.path.join(dp, f)) for dp, _, fs in os.walk(OUT) for f in fs)))
