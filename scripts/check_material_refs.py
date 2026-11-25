"""
静态检查两件事，都不需要开 Isaac：
  1 材质引用的贴图文件，能不能在工程里某个 USD 图层目录下解析到；
  2 material:binding 有没有指向不存在（越出引用域）的目标。

为什么要它：视口"能出图"不等于"材质加载好了"。启动器为了绕开材质编译崩溃关掉了
mdlMaterialWarmup 与同步加载，抓到的帧很可能是白模；而引用本身就有问题的，运行时一定是白的。
这类问题在开跑之前就能查出来，不必靠人眼看图猜。

注意：Usd.Stage.GetSourceLayer 在本机这个 pxr 构建里不存在（实测 AttributeError），
所以不靠它判归属图层，改用"工程内全部图层目录"做候选解析——宁可多算命中，也不误报缺失。

用法：
    D:/IsaacSim5/env/Scripts/python.exe scripts/check_material_refs.py [场景]
"""
import os, sys, collections, json
sys.stdout.reconfigure(encoding="utf-8")

ROOT = os.environ.get("ISAAC_PY_ROOT", r"D:/IsaacSim5/env/Lib/site-packages/isaacsim")
U = os.path.join(ROOT, "extscache", "omni.usd.libs-1.0.1+69cbf6ad.wx64.r.cp311")
sys.path.insert(0, U)
for d in [os.path.join(U, "bin"), os.path.join(U, "bin", "deps"), os.path.join(ROOT, "kit")]:
    os.add_dll_directory(d)
from pxr import Usd, UsdShade, Sdf

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
scene = os.path.abspath(sys.argv[1] if len(sys.argv) > 1
                        else os.path.join(PROJ, "assets", "scenes", "biolab_main_v04.usda"))

# 工程里所有含 USD 图层的目录，作为相对路径的解析锚点候选
LAYER_DIRS = set()
for dp, dn, fn in os.walk(PROJ):
    if any(f.lower().endswith((".usd", ".usda", ".usdc")) for f in fn):
        LAYER_DIRS.add(os.path.normpath(dp))

st = Usd.Stage.Open(scene)
if st is None:
    print("打不开：", scene)
    sys.exit(1)

REMOTE = ("http://", "https://", "package://", "isaacsim://", "omni://", "archive://")

ok, bad = 0, collections.Counter()
where = {}
for prim in st.TraverseAll():
    if not prim.IsA(UsdShade.Shader):
        continue
    sh = UsdShade.Shader(prim)
    for inp in sh.GetInputs():
        val = inp.Get()
        if not isinstance(val, Sdf.AssetPath) or not val.path:
            continue
        p = val.path
        if p.startswith(REMOTE):
            continue
        if os.path.isabs(p):
            hit = os.path.isfile(p)
        else:
            hit = any(os.path.isfile(os.path.normpath(os.path.join(d, p))) for d in LAYER_DIRS)
        if hit:
            ok += 1
        else:
            bad[p] += 1
            where.setdefault(p, []).append(str(prim.GetPath()))

# 越界/悬空材质绑定。
# 只查"目标存在不存在"是查不出越域的：/World/TubeCap_Looks/X 这个 prim 在合成阶段里确实存在，
# 但它对那条 binding 来说在引用域之外，USD 会直接忽略 —— 这种话只有 USD 自己会说。
# 所以起子进程强制遍历求值，父进程抓 stderr 里的 "outside the scope of the reference"。
if "--touch" in sys.argv:
    import re as _re
    n_mesh = 0
    n_unbound = 0
    for prim in st.TraverseAll():
        if str(prim.GetTypeName()) != "Mesh":
            continue
        n_mesh += 1
        rel = prim.GetRelationship("material:binding")
        if rel:
            targets=rel.GetTargets()  # 求值才会触发组合诊断
        else:
            targets=[]
        if not targets:
            n_unbound+=1
        st.GetPrimAtPath(str(prim.GetPath()))
    print("TOUCHED %d UNBOUND %d" % (n_mesh,n_unbound))
    sys.exit(0)

import subprocess, re
child = subprocess.run([sys.executable, os.path.abspath(__file__), scene, "--touch"],
                       capture_output=True, text=True, errors="replace")
scope_hits = re.findall(
    r"The relationship target <([^>]+)> from <([^>]+)\.[^>]*> in layer @([^@]+)@",
    child.stderr or "")
out_of_scope = collections.Counter()
for tgt, src, layer in scope_hits:
    out_of_scope[tgt] += 1
no_binding = 0
mesh_total = int(next((l.split()[1] for l in (child.stdout or "").splitlines()
                       if l.startswith("TOUCHED")), 0))
no_binding = int(next((l.split()[3] for l in (child.stdout or "").splitlines()
                       if l.startswith("TOUCHED") and "UNBOUND" in l), 0))

print("=" * 92)
print("场景：", scene)
print()
print("[1] 贴图引用：可解析 %d 条，**解析失败 %d 条（%d 个不同文件）**"
      % (ok, sum(bad.values()), len(bad)))
for p, n in bad.most_common(15):
    print("   x%-3d %s" % (n, p))
    for w in where[p][:2]:
        print("          被引用于 %s" % w)

print()
print("[2] 越出引用域、被 USD 忽略的材质绑定：共 %d 处（这些网格运行时一定没有材质）"
      % sum(out_of_scope.values()))
for t, n in out_of_scope.most_common(10):
    print("   x%-4d 目标 %s" % (n, t))
print("   网格总数 %d，其中没有直接材质绑定的 %d 个（可能继承绑定或使用 displayColor）" % (mesh_total, no_binding))

# ---------------- [3] .mdl / .mtl 里自己引用的贴图（USD 属性扫不到，得直接读源码） ----------------
import re as _re
mdl_bad = []
mdl_seen = 0
for dp, dn, fn in os.walk(PROJ):
    for f in fn:
        if not f.lower().endswith((".mdl", ".mtl")):
            continue
        fp = os.path.join(dp, f)
        try:
            txt = open(fp, encoding="utf-8", errors="replace").read()
        except Exception:
            continue
        for m in _re.findall(r'"([^"\\]+\.(?:png|jpg|jpeg|exr|tif|tga|hdr))"', txt, _re.I):
            if m.startswith(("http://", "https://", "mdl://", " texture", "usb:")):
                continue
            mdl_seen += 1
            cand = os.path.normpath(os.path.join(dp, m.lstrip("./")))
            alt = os.path.normpath(os.path.join(PROJ, m.lstrip("./")))
            if not (os.path.isfile(cand) or os.path.isfile(alt)):
                mdl_bad.append((os.path.relpath(fp, PROJ), m))
print()
print("[3] .mdl/.mtl 内部贴图引用：共 %d 条，缺失 %d 条" % (mdl_seen, len(mdl_bad)))
for src, m in mdl_bad[:12]:
    print("   %s  ->  %s" % (src, m))

total_bad = sum(bad.values()) + sum(out_of_scope.values()) + len(mdl_bad)

print()
print("引用完整性：[1]+[2]+[3] 合计 %d；这不证明内容真实，不能单凭归零开始视觉数据采集。" % total_bad)
manifest_path=os.path.join(PROJ,"assets","CONTENT_EXCEPTIONS.json")
used_layers={os.path.normcase(os.path.abspath(layer.realPath)) for layer in st.GetUsedLayers() if layer.realPath}
content_issues=[]
manifest_error=None
try:
    with open(manifest_path,encoding="utf-8") as handle:
        exceptions=json.load(handle)
    for entry in exceptions["entries"]:
        trigger=os.path.normcase(os.path.abspath(os.path.join(PROJ,entry["trigger_layer"])))
        used=trigger in used_layers
        if entry.get("trigger_prim"):
            used=used or bool(st.GetPrimAtPath(entry["trigger_prim"]))
        if used and entry["status"]!="verified":
            content_issues.append(entry)
except Exception as error:
    manifest_error=repr(error)
print("[4] 已知占位或来源未核实内容：%d 项" % len(content_issues))
for issue in content_issues:
    print("   "+issue["id"]+": "+issue["issue"])
if manifest_error:
    print("   内容例外清单读取失败："+manifest_error)
readiness=dict(scene=scene,reference_error_count=total_bad,unverified_content=content_issues,
    manifest_error=manifest_error,visual_data_ready=total_bad==0 and not content_issues and manifest_error is None,
    scope="reference integrity plus declared content exceptions; not photorealism or full visual authenticity validation")
print("MATERIAL_READINESS_JSON "+json.dumps(readiness,ensure_ascii=True),flush=True)
if "--report" in sys.argv:
    report_path=sys.argv[sys.argv.index("--report")+1]
    with open(report_path,"w",encoding="utf-8") as handle:
        json.dump(readiness,handle,ensure_ascii=False,indent=2)
sys.exit(0 if readiness["visual_data_ready"] else 2)
