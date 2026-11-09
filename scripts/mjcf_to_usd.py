"""
自写的 MJCF → USD 转换器（物体类：培养皿、离心管、冻存管、孔板、枪头盒、架）。

为什么不用 Isaac 官方 MJCF 导入器：它对这批物体反复硬崩（写 visual_tmp/*.tmp.usd 时
拿到空 stage，进程退出码 0x80000003），而且它不认 <replicate>。上游物体模型其实很简单：
一个 body + 若干图元碰撞（group="3" 的 cylinder/box/sphere）+ 一个视觉网格，自己转更可控。

对照上游写法（model/object/cell_dish_100.gen.xml）：
  <compiler meshdir="../../assets/"/>          ← 自己解析，不需要暂存或改写
  <mesh name=... file="container/.../visual.obj" scale="0.001 0.001 0.001"/>  ← obj 是毫米，必须乘 scale
  <geom type="cylinder" size="0.0445 0.001" group="3"/>   ← group 3 = 碰撞图元
  <geom type="mesh" contype="0" conaffinity="0" group="2"/> ← group 2 = 纯视觉

产物：assets/objects/<名字>.usda（文本格式，可 diff、可回滚）

用法：
    PY="D:/IsaacSim5/env/Scripts/python.exe"
    $PY scripts/mjcf_to_usd.py                 # 全部物体
    $PY scripts/mjcf_to_usd.py cell_dish pcr    # 只转名字含这些串的
"""
import os, re, sys, math, glob
import xml.etree.ElementTree as ET

ROOT = os.environ.get("ISAAC_PY_ROOT", r"D:/IsaacSim5/env/Lib/site-packages/isaacsim")
U = os.path.join(ROOT, "extscache", "omni.usd.libs-1.0.1+69cbf6ad.wx64.r.cp311")
sys.path.insert(0, U)
for d in [os.path.join(U, "bin"), os.path.join(U, "bin", "deps"), os.path.join(ROOT, "kit")]:
    os.add_dll_directory(d)
from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf, Tf

REPO = os.environ.get("AUTOBIO_REPO", r"C:/Users/17690/.codex/tmp-repos/AutoBio")
PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC_MODEL = os.path.join(REPO, "autobio", "model", "object")
OUT = os.path.join(PROJ, "assets", "objects")
os.makedirs(OUT, exist_ok=True)


# --------------------------------------------------------------- 网格读取
def parse_obj(path):
    pts, faces = [], []
    raw = open(path, encoding="utf-8", errors="replace").read().splitlines()
    for line in raw:
        if line.startswith("v "):
            p = line.split()[1:4]
            pts.append((float(p[0]), float(p[1]), float(p[2])))
        elif line.startswith("f "):
            idx = []
            for tok in line.split()[1:]:
                a = int(tok.split("/")[0])
                idx.append(a - 1 if a > 0 else len(pts) + a)   # obj 允许负索引
            faces.append(idx)
    return pts, faces


def parse_stl(path):
    """STL 两种写法上游都有：文本 solid... 与二进制（前 80 字节头 + 4 字节三角形数）。"""
    raw = open(path, "rb").read()
    head = raw[:80].lstrip()[:5].lower()
    if head == b"solid" and b"vertex" in raw[:2000].lower():
        pts, faces, tri = [], [], []
        for line in raw.decode("utf-8", errors="replace").splitlines():
            t = line.strip()
            if t.startswith("vertex"):
                v = [float(x) for x in t.split()[1:4]]
                pts.append(tuple(v)); tri.append(len(pts) - 1)
                if len(tri) == 3:
                    faces.append(tri); tri = []
        return pts, faces
    import struct
    n = struct.unpack("<I", raw[80:84])[0]
    pts, faces = [], []
    off = 84
    for _ in range(n):
        if off + 50 > len(raw):
            break
        # 12 float：法线 + 3 顶点，后面 2 字节属性忽略
        vals = struct.unpack("<12f", raw[off + 2 - 2:off + 50]) if False else             struct.unpack("<12f", raw[off:off + 48])[3:]
        base = len(pts)
        for k in range(3):
            v = vals[k * 3:k * 3 + 3]
            pts.append((v[0], v[1], v[2]))
        faces.append([base, base + 1, base + 2])
        off += 50
    return pts, faces


def load_mesh(path, scale):
    ext = os.path.splitext(path)[1].lower()
    if ext == ".obj":
        pts, faces = parse_obj(path)
    elif ext == ".stl":
        pts, faces = parse_stl(path)
    else:
        raise ValueError("不支持的网格格式：" + ext)
    s = scale or (1.0, 1.0, 1.0)
    pts = [(x * s[0], y * s[1], z * s[2]) for x, y, z in pts]
    return pts, faces


def load_root(path):
    """容错读 MJCF：个别上游文件的 <?xml ...?> 声明不规范（有 BOM/CRLF 干扰），
    MuJoCo 本来也不需要声明，直接剥掉再解析。"""
    txt = open(path, encoding="utf-8", errors="replace").read().lstrip("﻿")
    txt = re.sub(r"^\s*<\?xml[^>]*\?>", "", txt)
    return ET.fromstring(txt)


# --------------------------------------------------------------- 单个模型
def convert(xml_path, assets_root, out_usd):
    muj = load_root(xml_path)

    meshes = {}
    for m in muj.iter("mesh"):
        f = m.get("file")
        if not f:
            continue
        sc = m.get("scale")
        scale = tuple(float(x) for x in sc.split()) if sc else (1.0, 1.0, 1.0)
        meshes[m.get("name")] = (os.path.normpath(os.path.join(assets_root, f)), scale)

    mats = {m.get("name"): [float(x) for x in (m.get("rgba") or "0.8 0.8 0.8 1").split()]
            for m in muj.iter("material")}

    body = None
    wb = muj.find("worldbody")
    if wb is not None:
        body = wb.find("body")
    if body is None:
        return None, "没有 worldbody/body，可能只是被 include 的片段"

    stage = Usd.Stage.CreateNew(out_usd)
    obj = UsdGeom.Xform.Define(stage, "/Obj")
    stage.SetDefaultPrim(obj.GetPrim())
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    bp = body.get("pos") or "0 0 0"
    body_pos = [float(x) for x in bp.split()]
    UsdGeom.Xformable(obj).AddTranslateOp().Set(Gf.Vec3d(*body_pos))

    vis_c, col_c, skipped_frames = 0, 0, 0
    all_pts = []

    def emit_mesh(name, pts, faces, color, collide):
        nonlocal vis_c, col_c
        parent = "/Obj/Collision" if collide else "/Obj/Visual"
        path = parent + "/" + re.sub(r"[^0-9A-Za-z_]", "_", name)
        mesh = UsdGeom.Mesh.Define(stage, path)
        mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in pts])
        mesh.CreateFaceVertexCountsAttr([len(f) for f in faces])
        mesh.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
        xs = [q[0] for q in pts]; ys = [q[1] for q in pts]; zs = [q[2] for q in pts]
        mesh.CreateExtentAttr([Gf.Vec3f(min(xs), min(ys), min(zs)),
                               Gf.Vec3f(max(xs), max(ys), max(zs))])
        if collide:
            UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
            UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("convexHull")
            UsdGeom.Imageable(mesh.GetPrim()).CreateVisibilityAttr("invisible")
            # Dynamic bodies use explicit convex collision approximation.
            col_c += 1
        else:
            mesh.CreateDisplayColorAttr([Gf.Vec3f(*color[:3])])
            mesh.CreateDisplayOpacityAttr([color[3] if len(color) > 3 else 1.0])
            vis_c += 1
        return mesh

    # 视觉网格
    for g in body.iter("geom"):
        if g.get("type") != "mesh" or not g.get("mesh"):
            continue
        key = g.get("mesh")
        if key not in meshes:
            continue
        path, scale = meshes[key]
        if not os.path.isfile(path):
            print("     ! 上游缺网格文件，跳过该视觉件：", os.path.basename(path))
            continue
        pts, faces = load_mesh(path, scale)
        if not pts or not faces:
            print("     ! 网格解析为空，跳过：", os.path.basename(path))
            continue
        if g.get("pos"):
            off = [float(x) for x in g.get("pos").split()]
            pts = [(p[0] + off[0], p[1] + off[1], p[2] + off[2]) for p in pts]
        all_pts += pts
        col = mats.get(g.get("material"), [0.8, 0.8, 0.8, 1.0])
        emit_mesh(key, pts, faces, col, collide=False)

    # 碰撞：上游用图元（cylinder/box/sphere/capsule），比凸包更省更稳
    for g in body.iter("geom"):
        t = g.get("type")
        if t == "mesh":
            # 网格型碰撞：contype/conaffinity 非 0 才算
            if (g.get("contype") or "1") == "0" and (g.get("conaffinity") or "1") == "0":
                continue
            key = g.get("mesh")
            if key not in meshes:
                continue          # 上游引用写歪了的（个别文件 mesh 名带多余字符），跳过而不是整件崩
            path, scale = meshes[key]
            if not os.path.isfile(path):
                continue
            pts, faces = load_mesh(path, scale)
            emit_mesh(os.path.basename(path) + "_col", pts, faces, [0.5, 0.5, 0.5, 1], collide=True)
            continue
        if t not in ("box", "sphere", "cylinder", "capsule"):
            continue
        size = [float(x) for x in (g.get("size") or "0.01").split()]
        while len(size) < 3:
            size.append(size[-1])
        path = "/Obj/Collision/" + (g.get("name") or ("%s_%02d" % (t, col_c)))
        prim = stage.DefinePrim(path, t.capitalize() if t != "box" else "Cube")
        if t == "box":
            prim.CreateAttribute("extent", Sdf.ValueTypeNames.Normal3fArray).Set(
                [Gf.Vec3f(-size[0], -size[1], -size[2]), Gf.Vec3f(*size)])
            prim.CreateAttribute("size", Sdf.ValueTypeNames.Double).Set(2.0)
            prim.GetAttribute("extent").Set([Gf.Vec3f(-1), Gf.Vec3f(1)])
        else:
            prim.CreateAttribute("radius", Sdf.ValueTypeNames.Float).Set(size[0])
            if t in ("cylinder", "capsule"):
                prim.CreateAttribute("height", Sdf.ValueTypeNames.Float).Set(size[1] * 2)
                prim.CreateAttribute("axisLength", Sdf.ValueTypeNames.Float).Set(size[1] * 2)                    if t == "capsule" else None
        off = [float(x) for x in (g.get("pos") or "0 0 0").split()]
        xf = UsdGeom.Xformable(prim)
        if any(off):
            xf.AddTranslateOp().Set(Gf.Vec3d(*off))
        q = g.get("quat")
        if q:
            w, x, y, z = [float(v) for v in q.split()]
            xf.AddOrientOp().Set(Gf.Quatf(w, x, y, z))
        if t == "box":
            xf.AddScaleOp().Set(Gf.Vec3d(*size))
        UsdGeom.Imageable(prim).CreateVisibilityAttr("invisible")
        UsdPhysics.CollisionAPI.Apply(prim)
        col_c += 1

    skipped_frames = sum(1 for _ in body.iter("frame"))
    UsdPhysics.RigidBodyAPI.Apply(obj.GetPrim())
    m = body.get("mass")
    if m:
        obj.GetPrim().CreateAttribute("physics:mass", Sdf.ValueTypeNames.Float).Set(float(m))

    # 量一下成品尺寸，用于排版与常识校验
    if all_pts:
        xs = [p[0] for p in all_pts]; ys = [p[1] for p in all_pts]; zs = [p[2] for p in all_pts]
        dims = (max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs))
    else:
        dims = (0, 0, 0)
    return stage, dims, vis_c, col_c, skipped_frames


def main():
    filters = [a for a in sys.argv[1:] if not a.startswith("--")]
    xmls = sorted(glob.glob(os.path.join(SRC_MODEL, "*.xml")))
    stems = []
    for p in xmls:
        b = os.path.basename(p)
        if b.endswith(".gen.xml"):
            stems.append((b[:-8], p))
    for p in xmls:
        b = os.path.basename(p)
        if b.endswith(".xml") and not b.endswith(".gen.xml"):
            s = b[:-4]
            if not any(s == t[0] for t in stems):
                stems.append((s, p))
    # 有 .gen 就用 .gen（那是展开过的完整版）
    for p in xmls:
        b = os.path.basename(p)
        if b.endswith(".gen.xml"):
            s = b[:-8]
            stems = [(t[0], p) if t[0] == s else t for t in stems]

    print("=" * 92)
    print("源：", SRC_MODEL)
    print("出：", os.path.relpath(OUT, PROJ))
    print()
    rows = []
    for stem, path in sorted(stems):
        if filters and not any(f in stem for f in filters):
            continue
        xml_dir = os.path.dirname(path)
        root_el = load_root(path).find("compiler")
        meshdir = (root_el.get("meshdir") if root_el is not None else None) or "."
        assets_root = os.path.normpath(os.path.join(xml_dir, meshdir))
        out_usd = os.path.join(OUT, stem + ".usda")
        try:
            r = convert(path, assets_root, out_usd)
        except Exception as e:
            rows.append((stem, "异常：%s: %s" % (type(e).__name__, str(e)[:70]), 0, 0))
            print("  ✘ %-30s 异常：%s: %s" % (stem, type(e).__name__, str(e)[:70]))
            continue
        if r[0] is None:
            rows.append((stem, "跳过：" + r[1], 0, 0))
            continue
        stage, dims, vis, col, frames = r
        out = out_usd
        stage.GetRootLayer().Save()
        ok = "✔" if col >= 1 else "✘无碰撞"
        print("  %s %-28s 视觉%-3d 碰撞%-3d 外形 %.4f×%.4f×%.4f m  (frame 位 %d)"
              % (ok, stem, vis, col, dims[0], dims[1], dims[2], frames))
        rows.append((stem, "ok", vis, col))
    good = [r for r in rows if r[1] == "ok" and r[3] >= 1]
    print("\n成功 %d / %d；其中带碰撞体（能被夹爪碰到的前提）%d 个"
          % (len([r for r in rows if r[1] == "ok"]), len(rows), len(good)))
    for r in rows:
        if r[1] != "ok":
            print("   ✘ %-28s %s" % (r[0], r[1]))


if __name__ == "__main__":
    main()
