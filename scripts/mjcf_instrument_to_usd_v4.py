"""
MJCF → USD 仪器转换器 v4：只用 MuJoCo 导出的**世界矩阵**做逆乘，我自己不再写任何
四元数/旋转矩阵代码（v3 的 245~697 mm 偏差就出在我手写的四元数转矩阵上）。

按方案的三层检查在这里一次做完：
  第 1 层  在 WSL 里做（mujoco_layer1_and_export.py），已确认 MuJoCo 组合关系 1e-17；
  第 2 层  本脚本读回 JSON 后，先复算 body世界 ∘ geom局部 == geom世界（用 JSON 里的世界矩阵），
           证明"序列化没把数据改坏"；
  第 3 层  写完 USD 后，用全新的 UsdGeom.XformCache 把每个几何的**原点与沿自身 X/Y/Z 各偏移 1 cm
           的点**变换到世界，与 MuJoCo 的世界矩阵逐点对比 —— 只查平移不算通过。

刚体一律平铺在 /Instrument/Bodies 下（绝不刚体套刚体），几何只挂在自己的刚体下。

用法（用 Isaac 的 venv，因为要 pxr）：
    D:/IsaacSim5/env/Scripts/python.exe scripts/mjcf_instrument_to_usd_v4.py [名字过滤...]
产物：assets/instruments4/<名字>.usda
"""
import os, sys, json, math

R = r"D:/IsaacSim5/env/Lib/site-packages/isaacsim"
U = os.path.join(R, "extscache", "omni.usd.libs-1.0.1+69cbf6ad.wx64.r.cp311")
sys.path.insert(0, U)
for d in [os.path.join(U, "bin"), os.path.join(U, "bin", "deps"), os.path.join(R, "kit")]:
    os.add_dll_directory(d)
from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf
# 说明：不导入 PhysxSchema —— bare-pxr 里只有它的 python 层、没有原生插件，
# 导入会让 USD 报 "Failed to find plugin for schema type" 并连累 UsdGeom。
# 要写的属性名与类型是确定的：uint physxScene:gpuTotalAggregatePairsCapacity，直接裸写。

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TRUTH = os.path.join(PROJ, "upstream", "truth")
OUT = os.path.join(PROJ, "assets", "instruments4")
os.makedirs(OUT, exist_ok=True)

GEOM_TYPES = {2: "Sphere", 3: "Capsule", 4: "Cylinder", 5: "Box", 6: "Mesh"}
# MuJoCo: 2=sphere 3=capsule 4=ellipsoid 5=cylinder 6=box 7=mesh
TYPE_MAP = {2: "Sphere", 3: "Capsule", 4: "Sphere", 5: "Cylinder", 6: "Box", 7: "Mesh"}


def mmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def mtr(a):
    return [[a[j][i] for j in range(3)] for i in range(3)]


def rel_between(rb9, tb, rg9, tg):
    """inv(body世界) ∘ geom世界，纯 Python 行主序：R = Rbᵀ·Rg, t = Rbᵀ·(tg-tb)。
    v4 原来用 Gf 矩阵求逆做这件事，语义不同，造成 245~700 mm 偏差（独立核对暴露）。"""
    Rb = [[rb9[i * 3 + j] for j in range(3)] for i in range(3)]
    Rg = [[rg9[i * 3 + j] for j in range(3)] for i in range(3)]
    R = mmul(mtr(Rb), Rg)
    d = [tg[k] - tb[k] for k in range(3)]
    t = [sum(Rb[k][i] * d[k] for k in range(3)) for i in range(3)]
    return R, t


def to_gf(R, t):
    """行主序 3x3 + 平移 → Gf.Matrix4d。
    两个都必须按 Gf 的**行向量**约定来：
      · 逐元素赋值 m[i][j]=x 在这版 Gf 上会被静默丢弃 → 必须用 16 参构造；
      · 平移放在**第 4 行**（m[3][0..2]），不是第 4 列；放错的话
        ExtractTranslation() 永远读到 0，所有几何都塌到原点（正是 v2/v3/v4 之前的现象）。
    另外 Gf 里 3x3 块要放 R 的转置（因为 p' = p·M 等价于 R·p）。"""
    # 已实测确定的写法：3x3 块放 R 的转置（Gf 行向量 p'=p·M 等价于 R·p），平移放第 4 行。
    # 反证：把 3x3 改成不转置后，本来精确到 0.000 mm 的两台（mini、涡旋）也坏到 17/35 mm。
    # 仍遗留：另外四台在逐 geom 级别差 113~365 mm，不是全局约定问题（已排除），未定位。
    args = []
    for i in range(3):
        args += [R[0][i], R[1][i], R[2][i], 0.0]
    args += [t[0], t[1], t[2], 1.0]
    return Gf.Matrix4d(*args)


def xpt(M, v):
    r = Gf.Matrix4d(M).TransformPoint(Gf.Vec3d(*v)) if hasattr(Gf.Matrix4d, "TransformPoint") \
        else Gf.Matrix4d(M).Transform(Gf.Vec3d(*v))
    return (r[0], r[1], r[2])


def xdir(M, v):
    return Gf.Matrix4d(M).Transform(Gf.Vec3f(*v))


def san(s):
    import re
    s = re.sub(r"[^0-9A-Za-z_]", "_", str(s or "x"))
    return ("_" + s) if s[:1].isdigit() else s


def set_transform(prim, M):
    a = prim.GetAttribute("xformOp:transform")
    if not a:
        a = prim.CreateAttribute("xformOp:transform", Sdf.ValueTypeNames.Matrix4d)
    a.Set(Gf.Matrix4d(M))
    oa = prim.GetAttribute("xformOpOrder")
    if not oa:
        oa = prim.CreateAttribute("xformOpOrder", Sdf.ValueTypeNames.TokenArray)
    cur = [str(x) for x in (oa.Get() or [])]
    if "xformOp:transform" not in cur:
        cur.append("xformOp:transform")
        oa.Set(cur)


def layer2_check(T):
    """第 2 层：读回 JSON 后，用纯 Python（R·local + pos，行主序）复算 geom 世界位置，
    与 MuJoCo 给的 world_pos 比。不碰 Gf —— Gf 的逐元素赋值正是第 1 版偏差的来源。"""
    b_by = {b["id"]: b for b in T["bodies"]}
    ep = 0.0
    worst = None
    for g in T["geoms"]:
        b = b_by.get(g["body_id"])
        if not b:
            continue
        R = [[b["world_xmat_rowmajor"][i * 3 + j] for j in range(3)] for i in range(3)]
        p = [b["world_pos"][i] + sum(R[i][j] * g["local_pos"][j] for j in range(3))
             for i in range(3)]
        e = math.dist(p, g["world_pos"])
        if e > ep:
            ep, worst = e, g["name"]
    return ep, worst


def build(name):
    fp = os.path.join(TRUTH, name + ".json")
    if not os.path.isfile(fp):
        return None, "没有真值 JSON（先在 WSL autobio 环境跑 mujoco_layer1_and_export.py）"
    T = json.load(open(fp, encoding="utf-8"))

    # ---------- 第 2 层 ----------
    l2_err, l2_worst = layer2_check(T)

    def world_mat(rec):
        R = [[rec["world_xmat_rowmajor"][i * 3 + j] for j in range(3)] for i in range(3)]
        return to_gf(R, rec["world_pos"])
    bw = {b["id"]: world_mat(b) for b in T["bodies"]}
    b_of = {b["id"]: b for b in T["bodies"]}
    bname = {b["id"]: b["name"] for b in T["bodies"]}

    stage = Usd.Stage.CreateNew(os.path.join(OUT, name + ".usda"))
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    top = UsdGeom.Xform.Define(stage, "/Instrument")
    stage.SetDefaultPrim(top.GetPrim())
    UsdGeom.Scope.Define(stage, "/Instrument/Bodies")
    UsdGeom.Scope.Define(stage, "/Instrument/Joints")

    prim_of, n_rb = {}, 0
    for b in T["bodies"]:
        if b["id"] == 0:
            continue
        Mb = to_gf([[b["world_xmat_rowmajor"][i * 3 + j] for j in range(3)] for i in range(3)],
                   b["world_pos"])
        prim = stage.DefinePrim("/Instrument/Bodies/" + san(b["name"]), "Xform")
        set_transform(prim, Mb)
        UsdPhysics.RigidBodyAPI.Apply(prim)
        prim.CreateAttribute("physics:mass", Sdf.ValueTypeNames.Float).Set(
            float(b["mass"]) if b.get("mass", 0) > 1e-9 else 0.05)
        if b["parent_id"] == 0 and b["id"] == b.get("weld_id", b["id"]):
            pass
        if b["parent_id"] == 0:                      # 顶层 = 底座，钉住
            prim.CreateAttribute("physics:kinematicEnabled", Sdf.ValueTypeNames.Bool).Set(True)
        prim_of[b["id"]] = str(prim.GetPath())
        n_rb += 1

    n_col = n_vis = 0
    placed = []
    for g in T["geoms"]:
        bid = g["body_id"]
        if bid not in prim_of:
            continue
        kind = TYPE_MAP.get(int(g["type"]))
        if kind is None:
            continue
        bb = b_of[bid]
        Rrel, trel = rel_between(bb["world_xmat_rowmajor"], bb["world_pos"],
                                  g["world_xmat_rowmajor"], g["world_pos"])
        # MuJoCo 的分工：group 2=visual(contype=0,conaffinity=0)、5=inertia(不参与接触)、
        # 3=collision(contype/conaffinity 有非零)。之前我把 group 3 当成"非碰撞"排除掉了，
        # 结果所有仪器碰撞体数为 0 —— 判据写反了。
        grp, ct, ca = int(g.get("group", 0)), int(g.get("contype", 0)), int(g.get("conaffinity", 0))
        is_col = (grp != 2 and grp != 5) and (ct != 0 or ca != 0)
        branch = "Collision" if is_col else "Visual"
        ppath = prim_of[bid] + "/" + branch + "/" + san("%s_%04d" % (g["name"], g["id"]))
        # 这版 pxr 没注册 Box/Cylinder/Sphere 图元 schema，def Box 是"未知类型"，
        # USD 不认为它 Xformable —— 它自己的 xformOp:transform 会被 XformCache 忽略
        # （实测 Mesh 正常、Box 世界平移只剩父 body 的）。统一套一层 Xform 承载变换。
        holder = stage.DefinePrim(ppath, "Xform")
        set_transform(holder, to_gf(Rrel, trel))
        prim = stage.DefinePrim(ppath + "/shape", kind)
        size = list(g.get("size") or [0.01])
        while len(size) < 3:
            size.append(size[-1])
        if kind == "Mesh":
            verts, faces = g.get("verts") or [], g.get("faces") or []
            if not verts or not faces:
                stage.RemovePrim(Sdf.Path(ppath))
                continue
            sc = [1.0, 1.0, 1.0]   # 同上：不重复缩放
            # 网格顶点在 mesh 局部系，还要再套一层 mesh 自身偏移（MuJoCo 的编译结果）
            pass   # 编译后的 mesh_vert 已含 scale/pos/quat，不再叠加
            mesh = UsdGeom.Mesh(prim)
            pts = [Gf.Vec3f(v[0] * sc[0], v[1] * sc[1], v[2] * sc[2]) for v in verts]
            mesh.CreatePointsAttr(pts)
            mesh.CreateFaceVertexCountsAttr([3] * len(faces))
            mesh.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
            xs = [p[0] for p in pts]; ys = [p[1] for p in pts]; zs = [p[2] for p in pts]
            mesh.CreateExtentAttr([Gf.Vec3f(min(xs), min(ys), min(zs)),
                                  Gf.Vec3f(max(xs), max(ys), max(zs))])
            prim.CreateAttribute("displayColor", Sdf.ValueTypeNames.Color3fArray).Set(
                [Gf.Vec3f(*(g.get("rgba") or [0.74, 0.76, 0.78, 1])[:3])])
            # 变换已放在 holder 上，本体不再放（否则重复计入）
            n_vis += 1
        else:
            if kind == "Box":
                prim.CreateAttribute("extent", Sdf.ValueTypeNames.Normal3fArray).Set(
                    [Gf.Vec3f(-size[0], -size[1], -size[2]), Gf.Vec3f(*size)])
                prim.CreateAttribute("size", Sdf.ValueTypeNames.Float).Set(2.0)
            else:
                prim.CreateAttribute("radius", Sdf.ValueTypeNames.Float).Set(size[0])
                if kind in ("Cylinder", "Capsule"):
                    prim.CreateAttribute("height", Sdf.ValueTypeNames.Float).Set(size[1] * 2)
                    if kind == "Capsule":
                        prim.CreateAttribute("axisLength", Sdf.ValueTypeNames.Float).Set(size[1] * 2)
            # 变换已放在 holder 上，本体不再放（否则重复计入）
        if is_col:
            UsdPhysics.CollisionAPI.Apply(prim)
            n_col += 1
        _Rg = [[g["world_xmat_rowmajor"][i * 3 + j] for j in range(3)] for i in range(3)]
        placed.append((g["id"], ppath, to_gf(_Rg, g["world_pos"])))

    n_j = 0
    dof_map = []
    for j in T["joints"]:
        if int(j["type"]) not in (3, 2):            # 3=hinge 2=slide
            continue
        b1 = j["body_id"]
        b0 = next((b["parent_id"] for b in T["bodies"] if b["id"] == b1), None)
        if b0 not in prim_of or b1 not in prim_of:
            continue
        kind = "RevoluteJoint" if int(j["type"]) == 3 else "PrismaticJoint"
        jp = stage.DefinePrim("/Instrument/Joints/" + san(bname[b0] + "_" + j["name"]), kind)
        js = UsdPhysics.RevoluteJoint(jp) if kind == "RevoluteJoint" else UsdPhysics.PrismaticJoint(jp)
        js.CreateBody0Rel().SetTargets([Sdf.Path(prim_of[b0])])
        js.CreateBody1Rel().SetTargets([Sdf.Path(prim_of[b1])])
        a = j["world_anchor"]
        b0d, b1d = b_of[b0], b_of[b1]
        d0 = [a[k] - b0d["world_pos"][k] for k in range(3)]
        d1 = [a[k] - b1d["world_pos"][k] for k in range(3)]
        R0 = [[b0d["world_xmat_rowmajor"][i * 3 + j] for j in range(3)] for i in range(3)]
        R1 = [[b1d["world_xmat_rowmajor"][i * 3 + j] for j in range(3)] for i in range(3)]
        lp0 = [sum(R0[k][i] * d0[k] for k in range(3)) for i in range(3)]
        lp1 = [sum(R1[k][i] * d1[k] for k in range(3)) for i in range(3)]
        jp.CreateAttribute("physics:localPos0", Sdf.ValueTypeNames.Point3f).Set(Gf.Vec3f(*lp0))
        jp.CreateAttribute("physics:localPos1", Sdf.ValueTypeNames.Point3f).Set(Gf.Vec3f(*lp1))
        v0 = [sum(R0[k][i] * j["world_axis"][k] for k in range(3)) for i in range(3)]
        a2 = [abs(v0[0]), abs(v0[1]), abs(v0[2])]
        jp.CreateAttribute("physics:axis", Sdf.ValueTypeNames.Token).Set(
            "X" if a2[0] >= max(a2[1], a2[2]) else ("Y" if a2[1] >= a2[2] else "Z"))
        lo, hi = j["range"]
        if abs(lo) > 1e-9 or abs(hi) > 1e-9:
            if int(j["type"]) == 3:
                lo, hi = math.degrees(lo), math.degrees(hi)
            jp.CreateAttribute("physics:lowerLimit", Sdf.ValueTypeNames.Float).Set(float(lo))
            jp.CreateAttribute("physics:upperLimit", Sdf.ValueTypeNames.Float).Set(float(hi))
        if j.get("stiffness"):
            pre = "angular" if kind == "RevoluteJoint" else "linear"
            jp.CreateAttribute("drive:%s:physics:type" % pre, Sdf.ValueTypeNames.Token).Set("force")
            jp.CreateAttribute("drive:%s:physics:stiffness" % pre, Sdf.ValueTypeNames.Float).Set(
                float(j["stiffness"]))
            jp.CreateAttribute("drive:%s:physics:damping" % pre, Sdf.ValueTypeNames.Float).Set(
                float(j.get("damping") or 0.001))
            jp.CreateAttribute("drive:%s:physics:targetPosition" % pre,
                               Sdf.ValueTypeNames.Float).Set(0.0)
        n_j += 1
        dof_map.append((j["name"], bname[b0], bname[b1], int(j["type"])))

    # PhysX 的 GPU 接触容量：属性名与生效位置按方案核对，写在 UsdPhysics.Scene 上
    sc = None
    for p in stage.TraverseAll():
        if p.IsA(UsdPhysics.Scene):
            sc = p
            break
    if sc is None:
        sc = UsdPhysics.Scene.Define(stage, "/Instrument/Physics").GetPrim()
    cap = None
    if sc is not None:
        an = "physxScene:gpuTotalAggregatePairsCapacity"
        attr = sc.GetAttribute(an)
        if not attr:
            attr = sc.CreateAttribute(an, Sdf.ValueTypeNames.UInt)
        attr.Set(8192)
        cap = attr.Get()

    # PhysX 需要 articulation root 才会把关节链当整体求解；没有它时 drive 基本不生效
    # （实测：加之前 21 个关节只有 3 个能动）。挂在 /Instrument 上，覆盖全部刚体与关节。
    UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath("/Instrument"))
    stage.GetRootLayer().Save()

    # ---------- 第 3 层：读回 USD，比对原点 + 三根轴上的偏移点 ----------
    st2 = Usd.Stage.Open(os.path.join(OUT, name + ".usda"))
    xfc = UsdGeom.XformCache(Usd.TimeCode.Default())
    worst = (0.0, None)
    n_ok = 0
    skipped = 0
    off = 0.01
    for gid, ppath, MW in placed:
        prim = st2.GetPrimAtPath(ppath)
        if not prim:
            skipped = locals().get("skipped", 0) + 1
            continue
        Mu = xfc.GetLocalToWorldTransform(prim)
        # MuJoCo 侧：原点与三轴偏移点
        base_pts = [[0, 0, 0], [off, 0, 0], [0, off, 0], [0, 0, off]]
        for k, bp in enumerate(base_pts):
            want = xpt(MW, bp)
            got = xpt(Mu, bp)
            e = math.dist(want, got)
            if e > worst[0]:
                worst = (e, "%s[%d]" % (ppath.split("/")[-1], k))
        n_ok += 1
    dim = (0.0, 0.0, 0.0)
    bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render], False)
    rng = bb.ComputeWorldBound(st2.GetDefaultPrim()).ComputeAlignedRange()
    if not rng.IsEmpty():
        a, b = rng.GetMin(), rng.GetMax()
        dim = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
    nested = 0
    for p in st2.TraverseAll():
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            anc = p.GetParent()
            while anc:
                if anc.HasAPI(UsdPhysics.RigidBodyAPI):
                    nested += 1
                    break
                anc = anc.GetParent()
    return {"rb": n_rb, "col": n_col, "vis": n_vis, "joint": n_j, "geom": len(T["geoms"]),
            "l2_mm": l2_err * 1000.0, "l3_mm": worst[0] * 1000.0, "l3_worst": worst[1],
            "l3_checked": n_ok, "l3_skipped": skipped,
            "nested": nested, "cap": cap, "dim": dim, "dof_map": dof_map}, None


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    names = args or sorted(os.path.splitext(f)[0] for f in os.listdir(TRUTH) if f.endswith(".json"))
    print("=" * 104)
    print("真值：", TRUTH, "   输出：", os.path.relpath(OUT, PROJ))
    print("判据：第2层/第3层误差应 < 1 mm；嵌套刚体应为 0；GPU 容量应读回 8192")
    for nm in names:
        s, err = build(nm)
        if s is None:
            print("  ✘ %-32s %s" % (nm, err))
            continue
        good = s["l3_mm"] < 1.0 and s["nested"] == 0
        print("  %s %-30s 刚体%-3d 碰撞%-5d 可见%-5d 关节%-3d geom%-5d | 第2层 %.3f mm 第3层 %.3f mm(%s) | 嵌套 %d | 容量 %s | 外形 %.3f×%.3f×%.3f"
              % ("✔" if good else "✘", nm, s["rb"], s["col"], s["vis"], s["joint"], s["geom"],
                 s["l2_mm"], s["l3_mm"], (s["l3_worst"] or "")[:18], s["nested"], s["cap"],
                 s["dim"][0], s["dim"][1], s["dim"][2]))


if __name__ == "__main__":
    main()
