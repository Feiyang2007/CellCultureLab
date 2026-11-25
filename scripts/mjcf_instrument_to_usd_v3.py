"""
MJCF → USD 仪器转换器 v3：**只吃 MuJoCo 真值 JSON，几何照搬局部量，自己不算层级矩阵**。

三版教训：
  v1 按 XML 层级建 USD → 刚体嵌套，PhysX 报 "unpredicted results"，可动件根本没进仿真（实测能驱动 0/20）；
  v2 自己合成层级矩阵压平 → 算错，零件炸开；
  v3：body 的世界位姿取 MuJoCo（xpos/xquat），几何取 MuJoCo 的**局部量**（geom_pos/geom_quat，
      本来就是相对所属 body 的），刚体一律平铺在 /Instrument/Bodies 下 —— 于是：
        · 不会有嵌套刚体（PhysX 的抱怨消失）；
        · 不需要我推矩阵（推错的机会归零）；
        · <replicate> 由 MuJoCo 自己展开（5430 转子孔位因此是 2004 个 geom，前两版一个都没带出来）。
  自检：用 body世界 ∘ 几何局部 反算出的世界位置，必须等于 MuJoCo 给的 geom_xpos（>1mm 直接判失败）。

用法（先跑真值导出）：
    C:/Users/17690/AppData/Local/Programs/Python/Python314/python.exe scripts/mujoco_dump_truth.py
    D:/IsaacSim5/env/Scripts/python.exe scripts/mjcf_instrument_to_usd_v3.py [名字过滤...]
产物：assets/instruments3/<名字>.usda
"""
import os, sys, math, json

R = r"D:/IsaacSim5/env/Lib/site-packages/isaacsim"
U = os.path.join(R, "extscache", "omni.usd.libs-1.0.1+69cbf6ad.wx64.r.cp311")
sys.path.insert(0, U)
for d in [os.path.join(U, "bin"), os.path.join(U, "bin", "deps"), os.path.join(R, "kit")]:
    os.add_dll_directory(d)
from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TRUTH = os.path.join(PROJ, "upstream", "truth")
OUT = os.path.join(PROJ, "assets", "instruments3")
os.makedirs(OUT, exist_ok=True)


# ---------------------------------------------------------------- 小代数工具
def qmat(q):
    """(w,x,y,z) → 旋转矩阵（Gf 行向量约定，与 USD 的 xformOp:transform 一致）"""
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    # 注意约定：Gf / USD 的 xformOp:transform 走**行向量**（p' = p·M），
    # 所以这里要放标准旋转矩阵的**转置**；写成标准形式会让所有几何转错方向
    # （自检抓到过 244~697 mm 的偏差）。
    m = Gf.Matrix4d()
    m[0][0] = 1 - 2 * (y * y + z * z); m[1][0] = 2 * (x * y - z * w); m[2][0] = 2 * (x * z + y * w)
    m[0][1] = 2 * (x * y + z * w); m[1][1] = 1 - 2 * (x * x + z * z); m[2][1] = 2 * (y * z - x * w)
    m[0][2] = 2 * (x * z - y * w); m[1][2] = 2 * (y * z + x * w); m[2][2] = 1 - 2 * (x * x + y * y)
    return m


def pose_mat(pos, quat):
    m = qmat(quat or [1, 0, 0, 0])
    m[0][3], m[1][3], m[2][3] = pos[0], pos[1], pos[2]
    return m


def rigid_inv(M):
    m = Gf.Matrix4d(M)
    r = [[m[i][j] for j in range(3)] for i in range(3)]
    t = (m[0][3], m[1][3], m[2][3])
    inv = Gf.Matrix4d()
    for i in range(3):
        for j in range(3):
            inv[i][j] = r[j][i]
        inv[i][3] = -(r[0][i] * t[0] + r[1][i] * t[1] + r[2][i] * t[2])
    return inv


def xpt(M, v):
    r = Gf.Matrix4d(M).Transform(Gf.Vec3d(float(v[0]), float(v[1]), float(v[2])))
    return (r[0], r[1], r[2])


def qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


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


def san(s):
    import re
    s = re.sub(r"[^0-9A-Za-z_]", "_", str(s or "x"))
    return ("_" + s) if s[:1].isdigit() else s


# ---------------------------------------------------------------- 转换
def build(name):
    fp = os.path.join(TRUTH, name + ".json")
    if not os.path.isfile(fp):
        return None, "无真值 JSON（MuJoCo 编译失败的多半是缺插件，见 mujoco_dump_truth.py 输出）"
    T = json.load(open(fp, encoding="utf-8"))

    stage = Usd.Stage.CreateNew(os.path.join(OUT, name + ".usda"))
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    top = UsdGeom.Xform.Define(stage, "/Instrument")
    stage.SetDefaultPrim(top.GetPrim())
    UsdGeom.Scope.Define(stage, "/Instrument/Bodies")
    UsdGeom.Scope.Define(stage, "/Instrument/Joints")

    world_of, prim_of, body_of = {}, {}, {}
    n_rb = 0
    for b in T["bodies"]:
        if b["index"] == 0:
            continue
        M = pose_mat(b["pos"], b["quat"])
        prim = stage.DefinePrim("/Instrument/Bodies/" + san(b["name"]), "Xform")
        set_transform(prim, M)
        UsdPhysics.RigidBodyAPI.Apply(prim)
        prim.CreateAttribute("physics:mass", Sdf.ValueTypeNames.Float).Set(
            float(b["mass"]) if b.get("mass", 0) > 1e-9 else 0.05)
        if not b["parent"]:
            prim.CreateAttribute("physics:kinematicEnabled", Sdf.ValueTypeNames.Bool).Set(True)
        world_of[b["name"]], prim_of[b["name"]], body_of[b["name"]] = M, str(prim.GetPath()), b
        n_rb += 1

    err_mm = 0.0
    n_col = n_vis = 0
    for g in T["geoms"]:
        bn = g["body"]
        if bn not in world_of:
            continue
        lp = g.get("local_pos") or [0, 0, 0]
        lq = g.get("local_quat") or [1, 0, 0, 0]
        # 自检：body世界 ∘ 局部 必须复现 MuJoCo 给的 geom 世界位置
        est = xpt(world_of[bn], lp)
        err_mm = max(err_mm, math.dist(est, g["world_pos"]) * 1000.0)
        is_col = int(g.get("group", 0)) not in (2, 5) and not (
            int(g.get("contype", 1)) == 0 and int(g.get("conaffinity", 1)) == 0)
        branch = "Collision" if is_col else "Visual"
        ppath = prim_of[bn] + "/" + branch + "/" + san("%s_%04d" % (g["name"], g["index"]))
        kind = {"box": "Box", "sphere": "Sphere", "cylinder": "Cylinder",
                "capsule": "Capsule", "mesh": "Mesh"}.get(g["type"])
        if kind is None:
            continue
        prim = stage.DefinePrim(ppath, kind)
        if kind == "Mesh":
            verts, faces = g.get("verts") or [], g.get("faces") or []
            if not verts or not faces:
                stage.RemovePrim(Sdf.Path(ppath))
                continue
            sc = g.get("mesh_scale") or [1, 1, 1]
            mesh = UsdGeom.Mesh(prim)
            mesh.CreatePointsAttr([Gf.Vec3f(v[0] * sc[0], v[1] * sc[1], v[2] * sc[2]) for v in verts])
            mesh.CreateFaceVertexCountsAttr([len(f) for f in faces])
            mesh.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
            xs = [v[0] * sc[0] for v in verts]; ys = [v[1] * sc[1] for v in verts]
            zs = [v[2] * sc[2] for v in verts]
            mesh.CreateExtentAttr([Gf.Vec3f(min(xs), min(ys), min(zs)),
                                  Gf.Vec3f(max(xs), max(ys), max(zs))])
            prim.CreateAttribute("displayColor", Sdf.ValueTypeNames.Color3fArray).Set(
                [Gf.Vec3f(0.74, 0.76, 0.78)])
            n_vis += 1
        else:
            size = list(g.get("size") or [0.01])
            while len(size) < 3:
                size.append(size[-1])
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
            n_col += 1 if is_col else 0
        # 局部变换照搬 MuJoCo 的 geom_pos / geom_quat（mesh 再套一层 mesh 自身偏移）
        off_p, off_q = lp, lq
        if kind == "Mesh":
            off_p = [lp[0] + (g.get("mesh_pos") or [0, 0, 0])[0],
                     lp[1] + (g.get("mesh_pos") or [0, 0, 0])[1],
                     lp[2] + (g.get("mesh_pos") or [0, 0, 0])[2]]
            off_q = qmul(lq, g.get("mesh_quat") or [1, 0, 0, 0])
        set_transform(prim, pose_mat(off_p, off_q))
        if is_col:
            UsdPhysics.CollisionAPI.Apply(prim)
            n_col += 1 if kind != "Mesh" else 1

    n_j = 0
    for j in T["joints"]:
        if j["type"] not in ("hinge", "slide"):
            continue
        bn, pn = j["body"], j["parent"]
        if bn not in prim_of or pn not in prim_of:
            continue
        kind = "RevoluteJoint" if j["type"] == "hinge" else "PrismaticJoint"
        jp = stage.DefinePrim("/Instrument/Joints/" + san(pn + "_" + j["name"]), kind)
        js = UsdPhysics.RevoluteJoint(jp) if kind == "RevoluteJoint" else UsdPhysics.PrismaticJoint(jp)
        js.CreateBody0Rel().SetTargets([Sdf.Path(prim_of[pn])])
        js.CreateBody1Rel().SetTargets([Sdf.Path(prim_of[bn])])
        jp.CreateAttribute("physics:localPos0", Sdf.ValueTypeNames.Point3f).Set(
            Gf.Vec3f(*xpt(rigid_inv(world_of[pn]), j["anchor"])))
        jp.CreateAttribute("physics:localPos1", Sdf.ValueTypeNames.Point3f).Set(
            Gf.Vec3f(*xpt(rigid_inv(world_of[bn]), j["anchor"])))
        a2 = [abs(j["axis"][0]), abs(j["axis"][1]), abs(j["axis"][2])]
        jp.CreateAttribute("physics:axis", Sdf.ValueTypeNames.Token).Set(
            "X" if a2[0] >= max(a2[1], a2[2]) else ("Y" if a2[1] >= a2[2] else "Z"))
        lo, hi = j["range"]
        if abs(lo) > 1e-9 or abs(hi) > 1e-9:
            if j["type"] == "hinge":
                lo, hi = math.degrees(lo), math.degrees(hi)
            jp.CreateAttribute("physics:lowerLimit", Sdf.ValueTypeNames.Float).Set(float(lo))
            jp.CreateAttribute("physics:upperLimit", Sdf.ValueTypeNames.Float).Set(float(hi))
        if j.get("stiffness"):
            pre = "angular" if kind == "RevoluteJoint" else "linear"
            jp.CreateAttribute("drive:%s:physics:type" % pre, Sdf.ValueTypeNames.Token).Set("force")
            jp.CreateAttribute("drive:%s:physics:stiffness" % pre,
                               Sdf.ValueTypeNames.Float).Set(float(j["stiffness"]))
            jp.CreateAttribute("drive:%s:physics:damping" % pre,
                               Sdf.ValueTypeNames.Float).Set(float(j.get("damping") or 0.001))
            jp.CreateAttribute("drive:%s:physics:targetPosition" % pre,
                               Sdf.ValueTypeNames.Float).Set(0.0)
        n_j += 1

    stage.GetRootLayer().Save()

    # 尺寸自检：整体包围盒要合理（>1.5 m 的台面上仪器直接标警告）
    st2 = Usd.Stage.Open(os.path.join(OUT, name + ".usda"))
    bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render], False)
    rng = bb.ComputeWorldBound(st2.GetDefaultPrim()).ComputeAlignedRange()
    dim = (0.0, 0.0, 0.0)
    if not rng.IsEmpty():
        a, b = rng.GetMin(), rng.GetMax()
        dim = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
    return {"rb": n_rb, "col": n_col, "vis": n_vis, "joint": n_j,
            "geom_total": len(T["geoms"]), "err_mm": err_mm, "dim": dim}, None


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    names = args or (sorted(os.path.splitext(f)[0] for f in os.listdir(TRUTH)
                            if f.endswith(".json")) if os.path.isdir(TRUTH) else [])
    print("=" * 100)
    print("真值：", TRUTH, " 输出：", os.path.relpath(OUT, PROJ))
    for nm in names:
        s, err = build(nm)
        if s is None:
            print("  ✘ %-34s %s" % (nm, err))
            continue
        flag = "✔" if s["err_mm"] < 1.0 else "✘ 自检未过"
        print("  %s %-32s 刚体%-3d 碰撞%-5d 可见%-5d 关节%-3d 上游geom %-5d 位置误差 %.3f mm  外形 %.3f×%.3f×%.3f"
              % (flag, nm, s["rb"], s["col"], s["vis"], s["joint"], s["geom_total"],
                 s["err_mm"], s["dim"][0], s["dim"][1], s["dim"][2]))


if __name__ == "__main__":
    main()
