"""MuJoCo truth -> USD v5; preserves v4 assets for comparison.

Uses official PhysicsJoint types, DriveAPI/MassAPI, signed aligned joint frames,
Cube dimensions, convex mesh collisions, and fixed joints for welded bodies.
The host supplies one PhysicsScene; no articulation root is needed for these
maximal-coordinate joints with kinematic bases. Export only to instruments5/.

Geometry and frames are independently checked by check_v5_independent.py.
Runtime direction checks are separate. MuJoCo plugins, armature/frictionloss,
exact inertia/COM and C1000 lock behavior are not reproduced by this converter.
"""
import os, sys, json, math
from runtime_joint_math import quat_rotate
sys.stdout.reconfigure(encoding="utf-8")

R = r"D:/IsaacSim5/env/Lib/site-packages/isaacsim"
U = os.path.join(R, "extscache", "omni.usd.libs-1.0.1+69cbf6ad.wx64.r.cp311")
sys.path.insert(0, U)
for d in [os.path.join(U, "bin"), os.path.join(U, "bin", "deps"), os.path.join(R, "kit")]:
    os.add_dll_directory(d)
from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf
# Asset conversion uses bare USD; runtime scene configuration is a separate step.

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TRUTH = os.path.join(PROJ, "upstream", "truth")
OUT = os.path.join(PROJ, "assets", "instruments5")
os.makedirs(OUT, exist_ok=True)

# MuJoCo: 2=sphere 3=capsule 4=ellipsoid 5=cylinder 6=box 7=mesh
TYPE_MAP = {0: "Plane", 2: "Sphere", 3: "Capsule", 4: "Sphere", 5: "Cylinder", 6: "Box", 7: "Mesh"}


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

    stage = Usd.Stage.CreateInMemory()
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
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr().Set(
            float(b["mass"]) if b.get("mass", 0) > 1e-9 else 0.05)
        if b["parent_id"] == 0 and b["id"] == b.get("weld_id", b["id"]):
            pass
        if b["parent_id"] == 0:                      # 顶层 = 底座，钉住
            prim.CreateAttribute("physics:kinematicEnabled", Sdf.ValueTypeNames.Bool).Set(True)
        prim_of[b["id"]] = str(prim.GetPath())
        n_rb += 1

    # Multiple MuJoCo joints on one body are an ordered serial chain. Two
    # parallel USD hinges between the same bodies would collapse those DOFs.
    body_joints = {}
    for joint in T["joints"]:
        if int(joint["type"]) in (2,3):
            body_joints.setdefault(joint["body_id"],[]).append(joint)
    chains = {}
    auxiliary_records = {}
    for bid, records in body_joints.items():
        parent = b_of[bid]["parent_id"]
        if parent not in prim_of:
            continue
        links = [prim_of[parent]]
        if len(records) > 1:
            body = b_of[bid]
            total_mass = max(float(body["mass"]),0.05 if body["mass"]<=1e-9 else 0)
            # Keep total mass; split 1% onto the auxiliary link(s). The source
            # export lacks inertia/COM, so their proxy inertia is explicit.
            UsdPhysics.MassAPI(stage.GetPrimAtPath(prim_of[bid])).GetMassAttr().Set(total_mass*.99)
            for index in range(len(records)-1):
                path = "/Instrument/Bodies/__dof_%d_%d" % (bid,index)
                p = stage.DefinePrim(path,"Xform")
                p.SetCustomDataByKey("sourceBodyId",bid)
                p.SetCustomDataByKey("sourceJointId",int(records[index]["id"]))
                p.SetCustomDataByKey("auxiliaryDofLink",True)
                joint_q = Gf.Rotation(Gf.Vec3d(1,0,0),Gf.Vec3d(*records[index]["world_axis"])).GetQuat()
                xyzw = [*joint_q.GetImaginary(),joint_q.GetReal()]
                columns = [quat_rotate(xyzw,axis) for axis in ([1,0,0],[0,1,0],[0,0,1])]
                rotation = [[columns[j][i] for j in range(3)] for i in range(3)]
                anchor = records[index]["world_anchor"]
                set_transform(p,to_gf(rotation,anchor))
                auxiliary_records[path] = dict(world_pos=anchor,
                    world_quat_wxyz=[joint_q.GetReal(),*joint_q.GetImaginary()],
                    world_xmat_rowmajor=[v for row in rotation for v in row])
                UsdPhysics.RigidBodyAPI.Apply(p)
                mass = total_mass*.01/(len(records)-1)
                api = UsdPhysics.MassAPI.Apply(p)
                api.CreateMassAttr(mass)
                epsilon = max(mass*.0001,1e-8)
                armature = float(records[index].get("armature") or 0.0)
                api.CreateDiagonalInertiaAttr(Gf.Vec3f(max(armature,epsilon),epsilon,epsilon))
                api.CreatePrincipalAxesAttr(Gf.Quatf(1))
                api.CreateCenterOfMassAttr(Gf.Vec3f(0))
                links.append(path)
                n_rb += 1
        links.append(prim_of[bid])
        chains[bid] = links

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
        # A holder keeps the original geom frame independent of shape scaling.
        # USD's valid box type is Cube; Cylinder and Sphere are registered types.
        holder = stage.DefinePrim(ppath, "Xform")
        set_transform(holder, to_gf(Rrel, trel))
        prim = stage.DefinePrim(ppath + "/shape", "Cube" if kind == "Box" else "Mesh" if kind == "Plane" else kind)
        size = list(g.get("size") or [0.01])
        while len(size) < 3:
            size.append(size[-1])
        if kind == "Plane":
            if is_col:
                raise NotImplementedError("Colliding infinite planes require a host-scene plane")
            mesh = UsdGeom.Mesh(prim)
            mesh.CreatePointsAttr([Gf.Vec3f(-size[0],-size[1],0),Gf.Vec3f(size[0],-size[1],0),
                                  Gf.Vec3f(size[0],size[1],0),Gf.Vec3f(-size[0],size[1],0)])
            mesh.CreateFaceVertexCountsAttr([4])
            mesh.CreateFaceVertexIndicesAttr([0,1,2,3])
            mesh.CreateDoubleSidedAttr(True)
            mesh.CreateDisplayColorAttr([Gf.Vec3f(*(g.get("rgba") or [.5,.5,.5,1])[:3])])
            n_vis += 1
        elif kind == "Mesh":
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
                UsdGeom.Cube(prim).CreateSizeAttr(2.0)
                UsdGeom.Xformable(prim).AddScaleOp().Set(Gf.Vec3f(*size))
            else:
                schema = {"Sphere": UsdGeom.Sphere, "Cylinder": UsdGeom.Cylinder,
                          "Capsule": UsdGeom.Capsule}[kind](prim)
                schema.CreateRadiusAttr(float(size[0]))
                if int(g["type"]) == 4:  # MuJoCo ellipsoid, not a round sphere.
                    prim.GetAttribute("radius").Set(1.0)
                    UsdGeom.Xformable(prim).AddScaleOp().Set(Gf.Vec3f(*size))
                if kind in ("Cylinder", "Capsule"):
                    schema.CreateHeightAttr(float(size[1] * 2))
            # 变换已放在 holder 上，本体不再放（否则重复计入）
        if is_col:
            UsdPhysics.CollisionAPI.Apply(prim)
            if kind == "Mesh":
                UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexHull")
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
        joint_path = "/Instrument/Joints/" + san(bname[b0] + "_" + j["name"])
        js = (UsdPhysics.RevoluteJoint.Define(stage, joint_path) if kind == "RevoluteJoint"
              else UsdPhysics.PrismaticJoint.Define(stage, joint_path))
        jp = js.GetPrim()
        index = next(i for i,record in enumerate(body_joints[b1]) if record["id"]==j["id"])
        links = chains[b1]
        js.CreateBody0Rel().SetTargets([Sdf.Path(links[index])])
        js.CreateBody1Rel().SetTargets([Sdf.Path(links[index+1])])
        a = j["world_anchor"]
        b0d = auxiliary_records.get(links[index],b_of[b0] if index==0 else b_of[b1])
        b1d = auxiliary_records.get(links[index+1],b_of[b1])
        d0 = [a[k] - b0d["world_pos"][k] for k in range(3)]
        d1 = [a[k] - b1d["world_pos"][k] for k in range(3)]
        R0 = [[b0d["world_xmat_rowmajor"][i * 3 + j] for j in range(3)] for i in range(3)]
        R1 = [[b1d["world_xmat_rowmajor"][i * 3 + j] for j in range(3)] for i in range(3)]
        lp0 = [sum(R0[k][i] * d0[k] for k in range(3)) for i in range(3)]
        lp1 = [sum(R1[k][i] * d1[k] for k in range(3)) for i in range(3)]
        jp.CreateAttribute("physics:localPos0", Sdf.ValueTypeNames.Point3f).Set(Gf.Vec3f(*lp0))
        jp.CreateAttribute("physics:localPos1", Sdf.ValueTypeNames.Point3f).Set(Gf.Vec3f(*lp1))
        # Both joint frames coincide at qpos=0; their signed +X is the truth axis.
        joint_world = Gf.Rotation(Gf.Vec3d(1, 0, 0), Gf.Vec3d(*j["world_axis"])).GetQuat()
        for index, body in enumerate((b0d, b1d)):
            q = body["world_quat_wxyz"]
            body_world = Gf.Quatd(q[0], Gf.Vec3d(*q[1:]))
            local = body_world.GetInverse() * joint_world
            jp.CreateAttribute("physics:localRot%d" % index, Sdf.ValueTypeNames.Quatf).Set(
                Gf.Quatf(local.GetReal(), Gf.Vec3f(local.GetImaginary())))
        js.CreateAxisAttr("X")
        lo, hi = j["range"]
        if abs(lo) > 1e-9 or abs(hi) > 1e-9:
            if int(j["type"]) == 3:
                lo, hi = math.degrees(lo), math.degrees(hi)
            jp.CreateAttribute("physics:lowerLimit", Sdf.ValueTypeNames.Float).Set(float(lo))
            jp.CreateAttribute("physics:upperLimit", Sdf.ValueTypeNames.Float).Set(float(hi))
        if True:  # Expose a drive instance; zero stiffness preserves a passive joint.
            pre = "angular" if kind == "RevoluteJoint" else "linear"
            UsdPhysics.DriveAPI.Apply(jp, pre)
            coefficient = math.pi / 180.0 if pre == "angular" else 1.0
            jp.CreateAttribute("drive:%s:physics:type" % pre, Sdf.ValueTypeNames.Token).Set("force")
            jp.CreateAttribute("drive:%s:physics:stiffness" % pre, Sdf.ValueTypeNames.Float).Set(
                float(j.get("stiffness") or 0.0) * coefficient)
            jp.CreateAttribute("drive:%s:physics:damping" % pre, Sdf.ValueTypeNames.Float).Set(
                float(j.get("damping") or 0.0) * coefficient)
            jp.CreateAttribute("drive:%s:physics:targetPosition" % pre,
                               Sdf.ValueTypeNames.Float).Set(0.0)
        n_j += 1
        dof_map.append((j["name"], bname[b0], bname[b1], int(j["type"])))

    # MuJoCo bodies without a joint are welded to their parent, not free bodies.
    movable = {j["body_id"] for j in T["joints"]}
    for body in T["bodies"]:
        bid, parent = body["id"], body["parent_id"]
        if bid not in prim_of or parent not in prim_of or bid in movable:
            continue
        fixed = UsdPhysics.FixedJoint.Define(stage, "/Instrument/Joints/weld_" + san(body["name"]))
        fixed.CreateBody0Rel().SetTargets([Sdf.Path(prim_of[parent])])
        fixed.CreateBody1Rel().SetTargets([Sdf.Path(prim_of[bid])])
        pb = b_of[parent]
        diff = [body["world_pos"][i]-pb["world_pos"][i] for i in range(3)]
        rp = pb["world_xmat_rowmajor"]
        fixed.CreateLocalPos0Attr(Gf.Vec3f(*[sum(rp[k*3+i]*diff[k] for k in range(3)) for i in range(3)]))
        q0, q1 = pb["world_quat_wxyz"], body["world_quat_wxyz"]
        local = Gf.Quatd(q0[0],Gf.Vec3d(*q0[1:])).GetInverse() * Gf.Quatd(q1[0],Gf.Vec3d(*q1[1:]))
        fixed.CreateLocalRot0Attr(Gf.Quatf(local.GetReal(),Gf.Vec3f(local.GetImaginary())))
    # PhysicsScene belongs to the host stage. Maximal-coordinate joints work
    # without ArticulationRootAPI and allow the authored kinematic bases.
    cap = None
    stage.GetRootLayer().Export(os.path.join(OUT, name + ".usda"))

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
    print("判据：第2层/第3层误差应 < 1 mm；嵌套刚体应为 0；PhysicsScene 由宿主提供")
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
