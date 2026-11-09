"""
MJCF → USD 仪器转换器 v2：**把 MuJoCo 的 body 层级压平**。

v1 的问题（实测）：带关节的子 body 嵌在父刚体下面，PhysX 报 20 条
  "Rigid Body ... missing xformstack reset ... unpredicted results"
这版 USD 又不接受往 xformOpOrder 里塞 !resetXformStack!（非法 op，会把变换栈弄坏）。
官方导入器是靠"多套一层 body/body"规避，本文件用更干净的办法：
所有刚体平铺在 /Instrument/<body名> 同一层，各自的相对位姿用**预乘后的世界变换**
写成单个 xformOp:transform；几何挂在刚体之下但自己不再挂 RigidBodyAPI，于是永远没有嵌套刚体。

关节：body0=父刚体、body1=本刚体；localPos0/localPos1 用两侧世界矩阵的逆变换算出来，
不再靠"body pos 就是关节偏移"这种近似。

用法：
    D:/IsaacSim5/env/Scripts/python.exe scripts/mjcf_instrument_to_usd_v2.py [名字过滤...]
产物：assets/instruments3/<名字>.usda
"""
import os, sys, math

R = r"D:/IsaacSim5/env/Lib/site-packages/isaacsim"
U = os.path.join(R, "extscache", "omni.usd.libs-1.0.1+69cbf6ad.wx64.r.cp311")
sys.path.insert(0, U)
for d in [os.path.join(U, "bin"), os.path.join(U, "bin", "deps"), os.path.join(R, "kit")]:
    os.add_dll_directory(d)
from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mjcf_instrument_to_usd import (REPO, XML_DIR, load_root, load_mesh, sanitize,
                                    num, vec, ALL)

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(PROJ, "assets", "instruments3")
os.makedirs(OUT, exist_ok=True)


def quat_to_mat(w, x, y, z):
    """四元数 (w,x,y,z) → 旋转矩阵。手写公式，避免这版 pxr 的四元数 API 差异。"""
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    m = Gf.Matrix4d()
    m[0][0] = 1 - 2 * (y * y + z * z)
    m[0][1] = 2 * (x * y - z * w)
    m[0][2] = 2 * (x * z + y * w)
    m[1][0] = 2 * (x * y + z * w)
    m[1][1] = 1 - 2 * (x * x + z * z)
    m[1][2] = 2 * (y * z - x * w)
    m[2][0] = 2 * (x * z - y * w)
    m[2][1] = 2 * (y * z + x * w)
    m[2][2] = 1 - 2 * (x * x + y * y)
    return m


def local_pos(p, q):
    m = Gf.Matrix4d()
    pos = vec_3(p)
    m.SetTranslateOnly(Gf.Vec3d(*pos))
    if q:
        w, x, y, z = [float(v) for v in q.split()]
        m = m * quat_to_mat(w, x, y, z)
    return m


def vec_3(p):
    if not p:
        return [0.0, 0.0, 0.0]
    v = [float(x) for x in p.split()]
    while len(v) < 3:
        v.append(0.0)
    return v[:3]


def set_transform(prim, M):
    """自己管 xformOp:transform + xformOpOrder，避开 Xformable 对重复 op 抛异常。"""
    m = Gf.Matrix4d(M)
    a = prim.GetAttribute("xformOp:transform")
    if not a:
        a = prim.CreateAttribute("xformOp:transform", Sdf.ValueTypeNames.Matrix4d)
    a.Set(m)
    oa = prim.GetAttribute("xformOpOrder")
    if not oa:
        oa = prim.CreateAttribute("xformOpOrder", Sdf.ValueTypeNames.TokenArray)
    cur = [str(x) for x in (oa.Get() or [])]
    if "xformOp:transform" not in cur:
        cur.append("xformOp:transform")
        oa.Set(cur)


def rigid_inverse(M):
    """纯旋转+平移的逆（本文件无缩放）：显式按元素算，不碰这版 Gf 的矩阵 API 差异。"""
    m = Gf.Matrix4d(M)
    r = [[m[i][j] for j in range(3)] for i in range(3)]
    t = (m[0][3], m[1][3], m[2][3])
    inv = Gf.Matrix4d()
    for i in range(3):
        for j in range(3):
            inv[i][j] = r[j][i]
        inv[i][3] = -(r[0][i] * t[0] + r[1][i] * t[1] + r[2][i] * t[2])
    return inv


GEOM_N = [0]


def uniq(base):
    GEOM_N[0] += 1
    return "%s_%04d" % (base, GEOM_N[0])


def XP(M, v):
    """点变换 -> Gf.Vec3f（这版 Gf.Matrix4d 没有 TransformPoint；Vec3d 走 Transform 即点变换）"""
    r = M.Transform(Gf.Vec3d(float(v[0]), float(v[1]), float(v[2])))
    return Gf.Vec3f(r[0], r[1], r[2])


def XD(M, v):
    """方向变换 -> 三元组（Vec3f 走 Transform 即方向；本文件矩阵无缩放）"""
    r = M.Transform(Gf.Vec3f(float(v[0]), float(v[1]), float(v[2])))
    return (r[0], r[1], r[2])


def axis_letter(v):
    ax, ay, az = abs(v[0]), abs(v[1]), abs(v[2])
    if ax >= ay and ax >= az:
        return "X"
    if ay >= az:
        return "Y"
    return "Z"


def build(xml_name):
    xml = os.path.join(XML_DIR, xml_name + ".xml")
    root = load_root(xml)
    comp = root.find("compiler")
    base = (comp.get("meshdir") or comp.get("assetdir")) if comp is not None else None
    meshdir = os.path.normpath(os.path.join(XML_DIR, base or "."))

    stage = Usd.Stage.CreateNew(os.path.join(OUT, xml_name + ".usda"))
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    top = UsdGeom.Xform.Define(stage, "/Instrument")
    stage.SetDefaultPrim(top.GetPrim())

    meshes, mats, cls = {}, {}, {}
    asset = root.find("asset")
    if asset is not None:
        for m in asset.iter("mesh"):
            f = m.get("file")
            if not f:
                continue
            sc = m.get("scale")
            scale = [float(x) for x in sc.split()] if sc else [1.0, 1.0, 1.0]
            meshes[m.get("name")] = (os.path.normpath(os.path.join(meshdir, f)), scale)
        for mt in asset.iter("material"):
            rgba = mt.get("rgba")
            mats[mt.get("name")] = [float(x) for x in rgba.split()] if rgba else [0.8, 0.8, 0.8, 1.0]
    default = root.find("default")
    if default is not None:
        for d in default.iter("default"):
            nm, g = d.get("class"), d.find("geom")
            if nm and g is not None:
                cls[nm] = g.attrib

    stats = {"rb": 0, "col": 0, "mesh": 0, "joint": 0, "rep": 0, "nested_fixed": 0}
    cache = {}

    def emit_geometry(el_owner, world, prefix, container=None):
        """prefix 这个容器自带世界变换 container；本函数只写相对变换，绝不把
        世界矩阵烘进顶点里（那样再叠加容器变换就是双重变换，实测会把零件炸开）。"""
        """把某 owner 元素上的 geom 变成挂在刚体下的几何（自己不带 RigidBodyAPI）。"""
        for g in el_owner.findall("geom"):
            cl = cls.get(g.get("name") and None or g.get("class"), {})
            typ = g.get("type") or cl.get("type") or "box"
            grp = g.get("group") or cl.get("group") or "0"
            contype = g.get("contype") or cl.get("contype") or "1"
            conaff = g.get("conaffinity") or cl.get("conaffinity") or "1"
            is_col = not (grp in ("2", "5") or (contype == "0" and conaff == "0" and grp == "2"))
            if container is None:
                container = world
            lm = local_pos(g.get("pos"), g.get("quat"))
            rel = rigid_inverse(Gf.Matrix4d(container)) * Gf.Matrix4d(world * lm)
            if typ == "mesh":
                mn = g.get("mesh")
                if mn not in meshes:
                    continue
                fp, scale = meshes[mn]
                if not os.path.isfile(fp):
                    continue
                key = (mn, is_col)
                if key not in cache:
                    cache[key] = load_mesh(fp, scale)
                pts, faces = cache[key]
                if not pts:
                    continue
                path = prefix + ("/Collision/" if is_col else "/Visual/") + uniq(sanitize(os.path.basename(fp)))
                mesh = UsdGeom.Mesh.Define(stage, path)
                prim = mesh.GetPrim()
                mesh.CreatePointsAttr([Gf.Vec3f(*q) for q in pts])
                mesh.CreateFaceVertexCountsAttr([len(f) for f in faces])
                mesh.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
                xs = [q[0] for q in pts]; ys = [q[1] for q in pts]; zs = [q[2] for q in pts]
                mesh.CreateExtentAttr([Gf.Vec3f(min(xs), min(ys), min(zs)),
                                       Gf.Vec3f(max(xs), max(ys), max(zs))])
                set_transform(prim, rel)
                if is_col:
                    UsdPhysics.CollisionAPI.Apply(prim)
                    stats["col"] += 1
                else:
                    c = mats.get(g.get("material"), [0.8, 0.8, 0.8, 1.0])
                    prim.CreateAttribute("displayColor",
                                         Sdf.ValueTypeNames.Color3fArray).Set([Gf.Vec3f(*c[:3])])
                    if len(c) > 3 and c[3] < 1.0:
                        prim.CreateAttribute("displayOpacity",
                                             Sdf.ValueTypeNames.FloatArray).Set([c[3]])
                    stats["mesh"] += 1
                continue
            kind = {"box": "Box", "sphere": "Sphere", "cylinder": "Cylinder",
                    "capsule": "Capsule"}.get(typ)
            if kind is None:
                continue
            size = [float(x) for x in (g.get("size") or "0.01").split()]
            while len(size) < 3:
                size.append(size[-1])
            prim = stage.DefinePrim(prefix + ("/Collision/" if is_col else "/Visual/")
                                    + uniq(sanitize(g.get("name") or (typ + "_g"))), kind)
            if typ == "box":
                prim.CreateAttribute("extent", Sdf.ValueTypeNames.Normal3fArray).Set(
                    [Gf.Vec3f(-size[0], -size[1], -size[2]), Gf.Vec3f(*size)])
                prim.CreateAttribute("size", Sdf.ValueTypeNames.Float).Set(2.0)
            else:
                prim.CreateAttribute("radius", Sdf.ValueTypeNames.Float).Set(size[0])
                if typ in ("cylinder", "capsule"):
                    prim.CreateAttribute("height", Sdf.ValueTypeNames.Float).Set(size[1] * 2)
                    if typ == "capsule":
                        prim.CreateAttribute("axisLength", Sdf.ValueTypeNames.Float).Set(size[1] * 2)
            set_transform(prim, rel)
            if is_col:
                UsdPhysics.CollisionAPI.Apply(prim)
                stats["col"] += 1

    def walk_body(el, parent_world, parent_rb, name_prefix):
        """把一个 body（含其下 frame/replicate 的内容）压平输出到同一层。"""
        nm = sanitize(el.get("name") or "body")
        own_world = parent_world * local_pos(el.get("pos"), el.get("quat"))

        def has_joint(e, d=0):
            for ch in e:
                if ch.tag == "joint" and ch.get("name"):
                    return True
                if ch.tag in ("body", "frame", "replicate") and d < 6 and has_joint(ch, d + 1):
                    return True
            return False

        movable = has_joint(el)
        bpath = "/Instrument/" + (name_prefix + "_" if name_prefix else "") + nm
        prim = stage.DefinePrim(bpath, "Xform")
        set_transform(prim, own_world)
        rb_path = None
        if movable or parent_rb is None:
            UsdPhysics.RigidBodyAPI.Apply(prim)
            mass = el.get("mass")
            prim.CreateAttribute("physics:mass", Sdf.ValueTypeNames.Float).Set(
                float(mass) if mass else (10.0 if parent_rb is None else 0.02))
            if parent_rb is None:      # 顶层 = 底座，钉住
                prim.CreateAttribute("physics:kinematicEnabled",
                                     Sdf.ValueTypeNames.Bool).Set(True)
            rb_path = str(prim.GetPath())
            stats["rb"] += 1

        emit_geometry(el, own_world, bpath, own_world)

        # 关节：属于本 body，body1=本刚体，body0=最近的上级刚体
        for j in el.findall("joint"):
            jn = j.get("name")
            if not jn or parent_rb is None or rb_path is None:
                continue
            typ = j.get("type") or "hinge"
            kind = "RevoluteJoint" if typ == "hinge" else ("PrismaticJoint" if typ == "slide" else None)
            if kind is None:
                continue
            jworld = own_world * local_pos(j.get("pos"), None)
            jpath = "/Instrument/J_" + sanitize(nm) + "_" + sanitize(jn)
            jp = stage.DefinePrim(jpath, kind)
            js = UsdPhysics.RevoluteJoint(jp) if kind == "RevoluteJoint" else UsdPhysics.PrismaticJoint(jp)
            b0 = js.CreateBody0Rel(); b1 = js.CreateBody1Rel()
            b0.SetTargets([Sdf.Path(parent_rb)])
            b1.SetTargets([Sdf.Path(rb_path)])
            inv_parent = Gf.Matrix4d(parent_world).GetInverse()
            inv_self = Gf.Matrix4d(own_world).GetInverse()
            t = Gf.Vec3d(jworld[0][3], jworld[1][3], jworld[2][3])
            jp.CreateAttribute("physics:localPos0", Sdf.ValueTypeNames.Point3f).Set(
                XP(inv_parent, t))
            jp.CreateAttribute("physics:localPos1", Sdf.ValueTypeNames.Point3f).Set(
                XP(inv_self, t))
            axis = [float(x) for x in (j.get("axis") or "0 0 1").split()]
            vpar = XD(Gf.Matrix4d(parent_world), axis)
            jp.CreateAttribute("physics:axis", Sdf.ValueTypeNames.Token).Set(axis_letter(vpar))
            lim = j.get("range")
            if lim:
                lo, hi = [float(x) for x in lim.split()]
                if typ == "hinge":
                    lo, hi = math.degrees(lo), math.degrees(hi)
                jp.CreateAttribute("physics:lowerLimit", Sdf.ValueTypeNames.Float).Set(lo)
                jp.CreateAttribute("physics:upperLimit", Sdf.ValueTypeNames.Float).Set(hi)
            stiff = num(j, "stiffness", 0.0)
            if stiff:
                pre = "angular" if typ == "hinge" else "linear"
                jp.CreateAttribute("drive:%s:physics:type" % pre,
                                   Sdf.ValueTypeNames.Token).Set("force")
                jp.CreateAttribute("drive:%s:physics:stiffness" % pre,
                                   Sdf.ValueTypeNames.Float).Set(stiff)
                jp.CreateAttribute("drive:%s:physics:damping" % pre,
                                   Sdf.ValueTypeNames.Float).Set(num(j, "damping", 0.0) or 0.001)
                jp.CreateAttribute("drive:%s:physics:targetPosition" % pre,
                                   Sdf.ValueTypeNames.Float).Set(0.0)
            stats["joint"] += 1

        for ch in el:
            if ch.tag == "body":
                walk_body(ch, own_world, rb_path or parent_rb, "")
            elif ch.tag == "frame":
                fw = own_world * local_pos(ch.get("pos"), ch.get("quat"))
                if rb_path:
                    emit_geometry(ch, fw, bpath, own_world)
                else:
                    sp = "/Instrument/" + uniq("Static_" + nm)
                    sx = UsdGeom.Xform.Define(stage, sp)
                    set_transform(sx.GetPrim(), fw)
                    emit_geometry(ch, fw, sp, fw)
                for sub in ch:
                    if sub.tag == "body":
                        walk_body(sub, fw, rb_path or parent_rb, "")
                    elif sub.tag == "replicate":
                        expand_replicate(sub, fw, rb_path or parent_rb, nm)
            elif ch.tag == "replicate":
                expand_replicate(ch, own_world, rb_path or parent_rb, nm)

    def expand_replicate(rp, world, parent_rb, prefix_nm):
        count = int(float(rp.get("count") or 1))
        axis = [float(x) for x in (rp.get("axis") or "0 0 1").split()]
        if rp.get("rotation") is not None:
            stride = math.degrees(float(rp.get("rotation")))
        elif rp.get("period") is not None:
            stride = 360.0 / float(rp.get("period"))
        else:
            stride = 360.0 / max(count, 1)
        for i in range(count):
            stats["rep"] += 1
            ang = math.radians(stride * i)
            c, s = math.cos(ang), math.sin(ang)
            rot = Gf.Matrix4d()
            if axis[2] >= 1 - 1e-6:
                rot[0][0], rot[0][1] = c, -s
                rot[1][0], rot[1][1] = s, c
            elif axis[0] >= 1 - 1e-6:
                rot[1][1], rot[1][2] = c, -s
                rot[2][1], rot[2][2] = s, c
            else:
                rot[0][0], rot[0][2] = c, s
                rot[2][0], rot[2][2] = -s, c
            w = world * local_pos(rp.get("offset"), None) * rot
            sp = "/Instrument/" + uniq("Rep%02d" % i)
            sx = UsdGeom.Xform.Define(stage, sp)
            set_transform(sx.GetPrim(), w)
            emit_geometry(rp, w, sp, w)
            for sub in rp.findall("body"):
                walk_body(sub, w, parent_rb, "%s_r%02d" % (prefix_nm, i))
            for sub in rp.findall("frame"):
                fw = w * local_pos(sub.get("pos"), sub.get("quat"))
                sp = "/Instrument/" + uniq("Static_rep%02d" % i)
                sx = UsdGeom.Xform.Define(stage, sp)
                set_transform(sx.GetPrim(), fw)
                emit_geometry(sub, fw, sp, fw)
                for bb in sub.findall("body"):
                    walk_body(bb, fw, parent_rb, "%s_r%02d" % (prefix_nm, i))

    wb = root.find("worldbody")
    for ch in wb:
        if ch.tag == "body":
            walk_body(ch, Gf.Matrix4d(), None, "")
        elif ch.tag == "frame":
            fw = local_pos(ch.get("pos"), ch.get("quat"))
            for sub in ch.findall("body"):
                walk_body(sub, fw, None, "")
            sp = "/Instrument/" + uniq("Static")
            sx = UsdGeom.Xform.Define(stage, sp)
            set_transform(sx.GetPrim(), fw)
            emit_geometry(ch, fw, sp, fw)
    stage.GetRootLayer().Save()
    return stats


def main():
    want = [a for a in sys.argv[1:] if not a.startswith("--")] or ALL
    print("=" * 92)
    print("输出：", os.path.relpath(OUT, PROJ), "（压平层级版）")
    for nm in want:
        if not os.path.isfile(os.path.join(XML_DIR, nm + ".xml")):
            print("  ✘ 无此 XML：", nm)
            continue
        try:
            s = build(nm)
        except Exception as e:
            import traceback; traceback.print_exc()
            print("  ✘ %-34s %s: %s" % (nm, type(e).__name__, str(e)[:90]))
            continue
        st = Usd.Stage.Open(os.path.join(OUT, nm + ".usda"))
        bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                               [UsdGeom.Tokens.default_, UsdGeom.Tokens.render], False)
        rng = bb.ComputeWorldBound(st.GetDefaultPrim()).ComputeAlignedRange()
        dim = (0, 0, 0)
        if not rng.IsEmpty():
            a, b = rng.GetMin(), rng.GetMax()
            dim = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
        nested = 0
        for p in st.Traverse():
            if p.HasAPI(UsdPhysics.RigidBodyAPI):
                anc = p.GetParent()
                while anc:
                    if anc.HasAPI(UsdPhysics.RigidBodyAPI):
                        nested += 1
                        break
                    anc = anc.GetParent()
        print("  ✔ %-34s 刚体%-3d 碰撞%-4d 关节%-3d 网格%-4d replicate%-3d 外形 %.3f×%.3f×%.3f  嵌套刚体=%d"
              % (nm, s["rb"], s["col"], s["joint"], s["mesh"], s["rep"],
                 dim[0], dim[1], dim[2], nested))


if __name__ == "__main__":
    main()
