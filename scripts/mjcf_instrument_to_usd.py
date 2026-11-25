"""
MJCF → USD 转换器（仪器版：带层级 body、关节、<replicate> 展开）。

为什么要自己写：Isaac 官方 MJCF 导入器
  · 不认 <compiler assetdir/meshdir>（要改路径）、不认 <replicate>（5430 转子 30 孔位导不出来）；
  · 对物体类资产多次硬崩；对 5910 给出的包围盒是实物的约 3 倍；对恒温混匀仪转出空文件；
  · 建出的 rootJoint_body 的 body0 是空的（钉不住底座）。
上游 model/instrument/*.xml 本身把关节轴向、限位、刚度、阻尼都写全了，这些是权威数值，
照抄比猜靠谱。

产出：assets/instruments2/<名字>.usda（文本，可 diff）
用法：
    D:/IsaacSim5/env/Scripts/python.exe scripts/mjcf_instrument_to_usd.py [名字过滤...]
"""
import os, sys, math, xml.etree.ElementTree as ET

R = r"D:/IsaacSim5/env/Lib/site-packages/isaacsim"
U = os.path.join(R, "extscache", "omni.usd.libs-1.0.1+69cbf6ad.wx64.r.cp311")
sys.path.insert(0, U)
for d in [os.path.join(U, "bin"), os.path.join(U, "bin", "deps"), os.path.join(R, "kit")]:
    os.add_dll_directory(d)
from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf, UsdShade

REPO = os.environ.get("AUTOBIO_REPO", r"C:/Users/17690/.codex/tmp-repos/AutoBio")
PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
XML_DIR = os.path.join(REPO, "autobio", "model", "instrument")
OUT = os.path.join(PROJ, "assets", "instruments2")
os.makedirs(OUT, exist_ok=True)

ALL = ["centrifuge_eppendorf_5430", "centrifuge_eppendorf_5910_ri",
        "centrifuge_tiangen_tgear_mini", "thermal_cycler_biorad_c1000",
        "thermal_mixer_eppendorf_c", "vortex_mixer_genie_2"]


def load_root(path):
    txt = open(path, encoding="utf-8", errors="replace").read().lstrip("")
    import re
    txt = re.sub(r"^\s*<\?xml[^>]*\?>", "", txt)
    return ET.fromstring(txt)


def parse_obj(fp, scale):
    pts, faces = [], []
    for line in open(fp, encoding="utf-8", errors="replace"):
        if line.startswith("v "):
            p = line.split()[1:4]
            pts.append((float(p[0]) * scale[0], float(p[1]) * scale[1], float(p[2]) * scale[2]))
        elif line.startswith("f "):
            idx = [int(t.split("/")[0]) for t in line.split()[1:]]
            faces.append([(i - 1 if i > 0 else len(pts) + i) for i in idx])
    return pts, faces


def parse_stl(fp, scale):
    raw = open(fp, "rb").read()
    pts, faces = [], []
    if raw[:5].lower() == b"solid" and b"vertex" in raw[:4000].lower():
        tri = []
        for line in raw.decode("utf-8", "replace").splitlines():
            t = line.strip()
            if t.startswith("vertex"):
                v = [float(x) * scale[i] for i, x in enumerate(t.split()[1:4])]
                pts.append(tuple(v)); tri.append(len(pts) - 1)
                if len(tri) == 3:
                    faces.append(tri); tri = []
        return pts, faces
    import struct
    n = struct.unpack("<I", raw[80:84])[0]
    off = 84
    for _ in range(n):
        if off + 48 > len(raw):
            break
        vals = struct.unpack("<12f", raw[off:off + 48])
        base = len(pts)
        for k in range(3):
            v = vals[3 + k * 3: 6 + k * 3]
            pts.append((v[0] * scale[0], v[1] * scale[1], v[2] * scale[2]))
        faces.append([base, base + 1, base + 2])
        off += 50
    return pts, faces


def load_mesh(fp, scale):
    ext = os.path.splitext(fp)[1].lower()
    if ext == ".obj":
        return parse_obj(fp, scale)
    if ext == ".stl":
        return parse_stl(fp, scale)
    raise ValueError("不支持的网格：" + ext)


def num(el, name, default):
    v = el.get(name)
    return default if v is None else float(v)


def vec(el, name, default):
    v = el.get(name)
    if v is None:
        return list(default) if default is not None else None
    return [float(x) for x in v.split()]


def sanitize(s):
    import re
    return re.sub(r"[^0-9A-Za-z_]", "_", s or "")


class Conv:
    def __init__(self, stage, meshdir):
        self.st = stage
        self.meshdir = meshdir
        self.meshes = {}
        self.mats = {}
        self.rb_count = self.col_count = self.joint_count = self.mesh_count = 0
        self.replicated = 0

    def mesh_prim(self, parent, name, mesh_name, pos, quat, color, collide):
        fp, scale = self.meshes[mesh_name]
        if not os.path.isfile(fp):
            return None
        key = (mesh_name, collide)
        if key in self._cache:
            pts, faces = self._cache[key]
        else:
            pts, faces = load_mesh(fp, scale)
            self._cache[key] = (pts, faces)
        if not pts:
            return None
        path = parent + "/" + sanitize(name)
        m = UsdGeom.Mesh.Define(self.st, path)
        off = pos or [0, 0, 0]
        m.CreatePointsAttr([Gf.Vec3f(p[0] + off[0], p[1] + off[1], p[2] + off[2]) for p in pts])
        m.CreateFaceVertexCountsAttr([len(f) for f in faces])
        m.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
        xs = [p[0] for p in pts]; ys = [p[1] for p in pts]; zs = [p[2] for p in pts]
        m.CreateExtentAttr([Gf.Vec3f(min(xs), min(ys), min(zs)), Gf.Vec3f(max(xs), max(ys), max(zs))])
        if collide:
            UsdPhysics.CollisionAPI.Apply(m.GetPrim())
            self.col_count += 1
        else:
            m.CreateDisplayColorAttr([Gf.Vec3f(*color[:3])])
            if len(color) > 3 and color[3] < 1.0:
                m.CreateDisplayOpacityAttr([color[3]])
            self.mesh_count += 1
        return m

    _cache = {}


def build(xml_name):
    xml = os.path.join(XML_DIR, xml_name + ".xml")
    root = load_root(xml)
    comp = root.find("compiler")
    # 注意：物体 XML 用 meshdir，仪器 XML 用 assetdir，两个都要认
    base = None
    if comp is not None:
        base = comp.get("meshdir") or comp.get("assetdir")
    meshdir = os.path.normpath(os.path.join(XML_DIR, base or "."))
    default = root.find("default")

    # 把 <defaultclass> 里 geom 的 class 属性（type/density/group/contype）扁平化出来
    cls = {}
    if default is not None:
        for d in default.iter("default"):
            nm = d.get("class")
            g = d.find("geom")
            if nm and g is not None:
                cls[nm] = g.attrib

    cv = Conv.__new__(Conv)
    stage = Usd.Stage.CreateNew(os.path.join(OUT, xml_name + ".usda"))
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    cv.st = stage
    cv._cache = {}
    cv.meshes = {}
    cv.mats = {}
    cv.rb_count = cv.col_count = cv.joint_count = cv.mesh_count = cv.replicated = 0

    asset = root.find("asset")
    if asset is not None:
        for m in asset.iter("mesh"):
            f = m.get("file")
            if not f:
                continue
            sc = m.get("scale")
            scale = [float(x) for x in sc.split()] if sc else [1.0, 1.0, 1.0]
            cv.meshes[m.get("name")] = (os.path.normpath(os.path.join(meshdir, f)), scale)
        for mt in asset.iter("material"):
            rgba = mt.get("rgba")
            cv.mats[mt.get("name")] = ([float(x) for x in rgba.split()]
                                       if rgba else [0.8, 0.8, 0.8, 1.0])

    top = UsdGeom.Xform.Define(stage, "/Instrument")
    stage.SetDefaultPrim(top.GetPrim())
    if root.find("worldbody") is None:
        return None, "没有 worldbody"

    def walk(body, parent_path, parent_is_base, parent_rb=None):
        nm = sanitize(body.get("name") or "body")
        bpath = parent_path + "/" + nm
        xf = UsdGeom.Xform.Define(stage, bpath)
        pos = vec(body, "pos", [0, 0, 0])
        quat = body.get("quat")
        if pos != [0, 0, 0]:
            UsdGeom.Xformable(xf).AddTranslateOp().Set(Gf.Vec3d(*pos))
        if quat:
            w, x, y, z = [float(v) for v in quat.split()]
            UsdGeom.Xformable(xf).AddOrientOp().Set(Gf.Quatf(w, x, y, z))

        def has_joint(el, depth=0):
            """本 body 或其后代里有没有带名字的 joint（决定它算不算可动刚体）。"""
            for ch in el:
                if ch.tag == "joint" and ch.get("name"):
                    return True
                if ch.tag in ("body", "frame", "replicate") and depth < 6:
                    if has_joint(ch, depth + 1):
                        return True
            return False

        has_motion = has_joint(body)
        geoms = [g for g in body.findall("geom")]
        # TODO(flatten): 带关节的子 body 不应嵌在父刚体下面，应压平到同一层并预乘世界变换，
        # 否则 PhysX 判定为非法嵌套（见下方注释）。
        if has_motion or (parent_is_base and geoms):
            UsdPhysics.RigidBodyAPI.Apply(xf.GetPrim())
            cv.rb_count += 1
            # PhysX 要求：祖先已经是刚体的 prim 上再挂刚体，必须插一个 resetXformStack，
            # 否则报 "unpredicted results"（实测六台仪器全在报）。
            # 注意：嵌套刚体（lid 挂在 body 里）会让 PhysX 报
            #   "RigidBody ... missing xformstack reset ... unpredicted results"
            # 直接往 xformOpOrder 里塞 "xformOp:!resetXformStack!" 在这版 USD 里是非法 op，
            # 会把整条变换栈弄坏（结果：刚体数掉到 0，"0 位移=稳"是假通过）。
            # 正解是把 MuJoCo 的 body 层级压平（官方导入器就是用多套一层 body/body 规避），
            # 见 scripts/mjcf_instrument_to_usd.py 的 TODO(flatten)。
            pass
            m = body.get("mass")
            if m:
                xf.GetPrim().CreateAttribute("physics:mass", Sdf.ValueTypeNames.Float).Set(float(m))
            # 顶层 body（父级没有刚体可依附）= 底座，必须钉住：
            # 官方 MJCF 导入器在这里只留了一条 body0 为空的 rootJoint，PhysX 不认，整台仪器会自由落体。
            # 自己转的时候直接标运动学（与本项目里师兄那台离心机同样的做法）。
            if parent_rb is None:
                xf.GetPrim().CreateAttribute("physics:kinematicEnabled",
                                             Sdf.ValueTypeNames.Bool).Set(True)
                if not m:
                    xf.GetPrim().CreateAttribute("physics:mass",
                                                 Sdf.ValueTypeNames.Float).Set(10.0)

        for g in geoms:
            cl = cls.get(g.get("class"), {})
            typ = g.get("type") or cl.get("type") or "box"
            grp = g.get("group") or cl.get("group") or "0"
            contype = g.get("contype") or cl.get("contype") or "1"
            conaff = g.get("conaffinity") or cl.get("conaffinity") or "1"
            visual_only = (grp in ("2", "5")) or (contype == "0" and conaff == "0" and grp == "2")
            is_col = not visual_only
            if typ == "mesh":
                mn = g.get("mesh")
                if mn not in cv.meshes:
                    continue
                color = cv.mats.get(g.get("material"), [0.8, 0.8, 0.8, 1.0])
                cv.mesh_prim(bpath + ("/Collision" if is_col else "/Visual"),
                             os.path.basename(cv.meshes[mn][0]) + ("_c" if is_col else "_v"),
                             mn, vec(g, "pos", None), None, color, is_col)
                continue
            size = [float(x) for x in (g.get("size") or "0.01").split()]
            while len(size) < 3:
                size.append(size[-1])
            kind = {"box": "Box", "sphere": "Sphere", "cylinder": "Cylinder",
                    "capsule": "Capsule"}.get(typ)
            if kind is None:
                continue
            gp = g.get("name") or (typ + "_" + str(len(list(Usd.PrimRange(xf.GetPrim())))))
            prim = stage.DefinePrim(bpath + ("/Collision" if is_col else "/Visual") + "/" + sanitize(gp), kind)
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
            off = vec(g, "pos", None)
            if off and any(off):
                UsdGeom.Xformable(prim).AddTranslateOp().Set(Gf.Vec3d(*off))
            if is_col:
                UsdPhysics.CollisionAPI.Apply(prim)
                cv.col_count += 1

        this_rb = str(xf.GetPath()) if (has_motion or (parent_is_base and geoms)) else parent_rb
        # 关节：hinge / slide。body1 = 本 body（关节在它原点），body0 = 最近的上级刚体
        # （MuJoCo 的惯例是 body 的 pos 就是关节相对父级的偏移，所以 localPos0 = body pos）。
        # 顶层 body 自带的关节 = 底座固定，父级没有刚体可用 → 把本 body 设成运动学来钉住。
        for j in body.findall("joint"):
            jn = j.get("name")
            if not jn:
                continue
            typ = j.get("type") or "hinge"
            axis = vec(j, "axis", [0, 0, 1]) or [0, 0, 1]
            lim = j.get("range")
            stiff = num(j, "stiffness", 0.0)
            damp = num(j, "damping", 0.0)
            frictionloss = num(j, "frictionloss", 0.0)
            refpos = vec(j, "pos", [0, 0, 0]) or [0, 0, 0]
            if parent_rb is None:
                UsdPhysics.RigidBodyAPI.Apply(xf.GetPrim())
                xf.GetPrim().CreateAttribute("physics:kinematicEnabled",
                                             Sdf.ValueTypeNames.Bool).Set(True)
                xf.GetPrim().CreateAttribute("physics:mass",
                                             Sdf.ValueTypeNames.Float).Set(10.0)
                cv.joint_count += 1
                continue
            jpath = parent_path + "/J_" + sanitize(body.get("name")) + "_" + sanitize(jn)
            kind = "RevoluteJoint" if typ == "hinge" else ("PrismaticJoint" if typ == "slide" else None)
            if kind is None:
                continue
            jprim = stage.DefinePrim(jpath, kind)
            UsdGeom.Xformable(jprim).AddTranslateOp().Set(Gf.Vec3d(*refpos))
            main_axis = ("X" if abs(axis[0]) >= max(abs(axis[1]), abs(axis[2]))
                         else ("Y" if abs(axis[1]) >= abs(axis[2]) else "Z"))
            jsch = UsdPhysics.RevoluteJoint(jprim) if kind == "RevoluteJoint" else UsdPhysics.PrismaticJoint(jprim)
            r0 = jsch.CreateBody0Rel(); r1 = jsch.CreateBody1Rel()
            r0.SetTargets([Sdf.Path(parent_rb)])
            r1.SetTargets([Sdf.Path(this_rb)])
            jprim.CreateAttribute("physics:axis", Sdf.ValueTypeNames.Token).Set(main_axis)
            jprim.CreateAttribute("physics:localPos0", Sdf.ValueTypeNames.Point3f).Set(
                Gf.Vec3f(pos[0], pos[1], pos[2]))
            jprim.CreateAttribute("physics:localPos1", Sdf.ValueTypeNames.Point3f).Set(Gf.Vec3f(0, 0, 0))
            if lim:
                lo, hi = [float(x) for x in lim.split()]
                if typ == "hinge":
                    lo, hi = math.degrees(lo), math.degrees(hi)
                jprim.CreateAttribute("physics:lowerLimit", Sdf.ValueTypeNames.Float).Set(float(lo))
                jprim.CreateAttribute("physics:upperLimit", Sdf.ValueTypeNames.Float).Set(float(hi))
            if stiff:
                pre = "angular" if typ == "hinge" else "linear"
                jprim.CreateAttribute("drive:%s:physics:type" % pre,
                                      Sdf.ValueTypeNames.Token).Set("force")
                jprim.CreateAttribute("drive:%s:physics:stiffness" % pre,
                                      Sdf.ValueTypeNames.Float).Set(float(stiff))
                jprim.CreateAttribute("drive:%s:physics:damping" % pre,
                                      Sdf.ValueTypeNames.Float).Set(float(damp or 0.001))
                jprim.CreateAttribute("drive:%s:physics:targetPosition" % pre,
                                      Sdf.ValueTypeNames.Float).Set(0.0)
            if frictionloss:
                jprim.CreateAttribute("physics:friction", Sdf.ValueTypeNames.Float).Set(float(frictionloss))
            cv.joint_count += 1

        # 子 body / 子 frame（MuJoCo 3.x 的 frame 是无形容器，要穿过去）
        walk_children(body, bpath, False, this_rb)
        handle_replicates(el=body, path=bpath)
        return xf

    def body_like_geom(body, rp, g, ghost_path, i):
        # replicate 下直接挂的 geom（不套 body）：按 ghost 的旋转复制一份
        typ = g.get("type")
        if typ != "mesh":
            return
        mn = g.get("mesh")
        if mn not in cv.meshes:
            return
        cv.mesh_prim(ghost_path + "/Collision", "%s_%02d" % (sanitize(mn), i),
                     mn, vec(g, "pos", [0, 0, 0]), None,
                     cv.mats.get(g.get("material"), [0.7, 0.7, 0.7, 1.0]), True)

    def handle_replicates_single(rp, path, parent_rb=None):
        count = int(float(rp.get("count") or 1))
        ax = vec(rp, "axis", [0, 0, 1]) or [0, 0, 1]
        step = rp.get("rotation")
        period = rp.get("period")
        if step is not None:
            stride = math.degrees(float(step))
        elif period is not None:
            stride = 360.0 / float(period)
        else:
            stride = 360.0 / max(count, 1)
        base = vec(rp, "offset", [0, 0, 0]) or [0, 0, 0]
        for i in range(count):
            cv.replicated += 1
            rp_path = path + "/rep_%02d" % i
            ghost = UsdGeom.Xform.Define(stage, rp_path)
            UsdGeom.Xformable(ghost).AddTranslateOp().Set(Gf.Vec3d(*base))
            UsdGeom.Xformable(ghost).AddRotateXYZOp().Set(
                Gf.Vec3f(ax[0] * stride * i, ax[1] * stride * i, ax[2] * stride * i))
            walk_children(rp, rp_path, False, parent_rb)

    def handle_replicates(el, path):
        """<replicate count=N axis=..>：绕轴旋转复制 N 份。
        MuJoCo 默认 angle 步进 = 2*pi/count；5430 转子 30 个孔位就是这么写的，
        官方导入器不认这个标签，所以那版转出来转子是空的。"""
        for rp in el.findall("replicate"):
            count = int(float(rp.get("count") or 1))
            ax = vec(rp, "axis", [0, 0, 1]) or [0, 0, 1]
            period = rp.get("period")
            step = rp.get("rotation")
            if step is not None:
                stride = math.degrees(float(step))
            elif period is not None:
                stride = 360.0 / float(period)
            else:
                stride = 360.0 / max(count, 1)
            base = vec(rp, "offset", [0, 0, 0]) or [0, 0, 0]
            for i in range(count):
                cv.replicated += 1
                rp_path = path + "/rep_%02d" % i
                ghost = UsdGeom.Xform.Define(stage, rp_path)
                UsdGeom.Xformable(ghost).AddTranslateOp().Set(Gf.Vec3d(*base))
                UsdGeom.Xformable(ghost).AddRotateXYZOp().Set(
                    Gf.Vec3f(ax[0] * stride * i, ax[1] * stride * i, ax[2] * stride * i))
                walk_children(rp, rp_path, False, parent_rb)

    def walk_children(el, path, is_base, parent_rb=None):
        """el 的直接子级里的 body / frame 逐个处理。"""
        fi = 0
        for ch in el:
            if ch.tag == "body":
                walk(ch, path, is_base, parent_rb)
            elif ch.tag == "replicate":
                handle_replicates_single(ch, path, parent_rb)
            elif ch.tag == "frame":
                fi += 1
                nm = sanitize(ch.get("name") or ("frame_%02d" % fi))
                fpath = path + "/" + nm
                fx = UsdGeom.Xform.Define(stage, fpath)
                fpos = vec(ch, "pos", [0, 0, 0])
                if fpos and fpos != [0, 0, 0]:
                    UsdGeom.Xformable(fx).AddTranslateOp().Set(Gf.Vec3d(*fpos))
                fq = ch.get("quat")
                if fq:
                    w, x, y, z = [float(v) for v in fq.split()]
                    UsdGeom.Xformable(fx).AddOrientOp().Set(Gf.Quatf(w, x, y, z))
                # frame 上直接挂的 geom 属于静态（frame 本身没有质量）
                for g in ch.findall("geom"):
                    if g.get("type") != "mesh" or g.get("mesh") not in cv.meshes:
                        continue
                    cv.mesh_prim(fpath + "/StaticCollision",
                                 sanitize(g.get("mesh")) + "_s", g.get("mesh"),
                                 vec(g, "pos", [0, 0, 0]), None,
                                 cv.mats.get(g.get("material"), [0.7, 0.7, 0.7, 1.0]), True)
                walk_children(ch, fpath, is_base, parent_rb)

    wb = root.find("worldbody")
    walk_children(wb, "/Instrument", True)
    for g in wb.findall("geom"):        # worldbody 直属 geom（静态底盘/夹具）
        cv.mesh_prim("/Instrument/StaticCollision", sanitize(g.get("name") or "wgeom"),
                     g.get("mesh") or "", [0, 0, 0], None, [0.6, 0.6, 0.6, 1.0], True) \
            if (g.get("type") == "mesh" and (g.get("mesh") in cv.meshes)) else None
    stage.GetRootLayer().Save()
    return cv, None


def main():
    want = [a for a in sys.argv[1:] if not a.startswith("--")] or ALL
    print("=" * 96)
    print("输出目录：", os.path.relpath(OUT, PROJ))
    for nm in want:
        if not os.path.isfile(os.path.join(XML_DIR, nm + ".xml")):
            print("  ✘ 没有这个仪器 XML：", nm)
            continue
        try:
            cv, err = build(nm)
        except Exception as e:
            print("  ✘ %-34s 异常 %s: %s" % (nm, type(e).__name__, str(e)[:90]))
            continue
        if cv is None:
            print("  ✘ %-34s %s" % (nm, err))
            continue
        fp = os.path.join(OUT, nm + ".usda")
        st = Usd.Stage.Open(fp)
        bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                               [UsdGeom.Tokens.default_, UsdGeom.Tokens.render], False)
        rng = bb.ComputeWorldBound(st.GetDefaultPrim()).ComputeAlignedRange()
        dim = (0, 0, 0)
        if not rng.IsEmpty():
            a, b = rng.GetMin(), rng.GetMax()
            dim = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
        print("  ✔ %-34s 刚体%-4d 碰撞%-5d 关节%-4d 网格%-5d replicate%d  外形 %.3f×%.3f×%.3f m"
              % (nm, cv.rb_count, cv.col_count, cv.joint_count, cv.mesh_count,
                 cv.replicated, dim[0], dim[1], dim[2]))


if __name__ == "__main__":
    main()
