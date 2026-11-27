# -*- coding: utf-8 -*-
"""B 批：给 3 个 ART 仪器的 12 个刚体补几何、碰撞体和质量——全部写进一个 overlay。

现场（v09 合成场景实测）：
    Stations/{CentriMini,ThermalCycler,VortexGenie2}
    几何 0、碰撞 0，却挂着 12 个 RigidBodyAPI，physics:mass 全是 0.0。
PhysX 对"无碰撞体 + 零质量"的刚体算不出惯量，运行期每帧刷
Invalid PhysX transform，A 批写进去的关节 drive 也就没有着力点。

为什么长这样：MJCF→USD 导出的是标准四件套，几何只在 _base.usd 的根级
/visuals、/collisions 分组里；运动学树 _physics.usd 靠"空 assetPath 的内部
引用"去接它们，而内部引用要求 _physics.usd 自己是锚层。场景引的 <name>.usd
只是副出来的骨架，那两条内部引用在它身上永远解析不开。

为什么不直接改 crumb（试过的两条路都不通）：
  - 给 /Instrument 引 _physics.usd：prim 数对得上但 Mesh=0，理由就是上面那条。
  - 把 crumb 里 visuals/collisions 的内部引用改写成 @configuration/x_base.usd@
    的外部引用：元数据写进去了、读得回来，重开还是 0 个子级——这套绑定下
    crate 层的相对 assetPath 解析不开（同一条引用写在 .usda 里立刻生效）。
  所以改动收在 assets/scenes/art_shapes_v09.usda 这个文本 overlay 里，
  crumb 从 outputs/backup_assets_20261123/ 的原样恢复，一个字节都不留。

质量按真机的分量级给，不用包围盒算：包围盒 × 密度会把 20 g 的调速旋钮估成
1.1 kg、离心机盖子估成 0.74 kg，关节那点 maxForce 根本带不动。数值来源是同类
仪器的公开规格量级（TGear-mini 整机约 1.5 kg、VortexGenie2 约 2.5 kg、
Bio-Rad T100 Cycler 约 11 kg），扣掉外壳腔体按部件分下去，是估算值。

用法：
    D:/isaacsim-5.1.0/python.bat scripts/fix_instrument_geometry.py            # 看要写什么
    D:/isaacsim-5.1.0/python.bat scripts/fix_instrument_geometry.py --apply    # 写 overlay 并挂 sublayer
"""
import os
import shutil
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _pxr_env

_pxr_env.bootstrap()

from pxr import Sdf, Usd, UsdGeom, UsdPhysics  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
INSTR = os.path.join(ROOT, "assets", "instruments")
SCENES = os.path.join(ROOT, "assets", "scenes")
SCENE = os.path.join(SCENES, "biolab_main_v09.usda")
OVERLAY = os.path.join(SCENES, "art_shapes_v09.usda")
BAK = os.path.join(ROOT, "outputs", "backup_assets_20261123")

# 工位名 -> (stem, 各部件质量 kg)。只有这 3 个工位是"有刚体没几何"。
INSTRUMENTS = {
    "CentriMini": ("centrifuge_tiangen_tgear_mini",
                   {"body": 1.00, "lid": 0.12, "rotor": 0.15}),
    "ThermalCycler": ("thermal_cycler_biorad_c1000",
                      {"body": 8.00, "lid": 0.60, "lid_lever": 0.08,
                       "lid_force_knob": 0.05, "reaction_block": 0.35}),
    "VortexGenie2": ("vortex_mixer_genie_2",
                     {"body": 2.20, "platform": 0.15, "switch": 0.02, "knob": 0.03}),
}
STATION_ROOT = "/World/TaskArea/Stations"

# 台式机实跑（2026-11-24）报 possibly invalid inertia tensor of {1,1,1} and a negative
# mass 的只有三件，其余部件是被关节树带崩的：
#   rotor / reaction_block —— 一薄板网格的凸包（9.6 mm、95×92 mm）在 PhysX 110 里
#     煮不出有效惯量，等于这一树质量算错；
#   platform —— MJCF 源里就是个挂点，/visuals 和 /collisions 两个分组全空，
#     但 platform_pivot 的 body1 就是它，PhysX 非建 actor 不可。上一版把它
#     rigidBodyEnabled=false 关掉了，负质量照样报（零几何 + density 1000 = 体积 0）。
# 修法跟 A 类工位（LN2Dewar 那批一个都不报）对齐：解析碰撞体 + 显式质量，凸包网格
# 的碰撞一并关掉。尺寸是 scripts 外的探针在各部件**自己**的坐标系里量的
# （网格 extent 角点 × 从部件往下累乘的局部矩阵），所以直接当 Cube 的子级平移写。
# platform 没有网格可量：按仪器壳包围盒顶面（body 系 z=0.1027）往上搁一块板，
# 挂点在 body 系 z=0.0619，差 0.0468 写进部件系平移。
ANALYTIC = {
    ("CentriMini", "rotor"):
        ((0.0, 0.0, 0.0072), (0.0484, 0.0462, 0.0102)),
    ("ThermalCycler", "reaction_block"):
        ((0.0001, 0.0, 0.0031), (0.0554, 0.0370, 0.0048)),
    ("VortexGenie2", "platform"):
        ((0.0, 0.0, 0.0468), (0.0300, 0.0300, 0.0060)),
}

# 惯量修好之后，台式机上冒出来的是另一条更狠的：每个站点 12 个刚体全部
# "Invalid PhysX transform"，世界坐标从 1e13 一路涨到 1e19（v07 老日志里也有 551 条，
# 只是那时质量非法、这几棵树根本没参与积分，看不出来）。源头在 MuJoCo 导入器写的
# 底座焊点 rootJoint_body：physics:body0 是空的（= 焊到世界，锚点系取关节 prim 自己的
# 世界位姿），而 localPos/localRot 全写 0；可 MJCF 的根 body 自带 pos（ThermalCycler
# 还多一个 180° quat），两个 frame 差 2.3~11.6 cm。PhysX 建关节时就打印过
#   CreateJoint - found a joint with disjointed body transforms,
#   the simulation will most likely snap objects together
# ——snap 完第一帧就散架。这三组是把 frame1 拉回 frame0 的解，
# M(lp1,lr1) = frame0 * X_body^-1，在合成后的 v09 舞台上量的（行向量约定，
# 矩阵乘序是 local * parent，反过来写会解出 2.9 m 的假偏移）。
# 只改焊点相对偏移，站点在场景里的位置一动不动。
ROOT_JOINT = {
    "CentriMini": ((0.0, 0.0, -0.023406), (1.0, 0.0, 0.0, 0.0)),
    "ThermalCycler": ((0.0, -0.062156, -0.097352), (0.0, 0.0, 0.0, -1.0)),
    "VortexGenie2": ((0.0, 0.0, -0.029047), (1.0, 0.0, 0.0, 0.0)),
}

# 补完 localPos1 还是飞，于是用 scripts/test_art_stability.py 在台式机上做了一轮
# 四组对照（renderer=None，每组 330 帧左右，读 get_rigidbody_transformation 和作者
# 位姿比）：
#   base     整机离位 1.950 / 2.552 / 3.128 m，纹丝不动 —— 正好是三个站点到原点
#            的距离（CentriMini 作者位 (-1.429,-1.0,0.873)，模长 1.950）。也就是说
#            整站被焊到了世界原点：physics:body0 是空的，PhysX 不认"空 body0 就取
#            关节 prim 自己的世界位姿"这套（USD 转换器约定），它就取世界系原点，
#            而 localPos0/localRot0 又全写 0，frame0 落在 (0,0,0)。
#   single   worldBody 那个零链接的重复 ArticulationRoot 关掉，结果和 base 一字
#            不差 —— 这条嫌疑排除，双根不是原因。
#   off      焊点 jointEnabled=False，12 个部件同向位移 7.36 m，就是自由落体
#            （0.5·9.81·1.2²≈7.1），说明焊点是这几棵树唯一的支撑，不是它坏的。
#   weld     body0 显式指 /World，localPos0/localRot0 写成焊点 prim 自己的世界位姿
#            → 9 个部件 0.000 m 全程稳。
# /World 的世界变换是单位阵，所以 frame0 = M(lp0,lr0) · X_world 不管按行向量约定
# 还是列向量约定算，落点都一样，转换器那套口径分歧在这里自动消失。
# 数值不写死：见 anchor_poses()，每次生成时从合成场景量，站点搬了就跟着变。


def anchor_poses():
    """每个站点 rootJoint_body 这个 prim 的世界位姿（USD 作者值）。

    给 weld 用：写进 physics:localPos0/localRot0，让空 body0 变成"焊在 /World 的
    这个点上"。用 XformCache 沿父链累乘，和试车台里同一套算法，两边数对得上。
    """
    stage = Usd.Stage.Open(SCENE)
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    out = {}
    for station in sorted(INSTRUMENTS):
        m = cache.GetLocalToWorldTransform(
            stage.GetPrimAtPath("%s/%s/joints/rootJoint_body" % (STATION_ROOT, station)))
        rot = m.ExtractRotation().GetQuat()
        out[station] = (tuple(m.ExtractTranslation()),
                        (rot.GetReal(),) + tuple(rot.GetImaginary()))
    return out


def cube_block(box, pad):
    """解析碰撞盒。Cube 的 size 默认 2（半边长 1），乘 physicalSize 就是要的盒子——
    和 v04 修 96 孔板、fix_stock_colliders 修补货资产同一套写法。"""
    (cx, cy, cz), (hx, hy, hz) = box
    return [
        "%sdef Cube \"col_analytic\" (" % pad,
        "%s    prepend apiSchemas = [\"PhysicsCollisionAPI\"]" % pad,
        "%s)" % pad,
        "%s{" % pad,
        "%s    uniform token visibility = \"invisible\"" % pad,
        "%s    double3 xformOp:translate = (%.5f, %.5f, %.5f)" % (pad, cx, cy, cz),
        "%s    float3 xformOp:scale:physicalSize = (%.5f, %.5f, %.5f)" % (pad, hx, hy, hz),
        "%s    uniform token[] xformOpOrder ="
        " [\"xformOp:translate\", \"xformOp:scale:physicalSize\"]" % pad,
        "%s}" % pad,
    ]


def base_groups(stem):
    """_base.usd 里 /visuals 和 /collisions 分组下的网格相对路径（inertia 壳不算）。

    相对路径而不是光秃秃的名字：MJCF 有时在分组和网格之间还垫一层 Xform，
    按名字写会把覆盖块挂到错的位置上，USD 不会报错、只会静默多出个空 prim。
    """
    b = Usd.Stage.Open(os.path.join(INSTR, "configuration/%s_base.usd" % stem))
    out = {}
    for top in ("/visuals", "/collisions"):
        for grp in b.GetPrimAtPath(top).GetChildren():
            prefix = str(grp.GetPath()) + "/"
            out["%s/%s" % (top, grp.GetName())] = sorted(
                str(m.GetPath())[len(prefix):] for m in Usd.PrimRange(grp)
                if m.IsA(UsdGeom.Mesh) and not m.GetName().endswith("_inertia"))
    return out


def scene_state():
    """重新打开 v09，看每个刚体到底拿到多少几何/碰撞体/质量。

    碰撞体分三种数：还在生效的网格、被 collisionEnabled=false 关掉的、解析盒。
    只看"碰撞体个数"看不出退化凸包，必须分开。
    """
    stage = Usd.Stage.Open(SCENE)
    rows = []
    for station in sorted(INSTRUMENTS):
        root = stage.GetPrimAtPath(STATION_ROOT + "/" + station)
        bodies = []
        for p in [x for x in Usd.PrimRange(root) if x.HasAPI(UsdPhysics.RigidBodyAPI)]:
            cols = [x for x in Usd.PrimRange(p) if x.HasAPI(UsdPhysics.CollisionAPI)]
            live = [x for x in cols
                    if x.GetAttribute("physics:collisionEnabled").Get() is not False]
            analytic = [x for x in live if x.GetTypeName() != "Mesh"]
            attr = p.GetAttribute("physics:rigidBodyEnabled")
            bodies.append((p.GetName(),
                           sum(1 for x in Usd.PrimRange(p) if x.IsA(UsdGeom.Mesh)),
                           len([x for x in live if x.GetTypeName() == "Mesh"]),
                           len(analytic),
                           len(cols) - len(live),
                           (p.GetAttribute("physics:mass").Get()
                            if p.HasAttribute("physics:mass") else 0.0) or 0.0,
                           attr.Get() if attr else True))
        rows.append((station, bodies))
    return rows


def show(tag, rows):
    print(tag)
    for station, bodies in rows:
        print("  %-14s %s" % (station, "  ".join(
            "%s[几何%d 网格碰%d 解析%d 已关%d %.3fkg%s]" % (
                n, m, cm, ca, cd, mass, "" if on else " 刚体关")
            for n, m, cm, ca, cd, mass, on in bodies)))


def mesh_blocks(rels, pad, no_collision=False):
    """把 base 里的相对路径列表展开成嵌套的覆盖块（中间层用 def Xform 垫上）。

    no_collision 给换了解析盒的那三件用：碰撞 API 留着（是 crumb 自己加的），
    只把 collisionEnabled 关掉，回头要还原只要删一行。
    """
    return render_tree(build_tree(rels), pad, no_collision)


def build_tree(rels):
    tree = {}
    for rel in rels:
        segs = rel.split("/")
        node = tree
        for seg in segs[:-1]:
            node = node.setdefault(seg, {})
        node[segs[-1]] = None
    return tree


def render_tree(tree, pad, no_collision=False):
    lines = []
    # 值为 None 的是网格叶子，字典的是中间层；叶子排前面，读起来跟 base 一致。
    for name in sorted(tree, key=lambda n: (tree[n] is not None, n)):
        sub = tree[name]
        if sub is None:
            lines.append("%sdef Mesh \"%s\" (" % (pad, name))
            lines.append("%s    prepend apiSchemas = [\"PhysicsCollisionAPI\", "
                         "\"PhysicsMeshCollisionAPI\"]" % pad)
            lines.append("%s)" % pad)
            lines.append("%s{" % pad)
            lines.append("%s    uniform token physics:approximation = \"convexHull\"" % pad)
            if no_collision:
                lines.append("%s    bool physics:collisionEnabled = false" % pad)
            lines.append("%s}" % pad)
        else:
            lines.append("%sdef Xform \"%s\"" % (pad, name))
            lines.append("%s{" % pad)
            lines.extend(render_tree(sub, pad + "    ", no_collision))
            lines.append("%s}" % pad)
    return lines


def overlay_text():
    """生成 overlay 正文：部件接 base 分组，碰撞网格补 CollisionAPI，刚体写质量。"""
    anchors = anchor_poses()
    lines = ["#usda 1.0",
             "# ART 仪器的几何与碰撞体补丁（scripts/fix_instrument_geometry.py 生成，别手改）",
             "#",
             "# crumb 里 visuals/collisions 挂的是空 assetPath 的内部引用，只有 _base.usd",
             "# 自己当锚层时才解析得开，所以场景里这 12 个刚体既没几何也没碰撞体，",
             "# PhysX 算不出惯量、每帧刷 Invalid PhysX transform。这里按部件把 _base.usd 的",
             "# /visuals、/collisions 分组显式引过来，并给 MJCF 拆好的碰撞网格补碰撞 API。",
             "# 质量是真机分量级的估算值：包围盒 × 密度会把 20 g 的调速旋钮估成 1 kg。",
             "#",
             "# joints/rootJoint_body 那几条是底座焊点：MJCF 导入器把 body0 留空、",
             "# localPos/localRot 全写 0，PhysX 于是把整站焊到世界原点（实测离位 = 站点",
             "# 到原点的距离）。这里显式 body0=/World，localPos0/localRot0 取焊点 prim",
             "# 自己的世界位姿，localPos1/localRot1 把 body1 系拉回同一处。",
             "def \"World\"",
             "{",
             "    def \"TaskArea\"",
             "    {",
             "        def \"Stations\"",
             "        {"]
    for station in sorted(INSTRUMENTS):
        stem, masses = INSTRUMENTS[station]
        groups = base_groups(stem)
        rel = "../instruments/configuration/%s_base.usd" % stem
        lines.append("            def \"%s\"" % station)
        lines.append("            {")
        lines.append("                def \"body\"")
        lines.append("                {")
        for part, mass in sorted(masses.items()):
            box = ANALYTIC.get((station, part))
            visuals = groups.get("/visuals/" + part, [])
            collisions = groups.get("/collisions/" + part, [])
            # 没有碰撞分组的部件（CentriMini rotor、ThermalCycler reaction_block）
            # 拿它的视觉网格兼做碰撞体。
            hull_only = not collisions
            used = collisions or visuals
            lines.append("                    def \"%s\"" % part)
            # 属性写在 prim 的花括号里；圆括号只能放 metadata，塞 float 会语法错。
            lines.append("                    {")
            lines.append("                        float physics:mass = %g" % mass)
            if box and not used:
                # platform：两个分组在 base 里就是空的，只写得了解析盒。
                lines.extend(cube_block(box, " " * 24))
                lines.append("                    }")
                continue
            scope = "visuals" if hull_only else "collisions"
            group = ("/visuals/" if hull_only else "/collisions/") + part
            lines.append("                        def \"%s\" (" % scope)
            # 这两个 scope 在 crumb 里是 instanceable=True，子级走 Flattened_Prototype_*。
            # 实例化 prim 上加 references 是完全不生效的（实测内存里改也不长子级），
            # 必须先把 instanceable 关掉，引用才组得出孩子。
            lines.append("                            instanceable = false")
            lines.append("                            prepend references = @%s@<%s>" % (rel, group))
            lines.append("                        )")
            lines.append("                        {")
            lines.extend(mesh_blocks(used, " " * 28, no_collision=bool(box)))
            lines.append("                        }")
            if not hull_only:
                lines.append("                        def \"visuals\" (")
                lines.append("                            instanceable = false")
                lines.append("                            prepend references = @%s@</visuals/%s>" % (rel, part))
                lines.append("                        )")
                # 圆括号后面必须跟花括号，空着写会被 textFileFormat 判成语法错。
                lines.append("                        {")
                lines.append("                        }")
            if box:
                lines.extend(cube_block(box, " " * 24))
            lines.append("                    }")
        lines.append("                }")
        pos1, rot1 = ROOT_JOINT[station]
        pos0, rot0 = anchors[station]
        lines.append("                def \"joints\"")
        lines.append("                {")
        lines.append("                    def \"rootJoint_body\"")
        lines.append("                    {")
        lines.append("                        rel physics:body0 = </World>")
        lines.append("                        point3f physics:localPos0 = "
                     "(%.6f, %.6f, %.6f)" % pos0)
        lines.append("                        quatf physics:localRot0 = "
                     "(%.6f, %.6f, %.6f, %.6f)" % rot0)
        lines.append("                        point3f physics:localPos1 = "
                     "(%.6f, %.6f, %.6f)" % pos1)
        lines.append("                        quatf physics:localRot1 = "
                     "(%.6f, %.6f, %.6f, %.6f)" % rot1)
        lines.append("                    }")
        lines.append("                }")
        lines.append("            }")
    lines += ["        }", "    }", "}", ""]
    return "\n".join(lines)


def mount():
    """把 overlay 挂到 v09 的 sublayer 末尾（最强意见）。"""
    layer = Sdf.Layer.FindOrOpen(SCENE) or Usd.Stage.Open(SCENE).GetRootLayer()
    here = os.path.relpath(OVERLAY, SCENES).replace(os.sep, "/")
    if here in layer.subLayerPaths:
        return "已经挂过了"
    # v09 是 .usda，被 .gitignore 排除在外，动 subLayerPaths 前先自己留一份。
    os.makedirs(BAK, exist_ok=True)
    dst = os.path.join(BAK, "bak_biolab_main_v09.usda")
    if not os.path.exists(dst):
        shutil.copyfile(SCENE, dst)
    layer.subLayerPaths.append(here)
    layer.Export(os.path.abspath(SCENE))
    return "挂上 %s（原场景层备份在 %s）" % (here, os.path.relpath(dst, ROOT))


def main():
    apply = "--apply" in sys.argv
    show("改前：", scene_state())
    text = overlay_text()
    print("\noverlay 正文 %d 行，写到 %s" % (text.count("\n"), OVERLAY))
    if not apply:
        print("（dry-run，加 --apply 才写盘）")
        return
    with open(OVERLAY, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(text)
    print("  %s" % mount())
    print()
    show("改后：", scene_state())


if __name__ == "__main__":
    main()
