"""A 类缺失工位程序化建模 → assets/props/celllab/*.usda + biolab_main_v09.usda

为什么要新建这批：覆盖矩阵里 A1/A2/A4/A5 四条写的是"无生物安全柜/无 CO2 培养箱/
无计数设备/无冷链"，师兄的原话是"现在这个场景不是一个完整的细胞培养实验，
只能完成中间的一部分操作"。这几台补上，任务才从"取放+开盖"扩到"培养/计数/冻存"。

为什么不用 MJCF 转换：2026-11-23 实测那三台 ART 工位（docs/任务套件判据与工位交互体检_20261123.md）
是导入器给的坏产物 —— 12 个部件 0 碰撞体、drive 三项全 0、限位被二次换算成 ±10313°，
而且包围盒互嵌（旋钮嵌在外壳里 1085 cm³），加碰撞就互穿。这一批直接在 USD 层写：
  · 几何只用 Cube / Cylinder 原语 → PhysX 拿到的是 box/cylinder 解析碰撞体，
    不是三角网格，不存在"网格法线坏了 → 负质量/无效惯量"那条路；
  · 关节用 UsdPhysics.RevoluteJoint/PrismaticJoint + DriveAPI，限位按米/度直接写；
  · 外壳壁板是静态碰撞体，**只有当底座的那一块**（base=True）开 kinematic 刚体
    当关节的 body0；其余活动件（门/盖/推拉窗/样品台）是动态刚体，
    这样每台设备最多 2 个活动件，接触对数量可控（W3 那条老约束还在）。

落点不是随便挑的：新物体世界包围盒要先跟 v07 清单里所有刚体和静态碰撞体比一遍，
有任何重叠就拒绝写出 —— 枪头那 14 条 BROKEN 和 −19.5205 m 就是"摆放时已经互穿"的代价。

用法：
    D:/isaacsim-5.1.0/python.bat scripts/build_stations_a_class.py            # 只算几何、自检、报计划
    D:/isaacsim-5.1.0/python.bat scripts/build_stations_a_class.py --write    # 落资产与 v09
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _pxr_env  # noqa: E402

_pxr_env.bootstrap()
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics  # noqa: E402

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROP_DIR = os.path.join(PROJ, "assets", "props", "celllab")
SCENE_DIR = os.path.join(PROJ, "assets", "scenes")
INV = os.path.join(PROJ, "outputs", "task_suite",
                   "scene_inventory_biolab_main_v07.json")

BENCH_TOP = 0.8717        # 实测台面顶面（TableTopCollision_Physics 的 bbox.max z）
BENCH_X = (-1.999, -0.441)
BENCH_Y = (-3.457, 1.027)

W_SHELL = (0.42, 0.45, 0.48)
W_GLASS = (0.62, 0.72, 0.86)
W_STEEL = (0.30, 0.32, 0.35)
W_DARK = (0.18, 0.19, 0.21)


class B:
    """一台设备的一个几何件。rigid=True 才开刚体，碰撞默认都开。

    base=True 是"整机那块不动的底座"： kinematic 刚体（不是静态碰撞体）。
    必须是刚体是因为关节的 body0 只能指向刚体，而 2026-11-23 实测
    （outputs/probe_v09_stations.log）不写 body0 的世界锚定关节在 PhysX 里
    压根不成立：撤掉 ArticulationRootAPI 后 sash 直接坠 203 m。
    """

    def __init__(self, name, size, center, color=W_SHELL, shape="cube",
                 rigid=False, mass=None, collide=True, rot=None, base=False):
        self.name, self.size, self.center = name, size, center
        self.color, self.shape = color, shape
        self.base = base
        self.rigid, self.mass, self.collide = rigid or base, mass, collide
        self.rot = rot or (0.0, 0.0, 0.0)
        self.children = []

    def local_box(self):
        """带子件的刚体根（门、抽屉、盖子）**自己没有几何**，包围盒是子件的并。
        上一版直接拿根上顺手填的名义 size 去算，结果 door 的"盒子"以铰链原点为中心
        垂到台面以下 0.27 m，自检报出一堆"和台面互穿"的假案。"""
        if not self.children:
            return [list(self.center), list(self.size)]
        lo = [float("inf")] * 3
        hi = [float("-inf")] * 3
        for c in self.children:
            sub = c.local_box()
            a, b = sub[0], sub[1]
            # 子件坐标是相对本根的，且根上没写旋转（要转的都在 world_box 那层用
            # 根旋转处理），所以直接平移合成 AABB。
            for i in range(3):
                lo[i] = min(lo[i], self.center[i] + a[i] - b[i] / 2.0)
                hi[i] = max(hi[i], self.center[i] + a[i] + b[i] / 2.0)
        return [lo, hi]

    def world_box(self, root_pos, root_rot_z=0.0):
        """这台设备摆到场景里之后，这个件的世界轴对齐包围盒（局部坐标按
        根节点平移+绕 Z 旋转展开；rot 只允许 0/90 的整数倍，别自欺欺人地
        去算任意姿态的 AABB）。"""
        a, b = self.local_box()
        if self.children:
            lo, hi = a, b          # local_box 已经是中心±半宽的展开形式
        else:
            lo = [a[i] - b[i] / 2.0 for i in range(3)]
            hi = [a[i] + b[i] / 2.0 for i in range(3)]
        if abs(root_rot_z) < 1e-6:
            xlo, xhi = root_pos[0] + lo[0], root_pos[0] + hi[0]
            ylo, yhi = root_pos[1] + lo[1], root_pos[1] + hi[1]
        elif abs(root_rot_z - 90.0) < 1e-6 or abs(root_rot_z + 90.0) < 1e-6:
            # 绕 Z 转 90°：x/y 互换并取反，AABB 的上下界也要跟着换
            xlo, xhi = root_pos[0] - hi[1], root_pos[0] - lo[1]
            ylo, yhi = root_pos[1] + lo[0], root_pos[1] + hi[0]
        else:
            raise ValueError("自检只认 0/±90 的根旋转，别把没算过的姿态写出去")
        return (xlo, xhi, ylo, yhi, root_pos[2] + lo[2], root_pos[2] + hi[2])


def box(name, size, center, **kw):
    return B(name, (abs(size[0]), abs(size[1]), abs(size[2])), center, **kw)


def cyl(name, radius, height, center, **kw):
    b = B(name, (2 * radius, 2 * radius, height), center, shape="cylinder", **kw)
    b.radius, b.height = radius, height
    return b


# ---------------------------------------------------------------------------
# 设备定义：局部坐标原点是"占地中心 × 台面顶面"，+Z 上，−Y 是操作者正对的那一面
#
# 刚体件（门/窗/抽屉/盖/旋钮）的一条铁律：**它的 prim 原点 = 关节锚点**。
# PhysX 里刚体的 position 就是 rigid-body prim 的原点，关节的 localPos 也从
# 同一个锚点起算；上一版把两者分开写（件心在原点、锚点另给），跑起来就是
# "Disjointed body transform"。所以刚体根写成 size (0,0,0) 的空 Xform，
# 真实几何一律放 children（相对锚点的偏移）。
# 每台设备有一块 base=True 的底座（kinematic 刚体），所有关节的 physics:body0
# 都指它。这条是 2026-11-23 用两条相反假设试出来的：
#   · 早先的写法"外壳纯静态 + 关节不写 body0（锚在世界上）"， PhysX 只在
#     PhysicsArticulationRootAPI 下才认这种单侧关节，而根上加 ART 反而让每条
#     关节失去 dof（create_articulation_view 空壳）、运行期写 USD targetPosition
#     读回 0.00 —— 件不飘但完全不动；
#   · 撤掉 ART 根，同一批单侧关节当场从仿真里消失（sash 坠 203.024 m、
#     cnt_lid 201.613 m）—— 世界锚定根本不构成约束。
# 场景里唯一实测能驱动的是 Centrifuge/LidJoint：kinematic 刚体 Body 当 body0、
# 动态 Lid 当 body1、根上**没有** ArticulationRootAPI，通道 A 跑满 +180°。
# 这批工位就照抄那一套。
# ---------------------------------------------------------------------------
def co2_incubator():
    """台式 CO2 培养箱 0.50×0.55×0.62：前开门（竖直铰链）+ 两层搁板 + 12 个皿位。

    门绕 x=−0.255 这条**机身外侧**的竖轴转 100°，扫掠扇形全在 y<−0.278 的空中，
    所以腔内两层搁板（y −0.21..0.25）永远不在门的轨迹里 —— 这是这台设备唯一
    一处"必须先想清楚再写坐标"的地方，改门宽或改限位之前先重算这一段。
    "在空中"只对**负向**成立，所以限位写成 [−100, 0]（理由见下面 joints）。
    """
    HINGE = (-0.255, -0.278, 0.05)     # 门转轴：机身左前棱，落在壳外侧
    KNOB = (0.2775, 0.0, 0.12)         # 气阀装在右侧壁外，避开门的扫掠区
    parts = [
        box("shell_bottom", (0.50, 0.55, 0.04), (0, 0, 0.02), base=True, mass=8.0),
        box("shell_back", (0.50, 0.03, 0.62), (0, 0.26, 0.31)),
        box("shell_left", (0.03, 0.55, 0.62), (-0.235, 0, 0.31)),
        box("shell_right", (0.03, 0.55, 0.62), (0.235, 0, 0.31)),
        box("shell_top", (0.50, 0.55, 0.04), (0, 0, 0.60)),
        box("shelf_lower", (0.44, 0.46, 0.012), (0, 0.02, 0.20)),
        box("shelf_upper", (0.44, 0.46, 0.012), (0, 0.02, 0.40)),
    ]
    door = box("door", (0, 0, 0), HINGE, rigid=True, mass=0.55)
    door.children = [box("panel", (0.44, 0.02, 0.50), (0.2225, -0.01, 0.27),
                         color=W_GLASS),
                     box("handle", (0.03, 0.04, 0.20), (0.19, -0.05, 0.30),
                         color=W_STEEL)]
    parts.append(door)
    knob = cyl("gas_knob", 0, 0, KNOB, rigid=True, mass=0.03)
    knob.children = [cyl("dial", 0.018, 0.03, (0.015, 0, 0), color=W_DARK)]
    parts.append(knob)
    joints = [
        # 开向必须是**负**：门在铰链的 +X 侧，绕 +Z 正向转 100° 时远边的
        # y = −0.278 + 0.73·sin100° ≈ +0.44，直接扫过右侧壁和背板 —— 实测
        # drive 把它顶到 +4.94° 就卡死（outputs/probe_v09_body0.log）。
        # 负向转远角落在 (x≈−0.38, y≈−1.00) 的前左空区，那里没有资产。
        ("door", "revolute", "Z", -100.0, 0.0, (8.0, 2.0, 4.0)),
        # 轴竖直：绕 X 的旋钮在侧壁上，拧 90° = 开气
        ("gas_knob", "revolute", "X", 0.0, 90.0, (4.0, 1.0, 1.5)),
    ]
    slots = slot_grid("ShelfLowerSlots", x0=-0.15, y0=-0.16, dx=0.15, dy=0.16,
                      nx=3, ny=2, z=0.206)
    slots += slot_grid("ShelfUpperSlots", x0=-0.15, y0=-0.16, dx=0.15, dy=0.16,
                       nx=3, ny=2, z=0.406)
    return dict(name="Co2Incubator", parts=parts, joints=joints, slots=slots,
                cn="台式 CO2 培养箱", footprint=(0.50, 0.55), height=0.62)


def biosafety_cabinet():
    """超净台/生物安全柜 1.20×0.60×0.60：前窗竖直滑移（prismatic）+ 台面 6 位锚点。

    "安全柜内操作"这条在覆盖矩阵里是 A1，缺的就是这个能关起来的操作空间：
    前窗落到底 = 封闭，抬到 0.16 m = 手能伸进去。行程不敢给到 0.24 ——
    窗体相对锚点向上还有 0.045 m 的横梁，抬过 0.205 就顶进 cab_top（z 0.55）。
    操作区台面是一块静态碰撞板，上面 3×2 的锚点给 96 孔板和 100 mm 皿当目的地。
    """
    SASH = (0, -0.294, 0.23)           # 滑移原点 = 窗体原点，锚在取物口下沿
    parts = [
        box("cab_base", (1.20, 0.60, 0.05), (0, 0, 0.025),               # 机箱/工作台一体
            base=True, mass=25.0),
        box("cab_back", (1.20, 0.03, 0.55), (0, 0.285, 0.325)),
        box("cab_left", (0.03, 0.60, 0.55), (-0.585, 0, 0.325)),
        box("cab_right", (0.03, 0.60, 0.55), (0.585, 0, 0.325)),
        box("cab_top", (1.20, 0.60, 0.05), (0, 0, 0.575)),
        box("work_surface", (1.12, 0.52, 0.010), (0, 0.01, 0.080)),  # 柜内操作面
    ]
    sash = box("sash", (0, 0, 0), SASH, color=W_GLASS, rigid=True, mass=0.80)
    # 窗心就压在滑移原点上：合窗 z 0.08..0.38 正好盖住整个取物口，
    # 抬到上限 0.16 m 后窗顶 0.54 —— 距离 cab_top 下沿 0.55 还留 10 mm，
    # 这 10 mm 就是行程的真正来源（不是"给个好看的数"）。横梁装在窗**下沿**，
    # 装在上沿的话抬起来会顶穿顶板。
    sash.children = [box("glass", (1.10, 0.012, 0.30), (0, 0, 0),
                         color=W_GLASS),
                     box("pull_bar", (0.30, 0.03, 0.03), (0, -0.02, -0.13),
                         color=W_STEEL)]
    parts.append(sash)
    joints = [("sash", "prismatic", "Z", 0.0, 0.16, (200.0, 40.0, 60.0))]
    slots = slot_grid("WorkSlots", x0=-0.36, y0=-0.14, dx=0.36, dy=0.24,
                      nx=3, ny=2, z=0.085)
    return dict(name="BiosafetyCabinet", parts=parts, joints=joints, slots=slots,
                cn="生物安全柜/超净台", footprint=(1.20, 0.60), height=0.60)


def cell_counter():
    """自动细胞计数仪 0.28×0.28×0.34：样品仓抽屉（prismatic 进出）+ 仓盖（revolute）。

    计数**读数**仍然判不出来（无细胞状态量，B8 那条不动），能判的是
    "把计数板推进仓里"和"把仓盖盖上" —— 这就是把 A4 从"没有设备"变成
    "有设备、有交互、但结果不可见"，两者在任务规格里是完全不同的待遇。
    """
    TRAY = (0, -0.09, 0.215)           # 抽屉滑移原点（= 抽屉体原点）
    LID = (0, 0.075, 0.325)            # 铰链在后挡板**之外**，翻起时不穿过后壁
    parts = [
        box("cnt_body", (0.28, 0.28, 0.20), (0, 0, 0.10), base=True, mass=3.0),
        box("cnt_hood_left", (0.012, 0.20, 0.12), (-0.134, -0.03, 0.26)),
        box("cnt_hood_right", (0.012, 0.20, 0.12), (0.134, -0.03, 0.26)),
        box("cnt_hood_back", (0.28, 0.012, 0.12), (0, 0.064, 0.26)),
        box("cnt_hood_top", (0.28, 0.20, 0.012), (0, -0.03, 0.314)),
    ]
    tray = box("tray", (0, 0, 0), TRAY, color=W_STEEL, rigid=True, mass=0.06)
    # 托盘底面深度只到 0.12：再深一格就插进后挡板（y 0.058..0.070），
    # 而"互穿的第 0 帧"就是枪头那 14 条 BROKEN 的成因，不能带病写出去。
    tray.children = [box("tray_floor", (0.16, 0.12, 0.006), (0, 0.085, -0.003),
                         color=W_STEEL),
                     box("tray_grip", (0.05, 0.008, 0.02), (0, 0.025, 0.012),
                         color=W_DARK)]
    parts.append(tray)
    lid = box("cnt_lid", (0, 0, 0), LID, color=W_DARK, rigid=True, mass=0.08)
    lid.children = [box("lid_plate", (0.26, 0.18, 0.008), (0, -0.105, 0.004),
                        color=W_DARK)]
    parts.append(lid)
    joints = [
        # 两个行程都写成"负值 = 打开"：抽屉往操作者这侧（−Y）拉出，
        # 仓盖绕 +X 转轴负向翻起（正向是往**下**插进机身里）。
        ("tray", "prismatic", "Y", -0.11, 0.0, (120.0, 20.0, 25.0)),
        ("cnt_lid", "revolute", "X", -100.0, 0.0, (3.0, 0.8, 1.2)),
    ]
    slots = [dict(name="TraySlot", pos=[0.0, 0.02, 0.221])]
    return dict(name="CellCounter", parts=parts, joints=joints, slots=slots,
                cn="自动细胞计数仪", footprint=(0.28, 0.28), height=0.33)


def cryobox():
    """25 位冻存盒 0.13×0.13×0.06，滑盖（prismatic，往 −X 抽出）+ 5×5 孔位锚点。

    上一版这里写的是翻盖（revolute 105°），算完就否掉了：盒壁只有 0.055 m 高，
    盖心离转轴 0.064 m，绕水平的盒沿棱转起来，
        z = 0.065 − 0.128·sinθ ≥ 0  →  θ ≤ 30.5°
    —— 意思是"合上"和"完全打开"在物理上根本不可能同时成立，判据怎么写都是错的。
    换成滑盖：抽 0.10 m 就露出全部 25 个孔位，行程/限位都是干净的线性关系。
    """
    LID = (0, 0, 0.068)
    parts = [
        box("box_bottom", (0.13, 0.13, 0.010), (0, 0, 0.005), base=True, mass=0.2),
        box("box_wall_n", (0.13, 0.006, 0.055), (0, 0.062, 0.0375)),
        box("box_wall_s", (0.13, 0.006, 0.055), (0, -0.062, 0.0375)),
        box("box_wall_e", (0.006, 0.118, 0.055), (0.062, 0, 0.0375)),
        box("box_wall_w", (0.006, 0.118, 0.055), (-0.062, 0, 0.0375)),
    ]
    lid = box("cryo_lid", (0, 0, 0), LID, rigid=True, mass=0.04)
    lid.children = [box("lid_plate", (0.128, 0.128, 0.006), (0, 0, 0),
                        color=W_SHELL)]
    parts.append(lid)
    joints = [("cryo_lid", "prismatic", "X", -0.10, 0.0, (60.0, 12.0, 20.0))]
    slots = slot_grid("BoxSlots", x0=-0.048, y0=-0.048, dx=0.024, dy=0.024,
                      nx=5, ny=5, z=0.010)
    return dict(name="CryoBox25", parts=parts, joints=joints, slots=slots,
                cn="25 位冻存盒", footprint=(0.13, 0.13), height=0.074)


def ln2_dewar():
    """液氮罐 Ø0.24×0.385，铰链盖（绕竖直轴转开的圆盘）+ 6 个提筒位。

    罐口的盖不能是"翻起来"的：Ø0.164 的圆盘绕罐沿切线翻 90° 会横扫到罐身自己。
    这里转轴取 y=+0.13（罐口后沿外侧）的**竖直**轴，盖片相对锚点 (0,−0.07,−0.0025)，
    合盖位下表面 z=0.37 正好坐在颈口顶面上（零重叠，靠 drive 保持位而不是靠穿透）。

    颈圈 dewar_neck 故意**不开碰撞**（纯视觉）：原语只有实心 Cube/Cylinder，
    给颈圈挂碰撞就等于在罐口塞了一颗实心塞子，冻存管根本放不进去。
    同理提筒锚点 z=0.342 是"坐在罐体顶面（=内胆底 0.34）上方 2 mm"，
    而"管内放好冻存管之后再合盖"这条组合是写不出来的 —— 盖片下沿 0.37，
    管子 50 mm 高会顶穿。这是资产给的硬约束，记进工位体检文档。
    """
    LID = (0, 0.13, 0.385)
    parts = [
        cyl("dewar_body", 0.12, 0.34, (0, 0, 0.17), color=W_STEEL,
            base=True, mass=6.0),
        cyl("dewar_neck", 0.075, 0.04, (0, 0, 0.35), color=W_STEEL, collide=False),
    ]
    lid = cyl("dewar_lid", 0, 0, LID, rigid=True, mass=0.05)
    lid.children = [cyl("lid_disc", 0.082, 0.025, (0, -0.07, -0.0025), color=W_DARK)]
    parts.append(lid)
    joints = [("dewar_lid", "revolute", "Z", 0.0, 120.0, (2.5, 0.6, 1.0))]
    slots = []
    import math as _m
    for i in range(6):
        a = _m.radians(60 * i)
        slots.append(dict(name="Canister_%02d" % i,
                          pos=[round(0.055 * _m.cos(a), 4),
                               round(0.055 * _m.sin(a), 4), 0.342]))
    return dict(name="LN2Dewar", parts=parts, joints=joints, slots=slots,
                cn="液氮罐", footprint=(0.24, 0.24), height=0.3975)


def slot_grid(name, x0, y0, dx, dy, nx, ny, z):
    out = []
    for r in range(ny):
        for c in range(nx):
            out.append(dict(name="%s_%d%d" % (name, r, c),
                            pos=[round(x0 + c * dx, 4), round(y0 + r * dy, 4), z]))
    return out


STATIONS = [co2_incubator, biosafety_cabinet, cell_counter, cryobox, ln2_dewar]

# 世界落点：全部放在台面北段与东侧的空区（y>−0.6 且 x>−1.0 的东侧窄条）。
# 数字来自 2026-11-23 对 v07 清单占地做的空区核对，见 scripts/_space.py。
PLACEMENT = {
    "Co2Incubator": ((-1.72, 0.02, BENCH_TOP), 0.0),
    "BiosafetyCabinet": ((-1.22, 0.70, BENCH_TOP), 0.0),
    "CellCounter": ((-0.78, -0.10, BENCH_TOP), 0.0),
    "CryoBox25": ((-0.78, 0.28, BENCH_TOP), 0.0),
    "LN2Dewar": ((-0.70, -0.52, BENCH_TOP), 0.0),
}


# ---------------------------------------------------------------------------
# 几何自检
# ---------------------------------------------------------------------------
def self_check():
    """返回 (错误列表, 世界包围盒列表)。三件事一次查完：
    1. 每台设备自己有没有把刚体写进外壳里（门/窗/抽屉关闭位必须与静件零重叠）；
    2. 新落点与 v07 已有刚体/静态碰撞体有没有互穿；
    3. 有没有越过台面边界（越过就等于悬空，第 0 帧掉下去）。
    """
    errs, boxes = [], []
    per_dev = {}

    def dyn(p):
        """会被求解器推着走的那些件；底座是 kinematic，互穿规则里等同静态。"""
        return p.rigid and not p.base
    for f in STATIONS:
        spec = f()
        root, rotz = PLACEMENT[spec["name"]]
        mine = []
        for p in spec["parts"]:
            wb = p.world_box(root, rotz)
            mine.append((p, wb))
            boxes.append((spec["name"], p.name, wb, p.rigid))
        per_dev[spec["name"]] = mine
        for i in range(len(mine)):
            for j in range(i + 1, len(mine)):
                pa, a = mine[i][0], mine[i][1]
                pb, b = mine[j][0], mine[j][1]
                # 静–静不算病：一台壳子由 6 块壁板拼成，角部必然相接，
                # 而 PhysX 不会在两个静态碰撞体之间生成接触。
                # 只有**至少一边是活动刚体**时，互穿才会变成第 0 帧的弹飞。
                # 底座（base=True）算静的那一侧：它是 kinematic，求解器不会
                # 拿接触去推它，所以壁板跟它共角跟"静–静"同理不算病。
                if not (dyn(pa) or dyn(pb)):
                    continue
                ov = overlap3(a, b)
                if ov > 1e-8:
                    errs.append("%s：%s%s 与 %s%s 重叠 %.1f cm³"
                                % (spec["name"], pa.name, "(刚)" if dyn(pa) else "",
                                   pb.name, "(刚)" if dyn(pb) else "", ov * 1e6))
        #  footprint 越界
        xs = [wb[0] for _, wb in mine] + [wb[1] for _, wb in mine]
        ys = [wb[2] for _, wb in mine] + [wb[3] for _, wb in mine]
        if min(xs) < BENCH_X[0] or max(xs) > BENCH_X[1] or \
           min(ys) < BENCH_Y[0] or max(ys) > BENCH_Y[1]:
            errs.append("%s 超出台面：x %.3f..%.3f y %.3f..%.3f（台面 x %.3f..%.3f y %.3f..%.3f）"
                        % (spec["name"], min(xs), max(xs), min(ys), max(ys),
                           BENCH_X[0], BENCH_X[1], BENCH_Y[0], BENCH_Y[1]))
    if not os.path.exists(INV):
        errs.append("缺盘点文件 %s，无法与既有场景比邻" % INV)
        return errs, boxes
    with open(INV, encoding="utf-8") as fh:
        inv = json.load(fh)
    old = []
    for key in ("rigid_bodies", "static_colliders"):
        for rec in inv.get(key, []):
            b = rec.get("bbox")
            if b:
                old.append((rec["path"],
                            (b["min"][0], b["max"][0], b["min"][1], b["max"][1],
                             b["min"][2], b["max"][2])))
    for dev, pname, wb, _r in boxes:
        for path, ob in old:
            # 竖直重叠深度：贴在面上（0.3 mm 级，来自既有资产包围盒自身的
            # 1 mm 作者误差）不算病；埋进去 2 mm 以上才算 —— ART 那批
            # "旋钮嵌在外壳里 1085 cm³"是这条阈值的 100 倍。
            depth = min(wb[5], ob[5]) - max(wb[4], ob[4])
            if depth <= 0.002:
                continue
            area = surf_overlap(wb, ob)
            if area > 4e-6:
                errs.append("%s/%s 埋进既有 %s：竖直 %.1f mm、水平 %.1f cm²"
                            % (dev, pname, path.replace("/World/", ""),
                               1000 * depth, area * 1e4))
    return errs, boxes


def overlap3(a, b):
    v = 1.0
    for lo, hi in ((0, 1), (2, 3), (4, 5)):
        d = min(a[hi], b[hi]) - max(a[lo], b[lo])
        if d <= 0:
            return 0.0
        v *= d
    return v


def surf_overlap(a, b):
    dx = min(a[1], b[1]) - max(a[0], b[0])
    dy = min(a[3], b[3]) - max(a[2], b[2])
    return 0.0 if dx <= 0 or dy <= 0 else dx * dy


# ---------------------------------------------------------------------------
# USD 写出
# ---------------------------------------------------------------------------
def write_asset(spec, path):
    if os.path.exists(path):
        os.remove(path)
    layer = Sdf.Layer.CreateNew(path)
    if layer is None:
        raise SystemExit("建不出 %s" % path)
    stage = Usd.Stage.Open(layer)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = stage.DefinePrim("/" + spec["name"], "Xform")
    Usd.CreateXformStageRootAsRoot(stage, root) if hasattr(Usd, "CreateXformStageRootAsRoot") else None
    for p in spec["parts"]:
        if p.base and not p.children:
            hoist_geom_into_child(p)
        emit_part(stage, "/" + spec["name"], p)
    emit_joints(stage, spec)
    emit_slots(stage, spec)
    layer.Export(os.path.abspath(path))
    return stage


def hoist_geom_into_child(p):
    """底座这块刚体的几何挪进唯一子件，根上只留 translate（零尺寸、零缩放）。

    ①Cube 的 size 是靠一个 scale op 表达的，刚体根带缩放时 PhysX 报
      "Transform has scale"，而关节 localPos0 是相对 body0 的刚体局部系表达的 ——
      这个数到底被不被缩放，没人能替我保证；
    ②场景里唯一实测能驱动的那条关节（5430 机盖）对端 kinematic Body 正是
      "根只有 translate、几何在 children"的形状。
    self_check 在这之前跑，它按 center±size/2 算 AABB，挪法同值，不影响互穿判定。
    """
    geo = B(p.name + "_geom", p.size, (0.0, 0.0, 0.0), color=p.color,
            shape=p.shape, collide=p.collide)
    if p.shape == "cylinder":
        geo.radius, geo.height = p.radius, p.height
    p.children = [geo]
    p.size = (0.0, 0.0, 0.0)
    return p


def emit_part(stage, parent, p, depth=0):
    path = parent + "/" + p.name
    if p.children:
        # 带子件的刚体根（门/抽屉/盖）只是骨架，本体不带几何 ——
        # 上一版在这里建了一个 size=(0,0,0) 的 Cube，PhysX 见到零尺寸碰撞体
        # 直接给无效质量，整个关节链在启动时就废了。
        prim = stage.DefinePrim(path, "Xform")
    elif p.shape == "cylinder":
        prim = stage.DefinePrim(path, "Cylinder")
        prim.GetAttribute("radius").Set(p.radius)
        prim.GetAttribute("height").Set(p.height)
        prim.GetAttribute("axis").Set("Z")
    else:
        prim = stage.DefinePrim(path, "Cube")
        prim.GetAttribute("size").Set(1.0)
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*p.center))
    if any(abs(v) > 1e-9 for v in p.rot):
        r = xf.AddRotateXYZOp()
        r.Set(Gf.Vec3f(*p.rot))
    if not p.children and p.shape == "cube":
        xf.AddScaleOp().Set(Gf.Vec3f(*p.size))
    if not p.children:
        UsdGeom.Imageable(prim).MakeVisible()
        # displayColor 是 primvar，不是普通 attribute —— 上一版直接
        # GetAttribute("displayColor").Set(...) 建出来的是个无类型的空属性，
        # 报 "Empty typeName for </Co2Incubator/shell_bottom.displayColor>"。
        UsdGeom.PrimvarsAPI(prim).CreatePrimvar(
            "displayColor", Sdf.ValueTypeNames.Color3fArray,
            UsdGeom.Tokens.constant).Set([Gf.Vec3f(*p.color)])
    if p.rigid:
        rb = UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr().Set(p.mass or 0.1)
        if p.base:
            # 底座：kinematic = PhysX 里的"不可动的锚"。求解器当它是无限质量，
            # 但关节有合法的对端 → 关节真的进了仿真图。这正是 5430 离机盖
            # 那条唯一实测能驱动的关节的拓扑（Body kinematic ← Lid 动态）。
            rb.CreateKinematicEnabledAttr().Set(True)
    # 刚体的**几何子节点**带碰撞；刚体根本身不带（那是导入器那批
    # "外壳既是刚体又有网格"的麻烦源头）。静态件则直接自己带碰撞。
    if p.children:
        for c in p.children:
            emit_part(stage, path, c, depth + 1)
            if c.collide:
                UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(parent + "/" + p.name + "/" + c.name))
    elif p.collide:
        UsdPhysics.CollisionAPI.Apply(prim)
    return prim


def emit_joints(stage, spec):
    root = "/" + spec["name"]
    by_name = {p.name: p for p in spec["parts"]}
    bases = [p.name for p in spec["parts"] if p.base]
    if len(bases) != 1:
        raise SystemExit("%s：底座必须恰好一块（找到 %d 块：%s），"
                         "关节的 body0 只能有一个锚" % (root, len(bases), bases))
    scope = stage.DefinePrim(root + "/Joints", "Scope")
    for (nm, kind, axis, lo, hi, drive) in spec["joints"]:
        part = by_name[nm]
        jp = root + "/Joints/" + nm
        if kind == "revolute":
            j = UsdPhysics.RevoluteJoint.Define(stage, Sdf.Path(jp))
        else:
            j = UsdPhysics.PrismaticJoint.Define(stage, Sdf.Path(jp))
        prim = stage.GetPrimAtPath(jp)
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        # 关节锚点 = 刚体 prim 原点，二者必须同一个数（见文件头那条铁律）。
        # 轴靠 axis 字母选，不靠旋转 op：写 rotateXYZ 会让 localPos 的语义跟着转，
        # 那就又是一次"两个坐标系互相追"的坑。
        xf.AddTranslateOp().Set(Gf.Vec3d(*part.center))
        prim.GetRelationship("physics:body1").SetTargets([Sdf.Path(root + "/" + nm)])
        # body0 指向底座那块 kinematic 刚体。**不能留空**：留空 = "锚在世界上"，
        # 而实测（outputs/probe_v09_noart.log）这种单侧关节在 PhysX 里根本不成约束，
        # 撤掉 ArticulationRootAPI 后窗直接坠 203 m。写 body0 之后根上也就不需要
        # ART 根（5430 机盖那条能动的关节正是这个形状）。
        base = by_name[bases[0]]
        prim.GetRelationship("physics:body0").SetTargets([Sdf.Path(root + "/" + base.name)])
        # localPos0 必须写：关节 prim 摆在 body1 的原点上（上面那条铁律），
        # 而 base 的原点在别处 —— 不写 localPos0 就是叫 PhysX"关节系 = body0 系"，
        # 它照办并把件**瞬移**过去，日志里那句
        #   CreateJoint - found a joint with disjointed body transforms,
        #   the simulation will most likely snap objects together
        # 实测把门甩到别处、drive 从此推不动（outputs/probe_v09_body0.log，
        # 7 条关节全部中招、最大偏移 0.5837 m）。两个件都没有旋转，所以
        # localPos0 就是两个原点之差，写在 body0 的局部系里。
        if any(abs(v) > 1e-9 for v in base.rot) or any(abs(v) > 1e-9 for v in part.rot):
            raise SystemExit("%s/%s：base 或活动件带旋转时 localPos0 不再是简单差值，"
                             "要补 localRot0 才能写" % (root, nm))
        j.GetLocalPos0Attr().Set(Gf.Vec3f(*[part.center[i] - base.center[i]
                                            for i in range(3)]))
        j.GetAxisAttr().Set(axis)
        j.GetLowerLimitAttr().Set(lo)
        j.GetUpperLimitAttr().Set(hi)
        api = "angular" if kind == "revolute" else "linear"
        d = UsdPhysics.DriveAPI.Apply(prim, api)
        d.CreateStiffnessAttr().Set(drive[0])
        d.CreateDampingAttr().Set(drive[1])
        d.CreateMaxForceAttr().Set(drive[2])
        d.CreateTargetPositionAttr().Set(0.0)
        # 类型/行程/轴写进 customData：语义层和判据脚本都靠它认"这条轴该怎么读"，
        # 不用再去猜 joint prim 上那个 axis 字母对应世界的哪个方向。
        # 注意 SetCustomData 只收一个 VtValue（整个 dict），没有 key 这个重载。
        prim.SetCustomData({"ccl:joint": {"kind": kind, "lo": lo, "hi": hi,
                                          "axis": axis, "part": root + "/" + nm,
                                          "body0": root + "/" + bases[0]}})



def emit_slots(stage, spec):
    root = "/" + spec["name"]
    for s in spec["slots"]:
        p = stage.DefinePrim(root + "/SlotAnchors/" + s["name"], "Xform")
        UsdGeom.Xformable(p).ClearXformOpOrder()
        UsdGeom.Xformable(p).AddTranslateOp().Set(Gf.Vec3d(*s["pos"]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--write", action="store_true")
    args = ap.parse_args()

    errs, boxes = self_check()
    print("新工位 %d 台，几何件 %d 个，槽位锚点 %d 个，关节 %d 条"
          % (len(STATIONS), len(boxes),
             sum(len(f()["slots"]) for f in STATIONS),
             sum(len(f()["joints"]) for f in STATIONS)))
    for f in STATIONS:
        s = f()
        root, rotz = PLACEMENT[s["name"]]
        print("   %-20s 占地 %5.0f×%5.0f mm 高 %4.0f mm  静件 %d 动件 %d(底座另计) "
              "锚点 %2d 关节 %d  落点 (%.2f, %.2f)%s"
              % (s["name"], 1000 * s["footprint"][0], 1000 * s["footprint"][1],
                 1000 * s["height"],
                 sum(1 for p in s["parts"] if not p.rigid),
                 sum(1 for p in s["parts"] if p.rigid and not p.base),
                 len(s["slots"]),
                 len(s["joints"]), root[0], root[1],
                 "  绕Z %.0f°" % rotz if rotz else ""))
    if errs:
        print("\n!! 几何自检 %d 条问题：" % len(errs))
        for e in errs[:40]:
            print("   ", e)
        if len(errs) > 40:
            print("    …另有 %d 条" % (len(errs) - 40))
    else:
        print("\n几何自检通过：设备内部零互穿、与既有资产零互穿、都在台面内。")
    if not args.write:
        print("（试跑，没写文件。加 --write 才落 %s）" % PROP_DIR)
        return 1 if errs else 0

    os.makedirs(PROP_DIR, exist_ok=True)
    for f in STATIONS:
        s = f()
        write_asset(s, os.path.join(PROP_DIR, s["name"] + ".usda"))
        print("→ assets/props/celllab/%s.usda" % s["name"])
    print("（--write 只落单体资产；场景合成走 scripts/compose_v09.py）")
    return 1 if errs else 0


if __name__ == "__main__":
    sys.exit(main())
