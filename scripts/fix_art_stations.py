"""把 3 台 ART 工位从"只有关节、没有物理"改成"确实能交互"，另存 v08。

为什么要动这个：师兄那句"场景主要是要能实现交互，不是只要像就可以了"。
2026-11-23 实测（scripts/_probe_art.py）：

    CentriMini     刚体 3 个 / 带碰撞 0 个
    ThermalCycler  刚体 5 个 / 带碰撞 0 个
    VortexGenie2   刚体 4 个 / 带碰撞 0 个

三台仪器的所有部件——外壳、盖子、转子、反应板、涡旋平台——**一个碰撞体都没有**。
关节是绑上了（body0/body1 全有），drive schema 也在，但：

    drive: stiffness 0.0, damping 0.0, maxForce 0.0

也就是施加的力矩恒等于 0。所以"盖子一步不动"根本不用等报错，原因就在这行。
对照能用的两台：Centrifuge 盖子 stiffness 4.0 / damping 1.0 / maxForce 20.0，
Pipette 按键 80.0 / 2.0 / 8.0 —— 那两台是真的开得了。

还有一处 MJCF→USD 的单位二次换算：
    CentriMini/joints/rotor          upper = +10313.24°   ← 180 rad 又乘了一次 57.3
    ThermalCycler/lid_force_knob     upper = +3437.75°    ← 60 rad
    VortexGenie2/knob                upper =  ±263.56°    ← 4.6 rad，这个还行
超过一整圈的"限位"等于没有限位，策略可以把它转到任意圈数，判据里的
open_deg 就没有意义了。这一版把 |limit| > 360° 的钳到 ±90°。

三件事一起做（都在同一批 prim 上，分开做要跑三次 GPU）：
1. 部件加 CollisionAPI + 显式质量（负质量/无效惯量那批报错就是这么来的：
   没有 collision 也就没有可信的惯性，PhysX 自己去网格积分，网格法线是坏的）；
2. drive 给非零 stiffness/damping/maxForce；
3. 限位钳位。

派生场景另存新路径，不覆盖 v07 —— docs 里立过的规矩。

用法：
    D:/isaacsim-5.1.0/python.bat scripts/fix_art_stations.py           # 只报计划
    D:/isaacsim-5.1.0/python.bat scripts/fix_art_stations.py --write    # 落 v08
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _pxr_env  # noqa: E402

_pxr_env.bootstrap()
from pxr import Sdf, Usd, UsdGeom, UsdPhysics  # noqa: E402

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(PROJ, "assets", "scenes", "biolab_main_v07.usda")
OUT = os.path.join(PROJ, "assets", "scenes", "biolab_main_v08.usda")

STATIONS = ["CentriMini", "ThermalCycler", "VortexGenie2"]

# 每个部件按"是不是会动的"给质量。数值是同类仪器的量级，不是标定结果：
# 5430 转子（已能用的那台）驱动 maxForce 20 N·m 带的就是几十克量级的负载。
MASS_BY_ROLE = {"shell": 0.800, "lid": 0.060, "plate": 0.050, "knob": 0.030}
SHELL_WORDS = ("body", "shell", "base", "housing")
LID_WORDS = ("lid", "cover")
PLATE_WORDS = ("rotor", "platform", "block", "tray", "plate")
KNOB_WORDS = ("knob", "switch", "lever", "button")

DRIVE_DEFAULT = {"stiffness": 4.0, "damping": 1.0, "maxForce": 20.0}
LIMIT_CLAMP_DEG = 360.0


def role_of(name):
    n = name.lower()
    for words, role in ((SHELL_WORDS, "shell"), (LID_WORDS, "lid"),
                        (PLATE_WORDS, "plate"), (KNOB_WORDS, "knob")):
        if any(w in n for w in words):
            return role
    return "plate"


def bbox(prim, cache):
    try:
        r = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    except Exception:
        return None
    if r.IsEmpty():
        return None
    return list(r.GetMin()), list(r.GetMax())


def overlap(a, b):
    """两个轴对齐盒的重叠体积（立方米）。0 表示不相互穿。"""
    v = 1.0
    for i in range(3):
        d = min(a[1][i], b[1][i]) - max(a[0][i], b[0][i])
        if d <= 0:
            return 0.0
        v *= d
    return v


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--write", action="store_true")
    ap.add_argument("--with-collision", action="store_true",
                    help="第二批改动：连 CollisionAPI + 显式质量一起加。"
                         "会引入新接触，而实测这些部件的包围盒互相嵌着"
                         "（旋钮嵌在外壳里 1085 cm³），网格真互穿的话第 0 帧就弹飞，"
                         "所以默认不做，先只做 drive 与限位。")
    args = ap.parse_args()

    src = Usd.Stage.Open(SRC)
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])

    plan_col, plan_drv, plan_lim = [], [], []
    warn = []
    for stn in STATIONS:
        root = src.GetPrimAtPath("/World/TaskArea/Stations/" + stn)
        if not root:
            warn.append("工位 %s 不存在" % stn)
            continue
        boxes = []
        for p in Usd.PrimRange(root):
            if not p.HasAPI(UsdPhysics.RigidBodyAPI):
                continue
            if not is_root_body(p):
                continue
            nm = p.GetName()
            boxes.append((str(p.GetPath()), role_of(nm), bbox(p, cache)))
            # 这一批必须听 --with-collision：上一版无条件 append，
            # 结果只该做 drive+限定的 A 批把 12 个部件的网格碰撞体一起写进了 v08，
            # 而那些网格互相嵌套（旋钮嵌在外壳里 1085 cm³），开局就互穿。
            if args.with_collision and not p.HasAPI(UsdPhysics.CollisionAPI):
                plan_col.append((str(p.GetPath()), role_of(nm)))
        for path, role, bb in boxes:
            if bb is None:
                warn.append("%s 量不出包围盒" % path)
        for i in range(len(boxes)):
            for j in range(i + 1, len(boxes)):
                a, b = boxes[i][2], boxes[j][2]
                if not a or not b:
                    continue
                ov = overlap(a, b)
                if ov > 1e-7:
                    warn.append("加碰撞后会相互穿：%s 与 %s 重叠 %.1f cm³"
                                % (boxes[i][0].split("/")[-1], boxes[j][0].split("/")[-1],
                                   ov * 1e6))
        # MJCF 导入器给的关节类型名带 Physics 前缀（PhysicsRevoluteJoint），
        # drive 属性名是 drive:<kind>:physics:<参数>。这两个名字直接抄
        # survey_scene_assets.py 里已经跑通的那套，不再猜第二次。
        for p in Usd.PrimRange(root):
            tn = str(p.GetTypeName())
            if tn not in ("PhysicsRevoluteJoint", "PhysicsPrismaticJoint"):
                continue
            for attr in ("lowerLimit", "upperLimit"):
                a = p.GetAttribute("physics:%s" % attr)
                if a is None or not a.HasAuthoredValue():
                    continue
                v = a.Get()
                if v is None or abs(v) <= LIMIT_CLAMP_DEG:
                    continue
                plan_lim.append((str(p.GetPath()), attr, float(v),
                                 -LIMIT_CLAMP_DEG if v < 0 else LIMIT_CLAMP_DEG))
            kind = "angular" if tn == "PhysicsRevoluteJoint" else "linear"
            for nm in ("stiffness", "damping", "maxForce"):
                an = "drive:%s:physics:%s" % (kind, nm)
                a = p.GetAttribute(an)
                if a is None or not a.IsValid():
                    continue
                cur = a.Get()
                if cur:
                    continue
                plan_drv.append((str(p.GetPath()), an, cur, DRIVE_DEFAULT[nm]))

    print("计划：加碰撞 %d 个部件，改 drive %d 个属性，钳限位 %d 个"
          % (len(plan_col), len(plan_drv), len(plan_lim)))
    for p, role in plan_col:
        print("   碰撞 + mass %-6.6s %s" % (MASS_BY_ROLE[role], p.replace("/World/TaskArea/Stations/", "~")))
    for p, attr, cur, new in plan_drv:
        print("   drive %s: %s → %s   (%s)" % (p.replace("/World/TaskArea/Stations/", "~"),
                                               attr.split(":")[-1], new, cur))
    for p, attr, cur, new in plan_lim:
        print("   限位 %s.%s: %.1f → %.1f" % (p.replace("/World/TaskArea/Stations/", "~"),
                                              attr, cur, new))
    for w in warn:
        print("!!", w)
    if not args.write:
        print("（试跑，没写文件。加 --write 才落 %s）" % os.path.basename(OUT))
        return 0

    if os.path.exists(OUT):
        os.remove(OUT)   # CreateNew 不覆盖已有层；这是本脚本自己产出的派生场景
    out_layer = Sdf.Layer.CreateNew(OUT)
    if out_layer is None:
        raise SystemExit("建不出 %s" % OUT)
    # 底下垫的是语义层而不是 v07：semantics_v07.usda 自己已经 sublayer 了 v07，
    # 这样 v08 一个文件就同时带语义标签和这一批物理修正，任务套件的
    # base_scene 指哪都行，不用记两层。
    base = os.path.join(PROJ, "assets", "scenes", "semantics_v07.usda")
    if not os.path.exists(base):
        base = SRC
        print("!! 没有语义层 %s，退回直接垫 v07"
              % os.path.relpath(base, PROJ).replace("\\", "/"))
    out_layer.subLayerPaths.append(os.path.relpath(base, os.path.dirname(OUT)).replace("\\", "/"))
    st = Usd.Stage.Open(out_layer)
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)

    n = 0
    for path, role in plan_col:
        prim = st.OverridePrim(path)
        UsdPhysics.CollisionAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr().Set(MASS_BY_ROLE[role])
        n += 1
    for path, attr, _cur, new in plan_drv:
        st.OverridePrim(path).GetAttribute(attr).Set(new)
        n += 1
    for path, attr, _cur, new in plan_lim:
        prim = st.OverridePrim(path)
        a = prim.GetAttribute("physics:%s" % ("lowerLimit" if attr == "lowerLimit" else "upperLimit"))
        a.Set(new)
        n += 1
    out_layer.Export(os.path.abspath(OUT))
    print("写入 %d 处改动 → %s" % (n, os.path.relpath(OUT, PROJ).replace("\\", "/")))
    return 0


def is_root_body(prim):
    """只取刚体根：ART 部件都是 body/<name> 一层，父级有刚体的不重复算。"""
    p = prim.GetParent()
    while p:
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            return False
        p = p.GetParent()
    return True


if __name__ == "__main__":
    sys.exit(main())
