"""
用 Isaac Sim 官方 MJCF 导入器，把 AutoBio 上游的仪器（MJCF + obj）转成带物理的 USD。

为什么用官方导入器：上游 model/instrument/*.xml 里带着**关节的 body 绑定、轴向、限位、
弹簧刚度与阻尼**，师兄那批 USD 恰恰把这些丢了（LidJoint 的 body0/body1 是空的）。
官方导入器按 XML 原样建关节，不用猜。

用法（headless，不用开 GUI）：
    PY="D:/IsaacSim5/env/Scripts/python.exe"
    $PY scripts/convert_instruments.py --probe                  # 只看导入器有哪些开关
    $PY scripts/convert_instruments.py                          # 转全部 6 台
    $PY scripts/convert_instruments.py vortex_mixer_genie_2     # 只转一台
产物：assets/instruments/<名字>.usd
"""
import os, sys, traceback

REPO = os.environ.get("AUTOBIO_REPO", r"C:/Users/17690/.codex/tmp-repos/AutoBio")
PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(PROJ, "assets", "instruments")
# 读工程内改写过的 XML（不带 MuJoCo assetdir 的版本），不读上游原文件
XML_DIR = os.path.join(PROJ, "upstream", "mjcf")

ALL = ["centrifuge_eppendorf_5430", "centrifuge_eppendorf_5910_ri", "centrifuge_tiangen_tgear_mini",
       "thermal_cycler_biorad_c1000", "thermal_mixer_eppendorf_c", "vortex_mixer_genie_2"]

from isaacsim import SimulationApp
# 说明：从 agent 沙箱里跑会看到 kit/cache "拒绝访问" 与若干 OGN 注册失败 ——
# 那是沙箱不能写 D 盘造成的假象（沙箱只允许写工作区），你在自己终端/GUI 里跑没有这问题。
app = SimulationApp({"headless": True})

import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics
from isaacsim.asset.importer.mjcf import _mjcf


def new_config():
    cfg = _mjcf.ImportConfig()
    # 台面上的仪器应当被固定住（真仪器几十 kg，不会因为被抓盖子就整体飞走）
    for attr, want in (("fixBase", True), ("fix_base", True), ("makeCollisions", True),
                       ("make_collisions", True), ("importInverses", False), ("import_inverses", False)):
        if hasattr(cfg, attr):
            try:
                setattr(cfg, attr, want)
            except Exception:
                pass
    return cfg


def describe_config(cfg):
    names = [x for x in dir(cfg) if not x.startswith("_")]
    out = []
    for n in names:
        try:
            v = getattr(cfg, n)
        except Exception:
            v = "<读不了>"
        if callable(v):
            continue
        out.append((n, v))
    return out


if "--probe" in sys.argv:
    cfg = _mjcf.ImportConfig()
    print("=== ImportConfig 全部开关（名字 = 当前默认值）===")
    for n, v in describe_config(cfg):
        print("  %-34s %s" % (n, v))
    iface = _mjcf.acquire_mjcf_interface()
    print("=== 接口对象可调用 ===")
    print("  ", [x for x in dir(iface) if not x.startswith("_")])
    app.close()
    sys.exit(0)

targets = [a for a in sys.argv[1:] if not a.startswith("--")] or ALL

for name in targets:
    xml = os.path.join(XML_DIR, name + ".xml").replace("\\", "/")
    if not os.path.isfile(xml):
        print("[跳过] 找不到 XML：", xml)
        continue
    dest = os.path.join(OUT, name + ".usd").replace("\\", "/")
    print("=" * 84)
    print("[转换]", name)
    print("   输入", xml)
    try:
        omni.usd.get_context().new_stage()
        cfg = new_config()
        iface = _mjcf.acquire_mjcf_interface()
        got = iface.create_asset_mjcf(xml, "/Instrument", cfg, dest)
        print("   返回", str(got)[:160])
        stage = Usd.Stage.Open(dest)
        if stage is None:
            print("   !! 产物打不开")
            continue
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        n_rb = n_col = n_j = n_bound = 0
        for prim in stage.TraverseAll():
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                n_rb += 1
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                n_col += 1
            t = str(prim.GetTypeName())
            if t.endswith("Joint") and t != "PhysicsJointGroup":
                n_j += 1
                b0 = prim.GetAttribute("physics:body0").Get()
                b1 = prim.GetAttribute("physics:body1").Get()
                lo = prim.GetAttribute("physics:lowerLimit").Get()
                up = prim.GetAttribute("physics:upperLimit").Get()
                ax = prim.GetAttribute("physics:axis").Get()
                k = "drive:angular:physics:stiffness" if t.endswith("RevoluteJoint") else "drive:linear:physics:stiffness"
                st = prim.GetAttribute(k).Get()
                if b0 and b1:
                    n_bound += 1
                print("     关节 %-26s %-22s body0=%-5s body1=%-5s axis=%-4s 限位=[%s, %s] 刚度=%s"
                      % (str(prim.GetPath()).split("/")[-1], t.replace("Physics", ""),
                         bool(b0), bool(b1), ax, lo, up, st))
        stage.GetRootLayer().Save()
        size = os.path.getsize(dest) / 1e6 if os.path.isfile(dest) else 0
        print("   结果：刚体 %d 碰撞 %d 关节 %d（body 已绑定 %d）→ %s (%.1f MB)"
              % (n_rb, n_col, n_j, n_bound, os.path.basename(dest), size))
    except Exception:
        traceback.print_exc()

app.close()
