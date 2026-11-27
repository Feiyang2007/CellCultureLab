"""列出三台仪器 _base.usd 根级 /visuals、/collisions 下到底有哪些分组、每组什么类型。

上一版 fix_instrument_geometry.py 只按部件名找 `/collisions/<part>`，
VortexGenie2 的 platform 就是这么被判成"源里没有几何"的 —— 但 MJCF 源里
platform 有 2 个 cylinder + 一圈 tube-well sphere，是**图元**不是 mesh。
MJCF 转换器把图元碰撞体命名成 `_geom_N`（ThermalCycler 的 /collisions/body 里就有），
所以要按"每个分组属于哪个部件"重新对一遍。
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _pxr_env

_pxr_env.bootstrap()
from pxr import Usd, UsdGeom  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
INSTR = os.path.join(ROOT, "assets", "instruments", "configuration")
STEMS = ["centrifuge_tiangen_tgear_mini", "thermal_cycler_biorad_c1000",
         "vortex_mixer_genie_2"]


def main():
    for stem in STEMS:
        layer = os.path.join(INSTR, stem + "_base.usd")
        stage = Usd.Stage.Open(layer)
        print("=" * 72)
        print("=== %s ===" % stem)
        for group in ("visuals", "collisions"):
            root = stage.GetPrimAtPath("/" + group)
            if not root:
                print("  /%s : 不存在" % group)
                continue
            for child in root.GetChildren():
                names = []
                kinds = {}
                for d in Usd.PrimRange(child):
                    tn = d.GetTypeName()
                    if tn in ("Mesh", "Cylinder", "Sphere", "Cube", "Capsule", "Cone"):
                        kinds[tn] = kinds.get(tn, 0) + 1
                        names.append(d.GetName())
                print("  /%s/%-16s %s" % (group, child.GetName(),
                                          ", ".join("%s×%d" % kv for kv in sorted(kinds.items())) or "空"))
                if names:
                    print("      ", ", ".join(sorted(set(names))[:14]) +
                          (" …" if len(set(names)) > 14 else ""))
        print()


if __name__ == "__main__":
    main()
