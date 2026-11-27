"""为什么补齐了碰撞体和质量，运行期还报 Invalid PhysX transform？

报错是"整条 articulation 一起报"：CentriMini 的 body/lid/rotor、VortexGenie2 的
body/knob/switch 各 22 帧全中，而 ThermalCycler 一个都不报 —— 而 B 批之前它同样
零几何零质量，旧日志也不报。所以病根不在"有没有碰撞体/质量"，在这两台的
数值本身。这里把两台坏机 vs 一台好机的每个刚体、每个碰撞网格的实际尺寸
全打出来，找退化（零尺寸、零惯量、质心 NaN）。
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _pxr_env

_pxr_env.bootstrap()
from pxr import Gf, Usd, UsdGeom, UsdPhysics  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE = os.path.join(ROOT, "assets", "scenes", "biolab_main_v09.usda")
STATION_ROOT = "/World/TaskArea/Stations"

STATIONS = ["CentriMini", "VortexGenie2", "ThermalCycler"]


def short(p, sp):
    return str(p.GetPath()).replace(sp + "/", "")


def fmt(v):
    if isinstance(v, (Gf.Vec3d, Gf.Vec3f, Gf.Vec3i)):
        return "[%.6g %.6g %.6g]" % (v[0], v[1], v[2])
    if isinstance(v, Gf.Quatd):
        return "q(%.6g; %.6g %.6g %.6g)" % (v.GetReal(), v.GetImaginary()[0], v.GetImaginary()[1], v.GetImaginary()[2])
    if isinstance(v, Gf.Quath):
        return "qh(%.6g; %.6g %.6g %.6g)" % (v.GetReal(), v.GetImaginary()[0], v.GetImaginary()[1], v.GetImaginary()[2])
    return repr(v)


def dump_attrs(prim, tag):
    for attr in prim.GetAttributes():
        if attr.HasAuthoredValue():
            print("     %s %-46s = %s" % (tag, attr.GetBaseName(), fmt(attr.Get())))


def main():
    stage = Usd.Stage.Open(SCENE)
    xcache = UsdGeom.XformCache(Usd.TimeCode.Default())
    bcache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_], False, True)
    print("锚层 =", os.path.basename(SCENE), " 只读，不改任何东西\n")
    for station in STATIONS:
        sp = "%s/%s" % (STATION_ROOT, station)
        sroot = stage.GetPrimAtPath(sp)
        print("=" * 70)
        print("=== %s ===" % station)
        # 每个刚体吃到几个碰撞体：0 个 = PhysX 建不出 actor，整条 articulation 一起 NaN
        for p in Usd.PrimRange(sroot):
            if "PhysicsRigidBodyAPI" not in p.GetAppliedSchemas():
                continue
            ncol = sum(1 for c in Usd.PrimRange(p)
                       if "PhysicsCollisionAPI" in c.GetAppliedSchemas() and c != p)
            nmesh = sum(1 for c in Usd.PrimRange(p) if c.IsA(UsdGeom.Mesh) and c != p)
            print("[RB-SUMMARY] %-24s colliders=%d meshes=%d"
                  % (short(p, sp), ncol, nmesh))
        for p in Usd.PrimRange(sroot):
            tn = p.GetTypeName()
            if "Joint" in tn:
                tgt = {r.GetName(): [str(t) for t in r.GetTargets()] for r in p.GetRelationships()}
                print("[JOINT] %s  %s" % (short(p, sp), tn))
                print("        bodies: %s" % tgt)
                dump_attrs(p, "  ")
            elif "PhysicsRigidBodyAPI" in p.GetAppliedSchemas():
                m = xcache.GetLocalToWorldTransform(p)
                print("[RB] %s" % short(p, sp))
                print("     world det3=%.6g" % m.GetDeterminant3())
                dump_attrs(p, "  ")
            elif "PhysicsCollisionAPI" in p.GetAppliedSchemas():
                rng = bcache.ComputeWorldBound(p).ComputeAlignedRange()
                if rng.IsEmpty():
                    print("     [COLLIDER] %s : EMPTY BOUND" % short(p, sp))
                else:
                    lo, hi = rng.GetMin(), rng.GetMax()
                    d = [hi[i] - lo[i] for i in range(3)]
                    flag = " <<< 退化" if min(d) < 1e-4 else ""
                    print("     [COLLIDER] %-52s size=[%.4f %.4f %.4f]%s"
                          % (short(p, sp), d[0], d[1], d[2], flag))
        print()


if __name__ == "__main__":
    main()
