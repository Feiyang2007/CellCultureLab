"""rootJoint_body 的"disjointed body transforms"到底差多远？

启动日志里三台仪器都报了
    PhysicsUSD: CreateJoint - found a joint with disjointed body transforms,
    the simulation will most likely snap objects together: .../joints/rootJoint_body
但只有 CentriMini 和 VortexGenie2 在跑到 ~73 s 时整条 articulation 被弹到
1e23 m（RTX 的 extents 警告先给出坐标，PhysX 随后报 Invalid PhysX transform）。
这里把每个关节的两端世界坐标差算出来，看 snap 幅度到底多少，
以及为什么 ThermalCycler 扛住了。
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _pxr_env

_pxr_env.bootstrap()
from pxr import Gf, Usd, UsdGeom  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE = os.path.join(ROOT, "assets", "scenes", "biolab_main_v09.usda")


def world(stage, path):
    prim = stage.GetPrimAtPath(path)
    if not prim:
        return None
    m = UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(prim)
    r = m[3]
    return Gf.Vec3d(r[0], r[1], r[2])


def main():
    stage = Usd.Stage.Open(SCENE)
    print("%-58s %10s %10s %10s" % ("joint", "Δpos(m)", "Δrot(deg)", "body0"))
    for st in ("CentriMini", "ThermalCycler", "VortexGenie2"):
        jp = "/World/TaskArea/Stations/%s/joints" % st
        for child in stage.GetPrimAtPath(jp).GetChildren():
            b0 = [str(t) for t in child.GetRelationship("physics:body0").GetTargets()]
            b1 = [str(t) for t in child.GetRelationship("physics:body1").GetTargets()]
            if not b1:
                print("%-58s  body1 缺失!" % child.GetPath())
                continue
            jw = world(stage, str(child.GetPath()))
            b1w = world(stage, b1[0])
            dp = (jw - b1w).GetLength() if jw and b1w else float("nan")
            # localPos0/localPos1 若没写，PhysX 认为两端框架都在各自刚体原点
            lp0 = child.GetAttribute("physics:localPos0").Get()
            lp1 = child.GetAttribute("physics:localPos1").Get()
            b0w = world(stage, b0[0]) if b0 else jw
            gap = float("nan")
            if b0w and b1w:
                gap = (b0w - b1w).GetLength()
            print("%-58s joint↔body1=%.4f  body0↔body1=%.4f  body0=%s localPos0=%s localPos1=%s"
                  % (str(child.GetPath()).split("/Stations/")[1], dp, gap,
                     "世界(空)" if not b0 else b0[0].split("/")[-1],
                     "未写" if lp0 is None else "[%g %g %g]" % (lp0[0], lp0[1], lp0[2]),
                     "未写" if lp1 is None else "[%g %g %g]" % (lp1[0], lp1[1], lp1[2])))


if __name__ == "__main__":
    main()
