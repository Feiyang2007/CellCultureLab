"""ART 站点稳定性试车台：一次只改一个变量，看整站 PhysX 位姿会不会飞。

背景（2026-11-24 台式机 isaacsim 6.1 / PhysX 110.3）：
    惯量修好之后 v09 每帧刷 "Invalid PhysX transform"（CentriMini/ThermalCycler/
    VortexGenie2 全部 12 个刚体），rtx.scenedb 报世界坐标 1e13~1e19。
    把 MuJoCo 导入器写歪的底座焊点 rootJoint_body 的 localPos1/localRot1 补齐
    （frame0/frame1 离线自检 0.00000 m）之后，PhysX 照样报 disjointed。
    现在剩下的两个嫌疑：
      A 焊点的 physics:body0 是空的 —— 场景里所有健康的关节 body0 都指得出 prim，
        而 PhysX 若把空 body0 当"世界原点"而不是"关节 prim 自己的系"，
        frame0 就落在原点、frame1 在站点，差 2.3 m，正好是要 snap。
      B 每个站点有两个 ArticulationRoot：body/body 和零链接、无刚体的 worldBody。
    光靠 USD 口算判不出来，就做对照跑。

--variant：
    base     什么都不改（必须复现飞掉，否则这套判读没意义）
    weld     body0 显式指向 /World，localPos0/localRot0 写成焊点自己的世界位姿
    single   worldBody 的 physxArticulation:articulationEnabled 关掉，只留一个根
    off      rootJoint_body jointEnabled=False（底座不焊，靠台面撑着）

判读：读 get_rigidbody_transformation 与作者位姿比，超过 5 cm 算动过、1 m 算飞。
按墙上时间取样（headless 不渲染时一帧多快没有准头，秒数才是准头）。
日志里 "Invalid PhysX transform" 的条数用 grep 数，不挂回调。
"""
import argparse
import os
import sys
import time

from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--variant", default="base", choices=["base", "weld", "single", "off"])
parser.add_argument("--seconds", type=float, default=20.0)
ARGS = parser.parse_args()

app = SimulationApp({"headless": True, "renderer": "None", "width": 320, "height": 240})

import omni.physx  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom  # noqa: E402

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE = os.path.join(PROJ, "assets", "scenes", "biolab_main_v09.usda")
STATIONS = ["CentriMini", "ThermalCycler", "VortexGenie2"]
WATCH = {
    "CentriMini": ["body", "lid", "rotor"],
    "ThermalCycler": ["body", "lid", "reaction_block"],
    "VortexGenie2": ["body", "platform", "knob"],
}
FLY_M = 1.0


def pose_of(iface, path):
    r = iface.get_rigidbody_transformation(path)
    if not isinstance(r, dict) or r.get("ret_val") is False:
        return None
    return tuple(float(x) for x in r["position"])


def main():
    ctx = omni.usd.get_context()
    if not ctx.open_stage(SCENE):
        print("打不开场景", SCENE)
        app.close()
        return 2
    stage = ctx.get_stage()
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())

    def wmat(p):
        return cache.GetLocalToWorldTransform(stage.GetPrimAtPath(p))

    # 作者位姿：物理插手之前先存下来，之后一切偏移都以它为基准。
    authored, joints = {}, {}
    for s in STATIONS:
        rootp = "/World/TaskArea/Stations/" + s
        for part in WATCH[s]:
            p = "%s/body/%s" % (rootp, part)
            authored[p] = Gf.Vec3d(wmat(p).ExtractTranslation())
        jq = "%s/joints/rootJoint_body" % rootp
        joints[s] = (stage.GetPrimAtPath(jq), wmat(jq))

    if ARGS.variant != "base":
        for s in STATIONS:
            jp, jw = joints[s]
            if ARGS.variant == "weld":
                q = jw.ExtractRotation().GetQuat()
                # body0 这层关系在 crumb 里就声明了（rel physics:body0 = []），
                # 直接 SetTargets 即可；这版 pxr 没有 Usd.RelTypes，别走 Create。
                rel = jp.GetRelationship("physics:body0")
                rel.SetTargets([Sdf.Path("/World")])
                jp.GetAttribute("physics:localPos0").Set(
                    Gf.Vec3f(*jw.ExtractTranslation()))
                jp.GetAttribute("physics:localRot0").Set(
                    Gf.Quatf(q.GetReal(), *q.GetImaginary()))
            elif ARGS.variant == "off":
                attr = jp.GetAttribute("physics:jointEnabled") or jp.CreateAttribute(
                    "physics:jointEnabled", Sdf.ValueTypeNames.Bool)
                attr.Set(False)
            elif ARGS.variant == "single":
                wb = stage.GetPrimAtPath("/World/TaskArea/Stations/%s/worldBody" % s)
                wb.GetAttribute("physxArticulation:articulationEnabled").Set(False)
            print("改过 %s：%s -> body0=%s lp0=%s lr0=%s" % (
                s, ARGS.variant,
                [str(x) for x in (jp.GetRelationship("physics:body0").GetTargets()
                                  if jp.GetRelationship("physics:body0") else [])],
                jp.GetAttribute("physics:localPos0").Get(),
                jp.GetAttribute("physics:localRot0").Get()))

    iface = omni.physx.get_physx_interface()
    tl = omni.timeline.get_timeline_interface()
    tl.stop()
    tl.play()
    def drift(s, part):
        p = "/World/TaskArea/Stations/%s/body/%s" % (s, part)
        q = pose_of(iface, p)
        if q is None:
            return None
        d = (Gf.Vec3d(*q) - authored[p]).GetLength()
        worst_all[p] = max(worst_all.get(p, 0.0), d)
        return d

    t0 = time.time()
    nxt, frames, worst_all = 1.0, 0, {}
    while time.time() - t0 < ARGS.seconds:
        app.update()
        frames += 1
        el = time.time() - t0
        if el < nxt:
            continue
        items = []
        for s in STATIONS:
            one = []
            for part in WATCH[s]:
                d = drift(s, part)
                one.append("%s=%s" % (part, "读不到" if d is None else "%.3f m 离位" % d))
            items.append("%s: %s" % (s, " ".join(one)))
        print("[%.0fs 帧%d] %s" % (el, frames, "  ".join(items)))
        sys.stdout.flush()
        nxt += 2.0
    tl.stop()
    print("\n汇总 variant=%s 帧数=%d" % (ARGS.variant, frames))
    for p in sorted(worst_all):
        print("  %-58s 最大离位 %12.3f m %s" % (
            p.replace("/World/TaskArea/Stations/", ""), worst_all[p],
            "飞了" if worst_all[p] > FLY_M else ("动了" if worst_all[p] > 0.05 else "稳")))
    app.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
