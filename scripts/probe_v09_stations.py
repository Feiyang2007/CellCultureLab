"""A 类新工位在 Isaac 里到底动得起来吗 —— 两阶段实测。

阶段 1（静力学）：播timeline、安定若干帧，量 7 个新刚体的 PhysX 位姿与
    USD 作者位的偏差。这一步查的是"有没有掉穿台面/被弹飞"，
    −19.5205 m 那种事故就是这里抓的。
阶段 2（动力学）：逐条关节试**三条驱动通道**，读回都只认 PhysX 刚体现势。
    A 运行期写 USD targetPosition、B ArticulationView.set_dof_position_targets、
    C set_dof_actuation_forces。为什么三条一起试：v07 的 reachable 判据跑出来
    离心盖期望 -98.72°、实测 -0.25°，跟这批新关节在 A 通道上的表现一模一样，
    所以"drive 不动"到底是资产没写对还是通道不对，必须分开证。
    只读 PhysX 刚体位姿、按关节轴算带符号角，跟 eval_task_goal.py 同一套口径，
    别在这边另发明一种"看起来会动"的判法。

为什么不能只看 USD 层就下结论：v08 那批 ART 关节的教训 —— USD 里写了 drive
    不等于 PhysX 里有解算（部件没碰撞体→没质量→drive 拖着个 0 kg 的物体）。
    这批资产虽然一开始就带了碰撞体和显式 mass，仍然要实跑一遍才算数。

用法：
    D:/isaacsim-5.1.0/python.bat scripts/probe_v09_stations.py
    D:/isaacsim-5.1.0/python.bat scripts/probe_v09_stations.py --settle 90
"""
import argparse
import json
import math
import os
import sys

from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--headless", default="on", choices=["on", "off"])
parser.add_argument("--scene", default=None)
parser.add_argument("--settle", type=int, default=60)
parser.add_argument("--drive-frames", type=int, default=90)
parser.add_argument("--agg-cap", default="4096", help="GPU 接触对容量，0=不改")
parser.add_argument("--cpu-physics", default="0", choices=["0", "1"])
parser.add_argument("--out", default=None)
ARGS = parser.parse_args()

app = SimulationApp({"headless": ARGS.headless == "on", "width": 320, "height": 240})
import numpy as np  # noqa: E402  (SimulationApp 之后再导，别抢它的 numpy)

import omni.physx  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Usd, UsdGeom  # noqa: E402

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE = ARGS.scene or os.path.join(PROJ, "assets", "scenes", "biolab_main_v09.usda")
ROOT = "/World/TaskArea/Stations/AClass"


def quat_to_mat(q):
    """q = (w,x,y,z)，返回按列存的方向余弦，rot(v) = M·v。"""
    w, x, y, z = q
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def rot(M, v):
    return [sum(M[i][j] * v[j] for j in range(3)) for i in range(3)]


AXIS = {"X": (1.0, 0.0, 0.0), "Y": (0.0, 1.0, 0.0), "Z": (0.0, 0.0, 1.0)}


def ref_vec(axis):
    """挑一个跟轴垂直的单位向量当"参考指针"，用来量绕轴的带符号角。"""
    return (0.0, 0.0, 1.0) if axis == "X" else (1.0, 0.0, 0.0)


def signed_angle(q0, q1, axis):
    a = AXIS[axis]
    M0, M1 = quat_to_mat(q0), quat_to_mat(q1)
    v0, v1 = rot(M0, ref_vec(axis)), rot(M1, ref_vec(axis))
    c = [v0[1] * v1[2] - v0[2] * v1[1], v0[2] * v1[0] - v0[0] * v1[2],
         v0[0] * v1[1] - v0[1] * v1[0]]
    dot = sum(v0[i] * v1[i] for i in range(3))
    return math.degrees(math.atan2(sum(c[i] * a[i] for i in range(3)), dot))


def as_quat(rotation):
    """get_rigidbody_transformation 的 rotation 是 carb.Float4，顺序是 x/y/z/w。"""
    if rotation is None:
        return None
    try:
        if isinstance(rotation, (tuple, list)) and len(rotation) == 4:
            x, y, z, w = [float(v) for v in rotation]
            return (w, x, y, z)
    except Exception:
        pass
    for nm in ("w", "x", "y", "z"):
        if not hasattr(rotation, nm):
            return None
    return (float(rotation.w), float(rotation.x), float(rotation.y), float(rotation.z))


def pos_of(ret):
    p = ret.get("position") if ret else None
    if p is None:
        return None
    return [float(p[i]) for i in range(3)]


def rb(px, path):
    """问 PhysX 要刚体现势。路径没进仿真图时这个接口会抛，统一吞成 None。"""
    try:
        return px.get_rigidbody_transformation(path)
    except Exception:
        return None


def joint_from_usd(prim):
    """关节的轴向/限位/body1 一律现从 USD 属性读，别信 customData。

    ccl:joint 是我自己写进 A 类资产的备忘，属于"作者意图"；physics:lowerLimit
    才是 PhysX 真正拿去钳位的那份。两边不一致时（比如 compose 时改了限位忘了改
    customData）测的必须是 PhysX 那份，否则测的是我以为的世界。
    """
    t = str(prim.GetTypeName())
    kind = "revolute" if t.endswith("RevoluteJoint") else "prismatic"

    def num(name, default=None):
        a = prim.GetAttribute(name)
        v = a.Get() if a else None
        return default if v is None else float(v)

    ax = prim.GetAttribute("physics:axis").Get() if prim.GetAttribute("physics:axis") else None
    b1 = prim.GetRelationship("physics:body1").GetTargets() if \
        prim.GetRelationship("physics:body1") else []
    b0 = prim.GetRelationship("physics:body0").GetTargets() if \
        prim.GetRelationship("physics:body0") else []
    api = "angular" if kind == "revolute" else "linear"
    return dict(path=str(prim.GetPath()), kind=kind, axis=str(ax) if ax else "X",
                lo=num("physics:lowerLimit", 0.0), hi=num("physics:upperLimit", 0.0),
                body=str(b1[0]) if b1 else None,
                body0=str(b0[0]) if b0 else None,
                stiffness=num("drive:%s:physics:stiffness" % api, 0.0),
                max_force=num("drive:%s:physics:maxForce" % api, 0.0))


# 对照组：这条关节**不是** articulation（Centrifuge 没有 ArticulationRootAPI），
# 用来分开"通道 B 对普通关节不适用"和"drive 本来就点不着"这两件事。
CONTROL_JOINTS = ["/World/TaskArea/Equipment/Centrifuge/Joints/LidJoint"]


def main():
    ctx = omni.usd.get_context()
    if not ctx.open_stage(SCENE):
        raise SystemExit("打不开 %s" % SCENE)
    stage = ctx.get_stage()
    px = omni.physx.get_physx_interface()
    tl = omni.timeline.get_timeline_interface()
    # 有没有第四条通道可用（PhysX 侧直接改关节驱动/唤醒刚体）：先把接口名单打出来，
    # 免得下次又靠猜 API 名字。
    print("px 接口里与 drive/joint/wake 相关的方法：%s"
          % [m for m in dir(px)
             if any(k in m.lower() for k in ("drive", "joint", "wake"))])

    # 接触容量：v09 加了 5 台工位后 PhysX 又报 foundLostAggregatePairsCapacity
    # 要 2337（auto_tube_pick 当年为这个专门开了 --agg-cap）。漏接触会让"关得回"
    # 这种判据变得不可复现，先把容量抬够再测驱动。
    if ARGS.agg_cap != "0" or ARGS.cpu_physics == "1":
        from pxr import Sdf
        try:      # 这套 standalone python 里 PhysxSchema 挂在 pxr 命名空间下
            from pxr import PhysxSchema
        except Exception:
            try:
                import PhysxSchema
            except Exception:
                PhysxSchema = None

        def set_scene_attr(prim, name, value, vtype):
            if PhysxSchema is not None:
                api = PhysxSchema.PhysxSceneAPI.Apply(prim)
                meth = getattr(api, "Create" + name[0].upper() + name[1:] + "Attr", None)
                if meth is not None:
                    meth(value)
                    return prim.GetAttribute("physxScene:" + name).Get()
            prim.AddAppliedSchema("PhysxSceneAPI")
            a = prim.GetAttribute("physxScene:" + name)
            if not a:
                a = prim.CreateAttribute("physxScene:" + name, vtype)
            a.Set(value)
            return a.Get()

        # PhysX 接口上没有 get_physics_scenes，物理场景 prim 只能自己从 stage 里挑
        for sp in [q.GetPath() for q in stage.Traverse()
                   if q.GetTypeName() == "PhysicsScene"]:
            p = stage.GetPrimAtPath(sp)
            if ARGS.cpu_physics == "1":
                set_scene_attr(p, "enableGPUDynamics", False, Sdf.ValueTypeNames.Bool)
                set_scene_attr(p, "broadphaseType", "MBP", Sdf.ValueTypeNames.Token)
            if ARGS.agg_cap != "0":
                cap = int(ARGS.agg_cap)
                got = {nm: set_scene_attr(p, nm, cap, Sdf.ValueTypeNames.UInt)
                       for nm in ("gpuTotalAggregatePairsCapacity",
                                  "gpuFoundLostAggregatePairsCapacity")}
                print("物理场景 %s：GPU=%s 容量回写=%s" % (sp, ARGS.cpu_physics != "1", got))

    # 作者位：编辑期 XformCache 是可信的（判据那条坑读的是运行期回写）
    xform = UsdGeom.XformCache(Usd.TimeCode.Default())
    bodies, joints = [], []
    for prim in stage.Traverse():
        p = str(prim.GetPath())
        if not p.startswith(ROOT) and p not in CONTROL_JOINTS:
            continue
        t = str(prim.GetTypeName())
        if t in ("PhysicsRevoluteJoint", "PhysicsPrismaticJoint"):
            joints.append(joint_from_usd(prim))
        elif "PhysicsRigidBodyAPI" in prim.GetAppliedSchemas():
            # 编辑期取作者位用 GetLocalToWorldTransform（这套 pxr 里没有
            # ...MatrixOmni 那个 Omni 扩展方法，判据脚本早就踩过一次）。
            m = xform.GetLocalToWorldTransform(prim)
            tr = m.ExtractTranslation()
            qr = m.ExtractRotation().GetQuat()
            bodies.append(dict(path=p, pos=[tr[i] for i in range(3)],
                               quat=(qr.GetReal(),) + tuple(qr.GetImaginary())))

    print("场景 %s：新刚体 %d 个，被测关节 %d 条（含对照 %d 条）"
          % (os.path.basename(SCENE), len(bodies), len(joints), len(CONTROL_JOINTS)))
    for j in joints:
        print("   %-46s %-9s 轴 %s 限位 [%g, %g] drive k=%g maxF=%g body0=%s"
              % (j["path"].replace("/World/TaskArea/Stations/", "")
                            .replace("/World/TaskArea/Equipment/", ""),
                 j["kind"], j["axis"], j["lo"], j["hi"], j["stiffness"],
                 j["max_force"], "世界锚定" if not j["body0"] else j["body0"].rsplit("/", 1)[-1]))
    tl.play()
    for _ in range(ARGS.settle):
        app.update()

    rows, worst = [], 0.0
    for b in bodies:
        ret = rb(px, b["path"])
        p = pos_of(ret)
        if p is None:
            rows.append((b["path"], None, None, "PhysX 里没有这个刚体"))
            continue
        d = [p[i] - b["pos"][i] for i in range(3)]
        dist = math.sqrt(sum(v * v for v in d))
        worst = max(worst, dist)
        rows.append((b["path"], p, dist,
                     "OK" if dist < 0.02 else ("漂 %.3f m" % dist)))
    print("\n阶段 1 安定 %.1f s（%d 帧）后，新刚体相对作者位的偏移：" %
          (ARGS.settle / 60.0, ARGS.settle))
    for path, p, dist, verdict in rows:
        short = path.replace(ROOT + "/", "")
        print("   %-46s %-9s %s" % (short, ("%.3f m" % dist) if dist is not None else "-",
                                    verdict))
    print("   最大偏移 %.4f m" % worst)

    print("\n阶段 2 逐条关节：三条通道各试一次，看哪条真能推动 PhysX")
    print("   A) 运行期写 USD drive:*:physics:targetPosition（eval_task_goal._oracle_drive 用的就是这条）")
    print("   B) ArticulationView.set_dof_position_targets（PhysX 张量接口，每帧喂）")
    print("   C) set_dof_actuation_forces（直接给力矩，绕过 drive 参数）")
    from omni.physics.tensors import create_simulation_view
    sim_view = create_simulation_view("numpy")
    views = {}
    for j in joints:
        root = j["path"].rsplit("/Joints/", 1)[0]
        if root in views:
            continue
        v = sim_view.create_articulation_view(root)
        try:
            # 没有 ArticulationRootAPI 时这个接口不会返回 None，而是发一个
            # _backend=None 的空壳，第一次取属性才炸 —— 这里当场验一遍。
            _ = v.dof_count
        except Exception:
            v = None
        if v is None:
            print("   !! %s 不是 articulation（没有 PhysicsArticulationRootAPI？）" % root)
        views[root] = v
    for j in joints:
        j["view"] = views.get(j["path"].rsplit("/Joints/", 1)[0])
        j["dof_idx"] = None
        if j["view"] is not None:
            nm = j["path"].rsplit("/", 1)[1]
            names = list(j["view"].shared_metatype.dof_names)
            j["dof_idx"] = names.index(nm) if nm in names else None
            if j["dof_idx"] is None:
                print("   !! 关节 %s 不在 dof 列表 %s 里" % (nm, names))

    out = []
    for j in joints:
        rec = dict(j)
        ret = rb(px, j["body"])
        base_pos = pos_of(ret)
        base_q = as_quat(ret.get("rotation") if ret else None)
        prim = stage.GetPrimAtPath(j["path"])
        api = "angular" if j["kind"] == "revolute" else "linear"
        tgt = prim.GetAttribute("drive:%s:physics:targetPosition" % api)
        # 开位取"离 0 更远的那端限位"，符号跟着限位写：A 类里抽屉和滑盖是负向开
        open_v = float(j["hi"] if abs(j["hi"] or 0) >= abs(j["lo"] or 0) else j["lo"])
        tol = 8.0 if j["kind"] == "revolute" else 0.02
        ndof = j["view"].dof_count if j["view"] else 0
        idx = np.array([0], dtype=np.uint32)

        def measure():
            ret = rb(px, j["body"])
            p = pos_of(ret)
            q = as_quat(ret.get("rotation") if ret else None)
            if j["kind"] == "revolute":
                got = signed_angle(base_q, q, j["axis"]) if (q and base_q) else float("nan")
            else:
                got = sum((p[i] - base_pos[i]) * AXIS[j["axis"]][i] for i in range(3)) \
                    if (p and base_pos) else float("nan")
            qdof = None
            if j["view"] is not None and j["dof_idx"] is not None:
                qdof = float(np.asarray(j["view"].get_dof_positions(), float)[0][j["dof_idx"]])
            return got, qdof

        def run(frames, hook=None):
            for _ in range(frames):
                if hook:
                    hook()
                app.update()
            return measure()

        # A) 只写 USD
        tgt.Set(open_v)
        a_body, a_dof = run(ARGS.drive_frames)
        # B) 回零 USD，改用张量接口喂目标
        tgt.Set(0.0)
        run(15)
        if j["dof_idx"] is not None:
            tv = np.zeros((1, ndof), dtype=np.float32)
            tv[0, j["dof_idx"]] = math.radians(open_v) if j["kind"] == "revolute" else open_v
            b_body, b_dof = run(ARGS.drive_frames,
                                lambda: j["view"].set_dof_position_targets(tv, idx))
        else:
            b_body, b_dof = float("nan"), None
        # C) 直接给力矩/力
        tgt.Set(0.0)
        run(15)
        if j["dof_idx"] is not None:
            f = np.zeros((1, ndof), dtype=np.float32)
            f[0, j["dof_idx"]] = math.copysign(1.0, open_v) * (1.0 if j["kind"] == "revolute" else 8.0)
            c_body, c_dof = run(ARGS.drive_frames,
                                lambda: j["view"].set_dof_actuation_forces(f, idx))
        else:
            c_body, c_dof = float("nan"), None
        run(15)

        rec["chan"] = dict(usd=round(a_body, 3), dof=round(b_body, 3), force=round(c_body, 3),
                           dof_read=(None if a_dof is None else round(a_dof, 3)),
                           dof_read_b=(None if b_dof is None else round(b_dof, 3)),
                           tol=tol, want=round(open_v, 3))
        # 三通道里挑"朝开方向走得最远"的那条。不能用 abs()：抽屉/滑盖的开位是**负**的，
        # 往 +0.05 的抖动用 abs 看也算"动了"，而它其实是关紧方向；B/C 在没有 ART 根时
        # 读回来是 nan，先滤掉（nan 混进 max 会让结果整体变 nan）。
        sgn = 1.0 if open_v >= 0 else -1.0
        cand = [("A", a_body), ("B", b_body), ("C", c_body)]
        cand = [(nm, v) for nm, v in cand if v == v]
        chan_name, best = max(cand, key=lambda t: t[1] * sgn) if cand else ("-", float("nan"))
        rec["open"] = dict(want=round(open_v, 3), measured=round(best, 3),
                           via=chan_name, pass_=best * sgn > tol)
        # 关得回：所有通道归零后再量一次（上面 run(15) 已归零，这里补一段静置）
        z_body, _ = run(60)
        rec["close"] = dict(measured=round(z_body, 3), err=round(abs(z_body), 3),
                            pass_=abs(z_body) < tol)
        print("   %-38s A(USD) %+8.2f  B(目标) %+8.2f  C(力矩) %+8.2f  期望 %+.2f  "
              "归零 %+7.3f %s | dof 读数 A/B %s/%s"
              % (j["path"].replace(ROOT + "/Joints/", ""), a_body, b_body, c_body, open_v,
                 z_body, "✓" if rec["close"]["pass_"] else "✗",
                 "-" if a_dof is None else "%.3f" % a_dof,
                 "-" if b_dof is None else "%.3f" % b_dof))
        out.append(rec)

    def n(k, sub):
        return sum(1 for r in out if r[k][sub])

    def via(c):
        return sum(1 for r in out if r["open"]["pass_"] and r["open"]["via"] == c)
    # 达标一律看"朝开方向走完行程"，不是 |位移|>tol（负向开的件见上面注释）。
    # B/C 需要 PhysicsArticulationRootAPI，A 类这批**故意不加**根 API
    # （实测加了反而让每条关节失去 dof），所以这两列预期就是 nan，不是回归。
    print("\n结论：%d/%d 条能动 —— 其中 A（运行期写 USD targetPosition，"
          "eval_task_goal._oracle_drive 走的就是这条）%d 条、B %d 条、C %d 条；"
          "%d/%d 条能关回；新刚体最大偏移 %.4f m"
          % (n("open", "pass_"), len(joints), via("A"), via("B"), via("C"),
             n("close", "pass_"), len(joints), worst))
    if ARGS.out:
        os.makedirs(os.path.dirname(os.path.abspath(ARGS.out)), exist_ok=True)
        with open(ARGS.out, "w", encoding="utf-8") as f:
            json.dump({"worst_drift_m": worst,
                       "joints": [{k: v for k, v in r.items() if k != "view"} for r in out]},
                      f, ensure_ascii=False, indent=1)
        print("→", ARGS.out)
    tl.stop()
    app.close()
    return 0


if __name__ == "__main__":
    main()
