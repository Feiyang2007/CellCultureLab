"""
CellCultureLab 场景体检脚本（离线，不需要 Isaac GUI）

用法：
    python scripts/verify_scene.py [要检查的scene路径]
    python scripts/verify_scene.py --child        # 内部用：真正打开 stage 并打印统计

检查三件事：
  A. 引用完整性 —— 场景文本里每一条 @资产@ 是否能在磁盘上解析到
  B. 加载噪声   —— 真开一次 stage，统计 "Could not open asset" 的次数与来源
  C. 可交互程度 —— 刚体/碰撞体/关节计数，按顶层分组，外加尺寸是否合常理（米）
"""
import os, re, sys, subprocess, collections

# ---------- 让裸 pxr 能在 Isaac Sim 的 pip 环境里跑起来 ----------
ISAAC_ROOT = os.environ.get("ISAAC_PY_ROOT", r"D:/IsaacSim5/env/Lib/site-packages/isaacsim")
_USDLIBS = os.path.join(ISAAC_ROOT, "extscache", "omni.usd.libs-1.0.1+69cbf6ad.wx64.r.cp311")
if os.path.isdir(_USDLIBS):
    sys.path.insert(0, _USDLIBS)
    for d in [os.path.join(_USDLIBS, "bin"), os.path.join(_USDLIBS, "bin", "deps"),
              os.path.join(ISAAC_ROOT, "kit")]:
        if os.path.isdir(d):
            os.add_dll_directory(d)

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_SCENE = os.path.join(PROJ, "assets", "scenes", "biolab_main_v02.usda")

# 远程/包内协议：交给 Isaac 解析，本地不算失败
REMOTE = ("http://", "https://", "package://", "isaacsim://", "omni://", "archive://")


def rel_to_layer(layer_dir, ap):
    if ap.startswith(REMOTE):
        return None
    return os.path.normpath(os.path.join(layer_dir, ap)) if not os.path.isabs(ap) else ap


# ============================================================ 子进程：真开 stage
def child(scene):
    from pxr import Usd, UsdGeom, UsdPhysics, Gf
    stage = Usd.Stage.Open(scene)
    if stage is None:
        print("CHILD: 打不开", scene)
        return 1
    prims = list(stage.TraverseAll())
    print("CHILD prims=%d" % len(prims))

    def has(prim, api):
        try:
            return prim.HasAPI(api)
        except Exception:
            return False

    rb = [p for p in prims if has(p, UsdPhysics.RigidBodyAPI)]
    col = [p for p in prims if has(p, UsdPhysics.CollisionAPI)]
    art = [p for p in prims if has(p, UsdPhysics.ArticulationRootAPI)]
    # 注意：USD 里关节类型名带 Physics 前缀（PhysicsRevoluteJoint / PhysicsPrismaticJoint ...）
    joints = [p for p in prims if str(p.GetTypeName()).endswith("Joint")
              and str(p.GetTypeName()) != "PhysicsJointGroup"]
    meshes = [p for p in prims if str(p.GetTypeName()) == "Mesh"]
    print("CHILD rigidbody=%d collision=%d articulation=%d joints=%d meshes=%d"
          % (len(rb), len(col), len(art), len(joints), len(meshes)))

    # 按 /World/xxx 分组统计刚体
    grp = collections.Counter()
    for p in rb:
        parts = str(p.GetPath()).split("/")
        grp["/".join(parts[:4]) if len(parts) > 3 else str(p.GetPath())] += 1
    for k, v in sorted(grp.items()):
        print("CHILD RBGROUP %-60s %d" % (k, v))

    jg = collections.Counter()
    for p in joints:
        jg[str(p.GetTypeName())] += 1
    for k, v in sorted(jg.items()):
        print("CHILD JOINTKIND %-20s %d" % (k, v))
    for p in joints:
        # physics:body0 / body1 是【关系(relationship)】，不是属性！
        # 用 GetAttribute 读永远返回 None —— 曾因此把绑得好好的关节误判成"假关节"。
        r0 = p.GetRelationship("physics:body0")
        r1 = p.GetRelationship("physics:body1")
        body0 = r0.GetTargets()[0] if (r0 and r0.GetTargets()) else None
        body1 = r1.GetTargets()[0] if (r1 and r1.GetTargets()) else None
        # 限位写在 physics:lowerLimit / upperLimit（drive 里那个是马达目标，不叫限位）
        lim = p.GetAttribute("physics:lowerLimit").Get()
        up = p.GetAttribute("physics:upperLimit").Get()
        ax = p.GetAttribute("physics:axis").Get()
        print("CHILD JOINT %-62s %-22s body0=%s body1=%s axis=%s 限位=[%s, %s]"
              % (str(p.GetPath()), str(p.GetTypeName()), body0, body1, ax, lim, up))

    # ---- D. 逐个可交互对象的体检 ----
    from pxr import Tf
    def audit(path):
        prim = stage.GetPrimAtPath(path)
        if not prim:
            print("CHILD AUDIT %-58s <absent>" % path)
            return
        kinds = collections.Counter()
        nchild = 0
        for c in Usd.PrimRange(prim):
            nchild += 1
            t = str(c.GetTypeName())
            if t.endswith("Joint"):
                kinds["JOINT:" + t] += 1
            if has(c, UsdPhysics.RigidBodyAPI):
                kinds["RIGIDBODY"] += 1
            if has(c, UsdPhysics.CollisionAPI):
                kinds["COLLIDER"] += 1
            if str(c.GetTypeName()) == "Mesh":
                kinds["mesh"] += 1
        mass = None
        for c in Usd.PrimRange(prim):
            a = c.GetAttribute("physics:mass")
            if a and a.HasAuthoredValue():
                mass = a.Get()
                break
        rb_here = "RB" if has(prim, UsdPhysics.RigidBodyAPI) else "--"
        print("CHILD AUDIT %-58s prims=%-5d %s mass=%s  %s"
              % (path, nchild, rb_here, ("%.4g kg" % mass) if mass else "?",
                 " ".join("%s x%d" % (k, v) for k, v in sorted(kinds.items()))))

    print("CHILD --- 可交互对象体检 ---")
    for path in ["/World/TaskArea/Equipment/Centrifuge",
                 "/World/TaskArea/Equipment/Pipette",
                 "/World/TaskArea/Fixtures/TipRack200uL",
                 "/World/TaskArea/Fixtures/PipetteRack",
                 "/World/TaskArea/Fixtures/TubeRack",
                 "/World/TaskArea/Fixtures/TubeRack_B",
                 "/World/TaskArea/Fixtures/CentrifugeTubeRack",
                 "/World/TaskArea/Objects/Tips/RackPool/Tip_A01",
                 "/World/TaskArea/Objects/Tubes/RackPoolLarge/Tube_A01",
                 "/World/TaskArea/Objects/CentrifugeTubes/RackPool/Tube_R00_C01",
                 "/World/Robot/Robot1Mount/ur5e",
                 "/World/Robot/Gripper1Temp/Robotiq_2F_85_edit"]:
        audit(path)

    # 96 个枪头里有多少真的可抓取（既有刚体又有碰撞体）
    # 只数"物体本体"，不数引用资产内部的子 prim
    is_obj = lambda p, pat: bool(re.fullmatch(r"/World/TaskArea/Objects/[^/]+/[^/]+/" + pat,
                                              str(p.GetPath())))
    tips = [p for p in prims if is_obj(p, r"Tip_[A-H]\d\d")]
    tip_grab = [p for p in tips if has(p, UsdPhysics.RigidBodyAPI)
                and any(has(c, UsdPhysics.CollisionAPI) for c in Usd.PrimRange(p))]
    print("CHILD TIPSTATS 枪头总数=%d 可抓取(刚体+碰撞)=%d" % (len(tips), len(tip_grab)))
    tubes = [p for p in prims if is_obj(p, r"Tube_\w+")]
    tube_grab = [p for p in tubes if has(p, UsdPhysics.RigidBodyAPI)]
    print("CHILD TUBESTATS  管总数=%d 有刚体=%d" % (len(tubes), len(tube_grab)))

    # 世界包围盒（米）：判断尺度是否正常
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_, UsdGeom.Tokens.render], False)
    for path in ["/World/Environment", "/World/TaskArea", "/World/Robot",
                 "/World/TaskArea/Equipment/Centrifuge",
                 "/World/TaskArea/Equipment/Pipette",
                 "/World/TaskArea/Fixtures/TipRack200uL",
                 "/World/TaskArea/Objects/Tips/RackPool/Tip_A01",
                 "/World/TaskArea/Objects/CentrifugeTubes/RackPool/Tube_R00_C01",
                 "/World/Robot/Robot1Mount/ur5e",
                 "/World/Environment/Lab_021/TableTopCollision_Physics",
                 "/World/Environment/Lab_021/mesh_004"]:
        prim = stage.GetPrimAtPath(path)
        if not prim:
            print("CHILD BBOX %-52s <absent>" % path)
            continue
        rng = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        if rng.IsEmpty():
            print("CHILD BBOX %-52s <empty>" % path)
            continue
        mn, mx = rng.GetMin(), rng.GetMax()
        sz = tuple(mx[i] - mn[i] for i in range(3))
        print("CHILD BBOX %-52s size=%.3f x %.3f x %.3f m   z=[%.3f, %.3f]"
              % (path, sz[0], sz[1], sz[2], mn[2], mx[2]))
    return 0


# ============================================================ 父进程：汇总报告
def main():
    args = [a for a in sys.argv[1:]]
    if args and args[0] == "--child":
        sys.exit(child(args[1] if len(args) > 1 else DEFAULT_SCENE))

    scene = args[0] if args else DEFAULT_SCENE
    scene_dir = os.path.dirname(scene)
    print("=" * 92)
    print("体检目标：", scene, "(%.2f MB)" % (os.path.getsize(scene) / 1e6))
    print("=" * 92)

    # ---- A. 引用完整性（直接从 USDA 文本抓 @...@）----
    txt = open(scene, encoding="utf-8", errors="replace").read()
    refs = collections.Counter(re.findall(r"@([^@\n\"]{4,300})@", txt))
    bad, remote, good = [], 0, 0
    for ap, n in refs.most_common():
        if ap.startswith(REMOTE):
            remote += 1
            print("  [远程 x%-3d] %s" % (n, ap))
            continue
        p = rel_to_layer(scene_dir, ap)
        if p and os.path.isfile(p):
            good += 1
        else:
            bad.append((n, ap))
    print("\nA. 引用完整性：本地可解析 %d 条，远程 %d 条，**解析失败 %d 条**" % (good, remote, len(bad)))
    for n, ap in bad:
        print("     [失败 x%d] %s" % (n, ap))

    # ---- B. 真开一次 stage，抓 stderr 里的加载错误 ----
    r = subprocess.run([sys.executable, os.path.abspath(__file__), "--child", scene],
                       capture_output=True, text=True, errors="replace")
    fail = collections.Counter(re.findall(r"Could not open asset @([^@]+)@", r.stderr or ""))
    print("\nB. 加载噪声：'Could not open asset' 共 %d 次，%d 个不同资产"
          % (sum(fail.values()), len(fail)))
    for ap, n in fail.most_common():
        kind = "远程(在Isaac里能加载)" if ap.startswith(REMOTE) or ap.startswith("https:/") else "本地缺失"
        print("     x%-4d [%s] %s" % (n, kind, ap))

    # ---- C. 物理与尺度 ----
    print("\nC. 可交互程度与尺度：")
    for line in (r.stdout or "").splitlines():
        if line.startswith("CHILD prims") or line.startswith("CHILD rigidbody"):
            print("   ", line.replace("CHILD ", ""))
        elif line.startswith("CHILD RBGROUP"):
            print("      刚体", line.replace("CHILD RBGROUP ", "").strip())
        elif line.startswith("CHILD JOINTKIND"):
            print("      关节类型", line.replace("CHILD JOINTKIND ", "").strip())
        elif line.startswith("CHILD TIPSTATS") or line.startswith("CHILD TUBESTATS"):
            print("      ", line.replace("CHILD ", "").strip())
        elif line.startswith("CHILD JOINT "):
            print("      ", line.replace("CHILD JOINT ", "关节 ").strip())
        elif line.startswith("CHILD AUDIT"):
            print("      ", line.replace("CHILD AUDIT ", "").strip())
        elif line.startswith("CHILD BBOX"):
            print("      尺寸", line.replace("CHILD BBOX ", "").strip())


if __name__ == "__main__":
    main()
