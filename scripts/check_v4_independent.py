"""
对 v4 产物做**独立**核对：不比我自己造的矩阵，直接比 JSON 里的数值向量。

1) 位置：USD 组合出的世界平移 vs MuJoCo 给的 geom world_pos（纯向量，不涉及我的矩阵约定）
2) 朝向：USD 组合出的 3x3 vs MuJoCo xmat，分别按"行优先"和"列优先"两种解释算最大元素差，
   谁小说明 MuJoCo 的 xmat 该按哪种解释读——这直接判我 v4 里 mat_from 的约定对不对。
3) 顺带：顶层底座是否真的运动学钉住、关节两端是否都指向刚体。
"""
import os, sys, json, math, glob

R = r"D:/IsaacSim5/env/Lib/site-packages/isaacsim"
U = os.path.join(R, "extscache", "omni.usd.libs-1.0.1+69cbf6ad.wx64.r.cp311")
sys.path.insert(0, U)
for d in [os.path.join(U, "bin"), os.path.join(U, "bin", "deps"), os.path.join(R, "kit")]:
    os.add_dll_directory(d)
from pxr import Usd, UsdGeom, UsdPhysics, Gf

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def usd_mat_rows(M):
    m = Gf.Matrix4d(M)
    return [[m[i][j] for j in range(3)] for i in range(3)]


def maxdiff(a, b):
    return max(abs(x - y) for ra, rb in zip(a, b) for x, y in zip(ra, rb))


def run(name):
    TJ = os.path.join(PROJ, "upstream", "truth", name + ".json")
    US = os.path.join(PROJ, "assets", "instruments4", name + ".usda")
    if not (os.path.isfile(TJ) and os.path.isfile(US)):
        return None
    T = json.load(open(TJ, encoding="utf-8"))
    st = Usd.Stage.Open(US)
    xfc = UsdGeom.XformCache(Usd.TimeCode.Default())

    pos_err = 0.0
    rot_row = 0.0      # 按 xmat 行优先解释
    rot_col = 0.0      # 按 xmat 列优先解释
    n = 0
    for g in T["geoms"]:
        bid = g["body_id"]
        bn = next((b["name"] for b in T["bodies"] if b["id"] == bid), None)
        if bn is None:
            continue
        # 找到这个 geom 在 USD 里的 prim（按 id 后缀命名）
        want = "_%04d" % g["id"]
        hit = None
        for p in Usd.PrimRange(st.GetPrimAtPath("/Instrument/Bodies/" + __import__("re").sub(r"[^0-9A-Za-z_]", "_", bn))):
            if p.GetName().endswith(want):
                hit = p
                break
        if hit is None:
            continue
        M = xfc.GetLocalToWorldTransform(hit)
        got = M.ExtractTranslation()
        e = math.dist((got[0], got[1], got[2]), g["world_pos"])
        pos_err = max(pos_err, e)
        rows = usd_mat_rows(M)
        xm = g["world_xmat_rowmajor"]
        roww = [[xm[i * 3 + j] for j in range(3)] for i in range(3)]
        colw = [[xm[j * 3 + i] for j in range(3)] for i in range(3)]
        rot_row = max(rot_row, maxdiff(rows, roww))
        rot_col = max(rot_col, maxdiff(rows, colw))
        n += 1


    kin = sum(1 for p in st.TraverseAll()
              if p.HasAPI(UsdPhysics.RigidBodyAPI)
              and (p.GetAttribute("physics:kinematicEnabled").Get() is True))
    rb = sum(1 for p in st.TraverseAll() if p.HasAPI(UsdPhysics.RigidBodyAPI))
    nested = 0
    for p in st.TraverseAll():
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            a = p.GetParent()
            while a:
                if a.HasAPI(UsdPhysics.RigidBodyAPI):
                    nested += 1
                    break
                a = a.GetParent()
    bad_j = 0
    all_j = 0
    for p in st.TraverseAll():
        if str(p.GetTypeName()) in ("RevoluteJoint", "PrismaticJoint"):
            all_j += 1
            r0 = p.GetRelationship("physics:body0").GetTargets()
            r1 = p.GetRelationship("physics:body1").GetTargets()
            if not (r0 and r1 and st.GetPrimAtPath(r0[0]).HasAPI(UsdPhysics.RigidBodyAPI)
                    and st.GetPrimAtPath(r1[0]).HasAPI(UsdPhysics.RigidBodyAPI)):
                bad_j += 1
    return {"n": n, "pos_mm": pos_err * 1000.0, "rot_row": rot_row, "rot_col": rot_col,
            "rb": rb, "kin": kin, "nested": nested, "joints": all_j, "joints_bad": bad_j}


names = [os.path.splitext(os.path.basename(f))[0] for f in glob.glob(
    os.path.join(PROJ, "assets", "instruments4", "*.usda"))]
print("=" * 100)
print("%-32s %6s %9s %9s %9s  %5s %4s %5s %6s %9s" %
      ("仪器", "核对数", "位置mm", "旋(行优先)", "旋(列优先)", "刚体", "钉", "嵌套", "关节", "关节异常"))
for nm in sorted(names):
    r = run(nm)
    if not r:
        continue
    verdict = "行优先对" if r["rot_row"] < r["rot_col"] else "列优先对"
    print("%-32s %6d %9.4f %9.2e %9.2e  %5d %4d %5d %6d %9d   ← %s"
          % (nm[:32], r["n"], r["pos_mm"], r["rot_row"], r["rot_col"],
             r["rb"], r["kin"], r["nested"], r["joints"], r["joints_bad"], verdict))
