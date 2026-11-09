"""
用 MuJoCo 自己算权威位姿，导出 JSON 给 USD 转换器用。
必须用装了 mujoco 的那个解释器跑（本机是 Python 3.14）：
    C:/Users/17690/AppData/Local/Programs/Python/Python314/python.exe scripts/mujoco_dump_truth.py

为什么要它：我手推 MuJoCo 的 body 层级矩阵推错过一次（压平后零件炸开），
而 MuJoCo 编译后本来就把每个 body / geom 的世界位姿、关节锚点与轴向都算好了
（xpos / xquat / geom_xpos / geom_xquat / xanchor / xaxis）。
真值由它给，转换器只负责搬运，不再自己算。
"""
import os, sys, json

import mujoco

REPO = os.environ.get("AUTOBIO_REPO", r"C:/Users/17690/.codex/tmp-repos/AutoBio")
XML_DIR = os.path.join(REPO, "autobio", "model", "instrument")
PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(PROJ, "upstream", "truth")
os.makedirs(OUT, exist_ok=True)

NAMES = ["centrifuge_eppendorf_5430", "centrifuge_eppendorf_5910_ri",
         "centrifuge_tiangen_tgear_mini", "thermal_cycler_biorad_c1000",
         "thermal_mixer_eppendorf_c", "vortex_mixer_genie_2"]

def mv(obj, name, default=0.0):
    """按名字取字段，取不到给默认值 —— 不同 mujoco 版本字段名有出入，逐个撞太费时间。"""
    try:
        v = getattr(obj, name)
    except AttributeError:
        return default
    try:
        return float(v)
    except Exception:
        try:
            return [float(x) for x in v]
        except Exception:
            return default


TYPE_NAME = {0: "plane", 1: "hfield", 2: "sphere", 3: "capsule", 4: "ellipsoid", 5: "cylinder", 6: "box", 7: "mesh"}
JT_NAME = {0: "free", 1: "ball", 2: "slide", 3: "hinge", 4: "equaliteq"}


def dump(name):
    xml = os.path.join(XML_DIR, name + ".xml")
    m = mujoco.MjModel.from_xml_path(xml)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)

    body_of_geom = {}
    geoms = []
    for i in range(m.ngeom):
        bi = int(m.geom_bodyid[i])
        g = {
            "index": i,
            "name": mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i) or "geom_%02d" % i,
            "body": mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, bi) or "body_%d" % bi,
            "type": TYPE_NAME.get(int(m.geom_type[i]), "box"),
            "size": [float(x) for x in m.geom_size[i]],
            "world_pos": [float(x) for x in d.geom_xpos[i]],
            # 关键：MuJoCo 的 geom_pos / geom_quat 是**相对所属 body** 的局部量，
            # USD 里几何正是挂在对应刚体下面，所以照搬即可，不需要我自己合成矩阵。
            "local_pos": ([float(x) for x in m.geom_pos[i]] if hasattr(m,'geom_pos') else [0.0,0.0,0.0]),
            "local_quat": ([float(x) for x in m.geom_quat[i]] if hasattr(m,'geom_quat') else [1.0,0.0,0.0,0.0]),   # MuJoCo 给的世界位置（校验用）
            # MuJoCo 3.16 的 MjData 没有 geom_xquat；几何的世界朝向 = body 世界位姿 ∘
            # XML 里 geom 自己的 pos/quat，所以这里不需要它：body 世界位姿由 MuJoCo 权威给出，
            # 相对偏移从 XML 读，全程不做手推矩阵。
            "contype": int(m.geom_contype[i]),
            "conaffinity": int(m.geom_conaffinity[i]),
            "group": int(m.geom_group[i]),
        }
        if int(m.geom_type[i]) == 7:      # mesh
            mid = int(m.geom_dataid[i])
            g["mesh"] = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_MESH, mid)
            g["mesh_scale"] = [float(x) for x in m.mesh_scale[mid]]
            g["mesh_pos"] = [float(x) for x in m.mesh_pos[mid]]
            g["mesh_quat"] = [float(x) for x in m.mesh_quat[mid]]
            mf = m.mesh_face[mid * 3:(mid + 1) * 3] if False else None
            # 顶点表：mesh 的顶点在 mesh 局部坐标系，需再乘 mesh_pos/quat 与 scale
            base = int(m.mesh_vertadr[mid]); n = int(m.mesh_vertnum[mid])
            verts = m.mesh_vert[base:base + n]
            g["verts"] = [[float(v[0]), float(v[1]), float(v[2])] for v in verts]
            fbase = int(m.mesh_faceadr[mid]); fn = int(m.mesh_facenum[mid])
            faces = m.mesh_face[fbase:fbase + fn]
            g["faces"] = [[int(f[0]), int(f[1]), int(f[2])] for f in faces]
        geoms.append(g)

    bodies = []
    for i in range(m.nbody):
        bodies.append({
            "index": i,
            "name": mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i) or "body_%d" % i,
            "pos": [float(x) for x in d.xpos[i]],
            "quat": [float(x) for x in d.xquat[i]],
            "parent": int(m.body_parentid[i]),
            "mass": float(m.body_mass[i]),
        })

    joints = []
    for i in range(m.njnt):
        bi = int(m.jnt_bodyid[i])
        joints.append({
            "index": i,
            "name": mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i) or "joint_%d" % i,
            "type": JT_NAME.get(int(m.jnt_type[i]), "hinge"),
            "body": mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, bi) or "body_%d" % bi,
            "body_index": bi,
            "parent": mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY,
                                        int(m.body_parentid[bi])) or "",
            "anchor": [float(x) for x in d.xanchor[i]],          # 世界锚点
            "axis": [float(x) for x in d.xaxis[i]],              # 世界轴向
            "range": [float(x) for x in m.jnt_range[i]],
            "stiffness": mv(m, "jnt_stiffness", [0.0]*m.njnt)[i] if hasattr(m,"jnt_stiffness") else 0.0,
            "damping": mv(m, "jnt_damping", [0.0]*m.njnt)[i] if hasattr(m,"jnt_damping") else 0.0,
            "frictionloss": mv(m, "jnt_frictionloss", [0.0]*m.njnt)[i] if hasattr(m,"jnt_frictionloss") else 0.0,
        })
    return {"name": name, "bodies": bodies, "geoms": geoms, "joints": joints,
            "ngeom": m.ngeom, "nbody": m.nbody, "njnt": m.njnt}


def main():
    print("=" * 88)
    print("MuJoCo", mujoco.__version__, "  源：", XML_DIR)
    print("输出：", OUT)
    want = [a for a in sys.argv[1:] if not a.startswith("--")] or NAMES
    for nm in want:
        if not os.path.isfile(os.path.join(XML_DIR, nm + ".xml")):
            print("  ✘ 无此 XML:", nm)
            continue
        try:
            data = dump(nm)
        except Exception as e:
            print("  ✘ %-34s %s: %s" % (nm, type(e).__name__, str(e)[:110]))
            continue
        fp = os.path.join(OUT, nm + ".json")
        with open(fp, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False)
        print("  ✔ %-34s body %-3d geom %-4d joint %-3d  -> %s (%.1f MB)"
              % (nm, data["nbody"], data["ngeom"], data["njnt"],
                 nm + ".json", os.path.getsize(fp) / 1e6))


if __name__ == "__main__":
    main()
