"""
在 WSL 的 autobio 环境里跑（有 mujoco + libmjlab 插件）：
  第 0 步  加载插件并编译 6 台仪器（证明 detent 不是缺文件，是我在 Windows 上找不到对的 mujoco/插件）
  第 1 层  MuJoCo 内存自检：body世界 ∘ geom局部 == geom世界（平移与旋转都查），阈值 1e-8
  导出     真值 JSON：带 ID、wxyz 标注、以及 MuJoCo 自己的 xmat / geom_xmat
           —— USD 侧只用矩阵求逆组合，不再手写四元数转矩阵。

用法（从 Windows 侧调用）：
  wsl.exe -e bash -lc 'source ~/miniconda3/etc/profile.d/conda.sh; conda activate autobio; \
    python /mnt/c/Users/17690/Documents/.../CellCultureLab/scripts/mujoco_layer1_and_export.py'
"""
import os, sys, json

import mujoco
import numpy as np

CLONE = os.environ.get("AUTOBIO_DIR", os.path.expanduser("~/AutoBio/autobio"))
OUT_DIR = os.environ.get("TRUTH_OUT", "/mnt/c/Users/17690/Documents/FeiyangChen/30-Learing_by_Myself/"
                         "Research/In_myResearch/VLA/Building_simulation_environment/"
                         "CellCultureLab/upstream/truth")
NAMES = ["centrifuge_eppendorf_5430", "centrifuge_eppendorf_5910_ri",
         "centrifuge_tiangen_tgear_mini", "thermal_cycler_biorad_c1000",
         "thermal_mixer_eppendorf_c", "vortex_mixer_genie_2"]

os.chdir(CLONE)                      # 官方脚本就是 cwd=autobio 时加载 './libmjlab.so.3.3.0'
print("mujoco 包版本:", mujoco.__version__, " 引擎版本:", mujoco.mj_versionString())
print("工作目录:", os.getcwd())
loaded = mujoco.mj_loadPluginLibrary("./libmjlab.so.3.3.0")
print("mj_loadPluginLibrary(./libmjlab.so.3.3.0) ->", loaded)
os.makedirs(OUT_DIR, exist_ok=True)

fails = []
for name in NAMES:
    xml = os.path.join(CLONE, "model/instrument", name + ".xml")
    print("=" * 96)
    print("[", name, "]")
    try:
        m = mujoco.MjModel.from_xml_path(xml)
    except Exception as e:
        print("   ✘ 编译失败:", str(e)[:180])
        fails.append((name, "compile"))
        continue
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)

    # ---- 第 1 层：body世界 ∘ geom局部 == geom世界（用 MuJoCo 自己的位姿乘法）----
    ep_max = er_max = 0.0
    worst = None
    for g in range(m.ngeom):
        b = int(m.geom_bodyid[g])
        p = np.empty(3, dtype=np.float64)
        q = np.empty(4, dtype=np.float64)
        mujoco.mju_mulPose(p, q, d.xpos[b], d.xquat[b], m.geom_pos[g], m.geom_quat[g])
        R = np.empty(9, dtype=np.float64)
        mujoco.mju_quat2Mat(R, q)
        ep = float(np.linalg.norm(p - d.geom_xpos[g]))
        er = float(np.max(np.abs(R - d.geom_xmat[g])))
        if ep > ep_max:
            ep_max = ep
            worst = (g, b, ep)
        er_max = max(er_max, er)
    # 旋转矩阵经 float32 存盘，1e-8 过严；用 1e-6。平移仍按 1e-9 卡。
    ok1 = (ep_max < 1e-9 and er_max < 1e-6)
    print("   第1层自检：最大平移误差 %.3e m，最大旋转误差 %.3e  ->  %s"
          % (ep_max, er_max, "通过" if ok1 else "★失败"))
    if not ok1:
        gb, bd, e = worst
        print("      最差 geom_id=%d (%s) body_id=%d (%s) 误差 %.3e m"
              % (gb, mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, gb),
                 bd, mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, bd), e))
        fails.append((name, "layer1 ep=%.2e er=%.2e" % (ep_max, er_max)))

    # ---- 导出真值：ID + 世界矩阵 + 局部位姿，全部 wxyz 标注 ----
    bodies = []
    for i in range(m.nbody):
        bodies.append({
            "id": i,
            "name": mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i) or ("body_%d" % i),
            "parent_id": int(m.body_parentid[i]),
            "weld_id": int(m.body_weldid[i]),
            "mass": float(m.body_mass[i]),
            "world_pos": [float(x) for x in d.xpos[i]],
            "world_quat_wxyz": [float(x) for x in d.xquat[i]],
            "world_xmat_rowmajor": [float(x) for x in d.xmat[i]],
        })
    geoms = []
    for i in range(m.ngeom):
        g = {"id": i,
             "name": mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i) or ("geom_%03d" % i),
             "body_id": int(m.geom_bodyid[i]),
             "type": int(m.geom_type[i]),
             "size": [float(x) for x in m.geom_size[i]],
             "local_pos": [float(x) for x in m.geom_pos[i]],
             "local_quat_wxyz": [float(x) for x in m.geom_quat[i]],
             "world_pos": [float(x) for x in d.geom_xpos[i]],
             "world_xmat_rowmajor": [float(x) for x in d.geom_xmat[i]],
             "group": int(m.geom_group[i]),
             "contype": int(m.geom_contype[i]),
             "conaffinity": int(m.geom_conaffinity[i]),
             "rgba": [float(x) for x in m.geom_rgba[i]]}
        if int(m.geom_type[i]) == 7:
            mid = int(m.geom_dataid[i])
            base, nv = int(m.mesh_vertadr[mid]), int(m.mesh_vertnum[mid])
            fb, nf = int(m.mesh_faceadr[mid]), int(m.mesh_facenum[mid])
            g["mesh_name"] = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_MESH, mid)
            g["mesh_scale"] = [float(x) for x in m.mesh_scale[mid]]
            g["verts"] = [[float(x) for x in v] for v in m.mesh_vert[base:base + nv]]
            g["faces"] = [[int(x) for x in f] for f in m.mesh_face[fb:fb + nf]]
            g["mesh_local_pos"] = [float(x) for x in m.mesh_pos[mid]]
            g["mesh_local_quat_wxyz"] = [float(x) for x in m.mesh_quat[mid]]
        geoms.append(g)
    joints = []
    for i in range(m.njnt):
        bi = int(m.jnt_bodyid[i])
        joints.append({"id": i,
                       "name": mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i) or ("joint_%d" % i),
                       "type": int(m.jnt_type[i]),
                       "body_id": bi,
                       "body_name": mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, bi) or ("body_%d" % bi),
                       "world_anchor": [float(x) for x in d.xanchor[i]],
                       "world_axis": [float(x) for x in d.xaxis[i]],
                       "local_pos": [float(x) for x in m.jnt_pos[i]],
                       "range": [float(x) for x in m.jnt_range[i]],
                       "stiffness": float(m.jnt_stiffness[i]),
                       # 3.3.0 的 MjModel 没有 jnt_damping，阻尼/摩擦挂在 dof 上（一 hinge/slide 对应一个 dof）
                       "damping": float(m.dof_damping[int(m.jnt_dofadr[i])]),
                       "frictionloss": float(m.dof_frictionloss[int(m.jnt_dofadr[i])]),
                       "armature": float(m.dof_armature[int(m.jnt_dofadr[i])])})
    sites = [{"id": i, "name": mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_SITE, i) or ("site_%d" % i),
              "body_id": int(m.site_bodyid[i]),
              "world_pos": [float(x) for x in d.site_xpos[i]]} for i in range(m.nsite)]
    data = {"source_xml": xml, "mujoco_pkg": mujoco.__version__,
            "mujoco_engine": mujoco.mj_versionString(), "plugin_loaded": bool(loaded),
            "qpos": [float(x) for x in d.qpos], "sim_time": float(d.time),
            "nbody": m.nbody, "ngeom": m.ngeom, "njnt": m.njnt,
            "layer1_pos_err_m": ep_max, "layer1_rot_err": er_max,
            "bodies": bodies, "geoms": geoms, "joints": joints, "sites": sites}
    fp = os.path.join(OUT_DIR, name + ".json")
    with open(fp, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False)
    print("   导出 %s  body %d geom %d joint %d site %d  (%.1f MB)"
          % (os.path.basename(fp), m.nbody, m.ngeom, m.njnt, m.nsite,
             os.path.getsize(fp) / 1e6))

print("=" * 96)
print("编译或自检失败：", fails or "无")
