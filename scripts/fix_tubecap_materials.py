"""
补建 TubeWithCap.usd 里缺失的两个外观材质，并把 material:binding 改到引用域内。

背景：/Root/TubeWithCap/Cap/Cap/visual_135 与 /Root/TubeWithCap/Tube/Tube/visual_101
绑定的是 /World/TubeCap_Looks/CapOrangePlastic 和 /World/TubeCap_Looks/TubeMilkyPlastic，
而这两个材质**既不在本资产里、也不在交付的场景里**（原作者机器上的另一个场景才有），
所以这 10 支 50mL 管的盖子和管身运行时完全没有材质。绑定路径越出引用域本身也会被 USD 忽略。

做法：在资产自己的 Looks 作用域下补两个 UsdPreviewSurface 材质，再把绑定指过去。
颜色是按常规耗材外观给的默认值（原定义已丢失，无法还原），要改直接改这两个材质。
"""
import os, sys, shutil

R = r"D:/IsaacSim5/env/Lib/site-packages/isaacsim"
U = os.path.join(R, "extscache", "omni.usd.libs-1.0.1+69cbf6ad.wx64.r.cp311")
sys.path.insert(0, U)
for d in [os.path.join(U, "bin"), os.path.join(U, "bin", "deps"), os.path.join(R, "kit")]:
    os.add_dll_directory(d)
from pxr import Usd, UsdShade, Gf, Sdf

PROJ = (r"C:/Users/17690/Documents/FeiyangChen/30-Learing_by_Myself/Research/In_myResearch/VLA/"
        r"Building_simulation_environment/CellCultureLab")
ASSET = os.path.join(PROJ, "assets", "precision_tasks", "pipette_tip_200ul", "TubeWithCap.usd")
BAK = r"C:/Users/17690/backup_TubeWithCap_original.usd"

WANT = {
    "CapOrangePlastic":  {"diff": (0.945, 0.451, 0.086), "rough": 0.35, "opacity": 1.0},
    "TubeMilkyPlastic":  {"diff": (0.918, 0.937, 0.949), "rough": 0.22, "opacity": 0.55},
}
BINDINGS = {                       # 网格 prim -> 新材质名
    "/Root/TubeWithCap/Cap/Cap/visual_135": "CapOrangePlastic",
    "/Root/TubeWithCap/Tube/Tube/visual_101": "TubeMilkyPlastic",
}

if not os.path.isfile(BAK):
    shutil.copy2(ASSET, BAK)
    print("原件已备份到：", BAK)
else:
    print("已有备份，跳过：", BAK)

st = Usd.Stage.Open(ASSET)
if st is None:
    print("打不开资产"); sys.exit(1)
looks_path = "/Root/TubeWithCap/Looks"
if not st.GetPrimAtPath(looks_path):
    Usd.Scope.Define(st, looks_path)

made = []
for name, spec in WANT.items():
    mpath = looks_path + "/" + name
    spath = mpath + "/PreviewSurface"
    # 先建 Shader 并写属性
    sh = UsdShade.Shader.Define(st, spath)
    sh.CreateIdAttr()
    sh.GetPrim().CreateAttribute("id", Sdf.ValueTypeNames.Token).Set("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*spec["diff"]))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(spec["rough"])
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    if spec["opacity"] < 1.0:
        sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(spec["opacity"])
    sh.CreateOutput("surface", Sdf.ValueTypeNames.Token)
    # 再建 Material
    UsdShade.Material.Define(st, mpath)
    # 关键：两边都按路径重新取一次，别用创建时留下的句柄（会过期）
    mat = UsdShade.Material.Get(st, mpath)
    sh2 = UsdShade.Shader.Get(st, spath)
    mat.CreateSurfaceOutput().ConnectToSource(sh2.GetOutput("surface"))
    made.append(mpath)
    print("建好材质：", mpath)

for mpath, mat_name in BINDINGS.items():
    prim = st.GetPrimAtPath(mpath)
    if not prim:
        print("!! 找不到网格：", mpath); continue
    rel = prim.GetRelationship("material:binding")
    if not rel:
        rel = prim.CreateRelationship("material:binding", type=Sdf.ValueTypeNames.Obj)
    targets = [Sdf.Path(looks_path + "/" + mat_name)]
    rel.SetTargets(targets)
    print("重绑：%-46s -> %s" % (mpath, targets[0]))

st.GetRootLayer().Save()
print("已保存：", ASSET)
