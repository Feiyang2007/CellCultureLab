import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _pxr_env
_pxr_env.bootstrap()
from pxr import Usd, UsdGeom

for f in ("platform-visual-0_tmp/platform-visual-0.tmp.usd",
          "platform-inertia_tmp/platform-inertia.tmp.usd"):
    p = os.path.join("upstream", "assets", "instrument", "vortex_mixer_genie_2", f)
    st = Usd.Stage.Open(p)
    print("==", f, "锚层", st.GetRootLayer().identifierPath and os.path.basename(p))
    for prim in st.Traverse():
        if prim.GetTypeName():
            print("   ", prim.GetPath(), prim.GetTypeName())
            if prim.IsA(UsdGeom.Mesh):
                m = UsdGeom.Mesh(prim)
                pts = m.GetPointsAttr().Get()
                print("      points:", len(pts) if pts else 0,
                      "extent:", m.GetExtentAttr().Get() if pts else "-")
