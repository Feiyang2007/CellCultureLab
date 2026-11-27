import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _pxr_env
_pxr_env.bootstrap()
from pxr import Usd, UsdGeom

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
st = Usd.Stage.Open(os.path.join(PROJ, "assets", "scenes", "biolab_main_v07.usda"))
P = "/World/TaskArea/Equipment/Centrifuge"
for p in [P + "/Body", P + "/Lid", P + "/Rotor"]:
    pr = st.GetPrimAtPath(p)
    if not pr:
        print(p, "不存在"); continue
    a = pr.GetAttribute("physics:kinematicEnabled")
    print("%-46s api=%s kin=%s mass=%s" % (p, [s for s in pr.GetAppliedSchemas() if "Physics" in s],
                                           a.Get() if a else None,
                                           pr.GetAttribute("physics:mass").Get()))
root = st.GetPrimAtPath(P)
print("Centrifuge 根：api=%s" % [s for s in root.GetAppliedSchemas() if "Articulation" in s])
for p in ["/World/TaskArea/Stations/CentriMini", "/World/TaskArea/Stations/CentriMini/body/body"]:
    pr = st.GetPrimAtPath(p)
    if pr:
        print("%-52s api=%s" % (p, [s for s in pr.GetAppliedSchemas() if "Physics" in s]))
