import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _pxr_env
_pxr_env.bootstrap()
from pxr import Usd, UsdGeom

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
stage = Usd.Stage.Open(os.path.join(ROOT, "assets", "scenes", "biolab_main_v09.usda"))
xc = UsdGeom.XformCache(Usd.TimeCode.Default())
for st in ("CentriMini", "ThermalCycler", "VortexGenie2"):
    for tail in ("", "/body", "/body/body", "/body/lid", "/body/rotor", "/body/knob", "/body/switch", "/body/platform"):
        p = stage.GetPrimAtPath("/World/TaskArea/Stations/%s%s" % (st, tail))
        if not p:
            continue
        m = xc.GetLocalToWorldTransform(p)
        r = m[3]
        t = (r[0], r[1], r[2])
        print("%-40s world pos = [%.4f %.4f %.4f]" % (st + tail, t[0], t[1], t[2]))
