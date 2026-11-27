import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _pxr_env
_pxr_env.bootstrap()
from pxr import Usd

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
stage = Usd.Stage.Open(os.path.join(PROJ, "assets", "scenes", "biolab_main_v09.usda"))
st = stage.GetPrimAtPath("/World/TaskArea/Stations/AClass/Co2Incubator")
for nm in ("PrimIsLoaded", "PrimIsActive", "PrimIsAbstract", "PrimHasModelApiSchema",
           "PrimDefaultPredicate", "PrimAllPrimsPredicate"):
    pred = getattr(Usd, nm, None)
    if pred is None:
        print("%-24s 不存在" % nm)
        continue
    try:
        print("%-24s -> %d" % (nm, len(list(Usd.PrimRange(st, pred)))))
    except Exception as e:
        print("%-24s 报错 %s" % (nm, e))
sh = stage.GetPrimAtPath("/World/TaskArea/Stations/AClass/Co2Incubator/shell_bottom")
print("shell_bottom: active", sh.IsActive(), "loaded", sh.IsLoaded(),
      "abstract", sh.IsAbstract(), "typeName", sh.GetTypeName(),
      "api", sh.GetAppliedSchemas())
door = stage.GetPrimAtPath("/World/TaskArea/Stations/AClass/Co2Incubator/door")
print("door: active", door.IsActive(), "loaded", door.IsLoaded(),
      "abstract", door.IsAbstract(), "typeName", door.GetTypeName())
print("root meta kind:", st.GetMetadata("kind"), "owner:", st.GetOwnerLayerPaths())
