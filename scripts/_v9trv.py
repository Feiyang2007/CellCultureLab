import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _pxr_env
_pxr_env.bootstrap()
from pxr import Usd

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# 1) 单体资产自己能不能遍历
a = os.path.join(PROJ, "assets", "props", "celllab", "Co2Incubator.usda")
sa = Usd.Stage.Open(a)
print("资产自身 traverse:", len(list(sa.Traverse())),
      "root kids:", len(list(sa.GetPrimAtPath("/Co2Incubator").GetChildren())))

# 2) v09 里用 AllPrims 谓词再试一次
stage = Usd.Stage.Open(os.path.join(PROJ, "assets", "scenes", "biolab_main_v09.usda"))
p = "/World/TaskArea/Stations/AClass/Co2Incubator"
st = stage.GetPrimAtPath(p)
print("default PrimRange:", len(list(Usd.PrimRange(st))))
print("allprims PrimRange:", len(list(Usd.PrimRange(st, Usd.PrimAllPrimsPredicate))))
print("getChild:", st.GetChild("shell_bottom").IsValid())
ac = stage.GetPrimAtPath("/World/TaskArea/Stations/AClass")
print("AClass default kids:", len(list(ac.GetChildren())),
      "allprims range:", len(list(Usd.PrimRange(ac, Usd.PrimAllPrimsPredicate))))
