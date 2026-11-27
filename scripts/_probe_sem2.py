import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _pxr_env  # noqa: E402

_pxr_env.bootstrap()
import Semantics  # noqa: E402
from pxr import Sdf, Usd, UsdGeom  # noqa: E402

print("schema class:", Semantics.SemanticsAPI, getattr(Semantics.SemanticsAPI, "GetSchemaKind", None))
try:
    print("GetSchemaName:", Semantics.SemanticsAPI.GetSchemaName())
except Exception as e:
    print("GetSchemaName fail:", e)

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
st = Usd.Stage.Open(os.path.join(PROJ, "assets/scenes/biolab_main_v07.usda"))
p = st.GetPrimAtPath("/World/TaskArea/Objects/Tubes/RackPoolLarge/Tube_A01/TubeWithCap/Tube/Tube")
print("prim:", p, p.IsValid())
api = Semantics.SemanticsAPI.Apply(p, "semantics")
attr = api.CreateSemanticLabelsAttr()
attr.Set(["tube_50ml", "manipulable"])
print("labels 写后读回:", list(api.GetSemanticLabels()))
print("attr 名:", attr.GetName(), "类型:", attr.GetTypeName(), "是否 schema 派生:", attr.IsSchemaBased())
for a in p.GetAuthoredAttributes():
    print("  authored:", a.GetName(), a.Get())
