import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _pxr_env  # noqa: E402

_pxr_env.bootstrap()
import Semantics  # noqa: E402
from pxr import Sdf, Usd  # noqa: E402

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
print("== SemanticsAPI 方法 ==")
print([m for m in dir(Semantics.SemanticsAPI) if not m.startswith("_")])

st = Usd.Stage.Open(os.path.join(PROJ, "assets/scenes/biolab_main_v07.usda"))
p = st.GetPrimAtPath("/World/TaskArea/Objects/Tubes/RackPoolLarge/Tube_A01/TubeWithCap/Tube/Tube")
api = Semantics.SemanticsAPI.Apply(p, "semantics")
print("applied:", api)
for m in ("CreateSemanticDataAttr", "GetSemanticData", "GetLabels", "CreateLabelsAttr"):
    print("  has %s:" % m, hasattr(api, m))

d = api.GetSchemaDefaultAttributes() if hasattr(api, "GetSchemaDefaultAttributes") else None
print("schema attrs:", d)

# 直接看 schema 定义文件里叫什么名字
schemapath = os.path.join(_pxr_env.EXTS, "omni.usd.schema.semantics-0.0.0+69cbf6ad.wx64.r.cp311",
                          "plugins", "semantics", "resources", "generatedSchema.usda")
txt = open(schemapath, encoding="utf-8").read()
import re
for mm in re.finditer(r'(def (?:API )?Schema "\w+" \{\n(?:.*\n)*?\})', txt):
    blk = mm.group(1)
    if "Semantic" in blk and len(blk) < 3000:
        print("---- schema block ----")
        print(blk[:1800])
