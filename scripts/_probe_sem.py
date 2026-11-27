import glob
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _pxr_env  # noqa: E402

_pxr_env.bootstrap()
from pxr import Usd  # noqa: E402

try:
    import Semantics
    print("top-level Semantics OK:", Semantics.__file__)
    print("  attrs:", [a for a in dir(Semantics) if not a.startswith("_")][:12])
except Exception as e:
    print("top-level fail:", type(e).__name__, e)

try:
    from pxr import Semantics as S2
    print("pxr.Semantics OK:", [a for a in dir(S2) if not a.startswith("_")][:12])
except Exception as e:
    print("pxr.Semantics fail:", type(e).__name__, e)

print("schema resource dirs:")
for p in glob.glob("D:/isaacsim-5.1.0/extscache/*semantics*"):
    print("  ", p)
for p in glob.glob("D:/isaacsim-5.1.0/extscache/*semantics*/**/*.usda", recursive=True):
    print("     schema:", p)

reg = Usd.SchemaRegistry()
print("IsAPISchema(SemanticsSemanticsAPI):", reg.IsAPISchema("SemanticsSemanticsAPI"))
print("IsAPISchema(SemanticsAPI):", reg.IsAPISchema("SemanticsAPI"))
for name in ("SemanticsAPI", "SemanticsSemanticsAPI", "SemanticSchemaAPI", "SemanticsLabelsAPI"):
    try:
        print(name, "->", reg.IsAPISchema(name), reg.IsSchema(name))
    except Exception as e:
        print(name, "err", e)
