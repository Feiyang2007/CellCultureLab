import os, sys, tempfile
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _pxr_env
_pxr_env.bootstrap()
from pxr import Usd

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REF = "../props/celllab/Co2Incubator.usda"
P = "/World/TaskArea/Stations/AClass/Co2Incubator"
META = {
    "a 纯 reference": 'references = @%s@</Co2Incubator>' % REF,
    "b +apiSchemas": 'prepend apiSchemas = ["PhysicsArticulationRootAPI"]\n                    references = @%s@</Co2Incubator>' % REF,
    "c +customData": 'customData = { dictionary "x" = { int i = 1 } }\n                    references = @%s@</Co2Incubator>' % REF,
    "d 无显式 </Path>": 'references = @%s@' % REF,
    "e +extent": 'references = @%s@</Co2Incubator>' % REF,
}
BODY = ("                    double3 xformOp:translate = (-1.72, 0.02, 0.8717)\n"
        "                    uniform token[] xformOpOrder = [\"xformOp:translate\"]\n")

HEAD = ('#usda 1.0\n(\n    metersPerUnit = 1\n    upAxis = "Z"\n)\n\n'
        'def Xform "World"\n{\n    def Xform "TaskArea"\n    {\n        def Xform "Stations"\n        {\n'
        '            def Xform "AClass"\n            {\n')
TAIL = '            }\n        }\n    }\n}\n'

path = os.path.join(PROJ, "assets", "scenes", "_bisect_tmp.usda")
for nm, meta in META.items():
    spec = ('                def Xform "Co2Incubator" (\n                    %s\n'
            '                )\n                {\n%s                }\n' % (meta, BODY))
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(HEAD + spec + TAIL)
    try:
        stage = Usd.Stage.Open(path)
    except Exception as e:
        print("%-20s 解析失败：%s" % (nm, str(e).strip().replace("\n", " ")))
        continue
    hits = sum(1 for p in stage.Traverse() if "Co2Incubator" in str(p.GetPath()))
    st = stage.GetPrimAtPath(P)
    print("%-20s traverse 命中 %3d  GetPrimRange %3d  直接子件 %d  type=%s"
          % (nm, hits, len(list(Usd.PrimRange(st))), len(list(st.GetChildren())),
             st.GetTypeName()))
os.unlink(path)
