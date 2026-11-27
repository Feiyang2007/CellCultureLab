import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _pxr_env
_pxr_env.bootstrap()
from pxr import UsdGeom, Gf
print("BBoxCache:", [x for x in dir(UsdGeom.BBoxCache) if not x.startswith("_")])
print("BBox3d:", [x for x in dir(Gf.BBox3d) if not x.startswith("_")])
