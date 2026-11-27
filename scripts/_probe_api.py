import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _pxr_env  # noqa: E402

_pxr_env.bootstrap()
from pxr import PhysxSchema, UsdPhysics  # noqa: E402

for n in ("CollisionGroupMembershipsAPI", "CollisionAPI", "MassAPI",
          "MaterialAPI", "ArticulationRootAPI"):
    print("UsdPhysics.%-30s %s" % (n, hasattr(UsdPhysics, n)))
for n in ("PhysxCollisionAPI", "PhysxRigidBodyAPI", "PhysxArticulationAPI",
          "PhysxSelfCollisionAPI", "PhysxTriangleMeshCollisionAPI",
          "PhysxCollisionData", "PhysxConvexMeshCollisionAPI"):
    print("PhysxSchema.%-26s %s" % (n, hasattr(PhysxSchema, n)))
