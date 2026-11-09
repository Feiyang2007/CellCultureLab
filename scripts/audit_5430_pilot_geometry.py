"""Independent exported USD pose/scale audit against compiled source geometry."""
import json,math
from pathlib import Path
from validate_gpu_capacity import bare_usd
from runtime_joint_math import quat_rotate
bare_usd()
from pxr import Usd,UsdGeom,Gf
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'outputs/runtime_validation/5430_single_slot'
ref=json.loads((OUT/'reference.json').read_text(encoding='utf-8'));meta=json.loads((OUT/'build.json').read_text(encoding='utf-8'))
stage=Usd.Stage.Open(meta['scene']);cache=UsdGeom.XformCache();worst=0.;tested=0
q=meta['tube_quaternion_wxyz'];xyzw=q[1:]+q[:1]
def mul(R,v):return [sum(R[i][j]*v[j] for j in range(3)) for i in range(3)]
for g in ref['shapes']:
    tube=g['body_name'].startswith('tube/');path=('/World/Tube/' if tube else '/World/Instrument/')+f"g{g['id']:04}"
    prim=stage.GetPrimAtPath(path)
    if not prim:raise RuntimeError('Missing '+path)
    transform=cache.GetLocalToWorldTransform(prim)
    probes=[[0,0,0],[.01,0,0],[0,.01,0],[0,0,.01]]
    if g['type']==7:
        actual_points=list(UsdGeom.Mesh(prim).GetPointsAttr().Get());expected_points=ref['meshes'][str(g['mesh_id'])]['points']
        if len(actual_points)!=len(expected_points):raise RuntimeError('Mesh point count changed')
        probes+=expected_points
    for index,v in enumerate(probes):
        local=[v[i]*g['size'][i] for i in range(3)] if g['type']==6 else v
        expected=[g['position'][i]+mul(g['rotation'],local)[i] for i in range(3)]
        if tube:
            expected=[expected[i]-ref['tube_initial_root_position'][i] for i in range(3)]
            rotated=quat_rotate(xyzw,expected);expected=[meta['tube_start_position'][i]+rotated[i] for i in range(3)]
        point=actual_points[index-4] if g['type']==7 and index>=4 else v
        actual=transform.Transform(Gf.Vec3d(*point));worst=max(worst,math.dist(actual,expected));tested+=1
report=dict(shapes=len(ref['shapes']),points_tested=tested,max_world_point_error_m=worst,
    passed=worst<1e-6,scope='all source collider poses/scales and all mesh vertices; does not validate cooked contact equivalence')
(OUT/'geometry_audit.json').write_text(json.dumps(report,indent=2),encoding='utf-8');print(json.dumps(report))
if not report['passed']:raise RuntimeError('Geometry mismatch')
