"""Build all source collider geometry for a fixed-instrument, guided-tube fixture."""
import hashlib,json,math
from pathlib import Path
from validate_gpu_capacity import bare_usd
bare_usd()
from pxr import Usd,UsdGeom,UsdPhysics,UsdShade,Sdf,Gf
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'outputs/runtime_validation/5430_single_slot'
ref=json.loads((OUT/'reference.json').read_text(encoding='utf-8'))

def matrix(rotation,position):
    return Gf.Matrix4d(*[rotation[j][i] if i<3 and j<3 else (position[j] if i==3 and j<3 else float(i==j)) for i in range(4) for j in range(4)])
def quat(wxyz):return Gf.Quatf(wxyz[0],Gf.Vec3f(*wxyz[1:]))
def api(p,token):
    tokens=list(p.GetAppliedSchemas())
    if token not in tokens:tokens.append(token)
    p.SetMetadata('apiSchemas',Sdf.TokenListOp.CreateExplicit(tokens))

stage=Usd.Stage.CreateInMemory();UsdGeom.SetStageMetersPerUnit(stage,1);UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z)
stage.SetDefaultPrim(stage.DefinePrim('/World','Xform'))
scene=UsdPhysics.Scene.Define(stage,'/World/physicsScene');scene.CreateGravityDirectionAttr(Gf.Vec3f(0,0,-1));scene.CreateGravityMagnitudeAttr(9.81)
api(scene.GetPrim(),'PhysxSceneAPI')
for name,typ,value in [('timeStepsPerSecond',Sdf.ValueTypeNames.UInt,1000),('enableGPUDynamics',Sdf.ValueTypeNames.Bool,False),('broadphaseType',Sdf.ValueTypeNames.Token,'MBP')]:
    scene.GetPrim().CreateAttribute('physxScene:'+name,typ).Set(value)
mat=UsdShade.Material.Define(stage,'/World/ContactMaterial');ma=UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
ma.CreateStaticFrictionAttr(1);ma.CreateDynamicFrictionAttr(1);ma.CreateRestitutionAttr(0)
tube=UsdGeom.Xform.Define(stage,'/World/Tube');rb=UsdPhysics.RigidBodyAPI.Apply(tube.GetPrim());rb.CreateRigidBodyEnabledAttr(True)
mass=UsdPhysics.MassAPI.Apply(tube.GetPrim());mass.CreateMassAttr(ref['tube_mass_kg']);mass.CreateCenterOfMassAttr(Gf.Vec3f(*ref['tube_com_local']))
mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*ref['tube_principal_inertia']));mass.CreatePrincipalAxesAttr(quat(ref['tube_principal_axes_wxyz']))
api(tube.GetPrim(),'PhysxRigidBodyAPI')
api(tube.GetPrim(),'PhysxContactReportAPI')
tube.GetPrim().CreateAttribute('physxContactReport:threshold',Sdf.ValueTypeNames.Float).Set(0)
tube.GetPrim().CreateAttribute('physxRigidBody:linearDamping',Sdf.ValueTypeNames.Float).Set(0)
tube.GetPrim().CreateAttribute('physxRigidBody:angularDamping',Sdf.ValueTypeNames.Float).Set(0)
tube.GetPrim().CreateAttribute('physxRigidBody:enableCCD',Sdf.ValueTypeNames.Bool).Set(True)
slot=ref['slots'][0];R=slot['rotation'];p=slot['position'];start=[p[i]+.02*R[i][2] for i in range(3)]
q=ref['tube_rotation_wxyz'];tube.AddTranslateOp().Set(Gf.Vec3d(*start));tube.AddOrientOp().Set(Gf.Quatf(q[0],Gf.Vec3f(*q[1:])))
counts={'instrument':0,'tube':0};max_origin_error=0.
for g in ref['shapes']:
    is_tube=g['body_name'].startswith('tube/');group='tube' if is_tube else 'instrument';counts[group]+=1
    path=('/World/Tube/' if is_tube else '/World/Instrument/')+f"g{g['id']:04}"
    typ=g['type'];s=g['size']
    if typ==7:
        shape=UsdGeom.Mesh.Define(stage,path);mesh=ref['meshes'][str(g['mesh_id'])]
        shape.CreatePointsAttr([Gf.Vec3f(*v) for v in mesh['points']]);shape.CreateFaceVertexCountsAttr([3]*len(mesh['faces']))
        shape.CreateFaceVertexIndicesAttr([v for f in mesh['faces'] for v in f]);shape.CreateSubdivisionSchemeAttr('none')
        UsdPhysics.MeshCollisionAPI.Apply(shape.GetPrim()).CreateApproximationAttr('convexHull')
    elif typ==6:
        shape=UsdGeom.Cube.Define(stage,path);shape.CreateSizeAttr(2);shape.AddScaleOp().Set(Gf.Vec3f(*s))
    elif typ==5:
        shape=UsdGeom.Cylinder.Define(stage,path);shape.CreateAxisAttr('Z');shape.CreateRadiusAttr(s[0]);shape.CreateHeightAttr(2*s[1])
    else:raise ValueError('Unsupported source shape '+str(typ))
    pos=list(g['position'])
    if is_tube:pos=[pos[i]-ref['tube_initial_root_position'][i] for i in range(3)]
    # Matrix first in xformOpOrder, scale (if present) last: local scaling then pose.
    x=UsdGeom.Xformable(shape);ops=x.GetOrderedXformOps();op=x.AddTransformOp();op.Set(matrix(g['rotation'],pos));x.SetXformOpOrder([op,*ops])
    UsdPhysics.CollisionAPI.Apply(shape.GetPrim()).CreateCollisionEnabledAttr(True)
    api(shape.GetPrim(),'PhysxCollisionAPI')
    shape.GetPrim().CreateAttribute('physxCollision:contactOffset',Sdf.ValueTypeNames.Float).Set(.0002)
    shape.GetPrim().CreateAttribute('physxCollision:restOffset',Sdf.ValueTypeNames.Float).Set(0)
    UsdShade.MaterialBindingAPI.Apply(shape.GetPrim()).Bind(mat,materialPurpose='physics')
    if not is_tube:
        actual=UsdGeom.XformCache().GetLocalToWorldTransform(shape.GetPrim()).ExtractTranslation()
        max_origin_error=max(max_origin_error,math.dist(actual,g['position']))
guide=UsdPhysics.PrismaticJoint.Define(stage,'/World/Guide');guide.CreateBody1Rel().SetTargets(['/World/Tube'])
guide.CreateAxisAttr('Z');guide.CreateLocalPos0Attr(Gf.Vec3f(*start));guide.CreateLocalRot0Attr(quat(q))
guide.CreateLocalPos1Attr(Gf.Vec3f(0));guide.CreateLocalRot1Attr(Gf.Quatf(1))
guide.CreateLowerLimitAttr(-.10);guide.CreateUpperLimitAttr(.03)
destination=OUT/'scene.usda';stage.GetRootLayer().Export(str(destination))
reopened=Usd.Stage.Open(str(destination));errors=[str(e) for p in reopened.Traverse() for e in p.GetPrimIndex().localErrors]
if errors or max_origin_error>1e-7:raise RuntimeError(str((errors,max_origin_error)))
report=dict(scene=str(destination),scene_sha256=hashlib.sha256(destination.read_bytes()).hexdigest(),
    reference_sha256=hashlib.sha256((OUT/'reference.json').read_bytes()).hexdigest(),colliders=counts,
    max_instrument_origin_error_m=max_origin_error,composition_errors=errors,
    tube_start_position=start,tube_quaternion_wxyz=q,slot_rotation=R,slot_position=p,
    fixture='all instrument shapes static; tube guided along slot axis, other five DOFs fixed; no robot',
    contact_mapping='source convex pieces; Coulomb coefficient 1; rolling/torsional friction and soft-contact equivalence unverified',
    contact_offset_m=.0002,rest_offset_m=0,source_pose='lid open, rotor zero',production_ready=False)
(OUT/'build.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(report),flush=True)
