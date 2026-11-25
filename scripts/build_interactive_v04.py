"""Build reusable interactive assets and a non-destructive v04 scene overlay.
Run with Isaac's Python, or any Python environment providing pxr.
"""
from pathlib import Path
import hashlib, json, math, re, shutil, xml.etree.ElementTree as ET
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Sdf, Gf
from mjcf_to_usd import load_mesh, load_root

PROJECT = Path(__file__).resolve().parent.parent
OUT = PROJECT / 'assets/interactive'
SOURCE = PROJECT / 'assets/scenes/biolab_main_v03.usda'
SCENE = PROJECT / 'assets/scenes/biolab_main_v04.usda'
REPO = Path(__import__('os').environ.get('AUTOBIO_REPO', r'C:\Users\17690\.codex\tmp-repos\AutoBio'))

def fresh(path):
    # Generated outputs only; avoid changing any source layer through USD composition.
    if path.exists():
        path.unlink()
    return Usd.Stage.CreateNew(str(path))

def init(stage, path):
    root = UsdGeom.Xform.Define(stage, path).GetPrim()
    stage.SetDefaultPrim(root)
    UsdGeom.SetStageMetersPerUnit(stage, 1)
    UsdGeom.SetStageUpAxis(stage, 'Z')
    return root

def attr(prim, name, typ, value):
    prim.CreateAttribute(name, typ, custom=True).Set(value)

def rigid(prim, mass):
    UsdPhysics.RigidBodyAPI.Apply(prim).CreateRigidBodyEnabledAttr(True)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)
    prim.AddAppliedSchema('PhysxRigidBodyAPI')
    attr(prim, 'physxRigidBody:enableCCD', Sdf.ValueTypeNames.Bool, True)
    attr(prim, 'physxRigidBody:solverPositionIterationCount', Sdf.ValueTypeNames.Int, 16)
    attr(prim, 'physxRigidBody:solverVelocityIterationCount', Sdf.ValueTypeNames.Int, 4)

def collision(prim, mesh=False):
    UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(True)
    if mesh:
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr('convexHull')
    prim.AddAppliedSchema('PhysxCollisionAPI')
    attr(prim, 'physxCollision:contactOffset', Sdf.ValueTypeNames.Float, 0.0001)
    attr(prim, 'physxCollision:restOffset', Sdf.ValueTypeNames.Float, 0.0)
    UsdGeom.Imageable(prim).CreateVisibilityAttr('invisible')

def fix_boxes(stage):
    fixed = []
    for prim in list(stage.Traverse()):
        if prim.GetTypeName() != 'Box':
            continue
        extent = prim.GetAttribute('extent').Get()
        if not extent or len(extent) != 2:
            raise ValueError(f'Missing box dimensions: {prim.GetPath()}')
        half = (Gf.Vec3d(extent[1]) - Gf.Vec3d(extent[0])) / 2
        if min(half) <= 0:
            raise ValueError(f'Invalid box dimensions: {prim.GetPath()}')
        prim.SetTypeName('Cube')
        cube = UsdGeom.Cube(prim)
        cube.CreateSizeAttr(2)
        cube.CreateExtentAttr([Gf.Vec3f(-1), Gf.Vec3f(1)])
        UsdGeom.Xformable(prim).AddScaleOp(opSuffix='physicalSize').Set(half)
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            collision(prim)
        fixed.append(str(prim.GetPath()))
    return fixed

def make_tip():
    stage = fresh(OUT/'tip_200ul.usda')
    root = init(stage, '/Tip200uL')
    root.GetReferences().AddReference('../precision_tasks/pipette_tip_200ul/tip_200ul_final.usda')
    rigid(root, 0.00025)  # Approximate asset mass, 0.25 g; configurable.
    attr(root, 'lab:assetType', Sdf.ValueTypeNames.Token, 'pipette_tip_200ul')
    attr(root, 'lab:capacity_uL', Sdf.ValueTypeNames.Double, 200.0)
    attr(root, 'lab:liquidVolume_uL', Sdf.ValueTypeNames.Double, 0.0)
    attr(root, 'lab:attached', Sdf.ValueTypeNames.Bool, False)
    for p in stage.Traverse():
        if p.HasAPI(UsdPhysics.CollisionAPI):
            collision(p, p.IsA(UsdGeom.Mesh))
    stage.GetRootLayer().Save()

def make_pipette():
    stage = fresh(OUT/'pipette_200ul.usda')
    root = init(stage, '/Pipette200uL')
    source = REPO/'autobio/model/object/pipette.gen.xml'
    doc = load_root(source)
    meshes = {m.get('name'): m for m in doc.find('asset').findall('mesh')}
    assets = REPO/'autobio/assets'
    joints = []
    def emit_body(node, path, parent_body=None):
        body = UsdGeom.Xform.Define(stage, path).GetPrim()
        # The top-level MJCF body pose is an example placement, not asset geometry.
        if parent_body is not None:
            pos = [float(v) for v in node.get('pos','0 0 0').split()]
            UsdGeom.Xformable(body).AddTranslateOp().Set(Gf.Vec3d(*pos))
        rigid(body, 0.12 if parent_body is None else 0.005)
        for n, g in enumerate(node.findall('geom')):
            group = g.get('group','2')
            is_collision = group == '3'
            name = g.get('mesh') or g.get('name') or f'geom_{n}'
            name = re.sub(r'[^A-Za-z0-9_]', '_', name)
            gp = path + ('/Collision/' if is_collision else '/Visual/') + name
            if g.get('mesh'):
                info = meshes[g.get('mesh')]
                scale = tuple(map(float,info.get('scale','1 1 1').split()))
                points, faces = load_mesh(str(assets/info.get('file')), scale)
                geom = UsdGeom.Mesh.Define(stage,gp)
                geom.CreatePointsAttr([Gf.Vec3f(*x) for x in points])
                geom.CreateFaceVertexCountsAttr([len(x) for x in faces])
                geom.CreateFaceVertexIndicesAttr([v for x in faces for v in x])
                geom.CreateSubdivisionSchemeAttr('none')
                geom.CreateExtentAttr([Gf.Vec3f(*[min(v[i] for v in points) for i in range(3)]),Gf.Vec3f(*[max(v[i] for v in points) for i in range(3)])])
            else:
                size = list(map(float,g.get('size','0.004 0.004 0.004').split()))
                kind = g.get('type','box')
                if kind == 'box':
                    geom = UsdGeom.Cube.Define(stage,gp)
                    geom.CreateSizeAttr(2)
                    UsdGeom.Xformable(geom).AddScaleOp().Set(Gf.Vec3d(*size))
                elif kind == 'cylinder':
                    geom = UsdGeom.Cylinder.Define(stage,gp)
                    geom.CreateRadiusAttr(size[0]);geom.CreateHeightAttr(2*size[1]);geom.CreateAxisAttr('Z')
                else:
                    raise ValueError(f'Unsupported pipette geom {kind}')
            gpprim = geom.GetPrim()
            if g.get('pos'):
                UsdGeom.Xformable(gpprim).AddTranslateOp().Set(Gf.Vec3d(*map(float,g.get('pos').split())))
            if g.get('quat'):
                UsdGeom.Xformable(gpprim).AddOrientOp().Set(Gf.Quatf(*map(float,g.get('quat').split())))
            ops = UsdGeom.Xformable(gpprim).GetOrderedXformOps()
            order = {UsdGeom.XformOp.TypeTranslate: 0, UsdGeom.XformOp.TypeOrient: 1, UsdGeom.XformOp.TypeScale: 2}
            UsdGeom.Xformable(gpprim).SetXformOpOrder(sorted(ops, key=lambda op: order.get(op.GetOpType(), 1)))
            if is_collision:
                collision(gpprim, gpprim.IsA(UsdGeom.Mesh))
            else:
                color = (0.5,0.7,0.9) if 'knob' in name or 'pusher' in name else (0.8,0.8,0.8)
                geom.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        for spec in node.findall('joint'):
            linear = spec.get('type') == 'slide'
            jp = '/Pipette200uL/Joints/'+spec.get('name')
            joint = (UsdPhysics.PrismaticJoint if linear else UsdPhysics.RevoluteJoint).Define(stage,jp)
            joint.CreateBody0Rel().SetTargets([Sdf.Path(parent_body)])
            joint.CreateBody1Rel().SetTargets([body.GetPath()])
            joint.CreateAxisAttr('Z')
            # All three source joints have zero anchors and zero child body offsets.
            joint.CreateLocalPos0Attr(Gf.Vec3f(0));joint.CreateLocalPos1Attr(Gf.Vec3f(0))
            limits = list(map(float,spec.get('range').split()))
            if not linear: limits = [math.degrees(x) for x in limits]
            joint.CreateLowerLimitAttr(limits[0]);joint.CreateUpperLimitAttr(limits[1])
            joint.CreateCollisionEnabledAttr(False)
            drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(),'linear' if linear else 'angular')
            drive.CreateTypeAttr('force')
            drive.CreateStiffnessAttr(80 if linear else 0.5)
            drive.CreateDampingAttr(2 if linear else 0.1)
            drive.CreateMaxForceAttr(8 if linear else 0.05)
            drive.CreateTargetPositionAttr(0)
            joints.append({'path':jp,'type':'linear' if linear else 'angular','limits':limits})
        for child in node.findall('body'):
            emit_body(child,'/Pipette200uL/'+child.get('name'),path)
    emit_body(doc.find('worldbody/body'),'/Pipette200uL/Body')
    # Preserve contact exclusions for all moving controls, not just each joint pair.
    body = stage.GetPrimAtPath('/Pipette200uL/Body')
    UsdPhysics.FilteredPairsAPI.Apply(body).CreateFilteredPairsRel().SetTargets([
        Sdf.Path('/Pipette200uL/pipette_button'),
        Sdf.Path('/Pipette200uL/pipette_knob'),
        Sdf.Path('/Pipette200uL/pipette_ejector')])
    socket=UsdGeom.Xform.Define(stage,'/Pipette200uL/Body/TipAttachmentSocket')
    socket.AddTranslateOp().Set(Gf.Vec3d(0,0,-0.003))
    attr(root,'lab:assetType',Sdf.ValueTypeNames.Token,'pipette_200ul')
    attr(root,'lab:capacity_uL',Sdf.ValueTypeNames.Double,200.0)
    attr(root,'lab:liquidVolume_uL',Sdf.ValueTypeNames.Double,0.0)
    attr(root,'lab:volumeModel',Sdf.ValueTypeNames.Token,'conservative_scalar_no_fluid_dynamics')
    stage.GetRootLayer().Save()
    return joints

def make_scene():
    stage=fresh(SCENE)
    stage.GetRootLayer().subLayerPaths=['biolab_main_v03.usda']
    UsdGeom.SetStageMetersPerUnit(stage,1.0)
    UsdGeom.SetStageUpAxis(stage,'Z')
    source_layer=Sdf.Layer.FindOrOpen(str(SOURCE))
    stage.SetTimeCodesPerSecond(source_layer.timeCodesPerSecond)
    stage.SetStartTimeCode(source_layer.startTimeCode)
    stage.SetEndTimeCode(source_layer.endTimeCode)
    stage.SetDefaultPrim(stage.GetPrimAtPath('/World'))
    # Reuse existing precise open-socket geometry; keep all original placements.
    tips=[]
    for prim in list(stage.Traverse()):
        if re.fullmatch(r'/World/TaskArea/Objects/Tips/RackPool/Tip_[A-H][0-9]{2}',str(prim.GetPath())):
            prim.GetReferences().SetReferences([Sdf.Reference('../interactive/tip_200ul.usda')])
            tips.append(str(prim.GetPath()))
    pipette=stage.GetPrimAtPath('/World/TaskArea/Equipment/Pipette')
    bounds=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render'],False)
    original_center=bounds.ComputeWorldBound(pipette).ComputeAlignedRange().GetMidpoint()
    pipette.GetReferences().SetReferences([Sdf.Reference('../interactive/pipette_200ul.usda')])
    # Delete the legacy rigid body authored directly into the old root layer.
    old=stage.OverridePrim('/World/TaskArea/Equipment/Pipette/Pipette')
    old.SetActive(False)
    bounds.Clear()
    new_center=bounds.ComputeWorldBound(pipette).ComputeAlignedRange().GetMidpoint()
    delta=original_center-new_center
    xf=UsdGeom.Xformable(pipette)
    translate=next(op for op in xf.GetOrderedXformOps() if op.GetOpType()==UsdGeom.XformOp.TypeTranslate)
    parent_world=UsdGeom.XformCache().GetLocalToWorldTransform(pipette.GetParent())
    translate.Set(translate.Get()+parent_world.GetInverse().TransformDir(delta))
    fixed=fix_boxes(stage)
    for prim in stage.Traverse():
        if '/Collision/' in str(prim.GetPath()) and prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdGeom.Imageable(prim).CreateVisibilityAttr('invisible')
    vessels=[]
    for path,capacity in [('/World/TaskArea/Stations/Dish100',15000.0),('/World/TaskArea/Stations/Cryovial',1800.0)]:
        prim=stage.GetPrimAtPath(path)
        if prim:
            attr(prim,'lab:capacity_uL',Sdf.ValueTypeNames.Double,capacity)
            attr(prim,'lab:liquidVolume_uL',Sdf.ValueTypeNames.Double,0.0)
            attr(prim,'lab:volumeModel',Sdf.ValueTypeNames.Token,'conservative_scalar_no_fluid_dynamics')
            target=UsdGeom.Xform.Define(stage,path+'/FillTarget')
            target.AddTranslateOp().Set(Gf.Vec3d(0,0,0.01 if 'Dish' in path else 0.025))
            vessels.append(path)
    # Add a physically defined culture-medium source in an existing free bench slot.
    # This is a reusable open reservoir, not a biological or fluid solver.
    root=UsdGeom.Xform.Define(stage,'/World/TaskArea/CultureMedia').GetPrim()
    attr(root,'lab:zone',Sdf.ValueTypeNames.Token,'culture_media')
    reservoir=UsdGeom.Xform.Define(stage,'/World/TaskArea/CultureMedia/Reservoir')
    reservoir.AddTranslateOp().Set(Gf.Vec3d(-1.65,-0.36,0.873))
    rp=reservoir.GetPrim();rigid(rp,0.035)
    attr(rp,'lab:capacity_uL',Sdf.ValueTypeNames.Double,10000.0)
    attr(rp,'lab:liquidVolume_uL',Sdf.ValueTypeNames.Double,5000.0)
    attr(rp,'lab:volumeModel',Sdf.ValueTypeNames.Token,'conservative_scalar_no_fluid_dynamics')
    # Five wall pieces leave the top open for tool insertion.
    for name,pos,dims in [('Bottom',(0,0,0.001),(0.06,0.04,0.002)),('Left',(-0.029,0,0.012),(0.002,0.04,0.022)),('Right',(0.029,0,0.012),(0.002,0.04,0.022)),('Front',(0,-0.019,0.012),(0.056,0.002,0.022)),('Back',(0,0.019,0.012),(0.056,0.002,0.022))]:
        cube=UsdGeom.Cube.Define(stage,str(reservoir.GetPath())+'/'+name)
        cube.CreateSizeAttr(1)
        cube.AddTranslateOp().Set(Gf.Vec3d(*pos));cube.AddScaleOp().Set(Gf.Vec3d(*dims))
        cube.CreateDisplayColorAttr([Gf.Vec3f(0.72,0.82,0.88)])
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    UsdGeom.Xform.Define(stage,str(reservoir.GetPath())+'/FillTarget').AddTranslateOp().Set(Gf.Vec3d(0,0,0.018))
    vessels.append(str(reservoir.GetPath()))
    stage.GetRootLayer().customLayerData={**source_layer.customLayerData,'version':'v04','purpose':'reusable interaction foundation; no scripted task demonstration','base_scene':'biolab_main_v03.usda','fluid_model':'volume bookkeeping only; no CFD or cell growth'}
    stage.GetRootLayer().Save()
    return {'scene':str(SCENE),'tips':tips,'repaired_box_colliders':fixed,'pipette':'/World/TaskArea/Equipment/Pipette','vessels':vessels,'reservoir_initial_volume_uL':5000.0,'limitations':['No robot manipulation policy or task demonstration','Volume transfer requires explicit caller-provided contact and attachment checks; no CFD or biology','Screw caps and two previously invalid instrument conversions remain outside this revision']}

def main():
    OUT.mkdir(parents=True,exist_ok=True)
    before=hashlib.sha256(SOURCE.read_bytes()).hexdigest()
    make_tip();joints=make_pipette();report=make_scene()
    report['pipette_asset_joints']=joints
    report['source_v03_unchanged']=before==hashlib.sha256(SOURCE.read_bytes()).hexdigest()
    report['source_v03_sha256']=before
    (PROJECT/'docs/interactive_v04.json').write_text(json.dumps(report,indent=2,ensure_ascii=False),encoding='utf-8')
    print(json.dumps({'tips':len(report['tips']),'box_colliders_repaired':len(report['repaired_box_colliders']),'pipette_joints':len(joints),'source_unchanged':report['source_v03_unchanged'],'scene':report['scene']},ensure_ascii=False,indent=2))
if __name__=='__main__':main()
