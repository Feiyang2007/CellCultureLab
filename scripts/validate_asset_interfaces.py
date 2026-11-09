"""Acceptance for attachment guards, command limits and conservative volume state."""
from pathlib import Path
import json,math
from pxr import Usd,UsdGeom,Gf
from lab_assets import LabAssets
P=Path(__file__).resolve().parent.parent
stage=Usd.Stage.CreateInMemory()
stage.GetRootLayer().subLayerPaths=[str(P/'assets/scenes/biolab_main_v04.usda')]
readings={'ejector':0.0}
api=LabAssets(stage,read_joint=lambda path:readings['ejector'])
checks={}
def rejects(name,fn):
    try:fn()
    except ValueError:checks[name]=True
    else:checks[name]=False

def place_target_at_nozzle(container,tip):
    cache=UsdGeom.XformCache()
    target=stage.GetPrimAtPath(container+'/FillTarget')
    delta=cache.GetLocalToWorldTransform(stage.GetPrimAtPath(tip+'/Frames/TipEnd')).ExtractTranslation()-cache.GetLocalToWorldTransform(target).ExtractTranslation()
    prim=stage.GetPrimAtPath(container)
    xf=UsdGeom.Xformable(prim)
    op=next(x for x in xf.GetOrderedXformOps() if x.GetOpType()==UsdGeom.XformOp.TypeTranslate)
    parent_inv=cache.GetLocalToWorldTransform(prim.GetParent()).GetInverse()
    op.Set(op.Get()+parent_inv.TransformDir(delta))

tip='/World/TaskArea/Objects/Tips/RackPool/Tip_A01'
reservoir='/World/TaskArea/CultureMedia/Reservoir';dish='/World/TaskArea/Stations/Dish100'
rejects('unattached_aspiration_rejected',lambda:api.aspirate(reservoir,10))
rejects('remote_tip_attachment_rejected',lambda:api.attach_tip(tip))
cache=UsdGeom.XformCache()
world=cache.GetLocalToWorldTransform(stage.GetPrimAtPath(api.PIPETTE+'/Body/TipAttachmentSocket'))
parent_inv=cache.GetLocalToWorldTransform(stage.GetPrimAtPath(tip).GetParent()).GetInverse()
xf=UsdGeom.Xformable(stage.GetPrimAtPath(tip));xf.ClearXformOpOrder()
ops=xf.GetOrderedXformOps()
# A fixture setup places the objects together before testing constraint creation.
old=stage.GetPrimAtPath(tip).GetAttribute('xformOp:transform')
if old:old.Set(world*parent_inv);xf.SetXformOpOrder([UsdGeom.XformOp(old)])
else:xf.AddTransformOp().Set(world*parent_inv)
api.attach_tip(tip)
checks['physical_constraint_created']=bool(api._tip())
rejects('double_attachment_rejected',lambda:api.attach_tip(tip))
rejects('remote_fill_zone_rejected',lambda:api.aspirate(reservoir,50))
place_target_at_nozzle(reservoir,tip)
joint=stage.GetPrimAtPath(api._tip().GetRelationship('lab:attachmentJoint').GetTargets()[0])
joint.SetActive(False)
rejects('inactive_constraint_transfer_rejected',lambda:api.aspirate(reservoir,10))
joint.SetActive(True)
event=api.aspirate(reservoir,50)
checks['aspiration_conserves_volume']=event['total_before_uL']==event['total_after_uL']==5000
state=api.snapshot()
for value,name in [(-1,'negative_volume'),(float('nan'),'nan_volume'),(201,'over_capacity')]:
    rejects(name+'_rejected',lambda value=value:api.aspirate(reservoir,value))
checks['failed_transfers_do_not_mutate_state']=state==api.snapshot()
place_target_at_nozzle(dish,tip)
event=api.dispense(dish,20)
checks['dispense_conserves_volume']=event['total_before_uL']==event['total_after_uL']==50
checks['tip_remaining_volume_correct']=api._tip().GetAttribute('lab:liquidVolume_uL').Get()==30
rejects('insufficient_tip_volume_rejected',lambda:api.dispense(dish,31))
rejects('ejection_requires_measured_stroke',api.eject_tip)
readings['ejector']=-0.009
api.eject_tip()
checks['ejection_clears_attachment']=not stage.GetPrimAtPath(tip).GetAttribute('lab:attached').Get()
api.attach_tip(tip)
checks['reattachment_reactivates_constraint']=bool(api._tip())
rejects('plunger_limit_enforced',lambda:api.set_control('plunger',-0.1))
api.set_control('plunger',-0.004)
checks['drive_target_is_not_reported_as_measurement']=LabAssets(stage).snapshot()['controls']['plunger']['measured'] is None
result={'passed':all(checks.values()),'checks':checks,'scope':'API and authored constraint/state contracts; actual motion validated separately'}
(P/'docs/interface_validation_v04.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
print(json.dumps(result,indent=2))
raise SystemExit(not result['passed'])
