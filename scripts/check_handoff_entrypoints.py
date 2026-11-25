"""Compose selected working-copy entrypoints offline. Does not launch Isaac or render."""
from pathlib import Path
import argparse,json,sys
from handoff_portable import usd_bootstrap,error_text
ap=argparse.ArgumentParser();ap.add_argument('--project',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
root=a.project.resolve();usd_bootstrap()
from pxr import Usd,UsdGeom,UsdPhysics
entries=['outputs/gui_review/biolab_v06_original_local.usda','assets/scenes/test_instruments5.usda','assets/scenes/mini_closed_loop_v2.usda','outputs/runtime_validation/c1000_isaac_lock_pilot/scene.usda']
results=[]
for name in entries:
    row={'file':name,'composition_errors':[]}
    try:
        stage=Usd.Stage.Open(str(root/name));count=0;rigid=0;joints=0
        if stage is None:raise RuntimeError('Stage is None')
        for prim in stage.Traverse():
            count+=1;rigid+=prim.HasAPI(UsdPhysics.RigidBodyAPI);joints+=prim.IsA(UsdPhysics.Joint)
            row['composition_errors'].extend(str(e) for e in prim.GetPrimIndex().localErrors)
        row.update(prims=count,rigid_bodies=rigid,joints=joints,up_axis=UsdGeom.GetStageUpAxis(stage),meters_per_unit=UsdGeom.GetStageMetersPerUnit(stage))
        if name.startswith('outputs/gui_review'):
            row['robot_roots']={p:bool(stage.GetPrimAtPath(p)) for p in ['/World/Robot/Robot1Mount/ur5e','/World/Robot/Robot2Mount/ur5e','/World/Robot/Gripper1Temp/Robotiq_2F_85_edit','/World/Robot/Gripper2Temp/Robotiq_2F_85_edit']}
        row['static_composition_passed']=not row['composition_errors'] and all(row.get('robot_roots',{}).values()) and row['up_axis']=='Z' and row['meters_per_unit']==1
    except Exception as e:row.update(static_composition_passed=False,error=error_text(e))
    results.append(row)
out={'physics_rerun':False,'rendered':False,'scope':'USD static composition and metadata only; no GUI, shader or physics validation.','all_static_entries_passed':all(x['static_composition_passed'] for x in results),'entries':results}
a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(out,ensure_ascii=False,indent=2),encoding='utf8')
print(json.dumps({'all_static_entries_passed':out['all_static_entries_passed'],'entries':len(results)},ensure_ascii=False))
sys.exit(0 if out['all_static_entries_passed'] else 1)
