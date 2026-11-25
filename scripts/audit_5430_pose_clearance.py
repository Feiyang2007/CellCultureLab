"""Bounded geometric search inside source 5mm success tolerance, not a task run.

Original model geometry, never rescale shapes. Read-only candidate poses in
MuJoCo forward collision queries; these are NOT dynamics trajectories.
"""
import argparse,json,sys
from pathlib import Path
from xml.sax.saxutils import quoteattr
parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
home=Path.home()/'AppData/LocalLow/CellCultureLabReference';sys.path.insert(0,str(home/'mujoco330'))
import mujoco as mj
import numpy as np
from scipy.optimize import differential_evolution,minimize
ROOT=Path(__file__).resolve().parents[1];ref=json.loads((ROOT/'outputs/runtime_validation/5430_single_slot/reference.json').read_text(encoding='utf-8'))
xml=f'''<mujoco><asset><model name="inst" file={quoteattr(ref['instrument_xml'])}/><model name="tube" file={quoteattr(ref['tube_xml'])}/></asset>
<worldbody><attach model="inst" body="world" prefix="inst/"/><body name="tube_root" pos="0 0 1"><freejoint name="tube_free"/>
<attach model="tube" body="world" prefix="tube/"/></body></worldbody></mujoco>'''
m=mj.MjModel.from_xml_string(xml);d=mj.MjData(m);adr=m.jnt_qposadr[m.joint('tube_free').id]
slot=np.array(ref['slots'][0]['position']);R=np.array(ref['slots'][0]['rotation']);baseq=np.array(ref['tube_rotation_wxyz'])
root=m.body('tube_root').id;calls=0
def evaluate(x):
    global calls
    calls+=1;delta=np.array(x[:3]);position=slot+R@(np.array([0,0,-.03])+delta)
    q=baseq.copy()
    if len(x)>3:mj.mju_quatIntegrate(q,np.array([x[3],x[4],0.]),1.)
    d.qpos[adr:adr+3]=position;d.qpos[adr+3:adr+7]=q;mj.mj_forward(m,d)
    distances=[float(c.dist) for c in d.contact if m.body_weldid[m.geom_bodyid[c.geom1]]==root or m.body_weldid[m.geom_bodyid[c.geom2]]==root]
    negative=np.maximum(0,-np.array(distances));maximum=float(np.max(negative)) if len(negative) else 0.
    return dict(delta_slot_local_m=delta.tolist(),local_tilt_rad=list(x[3:]),position=position.tolist(),quaternion_wxyz=q.tolist(),
        target_distance_m=float(np.linalg.norm(delta)),max_penetration_m=maximum,contacts=len(distances),sum_squared_penetration=float(np.dot(negative,negative)))
def cost(x):
    r=evaluate(x);outside=max(0,r['target_distance_m']-.00499)
    return r['max_penetration_m']**2+.01*r['sum_squared_penetration']+100*outside**2
initial=evaluate([0,0,0]);runs=[]
for label,bounds in [('translation_only',[(-.003,.003),(-.003,.003),(0,.0049)]),
                     ('translation_and_tilt',[(-.003,.003),(-.003,.003),(0,.0049),(-.2,.2),(-.2,.2)])]:
    start=calls;opt=differential_evolution(cost,bounds,seed=0,maxiter=45,popsize=8,polish=False,tol=1e-8)
    polished=minimize(cost,opt.x,method='Powell',bounds=bounds,options={'maxfev':1500,'xtol':1e-9,'ftol':1e-9})
    best=opt.x if cost(opt.x)<cost(polished.x) else polished.x
    result=evaluate(best);result.update(search=label,evaluations=calls-start,
        geometric_candidate_found=result['max_penetration_m']<=.00005 and result['target_distance_m']<.005)
    runs.append(result)
report=dict(scope='bounded pose clearance diagnostic; no dynamic insertion or task acceptance',
    thresholds_declared_before_run=dict(max_penetration_m=.00005,target_distance_m=.005,max_local_tilt_per_axis_rad=.2),
    initial=initial,runs=runs,geometry_changed=False,full_task_validated=False)
args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,indent=2),encoding='utf-8');print(json.dumps(report),flush=True)
