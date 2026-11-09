"""Declared diagnostic actuator, independent of source robot actuator gains."""
import math

def design(protocol,mass):
    if protocol=='force':
        return dict(protocol=protocol,basis='diagnostic constant axial forces; not source robot control',
            phases=[dict(name='insert',steps=1800,force_n=-.05),dict(name='withdraw',steps=500,force_n=.3)])
    return dict(protocol=protocol,kp_n_per_m=200.,kd_n_s_per_m=2*math.sqrt(200.*mass),
        force_limit_n=1.,basis='diagnostic axial critically damped servo; not a source robot actuator',
        phases=[dict(name='hold_entry',steps=100),dict(name='insert',steps=2000),dict(name='hold_insert',steps=500),dict(name='withdraw',steps=1000)])

def schedule(protocol):
    return [('insert',1800),('withdraw',500)] if protocol=='force' else [('hold_entry',100),('insert',2000),('hold_insert',500),('withdraw',1000)]

def command(protocol,phase,index,steps,z,v,mass,gravity_axis):
    if protocol=='force':return (-.05 if phase=='insert' else .3),None,None,False
    if phase=='hold_entry':zr,vr=.02,0.
    elif phase=='hold_insert':zr,vr=-.03,0.
    else:
        a,b=(.02,-.03) if phase=='insert' else (-.03,.02)
        s=min(1.,(index+1)/steps);duration=steps*.001
        zr=a+(b-a)*(3*s*s-2*s*s*s);vr=(b-a)*6*s*(1-s)/duration
    constants=design(protocol,mass)
    raw=constants['kp_n_per_m']*(zr-z)+constants['kd_n_s_per_m']*(vr-v)-mass*gravity_axis
    limit=constants['force_limit_n'];return max(-limit,min(limit,raw)),zr,vr,abs(raw)>limit
