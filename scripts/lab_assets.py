"""Reusable asset controls and observable state; no task policy or demonstration.
Units: joint translations in metres, joint rotations in degrees, liquid volume in uL.
Volume is a conservative scalar model, not fluid dynamics or cell biology.
"""
import math, json, zlib
from pathlib import Path
from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf

class LabAssets:
    PIPETTE='/World/TaskArea/Equipment/Pipette'
    CONTROLS={'plunger':('pipette_button','linear',-0.008,0.0),
              'ejector':('pipette_ejector','linear',-0.0095,0.0),
              'volume_knob':('knob','angular',-math.degrees(1.5),math.degrees(1.5))}
    def __init__(self,stage,event_log=None,read_joint=None,sim_time=None):
        self.stage=stage
        self.event_log=Path(event_log) if event_log else None
        self.read_joint=read_joint
        self.sim_time=sim_time or (lambda: None)

    def _prim(self,path):
        prim=self.stage.GetPrimAtPath(path)
        if not prim:raise ValueError(f'Missing asset: {path}')
        return prim

    def _number(self,value):
        value=float(value)
        if not math.isfinite(value):raise ValueError('Value must be finite')
        return value

    def _event(self,kind,**details):
        event={'event':kind,'simulation_time_s':self.sim_time(),**details}
        if self.event_log:
            self.event_log.parent.mkdir(parents=True,exist_ok=True)
            with self.event_log.open('a',encoding='utf-8') as f:f.write(json.dumps(event,ensure_ascii=False)+'\n')
        return event

    def set_control(self,name,value):
        if name not in self.CONTROLS:raise ValueError(f'Unknown control: {name}')
        joint_name,kind,lo,hi=self.CONTROLS[name]
        value=self._number(value)
        if not lo<=value<=hi:raise ValueError(f'{name} must be in [{lo},{hi}]')
        path=self.PIPETTE+'/Joints/'+joint_name
        joint=self._prim(path)
        drive=UsdPhysics.DriveAPI(joint,kind)
        if not drive:raise ValueError(f'Missing physical drive: {path}')
        drive.GetTargetPositionAttr().Set(value)
        return self._event('control_target',control=name,target=value,units='m' if kind=='linear' else 'degrees')

    def _world(self,path):
        return UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(self._prim(path))

    def _position(self,path):return self._world(path).ExtractTranslation()

    def attach_tip(self,tip_path,tolerance_m=0.002):
        tip=self._prim(tip_path)
        if tip.GetAttribute('lab:assetType').Get()!='pipette_tip_200ul':raise ValueError('Not a compatible tip')
        tool=self._prim(self.PIPETTE)
        relation=tool.GetRelationship('lab:attachedTip')
        if relation and relation.GetTargets():raise ValueError('Pipette already has a tip')
        if tip.GetAttribute('lab:attached').Get():raise ValueError('Tip is already attached')
        body=self.PIPETTE+'/Body'
        socket=body+'/TipAttachmentSocket'
        entrance=tip_path+'/Frames/SocketEntrance'
        distance=(self._position(socket)-self._position(entrance)).GetLength()
        if distance>tolerance_m:raise ValueError(f'Tip is not inserted: separation={distance:.6f} m')
        direction0=self._world(socket).TransformDir(Gf.Vec3d(0,0,-1)).GetNormalized()
        direction1=self._world(entrance).TransformDir(Gf.Vec3d(0,0,-1)).GetNormalized()
        if Gf.Dot(direction0,direction1)<math.cos(math.radians(10)):raise ValueError('Tip axes are not aligned')
        # Create the constraint at the observed pose, with no pose teleportation.
        joint_path='/World/LabAttachments/Tip_'+format(zlib.crc32(tip_path.encode()),'08x')
        joint=UsdPhysics.FixedJoint.Define(self.stage,joint_path)
        joint.GetPrim().SetActive(True)
        joint.CreateBody0Rel().SetTargets([Sdf.Path(body)])
        joint.CreateBody1Rel().SetTargets([tip.GetPath()])
        anchor=self._world(entrance)
        for i,body_path in enumerate((body,tip_path)):
            frame=anchor*self._world(body_path).GetInverse()
            getattr(joint,f'CreateLocalPos{i}Attr')().Set(Gf.Vec3f(frame.ExtractTranslation()))
            getattr(joint,f'CreateLocalRot{i}Attr')().Set(Gf.Quatf(frame.ExtractRotationQuat()))
        joint.CreateCollisionEnabledAttr(False)
        UsdPhysics.FilteredPairsAPI.Apply(self._prim(body)).CreateFilteredPairsRel().AddTarget(tip.GetPath())
        tool.CreateRelationship('lab:attachedTip',custom=True).SetTargets([tip.GetPath()])
        tip.GetAttribute('lab:attached').Set(True)
        tip.CreateRelationship('lab:attachmentJoint',custom=True).SetTargets([joint.GetPath()])
        return self._event('tip_attached',tip=tip_path,separation_m=distance)

    def _tip(self):
        targets=self._prim(self.PIPETTE).GetRelationship('lab:attachedTip').GetTargets()
        if len(targets)!=1:raise ValueError('A physically attached tip is required')
        tip=self._prim(str(targets[0]))
        joints=tip.GetRelationship('lab:attachmentJoint').GetTargets()
        if len(joints)!=1:raise ValueError('Tip attachment constraint is missing')
        joint=self.stage.GetPrimAtPath(joints[0])
        if not joint or not joint.IsActive() or not UsdPhysics.Joint(joint).GetJointEnabledAttr().Get():
            raise ValueError('Tip attachment constraint is inactive')
        return tip

    def eject_tip(self):
        if self.read_joint is None:raise ValueError('Measured ejector position callback is required')
        position=self._number(self.read_joint(self.PIPETTE+'/Joints/pipette_ejector'))
        if position>-0.008:raise ValueError('Ejector has not reached its release stroke')
        tip=self._tip()
        jp=tip.GetRelationship('lab:attachmentJoint').GetTargets()[0]
        # Deactivate in the active edit layer; do not edit a referenced source asset.
        self.stage.OverridePrim(jp).SetActive(False)
        pairs=UsdPhysics.FilteredPairsAPI(self._prim(self.PIPETTE+'/Body')).GetFilteredPairsRel()
        pairs.RemoveTarget(tip.GetPath())
        tip.GetAttribute('lab:attached').Set(False)
        tip.GetRelationship('lab:attachmentJoint').ClearTargets(True)
        self._prim(self.PIPETTE).GetRelationship('lab:attachedTip').ClearTargets(True)
        return self._event('tip_ejected',tip=str(tip.GetPath()),measured_ejector_m=position)

    def _volume(self,prim):
        a=prim.GetAttribute('lab:liquidVolume_uL');c=prim.GetAttribute('lab:capacity_uL')
        if not a or not c:raise ValueError('Asset has no volume model')
        return self._number(a.Get()),self._number(c.Get())

    def _transfer(self,container_path,volume,aspirate):
        volume=self._number(volume)
        if volume<=0:raise ValueError('Transfer volume must be positive')
        tip=self._tip();container=self._prim(container_path)
        nozzle=str(tip.GetPath())+'/Frames/TipEnd'
        # This is a geometric fill-zone check; there is no liquid-contact solver.
        distance=(self._position(nozzle)-self._position(container_path+'/FillTarget')).GetLength()
        if distance>0.01:raise ValueError(f'Nozzle outside fill zone: separation={distance:.6f} m')
        tv,tc=self._volume(tip);cv,cc=self._volume(container)
        new_tv=tv+volume if aspirate else tv-volume
        new_cv=cv-volume if aspirate else cv+volume
        if not (0<=new_tv<=tc and 0<=new_cv<=cc):raise ValueError('Insufficient volume or capacity exceeded')
        before=tv+cv
        tip.GetAttribute('lab:liquidVolume_uL').Set(new_tv)
        container.GetAttribute('lab:liquidVolume_uL').Set(new_cv)
        self._prim(self.PIPETTE).GetAttribute('lab:liquidVolume_uL').Set(new_tv)
        return self._event('aspirate' if aspirate else 'dispense',container=container_path,tip=str(tip.GetPath()),volume_uL=volume,tip_volume_uL=new_tv,container_volume_uL=new_cv,total_before_uL=before,total_after_uL=new_tv+new_cv)

    def aspirate(self,source,volume_uL):return self._transfer(source,volume_uL,True)
    def dispense(self,destination,volume_uL):return self._transfer(destination,volume_uL,False)

    def snapshot(self):
        controls={}
        for name,(joint_name,kind,lo,hi) in self.CONTROLS.items():
            path=self.PIPETTE+'/Joints/'+joint_name
            joint=self.stage.GetPrimAtPath(path)
            if joint:
                controls[name]={'target':UsdPhysics.DriveAPI(joint,kind).GetTargetPositionAttr().Get(),'measured':self.read_joint(path) if self.read_joint else None,'units':'m' if kind=='linear' else 'degrees'}
        vessels={}
        for prim in self.stage.Traverse():
            if prim.GetAttribute('lab:liquidVolume_uL'):
                vessels[str(prim.GetPath())]={'volume_uL':prim.GetAttribute('lab:liquidVolume_uL').Get(),'capacity_uL':prim.GetAttribute('lab:capacity_uL').Get()}
        return {'simulation_time_s':self.sim_time(),'controls':controls,'volumes':vessels,'volume_model':'scalar_only_no_fluid_dynamics'}
