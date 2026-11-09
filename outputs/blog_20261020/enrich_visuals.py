from pathlib import Path
import sys,json,io
import numpy as np
HERE=Path(__file__).resolve().parent; ROOT=HERE.parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from validate_gpu_capacity import bare_usd
bare_usd()
from pxr import Usd,UsdGeom,Gf
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from PIL import Image
font_manager.fontManager.addfont('C:/Windows/Fonts/msyh.ttc')
plt.rcParams['font.family']=font_manager.FontProperties(fname='C:/Windows/Fonts/msyh.ttc').get_name()
plt.rcParams['axes.unicode_minus']=False
OUT=HERE/'images';OUT.mkdir(exist_ok=True)

def geometry(path):
 st=Usd.Stage.Open(str(path)); cache=UsdGeom.XformCache(); parts=[]
 for p in st.Traverse():
  if '/Visual/' not in str(p.GetPath()) or p.GetTypeName()!='Mesh':continue
  m=UsdGeom.Mesh(p); pts=np.array(m.GetPointsAttr().Get(),dtype=float)
  mat=np.array(cache.GetLocalToWorldTransform(p));pts=(np.c_[pts,np.ones(len(pts))]@mat)[:,:3]
  ids=np.array(m.GetFaceVertexIndicesAttr().Get());cnt=m.GetFaceVertexCountsAttr().Get(); faces=[]; k=0
  for n in cnt:
   for j in range(1,n-1):faces.append([ids[k],ids[k+j],ids[k+j+1]])
   k+=n
  faces=np.array(faces)
  body=str(p.GetPath()).split('/Bodies/')[1].split('/')[0]
  parts.append((body,pts[faces]))
 return st,parts

def draw(ax,parts,title='',angle=0,pivot=None):
 allp=[]
 for body,tri in parts:
  tri=tri.copy()
  if body=='lid' and pivot is not None:
   c,s=np.cos(angle),np.sin(angle);R=np.array([[1,0,0],[0,c,-s],[0,s,c]])
   tri=(tri-pivot)@R.T+pivot
  color='#1ba6ab' if body=='lid' else ('#28506c' if body=='rotor' else '#c7d2dd')
  from matplotlib.colors import to_rgb
  normal=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]);normal/=np.maximum(np.linalg.norm(normal,axis=1,keepdims=True),1e-12)
  light=np.array([-.4,-.6,1.]);light/=np.linalg.norm(light)
  shade=.48+.52*np.maximum(0,normal@light)
  colors=np.clip(np.array(to_rgb(color))[None,:]*shade[:,None],0,1)
  ax.add_collection3d(Poly3DCollection(tri,facecolors=colors,edgecolors='none',alpha=1))
  allp.append(tri.reshape(-1,3))
 pts=np.concatenate(allp);mid=(pts.min(0)+pts.max(0))/2;r=max(np.ptp(pts,axis=0))/2*1.15
 ax.set(xlim=(mid[0]-r,mid[0]+r),ylim=(mid[1]-r,mid[1]+r),zlim=(mid[2]-r,mid[2]+r))
 ax.set_box_aspect((1,1,1));ax.view_init(24,-65);ax.axis('off');ax.set_title(title,fontsize=12,pad=0,color='#17324d')

names=[('centrifuge_eppendorf_5430','Eppendorf 5430','2,004 geom · 2 DOF'),('centrifuge_eppendorf_5910_ri','Eppendorf 5910 Ri','40 geom · 10 DOF'),('centrifuge_tiangen_tgear_mini','TGear mini','16 geom · 2 DOF'),('thermal_cycler_biorad_c1000','Bio-Rad C1000','38 geom · 3 DOF'),('thermal_mixer_eppendorf_c','ThermoMixer C','13 geom · 0 DOF'),('vortex_mixer_genie_2','Vortex-Genie 2','69 geom · 4 DOF')]
fig=plt.figure(figsize=(13,8))
for i,(stem,title,sub) in enumerate(names):
 st,parts=geometry(ROOT/'assets/instruments5'/f'{stem}.usda')
 ax=fig.add_subplot(2,3,i+1,projection='3d');draw(ax,parts,title+'\n'+sub)
fig.suptitle('六台仪器：从已构建 USD 读取的实际视觉几何',fontsize=20,fontweight='bold',color='#17324d')
fig.text(.5,.025,'离线几何展示；统一配色用于辨识部件，不代表原始材质渲染。geom 数量来自源模型核对报告。',ha='center',fontsize=10,color='#526474')
fig.tight_layout(rect=(0,.05,1,.94));fig.savefig(OUT/'04_instrument_atlas.png',dpi=150);plt.close(fig)
print('atlas completed',flush=True)

st,parts=geometry(ROOT/'assets/instruments6_pilot/centrifuge_tiangen_tgear_mini.usda')
j=st.GetPrimAtPath('/Instrument/Joints/body_lid');b=st.GetPrimAtPath('/Instrument/Bodies/body')
pivot=np.array(UsdGeom.XformCache().GetLocalToWorldTransform(b).Transform(j.GetAttribute('physics:localPos0').Get()))
def frame(index):
 q=j.GetAttribute('physics:localRot'+str(index)).Get();p=j.GetAttribute('physics:localPos'+str(index)).Get()
 m=Gf.Matrix4d(1);m.SetRotate(Gf.Quatd(q));m.SetTranslateOnly(Gf.Vec3d(p));return m
cache=UsdGeom.XformCache();old=cache.GetLocalToWorldTransform(st.GetPrimAtPath('/Instrument/Bodies/lid'))
target=frame(1).GetInverse()*frame(0)*cache.GetLocalToWorldTransform(b)
delta=np.array(old.GetInverse()*target)
parts=[(body,(np.c_[tri.reshape(-1,3),np.ones(tri.size//3)]@delta)[:,:3].reshape(tri.shape) if body=='lid' else tri) for body,tri in parts]
rows=[json.loads(s) for s in (ROOT/'outputs/runtime_validation/mini_lid_slow_real_cpu_rev2.steps.jsonl').read_text(encoding='utf8').splitlines()][1:]
frames=[]
for index in np.linspace(0,len(rows)-1,45).astype(int):
 row=rows[index];fig=plt.figure(figsize=(8,4.8));ax=fig.add_subplot(111,projection='3d')
 draw(ax,parts,angle=row['q_post'][0],pivot=pivot)
 ax.set(xlim=(-.13,.13),ylim=(-.13,.13),zlim=(-.025,.235))
 fig.suptitle('mini 低速开合｜源几何＋已记录角度的离线回放',fontsize=15,color='#17324d')
 fig.text(.5,.07,f"t = {row['t_sim_s']:.2f} s   q = {row['q_post'][0]:.3f} rad   v = {row['v_post'][0]:+.3f} rad/s",ha='center',fontsize=12)
 fig.text(.5,.025,'非 Isaac GUI 录屏；动作数值来自 1,140 步实测记录，配色仅用于部件区分。',ha='center',fontsize=9,color='#526474')
 buf=io.BytesIO();fig.savefig(buf,format='png',dpi=100);plt.close(fig);buf.seek(0);frames.append(Image.open(buf).convert('RGB'))
frames[0].save(OUT/'05_mini_recorded_motion.gif',save_all=True,append_images=frames[1:],duration=110,loop=0)
print('gif completed',flush=True)

fig,ax=plt.subplots(figsize=(12,5.8));fig.patch.set_facecolor('#101e32');ax.set_facecolor('#101e32');ax.set(xlim=(0,12),ylim=(0,6));ax.axis('off')
for x in np.arange(0,12,.4):ax.plot([x,x],[0,6],color='#23364d',lw=.4)
for y in np.arange(0,6,.4):ax.plot([0,12],[y,y],color='#23364d',lw=.4)
ax.text(.5,5.35,'实验室机器人 · 仿真环境建设记录',color='#65ddd0',fontsize=16)
ax.text(.5,4.25,'基于师兄的实验室场景',color='white',fontsize=29,weight='bold')
ax.text(.5,3.25,'补齐设备结构、状态测量与单件控制验证',color='white',fontsize=23,weight='bold')
for x,n,label in [( .5,'6 台','独立仪器资产'),(3.5,'21 个','可动关节验证'),(6.5,'1,140 步','连续低速开合'),(9.5,'3,032 步','锁定功能流程')]:
 ax.text(x,1.65,n,color='#65ddd0',fontsize=27,weight='bold');ax.text(x,1.05,label,color='#becfdf',fontsize=13)
ax.text(.5,.35,'Isaac Sim 5.1  /  MuJoCo 3.3.0  /  USD  /  2026.10',color='#8fa9c1',fontsize=12)
fig.savefig(OUT/'00_cover.png',dpi=160,bbox_inches='tight',facecolor=fig.get_facecolor());plt.close(fig)

q=np.linspace(0,1.712,300);tau=.06906073367802575*np.sin(q)-.045642666693394375*np.cos(q)-4.0554398253638074e-5
fig,ax=plt.subplots(figsize=(10,4.5));ax.plot(q,tau*1000,color='#087f8c',lw=2.5);ax.axhline(0,color='#aaa',lw=1)
ax.axvline(.5844938765,color='#3478b8',ls='--',label='平衡角 0.58449 rad ≈ 33.49°')
ax.scatter([0,1.712],[tau[0]*1000,tau[-1]*1000],color='#b26331',zorder=4)
ax.set(xlabel='盖子角度 q（rad）',ylabel='重力矩（mN·m）',title='自由行程测量拟合：控制器的重力前馈依据');ax.legend();ax.grid(alpha=.15)
fig.text(.12,.015,'曲线为已保存拟合参数的重建；17 个自由行程点，残差约 3.82e-6 N·m。',fontsize=9,color='#526474')
fig.tight_layout(rect=(0,.04,1,1));fig.savefig(OUT/'06_gravity_curve.png',dpi=170);plt.close(fig)
print('all visuals completed',flush=True)
