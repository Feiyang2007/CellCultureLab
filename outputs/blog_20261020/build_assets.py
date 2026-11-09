"""Build publication figures from saved results and an offline reading copy."""
from pathlib import Path
import json,hashlib,html,re,base64,zipfile
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import FancyBboxPatch,FancyArrowPatch
import numpy as np
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1];DATA=ROOT/'outputs/runtime_validation';IMAGES=HERE/'images';IMAGES.mkdir(exist_ok=True)
font_manager.fontManager.addfont('C:/Windows/Fonts/msyh.ttc')
plt.rcParams.update({'font.family':font_manager.FontProperties(fname='C:/Windows/Fonts/msyh.ttc').get_name(),'font.size':11,'axes.unicode_minus':False,'figure.facecolor':'white','axes.spines.top':False,'axes.spines.right':False,'savefig.facecolor':'white'})
navy='#17324d';teal='#087f8c';blue='#3478b8';grey='#526474'

fig,ax=plt.subplots(figsize=(12,5.8));ax.set(xlim=(0,12),ylim=(0,6));ax.axis('off')
def box(x,y,w,h,title,body,fill='#eff6f9',edge='#c8dce6'):
 ax.add_patch(FancyBboxPatch((x,y),w,h,boxstyle='round,pad=0.12,rounding_size=0.12',facecolor=fill,edgecolor=edge,lw=1.2))
 ax.text(x+w/2,y+h*.69,title,ha='center',va='center',color=navy,size=14,weight='bold')
 ax.text(x+w/2,y+h*.30,body,ha='center',va='center',color=grey,size=11,linespacing=1.5)
ax.text(.05,5.8,'基于已有场景，补齐设备交互的测量与验证',size=19,weight='bold',color=navy)
box(.2,3.7,5.25,1.25,'师兄提供的场景基础','实验室空间 · 仪器模型 · 双机械臂布局')
box(6.5,3.7,5.25,1.25,'本阶段补充的参考依据','源模型参数 · MuJoCo 3.3.0 · 独立核对')
for x in (2.8,9.1):ax.add_patch(FancyArrowPatch((x,3.5),(x,2.9),arrowstyle='-|>',mutation_scale=17,color=teal,lw=2))
cards=[('资产结构','6 台仪器\n2,180 个几何元素'),('运行时测量','21 个可动关节\n隔离方向与端点验证'),('mini 控制样例','8 组参考协议\n1,140 步低速开合'),('C1000 锁定样例','3,032 步连续流程\n9 项功能判据')]
for i,(title,body) in enumerate(cards):box(.2+i*3,1.1,2.6,1.55,title,body,fill='#eaf5f3',edge='#bdddd6')
ax.text(6,.35,'场景展示、独立资产和单件试验分别保存，并保留输入、版本与结果记录',ha='center',color=grey,size=11)
fig.tight_layout();fig.savefig(IMAGES/'01_workflow.png',dpi=180,bbox_inches='tight');plt.close(fig)

comparison=json.loads((DATA/'mini_articulation_330_comparison.json').read_text(encoding='utf8'))
labels=['盖子 / 负力矩','盖子 / 零输入','盖子 / 正力矩','转子 / 负力矩','转子 / 零输入','转子 / 正力矩','盖子 / 重力','转子 / armature']
errors=[r['max_angle_error_deg'] for r in comparison['rows']]
assert len(errors)==8 and all(r['passed'] for r in comparison['rows'])
fig,ax=plt.subplots(figsize=(10.8,5.4));y=np.arange(8)
ax.barh(y,errors,color=[teal if r['joint']=='lid' else blue for r in comparison['rows']],height=.58)
ax.set_yticks(y,labels);ax.invert_yaxis();ax.set_xlim(0,max(errors)*1.38)
for i,e in enumerate(errors):ax.text(e+max(errors)*.025,i,f'{e:.3g}°',va='center',size=10,color=navy)
ax.set_xlabel('指定协议报告记录的最大角差（°）');ax.set_title('mini 与 MuJoCo 3.3.0 的八组参考对照',loc='left',weight='bold',pad=18,color=navy)
ax.ticklabel_format(axis='x',style='sci',scilimits=(-4,-4));ax.grid(axis='x',alpha=.18);ax.set_axisbelow(True)
fig.text(.13,.015,'单件指定协议；物理步频 240 Hz；结果来自保存的对照报告。',color=grey,size=10)
fig.tight_layout(rect=(0,.035,1,1));fig.savefig(IMAGES/'02_reference_comparison.png',dpi=190,bbox_inches='tight');plt.close(fig)

slow=json.loads((DATA/'mini_lid_slow_real_cpu_rev2.json').read_text(encoding='utf8'))
trace=[json.loads(x) for x in (DATA/'mini_lid_slow_real_cpu_rev2.steps.jsonl').read_text(encoding='utf8').splitlines()]
header=trace.pop(0);assert header['kind']=='header' and len(trace)==1140
assert [r['step'] for r in trace]==list(range(1140))
assert all(trace[i]['q_post']==trace[i+1]['q_pre'] and trace[i]['v_post']==trace[i+1]['v_pre'] for i in range(len(trace)-1))
t=np.array([r['t_sim_s'] for r in trace]);q=np.array([r['q_post'][0] for r in trace]);v=np.array([r['v_post'][0] for r in trace]);tau=np.array([r['torque_Nm'][0] for r in trace])
phase_names={'hold_at_start':'起点保持','open_slow':'低速打开','hold_open':'打开保持','close_slow':'低速关闭','hold_closed':'终点保持'}
groups=[]
for phase in phase_names:
 indices=[i for i,r in enumerate(trace) if r['segment']==phase]
 if indices:groups.append((phase,indices[0],indices[-1]))
fig,axes=plt.subplots(3,1,figsize=(11.2,8.1),sharex=True)
fig.suptitle('mini 盖子连续低速开合：1,140 步实测轨迹',x=.10,ha='left',weight='bold',color=navy,size=17)
for i,ax in enumerate(axes):
 for j,(phase,start,end) in enumerate(groups):
  a=t[start]-header['dt'];b=t[end]
  ax.axvspan(a,b,color=teal if j%2==0 else blue,alpha=.06 if j%2==0 else .10)
  if start:ax.axvline(a,color='#95a8b4',lw=.7,ls=':')
  if i==0:ax.text((a+b)/2,1.02,phase_names[phase],transform=ax.get_xaxis_transform(),ha='center',va='bottom',size=9.5,color=grey)
 ax.grid(alpha=.18);ax.set_axisbelow(True)
axes[0].plot(t,q,color=teal,lw=1.9);axes[0].set_ylabel('角度 q（rad）');axes[0].set_ylim(0,1.8)
axes[1].plot(t,v,color=blue,lw=1.8);axes[1].set_ylabel('角速度（rad/s）');axes[1].set_ylim(-1.25,1.25)
axes[2].plot(t,tau,color='#b26331',lw=1.6);axes[2].set_ylabel('输入力矩（N·m）');axes[2].set_xlabel('仿真时间（s）')
axes[2].set_xlim(0,t[-1])
fig.text(.10,.025,f'240 Hz · 峰值角速度 {max(abs(v)):.5f} rad/s · 24/24 项判据通过 · 阶段间无复位',color=grey,size=10.5)
fig.tight_layout(rect=(0,.05,1,.94),h_pad=1.8);fig.savefig(IMAGES/'03_slow_lid_trajectory.png',dpi=190,bbox_inches='tight');plt.close(fig)

# Portable reading copy: embeds figures, no external scripts/fonts or network dependency.
def inline(s):
 s=html.escape(s)
 s=re.sub(r'`([^`]+)`',r'<code>\1</code>',s)
 s=re.sub(r'\*\*([^*]+)\*\*',r'<strong>\1</strong>',s)
 s=re.sub(r'\[([^\]]+)\]\((https?://[^)]+)\)',r'<a href="\2">\1</a>',s)
 return s
def md_html(text):
 lines=text.splitlines();out=[];i=0
 while i<len(lines):
  line=lines[i]
  if not line.strip():i+=1;continue
  if line.startswith('```'):
   i+=1;buf=[]
   while i<len(lines) and not lines[i].startswith('```'):buf.append(lines[i]);i+=1
   out.append('<pre>'+html.escape('\n'.join(buf))+'</pre>');i+=1;continue
  m=re.match(r'^(#{1,6}) (.*)',line)
  if m:
   level=len(m[1]);out.append(f'<h{level}>{inline(m[2])}</h{level}>');i+=1;continue
  m=re.match(r'^!\[([^\]]*)\]\(([^)]+)\)$',line)
  if m:
   data=base64.b64encode((HERE/m[2]).read_bytes()).decode('ascii')
   mime='image/gif' if m[2].endswith('.gif') else 'image/png'
   out.append('<figure><img alt="'+html.escape(m[1])+'" src="data:'+mime+';base64,'+data+'"><figcaption>'+html.escape(m[1])+'</figcaption></figure>');i+=1;continue
  if line.startswith('|'):
   rows=[]
   while i<len(lines) and lines[i].startswith('|'):
    cells=[x.strip() for x in lines[i].strip('|').split('|')]
    if not all(re.fullmatch(r':?-+:?',x) for x in cells):rows.append(cells)
    i+=1
   out.append('<div class="table"><table>')
   for j,row in enumerate(rows):
    tag='th' if j==0 else 'td';out.append('<tr>'+''.join(f'<{tag}>{inline(c)}</{tag}>' for c in row)+'</tr>')
   out.append('</table></div>');continue
  if line=='---':out.append('<hr>');i+=1;continue
  if line.startswith('- ') or re.match(r'^\d+\. ',line):
   ordered=bool(re.match(r'^\d+\. ',line));tag='ol' if ordered else 'ul';out.append('<'+tag+'>')
   while i<len(lines) and ((bool(re.match(r'^\d+\. ',lines[i])) if ordered else lines[i].startswith('- '))):
    item=re.sub(r'^\d+\. ','',lines[i]) if ordered else lines[i][2:];out.append('<li>'+inline(item)+'</li>');i+=1
   out.append('</'+tag+'>');continue
  if line.startswith('> '):out.append('<blockquote>'+inline(line[2:])+'</blockquote>');i+=1;continue
  out.append('<p>'+inline(line)+'</p>');i+=1
 return '\n'.join(out)
body=md_html((HERE/'blog.md').read_text(encoding='utf8'))
css='''body{margin:0;background:#edf2f6;color:#25394b;font-family:"Microsoft YaHei",sans-serif;line-height:1.95}main{max-width:1000px;margin:40px auto;padding:50px 64px;background:white;border-top:7px solid #087f8c;box-shadow:0 8px 40px #17324d14}h1{font-size:31px;line-height:1.55;color:#17324d}h2{margin-top:44px;font-size:24px;color:#17324d;border-bottom:1px solid #dce6eb;padding-bottom:12px}h3{font-size:20px;margin-top:30px}p,li{font-size:16px}strong{color:#096d78}code{background:#eef3f6;padding:2px 5px;border-radius:4px;font-size:.90em}pre{white-space:pre-wrap;line-height:1.7;background:#eff5f8;padding:20px;border-left:4px solid #087f8c;font-size:15px}table{width:100%;border-collapse:collapse;font-size:14px;line-height:1.7}th{text-align:left;background:#eaf3f5;color:#17324d}td,th{padding:12px 14px;border-bottom:1px solid #dce6eb}tr:nth-child(even){background:#f8fafc}.table{overflow:auto}figure{margin:30px 0}img{width:100%;height:auto}figcaption{font-size:13px;color:#617789;text-align:center}a{color:#16748c}hr{border:0;border-top:1px solid #dce6eb;margin:38px 0}@media print{body{background:white}main{margin:0;box-shadow:none;padding:16px;border:0}h2{break-after:avoid}figure,table{break-inside:avoid}}@media(max-width:700px){main{margin:0;padding:25px 20px}h1{font-size:25px}}'''
css+='blockquote{margin:25px 0;padding:18px 24px;background:#f0f3f6;border-left:5px solid #8096a8;color:#526474;font-size:15px}pre{background:#142238;color:#e1ecf5;border-left:4px solid #36b9aa;font-family:Consolas,"Microsoft YaHei",monospace}h1{font-size:32px}figure{margin:38px 0}main{max-width:1000px}h3{border-left:4px solid #087f8c;padding-left:12px}figcaption{line-height:1.6;margin-top:10px}'
(HERE/'博客_离线阅读版.html').write_text('<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>实验室机器人仿真建设笔记 · 图文修订版</title><style>'+css+'</style><main>'+body+'</main></html>',encoding='utf8')
sources=['v5_independent.json','v5_runtime_acceptance_20261019.json','mini_articulation_330_comparison.json','mini_lid_slow_real_cpu_rev2.json','mini_lid_slow_real_cpu_rev2.steps.jsonl','c1000_isaac_lock_pilot/reviewed_summary.json','c1000_isaac_lock_pilot/cycle_physics_only.json']
evidence={'date':'2026-10-20','figure_data':'Only existing saved measurements; no simulation rerun','reference_blog':'https://blog.csdn.net/2301_77297031/article/details/167224240','sources':[{'path':'outputs/runtime_validation/'+n,'sha256':hashlib.sha256((DATA/n).read_bytes()).hexdigest()} for n in sources],'checks':{'trace_steps':len(trace),'step_continuity_verified':True,'mini_reference_cases':8,'mini_reference_max_angle_error_deg':max(errors),'slow_peak_v_from_full_trace':float(max(abs(v))),'slow_max_tracking_error_from_segment_reports':max(s['max_tracking_error_rad'] for s in slow['segments'].values()),'font_test_not_a_blog_result':True}}
(HERE/'结果来源.json').write_text(json.dumps(evidence,ensure_ascii=False,indent=2),encoding='utf8')
with zipfile.ZipFile(HERE/'博客与组会材料.zip','w',zipfile.ZIP_DEFLATED) as z:
 for p in [HERE/'blog.md',HERE/'组会讲稿_约3分钟.md',HERE/'博客_离线阅读版.html',HERE/'结果来源.json',*sorted(IMAGES.glob('*.png')),*sorted(IMAGES.glob('*.gif'))]:z.write(p,p.relative_to(HERE))
print(json.dumps({'output':str(HERE),'figures':len(list(IMAGES.glob('*.png'))),'trace_steps':len(trace),'blog_characters':len((HERE/'blog.md').read_text(encoding='utf8')),'source_max_angle_error_deg':max(errors)},ensure_ascii=False))
