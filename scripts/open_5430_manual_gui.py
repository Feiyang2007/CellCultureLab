"""Open the isolated contact fixture for manual GUI inspection, without autoplay.

Camera/light changes are session-only. No automatic force commands or baseline
file writes; normal GUI Play/Stop and editing remain under the user's control.
"""
import os
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
home=Path.home()/'AppData/LocalLow/CellCultureLabReference/manual_gui';home.mkdir(parents=True,exist_ok=True)
os.environ['TEMP']=os.environ['TMP']=str(home)
from isaacsim import SimulationApp
app=SimulationApp({'headless':False,'width':1280,'height':720,'window_width':1440,'window_height':900,
    'multi_gpu':False,'extra_args':['--portable-root',str(home),'--/app/settings/persistent=false']})
import omni.usd
from pxr import UsdLux,Usd
from isaacsim.core.utils.viewports import set_camera_view
ctx=omni.usd.get_context()
scene=ROOT/'outputs/runtime_validation/5430_single_slot/scene.usda'
if not ctx.open_stage(str(scene)):raise RuntimeError('Cannot open manual fixture')
stage=ctx.get_stage();stage.SetEditTarget(stage.GetSessionLayer())
UsdLux.DomeLight.Define(stage,'/ManualReviewLight').CreateIntensityAttr(1000)
for _ in range(5):app.update()
set_camera_view(eye=[.45,-.55,.48],target=[0,.03,.15])
print('MANUAL_GUI_READY '+str(scene),flush=True)
while app.is_running():app.update()
app.close()
