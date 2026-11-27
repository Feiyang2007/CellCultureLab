"""逐步隔离 ArmFollower 里哪一步让关节读数变坏。"""
import os
import numpy as np
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})
import omni.usd
import omni.timeline

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ctx = omni.usd.get_context()
assert ctx.open_stage(os.path.join(PROJ, "assets", "scenes", "biolab_main_v10.usda"))
tl = omni.timeline.get_timeline_interface()
tl.play()
for _ in range(30):
    app.update()

from isaacsim.core.prims import Articulation
path = "/World/Robot/Robot1Mount/ur5e"
a = Articulation(path)
a.initialize()


def q6(tag):
    q = np.asarray(a.get_joint_positions(), dtype=np.float64).reshape(-1)
    print("%-24s %s" % (tag, np.round(q[:6], 4).tolist()))


q6("after view")
import omni.physx
physx = omni.physx.get_physx_interface()
def eefpose():
    r = physx.get_rigidbody_transformation(path + "/wrist_3_link")
    if not r:
        return None
    t = r[0]
    return [round(float(t[i]), 3) for i in range(3)]
print("eef", eefpose())

a.set_gains(kps=np.array([[1000.0, 1000.0, 800.0, 400.0, 300.0, 200.0]]),
            kds=np.array([[50.0, 50.0, 40.0, 20.0, 15.0, 10.0]]),
            joint_indices=list(range(6)))
for _ in range(5):
    app.update()
q6("after gains"); print("eef", eefpose())

a.set_max_efforts(np.array([[150.0] * 6]), joint_indices=list(range(6)))
for _ in range(5):
    app.update()
q6("after maxefforts")

q = np.asarray(a.get_joint_positions(), dtype=np.float64).reshape(-1)
a.set_joint_position_targets(q.reshape(1, -1))
for _ in range(5):
    app.update()
q6("after target=q")

q = np.asarray(a.get_joint_positions(), dtype=np.float64).reshape(-1)
qt = q.copy()
qt[0] += 0.02
a.set_joint_position_targets(qt.reshape(1, -1))
for _ in range(20):
    app.update()
q6("after move 0.02")

tl.stop()
app.close()
