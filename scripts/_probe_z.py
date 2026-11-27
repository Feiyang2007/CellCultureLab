"""一次性探针：把作者写入的 z 与槽位锚点的 z 并排打出来，判断锚点到底是槽底还是上口。"""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _pxr_env  # noqa: E402

_pxr_env.bootstrap()
PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
d = json.load(open(os.path.join(PROJ, "outputs/task_suite/scene_inventory_biolab_main_v07.json"),
                   encoding="utf-8"))

groups = {}
for a in d["slot_anchors"]:
    groups.setdefault(a["group"], []).append(a)
for g in sorted(groups):
    items = groups[g]
    zs = sorted({round(x["pos"][2], 4) for x in items})
    print("%-62s n=%3d z=%s" % (g.replace("/World/TaskArea/", ""), len(items), zs))
    print("     样例锚点 xy=%s  间距参考=%s" % (items[0]["pos"][:2], items[1]["pos"][:2]))

def rng(label, pred):
    rs = [r for r in d["rigid_bodies"] if pred(r["path"])]
    if not rs:
        print("%s: 0 个" % label)
        return
    r0 = rs[0]
    print("%-14s n=%3d  z %.4f ~ %.4f   例 %s  pos=%s size=%s"
          % (label, len(rs), min(x["pos"][2] for x in rs), max(x["pos"][2] for x in rs),
             r0["path"].replace("/World/TaskArea/", ""), r0["pos"], r0["size"]))

rng("50mL管身", lambda p: p.endswith("/TubeWithCap/Tube/Tube"))
rng("50mL管盖", lambda p: p.endswith("/TubeWithCap/Cap/Cap"))
rng("1.5mL管", lambda p: "/CentrifugeTubes/RackPool/Tube_R" in p)
rng("枪头", lambda p: "/Tips/RackPool/Tip_" in p)
rng("台面板", lambda p: "Worktop" in p or "Table" in p)

# 架子本体（不是刚体，是普通 Xform/Mesh）的包围盒只能靠 USD，这里退而求其次：
# 把 Fixtures 下所有 prim 里带 size 的列出来，看架子自己的顶面在哪。
print()
print("== Fixtures 里的独立刚体（架子本体若可仿真会出现在这儿）==")
for r in d["rigid_bodies"]:
    if "/Fixtures/" in r["path"]:
        print("   %-58s pos=%s size=%s" % (r["path"].replace("/World/TaskArea/", ""), r["pos"], r["size"]))

print()
print("== 每支 50mL 管 → 最近的大管位锚点 的 z 差 ==")
for rack, grp in (("TubeRack", "/World/TaskArea/Fixtures/TubeRack/SlotAnchorsLarge"),
                  ("TubeRack_B", "/World/TaskArea/Fixtures/TubeRack_B/SlotAnchorsLarge")):
    large = groups.get(grp, [])
    if not large:
        continue
    tubes = sorted([r for r in d["rigid_bodies"]
                    if r["path"].endswith("/TubeWithCap/Tube/Tube")], key=lambda r: r["path"])
    for t in tubes[:3]:
        best = min(large, key=lambda a: (a["pos"][0] - t["pos"][0]) ** 2
                   + (a["pos"][1] - t["pos"][1]) ** 2)
        dxy = ((best["pos"][0] - t["pos"][0]) ** 2 + (best["pos"][1] - t["pos"][1]) ** 2) ** 0.5
        print("   %-11s %-22s z=%.4f  ← %s z=%.4f  dz=%+.4f dxy=%.4f"
              % (rack, t["path"].split("/")[-3], t["pos"][2], best["path"].split("/")[-1],
                 best["pos"][2], t["pos"][2] - best["pos"][2], dxy))

