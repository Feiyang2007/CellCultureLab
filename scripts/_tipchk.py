import json, io, sys
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")
d = json.load(open("outputs/task_suite/scene_inventory_biolab_main_v07.json", encoding="utf-8"))
for rb in d["rigid_bodies"]:
    p = rb["path"]
    if p.endswith("/Tip_G10") or p.endswith("/Tip_H02") or "Tube_R04_C11" in p or p.endswith("/Cryovial") or p.endswith("/Tube_R02_C02"):
        print("%-62s col=%-5s size=%s" % (p.replace("/World/TaskArea/", "~"), rb["has_collision"], rb["bbox"]["size"] if rb["bbox"] else None))
        print("      pos=%s  bbox.min=%s bbox.max=%s" % (rb["pos"], rb["bbox"]["min"], rb["bbox"]["max"]))
nc = [rb["path"] for rb in d["rigid_bodies"] if not rb["has_collision"]]
print("无碰撞刚体 %d 个：" % len(nc))
for p in nc: print("   ", p.replace("/World/TaskArea/", "~"))
