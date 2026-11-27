import json, io, sys
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")
d = json.load(open("outputs/task_suite/scene_inventory_biolab_main_v07.json", encoding="utf-8"))
print("== 大面积静态碰撞体（可能可站设备的平面）==")
for c in sorted(d["static_colliders"], key=lambda c: -(c["bbox"]["size"][0]*c["bbox"]["size"][1] if c.get("bbox") else 0))[:16]:
    b = c["bbox"]
    if not b: continue
    print("  %-58s 占地 %8.1f×%8.1f mm  顶z=%7.3f 底z=%7.3f  xy=[%.3f..%.3f, %.3f..%.3f]" % (
        c["path"].replace("/World/", ""), 1000*b["size"][0], 1000*b["size"][1],
        b["max"][2], b["min"][2], b["min"][0], b["max"][0], b["min"][1], b["max"][1]))
print("\n== 现有工位/设备刚体的水平占地 ==")
rows = []
for rb in d["rigid_bodies"]:
    if not rb.get("bbox"): continue
    p = rb["path"]
    if "/Stations/" in p or "/Equipment/" in p or "/Fixtures/" in p or "/CultureMedia/" in p:
        b = rb["bbox"]
        rows.append((b["min"][0], b["min"][1], p, b))
for x, y, p, b in sorted(rows)[:60]:
    print("  x=%7.3f y=%7.3f .. x=%7.3f y=%7.3f  z顶=%6.3f  %s" % (
        b["min"][0], b["min"][1], b["max"][0], b["max"][1], b["max"][2], p.replace("/World/TaskArea/", "~")))
