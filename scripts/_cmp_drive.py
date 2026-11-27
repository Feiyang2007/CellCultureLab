import json, io, sys
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")
d = json.load(open("outputs/task_suite/scene_inventory_biolab_main_v08.json", encoding="utf-8"))
for st in d["joint_stations"]:
    if "Stations/" not in st["station"]:
        continue
    print(st["station"].replace("/World/TaskArea/", ""))
    for j in st["joints"]:
        dr = j["drive"] or {}
        print("   %-18s bound=%s lower=%-9s upper=%-9s k=%s d=%s F=%s" % (
            j["path"].split("/")[-1], j["bound"], j["lower"], j["upper"],
            dr.get("stiffness"), dr.get("damping"), dr.get("maxForce")))
