import json, io, sys
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")
ev = json.load(open("outputs/task_suite/eval_reachable_v01_seed0.json", encoding="utf-8"))
td = json.load(open("outputs/task_suite/tasks_v01_seed0.json", encoding="utf-8"))
byid = {t["id"]: t for t in td["tasks"]}
print("汇总", json.dumps(ev["tally"], ensure_ascii=False))
for r in ev["rows"]:
    t = byid[r["id"]]
    subs = []
    def walk(g):
        if g["type"] == "sequence":
            for s in g["steps"]: walk(s)
        else: subs.append(g)
    walk(t["goal"])
    kinds = []
    for s in subs:
        if s["type"] in ("in_slot", "in_volume"):
            kinds.append("%s:%s->%s" % (s["type"], s["object"].split("/")[-1], s.get("target_group") or s.get("volume") or s.get("slot") or "?"))
        else:
            kinds.append(s["type"] + ":" + s["joint"].split("/")[-1] + "=" + str(s.get("expect_deg", s.get("expect_m"))))
    print("%-16s %-13s %-72s %s" % (r["id"], r["verdict"], " | ".join(kinds)[:72],
          json.dumps(r["detail"].get("pos") or (r["detail"].get("first") or {}).get("pos"), ensure_ascii=False)))
