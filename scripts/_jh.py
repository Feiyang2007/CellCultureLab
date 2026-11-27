import json, io, sys
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")
ev = json.load(open("outputs/task_suite/eval_reachable_longhorizon_v08_seed0.json", encoding="utf-8"))
for r in ev["rows"]:
    if r["suite"] != "long_horizon": continue
    print("==", r["id"])
    for s in (r["detail"].get("steps") or []):
        if s.get("type") == "joint_angle" or "angle_deg" in s:
            print("   joint %-52s ang=%-8s expect=%-8s hit=%-5s ever=%-5s ok=%s" % (
                s.get("lid","?").replace("/World/TaskArea/Stations/","~"), s.get("angle_deg"),
                s.get("expect_deg"), s.get("terminal_hit"), s.get("ever_seen"), s.get("ok")))
        elif "angle_deg" not in s and "pos" in s:
            print("   obj   pos=%s dxy=%s dz=%s ok=%s" % (s.get("pos"), s.get("dxy"), s.get("dz"), s.get("ok")))
        else:
            print("   ?", json.dumps(s, ensure_ascii=False)[:120])
