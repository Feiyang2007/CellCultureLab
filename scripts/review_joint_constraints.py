"""Independently inspect endpoint constraints and source hashes in a runtime report."""
import argparse
import hashlib
import json
import math
from pathlib import Path
from runtime_joint_math import quat_mul, quat_normalize, quat_rotate

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("report", type=Path)
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args()
source = json.loads(args.report.read_text(encoding="utf-8"))
identity = [0,0,0,0,0,0,1]
rows = []
for row in source["rows"]:
    d = row["joint"]
    unit = [0.,0.,0.]
    unit["XYZ".index(d["axis"])] = 1.
    maximum_position = maximum_angle = 0.
    for trial in row.get("trials",{}).values():
        for key in ("baseline_poses","final_poses"):
            poses = trial[key]
            p0 = poses[d["body0"]] if d["body0"] else identity
            p1 = poses[d["body1"]] if d["body1"] else identity
            f0 = quat_normalize(quat_mul(p0[3:],d["local_rot0"]))
            f1 = quat_normalize(quat_mul(p1[3:],d["local_rot1"]))
            off0,off1 = quat_rotate(p0[3:],d["local_pos0"]),quat_rotate(p1[3:],d["local_pos1"])
            gap = [p1[i]+off1[i]-p0[i]-off0[i] for i in range(3)]
            axis0,axis1 = quat_rotate(f0,unit),quat_rotate(f1,unit)
            if d["kind"]=="RevoluteJoint":
                position = math.sqrt(sum(v*v for v in gap))
                cosine = max(-1.,min(1.,sum(a*b for a,b in zip(axis0,axis1))))
                angle = math.degrees(math.acos(cosine))
            else:
                projection = sum(a*b for a,b in zip(gap,axis0))
                position = math.sqrt(sum((gap[i]-projection*axis0[i])**2 for i in range(3)))
                cosine = min(1.,abs(sum(a*b for a,b in zip(f0,f1))))
                angle = math.degrees(2*math.acos(cosine))
            maximum_position = max(maximum_position,position)
            maximum_angle = max(maximum_angle,angle)
    matching = []
    for path,checksum in source.get("source_layers_sha256",{}).items():
        name = Path(path).stem
        if "/"+name+"/" in d["path"]:
            matching.append(Path(path).is_file() and hashlib.sha256(Path(path).read_bytes()).hexdigest()==checksum)
    constraint_ok = bool(row.get("trials")) and maximum_position <= .0001 and maximum_angle <= 1.
    passed = row["status"]=="DIRECTION_CONFIRMED" and constraint_ok and bool(matching) and all(matching)
    rows.append(dict(joint=d["path"],direction_status=row["status"],source_asset_matches=bool(matching) and all(matching),
                     maximum_constraint_position_error_mm=maximum_position*1000,
                     maximum_constraint_angle_error_deg=maximum_angle,constraints_passed=constraint_ok,passed=passed))
review = dict(source_report=str(args.report.resolve()),joint_count=len(rows),rows=rows,
              tolerance_position_mm=.1,tolerance_angle_deg=1.,
              verified_count=sum(r["passed"] for r in rows),
              interpretation="Endpoint constraints and current target-asset hashes only; not trajectory-wide or contact/task validation.")
args.output.write_text(json.dumps(review,indent=2),encoding="utf-8")
print(json.dumps({k:v for k,v in review.items() if k!="rows"}))
for row in rows:
    if not row["passed"]:
        print(json.dumps(row))

