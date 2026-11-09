"""Build a current-asset acceptance manifest from explicitly supplied runtime reviews."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import mjcf_instrument_to_usd_v5
from pxr import Usd, UsdPhysics

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("reviews", nargs="+",type=Path)
parser.add_argument("--output",type=Path,required=True)
args = parser.parse_args()
stage = Usd.Stage.Open(str(ROOT / "assets/scenes/test_instruments5.usda"))
expected = {str(p.GetPath()) for p in stage.Traverse()
            if p.IsA(UsdPhysics.RevoluteJoint) or p.IsA(UsdPhysics.PrismaticJoint)}
evidence = {}
for review_path in args.reviews:
    review = json.loads(review_path.read_text(encoding="utf-8"))
    raw_path = Path(review["source_report"])
    raw = json.loads(raw_path.read_text(encoding="utf-8"))
    if not (raw.get("complete") and raw.get("physics_device")=="CPU"
            and raw.get("contacts") is False and raw.get("gravity")=="zero"
            and raw.get("zero_velocities_after_settle")):
        raise SystemExit("Incompatible/incomplete experiment: "+str(raw_path))
    for row in review["rows"]:
        relevant = {path:checksum for path,checksum in raw.get("source_layers_sha256",{}).items()
                    if "/"+Path(path).stem+"/" in row["joint"]}
        current = bool(relevant) and all(Path(path).is_file() and
            hashlib.sha256(Path(path).read_bytes()).hexdigest()==checksum
            for path,checksum in relevant.items())
        if not current:
            continue
        # Later supplied current-asset evidence replaces earlier evidence,
        # including failures. Never backfill a failed latest test with a pass.
        evidence[row["joint"]] = dict(row,source_report=str(raw_path),
            review=str(review_path.resolve()),asset_hashes=relevant)
missing = sorted(expected-evidence.keys())
failed = [path for path,row in evidence.items() if path in expected and not row["passed"]]
report = dict(passed=bool(expected) and not missing and not failed,
    expected_joints=len(expected),verified_joints=sum(v["passed"] for k,v in evidence.items() if k in expected),
    missing=missing,failed=failed,scope="CPU; contacts disabled; zero gravity; injected test drives; explicit zero initial velocities",
    constraints="baseline/end positions within 0.1 mm; forbidden angular displacement within 1 degree",
    evidence={key:evidence[key] for key in sorted(evidence) if key in expected},
    task_contact_validation=False,mujoco_dynamic_equivalence=False,
    notes=["Per-asset evidence from separate runs; vortex uses a distinct documented drive/solver profile.",
           "Auxiliary vortex link preserves serial topology and approximates pivot armature; exact source inertia/COM is not available.",
           "No robot interaction, tube insertion, detent or locking-state acceptance is implied."])
args.output.write_text(json.dumps(report,indent=2,ensure_ascii=False),encoding="utf-8")
print(json.dumps({key:report[key] for key in ("passed","expected_joints","verified_joints","missing","failed")}))
raise SystemExit(0 if report["passed"] else 1)

