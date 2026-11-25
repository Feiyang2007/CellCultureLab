"""Review externally captured PhysX output; never treat completion as clean contacts."""
import argparse
from collections import Counter
import json
from pathlib import Path
import re

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("log", type=Path)
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args()
raw = args.log.read_text(encoding="utf-8", errors="replace")
reports = []
for line in raw.splitlines():
    if line.startswith("REPORT_JSON "):
        reports.append(json.loads(line[len("REPORT_JSON "):]))
if not reports:
    raise SystemExit("No completed report in log; cannot review an interrupted run")
report = reports[-1]
physics_errors = [s for s in raw.splitlines() if "[Error]" in s and re.search(r"omni\.phys(?:x|ics)",s)]
capacity_errors = [s for s in physics_errors if re.search(
    r"aggregatePairsCapacity|buffer overflow|contacts have been dropped|miss interactions",s,re.I)]
patterns = {
    "total_aggregate_pairs": r"totalAggregatePairsCapacity(?: buffers)? to (\d+)",
    "found_lost_aggregate_pairs": r"foundLost(?:Aggregate)?PairsCapacity(?: buffers)? to (\d+)",
    "rigid_patches": r"Patch buffer overflow.*?at least (\d+)",
    "collision_stack_bytes": r"collisionStackSize buffer overflow.*?at least (\d+)",
}
minimums = {key:max([int(v) for v in re.findall(pattern,raw,re.I)] or [0])
            for key,pattern in patterns.items()}
completed = bool(report.get("runtime_completed") and report.get("steps_completed")==report.get("steps_requested"))
log_truncated = bool(re.search(r"Warning: truncated output|tokens truncated",raw,re.I))
review = dict(completed=completed,log=str(args.log.resolve()),log_scope="externally captured physics output",
              log_truncated=log_truncated,
              physics_error_count=len(physics_errors),capacity_error_count=len(capacity_errors),
              capacity_overflow_observed=bool(capacity_errors),reported_minimums=minimums,
              physics_errors=physics_errors,capacity_errors=capacity_errors)
report["external_log_review"] = review
report["capacity_overflow_observed"] = bool(capacity_errors)
report["log_review_required"] = False
report["capacity_run_validated"] = completed and report.get("requested_mode")=="gpu" and not capacity_errors and not log_truncated
report["runtime_validated"] = bool(report["capacity_run_validated"] and not physics_errors
                                 and not report.get("after",{}).get("composition_errors"))
report["full_scene_validated"] = bool(report["runtime_validated"] and
    not report.get("source_layer_metadata",{}).get("validationScope"))
report["status"] = ("runtime_with_capacity_overflow" if capacity_errors else
                    "runtime_with_other_physics_errors" if physics_errors else
                    "incomplete_log" if log_truncated else
                    "runtime_clean_in_tested_scope") if completed else "incomplete_run"
args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding="utf-8")
print(json.dumps({key:report[key] for key in ("status","steps_completed","capacity_run_validated",
                                              "runtime_validated","full_scene_validated")}))
print(json.dumps(minimums))

