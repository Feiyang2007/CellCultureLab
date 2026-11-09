"""Run only the project physics validator under an independent resource monitor.

The parent can stop a native step that has not returned to Python. It terminates
only the process tree it created, with psutil's process-identity checks. A complete
child report still requires external physics-log review; exit code is not acceptance.
"""
import argparse
from collections import deque
import json
from pathlib import Path
import queue
import re
import subprocess
import sys
import threading
import time
import psutil
from runtime_resources import snapshot

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout",type=float,default=180)
    parser.add_argument("--max-commit-fraction",type=float,default=.94)
    parser.add_argument("--max-vram-fraction",type=float,default=.95)
    parser.add_argument("--sample-period",type=float,default=2)
    parser.add_argument("--runner",choices=("validate_gpu_capacity.py","check_c1000_isaac_lock.py","check_5430_contact_pilot.py","run_single_contact_calibration.py"),default="validate_gpu_capacity.py")
    parser.add_argument("validator_args",nargs=argparse.REMAINDER)
    args=parser.parse_args()
    if not 0<args.max_commit_fraction<=1 or not 0<args.max_vram_fraction<=1 or args.timeout<=0 or args.sample_period<=0:
        parser.error("Invalid resource limits")
    forwarded=args.validator_args
    if forwarded and forwarded[0]=="--":
        forwarded=forwarded[1:]
    if not forwarded:
        parser.error("Provide validator arguments after --")
    command=[sys.executable,"-u","-B",str(ROOT/"scripts"/args.runner),*forwarded]
    if args.runner=="validate_gpu_capacity.py" and "--stdout-only" not in command:
        command.append("--stdout-only")
    report=dict(child_started=False,resource_guard_triggered=False,physics_error_observed=False,physics_errors=[],samples=[],
        physics_acceptance_requires_external_review=True,command=command)

    def reason(sample):
        if sample.get("system_commit_limit_bytes",0) and sample["system_commit_bytes"]/sample["system_commit_limit_bytes"]>=args.max_commit_fraction:
            return "system_commit_limit"
        if any(g["used_mib"]/g["total_mib"]>=args.max_vram_fraction for g in sample.get("gpu",[]) if g["total_mib"]):
            return "gpu_memory_limit"
        return None

    before=snapshot("monitor_preflight")
    report["samples"].append(before)
    blocked=reason(before)
    if blocked:
        report.update(resource_guard_triggered=True,stop_reason=blocked)
        print("MONITOR_REPORT_JSON "+json.dumps(report),flush=True)
        return 2
    process=subprocess.Popen(command,cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,
        text=True,encoding="utf-8",errors="replace",creationflags=getattr(subprocess,"CREATE_NO_WINDOW",0))
    root=psutil.Process(process.pid)
    root.create_time()
    owned={root.pid:root}
    report["child_started"]=True
    report["root_pid"]=process.pid
    pending=queue.Queue()
    def reader():
        for line in process.stdout:
            pending.put(line.rstrip("\r\n"))
        pending.put(None)
    thread=threading.Thread(target=reader,daemon=True)
    thread.start()
    started=time.monotonic()
    next_sample=started
    received=None
    eof=False
    child_report=None
    stopped=False
    diagnostic_tail=deque(maxlen=60)
    def stop_owned():
        nonlocal stopped
        targets=[p for p in owned.values() if p.is_running()]
        for p in reversed(targets):
            try:p.terminate()
            except psutil.Error:pass
        _,alive=psutil.wait_procs(targets,timeout=2)
        for p in alive:
            try:p.kill()
            except psutil.Error:pass
        stopped=True
    try:
        while True:
            try:line=pending.get(timeout=.1)
            except queue.Empty:line=""
            if line is None:
                eof=True
            elif line:
                if not line.startswith("REPORT_JSON "):
                    diagnostic_tail.append(line[:2000])
                if re.search(r"^(RUNTIME_PHASE|PHYSICS_STEPS|REPORT_JSON|CAPACITY_REPORT|LATCH_PHASE)|\[omni\.physx|PhysX error|OUT_OF_MEMORY|bad allocation|^usage:|error:|Traceback",line,re.I):
                    print(line,flush=True)
                if line.startswith("REPORT_JSON "):
                    child_report=json.loads(line[12:])
                    received=time.monotonic()
                if "[Error]" in line and re.search(r"omni\.phys(?:x|ics)",line):
                    report["physics_error_observed"]=True
                    report["physics_errors"].append(line)
                    report["stop_reason"]="native_physics_error"
                    stop_owned()
                    break
            now=time.monotonic()
            try:
                descendants=root.children(recursive=True) if root.is_running() else []
                for child in descendants:
                    child.create_time()
                    owned[child.pid]=child
            except psutil.Error:pass
            if now>=next_sample and received is None:
                sample=snapshot("monitor_running")
                sample["owned_private_bytes"]=0
                for p in owned.values():
                    try:sample["owned_private_bytes"]+=getattr(p.memory_info(),"private",0)
                    except psutil.Error:pass
                report["samples"].append(sample)
                blocked=reason(sample)
                if blocked:
                    report.update(resource_guard_triggered=True,stop_reason=blocked)
                    stop_owned()
                    break
                next_sample=now+args.sample_period
            if received is not None and now-received>5:
                report["shutdown_cleanup_after_report"]=True
                stop_owned()
                break
            if process.poll() is not None and eof and pending.empty():break
            if now-started>args.timeout:
                report["stop_reason"]="wall_time_limit"
                stop_owned()
                break
    finally:
        if any(p.is_running() for p in owned.values()):stop_owned()
        report.update(child_report_received=child_report is not None,
            child_runtime_completed=bool(child_report and child_report.get("runtime_completed")),
            elapsed_wall_seconds=time.monotonic()-started,child_exit_code=process.poll(),
            owned_processes_remaining=[p.pid for p in owned.values() if p.is_running()])
        if child_report is None or not report["child_runtime_completed"] or report.get("stop_reason"):
            report["diagnostic_output_tail"]=list(diagnostic_tail)
        print("MONITOR_REPORT_JSON "+json.dumps(report),flush=True)
    return 0 if report["child_runtime_completed"] and not report["resource_guard_triggered"] and not report["physics_error_observed"] else 2


if __name__=="__main__":
    raise SystemExit(main())
