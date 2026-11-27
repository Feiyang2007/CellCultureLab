"""盘点 outputs/demo_data：verdict、帧数、相机路数、动作幅度、seed。

默认只打印；--move-broken 才把 verdict != well-formed 的样本移到 rejected/。
用法：
  python scripts/audit_demo.py
  python scripts/audit_demo.py --move-broken
"""
import argparse
import glob
import json
import os
import shutil

import h5py
import numpy as np

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=os.path.join(PROJ, "outputs", "demo_data"))
    ap.add_argument("--move-broken", action="store_true")
    args = ap.parse_args()

    files = sorted(glob.glob(os.path.join(args.dir, "*.hdf5")))
    if not files:
        print("没有样本：", args.dir)
        return 1
    bad = []
    n_ok = 0
    for p in files:
        with h5py.File(p) as f:
            d = f["data"]
            ap_ = d["obs/agent_pos"][:]
            ac = d["actions"][:]
            verdict = d.attrs.get("success_verdict", "?")
            keys = json.loads(d.attrs.get("image_keys", "[]"))
            views = len([k for k in keys if k in d])
            imgs = int(sum(d[k].shape[0] for k in keys if k in d))
            l1 = float(np.abs(ac).sum(1).max()) if ac.size else 0.0
            finite = bool(np.isfinite(ap_).all()) and float(np.abs(ap_).max(initial=0)) < 1e3
            ok = verdict == "well-formed" and finite and ap_.shape[0] > 10
            n_ok += int(ok)
            if not ok:
                bad.append(p)
            print("%-52s %-11s %3d帧 %2d路 %3d图 L1=%.3f %s"
                  % (os.path.basename(p), verdict, ap_.shape[0], views, imgs, l1,
                     "" if ok else "EXCLUDE"))
    print("\n合计 %d 条：可用 %d，需剔除 %d" % (len(files), n_ok, len(bad)))
    if args.move_broken and bad:
        rej = os.path.join(args.dir, "rejected")
        os.makedirs(rej, exist_ok=True)
        for p in bad:
            dst = os.path.join(rej, os.path.basename(p))
            if not os.path.exists(dst):
                shutil.move(p, dst)
            else:
                print("已存在，跳过：", os.path.basename(p))
        print("已移入", rej)
    elif bad:
        print("（加 --move-broken 才会移动到 rejected/）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
