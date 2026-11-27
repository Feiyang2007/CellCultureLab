"""回读校验语义标注层：标签要**用 Isaac 自己那套读法**读出来才算数。

写的时候用的是 pxr.UsdSemantics.LabelsAPI（isaacsim.core.utils.semantics
的 add_labels/get_labels 走的就是这个 schema），这里就照 get_labels 的口径读：
遍历 applied schemas，找 SemanticsLabelsAPI:<实例>，取 semantics:labels:<实例>。
只验"能不能读回来"是不够的，所以另外核两件事：
  1. 合成后（overlay 叠在 v07 上）这些 prim 是否真的还存在 —— 路径写错时
     OverridePrim 会照样造出一个孤立的 over prim，写得进但场景里没人；
  2. YAML 声明的每一个类名是否都至少落在一个现存 prim 上。

用法：
    D:/isaacsim-5.1.0/python.bat scripts/check_semantics.py
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _pxr_env  # noqa: E402

_pxr_env.bootstrap()
from pxr import Usd  # noqa: E402

try:
    from pxr import UsdSemantics  # 5.1 的 pxr 命名空间里有；pip 版没有
except ImportError:
    UsdSemantics = None

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def read_labels(prim):
    """与 isaacsim.core.utils.semantics.get_labels 同一口径。"""
    out = {}
    for schema_name in prim.GetAppliedSchemas():
        if schema_name.startswith("SemanticsLabelsAPI:"):
            inst = schema_name.split(":", 1)[1]
            if UsdSemantics is not None:
                attr = UsdSemantics.LabelsAPI(prim, inst).GetLabelsAttr()
            else:
                # 标签本来就是 prim 上的普通 token[] 属性，
                # LabelsAPI 只是 schema 给的读法，绕过它结果一样。
                attr = prim.GetAttribute("semantics:labels:" + inst)
            vals = attr.Get() if attr and attr.IsValid() else None
            out[inst] = list(vals) if vals else []
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--overlay",
                    default=os.path.join(PROJ, "assets", "scenes", "semantics_v07.usda"))
    ap.add_argument("--base",
                    default=os.path.join(PROJ, "assets", "scenes", "biolab_main_v07.usda"))
    args = ap.parse_args()

    if not os.path.exists(args.overlay):
        raise SystemExit("没有 %s —— 先跑 scripts/label_semantics.py --write" % args.overlay)
    st = Usd.Stage.Open(args.overlay)
    if not st:
        raise SystemExit("打不开：%s" % args.overlay)
    base = Usd.Stage.Open(args.base)
    if not base:
        raise SystemExit("打不开基准场景：%s" % args.base)
    # overlay 里 OverridePrim 一个基准场景不存在的路径，USD 不会报错，
    # 它会凭空造出一个只有标签、没有几何没有物理的空 prim。
    # 这种"标注成功"是假的，必须靠回查基准场景才抓得出来。
    base_paths = {str(p.GetPath()) for p in base.Traverse()}

    labeled = {}
    orphan = []
    for prim in st.Traverse():
        labs = read_labels(prim)
        if not labs:
            continue
        path = str(prim.GetPath())
        if path not in base_paths:
            orphan.append(path)
            continue
        labeled[path] = labs

    classes = {}
    roles = {}
    for path, labs in labeled.items():
        for c in labs.get("class", []):
            classes[c] = classes.get(c, 0) + 1
        for r in labs.get("role", []):
            roles[r] = roles.get(r, 0) + 1

    print("回读到标签的 prim：%d 个（层内声明 %d 个）" % (len(labeled), len(labeled) + len(orphan)))
    print("角色:", json.dumps(roles, ensure_ascii=False))
    print("类名 %d 种：" % len(classes))
    for c, n in sorted(classes.items(), key=lambda x: -x[1]):
        print("   %-24s %d" % (c, n))
    if orphan:
        print("!! %d 个 prim 写了标签但合成后的场景里不存在：%s"
              % (len(orphan), ", ".join(orphan[:5])))

    suite_path = os.path.join(PROJ, "tasks", "celllab_suite_v01.yaml")
    if os.path.exists(suite_path):
        import yaml
        with open(suite_path, encoding="utf-8") as f:
            suite = yaml.safe_load(f)
        want = set(suite["objects"]) | set(suite["receptacles"])
        want |= {"joint_" + j for j in suite["joints"]}
        missing = sorted(want - set(classes))
        if missing:
            print("!! YAML 声明了但场景里一个都没标上：%s" % ", ".join(missing))
        else:
            print("YAML 的 %d 个类名全部落到了现存 prim 上" % len(want))
        return 0 if not missing and not orphan else 3
    return 3 if orphan else 0


if __name__ == "__main__":
    sys.exit(main())
