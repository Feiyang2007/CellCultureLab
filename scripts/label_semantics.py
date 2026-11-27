"""给任务涉及的对象打语义标签，写到独立 overlay 层。

为什么单独一层而不是改 v07：
    源图层 v07 自己只是 v03 上的 overlay，再往里写会把这一轮的东西和上一轮搅在一起，
    而且 docs 里已经立过规矩 —— 派生场景另存新路径，别 Save 覆盖源图层。

为什么这一步是"数据生成的前置"：
    两篇基线（Spatial Forcing 2510.12276 / ROCKET 2602.17951）都是语言条件策略，
    指令里的"50 mL 管""5430 转子孔位"必须能落到 prim 上。现在仓库里
    语义标签一个都没写过，语言接地是空的。

用哪个 API（这一条查得费劲，记下来）：
    pxr.Semantics 在 USD 24.5 里只是**废弃转发 shim**，它内部找的是
    'SemanticsSemanticsAPI' 这个 schema 名，而 omni.usd.schema.semantics 注册的是
    'SemanticsAPI'（只有 semanticType/semanticData 两个 string，也是废弃的旧格式）。
    Isaac Sim 5.1 的正主是 **pxr.UsdSemantics.LabelsAPI**（多实例 API，
    schema 名 SemanticsLabelsAPI:<实例>，属性 semantics:labels:<实例> 是 token[]），
    isaacsim.core.utils.semantics.add_labels/get_labels 走的就是它，
    Replicator 的语义 writer 读的也是它。用错的话标签写得进去但没人读得出来。
    要让 Apply 不报"not registered"，schema 的 plugInfo.json 目录必须在
    PXR_PLUGINPATH_NAME 里 —— 见 scripts/_pxr_env.py。

标签从 tasks/celllab_suite_v01.yaml + 场景清单推出来，不手写路径：
    YAML 里只写正则，清单给真实 prim，两边对不上的直接报错，不会静默漏标。

用法：
    D:/isaacsim-5.1.0/python.bat scripts/label_semantics.py            # 试跑，只报数
    D:/isaacsim-5.1.0/python.bat scripts/label_semantics.py --write    # 落 overlay 层

产物名跟着 YAML 的 base_scene 走（v09 → assets/scenes/semantics_v09.usda），
一层只服务一个场景版本；写之前会把 base_scene 的整条 subLayers 链摊开看一眼，
新层自己已经在这条链里就直接拒绝（老 semantics_v07 已被 v08→v09 引着，
再让它子层 v09 就是环）。下游要带标签的完整场景，打开 semantics_<版本>.usda，
它本身就子层了 base_scene，几何与标签一起有。
"""
import argparse
import json
import os
import re
import sys

import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _pxr_env  # noqa: E402

_pxr_env.bootstrap()
from pxr import Sdf, Usd, UsdGeom, UsdSemantics  # noqa: E402

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# labels 属性是 token[]，"50 mL 离心管" 这种带空格/非 ASCII 的字符串进不去。
# 所以 token 化的类名走 semantics:labels，中文显示名单独挂一个 string 自定义属性。
CN_ATTR = "celllab:cn_label"
INST_CLASS = "class"
INST_ROLE = "role"


def load_inventory(suite_def):
    with open(os.path.join(PROJ, suite_def["inventory"]), encoding="utf-8") as f:
        return json.load(f)


def sublayer_closure(scene, _seen=None):
    """沿 subLayers 递归收集（绝对路径）集合 —— 用来判断新层会不会成环。

    环是这一轮真踩过的：v09 → v08 → semantics_v07 → v07 已经串好了，
    若把 base_scene 提到 v09 之后还按老默认名往 semantics_v07.usda 写，
    就等于让 semantics_v07 再子层 v09，USD 会静默裁掉其中一条弧，
    结果是标签写进去了但场景里读不到（或反过来说几何没了），比报错更难查。
    """
    if _seen is None:
        _seen = set()
    ap = os.path.abspath(scene)
    if ap in _seen or not os.path.exists(ap):
        return _seen
    _seen.add(ap)
    layer = Sdf.Layer.FindOrOpen(ap)
    if layer is None:
        return _seen
    for sub in layer.subLayerPaths:
        sub = sub.replace("\\", "/")
        if sub.startswith("@"):
            sub = sub.strip("@")
        base = sub if os.path.isabs(sub) else \
            os.path.normpath(os.path.join(os.path.dirname(ap), sub))
        sublayer_closure(base, _seen)
    return _seen


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--suite", default=os.path.join(PROJ, "tasks", "celllab_suite_v01.yaml"))
    ap.add_argument("--out", default=None,
                    help="默认 assets/scenes/semantics_<base_scene 同名>.usda，"
                         "跟着 base_scene 走，省得把新标签写进老层里成环")
    ap.add_argument("--write", action="store_true", help="真落盘；不加就只试跑")
    args = ap.parse_args()

    with open(args.suite, encoding="utf-8") as f:
        suite_def = yaml.safe_load(f)
    base = os.path.join(PROJ, suite_def["base_scene"])
    if args.out is None:
        # 沿用仓库里 semantics_v07.usda 那个名字格式：去掉 biolab_main_ 前缀只留版本号。
        stem = os.path.splitext(os.path.basename(base))[0]
        args.out = os.path.join(PROJ, "assets", "scenes",
                                "semantics_%s.usda" % stem.replace("biolab_main_", ""))
    args.out = os.path.abspath(args.out)
    inv = load_inventory(suite_def)
    bodies = {rb["path"]: rb for rb in inv["rigid_bodies"]}
    colliders = {c["path"]: c for c in inv.get("static_colliders", [])}
    slots = {}
    for a in inv["slot_anchors"]:
        slots.setdefault(a["group"], []).append(a)
    joints = {}
    for s in inv.get("joint_stations", []):
        for j in s["joints"]:
            joints[j["path"]] = j

    plan = []

    for name, spec in suite_def["objects"].items():
        rx = re.compile(spec["prim_pattern"])
        hits = [p for p in bodies if rx.match(p)]
        if not hits:
            raise SystemExit("对象类 %s 的正则 %r 匹配到 0 个刚体，先修 YAML 或重跑盘点"
                             % (name, spec["prim_pattern"]))
        for p in hits:
            plan.append((p, name, "manipulable", spec["cn"]))

    for name, spec in suite_def["receptacles"].items():
        if spec["kind"] == "slot_group":
            rx = re.compile(spec["pattern"])
            hits = [a["path"] for g, items in slots.items() for a in items if rx.match(a["path"])]
            if not hits:
                raise SystemExit("容器 %s 的正则 %r 匹配到 0 个槽位" % (name, spec["pattern"]))
            for p in hits:
                plan.append((p, name, "receptacle", spec["cn"]))
        elif spec["kind"] == "surface":
            # 台面是静态碰撞体、不是刚体，走的是清单里另一栏。
            # 语言指令里"放到实验台面上"要能接地到这个 prim，所以它也得有标签。
            if spec["prim"] not in colliders:
                raise SystemExit("容器 %s 指向 %s，不是清单里的静态碰撞体"
                                 % (name, spec["prim"]))
            plan.append((spec["prim"], name, "receptacle", spec["cn"]))
        else:
            if spec["prim"] not in bodies:
                raise SystemExit("容器 %s 指向 %s，不是清单里的独立刚体" % (name, spec["prim"]))
            plan.append((spec["prim"], name, "receptacle", spec["cn"]))

    for name, spec in suite_def["joints"].items():
        if spec["path"] not in joints:
            raise SystemExit("关节 %s 指向 %s，不在清单里" % (name, spec["path"]))
        plan.append((spec["path"], "joint_" + name, "articulated", spec["cn"]))

    uniq = {}
    for p, cls, role, cn in plan:
        ent = uniq.setdefault(p, {"class": [], "role": [], "cn": cn})
        if cls not in ent["class"]:
            ent["class"].append(cls)
        if role not in ent["role"]:
            ent["role"].append(role)

    print("准备给 %d 个 prim 打语义标签" % len(uniq))
    kinds = {}
    for p, ent in uniq.items():
        for r in ent["role"]:
            kinds[r] = kinds.get(r, 0) + 1
    print("分类:", json.dumps(kinds, ensure_ascii=False))

    if not args.write:
        print("（试跑，没写文件。目标层 %s，加 --write 才落盘）"
              % os.path.relpath(args.out, PROJ).replace("\\", "/"))
        for p, ent in list(uniq.items())[:5]:
            print("   ", p, ent["class"], ent["role"], ent["cn"])
        return 0

    closure = sublayer_closure(base)
    if args.out in closure:
        raise SystemExit("要把标签写进 %s，可它已经在 base_scene %s 的子层链里了 —— "
                         "再让新层子层 base_scene 就成了环（USD 只会静默裁弧，不报错）。"
                         "换个和 base_scene 同名的新层（默认就是 semantics_<scene>.usda）。"
                         % (os.path.basename(args.out), os.path.basename(base)))
    layer = Sdf.Layer.CreateNew(args.out)
    if layer is None:
        raise SystemExit("建不出 overlay 图层 %s" % args.out)
    layer.subLayerPaths.append(os.path.relpath(base, os.path.dirname(args.out)).replace("\\", "/"))
    stage = Usd.Stage.Open(layer)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    n = n_cn = n_skip = 0
    for p, ent in sorted(uniq.items()):
        # overlay 层里没显式声明的 prim 是不存在的，必须先造一个 override spec，
        # 否则拿回来的是 invalid prim，Apply 会静默失败。
        prim = stage.OverridePrim(p)
        if not prim:
            print("!! 造不出 %s 的 override，跳过" % p)
            n_skip += 1
            continue
        for inst, toks in ((INST_CLASS, ent["class"]), (INST_ROLE, ent["role"])):
            api = UsdSemantics.LabelsAPI.Apply(prim, inst)
            api.CreateLabelsAttr().Set(toks)
        if ent["cn"]:
            prim.CreateAttribute(CN_ATTR, Sdf.ValueTypeNames.String, custom=True).Set(ent["cn"])
            n_cn += 1
        n += 1
    # 这个 USD 构建里 Sdf.Layer.Export() 必须给文件名，不跟 0.20 的无参重载。
    layer.Export(os.path.abspath(args.out))
    print("写入 %d 个 prim（%d 个带中文名，%d 个跳过）→ %s"
          % (n, n_cn, n_skip, os.path.relpath(args.out, PROJ).replace("\\", "/")))
    return 0 if n_skip == 0 else 3


if __name__ == "__main__":
    sys.exit(main())
