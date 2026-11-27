"""离线 USD 工具共用的引导：只加 pxr 的路径、DLL 目录与 USD schema 插件路径，不启 SimulationApp。

用途是给"不改场景、只读场景"的批量检查用的。跑一次 SimulationApp 要几十秒，
把静态盘点也放上去会把迭代拖死；需要真物理状态（刚体是否被仿真）时必须另写
带 app 的脚本，不能用这里读 USD 变换代替。

schema 插件路径这个坑：只把包目录塞进 sys.path 是不够的 —— `import Semantics`
能成功，但 `SemanticsAPI.Apply()` 会报
    Class 'SemanticsSemanticsAPI' is not correctly registered with the UsdSchemaRegistry
因为 C++ 侧的 schema 是靠 plugInfo.json + generatedSchema.usda 注册的，
得让这些目录出现在 PXR_PLUGINPATH_NAME 里，而且必须在第一次 import pxr.Usd 之前设好。
"""
import glob
import importlib.util
import os
import sys


def _find_root():
    """ISAAC_ROOT > pip 版 isaacsim 包 > 本机 standalone。

    pip 装法（Isaac Lab 的 isaacsim extra）把 extscache 放在包目录里面，
    位置随版本与环境走，所以让它自己说；find_spec 不执行模块，不会提前拉起 carb。
    """
    cands = [os.environ.get("ISAAC_ROOT")]
    spec = importlib.util.find_spec("isaacsim")
    if spec and spec.submodule_search_locations:
        cands.append(str(list(spec.submodule_search_locations)[0]))
    cands.append(r"D:/isaacsim-5.1.0")
    for c in cands:
        if c and os.path.isdir(os.path.join(c, "extscache")):
            return c
    return cands[-1]


ISAAC = _find_root()
EXTS = os.path.join(ISAAC, "extscache")

# 包名后缀是构建串：5.1 是 +69cbf6ad...cp311，6.x 换 Python 3.12 后整串都变。
# 写死名字的话到新机器上第一条就 import 不到 pxr，所以按前缀找、取最新的那个。
_PXR_PKGS = [
    "omni.usd.libs-",
    "omni.usd.schema.semantics-",
    "omni.usd.schema.physx-",
    "omni.usd.schema.anim-",
    "omni.usd.schema.physics-",
    "omni.usd.schema.rendering-",
]

# plugInfo.json 最深挖到第 4 层：语义 schema 的在 plugins/semantics/resources，
# 再深的基本是测试数据，塞进去只会让注册表变慢并撞名字。
_MAX_DEPTH = 4


def _plugin_dirs(root):
    found = []
    base_depth = root.rstrip("/\\").count(os.sep)
    for dirpath, dirnames, filenames in os.walk(root):
        if dirpath.count(os.sep) - base_depth > _MAX_DEPTH:
            dirnames[:] = []
            continue
        if "plugInfo.json" in filenames:
            found.append(dirpath)
    return found


def _resolve(prefix):
    """前缀 -> 实际包目录名（同前缀并存时取版本号最大的）。"""
    hits = [os.path.basename(p) for p in glob.glob(os.path.join(EXTS, prefix + "*"))
            if os.path.isdir(p)]
    if not hits:
        return None
    return sorted(hits)[-1]


def _alias_top_level_schemas():
    """6.x 把 PhysxSchema / Semantics 放在顶层包，而仓库里写的是 `from pxr import PhysxSchema`。
    5.1 把它们合在 pxr 命名空间下，所以那条写法在老版本才对。这里补一层别名，
    让 27 个离线脚本不用各自写两套导入。
    """
    import importlib
    import pxr
    for name in ("PhysxSchema", "Semantics", "UsdRepair"):
        if importlib.util.find_spec("pxr." + name) is not None:
            continue
        try:
            mod = importlib.import_module(name)
        except ImportError:
            continue
        sys.modules["pxr." + name] = mod
        setattr(pxr, name, mod)


def _pxr_from_pip():
    """pxr 是不是解析到 extscache 之外 —— pip 装的 isaacsim 把 USD 放在 site-packages。

    必须在把 Isaac 的包目录塞进 sys.path 之前问（find_spec 不执行模块，不会提前 import carb）。
    5.1 standalone 返回 False（pxr 只在 extscache 里），pip 装法返回 True。
    """
    try:
        spec = importlib.util.find_spec("pxr")
    except (ImportError, ValueError):
        return False
    return bool(spec and spec.origin
                and os.path.normcase(EXTS) not in os.path.normcase(spec.origin))


def bootstrap():
    # site-packages 那份 pxr 和 extscache 的基础 USD 插件不是同一个 C++ 构建，
    # 两边同时加载会让每一次 Sdf.Layer.FindOrOpen 抛 Tf.ErrorException
    # （底层报"找不到指定的程序"，即 ERROR_PROC_NOT_FOUND）。
    # 实测只有 omni.usd.libs 的 bin/usd 与 bin/usd/sdf/resources 会炸，
    # 但整包基础插件都不需要 —— 核心 schema 已经在 site-packages 那份 pxr 里。
    pip_pxr = _pxr_from_pip()
    plugin_paths = []
    for prefix in _PXR_PKGS:
        name = _resolve(prefix)
        if name is None:
            continue
        root = os.path.join(EXTS, name)
        if root not in sys.path:
            sys.path.insert(0, root)
        bindir = os.path.join(root, "bin")
        if os.path.isdir(bindir):
            try:
                os.add_dll_directory(bindir)
            except OSError:
                pass
        if pip_pxr and prefix == "omni.usd.libs-":
            continue
        plugin_paths.extend(_plugin_dirs(root))
    if not plugin_paths:
        raise SystemExit("在 %s 下找不到 omni.usd.* 包 —— 设 ISAAC_ROOT 指向 Isaac 安装根目录" % EXTS)
    old = os.environ.get("PXR_PLUGINPATH_NAME", "")
    merged = [p for p in plugin_paths if p not in old]
    os.environ["PXR_PLUGINPATH_NAME"] = os.pathsep.join(
        ([old] if old else []) + merged)
    from pxr import Gf, Usd, UsdGeom, UsdPhysics  # noqa: F401
    _alias_top_level_schemas()
    return True
