"""
把 AutoBio 上游资产搬进 CellCultureLab，并把 MJCF 里的 mesh/texture 引用改成工程内相对路径。

为什么要改 XML：Isaac Sim 的 MJCF 导入器不支持 MuJoCo 的 <compiler assetdir="../../assets"/>，
它会把 file="instrument/xxx/y.obj" 当成相对 XML 自身目录的路径去找（结果找到 model/instrument/instrument/…），
于是一件资产都加载不上、转出来是空壳。改成相对 XML 位置能直接解析的路径即可绕开。

用法：
    PY="D:/IsaacSim5/env/Scripts/python.exe"
    $PY scripts/stage_upstream.py            # 搬场景要用的族（仪器/容器/架/工具）
    $PY scripts/stage_upstream.py --all      # 整个 assets 全搬（约 75MB，含机器人与字体）
产物：
    upstream/assets/<族>/...      真实网格与贴图
    upstream/mjcf/<名字>.xml      改写过的 MJCF（从这里喂给导入器）
"""
import os, re, shutil, sys

REPO = os.environ.get("AUTOBIO_REPO", r"C:/Users/17690/.codex/tmp-repos/AutoBio")
SRC_ASSETS = os.path.join(REPO, "autobio", "assets")
SRC_MODEL = os.path.join(REPO, "autobio", "model")
PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DST_ASSETS = os.path.join(PROJ, "upstream", "assets")
DST_MJCF = os.path.join(PROJ, "upstream", "mjcf")

# 场景/子任务真正用得上的族；--all 时改成整个 assets
NEED_FAMILIES = [
    "instrument", "container", "rack", "tool",
]
INSTRUMENTS = ["centrifuge_eppendorf_5430", "centrifuge_eppendorf_5910_ri", "centrifuge_tiangen_tgear_mini",
               "thermal_cycler_biorad_c1000", "thermal_mixer_eppendorf_c", "vortex_mixer_genie_2"]

ALLMODE = "--all" in sys.argv
MB = lambda n: "%.1f MB" % (n / 1e6)


def copy_tree(src, dst):
    """只补缺，不覆盖已有同名同尺寸文件；不删任何东西。"""
    n = skipped = 0
    total = 0
    for dirpath, dirnames, filenames in os.walk(src):
        for fn in filenames:
            s = os.path.join(dirpath, fn)
            rel = os.path.relpath(s, src)
            d = os.path.join(dst, rel)
            os.makedirs(os.path.dirname(d), exist_ok=True)
            if os.path.isfile(d) and os.path.getsize(d) == os.path.getsize(s):
                skipped += 1
                continue
            shutil.copy2(s, d)
            n += 1
            total += os.path.getsize(s)
    return n, skipped, total


print("=" * 78)
print("源：", SRC_ASSETS)
print("目标：", DST_ASSETS)
if not os.path.isdir(SRC_ASSETS):
    print("!! 本机克隆不存在，检查 AUTOBIO_REPO 环境变量")
    sys.exit(1)

if ALLMODE:
    families = sorted(next(os.walk(SRC_ASSETS))[1])
else:
    families = [f for f in NEED_FAMILIES if os.path.isdir(os.path.join(SRC_ASSETS, f))]

grand = 0
for fam in families:
    n, sk, byts = copy_tree(os.path.join(SRC_ASSETS, fam), os.path.join(DST_ASSETS, fam))
    grand += byts
    print("  %-12s 新复制 %-4d 已存在跳过 %-4d %+10s" % (fam, n, sk, MB(byts)))
print("  合计搬运", MB(grand))

# ---- 改写仪器 MJCF：file="X/..." -> file="../assets/X/..."，并去掉 assetdir ----
os.makedirs(DST_MJCF, exist_ok=True)
PREFIX = re.compile(r'file="((?:instrument|container|rack|tool|robot|common)/)')
print()
print("改写 MJCF 到", DST_MJCF)
for name in INSTRUMENTS:
    src = os.path.join(SRC_MODEL, "instrument", name + ".xml")
    if not os.path.isfile(src):
        print("  [缺] ", src)
        continue
    txt = open(src, encoding="utf-8").read()
    before = len(PREFIX.findall(txt))
    txt = PREFIX.sub(r'file="../assets/\1', txt)
    txt = re.sub(r'\s*assetdir="[^"]*"', "", txt)
    out = os.path.join(DST_MJCF, name + ".xml")
    open(out, "w", encoding="utf-8", newline="\n").write(txt)
    # 逐条核对改写后的 file= 是否真能在磁盘上找到
    bad = []
    for m in re.findall(r'file="([^"]+)"', txt):
        if not os.path.isfile(os.path.normpath(os.path.join(DST_MJCF, m))):
            bad.append(m)
    print("  %-34s 改写 file 引用 %-3d 条，仍找不到文件的 %-3d %s"
          % (name, before, len(bad), ("← " + bad[0]) if bad else ""))
print()
print("下一步：python scripts/convert_instruments.py")
