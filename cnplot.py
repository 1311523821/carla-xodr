"""matplotlib 中文字体配置（一个地方配好，所有出图脚本共用）。

系统里本来就有 CJK 字体（fc-list :lang=zh 有 89 个），出现方框不是因为缺字体，
而是 matplotlib 默认的 font.sans-serif 列表里没有 CJK 家族，于是回退到不含中文的
DejaVu Sans。这里把 CJK 家族插到最前面，并关掉 unicode_minus，否则负号也会变方框。

用法：在 import pyplot 之后调用一次 use_chinese()。
"""

import matplotlib as mpl
import matplotlib.font_manager as fm

# 优先明确简中的家族；Noto Sans CJK JP 含全部中文字形，只是共用汉字字形偏好略有差异
PREF = ["Noto Sans CJK SC", "Noto Sans CJK JP", "Droid Sans Fallback",
        "AR PL UMing CN", "WenQuanYi Micro Hei", "Microsoft YaHei"]


def available():
    have = {f.name for f in fm.fontManager.ttflist}
    return [n for n in PREF if n in have]


def use_chinese(verbose=False):
    chain = available() + ["DejaVu Sans"]
    mpl.rcParams["font.sans-serif"] = chain
    mpl.rcParams["font.family"] = "sans-serif"
    mpl.rcParams["axes.unicode_minus"] = False
    if verbose:
        print("matplotlib 中文字体链:", " > ".join(chain))
    return chain
