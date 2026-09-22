#!/usr/bin/env python3
"""俯视叠图：把 traces.json 的描线、拟合出的 xodr 参考线与车道边线，
画在扫描地板高度图上，让"画在哪、画对没有"变成肉眼可见的事。

  python3 render_plan.py --mesh <扫描网格> --traces traces.json \
      --fitted out/roads_fitted_xodr.json --out plan.png

黄叉 = 该处垂直净空不足（车会撞上桌/椅），是描线要避开的地方。
"""

import argparse
import json
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from cnplot import use_chinese  # noqa: E402
from fit_geometry import geom_from_dict  # noqa: E402
from mesh_field import FloorField  # noqa: E402
from xodr_geom import lateral_normal  # noqa: E402

use_chinese()


def chain_sample(fitted, step=0.25):
    """把一份 roads_fitted_*.json 稠密采样成逐路的 (ref, left, right) 点列。"""
    out = []
    for r in sorted(fitted["roads"], key=lambda x: x["id"]):
        gs = [geom_from_dict(d) for d in r["geometry"]]
        ref = []
        for g in gs:
            n = max(int(math.ceil(g.length / step)), 2)
            for i in range(n + 1):
                p = g.at(g.length * i / n)
                ref.append((p.x, p.y, p.hdg))
        ref = np.asarray(ref)
        left, right = [], []
        for x, y, h in ref:
            ux, uy = lateral_normal(h)
            left.append((x + ux * r["width_left"], y + uy * r["width_left"]))
            right.append((x - ux * r["width_right"], y - uy * r["width_right"]))
        out.append((r, ref[:, :2], np.asarray(left), np.asarray(right)))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mesh", required=True)
    ap.add_argument("--traces", default="traces.json")
    ap.add_argument("--fitted", default="out/roads_fitted_xodr.json")
    ap.add_argument("--clearance", type=float, default=1.5)
    ap.add_argument("--out", default="plan.png")
    args = ap.parse_args()

    field = FloorField(args.mesh)
    n = 260
    xs = np.linspace(-14, 22, n)
    ys = np.linspace(-34, 7, n)
    XY = np.stack([v.ravel() for v in np.meshgrid(xs, ys)], axis=1)
    z = field.floor_z(XY).reshape(len(ys), len(xs))
    oh = field.obstacle_height(XY, z.ravel()).reshape(z.shape)
    gx, gy = np.meshgrid(xs, ys)

    fig, ax = plt.subplots(figsize=(11, 12))
    im = ax.imshow(np.ma.masked_invalid(z), origin="lower",
                   extent=[xs[0], xs[-1], ys[0], ys[-1]], cmap="terrain",
                   vmin=-2.5, vmax=3.0, alpha=0.85)
    plt.colorbar(im, ax=ax, label="地板高程 z (m)")
    ax.set_title("俯视图：扫描地板 + 描线 + 拟合参考线 + 车道边线（x 向右，y 向上）")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.grid(alpha=0.25)

    # 净空不足处
    blocked = np.isfinite(oh) & (oh < args.clearance) & np.isfinite(z)
    ax.scatter(gx[blocked], gy[blocked], s=3, c="orange", marker="x",
               alpha=0.45, label="净空 < %.1f m（车过不去）" % args.clearance)

    if os.path.exists(args.traces):
        doc = json.load(open(args.traces))
        first = True
        for r in sorted(doc["roads"], key=lambda x: x["id"]):
            p = np.asarray([(q["x"], q["y"]) for q in r["points"]])
            ax.plot(p[:, 0], p[:, 1], "o-", color="lime", ms=4, lw=1.2,
                    label="你描的线" if first else "")
            first = False
            for q, (x, y) in zip(r["points"], p):
                ax.annotate("s=%.1f\nz=%.2f" % (q["s"], q["z"]), (x, y),
                            fontsize=6, color="darkgreen", xytext=(4, 4),
                            textcoords="offset points")

    if os.path.exists(args.fitted):
        fitted = json.load(open(args.fitted))
        for r, ref, left, right in chain_sample(fitted):
            ax.plot(ref[:, 0], ref[:, 1], "-", color="red", lw=1.6,
                    label="拟合出的 xodr 参考线")
            ax.plot(left[:, 0], left[:, 1], "--", color="blue", lw=1.0,
                    label="车道边线")
            ax.plot(right[:, 0], right[:, 1], "--", color="blue", lw=1.0)
            ax.annotate("road %d  %.1f m" % (r["id"], r["length"]), ref[len(ref) // 2],
                        fontsize=8, color="red", weight="bold")

    ax.legend(loc="upper left", fontsize=8)
    fig.tight_layout()
    fig.savefig(args.out, dpi=115)
    print("wrote %s" % args.out)


if __name__ == "__main__":
    main()
