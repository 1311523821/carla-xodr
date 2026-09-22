"""竖直剖面：证明描出来的路落在地板上，而不是屋顶或树冠上。

把扫描网格的顶点按 y（或 x）切片投影，叠上三条线：
  地板线 = 从下往上打的第一个命中（当前实现采用）
  屋顶线 = 从上往下打的第一个命中（朴素俯视会误用这条）
  路中心 = traces.json 描线经服务端反推出的 z
"""
import argparse
import json
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from cnplot import use_chinese  # noqa: E402
from mesh_field import FloorField  # noqa: E402

use_chinese()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mesh", required=True)
    ap.add_argument("--traces", default="traces.json")
    ap.add_argument("--axis", choices=["y", "x"], default="y")
    ap.add_argument("--at", type=float, default=None, help="切片位置，默认取路经过处")
    ap.add_argument("--thick", type=float, default=0.6, help="切片厚度（米）")
    ap.add_argument("--out", default="section.png")
    args = ap.parse_args()

    up = FloorField(args.mesh, "up")
    dn = FloorField(args.mesh, "down")
    m = o3d.io.read_triangle_mesh(args.mesh)
    V = np.asarray(m.vertices)

    doc = json.load(open(args.traces))
    roads = sorted(doc["roads"], key=lambda r: r["id"])
    # --axis y 表示"在 y=at 处切一刀、沿 x 展开"；--axis x 反之。
    # 剖面线、点云散点、路点必须都用"展开轴"，混用会画出一张两轴不自洽的图。
    fixed = 1 if args.axis == "y" else 0
    along = 0 if args.axis == "y" else 1
    if args.at is None:
        r0 = roads[0]["points"]
        args.at = r0[len(r0) // 2][args.axis]

    W = max(V[:, 0].max(), V[:, 1].max()) + 5
    t = np.linspace(-W, W, 400)
    XY = (np.stack([t, np.full_like(t, args.at)], axis=1) if along == 0
          else np.stack([np.full_like(t, args.at), t], axis=1))
    zf = up.floor_z(XY)
    zc = dn.floor_z(XY)

    fig, ax = plt.subplots(figsize=(13, 6))
    sv = V[np.abs(V[:, fixed] - args.at) < args.thick]
    ax.scatter(sv[:, along], sv[:, 2], s=1.2, c="#8899aa", alpha=0.45,
               label="切片内扫描点")

    ax.plot(t, zf, "-", lw=2.5, c="#2ca02c", label="地板（下→上射线，本工具采用）")
    ax.plot(t, zc, "-", lw=2.0, c="#d62728", label="屋顶（朴素俯视会误规划在这）")

    for r in roads:
        p = np.asarray([(q["x"] if along == 0 else q["y"], q["z"]) for q in r["points"]])
        ax.plot(p[:, 0], p[:, 1], "o-", lw=2, ms=5, c="#1f77b4",
                label="描线高程（贴在地板上）")
        hw = max(r["width_left"], r["width_right"])
        ax.annotate("路%d 宽%.1fm" % (r["id"], 2 * hw), (p[len(p) // 2, 0], p[len(p) // 2, 1]),
                    xytext=(0, 14), textcoords="offset points", fontsize=9, color="#1f77b4")
        # 车高：从地板向上 1.3 m
        for q in r["points"]:
            x0 = q["x"] if along == 0 else q["y"]
            ax.plot([x0, x0], [q["z"], q["z"] + 1.3], "-", c="#1f77b4", lw=1, alpha=.5)

    ax.set_xlabel("%s 轴 (m)　　切片固定在 %s=%.2f m，厚度 %.1f m" %
                  (["x", "y"][along], args.axis, args.at, args.thick))
    ax.set_ylabel("高程 z (m)")
    ax.set_title("竖直剖面：路贴在地板上，比屋顶低 %.2f m"
                 % (np.nanmedian(zc) - np.nanmedian(zf)))
    ax.grid(alpha=.25)
    ax.legend(loc="upper right", fontsize=9)
    fig.tight_layout()
    fig.savefig(args.out, dpi=110)
    print("wrote %s  (地板中位 %.2f, 屋顶中位 %.2f, 差 %.2f m)" % (
        args.out, np.nanmedian(zf), np.nanmedian(zc),
        np.nanmedian(zc) - np.nanmedian(zf)))


if __name__ == "__main__":
    main()
