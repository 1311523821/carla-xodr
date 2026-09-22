#!/usr/bin/env python3
"""通道可通行性探测：给定中心线，检查沿线的地板存在性、平整度与**垂直净空**。

办公室这类扫描场景里，"有地板"不等于"车能过"——桌腿、椅子、纸箱都在地板上。
floor_z 从下往上先撞地板，探测不到这些，所以必须再从地板上方打一次射线量净空。

  python3 probe_clearance.py --traces traces.json --mesh <扫描网格> \
      --clearance 1.5 --flat 0.15 [--report-only]
"""

import argparse
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mesh_field import FloorField  # noqa: E402


def resample(pts, ds=0.25):
    a = np.asarray(pts, dtype=float)
    seg = np.hypot(np.diff(a[:, 0]), np.diff(a[:, 1]))
    s = np.concatenate([[0.0], np.cumsum(seg)])
    n = max(int(s[-1] / ds), 2)
    t = np.linspace(0.0, s[-1], n)
    return np.stack([np.interp(t, s, a[:, k]) for k in range(3)], axis=1)


def probe_line(field, pts, half_width, clearance_min, flat_tol, ds=0.25):
    """返回 (通过率, 最差样本, 问题列表)。"""
    P = resample(pts, ds)
    hd = np.gradient(P[:, 1], P[:, 0])
    ang = np.arctan2(hd, np.ones(len(hd)))
    # 横向采样点：法向 = (-sin, cos)
    offs = np.linspace(-half_width, half_width, 2 * int(half_width / 0.25) + 1)
    bad_floor = bad_flat = bad_clear = 0
    worst = (0.0, None)
    n = len(P)
    XY = []
    for o in offs:
        XY.append(np.stack([P[:, 0] - np.sin(ang) * o, P[:, 1] + np.cos(ang) * o], axis=1))
    XY = np.concatenate(XY, axis=0)
    fz = field.floor_z(XY)
    oh = field.obstacle_height(XY, fz)
    # XY 是 offset-major（点 i 的第 j 个偏移在行 i + j*n），一个断面的采样点是
    # 跨 offset 的那一组。以前这里写成 slice(i*len(offs), (i+1)*len(offs))，取的
    # 是同一偏移上连续的 len(offs) 个站 —— 于是"横断面平整度"变成量纵向坡度：
    # 实测干净 1% 坡道被判 6 处不平，地板只覆盖中心线附近时报 84/120 断面无地板。
    for i in range(n):
        sel = np.arange(i, len(XY), n)
        z = fz[sel]
        ok = np.isfinite(z)
        if not ok.any():
            bad_floor += 1
            continue
        if (z[ok] - np.median(z[ok])).ptp() > flat_tol:
            bad_flat += 1
        c = oh[sel][ok]
        # obstacle_height 的约定是"没打到东西 = inf = 净空充足"，所以整条横断面
        # 都 inf 时必须按 inf 算，不能当 0 —— 之前写成 else 0.0，把最通畅的断面
        # 判成净空最差（实测 105 个断面里 103 个"不合格"，且"最差净空 0m"）。
        minc = float(np.min(np.where(np.isfinite(c), c, np.inf))) if len(c) else np.inf
        if minc < clearance_min:
            bad_clear += 1
        if worst[1] is None or minc < worst[0]:
            worst = (minc, (float(P[i, 0]), float(P[i, 1]), float(P[i, 2])))
    tot = len(offs) * n
    hit = int(np.isfinite(fz).sum())
    frac = hit / tot
    issues = []
    if bad_floor:
        issues.append("%d/%d 横断面无地板" % (bad_floor, n))
    if bad_flat:
        issues.append("%d 处横断面不平(>%.2fm)" % (bad_flat, flat_tol))
    if bad_clear:
        issues.append("%d 处净空<%.2fm" % (bad_clear, clearance_min))
    return frac, worst, issues


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--traces", default="traces.json")
    ap.add_argument("--mesh", required=True)
    ap.add_argument("--clearance", type=float, default=1.5, help="所需垂直净空（米）")
    ap.add_argument("--flat", type=float, default=0.15, help="横断面允许高差（米）")
    ap.add_argument("--pad", type=float, default=0.3, help="车道宽度外的安全余量（米）")
    args = ap.parse_args()

    field = FloorField(args.mesh)
    doc = json.load(open(args.traces))
    print("网格 %d 面；所需净空 %.2f m" % (field.tri_count, args.clearance))
    bad = 0
    for r in sorted(doc["roads"], key=lambda x: x["id"]):
        pts = [(p["x"], p["y"], p["z"]) for p in r["points"]]
        hw = max(r["width_left"], r["width_right"]) + args.pad
        frac, worst, issues = probe_line(field, pts, hw, args.clearance, args.flat)
        flag = "" if (frac > 0.97 and not issues) else "   <-- 需改线"
        if flag:
            bad += 1
        print("  road %-3d %-16s 地板覆盖 %5.1f%%  半宽 %.2f m  %s%s" % (
            r["id"], r["name"], 100 * frac, hw,
            "; ".join(issues) if issues else "通畅",
            flag))
        if worst[1] is not None and issues:
            print("        最差净空 %s 在 (%.1f, %.1f)" % (
                "无障碍(>3m)" if math.isinf(worst[0]) else "%.2f m" % worst[0],
                worst[1][0], worst[1][1]))
    print("\n%d/%d 条路需要调整" % (bad, len(doc["roads"])))
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
