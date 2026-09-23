#!/usr/bin/env python3
"""离线对齐检查：xodr 生成的路面 vs 扫描网格地板，全程不需要服务端/UE。

把 CARLA 实算出的 lane 中心点用 frame.json 的 A_S2W 逆映射回扫描帧 S，
再在扫描网格上打射线取地板高度，量化 Δz。这是"车会不会悬空/陷进去"的预测量。

注意：A 未标定时本检查只反映"预测的手性镜像"对不对，残差里含真实平移量；
标定后残差才是纯粹的拟合误差。
"""

import argparse
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mesh_field import FloorField            # noqa: E402
from fit_geometry import (FRAME_ACCEPTANCE, M_XODR2WORLD4,
                          frame_json_path)  # noqa: E402

INLIER_M = 0.30


def run(mesh, xodr, frame_json=None, step=0.5, lane=0):
    """跑一遍对齐检查，返回结构化结果。CLI 和编辑器的「对齐检查」按钮共用这一份。

    Δz 是"xodr 认为的路面高度 - 扫描网格上的地板高度"，也就是"车会不会悬空/陷进去"。
    """
    fjp = frame_json or frame_json_path(mesh)
    note = None
    try:
        with open(fjp) as f:
            fj = json.load(f)
        A = np.asarray(fj["A_S2W"], dtype=float)
    except (OSError, KeyError, TypeError, ValueError) as e:
        # 没标定过不是错误，是"这一步还没做"。未标定时 A_S2W 取 CARLA 内建的那个
        # 镜像（和 derive_T_S2X 同一个约定 => T_S2X 退化成单位阵），检查照样能跑，
        # 只是残差里含真实平移量，所以要把原因说出来。
        fj = {}
        A = M_XODR2WORLD4.copy()
        note = ("%s：%s —— 按未标定计算，残差里含扫描帧→world 的平移量"
                % ("还没有标定过（缺 frame.json）" if isinstance(e, OSError)
                   else "frame.json 读不动（%s: %s）" % (type(e).__name__, e),
                   os.path.relpath(fjp)))
    Ai = np.linalg.inv(A)

    import carla
    with open(xodr) as f:
        xml = f.read()
    m = carla.Map("alignprobe", xml)
    seed = m.generate_waypoints(step)
    if not seed:
        raise SystemExit("generate_waypoints 返回空")

    # 高程定义在参考线(lane 0)上，车道中心只是继承它；拿车道中心比地板会把
    # "路没对齐"和"路太宽压到家具上"混成一个数字。所以默认只比参考线。
    tops = {}
    for w in seed:
        tops[w.road_id] = max(tops.get(w.road_id, 0.0), w.s)
    W, ids = [], []
    for rid, smax in tops.items():
        s = 0.0
        while s <= smax:
            w = m.get_waypoint_xodr(rid, lane, min(s, max(smax - 1e-3, 0.0)))
            if w is not None:
                W.append([w.transform.location.x, w.transform.location.y,
                          w.transform.location.z])
                ids.append(rid)
            s += step
    W = np.asarray(W, dtype=float)
    ids = np.asarray(ids)

    field = FloorField(mesh)
    q = (Ai @ np.hstack([W, np.ones((len(W), 1))]).T).T[:, :3]
    z = field.floor_z(q[:, :2])
    r = q[:, 2] - z
    ok = np.isfinite(r)
    if ok.sum() < max(3, int(len(W) * 0.2)):
        raise SystemExit("可对比点太少（%d/%d）：A_S2W 可能完全错位，先看 xy 是否覆盖网格"
                         % (int(ok.sum()), len(W)))
    ri = r[ok]

    roads = []
    worst = 0.0
    for rid in sorted(set(ids.tolist())):
        sel = ok & (ids == rid)
        if sel.sum() < 3:
            roads.append({"id": rid, "n": int(sel.sum()), "few": True})
            continue
        rr = r[sel]
        rmse = math.sqrt(float(np.mean(rr ** 2)))
        worst = max(worst, rmse)
        xy = q[sel]
        roads.append({"id": rid, "n": int(sel.sum()), "rmse": rmse,
                      "med": float(np.median(rr)),
                      "x": [float(xy[:, 0].min()), float(xy[:, 0].max())],
                      "y": [float(xy[:, 1].min()), float(xy[:, 1].max())]})

    lim = fj.get("acceptance", {}).get("rmse_z_m", FRAME_ACCEPTANCE["rmse_z_m"])
    return {"frame_json": fjp, "calibrated": bool(fj.get("calibrated")),
            "frame_note": note,
            "lane": lane, "step": step, "n": len(W), "tri_count": field.tri_count,
            "floor_frac": float(ok.mean()),
            "rmse": math.sqrt(float(np.mean(ri ** 2))), "p50": float(np.median(ri)),
            "p95": float(np.percentile(np.abs(ri), 95)), "max": float(np.abs(ri).max()),
            "inlier_m": INLIER_M, "inlier_frac": float(np.mean(np.abs(ri) < INLIER_M)),
            "roads": roads, "worst": worst, "limit": lim, "pass": worst <= lim}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--xodr", required=True,
                    help="要检查的 xodr，例如 cache/<场景>/out/<地图名>.xodr")
    ap.add_argument("--mesh", required=True)
    ap.add_argument("--frame-json", default=None,
                    help="默认取 mesh 所在场景目录的 frame.json")
    ap.add_argument("--step", type=float, default=0.5)
    ap.add_argument("--lane", type=int, default=0, help="检查哪条车道（0=参考线）")
    args = ap.parse_args()

    d = run(args.mesh, args.xodr, args.frame_json, args.step, args.lane)
    print("A_S2W calibrated=%s" % d["calibrated"])
    if d.get("frame_note"):
        print("  " + d["frame_note"])
    print("\n%d 个 %s 采样点，网格 %d 面" % (
        d["n"], "参考线(lane 0)" if d["lane"] == 0 else "lane %d" % d["lane"],
        d["tri_count"]))
    print("  映射回 S 帧后有地板可查的: %.1f%%" % (100 * d["floor_frac"]))
    print("  Δz  RMSE=%.4f  p50=%+.4f  p95|.|=%.4f  max|.|=%.4f m" % (
        d["rmse"], d["p50"], d["p95"], d["max"]))
    print("  内点率(|Δz|<%.2f m) = %.1f%%" % (d["inlier_m"], 100 * d["inlier_frac"]))
    print("\n  逐路:")
    for rr in d["roads"]:
        if rr.get("few"):
            print("    road %-3d  可对比点 %d —— 太少" % (rr["id"], rr["n"]))
            continue
        print("    road %-3d  n=%-4d Δz RMSE=%.4f  中位=%+.3f  S帧 x[%.1f,%.1f] y[%.1f,%.1f]" % (
            rr["id"], rr["n"], rr["rmse"], rr["med"],
            rr["x"][0], rr["x"][1], rr["y"][0], rr["y"][1]))
    print("\n验收: 最差 Δz RMSE=%.4f m，阈值 %.2f -> %s" % (
        d["worst"], d["limit"], "通过" if d["pass"] else "未通过"))
    if not d["calibrated"]:
        print("（A_S2W 尚未标定，此结果含未补偿的平移/旋转；标定后重跑本检查）")
    sys.exit(0 if d["pass"] else 1)


if __name__ == "__main__":
    main()
