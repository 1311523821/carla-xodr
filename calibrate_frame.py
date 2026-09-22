#!/usr/bin/env python3
"""实测 Blender/扫描帧 S -> CARLA world 帧 W 的刚体（含镜像），写回 frame.json。

不靠推导 UE 导入到底翻哪个轴：在关卡里按网格**从上往下**打射线取地板，
再把每个 world 点映回 S 帧，与扫描网格的地板高度做面-面对比，
最小二乘解 (mirror, yaw, scale, tx, ty, tz)。

方向必须是向下，和扫描帧那一侧相反 —— UE 的复杂碰撞三角形是**单面**的，
扫描网格的地板面朝上，从下往上打会穿过它打到天花板（实测同一点：向上 +4.9、
向下 -5.6，而 xodr 路面在 -5.6）。open3d 那侧是双面的，所以它照旧从下往上。

用法：先在有该关卡地图的 CARLA 服务端上跑
  python3 calibrate_frame.py --mesh <扫描网格> --center 0 0 --extent 60 --step 2.0
"""

import argparse
import json
import os
import sys
import time

import numpy as np
from scipy.optimize import least_squares

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mesh_field import FloorField, grid_xy  # noqa: E402
from fit_geometry import FRAME_ACCEPTANCE, frame_json_path  # noqa: E402

MISS_PENALTY = 2.0          # 射线未命中时的软惩罚（米）
INLIER_M = 0.30             # 判为内点的残差阈值（米）


def build_A(mirror, theta, scale, t):
    c, s = np.cos(theta), np.sin(theta)
    R = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    D = np.diag([1.0, float(mirror), 1.0])
    A = np.eye(4)
    A[:3, :3] = scale * (R @ D)
    A[:3, 3] = t
    return A


RAY_LO, RAY_HI = -400.0, 400.0


def _floor_ray(world, carla, x, y):
    """从上往下打，取第一个命中 = 最上面那个"朝上"的面。

    `world.cast_ray` 的签名是 (initial_location, final_location) 两个**绝对点**，
    没有 direction/distance 那套（那是 0.9.12 之前的旧签名），而且实测每列只返回
    一个命中，所以拿不到"最低的面"。空心的扫描网格只有地板和家具顶朝上，天花板
    朝下、从上面根本打不到，于是绝大多数列的第一命中就是地板：实测 616 列里
    357 个命中聚在 z -6.0..-5.5（地板峰），另有 55 个聚在 -3.0..-2.5，那是家具顶，
    由 sample_world_flat 的邻域平整度筛掉（桌子没几个跨得过 ±1.5·step）。
    """
    return world.cast_ray(carla.Location(x=x, y=y, z=RAY_HI),
                          carla.Location(x=x, y=y, z=RAY_LO))


def world_floor_points(host, port, center, extent, step, timeout):
    import carla
    client = carla.Client(host, port)
    client.set_timeout(timeout)
    world = client.get_world()
    xy = grid_xy(center[0], center[1], extent, step)
    pts = []
    t0 = time.time()
    for i, (x, y) in enumerate(xy):
        # 每 100 条报一次并给出速率：以前 2000 条才刷一次，一万多条射线跑起来
        # 像卡死在"射线 0/14641"，其实一直在打。
        if i % 100 == 0:
            el = time.time() - t0
            rate = i / el if el > 0.5 else 0.0
            left = (len(xy) - i) / rate if rate else 0.0
            print("  射线 %d/%d  %.0f 条/秒  命中 %d  还需 %.0f s   " % (
                i, len(xy), rate, len(pts), left), end="\r", flush=True)
        hits = _floor_ray(world, carla, x, y)
        if i == 0:
            first = time.time() - t0
            if first > 0.02:
                print("\n  单条射线 %.0f ms（复杂碰撞的 line trace 就这个量级），"
                      "%d 条预计 %.0f s" % (first * 1e3, len(xy), first * len(xy)))
        if hits:
            pts.append((x, y, hits[0].location.z))
    print("  射线 %d/%d，命中 %d   \n" % (len(xy), len(xy), len(pts)))
    if len(pts) < 50:
        # 把命中的点摊开告诉你网格到底在哪儿 —— 不然只说"命中太少"，
        # 人只能瞎猜是碰撞没开还是中心点选错了。
        a = np.asarray(pts, dtype=float)
        if len(a):
            print("world 里命中的那 %d 个点的范围：" % len(a))
            print("  x [%8.1f, %8.1f]   y [%8.1f, %8.1f]   z [%8.1f, %8.1f]" % (
                a[:, 0].min(), a[:, 0].max(), a[:, 1].min(), a[:, 1].max(),
                a[:, 2].min(), a[:, 2].max()))
            print("  命中点的 z 中位数 %.1f m（UE 里地面大概就在这个高度）" % np.median(a[:, 2]))
            print("  本次扫描范围：x[%8.1f, %8.1f] y[%8.1f, %8.1f]" % (
                xy[:, 0].min(), xy[:, 0].max(), xy[:, 1].min(), xy[:, 1].max()))
            print("  → 把 --center 往命中范围挪，并放大 --extent 盖住整块网格")
        else:
            print("一条都没打上：关卡里没有可射线的碰撞体，先确认 actor 已放进关卡且 "
                  "Collision 设为 Use Complex Collision As Simple")
        raise SystemExit("world 地板命中太少（%d），不足以标定" % len(pts))
    return np.asarray(pts, dtype=float)


def sample_world_flat(world, P, step, flat_tol):
    """用邻域射线把非平整/边缘点剔掉。返回过滤后的 (N,3)。"""
    import carla

    def z_at(x, y):
        h = _floor_ray(world, carla, x, y)
        return h[0].location.z if h else None

    out = []
    d = max(step, 0.5) * 1.5
    for i, (x, y, z) in enumerate(P):
        nb = [z_at(x + ox, y + oy) for ox, oy in
              ((d, 0), (-d, 0), (0, d), (0, -d))]
        if any(v is None for v in nb):
            continue
        if max(nb + [z]) - min(nb + [z]) > flat_tol:
            continue
        out.append((x, y, z))
    return np.asarray(out, dtype=float)


def solve(P_W, field, guesses, verbose=True):
    def resid(par, mirror):
        A = build_A(mirror, par[3], np.exp(par[4]), par[0:3])
        Ai = np.linalg.inv(A)
        q = (Ai @ np.hstack([P_W, np.ones((len(P_W), 1))]).T).T[:, :3]
        z = field.floor_z(q[:, :2])
        r = q[:, 2] - z
        r[~np.isfinite(r)] = MISS_PENALTY
        return r

    best = None
    for mirror, th0 in guesses:
        p0 = np.array([0.0, 0.0, 0.0, th0, 0.0])
        try:
            sol = least_squares(resid, p0, args=(mirror,), method="lm", max_nfev=400)
        except Exception as e:
            if verbose:
                print("  mirror=%+d 失败: %s" % (mirror, e))
            continue
        r = resid(sol.x, mirror)
        rmse = float(np.sqrt(np.mean(r ** 2)))
        if verbose:
            print("  mirror=%+d yaw=%+7.3f deg scale=%.6f t=(%7.3f,%7.3f,%6.3f) RMSE=%.4f m"
                  % (mirror, np.degrees(sol.x[3]), np.exp(sol.x[4]),
                     sol.x[0], sol.x[1], sol.x[2], rmse))
        if best is None or rmse < best[0]:
            best = (rmse, mirror, sol.x)
    if best is None:
        raise SystemExit("所有分支都求解失败")
    rmse, mirror, par = best
    A = build_A(mirror, par[3], np.exp(par[4]), par[0:3])
    # 精修：只保留内点再解一次，避免家具/边缘把解拉偏
    for _ in range(3):
        Ai = np.linalg.inv(A)
        q = (Ai @ np.hstack([P_W, np.ones((len(P_W), 1))]).T).T[:, :3]
        z = field.floor_z(q[:, :2])
        r = q[:, 2] - z
        keep = np.isfinite(r) & (np.abs(r) < INLIER_M)
        if keep.sum() < 30:
            break
        Pk = P_W[keep]

        def resid2(par2, m=mirror):
            AA = build_A(m, par2[3], np.exp(par2[4]), par2[0:3])
            qq = (np.linalg.inv(AA) @ np.hstack([Pk, np.ones((len(Pk), 1))]).T).T[:, :3]
            zz = field.floor_z(qq[:, :2])
            rr = qq[:, 2] - zz
            rr[~np.isfinite(rr)] = MISS_PENALTY
            return rr

        sol = least_squares(resid2, np.r_[par[:4], np.log(np.exp(par[4]))],
                            method="lm", max_nfev=400)
        A = build_A(mirror, sol.x[3], np.exp(sol.x[4]), sol.x[0:3])
        par = sol.x
    return A, par, keep


def metrics(A, P_W, field):
    Ai = np.linalg.inv(A)
    q = (Ai @ np.hstack([P_W, np.ones((len(P_W), 1))]).T).T[:, :3]
    z = field.floor_z(q[:, :2])
    r = q[:, 2] - z
    ok = np.isfinite(r)
    ri = r[ok]
    return {
        "n_world_points": int(len(P_W)),
        "n_matched": int(ok.sum()),
        "match_frac": float(ok.mean()),
        "rmse_z_m": float(np.sqrt(np.mean(ri ** 2))) if ok.any() else None,
        "p95_abs_z_m": float(np.percentile(np.abs(ri), 95)) if ok.any() else None,
        "inlier_frac": float(np.mean(np.abs(ri) < INLIER_M)) if ok.any() else None,
        "scale": float(np.abs(np.linalg.det(A[:3, :3])) ** (1 / 3)),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--mesh", required=True, help="扫描网格（ply/obj），必须与导出 FBX 同源同帧")
    ap.add_argument("--frame-json", default=None,
                    help="默认取 mesh 所在场景目录的 frame.json（按场景存，换扫描要重标）")
    ap.add_argument("--center", type=float, nargs=2, default=[0.0, 0.0])
    ap.add_argument("--extent", type=float, default=60.0)
    ap.add_argument("--step", type=float, default=1.0)
    ap.add_argument("--flat-tol", type=float, default=0.25)
    ap.add_argument("--timeout", type=float, default=30.0)
    ap.add_argument("--write", action="store_true", help="把解写回 frame.json")
    args = ap.parse_args()

    field = FloorField(args.mesh)
    print("网格 %s: %d 面, z[%.2f, %.2f]" % (args.mesh, field.tri_count,
                                             field.z_min, field.z_max))

    import carla
    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    world = client.get_world()
    print("关卡: %s" % world.get_settings().synchronous_mode)
    try:
        nm = world.get_map().name
    except Exception as e:
        nm = "<无 xodr: %s>" % type(e).__name__
    print("当前地图: %s" % nm)

    P = world_floor_points(args.host, args.port, args.center, args.extent,
                           args.step, args.timeout)
    print("  平整度筛选中…")
    P = sample_world_flat(world, P, args.step, args.flat_tol)
    if len(P) < 30:
        raise SystemExit("平整地板点只剩 %d 个，扩大 --extent 或放宽 --flat-tol" % len(P))
    print("  采用 %d 个 world 地板点" % len(P))

    print("分支搜索:")
    A, _, _ = solve(P, field, [(+1, 0.0), (-1, 0.0),
                               (+1, np.pi / 2), (-1, np.pi / 2)])
    m = metrics(A, P, field)
    print("\n解:")
    for k, v in m.items():
        print("  %-16s %s" % (k, ("%.4f" % v) if isinstance(v, float) else v))
    print("  A_S2W =")
    for row in A:
        print("    [% .6f, % .6f, % .6f, % .6f]" % tuple(row))

    fjp = args.frame_json or frame_json_path(args.mesh)
    acc = json.load(open(fjp)) if os.path.exists(fjp) else {}
    tol = dict(FRAME_ACCEPTANCE, **acc.get("acceptance", {}))
    bad = []
    if abs(m["scale"] - 1.0) > tol["scale_tol"]:
        bad.append("scale 偏离 1 达 %.4f —— 单位/缩放错误，检查 FBX 导出与 bConvertSceneUnit"
                   % (m["scale"] - 1.0))
    if m["rmse_z_m"] is None or m["rmse_z_m"] > tol["rmse_z_m"]:
        bad.append("rmse_z=%.4f 超标" % (m["rmse_z_m"] or -1))
    if (m["inlier_frac"] or 0) < tol["inlier_frac"]:
        bad.append("inlier_frac=%.3f 偏低" % (m["inlier_frac"] or 0))
    print("\n验收: %s" % ("通过" if not bad else "未通过 -> " + "; ".join(bad)))

    if args.write:
        acc["A_S2W"] = [[float(x) for x in row] for row in A]
        acc["calibrated"] = True
        acc["A_note"] = "calibrate_frame.py 实测解，mesh=%s, %d 个 world 地板点" % (
            os.path.abspath(args.mesh), m["n_world_points"])
        acc["A_metrics"] = m
        with open(fjp, "w") as f:
            json.dump(acc, f, indent=1)
        print("已写回 %s（记得重跑 fit_geometry.py + emit_xodr.py + validate.py）" % fjp)
    else:
        print("未写文件（加 --write 生效）；目标路径 %s" % fjp)


if __name__ == "__main__":
    main()
