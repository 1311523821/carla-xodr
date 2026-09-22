#!/usr/bin/env python3
"""traces.json（折线）-> roads_fitted_{xodr,asam}.json（line/arc/paramPoly3 解析几何链）。

按目标帧各拟合一次，**不做参数移植**：镜像会交换车道左右侧，移植极易放错边。

C0 衔接是构造性的：每段起点 (x,y,hdg) 直接取上一段算出的终点，本段只优化
length 与内部系数，所以接缝误差恒为 0（除浮点）。每段先试 line -> arc ->
paramPoly3，残差超阈值就在最大偏差处二分递归，直到达标或到深度上限。
"""

import argparse
import json
import math
import os
import sys

import numpy as np
from scipy.signal import savgol_filter

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from xodr_geom import Geometry, Poly3, XODR2WORLD, invert_rotate  # noqa: E402

M_XODR2WORLD = np.array(XODR2WORLD, dtype=float)
M_XODR2WORLD4 = np.eye(4)
M_XODR2WORLD4[:3, :3] = M_XODR2WORLD

DS = 0.25                 # 重采样步长（米）
HEAD_WIN = 3.0            # heading 局部二次拟合窗口（米）
KAPPA_WIN = 15.0          # 曲率 Savitzky-Golay 窗口（米）
R_MIN = 15.0              # 初始分段阈值：|kappa| 越过 1/R_MIN 就切段
FIT_TOL = 0.05            # 单段拟合对源折线的最大允许偏差（米）
DEV_STEP = 0.02           # 偏差度量的采样步长：离散化下限 = DEV_STEP/2
SMOOTH_LEN = 2.0          # 平滑核的弧长（米）
SMOOTH_MAX_DISP = 0.30    # 允许的最大挪线量（米），超出即报错而不是静默改线
MAX_SMOOTH_PASSES = 40    # 拟合驱动平滑的迭代上限
MIN_SEG_PTS = 6           # 少于此点数不再细分
MAX_DEPTH = 7
ELEV_STEP = 15.0          # 高程节点间隔（米）


def load_json(path):
    with open(path) as f:
        return json.load(f)


# 验收阈值。以前只写在仓库根那份 frame.json 里，而 calibrate_frame 和
# check_alignment 各自还有一份不一致的代码兜底值（0.10 / 0.15）；frame.json 改成
# 按场景存之后，新场景可能根本没有这个文件，阈值必须有个唯一的出处。
FRAME_ACCEPTANCE = {"rmse_z_m": 0.10, "p95_xy_m": 0.25,
                    "inlier_frac": 0.90, "scale_tol": 0.001}


def frame_json_path(artifact, fallback="frame.json"):
    """这份场景的 frame.json —— 从任意场景产物路径定位它所在的 `cache/<场景>/`。

    `A_S2W` 是"这一份扫描 -> world"的刚体，换一张扫描就得重标。以前全局一份，
    第二个场景标定完会把第一个的解悄悄覆盖掉，而两边生成的 xodr 都还留在磁盘上。
    显式传了 `--frame-json` 就用传的，这里只管默认值怎么来。
    """
    d = os.path.dirname(os.path.abspath(artifact))
    if os.path.basename(d) == "out":
        d = os.path.dirname(d)
    return os.path.join(d, "frame.json") if os.path.isdir(d) else fallback


def derive_T_S2X(frame):
    """发射进 CARLA 那份 xodr 的帧变换 S -> X。

    A_S2W 是实测的 Blender/扫描帧 -> CARLA world 刚体（含镜像）；
    XODR2WORLD 是 CARLA 内建的 xodr -> world 镜像，故 T_S2X = M^-1 @ A = M @ A。
    未标定时 A_S2W 取 M（UE bConvertScene 只做一次手性翻转）=> T_S2X = 单位，
    即"CARLA 版与 ASAM 版数值相同"——标定残差就是两者的差异。
    """
    A = np.asarray(frame["A_S2W"], dtype=float)
    return (M_XODR2WORLD4 @ A).tolist()


def apply_frame(pts, T):
    if T is None:
        return pts
    M = np.asarray(T, dtype=float)
    return [tuple(M @ np.array([p[0], p[1], p[2], 1.0]))[:3] for p in pts]


def smooth_once(pts, win=SMOOTH_LEN):
    """拉普拉斯平滑**单遍**：interior 点移向 ±win/2 弧长处两邻居的中点，端点固定。

    只走单遍、由拟合结果驱动迭代次数（见 build）。一次性迭代几十遍会把
    真实存在的圆弧也熨平——曲率扩散必然收缩，21 m 的 60 度弧会被挪走 0.7 m。
    """
    a = np.asarray(pts, dtype=float)
    if len(a) < 5:
        return a, 0.0
    seg = np.hypot(np.diff(a[:, 0]), np.diff(a[:, 1]))
    s = np.concatenate([[0.0], np.cumsum(seg)])
    half = win / 2.0
    idx = np.arange(len(a), dtype=float)
    li = np.interp(s - half, s, idx)
    ri = np.interp(s + half, s, idx)
    b = a.copy()
    for i in range(1, len(a) - 1):
        l0, l1 = int(math.floor(li[i])), int(math.ceil(li[i]))
        r0, r1 = int(math.floor(ri[i])), int(math.ceil(ri[i]))
        if r0 <= l1:
            continue
        lo = a[l0] + (li[i] - l0) * (a[l1] - a[l0])
        ro = a[r0] + (ri[i] - r0) * (a[r1] - a[r0])
        b[i] = a[i] + 0.5 * ((lo + ro) / 2.0 - a[i])
    b[0], b[-1] = a[0], a[-1]
    shift = float(np.max(np.hypot(b[:, 0] - a[:, 0], b[:, 1] - a[:, 1])))
    b = resample([tuple(p) for p in b], DS)
    return np.asarray(b, dtype=float), shift


def _disp(a, b):
    """原始折线到平滑后折线的最大横向挪动量。"""
    worst = 0.0
    for p in a:
        d = np.hypot(b[:, 0] - p[0], b[:, 1] - p[1])
        worst = max(worst, float(d.min()))
    return worst


def resample(pts, ds=DS):
    a = np.asarray(pts, dtype=float)
    seg = np.hypot(a[1:, 0] - a[:-1, 0], a[1:, 1] - a[:-1, 1])
    s = np.concatenate([[0.0], np.cumsum(seg)])
    total = s[-1]
    if total <= ds:
        return [tuple(p) for p in a]
    n = int(math.floor(total / ds)) + 1
    t = np.linspace(0.0, total, n)
    return [tuple(np.interp(t, s, a[:, k]) for k in range(3)) for t in t]


def _local_hdg(pts, i, win):
    """窗口内二次拟合求切向。方向符号取自窗口整体位移，否则在 |dy|>|dx|
    的分支上会返回反向 180 度的 heading。"""
    r = max(int(win / DS), 2)
    lo, hi = max(0, i - r), min(len(pts), i + r + 1)
    sub = np.asarray(pts[lo:hi], dtype=float)
    span = sub[-1, :2] - sub[0, :2]
    if len(sub) < 3:
        return math.atan2(span[1], span[0])
    if abs(span[0]) >= abs(span[1]):
        c = np.polyfit(sub[:, 0], sub[:, 1], 2)
        d = np.array([1.0, 2.0 * c[0] * sub[i - lo, 0] + c[1]])
        if span[0] < 0:
            d = -d
    else:
        c = np.polyfit(sub[:, 1], sub[:, 0], 2)
        d = np.array([2.0 * c[0] * sub[i - lo, 1] + c[1], 1.0])
        if span[1] < 0:
            d = -d
    return math.atan2(d[1], d[0])


def headings(pts, win=HEAD_WIN):
    return np.array([_local_hdg(pts, i, win) for i in range(len(pts))])


def curvatures(h, ds=DS):
    hu = np.unwrap(h)
    k = np.gradient(hu, ds, edge_order=2)
    w = max(int(KAPPA_WIN / ds) | 1, 5)
    w = min(w, len(k) if len(k) % 2 else len(k) - 1)
    if w < 5:
        return k
    return savgol_filter(k, w, 2)


def initial_segments(k, ds=DS, r_min=R_MIN):
    thr = 1.0 / r_min
    straight = np.abs(k) < thr
    bounds = [0] + [i for i in range(1, len(k)) if straight[i] != straight[i - 1]] + \
        [len(k) - 1]
    return [(a, b) for a, b in zip(bounds[:-1], bounds[1:]) if b - a >= 2]


# ---------------------------------------------------------------- 单段拟合 --
def _chord(pts):
    return sum(math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
               for i in range(len(pts) - 1))


def _sample_xy(g, n):
    pts = [g.at(g.length * i / n) for i in range(n + 1)]
    return np.array([(p.x, p.y) for p in pts])


def _max_deviation(g, pts):
    """源点到拟合曲线的最大偏差（米）及其源点下标。

    采样必须远密于源折线，否则量到的是采样离散步长而不是拟合误差。
    """
    if g.length <= 0:
        return float("inf"), 0
    arr = _sample_xy(g, max(int(math.ceil(g.length / DEV_STEP)), 2))
    q = np.asarray(pts, dtype=float)
    d = np.hypot(q[:, None, 0] - arr[None, :, 0], q[:, None, 1] - arr[None, :, 1]).min(axis=1)
    i = int(d.argmax())
    return float(d[i]), i


def _deviation(g, pts):
    return _max_deviation(g, pts)[0]


def _fit_arc_kappa(p0, h0, pts):
    """圆心受限于 p0 + (1/k)*local_v(h0)（CARLA 的局部 +v 方向），一维搜最优 k。"""
    vdir = (-math.sin(h0), math.cos(h0))

    def cost(k):
        r = 1.0 / k
        cx, cy = p0[0] + r * vdir[0], p0[1] + r * vdir[1]
        return sum((math.hypot(p[0] - cx, p[1] - cy) - abs(r)) ** 2 for p in pts)

    grid = np.linspace(-0.33, 0.33, 661)
    grid = grid[np.abs(grid) > 1e-4]
    best = min(grid, key=cost)
    lo, hi = best - 1e-3, best + 1e-3
    for _ in range(50):
        m1, m2 = lo + (hi - lo) / 3.0, hi - (hi - lo) / 3.0
        if cost(m1) < cost(m2):
            hi = m2
        else:
            lo = m1
    return 0.5 * (lo + hi)


def _arc_sweep(p0, h0, kap, pend):
    vdir = (-math.sin(h0), math.cos(h0))
    r = 1.0 / kap
    cx, cy = p0[0] + r * vdir[0], p0[1] + r * vdir[1]
    d = math.atan2(pend[1] - cy, pend[0] - cx) - math.atan2(p0[1] - cy, p0[0] - cx)
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


def _fit_parampoly3(p0, h0, pts):
    """目标点反解进本段局部系，对**米制弦长参数** p 三次最小二乘拟合 u(p), v(p)。

    aU/aV 归零 => 曲线精确从局部原点出发 => C0 由构造保证。
    pRange 用 arcLength，故 p 必须以米为单位、终点为 L。
    """
    loc = [invert_rotate(h0, p[0] - p0[0], p[1] - p0[1]) for p in pts]
    u = np.array([q[0] for q in loc])
    v = np.array([q[1] for q in loc])
    p = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(u), np.diff(v)))])
    L = float(p[-1])
    if L <= 1e-6:
        return None
    A = np.vstack([np.ones_like(p), p, p ** 2, p ** 3]).T
    cu = np.linalg.lstsq(A, u, rcond=None)[0]
    cv = np.linalg.lstsq(A, v, rcond=None)[0]
    return Poly3(0.0, cu[1], cu[2], cu[3]), Poly3(0.0, cv[1], cv[2], cv[3]), L


def _candidates(s0, x0, y0, h0, pts):
    out = []
    fwd = np.array([math.cos(h0), math.sin(h0)])
    d = np.array([pts[-1][0] - x0, pts[-1][1] - y0])
    ln = float(np.dot(d, fwd))
    if ln > 1e-3:
        out.append(Geometry(s0, x0, y0, h0, ln, "line"))
    if len(pts) >= 4:
        kap = _fit_arc_kappa((x0, y0), h0, pts)
        if abs(kap) > 1e-4:
            sw = _arc_sweep((x0, y0), h0, kap, pts[-1])
            if sw * kap > 0 and abs(sw) < math.pi:
                out.append(Geometry(s0, x0, y0, h0, abs(sw) / abs(kap), "arc",
                                    curvature=kap))
        pp = _fit_parampoly3((x0, y0), h0, pts)
        if pp:
            pu, pv, L = pp
            out.append(Geometry(s0, x0, y0, h0, L, "paramPoly3", pu=pu, pv=pv))
    return out


def fit_road(pts3d):
    xy = [(p[0], p[1]) for p in pts3d]
    h = headings(pts3d)
    k = curvatures(h)
    segs = initial_segments(k)
    geoms, worst = [], 0.0
    s, st = 0.0, (xy[0][0], xy[0][1], float(h[0]))
    for (a, b) in segs:
        gs, st, dev = _recurse(s, st[0], st[1], st[2], xy[a:b + 1], 1)
        for g in gs:
            g.s = s
            s += g.length
        geoms += gs
        worst = max(worst, dev)
    if not segs or segs[-1][1] != len(xy) - 1:
        tail = xy[-2:] if segs else xy
        gs, st, dev = _recurse(s, st[0], st[1], st[2], tail, MAX_DEPTH)
        for g in gs:
            g.s = s
            s += g.length
        geoms += gs
        worst = max(worst, dev)
    return geoms, worst


def _recurse(s0, x0, y0, h0, pts, depth):
    cands = _candidates(s0, x0, y0, h0, pts)
    if not cands:
        return [], (x0, y0, h0), float("inf")
    g, dev = min(((g, _deviation(g, pts)) for g in cands), key=lambda t: t[1])
    if dev <= FIT_TOL or len(pts) < MIN_SEG_PTS or depth >= MAX_DEPTH:
        e = g.end()
        return [g], (e.x, e.y, e.hdg), dev
    _, worst_i = _max_deviation(g, pts)
    cut = min(max(worst_i, 3), len(pts) - 3)
    left, lend, ldev = _recurse(s0, x0, y0, h0, pts[:cut + 1], depth + 1)
    right, rend, rdev = _recurse(0.0, lend[0], lend[1], lend[2], pts[cut:], depth + 1)
    return left + right, rend, max(ldev, rdev)


def _elev_once(s, z, dz, total, step):
    n = max(int(math.ceil(total / step)), 1)
    knots = sorted(set([0.0] + [min(total, step * i) for i in range(1, n)] + [total]))
    out, max_res = [], 0.0
    for k0, k1 in zip(knots[:-1], knots[1:]):
        L = k1 - k0
        if L <= 1e-9:
            continue
        z0, z1 = float(np.interp(k0, s, z)), float(np.interp(k1, s, z))
        m0 = float(np.interp(k0, s, dz)) * L
        m1 = float(np.interp(k1, s, dz)) * L
        b = m0 / L
        c = (3.0 * (z1 - z0) - 2.0 * m0 - m1) / (L * L)
        d = (2.0 * (z0 - z1) + m0 + m1) / (L ** 3)
        out.append((k0, z0, b, c, d))
        t = np.linspace(0.0, L, 25)
        fit = z0 + b * t + c * t ** 2 + d * t ** 3
        m = (s >= k0) & (s <= k1)
        if m.any():
            max_res = max(max_res, float(np.max(np.abs(
                z[m] - np.interp(s[m], t + k0, fit)))))
    return out, max_res


def fit_elevation(pts3d, step=ELEV_STEP):
    """z(s) 分段三次，节点处用 (z, dz/ds) Hermite -> 幂式，保证 C1。
    残差超阈值就加密节点。"""
    a = np.asarray(pts3d, dtype=float)
    seg = np.hypot(a[1:, 0] - a[:-1, 0], a[1:, 1] - a[:-1, 1])
    s = np.concatenate([[0.0], np.cumsum(seg)])
    total = float(s[-1])
    z = a[:, 2]
    if total <= 1e-9:
        return [(0.0, float(z[0]), 0.0, 0.0, 0.0)], 0.0
    dz = np.gradient(z, s, edge_order=2)
    st = step
    while True:
        out, res = _elev_once(s, z, dz, total, st)
        if res <= FIT_TOL or st <= 2.0 or not out:
            if not out:
                out = [(0.0, float(z[0]), 0.0, 0.0, 0.0)]
            return out, res
        st /= 2.0


def fill_missing_z(points, rid):
    """把 z_missing 的点按 s 从有值的邻居线性插值补齐。

    这个标记的含义是"这一点上没量到地板"（描线落在空洞/门口），trace_editor 的
    enrich 会把它当成 z=0.0 写进存档。0.0 不是它真实的高度，直接喂进高程拟合的
    后果是实测一条平地上 -5.58 m 的路，因为一个空洞点，残差从 0.0000 顶到 5.5800 m，
    然后在 validate 的 <=0.05 断言上硬失败 —— 而这个标记存在的意义恰恰是处理它。

    返回 (pts, n_filled)。一个有效点都没有时明确报错，而不是静默出一份 z=0 的图。
    """
    n = len(points)
    if not n or not any(p.get("z_missing") for p in points):
        return points, 0
    ok = [i for i, p in enumerate(points) if not p.get("z_missing")]
    if not ok:
        raise SystemExit("road %d 所有点的 z 都没量到（描线落在无地板处？），"
                         "无法拟合高程 —— 请把线描在有地板的地方" % rid)
    ss = np.array([float(p["s"]) for p in points], dtype=float)
    zs = np.array([float(p["z"]) for p in points], dtype=float)
    filled = np.interp(ss, ss[ok], zs[ok])
    out = []
    nf = 0
    for i, p in enumerate(points):
        if p.get("z_missing"):
            q = dict(p)
            q["z"] = float(filled[i])
            q.pop("z_missing", None)
            out.append(q)
            nf += 1
        else:
            out.append(p)
    return out, nf


def build(traces, T, name, max_disp=SMOOTH_MAX_DISP):
    roads = []
    for r in sorted(traces["roads"], key=lambda x: x["id"]):
        src, n_zfill = fill_missing_z(r["points"], r["id"])
        pts = apply_frame([(p["x"], p["y"], p["z"]) for p in src], T)
        rp = np.asarray(resample(pts), dtype=float)
        orig = rp.copy()
        for _ in range(MAX_SMOOTH_PASSES):
            geoms, dev = fit_road(rp)
            if not geoms or dev <= FIT_TOL:
                break
            rp, _ = smooth_once(rp)
        else:
            geoms, dev = fit_road(rp)
        disp = _disp(orig, rp)
        if disp > max_disp:
            raise SystemExit("road %d 平滑挪线 %.2f m 超过上限 %.2f m —— 描线拐角太急。"
                             "人工描线不该触发这条：请补点后重描，"
                             "或显式加大 --max-smooth-disp 并接受挪线结果" % (
                                 r["id"], disp, max_disp))
        if not geoms:
            raise SystemExit("road %d 拟合失败：无有效几何段" % r["id"])
        elev, zres = fit_elevation(rp)
        total = sum(g.length for g in geoms)
        roads.append({
            "id": r["id"],
            "name": r.get("name", "road_%04d" % r["id"]),
            "frame": name,
            "length": total,
            "width_left": float(r.get("width_left", 3.5)),
            "width_right": float(r.get("width_right", 3.5)),
            "speed_kmh": float(r.get("speed_kmh", 5.0)),
            "link_next": r.get("link_next"),
            "junction_in": r.get("junction_in"),
            "junction_out": r.get("junction_out"),
            "geometry": [{
                "s": g.s, "x": g.x, "y": g.y, "hdg": g.hdg, "length": g.length,
                "kind": g.kind, "curvature": g.curvature,
                "pu": None if g.pu is None else [g.pu.a, g.pu.b, g.pu.c, g.pu.d],
                "pv": None if g.pv is None else [g.pv.a, g.pv.b, g.pv.c, g.pv.d],
            } for g in geoms],
            "elevation": [{"s": e[0], "a": e[1], "b": e[2], "c": e[3], "d": e[4]}
                          for e in elev],
            "quality": {
                "n_geom": len(geoms),
                "kinds": {kk: sum(1 for g in geoms if g.kind == kk)
                          for kk in ("line", "arc", "paramPoly3")},
                "max_fit_deviation_m": dev,
                "max_smoothing_disp_m": disp,
                "max_elevation_residual_m": zres,
                "n_z_filled": n_zfill,
                "source_length_m": _chord([(p[0], p[1]) for p in rp]),
                "chain_length_m": total,
            },
        })
    return {"frame": name, "roads": roads}


def geom_from_dict(d):
    kind = d["kind"]
    pu = pv = None
    if kind == "paramPoly3":
        pu, pv = Poly3(*d["pu"]), Poly3(*d["pv"])
    return Geometry(d["s"], d["x"], d["y"], d["hdg"], d["length"], kind,
                    curvature=d.get("curvature", 0.0), pu=pu, pv=pv)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--traces", default="traces.json")
    ap.add_argument("--frame-json", default=None,
                    help="默认取 traces 所在场景目录的 frame.json")
    ap.add_argument("--out-dir", default="out")
    ap.add_argument("--max-smooth-disp", type=float, default=SMOOTH_MAX_DISP,
                    help="允许平滑挪线的上限（米）；网格搜索出来的锯齿线需要很大挪量，"
                         "人工描线一般用不到放宽")
    args = ap.parse_args()

    traces = load_json(args.traces)
    fjp = args.frame_json or frame_json_path(args.traces)
    T = None
    if os.path.exists(fjp):
        T = derive_T_S2X(load_json(fjp))

    os.makedirs(args.out_dir, exist_ok=True)
    # 两份都出，**不因为缺 frame.json 就跳过 xodr 那份**：未标定时 T_S2X 退化成单位阵，
    # 即"CARLA 版与 ASAM 版数值相同"（见 derive_T_S2X），这正是给 CARLA 的那份最该
    # 存在的场合。编辑器 /api/emit 一直是无条件 build 两次，这里以前写成
    # `if T is not None`，于是 README 的「一键跑通」（demo 没有 frame.json）会在
    # emit_xodr 找不到 roads_fitted_xodr.json 时崩掉，而网页路径完全正常。
    jobs = [("xodr", T), ("asam", None)]
    for name, tf in jobs:
        out = build(traces, tf, name, args.max_smooth_disp)
        path = os.path.join(args.out_dir, "roads_fitted_%s.json" % name)
        with open(path, "w") as f:
            json.dump(out, f, indent=1)
        print("wrote %s" % path)
        bad = 0
        for r in out["roads"]:
            q = r["quality"]
            flag = "" if q["max_fit_deviation_m"] <= FIT_TOL else "  <-- 超差"
            if flag:
                bad += 1
            print("  road %-3d len=%7.2f  src=%7.2f  n=%2d %-30s dev=%.4f 挪线=%.3f zres=%.4f%s"
                  % (r["id"], r["length"], q["source_length_m"], q["n_geom"],
                     str(q["kinds"]), q["max_fit_deviation_m"],
                     q["max_smoothing_disp_m"], q["max_elevation_residual_m"], flag))
        if bad:
            print("  !! %d 条路拟合超差 (>%.2f m)" % (bad, FIT_TOL))


if __name__ == "__main__":
    main()
