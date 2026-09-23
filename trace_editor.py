#!/usr/bin/env python3
"""浏览器版描线编辑器（Flask + canvas）。

设计原则：**几何判定一律在服务端复用既有模块**（mesh_field / probe_clearance /
fit_geometry / emit_xodr），JS 只负责产生 x,y 点。绝不在浏览器里重写一套逻辑，
否则两套实现迟早不一致，那类 bug 最难查。

  python3 trace_editor.py [--port 8071]

启动时**不加载任何场景**：浏览器打开 http://127.0.0.1:8071 后点「加载数据」，
在对话框里翻到并选一个扫描 FBX（默认从家目录开始翻，`--scan-root` 只用来圈定
能浏览的范围），它才是这个工具唯一的场景输入。选中的 FBX 会烘出
一份带纹理的 GLB 缓存在 `cache/<名字>/scene.glb`（mtime 没变就跳过重烘），
浏览器显示它，服务端的地板高度场和遮挡判定也读同一个文件 —— 一份几何，
不存在"看的"和"吸附的"对不上。描完点「生成 xodr」得到 .xodr。
"""

import argparse
import glob
import hashlib
import io
import json
import math
import os
import shlex
import shutil
import subprocess
import sys
import threading
import time
import traceback

import numpy as np
from flask import Flask, jsonify, request, send_from_directory

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import emit_xodr                      # noqa: E402
import check_alignment                # noqa: E402
import fit_geometry as FG             # noqa: E402
import glb_shrink                     # noqa: E402
import validate                       # noqa: E402
from mesh_field import FloorField     # noqa: E402
from probe_clearance import probe_line  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
# 默认 CARLA 根目录。它只是**默认值**，优先级：面板存的 > --carla-root > $CARLA_ROOT > 这里。
# 名字里不要带 IMPORT：它当 carla_root 整体用，不是"Import 的父目录"。
CARLA_ROOT_DEFAULT = os.path.expanduser("~/carla")
STATIC = os.path.join(HERE, "editor_static")
PPM = 20.0                    # 底图分辨率：像素/米
COL_LAYERS = 8                # 每列最多取几个表面
CLEAR_MIN = 1.50
FLAT_TOL = 0.15
PAD_M = 3.0                   # 底图四周留白（米）

app = Flask(__name__, static_folder=STATIC)
field = None
STATE = {"fbx": None, "cache": None, "xmin": 0.0, "ymin": 0.0, "w": 1, "h": 1,
         "traces_path": None}      # 载入场景时才定，默认落在该场景自己的目录里

# 载入是半分钟量级的后台活（Blender 烘 GLB + 缩纹理 + 射线场 + 底图），
# 所以阶段要能被前端轮询到，而不是转一个圈糊过去。
PHASES = ["烘焙 GLB", "建射线场", "生成底图"]
PROG = {"state": "idle", "fbx": None, "phase": -1, "detail": "",
        "t0": 0.0, "error": None, "marks": {}, "end": 0.0}
LOAD_LOCK = threading.Lock()
SCANS = {"root": None, "cache": None, "traces": None}


def prog(state, phase=None, detail=None):
    """记录阶段和每阶段耗时 —— 前端要显示"烘焙 27.7s、射线场 9.3s"，
    而不是一个不知道在干什么的转圈。"""
    with LOAD_LOCK:
        el = round(time.time() - PROG["t0"], 1) if PROG["t0"] else 0.0
        prev = PROG["phase"]
        if phase is not None and phase != prev:
            if 0 <= prev < len(PHASES):
                PROG["marks"][prev] = el
            PROG["phase"] = phase
        if state in ("done", "error") and 0 <= prev < len(PHASES):
            PROG["marks"][prev] = el
            PROG["end"] = el
        PROG["state"] = state
        if detail is not None:
            PROG["detail"] = detail


# ------------------------------------------------------- FBX -> 内部资产 --
def find_blender(cfg):
    """Blender 不在 PATH 里是常态，按 设置面板 > 环境变量 > which > 家目录 找。
    最后那层通配只是"本机解压成 ~/blender-*-linux-x64 时能自动中"的便利，
    换台机器不成立，所以真找不到就明说去哪儿填，而不是猜一个路径。"""
    for p in (cfg.get("blender"), os.environ.get("BLENDER"), shutil.which("blender")):
        if p and os.path.exists(cfg_path(p)):
            return cfg_path(p)
    hits = sorted(glob.glob(os.path.expanduser("~/blender-*-linux-x64/blender")))
    if hits:
        return hits[-1]
    raise SystemExit("找不到 Blender 可执行文件 —— 在页面「⚙ 路径」的 Blender 一栏填绝对路径"
                     "（留空则依次试 $BLENDER、which blender）")



def report_box(glb):
    """把 GLB 顶点自己的框报给前端，顺手做一次轴向体检。

    今天两次翻车（导入器把扫描躺倒；另一份被导成 (4y,4x,-4z)，即 X/Y 对调 +
    Z 翻转 + 放大 4 倍）在浏览器里都看不出来，一路导完 xodr、开进 UE 才发现。
    躺倒是能查的：扫描的高度轴必须是 Z，而任何地面扫描的 Z 跨度都不会大过水平
    两轴。缩放对不对在只有 FBX 的前提下无法独立判定（没有源 obj 可对账），
    所以把三个数显示出来让人看一眼 —— 30 m 的办公室写成 143 m 是很扎眼的。
    """
    mn, mx = glb_shrink.axis_box(glb)
    s = [round(mx[i] - mn[i], 3) for i in range(3)]
    STATE["box"] = s
    txt = "网格 %.1f × %.1f × %.1f m（X/Y 水平，Z 是高度）" % tuple(s)
    if s[2] > min(s[0], s[1]):
        txt += "  ⚠ Z 比水平两轴还大，这份 FBX 是躺倒的，别往下走"
    prog("running", 0, txt)
    print("  " + txt)
    return s


def bake(fbx, cache, cfg):
    """从 FBX 烘出唯一的场景资产 scene.glb。显示层直接拿它渲染，服务端的
    地板高度场/遮挡判定也用 open3d 读同一个文件，所以不存在"两份几何对不上"。"""
    maxtex = int(cfg["maxtex"])
    glb = os.path.join(cache, "scene.glb")
    if os.path.exists(glb) and os.path.getmtime(glb) > os.path.getmtime(fbx):
        big = [s for s in glb_shrink.image_sizes(glb) if max(s) > maxtex]
        nodes = glb_shrink.node_transforms(glb)
        if not big and not nodes:
            prog("running", 0, "缓存命中，跳过烘焙")
            print("缓存命中（FBX 未改动），跳过烘焙")
            report_box(glb)
            return glb
        if nodes:
            # 带物体缩放的 FBX 以前会烘出非恒等节点，显示和射线场差一个倍数，
            # 这种缓存不能要，重烘。
            print("缓存里节点带非恒等变换 %s，重烘" % nodes[:3])
            prog("running", 0, "缓存节点变换非恒等，重烘")
        else:
            # 缓存是旧版烘焙脚本产的（纹理没缩过）：就地补缩，不用重跑 Blender
            prog("running", 0, "缓存里 %d 张纹理超限，补缩" % len(big))
            done, b1, b2 = glb_shrink.shrink(glb, maxtex)
            prog("running", 0, "补缩 %d 张：%.1f MB -> %.1f MB" % (
                done, b1 / 1e6, b2 / 1e6))
            report_box(glb)
            return glb
    blender_bin = find_blender(cfg)         # 只有真要烘时才要求找得到 Blender
    os.makedirs(cache, exist_ok=True)
    cmd = [blender_bin, "--background", "--python",
           os.path.join(HERE, "blender", "prepare_mesh.py"), "--",
           "--fbx", fbx, "--glb", glb]
    print("烘焙 %s ->\n  %s\n%s" % (fbx, glb, " ".join(cmd)))
    t0 = time.time()
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                         text=True, bufsize=1)
    tail = []
    for ln in p.stdout:                        # 逐行读，才能把阶段实时报给前端
        ln = ln.rstrip()
        tail.append(ln)
        if ln.startswith(("导入", "写出", "同帧")):
            prog("running", 0, ln.strip())
            print("  " + ln)
    rc = p.wait()
    if rc:
        raise SystemExit("Blender 烘焙失败 (%d):\n%s" % (rc, "\n".join(tail[-40:])))
    # 缩小必须在 Blender 外面做：Image.scale() 只改内存缓冲，导出器按原始分辨率
    # 打包，Chromium 扛得住 2.5 GB 纹理而 Firefox 会把分配失败的材质画成白色。
    prog("running", 0, "缩小纹理到 %d px" % maxtex)
    done, b1, b2 = glb_shrink.shrink(glb, maxtex)
    prog("running", 0, "纹理 %d 张超限已缩小：%.1f MB -> %.1f MB，GLB 现 %.1f MB" % (
        done, b1 / 1e6, b2 / 1e6, os.path.getsize(glb) / 1e6))
    print("  " + "纹理 %d 张超限已缩小：%.1f MB -> %.1f MB" % (done, b1 / 1e6, b2 / 1e6))
    nodes = glb_shrink.node_transforms(glb)
    if nodes:
        raise SystemExit("烘出来的 GLB 节点仍带非恒等变换 %s —— open3d 不应用节点变换、"
                         "three.js 会应用，显示和射线场会差一个倍数" % nodes[:4])
    print("  节点变换校验：全部恒等 ✓" )
    report_box(glb)
    print("  耗时 %.1fs" % (time.time() - t0))
    return glb


# ---------------------------------------------------------------- 载入场景 --
def cache_dir_for(fbx):
    """一个场景一个目录，目录名 = **FBX 文件名**（不带扩展名），不经过 scan-root。

    以前用"相对扫描根的路径把 / 换成 __"，于是换一个 --scan-root，同一份扫描就
    映射到另一个目录：描的线和标定解全都"看不见"了，而工具不报错，只是在新目录里
    重新烘一份空的 —— 你会以为线丢了，然后重描一遍，磁盘上就留下两份 traces.json。

    同名不同文件（`test.fbx` 和 `model/test.fbx`）靠目录里的 `source` 认领文件区分：
    谁先占了这个名字谁用干净的 `test/`，后来者退到 `test__<路径哈希前6位>/`。
    同一个文件路径永远算出同一个目录，所以换 scan-root 不再改变归属。
    """
    ab = os.path.abspath(fbx)
    base = os.path.splitext(os.path.basename(ab))[0]
    cand = os.path.join(SCANS["cache"], base)
    claim = os.path.join(cand, "source")
    if os.path.isfile(claim):
        with open(claim) as f:
            got = f.read().strip()
        if got and got != ab:
            cand = os.path.join(SCANS["cache"], base + "__"
                                + hashlib.sha1(ab.encode()).hexdigest()[:6])
    return cand


def claim_scene(cache, fbx):
    """把"这个目录属于哪个 FBX"记下来，供 cache_dir_for 区分同名不同文件。"""
    with open(os.path.join(cache, "source"), "w") as f:
        f.write(os.path.abspath(fbx) + "\n")



def migrate_legacy_traces(dest, fbx):
    """老版本 traces.json 是全局一份，换场景会串台。若那份老文件确实属于本场景，
    搬进场景目录，并把原件改名留底。

    必须**搬**不能**复制**：复制过一次之后，只要场景目录里的存档被删，下次加载又会
    从根目录再复制一份回来 —— 用户删了 cache 却发现路又出现，就是这么复活的。
    """
    legacy = os.path.join(HERE, "traces.json")
    if not os.path.isfile(legacy) or os.path.exists(dest):
        return
    if os.path.abspath(dest) == os.path.abspath(legacy):
        return
    try:
        with open(legacy) as f:
            doc = json.load(f)
    except (ValueError, OSError):
        return
    if doc.get("scene") != fbx or not doc.get("roads"):
        return
    os.makedirs(os.path.dirname(dest), exist_ok=True)
    shutil.move(legacy, dest)
    print("根目录的老 traces.json 已搬进 %s（原位置不再留副本，删了就真删了）" % dest)


def load_worker(fbx):
    """后台线程跑完三步，阶段实时写进 PROG 给前端轮询。
    半分钟的活只给个转圈，人是会以为卡死的。"""
    global field
    try:
        cache = cache_dir_for(fbx)
        os.makedirs(cache, exist_ok=True)
        claim_scene(cache, fbx)           # 先认领，烘焙失败也不改归属：路径没变
        glb = bake(fbx, cache, load_config())
        prog("running", 1, "open3d 解析 GLB 三角面")
        f = FloorField(glb)
        traces = SCANS["traces"] or os.path.join(cache, "traces.json")
        migrate_legacy_traces(traces, fbx)
        STATE.update(loaded=False, fbx=fbx, cache=cache, traces_path=traces)
        field = f                       # init_basemap 用全局 field
        prog("running", 2, "地板高度 + 净空射线")
        init_basemap()
        STATE["loaded"] = True
        prog("done", len(PHASES), "%d 面，z[%.2f, %.2f]" % (
            f.tri_count, f.z_min, f.z_max))
        print("已载入 %s" % fbx)
    except (SystemExit, Exception) as e:      # SystemExit 也来自 find_blender/bake
        with LOAD_LOCK:
            PROG.update(state="error", error="%s: %s" % (type(e).__name__, e)
                        if not isinstance(e, SystemExit) else str(e))
        print("载入失败:\n%s" % traceback.format_exc(), file=sys.stderr)


def inside_root(p):
    """只允许浏览/加载 --scan-root 之内的路径：否则等于让浏览器把这台机器上
    任意文件交给 Blender 去解析。"""
    root = os.path.realpath(SCANS["root"])
    rp = os.path.realpath(p)
    return rp == root or rp.startswith(root + os.sep)


def scan_list(d):
    dirs, files = [], []
    with os.scandir(d) as it:
        for e in sorted(it, key=lambda x: x.name.lower()):
            if e.name.startswith("."):
                continue
            if e.is_dir(follow_symlinks=False):
                if inside_root(e.path):
                    dirs.append(e.name)
            elif e.name.lower().endswith(".fbx") and e.is_file(follow_symlinks=False):
                st = e.stat()
                files.append({"name": e.name, "bytes": st.st_size,
                              "mtime": int(st.st_mtime)})
    parent = os.path.dirname(os.path.normpath(d))
    return {"dir": d, "root": SCANS["root"],
            "parent": parent if parent != d and inside_root(parent) else None,
            "dirs": dirs, "fbx": files}


def not_loaded():
    return jsonify({"error": "还没有加载场景，先点「加载数据」选一个 FBX"}), 409


# --------------------------------------------------------------- 底图渲染 --
def _ramp(z, lo, hi):
    """terrain 风格三段渐变，纯 numpy，不依赖 matplotlib colormap。"""
    t = np.clip((z - lo) / max(hi - lo, 1e-6), 0.0, 1.0)
    stops = [(0.00, 0.16, 0.36, 0.62), (0.35, 0.20, 0.62, 0.66),
             (0.60, 0.42, 0.76, 0.40), (0.80, 0.86, 0.82, 0.60),
             (1.00, 0.96, 0.96, 0.96)]
    rgb = np.zeros(t.shape + (3,))
    for i in range(3):
        xs = [s[0] for s in stops]
        ys = [s[i + 1] for s in stops]
        rgb[..., i] = np.interp(t, xs, ys)
    return rgb


def init_basemap():
    """预渲染地板高度 + 净空禁区，缓存成 PNG；同时确定像素<->world 映射。"""
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib import image as mimg

    # 顶视图包围盒直接取自射线网格的顶点统计，不必再读一遍文件
    x0, x1 = field.x_min - PAD_M, field.x_max + PAD_M
    y0, y1 = field.y_min - PAD_M, field.y_max + PAD_M
    w = int((x1 - x0) * PPM)
    h = int((y1 - y0) * PPM)
    px = np.arange(w)
    py = np.arange(h)
    gx, gy = np.meshgrid(x0 + (px + 0.5) / PPM, y0 + (h - 0.5 - py) / PPM)
    XY = np.stack([gx.ravel(), gy.ravel()], axis=1)
    z = field.floor_z(XY).reshape(h, w)
    oh = field.obstacle_height(XY, z.ravel()).reshape(h, w)

    lo, hi = np.nanpercentile(z, [3, 92])
    # 整张图一个地板点都没有时 nanpercentile 返回 nan（只 warning 不抛），往下会让
    # _ramp 的 max(hi-lo, 1e-6) 变 nan、整张底图成 nan，而 z_lo/z_hi 经 /api/meta
    # 出去是字面量 NaN —— 浏览器 JSON.parse 直接炸（Python 的 json.loads 反而收得下，
    # 所以命令行看不出问题）。退回扫描体的 z 范围，出一张全"无地板"的底图。
    if not (np.isfinite(lo) and np.isfinite(hi)):
        lo, hi = float(field.z_min), float(field.z_max)
        if not np.isfinite(lo) or not np.isfinite(hi) or hi <= lo:
            lo, hi = -1.0, 1.0
    rgb = _ramp(np.nan_to_num(z, nan=(lo + hi) / 2), lo, hi)
    nofloor = ~np.isfinite(z)
    rgb[nofloor] = (0.10, 0.11, 0.14)                    # 无地板：近黑
    blocked = np.isfinite(oh) & (oh < CLEAR_MIN) & ~nofloor
    rgb[blocked] = 0.55 * rgb[blocked] + 0.45 * np.array([1.0, 0.55, 0.05])
    alpha = np.where(nofloor, 0.55, 1.0)

    rgba = np.dstack([rgb, alpha]).astype(np.float32)
    buf = io.BytesIO()
    mimg.imsave(buf, rgba, format="png")
    with open(os.path.join(STATE["cache"], "basemap.png"), "wb") as f:
        f.write(buf.getvalue())

    STATE.update(xmin=x0, ymin=y0, w=w, h=h, z_lo=float(lo), z_hi=float(hi),
                 z_min=field.z_min, z_max=field.z_max,
                 floor_frac=float(1 - nofloor.mean()))
    print("底图 %dx%d px (%.0f px/m)，地板覆盖 %.1f%%，净空禁区 %.1f%%" % (
        w, h, PPM, 100 * (1 - nofloor.mean()), 100 * blocked.mean()))


# ----------------------------------------------------------------- 数据层 --
def enrich(pts_xy):
    """给每个点补上从扫描网格反推的 z 与弧长 s。客户端只管平面坐标。

    入参可以是浏览器传来的 {"x":..,"y":..}，也可以是 (x, y) 元组。
    """
    def xy(p):
        return (p["x"], p["y"]) if isinstance(p, dict) else (p[0], p[1])

    pts = [xy(p) for p in pts_xy]
    out, s = [], 0.0
    if pts:
        zs = field.floor_z(np.asarray(pts, dtype=float))
        for i, (x, y) in enumerate(pts):
            if i:
                s += math.hypot(x - pts[i - 1][0], y - pts[i - 1][1])
            z = zs[i]
            out.append({"s": round(s, 4), "x": round(float(x), 5),
                        "y": round(float(y), 5),
                        "z": round(float(z), 5) if np.isfinite(z) else 0.0,
                        "z_missing": bool(not np.isfinite(z))})
    return out


def road_report(road):
    """单条路的完整诊断：净空/平整/地板 + 拟合残差 + 路面-地板 Δz。"""
    pts = [(p["x"], p["y"], p["z"]) for p in road["points"]]
    hw = max(float(road.get("width_left", 1.75)),
             float(road.get("width_right", 1.75)))
    r = {"id": road["id"], "name": road.get("name", ""),
         "n_points": len(pts), "length_m": round(pts and road["points"][-1]["s"] or 0.0, 2),
         "z_missing": sum(1 for p in road["points"] if p.get("z_missing"))}
    if len(pts) < 2:
        r["error"] = "至少需要 2 个点"
        return r
    frac, worst, issues = probe_line(field, pts, hw + 0.3, CLEAR_MIN, FLAT_TOL)
    r["floor_frac"] = round(frac, 4)
    wm, wpos = worst if worst[1] else (None, None)
    # inf 会序列化成 JSON 里不存在的 Infinity，浏览器 JSON.parse 直接炸，所以转 None
    r["clearance_worst_m"] = (None if wm is None or math.isinf(wm) else round(wm, 3))
    # 只有真的低于要求才标位置：以前无条件标"最差断面"，一条完全通畅的路
    # 也会被画上一个橙色 ⊗，看着像出了毛病。
    r["clearance_worst_at"] = ([round(v, 2) for v in wpos]
                               if wpos is not None and wm is not None and wm < CLEAR_MIN
                               else None)
    r["issues"] = issues
    try:
        fitted = FG.build({"roads": [road]}, None, "asam")["roads"][0]
        r["fit_deviation_m"] = round(fitted["quality"]["max_fit_deviation_m"], 4)
        r["smooth_disp_m"] = round(fitted["quality"]["max_smoothing_disp_m"], 3)
        r["n_geometry"] = fitted["quality"]["n_geom"]
        r["kinds"] = fitted["quality"]["kinds"]
        # 路面贴在扫描地板上的程度：参考线逐点比 z
        gs = [FG.geom_from_dict(d) for d in fitted["geometry"]]
        samp = []
        for g in gs:
            n = max(int(math.ceil(g.length / 0.25)), 2)
            for i in range(n + 1):
                p = g.at(g.length * i / n)
                samp.append((p.x, p.y))
        S = np.asarray(samp)
        fz = field.floor_z(S)
        ez = np.array([_elev_at(fitted["elevation"], s) for s in
                       np.cumsum([0.0] + list(np.hypot(np.diff(S[:, 0]), np.diff(S[:, 1]))))])
        ok = np.isfinite(fz)
        dz = ez[ok] - fz[ok]
        r["dz_rmse_m"] = round(float(np.sqrt(np.mean(dz ** 2))), 4) if len(dz) else None
        r["dz_p95_m"] = round(float(np.percentile(np.abs(dz), 95)), 4) if len(dz) else None
    except SystemExit as e:
        r["error"] = str(e)
    except Exception as e:
        r["error"] = "%s: %s" % (type(e).__name__, e)
    return r


def _elev_at(elev, s):
    seg = elev[0]
    for e in elev:
        if e["s"] <= s:
            seg = e
    d = s - seg["s"]
    return seg["a"] + seg["b"] * d + seg["c"] * d ** 2 + seg["d"] * d ** 3


def load_state():
    if os.path.exists(STATE["traces_path"]):
        with open(STATE["traces_path"]) as f:
            return json.load(f)
    return {"schema": 1, "frame": "S", "frame_note": "扫描/Blender 帧，米，右手 Z-up",
            "source": "trace_editor.py", "roads": []}


# ------------------------------------------------------------------- 路由 --
@app.route("/")
def index():
    return send_from_directory(STATIC, "index.html")


@app.route("/<path:fname>")
def static_files(fname):
    return send_from_directory(STATIC, fname)


@app.route("/assets/<path:name>")
def assets(name):
    """从 FBX 烘出来的场景资产（scene.glb / basemap.png）走这里，
    和代码分开存放，换一份 FBX 不会读到上一份的残留。"""
    if not STATE.get("loaded"):
        return not_loaded()
    return send_from_directory(STATE["cache"], name)


@app.route("/api/scanlist")
def api_scanlist():
    d = request.args.get("dir") or SCANS["root"]
    if not inside_root(d):
        return jsonify({"error": "只能浏览加载根目录 %s 之内" % SCANS["root"]}), 403
    if not os.path.isdir(d):
        return jsonify({"error": "目录不存在：%s" % d}), 404
    return jsonify(scan_list(os.path.realpath(d)))


@app.route("/api/load", methods=["POST"])
def api_load():
    p = os.path.abspath(str(request.get_json(force=True).get("path", "")))
    with LOAD_LOCK:
        if PROG["state"] == "running":
            return jsonify({"error": "上一次载入还没结束"}), 409
    if not p.lower().endswith(".fbx"):
        return jsonify({"error": "只能加载 .fbx 文件：%s" % p}), 400
    if not inside_root(p):
        return jsonify({"error": "只能加载 %s 之内的文件" % SCANS["root"]}), 403
    if not os.path.isfile(p):
        return jsonify({"error": "找不到文件：%s" % p}), 404
    with LOAD_LOCK:
        PROG.update(state="running", fbx=p, phase=0, detail="排队中",
                    t0=time.time(), error=None, marks={}, end=0.0)
    threading.Thread(target=load_worker, args=(p,), daemon=True).start()
    return jsonify({"started": True, "fbx": p})


@app.route("/api/loadstatus")
def api_loadstatus():
    with LOAD_LOCK:
        j = dict(PROG)
    j["phases"] = PHASES
    j["elapsed"] = round(time.time() - j["t0"], 1) if j["t0"] else 0.0
    j.pop("t0", None)
    j["loaded"] = bool(STATE.get("loaded"))
    return jsonify(j)


def json_safe(o):
    """把非有限浮点（nan/inf）换成 None —— JSON 里没有 NaN/Infinity 字面量。

    flask 的 jsonify 会把它们原样写成 `NaN`，而浏览器 JSON.parse 直接抛异常。
    Python 的 json.loads 反而收得下，所以只用命令行测是发现不了的。作者已经在
    road_report 里为 inf 单独防过一次，这里在出口统一兜一次底。
    """
    if isinstance(o, float):
        return o if math.isfinite(o) else None
    if isinstance(o, dict):
        return {k: json_safe(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [json_safe(v) for v in o]
    return o


@app.route("/api/meta")
def api_meta():
    if not STATE.get("loaded"):
        return jsonify({"loaded": False})
    meta = {k: STATE.get(k) for k in
            ("fbx", "xmin", "ymin", "w", "h", "z_lo", "z_hi", "floor_frac",
             "z_min", "z_max", "box")}
    meta["traces"] = (os.path.relpath(STATE["traces_path"], HERE)
                      if STATE["traces_path"] else None)
    meta.update(loaded=True, ppm=PPM, clear_min=CLEAR_MIN)
    return jsonify(json_safe(meta))


@app.route("/api/state", methods=["GET"])
def api_get():
    if not STATE.get("loaded"):
        return not_loaded()
    doc = load_state()
    # 不要在这里把 doc["scene"] 改写成当前场景：那正是"这份存档属于哪份扫描"的证据，
    # 覆盖掉之后客户端 editor.js:1062 那条"存档属于别的场景就先不显示"的判断永远为假，
    # 等于把保护性检查变成死代码。文件里的 scene 就是它被写下去时的场景，
    # 缺字段时才用当前场景补上。
    doc.setdefault("scene", STATE["fbx"])
    return jsonify(doc)


@app.route("/api/state", methods=["POST"])
def api_put():
    if not STATE.get("loaded"):
        return not_loaded()
    doc = request.get_json(force=True)
    roads = []
    for r in sorted(doc.get("roads", []), key=lambda x: int(x["id"])):
        pts = enrich(r.get("points", []))
        if len(pts) < 2:
            return jsonify({"error": "road %s 点数不足" % r.get("id")}), 400
        roads.append({"id": int(r["id"]), "name": r.get("name", "road_%04d" % int(r["id"])),
                      "points": pts,
                      "width_left": float(r.get("width_left", 1.75)),
                      "width_right": float(r.get("width_right", 1.75)),
                      "speed_kmh": float(r.get("speed_kmh", 5.0)),
                      "link_next": r.get("link_next"),
                      "junction_in": r.get("junction_in"),
                      "junction_out": r.get("junction_out"),
                      "source": "trace_editor"})
    ids = [r["id"] for r in roads]
    if len(set(ids)) != len(ids):
        return jsonify({"error": "xodr.id 重复：%s" % ids}), 400
    out = {"schema": 1, "frame": "S", "frame_note": "扫描/Blender 帧，米，右手 Z-up",
           "source": "trace_editor.py", "scene": STATE["fbx"], "roads": roads}
    os.makedirs(os.path.dirname(STATE["traces_path"]), exist_ok=True)   # 缓存被删过也要存得下
    with open(STATE["traces_path"], "w") as f:
        json.dump(out, f, indent=1, ensure_ascii=False)
    return jsonify({"ok": True, "roads": len(roads),
                    "reports": [road_report(r) for r in roads]})


@app.route("/api/check", methods=["POST"])
def api_check():
    if not STATE.get("loaded"):
        return not_loaded()
    doc = request.get_json(force=True)
    try:
        reps = []
        for r in doc.get("roads", []):
            if len(r.get("points", [])) < 2:
                continue
            r = dict(r)
            r["points"] = enrich(r["points"])   # 客户端只给 x,y，这里补 z/s
            reps.append(road_report(r))
        return jsonify({"reports": reps})
    except Exception:
        return jsonify({"error": traceback.format_exc()}), 500


@app.route("/api/occlusion", methods=["POST"])
def api_occlusion():
    """道路点的可见性，规则与 WebGL 剖切面完全一致。

    点 (x, y, z) 可见  <=>  z <= cut 且 不存在扫描面满足 z < h <= cut。
    即"在高度 cut 处水平剖切、从上往下看"：剖切面以上被削掉，看到的是剩下
    最高的那个面。道路被屋顶压住就该看不见，高架桥上的路也会遮住桥下的路。

    返回逐采样点状态：ok / occluded（被更高的面压住）/ cut（自身高于阈值）/
    off（这里根本没有地面，谈不上可见性）。
    """
    if not STATE.get("loaded"):
        return not_loaded()
    body = request.get_json(force=True)
    cut = float(body.get("cut", 0.0))
    roads = body.get("roads", [])

    allpts, spans = [], []
    for r in roads:
        a = len(allpts)
        allpts.extend([(float(px), float(py)) for px, py in r["pts"]])
        spans.append((a, len(allpts), r["id"]))
    if not allpts:
        return jsonify({"roads": []})
    XY = np.asarray(allpts, dtype=float)
    # 高程和各层表面必须出自**同一套采样**。以前 z 在精确点上打射线、遮挡却去查
    # 0.1 m 的预计算网格，两者差 ±0.9 mm，而判据是严格大于 —— 实测 27 个采样点里
    # 13 个被自己脚下的地板判成"遮挡"，表现为路画到中间莫名其妙断掉，且拖动阈值
    # 毫无反应。改成同样按精确点求交之后差值是 0，网格整套也就没人用了。
    zs = field.floor_z(XY)
    hits = field.column_hits(XY, k=COL_LAYERS)

    flat = []
    heights = []
    for a, b, rid in spans:
        st = []
        for k in range(a, b):
            z = zs[k]
            if not np.isfinite(z):
                st.append("off")
                continue
            if z > cut:
                st.append("cut")
                continue
            col = hits[:, k]
            col = col[np.isfinite(col)]
            up = col[(col > z) & (col <= cut)]
            if len(up):
                st.append("occluded")
                heights.append(float(up.min()))    # 压住它的那个面有多高
            else:
                st.append("ok")
        flat.append({"id": rid, "status": st})
    # 把遮挡面的高程区间一起报回去：阈值卡在天花板/桌面的起伏里时，线会断成
    # 几截却看不出为什么。给出"是哪些高度在遮"，用户才知道该往哪边拖。
    return jsonify({"roads": flat,
                    "occl_lo": round(min(heights), 2) if heights else None,
                    "occl_hi": round(max(heights), 2) if heights else None})


def pair_fbx(src, out_dir, stem):
    """把源 FBX 以同名放进 xodr 旁边 —— CARLA 的 Import.py 就是按"同目录同名
    .xodr + .fbx"配对地图的（Util/BuildTools/Import.py:63-68）。

    用符号链接而不是复制：源文件 340 MB，复制一份不仅占地方，更糟的是你在 Blender
    里重新导出之后，副本会悄悄变成旧数据，而 xodr 是新算的。链接永远指向源文件。
    """
    dst = os.path.join(out_dir, stem + ".fbx")
    if os.path.islink(dst):
        os.remove(dst)
    elif os.path.exists(dst):
        return dst                      # 用户自己放的实体文件，不覆盖
    try:
        os.symlink(src, dst)
    except OSError:                     # 跨设备/文件系统不支持链接
        shutil.copy2(src, dst)
    return dst


def stage_for_import(src_xodr, src_fbx, import_dir, stem):
    """把同名配对放进 CARLA 的 Import/<stem>/，让 `make import` 直接能扫到。

    目录必须是**真实目录**：Import.py 用 `os.walk(Import)` 扫描，而 os.walk 默认
    不进入符号链接的目录（实测过，链进去的目录整个被跳过）。所以这里 xodr 用复制
    （几 KB），FBX 用符号链接（340 MB，且你在 Blender 里重导之后链接自动跟到新内容，
    复制则会悄悄留下旧数据）。
    """
    if not import_dir:
        return None
    dst = os.path.join(import_dir, stem)
    os.makedirs(dst, exist_ok=True)
    shutil.copy2(src_xodr, os.path.join(dst, stem + ".xodr"))
    fx = os.path.join(dst, stem + ".fbx")
    if os.path.islink(fx):
        os.remove(fx)
    if not os.path.exists(fx):
        try:
            os.symlink(os.path.abspath(src_fbx), fx)
        except OSError:
            shutil.copy2(src_fbx, fx)
    return dst


def emit_paths():
    """当前场景的三个产物坐标：out/ 目录、地图名、xodr 路径。

    地图名跟着 FBX 文件名走 —— CARLA 的配对靠同名（见 pair_fbx），而 `make import`
    建出来的关卡目录也就是这个名字。
    """
    out_dir = os.path.join(STATE["cache"], "out")
    stem = os.path.splitext(os.path.basename(STATE["fbx"]))[0]
    return out_dir, stem, os.path.join(out_dir, stem + ".xodr")


# 换台机器就不成立的东西一律放这里：路径、主机端口、Blender 位置、纹理上限。
# 存在 deploy_config.json（已 gitignore），留空 = 用下面的默认值。
# 规则：**命令行参数只是给默认值播种，面板里存的优先** —— 同一个值不能有两个
# 各说各话的来源（以前 --import-dir 和面板里的"CARLA 根目录"就是各管一半）。
TOOL_DEFAULTS = {
    # $CARLA_ROOT 与 $CARLA_CACHE_DIR 对称：无头/CI 里没有面板可点，
    # 少了它，"换台机器"里唯一只能手写 deploy_config.json 的就是这一项。
    "carla_root": os.environ.get("CARLA_ROOT") or CARLA_ROOT_DEFAULT,
    "import_dir": "",                                # 空 = <carla_root>/Import
    "package": "",                                   # 空 = 跟地图名同名
    "client_cache": os.environ.get("CARLA_CACHE_DIR")
                    or os.path.expanduser("~/carlaCache"),
    "carla_host": "localhost",                       # 标定要连的服务端
    "carla_port": "2000",
    "blender": "",                                   # 空 = $BLENDER / which / 家目录通配
    "maxtex": "1024",                                # 按显存定，见 PITFALLS 的 Firefox 那条
}
TOOL_CONFIG = os.path.join(HERE, "deploy_config.json")


def load_config():
    cfg = dict(TOOL_DEFAULTS)
    if os.path.isfile(TOOL_CONFIG):
        with open(TOOL_CONFIG) as f:
            cfg.update({k: v for k, v in json.load(f).items() if k in cfg})
    return cfg


def cfg_path(v):
    """配置里的一个路径值 -> 绝对路径。**expanduser 只在这一个地方做。**

    以前 carla_target 展开了、deploy_status/api_deploy 没展开：面板里填 ~/carla 时
    实际部署到 /home/u/carla/…，侧栏却显示 ../../../carla/…（校验用展开值、显示用原值）。
    同一个值两套解释，回显和事实就会分叉 —— 面板存在的意义就是让回显可信。
    """
    return os.path.abspath(os.path.expanduser(str(v).strip()))


def carla_layout(root):
    """这个根目录像哪种 CARLA 装法，或 None（不像 CARLA）。

    两种装法的 Content 位置不同，所以"根目录对不对"不能只看目录存不存在：
      source   <root>/Unreal/CarlaUE4/Content，且有 Util/BuildTools/Import.py（能 make import）
      package  <root>/CarlaUE4/Content（发布版，没有 make import，关卡得在别处生成）
    ~/carla 与 ~/carla-01 并存时，指错的那个照样"存在"，于是把地图静默投进另一棵树。
    """
    if os.path.isfile(os.path.join(root, "Util", "BuildTools", "Import.py")):
        return "source"
    if os.path.isdir(os.path.join(root, "CarlaUE4", "Content")):
        return "package"
    return None


def content_dir(root):
    """<root> 下的 Content 目录，按实际布局选。

    两种布局都不像时按源码布局给一个路径：没构建过的 CARLA 检出是正常的前置状态，
    这里不报错，交给调用方按"目录不存在"去说。
    """
    if carla_layout(root) == "package":
        return os.path.join(root, "CarlaUE4", "Content")
    return os.path.join(root, "Unreal", "CarlaUE4", "Content")


def import_dir_of(cfg=None):
    """生成 xodr 时同名配对投到哪儿。目录不存在就返回 None（跳过投放，不报错）。"""
    cfg = cfg or load_config()
    d = (cfg_path(cfg["import_dir"]) if cfg["import_dir"]
         else os.path.join(cfg_path(cfg["carla_root"]), "Import"))
    return d if os.path.isdir(d) else None


def carla_target(stem, cfg=None):
    """`make import` 给这张图建出来的关卡目录。"""
    cfg = cfg or load_config()
    maps = os.path.join(content_dir(cfg_path(cfg["carla_root"])),
                        cfg["package"] or stem, "Maps", stem)
    return {"maps": maps,
            "xodr": os.path.join(maps, "OpenDrive", stem + ".xodr"),
            "tm": os.path.join(maps, "TM", stem + ".bin")}


def md5(path):
    h = hashlib.md5()
    with open(path, "rb") as f:
        for blk in iter(lambda: f.read(1 << 16), b""):
            h.update(blk)
    return h.hexdigest()


def client_cache_dirs(stem, cfg=None):
    """客户端镜像缓存里属于这张图的目录：`<缓存根>/<客户端版本>/<包>/Maps/<图>/`。

    路径里只有名字、没有内容哈希（LibCarla `client/detail/Client.cpp:234-240` 见本地
    已存在就**不重新下载**），所以服务端换了图，这份旧的还会被原样读 —— 今天那次
    `get_trafficmanager()` 段错误就是这么来的。版本目录名跟着构建走（现在是
    `ac006dd1b-dirty`），别猜，通配。

    包名那一段必须和 carla_target 一样用 `cfg["package"] or stem`：以前这里两个
    槽位都写 stem，配了独立包名（如 package=MyPkg、地图 MyMap）时通配不到任何
    目录，于是"部署成功"却把陈旧采样表留在原地，段错误照旧。默认 package 为空、
    包名等于地图名，所以一直没暴露。
    """
    cfg = cfg or load_config()
    root = cfg_path(cfg["client_cache"] or TOOL_DEFAULTS["client_cache"])
    pkg = cfg["package"] or stem
    # 只有 <客户端版本> 那一段是通配，包名和地图名是**字面量**，必须转义：
    # 含 [ ] ? * 的地图名（如 map[1].fbx）会让整条 glob 匹配不到任何目录，
    # 于是"部署成功"却把陈旧采样表留在原地 —— 正是上面那段 139 的成因。
    pat = os.path.join(root, "*", glob.escape(pkg), "Maps", glob.escape(stem))
    return [d for d in sorted(glob.glob(pat)) if os.path.isdir(d)]


def client_cache_files(stem, cfg=None, sub=None):
    """镜像缓存里的文件。`sub="TM"` 只要采样表那一层。"""
    out = []
    for d in client_cache_dirs(stem, cfg):
        base = os.path.join(d, sub) if sub else d
        for s, _, fs in os.walk(base):
            out.extend(os.path.join(s, n) for n in fs)
    return sorted(out)


def calib_status():
    """标定这一步只做成"给你命令 + 告诉你现在标没标"，不在浏览器里跑：
    它需要一个已经 Play 起来、且加载了这张图的 CARLA 服务端，默认参数下要打
    一万四千多条同步射线（sample_world_flat 还要按 3x3 邻域再打一轮），
    是分钟级的活，和 validate / check_alignment 那种纯文件运算不是一个量级。"""
    mesh = os.path.join(STATE["cache"], "scene.glb")
    fjp = FG.frame_json_path(mesh)
    cfg = load_config()
    try:
        acc = json.load(open(fjp))
    except Exception:
        acc = {}
    m = acc.get("A_metrics") or {}
    return {
        # host/port 显式写进命令：默认值 localhost:2000 是"服务端在同一台机器"的
        # 假设，换个人/换台机器不成立，让复制命令的人自己去猜连不上是因为啥不值当。
        # 逐项 shlex.quote：这条命令是要**粘进终端执行**的，而 host/port 是面板里
        # 手输的自由文本（以前只查了非空、端口查了范围，没有字符集限制）——
        # 一个 `--host 'h; rm -rf ~'` 就会在粘贴时变成两条命令。
        # quote 之后值里的空格/分号/反引号都只是普通字符，不会被执行。
        "cmd": ("cd %s && python3 calibrate_frame.py --mesh %s --host %s --port %s"
                " --extent 60 --step 1.0 --write"
                % (shlex.quote(HERE), shlex.quote(os.path.relpath(mesh, HERE)),
                   shlex.quote(cfg["carla_host"]), shlex.quote(cfg["carla_port"]))),
        "frame_json": os.path.relpath(fjp, HERE),
        "calibrated": bool(acc.get("calibrated")),
        "rmse_z_m": m.get("rmse_z_m"), "inlier_frac": m.get("inlier_frac"),
        "mesh_ready": os.path.exists(mesh),
    }


def deploy_status(src_xodr, stem):
    """不改任何东西，只回答"CARLA 里那份跟你刚生成的是不是同一个"。"""
    cfg = load_config()
    root = cfg_path(cfg["carla_root"])
    t = carla_target(stem, cfg)
    pkg = cfg["package"] or stem
    st = {"target": os.path.relpath(t["xodr"], root),
          "root": root, "package": pkg,
          # 布局给出去：package 版没有 Util/BuildTools，也就没有 make import，
          # 前端据此把那句命令标成"这份 CARLA 里没有"而不是让人复制完才发现。
          "layout": carla_layout(root),
          # 首次没有关卡目录时部署无从下手，只能先让 CARLA 把 FBX 导成关卡。
          # 命令原样给出去，前端做成一键复制，省得回终端翻 README。
          # 前面那句 rm 不是可有可无的清理：Import.py:612-617 只在 Import/ 下一个
          # .json 都没有时才扫描 fbx/xodr 配对，跑过一次留下的 <包>.json 会让它直接用
          # 旧配置、根本不看新放的地图 —— 少了这步，重导会静默导回上一版。
          # 命令是要**粘进终端执行**的，两个值都 shlex.quote：包名来自面板自由文本、
          # 根目录可能带空格。不转义时 `--package=x"; rm -rf ~; #` 就是两条命令。
          "import_cmd": 'cd %s && rm -f Import/*.json && make import ARGS=%s'
                        % (shlex.quote(root), shlex.quote("--package=" + pkg)),
          "calib": calib_status(),
          "state": "未导入"}
    if not os.path.isdir(os.path.dirname(t["xodr"])):
        return st
    if not os.path.exists(src_xodr):
        return dict(st, state="还没生成 xodr")
    same = md5(src_xodr) == md5(t["xodr"])
    # 判"一致"的充要条件：两份 xodr 同内容，且没有任何一份采样表比现行 xodr 还旧。
    # 不能只看客户端镜像里有没有文件 —— 那里必然躺着一份 OpenDrive/<图>.xodr，
    # 而它永远不被读（Map 走 GetMapData() 现取），拿它判过期会让按钮一直橙着。
    xt = os.path.getmtime(t["xodr"])
    bins = ([t["tm"]] if os.path.exists(t["tm"]) else []) \
        + client_cache_files(stem, cfg, "TM")
    stale = [p for p in bins if os.path.getmtime(p) < xt]
    st["state"] = "一致" if (same and not stale) else "已过期"
    st["tm_bin"] = os.path.exists(t["tm"])
    st["stale_bins"] = len(stale)
    st["client_cache"] = len(client_cache_files(stem, cfg))
    return st


def deploy_to_carla(src_xodr, stem):
    """把刚生成的 xodr 装进 CARLA，并清掉所有会跟着失配的缓存。

    三步缺一不可，实测矩阵见 README：换 xodr 之后，服务端 `TM/<图>.bin` 和客户端
    `~/carlaCache/…` 任何一份留着旧的，TrafficManager 就会拿旧采样表去查新路，
    `GetWaypointXODR` 返回空指针，`get_trafficmanager()` 当场段错误（退出码 139）。
    这里只删不烘 —— 采样表由客户端运行时重建，本例规模下是瞬间的事。
    """
    cfg = load_config()
    t = carla_target(stem, cfg)
    if not os.path.isdir(os.path.dirname(t["xodr"])):
        raise SystemExit("CARLA 里还没有 %s 这个目录 —— 先跑一次 make import，"
                         "或到「部署设置」里把根目录/包名改对" % os.path.dirname(t["xodr"]))
    done = {"xodr": t["xodr"]}
    shutil.copy2(src_xodr, t["xodr"])
    done["md5"] = md5(t["xodr"])
    done["md5_ok"] = done["md5"] == md5(src_xodr)
    removed = []
    if os.path.exists(t["tm"]):
        os.remove(t["tm"])
        removed.append(t["tm"])
    for p in client_cache_files(stem, cfg):
        os.remove(p)
        removed.append(p)
    for d in client_cache_dirs(stem, cfg):        # 自底向上把空壳收掉
        for sub, _, _ in sorted(os.walk(d), reverse=True):
            if not os.listdir(sub):
                os.rmdir(sub)
        for up in (d, os.path.dirname(d), os.path.dirname(os.path.dirname(d))):
            if os.path.isdir(up) and not os.listdir(up):
                os.rmdir(up)                     # 连 <包>/Maps/<图> 的空壳一起收
    done["removed"] = removed
    return done


@app.route("/api/deploy", methods=["GET", "POST"])
def api_deploy():
    """GET = 只报漂移状态；POST = 真的部署。"""
    if not STATE.get("loaded"):
        return not_loaded()
    _, stem, xodr = emit_paths()
    if request.method == "GET":
        return jsonify(deploy_status(xodr, stem))
    try:
        out = deploy_to_carla(xodr, stem)
    except (SystemExit, OSError) as e:
        return jsonify({"error": str(e)}), 400
    root = cfg_path(load_config()["carla_root"])
    out["map"] = stem
    out["target"] = os.path.relpath(out["xodr"], root)
    out["xodr"] = os.path.relpath(out["xodr"], HERE)
    out["removed"] = [os.path.relpath(p, root) if p.startswith(root)
                      else os.path.relpath(p, os.path.expanduser("~"))
                      for p in out["removed"]]
    return jsonify(out)


@app.route("/api/config", methods=["GET", "POST"])
def api_config():
    """换机器要改的东西全在这。GET 顺带把解析出来的完整路径摊开给人看，
    POST 逐项校验后才落盘 —— 填错了当场说，别等生成/部署那步再炸。"""
    cfg = load_config()
    stem = os.path.splitext(os.path.basename(STATE["fbx"]))[0] if STATE.get("loaded") else ""

    def view(c):
        try:
            found = find_blender(c)          # 空值也要解析一遍：面板要能回答
        except SystemExit:                    # "这台机器上烘得了吗"
            found = None
        root = cfg_path(c["carla_root"])
        return dict(c, file=os.path.relpath(TOOL_CONFIG, HERE), map=stem,
                    defaults=TOOL_DEFAULTS,
                    staged=import_dir_of(c) or "（目录不存在，生成时跳过投放）",
                    blender_found=found,
                    # 解析出来的绝对路径和布局都给前端：只回显原始字符串的话，
                    # 填了 ~/xxx 的人看不到它到底落在哪，也就看不出填错了。
                    root_resolved=root, layout=carla_layout(root),
                    preview=carla_target(stem, c)["xodr"] if stem else None)

    if request.method == "GET":
        return jsonify(view(cfg))
    for k in TOOL_DEFAULTS:
        if k in request.json:
            cfg[k] = str(request.json[k]).strip()
    if cfg["package"] and ("/" in cfg["package"] or ".." in cfg["package"]):
        return jsonify({"error": "包名不能含路径分隔符：%s" % cfg["package"]}), 400
    root = cfg_path(cfg["carla_root"])
    if not os.path.isdir(root):
        return jsonify({"error": "CARLA 根目录不存在：%s" % root}), 400
    # 存在 ≠ 是 CARLA。~/carla（源码版）和 ~/carla-01（发布版）并存时，指错的那个
    # 照样能过上一关，于是地图被静默投进另一棵树 —— 这里当场退回，不等部署那步。
    # 只是"不像"就拒绝是有依据的：两种布局的 Content 位置不同，猜不出该往哪写；
    # 而 load_config 不做校验，真遇到第三种布局还能手写 deploy_config.json 绕过面板。
    if carla_layout(root) is None:
        return jsonify({"error": "这不像 CARLA 根目录：%s —— 底下既没有 "
                                 "Util/BuildTools/Import.py（源码版），也没有 "
                                 "CarlaUE4/Content（发布版）。要填的是 CARLA 的根目录，"
                                 "不是 Unreal/ 或 CarlaUE4/ 那一层" % root}), 400
    if cfg["import_dir"] and not os.path.isdir(cfg_path(cfg["import_dir"])):
        return jsonify({"error": "投放目录不存在：%s（留空 = 用 <CARLA 根目录>/Import）"
                                  % cfg["import_dir"]}), 400
    if not cfg["carla_host"]:
        return jsonify({"error": "服务端地址不能为空；标定要连它"}), 400
    try:
        port = int(cfg["carla_port"])
        assert 1 <= port <= 65535
    except (ValueError, AssertionError):
        return jsonify({"error": "端口得是 1~65535 的整数，现在是 %r" % cfg["carla_port"]}), 400
    cfg["carla_port"] = str(port)
    try:
        tex = int(cfg["maxtex"])
        assert tex >= 128 and (tex & (tex - 1)) == 0
    except (ValueError, AssertionError):
        return jsonify({"error": "纹理上限得是 128 及以上的 2 的幂（如 512/1024/2048），"
                                 "现在是 %r" % cfg["maxtex"]}), 400
    cfg["maxtex"] = str(tex)
    # 必须用 cfg_path：以前这里 expanduser 了、find_blender 没有，于是面板里填
    # ~/blender/blender 能通过校验，到烘焙那步却报"找不到 Blender"。
    if cfg["blender"] and not os.path.isfile(cfg_path(cfg["blender"])):
        return jsonify({"error": "Blender 可执行文件不存在：%s（留空 = 自动找）"
                                  % cfg["blender"]}), 400
    out = {k: cfg[k] for k in TOOL_DEFAULTS}
    with open(TOOL_CONFIG, "w") as f:
        json.dump(out, f, indent=1, ensure_ascii=False)
    return jsonify(dict(view(out), ok=True, saved=os.path.relpath(TOOL_CONFIG, HERE)))



@app.route("/api/emit", methods=["POST"])
def api_emit():
    """跑完整链路：拟合 -> 发射 xodr，并回报校验结果。"""
    if not STATE.get("loaded"):
        return not_loaded()
    doc = load_state()
    if not doc["roads"]:
        return jsonify({"error": "还没有任何路"}), 400
    try:
        # frame.json 按场景存：A_S2W 是"这一份扫描 -> world"的解，换场景必须重标
        T = FG.derive_T_S2X(json.load(open(
            FG.frame_json_path(os.path.join(STATE["cache"], "scene.glb")))))
    except Exception as e:
        # 不能静默吞：T=None 时 xodr 是按未标定（T_S2X=单位阵）发的，而 frame.json
        # 存在却是坏的/读不动，说明本该带标定的场景发了一份没带标定的图 —— 后面
        # validate 的 A_S2W 断言可能照样过，用户看不出任何异常。标定前的旧场景
        # 本来就没有 frame.json，那种情况只打一行提示。
        if os.path.exists(FG.frame_json_path(
                os.path.join(STATE["cache"], "scene.glb"))):
            frame_note = ("frame.json 读取失败（%s），本次按未标定发射："
                          "CARLA 版与 ASAM 版数值相同" % type(e).__name__)
            print("  警告: frame.json 读取失败（%s: %s），本次按未标定发射 "
                  "(T_S2X=单位阵) —— CARLA 版与 ASAM 版数值会相同"
                  % (type(e).__name__, e))
        else:
            frame_note = "本场景没有 frame.json，按未标定发射（两版数值相同）"
            print("  提示: 本场景没有 frame.json，按未标定发射 (T_S2X=单位阵)")
        T = None
    else:
        frame_note = None
    # 每个场景一份 out/，否则换场景生成会把上一个场景的 xodr 覆盖掉
    out_dir, stem, xodr = emit_paths()
    os.makedirs(out_dir, exist_ok=True)
    reps, files = [], []
    try:
        fx = FG.build(doc, T, "xodr")
        fa = FG.build(doc, None, "asam")
        json.dump(fx, open(os.path.join(out_dir, "roads_fitted_xodr.json"), "w"), indent=1)
        json.dump(fa, open(os.path.join(out_dir, "roads_fitted_asam.json"), "w"), indent=1)
        # 地图名跟着 FBX 走，不能写死 TestMap：配对靠的是同名（见 pair_fbx）
        emit_xodr.emit(fx, stem, xodr)
        emit_xodr.emit(fa, stem + "_asam", os.path.join(out_dir, stem + "_asam.xodr"))
        pair_fbx(STATE["fbx"], out_dir, stem)
        files = [os.path.relpath(os.path.join(out_dir, n), HERE)
                 for n in (stem + ".xodr", stem + "_asam.xodr", stem + ".fbx")]
        staged = stage_for_import(xodr, STATE["fbx"], import_dir_of(), stem)
        reps = [road_report(r) for r in doc["roads"]]
        # 校验直接调 validate.run：检查项只有一份实现，web 里全绿而命令行有 FAIL
        # 是最难查的那种不一致。它自己会 import carla 做真解析，所以这里不再单独探一次。
        vrep, _, _ = validate.run(out_dir, stem, stem + "_asam")
        checks = {"total": len(vrep.rows), "fails": vrep.fails}
        parse_ok = not any("carla.Map 解析" in f for f in vrep.fails)
    except SystemExit as e:
        return jsonify({"error": str(e)}), 400
    except Exception as e:
        return jsonify({"error": "%s: %s" % (type(e).__name__, e),
                        "trace": traceback.format_exc()}), 500
    return jsonify({"ok": True, "files": files, "reports": reps,
                "staged": os.path.relpath(
                    staged, cfg_path(load_config()["carla_root"])) if staged else None,
                "carla_parse_ok": parse_ok, "checks": checks,
                "frame_note": frame_note,
                "deploy": deploy_status(xodr, stem)})


@app.route("/api/align", methods=["POST"])
def api_align():
    """离线对齐检查：xodr 认为的路面高度 vs 扫描网格上的地板，量化 Δz。

    约 3 秒（大头是 open3d 读网格），全程不需要服务端/UE，所以可以跟「生成 xodr」
    一样随手点。检查项只有 check_alignment.run() 一份，命令行跑出来的字完全一样。
    """
    if not STATE.get("loaded"):
        return not_loaded()
    _, stem, xodr = emit_paths()
    if not os.path.exists(xodr):
        return jsonify({"error": "还没有 %s，先点「生成 xodr」" % stem}), 400
    try:
        d = check_alignment.run(os.path.join(STATE["cache"], "scene.glb"), xodr)
    except (SystemExit, OSError) as e:
        return jsonify({"error": str(e)}), 400
    d["frame_json"] = os.path.relpath(d["frame_json"], HERE)
    d["xodr"] = os.path.relpath(xodr, HERE)
    return jsonify(d)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scan-root", default=os.path.expanduser("~"),
                    help="「加载数据」能浏览到的根目录，默认家目录 —— 对话框里自己往下翻。"
                         "扫描放在别的挂载点上时才需要指，例如 --scan-root /data/scans")
    ap.add_argument("--cache", default=os.path.join(HERE, "cache"),
                    help="烘焙产物目录，实际用 <cache>/<fbx名>/")
    ap.add_argument("--blender", default=None,
                    help="Blender 可执行文件路径；也可以事后在页面「⚙ 路径」里填，面板优先")
    ap.add_argument("--max-texture", type=int, default=None,
                    help="烘进 GLB 的纹理边长上限，默认 1024；面板里填过的优先")
    ap.add_argument("--import-dir", default=None,
                    help="生成 xodr 时把同名 fbx/xodr 配对放进去的目录，"
                         "默认 <CARLA 根目录>/Import；面板优先")
    ap.add_argument("--carla-root", default=None,
                    help="CARLA 根目录（源码版含 Util/BuildTools，发布版含 CarlaUE4）；"
                         "不填则依次试 $CARLA_ROOT、~/carla。面板里存过的优先")
    ap.add_argument("--traces", default=None,
                    help="路点存档路径；不给就用该场景目录里的 traces.json（每场景一份）")
    ap.add_argument("--port", type=int, default=8071)
    ap.add_argument("--host", default="127.0.0.1")
    args = ap.parse_args()

    root = os.path.abspath(os.path.expanduser(args.scan_root))
    if not os.path.isdir(root):
        raise SystemExit("扫描目录不存在：%s\n用 --scan-root 指到放 FBX 的地方" % root)
    # 命令行只负责"给默认值播种"；deploy_config.json 里存了的以面板为准，
    # 否则同一个值会有两个各说各话的来源（以前 --import-dir 就是）。
    if args.blender:
        TOOL_DEFAULTS["blender"] = args.blender
    if args.max_texture:
        TOOL_DEFAULTS["maxtex"] = str(args.max_texture)
    if args.import_dir:
        TOOL_DEFAULTS["import_dir"] = os.path.expanduser(args.import_dir)
    if args.carla_root:
        TOOL_DEFAULTS["carla_root"] = args.carla_root
    cfg = load_config()
    SCANS.update(root=root, cache=os.path.abspath(args.cache))
    SCANS["traces"] = os.path.abspath(args.traces) if args.traces else None
    # 根目录和布局都打出来：这是"部署会写进哪棵树"唯一的启动期证据。
    # 指错时（比如指到没构建过的检出、或指到另一份 CARLA）能当场看见，
    # 而不是等到部署那步才发现地图进了别的目录。
    croot = cfg_path(cfg["carla_root"])
    lay = carla_layout(croot)
    print("CARLA 根目录：%s（%s）" % (
        croot, {"source": "源码版，可 make import", "package": "发布版，没有 make import",
                None: "⚠ 不像 CARLA 根目录 —— 在「⚙ 路径」里改"}[lay]))
    imp = import_dir_of(cfg)
    if imp:
        print("生成 xodr 时会把同名 fbx/xodr 配对放进 %s/<地图名>/" % imp)
    else:
        print("投放目录 %s 不存在，生成时跳过配对投放（可在「⚙ 路径」里改）"
              % (cfg["import_dir"] or os.path.join(croot, "Import")))
    try:
        print("Blender：%s" % find_blender(cfg))
    except SystemExit as e:
        print("Blender：未找到 —— %s" % e)

    n = sum(1 for _ in glob.glob(os.path.join(root, "**", "*.fbx"), recursive=True))
    print("可加载的 FBX：%d 个（%s 之下）—— 启动不自动加载，去浏览器里点「加载数据」"
          % (n, root))
    print("设置：%s（不存在则全用默认值，页面「⚙ 路径」可改）"
          % (TOOL_CONFIG if os.path.isfile(TOOL_CONFIG) else "未创建"))
    print("\n浏览器打开  http://%s:%d" % (args.host, args.port))
    app.run(host=args.host, port=args.port, debug=False, threaded=True)


if __name__ == "__main__":
    main()
