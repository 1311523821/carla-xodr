#!/usr/bin/env python3
"""xodr 校验：结构不变量 + CARLA 真解析器闭环比对。

三层，从便宜到权威：
  1) 手写不变量（road.length==Σgeo、s 连续、C0 衔接、车道宽度/id/laneSection 覆盖）
  2) carla.Map(name, xml) —— 用 CARLA 自己的解析器与道路模型，无需服务端/UE。
     没装 carla 包时这一层和下面的闭环记 SKIP，不记 FAIL（结构检查照常）。
     import 抛了别的错（例如 numpy ABI）仍是 FAIL，因为包在却不能用。
  3) 闭环：本仓库 xodr_geom 的解析求值 vs CARLA get_waypoint_xodr 返回的坐标。
     两者逐点吻合，才说明"我们理解的 CARLA 几何语义"不是空想；
     同时断言 lane +1 落在 +t 法向侧、且 C 帧与 S 帧两份文件互为声明过的镜像。
"""

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fit_geometry import geom_from_dict, load_json, frame_json_path  # noqa: E402
from xodr_geom import lateral_normal, xodr_to_world, XODR2WORLD  # noqa: E402

C0_TOL = 1e-4           # 接缝位置误差（米）
HDG_TOL = 1e-4          # 接缝朝向误差（弧度）
LEN_TOL = 1e-6          # road.length 与 Σgeo 的容差
CARLA_TOL = 0.02        # 本地解析求值 vs CARLA 实算的容差（米）
LANE_SIDE_TOL = 0.05    # 车道横向归属容差（米）


class Report:
    def __init__(self):
        self.rows = []          # (ok, label, detail)；ok 为 None 表示 SKIP
        self.fails = []
        self.skips = []

    def check(self, ok, label, detail=""):
        self.rows.append((bool(ok), label, detail))
        if not ok:
            self.fails.append("%s  %s" % (label, detail))
        return ok

    def skip(self, label, detail=""):
        self.rows.append((None, label, detail))
        self.skips.append("%s  %s" % (label, detail) if detail else label)
        return None

    def n_checks(self):
        return sum(1 for ok, _, _ in self.rows if ok is not None)

    def dump(self):
        w = max(len(l) for _, l, _ in self.rows) if self.rows else 10
        for ok, label, detail in self.rows:
            tag = "SKIP" if ok is None else ("PASS" if ok else "FAIL")
            print("  %s  %-*s  %s" % (tag, w, label, detail))
        extra = ", %d skipped" % len(self.skips) if self.skips else ""
        print("\n%d checks, %d failed%s" % (self.n_checks(), len(self.fails), extra))
        return 0 if not self.fails else 1


def check_structure(fitted, rep, tag):
    for r in sorted(fitted["roads"], key=lambda x: x["id"]):
        rid, geoms = r["id"], r["geometry"]
        ssum = sum(g["length"] for g in geoms)
        rep.check(abs(ssum - r["length"]) < LEN_TOL,
                  "%s road %d length==sum(geo)" % (tag, rid),
                  "length=%.9f sum=%.9f diff=%.2e" % (r["length"], ssum, r["length"] - ssum))
        rep.check(all(geoms[i]["length"] > 1e-6 for i in range(len(geoms))),
                  "%s road %d 无零长度几何段" % (tag, rid),
                  "min=%.2e" % min(g["length"] for g in geoms))
        prev_end_s = 0.0
        worst_p, worst_h = 0.0, 0.0
        ok_s = True
        for d in geoms:
            if abs(d["s"] - prev_end_s) > 1e-9:
                ok_s = False
            prev_end_s = d["s"] + d["length"]
        for a, b in zip(geoms[:-1], geoms[1:]):
            ga = geom_from_dict(a)
            e = ga.end()
            worst_p = max(worst_p, math.hypot(e.x - b["x"], e.y - b["y"]))
            worst_h = max(worst_h, abs((e.hdg - b["hdg"] + math.pi) % (2 * math.pi) - math.pi))
        rep.check(ok_s, "%s road %d 几何 s 连续" % (tag, rid),
                  "覆盖 [0, %.4f]" % prev_end_s)
        rep.check(worst_p < C0_TOL and worst_h < HDG_TOL,
                  "%s road %d C0 衔接" % (tag, rid),
                  "max|dPos|=%.2e max|dHdg|=%.2e" % (worst_p, worst_h))
        rep.check(r["width_left"] > 0 and r["width_right"] > 0,
                  "%s road %d 车道宽度为正" % (tag, rid),
                  "L=%.2f R=%.2f" % (r["width_left"], r["width_right"]))
        rep.check(0.0 < r["length"] < 5000.0, "%s road %d 长度合理" % (tag, rid),
                  "%.2f m" % r["length"])
        rep.check(len(r["elevation"]) >= 1, "%s road %d 显式 elevation" % (tag, rid),
                  "%d 段（缺省会被 CARLA 注入 0）" % len(r["elevation"]))
        q = r["quality"]
        rep.check(q["max_fit_deviation_m"] <= 0.05,
                  "%s road %d 拟合偏差" % (tag, rid),
                  "%.4f m  %s" % (q["max_fit_deviation_m"], str(q["kinds"])))
        rep.check(q["max_elevation_residual_m"] <= 0.05,
                  "%s road %d 高程残差" % (tag, rid), "%.4f m" % q["max_elevation_residual_m"])


def check_xml(path, rep, tag):
    from lxml import etree
    try:
        tree = etree.parse(path)
    except Exception as e:
        rep.check(False, "%s XML 可解析" % tag, str(e))
        return None
    root = tree.getroot()
    rep.check(root.tag == "OpenDRIVE", "%s 根元素 OpenDRIVE" % tag, root.tag)
    hdr = root.find("header")
    rep.check(hdr is not None and hdr.get("revMajor") == "1"
              and hdr.get("revMinor") == "4", "%s header 1.4" % tag,
              "revMajor=%s revMinor=%s" % (hdr.get("revMajor"), hdr.get("revMinor")))
    order = ["link", "type", "planView", "elevationProfile", "lateralProfile",
             "lanes", "objects", "signals"]
    bad = []
    for road in root.findall("road"):
        seen = [c.tag for c in road if c.tag in order]
        idx = [order.index(t) for t in seen]
        if idx != sorted(idx):
            bad.append(road.get("id"))
        ls = road.find("lanes/laneSection")
        if ls is not None:
            sec = [c.tag for c in ls if c.tag in ("left", "center", "right")]
            if sec != ["left", "center", "right"]:
                bad.append("laneSection order road " + road.get("id"))
    rep.check(not bad, "%s road/laneSection 子元素顺序" % tag, str(bad[:4]))
    return root


def check_carla(xml_text, map_name, rep, tag):
    try:
        import carla
    except ModuleNotFoundError:
        # 没装是 README 写明的可选依赖，结构检查仍有效。包在却 import 失败
        #（numpy ABI 等）走下面的 Exception，必须 FAIL，否则会把坏环境当成"跳过"。
        rep.skip("%s CARLA 真解析" % tag, "未安装 carla PythonAPI，跳过真解析与闭环")
        return None, None
    except Exception as e:
        rep.check(False, "%s import carla" % tag, str(e))
        return None, None
    try:
        m = carla.Map(map_name, xml_text)
    except Exception as e:
        rep.check(False, "%s carla.Map 解析" % tag, str(e))
        return None, None
    rep.check(True, "%s carla.Map 解析" % tag, "")
    try:
        tp = m.get_topology()
    except Exception as e:
        tp = []
        rep.check(False, "%s get_topology" % tag, str(e))
    wps = m.generate_waypoints(0.5)
    rep.check(len(wps) > 0, "%s generate_waypoints" % tag, "%d 个" % len(wps))
    rep.check(len(tp) > 0, "%s get_topology" % tag, "%d 条边" % len(tp))
    return m, carla


def check_closed_loop(m, carla, fitted, rep, tag, step=1.0):
    """本地解析求值 vs CARLA 实算：这是整套推理是否成立的判决性检验。

    比对的是 world = (xodr_x, -xodr_y, z) 且 yaw_world = -hdg（见 xodr_geom docstring）。
    """
    worst_xy = 0.0
    worst_z = 0.0
    worst_yaw = 0.0
    worst_lane = 0.0
    n = 0
    for r in sorted(fitted["roads"], key=lambda x: x["id"]):
        geoms = [geom_from_dict(d) for d in r["geometry"]]
        elev = r["elevation"]

        def z_at(s):
            seg = elev[0]
            for e in elev:
                if e["s"] <= s:
                    seg = e
            d = s - seg["s"]
            return seg["a"] + seg["b"] * d + seg["c"] * d ** 2 + seg["d"] * d ** 3

        # CARLA 在 s == road.length 处查 lane 会返回 None，故全部夹到长度以内
        ss = sorted(set([min(i * step, r["length"] - 1e-3)
                         for i in range(int(r["length"] / step) + 1)] +
                        [max(r["length"] - 1e-3, 0.0)]))
        for s in ss:
            g = geoms[-1]
            for gg in geoms:
                if gg.s <= s <= gg.s + gg.length + 1e-9:
                    g = gg
                    break
            d = min(max(s - g.s, 0.0), g.length)
            p = g.at(d)
            try:
                w0 = m.get_waypoint_xodr(r["id"], 0, s)
                wl = m.get_waypoint_xodr(r["id"], 1, s)
                wr = m.get_waypoint_xodr(r["id"], -1, s)
            except Exception as e:
                rep.check(False, "%s get_waypoint_xodr road %d s=%.1f" % (tag, r["id"], s),
                          str(e))
                return
            if w0 is None:
                rep.check(False, "%s lane0 查询失败 road %d s=%.1f" % (tag, r["id"], s), "")
                return
            L = carla.Location(*xodr_to_world(p.x, p.y, z_at(s)))
            worst_xy = max(worst_xy, math.hypot(w0.transform.location.x - L.x,
                                                w0.transform.location.y - L.y))
            worst_z = max(worst_z, abs(w0.transform.location.z - L.z))
            worst_yaw = max(worst_yaw, abs((w0.transform.rotation.yaw
                                            + math.degrees(p.hdg) + 180.0) % 360.0 - 180.0))
            # lane +1 必须落在 CARLA 的 +t 法向 (sin h, -cos h) 一侧
            h = math.radians(w0.transform.rotation.yaw)
            nx, ny = lateral_normal(h)
            dx = wl.transform.location.x - w0.transform.location.x
            dy = wl.transform.location.y - w0.transform.location.y
            dr = wr.transform.location.x - w0.transform.location.x
            dy2 = wr.transform.location.y - w0.transform.location.y
            worst_lane = max(worst_lane,
                             abs((dx * nx + dy * ny) - r["width_left"] / 2.0),
                             abs(-(dr * nx + dy2 * ny) - r["width_right"] / 2.0))
            n += 1
    rep.check(worst_xy <= CARLA_TOL, "%s 参考线位置 vs CARLA 实算" % tag,
              "max|dXY|=%.4f m over %d samples (容差 %.3f)" % (worst_xy, n, CARLA_TOL))
    rep.check(worst_z <= CARLA_TOL, "%s 高程 vs CARLA 实算" % tag,
              "max|dZ|=%.4f m" % worst_z)
    rep.check(worst_yaw <= 0.05, "%s yaw_world == -hdg" % tag,
              "max=%.4f deg" % worst_yaw)
    rep.check(worst_lane <= LANE_SIDE_TOL, "%s 车道横向归属(+t=左)" % tag,
              "max=%.4f m" % worst_lane)


def check_frame_pair(mx, ma, carla, fittedX, A, rep):
    """两份 xodr 必须满足声明的帧关系：world_xodr == A @ M @ world_asam。

    即 CARLA 版与 ASAM 版的差异恰好等于实测的 Blender->world 变换 A（未标定时 A=M，
    两者数值相同）。这条断言防止"改了 frame.json 但只重新发射了一份文件"。
    """
    import numpy as np
    AA = np.asarray(A, dtype=float)
    MM = np.eye(4)
    MM[:3, :3] = np.asarray(XODR2WORLD, dtype=float)
    worst = 0.0
    for r in sorted(fittedX["roads"], key=lambda x: x["id"]):
        s = r["length"] / 2.0
        try:
            wx = mx.get_waypoint_xodr(r["id"], 0, s)
            wa = ma.get_waypoint_xodr(r["id"], 0, s)
        except Exception:
            continue
        wa = np.array([wa.transform.location.x, wa.transform.location.y,
                       wa.transform.location.z, 1.0])
        v = AA @ (MM @ wa)
        worst = max(worst, float(np.linalg.norm(v[:3] - [
            wx.transform.location.x, wx.transform.location.y,
            wx.transform.location.z])))
    rep.check(worst <= CARLA_TOL, "两份 xodr 满足声明的 A_S2W",
              "max=%.4f m" % worst)


def autodetect(in_dir):
    """按 CARLA 的配对规则认地图名：目录里**有同名 .fbx 的那个 .xodr** 就是给 CARLA
    的（Util/BuildTools/Import.py:63-68），ASAM 那份则是它的 _asam 兄弟。
    这样换了 FBX 文件名也不用记着同步 --map-name。

    没有 fbx 时（合成数据，如 make_demo_trace.py 那条自测链）退回"排除 _asam 后的
    唯一一份" —— 否则 README 的「一键跑通」最后一步就卡在这里。真实场景总有 fbx，
    走的还是上面那条配对规则。"""
    cands = sorted(f for f in os.listdir(in_dir) if f.endswith(".xodr")
                   and os.path.exists(os.path.join(in_dir, f[:-5] + ".fbx")))
    if not cands:
        rest = sorted(f for f in os.listdir(in_dir)
                      if f.endswith(".xodr") and not f.endswith("_asam.xodr"))
        if len(rest) == 1:
            name = rest[0][:-5]
            print("提示: %s 里没有配对的 .fbx，按唯一的非 _asam xodr 认定地图名 %s"
                  % (in_dir, name))
            return name, name + "_asam"
    if len(cands) != 1:
        raise SystemExit("无法从 %s 自动认地图（找到 %d 个带同名 fbx 的 xodr），"
                         "请用 --map-name 指定" % (in_dir, len(cands)))
    name = cands[0][:-5]
    return name, name + "_asam"


def run(in_dir, map_name=None, asam_name=None, frame_json=None):
    """跑全部校验，返回 (Report, 地图名, ASAM 名)。

    CLI 和编辑器的「生成 xodr」共用这一份编排 —— 检查项只列一遍。两边各写一份的话，
    "web 里全绿、命令行有 FAIL" 是最难查的那种不一致。
    """
    if not map_name:
        map_name, asam_name = autodetect(in_dir)
    elif asam_name is None:
        asam_name = map_name + "_asam"
    fjp = frame_json or frame_json_path(in_dir)
    print("地图名 %s / ASAM 名 %s" % (map_name, asam_name))

    rep = Report()
    fx = load_json(os.path.join(in_dir, "roads_fitted_xodr.json"))
    check_structure(fx, rep, "xodr")
    path_x = os.path.join(in_dir, map_name + ".xodr")
    check_xml(path_x, rep, "xodr")
    with open(path_x) as f:
        xml_x = f.read()
    mx, carla = check_carla(xml_x, map_name, rep, "xodr")

    path_a = os.path.join(in_dir, "roads_fitted_asam.json")
    xodr_a = os.path.join(in_dir, asam_name + ".xodr")
    if os.path.exists(path_a) and os.path.exists(xodr_a):
        fa = load_json(path_a)
        check_structure(fa, rep, "asam")
        check_xml(xodr_a, rep, "asam")
        with open(xodr_a) as f:
            xml_a = f.read()
        ma, _ = check_carla(xml_a, asam_name, rep, "asam")
        # frame.json 读不出来是"检查项失败"，不是崩溃：调用方（编辑器 /api/emit）
        # 已经在发射时降级成了未标定，这里再抛一次 JSONDecodeError 会让整个请求 500，
        # 前面的发射结果反而报不出来。没有 frame.json 是正常的（未标定场景），
        # 有却读不动才是问题。
        A = None
        if os.path.exists(fjp):
            try:
                A = load_json(fjp).get("A_S2W")
            except (ValueError, OSError) as e:
                rep.check(False, "读取 frame.json", "%s: %s" % (type(e).__name__, e))
        if mx and ma and A and carla:
            check_frame_pair(mx, ma, carla, fx, A, rep)

    if mx and carla:
        check_closed_loop(mx, carla, fx, rep, "xodr")
        print("\n  提示: get_spawn_points() 离线构造下恒为 0，需连服务端才有值。")
    return rep, map_name, asam_name


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in-dir", default="out")
    ap.add_argument("--map-name", default=None,
                    help="不给就按'有同名 .fbx 的 .xodr'自动认")
    ap.add_argument("--asam-name", default=None)
    ap.add_argument("--frame-json", default=None,
                    help="默认取 in-dir 所在场景目录的 frame.json")
    args = ap.parse_args()

    rep, map_name, asam_name = run(args.in_dir, args.map_name, args.asam_name,
                                   args.frame_json)
    sys.exit(rep.dump())


if __name__ == "__main__":
    main()
