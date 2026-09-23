#!/usr/bin/env python3
"""roads_fitted_{xodr,asam}.json -> xodr。同一代码路径出两份：

  out/<地图名>.xodr          帧 X（CARLA world 系，未标定时 = 扫描帧），
                             与同名 .fbx 一起走 make import
  out/<地图名>_asam.xodr     帧 S（右手系，扫描/Blender 帧），给 Autoware / lanelet2

镜像不交换车道：CARLA 的 +t 法向 (sin,-cos) 本身就是 ASAM (-sin,cos) 的镜像，
两次镜像相抵，所以 width_left/right 与 lane id 在两个帧里指同一物理侧。
（validate.py 的 lane_side 断言会实测这一点。）
"""

import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fit_geometry import geom_from_dict  # noqa: E402

VENDOR = "carla-xodr"
DATE = "2026-09-20T00:00:00"


def _fmt(v):
    return "%.18g" % v


# XML 里必须转义的五个字符。之前只把双引号换成单引号，路名带 & 或 < 就会发出
# 一份解析不了的 xodr（实测 <road name="A&B <road>"> 让 lxml 报 EntityRef 错）。
# 路名是可以从 Blender 曲线的 xodr.* 属性里来的，不是纯程序生成的安全串。
_XESC = {"&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&apos;"}


def _esc(s):
    return "".join(_XESC.get(c, c) for c in str(s))


def bbox(roads):
    xs, ys = [], []
    for r in roads:
        for d in r["geometry"]:
            g = geom_from_dict(d)
            n = max(int(math.ceil(g.length / 1.0)), 1)
            for i in range(n + 1):
                p = g.at(g.length * i / n)
                xs.append(p.x)
                ys.append(p.y)
    if not xs:
        return -1.0, 1.0, -1.0, 1.0
    return max(ys), min(ys), max(xs), min(xs)


def emit_header(roads, name):
    north, south, east, west = bbox(roads)
    return ('<header revMajor="1" revMinor="4" name="%s" version="1" date="%s" '
            'north="%s" south="%s" east="%s" west="%s" vendor="%s"/>' % (
                _esc(name), DATE, _fmt(north), _fmt(south), _fmt(east), _fmt(west), VENDOR))


def link_index(roads):
    """按每条路的 link_next 建双向索引 (前驱表, 后继表)。

    只写一边是不够的：路 A 声明 successor=B，B 那边必须有对应的 predecessor，
    否则 CARLA 的图里 B 的入口是悬空的。两条路汇入同一条需要 junction（v2 再说），
    这里直接报错而不是悄悄发一张连不通的图。
    """
    nxt = {r["id"]: r.get("link_next") for r in roads}
    prv = {}
    for a, b in nxt.items():
        if b is None:
            continue
        if b not in nxt:
            raise SystemExit("road %s 的 link_next 指向不存在的 road %s" % (a, b))
        if b in prv and prv[b] != a:
            raise SystemExit("road %s 和 road %s 都 link_next 到 road %s —— 汇入同一条路要 "
                             "junction，当前发射器不支持" % (prv[b], a, b))
        prv[b] = a
    return prv, nxt


def emit_link(rid, prv, nxt):
    """路级 <link>。contactPoint 说的是**对方**那条路靠哪一端接过来。"""
    parts = []
    p = prv.get(rid)
    s = nxt.get(rid)
    if p is not None:
        parts.append('<predecessor elementType="road" elementId="%d" contactPoint="end"/>' % p)
    if s is not None:
        parts.append('<successor elementType="road" elementId="%d" contactPoint="start"/>' % s)
    return "<link>%s</link>" % "".join(parts) if parts else ""


def emit_planview(geoms):
    out = ["<planView>"]
    for d in geoms:
        out.append(geom_from_dict(d).to_xml_children())
    out.append("</planView>")
    return "".join(out)


def emit_elevation(elev):
    out = ["<elevationProfile>"]
    for e in elev:
        out.append('<elevation s="%s" a="%s" b="%s" c="%s" d="%s"/>' % (
            _fmt(e["s"]), _fmt(e["a"]), _fmt(e["b"]), _fmt(e["c"]), _fmt(e["d"])))
    out.append("</elevationProfile>")
    return "".join(out)


def emit_lane(lane_id, width, speed, has_prev, has_next):
    """车道级 <link>。predecessor/successor 是按 **s 端点**说的（跟车道正负无关）：
    predecessor 在 s=0 那一头，successor 在 s=L 那一头，接的都是同一条路。
    我们发射的图里两端都是"绕过去"的连续关系，所以接过来的车道 id 同号；
    正负号翻转的情形是中央隔离带两幅路互为前后继，本发射器不产那种拓扑。
    """
    inner = ""
    if has_prev:
        inner += '<predecessor id="%d"/>' % lane_id
    if has_next:
        inner += '<successor id="%d"/>' % lane_id
    lk = "<link>%s</link>" % inner if inner else ""
    return ('<lane id="%d" type="driving" level="false">'
            '%s'
            '<width sOffset="0" a="%s" b="0" c="0" d="0"/>'
            '<roadMark sOffset="0" type="broken" material="standard" color="white" '
            'laneChange="none" width="0.12"/>'
            '<speed sOffset="0" max="%s" unit="km/h"/>'
            '</lane>') % (lane_id, lk, _fmt(width), _fmt(speed))


def emit_lanes(r, prv, nxt):
    wl, wr, spd = r["width_left"], r["width_right"], r["speed_kmh"]
    rid = r["id"]
    # 必须取值判 None：每条路都带 link_next 这个键（没连就是 None），
    # 用 `rid in nxt` 会一路都当真，给每条车道写上指向自己的后继。
    hp, hn = prv.get(rid) is not None, nxt.get(rid) is not None
    parts = ["<lanes>",
             '<laneOffset s="0" a="0" b="0" c="0" d="0"/>',
             '<laneSection s="0">',
             "<left>",
             emit_lane(1, wl, spd, hp, hn),
             "</left>",
             '<center><lane id="0" type="none" level="false">'
             '<roadMark sOffset="0" type="solid" material="standard" color="white" '
             'laneChange="none" width="0.12"/></lane></center>',
             "<right>",
             emit_lane(-1, wr, spd, hp, hn),
             "</right>",
             "</laneSection>", "</lanes>"]
    return "".join(parts)


def emit_road(r, prv, nxt):
    return ('<road name="%s" length="%s" id="%d" junction="-1">%s%s%s%s%s</road>' % (
        _esc(r["name"]), _fmt(r["length"]), r["id"],
        emit_link(r["id"], prv, nxt),
        '<type s="0" type="town"><speed max="%s" unit="km/h"/></type>' % _fmt(r["speed_kmh"]),
        emit_planview(r["geometry"]),
        emit_elevation(r["elevation"]),
        emit_lanes(r, prv, nxt)))


def emit(fitted, map_name, out_path, geo_ref=None):
    roads = fitted["roads"]
    prv, nxt = link_index(roads)
    body = ['<?xml version="1.0" encoding="UTF-8"?>', "<OpenDRIVE>",
            emit_header(roads, map_name)]
    if geo_ref:
        body.append("<geoReference><![CDATA[%s]]></geoReference>" % geo_ref)
    body += [emit_road(r, prv, nxt) for r in sorted(roads, key=lambda x: x["id"])]
    body.append("</OpenDRIVE>")
    xml = "\n".join(body)
    with open(out_path, "w") as f:
        f.write(xml)
    return xml


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in-dir", default="out")
    # 地图名不给就报错，不默认成 TestMap：编辑器那边名字跟着 FBX 走，
    # 一个写死的默认值只会让命令行用户悄悄导出一份和配对不符的文件。
    ap.add_argument("--map-name", required=True,
                    help="地图名 = FBX 文件名去掉扩展名（配对靠同名，见 README 第 5 步）")
    ap.add_argument("--asam-name", default=None,
                    help="默认 <map-name>_asam —— validate.py 的 autodetect 就是这个约定，"
                         "两边不一致时那份 ASAM 检查会被静默跳过")
    ap.add_argument("--out-dir", default="out")
    args = ap.parse_args()
    if not args.asam_name:
        args.asam_name = args.map_name + "_asam"

    os.makedirs(args.out_dir, exist_ok=True)
    xpath = os.path.join(args.in_dir, "roads_fitted_xodr.json")
    if not os.path.exists(xpath):
        raise SystemExit("缺 %s —— 先跑 fit_geometry.py（它默认 --out-dir 也是 out）" % xpath)
    with open(xpath) as f:
        fitted_x = json.load(f)
    xml = emit(fitted_x, args.map_name,
               os.path.join(args.out_dir, args.map_name + ".xodr"))
    print("wrote %s/%s.xodr  (%d roads, %d bytes, 帧 %s)" % (
        args.out_dir, args.map_name, len(fitted_x["roads"]), len(xml),
        fitted_x["frame"]))

    spath = os.path.join(args.in_dir, "roads_fitted_asam.json")
    if os.path.exists(spath):
        with open(spath) as f:
            fitted_s = json.load(f)
        xml = emit(fitted_s, args.asam_name,
                   os.path.join(args.out_dir, args.asam_name + ".xodr"))
        print("wrote %s/%s.xodr  (%d roads, %d bytes, 帧 %s — ASAM/右手系，给 Autoware)" % (
            args.out_dir, args.asam_name, len(fitted_s["roads"]), len(xml),
            fitted_s["frame"]))
    else:
        print("跳过 ASAM 版：缺 %s" % spath)


if __name__ == "__main__":
    main()
