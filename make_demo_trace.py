#!/usr/bin/env python3
"""生成合成 traces.json，用于在没有人工描线前先验证整条链路。

三条路刻意覆盖三种几何：纯直路(line)、常曲率弯(arc)、变曲率 S 形(paramPoly3)。
坐标在帧 S（Blender/扫描帧），单位米。
"""
import json
import math
import sys

DS = 0.25


def sample(fn, n):
    return [fn(i / (n - 1)) for i in range(n)]


def road(rid, name, pts, wl=1.75, wr=1.75, spd=5.0):
    out = []
    s = 0.0
    for i, (x, y, z) in enumerate(pts):
        if i:
            s += math.hypot(x - pts[i - 1][0], y - pts[i - 1][1])
        out.append({"s": round(s, 4), "x": x, "y": y, "z": z})
    return {"id": rid, "name": name, "points": out,
            "width_left": wl, "width_right": wr, "speed_kmh": spd,
            "link_next": None, "junction_in": None, "junction_out": None}


def main():
    # road 1: 25 m 直路，带 2% 纵坡
    r1 = sample(lambda t: (t * 25.0, 0.0, -1.40 + 0.5 * t), 101)

    # road 2: 常曲率圆弧，R=20 m，转 60 度
    R, sweep = 20.0, math.radians(60.0)

    def arc(t):
        a = sweep * t
        return (R * math.sin(a), -(R - R * math.cos(a)), -0.9)

    r2 = sample(arc, 141)

    # road 3: S 形变曲率（三次正弦），模拟绕开工位的一条通道
    L = 34.0

    def sshape(t):
        x = t * L
        y = 3.2 * math.sin(2.0 * math.pi * t) * (1.0 - 0.25 * t)
        z = -1.45 + 0.15 * math.sin(3.0 * math.pi * t)
        return (x + 40.0, y - 20.0, z)

    r3 = sample(sshape, 161)

    doc = {
        "schema": 1,
        "frame": "S",
        "source": "make_demo_trace.py (synthetic, not scanned data)",
        "roads": [road(1, "demo_straight", r1),
                  road(2, "demo_arc", r2),
                  road(3, "demo_sshape", r3)],
    }
    out = sys.argv[1] if len(sys.argv) > 1 else "traces.json"
    with open(out, "w") as f:
        json.dump(doc, f, indent=1)
    print("wrote %s: %d roads, %d pts" % (
        out, len(doc["roads"]), sum(len(r["points"]) for r in doc["roads"])))


if __name__ == "__main__":
    main()
