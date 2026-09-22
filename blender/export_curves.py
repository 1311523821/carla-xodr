"""从 .blend 里把描好的 POLY 曲线导出成 traces.json（帧 S = Blender 世界帧，米）。

  blender --background --python export_curves.py -- \
      --blend trace.blend --out ../traces.json [--scan SCAN_MESH]

规则：
  * 一条路一条曲线，对象名 road_NNNN；必须是 POLY 曲线（Bezier/NURBS 先
    Curve 菜单 > Convert > To Poly Curve）
  * 点序即 s 增大方向，也是参考线正方向
  * 曲线属性 xodr.id / xodr.width_left / xodr.width_right / xodr.speed_kmh /
    xodr.link_next / xodr.junction_in / xodr.junction_out
  * 描线只画平面：z 由本脚本从扫描网格反推（从下往上打射线取地板），
    这样描线与高程解耦，重描不会破坏高程
"""
import hashlib
import json
import math
import os
import sys

import bpy
from mathutils import Vector

DOWN_FROM = -50.0
UP_FROM = 50.0


def arg(name, default=None):
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    for i, a in enumerate(argv):
        if a == name and i + 1 < len(argv):
            return argv[i + 1]
    return default


def load(blend):
    if blend and os.path.exists(blend):
        bpy.ops.wm.open_mainfile(filepath=blend)


def curve_points(ob):
    """世界帧折线点。只接受 POLY。"""
    pts = []
    for sp in ob.data.splines:
        if sp.type != 'POLY':
            raise SystemExit("%s 里有非 POLY 样条（%s）：先 Curve > Convert > "
                             "To Poly Curve" % (ob.name, sp.type))
        for p in sp.points:
            pts.append(ob.matrix_world @ Vector(p.co))
    return pts


def floor_z(scene, depsgraph, x, y):
    """从下往上打射线取地板；找不到再试从上往下（室外无顶场景）。

    Blender 4.x/5.x 的 scene.ray_cast 首参是 depsgraph。
    """
    for origin, direction in ((Vector((x, y, DOWN_FROM)), Vector((0, 0, 1.0))),
                              (Vector((x, y, UP_FROM)), Vector((0, 0, -1.0)))):
        hit, loc, _n, _i, _o, _m = scene.ray_cast(depsgraph, origin, direction)
        if hit:
            return loc.z
    return None


def main():
    blend = arg("--blend", "trace.blend")
    out = arg("--out", "../traces.json")
    load(blend)
    bpy.context.view_layer.update()
    scene = bpy.context.scene
    deps = bpy.context.evaluated_depsgraph_get()

    roads = []
    for ob in sorted(scene.objects, key=lambda o: o.name):
        if not ob.name.startswith("road_") or ob.type != 'CURVE':
            continue
        raw = curve_points(ob)
        if len(raw) < 2:
            raise SystemExit("%s 点数不足 2" % ob.name)
        rid = int(ob.get("xodr.id", ob.name.split("_")[1]))
        pts, missed = [], 0
        s = 0.0
        for i, p in enumerate(raw):
            if i:
                s += math.hypot(p.x - raw[i - 1].x, p.y - raw[i - 1].y)
            gz = floor_z(scene, deps, p.x, p.y)
            if gz is None:
                missed += 1
                gz = p.z
            pts.append({"s": round(s, 4), "x": round(p.x, 5), "y": round(p.y, 5),
                        "z": round(gz, 5), "z_trace": round(p.z, 5)})
        if missed:
            print("  ! %s 有 %d/%d 点射线未命中，回落到曲线自身 z" % (ob.name, missed, len(raw)))
        roads.append({
            "id": rid,
            "name": str(ob.get("xodr.name", ob.name)),
            "blender_object": ob.name,
            "points": pts,
            "width_left": float(ob.get("xodr.width_left", 1.75)),
            "width_right": float(ob.get("xodr.width_right", 1.75)),
            "speed_kmh": float(ob.get("xodr.speed_kmh", 5.0)),
            "link_next": None if int(ob.get("xodr.link_next", -1)) < 0
            else int(ob["xodr.link_next"]),
            "junction_in": None if int(ob.get("xodr.junction_in", -1)) < 0
            else int(ob["xodr.junction_in"]),
            "junction_out": None if int(ob.get("xodr.junction_out", -1)) < 0
            else int(ob["xodr.junction_out"]),
        })

    if not roads:
        raise SystemExit("没找到 road_* 曲线对象")
    ids = [r["id"] for r in roads]
    dup = {i for i in ids if ids.count(i) > 1}
    if dup:
        raise SystemExit("xodr.id 重复: %s" % sorted(dup))

    doc = {"schema": 1, "frame": "S", "frame_note": "Blender 世界帧，米，右手 Z-up",
           "source_blend": os.path.abspath(blend), "roads": roads}
    blob = json.dumps(doc, indent=1, ensure_ascii=False)
    with open(out, "w") as f:
        f.write(blob)
    with open(out + ".sha256", "w") as f:
        f.write(hashlib.sha256(blob.encode()).hexdigest() + "  " + out + "\n")
    print("wrote %s  (%d roads, %d pts)" % (
        out, len(roads), sum(len(r["points"]) for r in roads)))
    for r in roads:
        print("  %-10s id=%-3d n=%-4d 起(%.1f,%.1f) 终(%.1f,%.1f) L=%.2f w=%.2f/%.2f" % (
            r["blender_object"], r["id"], len(r["points"]),
            r["points"][0]["x"], r["points"][0]["y"],
            r["points"][-1]["x"], r["points"][-1]["y"],
            r["points"][-1]["s"], r["width_left"], r["width_right"]))


main()
