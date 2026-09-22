"""把 tmp/spine.npy 里的最宽路径写进 trace.blend，作为起始描线曲线。

  blender --background --python add_spine_curves.py -- --blend ../trace.blend
"""
import os
import sys

import bpy
import numpy as np


def arg(name, default=None):
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    for i, a in enumerate(argv):
        if a == name and i + 1 < len(argv):
            return argv[i + 1]
    return default


def main():
    blend = arg("--blend", "../trace.blend")
    spine_path = arg("--spine", os.path.join(os.path.dirname(__file__), "..", "tmp", "spine.npy"))
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.wm.open_mainfile(filepath=blend)
    coll = bpy.data.collections.get("RoadTrace")
    if coll is None:
        raise SystemExit("没有 RoadTrace 集合，先跑 setup_trace_scene.py")

    # 清掉模板曲线
    for ob in list(coll.objects):
        bpy.data.objects.remove(ob, do_unlink=True)

    pts = np.load(spine_path)
    # 抽稀到 ~1 m 一个控制点，描线点太密没意义
    keep = [0]
    for i in range(1, len(pts)):
        if np.hypot(*(pts[i, :2] - pts[keep[-1], :2])) >= 1.0:
            keep.append(i)
    if keep[-1] != len(pts) - 1:
        keep.append(len(pts) - 1)
    pts = pts[keep]

    cu = bpy.data.curves.new("road_0001", 'CURVE')
    cu.dimensions = '3D'
    sp = cu.splines.new('POLY')
    sp.points.add(len(pts) - 1)
    for i, p in enumerate(pts):
        sp.points[i].co = (float(p[0]), float(p[1]), 0.0, 1.0)
    ob = bpy.data.objects.new("road_0001", cu)
    coll.objects.link(ob)
    for k, v in {"xodr.id": 1, "xodr.name": "corridor_main", "xodr.width_left": 1.10,
                 "xodr.width_right": 1.10, "xodr.speed_kmh": 5.0, "xodr.link_next": -1,
                 "xodr.junction_in": -1, "xodr.junction_out": -1}.items():
        ob[k] = v

    bpy.ops.wm.save_as_mainfile(filepath=blend)
    print("写入 %d 个控制点到 road_0001 (宽 1.10/1.10)" % len(pts))


main()
