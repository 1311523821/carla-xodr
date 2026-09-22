"""在 Blender 里搭好描线场景：导入扫描网格 + 建一条带 xodr 属性的模板 POLY 曲线。

用法（无头，产出一个 .blend 给你在 GUI 里描线）：
  blender --background --python setup_trace_scene.py -- \
      --mesh ~/scans/<项目>/MANIFOLD_...-Opt_Meshv.ply \
      --out trace.blend

之后在 Blender GUI 里打开 trace.blend，选中 RoadTrace 集合里的曲线，
沿主通道用 Alt+左键 加点描线；一条路一条曲线，对象名 road_0001、road_0002…
曲线属性里改 width_left / width_right / speed_kmh。
"""
import os
import sys

import bpy

ATTRS = {
    "xodr.id": 1,
    "xodr.name": "road_0001",
    "xodr.width_left": 1.75,
    "xodr.width_right": 1.75,
    "xodr.speed_kmh": 5.0,
    "xodr.link_next": -1,
    "xodr.junction_in": -1,
    "xodr.junction_out": -1,
}


def arg(name, default=None):
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    for i, a in enumerate(argv):
        if a == name and i + 1 < len(argv):
            return argv[i + 1]
    return default


def import_scan(path):
    """Blender 4.x/5.x 用 wm.ply_import / wm.obj_import；旧版回退 import_mesh.*。"""
    ext = os.path.splitext(path)[1].lower()
    if ext == ".fbx":
        bpy.ops.wm.fbx_import(filepath=path)
        return
    new = {'ply': 'bpy.ops.wm.ply_import', 'obj': 'bpy.ops.wm.obj_import'}.get(ext[1:])
    old = {'ply': 'bpy.ops.import_mesh.ply', 'obj': 'bpy.ops.import_mesh.obj'}.get(ext[1:])
    for expr in (new, old):
        if not expr:
            continue
        mod, fn = expr.rsplit('.', 2)[1:]
        try:
            getattr(getattr(bpy.ops, mod), fn)(filepath=path)
            return
        except Exception:
            continue
    raise SystemExit("Blender 无法导入 %s（扩展名不支持或导入算子缺失）" % path)


def main():
    mesh_path = arg("--mesh")
    out = arg("--out", "trace.blend")
    if not mesh_path:
        raise SystemExit("必须给 --mesh <扫描网格>")

    bpy.ops.wm.read_factory_settings(use_empty=True)
    sc = bpy.context.scene
    sc.unit_settings.system = 'METRIC'
    sc.unit_settings.scale_length = 1.0

    import_scan(mesh_path)
    scan = bpy.context.active_object
    scan.name = "SCAN_MESH"
    scan.display_type = 'WIRE'

    coll = bpy.data.collections.new("RoadTrace")
    sc.collection.children.link(coll)

    cu = bpy.data.curves.new("road_0001", 'CURVE')
    cu.dimensions = '3D'
    cu.splines.new('POLY')
    ob = bpy.data.objects.new("road_0001", cu)
    coll.objects.link(ob)
    for k, v in ATTRS.items():
        ob[k] = v

    cd = bpy.data.cameras.new("Cam")
    cam = bpy.data.objects.new("TopCam", cd)
    cam.location = (0, 0, 60)
    cam.rotation_euler = (0, 0, 0)
    sc.collection.objects.link(cam)
    sc.camera = cam

    bpy.ops.wm.save_as_mainfile(filepath=out)
    print("已写出 %s" % out)
    print("  扫描网格: %s  描线集合: RoadTrace  模板曲线: road_0001 (POLY)" % scan.name)


main()
