"""把扫描网格导出成 CARLA `make import` 用的 FBX，导入/导出参数全部锁定。

  # MindCloud 出的 out.obj 直接转 FBX（推荐，不用开 GUI）
  blender --background --python export_fbx.py -- \
      --obj ~/scans/<项目>/model/out.obj --out ~/scans/<项目>/<地图名>.fbx
  # 室内扫描太小、摆不下真实路网时整体放大（会烘进顶点，放大后 xodr 的米跟着变）
  blender --background --python export_fbx.py -- \
      --obj .../out.obj --out .../<地图名>.fbx --scale 4
  # 或者从已经排好版的 .blend 里挑一个对象
  blender --background --python export_fbx.py -- \
      --blend trace.blend --out .../<地图名>.fbx [--object SCAN_MESH]

**--obj 那条路径上的轴向参数是这次踩坑的根源，别改。** Blender 的 OBJ 导入器
默认以为文件是 Y-up（forward=NEGATIVE_Z, up=Y），而 MindCloud 的 out.obj 是
Z-up，于是它给对象塞了个 rotation_euler.x = 1.570796（+90°）——网格在 Blender
里就躺下了，导出的 FBX 忠实地把躺着的姿态带进 UE（UE 里 Roll=-90 能救回来，
正是同一个角度）。forward_axis='Y', up_axis='Z' 是恒等映射，实测质心和逐轴
跨度和文件里 v 行的数字一模一样，对象变换矩阵 == 单位阵。

导出侧锁死这些参数是为了让 S->world 的推导没有自由量（实测结论仍以 calibrate_frame.py 为准）：
  axis_forward='X' axis_up='Z'  => 水平面恒等映射
  bake_space_transform=True     => 把轴变换烘进顶点，对象变换归零
  apply_scale_options='FBX_SCALE_ALL', global_scale=1.0, 场景单位=米
  embed_textures=True           => 纹理打进 FBX 本体，详见导出处的注释
曲线对象一律排除——make import 会把 FBX 里的东西全导成 StaticMesh。
"""
import os
import sys

import bpy
from mathutils import Matrix


def arg(name, default=None):
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    for i, a in enumerate(argv):
        if a == name and i + 1 < len(argv):
            return argv[i + 1]
    return default


def read_obj():
    """MindCloud 的 out.obj -> Blender 场景，轴映射必须是恒等，否则当场停。"""
    obj = arg("--obj")
    if not obj:
        return
    if not os.path.exists(obj):
        raise SystemExit("--obj 文件不存在: %s" % obj)
    bpy.ops.wm.read_factory_settings(use_empty=True)
    # forward='Y' + up='Z' => 文件 (x,y,z) 原样落到 Blender (x,y,z)。
    # 少任一个参数，导入器就会塞一个 ±90° 的 rotation_euler，扫描直接躺下。
    bpy.ops.wm.obj_import(filepath=obj, forward_axis='Y', up_axis='Z')
    bad = [o.name for o in bpy.context.scene.objects
           if o.type == 'MESH' and o.matrix_world != Matrix.Identity(4)]
    if bad:
        raise SystemExit("OBJ 导入后对象仍带变换 %s —— 轴向参数被改过了，"
                         "导出的 FBX 会躺倒，别往下走" % bad)
    print("obj -> scene: %s  （轴向恒等，未塞旋转）" % os.path.basename(obj))


def mesh_box(vl):
    """选中网格的世界包围盒。导出前把这三个数打出来，是为了让"这份 FBX 到底多大、
    哪一轴是高度"留在日志里，而不是等到在 UE 里发现地图不对再倒推。"""
    vl.update()
    mn, mx = [1e9] * 3, [-1e9] * 3
    for ob in vl.objects:
        if ob.type != 'MESH' or not ob.select_get():
            continue
        mw = ob.matrix_world
        for v in ob.data.vertices:
            c = mw @ v.co
            for i in range(3):
                mn[i] = min(mn[i], c[i])
                mx[i] = max(mx[i], c[i])
    return mn, mx


def main():
    blend = arg("--blend", "trace.blend")
    out = arg("--out")
    # 整体放大是合法的用法：室内扫描 30×36 m 摆不下真实路网，用过 4 倍。
    # 但缩放一定要烘进顶点、不能留在对象变换上 —— 一来下面的循环会把对象变换归零，
    # 二来对象变换上的 scale 两个下游处理不一致（实测 FBX 里 scale=2 时浏览器按
    # 60 m 显示、open3d 射线场按 30 m 算，描线吸附和看图差一倍）。
    s = float(arg("--scale", "1.0"))
    want = arg("--object", "ALL" if arg("--obj") else "SCAN_MESH")
    if not out:
        raise SystemExit("必须给 --out <目标 .fbx>")
    if arg("--obj"):
        read_obj()
    elif os.path.exists(blend):
        bpy.ops.wm.read_factory_settings(use_empty=True)
        bpy.ops.wm.open_mainfile(filepath=blend)

    sc = bpy.context.scene
    sc.unit_settings.system = 'METRIC'
    sc.unit_settings.scale_length = 1.0

    bpy.ops.object.select_all(action='DESELECT')
    n = 0
    for ob in sc.objects:
        if ob.type != 'MESH' or (want != "ALL" and ob.name != want):
            continue
        ob.select_set(True)
        ob.location = (0.0, 0.0, 0.0)
        ob.rotation_euler = (0.0, 0.0, 0.0)
        ob.scale = (s, s, s)
        n += 1
    if not n:
        raise SystemExit("没有可导出的 MESH 对象（--object %s）" % want)
    if s != 1.0:
        bpy.context.view_layer.objects.active = [
            o for o in sc.objects if o.select_get()][0]
        bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    mn, mx = mesh_box(bpy.context.view_layer)
    print("导出帧包围盒  X %.3f..%.3f  Y %.3f..%.3f  Z %.3f..%.3f  （跨度 %.3f × %.3f × %.3f m，scale=%g）"
          % (mn[0], mx[0], mn[1], mx[1], mn[2], mx[2],
             mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2], s))

    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    # 纹理一律内嵌：外部纹理会写成 <名字>.fbm/ 的相对路径，而下游是把这个 fbx
    # **软链**进 ~/carla/Import/<stem>/ 的（见 trace_editor.pair_fbx）——链那头的
    # .fbm 目录不会跟过来，UE 就找不到贴图。一个自包含的文件没有这个问题。
    embed = "--no-embed" not in sys.argv
    bpy.ops.export_scene.fbx(
        filepath=out, use_selection=True, object_types={'MESH'},
        axis_forward='X', axis_up='Z',
        use_space_transform=True, bake_space_transform=True,
        apply_scale_options='FBX_SCALE_ALL', global_scale=1.0,
        use_mesh_modifiers=True, mesh_smooth_type='FACE',
        add_leaf_bones=False, path_mode='COPY', embed_textures=embed)
    print("wrote %s  (%d 个 mesh 对象, %.1f MB, 纹理%s内嵌)" % (
        out, n, os.path.getsize(out) / 1e6, "" if embed else "未"))


main()
